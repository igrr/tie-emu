#include <inttypes.h>
#include <string.h>
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"
#include "card.h"

static const char *TAG = "card";

#define CARD_POWER_OFF GPIO_NUM_45 /* high: the board's switch cuts the card's supply */

/* A UHS-I card stays at 1.8 V through a reset of the chip, and then no longer answers the
 * initialization at 3.3 V: switch its supply off and on first. Its bus lines (slot 0: CLK 43,
 * CMD 44, D0-D3 39-42) are driven low meanwhile, or their pull-ups keep it powered. */
static void card_power_cycle(void)
{
    const uint64_t bus = 1ULL << 39 | 1ULL << 40 | 1ULL << 41 | 1ULL << 42 | 1ULL << 43 | 1ULL << 44;
    const gpio_config_t io = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << CARD_POWER_OFF | bus };
    gpio_config(&io);
    for (int pin = 39; pin <= 44; pin++) {
        gpio_set_level(pin, 0);
    }
    gpio_set_level(CARD_POWER_OFF, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CARD_POWER_OFF, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* Sector cache under FATFS. FATFS reads the card a 512-byte sector at a time for directories,
 * the FAT, and the unaligned heads and tails of file reads, and every command costs ~200 us
 * whatever its length: opening a file in RESOURCE took ~35 of them. Reads shorter than a line
 * fetch the whole aligned line (8 KB) into an LRU set of lines in PSRAM; longer ones go to the
 * card directly. Writes go through to the card and update the lines they overlap. */
#define LINE_SECTORS 16
#define LINE_BYTES (LINE_SECTORS * 512)
#define LINES 128
#define NO_LINE UINT32_MAX

static sdmmc_card_t *s_card;
static uint8_t *s_line;             /* LINES * LINE_BYTES */
static uint32_t s_tag[LINES];       /* sector / LINE_SECTORS, or NO_LINE */
static uint32_t s_used[LINES];      /* s_clock at the last use */
static uint32_t s_clock;

static int line_find(uint32_t tag)
{
    for (int i = 0; i < LINES; i++) {
        if (s_tag[i] == tag) {
            return i;
        }
    }
    return -1;
}

static void lines_drop(uint32_t sector, uint32_t count)
{
    for (int i = 0; i < LINES; i++) {
        if (s_tag[i] != NO_LINE && s_tag[i] >= sector / LINE_SECTORS &&
            s_tag[i] <= (sector + count - 1) / LINE_SECTORS) {
            s_tag[i] = NO_LINE;
        }
    }
}

static DSTATUS cache_init(BYTE pdrv)
{
    return sdmmc_get_status(s_card) == ESP_OK ? 0 : STA_NOINIT;
}

static DSTATUS cache_status(BYTE pdrv)
{
    return 0;
}

static DRESULT cache_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (count >= LINE_SECTORS) {
        return sdmmc_read_sectors(s_card, buff, sector, count) == ESP_OK ? RES_OK : RES_ERROR;
    }
    while (count) {
        uint32_t tag = sector / LINE_SECTORS, at = sector % LINE_SECTORS;
        uint32_t n = count < LINE_SECTORS - at ? count : LINE_SECTORS - at;
        int i = line_find(tag);
        if (i < 0) {
            i = 0;
            for (int j = 1; j < LINES; j++) {
                if (s_used[j] < s_used[i]) {
                    i = j;
                }
            }
            uint32_t first = tag * LINE_SECTORS, len = LINE_SECTORS;
            if (first + len > (uint32_t)s_card->csd.capacity) {
                len = (uint32_t)s_card->csd.capacity - first;
            }
            if (sdmmc_read_sectors(s_card, s_line + i * LINE_BYTES, first, len) != ESP_OK) {
                s_tag[i] = NO_LINE;
                return RES_ERROR;
            }
            s_tag[i] = tag;
        }
        s_used[i] = ++s_clock;
        memcpy(buff, s_line + i * LINE_BYTES + at * 512, n * 512);
        buff += n * 512;
        sector += n;
        count -= n;
    }
    return RES_OK;
}

static DRESULT cache_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (sdmmc_write_sectors(s_card, buff, sector, count) != ESP_OK) {
        lines_drop(sector, count);  /* what the card holds there is unknown */
        return RES_ERROR;
    }
    for (int i = 0; i < LINES; i++) {
        if (s_tag[i] == NO_LINE) {
            continue;
        }
        uint32_t first = s_tag[i] * LINE_SECTORS;
        uint32_t lo = sector > first ? sector : first;
        uint32_t hi = sector + count < first + LINE_SECTORS ? sector + count : first + LINE_SECTORS;
        if (lo < hi) {
            memcpy(s_line + i * LINE_BYTES + (lo - first) * 512, buff + (lo - sector) * 512, (hi - lo) * 512);
        }
    }
    return RES_OK;
}

static DRESULT cache_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        *(DWORD *)buff = s_card->csd.capacity;
        return RES_OK;
    case GET_SECTOR_SIZE:
        *(WORD *)buff = s_card->csd.sector_size;
        return RES_OK;
    case CTRL_TRIM: {
        DWORD first = ((DWORD *)buff)[0], count = ((DWORD *)buff)[1] - first + 1;
        if (sdmmc_can_trim(s_card) != ESP_OK) {
            return RES_PARERR;
        }
        lines_drop(first, count);
        sdmmc_erase_arg_t arg = sdmmc_can_discard(s_card) == ESP_OK ? SDMMC_DISCARD_ARG : SDMMC_ERASE_ARG;
        return sdmmc_erase_sectors(s_card, first, count, arg) == ESP_OK ? RES_OK : RES_ERROR;
    }
    default:
        return RES_ERROR;
    }
}

/* Put the cache between FATFS and the card mounted with it (the card has 512-byte sectors) */
static void cache_start(sdmmc_card_t *card)
{
    BYTE pdrv = ff_diskio_get_pdrv_card(card);
    s_line = heap_caps_aligned_alloc(64, LINES * LINE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (pdrv == 0xff || !s_line || card->csd.sector_size != 512) {
        ESP_LOGW(TAG, "no sector cache");
        return;
    }
    s_card = card;
    for (int i = 0; i < LINES; i++) {
        s_tag[i] = NO_LINE;
    }
    static const ff_diskio_impl_t impl = {
        .init = cache_init, .status = cache_status, .read = cache_read, .write = cache_write, .ioctl = cache_ioctl,
    };
    ff_diskio_register(pdrv, &impl);
}

esp_err_t card_mount(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
#if CONFIG_TIE_SD_SPEED_SDR50
    host.max_freq_khz = SDMMC_FREQ_SDR50;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#elif CONFIG_TIE_SD_SPEED_DDR50
    host.max_freq_khz = SDMMC_FREQ_DDR50;
#elif CONFIG_TIE_SD_SPEED_SDR104
    host.max_freq_khz = SDMMC_FREQ_SDR104;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#else
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
#endif
    sd_pwr_ctrl_ldo_config_t ldo = { .ldo_chan_id = 4 };
    esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo, &host.pwr_ctrl_handle);
    if (err != ESP_OK) {
        return err;
    }
    card_power_cycle();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
#if !CONFIG_TIE_SD_SPEED_HS
    slot.flags |= SDMMC_SLOT_FLAG_UHS1;
#endif
    const esp_vfs_fat_mount_config_t mount = { .max_files = 32, .allocation_unit_size = 64 * 1024 };
    sdmmc_card_t *card;
    err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount, &card);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s, %" PRIu64 " MB, %d kHz%s%s", card->cid.name,
                 (uint64_t)card->csd.capacity * card->csd.sector_size >> 20, card->real_freq_khz,
                 card->is_uhs1 ? ", UHS-I" : "", card->is_ddr ? " DDR" : "");
        cache_start(card);
    }
    return err;
}

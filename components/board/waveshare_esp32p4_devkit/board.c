/* The Waveshare ESP32-P4 DEV-KIT with the 7-DSI-TOUCH-A panel: the game from the SD card on the
 * display, the speaker, and USB keyboards and mice.
 *
 * The emulation task has core 1 to itself. Core 0 runs the display task, which converts each
 * new frame to RGB565 and has the PPA scale it to 4:3 and rotate it into the panel's frame
 * buffer, and the audio task, which synthesizes and mixes the machine's audio (most of the
 * audio cost, see machine_audio_read()) and feeds the codec. Hardware setup follows the
 * board's BSP (waveshare/esp32_p4_platform), which is not used since it brings LVGL.
 *
 * Card layout: /sdcard/tie/c is C: (the game directory), /sdcard/tie/d is D: (the CD).
 */
#include <inttypes.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/ppa.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9881c.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "board.h"
#include "card.h"
#include "input.h"
#include "usb_hid.h"

static const char *TAG = "board";

#define ROOT "/sdcard/tie"

/* board wiring (BSP esp32_p4_platform.h) */
#define I2C_SCL 8
#define I2C_SDA 7
#define I2S_MCLK 13
#define I2S_BCLK 12
#define I2S_WS 10
#define I2S_DOUT 9
#define I2S_DIN 11
#define PA_ENABLE 53
/* the panel driver sets the backlight over I2C port 1 (i2c_bus), and uses the bus found there */
#define I2C_PORT I2C_NUM_1
#define DSI_PHY_LDO 3           /* LDO_VO3 powers the MIPI D-PHY */

/* the panel is 720x1280 portrait, mounted landscape */
#define LCD_W 720
#define LCD_H 1280
#define OUT_W 960               /* the game's 4:3 picture, landscape, centred */
#define OUT_H 720
#define SRC_W 640               /* the largest mode */
#define SRC_H 480
#define STRIPE 32               /* rows converted at a time */
#define RAW_PITCH 1024          /* the widest row pitch copied by DMA; wider ones are read in place */

#define DISPLAY_CORE 0
#define AUDIO_CORE 0

static i2c_master_bus_handle_t s_i2c;

/* ---- display ---- */

static esp_lcd_panel_handle_t s_panel;
static void *s_fb[2];
static int s_shown;                     /* the frame buffer the panel is scanning */
static bool s_switching;                /* ... and it has not yet moved to it from the other */
static SemaphoreHandle_t s_fb_free;     /* given when the panel has moved to the other buffer */
static ppa_client_handle_t s_ppa;
static SemaphoreHandle_t s_ppa_done;    /* given for each PPA operation done */
static uint16_t *s_stripe[2];           /* STRIPE rows of the frame in RGB565: the PPA's input */
static uint8_t *s_raw[2];               /* STRIPE rows of the frame as the guest has them */
static async_memcpy_handle_t s_dma;
static SemaphoreHandle_t s_dma_done;    /* given for each copy into s_raw done */
static TaskHandle_t s_display;
static machine_t *s_m;

static bool IRAM_ATTR on_frame_buf_complete(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *ev, void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_fb_free, &woken);
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_ppa_done(ppa_client_handle_t client, ppa_event_data_t *ev, void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_ppa_done, &woken);
    return woken == pdTRUE;
}

static bool IRAM_ATTR on_dma_done(async_memcpy_handle_t mcp, async_memcpy_event_t *ev, void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_done, &woken);
    return woken == pdTRUE;
}

static void display_init(void)
{
    esp_ldo_channel_handle_t ldo;
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = DSI_PHY_LDO, .voltage_mv = 2500 };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo));

    esp_lcd_dsi_bus_handle_t bus;
    esp_lcd_dsi_bus_config_t bus_cfg = ILI9881C_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &bus));
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_cfg = ILI9881C_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(bus, &dbi_cfg, &io));

    esp_lcd_dpi_panel_config_t dpi_cfg = ILI9881C_720_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
    dpi_cfg.num_fbs = 2;
    ili9881c_vendor_config_t vendor = {
        .mipi_config = { .dsi_bus = bus, .dpi_config = &dpi_cfg, .lane_num = 2 },
    };
    esp_lcd_panel_dev_config_t dev_cfg = {
        .bits_per_pixel = 16,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .reset_gpio_num = -1,
        .vendor_config = &vendor,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io, &dev_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &s_fb[0], &s_fb[1]));
    s_fb_free = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = { .on_frame_buf_complete = on_frame_buf_complete };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL));

    ppa_client_config_t ppa_cfg = { .oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 2 };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &s_ppa));
    ppa_event_callbacks_t ppa_cbs = { .on_trans_done = on_ppa_done };
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(s_ppa, &ppa_cbs));
    s_ppa_done = xSemaphoreCreateCounting(2, 0);
    s_dma_done = xSemaphoreCreateCounting(2, 0);
    for (int i = 0; i < 2; i++) {
        s_stripe[i] = heap_caps_aligned_calloc(64, SRC_W * STRIPE, 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        s_raw[i] = heap_caps_aligned_calloc(128, RAW_PITCH * STRIPE, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        assert(s_stripe[i] && s_raw[i]);
    }
    async_memcpy_config_t mcp_cfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
    if (esp_async_memcpy_install_gdma_axi(&mcp_cfg, &s_dma) != ESP_OK) {
        ESP_LOGW(TAG, "no DMA for reading frames: read in place");
        s_dma = NULL;
    }
}

/* Before writing the frame buffer the panel was last switched away from: the refresh in
 * progress at the switch still reads it, and the panel takes the new buffer when that refresh
 * ends. Usually long past by the next frame. */
static void fb_wait(void)
{
    if (s_switching) {
        xSemaphoreTake(s_fb_free, pdMS_TO_TICKS(100));
        s_switching = false;
    }
}

/* Converts the current frame to RGB565 a stripe of rows at a time, and has the PPA scale and
 * rotate each stripe into the frame buffer the panel is not showing; then switches the panel to
 * that buffer.
 *
 * The stripes go through internal RAM, both ways: reading video memory in place (in PSRAM)
 * would push the emulator's code and data out of the L2 cache the cores share, which in flight
 * cost the emulator more than the panel's refresh does. The DMA copies rows in, the CPU converts
 * them, and the PPA writes the result out, each on a different stripe at a time.
 *
 * The picture is 4:3 whatever the mode's pixel aspect, 960x720 in the middle of the landscape
 * screen. The PPA scales by sixteenths: the source's rows y0..y1 cover rows_out(y1) - rows_out(y0)
 * output lines, exactly, when y0 is a multiple of 16. */
static void display_frame(void)
{
    static int last_w, last_h, cleared;
    machine_frame_t fr;
    machine_frame(s_m, &fr);
    int w = fr.width, h = fr.height;
    if (w > SRC_W || h > SRC_H || !w || !h || w % 4) {
        return;
    }
    if (fr.size < (size_t)(h - 1) * fr.pitch + w) {
        return; /* a start address near the end of video memory: mid-scroll, skip it */
    }
    if (w != last_w || h != last_h) {
        cleared = 0; /* a new mode: the old picture's edges may show around the new one */
        last_w = w;
        last_h = h;
    }
    uint16_t pal[256];
    for (int i = 0; i < 256; i++) {
        uint32_t c = fr.palette[i];
        pal[i] = (uint16_t)(((c >> 8) & 0xf800) | ((c >> 5) & 0x07e0) | ((c >> 3) & 0x001f));
    }
    int back = !s_shown;
    if (cleared < 2) {
        fb_wait();
        memset(s_fb[back], 0, LCD_W * LCD_H * 2);
        esp_cache_msync(s_fb[back], LCD_W * LCD_H * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        cleared++;
    }
    int kx = OUT_W * 16 / w, ky = OUT_H * 16 / h;   /* scale factors in sixteenths */
    int out_w = w * kx / 16, out_h = h * ky / 16;
    int left = (LCD_W - out_h) / 2;                 /* rotated: the source's rows run across */
    int stripes = (h + STRIPE - 1) / STRIPE;
    /* the DMA reads memory: the cache writes back what the emulator wrote first */
    bool dma = s_dma && fr.pitch <= RAW_PITCH && (fr.pitch & 3) == 0;
    if (dma) {
        esp_cache_msync((void *)fr.pixels, (size_t)(h - 1) * fr.pitch + w,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
    bool copying = false;           /* a copy of the next stripe into s_raw is under way */
    int issued = 0, done = 0;       /* PPA operations */
    bool ok = true;
    for (int k = 0; k < stripes && ok; k++) {
        int y0 = k * STRIPE, n = h - y0 < STRIPE ? h - y0 : STRIPE;
        size_t len = ((size_t)(n - 1) * fr.pitch + w + 127) & ~(size_t)127;
        const uint8_t *rows = fr.pixels + (size_t)y0 * fr.pitch;
        if (dma && k == 0) {
            copying = esp_async_memcpy(s_dma, s_raw[0], (void *)rows, len, on_dma_done, NULL) == ESP_OK;
        }
        if (copying) {
            xSemaphoreTake(s_dma_done, portMAX_DELAY);
            esp_cache_msync(s_raw[k & 1], len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
            rows = s_raw[k & 1];
            copying = false;
            if (k + 1 < stripes) {
                int n1 = h - y0 - STRIPE < STRIPE ? h - y0 - STRIPE : STRIPE;
                size_t len1 = ((size_t)(n1 - 1) * fr.pitch + w + 127) & ~(size_t)127;
                copying = esp_async_memcpy(s_dma, s_raw[(k + 1) & 1], (void *)(fr.pixels + (size_t)(y0 + STRIPE) * fr.pitch),
                                           len1, on_dma_done, NULL) == ESP_OK;
            }
        }
        /* this stripe buffer was the input of the operation before last */
        while (done < issued - 1) {
            xSemaphoreTake(s_ppa_done, portMAX_DELAY);
            done++;
        }
        uint16_t *stripe = s_stripe[k & 1];
        for (int y = 0; y < n; y++) {
            const uint32_t *src = (const uint32_t *)(rows + (size_t)y * fr.pitch);
            uint32_t *dst = (uint32_t *)(stripe + (size_t)y * w);
            for (int x = 0; x < w / 4; x++) {
                uint32_t p = src[x];
                dst[2 * x] = pal[p & 0xff] | (uint32_t)pal[(p >> 8) & 0xff] << 16;
                dst[2 * x + 1] = pal[(p >> 16) & 0xff] | (uint32_t)pal[p >> 24] << 16;
            }
        }
        fb_wait();
        /* 270 degrees counter-clockwise: the top row lands in the rightmost column */
        ppa_srm_oper_config_t op = {
            .in = {
                .buffer = stripe, .pic_w = w, .pic_h = n, .block_w = w, .block_h = n,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer = s_fb[back], .buffer_size = LCD_W * LCD_H * 2, .pic_w = LCD_W, .pic_h = LCD_H,
                .block_offset_x = left + out_h - (y0 + n) * ky / 16,
                .block_offset_y = (LCD_H - out_w) / 2,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_270,   /* as the BSP's LVGL rotation for the kit */
            .scale_x = kx / 16.0f,
            .scale_y = ky / 16.0f,
            .mode = PPA_TRANS_MODE_NON_BLOCKING,
        };
        ok = ppa_do_scale_rotate_mirror(s_ppa, &op) == ESP_OK;
        issued += ok;
    }
    if (copying) {
        xSemaphoreTake(s_dma_done, portMAX_DELAY);
    }
    while (done < issued) {
        xSemaphoreTake(s_ppa_done, portMAX_DELAY);
        done++;
    }
    if (!ok) {
        return;
    }
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_W, LCD_H, s_fb[back]);
    s_shown = back;
    /* fb_wait waits for the end of the refresh in progress now. A give left from before the
     * switch is dropped (when a refresh ended just after the switch, it waits one refresh more) */
    xSemaphoreTake(s_fb_free, 0);
    s_switching = true;
}

static void display_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        display_frame();
    }
}

/* ---- audio: ES8311 codec on I2S, mono speaker ---- */

#define AUDIO_CHUNK 256         /* frames, ~5 ms */

static esp_codec_dev_handle_t s_spk;

static esp_err_t audio_init(int volume)
{
    i2s_chan_handle_t tx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; /* silence, not the last buffers again, when the task falls behind */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, NULL), TAG, "I2S channel");
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MACHINE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = { .mclk = I2S_MCLK, .bclk = I2S_BCLK, .ws = I2S_WS, .dout = I2S_DOUT, .din = I2S_DIN },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "I2S mode");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "I2S enable");

    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .tx_handle = tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    audio_codec_i2c_cfg_t i2c_cfg = { .port = I2C_PORT, .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = s_i2c };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = audio_codec_new_gpio(),
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = PA_ENABLE,
        .use_mclk = true,
        .hw_gain = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
    };
    const audio_codec_if_t *es = data_if && ctrl_if ? es8311_codec_new(&es_cfg) : NULL;
    if (!es) {
        return ESP_FAIL;
    }
    esp_codec_dev_cfg_t dev_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = es, .data_if = data_if };
    s_spk = esp_codec_dev_new(&dev_cfg);
    esp_codec_dev_sample_info_t fs = { .sample_rate = MACHINE_AUDIO_RATE, .channel = 1, .bits_per_sample = 16 };
    if (!s_spk || esp_codec_dev_open(s_spk, &fs) != ESP_CODEC_DEV_OK) {
        return ESP_FAIL;
    }
    esp_codec_dev_set_out_vol(s_spk, volume);
    return ESP_OK;
}

/* Reads the machine's audio as the codec takes it; the writes block while the DMA buffers are
 * full, which paces the task. The emulation keeps guest time within a few ms of wall time, but
 * its cost per ms of guest time varies (flight has 10 ms stretches over real time), so playing
 * each frame as soon as it is made would run dry at every one of them. Playback starts once the
 * ring holds PRIME frames, and again after running dry, so the ring stays about that far ahead.
 * While dry the codec plays silence (auto_clear). */
#define PRIME (MACHINE_AUDIO_RATE * 60 / 1000)

static void audio_task(void *arg)
{
    static int16_t buf[AUDIO_CHUNK * 2], mono[AUDIO_CHUNK];
    bool primed = false;
    for (;;) {
        if (!primed && machine_audio_available(s_m) < PRIME) {
            vTaskDelay(1);
            continue;
        }
        primed = true;
        size_t n = machine_audio_read(s_m, buf, AUDIO_CHUNK);
        if (!n) {
            primed = false;
            continue;
        }
        for (size_t i = 0; i < n; i++) {
            mono[i] = (int16_t)((buf[2 * i] + buf[2 * i + 1]) >> 1);
        }
        esp_codec_dev_write(s_spk, mono, (int)(n * sizeof(int16_t)));
    }
}

/* ---- board.h ---- */

void board_init(board_t *b)
{
    b->drive_c = ROOT "/c";
    b->drive_d = ROOT "/d";
    if (card_mount() != ESP_OK) {
        ESP_LOGE(TAG, "no SD card");
    }
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .i2c_port = I2C_PORT,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &s_i2c));
    display_init();
    if (audio_init(CONFIG_TIE_VOLUME) != ESP_OK) {
        ESP_LOGE(TAG, "no audio output");
        s_spk = NULL;
    }
    usb_hid_start();
}

void board_start(machine_t *m)
{
    s_m = m;
    if (xTaskCreatePinnedToCore(display_task, "display", 4096, NULL, 4, &s_display, DISPLAY_CORE) != pdPASS) {
        ESP_LOGE(TAG, "no memory for the display task");
    }
    if (s_spk && xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 6, NULL, AUDIO_CORE) != pdPASS) {
        ESP_LOGE(TAG, "no memory for the audio task");
    }
}

/* When to present. In the banked 640x480 mode the game draws a frame straight into video memory,
 * a pass over the banks and a few shorter ones for the cockpit, then computes the next frame for
 * about 10 ms without touching it (in flight, a frame takes about 1M instructions, and the gaps
 * between bank moves while it is drawn are under 150k). So a frame is complete once the banks have
 * been still for QUIET_US: showing it then neither tears nor shows a half-drawn cockpit. Screens
 * that move no banks (menus, cursor moves inside a bank, the other modes) are shown on vertical
 * retrace. Either way at most every MIN_US (IDLE_MIN_US without bank moves) of guest time: each
 * frame shown costs PSRAM bandwidth. */
#define QUIET_US 3000
#define MIN_US 25000
#define IDLE_MIN_US 33000

void board_poll(machine_t *m)
{
    static uint32_t last_frame, banks, banks_shown;
    static uint64_t moved_at, shown_at;
    input_drain(m);
    uint64_t now = machine_time_us(m);
    uint32_t b = machine_vga_banks(m), f = machine_frame_count(m);
    if (b != banks) {
        banks = b;
        moved_at = now;
    }
    bool show;
    if (banks != banks_shown) {
        show = now - moved_at >= QUIET_US && now - shown_at >= MIN_US;
    } else {
        show = f != last_frame && now - shown_at >= IDLE_MIN_US;
    }
    last_frame = f;
    if (!show || !s_display) {
        return;
    }
    banks_shown = banks;
    shown_at = now;
    xTaskNotifyGive(s_display);
}

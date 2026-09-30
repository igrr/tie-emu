#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board.h"
#include "machine.h"

static const char *TAG = "tie";

#define IPS 50000000u           /* virtual CPU speed, guest instructions per second */
#define SLICE_US 20000u         /* guest time per machine_run() call */
#define POLL_US 1000u           /* input, frames and pacing, at most this often (machine_set_poll) */
/* The emulation task has core 1 to itself (the board's display and audio tasks run on core 0).
 * Its stack is in internal RAM, and translated code nests native calls in it (components/aot,
 * STACK_ROOM). */
#define EMU_STACK (48 * 1024)
#define EMU_CORE 1

/* machine_set_poll: input, frames, and keeping guest time in step with wall time. When the
 * emulation falls behind, guest time slips instead of bursting to catch up. */
static void emu_poll(machine_t *m, void *arg)
{
    int64_t *t0 = arg;          /* wall time at guest time 0 */
    board_poll(m);
    int64_t ahead = (int64_t)machine_time_us(m) - (esp_timer_get_time() - *t0);
    if (ahead > 2000) {
        vTaskDelay(pdMS_TO_TICKS(ahead / 1000));
    } else if (ahead < -100000) {
        *t0 -= ahead;
    }
}

static void emu_task(void *arg)
{
    machine_t *m = arg;
    int64_t t0 = esp_timer_get_time();
    machine_set_poll(m, emu_poll, &t0, POLL_US);
    for (;;) {
        machine_state_t st = machine_run(m, (uint64_t)IPS * SLICE_US / 1000000);
        if (st != MACH_RUNNING) {
            char buf[1024];
            machine_describe_fault(m, buf, sizeof(buf));
            ESP_LOGE(TAG, "machine stopped (%s): %s", st == MACH_EXITED ? "exit" : "fault", buf);
            break;
        }
        emu_poll(m, &t0);       /* in case no device event came in the whole slice */
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    board_t board;
    board_init(&board);
    machine_cfg_t cfg = {
        .drive_c = board.drive_c,
        .drive_d = board.drive_d,
        .cwd = "\\",
        .mem_size = 16u << 20,
        .ips = IPS,
    };
    machine_t *m = machine_create(&cfg);
    /* /trebla: what Z_TIE2__.EXE, which TIE.EXE runs, starts the game with */
    if (!m || machine_load(m, "C:\\Z_TIE__.EXE", "/trebla") != 0) {
        char buf[512] = "out of memory";
        if (m) {
            machine_describe_fault(m, buf, sizeof(buf));
        }
        ESP_LOGE(TAG, "cannot start the game: %s", buf);
        return;
    }
    if (!machine_aot(m)) {
        ESP_LOGW(TAG, "Z_TIE__.EXE is not the one translated at build time: interpreter only");
    }
    board_start(m);
    if (xTaskCreatePinnedToCore(emu_task, "emu", EMU_STACK, m, 5, NULL, EMU_CORE) != pdPASS) {
        ESP_LOGE(TAG, "no memory for the emulation task's stack");
    }
}

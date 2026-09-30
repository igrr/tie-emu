/* The emulated DOS/4GW PC that runs Z_TIE__.EXE.
 *
 * Replaces DOS, the DOS/4GW extender and the BIOS with high-level emulation, and
 * models the devices the game programs directly (PIC, PIT, VGA, keyboard controller)
 * at register level. Files are reached through POSIX stdio (the SD card's VFS mount).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct machine machine_t;

typedef struct {
    const char *drive_c;        /* host directory mounted as C: */
    const char *drive_d;        /* host directory mounted as D: (the CD), or NULL */
    const char *cwd;            /* DOS current directory on C:, e.g. "\\TIE" */
    uint32_t mem_size;          /* guest RAM, bytes */
    uint32_t ips;               /* virtual CPU speed: guest instructions per second of guest time */
} machine_cfg_t;

typedef enum {
    MACH_RUNNING,
    MACH_EXITED,                /* the program terminated (INT 21h/4Ch) */
    MACH_FAULT,                 /* CPU exception or unimplemented service; machine_describe_fault() */
} machine_state_t;

/* A finished VGA frame: 8-bit indexed pixels and the 256-entry palette as 0x00RRGGBB */
typedef struct {
    const uint8_t *pixels;
    uint32_t size;              /* bytes readable from pixels on (the rest of video memory) */
    uint32_t pitch;
    uint16_t width, height;
    const uint32_t *palette;
} machine_frame_t;

machine_t *machine_create(const machine_cfg_t *cfg);

/* Loads an LE executable (DOS path, e.g. "C:\\TIE\\Z_TIE__.EXE") and prepares the CPU at its entry */
int machine_load(machine_t *m, const char *dos_path, const char *args);

/* True when machine_load found translated code for the program */
bool machine_aot(const machine_t *m);

/* Runs about `insns` guest instructions, servicing timers and interrupts */
machine_state_t machine_run(machine_t *m, uint64_t insns);

/* Has machine_run() call fn at most every period_us of guest time, where it stops the CPU for
 * a device event anyway. fn may call machine_key() and machine_mouse(). NULL: none. */
typedef void (*machine_poll_fn)(machine_t *m, void *arg);
void machine_set_poll(machine_t *m, machine_poll_fn fn, void *arg, uint32_t period_us);

/* Guest time in microseconds, derived from executed instructions */
uint64_t machine_time_us(const machine_t *m);

/* Host input: PC/XT set 1 scancode bytes (0xE0 prefixes included, bit 7 = release) */
void machine_key(machine_t *m, uint8_t scancode);
/* Host mouse: relative motion in mickeys and the button state (bit 0 left, bit 1 right) */
void machine_mouse(machine_t *m, int dx, int dy, unsigned buttons);

/* The current screen; valid until the next machine_run() */
void machine_frame(machine_t *m, machine_frame_t *out);
/* Frames the guest completed (vertical retraces passed) since creation */
uint32_t machine_frame_count(const machine_t *m);

#define MACHINE_AUDIO_RATE 48000
size_t machine_audio_read(machine_t *m, int16_t *frames, size_t max_frames);
/* Frames waiting in the ring */
size_t machine_audio_available(machine_t *m);

/* VESA window moves since creation: the game moves the window over the banks as it draws a
 * frame in a banked mode */
uint32_t machine_vga_banks(const machine_t *m);

void machine_describe_fault(const machine_t *m, char *buf, int len);

#ifdef __cplusplus
}
#endif

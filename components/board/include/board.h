/* What the emulator needs from the board it runs on (the TIE_BOARD option selects the board) */
#pragma once

#include "machine.h"

typedef struct {
    const char *drive_c;            /* the game directory */
    const char *drive_d;            /* the CD's files */
} board_t;

/* Mounts the game's files, creates the display and the audio output, and starts taking input */
void board_init(board_t *b);

/* The machine is ready: starts showing its frames and playing its audio. Tasks on the other core
 * read machine_frame() and machine_audio_read() from now on, so the emulation task only has to
 * tell them about new frames (board_poll()). */
void board_start(machine_t *m);

/* Called by the emulation task at every poll (machine_set_poll()): hands the input that came in
 * to the machine, and a finished frame to the display */
void board_poll(machine_t *m);

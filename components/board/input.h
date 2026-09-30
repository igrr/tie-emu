/* Input to the emulated PC: a single-producer single-consumer queue from the board's input
 * source (the USB HID task) to the emulation task. */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "machine.h"

typedef enum { INPUT_KEY, INPUT_MOUSE } input_type_t;

typedef struct {
    uint8_t type;
    uint8_t buttons;    /* mouse: bit 0 left, bit 1 right */
    uint16_t code;      /* key: USB HID usage */
    bool down;
    int16_t dx, dy;
} input_event_t;

bool input_push(const input_event_t *ev);
/* Applies queued events to the machine; call from the emulation task */
void input_drain(machine_t *m);

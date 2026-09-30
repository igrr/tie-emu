/* OPL3 (YMF262) tone generator. The machine owns the register interface and timers
 * (sound.c); this renders what the registers describe, as ymfm does (opl3.c). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The chip's native sample rate: 14.31818 MHz / 288 */
#define OPL3_RATE 49716

typedef struct opl3 opl3_t;

opl3_t *opl3_create(void);
/* bank 0 is the 388h pair, bank 1 the 38Ah pair */
void opl3_write(opl3_t *o, int bank, uint8_t reg, uint8_t v);
/* Renders one stereo sample at OPL3_RATE (outputs A and B; C and D are not wired on the SB16) */
void opl3_sample(opl3_t *o, int16_t *l, int16_t *r);

#ifdef __cplusplus
}
#endif

#pragma once

#include <inttypes.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include "machine.h"
#include "x86.h"
#include "opl3.h"

/* Guest linear memory map */
#define MEM_BDA         0x00400u
#define MEM_PSP         0x08000u    /* program segment prefix (real-mode segment 0x0800) */
#define MEM_ENV         0x08200u    /* environment block */
#define MEM_DOS_LOW     0x10000u    /* conventional memory handed out by DPMI 0100h / INT 21h 48h */
#define MEM_DOS_HIGH    0xa0000u
#define MEM_VGA         0xa0000u
#define MEM_VGA_SIZE    0x20000u
#define MEM_STUBS       0xf0000u    /* default interrupt handlers: 0F FF nn CF, 4 bytes per vector */
#define MEM_IMAGE_DELTA 0x100000u   /* LE objects are loaded at their link address plus this */

/* Selectors; the table index is sel >> 3 (TI and RPL ignored) */
#define SEL_CODE        0x08
#define SEL_DATA        0x10
#define SEL_PSP         0x24
#define SEL_ENV         0x2c
#define SEL_FIRST_FREE  0x40
#define NUM_DESC        512

#define PIT_HZ          1193182u
#define MAX_FILES       64
#define DIR_CACHE       8       /* host directory listings kept (dos.c) */
#define CLOSED_FILES    16      /* read-only files kept open after the program closes them (dos.c) */
#define MAX_DOS_BLOCKS  64
#define MAX_XMS_BLOCKS  256

typedef struct {
    uint32_t base, limit;
    bool used;
    bool code;
} desc_t;

typedef struct {
    uint16_t sel;
    uint32_t off;
} farptr_t;

typedef struct {
    char *name;
    uint8_t kind;       /* 1 a file, 2 a directory, 0 not known (a symlink) */
} dos_dirent_t;

typedef struct {
    FILE *f;
    bool used;
    bool device;        /* stdin/stdout/stderr/aux/prn */
    bool wrote;         /* the last access was a write (stdio needs a seek before switching direction) */
    char *key;          /* opened read-only: its drive and DOS path, to keep it open after the close */
} dosfile_t;

typedef struct {
    uint32_t base, size;    /* linear */
    bool used;
} memblock_t;

typedef struct {
    /* 8254 channel */
    uint16_t reload;        /* 0 means 65536 */
    uint8_t mode;
    uint8_t access;         /* 1 lo, 2 hi, 3 lo/hi */
    bool write_hi_next, read_hi_next;
    bool latched;
    uint16_t latch;
    uint64_t start_tick;    /* PIT tick when the count was loaded */
    bool gate;
} pit_chan_t;

typedef struct {
    uint8_t imr, irr, isr;
    uint8_t icw_step;
    bool read_isr;
    uint8_t vector_base;
} pic_t;

typedef struct {
    uint16_t base_addr, base_count;
    uint32_t done;          /* transfers since programming (bytes or words) */
    uint8_t page, mode;
    bool masked, terminal;
} dma_chan_t;

typedef struct {
    uint8_t mixer_idx, mixer[256];
    bool resetting;
    uint8_t out[64];
    int out_head, out_n;
    uint8_t cmd, args[4];
    int nargs, argi;
    bool in_cmd;
    uint8_t test;
    bool speaker;
    uint32_t rate;
    uint16_t block_size;
    /* running transfer */
    bool active, paused, bits16, autoinit, stereo, is_signed;
    uint32_t block;         /* sample frames per block */
    uint64_t block_start;   /* PIT tick */
    uint32_t frames_done;
    uint32_t len_block, len_rate;   /* block length in PIT ticks for this block and rate */
    uint64_t len_ticks;
} sb16_t;

typedef struct {
    bool running, expired, masked;
    uint64_t expire;        /* PIT tick */
} opl_timer_t;

#define AUDIO_RING 16384u   /* stereo frames, a power of two (~340 ms) */
#define AUDIO_EVENTS 4096u  /* a power of two */

/* A change the reader makes before it mixes ring frame `pos`: an OPL register write
 * (AUDIO_EV_OPL | bank << 16 | reg << 8 | value) or a mixer gain (AUDIO_EV_GAIN, see sound.c) */
typedef struct {
    uint32_t pos;
    uint32_t data;
} audio_event_t;

/* SB16 transfer position at output frame k, kept from one frame to the next without dividing:
 * pos * 65536 = (idx * 65536 + frac) * den + rem, where pos / den is the sample frame */
typedef struct {
    uint64_t k, block_start;    /* valid for this frame, block and rate */
    uint32_t rate;
    uint32_t idx, frac;
    uint64_t rem;
    uint32_t step_idx, step_frac;   /* the same for one output frame */
    uint64_t step_rem;
} sb_cursor_t;

/* The emulation (the producer) renders the SB16's PCM along guest time into the ring, and queues
 * what the OPL3 and the mixer are told as events at ring positions. The reader (machine_audio_read,
 * the consumer, possibly on another core) synthesizes the OPL3 and mixes: the costly part. */
typedef struct {
    /* producer */
    uint64_t frames;        /* output frames rendered since creation; frame k is at k / MACHINE_AUDIO_RATE s */
    uint64_t frames_at;     /* frames * PIT_HZ: the next frame's time in 1 / (PIT_HZ * MACHINE_AUDIO_RATE) s */
    int32_t gain_pcm[2], gain_fm[2];    /* Q15 per channel, from the mixer */
    sb_cursor_t cur;
    /* single producer, single consumer */
    int16_t ring[AUDIO_RING * 2];       /* PCM before the mixer */
    audio_event_t ev[AUDIO_EVENTS];
    atomic_uint_fast32_t head, tail, ev_head, ev_tail;
    /* the reader's state is used by the reader, and by the producer when the events overflow:
     * Peterson's lock between the two (loads and stores only; PSRAM may not do atomic swaps) */
    atomic_int want[2], turn;
    /* reader */
    opl3_t *opl;
    int32_t mix_pcm[2], mix_fm[2];
    /* OPL resampling, for the last frame read: (frame * OPL3_RATE) % MACHINE_AUDIO_RATE, and the
     * native samples at and after that frame */
    int32_t opl_rem;
    int16_t opl_s0[2], opl_s1[2];
} audio_out_t;

struct machine {
    machine_cfg_t cfg;
    x86_cpu cpu;
    uint8_t *mem;
    uint32_t mem_size;
    machine_state_t state;
    bool halted;            /* HLT executed; waiting for an interrupt */
    machine_poll_fn poll;   /* machine_set_poll */
    void *poll_arg;
    uint64_t poll_period, poll_next;    /* in instructions */
    bool key_wait;          /* a keyboard read found no key and re-executes its INT (mach_wait_key) */
    bool aot;               /* translated code matches the loaded program */
    uint32_t vga_banks;     /* machine_vga_banks */
    char fault[256];

    desc_t desc[NUM_DESC];
    farptr_t pmvec[256];
    farptr_t rmvec[256];

    /* DOS */
    char drive_c[256], drive_d[256];
    int cur_drive;          /* 2 = C: */
    uint8_t con_scan;       /* INT 21h console input: an extended key's scan code, for the next read */
    char cwd[4][128];       /* per drive, DOS form without the drive, e.g. "\\TIE" */
    dosfile_t files[MAX_FILES];
    uint32_t dta;           /* linear */
    memblock_t dosmem[MAX_DOS_BLOCKS];
    memblock_t xms[MAX_XMS_BLOCKS];
    uint32_t heap_lo, heap_hi;  /* linear range for DPMI 0501h */
    struct {
        char (*names)[13];  /* the search's matches, sorted by name */
        int n, next;
        char pattern[16];
        char host_dir[512];
        uint8_t attr;
    } find;
    /* Opening a file on FATFS reads the directories on its path from the card, and matching
     * names without regard to case lists them: the game opens ~10 files a second while it loads,
     * most of them again and again. Directory listings are kept until the program changes a
     * directory, and files it opened read-only stay open for a while after it closes them. */
    struct {
        char *path;             /* host */
        dos_dirent_t *ents;     /* sorted by name ignoring case */
        int n;
    } dirs[DIR_CACHE];
    int dirs_next;
    struct {
        FILE *f;
        char *key;
    } closed[CLOSED_FILES];
    int closed_next;

    /* devices */
    pic_t pic[2];
    pit_chan_t pit[3];
    uint64_t next_irq0_tick;
    uint8_t port61;
    /* time conversions, kept incrementally (mach_pit_ticks, machine.c) */
    struct {
        uint64_t icount, ticks;
        uint32_t frac;          /* icount * PIT_HZ - ticks * ips */
        uint32_t max_step;      /* the largest icount step updated in 32 bits */
        uint64_t ev_tick, ev_icount;    /* the last ticks_to_icount */
    } clk;
    struct {
        uint8_t dac_write_idx, dac_read_idx, dac_comp;
        uint8_t dac[256][3];    /* 6-bit */
        uint32_t palette[256];
        uint8_t crtc_idx, crtc[32];
        uint8_t gc_idx, gc[16];
        uint8_t seq_idx, seq[8];
        uint8_t attr_idx; bool attr_flip; uint8_t attr[32];
        uint8_t misc;
        int mode;               /* BIOS mode number, or 0x100+ for VESA */
        uint16_t width, height;
        uint32_t pitch;
        uint32_t bank;          /* current 64 KB window for banked VESA modes */
        uint8_t *vram;          /* backing store for banked modes (VESA) */
        uint32_t vram_size;
        uint32_t frames;
    } vga;
    struct {
        uint8_t buf[32];
        int head, tail;
        uint8_t data;           /* last byte presented at port 60h */
        uint8_t cmd;
        bool irq_pending;
    } kbd;
    struct {
        int x, y;
        int dx, dy;             /* mickeys since the last read */
        unsigned buttons;
        int minx, maxx, miny, maxy;
        bool shown;
    } mouse;

    struct {
        dma_chan_t dma[8];
        bool dma_ff[2];
        sb16_t sb;
        uint8_t opl_idx[2];
        uint8_t opl_regs[2][256];
        uint8_t opl_status;
        opl_timer_t opl_timer[2];
        audio_out_t out;
    } snd;
};

/* Division that takes the 32-bit path when both operands fit: RV32 divides 32-bit values
 * in hardware, 64-bit ones in a slow library routine */
static inline uint64_t mach_div(uint64_t n, uint64_t d)
{
    return (n | d) >> 32 ? n / d : (uint32_t)n / (uint32_t)d;
}
static inline uint64_t mach_mod(uint64_t n, uint64_t d)
{
    return (n | d) >> 32 ? n % d : (uint32_t)n % (uint32_t)d;
}

/* machine.c */
void mach_log(machine_t *m, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void mach_fault(machine_t *m, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
uint64_t mach_pit_ticks(const machine_t *m);
void mach_raise_irq(machine_t *m, int irq);
/* The device dropped its request line: a request not yet acknowledged is withdrawn (8259) */
void mach_lower_irq(machine_t *m, int irq);
int desc_alloc(machine_t *m, uint32_t base, uint32_t limit);
static inline uint32_t sel_base(const machine_t *m, uint16_t sel)
{
    return m->desc[(sel >> 3) % NUM_DESC].base;
}
static inline uint8_t *lin_ptr(machine_t *m, uint32_t lin, uint32_t len)
{
    return x86_ptr(&m->cpu, lin, len);
}
static inline farptr_t stub_vector(int vec)
{
    return (farptr_t){ SEL_CODE, MEM_STUBS + 4u * vec };
}

/* le_load.c */
int le_load(machine_t *m, const char *host_path, uint32_t delta, uint32_t *eip, uint32_t *esp);

/* dos.c */
void dos_init(machine_t *m);
void dos_int21(machine_t *m);
void dos_find_end(machine_t *m);
/* Closes the host file of handle h unless another handle shares it (INT 21h/45h) */
void dos_close_file(machine_t *m, int h);
bool dos_host_path(machine_t *m, const char *dos_path, char *out, int outlen, bool must_exist);
uint32_t dos_alloc(machine_t *m, uint32_t paras, uint32_t *largest);
bool dos_free(machine_t *m, uint32_t lin);

/* dpmi.c */
void dpmi_int31(machine_t *m);
void dpmi_int2f(machine_t *m);

/* bios.c */
void bios_init(machine_t *m);
void bios_int10(machine_t *m, uint32_t *regs_rm);
void bios_int16(machine_t *m);
void bios_int1a(machine_t *m);
void bios_int33(machine_t *m);
void bios_irq0(machine_t *m);
void bios_irq1(machine_t *m);
/* BIOS keyboard buffer: scancode << 8 | ASCII */
bool bios_key_get(machine_t *m, uint16_t *key, bool remove);
/* In a keyboard read's INT handler when no key is there: re-executes the INT, as DOS and the BIOS
 * wait. The loop can only end when an interrupt adds a key, so machine_run skips it ahead to the
 * next device event, counting the instructions the loop would have executed (one per INT). */
void mach_wait_key(machine_t *m);

/* devices */
void pic_init(machine_t *m);
uint8_t pic_in(machine_t *m, uint16_t port);
void pic_out(machine_t *m, uint16_t port, uint8_t v);
int pic_next_irq(machine_t *m);
void pic_update_intr(machine_t *m);
void pic_ack(machine_t *m, int irq);
void pit_init(machine_t *m);
uint8_t pit_in(machine_t *m, uint16_t port);
void pit_out(machine_t *m, uint16_t port, uint8_t v);
void pit_update(machine_t *m);
uint64_t pit_next_event(machine_t *m);
void vga_init(machine_t *m);
uint8_t vga_in(machine_t *m, uint16_t port);
uint64_t vga_status_until(machine_t *m, uint8_t mask);
void vga_out(machine_t *m, uint16_t port, uint8_t v);
void vga_set_mode(machine_t *m, int mode);
void vga_set_bank(machine_t *m, uint32_t bank);
void sound_init(machine_t *m);
bool sound_in(machine_t *m, uint16_t port, uint8_t *v);
bool sound_out(machine_t *m, uint16_t port, uint8_t v);
/* Whether an access to port moves no sound event and changes nothing pending by itself (machine.c,
 * quiet_port) */
bool sound_port_quiet(const machine_t *m, uint16_t port, bool out);
void sound_tick(machine_t *m);
/* Renders audio up to the current time without touching guest-visible state */
void sound_flush(machine_t *m);
uint64_t sound_next_event(machine_t *m);
uint8_t kbd_in(machine_t *m, uint16_t port);
void kbd_out(machine_t *m, uint16_t port, uint8_t v);
void kbd_update(machine_t *m);

/* Register helpers for HLE; the 8/16-bit views are plain unsigned (printf %x) */
#define EAX (m->cpu.r[X86_EAX])
#define ECX (m->cpu.r[X86_ECX])
#define EDX (m->cpu.r[X86_EDX])
#define EBX (m->cpu.r[X86_EBX])
#define ESI (m->cpu.r[X86_ESI])
#define EDI (m->cpu.r[X86_EDI])
#define EBP (m->cpu.r[X86_EBP])
#define AL ((unsigned)(EAX & 0xff))
#define AH ((unsigned)((EAX >> 8) & 0xff))
#define AX ((unsigned)(EAX & 0xffff))
#define BL ((unsigned)(EBX & 0xff))
#define BH ((unsigned)((EBX >> 8) & 0xff))
#define BX ((unsigned)(EBX & 0xffff))
#define CX ((unsigned)(ECX & 0xffff))
#define CL ((unsigned)(ECX & 0xff))
#define CH ((unsigned)((ECX >> 8) & 0xff))
#define DX ((unsigned)(EDX & 0xffff))
#define DL ((unsigned)(EDX & 0xff))
#define DH ((unsigned)((EDX >> 8) & 0xff))
#define SI ((unsigned)(ESI & 0xffff))
#define DI ((unsigned)(EDI & 0xffff))
#define SET_AX(v) (EAX = (EAX & 0xffff0000u) | ((v) & 0xffff))
#define SET_AL(v) (EAX = (EAX & 0xffffff00u) | ((v) & 0xff))
#define SET_AH(v) (EAX = (EAX & 0xffff00ffu) | (((v) & 0xff) << 8))
#define SET_BX(v) (EBX = (EBX & 0xffff0000u) | ((v) & 0xffff))
#define SET_CX(v) (ECX = (ECX & 0xffff0000u) | ((v) & 0xffff))
#define SET_DX(v) (EDX = (EDX & 0xffff0000u) | ((v) & 0xffff))
#define SET_SI(v) (ESI = (ESI & 0xffff0000u) | ((v) & 0xffff))
#define SET_DI(v) (EDI = (EDI & 0xffff0000u) | ((v) & 0xffff))
#define SET_CF() (m->cpu.eflags |= X86_CF)
#define CLR_CF() (m->cpu.eflags &= ~X86_CF)
#define SEG_BASE(s) (m->cpu.base[s])

/* INT 21h as DOS/4GW presents it to a flat 32-bit program: pointers are DS:EDX-style
 * 32-bit offsets, counts are ECX, results in EAX. Drives map to host directories;
 * names are matched case-insensitively. */
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "aot.h"
#include "machine_priv.h"

enum {
    DOSERR_FUNC = 1,
    DOSERR_NOFILE = 2,
    DOSERR_NOPATH = 3,
    DOSERR_TOOMANY = 4,
    DOSERR_ACCESS = 5,
    DOSERR_HANDLE = 6,
    DOSERR_NOMEM = 8,
    DOSERR_BADBLOCK = 9,
    DOSERR_NOMORE = 18,
};

static void fail(machine_t *m, int err)
{
    SET_AX(err);
    SET_CF();
}

static char *guest_str(machine_t *m, uint32_t lin, char *buf, int len)
{
    int i = 0;
    for (; i < len - 1; i++) {
        uint8_t *p = lin_ptr(m, lin + i, 1);
        if (!p || !*p) {
            break;
        }
        buf[i] = (char)*p;
    }
    buf[i] = 0;
    return buf;
}

/* ---- paths ---- */

static const char *drive_root(machine_t *m, int drive)
{
    if (drive == 2) {
        return m->drive_c;
    }
    if (drive == 3 && m->drive_d[0]) {
        return m->drive_d;
    }
    return NULL;
}

/* "a:\x\..\y" -> drive, and "\X\Y" normalized, uppercase */
static bool dos_normalize(machine_t *m, const char *in, int *drive, char *out, int outlen)
{
    int drv = m->cur_drive;
    if (in[0] && in[1] == ':') {
        drv = toupper((unsigned char)in[0]) - 'A';
        in += 2;
    }
    if (drv < 0 || drv > 3) {
        return false;
    }
    char tmp[256];
    if (in[0] == '\\' || in[0] == '/') {
        snprintf(tmp, sizeof(tmp), "%s", in);
    } else {
        snprintf(tmp, sizeof(tmp), "%s\\%s", m->cwd[drv], in);
    }
    /* split and rebuild, resolving . and .. */
    char *parts[32];
    int np = 0;
    for (char *t = strtok(tmp, "\\/"); t; t = strtok(NULL, "\\/")) {
        if (!strcmp(t, ".")) {
            continue;
        }
        if (!strcmp(t, "..")) {
            if (np) {
                np--;
            }
            continue;
        }
        if (np < 32) {
            parts[np++] = t;
        }
    }
    int n = 0;
    out[0] = 0;
    for (int i = 0; i < np; i++) {
        n += snprintf(out + n, outlen - n, "\\%s", parts[i]);
    }
    if (!np) {
        snprintf(out, outlen, "\\");
    }
    for (char *p = out; *p; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
    *drive = drv;
    return true;
}

static void dir_free(machine_t *m, int i)
{
    for (int k = 0; k < m->dirs[i].n; k++) {
        free(m->dirs[i].ents[k].name);
    }
    free(m->dirs[i].ents);
    free(m->dirs[i].path);
    m->dirs[i].ents = NULL;
    m->dirs[i].path = NULL;
    m->dirs[i].n = 0;
}

static void closed_free(machine_t *m, int i)
{
    if (m->closed[i].f) {
        fclose(m->closed[i].f);
    }
    free(m->closed[i].key);
    m->closed[i].f = NULL;
    m->closed[i].key = NULL;
}

static void close_kept(machine_t *m)
{
    for (int i = 0; i < CLOSED_FILES; i++) {
        closed_free(m, i);
    }
}

/* Before the program changes a directory or opens a file to write it: what is kept of the
 * directories and the files may be out of date after it */
static void forget(machine_t *m)
{
    for (int i = 0; i < DIR_CACHE; i++) {
        dir_free(m, i);
    }
    close_kept(m);
}

static int dirent_cmp(const void *a, const void *b)
{
    return strcasecmp(((const dos_dirent_t *)a)->name, ((const dos_dirent_t *)b)->name);
}

/* Host directory `dir`, listed on first use; -1 when it cannot be read */
static int dir_list(machine_t *m, const char *dir)
{
    for (int i = 0; i < DIR_CACHE; i++) {
        if (m->dirs[i].path && !strcmp(m->dirs[i].path, dir)) {
            return i;
        }
    }
    DIR *d = opendir(dir);
    if (!d) {
        return -1;
    }
    int i = m->dirs_next;
    m->dirs_next = (i + 1) % DIR_CACHE;
    dir_free(m, i);
    int cap = 0, n = 0;
    dos_dirent_t *ents = NULL;
    struct dirent *e;
    while ((e = readdir(d))) {
        if (n == cap) {
            void *p = realloc(ents, (cap = cap ? 2 * cap : 32) * sizeof(*ents));
            if (!p) {
                break;
            }
            ents = p;
        }
        if (!(ents[n].name = strdup(e->d_name))) {
            break;
        }
        ents[n++].kind = e->d_type == DT_REG ? 1 : e->d_type == DT_DIR ? 2 : 0;
    }
    closedir(d);
    qsort(ents, n, sizeof(*ents), dirent_cmp);
    m->dirs[i].path = strdup(dir);
    m->dirs[i].ents = ents;
    m->dirs[i].n = n;
    return i;
}

/* Finds `name` in host directory `dir` ignoring case; writes the host name, and its kind
 * (dos_dirent_t) */
static bool match_ci(machine_t *m, const char *dir, const char *name, char *out, int outlen, int *kind)
{
    int i = dir_list(m, dir);
    if (i < 0) {
        return false;
    }
    dos_dirent_t key = { .name = (char *)name };
    const dos_dirent_t *e = bsearch(&key, m->dirs[i].ents, m->dirs[i].n, sizeof(key), dirent_cmp);
    if (!e) {
        return false;
    }
    snprintf(out, outlen, "%s", e->name);
    *kind = e->kind;
    return true;
}

/* norm (dos_normalize) under host directory root */
static bool host_path_in(machine_t *m, const char *root, const char *norm, char *out, int outlen, bool must_exist,
                         int *kind)
{
    int n = snprintf(out, outlen, "%s", root);
    char *save = NULL;
    char *comps = strdup(norm);
    char *c = strtok_r(comps, "\\", &save);
    *kind = 2;
    while (c) {
        char *next = strtok_r(NULL, "\\", &save);
        char host[256];
        if (match_ci(m, out, c, host, sizeof(host), kind)) {
            n += snprintf(out + n, outlen - n, "/%s", host);
        } else if (!next && !must_exist) {
            *kind = 0;
            n += snprintf(out + n, outlen - n, "/%s", c);
        } else {
            free(comps);
            return false;
        }
        c = next;
    }
    free(comps);
    return n < outlen;
}

/* dos_host_path, and the kind of what it found (dos_dirent_t; 0 also when it found nothing) */
static bool host_path(machine_t *m, const char *dos_path, char *out, int outlen, bool must_exist, int *kind)
{
    int drive;
    char norm[256];
    if (!dos_normalize(m, dos_path, &drive, norm, sizeof(norm))) {
        return false;
    }
    const char *root = drive_root(m, drive);
    return root && host_path_in(m, root, norm, out, outlen, must_exist, kind);
}

bool dos_host_path(machine_t *m, const char *dos_path, char *out, int outlen, bool must_exist)
{
    int kind;
    return host_path(m, dos_path, out, outlen, must_exist, &kind);
}

/* ---- file handles ---- */

static int alloc_handle(machine_t *m)
{
    for (int i = 5; i < MAX_FILES; i++) {
        if (!m->files[i].used) {
            return i;
        }
    }
    return -1;
}

void dos_close_file(machine_t *m, int h)
{
    dosfile_t *f = &m->files[h];
    bool shared = false;
    for (int i = 0; i < MAX_FILES; i++) {
        shared |= i != h && m->files[i].used && m->files[i].f == f->f;
    }
    if (!shared && f->f && f->key) {
        /* kept open, for the program to open again (do_open) */
        int i = m->closed_next;
        m->closed_next = (i + 1) % CLOSED_FILES;
        closed_free(m, i);
        m->closed[i].f = f->f;
        m->closed[i].key = f->key;
    } else if (!shared) {
        if (f->f) {
            fclose(f->f);
        }
        free(f->key);
    }
    f->f = NULL;
    f->key = NULL;
}

static dosfile_t *handle(machine_t *m, int h)
{
    return h >= 0 && h < MAX_FILES && m->files[h].used ? &m->files[h] : NULL;
}

static void do_open(machine_t *m, bool create, uint8_t mode)
{
    char name[256], host[512];
    guest_str(m, SEG_BASE(X86_DS) + EDX, name, sizeof(name));
    if (m->aot) {
        aot_invalidate(); /* code the program loads comes from a file (IMUSE.EXE) */
    }
    int h = alloc_handle(m);
    if (h < 0) {
        fail(m, DOSERR_TOOMANY);
        return;
    }
    if (!strcasecmp(name, "NUL") || !strcasecmp(name, "CON")) {
        m->files[h] = (dosfile_t){ .used = true, .device = true };
        SET_AX(h);
        CLR_CF();
        return;
    }
    /* read-only: a file kept open since the program closed it (dos_close_file) is as good as new */
    bool ro = !create && (mode & 3) == 0;
    char key[260];
    int drive;
    if (ro && dos_normalize(m, name, &drive, key + 1, sizeof(key) - 1)) {
        key[0] = (char)('A' + drive);
        for (int i = 0; i < CLOSED_FILES; i++) {
            if (m->closed[i].key && !strcmp(m->closed[i].key, key)) {
                rewind(m->closed[i].f);
                m->files[h] = (dosfile_t){ .used = true, .f = m->closed[i].f, .key = m->closed[i].key };
                m->closed[i].f = NULL;
                m->closed[i].key = NULL;
                SET_AX(h);
                CLR_CF();
                return;
            }
        }
    } else {
        ro = false;
    }
    if (!ro) {
        forget(m);
    }
    int kind;
    if (!host_path(m, name, host, sizeof(host), !create, &kind)) {
        fail(m, DOSERR_NOFILE);
        return;
    }
    struct stat st;
    if (!create && (kind == 2 || (kind == 0 && stat(host, &st) == 0 && S_ISDIR(st.st_mode)))) {
        /* The game opens "D:" when a file is not on C:, and reads nothing from it. FATFS
         * refuses to open a directory: an empty handle instead. */
        m->files[h] = (dosfile_t){ .used = true };
        SET_AX(h);
        CLR_CF();
        return;
    }
    const char *fm = create ? "w+b" : (mode & 3) == 0 ? "rb" : "r+b";
    FILE *f = fopen(host, fm);
    if (!f && (errno == EMFILE || errno == ENFILE)) {
        close_kept(m);      /* the host's limit: the files kept open count too */
        f = fopen(host, fm);
    }
    if (!f && !create && (mode & 3) != 0) {
        f = fopen(host, "rb"); /* read-only media: fall back to reading */
    }
    if (!f) {
        fail(m, errno == ENOENT ? DOSERR_NOFILE : DOSERR_ACCESS);
        return;
    }
    m->files[h] = (dosfile_t){ .used = true, .f = f, .key = ro ? strdup(key) : NULL };
    SET_AX(h);
    CLR_CF();
}

/* ---- memory (conventional, for real-mode buffers) ---- */

uint32_t dos_alloc(machine_t *m, uint32_t paras, uint32_t *largest)
{
    uint32_t size = paras * 16;
    /* first fit over the blocks sorted by address */
    uint32_t at = MEM_DOS_LOW;
    for (;;) {
        bool moved = false;
        for (int i = 0; i < MAX_DOS_BLOCKS; i++) {
            memblock_t *b = &m->dosmem[i];
            if (b->used && at < b->base + b->size && b->base < at + size) {
                at = (b->base + b->size + 15) & ~15u;
                moved = true;
            }
        }
        if (!moved) {
            break;
        }
    }
    if (at + size > MEM_DOS_HIGH) {
        if (largest) {
            *largest = at < MEM_DOS_HIGH ? (MEM_DOS_HIGH - at) / 16 : 0;
        }
        return 0;
    }
    for (int i = 0; i < MAX_DOS_BLOCKS; i++) {
        if (!m->dosmem[i].used) {
            m->dosmem[i] = (memblock_t){ .base = at, .size = size, .used = true };
            memset(m->mem + at, 0, size);
            return at;
        }
    }
    return 0;
}

bool dos_free(machine_t *m, uint32_t lin)
{
    for (int i = 0; i < MAX_DOS_BLOCKS; i++) {
        if (m->dosmem[i].used && m->dosmem[i].base == lin) {
            m->dosmem[i].used = false;
            return true;
        }
    }
    return false;
}

/* ---- find first / next ---- */

static bool wild_match(const char *pat, const char *name)
{
    /* DOS 8.3 semantics: match base and extension separately; '*' eats the rest of its part */
    char pb[16] = "", pe[8] = "", nb[16] = "", ne[8] = "";
    const char *dot = strchr(pat, '.');
    snprintf(pb, sizeof(pb), "%.*s", dot ? (int)(dot - pat) : (int)strlen(pat), pat);
    if (dot) {
        snprintf(pe, sizeof(pe), "%s", dot + 1);
    } else {
        snprintf(pe, sizeof(pe), "%s", "");
    }
    dot = strrchr(name, '.');
    snprintf(nb, sizeof(nb), "%.*s", dot ? (int)(dot - name) : (int)strlen(name), name);
    snprintf(ne, sizeof(ne), "%s", dot ? dot + 1 : "");
    const char *pp[2] = { pb, pe }, *np[2] = { nb, ne };
    for (int k = 0; k < 2; k++) {
        const char *p = pp[k], *n = np[k];
        for (;;) {
            if (*p == '*') {
                break;
            }
            if (!*p) {
                if (*n) {
                    return false;
                }
                break;
            }
            if (*p == '?') {
                p++;
                if (*n) {
                    n++;
                }
                continue;
            }
            if (toupper((unsigned char)*p) != toupper((unsigned char)*n)) {
                return false;
            }
            p++;
            n++;
        }
    }
    return true;
}

static bool is_83(const char *n)
{
    const char *dot = strchr(n, '.');
    size_t base = dot ? (size_t)(dot - n) : strlen(n);
    if (base == 0 || base > 8 || (dot && (strlen(dot + 1) > 3 || strchr(dot + 1, '.')))) {
        return false;
    }
    return true;
}

static void find_next(machine_t *m)
{
    while (m->find.next < m->find.n) {
        const char *name = m->find.names[m->find.next++];
        char full[1024];
        struct stat st;
        snprintf(full, sizeof(full), "%s/%s", m->find.host_dir, name);
        if (stat(full, &st) != 0) {
            continue;
        }
        bool dir = S_ISDIR(st.st_mode);
        if (dir && !(m->find.attr & 0x10)) {
            continue;
        }
        uint8_t *dta = lin_ptr(m, m->dta, 43);
        if (!dta) {
            break;
        }
        struct tm tm;
        localtime_r(&st.st_mtime, &tm);
        uint16_t ftime = (uint16_t)(tm.tm_hour << 11 | tm.tm_min << 5 | tm.tm_sec / 2);
        uint16_t fdate = (uint16_t)((tm.tm_year - 80) << 9 | (tm.tm_mon + 1) << 5 | tm.tm_mday);
        uint32_t size = (uint32_t)st.st_size;
        dta[0x15] = dir ? 0x10 : 0x20;
        memcpy(dta + 0x16, &ftime, 2);
        memcpy(dta + 0x18, &fdate, 2);
        memcpy(dta + 0x1a, &size, 4);
        memset(dta + 0x1e, 0, 13);
        for (int i = 0; i < 12 && name[i]; i++) {
            dta[0x1e + i] = (uint8_t)toupper((unsigned char)name[i]);
        }
        CLR_CF();
        return;
    }
    dos_find_end(m);
    fail(m, DOSERR_NOMORE);
}

void dos_find_end(machine_t *m)
{
    free(m->find.names);
    m->find.names = NULL;
    m->find.n = m->find.next = 0;
}

static int name_cmp(const void *a, const void *b)
{
    return strcasecmp(a, b);
}

static void find_first(machine_t *m)
{
    char spec[256], norm[256];
    int drive;
    guest_str(m, SEG_BASE(X86_DS) + EDX, spec, sizeof(spec));
    dos_find_end(m);
    if (!dos_normalize(m, spec, &drive, norm, sizeof(norm))) {
        fail(m, DOSERR_NOPATH);
        return;
    }
    char *slash = strrchr(norm, '\\');
    snprintf(m->find.pattern, sizeof(m->find.pattern), "%s", slash + 1);
    slash[slash == norm ? 1 : 0] = 0;
    char dirspec[260];
    snprintf(dirspec, sizeof(dirspec), "%c:%s", 'A' + drive, norm);
    if (!dos_host_path(m, dirspec, m->find.host_dir, sizeof(m->find.host_dir), true)) {
        fail(m, DOSERR_NOPATH);
        return;
    }
    m->find.attr = CL;
    int cap = 0;
    DIR *d = opendir(m->find.host_dir);
    struct dirent *e;
    while (d && (e = readdir(d))) {
        if (e->d_name[0] == '.' || !is_83(e->d_name) || !wild_match(m->find.pattern, e->d_name)) {
            continue;
        }
        if (m->find.n == cap) {
            void *p = realloc(m->find.names, (cap = cap ? 2 * cap : 16) * sizeof(*m->find.names));
            if (!p) {
                break;
            }
            m->find.names = p;
        }
        snprintf(m->find.names[m->find.n++], sizeof(*m->find.names), "%s", e->d_name);
    }
    if (d) {
        closedir(d);
    }
    if (m->find.n) {
        qsort(m->find.names, m->find.n, sizeof(*m->find.names), name_cmp);
    }
    find_next(m);
    if (m->cpu.eflags & X86_CF) {
        SET_AX(DOSERR_NOFILE);
    }
}

/* ---- INT 21h ---- */

void dos_init(machine_t *m)
{
    snprintf(m->drive_c, sizeof(m->drive_c), "%s", m->cfg.drive_c ? m->cfg.drive_c : ".");
    snprintf(m->drive_d, sizeof(m->drive_d), "%s", m->cfg.drive_d ? m->cfg.drive_d : "");
    m->cur_drive = 2;
    for (int i = 0; i < 4; i++) {
        snprintf(m->cwd[i], sizeof(m->cwd[i]), "\\");
    }
    if (m->cfg.cwd) {
        snprintf(m->cwd[2], sizeof(m->cwd[2]), "%s", m->cfg.cwd);
    }
    for (int i = 0; i < 5; i++) {
        m->files[i] = (dosfile_t){ .used = true, .device = true };
    }
}

/* A key for the DOS console input functions. Keys without an ASCII code (function keys, arrows,
 * Alt combinations) come as two characters, 0 and then the scan code, as programs expect when
 * they read a 0: the scan code waits in con_scan for the next read. */
static bool con_key(machine_t *m, uint8_t *ch, bool remove)
{
    uint16_t key;
    if (m->con_scan) {
        *ch = m->con_scan;
        if (remove) {
            m->con_scan = 0;
        }
        return true;
    }
    if (!bios_key_get(m, &key, remove)) {
        return false;
    }
    *ch = key & 0xff;
    if (remove && !*ch) {
        m->con_scan = key >> 8;
    }
    return true;
}

void dos_int21(machine_t *m)
{
    uint32_t ds = SEG_BASE(X86_DS);
    dosfile_t *f;
    CLR_CF();
    switch (AH) {
    case 0x02:
        fputc(DL, stderr);
        return;
    case 0x01:
    case 0x07:
    case 0x08: {
        /* character input: wait for a key */
        uint8_t ch;
        if (!con_key(m, &ch, true)) {
            mach_wait_key(m);
            return;
        }
        SET_AL(ch);
        return;
    }
    case 0x06: {
        uint8_t ch;
        if (DL != 0xff) {
            fputc(DL, stderr);
        } else if (con_key(m, &ch, true)) {
            SET_AL(ch);
            m->cpu.eflags &= ~X86_ZF;
        } else {
            SET_AL(0);
            m->cpu.eflags |= X86_ZF;
        }
        return;
    }
    case 0x09: {
        for (uint32_t i = 0; i < 4096; i++) {
            uint8_t *p = lin_ptr(m, ds + EDX + i, 1);
            if (!p || *p == '$') {
                break;
            }
            fputc(*p, stderr);
        }
        return;
    }
    case 0x0b: {
        uint8_t ch;
        SET_AL(con_key(m, &ch, false) ? 0xff : 0);
        return;
    }
    case 0x0c: { /* flush the buffer, then run the input function in AL */
        uint16_t key;
        while (bios_key_get(m, &key, true)) {
        }
        m->con_scan = 0;
        uint32_t fn = AL;
        if (fn == 0x01 || fn == 0x06 || fn == 0x07 || fn == 0x08 || fn == 0x0a) {
            SET_AH(fn);
            dos_int21(m);
        }
        return;
    }
    case 0x0e:
        if (DL < 4 && drive_root(m, DL)) {
            m->cur_drive = DL;
        }
        SET_AL(5);
        return;
    case 0x19:
        SET_AL(m->cur_drive);
        return;
    case 0x1a:
        m->dta = ds + EDX;
        return;
    case 0x25:
        m->pmvec[AL] = (farptr_t){ m->cpu.sel[X86_DS], EDX };
        return;
    case 0x2a: { /* date: a fixed day, Friday 10 March 1995 (the board has no calendar clock) */
        ECX = (ECX & 0xffff0000u) | 1995;
        SET_DX(3 << 8 | 10);
        SET_AL(5);
        return;
    }
    case 0x2c: {
        uint64_t cs = machine_time_us(m) / 10000;
        uint32_t s = (uint32_t)(cs / 100);
        SET_CX(((12 + s / 3600) % 24) << 8 | (s / 60) % 60);
        SET_DX((s % 60) << 8 | (uint32_t)(cs % 100));
        return;
    }
    case 0x2f:
        x86_set_seg(&m->cpu, X86_ES, SEL_DATA);
        EBX = m->dta;
        return;
    case 0x30:
        EAX = 0x0005; /* DOS 5.0 */
        EBX = 0;
        ECX = 0;
        return;
    case 0x35:
        x86_set_seg(&m->cpu, X86_ES, m->pmvec[AL].sel);
        EBX = m->pmvec[AL].off;
        return;
    case 0x36: /* free space: plenty */
        SET_AX(64);
        SET_BX(0x8000);
        SET_CX(512);
        SET_DX(0xffff);
        return;
    case 0x37: /* switch character */
        if (AL == 0) {
            SET_AL(0);
            SET_DX('/');
        }
        return;
    case 0x39:
    case 0x3a:
    case 0x41:
    case 0x43: {
        char name[256], host[512];
        guest_str(m, ds + EDX, name, sizeof(name));
        bool exists = dos_host_path(m, name, host, sizeof(host), true);
        if (AH == 0x43) {
            struct stat st;
            if (!exists || stat(host, &st) != 0) {
                fail(m, DOSERR_NOFILE);
            } else if (AL == 0) {
                SET_CX(S_ISDIR(st.st_mode) ? 0x10 : 0x20);
            }
            return;
        }
        forget(m);
        if (AH == 0x39) {
            if (!dos_host_path(m, name, host, sizeof(host), false) || mkdir(host, 0755) != 0) {
                fail(m, DOSERR_ACCESS);
            }
            return;
        }
        if (!exists || (AH == 0x3a ? rmdir(host) : unlink(host)) != 0) {
            fail(m, exists ? DOSERR_ACCESS : DOSERR_NOFILE);
        }
        return;
    }
    case 0x3b: {
        char name[256], norm[256], host[512];
        int drive;
        guest_str(m, ds + EDX, name, sizeof(name));
        if (!dos_normalize(m, name, &drive, norm, sizeof(norm)) || !dos_host_path(m, name, host, sizeof(host), true) ||
            strlen(norm) >= sizeof(m->cwd[drive])) {
            fail(m, DOSERR_NOPATH);
            return;
        }
        memcpy(m->cwd[drive], norm, strlen(norm) + 1);
        return;
    }
    case 0x3c:
        do_open(m, true, 2);
        return;
    case 0x3d:
        do_open(m, false, AL);
        return;
    case 0x3e:
        if (!(f = handle(m, BX))) {
            fail(m, DOSERR_HANDLE);
            return;
        }
        dos_close_file(m, BX);
        if (BX >= 5) {
            *f = (dosfile_t){ 0 };
        }
        return;
    case 0x3f: {
        if (!(f = handle(m, BX))) {
            fail(m, DOSERR_HANDLE);
            return;
        }
        uint8_t *p = lin_ptr(m, ds + EDX, ECX);
        if (!p && ECX) {
            fail(m, DOSERR_ACCESS);
            return;
        }
        if (f->f && f->wrote) {
            fseek(f->f, 0, SEEK_CUR); /* DOS handles switch between writing and reading freely, stdio does not */
            f->wrote = false;
        }
        EAX = f->f ? (uint32_t)fread(p, 1, ECX, f->f) : 0;
        return;
    }
    case 0x40: {
        if (!(f = handle(m, BX))) {
            fail(m, DOSERR_HANDLE);
            return;
        }
        uint8_t *p = lin_ptr(m, ds + EDX, ECX);
        if (!p && ECX) {
            fail(m, DOSERR_ACCESS);
            return;
        }
        if (!f->device) {
            close_kept(m);  /* a file kept open may hold what this changes */
        }
        if (f->device) {
            if (BX == 1 || BX == 2) {
                fwrite(p, 1, ECX, stderr);
            }
            EAX = ECX;
        } else if (!f->f) {
            fail(m, DOSERR_ACCESS);
        } else if (ECX == 0) {
            /* write of zero bytes truncates at the current position */
            EAX = 0;
            fflush(f->f);
            if (ftruncate(fileno(f->f), ftell(f->f)) != 0) {
                fail(m, DOSERR_ACCESS);
            }
        } else {
            if (!f->wrote) {
                fseek(f->f, 0, SEEK_CUR);
                f->wrote = true;
            }
            EAX = (uint32_t)fwrite(p, 1, ECX, f->f);
        }
        return;
    }
    case 0x42: {
        if (!(f = handle(m, BX))) {
            fail(m, DOSERR_HANDLE);
            return;
        }
        int32_t off = (int32_t)((uint32_t)CX << 16 | DX);
        long pos = 0;
        if (f->f) {
            fseek(f->f, off, AL == 0 ? SEEK_SET : AL == 1 ? SEEK_CUR : SEEK_END);
            pos = ftell(f->f);
        }
        SET_AX(pos & 0xffff);
        SET_DX((uint32_t)pos >> 16);
        return;
    }
    case 0x44:
        if (!(f = handle(m, BX)) && AL <= 7) {
            fail(m, DOSERR_HANDLE);
            return;
        }
        switch (AL) {
        case 0x00:
            SET_DX(f->device ? 0x80d3 : m->cur_drive);
            return;
        case 0x01:
            return;
        case 0x06:
            SET_AL(0xff);
            return;
        case 0x07:
            SET_AL(0xff);
            return;
        case 0x08: /* removable? */
            SET_AX(BL == 4 ? 0 : 1);
            return;
        case 0x09: /* remote? */
            SET_DX(BL == 4 ? 0x1000 : 0);
            return;
        default:
            fail(m, DOSERR_FUNC);
            return;
        }
    case 0x45: {
        if (!(f = handle(m, BX))) {
            fail(m, DOSERR_HANDLE);
            return;
        }
        int h = alloc_handle(m);
        if (h < 0) {
            fail(m, DOSERR_TOOMANY);
            return;
        }
        /* both handles use the one host file (its position too), closed with the last of them */
        m->files[h] = *f;
        SET_AX(h);
        return;
    }
    case 0x47:
        snprintf((char *)lin_ptr(m, ds + ESI, 64), 64, "%s", m->cwd[DL ? DL - 1 : m->cur_drive] + 1);
        return;
    case 0x48: {
        uint32_t largest = 0;
        uint32_t lin = dos_alloc(m, BX, &largest);
        if (!lin) {
            SET_BX(largest);
            fail(m, DOSERR_NOMEM);
            return;
        }
        SET_AX(lin >> 4);
        return;
    }
    case 0x49:
        if (!dos_free(m, SEG_BASE(X86_ES))) {
            fail(m, DOSERR_BADBLOCK);
        }
        return;
    case 0x4a:
        return; /* the program block: nothing to resize */
    case 0x4b: {
        /* exec: no child processes; log what the program wanted to run */
        char name[256], tail[130] = "";
        guest_str(m, ds + EDX, name, sizeof(name));
        uint8_t *pb = lin_ptr(m, SEG_BASE(X86_ES) + EBX, 12);
        if (pb) {
            uint32_t off;
            uint16_t sel;
            memcpy(&off, pb + 4, 4);
            memcpy(&sel, pb + 8, 2);
            uint8_t *t = lin_ptr(m, sel_base(m, sel) + off, 128);
            if (t) {
                snprintf(tail, sizeof(tail), "%.*s", t[0], (char *)t + 1);
            }
        }
        mach_log(m, "exec %s%s refused", name, tail);
        fail(m, DOSERR_NOFILE);
        return;
    }
    case 0x4c:
        m->state = MACH_EXITED;
        x86_request_stop(&m->cpu);
        return;
    case 0x4e:
        find_first(m);
        return;
    case 0x4f:
        find_next(m);
        return;
    case 0x56: {
        char a[256], b[256], ha[512], hb[512];
        guest_str(m, ds + EDX, a, sizeof(a));
        guest_str(m, SEG_BASE(X86_ES) + EDI, b, sizeof(b));
        forget(m);
        if (!dos_host_path(m, a, ha, sizeof(ha), true) || !dos_host_path(m, b, hb, sizeof(hb), false) ||
            rename(ha, hb) != 0) {
            fail(m, DOSERR_ACCESS);
        }
        return;
    }
    case 0x57:
        if (AL == 0) {
            SET_CX(0);
            SET_DX((15 << 9) | (3 << 5) | 10);
        }
        return;
    case 0x58:
        SET_AX(0);
        return;
    case 0x62:
        SET_BX(SEL_PSP);
        return;
    case 0x67:
    case 0x68:
        return;
    case 0xff:
        if (AL == 0x00 && DX == 0x78) { /* DOS/4G presence check */
            EAX = 0x4734ffff;
            return;
        }
        break;
    }
    mach_fault(m, "INT 21h AH=%02xh not implemented (eax=%08" PRIx32 ")", AH, EAX);
}

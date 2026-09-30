/*
 * USB HID host (keyboard + mouse) on top of CherryUSB.
 *
 * esp-usb-host does not support full-speed devices behind the DEV-KIT's
 * high-speed hub (no transaction translator support), so CherryUSB is used. CherryUSB enumerates devices and hubs itself and calls
 * usbh_hid_run()/usbh_hid_stop() for each HID interface.
 *
 * Keyboards are switched to the boot protocol (fixed 8-byte reports). Mice are
 * left in report protocol and decoded using their report descriptor, because
 * many receivers ignore SET_PROTOCOL and send report IDs and 12/16-bit axes.
 *
 * Reports become input.h events: key presses and releases as HID usages, mouse
 * motion and buttons. The report task is the queue's only producer.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "input.h"
#include "usb_hid.h"

static const char *TAG = "usb_hid";

#define HID_REPORT_MAX        64

typedef struct {
    uint8_t minor;
    uint8_t len;
    uint8_t data[HID_REPORT_MAX];
} hid_report_t;

/* Location of one value inside an input report */
typedef struct {
    bool valid;
    bool is_signed;
    uint8_t report_id;
    uint8_t size;       /* bits */
    uint16_t bit_offset; /* after the report ID byte */
} hid_field_t;

typedef struct {
    uint8_t protocol;   /* HID_PROTOCOL_KEYBOARD, or anything else = decode as mouse */
    bool uses_report_id;
    hid_field_t buttons; /* first 3 buttons, packed by the descriptor as 1-bit fields */
    hid_field_t x, y, wheel;
} hid_layout_t;

/* DMA target buffers and report layouts, one per HID class instance (indexed by hid_class->minor) */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t s_hid_buf[CONFIG_USBHOST_MAX_HID_CLASS][USB_ALIGN_UP(HID_REPORT_MAX, CONFIG_USB_ALIGN_SIZE)];
static hid_layout_t s_layout[CONFIG_USBHOST_MAX_HID_CLASS];

static QueueHandle_t s_report_queue;
static QueueHandle_t s_start_queue;
static int s_keyboards, s_mice;

static void update_status(void)
{
    ESP_LOGI(TAG, "keyboards %d, mice %d", s_keyboards, s_mice);
}

/* ---------------------------------------------------------------------------
 * Report descriptor parsing: find the mouse buttons, X, Y and wheel fields
 * ------------------------------------------------------------------------- */

static void set_field(hid_field_t *f, uint8_t report_id, uint16_t offset, uint8_t size, int32_t logical_min)
{
    if (!f->valid) {
        *f = (hid_field_t) {
            .valid = true, .is_signed = logical_min < 0,
            .report_id = report_id, .size = size, .bit_offset = offset,
        };
    }
}

static void parse_mouse_layout(const uint8_t *desc, size_t len, hid_layout_t *out)
{
    uint16_t usage_page = 0;
    int32_t logical_min = 0;
    uint8_t report_size = 0, report_id = 0;
    uint32_t report_count = 0;
    uint32_t usages[16];
    int n_usages = 0;
    uint32_t usage_min = 0, usage_max = 0;
    bool in_mouse = false;          /* inside a Generic Desktop / Mouse application collection */
    int depth = 0, mouse_depth = -1;
    uint16_t offsets[256] = {0};    /* running input bit offset per report ID */

    for (size_t i = 0; i < len;) {
        uint8_t prefix = desc[i];
        if (prefix == 0xFE) { /* long item */
            i += 3 + (i + 1 < len ? desc[i + 1] : 0);
            continue;
        }
        uint8_t size = prefix & 3;
        size = size == 3 ? 4 : size;
        uint8_t type = (prefix >> 2) & 3;
        uint8_t tag = prefix >> 4;
        if (i + 1 + size > len) {
            break;
        }
        uint32_t uval = 0;
        for (int b = 0; b < size; b++) {
            uval |= (uint32_t)desc[i + 1 + b] << (8 * b);
        }
        int32_t sval = uval;
        if (size && size < 4 && (uval & (1u << (size * 8 - 1)))) {
            sval = (int32_t)(uval | (~0u << (size * 8))); /* sign-extend */
        }
        i += 1 + size;

        if (type == 1) { /* global */
            switch (tag) {
            case 0: usage_page = uval; break;
            case 1: logical_min = sval; break;
            case 7: report_size = uval; break;
            case 8: report_id = uval; out->uses_report_id = true; break;
            case 9: report_count = uval; break;
            }
        } else if (type == 2) { /* local */
            uint32_t u = size == 4 ? uval : ((uint32_t)usage_page << 16) | uval;
            if (tag == 0 && n_usages < 16) {
                usages[n_usages++] = u;
            } else if (tag == 1) {
                usage_min = u;
            } else if (tag == 2) {
                usage_max = u;
            }
        } else if (type == 0) { /* main */
            if (tag == 0xA) { /* collection */
                depth++;
                /* application collection (0x01) with usage Generic Desktop / Mouse (0x0001_0002) */
                if (uval == 0x01 && n_usages > 0 && usages[0] == 0x00010002) {
                    in_mouse = true;
                    mouse_depth = depth;
                }
            } else if (tag == 0xC) { /* end collection */
                if (depth == mouse_depth) {
                    in_mouse = false;
                    mouse_depth = -1;
                }
                depth--;
            } else if (tag == 0x8) { /* input */
                bool constant = uval & 1;
                for (uint32_t k = 0; k < report_count; k++) {
                    uint16_t off = offsets[report_id] + k * report_size;
                    uint32_t u;
                    if (k < (uint32_t)n_usages) {
                        u = usages[k];
                    } else if (usage_min || usage_max) {
                        u = MIN(usage_min + k, usage_max);
                    } else {
                        u = n_usages ? usages[n_usages - 1] : 0;
                    }
                    if (!in_mouse || constant) {
                        continue;
                    }
                    if (u == 0x00090001 && report_size == 1) {
                        set_field(&out->buttons, report_id, off, MIN(report_count - k, 3), 0);
                    } else if (u == 0x00010030) {
                        set_field(&out->x, report_id, off, report_size, logical_min);
                    } else if (u == 0x00010031) {
                        set_field(&out->y, report_id, off, report_size, logical_min);
                    } else if (u == 0x00010038) {
                        set_field(&out->wheel, report_id, off, report_size, logical_min);
                    }
                }
                offsets[report_id] += report_count * report_size;
            }
            /* local items only apply to the next main item */
            n_usages = 0;
            usage_min = usage_max = 0;
        }
    }
}

static int32_t get_field(const hid_field_t *f, const uint8_t *payload, size_t payload_len)
{
    uint32_t v = 0;
    for (int b = 0; b < f->size; b++) {
        uint32_t bit = f->bit_offset + b;
        if (bit / 8 >= payload_len) {
            return 0;
        }
        v |= ((payload[bit / 8] >> (bit % 8)) & 1u) << b;
    }
    if (f->is_signed && f->size < 32 && (v & (1u << (f->size - 1)))) {
        v |= ~0u << f->size;
    }
    return (int32_t)v;
}

/* ---------------------------------------------------------------------------
 * Report handling (runs in hid_report task)
 * ------------------------------------------------------------------------- */

static void key_event(uint8_t usage, bool down)
{
    input_event_t ev = { .type = INPUT_KEY, .code = usage, .down = down };
    input_push(&ev);
}

/* Boot keyboard report: modifiers, reserved, 6 keycodes. Presses and releases are found by
 * comparing with the previous report; the modifier bits are usages 0xE0..0xE7. */
static void handle_keyboard(const uint8_t *data, size_t len)
{
    static uint8_t prev[8];
    if (len < 8 || data[2] == 0x01) {
        return; /* short, or ErrorRollOver: the report says nothing about which keys are down */
    }
    for (int b = 0; b < 8; b++) {
        if ((data[0] ^ prev[0]) & (1u << b)) {
            key_event(0xe0 + b, data[0] & (1u << b));
        }
    }
    for (int i = 2; i < 8; i++) {
        if (prev[i] >= 4 && !memchr(&data[2], prev[i], 6)) {
            key_event(prev[i], false);
        }
    }
    for (int i = 2; i < 8; i++) {
        if (data[i] >= 4 && !memchr(&prev[2], data[i], 6)) {
            key_event(data[i], true);
        }
    }
    memcpy(prev, data, sizeof(prev));
}

static void handle_mouse(const hid_layout_t *l, const uint8_t *data, size_t len)
{
    uint8_t report_id = 0;
    if (l->uses_report_id) {
        if (len < 1) {
            return;
        }
        report_id = data[0];
        data++;
        len--;
    }
    if (!l->x.valid || l->x.report_id != report_id) {
        return; /* not the mouse report (e.g. consumer keys on the same interface) */
    }
    int dx = get_field(&l->x, data, len);
    int dy = l->y.valid ? get_field(&l->y, data, len) : 0;
    unsigned buttons = l->buttons.valid ? get_field(&l->buttons, data, len) & 3 : 0;
    input_event_t ev = { .type = INPUT_MOUSE, .buttons = (uint8_t)buttons, .dx = (int16_t)dx, .dy = (int16_t)dy };
    input_push(&ev);
}

static void report_task(void *arg)
{
    hid_report_t r;
    while (1) {
        xQueueReceive(s_report_queue, &r, portMAX_DELAY);
        const hid_layout_t *l = &s_layout[r.minor];
        if (l->protocol == HID_PROTOCOL_KEYBOARD) {
            handle_keyboard(r.data, r.len);
        } else {
            handle_mouse(l, r.data, r.len);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Interrupt IN polling
 * ------------------------------------------------------------------------- */

static void submit_intin(struct usbh_hid *hid_class);

/* URB completion, called from the USB ISR */
static void IRAM_ATTR intin_complete_cb(void *arg, int nbytes)
{
    struct usbh_hid *hid_class = arg;
    if (nbytes > 0) {
        hid_report_t r = {.minor = hid_class->minor, .len = MIN(nbytes, HID_REPORT_MAX)};
        memcpy(r.data, s_hid_buf[hid_class->minor], r.len);
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(s_report_queue, &r, &woken);
        portYIELD_FROM_ISR(woken);
    } else if (nbytes != -USB_ERR_NAK && nbytes != 0) {
        return; /* device gone or URB killed: stop polling */
    }
    submit_intin(hid_class);
}

static void submit_intin(struct usbh_hid *hid_class)
{
    usbh_int_urb_fill(&hid_class->intin_urb, hid_class->hport, hid_class->intin,
                      s_hid_buf[hid_class->minor], MIN(hid_class->intin->wMaxPacketSize, HID_REPORT_MAX),
                      0, intin_complete_cb, hid_class);
    usbh_submit_urb(&hid_class->intin_urb);
}

/* ---------------------------------------------------------------------------
 * Interface setup. Runs in one task so control transfers for multiple
 * interfaces of the same device don't overlap.
 * ------------------------------------------------------------------------- */

static bool setup_mouse(struct usbh_hid *hid_class, hid_layout_t *l)
{
    size_t len = hid_class->report_size;
    uint8_t *desc = heap_caps_aligned_calloc(CONFIG_USB_ALIGN_SIZE, 1, USB_ALIGN_UP(len, CONFIG_USB_ALIGN_SIZE),
                                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!desc) {
        return false;
    }
    bool ok = false;
    int ret = usbh_hid_get_report_descriptor(hid_class, desc, len);
    if (ret < 0) {
        ESP_LOGW(TAG, "GET_DESCRIPTOR(report) failed: %d", ret);
    } else {
        parse_mouse_layout(desc, len, l);
        ok = l->x.valid;
        ESP_LOGI(TAG, "Mouse layout: report_id %s, X %d@%d/%d, Y %d@%d/%d, buttons %d@%d, wheel %s",
                 l->uses_report_id ? "yes" : "no",
                 l->x.report_id, l->x.bit_offset, l->x.size,
                 l->y.report_id, l->y.bit_offset, l->y.size,
                 l->buttons.size, l->buttons.bit_offset, l->wheel.valid ? "yes" : "no");
    }
    heap_caps_free(desc);
    return ok;
}

static void hid_start_task(void *arg)
{
    struct usbh_hid *hid_class;
    while (1) {
        xQueueReceive(s_start_queue, &hid_class, portMAX_DELAY);
        struct usbh_hubport *hport = hid_class->hport;
        hid_layout_t *l = &s_layout[hid_class->minor];
        memset(l, 0, sizeof(*l));
        l->protocol = hid_class->protocol;
        ESP_LOGI(TAG, "HID interface %d: VID %04x PID %04x, protocol %d",
                 hid_class->intf, hport->device_desc.idVendor, hport->device_desc.idProduct, hid_class->protocol);

        if (hid_class->protocol == HID_PROTOCOL_KEYBOARD) {
            /* Boot protocol gives a fixed report layout */
            if (usbh_hid_set_protocol(hid_class, 0) < 0) {
                ESP_LOGW(TAG, "SET_PROTOCOL(boot) failed");
            }
            usbh_hid_set_idle(hid_class, 0, 0);
            s_keyboards++;
        } else if (setup_mouse(hid_class, l)) {
            s_mice++;
        } else {
            ESP_LOGI(TAG, "Interface %d is neither a boot keyboard nor a mouse, ignoring", hid_class->intf);
            continue;
        }
        update_status();
        if (hid_class->intin) {
            submit_intin(hid_class);
        }
    }
}

void usbh_hid_run(struct usbh_hid *hid_class)
{
    xQueueSend(s_start_queue, &hid_class, portMAX_DELAY);
}

void usbh_hid_stop(struct usbh_hid *hid_class)
{
    ESP_LOGI(TAG, "HID interface %d disconnected", hid_class->intf);
    const hid_layout_t *l = &s_layout[hid_class->minor];
    if (l->protocol == HID_PROTOCOL_KEYBOARD && s_keyboards > 0) {
        s_keyboards--;
    } else if (l->x.valid && s_mice > 0) {
        s_mice--;
    }
    update_status();
}

void usb_hid_start(void)
{
    s_report_queue = xQueueCreate(32, sizeof(hid_report_t));
    s_start_queue = xQueueCreate(CONFIG_USBHOST_MAX_HID_CLASS, sizeof(struct usbh_hid *));
    xTaskCreate(report_task, "hid_report", 4096, NULL, 5, NULL);
    xTaskCreate(hid_start_task, "hid_start", 4096, NULL, 5, NULL);
    if (usbh_initialize(0, ESP_USBH_BASE, NULL) != 0) {
        ESP_LOGE(TAG, "CherryUSB host init failed");
        return;
    }
    ESP_LOGI(TAG, "CherryUSB host started");
}

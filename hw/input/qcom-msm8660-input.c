/*
 * Experimental MSM8660 QUP/TLMM/Cypress TTSP input path.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register/tag definitions follow the public Linux i2c-qup, msm8660 pinctrl
 * and cyttsp Gen3 interfaces. This is a bounded PIO/single-contact subset,
 * intended for local bring-up with the original Windows Phone drivers.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/system.h"
#include "ui/input.h"
#include "trace.h"

#define TYPE_QCOM_NATIVE_INPUT "qcom-msm8660-input"
OBJECT_DECLARE_SIMPLE_TYPE(QcomNativeInputState, QCOM_NATIVE_INPUT)

#define GPIO_SIZE       0x4000
#define QUP_SIZE        0x800
#define TOUCH_PIN       61
#define BUTTON_COUNT    3
#define TOUCH_ADDRESS   0x24
#define TOUCH_PACKET    32
#define RX_CAPACITY     512
#define REPORT_CAPACITY 64

typedef struct NativeTouchReport {
    uint16_t x;
    uint16_t y;
    bool down;
} NativeTouchReport;

enum {
    QUP_CONFIG = 0x000,
    QUP_STATE = 0x004,
    QUP_IO_MODE = 0x008,
    QUP_SW_RESET = 0x00c,
    QUP_OPERATIONAL = 0x018,
    QUP_ERROR_FLAGS = 0x01c,
    QUP_ERROR_FLAGS_EN = 0x020,
    QUP_OPERATIONAL_MASK = 0x028,
    QUP_HW_VERSION = 0x030,
    QUP_MX_OUTPUT_CNT = 0x100,
    QUP_OUT_FIFO = 0x110,
    QUP_MX_WRITE_CNT = 0x150,
    QUP_MX_INPUT_CNT = 0x200,
    QUP_MX_READ_CNT = 0x208,
    QUP_IN_FIFO = 0x218,
    QUP_I2C_CLK_CTL = 0x400,
    QUP_I2C_STATUS = 0x404,
    QUP_I2C_MASTER_GEN = 0x408,
};

#define QUP_STATE_VALID     (1 << 2)
#define QUP_MASTER_GEN      (1 << 4)
#define QUP_IN_NOT_EMPTY    (1 << 5)
#define QUP_OUT_SERVICE     (1 << 8)
#define QUP_IN_SERVICE      (1 << 9)
#define QUP_OUTPUT_DONE     (1 << 10)
#define QUP_INPUT_DONE      (1 << 11)
#define QUP_OUT_BLOCK_REQ   (1 << 12)
#define QUP_IN_BLOCK_REQ    (1 << 13)
#define QUP_OUT_BLOCK_MODE  (1 << 10)
#define QUP_IN_BLOCK_MODE   (1 << 12)
/* Output: 16-byte blocks/64-byte FIFO; input: 4-byte blocks/32-byte FIFO. */
#define QUP_IO_CAPS         ((1U << 0) | (1U << 2) | (2U << 7))
#define GPIO_REGISTER(pin, reg) (0x1000 + 0x10 * (pin) + (reg))

struct QcomNativeInputState {
    SysBusDevice parent_obj;
    MemoryRegion gpio_mmio;
    MemoryRegion qup_mmio;
    qemu_irq gpio_irq;
    qemu_irq qup_irq;
    QemuInputHandlerState *input;
    QemuInputHandlerState *keys;
    Notifier machine_done;
    bool buttons;
    int active_button;
    uint32_t button_strip_height;
    uint32_t gpio[GPIO_SIZE / 4];
    uint32_t qup[QUP_SIZE / 4];
    uint16_t rx[RX_CAPACITY];
    unsigned rx_head;
    unsigned rx_count;
    unsigned output_seen;
    unsigned rx_block_remaining;
    uint8_t slave;
    uint16_t touch_register;
    uint16_t touch_position;
    uint8_t host_mode;
    uint8_t touch_data[256];
    unsigned address_bytes;
    bool bootloader;
    bool contact;
    bool input_dirty;
    bool report_pending;
    NativeTouchReport reports[REPORT_CAPACITY];
    unsigned report_head;
    unsigned report_count;
    QEMUTimer *report_timer;
    int x;
    int y;
    uint32_t max_x;
    uint32_t max_y;
    uint32_t x_offset;
    uint32_t y_offset;
    uint32_t sensor_y_max;
    bool invert_y;
};

static void native_gpio_irq(QcomNativeInputState *s)
{
    uint32_t enabled = s->gpio[GPIO_REGISTER(TOUCH_PIN, 8) / 4];
    uint32_t pending = s->gpio[GPIO_REGISTER(TOUCH_PIN, 12) / 4];

    qemu_set_irq(s->gpio_irq, (enabled & pending & 1) != 0);
}

static void native_touch_ready(QcomNativeInputState *s, bool ready)
{
    uint32_t *level = &s->gpio[GPIO_REGISTER(TOUCH_PIN, 4) / 4];

    if (ready && (*level & 1)) {
        s->gpio[GPIO_REGISTER(TOUCH_PIN, 12) / 4] |= 1;
    }
    *level = (*level & ~1U) | !ready;
    native_gpio_irq(s);
}

static void native_touch_publish(QcomNativeInputState *s,
                                 NativeTouchReport report)
{
    memset(s->touch_data, 0, TOUCH_PACKET);
    s->touch_data[2] = report.down ? 1 : 0;
    stw_be_p(&s->touch_data[3], report.x);
    stw_be_p(&s->touch_data[5], report.y);
    s->touch_data[7] = report.down ? 128 : 0;
    s->touch_data[8] = 0x10; /* Contact 1, no second contact. */
    s->report_pending = true;
    trace_qcom_native_touch_report(report.x, report.y, report.down);
    native_touch_ready(s, true);
}

static void native_touch_next(void *opaque)
{
    QcomNativeInputState *s = opaque;

    if (!s->report_pending && s->report_count) {
        NativeTouchReport report = s->reports[s->report_head];

        s->report_head = (s->report_head + 1) % REPORT_CAPACITY;
        s->report_count--;
        native_touch_publish(s, report);
    }
}

static void native_touch_ack(QcomNativeInputState *s)
{
    s->report_pending = false;
    native_touch_ready(s, false);
    if (s->report_count) {
        /* Give the GPIO consumer a distinct inactive interval between frames. */
        timer_mod_ns(s->report_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
    }
}

static void native_touch_enqueue(QcomNativeInputState *s,
                                 NativeTouchReport report)
{
    unsigned tail;

    if (!s->report_pending && !s->report_count) {
        native_touch_publish(s, report);
        return;
    }
    if (s->report_count) {
        tail = (s->report_head + s->report_count - 1) % REPORT_CAPACITY;
        if (s->reports[tail].down == report.down) {
            /* Coalesce movement, while keeping press/release transitions. */
            s->reports[tail] = report;
            return;
        }
    }
    if (s->report_count == REPORT_CAPACITY) {
        /* Keep the newest release even if a stalled guest exhausts the queue. */
        s->report_head = (s->report_head + 1) % REPORT_CAPACITY;
        s->report_count--;
    }
    tail = (s->report_head + s->report_count++) % REPORT_CAPACITY;
    s->reports[tail] = report;
}

static void native_qup_irq(QcomNativeInputState *s)
{
    uint32_t flags = s->qup[QUP_OPERATIONAL / 4];
    bool running = (s->qup[QUP_STATE / 4] & 3) == 1;

    if (s->rx_count) {
        /* Acknowledging SERVICE permits deferred FIFO draining in the DPC. */
        flags |= QUP_IN_NOT_EMPTY;
    } else {
        flags &= ~QUP_IN_NOT_EMPTY;
    }
    s->qup[QUP_OPERATIONAL / 4] = flags;
    flags &= ~s->qup[QUP_OPERATIONAL_MASK / 4];
    /* DONE bits are status, not interrupt sources; software acks SERVICE. */
    qemu_set_irq(s->qup_irq, running &&
        (flags & (QUP_OUT_SERVICE | QUP_IN_SERVICE)));
}

static unsigned native_qup_block_entries(QcomNativeInputState *s, bool input)
{
    unsigned shift = input ? 5 : 0;
    unsigned code = (s->qup[QUP_IO_MODE / 4] >> shift) & 3;

    /* A block holds 4, 16 or 32 bytes; each I2C entry is a 16-bit tag. */
    return (code ? 16U * code : 4) / sizeof(uint16_t);
}

static void native_qup_rx_service(QcomNativeInputState *s)
{
    s->qup[QUP_OPERATIONAL / 4] |= QUP_IN_SERVICE;
    if (s->qup[QUP_IO_MODE / 4] & QUP_IN_BLOCK_MODE) {
        s->qup[QUP_OPERATIONAL / 4] |= QUP_IN_BLOCK_REQ;
        s->rx_block_remaining = MIN(s->rx_count,
                                    native_qup_block_entries(s, true));
    }
}

static void native_qup_reset(QcomNativeInputState *s)
{
    memset(s->qup, 0, sizeof(s->qup));
    s->qup[QUP_STATE / 4] = QUP_STATE_VALID | QUP_MASTER_GEN;
    s->qup[QUP_IO_MODE / 4] = QUP_IO_CAPS;
    s->qup[QUP_HW_VERSION / 4] = 0x10010000;
    s->rx_head = s->rx_count = s->output_seen = 0;
    s->rx_block_remaining = 0;
    s->address_bytes = 0;
    native_qup_irq(s);
}

static uint8_t native_touch_read(QcomNativeInputState *s)
{
    unsigned offset = s->touch_position++;

    if (s->bootloader) {
        static const uint8_t boot_info[] = {
            0, 0x11, 0, 1, 0, 1, 0, 3, 0, 0x20, 0, 1, 0, 0, 0, 1,
        };
        return offset < sizeof(boot_info) ? boot_info[offset] : 0;
    }
    if ((s->host_mode & 0x70) == 0x10) {
        static const uint8_t sys_info[TOUCH_PACKET] = {
            [3] = 1, [6] = 1, [15] = 1, [17] = 3,
            [19] = 0x20, [21] = 1, [29] = 0, [30] = 0xff, [31] = 10,
        };
        return offset == 0 ? s->host_mode :
            offset < sizeof(sys_info) ? sys_info[offset] : 0;
    }
    return offset == 0 ? s->host_mode :
        offset < sizeof(s->touch_data) ? s->touch_data[offset] : 0xff;
}

static void native_touch_write(QcomNativeInputState *s, uint8_t value)
{
    unsigned offset = s->touch_position++;

    trace_qcom_native_touch_command(offset, value);
    if (offset == 0) {
        if (value & 1) {
            s->bootloader = true;
        } else {
            s->host_mode = value & 0xf0;
            s->bootloader = false;
        }
        if (value & 1) {
            s->report_head = s->report_count = 0;
            timer_del(s->report_timer);
        }
        native_touch_ack(s);
    } else if (s->bootloader && offset == 1 && value == 0xa5) {
        /* Exit-bootloader command: the remaining passkey is not touch data. */
        s->bootloader = false;
        s->host_mode = 0;
    }
    /* Contact/status registers are controller-owned, never command storage. */
    if (offset >= 0x1b && offset < sizeof(s->touch_data)) {
        s->touch_data[offset] = value;
    }
}

static void native_qup_tag(QcomNativeInputState *s, uint16_t word)
{
    unsigned tag = (word >> 8) & 7;
    uint8_t value = word;

    if (!tag) {
        return;
    }
    trace_qcom_native_qup_tag(word);
    s->output_seen++;
    switch (tag) {
    case 1:                         /* START: address plus R/W bit. */
        s->slave = value >> 1;
        /* This Cypress firmware uses a persistent, 16-bit LE register selector. */
        s->address_bytes = (value & 1) ? 0 : 2;
        if (value & 1) {
            s->touch_position = s->touch_register;
        } else {
            s->touch_register = 0;
        }
        if (s->slave != TOUCH_ADDRESS) {
            s->qup[QUP_I2C_STATUS / 4] |= 1 << 3;
        }
        break;
    case 2:                         /* Data byte, possibly last before STOP. */
    case 3:
        if (s->slave == TOUCH_ADDRESS) {
            if (s->address_bytes == 2) {
                s->touch_register = value;
                s->address_bytes--;
            } else if (s->address_bytes == 1) {
                s->touch_register |= value << 8;
                s->touch_position = s->touch_register;
                s->address_bytes--;
            } else {
                native_touch_write(s, value);
            }
        }
        break;
    case 4: {                       /* Receive count: zero represents 256. */
        unsigned count = value ? value : 256;
        unsigned start = s->touch_register;

        for (unsigned i = 0; i < count; i++) {
            unsigned tail;
            uint16_t byte;

            if (s->rx_count == RX_CAPACITY) {
                s->qup[QUP_ERROR_FLAGS / 4] |= 1;
                break;
            }
            byte = s->slave == TOUCH_ADDRESS ? native_touch_read(s) : 0xff;
            tail = (s->rx_head + s->rx_count++) % RX_CAPACITY;
            s->rx[tail] = byte | ((i + 1 == count ? 6 : 5) << 8);
        }
        trace_qcom_native_touch_read(start, count, s->host_mode);
        if (!start && count >= TOUCH_PACKET && s->report_pending &&
            !s->bootloader && !(s->host_mode & 0x70)) {
            /* The original native driver acknowledges frames by reading them. */
            native_touch_ack(s);
        }
        s->qup[QUP_OPERATIONAL / 4] |= QUP_INPUT_DONE;
        native_qup_rx_service(s);
        break;
    }
    default:
        s->qup[QUP_ERROR_FLAGS / 4] |= 1;
        break;
    }
    if (s->output_seen >= MAX(s->qup[QUP_MX_OUTPUT_CNT / 4],
                              s->qup[QUP_MX_WRITE_CNT / 4])) {
        s->qup[QUP_OPERATIONAL / 4] |= QUP_OUTPUT_DONE | QUP_OUT_SERVICE;
        s->qup[QUP_OPERATIONAL / 4] &= ~QUP_OUT_BLOCK_REQ;
    } else if ((s->qup[QUP_IO_MODE / 4] & QUP_OUT_BLOCK_MODE) &&
               !(s->output_seen % native_qup_block_entries(s, false))) {
        s->qup[QUP_OPERATIONAL / 4] |= QUP_OUT_BLOCK_REQ | QUP_OUT_SERVICE;
    }
    native_qup_irq(s);
}

static uint64_t native_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    QcomNativeInputState *s = opaque;
    uint32_t value = s->gpio[offset / 4];

    trace_qcom_native_input_mmio(false, false, offset, value);
    return value;
}

static void native_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    QcomNativeInputState *s = opaque;

    trace_qcom_native_input_mmio(false, true, offset, value);
    if (offset >= 0x1000 && ((offset - 0x1000) % 16) == 12) {
        s->gpio[offset / 4] &= ~value; /* Interrupt status is W1C. */
    } else {
        s->gpio[offset / 4] = value;
    }
    native_gpio_irq(s);
}

static uint64_t native_qup_read(void *opaque, hwaddr offset, unsigned size)
{
    QcomNativeInputState *s = opaque;
    uint32_t value = 0;

    if (offset == QUP_IN_FIFO) {
        unsigned words = size == 4 ? 2 : 1;

        for (unsigned i = 0; i < words && s->rx_count; i++) {
            value |= s->rx[s->rx_head] << (16 * i);
            s->rx_head = (s->rx_head + 1) % RX_CAPACITY;
            s->rx_count--;
            if (s->rx_block_remaining) {
                s->rx_block_remaining--;
                if (!s->rx_block_remaining) {
                    s->qup[QUP_OPERATIONAL / 4] &= ~QUP_IN_BLOCK_REQ;
                    if (s->rx_count) {
                        native_qup_rx_service(s);
                    }
                }
            }
        }
        native_qup_irq(s);
    } else {
        native_qup_irq(s);
        value = s->qup[offset / 4];
    }
    trace_qcom_native_input_mmio(true, false, offset, value);
    return value;
}

static void native_qup_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    QcomNativeInputState *s = opaque;

    trace_qcom_native_input_mmio(true, true, offset, value);
    switch (offset) {
    case QUP_SW_RESET:
        native_qup_reset(s);
        return;
    case QUP_STATE:
        if (!(value & 3)) {
            s->qup[QUP_OPERATIONAL / 4] = 0;
            s->rx_head = s->rx_count = s->output_seen = 0;
            s->rx_block_remaining = 0;
        }
        s->qup[offset / 4] = (value & 3) | QUP_STATE_VALID | QUP_MASTER_GEN;
        break;
    case QUP_OPERATIONAL:
    case QUP_ERROR_FLAGS:
    case QUP_I2C_STATUS:
        s->qup[offset / 4] &= ~value;
        break;
    case QUP_OUT_FIFO:
        native_qup_tag(s, value);
        if (size == 4) {
            native_qup_tag(s, value >> 16);
        }
        break;
    case QUP_IO_MODE:
        s->qup[offset / 4] = (value & ~0x3ffU) | QUP_IO_CAPS;
        break;
    case QUP_MX_OUTPUT_CNT:
    case QUP_MX_WRITE_CNT:
        s->output_seen = 0;
        s->qup[offset / 4] = value & 0xffff;
        break;
    default:
        s->qup[offset / 4] = value;
        break;
    }
    native_qup_irq(s);
}

static const MemoryRegionOps native_gpio_ops = {
    .read = native_gpio_read,
    .write = native_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps native_qup_ops = {
    .read = native_qup_read,
    .write = native_qup_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
};

static void native_input_event(DeviceState *dev, QemuConsole *src,
                                QemuInputEvent *event)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(dev);

    if (event->type == INPUT_EVENT_KIND_ABS) {
        if (event->abs.axis == INPUT_AXIS_X) {
            s->x = event->abs.value;
        } else if (event->abs.axis == INPUT_AXIS_Y) {
            s->y = event->abs.value;
        }
        s->input_dirty |= s->contact;
    } else if (event->type == INPUT_EVENT_KIND_BTN &&
               event->btn.button == INPUT_BUTTON_LEFT) {
        if (s->active_button >= 0) {
            /* One contact: a pointer click cannot steal a held key. */
            return;
        }
        s->contact = event->btn.down;
        s->input_dirty = true;
    }
}

static void native_touch_from_view(QcomNativeInputState *s, unsigned x,
                                  unsigned y, bool down)
{
    x += s->x_offset;
    y += s->y_offset;
    if (s->invert_y) {
        y = s->sensor_y_max - y;
    }
    native_touch_enqueue(s, (NativeTouchReport) { x, y, down });
}

static void native_input_sync(DeviceState *dev)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(dev);
    unsigned x, y;

    if (!s->input_dirty) {
        return;
    }
    s->input_dirty = false;
    x = qemu_input_scale_axis(s->x, 0, INPUT_EVENT_ABS_MAX, 0, s->max_x);
    y = qemu_input_scale_axis(s->y, 0, INPUT_EVENT_ABS_MAX, 0, s->max_y);
    native_touch_from_view(s, x, y, s->contact);
}

static const QemuInputHandler native_input_handler = {
    .name = "MSM8660 Cypress TrueTouch",
    .mask = INPUT_EVENT_MASK_ABS | INPUT_EVENT_MASK_BTN,
    .event = native_input_event,
    .sync = native_input_sync,
};

static void native_key_event(DeviceState *dev, QemuConsole *src,
                            QemuInputEvent *event)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(dev);
    int key = qemu_input_linux_to_qcode(event->key.key);
    unsigned button;
    unsigned x, y;

    switch (key) {
    case Q_KEY_CODE_ESC:
        button = 0; /* Back */
        break;
    case Q_KEY_CODE_F2:
        button = 1; /* Start */
        break;
    case Q_KEY_CODE_F3:
        button = 2; /* Search */
        break;
    default:
        return;
    }
    if (event->key.down) {
        if (s->contact || s->active_button >= 0) {
            /* Ignore repeats and chords on this single-contact sensor. */
            return;
        }
        s->active_button = button;
    } else {
        if (s->active_button != button) {
            return;
        }
        s->active_button = -1;
    }
    /*
     * This panel's navigation keys are capacitive zones below the visible
     * display, not GPIO side buttons. Send an ordinary Cypress contact there;
     * the original TOUCH/BUTTONS configuration supplies the key mapping.
     * Use three evenly spaced centres within the calibrated button strip.
     */
    x = (2 * button + 1) * (uint64_t)s->max_x / (2 * BUTTON_COUNT);
    y = s->max_y + 1 + s->button_strip_height / 2;
    trace_qcom_native_button(button, event->key.down);
    native_touch_from_view(s, x, y, event->key.down);
}

static const QemuInputHandler native_key_handler = {
    .name = "MSM8660 Cypress navigation buttons",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = native_key_event,
};

static void native_keys_activate(Notifier *notifier, void *data)
{
    QcomNativeInputState *s = container_of(notifier, QcomNativeInputState,
                                         machine_done);

    /* Select the native backend after command-line USB devices. */
    qemu_input_handler_activate(s->keys);
}

static void native_input_reset(DeviceState *dev)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(dev);

    memset(s->gpio, 0, sizeof(s->gpio));
    for (unsigned pin = 0; pin < 173; pin++) {
        s->gpio[GPIO_REGISTER(pin, 4) / 4] = 1;
    }
    memset(s->touch_data, 0, sizeof(s->touch_data));
    s->host_mode = s->touch_register = s->touch_position = s->slave = 0;
    s->bootloader = false;
    s->contact = s->input_dirty = false;
    s->active_button = -1;
    s->report_pending = false;
    s->report_head = s->report_count = 0;
    timer_del(s->report_timer);
    native_qup_reset(s);
    native_gpio_irq(s);
}

static void native_input_realize(DeviceState *dev, Error **errp)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(dev);

    if (!s->max_x || s->max_x > UINT16_MAX ||
        !s->max_y || s->max_y > UINT16_MAX ||
        s->x_offset > UINT16_MAX - s->max_x ||
        s->y_offset > UINT16_MAX - s->max_y ||
        (s->invert_y && (s->sensor_y_max > UINT16_MAX ||
                        s->sensor_y_max < s->y_offset + s->max_y))) {
        error_setg(errp, "Native touch coordinate limits must fit 16 bits");
        return;
    }
    if (s->buttons) {
        uint64_t button_y = s->max_y + 1 +
                            (uint64_t)s->button_strip_height / 2 + s->y_offset;

        if (!s->button_strip_height || button_y > UINT16_MAX ||
            (s->invert_y && button_y > s->sensor_y_max)) {
            error_setg(errp,
                       "Native navigation button strip exceeds sensor bounds");
            return;
        }
    }
    s->report_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, native_touch_next, s);
    s->input = qemu_input_handler_register(dev, &native_input_handler);
    if (s->buttons) {
        s->keys = qemu_input_handler_register(dev, &native_key_handler);
        s->machine_done.notify = native_keys_activate;
        qemu_add_machine_init_done_notifier(&s->machine_done);
    }
}

static void native_input_unrealize(DeviceState *dev)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(dev);

    qemu_input_handler_unregister(s->input);
    if (s->buttons) {
        qemu_remove_machine_init_done_notifier(&s->machine_done);
        qemu_input_handler_unregister(s->keys);
    }
    timer_free(s->report_timer);
}

static void native_input_init(Object *obj)
{
    QcomNativeInputState *s = QCOM_NATIVE_INPUT(obj);
    SysBusDevice *bus = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->gpio_mmio, obj, &native_gpio_ops, s,
                          "msm8660-tlmm", GPIO_SIZE);
    memory_region_init_io(&s->qup_mmio, obj, &native_qup_ops, s,
                          "msm8660-qup3", QUP_SIZE);
    sysbus_init_mmio(bus, &s->gpio_mmio);
    sysbus_init_mmio(bus, &s->qup_mmio);
    sysbus_init_irq(bus, &s->gpio_irq);
    sysbus_init_irq(bus, &s->qup_irq);
}

static const Property native_input_properties[] = {
    DEFINE_PROP_BOOL("buttons", QcomNativeInputState, buttons, false),
    /* Original calibration: a 135-pixel strip below the 800-pixel view. */
    DEFINE_PROP_UINT32("button-strip-height", QcomNativeInputState,
                       button_strip_height, 135),
    DEFINE_PROP_UINT32("max-x", QcomNativeInputState, max_x, 479),
    DEFINE_PROP_UINT32("max-y", QcomNativeInputState, max_y, 799),
    /* MSM8660 reference panel: 540x995 sensor, 480x800 view, inverted Y. */
    DEFINE_PROP_UINT32("x-offset", QcomNativeInputState, x_offset, 30),
    DEFINE_PROP_UINT32("y-offset", QcomNativeInputState, y_offset, 60),
    DEFINE_PROP_UINT32("sensor-y-max", QcomNativeInputState, sensor_y_max, 994),
    DEFINE_PROP_BOOL("invert-y", QcomNativeInputState, invert_y, true),
};

static const VMStateDescription native_input_vmstate = {
    .name = TYPE_QCOM_NATIVE_INPUT,
    .unmigratable = 1,
};

static void native_input_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = native_input_realize;
    dc->unrealize = native_input_unrealize;
    device_class_set_legacy_reset(dc, native_input_reset);
    dc->vmsd = &native_input_vmstate;
    device_class_set_props(dc, native_input_properties);
    dc->desc = "Experimental MSM8660 native Cypress input path";
}

static const TypeInfo native_input_type = {
    .name = TYPE_QCOM_NATIVE_INPUT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(QcomNativeInputState),
    .instance_init = native_input_init,
    .class_init = native_input_class_init,
};

static void native_input_register_types(void)
{
    type_register_static(&native_input_type);
}

type_init(native_input_register_types)

/*
 * Experimental MSM8998 RPM GLINK transport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The RPM message-RAM TOC and intentless GLINK framing are documented by
 * Linux drivers/rpmsg/qcom_glink_rpm.c and qcom_glink_native.c. This models
 * a protocol peer, not execution of the original RPM firmware. Test and CX/MX
 * rail votes are recorded; other resource classes receive an error. Rail votes
 * do not simulate a PMIC, physical voltage settling or CPU/GPU DVFS.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/arm/qcom8998-rpm.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_QCOM8998_RPM "qcom8998-rpm"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom8998Rpm, QCOM8998_RPM)

#define RPM_MSG_RAM_BASE 0x00778000
#define RPM_MSG_RAM_SIZE 0x7000
#define RPM_IPC_BASE 0x17911008
#define RPM_DATA_RAM_BASE 0x00290000
#define RPM_CODE_RAM_BASE 0x00200000
#define RPM_SLEEP_STATS_POINTER 0x14
#define RPM_SLEEP_STATS_OFFSET 0x1d0
#define RPM_SLEEP_STATS_STRIDE 0x30
#define RPM_SLEEP_STATS_COUNT 2
#define RPM_SLEEP_STATS_END \
    (RPM_SLEEP_STATS_OFFSET + RPM_SLEEP_STATS_COUNT * RPM_SLEEP_STATS_STRIDE)
#define RPM_SLEEP_TYPE_VLOW 0x776f6c76 /* vlow */
#define RPM_SLEEP_TYPE_VMIN 0x6e696d76 /* vmin */
#define RPM_GSIV 200
#define GIC_INTERNAL_IRQS 32
#define RPM_TOC_OFFSET (RPM_MSG_RAM_SIZE - 0x100)
#define RPM_TOC_MAGIC 0x67727430 /* grt0 */
#define RPM_TX_FIFO_ID 0x61703272 /* ap2r */
#define RPM_RX_FIFO_ID 0x72326170 /* r2ap */
#define RPM_TX_OFFSET 0x1000
#define RPM_RX_OFFSET 0x3000
#define RPM_FIFO_SIZE 0x1000
#define FIFO_TAIL 0
#define FIFO_HEAD 4
#define FIFO_DATA 8
#define GLINK_HEADER_SIZE 8
#define GLINK_VERSION 1
#define GLINK_INTENTLESS BIT(1)
#define GLINK_NAME_SIZE 64
#define RPM_REQUEST_HEADER_SIZE 28
#define RPM_REQUEST_LIMIT 256
#define RPM_SERVICE_REQUEST 0x00716572 /* req */
#define RPM_MESSAGE_ID 0x2367736d /* msg# */
#define RPM_MESSAGE_ERROR 0x00727265 /* err */
#define RPM_RESOURCE_TEST 0x74736574 /* test */
#define RPM_RESOURCE_CX 0x78637772 /* rwcx */
#define RPM_RESOURCE_MX 0x786d7772 /* rwmx */
#define RPM_KEY_FLOOR_LEVEL 0x006c6676 /* vfl */
#define RPM_KEY_LEVEL 0x6c766c76 /* vlvl */
/* The original PEP's startup test uses two independent four-byte vote keys. */
#define RPM_TEST_KEY_0 0x00ddba11
#define RPM_TEST_KEY_1 0x00fab1e5

enum {
    GLINK_CMD_VERSION,
    GLINK_CMD_VERSION_ACK,
    GLINK_CMD_OPEN,
    GLINK_CMD_CLOSE,
    GLINK_CMD_OPEN_ACK,
    GLINK_CMD_INTENT,
    GLINK_CMD_RX_DONE,
    GLINK_CMD_RX_INTENT_REQ,
    GLINK_CMD_RX_INTENT_REQ_ACK,
    GLINK_CMD_TX_DATA,
    GLINK_CMD_CLOSE_ACK = 11,
    GLINK_CMD_TX_DATA_CONT,
    GLINK_CMD_READ_NOTIF,
    GLINK_CMD_RX_DONE_REUSE,
    GLINK_CMD_SIGNALS,
};

/* PEP opens its subsystem-restart control channel before rpm_requests. */
static const char * const rpm_channel_names[] = {
    "glink_ssr",
    "rpm_requests",
};

enum {
    RPM_VOTE_TEST,
    RPM_VOTE_CX,
    RPM_VOTE_MX,
    RPM_VOTE_COUNT,
};

struct Qcom8998Rpm {
    SysBusDevice parent_obj;
    MemoryRegion ram_region;
    MemoryRegion ipc_region;
    MemoryRegion sleep_region;
    qemu_irq irq;
    uint8_t ram[RPM_MSG_RAM_SIZE];
    bool version_sent;
    uint16_t guest_channels[ARRAY_SIZE(rpm_channel_names)];
    /* Resource, active/sleep set, then the two supported four-byte keys. */
    uint32_t resource_votes[RPM_VOTE_COUNT][2][2];
};

static uint32_t rpm_fifo_available(uint32_t head, uint32_t tail)
{
    return (head + RPM_FIFO_SIZE - tail) % RPM_FIFO_SIZE;
}

static uint32_t rpm_reply_space(Qcom8998Rpm *s)
{
    uint32_t head = ldl_le_p(s->ram + RPM_RX_OFFSET + FIFO_HEAD);
    uint32_t tail = ldl_le_p(s->ram + RPM_RX_OFFSET + FIFO_TAIL);

    if (head >= RPM_FIFO_SIZE || tail >= RPM_FIFO_SIZE || (head | tail) & 7) {
        return 0;
    }
    /* Leave one aligned slot empty to distinguish full from empty. */
    return RPM_FIFO_SIZE - 8 - rpm_fifo_available(head, tail);
}

static void rpm_fifo_copy(Qcom8998Rpm *s, unsigned fifo, unsigned position,
                          uint8_t *buffer, size_t length, bool to_fifo)
{
    for (size_t i = 0; i < length; i++) {
        uint8_t *byte = &s->ram[fifo + FIFO_DATA +
                                 (position + i) % RPM_FIFO_SIZE];

        if (to_fifo) {
            *byte = buffer[i];
        } else {
            buffer[i] = *byte;
        }
    }
}

static bool rpm_send_message(Qcom8998Rpm *s, uint8_t *message, unsigned length)
{
    uint32_t head = ldl_le_p(s->ram + RPM_RX_OFFSET + FIFO_HEAD);

    if (length > rpm_reply_space(s)) {
        return false;
    }
    rpm_fifo_copy(s, RPM_RX_OFFSET, head, message, length, true);
    stl_le_p(s->ram + RPM_RX_OFFSET + FIFO_HEAD,
             (head + length) % RPM_FIFO_SIZE);
    trace_qcom8998_glink_command(false, lduw_le_p(message),
                                 lduw_le_p(message + 2),
                                 ldl_le_p(message + 4));
    qemu_irq_pulse(s->irq);
    return true;
}

static bool rpm_reply(Qcom8998Rpm *s, uint16_t command, uint16_t parameter,
                       uint32_t value, const char *name)
{
    uint8_t message[GLINK_HEADER_SIZE + GLINK_NAME_SIZE] = { 0 };
    unsigned length = GLINK_HEADER_SIZE;

    stw_le_p(message, command);
    stw_le_p(message + 2, parameter);
    stl_le_p(message + 4, value);
    if (name) {
        length += strlen(name) + 1;
        memcpy(message + GLINK_HEADER_SIZE, name, strlen(name) + 1);
    }
    return rpm_send_message(s, message, ROUND_UP(length, 8));
}

static bool rpm_resource_request(Qcom8998Rpm *s, const uint8_t *request,
                                  unsigned length)
{
    uint8_t response[128] = { 0 };
    const char *error = NULL;
    uint32_t votes[2], message_id, set, resource, id, data_length;
    unsigned vote_resource = RPM_VOTE_COUNT;
    unsigned response_length = 36; /* GLINK data + RPM header + msg# TLV. */

    if (length < RPM_REQUEST_HEADER_SIZE ||
        ldl_le_p(request) != RPM_SERVICE_REQUEST ||
        ldl_le_p(request + 4) != length - 8 || rpm_reply_space(s) < 128) {
        return false;
    }
    message_id = ldl_le_p(request + 8);
    set = ldl_le_p(request + 12);
    resource = ldl_le_p(request + 16);
    id = ldl_le_p(request + 20);
    data_length = ldl_le_p(request + 24);
    qemu_log_mask(LOG_UNIMP,
                  "RPM: request %u set %u resource 0x%08x:%u bytes %u\n",
                  message_id, set, resource, id, data_length);
    switch (resource) {
    case RPM_RESOURCE_TEST:
        vote_resource = RPM_VOTE_TEST;
        break;
    case RPM_RESOURCE_CX:
        vote_resource = RPM_VOTE_CX;
        break;
    case RPM_RESOURCE_MX:
        vote_resource = RPM_VOTE_MX;
        break;
    }
    if (data_length != length - RPM_REQUEST_HEADER_SIZE || set > 1) {
        error = "malformed request";
    } else if (vote_resource == RPM_VOTE_COUNT || id != 0) {
        error = "resource does not exist";
    } else {
        uint32_t key0 = resource == RPM_RESOURCE_TEST ?
                        RPM_TEST_KEY_0 : RPM_KEY_FLOOR_LEVEL;
        uint32_t key1 = resource == RPM_RESOURCE_TEST ?
                        RPM_TEST_KEY_1 : RPM_KEY_LEVEL;

        memcpy(votes, s->resource_votes[vote_resource][set], sizeof(votes));
        if (!data_length) {
            memset(votes, 0, sizeof(votes));
        }
        for (unsigned offset = RPM_REQUEST_HEADER_SIZE; offset < length;) {
            uint32_t key, bytes;
            unsigned index;

            if (length - offset < 12) {
                error = "malformed request";
                break;
            }
            key = ldl_le_p(request + offset);
            bytes = ldl_le_p(request + offset + 4);
            if (bytes != 4 ||
                (key != key0 && key != key1)) {
                error = "unsupported key";
                break;
            }
            index = key == key0 ? 0 : 1;
            votes[index] = ldl_le_p(request + offset + 8);
            offset += 12;
        }
    }
    stw_le_p(response, GLINK_CMD_TX_DATA);
    stw_le_p(response + 2, 2); /* Peer-local rpm_requests channel. */
    stl_le_p(response + 16, RPM_SERVICE_REQUEST);
    stl_le_p(response + 24, RPM_MESSAGE_ID);
    stl_le_p(response + 28, 4);
    stl_le_p(response + 32, message_id);
    if (error) {
        unsigned bytes = strlen(error) + 1;

        stl_le_p(response + 36, RPM_MESSAGE_ERROR);
        stl_le_p(response + 40, bytes);
        memcpy(response + 44, error, bytes);
        response_length = 44 + ROUND_UP(bytes, 4);
    }
    stl_le_p(response + 8, response_length - 16);
    stl_le_p(response + 20, response_length - 24);
    if (!rpm_send_message(s, response, ROUND_UP(response_length, 8))) {
        return false;
    }
    if (!error) {
        memcpy(s->resource_votes[vote_resource][set], votes, sizeof(votes));
    }
    return true;
}

static void rpm_process(Qcom8998Rpm *s)
{
    uint32_t head = ldl_le_p(s->ram + RPM_TX_OFFSET + FIFO_HEAD);
    uint32_t tail = ldl_le_p(s->ram + RPM_TX_OFFSET + FIFO_TAIL);
    uint8_t message[GLINK_HEADER_SIZE + GLINK_NAME_SIZE];

    if (head >= RPM_FIFO_SIZE || tail >= RPM_FIFO_SIZE || (head | tail) & 7) {
        qemu_log_mask(LOG_GUEST_ERROR, "RPM: invalid GLINK FIFO indices\n");
        return;
    }
    for (unsigned i = 0; i < RPM_FIFO_SIZE / GLINK_HEADER_SIZE; i++) {
        uint32_t available = rpm_fifo_available(head, tail);
        unsigned length = GLINK_HEADER_SIZE;
        uint16_t command, parameter;
        uint32_t value;

        if (available < GLINK_HEADER_SIZE) {
            break;
        }
        rpm_fifo_copy(s, RPM_TX_OFFSET, tail, message,
                      GLINK_HEADER_SIZE, false);
        command = lduw_le_p(message);
        parameter = lduw_le_p(message + 2);
        value = ldl_le_p(message + 4);
        if (command == GLINK_CMD_OPEN) {
            unsigned name_length = value & 0xffff;

            if (!name_length || name_length > GLINK_NAME_SIZE) {
                break;
            }
            length = ROUND_UP(GLINK_HEADER_SIZE + name_length, 8);
        } else if (command == GLINK_CMD_TX_DATA ||
                   command == GLINK_CMD_TX_DATA_CONT) {
            uint8_t payload_header[8];
            uint32_t chunk;

            if (available < GLINK_HEADER_SIZE + sizeof(payload_header)) {
                break;
            }
            rpm_fifo_copy(s, RPM_TX_OFFSET, tail + GLINK_HEADER_SIZE,
                          payload_header, sizeof(payload_header), false);
            chunk = ldl_le_p(payload_header);
            if (chunk > RPM_FIFO_SIZE - 24) {
                break;
            }
            length = ROUND_UP(16 + chunk, 8);
            trace_qcom8998_rpm_payload(parameter, chunk,
                                       ldl_le_p(payload_header + 4));
        }
        if (length > available) {
            break;
        }
        /* Reserve both replies before consuming VERSION or OPEN. */
        if ((command == GLINK_CMD_VERSION || command == GLINK_CMD_OPEN ||
             command == GLINK_CMD_CLOSE) && rpm_reply_space(s) < 32) {
            return;
        }
        trace_qcom8998_glink_command(true, command, parameter, value);
        switch (command) {
        case GLINK_CMD_VERSION:
            if (parameter != GLINK_VERSION) {
                return;
            }
            if (!rpm_reply(s, GLINK_CMD_VERSION_ACK, GLINK_VERSION,
                           value & GLINK_INTENTLESS, NULL)) {
                return;
            }
            if (!s->version_sent) {
                if (!rpm_reply(s, GLINK_CMD_VERSION, GLINK_VERSION,
                               GLINK_INTENTLESS, NULL)) {
                    return;
                }
                s->version_sent = true;
            }
            break;
        case GLINK_CMD_OPEN: {
            unsigned channel;

            rpm_fifo_copy(s, RPM_TX_OFFSET, tail, message, length, false);
            if (!parameter ||
                !memchr(message + GLINK_HEADER_SIZE, 0, value & 0xffff)) {
                return;
            }
            for (channel = 0; channel < ARRAY_SIZE(rpm_channel_names);
                 channel++) {
                if (!strcmp((char *)message + GLINK_HEADER_SIZE,
                            rpm_channel_names[channel])) {
                    break;
                }
            }
            if (channel == ARRAY_SIZE(rpm_channel_names)) {
                qemu_log_mask(LOG_UNIMP, "RPM: unsupported GLINK channel\n");
                return;
            }
            if (!rpm_reply(s, GLINK_CMD_OPEN_ACK, parameter, 0, NULL) ||
                !rpm_reply(s, GLINK_CMD_OPEN, channel + 1,
                           strlen(rpm_channel_names[channel]) + 1,
                           rpm_channel_names[channel])) {
                return;
            }
            s->guest_channels[channel] = parameter;
            break;
        }
        case GLINK_CMD_CLOSE:
            if (!rpm_reply(s, GLINK_CMD_CLOSE_ACK, parameter, 0, NULL)) {
                return;
            }
            for (unsigned channel = 0;
                 channel < ARRAY_SIZE(rpm_channel_names); channel++) {
                if (s->guest_channels[channel] == parameter) {
                    s->guest_channels[channel] = 0;
                }
            }
            break;
        case GLINK_CMD_VERSION_ACK:
        case GLINK_CMD_OPEN_ACK:
        case GLINK_CMD_READ_NOTIF:
        case GLINK_CMD_SIGNALS:
            break;
        case GLINK_CMD_TX_DATA: {
            uint8_t payload[RPM_REQUEST_LIMIT];
            uint32_t bytes, remaining;

            rpm_fifo_copy(s, RPM_TX_OFFSET, tail + GLINK_HEADER_SIZE,
                          payload, 8, false);
            bytes = ldl_le_p(payload);
            remaining = ldl_le_p(payload + 4);
            if (!s->guest_channels[1] ||
                parameter != s->guest_channels[1] || remaining ||
                bytes > sizeof(payload)) {
                return;
            }
            rpm_fifo_copy(s, RPM_TX_OFFSET, tail + 16, payload, bytes, false);
            if (!rpm_resource_request(s, payload, bytes)) {
                return;
            }
            break;
        }
        default:
            qemu_log_mask(LOG_UNIMP, "RPM: unsupported GLINK command %u\n",
                          command);
            return;
        }
        tail = (tail + length) % RPM_FIFO_SIZE;
        stl_le_p(s->ram + RPM_TX_OFFSET + FIFO_TAIL, tail);
    }
}

static uint64_t rpm_ram_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom8998Rpm *s = opaque;
    uint64_t value = 0;

    memcpy(&value, s->ram + offset, size);
    return le64_to_cpu(value);
}

static void rpm_ram_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    Qcom8998Rpm *s = opaque;
    uint64_t little = cpu_to_le64(value);

    memcpy(s->ram + offset, &little, size);
    /* The producer head publishes a complete, aligned GLINK packet. */
    if (offset == RPM_TX_OFFSET + FIFO_HEAD && size == 4) {
        rpm_process(s);
    }
    /* Retry a back-pressured command when the guest drains our responses. */
    if (offset == RPM_RX_OFFSET + FIFO_TAIL && size == 4) {
        rpm_process(s);
    }
}

static uint64_t rpm_ipc_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static void rpm_ipc_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    rpm_process(opaque);
}

static const MemoryRegionOps rpm_ram_ops = {
    .read = rpm_ram_read,
    .write = rpm_ram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps rpm_ipc_ops = {
    .read = rpm_ipc_read,
    .write = rpm_ipc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * The OEMSK RPM image exports a local address at data RAM + 0x14.
 * AP software adds the RPM code RAM base, then reads two sleep statistics
 * records. Preserve that address calculation and the record identifiers.
 * No RPM deep sleep transitions are simulated, so all counters are zero.
 */
static uint64_t rpm_sleep_read(void *opaque, hwaddr offset, unsigned size)
{
    static const uint32_t words[RPM_SLEEP_STATS_END / 4] = {
        [RPM_SLEEP_STATS_POINTER / 4] =
            RPM_DATA_RAM_BASE - RPM_CODE_RAM_BASE + RPM_SLEEP_STATS_OFFSET,
        [RPM_SLEEP_STATS_OFFSET / 4] = RPM_SLEEP_TYPE_VLOW,
        [(RPM_SLEEP_STATS_OFFSET + RPM_SLEEP_STATS_STRIDE) / 4] =
            RPM_SLEEP_TYPE_VMIN,
    };
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        hwaddr byte = offset + i;
        uint64_t part = (words[byte / 4] >> ((byte % 4) * 8)) & 0xff;

        value |= part << (i * 8);
    }
    return value;
}

static void rpm_sleep_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    /* The statistics are written by the RPM, never by the application CPU. */
}

static const MemoryRegionOps rpm_sleep_ops = {
    .read = rpm_sleep_read,
    .write = rpm_sleep_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

static void rpm_reset(DeviceState *dev)
{
    Qcom8998Rpm *s = QCOM8998_RPM(dev);
    uint8_t *toc = s->ram + RPM_TOC_OFFSET;

    memset(s->ram, 0, sizeof(s->ram));
    stl_le_p(toc, RPM_TOC_MAGIC);
    stl_le_p(toc + 4, 2);
    stl_le_p(toc + 8, RPM_TX_FIFO_ID);
    stl_le_p(toc + 12, RPM_TX_OFFSET);
    stl_le_p(toc + 16, RPM_FIFO_SIZE);
    stl_le_p(toc + 20, RPM_RX_FIFO_ID);
    stl_le_p(toc + 24, RPM_RX_OFFSET);
    stl_le_p(toc + 28, RPM_FIFO_SIZE);
    s->version_sent = false;
    memset(s->guest_channels, 0, sizeof(s->guest_channels));
    memset(s->resource_votes, 0, sizeof(s->resource_votes));
    qemu_set_irq(s->irq, 0);
}

static void rpm_init(Object *obj)
{
    Qcom8998Rpm *s = QCOM8998_RPM(obj);
    SysBusDevice *bus = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ram_region, obj, &rpm_ram_ops, s,
                          "qcom8998-rpm-msg-ram", RPM_MSG_RAM_SIZE);
    memory_region_init_io(&s->ipc_region, obj, &rpm_ipc_ops, s,
                          "qcom8998-rpm-ipc", 4);
    memory_region_init_io(&s->sleep_region, obj, &rpm_sleep_ops, s,
                          "qcom8998-rpm-sleep-stats", RPM_SLEEP_STATS_END);
    sysbus_init_mmio(bus, &s->ram_region);
    sysbus_init_mmio(bus, &s->ipc_region);
    sysbus_init_mmio(bus, &s->sleep_region);
    sysbus_init_irq(bus, &s->irq);
}

static const VMStateDescription rpm_vmstate = {
    .name = TYPE_QCOM8998_RPM,
    .unmigratable = true,
};

static void rpm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, rpm_reset);
    dc->desc = "Experimental MSM8998 RPM GLINK peer";
    dc->vmsd = &rpm_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo rpm_type = {
    .name = TYPE_QCOM8998_RPM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom8998Rpm),
    .instance_init = rpm_init,
    .class_init = rpm_class_init,
};

static void rpm_register_types(void)
{
    type_register_static(&rpm_type);
}
type_init(rpm_register_types)

void qcom8998_rpm_create(DeviceState *gic)
{
    SysBusDevice *bus = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM8998_RPM));

    sysbus_realize_and_unref(bus, &error_fatal);
    sysbus_mmio_map_overlap(bus, 0, RPM_MSG_RAM_BASE, 1);
    /* Native IPC lies within virt's synthetic PCI MMIO aperture. */
    sysbus_mmio_map_overlap(bus, 1, RPM_IPC_BASE, 1);
    sysbus_mmio_map_overlap(bus, 2, RPM_DATA_RAM_BASE, 1);
    sysbus_connect_irq(bus, 0,
                       qdev_get_gpio_in(gic, RPM_GSIV - GIC_INTERNAL_IRQS));
}

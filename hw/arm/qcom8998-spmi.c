/*
 * MSM8998 SPMI PMIC Arbiter discovery and revision-ID transactions.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/qcom8998-spmi.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_QCOM8998_SPMI "qcom8998-spmi"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom8998Spmi, QCOM8998_SPMI)

/* SPMI arbiter v3 subregions within the OEMSK ACPI memory resource. */
#define ARB_VERSION 0xf000
#define ARB_VERSION_V3 0x30000000
#define ARB_APID_MAP 0xf800
#define ARB_APID_COUNT 512
#define ARB_WRITE_CHANNELS 0x00400000
#define ARB_READ_CHANNELS 0x01400000
#define ARB_CHANNEL_STRIDE 0x8000
#define ARB_CHANNEL_BYTES (ARB_APID_COUNT * ARB_CHANNEL_STRIDE)
#define ARB_PPID_PERIPHERAL_SHIFT 8
#define ARB_PPID_SLAVE_SHIFT 16

#define ARB_CMD 0x00
#define ARB_STATUS 0x08
#define ARB_READ_DATA0 0x18
#define ARB_READ_DATA1 0x1c
#define ARB_DONE BIT(0)
#define ARB_FAILURE BIT(1)
#define ARB_CMD_OPCODE_SHIFT 27
#define ARB_CMD_ADDRESS_SHIFT 4
#define ARB_CMD_COUNT_MASK 7
#define ARB_EXT_READ 1

enum {
    PMIC_REVISION_PERIPHERAL = 1,
    PMIC_REVISION_MAJOR = 3,
    PMIC_REVISION_TYPE = 4,
    PMIC_REVISION_SUBTYPE = 5,
    PMIC_REVISION_FAB_ID = 0xf2,
    PMIC_TYPE = 0x51,
    PM8998_SUBTYPE = 0x14,
    PMI8998_SUBTYPE = 0x15,
    PM8005_SUBTYPE = 0x18,
};

/* PMCF pairs USIDs 0/1, 2/3 and 4/5. Revision data lives on the first. */
static const struct {
    uint8_t slave;
    uint8_t subtype;
    uint8_t major;
    uint8_t fab;
} rev_ids[] = {
    { 0, PM8998_SUBTYPE, 2, 0 },
    { 2, PMI8998_SUBTYPE, 2, 0x11 },
    { 4, PM8005_SUBTYPE, 1, 0 },
};

typedef struct SpmiChannel {
    uint32_t status;
    uint64_t data;
} SpmiChannel;

struct Qcom8998Spmi {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    SpmiChannel reader[ARRAY_SIZE(rev_ids)];
    SpmiChannel writer[ARRAY_SIZE(rev_ids)];
};

static SpmiChannel *spmi_channel(Qcom8998Spmi *s, hwaddr *offset,
                                 unsigned *apid, bool *observer)
{
    if (*offset >= ARB_READ_CHANNELS &&
        *offset < ARB_READ_CHANNELS + ARB_CHANNEL_BYTES) {
        *offset -= ARB_READ_CHANNELS;
        *observer = true;
    } else if (*offset >= ARB_WRITE_CHANNELS && *offset < ARB_READ_CHANNELS) {
        *offset -= ARB_WRITE_CHANNELS;
        *observer = false;
    } else {
        return NULL;
    }
    *apid = *offset / ARB_CHANNEL_STRIDE;
    *offset %= ARB_CHANNEL_STRIDE;
    if (*apid >= ARRAY_SIZE(rev_ids)) {
        return NULL;
    }
    return *observer ? &s->reader[*apid] : &s->writer[*apid];
}

static uint8_t spmi_revision_byte(unsigned apid, unsigned byte)
{
    switch (byte) {
    case PMIC_REVISION_MAJOR:
        return rev_ids[apid].major;
    case PMIC_REVISION_TYPE:
        return PMIC_TYPE;
    case PMIC_REVISION_SUBTYPE:
        return rev_ids[apid].subtype;
    case PMIC_REVISION_FAB_ID:
        return rev_ids[apid].fab;
    default:
        return 0;
    }
}

static uint64_t spmi_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom8998Spmi *s = opaque;
    unsigned apid;
    bool observer;
    SpmiChannel *channel;

    if (offset == ARB_VERSION) {
        return ARB_VERSION_V3;
    }
    if (offset >= ARB_APID_MAP && offset < ARB_APID_MAP + ARB_APID_COUNT * 4) {
        apid = (offset - ARB_APID_MAP) / 4;
        if (apid < ARRAY_SIZE(rev_ids)) {
            return (rev_ids[apid].slave << ARB_PPID_SLAVE_SHIFT) |
                   (PMIC_REVISION_PERIPHERAL << ARB_PPID_PERIPHERAL_SHIFT);
        }
        return 0;
    }
    channel = spmi_channel(s, &offset, &apid, &observer);
    if (!channel) {
        return 0;
    }
    switch (offset) {
    case ARB_STATUS:
        return channel->status;
    case ARB_READ_DATA0:
        return (uint32_t)channel->data;
    case ARB_READ_DATA1:
        return channel->data >> 32;
    default:
        return 0;
    }
}

static void spmi_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    Qcom8998Spmi *s = opaque;
    unsigned apid, opcode, address, count;
    bool observer;
    SpmiChannel *channel = spmi_channel(s, &offset, &apid, &observer);

    if (!channel || offset != ARB_CMD) {
        return;
    }
    opcode = value >> ARB_CMD_OPCODE_SHIFT;
    address = (value >> ARB_CMD_ADDRESS_SHIFT) & 0xff;
    count = (value & ARB_CMD_COUNT_MASK) + 1;
    channel->data = 0;
    channel->status = ARB_DONE;
    if (!observer || opcode != ARB_EXT_READ || count > 0x100 - address) {
        channel->status |= ARB_FAILURE;
        qemu_log_mask(LOG_UNIMP,
                      "MSM8998 SPMI: unsupported APID %u command 0x%08x\n",
                      apid, (uint32_t)value);
        return;
    }
    for (unsigned i = 0; i < count; i++) {
        channel->data |= (uint64_t)spmi_revision_byte(apid, address + i) <<
                         (i * 8);
    }
}

static const MemoryRegionOps spmi_ops = {
    .read = spmi_read,
    .write = spmi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void spmi_reset(DeviceState *dev)
{
    Qcom8998Spmi *s = QCOM8998_SPMI(dev);

    for (unsigned i = 0; i < ARRAY_SIZE(rev_ids); i++) {
        s->reader[i] = (SpmiChannel) { .status = ARB_DONE };
        s->writer[i] = (SpmiChannel) { .status = ARB_DONE };
    }
}

static void spmi_init(Object *obj)
{
    Qcom8998Spmi *s = QCOM8998_SPMI(obj);

    memory_region_init_io(&s->mmio, obj, &spmi_ops, s, "qcom8998-spmi",
                          QCOM8998_SPMI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription spmi_vmstate = {
    .name = TYPE_QCOM8998_SPMI,
    .unmigratable = true,
};

static void spmi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, spmi_reset);
    dc->desc = "Experimental MSM8998 SPMI PMIC Arbiter";
    dc->vmsd = &spmi_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo spmi_type = {
    .name = TYPE_QCOM8998_SPMI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom8998Spmi),
    .instance_init = spmi_init,
    .class_init = spmi_class_init,
};

static void spmi_register_types(void)
{
    type_register_static(&spmi_type);
}
type_init(spmi_register_types)

void qcom8998_spmi_create(void)
{
    SysBusDevice *bus = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM8998_SPMI));

    sysbus_realize_and_unref(bus, &error_fatal);
    sysbus_mmio_map(bus, 0, QCOM8998_SPMI_BASE);
}

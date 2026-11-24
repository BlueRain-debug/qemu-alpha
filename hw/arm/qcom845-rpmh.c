/*
 * SDM845 RSC v2.7 software vote transport for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Implements bounded register-backed resource values and TCS completion.
 * No physical rail, frequency, sleep sequencer or remote AOP is simulated.
 * Layout reference: Linux drivers/soc/qcom/rpmh-rsc.c and sdm845.dtsi;
 * Windows QCpep845 also describes 0x179c0000/0x30000 and 0x0af20000/0x10000.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"

#define TYPE_QCOM845_RPMH "qcom845-rpmh"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Rpmh, QCOM845_RPMH)
#define RSC_BANKS 4
#define RSC_SIZE 0x10000
#define RSC_TCS_STRIDE 0x2a0
#define RSC_TCS_COUNT 9
#define RSC_COMMANDS 16
#define RSC_COMMAND_BASE 0x30
#define RSC_COMMAND_STRIDE 0x14
#define RSC_RESOURCES 256
#define TCS_TRIGGER BIT(24)
#define TCS_ENABLE BIT(16)
#define CMD_WRITE BIT(16)
#define CMD_COMPLETE (BIT(8) | BIT(16))

typedef struct Rsc845Bank {
    Qcom845Rpmh *owner;
    unsigned index;
    unsigned tcs_base;
    MemoryRegion mmio;
    qemu_irq irq;
    uint32_t registers[RSC_SIZE / 4];
} Rsc845Bank;

typedef struct Rsc845Resource {
    uint32_t address;
    uint32_t value;
} Rsc845Resource;

struct Qcom845Rpmh {
    SysBusDevice parent_obj;
    Rsc845Bank banks[RSC_BANKS];
    MemoryRegion bcm_version;
    unsigned resource_count;
    Rsc845Resource resources[RSC_RESOURCES];
};

static const hwaddr rsc845_bases[] = {
    0x179c0000, 0x179d0000, 0x179e0000, 0x0af20000,
};
static const unsigned rsc845_intids[] = { 35, 36, 37, 161 };

static void rsc845_irq(Rsc845Bank *bank)
{
    qemu_set_irq(bank->irq, (bank->registers[bank->tcs_base / 4] &
                            bank->registers[(bank->tcs_base + 4) / 4]) != 0);
}

static Rsc845Resource *rsc845_resource(Qcom845Rpmh *s, uint32_t address)
{
    for (unsigned i = 0; i < s->resource_count; i++) {
        if (s->resources[i].address == address) {
            return &s->resources[i];
        }
    }
    if (s->resource_count == RSC_RESOURCES) {
        return NULL;
    }
    s->resources[s->resource_count].address = address;
    return &s->resources[s->resource_count++];
}

static void rsc845_trigger(Rsc845Bank *bank, unsigned tcs)
{
    Qcom845Rpmh *s = bank->owner;
    unsigned base = (bank->tcs_base + tcs * RSC_TCS_STRIDE) / 4;
    uint32_t enabled = bank->registers[base + 0x1c / 4] & 0xffff;
    bool completed = enabled != 0;

    bank->registers[base + 0x18 / 4] = 0;
    for (unsigned cmd = 0; cmd < RSC_COMMANDS; cmd++) {
        unsigned reg = base + (RSC_COMMAND_BASE + cmd * RSC_COMMAND_STRIDE) / 4;
        uint32_t message = bank->registers[reg];
        uint32_t address = bank->registers[reg + 1];
        Rsc845Resource *resource;

        if (!(enabled & BIT(cmd))) {
            continue;
        }
        bank->registers[reg + 3] = 0;
        /* Resource keys are aligned, 24-bit RPMh addresses, not guest RAM. */
        if ((address & 3) || address > 0xffffff ||
            !(resource = rsc845_resource(s, address))) {
            completed = false;
            qemu_log_mask(LOG_GUEST_ERROR,
                          "qcom845-rpmh: unsupported resource 0x%x\n", address);
            continue;
        }
        if (message & CMD_WRITE) {
            resource->value = bank->registers[reg + 2];
        }
        bank->registers[reg + 4] = resource->value;
        bank->registers[reg + 3] = CMD_COMPLETE;
    }
    bank->registers[base + 0x18 / 4] = 1;
    if (completed) {
        bank->registers[(bank->tcs_base + 4) / 4] |= BIT(tcs);
    }
    trace_qcom845_rsc_complete(bank->index, tcs, enabled, completed);
    rsc845_irq(bank);
}

static uint64_t rsc845_read(void *opaque, hwaddr offset, unsigned size)
{
    Rsc845Bank *bank = opaque;
    uint32_t value;

    switch (offset) {
    case 0:
        value = 0x00020700;
        break;
    case 4:
        value = 0; /* No hardware sleep solver. */
        break;
    case 0xc:
        /* NUM_TCS is a separate six-bit field for each local DRV ID. */
        value = (RSC_COMMANDS << 27) | RSC_TCS_COUNT |
                (RSC_TCS_COUNT << 6) | (RSC_TCS_COUNT << 12);
        break;
    default:
        value = bank->registers[offset / 4];
        break;
    }
    trace_qcom845_rsc_access(bank->index, false, offset, value);
    return value;
}

static void rsc845_write(void *opaque, hwaddr offset, uint64_t value,
                         unsigned size)
{
    Rsc845Bank *bank = opaque;
    uint32_t old = bank->registers[offset / 4];

    trace_qcom845_rsc_access(bank->index, true, offset, value);
    if (offset < 0x10) {
        return;
    }
    if (offset == bank->tcs_base + 8) {
        bank->registers[(bank->tcs_base + 4) / 4] &= ~value;
        rsc845_irq(bank);
        return;
    }
    if (offset == bank->tcs_base + 4) {
        return;
    }
    bank->registers[offset / 4] = value;
    if (offset == bank->tcs_base) {
        bank->registers[offset / 4] &= BIT(RSC_TCS_COUNT) - 1;
        rsc845_irq(bank);
    } else if (offset >= bank->tcs_base &&
               offset < bank->tcs_base + RSC_TCS_COUNT * RSC_TCS_STRIDE &&
               (offset - bank->tcs_base) % RSC_TCS_STRIDE == 0x14 &&
               (value & (TCS_ENABLE | TCS_TRIGGER)) ==
                        (TCS_ENABLE | TCS_TRIGGER) && !(old & TCS_TRIGGER)) {
        rsc845_trigger(bank, (offset - bank->tcs_base) / RSC_TCS_STRIDE);
    }
}

static const MemoryRegionOps rsc845_ops = {
    .read = rsc845_read,
    .write = rsc845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t bcm845_version_read(void *opaque, hwaddr offset, unsigned size)
{
    /*
     * Virtual BCM 1.5 compatibility revision. QCpep845's ICB initialization
     * reads this word and uses 1.5 as its zero-bandwidth-vote erratum cutoff.
     * This is not physical readback. BCM status/counters are not implemented;
     * only the version word is mapped, not an all-zero placeholder window.
     */
    return 0x00010500;
}

static void bcm845_version_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    /* Read-only revision. */
}

static const MemoryRegionOps bcm845_version_ops = {
    .read = bcm845_version_read,
    .write = bcm845_version_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rsc845_reset(DeviceState *dev)
{
    Qcom845Rpmh *s = QCOM845_RPMH(dev);

    s->resource_count = 0;
    memset(s->resources, 0, sizeof(s->resources));
    for (unsigned i = 0; i < RSC_BANKS; i++) {
        Rsc845Bank *bank = &s->banks[i];

        memset(bank->registers, 0, sizeof(bank->registers));
        for (unsigned tcs = 0; tcs < RSC_TCS_COUNT; tcs++) {
            bank->registers[(bank->tcs_base + tcs * RSC_TCS_STRIDE + 0x18) / 4] = 1;
        }
        qemu_set_irq(bank->irq, 0);
    }
}

static void rsc845_init(Object *obj)
{
    Qcom845Rpmh *s = QCOM845_RPMH(obj);

    memory_region_init_io(&s->bcm_version, obj, &bcm845_version_ops, s,
                          "qcom845-bcm-version", 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bcm_version);
    for (unsigned i = 0; i < RSC_BANKS; i++) {
        Rsc845Bank *bank = &s->banks[i];

        bank->owner = s;
        bank->index = i;
        /* Native QCpep845: APSS TCS at +0xd00, display TCS at +0x1c00. */
        bank->tcs_base = i == 3 ? 0x1c00 : 0xd00;
        memory_region_init_io(&bank->mmio, obj, &rsc845_ops, bank,
                              TYPE_QCOM845_RPMH, RSC_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &bank->mmio);
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &bank->irq);
    }
}

static const VMStateDescription rsc845_vmstate = {
    .name = TYPE_QCOM845_RPMH,
    .unmigratable = true,
};

static void rsc845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, rsc845_reset);
    dc->desc = "Experimental SDM845 RPMh RSC vote transport";
    dc->vmsd = &rsc845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo rsc845_type = {
    .name = TYPE_QCOM845_RPMH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Rpmh),
    .instance_init = rsc845_init,
    .class_init = rsc845_class_init,
};

static void rsc845_register_types(void)
{
    type_register_static(&rsc845_type);
}
type_init(rsc845_register_types)

void qcom845_rpmh_create(DeviceState *gic)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_RPMH));

    sysbus_realize_and_unref(dev, &error_fatal);
    for (unsigned i = 0; i < RSC_BANKS; i++) {
        sysbus_mmio_map(dev, i + 1, rsc845_bases[i]);
        sysbus_connect_irq(dev, i, qdev_get_gpio_in(gic, rsc845_intids[i] - 32));
    }
    sysbus_mmio_map(dev, 0, 0x0bbf1708);
}

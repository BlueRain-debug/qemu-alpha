/*
 * SDM845 display clock-control frontend for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register facts: Linux drivers/clk/qcom/dispcc-sdm845.c. Branch status,
 * RCG updates, Fabia lock and software GDSC collapse follow stored control
 * state. No DPU/DSI output, physical clock, rail timing or sleep solver is
 * provided by this frontend.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define TYPE_QCOM845_DISPCC "qcom845-dispcc"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Dispcc, QCOM845_DISPCC)
#define DISPCC_SIZE 0x10000

struct Qcom845Dispcc {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint32_t regs[DISPCC_SIZE / 4];
};

static const unsigned dispcc845_branches[] = {
    0x4004, 0x4008, 0x4010, 0x4014, 0x2028, 0x202c, 0x2030, 0x2034,
    0x2054, 0x2048, 0x2040, 0x2044, 0x2050, 0x204c, 0x2038, 0x203c,
    0x200c, 0x201c, 0x2004, 0x2008, 0x2014, 0x5004, 0x5008, 0x2024,
    0x600c, 0x7004, 0x7008, 0x700c, 0x7010, 0x7014, 0x7018, 0x701c,
    0x7020,
};
static const unsigned dispcc845_roots[] = {
    0x20d0, 0x20ec, 0x219c, 0x2154, 0x2138, 0x2184, 0x216c, 0x2108,
    0x2120, 0x2088, 0x2058, 0x2070, 0x20a0, 0x20b8,
};

static bool dispcc845_contains(const unsigned *regs, unsigned count,
                               hwaddr offset)
{
    for (unsigned i = 0; i < count; i++) {
        if (regs[i] == offset) {
            return true;
        }
    }
    return false;
}

static uint64_t dispcc845_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom845Dispcc *s = opaque;
    uint32_t value = s->regs[offset / 4];

    if (dispcc845_contains(dispcc845_branches,
                           ARRAY_SIZE(dispcc845_branches), offset)) {
        return (value & ~BIT(31)) | ((value & 1) ? 0 : BIT(31));
    }
    if (offset == 0x3000) {
        return (value & ~BIT(31)) | ((value & 1) ? 0 : BIT(31));
    }
    return value;
}

static void dispcc845_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    Qcom845Dispcc *s = opaque;

    if (dispcc845_contains(dispcc845_roots,
                           ARRAY_SIZE(dispcc845_roots), offset)) {
        value &= ~1U;
    }
    if (offset == 0) {
        value &= ~(BIT(31) | BIT(22));
        if ((value & BIT(2)) && s->regs[0x2c / 4] == 1) {
            value |= BIT(31);
        }
    } else if (offset == 0x2c) {
        s->regs[0] &= ~BIT(31);
        if (value == 1 && (s->regs[0] & BIT(2))) {
            s->regs[0] |= BIT(31);
        }
    }
    s->regs[offset / 4] = value;
}

static const MemoryRegionOps dispcc845_ops = {
    .read = dispcc845_read,
    .write = dispcc845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void dispcc845_reset(DeviceState *dev)
{
    Qcom845Dispcc *s = QCOM845_DISPCC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x3000 / 4] = 1;
}

static void dispcc845_init(Object *obj)
{
    Qcom845Dispcc *s = QCOM845_DISPCC(obj);

    memory_region_init_io(&s->mmio, obj, &dispcc845_ops, s,
                          TYPE_QCOM845_DISPCC, DISPCC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription dispcc845_vmstate = {
    .name = TYPE_QCOM845_DISPCC,
    .unmigratable = true,
};

static void dispcc845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, dispcc845_reset);
    dc->desc = "Experimental SDM845 display clock-control registers";
    dc->vmsd = &dispcc845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo dispcc845_type = {
    .name = TYPE_QCOM845_DISPCC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Dispcc),
    .instance_init = dispcc845_init,
    .class_init = dispcc845_class_init,
};

static void dispcc845_register_types(void)
{
    type_register_static(&dispcc845_type);
}
type_init(dispcc845_register_types)

void qcom845_dispcc_create(void)
{
    sysbus_create_simple(TYPE_QCOM845_DISPCC, 0x0af00000, NULL);
}

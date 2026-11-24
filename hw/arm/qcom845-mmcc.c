/*
 * SDM845 VIDEO/CAM/GPU clock-control frontends for OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register indexes follow the public Linux SDM845 clock drivers. These are
 * bounded software control registers, not media engines. No video codec,
 * camera, Adreno/GMU, physical timing or hardware sleep solver is simulated.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"

#define TYPE_QCOM845_MMCC "qcom845-mmcc"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Mmcc, QCOM845_MMCC)
#define MMCC_BANKS 3
#define MMCC_SIZE 0x10000

typedef struct Mmcc845Branch {
    unsigned status;
    unsigned enable;
    uint32_t mask;
} Mmcc845Branch;

#include "qcom845-mmcc-regs.inc"

typedef struct Mmcc845Config {
    hwaddr base;
    unsigned size;
    const Mmcc845Branch *branches;
    unsigned branch_count;
    const unsigned *roots;
    unsigned root_count;
    const unsigned *gdscs;
    unsigned gdsc_count;
    const unsigned *plls;
    unsigned pll_count;
} Mmcc845Config;

#define MMCC_CONFIG(prefix, address, region_size) { \
    address, region_size, prefix##_branches, ARRAY_SIZE(prefix##_branches), \
    prefix##_roots, ARRAY_SIZE(prefix##_roots), \
    prefix##_gdscs, ARRAY_SIZE(prefix##_gdscs), \
    prefix##_plls, ARRAY_SIZE(prefix##_plls), \
}
static const Mmcc845Config mmcc845_configs[MMCC_BANKS] = {
    MMCC_CONFIG(videocc, 0x0ab00000, 0x10000),
    MMCC_CONFIG(camcc, 0x0ad00000, 0x10000),
    MMCC_CONFIG(gpucc, 0x05090000, 0x9000),
};
#undef MMCC_CONFIG

typedef struct Mmcc845Bank {
    MemoryRegion mmio;
    unsigned index;
    uint32_t regs[MMCC_SIZE / 4];
} Mmcc845Bank;

struct Qcom845Mmcc {
    SysBusDevice parent_obj;
    Mmcc845Bank banks[MMCC_BANKS];
};

static bool mmcc845_contains(const unsigned *regs, unsigned count,
                             hwaddr offset)
{
    for (unsigned i = 0; i < count; i++) {
        if (regs[i] == offset) {
            return true;
        }
    }
    return false;
}

static uint64_t mmcc845_read(void *opaque, hwaddr offset, unsigned size)
{
    Mmcc845Bank *bank = opaque;
    const Mmcc845Config *config = &mmcc845_configs[bank->index];
    uint32_t value = bank->regs[offset / 4];

    for (unsigned i = 0; i < config->branch_count; i++) {
        const Mmcc845Branch *branch = &config->branches[i];

        if (offset == branch->status) {
            return (value & ~BIT(31)) |
                   ((bank->regs[branch->enable / 4] & branch->mask) ?
                    0 : BIT(31));
        }
    }
    if (mmcc845_contains(config->gdscs, config->gdsc_count, offset)) {
        return (value & ~BIT(31)) | ((value & 1) ? 0 : BIT(31));
    }
    if (mmcc845_contains(config->plls, config->pll_count, offset)) {
        value &= ~(BIT(31) | BIT(30));
        if ((value & 6) == 6 && bank->regs[(offset + 0x2c) / 4] == 1) {
            value |= BIT(31);
        }
    }
    return value;
}

static void mmcc845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Mmcc845Bank *bank = opaque;
    const Mmcc845Config *config = &mmcc845_configs[bank->index];

    if (mmcc845_contains(config->roots, config->root_count, offset)) {
        value &= ~1U; /* Immediate RCG UPDATE completion. */
    }
    if (mmcc845_contains(config->plls, config->pll_count, offset)) {
        value &= ~(BIT(31) | BIT(30));
    }
    bank->regs[offset / 4] = value;
}

static const MemoryRegionOps mmcc845_ops = {
    .read = mmcc845_read,
    .write = mmcc845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void mmcc845_reset(DeviceState *dev)
{
    Qcom845Mmcc *s = QCOM845_MMCC(dev);

    for (unsigned i = 0; i < MMCC_BANKS; i++) {
        const Mmcc845Config *config = &mmcc845_configs[i];

        memset(s->banks[i].regs, 0, sizeof(s->banks[i].regs));
        for (unsigned d = 0; d < config->gdsc_count; d++) {
            s->banks[i].regs[config->gdscs[d] / 4] = 1;
        }
    }
}

static void mmcc845_init(Object *obj)
{
    Qcom845Mmcc *s = QCOM845_MMCC(obj);

    for (unsigned i = 0; i < MMCC_BANKS; i++) {
        s->banks[i].index = i;
        memory_region_init_io(&s->banks[i].mmio, obj, &mmcc845_ops,
                              &s->banks[i], TYPE_QCOM845_MMCC,
                              mmcc845_configs[i].size);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->banks[i].mmio);
    }
}

static const VMStateDescription mmcc845_vmstate = {
    .name = TYPE_QCOM845_MMCC,
    .unmigratable = true,
};

static void mmcc845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mmcc845_reset);
    dc->desc = "Experimental SDM845 VIDEO/CAM/GPU clock-control registers";
    dc->vmsd = &mmcc845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo mmcc845_type = {
    .name = TYPE_QCOM845_MMCC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Mmcc),
    .instance_init = mmcc845_init,
    .class_init = mmcc845_class_init,
};

static void mmcc845_register_types(void)
{
    type_register_static(&mmcc845_type);
}
type_init(mmcc845_register_types)

void qcom845_mmcc_create(void)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_MMCC));

    sysbus_realize_and_unref(dev, &error_fatal);
    for (unsigned i = 0; i < MMCC_BANKS; i++) {
        sysbus_mmio_map(dev, i, mmcc845_configs[i].base);
    }
}

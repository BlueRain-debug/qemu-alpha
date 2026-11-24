/*
 * SDM845 GCC clock register frontend for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Software branch votes drive CLOCK_OFF, RCG updates complete immediately,
 * GDSC status follows collapse, and Fabia PLL control exposes bounded lock
 * state. Peripheral/CPU clocks do not change QEMU execution speed. DFS is
 * disabled; no frequency tables or physical PLL/calibration are simulated.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_QCOM845_GCC "qcom845-gcc"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Gcc, QCOM845_GCC)
#define GCC_SIZE 0x1f0000
static const unsigned gcc845_pll_bases[] = { 0, 0x13000, 0x76000 };
static const unsigned gcc845_pll_vote_bits[] = { 0, 6, 4 };

typedef struct Gcc845Branch {
    unsigned status;
    unsigned enable;
    uint32_t mask;
} Gcc845Branch;

#include "qcom845-gcc-regs.inc"

struct Qcom845Gcc {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint32_t regs[GCC_SIZE / 4];
};

static bool gcc845_contains(const unsigned *registers, unsigned count,
                            hwaddr offset)
{
    for (unsigned i = 0; i < count; i++) {
        if (registers[i] == offset) {
            return true;
        }
    }
    return false;
}

static uint64_t gcc845_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom845Gcc *s = opaque;
    uint32_t value = s->regs[offset / 4];

    for (unsigned i = 0; i < ARRAY_SIZE(gcc845_branches); i++) {
        const Gcc845Branch *branch = &gcc845_branches[i];

        if (offset == branch->status) {
            value &= ~BIT(31);
            if (!(s->regs[branch->enable / 4] & branch->mask)) {
                value |= BIT(31);
            }
            return value;
        }
    }
    if (gcc845_contains(gcc845_gdscs, ARRAY_SIZE(gcc845_gdscs), offset)) {
        value = (value & ~BIT(31)) | ((value & 1) ? 0 : BIT(31));
    }
    for (unsigned i = 0; i < ARRAY_SIZE(gcc845_pll_bases); i++) {
        unsigned base = gcc845_pll_bases[i];

        if (offset == base) {
            bool running = (value & BIT(2)) &&
                           s->regs[(base + 0x2c) / 4] == 1;

            value &= ~(BIT(31) | BIT(30));
            if (running) {
                value |= BIT(31);
                if ((value & BIT(20)) &&
                    (s->regs[0x52000 / 4] & BIT(gcc845_pll_vote_bits[i]))) {
                    value |= BIT(30); /* FSM vote ACTIVE_FLAG. */
                }
            }
            break;
        }
    }
    return value;
}

static void gcc845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Qcom845Gcc *s = opaque;

    if (gcc845_contains(gcc845_dfs, ARRAY_SIZE(gcc845_dfs), offset)) {
        if (value & 1) {
            qemu_log_mask(LOG_UNIMP, "qcom845-gcc: DFS is unsupported\n");
        }
        value &= ~1U;
    }
    if (gcc845_contains(gcc845_roots, ARRAY_SIZE(gcc845_roots), offset)) {
        value &= ~1U; /* UPDATE strobe completes with stored CFG/M/N/D. */
    }
    for (unsigned i = 0; i < ARRAY_SIZE(gcc845_pll_bases); i++) {
        if (offset == gcc845_pll_bases[i]) {
            value &= ~(BIT(31) | BIT(30) | BIT(22));
        }
    }
    s->regs[offset / 4] = value;
}

static const MemoryRegionOps gcc845_ops = {
    .read = gcc845_read,
    .write = gcc845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gcc845_reset(DeviceState *dev)
{
    Qcom845Gcc *s = QCOM845_GCC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* Virtual firmware's GPLL0 handoff: 600 MHz from 19.2 MHz * 31.25. */
    s->regs[0] = BIT(20) | 7; /* Firmware enabled GPLL0 and its vote FSM. */
    s->regs[4 / 4] = 31;
    s->regs[0x0c / 4] = 0x101; /* Main output and even post-divider. */
    s->regs[0x2c / 4] = 1;
    s->regs[0x38 / 4] = 0x4000;
    for (unsigned i = 0; i < ARRAY_SIZE(gcc845_gdscs); i++) {
        s->regs[gcc845_gdscs[i] / 4] = 1;
    }
}

static void gcc845_init(Object *obj)
{
    Qcom845Gcc *s = QCOM845_GCC(obj);

    memory_region_init_io(&s->mmio, obj, &gcc845_ops, s, TYPE_QCOM845_GCC,
                          GCC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription gcc845_vmstate = {
    .name = TYPE_QCOM845_GCC,
    .unmigratable = true,
};

static void gcc845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, gcc845_reset);
    dc->desc = "Experimental SDM845 GCC clock register frontend (DFS off)";
    dc->vmsd = &gcc845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo gcc845_type = {
    .name = TYPE_QCOM845_GCC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Gcc),
    .instance_init = gcc845_init,
    .class_init = gcc845_class_init,
};

static void gcc845_register_types(void)
{
    type_register_static(&gcc845_type);
}
type_init(gcc845_register_types)

void qcom845_gcc_create(void)
{
    sysbus_create_simple(TYPE_QCOM845_GCC, 0x00100000, NULL);
}

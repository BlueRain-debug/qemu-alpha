/*
 * SDM845 TLMM pin discovery/configuration for local OEMSK850 experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Pin/tile/register facts: Linux drivers/pinctrl/qcom/pinctrl-sdm845.c.
 * The native QCpep845 HAL probes GPIO_ID_STATUS +0x10 in each tile.
 * No external pin devices, input transitions, GPIO IRQs or wake events are
 * connected. Unconnected inputs read low; an enabled output reads its latch.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"

#define TYPE_QCOM845_TLMM "qcom845-tlmm"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Tlmm, QCOM845_TLMM)
#define TLMM_PINS 150
#define TLMM_STRIDE 0x1000
enum { EAST, NORTH, SOUTH, TLMM_TILES };

/* Public SDM845 GPIO tile assignment, indexed by GPIO number. */
static const uint8_t tlmm845_tile[TLMM_PINS] = {
    EAST, EAST, EAST, EAST, NORTH, NORTH, NORTH, NORTH, EAST, EAST,
    EAST, EAST, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH,
    SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, EAST, EAST, EAST,
    EAST, NORTH, NORTH, NORTH, NORTH, SOUTH, SOUTH, SOUTH, NORTH, EAST,
    SOUTH, EAST, EAST, EAST, EAST, EAST, EAST, EAST, EAST, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, EAST,
    EAST, EAST, EAST, EAST, EAST, EAST, EAST, EAST, EAST, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, EAST, EAST, EAST, EAST, SOUTH,
    SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, SOUTH, NORTH, NORTH, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH,
    NORTH, NORTH, EAST, EAST, EAST, EAST, EAST, NORTH, NORTH, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH,
    NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH, NORTH,
};

typedef struct Tlmm845Tile {
    Qcom845Tlmm *owner;
    unsigned index;
    MemoryRegion mmio;
} Tlmm845Tile;

typedef struct Tlmm845Pin {
    uint32_t control;
    uint32_t output;
    uint32_t interrupt;
} Tlmm845Pin;

struct Qcom845Tlmm {
    SysBusDevice parent_obj;
    Tlmm845Tile tiles[TLMM_TILES];
    Tlmm845Pin pins[TLMM_PINS];
};

static uint64_t tlmm845_read(void *opaque, hwaddr offset, unsigned size)
{
    Tlmm845Tile *tile = opaque;
    unsigned pin = offset / TLMM_STRIDE;
    unsigned reg = offset % TLMM_STRIDE;
    Tlmm845Pin *p = &tile->owner->pins[pin];

    if (tlmm845_tile[pin] != tile->index) {
        return 0;
    }
    switch (reg) {
    case 0:
        return p->control;
    case 4:
        return (p->output << 1) | ((p->control & BIT(9)) ? p->output : 0);
    case 8:
        return p->interrupt;
    case 0xc:
        return 0; /* No external inputs/interrupt events. */
    case 0x10:
        return 1; /* Pin is implemented in this tile. */
    default:
        qemu_log_mask(LOG_UNIMP, "qcom845-tlmm: unknown GPIO %u offset 0x%x\n",
                      pin, reg);
        return 0;
    }
}

static void tlmm845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Tlmm845Tile *tile = opaque;
    unsigned pin = offset / TLMM_STRIDE;
    unsigned reg = offset % TLMM_STRIDE;
    Tlmm845Pin *p = &tile->owner->pins[pin];

    if (tlmm845_tile[pin] != tile->index) {
        return;
    }
    switch (reg) {
    case 0:
        p->control = value & 0x3ff;
        break;
    case 4:
        p->output = (value >> 1) & 1;
        break;
    case 8:
        p->interrupt = value & 0xff;
        break;
    case 0xc:
    case 0x10:
        break; /* Status clear with no pending events; read-only ID. */
    default:
        qemu_log_mask(LOG_UNIMP, "qcom845-tlmm: unknown GPIO write %u +0x%x\n",
                      pin, reg);
    }
}

static const MemoryRegionOps tlmm845_ops = {
    .read = tlmm845_read,
    .write = tlmm845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void tlmm845_reset(DeviceState *dev)
{
    Qcom845Tlmm *s = QCOM845_TLMM(dev);

    memset(s->pins, 0, sizeof(s->pins));
}

static void tlmm845_init(Object *obj)
{
    Qcom845Tlmm *s = QCOM845_TLMM(obj);

    for (unsigned i = 0; i < TLMM_TILES; i++) {
        s->tiles[i].owner = s;
        s->tiles[i].index = i;
        memory_region_init_io(&s->tiles[i].mmio, obj, &tlmm845_ops,
                              &s->tiles[i], TYPE_QCOM845_TLMM,
                              TLMM_PINS * TLMM_STRIDE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->tiles[i].mmio);
    }
}

static const VMStateDescription tlmm845_vmstate = {
    .name = TYPE_QCOM845_TLMM,
    .unmigratable = true,
};

static void tlmm845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, tlmm845_reset);
    dc->desc = "Experimental SDM845 TLMM pin discovery and output latches";
    dc->vmsd = &tlmm845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo tlmm845_type = {
    .name = TYPE_QCOM845_TLMM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Tlmm),
    .instance_init = tlmm845_init,
    .class_init = tlmm845_class_init,
};

static void tlmm845_register_types(void)
{
    type_register_static(&tlmm845_type);
}
type_init(tlmm845_register_types)

void qcom845_tlmm_create(void)
{
    static const hwaddr bases[] = { 0x03500000, 0x03900000, 0x03d00000 };
    SysBusDevice *dev = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM845_TLMM));

    sysbus_realize_and_unref(dev, &error_fatal);
    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        sysbus_mmio_map(dev, i, bases[i]);
    }
}

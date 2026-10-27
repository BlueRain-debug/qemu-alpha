/*
 * Experimental Tegra20 APB DMA register probe for early Windows ARM HALs.
 *
 * This models initialization and idle state only. Enabled channels remain
 * busy: no data transfer or completion interrupt is synthesized.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_TEGRA20_APB_DMA "tegra20-apb-dma"
OBJECT_DECLARE_SIMPLE_TYPE(Tegra20ApbDmaState, TEGRA20_APB_DMA)

#define DMA_CHANNELS 16
#define DMA_CHANNEL_BASE 0x1000
#define DMA_CHANNEL_STRIDE 0x20
#define DMA_CHANNEL_REGS (DMA_CHANNELS * 8)
#define DMA_ENABLE BIT(31)
#define DMA_STATUS_BUSY BIT(31)

struct Tegra20ApbDmaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t general;
    uint32_t control;
    uint32_t irq_mask;
    uint32_t channel_regs[DMA_CHANNEL_REGS];
};

static uint64_t tegra20_apbdma_read(void *opaque, hwaddr addr, unsigned size)
{
    Tegra20ApbDmaState *s = opaque;

    switch (addr) {
    case 0: return s->general;
    case 0x10: return s->control;
    case 0x1c: return s->irq_mask;
    case 0x20:
    case 0x24: return 0;
    default:
        if (addr >= DMA_CHANNEL_BASE &&
            addr < DMA_CHANNEL_BASE + DMA_CHANNELS * DMA_CHANNEL_STRIDE) {
            unsigned index = (addr - DMA_CHANNEL_BASE) / 4;

            if (index % 8 == 1) {
                return (s->general & DMA_ENABLE) &&
                       (s->channel_regs[index - 1] & DMA_ENABLE) ?
                       DMA_STATUS_BUSY : 0;
            }
            return s->channel_regs[index];
        }
        return 0;
    }
}

static void tegra20_apbdma_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    Tegra20ApbDmaState *s = opaque;

    switch (addr) {
    case 0:
        s->general = value & DMA_ENABLE;
        break;
    case 0x10:
        s->control = value;
        break;
    case 0x1c:
        s->irq_mask = value & 0xffff;
        break;
    case 0x20:
        s->irq_mask |= value & 0xffff;
        break;
    case 0x24:
        s->irq_mask &= ~(value & 0xffff);
        break;
    default:
        if (addr >= DMA_CHANNEL_BASE &&
            addr < DMA_CHANNEL_BASE + DMA_CHANNELS * DMA_CHANNEL_STRIDE) {
            unsigned index = (addr - DMA_CHANNEL_BASE) / 4;

            if (index % 8 == 1) {
                return; /* No completion status is pending to acknowledge. */
            }
            s->channel_regs[index] = value;
            if (index % 8 == 0 && (value & DMA_ENABLE)) {
                qemu_log_mask(LOG_UNIMP,
                              "tegra20-apbdma: channel %u transfers are not implemented\n",
                              index / 8);
            }
        }
        break;
    }
}

static const MemoryRegionOps tegra20_apbdma_ops = {
    .read = tegra20_apbdma_read,
    .write = tegra20_apbdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void tegra20_apbdma_reset(DeviceState *dev)
{
    Tegra20ApbDmaState *s = TEGRA20_APB_DMA(dev);

    s->general = s->control = s->irq_mask = 0;
    memset(s->channel_regs, 0, sizeof(s->channel_regs));
}

static const VMStateDescription vmstate_tegra20_apbdma = {
    .name = TYPE_TEGRA20_APB_DMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(general, Tegra20ApbDmaState),
        VMSTATE_UINT32(control, Tegra20ApbDmaState),
        VMSTATE_UINT32(irq_mask, Tegra20ApbDmaState),
        VMSTATE_UINT32_ARRAY(channel_regs, Tegra20ApbDmaState,
                            DMA_CHANNEL_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void tegra20_apbdma_init(Object *obj)
{
    Tegra20ApbDmaState *s = TEGRA20_APB_DMA(obj);

    memory_region_init_io(&s->iomem, obj, &tegra20_apbdma_ops, s,
                          TYPE_TEGRA20_APB_DMA, 0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void tegra20_apbdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Experimental Tegra20 APB DMA register probe (no transfers)";
    dc->vmsd = &vmstate_tegra20_apbdma;
    device_class_set_legacy_reset(dc, tegra20_apbdma_reset);
}

static const TypeInfo tegra20_apbdma_type = {
    .name = TYPE_TEGRA20_APB_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Tegra20ApbDmaState),
    .instance_init = tegra20_apbdma_init,
    .class_init = tegra20_apbdma_class_init,
};

static void tegra20_apbdma_register_types(void)
{
    type_register_static(&tegra20_apbdma_type);
}

type_init(tegra20_apbdma_register_types)

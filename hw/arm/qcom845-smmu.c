/*
 * SDM845 SMMUv2 register and bounded AArch64 stage-1 translation subset.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Separate from the MSM8998 graphics implementation. Stage 2, AArch32,
 * non-4K granules and stalled-transaction replay are not implemented.
 */
#include "qemu/osdep.h"
#include "hw/arm/oemsk850.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "trace.h"

#define TYPE_QCOM845_SMMU "qcom845-smmu"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Smmu, QCOM845_SMMU)
#define MAX_CONTEXTS 64
#define MAX_STREAMS 128
#define FSR_MASK (BIT(31) | 0x1fe)
#define FSR_TF BIT(1)
#define FSR_AFF BIT(2)
#define FSR_PF BIT(3)
#define FSR_EF BIT(4)
#define FSR_ASF BIT(7)
#define FSR_UUT BIT(8)
#define ADDRESS_MASK UINT64_C(0x000000fffffff000)

struct Qcom845Smmu {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    hwaddr bytes;
    hwaddr cb_base;
    unsigned contexts;
    unsigned num_irqs;
    unsigned streams;
    uint32_t *registers;
    qemu_irq global_irq;
    qemu_irq context_irq[MAX_CONTEXTS];
};

static uint64_t smmu845_reg64(Qcom845Smmu *s, unsigned off)
{
    return s->registers[off / 4] | (uint64_t)s->registers[off / 4 + 1] << 32;
}

static void smmu845_irq(Qcom845Smmu *s)
{
    qemu_set_irq(s->global_irq, (s->registers[0x48 / 4] &
                               (BIT(31) | 0x1ff)) &&
                              (s->registers[0] & BIT(2)));
    /* Aggregate before changing a shared line: no spurious low/high pulse. */
    for (unsigned irq = 0; irq < s->num_irqs; irq++) {
        bool pending = false;

        for (unsigned cb = 0; cb < s->contexts; cb++) {
            unsigned base = (s->cb_base + cb * 0x1000) / 4;

            if ((s->registers[(0x1000 + cb * 4) / 4] >> 24) == irq &&
                (s->registers[base] & BIT(6)) &&
                (s->registers[base + 0x58 / 4] & FSR_MASK)) {
                pending = true;
                break;
            }
        }
        qemu_set_irq(s->context_irq[irq], pending);
    }
}

static bool smmu845_descriptor(uint64_t address, uint64_t *value)
{
    MemoryRegionSection section = memory_region_find(get_system_memory(),
                                                     address, 8);
    uint8_t data[8];
    bool ram = section.mr && memory_region_is_ram(section.mr) &&
               int128_get64(section.size) >= sizeof(data);

    if (section.mr) {
        memory_region_unref(section.mr);
    }
    if (!ram || address_space_read(&address_space_memory, address,
                                  MEMTXATTRS_UNSPECIFIED, data, 8) != MEMTX_OK) {
        return false;
    }
    *value = ldq_le_p(data);
    return true;
}

static bool smmu845_walk(Qcom845Smmu *s, unsigned cb, uint64_t iova,
                         bool write, hwaddr *physical, uint32_t *fault)
{
    unsigned base = s->cb_base + cb * 0x1000;
    uint32_t control = s->registers[base / 4];
    uint32_t tcr2 = s->registers[(base + 0x10) / 4];
    uint32_t tcr = s->registers[(base + 0x30) / 4];
    uint32_t cbar = s->registers[(0x1000 + cb * 4) / 4];
    bool upper;
    unsigned va_bits;
    uint64_t va_mask;
    uint64_t table;
    bool table_ro = false;
    static const unsigned sep_bits[8] = { 32, 36, 40, 42, 44, 48, 0, 64 };
    unsigned sep = sep_bits[(tcr2 >> 15) & 7];

    *fault = FSR_UUT;
    if (!(control & 1)) {
        *physical = iova;
        return true;
    }
    if (((cbar >> 16) & 3) != 1 ||
        !(s->registers[(0x1800 + cb * 4) / 4] & 1) ||
        sep == 0) {
        return false;
    }
    if (sep < 64) {
        if (iova >> sep) {
            *fault = FSR_TF;
            return false;
        }
        if (sep > 32) {
            iova = sextract64(iova, 0, sep);
        }
    }
    upper = (iova >> 63) != 0;
    if (tcr & (upper ? BIT(23) : BIT(7))) {
        *fault = FSR_TF;
        return false;
    }
    if (upper ? (((tcr >> 30) & 3) != 2) : (((tcr >> 14) & 3) != 0)) {
        return false;
    }
    va_bits = 64 - ((tcr >> (upper ? 16 : 0)) & 0x3f);
    if (va_bits < 16 || va_bits > 48) {
        return false;
    }
    *fault = FSR_TF;
    va_mask = MAKE_64BIT_MASK(0, va_bits);
    if ((iova & ~va_mask) != (upper ? ~va_mask : 0)) {
        return false;
    }
    table = smmu845_reg64(s, base + (upper ? 0x28 : 0x20));
    if (table & UINT64_C(0x0000ff0000000000)) {
        *fault = FSR_ASF;
        return false;
    }
    table &= ADDRESS_MASK;
    for (unsigned shift = 12 + 9 * ((va_bits - 12 - 1) / 9); ; shift -= 9) {
        uint64_t descriptor;
        unsigned bits = MIN(9U, va_bits - shift);
        uint64_t at = (table & ~MAKE_64BIT_MASK(0, bits + 3)) +
                      extract64(iova, shift, bits) * 8;

        if (!smmu845_descriptor(at, &descriptor)) {
            *fault = FSR_EF;
            return false;
        }
        if (!(descriptor & 1)) {
            return false;
        }
        if (descriptor & UINT64_C(0x0000ff0000000000)) {
            *fault = FSR_ASF;
            return false;
        }
        if ((descriptor & 3) == 3 && shift > 12) {
            table_ro |= (descriptor & BIT_ULL(62)) != 0;
            table = descriptor & ADDRESS_MASK;
            continue;
        }
        if ((shift == 39) || ((descriptor & 3) != (shift == 12 ? 3 : 1))) {
            return false;
        }
        if (descriptor & ADDRESS_MASK & ((UINT64_C(1) << shift) - 1)) {
            return false;
        }
        if (!(descriptor & BIT_ULL(10))) {
            *fault = FSR_AFF;
            return false;
        }
        if (write && (table_ro || (descriptor & BIT_ULL(7)))) {
            *fault = FSR_PF;
            return false;
        }
        *physical = (descriptor & ADDRESS_MASK & ~((UINT64_C(1) << shift) - 1)) |
                    (iova & ((UINT64_C(1) << shift) - 1));
        return true;
    }
    return false;
}

static void smmu845_fault(Qcom845Smmu *s, unsigned cb, uint16_t sid,
                          uint64_t iova, bool write, uint32_t fault)
{
    unsigned base = (s->cb_base + cb * 0x1000) / 4;

    if (s->registers[base + 0x58 / 4] & FSR_MASK) {
        s->registers[base + 0x58 / 4] |= BIT(31);
    } else {
        s->registers[base + 0x60 / 4] = iova;
        s->registers[base + 0x64 / 4] = iova >> 32;
        s->registers[base + 0x68 / 4] = write ? BIT(4) : 0;
        s->registers[(0x1400 + cb * 4) / 4] = sid;
    }
    s->registers[base + 0x58 / 4] |= fault;
    trace_qcom845_smmu_fault(s->cb_base, cb, sid, iova, fault);
    smmu845_irq(s);
}

bool qcom845_smmu_translate(Qcom845Smmu *s, uint16_t sid, uint64_t iova,
                            bool write, hwaddr *physical)
{
    unsigned selected = MAX_STREAMS;
    unsigned matches = 0;
    uint32_t syndrome = 0;

    if (s->registers[0] & 1) {
        *physical = iova;
        return true;
    }
    for (unsigned i = 0; i < s->streams; i++) {
        uint32_t smr = s->registers[(0x800 + i * 4) / 4];
        uint16_t mask = (smr >> 16) & 0x7fff;

        if ((smr & BIT(31)) && ((sid ^ smr) & ~mask & 0x7fff) == 0) {
            selected = i;
            matches++;
        }
    }
    if (!matches && !(s->registers[0] & BIT(10))) {
        *physical = iova;
        return true;
    }
    if (matches == 1) {
        uint32_t route = s->registers[(0xc00 + selected * 4) / 4];
        unsigned cb = route & 0xff;
        unsigned type = (route >> 16) & 3;

        if (type == 1) {
            *physical = iova;
            return true;
        }
        if (type == 0 && cb < s->contexts) {
            uint32_t fault;

            if (smmu845_walk(s, cb, iova, write, physical, &fault)) {
                return true;
            }
            smmu845_fault(s, cb, sid, iova, write, fault);
            return false;
        }
        syndrome = type == 2 ? BIT(2) : BIT(3);
    } else {
        syndrome = matches ? BIT(2) : BIT(1);
    }
    if (s->registers[0x48 / 4]) {
        s->registers[0x48 / 4] |= BIT(31);
    }
    s->registers[0x48 / 4] |= syndrome;
    s->registers[0x54 / 4] = sid;
    smmu845_irq(s);
    return false;
}

static uint64_t smmu845_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom845Smmu *s = opaque;
    uint32_t value;

    switch (offset) {
    case 0x20:
        value = BIT(30) | BIT(27) | (1U << 16) | (15U << 9) | s->streams;
        break;
    case 0x24:
        value = ((ctz64(s->cb_base) - 13) << 28) | s->contexts;
        break;
    case 0x28:
        value = BIT(12) | (5 << 8) | (2 << 4) | 2;
        break;
    case 0x3c:
        value = 0x20;
        break;
    default:
        value = s->registers[offset / 4];
        break;
    }
    return size == 8 ? value | (uint64_t)s->registers[offset / 4 + 1] << 32 : value;
}

static void smmu845_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    Qcom845Smmu *s = opaque;
    unsigned local = offset >= s->cb_base ? (offset - s->cb_base) % 0x1000 : 0;
    unsigned cb = offset >= s->cb_base ? (offset - s->cb_base) / 0x1000 : MAX_CONTEXTS;

    if (offset >= 0x20 && offset <= 0x3f) {
        return;
    }
    if (offset == 0x48 || (cb < s->contexts && local == 0x58)) {
        s->registers[offset / 4] &= ~value;
    } else if (offset == 0x4c || (cb < s->contexts && local == 0x5c)) {
        s->registers[offset / 4 - 1] |= value & (cb < s->contexts ? FSR_MASK : (BIT(31) | 0x1ff));
    } else if (offset == 0x74 || (cb < s->contexts && local == 0x7f4)) {
        return; /* TLBI operations complete synchronously; no cached translations. */
    } else {
        s->registers[offset / 4] = value;
        if (size == 8) {
            s->registers[offset / 4 + 1] = value >> 32;
        }
    }
    if (cb < s->contexts && (local == 0x800 || local == 0x808) && size == 8) {
        hwaddr physical;
        uint32_t fault;
        uint64_t par = smmu845_walk(s, cb, value, local == 0x808, &physical, &fault) ?
                       physical & ~UINT64_C(0xfff) : 1;
        unsigned at = (s->cb_base + cb * 0x1000 + 0x50) / 4;

        /* ATOS reports failure through PAR, without creating a DMA fault. */
        s->registers[at] = par;
        s->registers[at + 1] = par >> 32;
    }
    smmu845_irq(s);
}

static const MemoryRegionOps smmu845_ops = {
    .read = smmu845_read,
    .write = smmu845_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .valid.unaligned = false,
    .impl.min_access_size = 4,
    .impl.max_access_size = 8,
};

static void smmu845_reset(DeviceState *dev)
{
    Qcom845Smmu *s = QCOM845_SMMU(dev);

    memset(s->registers, 0, s->bytes);
    s->registers[0] = 1; /* CLIENTPD */
    for (unsigned cb = 0; cb < s->contexts; cb++) {
        s->registers[(0x1000 + cb * 4) / 4] =
            ((cb % s->num_irqs) << 24) | (1U << 16);
    }
    smmu845_irq(s);
}

static void smmu845_finalize(Object *obj)
{
    g_free(QCOM845_SMMU(obj)->registers);
}

static const VMStateDescription smmu845_vmstate = {
    .name = TYPE_QCOM845_SMMU,
    .unmigratable = true,
};

static void smmu845_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, smmu845_reset);
    dc->desc = "Experimental SDM845 SMMUv2 stage-1 subset";
    dc->vmsd = &smmu845_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo smmu845_type = {
    .name = TYPE_QCOM845_SMMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Smmu),
    .instance_finalize = smmu845_finalize,
    .class_init = smmu845_class_init,
};

static void smmu845_register_types(void)
{
    type_register_static(&smmu845_type);
}
type_init(smmu845_register_types)

Qcom845Smmu *qcom845_smmu_create(DeviceState *gic, hwaddr base,
                                hwaddr size, unsigned contexts,
                                unsigned streams, unsigned global_intid,
                                unsigned num_irqs,
                                const unsigned *context_intids)
{
    Qcom845Smmu *s = QCOM845_SMMU(qdev_new(TYPE_QCOM845_SMMU));
    SysBusDevice *dev = SYS_BUS_DEVICE(s);

    assert(contexts > 0 && contexts <= MAX_CONTEXTS && streams <= MAX_STREAMS);
    assert(num_irqs > 0 && num_irqs <= contexts);
    s->bytes = size;
    s->cb_base = size / 2;
    s->contexts = contexts;
    s->num_irqs = num_irqs;
    s->streams = streams;
    assert(s->cb_base + contexts * 0x1000 <= size);
    s->registers = g_new0(uint32_t, size / 4);
    memory_region_init_io(&s->mmio, OBJECT(s), &smmu845_ops, s,
                          TYPE_QCOM845_SMMU, size);
    sysbus_init_mmio(dev, &s->mmio);
    sysbus_init_irq(dev, &s->global_irq);
    for (unsigned i = 0; i < num_irqs; i++) {
        sysbus_init_irq(dev, &s->context_irq[i]);
    }
    sysbus_realize_and_unref(dev, &error_fatal);
    sysbus_mmio_map(dev, 0, base);
    sysbus_connect_irq(dev, 0, qdev_get_gpio_in(gic, global_intid - 32));
    for (unsigned i = 0; i < num_irqs; i++) {
        sysbus_connect_irq(dev, i + 1,
                           qdev_get_gpio_in(gic, context_intids[i] - 32));
    }
    return s;
}

/*
 * Local MSM8x60 graphics initialization probe.
 *
 * Provides native MMIO windows and memory so that the original miniport's
 * first accesses can be traced. Provides device IDs and a deliberately small
 * ARMv7 GPU translation and a small A2xx state/synchronization command subset.
 * Unsupported packets remain pending. Includes a bounded software drawing
 * subset and the linear RGB1/LCDC display path for local bring-up.
 * The original machine's SSDT/panel data must be supplied separately.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/display/qcom-mdp4-probe.h"
#include "hw/display/qcom-adreno-a2xx.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(QcomMdp4ProbeState, QCOM_MDP4_PROBE)

typedef struct QcomMdp4ProbeBank {
    MemoryRegion mmio;
    unsigned index;
    uint32_t *registers;
    QcomMdp4ProbeState *owner;
    hwaddr last_read_offset;
    uint32_t last_read_value;
    uint64_t repeated_reads;
} QcomMdp4ProbeBank;

/* Keep nested IB progress across a pending memory wait. Replaying a partly
 * executed IB would duplicate memory writes and interrupts. */
#define QCOM_CP_MAX_STREAMS 8
typedef struct QcomProbeCommandStream {
    uint32_t base;
    uint32_t capacity;
    uint32_t position;
    uint32_t end;
    uint32_t parent_packet_words;
    bool ring;
} QcomProbeCommandStream;

struct QcomMdp4ProbeState {
    SysBusDevice parent_obj;
    QcomMdp4ProbeBank banks[QCOM_MDP4_PROBE_BANKS];
    MemoryRegion vram;
    qemu_irq irq[QCOM_MDP4_PROBE_IRQS];
    uint32_t me_init[18];
    bool me_initialized;
    bool upload_endian_workaround;
    QEMUTimer *command_timer;
    QEMUTimer *display_timer;
    QemuConsole *console;
    uint32_t rgb1[24];
    uint32_t solid_color;
    bool overlay_pending;
    QcomProbeCommandStream streams[QCOM_CP_MAX_STREAMS];
    unsigned stream_count;
    bool protected_mode;
    uint32_t draw_init_flags;
    uint32_t shader_bases;
    uint32_t shader_instructions[3][4096];
    uint32_t shader_words[3];
    uint32_t stall_address;
    uint32_t stall_header;
    const char *stall_reason;
    const char *draw_failure;
    unsigned draw_failure_detail;
    /* A220 has 512 KiB of on-chip render memory, independent of system RAM. */
    uint32_t gmem[512 * 1024 / 4];
};

/* MDP_VERSION is the first register; its top byte identifies MDP generation. */
#define MDP4_VERSION_REGISTER 0
#define MDP4_VERSION_4_0 (4U << 24)
#define ADRENO_BANK 4
#define ADRENO_PERIPHID1 0xfe4
#define ADRENO_PERIPHID2 0xfe8
#define ADRENO_PATCH_RELEASE 0x4

/* A2xx command-processor register offsets are byte addresses here. */
#define CP_RB_BASE 0x700
#define CP_RB_CNTL 0x704
#define CP_RB_RPTR_ADDR 0x70c
#define CP_RB_RPTR 0x710
#define CP_RB_WPTR 0x714
#define CP_ME_CNTL 0x7d8
#define CP_ME_HALT (1U << 28)
#define CP_INT_CNTL 0x7c8
#define CP_INT_STATUS 0x7cc
#define CP_INT_ACK 0x7d0
#define GPU_MASTER_INT_SIGNAL 0xedc
#define GPU_MASTER_CP_INT (1U << 30)
#define GPU_3D_IRQ 2 /* Native GSI 112, third interrupt in GPU0._CRS. */
#define CP_PACKET_TYPE3 3
#define CP_OPCODE_NOP 0x10
#define CP_OPCODE_REG_RMW 0x21
#define CP_OPCODE_DRAW_INDX 0x22
#define CP_OPCODE_WAIT_IDLE 0x26
#define CP_OPCODE_IM_LOAD_IMMEDIATE 0x2b
#define CP_OPCODE_SET_CONSTANT 0x2d
#define CP_OPCODE_IB_PFD 0x37
#define CP_OPCODE_INVALIDATE_STATE 0x3b
#define CP_OPCODE_WAIT_REG_MEM 0x3c
#define CP_OPCODE_MEM_WRITE 0x3d
#define CP_OPCODE_REG_TO_MEM 0x3e
#define CP_OPCODE_IB 0x3f
#define CP_OPCODE_INTERRUPT 0x40
#define CP_OPCODE_COND_EXEC 0x44
#define CP_OPCODE_EVENT_WRITE 0x46
#define CP_OPCODE_ME_INIT 0x48
#define CP_OPCODE_SET_SHADER_BASES 0x4a
#define CP_OPCODE_SET_DRAW_INIT_FLAGS 0x4b
#define CP_OPCODE_WAIT_REG_EQ 0x52
#define CP_OPCODE_PROTECTED_MODE 0x5f
#define CP_EVENT_CACHE_FLUSH_TS 4
#define CP_EVENT_CACHE_FLUSH 6
#define CP_EVENT_CACHE_FLUSH_INVALIDATE 22
#define CP_MAX_PACKET_WORDS 16384
#define CP_CALLBACK_PACKET_BUDGET 256
#define CP_DISPATCH_DELAY_NS 1000 /* Decouple doorbells from command execution. */
#define CP_WAIT_RETRY_NS 1000000  /* Let the CPU change a pending wait operand. */
#define GPU_SMMU_BANK 16
#define SMMU_SCTLR 0x00
#define SMMU_TTBR0 0x10
#define SMMU_TTBR1 0x14
#define SMMU_TTBCR 0x18

/* MDP4 primary LCD path: RGB1 -> mixer 0 -> DMA_P -> LCDC. */
#define MDP_BANK 0
#define MDP_SMMU_BANK 9
#define MDP_IRQ 0
#define MDP_INTR_ENABLE 0x50
#define MDP_INTR_STATUS 0x54
#define MDP_INTR_CLEAR 0x58
#define MDP_MIXER_CONFIG 0x10100
#define MDP_OVERLAY_FLUSH 0x18000
#define MDP_RGB1_BASE 0x40000
#define MDP_RGB1_COLOR 0x41008
#define MDP_LCDC_ENABLE 0xc0000
#define MDP_RGB1_FLUSH (1U << 4)
#define MDP_OVERLAY0_FLUSH (1U << 0)
#define MDP_RGB1_MIXER0_BASE (1U << 8)
#define MDP_IRQ_OVERLAY0_DONE (1U << 0)
#define MDP_IRQ_DMA_P_DONE (1U << 4)
#define MDP_IRQ_PRIMARY_VSYNC (1U << 7)
#define MDP_IRQ_PRIMARY_RDPTR (1U << 11)
#define MDP_FRAME_NS (NANOSECONDS_PER_SECOND / 60)
#define MDP_MAX_DIMENSION 2048

static bool qcom_probe_ram32(hwaddr address, uint32_t *value, bool write)
{
    MemoryRegionSection section = memory_region_find(get_system_memory(),
                                                      address, sizeof(*value));
    bool valid = section.mr && memory_region_is_ram(section.mr) &&
                 int128_ge(section.size, int128_make64(sizeof(*value))) &&
                 (!write || !memory_region_is_rom(section.mr));
    MemTxResult result;

    if (section.mr) {
        memory_region_unref(section.mr);
    }
    if (!valid) {
        return false; /* Never interpret a GPU pointer as a CPU MMIO access. */
    }
    if (write) {
        address_space_stl_le(&address_space_memory, address, *value,
                            MEMTXATTRS_UNSPECIFIED, &result);
    } else {
        *value = address_space_ldl_le(&address_space_memory, address,
                                     MEMTXATTRS_UNSPECIFIED, &result);
    }
    return result == MEMTX_OK;
}

static bool qcom_probe_iommu_address(QcomMdp4ProbeState *s, unsigned bank,
                                     uint32_t address, hwaddr *physical)
{
    uint32_t *context = s->banks[bank].registers;
    uint32_t n = context[SMMU_TTBCR / 4] & 7;
    uint32_t table, entry;
    unsigned alignment;

    /* This bring-up subset uses the two identically configured context
     * banks observed in the original miniport. Refuse differing contexts,
     * rather than silently selecting the wrong stream's address space. */
    for (unsigned offset = 0; offset <= SMMU_TTBCR; offset += 4) {
        if (context[offset / 4] != context[(0x1000 + offset) / 4]) {
            return false;
        }
    }
    if (!(context[SMMU_SCTLR / 4] & 1)) {
        return false;
    }
    if (n && address >= (1ULL << (32 - n))) {
        table = context[SMMU_TTBR1 / 4];
        alignment = 14;
    } else {
        table = context[SMMU_TTBR0 / 4];
        alignment = 14 - n;
    }
    table &= ~((1U << alignment) - 1);
    if (!qcom_probe_ram32(table + (address >> 20) * 4, &entry, false)) {
        return false;
    }
    switch (entry & 3) {
    case 2: /* ARMv7 short-descriptor 1 MiB section (not a supersection). */
        if (entry & (1U << 18)) {
            return false;
        }
        *physical = (entry & 0xfff00000) | (address & 0xfffff);
        return true;
    case 1: /* Coarse second-level table. */
        if (!qcom_probe_ram32((entry & 0xfffffc00) +
                              ((address >> 12) & 255) * 4, &entry, false)) {
            return false;
        }
        if ((entry & 3) == 1) { /* 64 KiB large page. */
            *physical = (entry & 0xffff0000) | (address & 0xffff);
            return true;
        }
        if (entry & 2) { /* 4 KiB small page, including XN variant. */
            *physical = (entry & 0xfffff000) | (address & 0xfff);
            return true;
        }
        return false;
    default:
        return false;
    }
}

static bool qcom_probe_gpu_address(QcomMdp4ProbeState *s, uint32_t address,
                                   hwaddr *physical)
{
    return qcom_probe_iommu_address(s, GPU_SMMU_BANK, address, physical);
}

static void qcom_probe_update_mdp_irq(QcomMdp4ProbeState *s)
{
    uint32_t *regs = s->banks[MDP_BANK].registers;

    qemu_set_irq(s->irq[MDP_IRQ],
                 (regs[MDP_INTR_ENABLE / 4] & regs[MDP_INTR_STATUS / 4]) != 0);
}

static bool qcom_probe_display_read(QcomMdp4ProbeState *s, uint32_t address,
                                    uint8_t *data, size_t length)
{
    /* Translate once per page and copy RAM in chunks. Per-pixel page walks
     * would dominate host CPU time even for the original 480x800 panel. */
    while (length) {
        hwaddr physical;
        size_t chunk = MIN(length, 4096 - (address & 4095));
        MemoryRegionSection section;
        bool valid;

        if (!qcom_probe_iommu_address(s, MDP_SMMU_BANK, address, &physical)) {
            return false;
        }
        section = memory_region_find(get_system_memory(), physical, chunk);
        valid = section.mr && memory_region_is_ram(section.mr) &&
                int128_ge(section.size, int128_make64(chunk));
        if (section.mr) {
            memory_region_unref(section.mr);
        }
        if (!valid || address_space_read(&address_space_memory, physical,
                MEMTXATTRS_UNSPECIFIED, data, chunk) != MEMTX_OK) {
            return false;
        }
        address += chunk;
        data += chunk;
        length -= chunk;
    }
    return true;
}

static bool qcom_probe_display_update(void *opaque)
{
    QcomMdp4ProbeState *s = opaque;
    uint32_t *regs = s->banks[MDP_BANK].registers;
    uint32_t format = s->rgb1[0x50 / 4];
    uint32_t unpack = s->rgb1[0x54 / 4];
    unsigned width = s->rgb1[0] & 0xffff;
    unsigned height = s->rgb1[0] >> 16;
    unsigned stride = s->rgb1[0x40 / 4] & 0xffff;
    bool solid = format & (1U << 22);
    DisplaySurface *surface;
    g_autofree uint8_t *row = NULL;

    if (!(regs[MDP_LCDC_ENABLE / 4] & 1) ||
        (regs[MDP_MIXER_CONFIG / 4] & 0xf00) != MDP_RGB1_MIXER0_BASE ||
        !width || !height || width > MDP_MAX_DIMENSION ||
        height > MDP_MAX_DIMENSION || s->rgb1[1] || s->rgb1[3] ||
        s->rgb1[2] != s->rgb1[0] || s->rgb1[0x58 / 4] ||
        /* Only tightly packed, interleaved 8-bit RGBA is implemented. */
        (format & 0x7e0a67ff) != 0x000267ff ||
        (!solid && (stride < width * 4 ||
          (uint64_t)s->rgb1[4] + (uint64_t)stride * height > UINT32_MAX))) {
        return false;
    }
    if (!s->console) {
        return false;
    }
    row = g_malloc(width * 4);
    qemu_console_resize(s->console, width, height);
    surface = qemu_console_surface(s->console);
    for (unsigned y = 0; y < height; y++) {
        uint32_t *destination = (uint32_t *)(surface_data(surface) +
                                             y * surface_stride(surface));

        if (!solid && !qcom_probe_display_read(s, s->rgb1[4] + y * stride,
                                               row, width * 4)) {
            return false;
        }
        for (unsigned x = 0; x < width; x++) {
            uint8_t components[4] = {0};

            if (solid) {
                destination[x] = s->solid_color & 0xffffff;
                continue;
            }
            for (unsigned component = 0; component < 4; component++) {
                unsigned selector = (unpack >> (component * 8)) & 0xff;

                if (selector > 3) {
                    return false;
                }
                components[selector] = row[x * 4 + component];
            }
            /* MDP component numbers: 0=G, 1=B, 2=R, 3=A. */
            destination[x] = rgb_to_pixel32(components[2], components[0],
                                             components[1]);
        }
    }
    qemu_console_update(s->console, 0, 0, width, height);
    return true;
}

static bool qcom_probe_graphic_update(void *opaque)
{
    qcom_probe_display_update(opaque);
    return true; /* Synchronous, including an unconfigured or faulted plane. */
}

static const GraphicHwOps qcom_probe_display_ops = {
    .gfx_update = qcom_probe_graphic_update,
};

static void qcom_probe_display_tick(void *opaque)
{
    QcomMdp4ProbeState *s = opaque;
    uint32_t *regs = s->banks[MDP_BANK].registers;

    if (!(regs[MDP_LCDC_ENABLE / 4] & 1)) {
        return;
    }
    if (qcom_probe_display_update(s)) {
        /* FLUSH bits remain busy until the supported plane has reached a
         * real frame boundary. The miniport polls these before completing
         * a present. Preserve unsupported bits and pending failed scanout. */
        regs[MDP_OVERLAY_FLUSH / 4] &= ~(MDP_RGB1_FLUSH | MDP_OVERLAY0_FLUSH);
        /* Model read-pointer progress at frame granularity. The native
         * driver enables RDPTR as well as VSYNC and uses it to schedule
         * subsequent presents. Do not signal either on a failed scanout. */
        regs[MDP_INTR_STATUS / 4] |= MDP_IRQ_PRIMARY_VSYNC |
                                    MDP_IRQ_PRIMARY_RDPTR | MDP_IRQ_DMA_P_DONE;
        if (s->overlay_pending) {
            regs[MDP_INTR_STATUS / 4] |= MDP_IRQ_OVERLAY0_DONE;
            s->overlay_pending = false;
        }
        qcom_probe_update_mdp_irq(s);
    }
    timer_mod(s->display_timer,
               qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MDP_FRAME_NS);
}

static bool qcom_probe_gpu_read32(QcomMdp4ProbeState *s, uint32_t address,
                                  uint32_t *value)
{
    hwaddr physical;

    return !(address & 3) && qcom_probe_gpu_address(s, address, &physical) &&
           qcom_probe_ram32(physical, value, false);
}

static bool qcom_probe_gpu_write32(QcomMdp4ProbeState *s, uint32_t address,
                                   uint32_t value)
{
    hwaddr physical;

    return !(address & 3) && qcom_probe_gpu_address(s, address, &physical) &&
           qcom_probe_ram32(physical, &value, true);
}

static bool qcom_probe_draw_read(void *opaque, uint32_t address, uint32_t *value)
{
    return qcom_probe_gpu_read32(opaque, address, value);
}

static bool qcom_probe_draw_write(void *opaque, uint32_t address, uint32_t value)
{
    return qcom_probe_gpu_write32(opaque, address, value);
}

static bool qcom_probe_draw_writable(void *opaque, uint32_t address)
{
    hwaddr physical;
    MemoryRegionSection section;
    bool valid;

    if ((address & 3) || !qcom_probe_gpu_address(opaque, address, &physical)) {
        return false;
    }
    section = memory_region_find(get_system_memory(), physical, 4);
    valid = section.mr && memory_region_is_ram(section.mr) &&
            !memory_region_is_rom(section.mr) &&
            int128_ge(section.size, int128_make64(4));
    if (section.mr) {
        memory_region_unref(section.mr);
    }
    return valid;
}

static void qcom_probe_update_gpu_irq(QcomMdp4ProbeState *s)
{
    uint32_t *regs = s->banks[ADRENO_BANK].registers;
    bool pending = (regs[CP_INT_STATUS / 4] & regs[CP_INT_CNTL / 4]) != 0;

    regs[GPU_MASTER_INT_SIGNAL / 4] = pending ? GPU_MASTER_CP_INT : 0;
    qemu_set_irq(s->irq[GPU_3D_IRQ], pending);
    trace_qcom_mdp4_probe_irq(regs[CP_INT_STATUS / 4],
                             regs[CP_INT_CNTL / 4], pending);
}

static uint32_t qcom_probe_stream_available(const QcomProbeCommandStream *stream)
{
    return stream->ring ? (stream->end - stream->position) &
                          (stream->capacity - 1) :
                          stream->end - stream->position;
}

static uint32_t qcom_probe_stream_index(const QcomProbeCommandStream *stream,
                                        uint32_t relative)
{
    uint32_t index = stream->position + relative;

    return stream->ring ? index & (stream->capacity - 1) : index;
}

static void qcom_probe_stream_advance(QcomProbeCommandStream *stream,
                                      uint32_t words)
{
    stream->position = qcom_probe_stream_index(stream, words);
}

static void qcom_probe_command_stall(QcomMdp4ProbeState *s,
                                     QcomProbeCommandStream *stream,
                                     uint32_t header, const char *reason)
{
    uint32_t address = stream->base + stream->position * 4;

    if (s->stall_address != address || s->stall_header != header ||
        s->stall_reason != reason) {
        trace_qcom_mdp4_probe_ring_stall(address, header, reason);
        s->stall_address = address;
        s->stall_header = header;
        s->stall_reason = reason;
    }
}

static bool qcom_probe_publish_rptr(QcomMdp4ProbeState *s)
{
    uint32_t *regs = s->banks[ADRENO_BANK].registers;
    uint32_t position = s->streams[0].position;

    if (!qcom_probe_gpu_write32(s, regs[CP_RB_RPTR_ADDR / 4], position)) {
        return false;
    }
    regs[CP_RB_RPTR / 4] = position;
    return true;
}

typedef enum QcomProbeCommandResult {
    QCOM_CP_DONE,
    QCOM_CP_WAIT,
    QCOM_CP_UNSUPPORTED,
    QCOM_CP_FAULT,
    QCOM_CP_PUSHED_IB,
} QcomProbeCommandResult;

static bool qcom_probe_compare(uint32_t value, uint32_t reference,
                                unsigned function)
{
    switch (function) {
    case 0: return true;
    case 1: return value < reference;
    case 2: return value <= reference;
    case 3: return value == reference;
    case 4: return value != reference;
    case 5: return value >= reference;
    case 6: return value > reference;
    default: return false;
    }
}

static QcomProbeCommandResult qcom_probe_execute_packet(
    QcomMdp4ProbeState *s, uint32_t header, const uint32_t *payload,
    uint32_t count, uint32_t *skip_words)
{
    uint32_t *regs = s->banks[ADRENO_BANK].registers;
    unsigned type = header >> 30;
    unsigned opcode = (header >> 8) & 0xff;
    uint32_t value, reference;

    if (type == 0) { /* Consecutive register writes, or one repeated register. */
        uint32_t reg = header & 0x7fff;
        bool repeated = header & (1U << 15);

        if (reg >= 0x8000 || (!repeated && count > 0x8000 - reg)) {
            return QCOM_CP_FAULT;
        }
        for (unsigned i = 0; i < count; i++) {
            regs[reg + (repeated ? 0 : i)] = payload[i];
        }
        return QCOM_CP_DONE;
    }
    if (type == 2) {
        return QCOM_CP_DONE; /* One DWORD type-2 padding packet. */
    }
    if (type != CP_PACKET_TYPE3 || (header & 0xff)) {
        return QCOM_CP_UNSUPPORTED;
    }
    switch (opcode) {
    case CP_OPCODE_DRAW_INDX: {
        QcomA2xxDrawContext context = {
            .registers = regs,
            .shader = {s->shader_instructions[0], s->shader_instructions[1]},
            .shader_words = {s->shader_words[0], s->shader_words[1]},
            .gmem = s->gmem,
            .gmem_bytes = sizeof(s->gmem),
            .upload_endian_workaround = s->upload_endian_workaround,
            .opaque = s,
            .read_word = qcom_probe_draw_read,
            .write_word = qcom_probe_draw_write,
            .writable_word = qcom_probe_draw_writable,
        };
        bool success;

        /* Auto-indexed RECTLIST, three vertices, no visibility query.
         * Bit 14 enables the driver's pre-draw setup, bit 11 selects index
         * width (irrelevant for auto indexing). Other modes stay pending. */
        if (count == 3 && !payload[0] && payload[2] == 3 &&
            (payload[1] & ~0x4800U) == 0x88) {
            success = qcom_a2xx_draw_rect(&context);
        } else if (count == 5 && !payload[0] &&
                   (payload[1] & ~0x4000U) == 4) {
            success = qcom_a2xx_draw_triangles(&context, payload[3],
                                                payload[2], payload[4]);
        } else {
            return QCOM_CP_UNSUPPORTED;
        }
        if (!success) {
            if (s->draw_failure != context.failure ||
                s->draw_failure_detail != context.failure_detail) {
                trace_qcom_mdp4_probe_draw(false, context.failure_detail,
                                            context.failure);
                s->draw_failure = context.failure;
                s->draw_failure_detail = context.failure_detail;
            }
            return QCOM_CP_UNSUPPORTED;
        }
        s->draw_failure = NULL;
        trace_qcom_mdp4_probe_draw(true, payload[2], "draw completed");
        return QCOM_CP_DONE;
    }
    case CP_OPCODE_NOP:
        return QCOM_CP_DONE;
    case CP_OPCODE_ME_INIT:
        if (count != ARRAY_SIZE(s->me_init)) {
            return QCOM_CP_UNSUPPORTED;
        }
        memcpy(s->me_init, payload, sizeof(s->me_init));
        s->me_initialized = true;
        return QCOM_CP_DONE;
    case CP_OPCODE_COND_EXEC:
        if (count != 4 || payload[0] > UINT32_MAX / 4 ||
            payload[1] > UINT32_MAX / 4) {
            return QCOM_CP_UNSUPPORTED;
        }
        if (!qcom_probe_gpu_read32(s, payload[0] * 4, &value) ||
            !qcom_probe_gpu_read32(s, payload[1] * 4, &reference)) {
            return QCOM_CP_FAULT;
        }
        /* Timestamp ordering is wrap-aware: REF is ahead of the fetched
         * timestamp when their unsigned difference is less than 2^31. */
        if (!(value && (uint32_t)(payload[2] - reference) < 0x80000000U)) {
            *skip_words = payload[3];
        }
        trace_qcom_mdp4_probe_cond(value, reference, payload[2], *skip_words);
        return QCOM_CP_DONE;
    case CP_OPCODE_MEM_WRITE:
        if (count < 2 || (payload[0] & 3) ||
            count - 2 > (UINT32_MAX - payload[0]) / 4) {
            return QCOM_CP_UNSUPPORTED;
        }
        for (unsigned i = 1; i < count; i++) {
            if (!qcom_probe_gpu_write32(s, payload[0] + (i - 1) * 4,
                                        payload[i])) {
                return QCOM_CP_FAULT;
            }
        }
        return QCOM_CP_DONE;
    case CP_OPCODE_REG_RMW:
        if (count != 3 || (payload[0] & 0x3fff8000)) {
            return QCOM_CP_UNSUPPORTED;
        }
        value = regs[payload[0] & 0x7fff];
        value &= payload[0] & (1U << 31) ? regs[payload[1] & 0x7fff] : payload[1];
        value |= payload[0] & (1U << 30) ? regs[payload[2] & 0x7fff] : payload[2];
        regs[payload[0] & 0x7fff] = value;
        return QCOM_CP_DONE;
    case CP_OPCODE_REG_TO_MEM:
        if (count != 2 || payload[0] >= 0x8000) {
            return QCOM_CP_UNSUPPORTED;
        }
        return qcom_probe_gpu_write32(s, payload[1], regs[payload[0]]) ?
               QCOM_CP_DONE : QCOM_CP_FAULT;
    case CP_OPCODE_WAIT_IDLE:
        /* Every preceding supported command is synchronous. An unsupported
         * draw stops the stream before this packet, so no work is skipped. */
        return count == 1 ? QCOM_CP_DONE : QCOM_CP_UNSUPPORTED;
    case CP_OPCODE_WAIT_REG_EQ:
        if (count != 4 || payload[0] >= 0x8000) {
            return QCOM_CP_UNSUPPORTED;
        }
        return (regs[payload[0]] & payload[2]) == payload[1] ?
               QCOM_CP_DONE : QCOM_CP_WAIT;
    case CP_OPCODE_INVALIDATE_STATE:
        /* The interpreter reads state directly from registers and RAM;
         * there is no derived state or instruction cache to invalidate. */
        return count == 1 ? QCOM_CP_DONE : QCOM_CP_UNSUPPORTED;
    case CP_OPCODE_SET_DRAW_INIT_FLAGS:
        if (count != 1) {
            return QCOM_CP_UNSUPPORTED;
        }
        s->draw_init_flags = payload[0];
        return QCOM_CP_DONE;
    case CP_OPCODE_SET_SHADER_BASES:
        if (count != 1) {
            return QCOM_CP_UNSUPPORTED;
        }
        s->shader_bases = payload[0];
        return QCOM_CP_DONE;
    case CP_OPCODE_IM_LOAD_IMMEDIATE: {
        unsigned stage, start, words;

        if (count < 2) {
            return QCOM_CP_UNSUPPORTED;
        }
        stage = payload[0];
        start = payload[1] >> 16;
        words = payload[1] & 0xffff;
        if (stage >= ARRAY_SIZE(s->shader_words) ||
            words != count - 2 || start > 4096 || words > 4096 - start) {
            return QCOM_CP_UNSUPPORTED;
        }
        memcpy(&s->shader_instructions[stage][start], &payload[2], words * 4);
        s->shader_words[stage] = MAX(s->shader_words[stage], start + words);
        return QCOM_CP_DONE;
    }
    case CP_OPCODE_SET_CONSTANT: {
        /* A2xx constant selectors name ALU, fetch, boolean, loop, and
         * ordinary graphics registers, respectively (DWORD addresses). */
        static const uint32_t bases[] = {0x4000, 0x4800, 0x4900,
                                         0x4908, 0x2000};
        unsigned selector = (payload[0] >> 16) & 0xff;
        uint32_t first = payload[0] & 0x7ff;

        if (count < 2 || selector >= ARRAY_SIZE(bases) ||
            (payload[0] & 0xff00f800)) {
            return QCOM_CP_UNSUPPORTED;
        }
        first += bases[selector];
        if (count - 1 > 0x8000 - first) {
            return QCOM_CP_FAULT;
        }
        memcpy(&regs[first], &payload[1], (count - 1) * sizeof(*payload));
        return QCOM_CP_DONE;
    }
    case CP_OPCODE_WAIT_REG_MEM:
        if (count != 5 || (payload[0] & ~0x17U) || (payload[0] & 7) == 7) {
            return QCOM_CP_UNSUPPORTED;
        }
        if (payload[0] & 0x10) {
            if (!qcom_probe_gpu_read32(s, payload[1], &value)) {
                return QCOM_CP_FAULT;
            }
        } else {
            if (payload[1] >= 0x8000) {
                return QCOM_CP_UNSUPPORTED;
            }
            value = regs[payload[1]];
        }
        return qcom_probe_compare(value & payload[3], payload[2],
                                  payload[0] & 7) ? QCOM_CP_DONE : QCOM_CP_WAIT;
    case CP_OPCODE_EVENT_WRITE:
        if (count == 1 && (payload[0] == CP_EVENT_CACHE_FLUSH ||
                          payload[0] == CP_EVENT_CACHE_FLUSH_INVALIDATE)) {
            return QCOM_CP_DONE; /* RAM is coherent; earlier work is complete. */
        }
        if (count == 3 && payload[0] == CP_EVENT_CACHE_FLUSH_TS) {
            return qcom_probe_gpu_write32(s, payload[1], payload[2]) ?
                   QCOM_CP_DONE : QCOM_CP_FAULT;
        }
        return QCOM_CP_UNSUPPORTED;
    case CP_OPCODE_INTERRUPT:
        if (count != 1) {
            return QCOM_CP_UNSUPPORTED;
        }
        regs[CP_INT_STATUS / 4] |= payload[0];
        qcom_probe_update_gpu_irq(s);
        return QCOM_CP_DONE;
    case CP_OPCODE_PROTECTED_MODE:
        if (count != 1 || payload[0] > 1) {
            return QCOM_CP_UNSUPPORTED;
        }
        s->protected_mode = payload[0];
        return QCOM_CP_DONE;
    case CP_OPCODE_IB:
    case CP_OPCODE_IB_PFD:
        if (count != 2 || !payload[1] || payload[1] > CP_MAX_PACKET_WORDS ||
            (payload[0] & 3) || payload[1] > (UINT32_MAX - payload[0]) / 4 ||
            s->stream_count == QCOM_CP_MAX_STREAMS) {
            return QCOM_CP_UNSUPPORTED;
        }
        s->streams[s->stream_count++] = (QcomProbeCommandStream) {
            .base = payload[0], .capacity = payload[1], .end = payload[1],
            .parent_packet_words = count + 1,
        };
        return QCOM_CP_PUSHED_IB;
    default:
        return QCOM_CP_UNSUPPORTED;
    }
}

static void qcom_probe_process_ring(void *opaque)
{
    QcomMdp4ProbeState *s = opaque;
    uint32_t *regs = s->banks[ADRENO_BANK].registers;
    unsigned size_bits = regs[CP_RB_CNTL / 4] & 0x3f;
    uint32_t ring_words;

    if ((regs[CP_ME_CNTL / 4] & CP_ME_HALT) || size_bits > 13) {
        return;
    }
    ring_words = 1U << (size_bits + 1);
    if (regs[CP_RB_RPTR / 4] >= ring_words ||
        regs[CP_RB_WPTR / 4] >= ring_words ||
        regs[CP_RB_BASE / 4] > UINT32_MAX - ring_words * 4) {
        return;
    }
    if (!s->stream_count) {
        s->streams[0] = (QcomProbeCommandStream) {
            .base = regs[CP_RB_BASE / 4], .capacity = ring_words,
            .position = regs[CP_RB_RPTR / 4], .ring = true,
        };
        s->stream_count = 1;
    }
    s->streams[0].end = regs[CP_RB_WPTR / 4];
    for (unsigned budget = 0; budget < CP_CALLBACK_PACKET_BUDGET; budget++) {
        QcomProbeCommandStream *stream = &s->streams[s->stream_count - 1];
        uint32_t available = qcom_probe_stream_available(stream);
        uint32_t header, count, skip_words = 0;
        QcomProbeCommandResult result;
        g_autofree uint32_t *payload = NULL;

        if (!available) {
            uint32_t parent_words = stream->parent_packet_words;

            if (--s->stream_count == 0) {
                return;
            }
            qcom_probe_stream_advance(&s->streams[s->stream_count - 1], parent_words);
            if (!qcom_probe_publish_rptr(s)) {
                return;
            }
            continue;
        }
        if (!qcom_probe_gpu_read32(s, stream->base + stream->position * 4,
                                   &header)) {
            qcom_probe_command_stall(s, stream, 0, "unmapped command");
            return;
        }
        count = header >> 30 == 2 ? 0 : ((header >> 16) & 0x3fff) + 1;
        if (count + 1 > available) {
            qcom_probe_command_stall(s, stream, header, "incomplete packet");
            return;
        }
        payload = g_new(uint32_t, MAX(count, 1));
        for (unsigned i = 0; i < count; i++) {
            uint32_t index = qcom_probe_stream_index(stream, i + 1);

            if (!qcom_probe_gpu_read32(s, stream->base + index * 4, &payload[i])) {
                qcom_probe_command_stall(s, stream, header, "unmapped payload");
                return;
            }
        }
        trace_qcom_mdp4_probe_command(s->stream_count - 1,
            stream->base + stream->position * 4, header,
            count ? payload[0] : 0, count > 1 ? payload[1] : 0);
        result = qcom_probe_execute_packet(s, header, payload, count, &skip_words);
        if (result == QCOM_CP_PUSHED_IB) {
            continue;
        }
        if (result != QCOM_CP_DONE) {
            qcom_probe_command_stall(s, stream, header,
                result == QCOM_CP_WAIT ? "memory/register wait" :
                result == QCOM_CP_FAULT ? "address translation fault" :
                                         "unsupported packet");
            if (result == QCOM_CP_WAIT) {
                timer_mod(s->command_timer,
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CP_WAIT_RETRY_NS);
            }
            return;
        }
        if (skip_words > available - count - 1) {
            qcom_probe_command_stall(s, stream, header, "incomplete conditional span");
            return;
        }
        qcom_probe_stream_advance(stream, count + 1 + skip_words);
        if (!qcom_probe_publish_rptr(s)) {
            return;
        }
        s->stall_reason = NULL;
        trace_qcom_mdp4_probe_packet(header, regs[CP_RB_RPTR / 4],
                                    regs[CP_RB_WPTR / 4]);
    }
    timer_mod(s->command_timer,
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CP_DISPATCH_DELAY_NS);
}

static uint64_t qcom_mdp4_probe_read(void *opaque, hwaddr offset, unsigned size)
{
    QcomMdp4ProbeBank *bank = opaque;
    uint32_t value = bank->registers[offset / sizeof(uint32_t)];

    if (bank->repeated_reads && bank->last_read_offset == offset &&
        bank->last_read_value == value) {
        bank->repeated_reads++;
        if (is_power_of_2(bank->repeated_reads)) {
            trace_qcom_mdp4_probe_read_repeat(bank->index,
                qcom_mdp4_probe_banks[bank->index].base + offset,
                value, bank->repeated_reads);
        }
    } else {
        bank->last_read_offset = offset;
        bank->last_read_value = value;
        bank->repeated_reads = 1;
        trace_qcom_mdp4_probe_read(bank->index,
                                 qcom_mdp4_probe_banks[bank->index].base + offset,
                                 value, current_cpu ?
                                 current_cpu->cc->get_pc(current_cpu) : 0);
    }
    return value;
}

static void qcom_mdp4_probe_write(void *opaque, hwaddr offset, uint64_t value,
                                  unsigned size)
{
    QcomMdp4ProbeBank *bank = opaque;

    trace_qcom_mdp4_probe_write(bank->index,
                              qcom_mdp4_probe_banks[bank->index].base + offset,
                              value, current_cpu ?
                              current_cpu->cc->get_pc(current_cpu) : 0);
    bank->repeated_reads = 0;
    if (bank->index == MDP_BANK && offset == MDP_INTR_CLEAR) {
        bank->registers[MDP_INTR_STATUS / 4] &= ~value;
        qcom_probe_update_mdp_irq(bank->owner);
        return;
    }
    if (bank->index == MDP_BANK && offset == MDP_OVERLAY_FLUSH) {
        /* Separate RGB and mixer flush requests can be submitted before the
         * same frame. Writing a new mask must not cancel the earlier one. */
        bank->registers[offset / sizeof(uint32_t)] |= value;
    } else if (!((bank->index == 0 && offset == MDP4_VERSION_REGISTER) ||
          (bank->index == ADRENO_BANK &&
           (offset == ADRENO_PERIPHID1 || offset == ADRENO_PERIPHID2 ||
            offset == ADRENO_PATCH_RELEASE)))) {
        bank->registers[offset / sizeof(uint32_t)] = value;
    }
    if (bank->index == ADRENO_BANK &&
        (offset == CP_RB_WPTR || offset == CP_ME_CNTL)) {
        timer_mod(bank->owner->command_timer,
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CP_DISPATCH_DELAY_NS);
    }
    if (bank->index == ADRENO_BANK) {
        QcomMdp4ProbeState *s = bank->owner;

        if (offset == CP_INT_ACK) {
            bank->registers[CP_INT_STATUS / 4] &= ~value;
        }
        if (offset == CP_INT_ACK || offset == CP_INT_CNTL) {
            qcom_probe_update_gpu_irq(s);
        }
        if (offset == CP_RB_BASE || offset == CP_RB_CNTL ||
            (offset == CP_ME_CNTL && (value & CP_ME_HALT))) {
            s->stream_count = 0;
        }
    }
    if (bank->index == MDP_BANK) {
        QcomMdp4ProbeState *s = bank->owner;

        if (offset == MDP_INTR_ENABLE) {
            qcom_probe_update_mdp_irq(s);
        }
        if (offset == MDP_OVERLAY_FLUSH && (value & MDP_RGB1_FLUSH)) {
            memcpy(s->rgb1, &bank->registers[MDP_RGB1_BASE / 4], sizeof(s->rgb1));
            s->solid_color = bank->registers[MDP_RGB1_COLOR / 4];
            if (!s->console) {
                s->console = qemu_graphic_console_create(DEVICE(s), 0,
                                                          &qcom_probe_display_ops, s);
            }
        }
        if (offset == MDP_OVERLAY_FLUSH &&
            (value & (MDP_RGB1_FLUSH | MDP_OVERLAY0_FLUSH))) {
            s->overlay_pending = true;
        }
        if (offset == MDP_OVERLAY_FLUSH || offset == MDP_LCDC_ENABLE) {
            timer_mod(s->display_timer,
                       qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MDP_FRAME_NS);
        }
    }
}

static const MemoryRegionOps qcom_mdp4_probe_ops = {
    .read = qcom_mdp4_probe_read,
    .write = qcom_mdp4_probe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
};

static void qcom_mdp4_probe_reset(DeviceState *dev)
{
    QcomMdp4ProbeState *s = QCOM_MDP4_PROBE(dev);

    for (unsigned i = 0; i < QCOM_MDP4_PROBE_BANKS; i++) {
        memset(s->banks[i].registers, 0, qcom_mdp4_probe_banks[i].size);
        s->banks[i].repeated_reads = 0;
    }
    s->banks[0].registers[MDP4_VERSION_REGISTER] = MDP4_VERSION_4_0;
    /* The original miniport combines these into supported ID 0x02020005:
     * PERIPHID1[3:0], PERIPHID2[7:4], PATCH[7:0], PATCH[23:16]. This identifies
     * the initialization target only; it does not implement Adreno execution. */
    s->banks[ADRENO_BANK].registers[ADRENO_PERIPHID1 / 4] = 2;
    s->banks[ADRENO_BANK].registers[ADRENO_PERIPHID2 / 4] = 0x20;
    s->banks[ADRENO_BANK].registers[ADRENO_PATCH_RELEASE / 4] = 0x00050000;
    memset(s->me_init, 0, sizeof(s->me_init));
    s->me_initialized = false;
    s->protected_mode = false;
    s->draw_init_flags = 0;
    s->shader_bases = 0;
    memset(s->shader_words, 0, sizeof(s->shader_words));
    memset(s->shader_instructions, 0, sizeof(s->shader_instructions));
    s->stream_count = 0;
    s->stall_reason = NULL;
    s->draw_failure = NULL;
    s->draw_failure_detail = 0;
    memset(s->gmem, 0, sizeof(s->gmem));
    timer_del(s->command_timer);
    timer_del(s->display_timer);
    memset(s->rgb1, 0, sizeof(s->rgb1));
    s->overlay_pending = false;
    for (unsigned i = 0; i < QCOM_MDP4_PROBE_IRQS; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void qcom_mdp4_probe_init(Object *obj)
{
    QcomMdp4ProbeState *s = QCOM_MDP4_PROBE(obj);
    SysBusDevice *busdev = SYS_BUS_DEVICE(obj);

    s->command_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   qcom_probe_process_ring, s);
    s->display_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   qcom_probe_display_tick, s);

    for (unsigned i = 0; i < QCOM_MDP4_PROBE_BANKS; i++) {
        g_autofree char *name = g_strdup_printf("qcom-mdp4-probe.%u", i);
        QcomMdp4ProbeBank *bank = &s->banks[i];

        bank->index = i;
        bank->owner = s;
        bank->registers = g_malloc0(qcom_mdp4_probe_banks[i].size);
        memory_region_init_io(&bank->mmio, obj, &qcom_mdp4_probe_ops, bank,
                              name, qcom_mdp4_probe_banks[i].size);
        sysbus_init_mmio(busdev, &bank->mmio);
    }
    memory_region_init_ram(&s->vram, obj, "qcom-mdp4-probe.vram",
                           QCOM_MDP4_VRAM_SIZE, &error_abort);
    sysbus_init_mmio(busdev, &s->vram);
    for (unsigned i = 0; i < QCOM_MDP4_PROBE_IRQS; i++) {
        sysbus_init_irq(busdev, &s->irq[i]);
    }
}

static void qcom_mdp4_probe_finalize(Object *obj)
{
    QcomMdp4ProbeState *s = QCOM_MDP4_PROBE(obj);

    timer_free(s->command_timer);
    timer_free(s->display_timer);
    for (unsigned i = 0; i < QCOM_MDP4_PROBE_BANKS; i++) {
        g_free(s->banks[i].registers);
    }
}

static const VMStateDescription qcom_mdp4_probe_vmstate = {
    .name = TYPE_QCOM_MDP4_PROBE,
    .unmigratable = 1,
};

static const Property qcom_mdp4_probe_properties[] = {
    DEFINE_PROP_BOOL("upload-endian-workaround", QcomMdp4ProbeState,
                     upload_endian_workaround, false),
};

static void qcom_mdp4_probe_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, qcom_mdp4_probe_reset);
    device_class_set_props(dc, qcom_mdp4_probe_properties);
    dc->vmsd = &qcom_mdp4_probe_vmstate;
    dc->desc = "Experimental MSM8x60 graphics command interpreter and scanout";
    dc->user_creatable = false;
}

static const TypeInfo qcom_mdp4_probe_type = {
    .name = TYPE_QCOM_MDP4_PROBE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(QcomMdp4ProbeState),
    .instance_init = qcom_mdp4_probe_init,
    .instance_finalize = qcom_mdp4_probe_finalize,
    .class_init = qcom_mdp4_probe_class_init,
};

static void qcom_mdp4_probe_register(void)
{
    type_register_static(&qcom_mdp4_probe_type);
}

type_init(qcom_mdp4_probe_register)

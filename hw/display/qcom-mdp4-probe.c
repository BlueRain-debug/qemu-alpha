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
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/memory_cached.h"
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

/* Cache host RAM mappings for one DRAW_INDX only, including page-table RAM.
 * Descriptors themselves are always reloaded: guest page-table writes must
 * remain visible without inventing a new, persistent GPU TLB contract. */
#define QCOM_DRAW_PAGE_SIZE 4096
#define QCOM_DRAW_CACHE_PAGES 128
typedef struct QcomProbeRamPage {
    MemoryRegionCache mapping;
    hwaddr address;
    unsigned dirty_start;
    unsigned dirty_end;
} QcomProbeRamPage;

typedef struct QcomProbeDrawCache {
    QcomProbeRamPage pages[QCOM_DRAW_CACHE_PAGES];
} QcomProbeDrawCache;

struct QcomMdp4ProbeState {
    SysBusDevice parent_obj;
    QcomMdp4ProbeBank banks[QCOM_MDP4_PROBE_BANKS];
    MemoryRegion vram;
    qemu_irq irq[QCOM_MDP4_PROBE_IRQS];
    uint32_t me_init[18];
    bool me_initialized;
    bool upload_endian_workaround;
    bool draw_cache_enabled;
    bool fast_resolve;
    bool fast_fragments;
    bool damage_scanout;
    QcomProbeDrawCache *draw_cache;
    bool boot_lcdc;
    uint32_t boot_width;
    uint32_t boot_height;
    uint32_t gpu_chip_id;
    QEMUTimer *command_timer;
    QEMUTimer *display_timer;
    QemuConsole *console;
    uint32_t scanout[25];
    uint32_t scanout_flush;
    bool scanout_identity_csc;
    bool scanout_needs_full_update;
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

/* MDP4 primary LCD path: RGB1 or VIG1 -> mixer 0 -> DMA_P -> LCDC. */
#define MDP_BANK 0
#define MDP_SMMU_BANK 9
#define MDP_IRQ 0
#define MDP_INTR_ENABLE 0x50
#define MDP_INTR_STATUS 0x54
#define MDP_INTR_CLEAR 0x58
#define MDP_MIXER_CONFIG 0x10100
#define MDP_OVERLAY_FLUSH 0x18000
#define MDP_OVERLAY0_SIZE 0x10008
#define MDP_DMA_P_SIZE 0x90004
#define MDP_DMA_P_ADDRESS 0x90008
#define MDP_DMA_P_STRIDE 0x9000c
#define MDP_VIG1_BASE 0x20000
#define MDP_RGB1_BASE 0x40000
#define MDP_VIG1_COLOR 0x21008
#define MDP_RGB1_COLOR 0x41008
#define MDP_LCDC_ENABLE 0xc0000
#define MDP_RGB1_FLUSH (1U << 4)
#define MDP_VIG1_FLUSH (1U << 2)
#define MDP_OVERLAY0_FLUSH (1U << 0)
#define MDP_RGB1_MIXER0_BASE (1U << 8)
#define MDP_VIG1_MIXER0_BASE 1U
#define MDP_SCALE_X_ENABLE (1U << 0)
#define MDP_SCALE_Y_ENABLE (1U << 1)
#define MDP_CSC_ENABLE (1U << 11)
#define MDP_SCALE_PHASE_BITS 29
#define MDP_VIG1_CSC_MATRIX 0x24400
#define MDP_VIG1_CSC_PRE_BIAS 0x24500
#define MDP_VIG1_CSC_POST_BIAS 0x24580
#define MDP_VIG1_CSC_PRE_LIMIT 0x24600
#define MDP_VIG1_CSC_POST_LIMIT 0x24680
#define MDP_CSC_UNITY (1U << 9)
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

static void qcom_probe_ram_page_clear(QcomProbeRamPage *page)
{
    if (page->dirty_end > page->dirty_start) {
        /* Cached stores require explicit dirty/TB invalidation. Finish it
         * before dropping or replacing the mapping, and before a fence. */
        address_space_cache_invalidate(&page->mapping, page->dirty_start,
                                        page->dirty_end - page->dirty_start);
    }
    address_space_cache_destroy(&page->mapping);
    page->dirty_start = QCOM_DRAW_PAGE_SIZE;
    page->dirty_end = 0;
}

static void qcom_probe_draw_cache_clear(QcomProbeDrawCache *cache)
{
    for (unsigned index = 0; index < QCOM_DRAW_CACHE_PAGES; index++) {
        qcom_probe_ram_page_clear(&cache->pages[index]);
    }
}

static QcomProbeRamPage *qcom_probe_draw_page(QcomMdp4ProbeState *s,
                                             hwaddr address, bool write)
{
    hwaddr base = address & ~(hwaddr)(QCOM_DRAW_PAGE_SIZE - 1);
    QcomProbeRamPage *page;
    unsigned slot;
    int64_t length;

    if (!s->draw_cache) {
        return NULL;
    }
    /* Mix high address bits so a texture and render target at the same
     * offset in different GPU allocations do not always evict each other. */
    slot = ((base >> 12) ^ (base >> 20)) & (QCOM_DRAW_CACHE_PAGES - 1);
    page = &s->draw_cache->pages[slot];
    if (page->mapping.mrs.mr && page->address == base &&
        (!write || page->mapping.is_write)) {
        return page;
    }
    qcom_probe_ram_page_clear(page);
    length = address_space_cache_init(&page->mapping, &address_space_memory,
                                      base, QCOM_DRAW_PAGE_SIZE, write);
    if (length != QCOM_DRAW_PAGE_SIZE || !page->mapping.ptr ||
        !memory_region_is_ram(page->mapping.mrs.mr) ||
        (write && memory_region_is_rom(page->mapping.mrs.mr))) {
        /* Short boundary pages and nondirect mappings use the checked
         * original accessors. Never execute an MMIO bounce-buffer access. */
        qcom_probe_ram_page_clear(page);
        return NULL;
    }
    page->address = base;
    return page;
}

static bool qcom_probe_draw_ram32(QcomMdp4ProbeState *s, hwaddr address,
                                  uint32_t *value, bool write)
{
    QcomProbeRamPage *page = qcom_probe_draw_page(s, address, write);
    unsigned offset = address & (QCOM_DRAW_PAGE_SIZE - 1);

    if (!page) {
        return qcom_probe_ram32(address, value, write);
    }
    if (write) {
        address_space_stl_le_cached(&page->mapping, offset, *value,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
        page->dirty_start = MIN(page->dirty_start, offset);
        page->dirty_end = MAX(page->dirty_end, offset + sizeof(*value));
    } else {
        *value = address_space_ldl_le_cached(&page->mapping, offset,
                                             MEMTXATTRS_UNSPECIFIED, NULL);
    }
    return true;
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
    if (!qcom_probe_draw_ram32(s, table + (address >> 20) * 4,
                               &entry, false)) {
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
        if (!qcom_probe_draw_ram32(s, (entry & 0xfffffc00) +
                                    ((address >> 12) & 255) * 4,
                                    &entry, false)) {
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

static bool qcom_probe_identity_csc(const uint32_t *regs)
{
    for (unsigned i = 0; i < 9; i++) {
        if (regs[MDP_VIG1_CSC_MATRIX / 4 + i] !=
            (i % 4 == 0 ? MDP_CSC_UNITY : 0)) {
            return false;
        }
    }
    for (unsigned i = 0; i < 3; i++) {
        if (regs[MDP_VIG1_CSC_PRE_BIAS / 4 + i] ||
            regs[MDP_VIG1_CSC_POST_BIAS / 4 + i]) {
            return false;
        }
    }
    for (unsigned i = 0; i < 6; i++) {
        if (regs[MDP_VIG1_CSC_PRE_LIMIT / 4 + i] != (i & 1 ? 255 : 0) ||
            regs[MDP_VIG1_CSC_POST_LIMIT / 4 + i] != (i & 1 ? 255 : 0)) {
            return false;
        }
    }
    return true;
}

static bool qcom_probe_display_update(void *opaque)
{
    QcomMdp4ProbeState *s = opaque;
    uint32_t *regs = s->banks[MDP_BANK].registers;
    uint32_t format = s->scanout[0x50 / 4];
    uint32_t unpack = s->scanout[0x54 / 4];
    uint32_t scale = s->scanout[0x58 / 4];
    unsigned source_width = s->scanout[0] & 0xffff;
    unsigned source_height = s->scanout[0] >> 16;
    unsigned width = s->scanout[2] & 0xffff;
    unsigned height = s->scanout[2] >> 16;
    unsigned stride = s->scanout[0x40 / 4] & 0xffff;
    bool vig = s->scanout_flush == MDP_VIG1_FLUSH;
    bool solid = format & (1U << 22);
    unsigned component_shift[3] = {0};
    unsigned component_mask[3] = {0};
    unsigned first_changed = height, last_changed = 0;
    DisplaySurface *surface;
    g_autofree uint8_t *row = NULL;
    g_autofree uint32_t *converted = NULL;

    if (!(regs[MDP_LCDC_ENABLE / 4] & 1) ||
        (vig ? (regs[MDP_MIXER_CONFIG / 4] & 0xf) != MDP_VIG1_MIXER0_BASE :
         (regs[MDP_MIXER_CONFIG / 4] & 0xf00) != MDP_RGB1_MIXER0_BASE) ||
        !source_width || !source_height ||
        source_width > MDP_MAX_DIMENSION || source_height > MDP_MAX_DIMENSION ||
        !width || !height || width > MDP_MAX_DIMENSION ||
        height > MDP_MAX_DIMENSION || s->scanout[1] || s->scanout[3] ||
        (vig ? (scale & ~(MDP_SCALE_X_ENABLE | MDP_SCALE_Y_ENABLE |
                         MDP_CSC_ENABLE)) : scale != 0) ||
        ((scale & MDP_CSC_ENABLE) && !s->scanout_identity_csc) ||
        (!(scale & MDP_SCALE_X_ENABLE) && width != source_width) ||
        (!(scale & MDP_SCALE_Y_ENABLE) && height != source_height) ||
        ((scale & MDP_SCALE_X_ENABLE) && !s->scanout[0x5c / 4]) ||
        ((scale & MDP_SCALE_Y_ENABLE) && !s->scanout[0x60 / 4]) ||
        /* Only tightly packed, interleaved 8-bit RGBA is implemented. */
        (format & 0x7e0a67ff) != 0x000267ff ||
        (!solid && (stride < source_width * 4 ||
          (uint64_t)s->scanout[4] +
          (uint64_t)stride * source_height > UINT32_MAX))) {
        s->scanout_needs_full_update = true;
        return false;
    }
    if (!s->console) {
        s->scanout_needs_full_update = true;
        return false;
    }
    if (!solid) {
        /*
         * Decode the byte routing once per scanout, including the original
         * last-write-wins behavior for repeated component selectors.
         */
        for (unsigned byte = 0; byte < 4; byte++) {
            unsigned selector = (unpack >> (byte * 8)) & 0xff;

            if (selector > 3) {
                s->scanout_needs_full_update = true;
                return false;
            }
            if (selector < 3) {
                component_shift[selector] = byte * 8;
                component_mask[selector] = 255;
            }
        }
    }
    row = g_malloc(source_width * 4);
    converted = g_new(uint32_t, width);
    surface = qemu_console_surface(s->console);
    if (!surface || surface_width(surface) != width ||
        surface_height(surface) != height || surface_is_placeholder(surface)) {
        s->scanout_needs_full_update = true;
    }
    qemu_console_resize(s->console, width, height);
    surface = qemu_console_surface(s->console);
    for (unsigned y = 0; y < height; y++) {
        uint32_t *destination = (uint32_t *)(surface_data(surface) +
                                             y * surface_stride(surface));
        /* VIG phase steps are unsigned Q29 source-pixel increments. Use
         * bounded nearest sampling for this experimental linear RGBA path;
         * hardware polyphase filtering and nonidentity CSC are unsupported. */
        unsigned source_y = scale & MDP_SCALE_Y_ENABLE ?
            MIN((uint64_t)y * s->scanout[0x60 / 4] >> MDP_SCALE_PHASE_BITS,
                source_height - 1) : y;

        if (!solid && !qcom_probe_display_read(s,
                    s->scanout[4] + source_y * stride, row, source_width * 4)) {
            /*
             * Earlier rows may already have changed in the surface. Ensure
             * the next successful frame also notifies those partial writes.
             */
            s->scanout_needs_full_update = true;
            return false;
        }
        for (unsigned x = 0; x < width; x++) {
            uint32_t packed;
            unsigned source_x = scale & MDP_SCALE_X_ENABLE ?
                MIN((uint64_t)x * s->scanout[0x5c / 4] >> MDP_SCALE_PHASE_BITS,
                    source_width - 1) : x;

            if (solid) {
                converted[x] = s->solid_color & 0xffffff;
                continue;
            }
            packed = ldl_le_p(row + source_x * 4);

            /* MDP component numbers: 0=G, 1=B, 2=R, 3=A. */
            converted[x] = rgb_to_pixel32(
                (packed >> component_shift[2]) & component_mask[2],
                (packed >> component_shift[0]) & component_mask[0],
                (packed >> component_shift[1]) & component_mask[1]);
        }
        if (!s->damage_scanout || s->scanout_needs_full_update ||
            memcmp(destination, converted, width * sizeof(*converted))) {
            memcpy(destination, converted, width * sizeof(*converted));
            first_changed = MIN(first_changed, y);
            last_changed = y + 1;
        }
    }
    if (last_changed > first_changed) {
        qemu_console_update(s->console, 0, first_changed, width,
                              last_changed - first_changed);
    }
    s->scanout_needs_full_update = false;
    /*
     * An unchanged image still scanned out successfully. In particular the
     * display timer must keep completing FLUSH, VSYNC and RDPTR normally.
     */
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
        regs[MDP_OVERLAY_FLUSH / 4] &= ~(s->scanout_flush |
                                        MDP_OVERLAY0_FLUSH);
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
           qcom_probe_draw_ram32(s, physical, value, false);
}

static bool qcom_probe_gpu_write32(QcomMdp4ProbeState *s, uint32_t address,
                                   uint32_t value)
{
    hwaddr physical;

    return !(address & 3) && qcom_probe_gpu_address(s, address, &physical) &&
           qcom_probe_draw_ram32(s, physical, &value, true);
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
    if (qcom_probe_draw_page(opaque, physical, true)) {
        return true;
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

static bool qcom_probe_draw_writable_range(void *opaque, uint32_t address,
                                           size_t length)
{
    if ((address & 3) || (length & 3) ||
        (uint64_t)address + length > (1ULL << 32)) {
        return false;
    }
    while (length) {
        hwaddr physical;
        size_t chunk = MIN(length, QCOM_DRAW_PAGE_SIZE -
                                     (address & (QCOM_DRAW_PAGE_SIZE - 1)));

        if (!qcom_probe_gpu_address(opaque, address, &physical)) {
            return false;
        }
        if (!qcom_probe_draw_page(opaque, physical, true)) {
            /* Retain exact word-level boundary checks on short RAM pages. */
            for (size_t offset = 0; offset < chunk; offset += 4) {
                if (!qcom_probe_draw_writable(opaque, address + offset)) {
                    return false;
                }
            }
        }
        address += chunk;
        length -= chunk;
    }
    return true;
}

static bool qcom_probe_draw_write_bytes(void *opaque, uint32_t address,
                                        const uint8_t *data, size_t length)
{
    if ((address & 3) || (length & 3) ||
        (uint64_t)address + length > (1ULL << 32)) {
        return false;
    }
    while (length) {
        hwaddr physical;
        QcomProbeRamPage *page;
        unsigned offset = address & (QCOM_DRAW_PAGE_SIZE - 1);
        size_t chunk = MIN(length, QCOM_DRAW_PAGE_SIZE - offset);

        /* Rewalk descriptors for every committed page, including after
         * preflight: a guest can change its mappings before the commit. */
        if (!qcom_probe_gpu_address(opaque, address, &physical)) {
            return false;
        }
        page = qcom_probe_draw_page(opaque, physical, true);
        if (page) {
            address_space_write_cached(&page->mapping, offset, data, chunk);
            page->dirty_start = MIN(page->dirty_start, offset);
            page->dirty_end = MAX(page->dirty_end, offset + chunk);
        } else {
            for (size_t index = 0; index < chunk; index += 4) {
                if (!qcom_probe_gpu_write32(opaque, address + index,
                                             ldl_le_p(data + index))) {
                    return false;
                }
            }
        }
        address += chunk;
        data += chunk;
        length -= chunk;
    }
    return true;
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
        QcomProbeDrawCache cache = {0};
        QcomA2xxDrawContext context = {
            .registers = regs,
            .shader = {s->shader_instructions[0], s->shader_instructions[1]},
            .shader_words = {s->shader_words[0], s->shader_words[1]},
            .gmem = s->gmem,
            .gmem_bytes = sizeof(s->gmem),
            .upload_endian_workaround = s->upload_endian_workaround,
            .fast_resolve = s->fast_resolve,
            .fast_fragments = s->fast_fragments,
            .opaque = s,
            .read_word = qcom_probe_draw_read,
            .write_word = qcom_probe_draw_write,
            .writable_word = qcom_probe_draw_writable,
            .writable_range = qcom_probe_draw_writable_range,
            .write_bytes = qcom_probe_draw_write_bytes,
        };
        bool success;

        s->draw_cache = s->draw_cache_enabled ? &cache : NULL;
        /* Auto-indexed RECTLIST, three vertices, no visibility query.
         * Bit 14 enables the driver's pre-draw setup, bit 11 selects index
         * width (irrelevant for auto indexing). Other modes stay pending. */
        if (count == 3 && !payload[0] && payload[2] == 3 &&
            (payload[1] & ~0x4800U) == 0x88) {
            success = qcom_a2xx_draw_rect(&context);
        } else if (count == 3 && !payload[0] &&
                   (payload[1] & ~0x4800U) == 0x86) {
            /* DI_PT_TRISTRIP with AUTO_INDEX: the phone soft keyboard uses
             * this for its key popup. Index width has no effect in this mode. */
            success = qcom_a2xx_draw_triangle_strip(&context, payload[2]);
        } else if (count == 3 && !payload[0] &&
                   (payload[1] & ~0x4800U) == 0x84) {
            /* DI_PT_TRILIST with AUTO_INDEX, used by the message editor. */
            success = qcom_a2xx_draw_auto_triangles(&context, payload[2]);
        } else if (count == 5 && !payload[0] &&
                   (payload[1] & ~0x4000U) == 4) {
            success = qcom_a2xx_draw_triangles(&context, payload[3],
                                                payload[2], payload[4]);
        } else {
            s->draw_cache = NULL;
            qcom_probe_draw_cache_clear(&cache);
            return QCOM_CP_UNSUPPORTED;
        }
        s->draw_cache = NULL;
        qcom_probe_draw_cache_clear(&cache);
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
        if (offset == MDP_OVERLAY_FLUSH &&
            (value & (MDP_RGB1_FLUSH | MDP_VIG1_FLUSH))) {
            bool vig = (value & MDP_VIG1_FLUSH) &&
                (bank->registers[MDP_MIXER_CONFIG / 4] & 0xf) ==
                    MDP_VIG1_MIXER0_BASE;
            unsigned base = vig ? MDP_VIG1_BASE : MDP_RGB1_BASE;
            unsigned color = vig ? MDP_VIG1_COLOR : MDP_RGB1_COLOR;

            s->scanout_flush = vig ? MDP_VIG1_FLUSH : MDP_RGB1_FLUSH;
            memcpy(s->scanout, &bank->registers[base / 4], sizeof(s->scanout));
            s->scanout_identity_csc = vig &&
                                     qcom_probe_identity_csc(bank->registers);
            s->solid_color = bank->registers[color / 4];
            if (!s->console) {
                s->console = qemu_graphic_console_create(DEVICE(s), 0,
                                                          &qcom_probe_display_ops, s);
            }
        }
        if (offset == MDP_OVERLAY_FLUSH &&
            (value & (MDP_RGB1_FLUSH | MDP_VIG1_FLUSH | MDP_OVERLAY0_FLUSH))) {
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
    /* Miniports assemble core.major.minor.patch from PERIPHID1[3:0],
     * PERIPHID2[7:4], PATCH[7:0], PATCH[23:16]. Earlier miniports recognize
     * 2.1.0.0 but reject the default 2.2.0.5 before starting the GPU. This
     * property selects identification registers, not a complete GPU revision. */
    s->banks[ADRENO_BANK].registers[ADRENO_PERIPHID1 / 4] =
        (s->gpu_chip_id >> 24) & 0xf;
    s->banks[ADRENO_BANK].registers[ADRENO_PERIPHID2 / 4] =
        ((s->gpu_chip_id >> 16) & 0xf) << 4;
    s->banks[ADRENO_BANK].registers[ADRENO_PATCH_RELEASE / 4] =
        ((s->gpu_chip_id >> 8) & 0xff) | ((s->gpu_chip_id & 0xff) << 16);
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
    memset(s->scanout, 0, sizeof(s->scanout));
    s->scanout_flush = 0;
    s->scanout_identity_csc = false;
    s->scanout_needs_full_update = true;
    /* Some miniports preserve an output enabled by the platform firmware.
     * Keep this opt-in: a cold, disabled controller is still the default. */
    if (s->boot_lcdc) {
        uint32_t *regs = s->banks[MDP_BANK].registers;
        uint32_t size = s->boot_width | (s->boot_height << 16);

        regs[MDP_LCDC_ENABLE / 4] = 1;
        regs[MDP_OVERLAY0_SIZE / 4] = size;
        regs[MDP_DMA_P_SIZE / 4] = size;
        regs[MDP_DMA_P_ADDRESS / 4] = QCOM_MDP4_VIDEO_BASE;
        regs[MDP_DMA_P_STRIDE / 4] = s->boot_width * 4;
    }
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
    DEFINE_PROP_BOOL("draw-cache", QcomMdp4ProbeState, draw_cache_enabled, true),
    DEFINE_PROP_BOOL("fast-resolve", QcomMdp4ProbeState, fast_resolve, true),
    DEFINE_PROP_BOOL("fast-fragments", QcomMdp4ProbeState,
                     fast_fragments, true),
    DEFINE_PROP_BOOL("damage-scanout", QcomMdp4ProbeState,
                     damage_scanout, true),
    DEFINE_PROP_BOOL("boot-lcdc", QcomMdp4ProbeState, boot_lcdc, false),
    DEFINE_PROP_UINT32("boot-width", QcomMdp4ProbeState, boot_width, 480),
    DEFINE_PROP_UINT32("boot-height", QcomMdp4ProbeState, boot_height, 800),
    DEFINE_PROP_UINT32("gpu-chip-id", QcomMdp4ProbeState,
                       gpu_chip_id, 0x02020005),
    DEFINE_PROP_BOOL("upload-endian-workaround", QcomMdp4ProbeState,
                     upload_endian_workaround, false),
};

static void qcom_mdp4_probe_realize(DeviceState *dev, Error **errp)
{
    QcomMdp4ProbeState *s = QCOM_MDP4_PROBE(dev);

    if ((s->gpu_chip_id >> 24) != 2 ||
        ((s->gpu_chip_id >> 16) & 0xff) > 0xf) {
        error_setg(errp, "GPU chip ID must identify an A2xx core with a "
                   "4-bit major revision");
        return;
    }
    if (s->boot_lcdc && (!s->boot_width || !s->boot_height ||
        s->boot_width > MDP_MAX_DIMENSION ||
        s->boot_height > MDP_MAX_DIMENSION)) {
        error_setg(errp, "boot LCDC dimensions must be between 1 and %u",
                   MDP_MAX_DIMENSION);
    }
}

static void qcom_mdp4_probe_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = qcom_mdp4_probe_realize;
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

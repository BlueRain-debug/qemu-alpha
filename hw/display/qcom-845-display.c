/*
 * Experimental SDM845 Adreno 630 / GMU / DPU 4.0 register and display model
 * for local OEMSK850 bring-up.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Strictly independent from MSM8998 (qcom-8998-display.c).
 */
#include "qemu/osdep.h"
#include "hw/display/qcom-845-display.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "trace.h"

OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Display, QCOM845_DISPLAY)

/* Native SDM845 GPU0 MMIO windows from 17047 ACPI _CRS (excluding GPU_CC). */
#define QCOM845_MDSS_BASE        0x0ae00000
#define QCOM845_MDSS_SIZE        0x000d0000
#define QCOM845_DP_PHY_BASE      0x088e0000
#define QCOM845_DP_PHY_SIZE      0x000f4000
#define QCOM845_ADRENO_BASE      0x05000000
#define QCOM845_ADRENO_SIZE      0x00040000
#define QCOM845_GMU_CX_BASE      0x05060000
#define QCOM845_GMU_CX_SIZE      0x00030000
#define QCOM845_CX_MISC_BASE     0x05099000
#define QCOM845_CX_MISC_SIZE     0x00006000
#define QCOM845_PDC_SEQ_BASE     0x0b280000
#define QCOM845_PDC_SEQ_SIZE     0x00010000
#define QCOM845_PDC_REGS_BASE    0x0b480000
#define QCOM845_PDC_REGS_SIZE    0x00010000
#define QCOM845_VENUS_BASE       0x0aa00000
#define QCOM845_VENUS_SIZE       0x00100000 /* VIDEO_CC owns 0x0ab00000. */

/* Native GPU0 INTIDs (GIC SPI = INTID - 32). */
#define QCOM845_IRQ_VSYNC        0x73   /* 115 -> SPI 83 */
#define QCOM845_IRQ_GFX          0x14c  /* 332 -> SPI 300 */
#define QCOM845_IRQ_VIDC         0xce   /* 206 -> SPI 174 */
#define QCOM845_IRQ_VIDC_WD      0xcf   /* 207 -> SPI 175 */

/* DPU 4.0 (SDM845) hardware version and core control/IRQ registers. */
#define DPU4_MDSS_HW_VERSION     0x0000
#define DPU4_MDSS_HW_INTR_STATUS 0x0010
#define DPU4_MDSS_MDP_INTR       BIT(0)
#define DPU4_MDSS_DSI0_INTR      BIT(4)
#define DPU4_MDSS_DSI1_INTR      BIT(5)
#define DPU4_MDP_HW_VERSION      0x1000
#define DPU4_MDP_INTR_EN         0x1010
#define DPU4_MDP_INTR_STATUS     0x1014
#define DPU4_MDP_INTR_CLEAR      0x1018
#define DPU4_MDP_PP_DONE_IRQ(i)  BIT(8 + (i))
#define DPU4_MDP_PP_RD_PTR(i)    BIT(12 + (i))
#define DPU4_CTL_BASE            0x2000
#define DPU4_CTL_STRIDE          0x200
#define DPU4_CTL_COUNT           5
#define DPU4_CTL_FLUSH           0x18
#define DPU4_CTL_START           0x1c
#define DPU4_CTL_SW_RESET        0x30
#define DPU4_CTL_LAYER_EXT       0x40
#define DPU4_CTL_LAYER_EXT2      0x70
#define DPU4_DSI0_BASE           0x94000
#define DPU4_DSI1_BASE           0x96000
#define DPU4_DSI_SIZE            0x400
#define DPU4_DSI_DMA_SW_TRIGGER  0x090
#define DPU4_DSI_INT_CTRL        0x110
#define DPU4_DSI_SOFT_RESET      0x118
#define DPU4_DSI_INT_CMD_DONE    BIT(0)
#define DPU4_HW_VERSION_401      0x40000001U
#define DPU4_MDSS_SMMU_SID       0x880
#define DPU4_PANEL_WIDTH         1350
#define DPU4_PANEL_HEIGHT        1800
#define DPU4_VSYNC_NS            (NANOSECONDS_PER_SECOND / 60)

/* Adreno 630 (A6x) core register byte offsets (dword_reg * 4). */
#define A6XX_RBBM_INT_CLEAR_ALT  (0x01f * 4)
#define A6XX_RBBM_INT_CLEAR_CMD  (0x037 * 4)
#define A6XX_RBBM_INT_0_MASK     (0x038 * 4)
#define A6XX_RBBM_SW_RESET_CMD   (0x043 * 4)
#define A6XX_RBBM_INT_0_STATUS   (0x201 * 4)
#define A6XX_RBBM_STATUS         (0x210 * 4)
#define A6XX_CP_RB_BASE          (0x800 * 4)
#define A6XX_CP_RB_BASE_HI       (0x801 * 4)
#define A6XX_CP_RB_CNTL          (0x802 * 4)
#define A6XX_CP_RB_RPTR_ADDR     (0x804 * 4)
#define A6XX_CP_RB_RPTR_ADDR_HI  (0x805 * 4)
#define A6XX_CP_RB_RPTR          (0x806 * 4)
#define A6XX_CP_RB_WPTR          (0x807 * 4)
#define A6XX_CP_SQE_CNTL         (0x808 * 4)
#define A6XX_INT_CP_RB_SW        BIT(13)
#define A6XX_INT_CP_SW           BIT(14)
#define A6XX_INT_CACHE_FLUSH_TS  BIT(15)

/* GPU RSC TCS0 status & AO GFX IRQ status offsets within GMU_CX (0x05060000). */
#define A6XX_RSCC_TCS0_DRV0_STATUS 0x2dd18
#define A6XX_RSCC_TCS_IDLE         BIT(0)
#define A6XX_GMU_AO_GFX_INT_STATUS 0x2ec54

/* Venus 4.4 (SDM845) wrapper version register. */
#define VENUS4_WRAPPER_HW_VERSION 0xe0000
#define VENUS4_HW_VERSION_44      0x40440000U

enum Qcom845BankId {
    BANK845_MDSS = 0,
    BANK845_DP_PHY,
    BANK845_ADRENO,
    BANK845_GMU_CX,
    BANK845_CX_MISC,
    BANK845_PDC_SEQ,
    BANK845_PDC_REGS,
    BANK845_VENUS,
    BANK845_COUNT,
};

typedef struct Qcom845BankDesc {
    const char *name;
    hwaddr base;
    hwaddr size;
} Qcom845BankDesc;

static const Qcom845BankDesc qcom845_bank_descs[BANK845_COUNT] = {
    [BANK845_MDSS]     = { "mdss",     QCOM845_MDSS_BASE,     QCOM845_MDSS_SIZE },
    [BANK845_DP_PHY]   = { "dp-phy",   QCOM845_DP_PHY_BASE,   QCOM845_DP_PHY_SIZE },
    [BANK845_ADRENO]   = { "adreno",   QCOM845_ADRENO_BASE,   QCOM845_ADRENO_SIZE },
    [BANK845_GMU_CX]   = { "gmu-cx",   QCOM845_GMU_CX_BASE,   QCOM845_GMU_CX_SIZE },
    [BANK845_CX_MISC]  = { "cx-misc",  QCOM845_CX_MISC_BASE,  QCOM845_CX_MISC_SIZE },
    [BANK845_PDC_SEQ]  = { "pdc-seq",  QCOM845_PDC_SEQ_BASE,  QCOM845_PDC_SEQ_SIZE },
    [BANK845_PDC_REGS] = { "pdc-regs", QCOM845_PDC_REGS_BASE, QCOM845_PDC_REGS_SIZE },
    [BANK845_VENUS]    = { "venus",    QCOM845_VENUS_BASE,    QCOM845_VENUS_SIZE },
};

typedef struct Qcom845DisplayBank {
    Qcom845Display *parent;
    enum Qcom845BankId id;
    MemoryRegion mmio;
    uint32_t *regs;
    uint32_t reg_count;
} Qcom845DisplayBank;

struct Qcom845Display {
    SysBusDevice parent_obj;
    Qcom845Smmu *apps_smmu;
    Qcom845Smmu *gpu_smmu;
    Qcom845DisplayBank banks[BANK845_COUNT];
    qemu_irq vsync_irq;
    qemu_irq gfx_irq;
    qemu_irq vidc_irq;
    qemu_irq vidc_wd_irq;
    QEMUTimer *vsync_timer;
    QemuConsole *con[3]; /* 0: head 0, 1: head 1, 2: dual 2700x1800 */
    uint32_t *panel_fb[2];
    bool panel_dirty[2];
    bool sspp_dirty;
};

static void qcom845_update_vsync_irq(Qcom845Display *s);
static bool qcom845_gfx_update(void *opaque);

static bool qcom845_mdss_read_bytes(Qcom845Display *s, uint64_t iova,
                                    void *buf, size_t len, hwaddr *first_pa)
{
    uint8_t *dst = buf;
    size_t done = 0;

    while (done < len) {
        uint64_t cur = iova + done;
        hwaddr pa = cur;
        size_t chunk = MIN(len - done, 4096 - (cur & 4095));

        if (s->apps_smmu &&
            !qcom845_smmu_translate(s->apps_smmu, DPU4_MDSS_SMMU_SID, cur,
                                    false, &pa)) {
            return false;
        }
        if (done == 0 && first_pa) {
            *first_pa = pa;
        }
        if (address_space_read(&address_space_memory, pa,
                               MEMTXATTRS_UNSPECIFIED, dst + done,
                               chunk) != MEMTX_OK) {
            return false;
        }
        done += chunk;
    }
    return true;
}

static bool qcom845_ctl_uses_pipe(Qcom845DisplayBank *mdss, unsigned ctl,
                                  unsigned pipe_idx)
{
    /*
     * Pipe table order: 0..3 = VIG0..3, 4..7 = DMA0..3.
     * Check LM0..LM3 staging slots for this CTL.
     */
    hwaddr base = DPU4_CTL_BASE + ctl * DPU4_CTL_STRIDE;

    for (unsigned lm = 0; lm < 4; lm++) {
        uint32_t layer = mdss->regs[(base + lm * 4) / 4];
        uint32_t ext2 = mdss->regs[(base + DPU4_CTL_LAYER_EXT2 + lm * 4) / 4];

        switch (pipe_idx) {
        case 0: if (layer & 0x7) { return true; } break;
        case 1: if ((layer >> 3) & 0x7) { return true; } break;
        case 2: if ((layer >> 6) & 0x7) { return true; } break;
        case 3: if ((layer >> 26) & 0x7) { return true; } break;
        case 4: if ((layer >> 18) & 0x7) { return true; } break;
        case 5: if ((layer >> 21) & 0x7) { return true; } break;
        case 6: if (ext2 & 0x7) { return true; } break;
        case 7: if ((ext2 >> 4) & 0x7) { return true; } break;
        }
    }
    return false;
}

static void qcom845_scanout_head(Qcom845Display *s, unsigned head)
{
    static const hwaddr sspp_bases[8] = {
        0x5000, 0x7000, 0x9000, 0xb000,
        0x25000, 0x27000, 0x29000, 0x2b000,
    };
    Qcom845DisplayBank *mdss = &s->banks[BANK845_MDSS];
    uint32_t *fb;
    uint32_t row_buf[DPU4_PANEL_WIDTH];
    bool any_drawn = false;
    uint32_t nonblack = 0;
    uint64_t last_iova = 0;
    hwaddr last_pa = 0;
    uint32_t last_w = 0, last_h = 0, last_stride = 0;

    if (head >= 2 || !s->panel_fb[head]) {
        return;
    }
    fb = s->panel_fb[head];

    for (unsigned p = 0; p < ARRAY_SIZE(sspp_bases); p++) {
        hwaddr b = sspp_bases[p];
        uint32_t addr = mdss->regs[(b + 0x14) / 4];
        uint32_t src_size = mdss->regs[(b + 0x00) / 4];
        uint32_t src_xy = mdss->regs[(b + 0x08) / 4];
        uint32_t out_size = mdss->regs[(b + 0x0c) / 4];
        uint32_t out_xy = mdss->regs[(b + 0x10) / 4];
        uint32_t stride = mdss->regs[(b + 0x24) / 4] & 0xffff;
        uint32_t unpack = mdss->regs[(b + 0x34) / 4];
        bool swap_rb = ((unpack & 0xff) == 2); /* C0=R instead of C0=B */
        uint32_t sw = src_size & 0xffff;
        uint32_t sh = src_size >> 16;
        uint32_t ow = out_size & 0xffff;
        uint32_t oh = out_size >> 16;
        uint32_t sx = src_xy & 0xffff;
        uint32_t sy = src_xy >> 16;
        uint32_t ox = out_xy & 0xffff;
        uint32_t oy = out_xy >> 16;

        if (!addr || !sw || !sh || !ow || !oh || stride < sw * 4) {
            continue;
        }
        /* Match pipe to head via CTL layer staging, or default split assignment. */
        bool on_this = qcom845_ctl_uses_pipe(mdss, head, p);
        bool on_other = qcom845_ctl_uses_pipe(mdss, head ^ 1, p);
        if (!on_this) {
            if (on_other) {
                continue;
            }
            /* Fallback before CTL_LAYER is populated: DMA0/1 -> head 0, DMA2/3 -> head 1. */
            if (head == 0 && !(p == 0 || p == 1 || p == 4 || p == 5)) {
                continue;
            }
            if (head == 1 && !(p == 2 || p == 3 || p == 6 || p == 7)) {
                continue;
            }
        }

        uint32_t w = MIN(sw, ow);
        uint32_t h = MIN(sh, oh);
        if (ox >= DPU4_PANEL_WIDTH || oy >= DPU4_PANEL_HEIGHT) {
            continue;
        }
        w = MIN(w, DPU4_PANEL_WIDTH - ox);
        h = MIN(h, DPU4_PANEL_HEIGHT - oy);

        hwaddr pa0 = 0;
        for (uint32_t y = 0; y < h; y++) {
            uint64_t row_iova = (uint64_t)addr + (uint64_t)(sy + y) * stride +
                                (uint64_t)sx * 4;
            if (!qcom845_mdss_read_bytes(s, row_iova, row_buf, w * 4,
                                         y == 0 ? &pa0 : NULL)) {
                break;
            }
            uint32_t *dst = fb + (oy + y) * DPU4_PANEL_WIDTH + ox;
            for (uint32_t x = 0; x < w; x++) {
                uint32_t px = le32_to_cpu(row_buf[x]);
                if (swap_rb) {
                    px = (px & 0xff00ff00U) | ((px & 0xff) << 16) |
                         ((px >> 16) & 0xff);
                }
                px |= 0xff000000U;
                dst[x] = px;
                if (px != 0xff000000U) {
                    nonblack++;
                }
            }
        }
        any_drawn = true;
        last_iova = addr;
        last_pa = pa0;
        last_w = w;
        last_h = h;
        last_stride = stride;
    }

    if (any_drawn) {
        s->panel_dirty[head] = true;
        trace_qcom845_scanout(head, last_iova, last_pa, last_w, last_h,
                              last_stride, nonblack);
        qcom845_gfx_update(s);
    }
}

static void qcom845_vsync_tick(void *opaque)
{
    Qcom845Display *s = opaque;
    Qcom845DisplayBank *mdss = &s->banks[BANK845_MDSS];
    uint32_t en = mdss->regs[DPU4_MDP_INTR_EN / 4];
    uint32_t rd_ptr = en & (DPU4_MDP_PP_RD_PTR(0) | DPU4_MDP_PP_RD_PTR(1));

    if (!rd_ptr) {
        return;
    }
    if (s->sspp_dirty) {
        s->sspp_dirty = false;
        qcom845_scanout_head(s, 0);
        qcom845_scanout_head(s, 1);
    }
    mdss->regs[DPU4_MDP_INTR_STATUS / 4] |= rd_ptr;
    qcom845_update_vsync_irq(s);
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DPU4_VSYNC_NS);
}

static void qcom845_update_vsync_timer(Qcom845Display *s)
{
    Qcom845DisplayBank *mdss = &s->banks[BANK845_MDSS];
    uint32_t en = mdss->regs[DPU4_MDP_INTR_EN / 4];

    if (en & (DPU4_MDP_PP_RD_PTR(0) | DPU4_MDP_PP_RD_PTR(1))) {
        if (!timer_pending(s->vsync_timer)) {
            timer_mod(s->vsync_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DPU4_VSYNC_NS);
        }
    } else {
        timer_del(s->vsync_timer);
    }
}

static bool qcom845_gfx_update(void *opaque)
{
    Qcom845Display *s = opaque;
    bool updated = false;

    for (unsigned head = 0; head < 2; head++) {
        if (!s->con[head] || !s->panel_dirty[head]) {
            continue;
        }
        DisplaySurface *surf = qemu_console_surface(s->con[head]);
        if (surf && surface_width(surf) == DPU4_PANEL_WIDTH &&
            surface_height(surf) == DPU4_PANEL_HEIGHT) {
            uint8_t *dst = surface_data(surf);
            int dst_stride = surface_stride(surf);
            for (unsigned y = 0; y < DPU4_PANEL_HEIGHT; y++) {
                memcpy(dst + y * dst_stride,
                       s->panel_fb[head] + y * DPU4_PANEL_WIDTH,
                       DPU4_PANEL_WIDTH * 4);
            }
            qemu_console_update(s->con[head], 0, 0, DPU4_PANEL_WIDTH,
                                DPU4_PANEL_HEIGHT);
            updated = true;
        }
    }
    if (s->con[2] && (s->panel_dirty[0] || s->panel_dirty[1])) {
        DisplaySurface *surf = qemu_console_surface(s->con[2]);
        if (surf && surface_width(surf) == DPU4_PANEL_WIDTH * 2 &&
            surface_height(surf) == DPU4_PANEL_HEIGHT) {
            uint8_t *dst = surface_data(surf);
            int dst_stride = surface_stride(surf);
            for (unsigned y = 0; y < DPU4_PANEL_HEIGHT; y++) {
                uint32_t *line = (uint32_t *)(dst + y * dst_stride);
                memcpy(line, s->panel_fb[0] + y * DPU4_PANEL_WIDTH,
                       DPU4_PANEL_WIDTH * 4);
                memcpy(line + DPU4_PANEL_WIDTH,
                       s->panel_fb[1] + y * DPU4_PANEL_WIDTH,
                       DPU4_PANEL_WIDTH * 4);
            }
            qemu_console_update(s->con[2], 0, 0, DPU4_PANEL_WIDTH * 2,
                                DPU4_PANEL_HEIGHT);
            updated = true;
        }
    }
    s->panel_dirty[0] = false;
    s->panel_dirty[1] = false;
    return updated;
}

static void qcom845_gfx_invalidate(void *opaque)
{
    Qcom845Display *s = opaque;

    s->panel_dirty[0] = true;
    s->panel_dirty[1] = true;
}

static const GraphicHwOps qcom845_gfx_ops = {
    .gfx_update = qcom845_gfx_update,
    .invalidate = qcom845_gfx_invalidate,
};

static void qcom845_update_vsync_irq(Qcom845Display *s)
{
    Qcom845DisplayBank *mdss = &s->banks[BANK845_MDSS];
    uint32_t mdp_en = mdss->regs[DPU4_MDP_INTR_EN / 4];
    uint32_t mdp_st = mdss->regs[DPU4_MDP_INTR_STATUS / 4];
    uint32_t dsi0_ic = mdss->regs[(DPU4_DSI0_BASE + DPU4_DSI_INT_CTRL) / 4];
    uint32_t dsi1_ic = mdss->regs[(DPU4_DSI1_BASE + DPU4_DSI_INT_CTRL) / 4];
    uint32_t hw_st = 0;

    if (mdp_en & mdp_st) {
        hw_st |= DPU4_MDSS_MDP_INTR;
    }
    if ((dsi0_ic & (dsi0_ic >> 1)) & 0x55555555U) {
        hw_st |= DPU4_MDSS_DSI0_INTR;
    }
    if ((dsi1_ic & (dsi1_ic >> 1)) & 0x55555555U) {
        hw_st |= DPU4_MDSS_DSI1_INTR;
    }
    mdss->regs[DPU4_MDSS_HW_INTR_STATUS / 4] = hw_st;
    qemu_set_irq(s->vsync_irq, hw_st != 0);
}

static void qcom845_update_gfx_irq(Qcom845Display *s)
{
    Qcom845DisplayBank *adreno = &s->banks[BANK845_ADRENO];
    uint32_t status = adreno->regs[A6XX_RBBM_INT_0_STATUS / 4];
    uint32_t mask = adreno->regs[A6XX_RBBM_INT_0_MASK / 4];

    qemu_set_irq(s->gfx_irq, (status & mask) != 0);
}

static bool qcom845_gpu_read32(Qcom845Display *s, uint64_t iova, uint32_t *out)
{
    hwaddr pa = iova;
    uint32_t val = 0;

    if (s->gpu_smmu &&
        !qcom845_smmu_translate(s->gpu_smmu, 0, iova, false, &pa)) {
        return false;
    }
    if (address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                           &val, sizeof(val)) != MEMTX_OK) {
        return false;
    }
    *out = le32_to_cpu(val);
    return true;
}

static bool qcom845_gpu_write32(Qcom845Display *s, uint64_t iova, uint32_t val)
{
    hwaddr pa = iova;
    uint32_t le = cpu_to_le32(val);

    if (s->gpu_smmu &&
        !qcom845_smmu_translate(s->gpu_smmu, 0, iova, true, &pa)) {
        return false;
    }
    return address_space_write(&address_space_memory, pa,
                               MEMTXATTRS_UNSPECIFIED, &le,
                               sizeof(le)) == MEMTX_OK;
}

static void qcom845_cp_exec_stream(Qcom845Display *s, uint64_t base,
                                   uint32_t start, uint32_t count,
                                   uint32_t ring_mask, unsigned depth)
{
    Qcom845DisplayBank *adreno = &s->banks[BANK845_ADRENO];
    uint32_t pos = 0;

    if (depth > 4 || count > 65536) {
        return;
    }

    while (pos < count) {
        uint32_t hdr = 0;
        uint32_t idx = (start + pos) & ring_mask;

        if (!qcom845_gpu_read32(s, base + (uint64_t)idx * 4, &hdr)) {
            break;
        }
        pos++;

        uint32_t type = hdr >> 28;
        if (type == 4) {
            uint32_t reg = (hdr >> 8) & 0x3ffff;
            uint32_t n = hdr & 0x7f;

            for (uint32_t i = 0; i < n && pos < count; i++, pos++) {
                uint32_t v = 0;
                uint32_t widx = (start + pos) & ring_mask;

                if (!qcom845_gpu_read32(s, base + (uint64_t)widx * 4, &v)) {
                    break;
                }
                if (reg + i < adreno->reg_count) {
                    adreno->regs[reg + i] = v;
                } else if (reg + i >= 0x10000 && reg + i < 0x14000 &&
                           s->gpu_smmu) {
                    /* GPU_SMMU (0x05040000..0x0504ffff) in Adreno dword space. */
                    qcom845_smmu_write32(s->gpu_smmu,
                                         (hwaddr)(reg + i - 0x10000) * 4, v);
                }
            }
        } else if (type == 7) {
            uint32_t op = (hdr >> 16) & 0x7f;
            uint32_t n = hdr & 0x7fff;
            uint32_t w[8] = { 0 };

            for (uint32_t i = 0; i < n && i < ARRAY_SIZE(w) &&
                                 pos + i < count; i++) {
                uint32_t widx = (start + pos + i) & ring_mask;
                qcom845_gpu_read32(s, base + (uint64_t)widx * 4, &w[i]);
            }
            trace_qcom845_cp_pkt(op, n, w[0], w[1]);

            if (op == 0x3d && n >= 3) {
                /* CP_MEM_WRITE */
                uint64_t dst = w[0] | ((uint64_t)w[1] << 32);

                for (uint32_t i = 2; i < n && pos + i < count; i++) {
                    uint32_t v = 0;
                    uint32_t widx = (start + pos + i) & ring_mask;

                    if (qcom845_gpu_read32(s, base + (uint64_t)widx * 4, &v)) {
                        qcom845_gpu_write32(s, dst + (uint64_t)(i - 2) * 4, v);
                    }
                }
            } else if (op == 0x3e && n >= 3) {
                /* CP_REG_TO_MEM */
                uint32_t reg = w[0] & 0x3ffff;
                uint32_t cnt = (w[0] >> 18) & 0xfff;
                uint64_t dst = w[1] | ((uint64_t)w[2] << 32);

                if (cnt == 0) {
                    cnt = 1;
                }
                for (uint32_t i = 0; i < cnt; i++) {
                    uint32_t rv = (reg + i < adreno->reg_count)
                                      ? adreno->regs[reg + i]
                                      : 0;
                    qcom845_gpu_write32(s, dst + (uint64_t)i * 4, rv);
                }
            } else if (op == 0x21 && n >= 3) {
                /* CP_REG_RMW */
                uint32_t reg = w[0] & 0x3ffff;
                if (reg < adreno->reg_count) {
                    adreno->regs[reg] = (adreno->regs[reg] & w[1]) | w[2];
                }
            } else if (op == 0x3f && n >= 3) {
                /* CP_INDIRECT_BUFFER */
                uint64_t ib = w[0] | ((uint64_t)w[1] << 32);
                uint32_t ib_dw = w[2] & 0xfffff;

                if (ib_dw > 0) {
                    qcom845_cp_exec_stream(s, ib, 0, ib_dw, UINT32_MAX,
                                           depth + 1);
                }
            } else if (op == 0x40 && n >= 1) {
                /*
                 * CP_INTERRUPT: bits [31:29] map to RBBM_INT_0_STATUS [15:13]
                 * (0x20000000 -> BIT(13) used by qcdxkm845 preempt/sync path).
                 */
                uint32_t mask = (w[0] >> 16) &
                                (A6XX_INT_CP_RB_SW | A6XX_INT_CP_SW |
                                 A6XX_INT_CACHE_FLUSH_TS);
                adreno->regs[A6XX_RBBM_INT_0_STATUS / 4] |=
                    mask ? mask : A6XX_INT_CP_SW;
                qcom845_update_gfx_irq(s);
            } else if (op == 0x44 && n >= 6) {
                /* CP_COND_EXEC: skip w[5] dwords if !enabled or ref < ts. */
                uint64_t bool_addr = w[0] | ((uint64_t)w[1] << 32);
                uint64_t ts_addr = w[2] | ((uint64_t)w[3] << 32);
                uint32_t enabled = 0, ts = 0;

                if (qcom845_gpu_read32(s, bool_addr, &enabled) &&
                    qcom845_gpu_read32(s, ts_addr, &ts)) {
                    if (!enabled || (uint32_t)(w[4] - ts) >= BIT(31)) {
                        pos += w[5];
                    }
                }
            } else if (op == 0x53 && n >= 2) {
                /* CP_SMMU_TABLE_UPDATE: update GPU_SMMU CB0 TTBR0 (+0x8020). */
                if (s->gpu_smmu) {
                    qcom845_smmu_write32(s->gpu_smmu, 0x8020, w[0]);
                    qcom845_smmu_write32(s->gpu_smmu, 0x8024, w[1]);
                }
            } else if (op == 0x46 && n >= 1) {
                /* CP_EVENT_WRITE */
                bool irq = (w[0] & BIT(31)) != 0;

                if (n >= 4) {
                    uint64_t dst = w[1] | ((uint64_t)w[2] << 32);
                    qcom845_gpu_write32(s, dst, w[3]);
                    if (n >= 5) {
                        qcom845_gpu_write32(s, dst + 4, w[4]);
                    }
                }
                if (irq) {
                    adreno->regs[A6XX_RBBM_INT_0_STATUS / 4] |=
                        A6XX_INT_CACHE_FLUSH_TS;
                    qcom845_update_gfx_irq(s);
                }
            }

            pos += n;
        } else {
            /* Unknown packet header; advance one dword safely. */
            trace_qcom845_cp_pkt(0xff, type, hdr, 0);
        }
    }
}

static void qcom845_cp_process_ring(Qcom845Display *s)
{
    Qcom845DisplayBank *adreno = &s->banks[BANK845_ADRENO];
    uint32_t cntl = adreno->regs[A6XX_CP_RB_CNTL / 4];
    uint32_t bufsz = cntl & 0x3f;
    uint32_t ring_dwords = (bufsz < 24) ? (1U << (bufsz + 1)) : 16384;
    uint32_t ring_mask = ring_dwords - 1;
    uint32_t rptr = adreno->regs[A6XX_CP_RB_RPTR / 4] & ring_mask;
    uint32_t wptr = adreno->regs[A6XX_CP_RB_WPTR / 4] & ring_mask;
    uint64_t rb_base = adreno->regs[A6XX_CP_RB_BASE / 4] |
                       ((uint64_t)adreno->regs[A6XX_CP_RB_BASE_HI / 4] << 32);
    uint64_t rptr_addr =
        adreno->regs[A6XX_CP_RB_RPTR_ADDR / 4] |
        ((uint64_t)adreno->regs[A6XX_CP_RB_RPTR_ADDR_HI / 4] << 32);
    uint32_t dwords = (wptr - rptr) & ring_mask;

    if (dwords == 0) {
        return;
    }

    trace_qcom845_cp_ring(rptr, wptr, rb_base, dwords);
    if (rb_base) {
        qcom845_cp_exec_stream(s, rb_base, rptr, dwords, ring_mask, 0);
    }

    adreno->regs[A6XX_CP_RB_RPTR / 4] = wptr;
    if (rptr_addr && !(cntl & BIT(27))) {
        qcom845_gpu_write32(s, rptr_addr, wptr);
    }
}

static uint64_t qcom845_bank_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom845DisplayBank *b = opaque;
    uint32_t idx = offset / 4;
    uint32_t val = idx < b->reg_count ? b->regs[idx] : 0;

    if (b->id == BANK845_MDSS) {
        if (offset == DPU4_MDSS_HW_VERSION || offset == DPU4_MDP_HW_VERSION) {
            val = DPU4_HW_VERSION_401;
        }
    } else if (b->id == BANK845_ADRENO) {
        if (offset == A6XX_RBBM_STATUS) {
            val = 0; /* Idle until command processor work is queued. */
        }
    } else if (b->id == BANK845_GMU_CX) {
        if (offset == A6XX_RSCC_TCS0_DRV0_STATUS) {
            val = A6XX_RSCC_TCS_IDLE;
        } else if (offset == A6XX_GMU_AO_GFX_INT_STATUS) {
            Qcom845DisplayBank *adreno = &b->parent->banks[BANK845_ADRENO];
            val = adreno->regs[A6XX_RBBM_INT_0_STATUS / 4] &
                  adreno->regs[A6XX_RBBM_INT_0_MASK / 4];
        }
    } else if (b->id == BANK845_VENUS) {
        if (offset == VENUS4_WRAPPER_HW_VERSION) {
            val = VENUS4_HW_VERSION_44;
        }
    }

    trace_qcom845_display_read(qcom845_bank_descs[b->id].name, offset, val);
    return val;
}

static void qcom845_bank_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    Qcom845DisplayBank *b = opaque;
    Qcom845Display *s = b->parent;
    uint32_t idx = offset / 4;
    uint32_t v = (uint32_t)value;

    trace_qcom845_display_write(qcom845_bank_descs[b->id].name, offset, v);

    if (idx >= b->reg_count) {
        return;
    }

    if (b->id == BANK845_MDSS) {
        if (offset == DPU4_MDSS_HW_VERSION || offset == DPU4_MDP_HW_VERSION ||
            offset == DPU4_MDSS_HW_INTR_STATUS ||
            offset == DPU4_MDP_INTR_STATUS) {
            return;
        }
        if (offset == DPU4_MDP_INTR_EN) {
            b->regs[idx] = v;
            qcom845_update_vsync_irq(s);
            qcom845_update_vsync_timer(s);
            return;
        }
        if (offset == DPU4_MDP_INTR_CLEAR) {
            b->regs[DPU4_MDP_INTR_STATUS / 4] &= ~v;
            qcom845_update_vsync_irq(s);
            return;
        }
        if (offset >= DPU4_CTL_BASE &&
            offset < DPU4_CTL_BASE + DPU4_CTL_COUNT * DPU4_CTL_STRIDE) {
            unsigned ctl = (offset - DPU4_CTL_BASE) / DPU4_CTL_STRIDE;
            unsigned reg = (offset - DPU4_CTL_BASE) % DPU4_CTL_STRIDE;

            if (reg == DPU4_CTL_SW_RESET) {
                b->regs[idx] = 0;
                return;
            }
            if (reg == DPU4_CTL_FLUSH) {
                b->regs[idx] = v;
                if (ctl < 2 && v != 0) {
                    qcom845_scanout_head(s, ctl);
                }
                return;
            }
            if (reg == DPU4_CTL_START && (v & 1)) {
                if (ctl < 2) {
                    qcom845_scanout_head(s, ctl);
                }
                b->regs[DPU4_MDP_INTR_STATUS / 4] |= DPU4_MDP_PP_DONE_IRQ(ctl);
                qcom845_update_vsync_irq(s);
                return;
            }
        }
        if ((offset >= DPU4_DSI0_BASE &&
             offset < DPU4_DSI0_BASE + DPU4_DSI_SIZE) ||
            (offset >= DPU4_DSI1_BASE &&
             offset < DPU4_DSI1_BASE + DPU4_DSI_SIZE)) {
            hwaddr dsi_base = (offset >= DPU4_DSI1_BASE)
                                  ? DPU4_DSI1_BASE
                                  : DPU4_DSI0_BASE;
            hwaddr reg = offset - dsi_base;

            if (reg == DPU4_DSI_SOFT_RESET) {
                b->regs[idx] = 0;
                return;
            }
            if (reg == DPU4_DSI_INT_CTRL) {
                /* Even bits are W1C status, odd bits are enable masks. */
                uint32_t cur = b->regs[idx];
                uint32_t st = (cur & 0x55555555U) & ~(v & 0x55555555U);
                uint32_t en = v & 0xaaaaaaaaU;
                b->regs[idx] = st | en;
                qcom845_update_vsync_irq(s);
                return;
            }
            if (reg == DPU4_DSI_DMA_SW_TRIGGER) {
                b->regs[(dsi_base + DPU4_DSI_INT_CTRL) / 4] |=
                    DPU4_DSI_INT_CMD_DONE;
                qcom845_update_vsync_irq(s);
                return;
            }
        }
        if (offset == 0x5014 || offset == 0x7014 || offset == 0x9014 ||
            offset == 0xb014 || offset == 0x25014 || offset == 0x27014 ||
            offset == 0x29014 || offset == 0x2b014) {
            s->sspp_dirty = true;
        }
        b->regs[idx] = v;
        return;
    }

    if (b->id == BANK845_VENUS && offset == VENUS4_WRAPPER_HW_VERSION) {
        return;
    }

    if (b->id == BANK845_ADRENO) {
        if (offset == A6XX_RBBM_INT_CLEAR_CMD ||
            offset == A6XX_RBBM_INT_CLEAR_ALT) {
            b->regs[A6XX_RBBM_INT_0_STATUS / 4] &= ~v;
            qcom845_update_gfx_irq(s);
            return;
        }
        if (offset == A6XX_RBBM_SW_RESET_CMD) {
            b->regs[idx] = 0;
            b->regs[A6XX_CP_RB_RPTR / 4] = 0;
            b->regs[A6XX_CP_RB_WPTR / 4] = 0;
            return;
        }
        b->regs[idx] = v;
        if (offset == A6XX_CP_RB_BASE || offset == A6XX_CP_RB_CNTL) {
            b->regs[A6XX_CP_RB_RPTR / 4] = 0;
            b->regs[A6XX_CP_RB_WPTR / 4] = 0;
        } else if (offset == A6XX_RBBM_INT_0_MASK) {
            qcom845_update_gfx_irq(s);
        } else if (offset == A6XX_CP_RB_WPTR) {
            qcom845_cp_process_ring(s);
        }
        return;
    }

    b->regs[idx] = v;
}

static const MemoryRegionOps qcom845_bank_ops = {
    .read = qcom845_bank_read,
    .write = qcom845_bank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void qcom845_display_reset(DeviceState *dev)
{
    Qcom845Display *s = QCOM845_DISPLAY(dev);

    if (s->vsync_timer) {
        timer_del(s->vsync_timer);
    }
    for (unsigned i = 0; i < BANK845_COUNT; i++) {
        memset(s->banks[i].regs, 0, s->banks[i].reg_count * sizeof(uint32_t));
    }
    for (unsigned h = 0; h < 2; h++) {
        if (s->panel_fb[h]) {
            memset(s->panel_fb[h], 0,
                   DPU4_PANEL_WIDTH * DPU4_PANEL_HEIGHT * sizeof(uint32_t));
            s->panel_dirty[h] = true;
        }
    }
    s->banks[BANK845_MDSS].regs[DPU4_MDSS_HW_VERSION / 4] = DPU4_HW_VERSION_401;
    s->banks[BANK845_MDSS].regs[DPU4_MDP_HW_VERSION / 4] = DPU4_HW_VERSION_401;
    s->banks[BANK845_VENUS].regs[VENUS4_WRAPPER_HW_VERSION / 4] =
        VENUS4_HW_VERSION_44;
    qemu_set_irq(s->vsync_irq, 0);
    qemu_set_irq(s->gfx_irq, 0);
    qemu_set_irq(s->vidc_irq, 0);
    qemu_set_irq(s->vidc_wd_irq, 0);
}

static void qcom845_display_realize(DeviceState *dev, Error **errp)
{
    Qcom845Display *s = QCOM845_DISPLAY(dev);

    s->vsync_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, qcom845_vsync_tick, s);
    for (unsigned h = 0; h < 2; h++) {
        s->panel_fb[h] = g_new0(uint32_t, DPU4_PANEL_WIDTH * DPU4_PANEL_HEIGHT);
        s->con[h] = qemu_graphic_console_create(dev, h, &qcom845_gfx_ops, s);
        qemu_console_resize(s->con[h], DPU4_PANEL_WIDTH, DPU4_PANEL_HEIGHT);
    }
    s->con[2] = qemu_graphic_console_create(dev, 2, &qcom845_gfx_ops, s);
    qemu_console_resize(s->con[2], DPU4_PANEL_WIDTH * 2, DPU4_PANEL_HEIGHT);
}

static void qcom845_display_init(Object *obj)
{
    Qcom845Display *s = QCOM845_DISPLAY(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    for (unsigned i = 0; i < BANK845_COUNT; i++) {
        Qcom845DisplayBank *b = &s->banks[i];
        char *mr_name = g_strdup_printf("qcom845.%s", qcom845_bank_descs[i].name);

        b->parent = s;
        b->id = i;
        b->reg_count = qcom845_bank_descs[i].size / 4;
        b->regs = g_new0(uint32_t, b->reg_count);
        memory_region_init_io(&b->mmio, obj, &qcom845_bank_ops, b, mr_name,
                              qcom845_bank_descs[i].size);
        g_free(mr_name);
        sysbus_init_mmio(sbd, &b->mmio);
    }

    sysbus_init_irq(sbd, &s->vsync_irq);
    sysbus_init_irq(sbd, &s->gfx_irq);
    sysbus_init_irq(sbd, &s->vidc_irq);
    sysbus_init_irq(sbd, &s->vidc_wd_irq);
}

static void qcom845_display_finalize(Object *obj)
{
    Qcom845Display *s = QCOM845_DISPLAY(obj);

    if (s->vsync_timer) {
        timer_free(s->vsync_timer);
    }
    for (unsigned h = 0; h < 2; h++) {
        g_free(s->panel_fb[h]);
    }
    for (unsigned i = 0; i < BANK845_COUNT; i++) {
        g_free(s->banks[i].regs);
    }
}

static const VMStateDescription qcom845_display_vmstate = {
    .name = TYPE_QCOM845_DISPLAY,
    .unmigratable = true,
};

static void qcom845_display_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = qcom845_display_realize;
    device_class_set_legacy_reset(dc, qcom845_display_reset);
    dc->desc = "Experimental SDM845 Adreno 630 / GMU / DPU 4.0 frontend";
    dc->vmsd = &qcom845_display_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo qcom845_display_type = {
    .name = TYPE_QCOM845_DISPLAY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom845Display),
    .instance_init = qcom845_display_init,
    .instance_finalize = qcom845_display_finalize,
    .class_init = qcom845_display_class_init,
};

static void qcom845_display_register_types(void)
{
    type_register_static(&qcom845_display_type);
}
type_init(qcom845_display_register_types)

void qcom845_display_create(DeviceState *gic, Qcom845Smmu *apps_smmu,
                            Qcom845Smmu *gpu_smmu)
{
    DeviceState *dev = qdev_new(TYPE_QCOM845_DISPLAY);
    Qcom845Display *s = QCOM845_DISPLAY(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    dev->id = g_strdup("qcom845-display");
    s->apps_smmu = apps_smmu;
    s->gpu_smmu = gpu_smmu;
    sysbus_realize_and_unref(sbd, &error_fatal);
    for (unsigned i = 0; i < BANK845_COUNT; i++) {
        sysbus_mmio_map(sbd, i, qcom845_bank_descs[i].base);
    }
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(gic, QCOM845_IRQ_VSYNC - 32));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(gic, QCOM845_IRQ_GFX - 32));
    sysbus_connect_irq(sbd, 2, qdev_get_gpio_in(gic, QCOM845_IRQ_VIDC - 32));
    sysbus_connect_irq(sbd, 3, qdev_get_gpio_in(gic, QCOM845_IRQ_VIDC_WD - 32));
}

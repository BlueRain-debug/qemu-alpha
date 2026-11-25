/*
 * Internal definitions shared by the experimental SDM845 display and
 * Adreno 630 software render backend.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_DISPLAY_QCOM_845_DISPLAY_INTERNAL_H
#define HW_DISPLAY_QCOM_845_DISPLAY_INTERNAL_H

#include "hw/display/qcom-845-display.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "ui/console.h"

OBJECT_DECLARE_SIMPLE_TYPE(Qcom845Display, QCOM845_DISPLAY)

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

typedef struct Qcom845DisplayBank {
    Qcom845Display *parent;
    enum Qcom845BankId id;
    MemoryRegion mmio;
    uint32_t *regs;
    uint32_t reg_count;
} Qcom845DisplayBank;

#define A6XX_MAX_DRAW_STATES  32
#define A6XX_LOAD_BLOCKS      16
#define A6XX_LOAD_TYPES       3
#define A6XX_LOAD_MAX_WORDS   32
#define A6XX_LOAD_MAX_UNITS   1024

typedef struct Qcom845DrawState {
    uint64_t addr;
    uint32_t dwords;
    bool enabled;
    bool dirty;
} Qcom845DrawState;

typedef struct Qcom845StateUnit {
    uint32_t words[A6XX_LOAD_MAX_WORDS];
    uint32_t ubo_slot;
    uint32_t ubo_offset;
    bool ubo;
    bool valid;
} Qcom845StateUnit;

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
    unsigned panel_y1[2], panel_y2[2];
    bool scanout_active;
    bool sspp_dirty;
    uint32_t cp_marker;
    uint32_t cp_reg_trace_count;
    Qcom845DrawState draw_states[A6XX_MAX_DRAW_STATES];
    GArray *load_state[A6XX_LOAD_BLOCKS][A6XX_LOAD_TYPES];
};

bool qcom845_gpu_read32(Qcom845Display *s, uint64_t iova, uint32_t *val);
bool qcom845_gpu_write32(Qcom845Display *s, uint64_t iova, uint32_t val);

void qcom845_render_reset(Qcom845Display *s);
void qcom845_render_finalize(Qcom845Display *s);
void qcom845_render_load_state6(Qcom845Display *s, uint64_t stream_base,
                                uint32_t start, uint32_t ring_mask,
                                uint32_t pkt_pos, uint32_t pkt_dwords,
                                const uint32_t *w);
bool qcom845_render_draw(Qcom845Display *s, uint32_t n, const uint32_t *w);

#endif

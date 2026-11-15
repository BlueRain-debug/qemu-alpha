/*
 * MSM8998 display bring-up for the Windows QCDX miniport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/acpi/aml-build.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/display/qcom-8998-display.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "trace.h"

#define TYPE_QCOM8998_DISPLAY "qcom-8998-display"
OBJECT_DECLARE_SIMPLE_TYPE(Qcom8998Display, QCOM8998_DISPLAY)

enum {
    BANK_MDSS,
    BANK_ADRENO,
    BANK_VIDEO,
    BANK_VIDEO_VMEM,
    BANK_MDSS_SE,
    BANK_MDSS_SMMU,
    BANK_ADRENO_SMMU,
    BANK_SOC_JTAG_ID,
    BANK_SOC_VERSION,
    BANK_CPRH_APC0,
    BANK_CPRH_APC1,
    BANK_QFPROM,
    BANK_MMCC,
    BANK_GPUCC,
    BANK_CPR_GFX,
    BANK_COUNT,
};

enum {
    MDSS_GSIV = 115,
    ADRENO_GSIV = 332,
    VIDEO_GSIV = 319,
    GIC_INTERNAL_IRQS = 32,
};

/* Display resource lengths and ordering are from the OEMSK PnP BootConfig. */
static const struct {
    const char *name;
    uint32_t base;
    uint32_t size;
} banks[BANK_COUNT] = {
    [BANK_MDSS] = { "mdss", 0x0c900000, 0xba008 },
    [BANK_ADRENO] = { "adreno", 0x05000000, 0x3f000 },
    [BANK_VIDEO] = { "venus", 0x0cc00000, 0x100000 },
    [BANK_VIDEO_VMEM] = { "venus-vmem", 0x0c880000, 0x800 },
    [BANK_MDSS_SE] = { "mdss-se", 0x0c9e0000, 0x2c },
    [BANK_MDSS_SMMU] = { "mdss-smmu", 0x0cd00000, 0x40000 },
    [BANK_ADRENO_SMMU] = { "adreno-smmu", 0x05040000, 0xe000 },
    /*
     * The SMMU driver reads these physical chip-info registers before it
     * maps its ACPI resources. Map just the words within the virt code flash.
     */
    [BANK_SOC_JTAG_ID] = { "soc-jtag-id", 0x03998010, 4 },
    [BANK_SOC_VERSION] = { "soc-version", 0x01fc8000, 4 },
    /* Native PEP CPRH_SW_CONTROLLER_SETTING entries for Silver and Gold. */
    [BANK_CPRH_APC0] = { "cprh-apc0", 0x179c8000, 0x4000 },
    [BANK_CPRH_APC1] = { "cprh-apc1", 0x179c4000, 0x4000 },
    /*
     * PEP maps raw OTP at 0x780000; the corrected QFPROM view starts at
     * 0x784000 and spans 0x621c bytes in the MSM8998 memory map. Raw OTP
     * contains a synthetic CPR calibration profile, without physical-device
     * keys or calibration copied from a particular manufactured chip.
     */
    [BANK_QFPROM] = { "qfprom", 0x00780000, 0xa21c },
    /* Native multimedia clock controller, through its last reset register. */
    [BANK_MMCC] = { "mmcc", 0x0c8c0000, 0x10008 },
    [BANK_GPUCC] = { "gpucc", 0x05065000, 0x9004 },
    /* Original PEP CPR_SW_CONTROLLER_SETTING for the graphics regulator. */
    [BANK_CPR_GFX] = { "cpr-gfx", 0x05061000, 0x4000 },
};

#define MDSS_HW_VERSION 0x0
#define MDSS_MDP_HW_VERSION 0x1000
#define MSM8998_MDP_REV_301 0x30000001
/* Five MDP control paths at 0x1000..0x1800 relative to the MDP block. */
#define MDSS_CTL_BASE 0x2000
#define MDSS_CTL_STRIDE 0x200
#define MDSS_CTL_COUNT 5
#define MDSS_CTL_REG_SIZE 0x94
#define MDSS_CTL_SW_RESET 0x30
#define MDSS_CTL_SW_RESET_REQUEST BIT(0)
#define MDSS_HW_INTR_STATUS 0x10
#define MDSS_DSI0_INTR BIT(4)
#define MDSS_DSI1_INTR BIT(5)

/* DSI 6G has a version word before the common DSI registers. */
#define MDSS_DSI0_BASE 0x94000
#define MDSS_DSI1_BASE 0x96000
#define MDSS_DSI_MMIO_SIZE 0x400
#define DSI_CTRL 0x004
#define DSI_CTRL_ENABLE BIT(0)
#define DSI_CTRL_CMD_MODE_EN BIT(2)
#define DSI_DMA_LEN 0x04c
#define DSI_TRIG_DMA 0x090
#define DSI_INTR_CTRL 0x110
#define DSI_IRQ_CMD_DMA_DONE BIT(0)
#define DSI_IRQ_STATUS (BIT(0) | BIT(8) | BIT(16) | BIT(20) | BIT(24))
#define DSI_IRQ_ENABLE (DSI_IRQ_STATUS << 1)
#define DSI_RESET 0x118
#define ADRENO_RBBM_HW_CONFIGURATION 0x8
/*
 * The native A5xx miniport decodes bits [23:16] as the silicon identifier:
 * 0x31 selects Adreno 540 (device 540), with stepping 2 in the low nibble.
 * A zero register makes its hardware discovery callback reject the engine.
 */
#define ADRENO540_HW_CONFIGURATION 0x00310002
/* GPMU register indices in a5xx.xml are in 32-bit words. */
#define ADRENO_GPMU_RBCCU_POWER_CNTL (0xa887 * 4)
#define ADRENO_GPMU_SP_POWER_CNTL (0xa881 * 4)
#define ADRENO_GPMU_RBCCU_PWR_CLK_STATUS (0xa88d * 4)
#define ADRENO_GPMU_SP_PWR_CLK_STATUS (0xa88b * 4)
#define ADRENO_GPMU_SW_COLLAPSE BIT(0)
#define ADRENO_GPMU_PWR_ON BIT(20)
#define VENUS_WRAPPER_HW_VERSION 0xe0000
/* Venus 4.4 identity recognized by the native MSM8998 video miniport. */
#define VENUS_MSM8998_HW_VERSION 0x40440000
#define APQ8098_JTAG_ID 0x000620e1
#define APQ8098_SOC_HW_VERSION_2_0 0x30020200
/* CPRh layout from Qualcomm's cpr3-regulator.c; MSM8998 uses 4 mV steps. */
#define CPRH_CTL 0x3aa0
#define CPRH_CTL_OSM_ENABLED BIT(0)
#define CPRH_BASE_MILLIVOLTS 352
#define CPRH_STEP_MILLIVOLTS 4
#define CPRH_CTL_BASE_SHIFT 1
#define CPRH_CTL_MULTIPLIER_SHIFT 25
#define QFPROM_ROW_BITS 64
#define QFPROM_QUOTIENT_BITS 12
/* Qualcomm clk-branch.h and gdsc.c. */
#define MMCC_CBCR_ENABLE BIT(0)
#define MMCC_CBCR_CLOCK_OFF BIT(31)
#define MMCC_CBCR_FSM_STATUS (7U << 28)
#define GDSC_SW_COLLAPSE BIT(0)
#define GDSC_POWER_ON BIT(31)
#define CLOCK_CMD_UPDATE BIT(0)
/* Fabia PLL MODE/OPMODE and MMCC vote register, per clk-alpha-pll.c. */
#define MMCC_PLL_VOTE 0x1e0
#define FABIA_PLL_RESET_N BIT(2)
#define FABIA_PLL_FSM_ENABLE BIT(20)
#define FABIA_PLL_FSM_RESET BIT(21)
#define FABIA_PLL_UPDATE BIT(22)
#define FABIA_PLL_UPDATE_BYPASS BIT(23)
#define FABIA_PLL_UPDATE_ACK BIT(29)
#define FABIA_PLL_ACTIVE BIT(30)
#define FABIA_PLL_LOCK BIT(31)
#define FABIA_PLL_STATUS \
    (FABIA_PLL_UPDATE_ACK | FABIA_PLL_ACTIVE | FABIA_PLL_LOCK)
#define FABIA_PLL_OPMODE 0x2c
#define FABIA_PLL_RUN 1
#define SMMU_IDR0 0x20
#define SMMU_IDR1 0x24
#define SMMU_IDR2 0x28
#define SMMU_IDR0_STAGE1 (1U << 30)
#define SMMU_IDR0_STREAM_MATCH (1U << 27)
#define SMMU_IDR0_CONTEXT_IRQS(n) ((n) << 16)
#define SMMU_IDR0_STREAM_ID_BITS(n) ((n) << 9)
#define SMMU_IDR1_PAGE_INDEX_BITS(n) ((n) << 28)
#define SMMU_IDR2_4K_PAGES (1U << 12)
#define SMMU_IDR2_VA_48BIT (5U << 8)
#define SMMU_IDR2_PA_40BIT (2U << 4)
#define SMMU_IDR2_IPA_40BIT 2U

typedef struct Qcom8998Bank {
    MemoryRegion region;
    Qcom8998Display *display;
    unsigned index;
    uint32_t *regs;
    hwaddr last_read_offset;
    uint32_t last_read_value;
} Qcom8998Bank;

struct Qcom8998Display {
    SysBusDevice parent_obj;
    Qcom8998Bank bank[BANK_COUNT];
    qemu_irq irq[3];
};

static void qcom8998_mdss_update_irq(Qcom8998Bank *bank)
{
    uint32_t dsi0 = bank->regs[(MDSS_DSI0_BASE + DSI_INTR_CTRL) / 4];
    uint32_t dsi1 = bank->regs[(MDSS_DSI1_BASE + DSI_INTR_CTRL) / 4];
    uint32_t pending = 0;

    if (dsi0 & (dsi0 >> 1) & DSI_IRQ_STATUS) {
        pending |= MDSS_DSI0_INTR;
    }
    if (dsi1 & (dsi1 >> 1) & DSI_IRQ_STATUS) {
        pending |= MDSS_DSI1_INTR;
    }
    bank->regs[MDSS_HW_INTR_STATUS / 4] = pending;
    qemu_set_irq(bank->display->irq[0], pending != 0);
}

/* RCG2 command offsets from Qualcomm's MSM8998 MMCC/GPUCC clock drivers. */
static bool qcom8998_clock_is_rcg(Qcom8998Bank *bank, hwaddr offset)
{
    static const uint32_t mmcc_rcgs[] = {
        0x1000, 0x1060, 0x1080, 0x2000, 0x2020, 0x2040, 0x2060, 0x2080,
        0x2100, 0x2120, 0x2140, 0x2160, 0x2180, 0x21a0, 0x2200, 0x2220,
        0x2240, 0x2260, 0x3000, 0x3030, 0x3060, 0x3090, 0x3100, 0x3160,
        0x31c0, 0x3300, 0x3360, 0x3390, 0x33c0, 0x33f0, 0x3500, 0x3600,
        0x3620, 0x3640, 0x3800, 0x3b00, 0x5000, 0xd000, 0xf020,
    };
    static const uint32_t gpucc_rcgs[] = { 0x1030, 0x1070, 0x10b0, 0x1100 };
    const uint32_t *registers;
    size_t count;

    if (bank->index == BANK_MMCC) {
        registers = mmcc_rcgs;
        count = ARRAY_SIZE(mmcc_rcgs);
    } else if (bank->index == BANK_GPUCC) {
        registers = gpucc_rcgs;
        count = ARRAY_SIZE(gpucc_rcgs);
    } else {
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        if (registers[i] == offset) {
            return true;
        }
    }
    return false;
}

/* Independent software votes within the virtual MMCC and GPUCC domains. */
static const uint16_t mmcc_gdscs[] = {
    0x1024, 0x1040, 0x1044, 0x2304, 0x34a0, 0x3664, 0x3674, 0x36d4, 0xe020,
};
static const uint16_t gpucc_gdscs[] = { 0x1004, 0x1094 };

/* Video/MDSS, camera AHB and shared fabric branches used during bring-up. */
static const uint16_t mmcc_cbcrs[] = {
    0x1028, 0x1030, 0x1034, 0x1038, 0x1048, 0x104c, 0x2308, 0x230c,
    0x2310, 0x2314, 0x2318, 0x231c, 0x2320, 0x2324, 0x2328, 0x2338,
    0x233c, 0x2340, 0x2344, 0x2348, 0x2350, 0x2354, 0x2358, 0x235c,
    0x2360, 0x2364, 0x2374, 0x2378, 0x3494, 0x5024, 0xe004, 0xe008,
    0xf004, 0xf064, 0xf068,
};
static const uint16_t gpucc_cbcrs[] = {
    0x1020, 0x1054, 0x1098, 0x10d0, 0x1124,
};

static bool qcom8998_clock_contains(hwaddr offset, const uint16_t *registers,
                                    size_t count)
{
    for (unsigned i = 0; i < count; i++) {
        if (registers[i] == offset) {
            return true;
        }
    }
    return false;
}

static bool qcom8998_clock_power_write(Qcom8998Bank *bank, hwaddr offset,
                                       uint32_t value)
{
    const uint16_t *gdscs, *cbcrs;
    size_t gdsc_count, cbcr_count;

    if (bank->index == BANK_MMCC) {
        gdscs = mmcc_gdscs;
        gdsc_count = ARRAY_SIZE(mmcc_gdscs);
        cbcrs = mmcc_cbcrs;
        cbcr_count = ARRAY_SIZE(mmcc_cbcrs);
    } else if (bank->index == BANK_GPUCC) {
        gdscs = gpucc_gdscs;
        gdsc_count = ARRAY_SIZE(gpucc_gdscs);
        cbcrs = gpucc_cbcrs;
        cbcr_count = ARRAY_SIZE(gpucc_cbcrs);
    } else {
        return false;
    }

    if (qcom8998_clock_contains(offset, gdscs, gdsc_count)) {
        bank->regs[offset / 4] = (value & ~GDSC_POWER_ON) |
            ((value & GDSC_SW_COLLAPSE) ? 0 : GDSC_POWER_ON);
        return true;
    }
    if (qcom8998_clock_contains(offset, cbcrs, cbcr_count)) {
        bank->regs[offset / 4] =
            (value & ~(MMCC_CBCR_CLOCK_OFF | MMCC_CBCR_FSM_STATUS)) |
            ((value & MMCC_CBCR_ENABLE) ? 0 : MMCC_CBCR_CLOCK_OFF);
        return true;
    }
    return false;
}

static bool qcom8998_clock_pll_base(Qcom8998Bank *bank, hwaddr offset,
                                   hwaddr *base)
{
    static const uint32_t mmcc_plls[] = {
        0x0000, 0x0050, 0x00a0, 0x00f0, 0x0140, 0x0190, 0xc000, 0xc050,
    };

    if (bank->index == BANK_GPUCC && offset < 0x50) {
        *base = 0;
        return true;
    }
    if (bank->index == BANK_MMCC) {
        for (unsigned i = 0; i < ARRAY_SIZE(mmcc_plls); i++) {
            if (offset >= mmcc_plls[i] && offset < mmcc_plls[i] + 0x50) {
                *base = mmcc_plls[i];
                return true;
            }
        }
    }
    return false;
}

static void qcom8998_clock_update_pll(Qcom8998Bank *bank, hwaddr base)
{
    uint32_t mode = bank->regs[base / 4];
    bool running;

    if (bank->index == BANK_MMCC && (base == 0xc000 || base == 0xc050) &&
        (mode & FABIA_PLL_FSM_ENABLE)) {
        unsigned vote_bit = base == 0xc000 ? 0 : 1;

        running = !(mode & FABIA_PLL_FSM_RESET) &&
                  (bank->regs[MMCC_PLL_VOTE / 4] & BIT(vote_bit));
    } else {
        running = (mode & FABIA_PLL_RESET_N) &&
                  bank->regs[(base + FABIA_PLL_OPMODE) / 4] == FABIA_PLL_RUN;
    }
    /* No analogue settling is simulated; a virtual PLL locks immediately. */
    mode &= ~FABIA_PLL_STATUS;
    if (running) {
        mode |= FABIA_PLL_ACTIVE | FABIA_PLL_LOCK;
        if (mode & FABIA_PLL_UPDATE) {
            if (mode & FABIA_PLL_UPDATE_BYPASS) {
                /* Software clears UPDATE after seeing the latched ACK. */
                mode |= FABIA_PLL_UPDATE_ACK;
            } else {
                mode &= ~FABIA_PLL_UPDATE;
            }
        }
    }
    bank->regs[base / 4] = mode;
}

static void qcom8998_dsi_write(Qcom8998Bank *bank, hwaddr base,
                               hwaddr offset, uint32_t value)
{
    uint32_t *dsi = &bank->regs[base / 4];

    switch (offset) {
    case DSI_TRIG_DMA:
        /*
         * The virtual panel consumes command writes without a physical link.
         * It does not implement DMA payload processing or panel readback.
         */
        if ((value & 1) && dsi[DSI_DMA_LEN / 4] &&
            (dsi[DSI_CTRL / 4] & (DSI_CTRL_ENABLE | DSI_CTRL_CMD_MODE_EN)) ==
            (DSI_CTRL_ENABLE | DSI_CTRL_CMD_MODE_EN)) {
            dsi[DSI_INTR_CTRL / 4] |= DSI_IRQ_CMD_DMA_DONE;
            qcom8998_mdss_update_irq(bank);
        }
        return;
    case DSI_INTR_CTRL:
        dsi[DSI_INTR_CTRL / 4] =
            (dsi[DSI_INTR_CTRL / 4] & DSI_IRQ_STATUS & ~value) |
            (value & DSI_IRQ_ENABLE);
        qcom8998_mdss_update_irq(bank);
        return;
    case DSI_RESET:
        if (value & 1) {
            dsi[DSI_INTR_CTRL / 4] &= DSI_IRQ_ENABLE;
            qcom8998_mdss_update_irq(bank);
        }
        break;
    }
    dsi[offset / 4] = value;
}

static bool qcom8998_adreno_power_write(Qcom8998Bank *bank, hwaddr offset,
                                        uint32_t value)
{
    hwaddr status;

    switch (offset) {
    case ADRENO_GPMU_RBCCU_PWR_CLK_STATUS:
    case ADRENO_GPMU_SP_PWR_CLK_STATUS:
        return true;
    case ADRENO_GPMU_RBCCU_POWER_CNTL:
        status = ADRENO_GPMU_RBCCU_PWR_CLK_STATUS;
        break;
    case ADRENO_GPMU_SP_POWER_CNTL:
        status = ADRENO_GPMU_SP_PWR_CLK_STATUS;
        break;
    default:
        return false;
    }

    /* The emulated power domains transition without analogue settling. */
    bank->regs[offset / 4] = value;
    bank->regs[status / 4] = value & ADRENO_GPMU_SW_COLLAPSE ?
                            0 : ADRENO_GPMU_PWR_ON;
    return true;
}

static uint64_t qcom8998_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom8998Bank *bank = opaque;
    uint32_t value = bank->regs[offset / sizeof(value)];

    /* Repeated polls should not dominate the bring-up trace. */
    if (offset != bank->last_read_offset || value != bank->last_read_value) {
        trace_qcom8998_mmio_read(banks[bank->index].name, offset, value,
                                current_cpu ?
                                current_cpu->cc->get_pc(current_cpu) : 0);
        bank->last_read_offset = offset;
        bank->last_read_value = value;
    }
    return value;
}

static void qcom8998_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    Qcom8998Bank *bank = opaque;
    bool smmu = bank->index == BANK_MDSS_SMMU ||
                bank->index == BANK_ADRENO_SMMU;
    hwaddr pll_base;

    trace_qcom8998_mmio_write(banks[bank->index].name, offset, value,
                             current_cpu ?
                             current_cpu->cc->get_pc(current_cpu) : 0);
    if (bank->index == BANK_SOC_JTAG_ID || bank->index == BANK_SOC_VERSION ||
        bank->index == BANK_QFPROM ||
        (smmu && offset >= SMMU_IDR0 && offset <= SMMU_IDR2) ||
        (bank->index == BANK_MDSS &&
         (offset == MDSS_HW_VERSION || offset == MDSS_MDP_HW_VERSION ||
          offset == MDSS_HW_INTR_STATUS)) ||
        (bank->index == BANK_ADRENO &&
         offset == ADRENO_RBBM_HW_CONFIGURATION) ||
        (bank->index == BANK_VIDEO && offset == VENUS_WRAPPER_HW_VERSION)) {
        return;
    }
    if (bank->index == BANK_MDSS) {
        if (offset >= MDSS_DSI0_BASE &&
            offset < MDSS_DSI0_BASE + MDSS_DSI_MMIO_SIZE) {
            qcom8998_dsi_write(bank, MDSS_DSI0_BASE, offset - MDSS_DSI0_BASE,
                               value);
            return;
        }
        if (offset >= MDSS_DSI1_BASE &&
            offset < MDSS_DSI1_BASE + MDSS_DSI_MMIO_SIZE) {
            qcom8998_dsi_write(bank, MDSS_DSI1_BASE, offset - MDSS_DSI1_BASE,
                               value);
            return;
        }
    }
    if (qcom8998_clock_is_rcg(bank, offset)) {
        /* RCG configuration latches immediately, clearing CMD_UPDATE. */
        bank->regs[offset / 4] = value & ~CLOCK_CMD_UPDATE;
        return;
    }
    if (bank->index == BANK_MMCC && offset == MMCC_PLL_VOTE) {
        bank->regs[offset / 4] = value;
        qcom8998_clock_update_pll(bank, 0xc000);
        qcom8998_clock_update_pll(bank, 0xc050);
        return;
    }
    if (qcom8998_clock_pll_base(bank, offset, &pll_base) &&
        (offset == pll_base || offset == pll_base + FABIA_PLL_OPMODE)) {
        bank->regs[offset / 4] = value;
        qcom8998_clock_update_pll(bank, pll_base);
        return;
    }
    if (qcom8998_clock_power_write(bank, offset, value)) {
        return;
    }
    if (bank->index == BANK_ADRENO &&
        qcom8998_adreno_power_write(bank, offset, value)) {
        return;
    }
    if (bank->index == BANK_MDSS &&
        offset >= MDSS_CTL_BASE &&
        offset < MDSS_CTL_BASE + MDSS_CTL_COUNT * MDSS_CTL_STRIDE &&
        (offset - MDSS_CTL_BASE) % MDSS_CTL_STRIDE == MDSS_CTL_SW_RESET) {
        if (value & MDSS_CTL_SW_RESET_REQUEST) {
            hwaddr control = offset - MDSS_CTL_SW_RESET;

            /* Reset completes immediately in this register-level model. */
            memset(&bank->regs[control / sizeof(uint32_t)], 0,
                   MDSS_CTL_REG_SIZE);
        }
        return;
    }
    bank->regs[offset / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps qcom8998_mmio_ops = {
    .read = qcom8998_read,
    .write = qcom8998_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void qcom8998_smmu_reset(Qcom8998Bank *bank, unsigned stream_entries,
                               unsigned page_index_bits, unsigned contexts)
{
    /*
     * Advertise the layout used by QCSMMU's stream and context-bank setup.
     * The probe models the registers only. It has no DMA translation yet.
     */
    bank->regs[SMMU_IDR0 / 4] = SMMU_IDR0_STAGE1 | SMMU_IDR0_STREAM_MATCH |
        SMMU_IDR0_CONTEXT_IRQS(1U) | SMMU_IDR0_STREAM_ID_BITS(15U) |
        stream_entries;
    bank->regs[SMMU_IDR1 / 4] = SMMU_IDR1_PAGE_INDEX_BITS(page_index_bits) |
        contexts;
    bank->regs[SMMU_IDR2 / 4] = SMMU_IDR2_4K_PAGES | SMMU_IDR2_VA_48BIT |
        SMMU_IDR2_PA_40BIT | SMMU_IDR2_IPA_40BIT;
}

static void qcom8998_qfprom_calibration(Qcom8998Bank *bank)
{
    /*
     * Qualcomm cprh-kbss-regulator.c defines these MSM8998 target-quotient
     * fuse locations. A field can straddle two adjacent 64-bit fuse rows.
     * All-zero quotients are not usable calibration: PEP's high corners
     * become negative after adjustment. Provide monotonic synthetic counts
     * for LowSVS, SVS, Nominal and Turbo while retaining zero voltage trims.
     * These counts describe the virtual chip, not original OEMSK fuse data.
     */
    static const struct {
        unsigned row;
        unsigned bit;
    } fields[2][4] = {
        { { 68, 18 }, { 68, 6 }, { 67, 58 }, { 67, 46 } }, /* Silver */
        { { 70, 32 }, { 70, 20 }, { 70, 8 }, { 69, 60 } }, /* Gold */
    };
    static const uint32_t quotient[] = { 400, 600, 800, 1000 };

    for (unsigned cluster = 0; cluster < ARRAY_SIZE(fields); cluster++) {
        for (unsigned corner = 0; corner < ARRAY_SIZE(quotient); corner++) {
            unsigned start = fields[cluster][corner].row * QFPROM_ROW_BITS +
                             fields[cluster][corner].bit;

            for (unsigned bit = 0; bit < QFPROM_QUOTIENT_BITS; bit++) {
                if (quotient[corner] & BIT(bit)) {
                    bank->regs[(start + bit) / 32] |= BIT((start + bit) % 32);
                }
            }
        }
    }
}

static void qcom8998_reset(DeviceState *dev)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(dev);

    for (unsigned i = 0; i < BANK_COUNT; i++) {
        memset(s->bank[i].regs, 0, banks[i].size);
        s->bank[i].last_read_offset = HWADDR_MAX;
    }
    s->bank[BANK_MDSS].regs[MDSS_HW_VERSION / 4] = MSM8998_MDP_REV_301;
    s->bank[BANK_MDSS].regs[MDSS_MDP_HW_VERSION / 4] = MSM8998_MDP_REV_301;
    s->bank[BANK_ADRENO].regs[ADRENO_RBBM_HW_CONFIGURATION / 4] =
        ADRENO540_HW_CONFIGURATION;
    s->bank[BANK_VIDEO].regs[VENUS_WRAPPER_HW_VERSION / 4] =
        VENUS_MSM8998_HW_VERSION;
    s->bank[BANK_SOC_JTAG_ID].regs[0] = APQ8098_JTAG_ID;
    s->bank[BANK_SOC_VERSION].regs[0] = APQ8098_SOC_HW_VERSION_2_0;
    qcom8998_qfprom_calibration(&s->bank[BANK_QFPROM]);
    for (unsigned i = 0; i < ARRAY_SIZE(mmcc_gdscs); i++) {
        s->bank[BANK_MMCC].regs[mmcc_gdscs[i] / 4] = GDSC_SW_COLLAPSE;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(gpucc_gdscs); i++) {
        s->bank[BANK_GPUCC].regs[gpucc_gdscs[i] / 4] = GDSC_SW_COLLAPSE;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(mmcc_cbcrs); i++) {
        s->bank[BANK_MMCC].regs[mmcc_cbcrs[i] / 4] = MMCC_CBCR_CLOCK_OFF;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(gpucc_cbcrs); i++) {
        s->bank[BANK_GPUCC].regs[gpucc_cbcrs[i] / 4] = MMCC_CBCR_CLOCK_OFF;
    }
    /*
     * Seed the firmware handoff state: generic virt EDK2 has no Qualcomm
     * XBL/OSM initialization. This is the MSM8998 CPRh base-voltage setting,
     * not a measurement of a simulated PMIC or a closed-loop CPR engine.
     */
    for (unsigned i = BANK_CPRH_APC0; i <= BANK_CPRH_APC1; i++) {
        s->bank[i].regs[CPRH_CTL / 4] = CPRH_CTL_OSM_ENABLED |
            ((CPRH_BASE_MILLIVOLTS / CPRH_STEP_MILLIVOLTS) <<
             CPRH_CTL_BASE_SHIFT) |
            (CPRH_STEP_MILLIVOLTS << CPRH_CTL_MULTIPLIER_SHIFT);
    }
    /* Context banks start at 0x20000 for MDSS and 0x8000 for Adreno. */
    qcom8998_smmu_reset(&s->bank[BANK_MDSS_SMMU], 60, 4, 16);
    qcom8998_smmu_reset(&s->bank[BANK_ADRENO_SMMU], 16, 2, 2);
    qemu_set_irq(s->irq[0], 0);
}

static void qcom8998_init(Object *obj)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(obj);
    SysBusDevice *bus = SYS_BUS_DEVICE(obj);

    for (unsigned i = 0; i < BANK_COUNT; i++) {
        Qcom8998Bank *bank = &s->bank[i];
        g_autofree char *name = g_strdup_printf("qcom8998.%s", banks[i].name);

        bank->display = s;
        bank->index = i;
        bank->regs = g_malloc0(banks[i].size);
        memory_region_init_io(&bank->region, obj, &qcom8998_mmio_ops, bank,
                              name, banks[i].size);
        sysbus_init_mmio(bus, &bank->region);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->irq); i++) {
        sysbus_init_irq(bus, &s->irq[i]);
    }
}

static void qcom8998_finalize(Object *obj)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(obj);

    for (unsigned i = 0; i < BANK_COUNT; i++) {
        g_free(s->bank[i].regs);
    }
}

static const VMStateDescription qcom8998_vmstate = {
    .name = TYPE_QCOM8998_DISPLAY,
    .unmigratable = 1,
};

static void qcom8998_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, qcom8998_reset);
    dc->desc = "Experimental MSM8998 QCDX MMIO bring-up";
    dc->vmsd = &qcom8998_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo qcom8998_type = {
    .name = TYPE_QCOM8998_DISPLAY,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Qcom8998Display),
    .instance_init = qcom8998_init,
    .instance_finalize = qcom8998_finalize,
    .class_init = qcom8998_class_init,
};

static void qcom8998_register_types(void)
{
    type_register_static(&qcom8998_type);
}
type_init(qcom8998_register_types)

void qcom8998_display_create(DeviceState *gic)
{
    static const uint32_t gsiv[] = { MDSS_GSIV, ADRENO_GSIV, VIDEO_GSIV };
    SysBusDevice *bus = SYS_BUS_DEVICE(qdev_new(TYPE_QCOM8998_DISPLAY));

    DEVICE(bus)->id = g_strdup("qcdx8998");
    sysbus_realize_and_unref(bus, &error_fatal);
    for (unsigned i = 0; i < BANK_COUNT; i++) {
        sysbus_mmio_map_overlap(bus, i, banks[i].base, 1);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(gsiv); i++) {
        sysbus_connect_irq(bus, i, qdev_get_gpio_in(gic,
                           gsiv[i] - GIC_INTERNAL_IRQS));
    }
}

static void qcom8998_acpi_mem(Aml *crs, unsigned bank)
{
    aml_append(crs, aml_memory32_fixed(banks[bank].base, banks[bank].size,
                                       AML_READ_WRITE));
}

static void qcom8998_acpi_irq(Aml *crs, uint32_t gsiv, AmlLevelAndEdge sense)
{
    aml_append(crs, aml_interrupt(AML_CONSUMER, sense, AML_ACTIVE_HIGH,
                                  AML_EXCLUSIVE, &gsiv, 1));
}

static Aml *qcom8998_acpi_device(const char *name, const char *hid, unsigned uid)
{
    Aml *dev = aml_device("%s", name);

    aml_append(dev, aml_name_decl("_HID", aml_string("%s", hid)));
    aml_append(dev, aml_name_decl("_SUB", aml_string("MTP08998")));
    aml_append(dev, aml_name_decl("_UID", aml_int(uid)));
    return dev;
}

static void qcom8998_acpi_smmu(Aml *scope, const char *name, unsigned uid,
                               unsigned bank, const uint32_t *irqs,
                               size_t count)
{
    Aml *dev = qcom8998_acpi_device(name, "QCOM0011", uid);
    Aml *crs = aml_resource_template();

    qcom8998_acpi_mem(crs, bank);
    for (size_t i = 0; i < count; i++) {
        qcom8998_acpi_irq(crs, irqs[i], AML_EDGE);
    }
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(scope, dev);
}

void qcom8998_display_acpi(Aml *scope)
{
    /*
     * RESI labels the descriptors in _CRS by position. The labels follow the
     * MSM8998 Windows ACPI ABI used by the original miniport.
     */
    static const struct {
        const char *name;
        const char *engine;
        unsigned bank;
        uint32_t gsiv;
    } resources[] = {
        { "MDP_REGS", "DISPLAY", BANK_MDSS },
        { "VSYNC_INTERRUPT", "DISPLAY", 0, MDSS_GSIV },
        { "GFX_REGS", "GRAPHICS", BANK_ADRENO },
        { "GFX_INTERRUPT", "GRAPHICS", 0, ADRENO_GSIV },
        { "VIDEO_REGS", "VIDEO", BANK_VIDEO },
        { "VIDEO_VMEM_REGS", "VIDEO", BANK_VIDEO_VMEM },
        { "VIDC_INTERRUPT", "VIDEO", 0, VIDEO_GSIV },
        { "SE_CTRL_REGS", "DISPLAY", BANK_MDSS_SE },
    };
    static const uint32_t mdss_smmu_irqs[] = {
        296, 295, 298, 299, 300, 276, 277, 279, 280,
        281, 282, 283, 284, 285, 286, 287, 288,
    };
    static const uint32_t adreno_smmu_irqs[] = { 366, 361, 362 };
    /* Scalar memory and scheduling capabilities from the OEMSK board REGR. */
    static const struct {
        const char *name;
        uint32_t value;
    } adapter_settings[] = {
        /* The synthetic SCM restores the probe's non-secure MDSS state. */
        { "DisableTzMDSSRestore", 0 },
        { "UseLowPTForGfxPerProcess", 1 },
        { "GPU64bAddrEnabled", 1 },
        { "MaxPreemptionOffsets", 0x80 },
        { "MaxRequiredDmaQueueEntry", 8 },
        { "SupportsSecureInAperture", 1 },
        { "ZeroFlagSupportInPTE", 1 },
        { "SupportsCacheCoherency", 1 },
    };
    Aml *dev = qcom8998_acpi_device("QGFX", "QCOM007C", 0);
    Aml *monitor = aml_device("MON0");
    Aml *cls = aml_package(3);
    Aml *crs = aml_resource_template();
    Aml *resi = aml_method("RESI", 0, AML_NOTSERIALIZED);
    Aml *resource_info = aml_package(2 + ARRAY_SIZE(resources));
    Aml *regr = aml_method("REGR", 0, AML_NOTSERIALIZED);
    Aml *settings = aml_package(ARRAY_SIZE(adapter_settings) + 1);
    Aml *graphics_settings = aml_package(2);
    Aml *dcvs_settings = aml_package(2);
    Aml *dcvs_enable = aml_package(2);

    aml_append(cls, aml_int(3));
    aml_append(cls, aml_int(0));
    aml_append(cls, aml_int(0));
    aml_append(dev, aml_name_decl("_CLS", cls));
    aml_append(dev, aml_name_decl("_CID", aml_string("ACPIQCOM007C")));
    aml_append(dev, aml_name_decl("_HRV", aml_int(0x7c)));
    /* Match the native display adapter's primary monitor child. */
    aml_append(monitor, aml_name_decl("_ADR", aml_int(0)));
    aml_append(dev, monitor);
    aml_append(resource_info, aml_int(3));
    aml_append(resource_info, aml_int(0));
    for (unsigned i = 0; i < ARRAY_SIZE(resources); i++) {
        Aml *entry = aml_package(3);

        if (resources[i].gsiv) {
            qcom8998_acpi_irq(crs, resources[i].gsiv, AML_LEVEL);
        } else {
            qcom8998_acpi_mem(crs, resources[i].bank);
        }
        aml_append(entry, aml_string("RESOURCE"));
        aml_append(entry, aml_string("%s", resources[i].name));
        aml_append(entry, aml_string("%s", resources[i].engine));
        aml_append(resource_info, entry);
    }
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(resi, aml_return(resource_info));
    aml_append(dev, resi);

    for (unsigned i = 0; i < ARRAY_SIZE(adapter_settings); i++) {
        Aml *entry = aml_package(2);

        aml_append(entry, aml_string("%s", adapter_settings[i].name));
        aml_append(entry, aml_int(adapter_settings[i].value));
        aml_append(settings, entry);
    }
    /*
     * Retain the board's GRAPHICS/DCVS section. QCDX's cached REGR lookup
     * requires a subkey list before it will read even the root values.
     * The probe has no DVFS controller or GPU utilization accounting. Do
     * not enable the native DCVS path with its uninitialized control state.
     */
    aml_append(dcvs_enable, aml_string("Enable"));
    aml_append(dcvs_enable, aml_int(0));
    aml_append(dcvs_settings, aml_string("DCVS"));
    aml_append(dcvs_settings, dcvs_enable);
    aml_append(graphics_settings, aml_string("GRAPHICS"));
    aml_append(graphics_settings, dcvs_settings);
    aml_append(settings, graphics_settings);
    aml_append(regr, aml_return(settings));
    aml_append(dev, regr);

    qcom8998_acpi_smmu(scope, "QSM0", 0, BANK_MDSS_SMMU, mdss_smmu_irqs,
                     ARRAY_SIZE(mdss_smmu_irqs));
    qcom8998_acpi_smmu(scope, "QSM3", 3, BANK_ADRENO_SMMU, adreno_smmu_irqs,
                     ARRAY_SIZE(adreno_smmu_irqs));
    /*
     * The native SCM interfaces are published only after TrEE starts.
     * Like the OEMSK DSDT, these enumerators have no MMIO resources.
     */
    aml_append(scope, qcom8998_acpi_device("SCM0", "QCOM0013", 0));
    aml_append(scope, qcom8998_acpi_device("TREE", "QCOM00BF", 0));
    aml_append(scope, dev);
}

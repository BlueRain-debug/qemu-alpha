/*
 * MSM8998 display bring-up for the Windows QCDX miniport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "fpu/softfloat.h"
#include "qapi/error.h"
#include "hw/acpi/aml-build.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/display/qcom-8998-display.h"
#include "migration/vmstate.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "ui/console.h"
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
#define MDSS_CTL_TOP 0x14
#define MDSS_CTL_TOP_INTF_SHIFT 4
#define MDSS_CTL_TOP_CMD_MODE BIT(17)
#define MDSS_CTL_FLUSH 0x18
#define MDSS_CTL_START 0x1c
#define MDSS_CTL_START_REQUEST BIT(0)
#define MDSS_CTL_SW_RESET 0x30
#define MDSS_CTL_SW_RESET_REQUEST BIT(0)
#define MDSS_CTL_LAYER_EXT 0x40
#define MDSS_CTL_LAYER_EXT2 0x70
#define MDSS_CTL_BORDER_OUT BIT(24)
#define MDSS_CTL_FLUSH_MIXER(index) BIT(6 + (index))
#define MDSS_HW_INTR_STATUS 0x10
#define MDSS_MDP_INTR BIT(0)
#define MDSS_DSI0_INTR BIT(4)
#define MDSS_DSI1_INTR BIT(5)
#define MDP_INTR_EN 0x1010
#define MDP_INTR_STATUS 0x1014
#define MDP_INTR_CLEAR 0x1018
#define MDP_PP_DONE_IRQ(index) BIT(8 + (index))
#define MDP_PP_RD_PTR_IRQ(index) BIT(12 + (index))
#define MDP_PP_BASE 0x71000
#define MDP_PP_STRIDE 0x800
#define MDP_PP_SYNC_CONFIG 0x004
#define MDP_PP_SYNC_HEIGHT 0x008
#define MDP_PP_COUNTER_EN BIT(19)
#define MDP_LM_BASE 0x45000
#define MDP_LM_STRIDE 0x1000
#define MDP_LM_OUT_SIZE 0x004
#define MDP_LM_BORDER_COLOR_0 0x008
#define MDP_LM_BORDER_COLOR_1 0x010
#define MDP_MAX_OUTPUT_SIZE 4096
/* The two OEMSK command-mode panels refresh at 60 Hz. */
#define MDP_PANEL_REFRESH_HZ 60

/* DSI 6G has a version word before the common DSI registers. */
#define MDSS_DSI0_BASE 0x94000
#define MDSS_DSI1_BASE 0x96000
#define MDSS_DSI_COUNT 2
#define MDP_INTF_DSI0 2
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
#define ADRENO_RBBM_INT_CLEAR_CMD (0x37 * 4)
#define ADRENO_RBBM_INT_0_MASK (0x38 * 4)
#define ADRENO_RBBM_INT_0_MASKED_STATUS (0x4e0 * 4)
#define ADRENO_RBBM_INT_0_STATUS (0x4e1 * 4)
#define ADRENO_IRQ_CP_IB2 BIT(13)
#define ADRENO_IRQ_CP_IB1 BIT(14)
#define ADRENO_IRQ_CP_RB BIT(15)
/* CP_INTERRUPT uses bits 29..31 for IB2/IB1/RB, unlike RBBM status. */
#define ADRENO_CP_INTERRUPT_SHIFT 16
#define ADRENO_CP_INTERRUPT_MASK ((ADRENO_IRQ_CP_IB2 | ADRENO_IRQ_CP_IB1 | \
                                   ADRENO_IRQ_CP_RB) << \
                                  ADRENO_CP_INTERRUPT_SHIFT)
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
/* A5xx command-processor registers, converted from dwords to bytes. */
#define ADRENO_CP_RB_BASE (0x800 * 4)
#define ADRENO_CP_RB_CNTL (0x802 * 4)
#define ADRENO_CP_RB_RPTR_ADDR (0x804 * 4)
#define ADRENO_CP_RB_RPTR (0x806 * 4)
#define ADRENO_CP_RB_WPTR (0x807 * 4)
#define ADRENO_CP_PFP_ME_CNTL (0x832 * 4)
#define ADRENO_CP_IB1_BASE (0xb1f * 4)
#define ADRENO_CP_IB1_BUFSZ (0xb21 * 4)
#define ADRENO_CP_IB_REGISTER_STRIDE (3 * 4)
/* The ring may call IB1, which may call IB2. */
#define ADRENO_CP_IB_STREAMS 3
/* An internal draw-state stream may execute inside IB2. */
#define ADRENO_CP_STREAMS (ADRENO_CP_IB_STREAMS + 1)
#define ADRENO_CP_IB_SIZE_MASK 0xfffff
/* Yield long submissions so a CPU MMIO kick cannot monopolize the main loop. */
#define ADRENO_CP_PACKET_BUDGET 4096
#define ADRENO_CP_WORD_BUDGET 16384
#define ADRENO_CP_DISPATCH_DELAY_NS 1000
#define ADRENO_CP_PROTECT_BASE 0x880
#define ADRENO_CP_PROTECT_COUNT 32
#define ADRENO_CP_PROTECT_CNTL (0x8a0 * 4)
#define ADRENO_CP_PROTECT_ENABLE BIT(0)
#define ADRENO_CP_PROTECT_WRITE BIT(29)
#define ADRENO_CP_RB_NO_UPDATE BIT(27)
#define ADRENO_CP_RB_SWAP_MASK (3U << 16)
#define ADRENO_CP_RB_SIZE_MASK 0x3f
#define ADRENO_CP_RB_MAX_SIZE_LOG2_QWORDS 15
#define ADRENO_PM4_TYPE4 0x40000000
#define ADRENO_PM4_TYPE_MASK 0xf0000000
#define ADRENO_PM4_TYPE4_MAX_PAYLOAD 127
#define ADRENO_PM4_TYPE4_RESERVED BIT(26)
#define ADRENO_PM4_TYPE7 0x70000000
#define ADRENO_PM4_TYPE7_MASK 0xff004000
#define ADRENO_PM4_NOP 0x10
#define ADRENO_PM4_DRAW_PRED_ENABLE_GLOBAL 0x19
#define ADRENO_PM4_DRAW_PRED_ENABLE_LOCAL 0x1a
#define ADRENO_PM4_SKIP_IB2_GLOBAL 0x1d
#define ADRENO_PM4_SKIP_IB2_LOCAL 0x23
#define ADRENO_PM4_SET_VISIBILITY_OVERRIDE 0x64
#define ADRENO_PM4_WAIT_FOR_ME 0x13
#define ADRENO_PM4_WAIT_FOR_IDLE 0x26
#define ADRENO_PM4_LOAD_STATE4 0x30
#define ADRENO_PM4_WAIT_REG_MEM 0x3c
#define ADRENO_PM4_DRAW_INDX_OFFSET 0x38
#define ADRENO_PM4_MEM_WRITE 0x3d
#define ADRENO_PM4_INDIRECT_BUFFER 0x3f
#define ADRENO_PM4_INTERRUPT 0x40
#define ADRENO_PM4_SET_DRAW_STATE 0x43
#define ADRENO_PM4_COND_EXEC 0x44
#define ADRENO_PM4_EVENT_WRITE 0x46
#define ADRENO_PM4_ME_INIT 0x48
#define ADRENO_PM4_SMMU_TABLE_UPDATE 0x53
#define ADRENO_PM4_SET_PROTECTED_MODE 0x5f
#define ADRENO_PM4_SET_SECURE_MODE 0x66
#define ADRENO_PM4_PREEMPT_ENABLE_GLOBAL 0x69
#define ADRENO_PM4_PREEMPT_ENABLE_LOCAL 0x6a
#define ADRENO_PM4_SET_RENDER_MODE 0x6c
#define ADRENO_RENDER_BYPASS 1
/* Global configuration words, in the PM4 register index space. */
#define ADRENO_RB_MODE_CNTL 0xcc6
#define ADRENO_RB_CCU_CNTL 0xcc7
#define ADRENO_UNKNOWN_0C82 0xc82
#define ADRENO_PC_MODE_CNTL 0xd02
#define ADRENO_PC_TESSFACTOR_ADDR_LO 0xd08
#define ADRENO_PC_TESSFACTOR_ADDR_HI 0xd09
#define ADRENO_HLSQ_TIMEOUT_THRESHOLD_0 0xe00
#define ADRENO_HLSQ_TIMEOUT_THRESHOLD_1 0xe01
#define ADRENO_HLSQ_MODE_CNTL 0xe06
#define ADRENO_VFD_MODE_CNTL 0xe42
#define ADRENO_VPC_DBG_ECO_CNTL 0xe60
#define ADRENO_VPC_MODE_CNTL 0xe62
#define ADRENO_SP_MODE_CNTL 0xec2
#define ADRENO_TPL1_MODE_CNTL 0xf02
#define ADRENO_UCHE_INVALIDATE_MIN 0xe91
#define ADRENO_UCHE_INVALIDATE 0xe95
/* GRAS/RB/PC/VFD/VPC/SP/TPL1/HLSQ context configuration. */
#define ADRENO_CONTEXT_REG_BASE 0xe000
#define ADRENO_CONTEXT_REG_END 0xe800
/* A5xx draw-state group IDs occupy bits 24..28 of each control word. */
#define ADRENO_CP_DRAW_GROUPS 32
/*
 * Bounded software state stores, not an advertised hardware capacity.
 * A5xx shader units contain 16 64-bit instructions. Constant vectors and
 * samplers use four dwords; texture descriptors use twelve dwords.
 */
#define ADRENO_LOAD_BLOCKS 14
#define ADRENO_LOAD_TYPES 3
#define ADRENO_LOAD_MAX_UNITS 1024
#define ADRENO_LOAD_MAX_WORDS 32
#define ADRENO_LOAD_SOURCE_DIRECT 0
#define ADRENO_LOAD_SOURCE_INDIRECT 2
#define ADRENO_LOAD_SOURCE_UBO 3
#define ADRENO_UBO_STAGE_STRIDE 20
#define ADRENO_UBO_STAGE_SLOTS 16
#define ADRENO_DRAW_STATE_COUNT_MASK 0xffff
#define ADRENO_DRAW_STATE_DIRTY BIT(16)
#define ADRENO_DRAW_STATE_DISABLE BIT(17)
#define ADRENO_DRAW_STATE_DISABLE_ALL BIT(18)
#define ADRENO_DRAW_STATE_LOAD_IMMED BIT(19)
#define ADRENO_DRAW_STATE_GROUP_MASK (0x1fU << 24)
#define ADRENO_DRAW_STATE_VALID (ADRENO_DRAW_STATE_COUNT_MASK | \
    ADRENO_DRAW_STATE_DIRTY | ADRENO_DRAW_STATE_DISABLE | \
    ADRENO_DRAW_STATE_DISABLE_ALL | ADRENO_DRAW_STATE_LOAD_IMMED | \
    ADRENO_DRAW_STATE_GROUP_MASK)
#define ADRENO_EVENT_CACHE_FLUSH_TS 0x04
#define ADRENO_EVENT_WT_DONE_TS 0x08
#define ADRENO_EVENT_RB_DONE_TS 0x16
#define ADRENO_EVENT_CCU_INVALIDATE_DEPTH 0x18
#define ADRENO_EVENT_CCU_INVALIDATE_COLOR 0x19
#define ADRENO_EVENT_CCU_FLUSH_DEPTH_TS 0x1c
#define ADRENO_EVENT_CCU_FLUSH_COLOR_TS 0x1d
#define ADRENO_EVENT_CCU_RESOLVE 0x1e
#define ADRENO_EVENT_LRZ_FLUSH_INVALIDATE 0x26
/* PM4 register indices for the fast-clear path. */
#define ADRENO_RB_CNTL 0xe140
#define ADRENO_RB_RENDER_CNTL 0xe141
#define ADRENO_RB_RAS_MSAA_CNTL 0xe142
#define ADRENO_RB_DEST_MSAA_CNTL 0xe143
#define ADRENO_RB_MRT_BUF_INFO 0xe152
#define ADRENO_RB_MRT_STRIDE 7
#define ADRENO_RB_BLIT_CNTL 0xe210
#define ADRENO_RB_RESOLVE_MIN 0xe211
#define ADRENO_RB_RESOLVE_MAX 0xe212
#define ADRENO_RB_RESOLVE_CNTL 0xe213
#define ADRENO_RB_BLIT_DST 0xe214
#define ADRENO_RB_BLIT_DST_PITCH 0xe216
#define ADRENO_RB_BLIT_DST_ARRAY_PITCH 0xe217
#define ADRENO_RB_CLEAR_COLOR 0xe218
#define ADRENO_RB_CLEAR_CNTL 0xe21c
#define ADRENO_RB_BYPASS BIT(17)
#define ADRENO_RB_FAST_CLEAR BIT(1)
#define ADRENO_RB_CLEAR_ALL_COMPONENTS (0xf << 4)
#define ADRENO_RB_RGBA8_UNORM 0x30
#define ADRENO_RB_COLOR_SWAP_MASK (3U << 13)
/* Limit synchronous clear work, including its preflight span allocation. */
#define ADRENO_CLEAR_MAX_BYTES (64U << 20)
/* A540 has 1 MiB of private, on-chip graphics memory. */
#define ADRENO_GMEM_SIZE (1U << 20)
#define ADRENO_ME_INIT_CONTEXTS BIT(0)
#define ADRENO_ME_INIT_ERROR_DETECTION BIT(1)
#define ADRENO_ME_INIT_HEADER_DUMP BIT(3)
/* PM4 register indices, in dwords. Data RAM is beyond the upload window. */
#define ADRENO_GPMU_INST_RAM_BASE 0x8800
#define ADRENO_GPMU_DATA_RAM_BASE 0x9800
#define ADRENO_GPMU_DATA_OFFSET \
    ((ADRENO_GPMU_DATA_RAM_BASE - ADRENO_GPMU_INST_RAM_BASE) * 4)
#define ADRENO_GPMU_CM3_SYSRESET (0xa8d8 * 4)
#define ADRENO_GPMU_GENERAL_0 (0xa8e0 * 4)
#define ADRENO_GPMU_GENERAL_1 (0xa8e1 * 4)
#define ADRENO_GPMU_RESET_ASSERT BIT(0)
/* A540 limits-management startup mailbox in the GPMU data RAM. */
#define ADRENO_GPMU_INIT_MAGIC (ADRENO_GPMU_DATA_RAM_BASE + 5)
#define ADRENO_GPMU_MSG_STATE (ADRENO_GPMU_DATA_RAM_BASE + 7)
#define ADRENO_GPMU_MSG_COMMAND (ADRENO_GPMU_DATA_RAM_BASE + 8)
#define ADRENO_GPMU_MSG_BYTES (ADRENO_GPMU_DATA_RAM_BASE + 10)
#define ADRENO_GPMU_MSG_PENDING BIT(31)
#define ADRENO_GPMU_MSG_POWER_CONFIG 1
#define ADRENO_GPMU_READY 0xbabeface
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
/* SMMUv2 global page 0, page 1, and context-bank registers. */
#define SMMU_GR0_CR0 0x0
#define SMMU_CR0_CLIENTPD BIT(0)
#define SMMU_CR0_GFIE BIT(2)
#define SMMU_CR0_USFCFG BIT(10)
#define SMMU_CR0_SMCFCFG BIT(21)
#define SMMU_GR0_GFSR 0x48
#define SMMU_GR0_GFSRRESTORE 0x4c
#define SMMU_GR0_GFSYNR0 0x50
#define SMMU_GR0_GFSYNR1 0x54
#define SMMU_GR0_GFSYNR2 0x58
#define SMMU_GFSR_ICF BIT(0)
#define SMMU_GFSR_USF BIT(1)
#define SMMU_GFSR_SMCF BIT(2)
#define SMMU_GFSR_UCBF BIT(3)
#define SMMU_GFSR_UUT BIT(8)
#define SMMU_FSR_MULTI BIT(31)
#define SMMU_GFSR_MASK (SMMU_FSR_MULTI | 0x1ff)
#define SMMU_GFSYNR0_WNR BIT(1)
#define SMMU_GFSYNR0_NS (BIT(4) | BIT(5))
#define SMMU_GR0_SMR 0x800
#define SMMU_GR0_S2CR 0xc00
#define SMMU_SMR_VALID BIT(31)
#define SMMU_SMR_ID_MASK 0x7fff
#define SMMU_S2CR_TYPE_TRANSLATE 0
#define SMMU_GR1_CBAR 0x1000
#define SMMU_GR1_CBA2R 0x1800
#define SMMU_GR1_CBFRSYNRA 0x1400
#define SMMU_CBAR_S1_S2_BYPASS 1
#define SMMU_CBA2R_VA64 BIT(0)
#define SMMU_ADRENO_CONTEXT_BASE 0x8000
#define SMMU_CONTEXT_STRIDE 0x1000
#define SMMU_ADRENO_CONTEXTS 2
#define SMMU_ADRENO_STREAMS 16
#define SMMU_ADRENO_CP_STREAM_ID 0
#define SMMU_CB_SCTLR 0
#define SMMU_CB_TCR2 0x10
#define SMMU_CB_TTBR0 0x20
#define SMMU_CB_TTBR1 0x28
#define SMMU_CB_TCR 0x30
#define SMMU_CB_CONTEXTIDR 0x34
#define SMMU_CB_TLBIALL 0x618
#define SMMU_CB_TLBSYNC 0x7f0
#define SMMU_CB_TLBSTATUS 0x7f4
#define SMMU_CB_FSR 0x58
#define SMMU_CB_FSRRESTORE 0x5c
#define SMMU_CB_FAR 0x60
#define SMMU_CB_FSYNR0 0x68
#define SMMU_CB_FSYNR1 0x6c
#define SMMU_FSR_TF BIT(1)
#define SMMU_FSR_AFF BIT(2)
#define SMMU_FSR_PF BIT(3)
#define SMMU_FSR_EF BIT(4)
#define SMMU_FSR_ASF BIT(7)
#define SMMU_FSR_UUT BIT(8)
#define SMMU_FSR_FORMAT_AARCH64 (2U << 9)
#define SMMU_FSR_FAULT_MASK (SMMU_FSR_MULTI | 0x1fe)
#define SMMU_FSYNR0_WNR BIT(4)
#define SMMU_FSYNR0_PTWF BIT(10)
#define SMMU_FSYNR0_NSATTR BIT(8)
#define SMMU_SCTLR_M BIT(0)
#define SMMU_SCTLR_CFIE BIT(6)
#define SMMU_SCTLR_CFCFG BIT(7)
#define SMMU_SCTLR_HUPCF BIT(8)
#define SMMU_TCR_EPD0 BIT(7)
#define SMMU_TCR_EPD1 BIT(23)
#define SMMU_TCR_TG0_4K 0
#define SMMU_TCR_TG1_4K 2
#define SMMU_TCR2_SEP_36BIT 1
#define SMMU_TCR2_SEP_UPSTREAM 7
#define SMMU_4K_SHIFT 12
#define SMMU_4K_LEVEL_BITS 9
#define SMMU_LONG_DESC_PA_BITS 48
#define SMMU_PTE_TYPE_MASK 3
#define SMMU_PTE_BLOCK 1
#define SMMU_PTE_TABLE_OR_PAGE 3
#define SMMU_PTE_READ_ONLY BIT_ULL(7)
#define SMMU_PTE_ACCESS_FLAG BIT_ULL(10)
#define SMMU_PTE_TABLE_READ_ONLY BIT_ULL(62)

typedef struct Qcom8998Bank {
    MemoryRegion region;
    Qcom8998Display *display;
    unsigned index;
    uint32_t *regs;
    hwaddr last_read_offset;
    uint32_t last_read_value;
} Qcom8998Bank;

typedef struct Qcom8998MdpTransfer {
    unsigned panel;
    unsigned width;
    unsigned height;
    uint32_t color;
    uint32_t flush;
} Qcom8998MdpTransfer;

typedef struct Qcom8998CommandStream {
    uint64_t base;
    unsigned position;
    unsigned end;
    unsigned parent_words;
    unsigned state_group; /* Group + 1, or zero for ring/ordinary IB. */
} Qcom8998CommandStream;

typedef struct Qcom8998DrawState {
    uint64_t address;
    uint32_t control;
    bool dirty;
} Qcom8998DrawState;

typedef struct Qcom8998StateUnit {
    uint32_t words[ADRENO_LOAD_MAX_WORDS];
    bool valid;
} Qcom8998StateUnit;

typedef struct Qcom8998ClearSpan {
    hwaddr address;
    unsigned length;
} Qcom8998ClearSpan;

typedef struct Qcom8998GmemSpan {
    hwaddr address;
    unsigned offset;
    unsigned length;
} Qcom8998GmemSpan;

struct Qcom8998Display {
    SysBusDevice parent_obj;
    Qcom8998Bank bank[BANK_COUNT];
    /* MDSS, Adreno, Venus, then Adreno SMMU global and two context IRQs. */
    qemu_irq irq[6];
    QEMUTimer mdss_vsync;
    QEMUTimer cp_resume;
    bool cp_wait_pending;
    QemuConsole *panel[MDSS_DSI_COUNT];
    Qcom8998MdpTransfer transfer[MDSS_CTL_COUNT];
    bool disable_macro_tiled_primary;
    bool me_initialized;
    bool register_protection;
    bool draw_predicate_global;
    bool draw_predicate_local;
    bool draw_predicate_pass;
    bool visibility_override;
    bool skip_ib2_global;
    bool skip_ib2_local;
    unsigned gpmu_image_dwords;
    Qcom8998CommandStream streams[ADRENO_CP_STREAMS];
    unsigned stream_count;
    Qcom8998DrawState draw_state[ADRENO_CP_DRAW_GROUPS];
    GArray *load_state[ADRENO_LOAD_BLOCKS][ADRENO_LOAD_TYPES];
    uint64_t shader_address[ADRENO_LOAD_BLOCKS];
    uint32_t render_mode[5];
    uint32_t *gmem;
};

static void qcom8998_adreno_update_irq(Qcom8998Display *s)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;

    qemu_set_irq(s->irq[1], (regs[ADRENO_RBBM_INT_0_STATUS / 4] &
                             regs[ADRENO_RBBM_INT_0_MASK / 4]) != 0);
}

static void qcom8998_mdss_update_irq(Qcom8998Bank *bank)
{
    uint32_t dsi0 = bank->regs[(MDSS_DSI0_BASE + DSI_INTR_CTRL) / 4];
    uint32_t dsi1 = bank->regs[(MDSS_DSI1_BASE + DSI_INTR_CTRL) / 4];
    uint32_t pending = 0;

    if (bank->regs[MDP_INTR_STATUS / 4] & bank->regs[MDP_INTR_EN / 4]) {
        pending |= MDSS_MDP_INTR;
    }
    if (dsi0 & (dsi0 >> 1) & DSI_IRQ_STATUS) {
        pending |= MDSS_DSI0_INTR;
    }
    if (dsi1 & (dsi1 >> 1) & DSI_IRQ_STATUS) {
        pending |= MDSS_DSI1_INTR;
    }
    bank->regs[MDSS_HW_INTR_STATUS / 4] = pending;
    qemu_set_irq(bank->display->irq[0], pending != 0);
}

static int qcom8998_mdss_ctl_panel(uint32_t top)
{
    unsigned intf = extract32(top, MDSS_CTL_TOP_INTF_SHIFT, 4);

    if (!(top & MDSS_CTL_TOP_CMD_MODE) || intf < MDP_INTF_DSI0 ||
        intf >= MDP_INTF_DSI0 + MDSS_DSI_COUNT) {
        return -1;
    }
    return intf - MDP_INTF_DSI0;
}

static uint32_t qcom8998_mdss_active_panels(Qcom8998Bank *bank)
{
    static const hwaddr dsi_bases[] = { MDSS_DSI0_BASE, MDSS_DSI1_BASE };
    const uint32_t *regs = bank->regs;
    const uint32_t dsi_cmd_enable = DSI_CTRL_ENABLE | DSI_CTRL_CMD_MODE_EN;
    uint32_t active = 0;

    for (unsigned ctl = 0; ctl < MDSS_CTL_COUNT; ctl++) {
        hwaddr base = MDSS_CTL_BASE + ctl * MDSS_CTL_STRIDE;
        int panel = qcom8998_mdss_ctl_panel(regs[(base + MDSS_CTL_TOP) / 4]);
        hwaddr pp;

        if (panel < 0) {
            continue;
        }
        pp = MDP_PP_BASE + panel * MDP_PP_STRIDE;
        if ((regs[(dsi_bases[panel] + DSI_CTRL) / 4] & dsi_cmd_enable) ==
            dsi_cmd_enable &&
            (regs[(pp + MDP_PP_SYNC_CONFIG) / 4] & MDP_PP_COUNTER_EN) &&
            (regs[(pp + MDP_PP_SYNC_HEIGHT) / 4] & 0xffff)) {
            active |= MDP_PP_RD_PTR_IRQ(panel);
        }
    }
    return active;
}

static void qcom8998_mdss_vsync_arm(Qcom8998Display *s)
{
    timer_mod_ns(&s->mdss_vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 NANOSECONDS_PER_SECOND / MDP_PANEL_REFRESH_HZ);
}

static void qcom8998_mdss_vsync_update(Qcom8998Bank *bank)
{
    Qcom8998Display *s = bank->display;
    uint32_t active = qcom8998_mdss_active_panels(bank);
    uint32_t pending = bank->regs[MDP_INTR_STATUS / 4] &
                       bank->regs[MDP_INTR_EN / 4];

    if (!active) {
        timer_del(&s->mdss_vsync);
    } else if (!(active & pending) && !timer_pending(&s->mdss_vsync)) {
        qcom8998_mdss_vsync_arm(s);
    }
}

static const GraphicHwOps qcom8998_panel_ops = {
    /* Command-mode updates are driven by CTL_START and the panel timer. */
};

static void qcom8998_mdss_queue_transfer(Qcom8998Bank *bank, unsigned ctl)
{
    Qcom8998MdpTransfer *transfer = &bank->display->transfer[ctl];
    const uint32_t *control =
        &bank->regs[(MDSS_CTL_BASE + ctl * MDSS_CTL_STRIDE) / 4];
    int panel = qcom8998_mdss_ctl_panel(control[MDSS_CTL_TOP / 4]);
    const uint32_t *mixer;
    uint32_t size, color_0, color_1;
    unsigned width, height;

    if (panel < 0 || transfer->flush ||
        !(control[MDSS_CTL_FLUSH / 4] & MDSS_CTL_FLUSH_MIXER(panel))) {
        return;
    }
    /* OEMSK routes LM0 and LM1 to DSI0 and DSI1, respectively. */
    if (control[panel] != MDSS_CTL_BORDER_OUT ||
        control[MDSS_CTL_LAYER_EXT / 4 + panel] ||
        control[MDSS_CTL_LAYER_EXT2 / 4 + panel]) {
        /* A source pipe is staged. The current model cannot fetch it. */
        return;
    }
    mixer = &bank->regs[(MDP_LM_BASE + panel * MDP_LM_STRIDE) / 4];
    size = mixer[MDP_LM_OUT_SIZE / 4];
    width = extract32(size, 0, 16);
    height = extract32(size, 16, 16);
    if (!width || !height ||
        width > MDP_MAX_OUTPUT_SIZE || height > MDP_MAX_OUTPUT_SIZE) {
        return;
    }

    /* Two 12-bit components per word: green, blue, red, alpha. */
    color_0 = mixer[MDP_LM_BORDER_COLOR_0 / 4];
    color_1 = mixer[MDP_LM_BORDER_COLOR_1 / 4];
    *transfer = (Qcom8998MdpTransfer) {
        .panel = panel,
        .width = width,
        .height = height,
        .color = (extract32(color_1, 4, 8) << 16) |
                 (extract32(color_0, 4, 8) << 8) |
                 extract32(color_0, 20, 8),
        .flush = control[MDSS_CTL_FLUSH / 4],
    };
}

static uint32_t qcom8998_mdss_transfer(Qcom8998Display *s, uint32_t active)
{
    uint32_t pending = 0;

    for (unsigned ctl = 0; ctl < MDSS_CTL_COUNT; ctl++) {
        Qcom8998MdpTransfer *transfer = &s->transfer[ctl];
        unsigned panel = transfer->panel;
        hwaddr base = MDSS_CTL_BASE + ctl * MDSS_CTL_STRIDE;
        uint32_t *control = &s->bank[BANK_MDSS].regs[base / 4];
        DisplaySurface *surface;

        if (!transfer->flush || !(active & MDP_PP_RD_PTR_IRQ(panel))) {
            continue;
        }
        if (!s->panel[panel]) {
            s->panel[panel] = qemu_graphic_console_create(DEVICE(s), panel,
                                                        &qcom8998_panel_ops, s);
        }
        qemu_console_resize(s->panel[panel], transfer->width, transfer->height);
        surface = qemu_console_surface(s->panel[panel]);
        for (unsigned y = 0; y < transfer->height; y++) {
            uint32_t *row = (uint32_t *)(surface_data(surface) +
                                        y * surface_stride(surface));

            for (unsigned x = 0; x < transfer->width; x++) {
                row[x] = transfer->color;
            }
        }
        qemu_console_update_full(s->panel[panel]);
        trace_qcom8998_mdss_transfer_done(ctl, panel, transfer->width,
                                         transfer->height, transfer->color);
        control[MDSS_CTL_FLUSH / 4] &= ~transfer->flush;
        control[MDSS_CTL_START / 4] &= ~MDSS_CTL_START_REQUEST;
        transfer->flush = 0;
        pending |= MDP_PP_DONE_IRQ(panel);
    }
    return pending;
}

static void qcom8998_mdss_vsync(void *opaque)
{
    Qcom8998Display *s = opaque;
    Qcom8998Bank *bank = &s->bank[BANK_MDSS];
    uint32_t pending = qcom8998_mdss_active_panels(bank);

    /*
     * Both read pointers share a level interrupt. If the guest has not
     * drained the previous refresh, coalesce this tick. Latching one panel
     * again while the guest acknowledges the other can hold the line high
     * indefinitely. The acknowledge or mask update restarts the timer.
     */
    if (pending & bank->regs[MDP_INTR_STATUS / 4] &
        bank->regs[MDP_INTR_EN / 4]) {
        return;
    }
    if (pending) {
        /*
         * A panel sync advances the read pointer once per refresh.
         * The raw status latches even when the interrupt is masked.
         */
        bank->regs[MDP_INTR_STATUS / 4] |=
            pending | qcom8998_mdss_transfer(s, pending);
        qcom8998_mdss_update_irq(bank);
        qcom8998_mdss_vsync_arm(s);
    }
}

static void qcom8998_mdp_write(Qcom8998Bank *bank, hwaddr offset,
                               uint32_t value)
{
    switch (offset) {
    case MDP_INTR_EN:
        bank->regs[MDP_INTR_EN / 4] = value;
        qcom8998_mdss_update_irq(bank);
        qcom8998_mdss_vsync_update(bank);
        return;
    case MDP_INTR_CLEAR:
        bank->regs[MDP_INTR_STATUS / 4] &= ~value;
        qcom8998_mdss_update_irq(bank);
        qcom8998_mdss_vsync_update(bank);
        return;
    case MDP_INTR_STATUS:
        return;
    }
    if (offset >= MDSS_CTL_BASE &&
        offset < MDSS_CTL_BASE + MDSS_CTL_COUNT * MDSS_CTL_STRIDE) {
        hwaddr reg = (offset - MDSS_CTL_BASE) % MDSS_CTL_STRIDE;
        unsigned ctl = (offset - MDSS_CTL_BASE) / MDSS_CTL_STRIDE;

        if (reg == MDSS_CTL_SW_RESET) {
            if (value & MDSS_CTL_SW_RESET_REQUEST) {
                /* Only this control path is reset. */
                memset(&bank->regs[(offset - reg) / 4], 0, MDSS_CTL_REG_SIZE);
                bank->display->transfer[ctl].flush = 0;
                qcom8998_mdss_vsync_update(bank);
            }
            return;
        }
        if (reg == MDSS_CTL_TOP) {
            if (qcom8998_mdss_ctl_panel(bank->regs[offset / 4]) !=
                qcom8998_mdss_ctl_panel(value)) {
                bank->display->transfer[ctl].flush = 0;
                bank->regs[(offset - reg + MDSS_CTL_START) / 4] = 0;
            }
            bank->regs[offset / 4] = value;
            qcom8998_mdss_vsync_update(bank);
            return;
        }
        if (reg == MDSS_CTL_START && (value & MDSS_CTL_START_REQUEST)) {
            qcom8998_mdss_queue_transfer(bank, ctl);
        }
    }
    bank->regs[offset / 4] = value;
    if (offset >= MDP_PP_BASE &&
        offset < MDP_PP_BASE + MDSS_DSI_COUNT * MDP_PP_STRIDE) {
        hwaddr reg = (offset - MDP_PP_BASE) % MDP_PP_STRIDE;

        if (reg == MDP_PP_SYNC_CONFIG || reg == MDP_PP_SYNC_HEIGHT) {
            qcom8998_mdss_vsync_update(bank);
        }
    }
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
    if (offset == DSI_CTRL) {
        qcom8998_mdss_vsync_update(bank);
    }
}

static void qcom8998_gpmu_set_reset(Qcom8998Bank *bank, uint32_t value)
{
    uint32_t *regs = bank->regs;
    uint32_t stack = regs[ADRENO_GPMU_INST_RAM_BASE];
    uint32_t entry = regs[ADRENO_GPMU_INST_RAM_BASE + 1];
    uint32_t bytes = regs[ADRENO_GPMU_MSG_BYTES];
    unsigned image_dwords = bank->display->gpmu_image_dwords;
    bool ready = false;

    regs[ADRENO_GPMU_CM3_SYSRESET / 4] = value & ADRENO_GPMU_RESET_ASSERT;
    /*
     * Interpret the startup mailbox without executing Cortex-M3 firmware.
     * Require a loaded Thumb reset vector, a stack in the data RAM, and a
     * valid power-config message before setting the ready word.
     * Thermal and adaptive voltage algorithms are outside this model.
     */
    if (!(value & ADRENO_GPMU_RESET_ASSERT) && image_dwords >= 2 &&
        stack >= ADRENO_GPMU_DATA_OFFSET &&
        stack <= ADRENO_GPMU_DATA_OFFSET * 2 && !(stack & 3) &&
        (entry & 1) && entry / 4 >= 2 && entry / 4 < image_dwords &&
        regs[ADRENO_GPMU_INIT_MAGIC] == ADRENO_GPMU_READY &&
        (regs[ADRENO_GPMU_MSG_STATE] & ADRENO_GPMU_MSG_PENDING) &&
        regs[ADRENO_GPMU_MSG_COMMAND] == ADRENO_GPMU_MSG_POWER_CONFIG &&
        bytes && !(bytes & 3) && bytes <= ADRENO_GPMU_DATA_OFFSET -
        (ADRENO_GPMU_MSG_BYTES - ADRENO_GPMU_DATA_RAM_BASE + 2) * 4) {
        ready = true;
    }
    regs[ADRENO_GPMU_GENERAL_0 / 4] = ready ? ADRENO_GPMU_READY : 0;
    regs[ADRENO_GPMU_GENERAL_1 / 4] = 0;
    trace_qcom8998_gpmu_reset(value, image_dwords, entry, bytes, ready);
}

static bool qcom8998_adreno_power_write(Qcom8998Bank *bank, hwaddr offset,
                                       uint32_t value)
{
    hwaddr status;

    switch (offset) {
    case ADRENO_GPMU_CM3_SYSRESET:
        qcom8998_gpmu_set_reset(bank, value);
        return true;
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

static uint64_t qcom8998_reg_pair(const uint32_t *regs, hwaddr offset)
{
    return deposit64(regs[offset / 4], 32, 32, regs[offset / 4 + 1]);
}

static bool qcom8998_ram_buffer(hwaddr address, size_t size, bool write)
{
    hwaddr length = size, translated;
    MemoryRegion *mr;

    mr = address_space_translate(&address_space_memory, address, &translated,
                                 &length, write, MEMTXATTRS_UNSPECIFIED);
    return memory_region_is_ram(mr) && !memory_region_is_ram_device(mr) &&
           !memory_region_is_rom(mr) && length == size;
}

static bool qcom8998_ram_access(hwaddr address, void *data, size_t size,
                               bool write)
{
    if (!qcom8998_ram_buffer(address, size, write)) {
        return false;
    }
    if (write) {
        return address_space_write(&address_space_memory, address,
                                   MEMTXATTRS_UNSPECIFIED, data, size) ==
               MEMTX_OK;
    }
    return address_space_read(&address_space_memory, address,
                              MEMTXATTRS_UNSPECIFIED, data, size) == MEMTX_OK;
}

static uint32_t *qcom8998_smmu_context(Qcom8998Display *s, unsigned context)
{
    return s->bank[BANK_ADRENO_SMMU].regs +
           (SMMU_ADRENO_CONTEXT_BASE + context * SMMU_CONTEXT_STRIDE) / 4;
}

static void qcom8998_smmu_update_irq(Qcom8998Display *s)
{
    const uint32_t *regs = s->bank[BANK_ADRENO_SMMU].regs;

    qemu_set_irq(s->irq[3], (regs[SMMU_GR0_CR0 / 4] & SMMU_CR0_GFIE) &&
                 (regs[SMMU_GR0_GFSR / 4] & SMMU_GFSR_MASK));
    for (unsigned i = 0; i < SMMU_ADRENO_CONTEXTS; i++) {
        const uint32_t *cb = qcom8998_smmu_context(s, i);

        qemu_set_irq(s->irq[4 + i], (cb[SMMU_CB_SCTLR / 4] & SMMU_SCTLR_CFIE) &&
                     (cb[SMMU_CB_FSR / 4] & SMMU_FSR_FAULT_MASK));
    }
}

static bool qcom8998_smmu_global_fault(Qcom8998Display *s, uint32_t fault,
                                      bool write)
{
    uint32_t *regs = s->bank[BANK_ADRENO_SMMU].regs;

    if (regs[SMMU_GR0_GFSR / 4]) {
        regs[SMMU_GR0_GFSR / 4] |= SMMU_FSR_MULTI;
    } else {
        regs[SMMU_GR0_GFSR / 4] = fault;
        regs[SMMU_GR0_GFSYNR0 / 4] = SMMU_GFSYNR0_NS |
            (write ? SMMU_GFSYNR0_WNR : 0);
        regs[SMMU_GR0_GFSYNR1 / 4] = SMMU_ADRENO_CP_STREAM_ID;
        regs[SMMU_GR0_GFSYNR2 / 4] = 0;
    }
    qcom8998_smmu_update_irq(s);
    return false;
}

static bool qcom8998_smmu_context_fault(Qcom8998Display *s, unsigned context,
                                       uint64_t iova, bool write,
                                       uint32_t fault, uint32_t syndrome)
{
    uint32_t *regs = qcom8998_smmu_context(s, context);

    if (regs[SMMU_CB_FSR / 4]) {
        regs[SMMU_CB_FSR / 4] |= SMMU_FSR_MULTI;
    } else {
        regs[SMMU_CB_FSR / 4] = fault;
        regs[SMMU_CB_FAR / 4] = iova;
        regs[SMMU_CB_FAR / 4 + 1] = iova >> 32;
        regs[SMMU_CB_FSYNR0 / 4] = syndrome | SMMU_FSYNR0_NSATTR |
            (write ? SMMU_FSYNR0_WNR : 0);
        regs[SMMU_CB_FSYNR1 / 4] = 0;
        s->bank[BANK_ADRENO_SMMU].regs[SMMU_GR1_CBFRSYNRA / 4 + context] =
            SMMU_ADRENO_CP_STREAM_ID;
    }
    qcom8998_smmu_update_irq(s);
    return false;
}

enum {
    SMMU_CONTEXT_FAULT = -1,
    SMMU_CONTEXT_BYPASS = -2,
};

/* Stream 0 is the CP's privileged DMA stream. No TLB or stage 2 is modeled. */
static int qcom8998_cp_context_index(Qcom8998Display *s, uint32_t *fault)
{
    uint32_t *regs = s->bank[BANK_ADRENO_SMMU].regs;
    int match = -1;
    unsigned context, type;

    if (regs[SMMU_GR0_CR0 / 4] & SMMU_CR0_CLIENTPD) {
        return SMMU_CONTEXT_BYPASS;
    }

    for (unsigned i = 0; i < SMMU_ADRENO_STREAMS; i++) {
        uint32_t smr = regs[SMMU_GR0_SMR / 4 + i];
        uint32_t mask = (smr >> 16) & SMMU_SMR_ID_MASK;

        if (!(smr & SMMU_SMR_VALID) ||
            ((smr ^ SMMU_ADRENO_CP_STREAM_ID) & SMMU_SMR_ID_MASK & ~mask)) {
            continue;
        }
        if (match >= 0) {
            *fault = SMMU_GFSR_SMCF;
            return regs[SMMU_GR0_CR0 / 4] & SMMU_CR0_SMCFCFG ?
                   SMMU_CONTEXT_FAULT : SMMU_CONTEXT_BYPASS;
        }
        match = i;
    }
    if (match < 0) {
        *fault = SMMU_GFSR_USF;
        return regs[SMMU_GR0_CR0 / 4] & SMMU_CR0_USFCFG ?
               SMMU_CONTEXT_FAULT : SMMU_CONTEXT_BYPASS;
    }
    type = extract32(regs[SMMU_GR0_S2CR / 4 + match], 16, 2);
    if (type == 1) {
        return SMMU_CONTEXT_BYPASS;
    }
    if (type != SMMU_S2CR_TYPE_TRANSLATE) {
        *fault = type == 2 ? SMMU_GFSR_USF : SMMU_GFSR_ICF;
        return SMMU_CONTEXT_FAULT;
    }
    context = extract32(regs[SMMU_GR0_S2CR / 4 + match], 0, 8);
    if (context >= SMMU_ADRENO_CONTEXTS) {
        *fault = SMMU_GFSR_UCBF;
        return SMMU_CONTEXT_FAULT;
    }
    if (extract32(regs[SMMU_GR1_CBAR / 4 + context], 16, 2) !=
        SMMU_CBAR_S1_S2_BYPASS ||
        !(regs[SMMU_GR1_CBA2R / 4 + context] & SMMU_CBA2R_VA64)) {
        *fault = SMMU_GFSR_UUT;
        return SMMU_CONTEXT_FAULT;
    }
    return context;
}

static uint32_t *qcom8998_cp_context(Qcom8998Display *s)
{
    uint32_t fault;
    int context = qcom8998_cp_context_index(s, &fault);

    return context < 0 ? NULL : qcom8998_smmu_context(s, context);
}

static bool qcom8998_cp_translate(Qcom8998Display *s, uint64_t iova,
                                 bool write, hwaddr *out)
{
    static const unsigned pa_width[] = { 32, 36, 40 };
    const uint32_t *regs;
    uint32_t fault;
    int context = qcom8998_cp_context_index(s, &fault);
    unsigned pasize, va_bits;
    uint32_t tcr, tcr2;
    uint64_t table, pa_mask, va_mask, invalid_pa;
    uint64_t original_iova = iova;
    bool upper;

    if (context == SMMU_CONTEXT_FAULT) {
        return qcom8998_smmu_global_fault(s, fault, write);
    }
    if (context == SMMU_CONTEXT_BYPASS) {
        *out = iova;
        return true;
    }
    regs = qcom8998_smmu_context(s, context);
    if (regs[SMMU_CB_FSR / 4] &&
        !(regs[SMMU_CB_SCTLR / 4] & SMMU_SCTLR_HUPCF)) {
        return false;
    }
    if (!(regs[SMMU_CB_SCTLR / 4] & SMMU_SCTLR_M)) {
        *out = iova;
        return true;
    }
    tcr = regs[SMMU_CB_TCR / 4];
    tcr2 = regs[SMMU_CB_TCR2 / 4];
    if (regs[SMMU_CB_SCTLR / 4] & BIT(4)) {
        goto unsupported;
    }
    switch (extract32(tcr2, 15, 3)) {
    case SMMU_TCR2_SEP_36BIT:
        if (iova >> 36) {
            goto address_fault;
        }
        iova = sextract64(iova, 0, 36);
        break;
    case SMMU_TCR2_SEP_UPSTREAM:
        break;
    default:
        goto unsupported;
    }
    upper = iova >> 63;
    if (tcr & (upper ? SMMU_TCR_EPD1 : SMMU_TCR_EPD0)) {
        goto translation_fault;
    }
    if (upper ? extract32(tcr, 30, 2) != SMMU_TCR_TG1_4K :
                extract32(tcr, 14, 2) != SMMU_TCR_TG0_4K) {
        goto unsupported;
    }
    va_bits = 64 - extract32(tcr, upper ? 16 : 0, 6);
    pasize = extract32(tcr2, 0, 4);
    if (va_bits < 25 || va_bits > 48 || pasize >= ARRAY_SIZE(pa_width)) {
        goto unsupported;
    }
    va_mask = MAKE_64BIT_MASK(0, va_bits);
    if ((iova & ~va_mask) != (upper ? ~va_mask : 0)) {
        goto address_fault;
    }
    pa_mask = MAKE_64BIT_MASK(0, pa_width[pasize]);
    invalid_pa = MAKE_64BIT_MASK(0, SMMU_LONG_DESC_PA_BITS) & ~pa_mask;
    table = qcom8998_reg_pair(regs, upper ? SMMU_CB_TTBR1 : SMMU_CB_TTBR0);
    for (unsigned shift = SMMU_4K_SHIFT + SMMU_4K_LEVEL_BITS *
                          ((va_bits - SMMU_4K_SHIFT - 1) / SMMU_4K_LEVEL_BITS);
         ; shift -= SMMU_4K_LEVEL_BITS) {
        unsigned bits = MIN(SMMU_4K_LEVEL_BITS, va_bits - shift);
        unsigned index = extract64(iova, shift, bits);
        uint64_t pte;
        unsigned type;

        if (table & invalid_pa) {
            goto address_fault;
        }
        table &= pa_mask & ~MAKE_64BIT_MASK(0, bits + 3);
        if (!qcom8998_ram_access(table + index * sizeof(pte), &pte,
                                 sizeof(pte), false)) {
            return qcom8998_smmu_context_fault(s, context, original_iova,
                       write, SMMU_FSR_EF, SMMU_FSYNR0_PTWF);
        }
        pte = le64_to_cpu(pte);
        type = pte & SMMU_PTE_TYPE_MASK;
        if (shift > SMMU_4K_SHIFT && type == SMMU_PTE_TABLE_OR_PAGE) {
            if (write && (pte & SMMU_PTE_TABLE_READ_ONLY)) {
                goto permission_fault;
            }
            table = pte & ~MAKE_64BIT_MASK(0, SMMU_4K_SHIFT);
            continue;
        }
        if (type != (shift == SMMU_4K_SHIFT ? SMMU_PTE_TABLE_OR_PAGE :
                     SMMU_PTE_BLOCK) || shift == 39) {
            goto translation_fault;
        }
        if (pte & invalid_pa) {
            goto address_fault;
        }
        if (!(pte & SMMU_PTE_ACCESS_FLAG)) {
            return qcom8998_smmu_context_fault(s, context, original_iova,
                                              write, SMMU_FSR_AFF, 0);
        }
        if (write && (pte & SMMU_PTE_READ_ONLY)) {
            goto permission_fault;
        }
        *out = (pte & pa_mask & ~MAKE_64BIT_MASK(0, shift)) |
               extract64(iova, 0, shift);
        return true;
    }

unsupported:
    return qcom8998_smmu_context_fault(s, context, original_iova, write,
                                      SMMU_FSR_UUT, 0);
translation_fault:
    return qcom8998_smmu_context_fault(s, context, original_iova, write,
                                      SMMU_FSR_TF, 0);
address_fault:
    return qcom8998_smmu_context_fault(s, context, original_iova, write,
                                      SMMU_FSR_ASF, 0);
permission_fault:
    return qcom8998_smmu_context_fault(s, context, original_iova, write,
                                      SMMU_FSR_PF, 0);
}

static bool qcom8998_cp_word(Qcom8998Display *s, uint64_t iova, uint32_t *word,
                            bool write)
{
    hwaddr pa;
    uint32_t le = write ? cpu_to_le32(*word) : 0;

    if ((iova & 3) || !qcom8998_cp_translate(s, iova, write, &pa) ||
        !qcom8998_ram_access(pa, &le, sizeof(le), write)) {
        return false;
    }
    if (!write) {
        *word = le32_to_cpu(le);
    }
    return true;
}

static bool qcom8998_cp_register_write(Qcom8998Display *s, unsigned reg,
                                      unsigned count)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;

    if (!s->register_protection ||
        !(regs[ADRENO_CP_PROTECT_CNTL / 4] & ADRENO_CP_PROTECT_ENABLE)) {
        return true;
    }
    for (unsigned i = 0; i < ADRENO_CP_PROTECT_COUNT; i++) {
        uint32_t range = regs[ADRENO_CP_PROTECT_BASE + i];
        uint64_t size = BIT_ULL(extract32(range, 24, 5));
        uint64_t first = extract32(range, 2, 18) & ~(size - 1);

        if ((range & ADRENO_CP_PROTECT_WRITE) &&
            reg < first + size && first < reg + count) {
            return false;
        }
    }
    return true;
}

/* CP register indices also address the adjacent, separately owned SMMU. */
static bool qcom8998_cp_tlb_register(unsigned reg, bool write)
{
    const hwaddr first = banks[BANK_ADRENO_SMMU].base -
                         banks[BANK_ADRENO].base + SMMU_ADRENO_CONTEXT_BASE;
    hwaddr offset = (hwaddr)reg * sizeof(uint32_t);

    if (offset < first || offset >= first +
        SMMU_ADRENO_CONTEXTS * SMMU_CONTEXT_STRIDE) {
        return false;
    }
    offset = (offset - first) % SMMU_CONTEXT_STRIDE;
    return write ? offset == SMMU_CB_TLBIALL || offset == SMMU_CB_TLBSYNC :
                   offset == SMMU_CB_TLBSTATUS;
}

static bool qcom8998_cp_stored_register(unsigned reg)
{
    if ((reg >= ADRENO_GPMU_INST_RAM_BASE &&
         reg < ADRENO_GPMU_DATA_RAM_BASE) ||
        (reg >= ADRENO_CONTEXT_REG_BASE && reg < ADRENO_CONTEXT_REG_END) ||
        (reg >= ADRENO_UCHE_INVALIDATE_MIN &&
         reg <= ADRENO_UCHE_INVALIDATE)) {
        return true;
    }
    switch (reg) {
    case ADRENO_RB_MODE_CNTL:
    case ADRENO_RB_CCU_CNTL:
    case ADRENO_UNKNOWN_0C82:
    case ADRENO_PC_MODE_CNTL:
    case ADRENO_PC_TESSFACTOR_ADDR_LO:
    case ADRENO_PC_TESSFACTOR_ADDR_HI:
    case ADRENO_HLSQ_TIMEOUT_THRESHOLD_0:
    case ADRENO_HLSQ_TIMEOUT_THRESHOLD_1:
    case ADRENO_HLSQ_MODE_CNTL:
    case ADRENO_VFD_MODE_CNTL:
    case ADRENO_VPC_DBG_ECO_CNTL:
    case ADRENO_VPC_MODE_CNTL:
    case ADRENO_SP_MODE_CNTL:
    case ADRENO_TPL1_MODE_CNTL:
        return true;
    default:
        return false;
    }
}

static bool qcom8998_cp_type4(Qcom8998Display *s, uint32_t header,
                             unsigned count, uint64_t base, unsigned mask,
                             unsigned rptr)
{
    uint32_t payload[ADRENO_PM4_TYPE4_MAX_PAYLOAD];
    unsigned reg = extract32(header, 8, 18);

    /*
     * Type 4 can stage instruction RAM and 3D configuration. Latching
     * context state does not execute a draw or complete a rendering fence.
     * Other controls/status remain excluded, except coherent UCHE handling
     * explicitly validated below.
     */
    if (!s->me_initialized || !count || (header & ADRENO_PM4_TYPE4_RESERVED) ||
        !(ctpop32(extract32(header, 0, 8)) & 1) ||
        !(ctpop32(extract32(header, 8, 20)) & 1) ||
        !qcom8998_cp_register_write(s, reg, count)) {
        return false;
    }
    for (unsigned i = 0; i < count; i++) {
        if ((!qcom8998_cp_stored_register(reg + i) &&
             !qcom8998_cp_tlb_register(reg + i, true)) ||
            !qcom8998_cp_word(s, base + (((rptr + 1 + i) & mask) * 4),
                              &payload[i], false)) {
            return false;
        }
    }
    for (unsigned i = 0; i < count; i++) {
        if (reg + i == ADRENO_UNKNOWN_0C82 && payload[i]) {
            return false;
        }
        if (reg + i == ADRENO_UCHE_INVALIDATE && payload[i] != 0x12) {
            return false;
        }
    }
    /*
     * 0x12 invalidates the entire UCHE. All supported work reads coherent
     * RAM synchronously, and there is no UCHE or deferred cache writeback.
     * Range/other cache operations must not be inferred from this case.
     */
    for (unsigned i = 0; i < count; i++) {
        if (!qcom8998_cp_tlb_register(reg + i, true)) {
            s->bank[BANK_ADRENO].regs[reg + i] = payload[i];
        }
        /*
         * TLBIALL/TLBSYNC complete synchronously: there is no GPU TLB or
         * outstanding page walk. Never index the Adreno array with an SMMU
         * register, and never discard fault state or translation tables.
         */
    }
    if (reg >= ADRENO_GPMU_INST_RAM_BASE &&
        reg < ADRENO_GPMU_DATA_RAM_BASE &&
        reg <= ADRENO_GPMU_INST_RAM_BASE + s->gpmu_image_dwords) {
        s->gpmu_image_dwords = MAX(s->gpmu_image_dwords,
                                 reg + count - ADRENO_GPMU_INST_RAM_BASE);
    }
    return true;
}

static bool qcom8998_cp_mem_write(Qcom8998Display *s, unsigned count,
                                 uint64_t base, unsigned mask, unsigned rptr)
{
    g_autofree uint32_t *data = NULL;
    g_autofree hwaddr *addresses = NULL;
    uint32_t low, high;
    uint64_t destination;
    unsigned words;

    /* A5xx supplies a 64-bit destination followed by one or more dwords. */
    if (!s->me_initialized || count < 3 ||
        !qcom8998_cp_word(s, base + (((rptr + 1) & mask) * 4), &low, false) ||
        !qcom8998_cp_word(s, base + (((rptr + 2) & mask) * 4), &high, false)) {
        return false;
    }
    words = count - 2;
    destination = deposit64(low, 32, 32, high);
    if ((destination & 3) || destination > UINT64_MAX - words * 4) {
        return false;
    }

    /*
     * count is bounded by the current command stream before entering here.
     * Stage the payload and validate every destination before modifying RAM,
     * including when the source and destination alias or cross a page.
     */
    data = g_new(uint32_t, words);
    addresses = g_new(hwaddr, words);
    for (unsigned i = 0; i < words; i++) {
        if (!qcom8998_cp_word(s, base + (((rptr + 3 + i) & mask) * 4),
                              &data[i], false) ||
            !qcom8998_cp_translate(s, destination + i * 4, true,
                                   &addresses[i]) ||
            !qcom8998_ram_buffer(addresses[i], sizeof(data[i]), true)) {
            return false;
        }
        data[i] = cpu_to_le32(data[i]);
    }
    for (unsigned i = 0; i < words; i++) {
        if (address_space_write(&address_space_memory, addresses[i],
                                 MEMTXATTRS_UNSPECIFIED, &data[i],
                                 sizeof(data[i])) != MEMTX_OK) {
            return false;
        }
    }
    return true;
}

static bool qcom8998_cp_smmu_update(Qcom8998Display *s, const uint32_t *payload)
{
    static const unsigned pa_width[] = { 32, 36, 40 };
    uint32_t *regs = qcom8998_cp_context(s);
    unsigned pasize, va_bits, shift, root_bytes;
    uint64_t address;

    if (s->register_protection || !regs ||
        !(regs[SMMU_CB_SCTLR / 4] & SMMU_SCTLR_M)) {
        return false;
    }
    pasize = extract32(regs[SMMU_CB_TCR2 / 4], 0, 4);
    va_bits = 64 - extract32(regs[SMMU_CB_TCR / 4], 0, 6);
    if (pasize >= ARRAY_SIZE(pa_width) || va_bits < 25 || va_bits > 48 ||
        extract32(regs[SMMU_CB_TCR / 4], 14, 2) != SMMU_TCR_TG0_4K) {
        return false;
    }
    address = deposit64(payload[0], 32, 16, payload[1]);
    shift = SMMU_4K_SHIFT + SMMU_4K_LEVEL_BITS *
            ((va_bits - SMMU_4K_SHIFT - 1) / SMMU_4K_LEVEL_BITS);
    root_bytes = 8U << MIN(SMMU_4K_LEVEL_BITS, va_bits - shift);
    if ((address >> pa_width[pasize]) || (address & (root_bytes - 1)) ||
        !qcom8998_ram_buffer(address, root_bytes, false)) {
        return false;
    }

    /*
     * A5xx's three-dword packet updates the CP context's TTBR0 and ASID.
     * Keep TTBR1 intact for privileged ring and fence mappings. This model
     * walks tables on each DMA, so no cached translations need invalidation.
     */
    regs[SMMU_CB_TTBR0 / 4] = payload[0];
    regs[SMMU_CB_TTBR0 / 4 + 1] = payload[1];
    regs[SMMU_CB_CONTEXTIDR / 4] = payload[2];
    return true;
}

static bool qcom8998_cp_draw_state(Qcom8998Display *s, unsigned count,
                                  uint64_t base, unsigned mask, unsigned rptr)
{
    Qcom8998DrawState pending[ADRENO_CP_DRAW_GROUPS];

    if (!s->me_initialized || !count || count % 3) {
        return false;
    }
    memcpy(pending, s->draw_state, sizeof(pending));
    for (unsigned offset = 0; offset < count; offset += 3) {
        uint32_t words[3];
        uint32_t control;
        uint64_t address;
        unsigned group, length;

        for (unsigned i = 0; i < ARRAY_SIZE(words); i++) {
            if (!qcom8998_cp_word(s,
                    base + (((rptr + 1 + offset + i) & mask) * 4),
                    &words[i], false)) {
                return false;
            }
        }
        control = words[0];
        address = deposit64(words[1], 32, 32, words[2]);
        group = extract32(control, 24, 5);
        length = control & ADRENO_DRAW_STATE_COUNT_MASK;
        if ((control & ~ADRENO_DRAW_STATE_VALID) ||
            (control & ADRENO_DRAW_STATE_LOAD_IMMED) || (address & 3) ||
            address > UINT64_MAX - length * 4) {
            /* Immediate loads need their own resumable execution path. */
            return false;
        }
        if (control & ADRENO_DRAW_STATE_DISABLE_ALL) {
            if (control != ADRENO_DRAW_STATE_DISABLE_ALL || address) {
                return false;
            }
            memset(pending, 0, sizeof(pending));
        } else {
            if ((control & ADRENO_DRAW_STATE_DISABLE) && (length || address)) {
                return false;
            }
            pending[group].dirty = length &&
                (pending[group].dirty ||
                 pending[group].address != address ||
                 (pending[group].control & ADRENO_DRAW_STATE_COUNT_MASK) !=
                     length ||
                 (pending[group].control & ADRENO_DRAW_STATE_DISABLE) ||
                 (control & ADRENO_DRAW_STATE_DIRTY));
            pending[group].address = address;
            pending[group].control = control & ~ADRENO_DRAW_STATE_GROUP_MASK;
        }
    }
    /* Publish a complete packet only; a later bad triple changes no groups. */
    memcpy(s->draw_state, pending, sizeof(pending));
    for (unsigned group = 0; group < ADRENO_CP_DRAW_GROUPS; group++) {
        trace_qcom8998_cp_draw_state(group, pending[group].control,
                                    pending[group].address);
    }
    return true;
}

static bool qcom8998_cp_dispatch_draw_state(Qcom8998Display *s)
{
    for (unsigned i = 0; i < ADRENO_CP_DRAW_GROUPS; i++) {
        const Qcom8998DrawState *state = &s->draw_state[i];

        if (state->dirty && !(state->control & ADRENO_DRAW_STATE_DISABLE)) {
            assert(s->stream_count < ADRENO_CP_STREAMS);
            s->streams[s->stream_count++] = (Qcom8998CommandStream) {
                .base = state->address,
                .end = state->control & ADRENO_DRAW_STATE_COUNT_MASK,
                .state_group = i + 1,
            };
            return true;
        }
    }
    return false;
}

static bool qcom8998_cp_load_state(Qcom8998Display *s, unsigned count,
                                  uint64_t base, unsigned mask, unsigned rptr)
{
    uint32_t header[3];
    unsigned block, type, source, destination, units, unit_words, words;
    uint64_t address;
    g_autofree uint32_t *pending = NULL;
    GArray *state;

    if (!s->me_initialized || count < ARRAY_SIZE(header)) {
        return false;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(header); i++) {
        if (!qcom8998_cp_word(s, base + (((rptr + 1 + i) & mask) * 4),
                              &header[i], false)) {
            return false;
        }
    }
    destination = extract32(header[0], 0, 14);
    source = extract32(header[0], 16, 2);
    block = extract32(header[0], 18, 4);
    units = extract32(header[0], 22, 10);
    type = extract32(header[1], 0, 2);
    address = deposit64(header[1] & ~3U, 32, 32, header[2]);
    if ((header[0] & (3U << 14)) || type >= ADRENO_LOAD_TYPES || !units ||
        destination >= ADRENO_LOAD_MAX_UNITS ||
        units > ADRENO_LOAD_MAX_UNITS - destination) {
        return false;
    }
    if (block == 6 && (type == 1 || type == 2)) {
        /* Shared UBO auxiliary words / 64-bit addresses, staged only. */
        unit_words = 2;
    } else if (block <= 5) {
        unit_words = type == 0 ? 4 : type == 1 ? 12 : 2;
    } else if (block >= 8 && block < ADRENO_LOAD_BLOCKS) {
        unit_words = type == 0 ? ADRENO_LOAD_MAX_WORDS : type == 1 ? 4 : 2;
    } else {
        /* SSBOs and other undocumented blocks need their own layout. */
        return false;
    }
    words = units * unit_words;
    if (source == ADRENO_LOAD_SOURCE_UBO) {
        const GArray *table = s->load_state[6][2];
        const Qcom8998StateUnit *descriptor;
        unsigned slot = address >> 16;
        unsigned offset = address & 0xffff;
        unsigned index;
        uint64_t buffer;

        /*
         * QCDX uses source 3 to load graphics constants from the shared UBO
         * address table. EXT_SRC_ADDR packs slot:16 and byte offset:16.
         * Keep this path bounded to vec4-aligned, non-bindless constants.
         * Auxiliary block-6 words remain staged; no shader UBO instruction
         * or preemption state is inferred from their contents.
         */
        if (type != 1 || block < 8 || block > 12 || address >> 32 ||
            slot >= ADRENO_UBO_STAGE_SLOTS || (offset & 15) || count != 3) {
            return false;
        }
        index = (block - 8) * ADRENO_UBO_STAGE_STRIDE + slot;
        if (!table || index >= table->len) {
            return false;
        }
        descriptor = &g_array_index(table, Qcom8998StateUnit, index);
        buffer = deposit64(descriptor->words[0], 32, 32, descriptor->words[1]);
        if (!descriptor->valid || !buffer || (buffer & 15) ||
            buffer > UINT64_MAX - offset) {
            return false;
        }
        address = buffer + offset;
    }
    if ((source == ADRENO_LOAD_SOURCE_DIRECT &&
         (count != ARRAY_SIZE(header) + words || address)) ||
        ((source == ADRENO_LOAD_SOURCE_INDIRECT ||
          source == ADRENO_LOAD_SOURCE_UBO) &&
         (count != ARRAY_SIZE(header) ||
          address > UINT64_MAX - words * sizeof(uint32_t))) ||
        (source != ADRENO_LOAD_SOURCE_DIRECT &&
         source != ADRENO_LOAD_SOURCE_INDIRECT &&
         source != ADRENO_LOAD_SOURCE_UBO)) {
        return false;
    }
    pending = g_new(uint32_t, words);
    for (unsigned i = 0; i < words; i++) {
        uint64_t input = source == ADRENO_LOAD_SOURCE_DIRECT ?
            base + (((rptr + 1 + ARRAY_SIZE(header) + i) & mask) * 4) :
            address + i * sizeof(uint32_t);

        if (!qcom8998_cp_word(s, input, &pending[i], false)) {
            return false;
        }
    }
    /* A later source fault must leave every previously loaded unit intact. */
    state = s->load_state[block][type];
    if (!state) {
        state = g_array_new(false, true, sizeof(Qcom8998StateUnit));
        s->load_state[block][type] = state;
    }
    if (state->len < destination + units) {
        g_array_set_size(state, destination + units);
    }
    for (unsigned i = 0; i < units; i++) {
        Qcom8998StateUnit *unit = &g_array_index(state, Qcom8998StateUnit,
                                                destination + i);

        memcpy(unit->words, pending + i * unit_words,
               unit_words * sizeof(uint32_t));
        unit->valid = true;
    }
    if (block >= 8 && type == 0 && !destination) {
        s->shader_address[block] = address;
    }
    trace_qcom8998_cp_load_state(block, type, destination, units, unit_words,
        address,
        g_array_index(state, Qcom8998StateUnit, destination).words[0],
        g_array_index(state, Qcom8998StateUnit,
                      destination + units - 1).words[unit_words - 1]);
    /* Loading state does not execute a draw or complete its fence. */
    return true;
}

static uint32_t qcom8998_color_swap(uint32_t color, unsigned swap)
{
    static const unsigned channel_order[][4] = {
        { 0, 1, 2, 3 }, { 2, 1, 0, 3 },
        { 3, 0, 1, 2 }, { 3, 2, 1, 0 },
    };
    uint32_t packed = 0;

    for (unsigned channel = 0; channel < 4; channel++) {
        packed |= extract32(color, channel_order[swap][channel] * 8, 8) <<
                  (channel * 8);
    }
    return cpu_to_le32(packed);
}

static uint32_t qcom8998_color_unswap(uint32_t color, unsigned swap)
{
    static const unsigned channel_order[][4] = {
        { 0, 1, 2, 3 }, { 2, 1, 0, 3 },
        { 3, 0, 1, 2 }, { 3, 2, 1, 0 },
    };
    uint32_t result = 0;

    for (unsigned c = 0; c < 4; c++) {
        result |= extract32(color, c * 8, 8) << (channel_order[swap][c] * 8);
    }
    return result;
}

static bool qcom8998_sysmem_surface(Qcom8998Display *s, unsigned x1,
                                    unsigned y1, unsigned x2, unsigned y2,
                                    GArray *spans, uint32_t **pixels)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    uint64_t base = qcom8998_reg_pair(regs, (ADRENO_RB_MRT_BUF_INFO + 3) * 4);
    uint64_t pitch = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 1] << 6;
    unsigned width = x2 - x1;
    unsigned swap = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 13, 2);
    uint32_t temporary[(1U << SMMU_4K_SHIFT) / 4];

    if (!width || y1 == y2) {
        return true;
    }
    *pixels = g_new(uint32_t, width * (y2 - y1));
    for (unsigned y = y1; y < y2; y++) {
        uint64_t address = base + y * pitch + x1 * 4;
        unsigned remaining = width * 4;
        unsigned offset = (y - y1) * width * 4;

        while (remaining) {
            Qcom8998GmemSpan span;
            unsigned page_left = (1U << SMMU_4K_SHIFT) -
                                 extract64(address, 0, SMMU_4K_SHIFT);

            span.length = MIN(remaining, page_left);
            span.offset = offset;
            if (!qcom8998_cp_translate(s, address, true, &span.address) ||
                !qcom8998_ram_buffer(span.address, span.length, true) ||
                address_space_read(&address_space_memory, span.address,
                    MEMTXATTRS_UNSPECIFIED, temporary,
                    span.length) != MEMTX_OK) {
                return false;
            }
            for (unsigned i = 0; i < span.length / 4; i++) {
                (*pixels)[offset / 4 + i] = qcom8998_color_unswap(
                    le32_to_cpu(temporary[i]), swap);
            }
            g_array_append_val(spans, span);
            address += span.length;
            offset += span.length;
            remaining -= span.length;
        }
    }
    return true;
}

static bool qcom8998_sysmem_publish(Qcom8998Display *s, const GArray *spans,
                                    const uint32_t *pixels)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned swap = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 13, 2);
    uint32_t temporary[(1U << SMMU_4K_SHIFT) / 4];

    for (unsigned i = 0; i < spans->len; i++) {
        const Qcom8998GmemSpan *span = &g_array_index(spans,
                                                     Qcom8998GmemSpan, i);

        for (unsigned p = 0; p < span->length / 4; p++) {
            temporary[p] = qcom8998_color_swap(pixels[span->offset / 4 + p],
                                               swap);
        }
        if (address_space_write(&address_space_memory, span->address,
            MEMTXATTRS_UNSPECIFIED, temporary, span->length) != MEMTX_OK) {
            return false;
        }
    }
    return true;
}

/*
 * GMEM is private: keep logical RGBA8 texels rather than expose a guessed
 * hardware tile swizzle. RB_CNTL defines the bin dimensions; a fast clear
 * can retain the previous sysmem MRT pitch. RB_WINDOW_OFFSET locates the
 * bin in framebuffer coordinates. External tiled/UBWC memory is separate.
 */
static bool qcom8998_gmem_layout(Qcom8998Display *s, unsigned target,
                                 unsigned minimum, unsigned maximum,
                                 unsigned *base, unsigned *width)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    unsigned height = extract32(regs[ADRENO_RB_CNTL], 9, 8) << 5;
    unsigned origin = regs[0xe1d0];
    unsigned ox = origin & 0xffff, oy = origin >> 16;
    uint64_t offset;

    *width = extract32(regs[ADRENO_RB_CNTL], 0, 8) << 5;
    if (target >= 8 || !*width || !height ||
        (regs[ADRENO_RB_CNTL] & ~0x1feffU) ||
        regs[ADRENO_RB_RAS_MSAA_CNTL] || regs[ADRENO_RB_DEST_MSAA_CNTL] ||
        ox + *width > 0x10000 || oy + height > 0x10000 ||
        (minimum & 0xffff) > (maximum & 0xffff) ||
        (minimum >> 16) > (maximum >> 16) ||
        (minimum & 0xffff) < ox || (minimum >> 16) < oy ||
        (maximum & 0xffff) >= ox + *width ||
        (maximum >> 16) >= oy + height ||
        (regs[mrt] & ~ADRENO_RB_COLOR_SWAP_MASK) !=
            (ADRENO_RB_RGBA8_UNORM | (2U << 8)) ||
        (regs[ADRENO_RB_RENDER_CNTL] & (BIT(16 + target) | BIT(24 + target)))) {
        return false;
    }
    offset = qcom8998_reg_pair(regs, (mrt + 3) * 4);
    if ((offset & 3) || offset >= ADRENO_GMEM_SIZE ||
        (uint64_t)*width * height * 4 > ADRENO_GMEM_SIZE - offset) {
        return false;
    }
    *base = offset / 4;
    return true;
}

static bool qcom8998_gmem_clear(Qcom8998Display *s, unsigned target,
                                unsigned minimum, unsigned maximum)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned base, width;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;

    if (!qcom8998_gmem_layout(s, target, minimum, maximum, &base, &width)) {
        return false;
    }
    for (unsigned y = minimum >> 16; y <= maximum >> 16; y++) {
        for (unsigned x = minimum & 0xffff; x <= (maximum & 0xffff); x++) {
            s->gmem[base + (y - oy) * width + x - ox] =
                regs[ADRENO_RB_CLEAR_COLOR];
        }
    }
    trace_qcom8998_gmem_transfer(true, target, base * 4, 0,
                                minimum, maximum);
    return true;
}

static bool qcom8998_gmem_resolve(Qcom8998Display *s, unsigned target,
                                  unsigned minimum, unsigned maximum)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned base, width, mrt;
    unsigned x1 = minimum & 0xffff, y1 = minimum >> 16;
    unsigned x2 = maximum & 0xffff, y2 = maximum >> 16;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;
    uint64_t destination, pitch, layer, last;
    g_autoptr(GArray) spans = g_array_new(false, false,
                                        sizeof(Qcom8998GmemSpan));
    uint32_t pixels[(1U << SMMU_4K_SHIFT) / 4];

    if (!qcom8998_gmem_layout(s, target, minimum, maximum, &base, &width)) {
        return false;
    }
    mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    destination = qcom8998_reg_pair(regs, ADRENO_RB_BLIT_DST * 4);
    pitch = (uint64_t)regs[ADRENO_RB_BLIT_DST_PITCH] << 6;
    layer = (uint64_t)regs[ADRENO_RB_BLIT_DST_ARRAY_PITCH] << 6;
    last = y2 * pitch + (uint64_t)(x2 + 1) * 4;
    /* Zero layer stride is valid for a single layer, as used by QCDX. */
    if ((destination & 3) || pitch < (uint64_t)(x2 + 1) * 4 ||
        (layer && last > layer) || destination > UINT64_MAX - last) {
        return false;
    }
    for (unsigned y = y1; y <= y2; y++) {
        uint64_t address = destination + y * pitch + x1 * 4;
        unsigned offset = (base + (y - oy) * width + x1 - ox) * 4;
        unsigned remaining = (x2 - x1 + 1) * 4;

        while (remaining) {
            Qcom8998GmemSpan span;
            unsigned page_left = (1U << SMMU_4K_SHIFT) -
                                 extract64(address, 0, SMMU_4K_SHIFT);

            span.length = MIN(remaining, page_left);
            span.offset = offset;
            if (!qcom8998_cp_translate(s, address, true, &span.address) ||
                !qcom8998_ram_buffer(span.address, span.length, true)) {
                return false;
            }
            g_array_append_val(spans, span);
            address += span.length;
            offset += span.length;
            remaining -= span.length;
        }
    }
    /* Preflight the complete destination before publishing any row. */
    for (unsigned i = 0; i < spans->len; i++) {
        const Qcom8998GmemSpan *span = &g_array_index(spans,
                                                     Qcom8998GmemSpan, i);

        for (unsigned p = 0; p < span->length / 4; p++) {
            pixels[p] = qcom8998_color_swap(s->gmem[span->offset / 4 + p],
                                             extract32(regs[mrt], 13, 2));
        }
        if (address_space_write(&address_space_memory, span->address,
                                MEMTXATTRS_UNSPECIFIED, pixels,
                                span->length) != MEMTX_OK) {
            return false;
        }
    }
    trace_qcom8998_gmem_transfer(false, target, base * 4, destination,
                                minimum, maximum);
    return true;
}

static bool qcom8998_fill_rect(Qcom8998Display *s, unsigned target,
                                unsigned minimum, unsigned maximum,
                                uint32_t color)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned x1 = extract32(minimum, 0, 16);
    unsigned y1 = extract32(minimum, 16, 16);
    unsigned x2 = extract32(maximum, 0, 16);
    unsigned y2 = extract32(maximum, 16, 16);
    unsigned mrt, width, height, swap;
    uint64_t base, pitch, layer_size, last;
    uint32_t packed_color;
    uint32_t pixels[(1U << SMMU_4K_SHIFT) / sizeof(uint32_t)];
    g_autoptr(GArray) spans = g_array_new(false, false,
                                        sizeof(Qcom8998ClearSpan));

    if (target >= 8 || x2 < x1 || y2 < y1) {
        return false;
    }
    mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    /*
     * Implement row-major RGBA8 first. Tiled/UBWC targets, GMEM, depth,
     * multisampling and partial component masks require their own layouts.
     * Do not complete the resolve or its timestamp for those modes.
     */
    if ((regs[mrt] & ~ADRENO_RB_COLOR_SWAP_MASK) != ADRENO_RB_RGBA8_UNORM ||
        (regs[ADRENO_RB_RENDER_CNTL] & (BIT(16 + target) | BIT(24 + target)))) {
        return false;
    }
    base = qcom8998_reg_pair(regs, (mrt + 3) * 4);
    pitch = (uint64_t)regs[mrt + 1] << 6;
    layer_size = (uint64_t)regs[mrt + 2] << 6;
    width = x2 - x1 + 1;
    height = y2 - y1 + 1;
    last = (uint64_t)y2 * pitch + (uint64_t)(x2 + 1) * sizeof(uint32_t);
    if ((base & 3) || pitch < (uint64_t)(x2 + 1) * sizeof(uint32_t) ||
        last > layer_size || base > UINT64_MAX - last ||
        (uint64_t)width * height * sizeof(uint32_t) > ADRENO_CLEAR_MAX_BYTES) {
        return false;
    }

    /* Resolve every destination before changing RAM, including later rows. */
    for (unsigned y = y1; y <= y2; y++) {
        uint64_t address = base + y * pitch + x1 * sizeof(uint32_t);
        unsigned remaining = width * sizeof(uint32_t);

        while (remaining) {
            Qcom8998ClearSpan span;
            unsigned page_left = (1U << SMMU_4K_SHIFT) -
                                 extract64(address, 0, SMMU_4K_SHIFT);

            span.length = MIN(remaining, page_left);
            if (!qcom8998_cp_translate(s, address, true, &span.address) ||
                !qcom8998_ram_buffer(span.address, span.length, true)) {
                return false;
            }
            g_array_append_val(spans, span);
            address += span.length;
            remaining -= span.length;
        }
    }
    /* CLEAR_COLOR is packed before the MRT's byte-order permutation. */
    swap = extract32(regs[mrt], 13, 2);
    packed_color = qcom8998_color_swap(color, swap);
    for (unsigned i = 0; i < ARRAY_SIZE(pixels); i++) {
        pixels[i] = packed_color;
    }
    for (unsigned i = 0; i < spans->len; i++) {
        const Qcom8998ClearSpan *span = &g_array_index(spans,
                                                      Qcom8998ClearSpan, i);

        if (address_space_write(&address_space_memory, span->address,
                                MEMTXATTRS_UNSPECIFIED, pixels,
                                span->length) != MEMTX_OK) {
            return false;
        }
    }
    return true;
}

static bool qcom8998_ccu_clear(Qcom8998Display *s)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned target = regs[ADRENO_RB_BLIT_CNTL];
    unsigned minimum = regs[ADRENO_RB_RESOLVE_MIN];
    unsigned maximum = regs[ADRENO_RB_RESOLVE_MAX];

    if (regs[ADRENO_RB_CNTL] != ADRENO_RB_BYPASS) {
        if ((regs[ADRENO_RB_RESOLVE_CNTL] == 0 ||
             regs[ADRENO_RB_RESOLVE_CNTL] == 4) &&
            regs[ADRENO_RB_CLEAR_CNTL] ==
                (ADRENO_RB_FAST_CLEAR | ADRENO_RB_CLEAR_ALL_COMPONENTS)) {
            /* FAST_CLEAR also overrides the previous tile's store direction. */
            return qcom8998_gmem_clear(s, target, minimum, maximum);
        }
        if (regs[ADRENO_RB_RESOLVE_CNTL] == 4 &&
            !regs[ADRENO_RB_CLEAR_CNTL]) {
            return qcom8998_gmem_resolve(s, target, minimum, maximum);
        }
        return false;
    }
    if (regs[ADRENO_RB_RAS_MSAA_CNTL] ||
        regs[ADRENO_RB_DEST_MSAA_CNTL] || regs[ADRENO_RB_RESOLVE_CNTL] ||
        regs[ADRENO_RB_CLEAR_CNTL] !=
            (ADRENO_RB_FAST_CLEAR | ADRENO_RB_CLEAR_ALL_COMPONENTS) ||
        !qcom8998_fill_rect(s, target, minimum, maximum,
                            regs[ADRENO_RB_CLEAR_COLOR])) {
        return false;
    }
    trace_qcom8998_ccu_clear(target,
        qcom8998_reg_pair(regs, (ADRENO_RB_MRT_BUF_INFO +
                         target * ADRENO_RB_MRT_STRIDE + 3) * 4),
        minimum & 0xffff, minimum >> 16,
        (maximum & 0xffff) - (minimum & 0xffff) + 1,
        (maximum >> 16) - (minimum >> 16) + 1,
        regs[ADRENO_RB_CLEAR_COLOR]);
    return true;
}

/* A bounded, side-effect-free IR3 arithmetic subset. */
#define ADRENO_SHADER_GPRS 244
#define ADRENO_SHADER_STEPS 512

typedef struct Qcom8998ShaderRegs {
    uint32_t full[ADRENO_SHADER_GPRS];
    uint16_t half[ADRENO_SHADER_GPRS];
    bool full_valid[ADRENO_SHADER_GPRS];
    bool half_valid[ADRENO_SHADER_GPRS];
    int16_t address;
    bool address_valid;
    uint32_t input[64];
    bool input_valid[64];
    unsigned steps;
    uint32_t instruction[2];
} Qcom8998ShaderRegs;

static bool qcom8998_shader_word(Qcom8998Display *s, unsigned block,
                                 unsigned type, unsigned index,
                                 uint32_t *value)
{
    const GArray *state = s->load_state[block][type];
    unsigned words = type ? 4 : 32;
    const Qcom8998StateUnit *unit;

    if (!state || index / words >= state->len) {
        return false;
    }
    unit = &g_array_index(state, Qcom8998StateUnit, index / words);
    if (!unit->valid) {
        return false;
    }
    *value = unit->words[index % words];
    return true;
}

static bool qcom8998_shader_finite(uint32_t value)
{
    unsigned exponent = extract32(value, 23, 8);

    /* Denormal and nonfinite arithmetic needs separate hardware policy. */
    return exponent != 255 && (exponent || !(value & 0x7fffffff));
}

static bool qcom8998_shader_float_source(Qcom8998Display *s, unsigned block,
                                         Qcom8998ShaderRegs *r,
                                         unsigned encoding, unsigned increment,
                                         uint32_t *value)
{
    unsigned index;

    if ((encoding & 0x3800) == 0x1000) {
        index = (encoding & 0x7ff) + increment;
        if (!qcom8998_shader_word(s, block, 1, index, value)) {
            return false;
        }
    } else {
        index = (encoding & 0xff) + increment;
        if ((encoding & 0x3b00) || index >= ADRENO_SHADER_GPRS ||
            !r->full_valid[index]) {
            return false;
        }
        *value = r->full[index];
    }
    if (encoding & BIT(14)) {
        *value &= ~BIT(31);
    }
    if (encoding & BIT(15)) {
        *value ^= BIT(31);
    }
    return qcom8998_shader_finite(*value);
}

static bool qcom8998_shader_execute(Qcom8998Display *s, unsigned block,
                                    Qcom8998ShaderRegs *r)
{
    float_status fp = { 0 };

    set_float_rounding_mode(float_round_nearest_even, &fp);
    for (unsigned pc = 0; pc < ADRENO_SHADER_STEPS; pc++) {
        uint32_t low, high;
        uint64_t instruction;
        unsigned category, dst;

        r->steps = pc + 1;
        if (!qcom8998_shader_word(s, block, 0, pc * 2, &low) ||
            !qcom8998_shader_word(s, block, 0, pc * 2 + 1, &high)) {
            return false;
        }
        instruction = deposit64(low, 32, 32, high);
        r->instruction[0] = low;
        r->instruction[1] = high;
        category = extract64(instruction, 61, 3);
        dst = extract64(instruction, 32, 8);
        if (category == 0) {
            uint64_t scheduling = BIT_ULL(44) | BIT_ULL(59) | BIT_ULL(60);

            if ((instruction & ~scheduling) == (6ULL << 55)) {
                return true; /* END, not the padding after it. */
            }
            if (!(instruction & ~(scheduling | (7ULL << 40)))) {
                continue; /* NOP and its sequential delay slots. */
            }
        } else if (category == 2) {
            unsigned source = low & 0xff;
            unsigned immediate = low >> 16;
            unsigned opcode = extract64(instruction, 53, 6);
            unsigned repeat = extract64(instruction, 40, 2);

            if (opcode == 0 || opcode == 3 || opcode == 57) {
                if (!(instruction & BIT_ULL(52)) ||
                    (instruction & (BIT_ULL(42) | BIT_ULL(46))) ||
                    extract64(instruction, 48, 3) ||
                    dst + repeat >= ADRENO_SHADER_GPRS) {
                    return false;
                }
                for (unsigned n = 0; n <= repeat; n++) {
                    unsigned inc1 = instruction & BIT_ULL(43) ? n : 0;
                    unsigned inc2 = instruction & BIT_ULL(51) ? n : 0;
                    uint32_t a, b, value;

                    if (opcode == 57) {
                        unsigned input = (low & 0x7ff) + inc1;

                        /* Barycentric interpolation of uniform inputs only. */
                        if (block != 12 || (low & 0xf800) != 0x2000 ||
                            input >= ARRAY_SIZE(r->input) ||
                            !r->input_valid[input]) {
                            return false;
                        }
                        value = r->input[input];
                    } else {
                        if ((instruction & BIT_ULL(47)) ||
                            !qcom8998_shader_float_source(s, block, r,
                                                         low & 0xffff, inc1,
                                                         &a) ||
                            !qcom8998_shader_float_source(s, block, r,
                                                         low >> 16, inc2, &b)) {
                            return false;
                        }
                        value = opcode == 3 ? float32_mul(a, b, &fp) :
                                              float32_add(a, b, &fp);
                    }
                    if (!qcom8998_shader_finite(value)) {
                        return false;
                    }
                    r->full[dst + n] = value;
                    r->full_valid[dst + n] = true;
                }
                continue;
            }

            /* Full-width SHL.B, register source and small immediate shift. */
            if (extract64(instruction, 53, 6) != 54 ||
                extract64(instruction, 40, 3) ||
                extract64(instruction, 46, 5) ||
                !(instruction & BIT_ULL(52)) || (low & 0xff00) ||
                (immediate & ~0x7ffU) != 0x2000 ||
                (immediate & 0x7ff) > 31 || dst >= ADRENO_SHADER_GPRS ||
                source >= ADRENO_SHADER_GPRS || !r->full_valid[source]) {
                return false;
            }
            r->full[dst] = r->full[source] << (immediate & 31);
            r->full_valid[dst] = true;
            continue;
        } else if (category == 3) {
            unsigned repeat = extract64(instruction, 40, 2);
            unsigned src2 = extract64(instruction, 47, 8);

            if (extract64(instruction, 55, 4) != 7 ||
                (instruction & (BIT_ULL(13) | BIT_ULL(42) | BIT_ULL(46))) ||
                dst + repeat >= ADRENO_SHADER_GPRS) {
                return false;
            }
            for (unsigned n = 0; n <= repeat; n++) {
                unsigned enc1 = low & 0x1fff;
                unsigned enc3 = extract32(low, 16, 13);
                unsigned inc1 = instruction & BIT_ULL(43) ? n : 0;
                unsigned inc2 = instruction & BIT_ULL(15) ? n : 0;
                unsigned inc3 = instruction & BIT_ULL(29) ? n : 0;
                uint32_t a, b, c, value;

                enc1 |= instruction & BIT_ULL(14) ? BIT(15) : 0;
                enc3 |= instruction & BIT_ULL(31) ? BIT(15) : 0;
                if (!qcom8998_shader_float_source(s, block, r,
                                                  enc1, inc1, &a) ||
                    !qcom8998_shader_float_source(s, block, r,
                        src2 | (instruction & BIT_ULL(30) ? BIT(15) : 0),
                        inc2, &b) ||
                    !qcom8998_shader_float_source(s, block, r,
                                                  enc3, inc3, &c)) {
                    return false;
                }
                /* IR3 MAD is an unfused operation: round the product first. */
                value = float32_add(float32_mul(a, b, &fp), c, &fp);
                if (!qcom8998_shader_finite(value)) {
                    return false;
                }
                r->full[dst + n] = value;
                r->full_valid[dst + n] = true;
            }
            continue;
        } else if (category == 1) {
            unsigned src_type = extract64(instruction, 50, 3);
            unsigned dst_type = extract64(instruction, 46, 3);
            unsigned mode = extract64(instruction, 53, 2);
            unsigned repeat = extract64(instruction, 40, 2);
            bool source_half = src_type == 2 || src_type == 4;
            bool dest_half = dst_type == 2 || dst_type == 4;

            if ((instruction & (BIT_ULL(42) | BIT_ULL(49))) ||
                extract64(instruction, 55, 4) || mode == 3 ||
                (src_type != 1 && src_type != 2 && src_type != 3 &&
                 src_type != 4 && src_type != 5) ||
                (src_type != dst_type && !(src_type == 5 && dst_type == 4)) ||
                (dst >= ADRENO_SHADER_GPRS &&
                 !(dst == 244 && dst_type == 4 && !repeat)) ||
                (dst < ADRENO_SHADER_GPRS &&
                 dst + repeat >= ADRENO_SHADER_GPRS)) {
                return false;
            }
            for (unsigned n = 0; n <= repeat; n++) {
                unsigned increment = instruction & BIT_ULL(43) ? n : 0;
                uint32_t value;
                int index;

                if (mode == 2) {
                    value = low;
                } else if (mode == 1 || (low & BIT(11))) {
                    if (source_half) {
                        return false; /* Packed half constants are separate. */
                    }
                    if (mode == 1) {
                        if (low & ~0x7ffU) {
                            return false;
                        }
                        index = low + increment;
                    } else {
                        if ((low & ~0xfffU) || !(low & BIT(10)) ||
                            !r->address_valid) {
                            return false;
                        }
                        index = r->address + sextract32(low, 0, 10) + increment;
                    }
                    if (index < 0 || !qcom8998_shader_word(s, block, 1,
                                                           index, &value)) {
                        return false;
                    }
                } else {
                    index = (low & 0xff) + increment;
                    if ((low & ~(0xffU | BIT(10))) ||
                        index >= ADRENO_SHADER_GPRS ||
                        !(source_half ? r->half_valid[index] :
                                        r->full_valid[index])) {
                        return false;
                    }
                    value = source_half ? r->half[index] : r->full[index];
                }
                /* Reject S32->S16 narrowing outside its exact range. */
                if (src_type == 5 && dst_type == 4 &&
                    ((int32_t)value < INT16_MIN ||
                     (int32_t)value > INT16_MAX)) {
                    return false;
                }
                if (dst == 244) {
                    r->address = (int16_t)value;
                    r->address_valid = true;
                } else if (dest_half) {
                    r->half[dst + n] = value;
                    r->half_valid[dst + n] = true;
                } else {
                    r->full[dst + n] = value;
                    r->full_valid[dst + n] = true;
                }
            }
            continue;
        }
        /* Branches, textures, memory effects and other ALU are not executed. */
        return false;
    }
    return false;
}

static bool qcom8998_shader_run(Qcom8998Display *s, unsigned block,
                                unsigned vertex, unsigned vertex_reg,
                                Qcom8998ShaderRegs *r)
{
    memset(r, 0, sizeof(*r));
    if (block == 8) {
        if (vertex_reg >= ADRENO_SHADER_GPRS) {
            return false;
        }
        r->full[vertex_reg] = vertex;
        r->full_valid[vertex_reg] = true;
    }
    return qcom8998_shader_execute(s, block, r);
}

static bool qcom8998_screen_coordinate(uint32_t bits, unsigned *value)
{
    unsigned exponent = extract32(bits, 23, 8);
    unsigned shift;
    uint32_t significand;

    if (!(bits & 0x7fffffff)) {
        *value = 0;
        return true;
    }
    if ((bits & BIT(31)) || exponent < 127 || exponent > 139) {
        return false;
    }
    shift = 150 - exponent;
    significand = (bits & 0x7fffff) | BIT(23);
    if ((significand & ((1U << shift) - 1)) ||
        (significand >> shift) > MDP_MAX_OUTPUT_SIZE) {
        return false;
    }
    *value = significand >> shift;
    return true;
}

static bool qcom8998_unorm8(uint32_t bits, unsigned *value)
{
    unsigned exponent = extract32(bits, 23, 8);
    uint64_t scaled, remainder, halfway;
    unsigned shift;

    if (exponent == 255) {
        return false;
    }
    if (bits & BIT(31)) {
        *value = 0;
    } else if (bits >= 0x3f800000) {
        *value = 255;
    } else {
        shift = 150 - exponent;
        if (shift >= 64) {
            *value = 0;
            return true;
        }
        scaled = (uint64_t)((bits & 0x7fffff) | BIT(23)) * 255;
        remainder = scaled & ((1ULL << shift) - 1);
        halfway = 1ULL << (shift - 1);
        *value = (scaled >> shift) + (remainder > halfway ||
                 (remainder == halfway && ((scaled >> shift) & 1)));
    }
    return true;
}

static bool qcom8998_cp_textured_rectangle(Qcom8998Display *s, unsigned count,
                                           const uint32_t *payload);

static bool qcom8998_cp_rectangle(Qcom8998Display *s, unsigned count,
                                  const uint32_t *payload)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    Qcom8998ShaderRegs vertex, fragment;
    unsigned coordinates[2][2];
    unsigned position = regs[0xe593] & 0xff;
    unsigned output = regs[0xe5cb] & 0xff;
    uint32_t color = 0;
    unsigned x1, y1, x2, y2;
    static const unsigned scissor[] = { 0xe0aa, 0xe0ca, 0xe0ea };
    static const unsigned disabled[] = {
        0xe001, 0xe002, 0xe003, 0xe090, 0xe0a1, 0xe0a2, 0xe0a3,
        0xe0a4, 0xe100, 0xe142, 0xe143, 0xe1a8, 0xe1b1, 0xe1c0,
        0xe1d0, 0xe1d1, 0xe2a1, 0xe385, 0xe400, 0xe408, 0xe409,
        0xe5db, 0xe704, 0xe705,
    };

    if (regs[0xe5db]) {
        return qcom8998_cp_textured_rectangle(s, count, payload);
    }

    /* A nonindexed, pretransformed screen rectangle and constant-color FS. */
    if (!s->me_initialized || count != 3 || (payload[0] & ~0xc00U) != 0x88 ||
        payload[1] != 1 || payload[2] != 2 ||
        s->render_mode[0] != ADRENO_RENDER_BYPASS ||
        regs[ADRENO_RB_CNTL] != ADRENO_RB_BYPASS ||
        (regs[ADRENO_RB_RENDER_CNTL] & ~BIT(8)) ||
        (regs[0xe0a0] & ~BIT(8)) ||
        regs[0xe000] != 0x181 || regs[0xe146] != 1 || regs[0xe147] != 0xf ||
        regs[0xe150] != 0x780 || regs[0xe1a9] != 0xffff0000 ||
        !(regs[0xe584] & 1) || !(regs[0xe585] & 1) ||
        ((regs[0xe586] | regs[0xe587] | regs[0xe588]) & 1) ||
        regs[0xe5ac] != 1 || regs[0xe5ad] ||
        regs[0xe5c3] != 1 || regs[0xe5c4] ||
        regs[0xe592] != 1 || (regs[0xe593] & ~0xffU) != 0xf00 ||
        regs[0xe5a3] || (regs[0xe5cb] & ~0xffU) ||
        (regs[0xe5ca] & 0x3f) != 1 || regs[0xe5d3] != 0x30 ||
        position > ADRENO_SHADER_GPRS - 4 ||
        output > ADRENO_SHADER_GPRS - 4) {
        return false;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(disabled); i++) {
        if (regs[disabled[i]]) {
            return false;
        }
    }
    for (unsigned v = 0; v < 2; v++) {
        if (!qcom8998_shader_run(s, 8, v, regs[0xe401] & 0xff, &vertex)) {
            return false;
        }
        for (unsigned c = 0; c < 4; c++) {
            if (!vertex.full_valid[position + c]) {
                return false;
            }
        }
        if ((vertex.full[position + 2] != 0 &&
             vertex.full[position + 2] != 0x3f800000) ||
            vertex.full[position + 3] != 0x3f800000 ||
            !qcom8998_screen_coordinate(vertex.full[position],
                                         &coordinates[v][0]) ||
            !qcom8998_screen_coordinate(vertex.full[position + 1],
                                         &coordinates[v][1])) {
            return false;
        }
    }
    x1 = MIN(coordinates[0][0], coordinates[1][0]);
    y1 = MIN(coordinates[0][1], coordinates[1][1]);
    x2 = MAX(coordinates[0][0], coordinates[1][0]);
    y2 = MAX(coordinates[0][1], coordinates[1][1]);
    for (unsigned i = 0; i < ARRAY_SIZE(scissor); i++) {
        x1 = MAX(x1, regs[scissor[i]] & 0xffff);
        y1 = MAX(y1, regs[scissor[i]] >> 16);
        x2 = MIN(x2, (regs[scissor[i] + 1] & 0xffff) + 1);
        y2 = MIN(y2, (regs[scissor[i] + 1] >> 16) + 1);
    }
    if (x1 >= x2 || y1 >= y2) {
        return true; /* No fragment invocation or attachment access. */
    }
    if (!qcom8998_shader_run(s, 12, 0, 0, &fragment)) {
        return false;
    }
    for (unsigned c = 0; c < 4; c++) {
        unsigned component;

        if (!fragment.full_valid[output + c] ||
            !qcom8998_unorm8(fragment.full[output + c], &component)) {
            return false;
        }
        color |= component << (c * 8);
    }
    if (!qcom8998_fill_rect(s, 0, x1 | (y1 << 16),
                            (x2 - 1) | ((y2 - 1) << 16), color)) {
        return false;
    }
    trace_qcom8998_cp_rectangle(x1, y1, x2 - x1, y2 - y1, color);
    return true;
}

typedef struct Qcom8998RasterVertex {
    int x, y; /* D3D 16.8 positions, bounded to +/-4096 pixels here. */
    uint32_t color[4];
} Qcom8998RasterVertex;

static bool qcom8998_vertex_fetch(Qcom8998Display *s, unsigned index,
                                  Qcom8998ShaderRegs *r)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned attributes = regs[0xe400];
    float_status fp = { 0 };

    if (attributes > 16 || (regs[0xe401] & ~0xffffffU) ||
        regs[0xe408] || regs[0xe409]) {
        return false;
    }
    memset(r, 0, sizeof(*r));
    for (unsigned i = 0; i < 3; i++) {
        unsigned reg = extract32(regs[0xe401], i * 8, 8);

        if (reg != 0xfc) {
            if (reg >= ADRENO_SHADER_GPRS || i == 2) {
                return false;
            }
            r->full[reg] = i ? 0 : index;
            r->full_valid[reg] = true;
        }
    }
    for (unsigned i = 0; i < attributes; i++) {
        unsigned decode = regs[0xe48a + i * 2];
        unsigned fetch = 0xe40a + (decode & 31) * 4;
        unsigned format = extract32(decode, 20, 8);
        unsigned offset = extract32(decode, 5, 12);
        unsigned dest = regs[0xe4ca + i];
        unsigned out = extract32(dest, 4, 8);
        unsigned components, bytes;
        uint32_t value[4] = { 0, 0, 0, 0x3f800000 };
        uint64_t address, position;

        if ((decode & (BIT(17) | BIT(18) | BIT(19))) ||
            (dest & ~0xfffU) || out > ADRENO_SHADER_GPRS - 4 ||
            (regs[0xe48b + i * 2] > 1)) {
            return false;
        }
        if (format == 0x74 || format == 0x67) {
            components = format == 0x74 ? 3 : 2;
            bytes = components * 4;
            if (extract32(decode, 28, 2)) {
                return false;
            }
        } else if (format == 0x30) {
            components = 4;
            bytes = 4;
        } else {
            return false;
        }
        address = qcom8998_reg_pair(regs, fetch * 4);
        position = (uint64_t)index * regs[fetch + 3] + offset;
        if ((address & 3) || (position & 3) ||
            position + bytes > regs[fetch + 2] ||
            address > UINT64_MAX - position - bytes) {
            return false;
        }
        address += position;
        if (format == 0x30) {
            uint32_t packed;

            if (!qcom8998_cp_word(s, address, &packed, false)) {
                return false;
            }
            packed = le32_to_cpu(qcom8998_color_swap(packed,
                                                     (decode >> 28) & 3));
            for (unsigned c = 0; c < 4; c++) {
                value[c] = float32_div(uint32_to_float32(
                    extract32(packed, c * 8, 8), &fp), 0x437f0000, &fp);
            }
        } else {
            for (unsigned c = 0; c < components; c++) {
                if (!qcom8998_cp_word(s, address + c * 4, &value[c], false) ||
                    !qcom8998_shader_finite(value[c])) {
                    return false;
                }
            }
        }
        for (unsigned c = 0; c < 4; c++) {
            if (dest & BIT(c)) {
                r->full[out + c] = value[c];
                r->full_valid[out + c] = true;
            }
        }
    }
    if (!qcom8998_shader_execute(s, 8, r)) {
        trace_qcom8998_cp_draw_reject("vertex shader", r->instruction[0],
                                      r->instruction[1]);
        return false;
    }
    return true;
}

static bool qcom8998_vertex_position(Qcom8998Display *s,
                                     const Qcom8998ShaderRegs *r,
                                     Qcom8998RasterVertex *v)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned position = regs[0xe593] & 0xff;
    unsigned color = extract32(regs[0xe593], 16, 8);
    float_status fp = { 0 };

    if (position > ADRENO_SHADER_GPRS - 4 ||
        color > ADRENO_SHADER_GPRS - 4) {
        return false;
    }
    for (unsigned c = 0; c < 4; c++) {
        if (!r->full_valid[position + c] || !r->full_valid[color + c]) {
            return false;
        }
        v->color[c] = r->full[color + c];
    }
    if ((r->full[position + 2] != 0 &&
         r->full[position + 2] != 0x3f800000) ||
        r->full[position + 3] != 0x3f800000) {
        trace_qcom8998_cp_draw_reject("clip Z/W", r->full[position + 2],
                                      r->full[position + 3]);
        return false; /* Perspective division and Z clipping are separate. */
    }
    for (unsigned c = 0; c < 2; c++) {
        uint32_t offset = regs[0xe010 + c * 2];
        uint32_t scale = regs[0xe011 + c * 2];
        uint32_t coordinate;
        int fixed;

        if (!qcom8998_shader_finite(r->full[position + c]) ||
            !qcom8998_shader_finite(offset) ||
            !qcom8998_shader_finite(scale)) {
            return false;
        }
        coordinate = float32_add(float32_mul(r->full[position + c], scale,
                                             &fp), offset, &fp);
        if (!qcom8998_shader_finite(coordinate) ||
            (coordinate & 0x7fffffff) > 0x45800000) {
            return false;
        }
        /* D3D11 requires eight fractional bits and nearest-even snap. */
        fixed = float32_to_int32_scalbn(coordinate, float_round_nearest_even,
                                        8, &fp);
        if (c) {
            v->y = fixed;
        } else {
            v->x = fixed;
        }
    }
    return true;
}

static int64_t qcom8998_edge(const Qcom8998RasterVertex *a,
                            const Qcom8998RasterVertex *b, int x, int y)
{
    return (int64_t)(b->x - a->x) * (y - a->y) -
           (int64_t)(b->y - a->y) * (x - a->x);
}

static bool qcom8998_top_left(const Qcom8998RasterVertex *a,
                              const Qcom8998RasterVertex *b)
{
    return b->y < a->y || (b->y == a->y && b->x > a->x);
}

static bool qcom8998_fragment_color(Qcom8998Display *s,
                                     Qcom8998ShaderRegs *shader,
                                     uint32_t destination, bool blend,
                                     uint32_t *color)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned output = regs[0xe5cb] & 0xff;
    float_status fp = { 0 };
    uint32_t factor = 0;

    if (output > ADRENO_SHADER_GPRS - 4 ||
        !qcom8998_shader_execute(s, 12, shader)) {
        trace_qcom8998_cp_draw_reject("fragment shader", shader->instruction[0],
                                      shader->instruction[1]);
        return false;
    }
    if (blend) {
        uint32_t alpha = shader->full[output + 3];

        if (!shader->full_valid[output + 3] || alpha > 0x3f800000) {
            return false;
        }
        factor = float32_sub(0x3f800000, alpha, &fp);
    }
    *color = 0;
    for (unsigned c = 0; c < 4; c++) {
        unsigned component;
        uint32_t value = shader->full[output + c];

        if (!shader->full_valid[output + c]) {
            return false;
        }
        if (blend) {
            uint32_t dest;

            /* ONE + ONE_MINUS_SRC_ALPHA, linear RGBA8, ADD on all lanes. */
            if (value > 0x3f800000) {
                return false;
            }
            dest = float32_div(uint32_to_float32(
                extract32(destination, c * 8, 8), &fp), 0x437f0000, &fp);
            value = float32_add(value, float32_mul(dest, factor, &fp), &fp);
        }
        if (!qcom8998_unorm8(value, &component)) {
            return false;
        }
        *color |= component << (c * 8);
    }
    return true;
}

/* Cache only sampled pages, before publishing any destination pixels. */
static bool qcom8998_texture_word(Qcom8998Display *s, GHashTable *pages,
                                   uint64_t address, unsigned cpp,
                                   uint32_t *value)
{
    uint64_t page = address & ~((1ULL << SMMU_4K_SHIFT) - 1);
    gpointer key = (gpointer)(uintptr_t)(page >> SMMU_4K_SHIFT);
    uint8_t *data = g_hash_table_lookup(pages, key);

    if (!data) {
        hwaddr physical;
        g_autofree uint8_t *pending = g_malloc(1U << SMMU_4K_SHIFT);

        if (!qcom8998_cp_translate(s, address, false, &physical)) {
            return false;
        }
        physical &= ~((1ULL << SMMU_4K_SHIFT) - 1);
        if (!qcom8998_ram_buffer(physical, 1U << SMMU_4K_SHIFT, false) ||
            address_space_read(&address_space_memory, physical,
                MEMTXATTRS_UNSPECIFIED, pending,
                1U << SMMU_4K_SHIFT) != MEMTX_OK) {
            return false;
        }
        data = g_steal_pointer(&pending);
        g_hash_table_insert(pages, key, data);
    }
    *value = cpp == 1 ? data[address - page] :
                       ldl_le_p(data + (address - page));
    return true;
}

static bool qcom8998_texture_surface(Qcom8998Display *s, uint64_t base,
                                      uint64_t pitch, unsigned x1, unsigned y1,
                                      unsigned x2, unsigned y2, unsigned cpp,
                                      GArray *spans,
                                      uint8_t **pixels)
{
    unsigned row_bytes = (x2 - x1) * cpp;

    *pixels = g_malloc(row_bytes * (y2 - y1));
    for (unsigned y = y1; y < y2; y++) {
        unsigned offset = (y - y1) * row_bytes;
        unsigned remaining = row_bytes;
        uint64_t address = base + y * pitch + x1 * cpp;

        while (remaining) {
            Qcom8998GmemSpan span;

            span.offset = offset;
            span.length = MIN(remaining, (1U << SMMU_4K_SHIFT) -
                              extract64(address, 0, SMMU_4K_SHIFT));
            if (!qcom8998_cp_translate(s, address, true, &span.address) ||
                !qcom8998_ram_buffer(span.address, span.length, true) ||
                address_space_read(&address_space_memory, span.address,
                    MEMTXATTRS_UNSPECIFIED, *pixels + offset,
                    span.length) != MEMTX_OK) {
                return false;
            }
            g_array_append_val(spans, span);
            offset += span.length;
            address += span.length;
            remaining -= span.length;
        }
    }
    return true;
}

static bool qcom8998_cp_textured_rectangle(Qcom8998Display *s, unsigned count,
                                           const uint32_t *payload)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    const GArray *samplers = s->load_state[4][0];
    const GArray *textures = s->load_state[4][1];
    const uint32_t *sampler, *texture;
    Qcom8998ShaderRegs vertex;
    unsigned coordinates[2][2];
    uint32_t uv[2][2], instruction[2];
    uint64_t address, pitch, layer, dest;
    unsigned width, height, swap, cpp, x1, y1, x2, y2;
    unsigned tile = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 8, 2);
    bool r8 = regs[0xe5d3] == 3;
    unsigned position = extract32(regs[0xe593], 16, 8);
    unsigned varying = regs[0xe593] & 0xff;
    unsigned *texel[2];
    float_status fp = { 0 };
    g_autoptr(GHashTable) pages = g_hash_table_new_full(g_direct_hash,
                                                       g_direct_equal,
                                                       NULL, g_free);
    g_autoptr(GArray) spans = g_array_new(false, false,
                                        sizeof(Qcom8998GmemSpan));
    g_autofree uint32_t *pending = NULL;
    g_autofree uint8_t *raw_pixels = NULL;
    g_autofree unsigned *texel_x = NULL;
    g_autofree unsigned *texel_y = NULL;
    static const unsigned scissor[] = { 0xe0aa, 0xe0ca, 0xe0ea };
    static const unsigned disabled[] = {
        0xe001, 0xe002, 0xe003, 0xe090, 0xe0a2, 0xe0a3, 0xe0a4,
        0xe100, 0xe142, 0xe143, 0xe1a8, 0xe1b1, 0xe1c0, 0xe2a1,
        0xe385, 0xe400, 0xe408, 0xe409, 0xe5c9, 0xe704, 0xe705,
    };

    /*
     * QCDX's single initial texture load: varying 4, sampler/texture 0,
     * full r0.xyzw, followed by END. A5xx's undocumented e5db/e5dc
     * encoding is restricted to the observed pair; this does not infer
     * other A6xx prefetch fields from the similar command word.
     */
    if (!s->me_initialized || count != 3 || (payload[0] & ~0xc00U) != 0x88 ||
        payload[1] != 1 || payload[2] != 2 ||
        s->render_mode[0] != ADRENO_RENDER_BYPASS ||
        regs[ADRENO_RB_CNTL] != ADRENO_RB_BYPASS ||
        (regs[ADRENO_RB_RENDER_CNTL] & ~BIT(8)) ||
        regs[0xe000] != 0x181 || (regs[0xe0a0] & ~BIT(8)) ||
        (regs[0xe0a1] && regs[0xe0a1] != 0x20) ||
        regs[0xe146] != 1 || regs[0xe147] != 0xf ||
        regs[0xe150] != 0x780 || regs[0xe1a9] != 0xffff0000 ||
        !(regs[0xe584] & 1) || !(regs[0xe585] & 1) ||
        ((regs[0xe586] | regs[0xe587] | regs[0xe588]) & 1) ||
        regs[0xe5ac] != 1 || regs[0xe5ad] ||
        regs[0xe5c3] != 1 || regs[0xe5c4] ||
        regs[0xe592] != 2 || (regs[0xe593] & 0xff00ff00) != 0x0f000300 ||
        regs[0xe5a3] != 4 || regs[0xe5cb] ||
        (regs[0xe5ca] & 0x3f) != 1 ||
        (regs[0xe5d3] != 0x30 && !r8) ||
        regs[0xe5db] != 0x19 || regs[0xe5dc] != 0x23c00004 ||
        regs[0xe791] != 4 || regs[0xe792] != 2 ||
        position > ADRENO_SHADER_GPRS - 4 ||
        varying > ADRENO_SHADER_GPRS - 2 ||
        !qcom8998_shader_word(s, 12, 0, 0, &instruction[0]) ||
        !qcom8998_shader_word(s, 12, 0, 1, &instruction[1]) ||
        instruction[0] || instruction[1] != 0x03000000 ||
        !samplers || !samplers->len || !textures || !textures->len ||
        !g_array_index(samplers, Qcom8998StateUnit, 0).valid ||
        !g_array_index(textures, Qcom8998StateUnit, 0).valid) {
        trace_qcom8998_cp_draw_reject("texture rectangle", regs[0xe5db],
                                      regs[0xe5dc]);
        return false;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(disabled); i++) {
        if (regs[disabled[i]]) {
            trace_qcom8998_cp_draw_reject("rectangle state", disabled[i],
                                          regs[disabled[i]]);
            return false;
        }
    }
    sampler = g_array_index(samplers, Qcom8998StateUnit, 0).words;
    texture = g_array_index(textures, Qcom8998StateUnit, 0).words;
    width = extract32(texture[1], 0, 15);
    height = extract32(texture[1], 15, 15);
    pitch = extract32(texture[2], 7, 22);
    address = deposit64(texture[4], 32, 17, texture[5] & 0x1ffff);
    swap = extract32(texture[0], 30, 2);
    cpp = r8 ? 1 : 4;
    /* Linear RGBA8/R8, nearest/clamp, unnormalized XY. */
    if (sampler[0] != 0x60920 || sampler[1] != 0x67 ||
        sampler[2] || sampler[3] ||
        (texture[0] & ~0xc0000000U) != (r8 ? 0x00c0b200 : 0x0c006880) ||
        (r8 && swap) ||
        !width || !height || width > MDP_MAX_OUTPUT_SIZE ||
        height > MDP_MAX_OUTPUT_SIZE || (texture[1] & 0xc0000000) ||
        (texture[2] & 0xe000007f) != 0x20000000 ||
        pitch < width * cpp || (pitch & 3) ||
        pitch * height > ADRENO_CLEAR_MAX_BYTES ||
        (texture[3] & ~0x0f803fffU) ||
        (texture[4] & 31) || (texture[5] & ~0x3ffffU) ||
        !(texture[5] & BIT(17)) ||
        address > UINT64_MAX - pitch * height) {
        trace_qcom8998_cp_draw_reject("texture descriptor", texture[0],
                                      texture[2]);
        return false;
    }
    for (unsigned i = 6; i < 12; i++) {
        if (texture[i]) {
            return false;
        }
    }
    for (unsigned v = 0; v < 2; v++) {
        if (!qcom8998_shader_run(s, 8, v, regs[0xe401] & 0xff, &vertex)) {
            trace_qcom8998_cp_draw_reject("rectangle shader",
                vertex.instruction[0], vertex.instruction[1]);
            return false;
        }
        for (unsigned c = 0; c < 4; c++) {
            if (!vertex.full_valid[position + c]) {
                return false;
            }
        }
        if (vertex.full[position + 2] ||
            vertex.full[position + 3] != 0x3f800000) {
            return false;
        }
        for (unsigned c = 0; c < 2; c++) {
            if (!vertex.full_valid[varying + c] ||
                !qcom8998_shader_finite(vertex.full[varying + c]) ||
                (vertex.full[varying + c] & 0x7fffffff) > 0x48000000 ||
                !qcom8998_screen_coordinate(vertex.full[position + c],
                                             &coordinates[v][c])) {
                trace_qcom8998_cp_draw_reject("rectangle coordinates",
                    vertex.full[position + c], vertex.full[varying + c]);
                return false;
            }
            uv[v][c] = vertex.full[varying + c];
        }
    }
    x1 = MIN(coordinates[0][0], coordinates[1][0]);
    y1 = MIN(coordinates[0][1], coordinates[1][1]);
    x2 = MAX(coordinates[0][0], coordinates[1][0]);
    y2 = MAX(coordinates[0][1], coordinates[1][1]);
    for (unsigned i = 0; i < ARRAY_SIZE(scissor); i++) {
        x1 = MAX(x1, regs[scissor[i]] & 0xffff);
        y1 = MAX(y1, regs[scissor[i]] >> 16);
        x2 = MIN(x2, (regs[scissor[i] + 1] & 0xffff) + 1);
        y2 = MIN(y2, (regs[scissor[i] + 1] >> 16) + 1);
    }
    if (x1 >= x2 || y1 >= y2) {
        return true;
    }
    pitch = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 1] << 6;
    layer = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 2] << 6;
    dest = qcom8998_reg_pair(regs, (ADRENO_RB_MRT_BUF_INFO + 3) * 4);
    if ((regs[ADRENO_RB_MRT_BUF_INFO] &
         ~(ADRENO_RB_COLOR_SWAP_MASK | 0x300U)) != (r8 ? 3 : 0x30) ||
        (r8 && (regs[ADRENO_RB_MRT_BUF_INFO] & ADRENO_RB_COLOR_SWAP_MASK)) ||
        (tile && tile != 3) ||
        /* Pixel zero is at BASE; other TILE5_3 addresses need a layout. */
        (tile && (!r8 || x1 || y1 || x2 != 1 || y2 != 1 || pitch % 128)) ||
        regs[ADRENO_RB_RAS_MSAA_CNTL] || regs[ADRENO_RB_DEST_MSAA_CNTL] ||
        !pitch || !layer || layer % pitch ||
        layer > ADRENO_CLEAR_MAX_BYTES || x2 * cpp > pitch ||
        y2 > layer / pitch || (dest & 3) || dest > UINT64_MAX - layer) {
        trace_qcom8998_cp_draw_reject("texture attachment",
            regs[ADRENO_RB_MRT_BUF_INFO], layer);
        return false;
    }
    if (r8 ? !qcom8998_texture_surface(s, dest, pitch, x1, y1, x2, y2, cpp,
                                        spans, &raw_pixels) :
              !qcom8998_sysmem_surface(s, x1, y1, x2, y2, spans, &pending)) {
        return false;
    }
    /* Affine XY are separable. Round interpolation to float32 once. */
    texel_x = g_new(unsigned, x2 - x1);
    texel_y = g_new(unsigned, y2 - y1);
    texel[0] = texel_x;
    texel[1] = texel_y;
    for (unsigned c = 0; c < 2; c++) {
        unsigned first = c ? y1 : x1;
        unsigned last = c ? y2 : x2;
        unsigned extent = c ? height : width;
        float64 start = uint32_to_float64(coordinates[0][c], &fp);
        float64 end = uint32_to_float64(coordinates[1][c], &fp);
        float64 origin = float32_to_float64(uv[0][c], &fp);
        float64 delta = float64_sub(float32_to_float64(uv[1][c], &fp),
                                     origin, &fp);

        for (unsigned p = first; p < last; p++) {
            float64 center = float64_add(uint32_to_float64(p, &fp),
                                          0x3fe0000000000000ULL, &fp);
            float64 weight = float64_div(float64_sub(center, start, &fp),
                                          float64_sub(end, start, &fp), &fp);
            uint32_t coord = float64_to_float32(float64_add(origin,
                float64_mul(delta, weight, &fp), &fp), &fp);
            int index = float32_to_int32_scalbn(coord, float_round_down,
                                                 0, &fp);

            texel[c][p - first] = CLAMP(index, 0, (int)extent - 1);
        }
    }
    for (unsigned y = y1; y < y2; y++) {
        for (unsigned x = x1; x < x2; x++) {
            uint32_t color;

            if (!qcom8998_texture_word(s, pages, address +
                texel_y[y - y1] * (uint64_t)extract32(texture[2], 7, 22) +
                texel_x[x - x1] * cpp, cpp, &color)) {
                return false;
            }
            if (r8) {
                unsigned offset = ((y - y1) * (x2 - x1) + x - x1) * cpp;

                raw_pixels[offset] = color;
            } else {
                pending[(y - y1) * (x2 - x1) + x - x1] =
                    qcom8998_color_unswap(color, swap);
            }
        }
    }
    if (r8) {
        for (unsigned i = 0; i < spans->len; i++) {
            const Qcom8998GmemSpan *span = &g_array_index(spans,
                                                         Qcom8998GmemSpan, i);

            if (address_space_write(&address_space_memory, span->address,
                MEMTXATTRS_UNSPECIFIED, raw_pixels + span->offset,
                span->length) != MEMTX_OK) {
                return false;
            }
        }
    } else if (!qcom8998_sysmem_publish(s, spans, pending)) {
        return false;
    }
    trace_qcom8998_cp_texture_rectangle(x1, y1, x2 - x1, y2 - y1, address);
    return true;
}

static bool qcom8998_cp_triangles(Qcom8998Display *s, unsigned count,
                                  const uint32_t *payload)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned base, width, height, total = 0;
    uint64_t indices, end;
    unsigned x1 = 0, y1 = 0, x2, y2;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;
    bool blend = regs[0xe150] == 0x783;
    bool sysmem = regs[ADRENO_RB_CNTL] == ADRENO_RB_BYPASS;
    uint64_t raster_budget = 8U << 20;
    uint64_t fragment_budget = 32U << 20;
    g_autoptr(GArray) spans = g_array_new(false, false,
                                        sizeof(Qcom8998GmemSpan));
    g_autofree uint32_t *pending = NULL;
    float_status fp = { 0 };
    static const unsigned scissor[] = { 0xe0aa, 0xe0ca, 0xe0ea };

    /* Single-instance indexed TRIANGLELIST and one vec4 color varying. */
    if (!s->me_initialized || count != 7 ||
        (payload[0] & ~0x300U) != 0x404 ||
        ((payload[0] & 0x300) && !s->visibility_override) ||
        payload[1] != 1 || !payload[2] || payload[2] % 3 || payload[2] > 255 ||
        regs[0xe000] != 0x60 || regs[0xe001] || regs[0xe389] ||
        (regs[0xe090] & 3) ||
        (regs[0xe0a0] & ~(BIT(3) | BIT(8))) ||
        (regs[0xe0a1] && regs[0xe0a1] != 0x20) ||
        regs[0xe0a2] || regs[0xe0a3] || (regs[0xe1a8] & BIT(8)) ||
        (regs[0xe1b1] & (BIT(0) | BIT(1) | BIT(6))) || regs[0xe1c0] ||
        regs[0xe100] || regs[0xe2a1] || regs[0xe385] ||
        (regs[0xe141] & ~(BIT(3) | BIT(8))) ||
        regs[0xe146] != 1 || regs[0xe147] != 0xf ||
        (blend ? (regs[0xe151] != 0x07010701 ||
                  regs[0xe1a9] != 0xffff00ff || regs[0xe5c9] != 1) :
                 (regs[0xe150] != 0x780 || regs[0xe1a9] != 0xffff0000 ||
                  regs[0xe5c9])) ||
        !(regs[0xe584] & 1) || !(regs[0xe585] & 1) ||
        regs[0xe070] || regs[0xe071] != 0x3f800000 ||
        regs[0xe200] || regs[0xe201] != 0x3f800000 ||
        regs[0xe388] != 0x12 || regs[0xe592] != 2 ||
        (regs[0xe593] & 0x0f000f00) != 0x0f000f00 ||
        regs[0xe5a3] != 0x400 || regs[0xe5d3] != 0x30 ||
        regs[0xe793] || regs[0xe794] || regs[0xe795] ||
        regs[0xe5db] ||
        (regs[0xe384] & BIT(9)) || regs[0xe1b0] ||
        !(regs[0xe590] & BIT(2)) || !(regs[0xe5c0] & BIT(2)) ||
        qcom8998_reg_pair(regs, 0xe5ac * 4) != (s->shader_address[8] | 1) ||
        qcom8998_reg_pair(regs, 0xe5c3 * 4) != (s->shader_address[12] | 1)) {
        trace_qcom8998_cp_draw_reject("draw configuration", regs[0xe140],
                                      regs[0xe592]);
        return false;
    }
    if (sysmem) {
        uint64_t pitch = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 1] << 6;
        uint64_t layer = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 2] << 6;
        uint64_t address = qcom8998_reg_pair(regs,
                                            (ADRENO_RB_MRT_BUF_INFO + 3) * 4);

        if ((regs[ADRENO_RB_MRT_BUF_INFO] & ~ADRENO_RB_COLOR_SWAP_MASK) !=
                ADRENO_RB_RGBA8_UNORM ||
            regs[ADRENO_RB_RAS_MSAA_CNTL] ||
            regs[ADRENO_RB_DEST_MSAA_CNTL] || !pitch || !layer ||
            pitch / 4 > MDP_MAX_OUTPUT_SIZE || layer % pitch ||
            layer / pitch > MDP_MAX_OUTPUT_SIZE ||
            layer > ADRENO_CLEAR_MAX_BYTES || (address & 3) ||
            address > UINT64_MAX - layer) {
            trace_qcom8998_cp_draw_reject("linear layout", pitch, layer);
            return false;
        }
        width = pitch / 4;
        height = layer / pitch;
        ox = oy = base = 0; /* RB_WINDOW_OFFSET applies only to GMEM bins. */
    } else {
        height = extract32(regs[ADRENO_RB_CNTL], 9, 8) << 5;
        width = extract32(regs[ADRENO_RB_CNTL], 0, 8) << 5;
        if (!height || !width || ox + width > 0x10000 ||
            oy + height > 0x10000 ||
            !qcom8998_gmem_layout(s, 0, regs[0xe1d0],
            (ox + width - 1) | ((oy + height - 1) << 16), &base, &width)) {
            return false;
        }
    }
    x1 = ox;
    y1 = oy;
    x2 = ox + width;
    y2 = oy + height;
    /* Restrict clipping to a positive, origin-zero symmetric viewport. */
    for (unsigned c = 0; c < 2; c++) {
        unsigned extent;
        uint32_t scale = regs[0xe011 + c * 2];

        if ((regs[0xe010 + c * 2] != (scale & ~BIT(31))) ||
            !qcom8998_screen_coordinate(float32_mul(scale & ~BIT(31),
                                                    0x40000000, &fp),
                                         &extent)) {
            return false;
        }
        if (c) {
            y2 = MIN(y2, extent);
        } else {
            x2 = MIN(x2, extent);
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(scissor); i++) {
        x1 = MAX(x1, regs[scissor[i]] & 0xffff);
        y1 = MAX(y1, regs[scissor[i]] >> 16);
        x2 = MIN(x2, (regs[scissor[i] + 1] & 0xffff) + 1);
        y2 = MIN(y2, (regs[scissor[i] + 1] >> 16) + 1);
    }
    x2 = MAX(x1, x2);
    y2 = MAX(y1, y2);
    indices = deposit64(payload[4], 32, 32, payload[5]);
    end = ((uint64_t)payload[3] + payload[2]) * 2;
    if ((indices & 1) || end > payload[6] || indices > UINT64_MAX - end) {
        return false;
    }
    indices += (uint64_t)payload[3] * 2;
    if (sysmem) {
        if (!qcom8998_sysmem_surface(s, x1, y1, x2, y2, spans, &pending)) {
            return false;
        }
        ox = x1;
        oy = y1;
        width = x2 - x1;
    } else {
        pending = g_memdup2(s->gmem, ADRENO_GMEM_SIZE);
    }
    for (unsigned t = 0; t < payload[2]; t += 3) {
        Qcom8998RasterVertex vertex[3];
        Qcom8998ShaderRegs shader;
        int64_t area;
        uint32_t color = 0;
        bool uniform = true;
        bool constant[4];
        int bx1, by1, bx2, by2;

        for (unsigned v = 0; v < 3; v++) {
            uint64_t address = indices + (t + v) * 2;
            uint32_t packed;
            unsigned index;

            if (!qcom8998_cp_word(s, address & ~3ULL, &packed, false)) {
                return false;
            }
            index = extract32(packed, (address & 2) * 8, 16);
            if ((regs[0xe384] & BIT(8)) &&
                index == (regs[0xe38c] & 0xffff)) {
                return false; /* Primitive restart is not decoded here. */
            }
            if (!qcom8998_vertex_fetch(s, index, &shader) ||
                !qcom8998_vertex_position(s, &shader, &vertex[v])) {
                return false;
            }
        }
        area = qcom8998_edge(&vertex[0], &vertex[1], vertex[2].x, vertex[2].y);
        if (!area) {
            continue;
        }
        if (x1 == x2 || y1 == y2) {
            /* Empty clipping causes no fragment invocation. */
            continue;
        }
        if (area < 0) {
            Qcom8998RasterVertex swap = vertex[1];

            vertex[1] = vertex[2];
            vertex[2] = swap;
            area = -area;
        }
        memset(&shader, 0, sizeof(shader));
        for (unsigned c = 0; c < 4; c++) {
            constant[c] = vertex[0].color[c] == vertex[1].color[c] &&
                          vertex[0].color[c] == vertex[2].color[c];
            uniform &= constant[c];
            for (unsigned v = 0; v < 3; v++) {
                if (!qcom8998_shader_finite(vertex[v].color[c])) {
                    return false;
                }
            }
            shader.input[4 + c] = vertex[0].color[c];
            shader.input_valid[4 + c] = true;
        }
        if (!qcom8998_fragment_color(s, &shader, 0, false, &color)) {
            return false;
        }
        bx1 = MAX((int)x1, MIN(vertex[0].x,
                  MIN(vertex[1].x, vertex[2].x)) / 256);
        by1 = MAX((int)y1, MIN(vertex[0].y,
                  MIN(vertex[1].y, vertex[2].y)) / 256);
        bx2 = MIN((int)x2, MAX(vertex[0].x,
                  MAX(vertex[1].x, vertex[2].x)) / 256 + 1);
        by2 = MIN((int)y2, MAX(vertex[0].y,
                  MAX(vertex[1].y, vertex[2].y)) / 256 + 1);
        if (bx1 >= bx2 || by1 >= by2) {
            continue;
        }
        if ((uint64_t)(bx2 - bx1) * (by2 - by1) > raster_budget) {
            return false;
        }
        raster_budget -= (uint64_t)(bx2 - bx1) * (by2 - by1);
        if (!uniform || blend) {
            uint64_t work = (uint64_t)(bx2 - bx1) * (by2 - by1) *
                            (shader.steps * 4 + 32);

            if (work > fragment_budget) {
                return false;
            }
            fragment_budget -= work;
        }
        for (unsigned y = by1; y < by2; y++) {
            for (unsigned x = bx1; x < bx2; x++) {
                bool inside = true;

                for (unsigned e = 0; e < 3; e++) {
                    unsigned next = (e + 1) % 3;
                    int64_t edge = qcom8998_edge(&vertex[e], &vertex[next],
                                                 x * 256 + 128, y * 256 + 128);

                    if (edge < 0 || (!edge &&
                        !qcom8998_top_left(&vertex[e], &vertex[next]))) {
                        inside = false;
                        break;
                    }
                }
                if (inside) {
                    if (!uniform || blend) {
                        uint32_t weights[3] = { 0 };

                        memset(&shader, 0, sizeof(shader));
                        if (!uniform) {
                            for (unsigned v = 0; v < 3; v++) {
                                unsigned next = (v + 1) % 3;
                                unsigned last = (v + 2) % 3;
                                uint64_t numerator = qcom8998_edge(
                                    &vertex[next], &vertex[last],
                                    x * 256 + 128, y * 256 + 128);

                                /* Edge integers fit exactly in float64. */
                                weights[v] = float64_to_float32(float64_div(
                                    uint64_to_float64(numerator, &fp),
                                    uint64_to_float64(area, &fp), &fp), &fp);
                            }
                        }
                        for (unsigned c = 0; c < 4; c++) {
                            uint32_t value = vertex[0].color[c];

                            if (!constant[c]) {
                                value = 0;
                                for (unsigned v = 0; v < 3; v++) {
                                    value = float32_add(value, float32_mul(
                                        vertex[v].color[c], weights[v], &fp),
                                        &fp);
                                }
                            }
                            shader.input[4 + c] = value;
                            shader.input_valid[4 + c] = true;
                        }
                        if (!qcom8998_fragment_color(s, &shader,
                            pending[base + (y - oy) * width + x - ox],
                            blend, &color)) {
                            return false;
                        }
                    }
                    pending[base + (y - oy) * width + x - ox] = color;
                    total++;
                }
            }
        }
    }
    if (sysmem) {
        if (!qcom8998_sysmem_publish(s, spans, pending)) {
            return false;
        }
    } else {
        memcpy(s->gmem, pending, ADRENO_GMEM_SIZE);
    }
    trace_qcom8998_cp_triangles(payload[2], total);
    return true;
}

static bool qcom8998_cp_event_write(Qcom8998Display *s, unsigned count,
                                   uint32_t *payload)
{
    uint64_t address;

    if (!s->me_initialized || (count != 1 && count != 4)) {
        return false;
    }
    switch (payload[0]) {
    case ADRENO_EVENT_CCU_RESOLVE:
        {
            hwaddr timestamp = 0;
            uint32_t value = count == 4 ? cpu_to_le32(payload[3]) : 0;

            if (count == 4) {
                address = deposit64(payload[1], 32, 32, payload[2]);
                if ((address & 3) ||
                    !qcom8998_cp_translate(s, address, true, &timestamp) ||
                    !qcom8998_ram_buffer(timestamp, sizeof(value), true)) {
                    return false;
                }
            }
            if (!qcom8998_ccu_clear(s)) {
                return false;
            }
            return count == 1 || address_space_write(&address_space_memory,
                timestamp, MEMTXATTRS_UNSPECIFIED, &value, sizeof(value)) ==
                MEMTX_OK;
        }
    case ADRENO_EVENT_LRZ_FLUSH_INVALIDATE:
        /* No LRZ cache or deferred writes exist in the synchronous model. */
        return count == 1;
    case ADRENO_EVENT_CCU_INVALIDATE_DEPTH:
    case ADRENO_EVENT_CCU_INVALIDATE_COLOR:
        /* The software model has no cached attachment contents. */
        return count == 1;
    case ADRENO_EVENT_CACHE_FLUSH_TS:
    case ADRENO_EVENT_WT_DONE_TS:
    case ADRENO_EVENT_RB_DONE_TS:
    case ADRENO_EVENT_CCU_FLUSH_DEPTH_TS:
    case ADRENO_EVENT_CCU_FLUSH_COLOR_TS:
        /*
         * Earlier commands complete synchronously. A blocked draw/resolve
         * keeps this packet pending, so its timestamp cannot pass that work.
         * Interrupt-request flags and rendering events are not handled here.
         */
        if (count == 1) {
            return true;
        }
        address = deposit64(payload[1], 32, 32, payload[2]);
        return qcom8998_cp_word(s, address, &payload[3], true);
    default:
        return false;
    }
}

/* Retry a real dependency without requiring another guest WPTR write. */
static bool qcom8998_cp_wait(Qcom8998Display *s, unsigned count,
                             const uint32_t *payload)
{
    uint64_t address;
    uint32_t value;
    unsigned function, poll;
    bool passed;
    int64_t left, right;

    if (!s->me_initialized || count != 6 || (payload[0] & ~0x3fU)) {
        return false;
    }
    function = extract32(payload[0], 0, 3);
    poll = extract32(payload[0], 4, 2);
    address = deposit64(payload[1], 32, 32, payload[2]);
    if (function == 7 || poll > 1) {
        /* Scratch and write modes need their own address/side-effect rules. */
        return false;
    }
    if (poll == 1) {
        if (!qcom8998_cp_word(s, address, &value, false)) {
            return false;
        }
    } else {
        /* Register addresses are dword indices, not memory addresses. */
        if (address > UINT_MAX) {
            return false;
        }
        if (qcom8998_cp_tlb_register(address, false)) {
            value = 0; /* No outstanding invalidation: SACTIVE is clear. */
        } else if (qcom8998_cp_stored_register(address)) {
            value = s->bank[BANK_ADRENO].regs[address];
        } else {
            return false;
        }
    }
    value &= payload[4];
    if (payload[0] & BIT(3)) {
        left = (int32_t)value;
        right = (int32_t)payload[3];
    } else {
        left = value;
        right = payload[3];
    }
    switch (function) {
    case 0:
        passed = true;
        break;
    case 1:
        passed = left < right;
        break;
    case 2:
        passed = left <= right;
        break;
    case 3:
        passed = left == right;
        break;
    case 4:
        passed = left != right;
        break;
    case 5:
        passed = left >= right;
        break;
    case 6:
        passed = left > right;
        break;
    default:
        g_assert_not_reached();
    }
    trace_qcom8998_cp_wait(address, value, payload[3], payload[4], function,
                           !!(payload[0] & BIT(3)), poll, passed);
    /*
     * No GPU core clock is modeled: coalesce polls at 1 ms virtual time,
     * rather than interpreting DELAY_LOOP_CYCLES as a host busy loop.
     */
    s->cp_wait_pending = !passed;
    return passed;
}

static bool qcom8998_cp_type7(Qcom8998Display *s, uint32_t header,
                             unsigned count, uint64_t base, unsigned mask,
                             unsigned rptr, unsigned available, unsigned *skip)
{
    uint32_t payload[7];
    unsigned opcode = extract32(header, 16, 7);

    if ((header & ADRENO_PM4_TYPE7_MASK) != ADRENO_PM4_TYPE7 ||
        !(ctpop32(extract32(header, 0, 16)) & 1) ||
        !(ctpop32(extract32(header, 16, 8)) & 1)) {
        return false;
    }
    if (s->streams[s->stream_count - 1].state_group &&
        opcode != ADRENO_PM4_LOAD_STATE4 && opcode != ADRENO_PM4_NOP &&
        opcode != ADRENO_PM4_WAIT_FOR_IDLE) {
        return false; /* State buffers cannot recursively dispatch work. */
    }
    if ((opcode == ADRENO_PM4_DRAW_INDX_OFFSET ||
         opcode == ADRENO_PM4_INDIRECT_BUFFER) &&
        s->skip_ib2_global && s->skip_ib2_local && !s->visibility_override) {
        return false; /* Consuming a visibility stream is not implemented. */
    }
    if (opcode == ADRENO_PM4_DRAW_INDX_OFFSET && s->me_initialized &&
        (count == 3 || count == 7) && qcom8998_cp_dispatch_draw_state(s)) {
        return true; /* The draw stays pending until all dirty groups return. */
    }
    if (opcode == ADRENO_PM4_NOP) {
        return true;
    }
    if (opcode == ADRENO_PM4_WAIT_FOR_ME) {
        /* No independent prefetch engine or queued ME work is modeled. */
        return s->me_initialized && count == 0;
    }
    if (opcode == ADRENO_PM4_WAIT_FOR_IDLE) {
        /* This command processor executes synchronously. */
        return count == 0;
    }
    if (opcode == ADRENO_PM4_MEM_WRITE) {
        return qcom8998_cp_mem_write(s, count, base, mask, rptr);
    }
    if (opcode == ADRENO_PM4_SET_DRAW_STATE) {
        return qcom8998_cp_draw_state(s, count, base, mask, rptr);
    }
    if (opcode == ADRENO_PM4_LOAD_STATE4) {
        return qcom8998_cp_load_state(s, count, base, mask, rptr);
    }
    if (count > ARRAY_SIZE(payload)) {
        return false;
    }
    for (unsigned i = 0; i < count; i++) {
        if (!qcom8998_cp_word(s, base + (((rptr + 1 + i) & mask) * 4),
                              &payload[i], false)) {
            return false;
        }
    }
    switch (opcode) {
    case ADRENO_PM4_SKIP_IB2_GLOBAL:
    case ADRENO_PM4_SKIP_IB2_LOCAL:
        if (!s->me_initialized || count != 1 || payload[0] > 1) {
            return false;
        }
        if (opcode == ADRENO_PM4_SKIP_IB2_GLOBAL) {
            s->skip_ib2_global = payload[0];
        } else {
            s->skip_ib2_local = payload[0];
        }
        return true;
    case ADRENO_PM4_SET_VISIBILITY_OVERRIDE:
        if (!s->me_initialized || count != 1 || payload[0] > 1) {
            return false;
        }
        s->visibility_override = payload[0];
        return true;
    case ADRENO_PM4_DRAW_INDX_OFFSET:
        return count == 7 ? qcom8998_cp_triangles(s, count, payload) :
                            qcom8998_cp_rectangle(s, count, payload);
    case ADRENO_PM4_WAIT_REG_MEM:
        return qcom8998_cp_wait(s, count, payload);
    case ADRENO_PM4_SET_RENDER_MODE:
        /*
         * Stage only non-preemptible sysmem mode: a one-word selector or
         * the native five-word form without a context-save address/GMEM.
         * Preserve its final control word without assigning it semantics.
         * Other modes and context-save work must remain pending.
         */
        if (!s->me_initialized || (count != 1 && count != 5) ||
            payload[0] != ADRENO_RENDER_BYPASS ||
            (count == 5 && (payload[1] || payload[2] || payload[3] ||
                            payload[4] != 1))) {
            return false;
        }
        memset(s->render_mode, 0, sizeof(s->render_mode));
        memcpy(s->render_mode, payload, count * sizeof(payload[0]));
        return true;
    case ADRENO_PM4_INTERRUPT:
        if (!s->me_initialized || count != 1 ||
            (payload[0] & ~ADRENO_CP_INTERRUPT_MASK)) {
            return false;
        }
        /* Only reaching this packet signals completion of earlier work. */
        s->bank[BANK_ADRENO].regs[ADRENO_RBBM_INT_0_STATUS / 4] |=
            payload[0] >> ADRENO_CP_INTERRUPT_SHIFT;
        qcom8998_adreno_update_irq(s);
        return true;
    case ADRENO_PM4_EVENT_WRITE:
        return qcom8998_cp_event_write(s, count, payload);
    case ADRENO_PM4_DRAW_PRED_ENABLE_GLOBAL:
    case ADRENO_PM4_DRAW_PRED_ENABLE_LOCAL:
        if (!s->me_initialized || count != 1 || payload[0] > 1) {
            return false;
        }
        if (opcode == ADRENO_PM4_DRAW_PRED_ENABLE_GLOBAL) {
            s->draw_predicate_global = payload[0];
            if (payload[0]) {
                s->draw_predicate_local = true;
                s->draw_predicate_pass = true;
            }
        } else {
            s->draw_predicate_local = payload[0];
        }
        /* Draws and predicate-setting packets remain unsupported. */
        return true;
    case ADRENO_PM4_INDIRECT_BUFFER:
        {
            uint64_t address;

            if (!s->me_initialized || count != 3 ||
                s->stream_count >= ADRENO_CP_IB_STREAMS ||
                (payload[2] & ~ADRENO_CP_IB_SIZE_MASK)) {
                return false;
            }
            address = deposit64(payload[0], 32, 32, payload[1]);
            if ((address & 3) || address > UINT64_MAX - payload[2] * 4) {
                return false;
            }
            s->streams[s->stream_count++] = (Qcom8998CommandStream) {
                .base = address,
                .end = payload[2],
                .parent_words = count + 1,
            };
            return true;
        }
    case ADRENO_PM4_SET_PROTECTED_MODE:
        if (!s->me_initialized || count != 1 || payload[0] > 1) {
            return false;
        }
        s->register_protection = payload[0];
        return true;
    case ADRENO_PM4_SMMU_TABLE_UPDATE:
        return s->me_initialized && count == 3 &&
               qcom8998_cp_smmu_update(s, payload);
    case ADRENO_PM4_COND_EXEC:
        {
            uint32_t enabled, timestamp;
            uint64_t bool_address, timestamp_address;

            if (!s->me_initialized || count != 6) {
                return false;
            }
            bool_address = deposit64(payload[0], 32, 32, payload[1]);
            timestamp_address = deposit64(payload[2], 32, 32, payload[3]);
            if (!qcom8998_cp_word(s, bool_address, &enabled, false) ||
                !qcom8998_cp_word(s, timestamp_address, &timestamp, false)) {
                return false;
            }
            /* Timestamp ordering is modulo 2^32, including wraparound. */
            if (!enabled || (uint32_t)(payload[4] - timestamp) >= BIT(31)) {
                if (payload[5] > available - count - 1) {
                    return false;
                }
                *skip = payload[5];
            }
            return true;
        }
    case ADRENO_PM4_ME_INIT:
        /*
         * Enable contexts and error detection with header dumping off.
         * The probe implements one ring; it cannot dump headers.
         */
        if (count != 4 ||
            payload[0] != (ADRENO_ME_INIT_CONTEXTS |
                           ADRENO_ME_INIT_ERROR_DETECTION |
                           ADRENO_ME_INIT_HEADER_DUMP) ||
            payload[3]) {
            return false;
        }
        s->me_initialized = true;
        return true;
    case ADRENO_PM4_PREEMPT_ENABLE_GLOBAL:
    case ADRENO_PM4_PREEMPT_ENABLE_LOCAL:
    case ADRENO_PM4_SET_SECURE_MODE:
        /* No preemptible or secure GPU contexts are exposed. */
        return s->me_initialized && count == 1 && payload[0] == 0;
    default:
        return false;
    }
}

static void qcom8998_cp_ib_status(Qcom8998Display *s)
{
    uint32_t *regs = s->bank[BANK_ADRENO].regs;

    for (unsigned i = 1; i < ADRENO_CP_IB_STREAMS; i++) {
        uint64_t address = 0;
        unsigned words = 0;
        hwaddr offset = (i - 1) * ADRENO_CP_IB_REGISTER_STRIDE;

        if (i < s->stream_count && !s->streams[i].state_group) {
            const Qcom8998CommandStream *stream = &s->streams[i];

            address = stream->base + stream->position * 4;
            words = stream->end - stream->position;
        }
        regs[(ADRENO_CP_IB1_BASE + offset) / 4] = address;
        regs[(ADRENO_CP_IB1_BASE + offset) / 4 + 1] = address >> 32;
        regs[(ADRENO_CP_IB1_BUFSZ + offset) / 4] = words;
    }
}

static void qcom8998_cp_abort(Qcom8998Display *s)
{
    timer_del(&s->cp_resume);
    s->cp_wait_pending = false;
    s->stream_count = 0;
    memset(s->streams, 0, sizeof(s->streams));
    qcom8998_cp_ib_status(s);
}

static void qcom8998_cp_advance(Qcom8998Display *s, unsigned words,
                               unsigned ring_mask)
{
    Qcom8998CommandStream *stream = &s->streams[s->stream_count - 1];

    stream->position += words;
    if (s->stream_count == 1) {
        stream->position &= ring_mask;
        s->bank[BANK_ADRENO].regs[ADRENO_CP_RB_RPTR / 4] = stream->position;
    }
    qcom8998_cp_ib_status(s);
}

static void qcom8998_cp_trace(Qcom8998Display *s, unsigned depth,
                             uint32_t header,
                             const char *result)
{
    const Qcom8998CommandStream *stream = &s->streams[depth - 1];

    if (depth == 1) {
        trace_qcom8998_cp_packet(stream->position, stream->end, header, result);
    } else {
        trace_qcom8998_cp_ib(depth - 1, stream->base,
                            stream->position, stream->end, header, result);
    }
}

static void qcom8998_cp_run(Qcom8998Bank *bank)
{
    Qcom8998Display *s = bank->display;
    uint32_t *regs = bank->regs;
    const uint32_t cntl = regs[ADRENO_CP_RB_CNTL / 4];
    unsigned size = cntl & ADRENO_CP_RB_SIZE_MASK;
    unsigned rptr = regs[ADRENO_CP_RB_RPTR / 4];
    unsigned wptr = regs[ADRENO_CP_RB_WPTR / 4];
    uint64_t base = qcom8998_reg_pair(regs, ADRENO_CP_RB_BASE);
    unsigned mask;
    unsigned work_words = 0;

    timer_del(&s->cp_resume);
    s->cp_wait_pending = false;
    if (size > ADRENO_CP_RB_MAX_SIZE_LOG2_QWORDS ||
        (cntl & ADRENO_CP_RB_SWAP_MASK) ||
        regs[ADRENO_CP_PFP_ME_CNTL / 4]) {
        return;
    }
    /* BUFSZ is log2(qwords), whereas the pointers count dwords. */
    mask = (2U << size) - 1;
    if (rptr > mask || wptr > mask || !base || base > UINT64_MAX - mask * 4) {
        return;
    }
    if (!s->stream_count) {
        s->streams[0] = (Qcom8998CommandStream) {
            .base = base,
            .position = rptr,
        };
        s->stream_count = 1;
    }
    s->streams[0].end = wptr;
    for (unsigned budget = 0; budget < ADRENO_CP_PACKET_BUDGET; budget++) {
        Qcom8998CommandStream *stream = &s->streams[s->stream_count - 1];
        unsigned stream_mask = s->stream_count == 1 ? mask : UINT_MAX;
        unsigned available = (stream->end - stream->position) & stream_mask;
        unsigned depth = s->stream_count;
        uint32_t header = 0;
        unsigned count = 0;
        unsigned skip = 0;
        bool executed = false;

        if (!available) {
            unsigned parent_words = stream->parent_words;

            if (stream->state_group) {
                Qcom8998DrawState *state =
                    &s->draw_state[stream->state_group - 1];

                state->dirty = false;
                state->control &= ~ADRENO_DRAW_STATE_DIRTY;
            }
            if (--s->stream_count == 0) {
                break;
            }
            /* Advance the caller only after its entire child has finished. */
            qcom8998_cp_advance(s, parent_words, mask);
            continue;
        }
        if (!qcom8998_cp_word(s, stream->base + stream->position * 4,
                              &header, false)) {
            qcom8998_cp_trace(s, depth, header, "command DMA fault");
            goto publish;
        }
        switch (header & ADRENO_PM4_TYPE_MASK) {
        case ADRENO_PM4_TYPE4:
            count = extract32(header, 0, 7);
            if (count < available) {
                if (count + 1 > ADRENO_CP_WORD_BUDGET - work_words) {
                    goto defer;
                }
                work_words += count + 1;
                executed = qcom8998_cp_type4(s, header, count, stream->base,
                                            stream_mask, stream->position);
            }
            break;
        case ADRENO_PM4_TYPE7:
            count = extract32(header, 0, 14);
            if (count < available) {
                if (count + 1 > ADRENO_CP_WORD_BUDGET - work_words) {
                    goto defer;
                }
                work_words += count + 1;
                executed = qcom8998_cp_type7(s, header, count, stream->base,
                                            stream_mask, stream->position,
                                            available, &skip);
            }
            break;
        default:
            break;
        }
        if (!executed) {
            qcom8998_cp_trace(s, depth, header, "not executed");
            if (s->cp_wait_pending) {
                goto defer;
            }
            goto publish;
        }
        if (s->stream_count != depth) {
            /* The parent remains pending while IB1/IB2 executes. */
            qcom8998_cp_trace(s, depth, header, "IB dispatched");
            qcom8998_cp_ib_status(s);
            continue;
        }
        qcom8998_cp_trace(s, depth, header, "executed");
        qcom8998_cp_advance(s, count + 1 + skip, mask);
    }
defer:
    if (s->stream_count) {
        timer_mod(&s->cp_resume, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                   (s->cp_wait_pending ? 1000000 :
                    ADRENO_CP_DISPATCH_DELAY_NS));
    }
publish:
    if (!(cntl & ADRENO_CP_RB_NO_UPDATE)) {
        uint64_t shadow = qcom8998_reg_pair(regs, ADRENO_CP_RB_RPTR_ADDR);

        rptr = regs[ADRENO_CP_RB_RPTR / 4];
        if (shadow && !qcom8998_cp_word(s, shadow, &rptr, true)) {
            trace_qcom8998_cp_packet(rptr, wptr, 0, "pointer DMA fault");
        }
    }
}

static void qcom8998_cp_resume(void *opaque)
{
    Qcom8998Display *s = opaque;

    qcom8998_cp_run(&s->bank[BANK_ADRENO]);
}

static uint64_t qcom8998_read(void *opaque, hwaddr offset, unsigned size)
{
    Qcom8998Bank *bank = opaque;
    uint32_t value = bank->regs[offset / sizeof(value)];

    /* The native ISR probes the masked summary before reading raw status. */
    if (bank->index == BANK_ADRENO &&
        offset == ADRENO_RBBM_INT_0_MASKED_STATUS) {
        value = bank->regs[ADRENO_RBBM_INT_0_STATUS / 4] &
                bank->regs[ADRENO_RBBM_INT_0_MASK / 4];
    }
    if (bank->index == BANK_ADRENO_SMMU &&
        offset >= SMMU_ADRENO_CONTEXT_BASE &&
        offset < SMMU_ADRENO_CONTEXT_BASE +
                 SMMU_ADRENO_CONTEXTS * SMMU_CONTEXT_STRIDE &&
        offset % SMMU_CONTEXT_STRIDE == SMMU_CB_FSR) {
        unsigned context = (offset - SMMU_ADRENO_CONTEXT_BASE) /
                           SMMU_CONTEXT_STRIDE;

        if (bank->regs[SMMU_GR1_CBA2R / 4 + context] & SMMU_CBA2R_VA64) {
            value |= SMMU_FSR_FORMAT_AARCH64;
        }
    }

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

static bool qcom8998_smmu_write(Qcom8998Bank *bank, hwaddr offset,
                                uint32_t value)
{
    Qcom8998Display *s = bank->display;
    unsigned context;
    hwaddr cb_offset;

    if (bank->index != BANK_ADRENO_SMMU) {
        return false;
    }
    if (offset == SMMU_GR0_GFSR) {
        bank->regs[offset / 4] &= ~(value & SMMU_GFSR_MASK);
    } else if (offset == SMMU_GR0_GFSRRESTORE) {
        bank->regs[SMMU_GR0_GFSR / 4] = value & SMMU_GFSR_MASK;
    } else if (offset == SMMU_GR0_CR0) {
        bank->regs[offset / 4] = value;
    } else if (offset >= SMMU_ADRENO_CONTEXT_BASE &&
               offset < SMMU_ADRENO_CONTEXT_BASE +
                        SMMU_ADRENO_CONTEXTS * SMMU_CONTEXT_STRIDE) {
        context = (offset - SMMU_ADRENO_CONTEXT_BASE) / SMMU_CONTEXT_STRIDE;
        cb_offset = offset % SMMU_CONTEXT_STRIDE;
        if (cb_offset == SMMU_CB_TLBIALL || cb_offset == SMMU_CB_TLBSYNC ||
            cb_offset == SMMU_CB_TLBSTATUS) {
            /* No TLB/pending invalidation; command/status writes are WI. */
            return true;
        } else if (cb_offset == SMMU_CB_FSR) {
            bank->regs[offset / 4] &= ~(value & SMMU_FSR_FAULT_MASK);
        } else if (cb_offset == SMMU_CB_FSRRESTORE) {
            qcom8998_smmu_context(s, context)[SMMU_CB_FSR / 4] =
                value & SMMU_FSR_FAULT_MASK;
        } else if (cb_offset == SMMU_CB_SCTLR) {
            /* This implementation supports terminating faults only. */
            bank->regs[offset / 4] = value & ~SMMU_SCTLR_CFCFG;
        } else if (cb_offset == 0x8) {
            /* No stalled transaction exists in the terminate-only model. */
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
    qcom8998_smmu_update_irq(s);
    return true;
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
         (offset == ADRENO_RBBM_HW_CONFIGURATION ||
          offset == ADRENO_RBBM_INT_0_STATUS ||
          offset == ADRENO_RBBM_INT_0_MASKED_STATUS)) ||
        (bank->index == BANK_VIDEO && offset == VENUS_WRAPPER_HW_VERSION)) {
        return;
    }
    if (qcom8998_smmu_write(bank, offset, value)) {
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
        qcom8998_mdp_write(bank, offset, value);
        return;
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
    if (bank->index == BANK_ADRENO &&
        (offset == ADRENO_RBBM_INT_CLEAR_CMD ||
         offset == ADRENO_RBBM_INT_0_MASK)) {
        if (offset == ADRENO_RBBM_INT_CLEAR_CMD) {
            bank->regs[ADRENO_RBBM_INT_0_STATUS / 4] &= ~value;
        } else {
            bank->regs[offset / 4] = value;
        }
        qcom8998_adreno_update_irq(bank->display);
        return;
    }
    if (bank->index == BANK_ADRENO &&
        (offset == ADRENO_CP_RB_BASE || offset == ADRENO_CP_RB_BASE + 4 ||
         offset == ADRENO_CP_RB_CNTL || offset == ADRENO_CP_RB_RPTR)) {
        qcom8998_cp_abort(bank->display);
    }
    bank->regs[offset / sizeof(uint32_t)] = value;
    if (bank->index == BANK_ADRENO &&
        (offset == ADRENO_CP_RB_WPTR || offset == ADRENO_CP_PFP_ME_CNTL)) {
        qcom8998_cp_run(bank);
    }
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
     * The Adreno CP uses the guest-programmed stage-1 context banks.
     */
    bank->regs[SMMU_IDR0 / 4] = SMMU_IDR0_STAGE1 | SMMU_IDR0_STREAM_MATCH |
        SMMU_IDR0_CONTEXT_IRQS(1U) | SMMU_IDR0_STREAM_ID_BITS(15U) |
        stream_entries;
    bank->regs[SMMU_IDR1 / 4] = SMMU_IDR1_PAGE_INDEX_BITS(page_index_bits) |
        contexts;
    bank->regs[SMMU_IDR2 / 4] = SMMU_IDR2_4K_PAGES | SMMU_IDR2_VA_48BIT |
        SMMU_IDR2_PA_40BIT | SMMU_IDR2_IPA_40BIT;
    /* Firmware handoff: unknown or conflicting streams terminate. */
    bank->regs[SMMU_GR0_CR0 / 4] = SMMU_CR0_USFCFG | SMMU_CR0_SMCFCFG;
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

    timer_del(&s->mdss_vsync);
    qcom8998_cp_abort(s);
    memset(s->transfer, 0, sizeof(s->transfer));
    for (unsigned panel = 0; panel < MDSS_DSI_COUNT; panel++) {
        if (s->panel[panel]) {
            qemu_graphic_console_close(s->panel[panel]);
            s->panel[panel] = NULL;
        }
    }
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
    qcom8998_smmu_reset(&s->bank[BANK_ADRENO_SMMU], SMMU_ADRENO_STREAMS, 2,
                       SMMU_ADRENO_CONTEXTS);
    s->me_initialized = false;
    s->register_protection = true;
    s->draw_predicate_global = false;
    s->draw_predicate_local = false;
    s->draw_predicate_pass = true;
    s->visibility_override = false;
    s->skip_ib2_global = false;
    s->skip_ib2_local = false;
    memset(s->gmem, 0, ADRENO_GMEM_SIZE);
    memset(s->draw_state, 0, sizeof(s->draw_state));
    memset(s->render_mode, 0, sizeof(s->render_mode));
    memset(s->shader_address, 0, sizeof(s->shader_address));
    for (unsigned block = 0; block < ADRENO_LOAD_BLOCKS; block++) {
        for (unsigned type = 0; type < ADRENO_LOAD_TYPES; type++) {
            if (s->load_state[block][type]) {
                g_array_set_size(s->load_state[block][type], 0);
            }
        }
    }
    s->gpmu_image_dwords = 0;
    s->bank[BANK_ADRENO].regs[ADRENO_GPMU_CM3_SYSRESET / 4] =
        ADRENO_GPMU_RESET_ASSERT;
    qemu_set_irq(s->irq[0], 0);
    qcom8998_adreno_update_irq(s);
    qcom8998_smmu_update_irq(s);
}

static void qcom8998_init(Object *obj)
{
    Qcom8998Display *s = QCOM8998_DISPLAY(obj);
    SysBusDevice *bus = SYS_BUS_DEVICE(obj);

    s->gmem = g_malloc0(ADRENO_GMEM_SIZE);
    timer_init_ns(&s->mdss_vsync, QEMU_CLOCK_VIRTUAL, qcom8998_mdss_vsync, s);
    timer_init_ns(&s->cp_resume, QEMU_CLOCK_VIRTUAL, qcom8998_cp_resume, s);
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

    timer_del(&s->mdss_vsync);
    timer_deinit(&s->mdss_vsync);
    timer_del(&s->cp_resume);
    s->cp_wait_pending = false;
    timer_deinit(&s->cp_resume);
    for (unsigned block = 0; block < ADRENO_LOAD_BLOCKS; block++) {
        for (unsigned type = 0; type < ADRENO_LOAD_TYPES; type++) {
            if (s->load_state[block][type]) {
                g_array_unref(s->load_state[block][type]);
            }
        }
    }
    for (unsigned i = 0; i < BANK_COUNT; i++) {
        g_free(s->bank[i].regs);
    }
    g_free(s->gmem);
}

static const VMStateDescription qcom8998_vmstate = {
    .name = TYPE_QCOM8998_DISPLAY,
    .unmigratable = 1,
};

static const Property qcom8998_properties[] = {
    DEFINE_PROP_BOOL("disable-macro-tiled-primary", Qcom8998Display,
                     disable_macro_tiled_primary, false),
};

static void qcom8998_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, qcom8998_reset);
    dc->desc = "Experimental MSM8998 QCDX MMIO bring-up";
    dc->vmsd = &qcom8998_vmstate;
    device_class_set_props(dc, qcom8998_properties);
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
    static const uint32_t gsiv[] = {
        MDSS_GSIV, ADRENO_GSIV, VIDEO_GSIV, 366, 361, 362,
    };
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
    Qcom8998Display *s = QCOM8998_DISPLAY(
        qdev_find_recursive(sysbus_get_default(), "qcdx8998"));
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
    Aml *settings = aml_package(ARRAY_SIZE(adapter_settings) + 1 +
                                s->disable_macro_tiled_primary);
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
    if (s->disable_macro_tiled_primary) {
        Aml *entry = aml_package(2);

        /* Native KMD option; it changes primary-surface layout policy. */
        aml_append(entry, aml_string("DisableMacroTiledPrimary"));
        aml_append(entry, aml_int(1));
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

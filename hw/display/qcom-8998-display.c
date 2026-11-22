/*
 * MSM8998 display bring-up for the Windows QCDX miniport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
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
#define MDP_PP_AUTOREFRESH 0x030
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
#define ADRENO_RBBM_STATUS (0x4f5 * 4)
#define ADRENO_RBBM_STATUS_GPU_BUSY BIT(31)
#define ADRENO_RBBM_STATUS_CP_BUSY BIT(6)
#define ADRENO_RBBM_STATUS_CP_BUSY_IGN_HYST BIT(7)
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
#define ADRENO_PM4_WAIT_MEM_WRITES 0x12
#define ADRENO_PM4_WAIT_FOR_ME 0x13
#define ADRENO_PM4_WAIT_FOR_IDLE 0x26
#define ADRENO_PM4_SET_BIN_DATA5 0x2f
#define ADRENO_PM4_LOAD_STATE4 0x30
#define ADRENO_PM4_WAIT_REG_MEM 0x3c
#define ADRENO_PM4_DRAW_INDX_OFFSET 0x38
#define ADRENO_PM4_REG_TEST 0x39
#define ADRENO_PM4_COND_REG_EXEC 0x47
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
#define ADRENO_RENDER_BINNING 2
#define ADRENO_RENDER_GMEM 3
/* VSC hardware-binning configuration registers, in PM4 dword index space. */
#define ADRENO_VSC_ADDR_MODE_CNTL 0xbc1
#define ADRENO_VSC_UNKNOWN_0BC6 0xbc6
#define ADRENO_VSC_PIPE_CONFIG_BASE 0xbd0
#define ADRENO_VSC_PIPE_DATA_LENGTH_END 0xc0f
#define ADRENO_VSC_RESOLVE_CNTL 0xcdd
/* QCDX tests bit 0 after selecting a bin with CP_SET_BIN_DATA5. */
#define ADRENO_VSC_BIN_VISIBILITY 0xc10
#define ADRENO_VSC_BIN_VISIBILITY_END 0xc1f
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
#define ADRENO_UBO_STAGE_SLOTS 20
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
#define ADRENO_EVENT_VSC_BINNING_START 0x2c
#define ADRENO_EVENT_VSC_BINNING_END 0x2d
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
#define SMMU_TCR2_SEP_32BIT 0
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

#define MDP_MAX_PIPES 14
#define SMMU_MDSS_STREAMS 60
#define SMMU_MDSS_CONTEXTS 16
#define SMMU_MDSS_CONTEXT_BASE 0x20000

typedef struct Qcom8998MdpPipe {
    unsigned base;
    unsigned stage;
    unsigned src_x;
    unsigned src_y;
    unsigned src_w;
    unsigned src_h;
    unsigned out_x;
    unsigned out_y;
    unsigned out_w;
    unsigned out_h;
    uint32_t src_addr;
    uint32_t stride;
    uint32_t format;
    uint32_t unpack;
    uint32_t op_mode;
} Qcom8998MdpPipe;

typedef struct Qcom8998MdpTransfer {
    unsigned panel;
    unsigned width;
    unsigned height;
    uint32_t color;
    uint32_t flush;
    unsigned pipe_count;
    Qcom8998MdpPipe pipes[MDP_MAX_PIPES];
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
    bool ubo;
    uint32_t ubo_offset;
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
    bool register_predicate;
    QemuConsole *panel[MDSS_DSI_COUNT];
    QemuConsole *merged_panel;
    uint32_t *panel_scratch[MDSS_DSI_COUNT];
    unsigned panel_scratch_pixels[MDSS_DSI_COUNT];
    bool panel_needs_full_update[MDSS_DSI_COUNT];
    bool merged_needs_full_update;
    bool merged_view;
    Qcom8998MdpTransfer transfer[MDSS_CTL_COUNT];
    bool autorefresh[MDSS_CTL_COUNT];
    unsigned refresh_tick[MDSS_CTL_COUNT];
    bool gpu_dirty;
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
    uint32_t render_mode[8];
    bool vsc_binning;
    bool bin_data_valid;
    uint32_t bin_data[5];
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

static uint64_t qcom8998_reg_pair(const uint32_t *regs, hwaddr offset);
static bool qcom8998_ram_buffer(hwaddr address, size_t size, bool write);
static uint8_t *qcom8998_ram_ptr(hwaddr address, size_t size, bool write);
static bool qcom8998_ram_access(hwaddr address, void *data, size_t size,
                                bool write);

static bool qcom8998_mdss_smmu_translate(Qcom8998Display *s, uint64_t iova,
                                         hwaddr *out)
{
    static const unsigned pa_width[] = { 32, 36, 40 };
    static const uint32_t stream_ids[] = { 0x000, 0xe00 };
    const uint32_t *smmu = s->bank[BANK_MDSS_SMMU].regs;
    const uint32_t *cb = NULL;
    unsigned pasize, va_bits;
    uint32_t tcr, tcr2;
    uint64_t table, pa_mask, va_mask, invalid_pa;
    bool upper;

    if (smmu[SMMU_GR0_CR0 / 4] & SMMU_CR0_CLIENTPD) {
        *out = iova;
        return true;
    }
    for (unsigned si = 0; si < ARRAY_SIZE(stream_ids) && !cb; si++) {
        for (unsigned i = 0; i < SMMU_MDSS_STREAMS; i++) {
            uint32_t smr = smmu[SMMU_GR0_SMR / 4 + i];
            uint32_t mask = (smr >> 16) & SMMU_SMR_ID_MASK;
            uint32_t s2cr;
            unsigned type, context;

            if (!(smr & SMMU_SMR_VALID) ||
                ((smr ^ stream_ids[si]) & SMMU_SMR_ID_MASK & ~mask)) {
                continue;
            }
            s2cr = smmu[SMMU_GR0_S2CR / 4 + i];
            type = extract32(s2cr, 16, 2);
            if (type == 1) {
                *out = iova;
                return true;
            }
            if (type != SMMU_S2CR_TYPE_TRANSLATE) {
                return false;
            }
            context = extract32(s2cr, 0, 8);
            if (context >= SMMU_MDSS_CONTEXTS) {
                return false;
            }
            cb = smmu + (SMMU_MDSS_CONTEXT_BASE +
                         context * SMMU_CONTEXT_STRIDE) / 4;
            break;
        }
    }
    if (!cb) {
        return false;
    }
    if (!(cb[SMMU_CB_SCTLR / 4] & SMMU_SCTLR_M)) {
        *out = iova;
        return true;
    }
    tcr = cb[SMMU_CB_TCR / 4];
    tcr2 = cb[SMMU_CB_TCR2 / 4];
    if (cb[SMMU_CB_SCTLR / 4] & BIT(4)) {
        return false;
    }
    switch (extract32(tcr2, 15, 3)) {
    case SMMU_TCR2_SEP_32BIT:
        if (iova >> 32) {
            return false;
        }
        break;
    case SMMU_TCR2_SEP_36BIT:
        if (iova >> 36) {
            return false;
        }
        iova = sextract64(iova, 0, 36);
        break;
    case SMMU_TCR2_SEP_UPSTREAM:
        break;
    default:
        return false;
    }
    upper = iova >> 63;
    if (tcr & (upper ? SMMU_TCR_EPD1 : SMMU_TCR_EPD0)) {
        return false;
    }
    if (upper ? extract32(tcr, 30, 2) != SMMU_TCR_TG1_4K :
                extract32(tcr, 14, 2) != SMMU_TCR_TG0_4K) {
        return false;
    }
    va_bits = 64 - extract32(tcr, upper ? 16 : 0, 6);
    pasize = extract32(tcr2, 0, 4);
    if (va_bits < 25 || va_bits > 48 || pasize >= ARRAY_SIZE(pa_width)) {
        return false;
    }
    va_mask = MAKE_64BIT_MASK(0, va_bits);
    if ((iova & ~va_mask) != (upper ? ~va_mask : 0)) {
        return false;
    }
    pa_mask = MAKE_64BIT_MASK(0, pa_width[pasize]);
    invalid_pa = MAKE_64BIT_MASK(0, SMMU_LONG_DESC_PA_BITS) & ~pa_mask;
    table = qcom8998_reg_pair(cb, upper ? SMMU_CB_TTBR1 : SMMU_CB_TTBR0);
    for (unsigned shift = SMMU_4K_SHIFT + SMMU_4K_LEVEL_BITS *
                          ((va_bits - SMMU_4K_SHIFT - 1) / SMMU_4K_LEVEL_BITS);
         ; shift -= SMMU_4K_LEVEL_BITS) {
        unsigned bits = MIN(SMMU_4K_LEVEL_BITS, va_bits - shift);
        unsigned index = extract64(iova, shift, bits);
        uint64_t pte;
        unsigned type;

        if (table & invalid_pa) {
            return false;
        }
        table &= pa_mask & ~MAKE_64BIT_MASK(0, bits + 3);
        if (!qcom8998_ram_access(table + index * sizeof(pte), &pte,
                                 sizeof(pte), false)) {
            return false;
        }
        pte = le64_to_cpu(pte);
        type = pte & SMMU_PTE_TYPE_MASK;
        if (shift > SMMU_4K_SHIFT && type == SMMU_PTE_TABLE_OR_PAGE) {
            table = pte & ~MAKE_64BIT_MASK(0, SMMU_4K_SHIFT);
            continue;
        }
        if (type != (shift == SMMU_4K_SHIFT ? SMMU_PTE_TABLE_OR_PAGE :
                     SMMU_PTE_BLOCK) || shift == 39) {
            return false;
        }
        if ((pte & invalid_pa) || !(pte & SMMU_PTE_ACCESS_FLAG)) {
            return false;
        }
        *out = (pte & pa_mask & ~MAKE_64BIT_MASK(0, shift)) |
               extract64(iova, 0, shift);
        return true;
    }
}

static const uint8_t *qcom8998_mdss_page(Qcom8998Display *s, GHashTable *pages,
                                         uint64_t address)
{
    uint64_t page = address & ~((1ULL << SMMU_4K_SHIFT) - 1);
    gpointer key = (gpointer)(uintptr_t)(page >> SMMU_4K_SHIFT);
    const uint8_t *data = g_hash_table_lookup(pages, key);

    if (!data) {
        hwaddr physical;

        if (!qcom8998_mdss_smmu_translate(s, page, &physical)) {
            return NULL;
        }
        physical &= ~((1ULL << SMMU_4K_SHIFT) - 1);
        data = qcom8998_ram_ptr(physical, 1U << SMMU_4K_SHIFT, false);
        if (!data) {
            return NULL;
        }
        g_hash_table_insert(pages, key, (gpointer)data);
    }
    return data;
}

static uint32_t qcom8998_mdss_unpack_pixel(uint32_t raw, uint32_t unpack)
{
    uint8_t bytes[4] = {
        raw & 0xff, (raw >> 8) & 0xff,
        (raw >> 16) & 0xff, (raw >> 24) & 0xff,
    };
    uint8_t comp[4] = { 0, 0, 0, 0xff };

    for (unsigned i = 0; i < 4; i++) {
        comp[extract32(unpack, i * 8, 2)] = bytes[i];
    }
    /* comp: 0=G, 1=B, 2=R, 3=A -> 0xAARRGGBB */
    return ((uint32_t)comp[3] << 24) | ((uint32_t)comp[2] << 16) |
           ((uint32_t)comp[0] << 8) | comp[1];
}

static uint32_t qcom8998_mdss_blend_over(uint32_t dst, uint32_t src)
{
    unsigned a = src >> 24;
    unsigned inv = 255 - a;
    unsigned sr = (src >> 16) & 0xff, sg = (src >> 8) & 0xff, sb = src & 0xff;
    unsigned dr = (dst >> 16) & 0xff, dg = (dst >> 8) & 0xff, db = dst & 0xff;
    unsigned r = MIN(255U, sr + (dr * inv + 127) / 255);
    unsigned g = MIN(255U, sg + (dg * inv + 127) / 255);
    unsigned b = MIN(255U, sb + (db * inv + 127) / 255);

    return (r << 16) | (g << 8) | b;
}

static void qcom8998_mdss_scanout_pipe(Qcom8998Display *s,
                                       uint32_t *dst_buf,
                                       unsigned width, unsigned height,
                                       const Qcom8998MdpPipe *pipe,
                                       GHashTable *pages)
{
    unsigned cpp = extract32(pipe->format, 9, 2) + 1;
    bool alpha_en = !!(pipe->format & BIT(8));
    unsigned fetch_type = extract32(pipe->format, 19, 2);
    bool bwc = !!(pipe->op_mode & BIT(0));
    bool blend = pipe->stage > 2 && alpha_en;
    bool fast_bgra = pipe->unpack == 0x03020001;
    unsigned out_x1, out_y1, out_x2, out_y2;

    if (cpp != 4 || fetch_type != 0 || bwc || !pipe->src_w || !pipe->src_h ||
        !pipe->out_w || !pipe->out_h || !pipe->stride) {
        return;
    }
    out_x1 = pipe->out_x;
    out_y1 = pipe->out_y;
    out_x2 = MIN(width, pipe->out_x + pipe->out_w);
    out_y2 = MIN(height, pipe->out_y + pipe->out_h);
    if (out_x1 >= out_x2 || out_y1 >= out_y2) {
        return;
    }
    bool unscaled_x = pipe->src_w == pipe->out_w;
    bool unscaled_y = pipe->src_h == pipe->out_h;
    uint64_t cached_pg = UINT64_MAX;
    const uint8_t *cached_data = NULL;

    for (unsigned y = out_y1; y < out_y2; y++) {
        unsigned oy = y - pipe->out_y;
        unsigned sy = unscaled_y ? (pipe->src_y + oy) :
                      (pipe->src_y + (uint64_t)oy * pipe->src_h / pipe->out_h);
        uint64_t row_addr = (uint64_t)pipe->src_addr +
                            (uint64_t)sy * pipe->stride;
        uint32_t *dst_row = dst_buf + (size_t)y * width;

        if (unscaled_x && fast_bgra && !blend) {
            unsigned x = out_x1;

            while (x < out_x2) {
                uint64_t paddr = row_addr +
                                 (uint64_t)(pipe->src_x + (x - pipe->out_x)) * 4;
                uint64_t pg = paddr & ~((1ULL << SMMU_4K_SHIFT) - 1);
                unsigned off = paddr & 4095;
                unsigned chunk = MIN(out_x2 - x, (4096U - off) / 4);

                if (pg != cached_pg) {
                    cached_pg = pg;
                    cached_data = qcom8998_mdss_page(s, pages, paddr);
                }
                if (cached_data) {
                    const uint8_t *src = cached_data + off;

                    for (unsigned k = 0; k < chunk; k++) {
                        dst_row[x + k] = ldl_le_p(src + k * 4) & 0x00ffffff;
                    }
                }
                x += chunk;
            }
            continue;
        }

        for (unsigned x = out_x1; x < out_x2; x++) {
            unsigned ox = x - pipe->out_x;
            unsigned sx = unscaled_x ? (pipe->src_x + ox) :
                          (pipe->src_x + (uint64_t)ox * pipe->src_w /
                                         pipe->out_w);
            uint64_t paddr = row_addr + (uint64_t)sx * 4;
            uint64_t pg = paddr & ~((1ULL << SMMU_4K_SHIFT) - 1);
            uint32_t raw, argb;

            if (pg != cached_pg) {
                cached_pg = pg;
                cached_data = qcom8998_mdss_page(s, pages, paddr);
            }
            if (!cached_data) {
                continue;
            }
            raw = ldl_le_p(cached_data + (paddr & 4095));
            argb = fast_bgra ? raw : qcom8998_mdss_unpack_pixel(raw,
                                                                pipe->unpack);
            if (blend) {
                dst_row[x] = qcom8998_mdss_blend_over(dst_row[x], argb);
            } else {
                dst_row[x] = argb & 0x00ffffff;
            }
        }
    }
}

static void qcom8998_mdss_queue_transfer(Qcom8998Bank *bank, unsigned ctl,
                                         bool force)
{
    static const struct {
        unsigned base;
        unsigned reg; /* 0 = LAYER + LAYER_EXT bit3, 1 = LAYER_EXT, 2 = LAYER_EXT2 */
        unsigned shift;
        unsigned ext_bit;
    } pipe_map[MDP_MAX_PIPES] = {
        { 0x05000, 0,  0,  0 }, /* VIG0 */
        { 0x07000, 0,  3,  2 }, /* VIG1 */
        { 0x09000, 0,  6,  4 }, /* VIG2 */
        { 0x0b000, 0, 26,  6 }, /* VIG3 */
        { 0x15000, 0,  9,  8 }, /* RGB0 */
        { 0x17000, 0, 12, 10 }, /* RGB1 */
        { 0x19000, 0, 15, 12 }, /* RGB2 */
        { 0x1b000, 0, 29, 14 }, /* RGB3 */
        { 0x25000, 0, 18, 16 }, /* DMA0 */
        { 0x27000, 0, 21, 18 }, /* DMA1 */
        { 0x29000, 2,  0,  0 }, /* DMA2 */
        { 0x2b000, 2,  4,  0 }, /* DMA3 */
        { 0x35000, 1, 20,  0 }, /* CURSOR0 */
        { 0x37000, 1, 26,  0 }, /* CURSOR1 */
    };
    Qcom8998MdpTransfer *transfer = &bank->display->transfer[ctl];
    const uint32_t *control =
        &bank->regs[(MDSS_CTL_BASE + ctl * MDSS_CTL_STRIDE) / 4];
    int panel = qcom8998_mdss_ctl_panel(control[MDSS_CTL_TOP / 4]);
    const uint32_t *mixer;
    uint32_t size, color_0, color_1, layer, ext, ext2;
    unsigned width, height, count = 0;

    if (panel < 0 || transfer->flush ||
        (!force && !control[MDSS_CTL_FLUSH / 4])) {
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

    layer = control[panel];
    ext = control[MDSS_CTL_LAYER_EXT / 4 + panel];
    ext2 = control[MDSS_CTL_LAYER_EXT2 / 4 + panel];
    for (unsigned i = 0; i < ARRAY_SIZE(pipe_map); i++) {
        unsigned stage;
        const uint32_t *p;
        Qcom8998MdpPipe entry;

        if (pipe_map[i].reg == 0) {
            stage = extract32(layer, pipe_map[i].shift, 3) |
                    (extract32(ext, pipe_map[i].ext_bit, 1) << 3);
        } else if (pipe_map[i].reg == 1) {
            stage = extract32(ext, pipe_map[i].shift, 4);
        } else {
            stage = extract32(ext2, pipe_map[i].shift, 4);
        }
        if (stage < 2) {
            continue;
        }
        p = &bank->regs[pipe_map[i].base / 4];
        entry = (Qcom8998MdpPipe) {
            .base = pipe_map[i].base,
            .stage = stage,
            .src_w = extract32(p[0], 0, 16),
            .src_h = extract32(p[0], 16, 16),
            .src_x = extract32(p[2], 0, 16),
            .src_y = extract32(p[2], 16, 16),
            .out_w = extract32(p[3], 0, 16),
            .out_h = extract32(p[3], 16, 16),
            .out_x = extract32(p[4], 0, 16),
            .out_y = extract32(p[4], 16, 16),
            .src_addr = p[5],
            .stride = extract32(p[9], 0, 16),
            .format = p[12],
            .unpack = p[13],
            .op_mode = p[14],
        };
        /* Insertion-sort by ascending mixer stage. */
        unsigned pos = count++;
        while (pos > 0 && transfer->pipes[pos - 1].stage > entry.stage) {
            transfer->pipes[pos] = transfer->pipes[pos - 1];
            pos--;
        }
        transfer->pipes[pos] = entry;
    }
    transfer->pipe_count = count;
}

static void qcom8998_mdss_update_merged(Qcom8998Display *s,
                                        unsigned updated_panel,
                                        unsigned first_changed,
                                        unsigned last_changed)
{
    DisplaySurface *s0, *s1, *ms;
    unsigned w0, h0, w1, h1, merged_w, merged_h;

    if (!s->merged_view || !s->panel[0] || !s->panel[1]) {
        return;
    }
    s0 = qemu_console_surface(s->panel[0]);
    s1 = qemu_console_surface(s->panel[1]);
    if (!s0 || !s1 || surface_is_placeholder(s0) ||
        surface_is_placeholder(s1)) {
        return;
    }
    w0 = surface_width(s0);
    h0 = surface_height(s0);
    w1 = surface_width(s1);
    h1 = surface_height(s1);
    if (!w0 || !h0 || !w1 || !h1) {
        return;
    }
    merged_w = w0 + w1;
    merged_h = MAX(h0, h1);
    if (!s->merged_panel) {
        s->merged_panel = qemu_graphic_console_create(
            DEVICE(s), MDSS_DSI_COUNT, &qcom8998_panel_ops, s);
        s->merged_needs_full_update = true;
    }
    ms = qemu_console_surface(s->merged_panel);
    if (!ms || surface_width(ms) != merged_w ||
        surface_height(ms) != merged_h || surface_is_placeholder(ms)) {
        s->merged_needs_full_update = true;
        qemu_console_resize(s->merged_panel, merged_w, merged_h);
        ms = qemu_console_surface(s->merged_panel);
    }
    if (s->merged_needs_full_update) {
        uint8_t *dst = surface_data(ms);
        unsigned dst_stride = surface_stride(ms);

        if (h0 != h1) {
            memset(dst, 0, (size_t)merged_h * dst_stride);
        }
        for (unsigned y = 0; y < h0; y++) {
            memcpy(dst + (size_t)y * dst_stride,
                   surface_data(s0) + (size_t)y * surface_stride(s0),
                   (size_t)w0 * sizeof(uint32_t));
        }
        for (unsigned y = 0; y < h1; y++) {
            memcpy(dst + (size_t)y * dst_stride +
                       (size_t)w0 * sizeof(uint32_t),
                   surface_data(s1) + (size_t)y * surface_stride(s1),
                   (size_t)w1 * sizeof(uint32_t));
        }
        qemu_console_update(s->merged_panel, 0, 0, merged_w, merged_h);
        s->merged_needs_full_update = false;
        return;
    }
    if (last_changed <= first_changed) {
        return;
    }
    {
        DisplaySurface *src = updated_panel == 0 ? s0 : s1;
        unsigned x_off = updated_panel == 0 ? 0 : w0;
        unsigned copy_w = updated_panel == 0 ? w0 : w1;
        unsigned copy_h = updated_panel == 0 ? h0 : h1;
        unsigned y_end = MIN(last_changed, copy_h);
        uint8_t *dst = surface_data(ms);
        unsigned dst_stride = surface_stride(ms);
        const uint8_t *src_data = surface_data(src);
        unsigned src_stride = surface_stride(src);

        for (unsigned y = first_changed; y < y_end; y++) {
            memcpy(dst + (size_t)y * dst_stride +
                       (size_t)x_off * sizeof(uint32_t),
                   src_data + (size_t)y * src_stride,
                   (size_t)copy_w * sizeof(uint32_t));
        }
        if (y_end > first_changed) {
            qemu_console_update(s->merged_panel, x_off, first_changed,
                                copy_w, y_end - first_changed);
        }
    }
}

static uint32_t qcom8998_mdss_transfer(Qcom8998Display *s, uint32_t active)
{
    uint32_t pending = 0;
    bool gpu_dirty = s->gpu_dirty;
    bool scanned_autorefresh = false;

    for (unsigned ctl = 0; ctl < MDSS_CTL_COUNT; ctl++) {
        Qcom8998MdpTransfer *transfer = &s->transfer[ctl];
        hwaddr base = MDSS_CTL_BASE + ctl * MDSS_CTL_STRIDE;
        uint32_t *control = &s->bank[BANK_MDSS].regs[base / 4];
        int ctl_panel = qcom8998_mdss_ctl_panel(control[MDSS_CTL_TOP / 4]);
        bool explicit_flush = transfer->flush != 0;
        bool periodic = false;
        unsigned panel, pixels;
        bool full_opaque = false;
        unsigned first_changed, last_changed;
        DisplaySurface *surface;
        uint32_t *scratch;

        if (!explicit_flush && s->autorefresh[ctl] && ctl_panel >= 0 &&
            (active & MDP_PP_RD_PTR_IRQ(ctl_panel))) {
            if (gpu_dirty || ++s->refresh_tick[ctl] >= 15) {
                s->refresh_tick[ctl] = 0;
                qcom8998_mdss_queue_transfer(&s->bank[BANK_MDSS], ctl, true);
                periodic = transfer->pipe_count > 0 &&
                           transfer->width > 0 && transfer->height > 0;
                transfer->flush = 0;
                scanned_autorefresh = true;
            }
        }
        panel = transfer->panel;
        if ((!explicit_flush && !periodic) ||
            !(active & MDP_PP_RD_PTR_IRQ(panel))) {
            continue;
        }
        if (!s->panel[panel]) {
            s->panel[panel] = qemu_graphic_console_create(DEVICE(s), panel,
                                                        &qcom8998_panel_ops, s);
            s->panel_needs_full_update[panel] = true;
        }
        surface = qemu_console_surface(s->panel[panel]);
        if (!surface || surface_width(surface) != transfer->width ||
            surface_height(surface) != transfer->height ||
            surface_is_placeholder(surface)) {
            s->panel_needs_full_update[panel] = true;
            qemu_console_resize(s->panel[panel], transfer->width,
                                transfer->height);
            surface = qemu_console_surface(s->panel[panel]);
        }
        pixels = transfer->width * transfer->height;
        if (s->panel_scratch_pixels[panel] < pixels) {
            g_free(s->panel_scratch[panel]);
            s->panel_scratch[panel] = g_new0(uint32_t, pixels);
            s->panel_scratch_pixels[panel] = pixels;
        }
        scratch = s->panel_scratch[panel];
        if (transfer->pipe_count) {
            const Qcom8998MdpPipe *p0 = &transfer->pipes[0];
            unsigned cpp0 = extract32(p0->format, 9, 2) + 1;
            bool blend0 = p0->stage > 2 && !!(p0->format & BIT(8));

            full_opaque = cpp0 == 4 && extract32(p0->format, 19, 2) == 0 &&
                          !(p0->op_mode & BIT(0)) && !blend0 &&
                          p0->src_w > 0 && p0->src_h > 0 && p0->stride > 0 &&
                          p0->out_x == 0 && p0->out_y == 0 &&
                          p0->out_w >= transfer->width &&
                          p0->out_h >= transfer->height;
        }
        if (!full_opaque) {
            if (transfer->color == 0) {
                memset(scratch, 0, (size_t)pixels * sizeof(uint32_t));
            } else {
                for (unsigned i = 0; i < pixels; i++) {
                    scratch[i] = transfer->color;
                }
            }
        }
        if (transfer->pipe_count) {
            g_autoptr(GHashTable) pages = g_hash_table_new(g_direct_hash,
                                                           g_direct_equal);

            for (unsigned i = 0; i < transfer->pipe_count; i++) {
                const Qcom8998MdpPipe *pipe = &transfer->pipes[i];

                qcom8998_mdss_scanout_pipe(s, scratch, transfer->width,
                                           transfer->height, pipe, pages);
                if (explicit_flush) {
                    s->bank[BANK_MDSS].regs[(pipe->base + 0xa4) / 4] =
                        pipe->src_addr;
                    trace_qcom8998_mdss_pipe_scanout(ctl, panel, pipe->base,
                        pipe->stage, pipe->src_addr, pipe->src_w, pipe->src_h,
                        pipe->out_w, pipe->out_h);
                }
            }
        }
        first_changed = transfer->height;
        last_changed = 0;
        for (unsigned y = 0; y < transfer->height; y++) {
            uint32_t *dst_row = (uint32_t *)(surface_data(surface) +
                                             y * surface_stride(surface));
            const uint32_t *src_row = scratch + (size_t)y * transfer->width;
            size_t row_bytes = (size_t)transfer->width * sizeof(uint32_t);

            if (s->panel_needs_full_update[panel] ||
                memcmp(dst_row, src_row, row_bytes) != 0) {
                memcpy(dst_row, src_row, row_bytes);
                if (first_changed == transfer->height) {
                    first_changed = y;
                }
                last_changed = y + 1;
            }
        }
        if (last_changed > first_changed) {
            qemu_console_update(s->panel[panel], 0, first_changed,
                                transfer->width, last_changed - first_changed);
        }
        s->panel_needs_full_update[panel] = false;
        qcom8998_mdss_update_merged(s, panel, first_changed, last_changed);
        if (explicit_flush) {
            trace_qcom8998_mdss_transfer_done(ctl, panel, transfer->width,
                                              transfer->height,
                                              transfer->color);
            control[MDSS_CTL_FLUSH / 4] &= ~transfer->flush;
            control[MDSS_CTL_START / 4] &= ~MDSS_CTL_START_REQUEST;
            transfer->flush = 0;
            s->refresh_tick[ctl] = 0;
            pending |= MDP_PP_DONE_IRQ(panel);
        }
    }
    if (scanned_autorefresh) {
        s->gpu_dirty = false;
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
        trace_qcom8998_mdss_ctl_write(offset, value);
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

        if (reg == MDSS_CTL_FLUSH || reg == MDSS_CTL_START ||
            reg == MDSS_CTL_TOP || reg == MDSS_CTL_SW_RESET) {
            trace_qcom8998_mdss_ctl_write(offset, value);
        }

        if (reg == MDSS_CTL_SW_RESET) {
            if (value & MDSS_CTL_SW_RESET_REQUEST) {
                /* Only this control path is reset. */
                memset(&bank->regs[(offset - reg) / 4], 0, MDSS_CTL_REG_SIZE);
                bank->display->transfer[ctl].flush = 0;
                bank->display->autorefresh[ctl] = false;
                bank->display->refresh_tick[ctl] = 0;
                qcom8998_mdss_vsync_update(bank);
            }
            return;
        }
        if (reg == MDSS_CTL_TOP) {
            if (qcom8998_mdss_ctl_panel(bank->regs[offset / 4]) !=
                qcom8998_mdss_ctl_panel(value)) {
                bank->display->transfer[ctl].flush = 0;
                bank->display->autorefresh[ctl] = false;
                bank->display->refresh_tick[ctl] = 0;
                bank->regs[(offset - reg + MDSS_CTL_START) / 4] = 0;
            }
            bank->regs[offset / 4] = value;
            qcom8998_mdss_vsync_update(bank);
            return;
        }
        if (reg == MDSS_CTL_START && (value & MDSS_CTL_START_REQUEST)) {
            int panel = qcom8998_mdss_ctl_panel(
                bank->regs[(offset - reg + MDSS_CTL_TOP) / 4]);

            qcom8998_mdss_queue_transfer(bank, ctl, false);
            if (panel >= 0 &&
                bank->regs[(MDP_PP_BASE + panel * MDP_PP_STRIDE +
                            MDP_PP_AUTOREFRESH) / 4]) {
                bank->display->autorefresh[ctl] = true;
            }
        }
    }
    bank->regs[offset / 4] = value;
    if (offset >= MDP_PP_BASE &&
        offset < MDP_PP_BASE + MDSS_DSI_COUNT * MDP_PP_STRIDE) {
        hwaddr reg = (offset - MDP_PP_BASE) % MDP_PP_STRIDE;
        unsigned pp_panel = (offset - MDP_PP_BASE) / MDP_PP_STRIDE;

        if (reg == 0 || reg == MDP_PP_SYNC_CONFIG ||
            reg == MDP_PP_SYNC_HEIGHT || reg == MDP_PP_AUTOREFRESH) {
            trace_qcom8998_mdss_ctl_write(offset, value);
        }
        if (reg == MDP_PP_AUTOREFRESH && !value) {
            for (unsigned ctl = 0; ctl < MDSS_CTL_COUNT; ctl++) {
                uint32_t top = bank->regs[(MDSS_CTL_BASE +
                    ctl * MDSS_CTL_STRIDE + MDSS_CTL_TOP) / 4];

                if (qcom8998_mdss_ctl_panel(top) == (int)pp_panel) {
                    bank->display->autorefresh[ctl] = false;
                }
            }
        }
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

static uint8_t *qcom8998_ram_ptr(hwaddr address, size_t size, bool write)
{
    hwaddr length = size, translated;
    MemoryRegion *mr;

    mr = address_space_translate(&address_space_memory, address, &translated,
                                 &length, write, MEMTXATTRS_UNSPECIFIED);
    if (!memory_region_is_ram(mr) || memory_region_is_ram_device(mr) ||
        memory_region_is_rom(mr) || length != size) {
        return NULL;
    }
    return (uint8_t *)memory_region_get_ram_ptr(mr) + translated;
}

static bool qcom8998_ram_buffer(hwaddr address, size_t size, bool write)
{
    return qcom8998_ram_ptr(address, size, write) != NULL;
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
    case SMMU_TCR2_SEP_32BIT:
        if (iova >> 32) {
            return false;
        }
        break;
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
         reg <= ADRENO_UCHE_INVALIDATE) ||
        (reg >= ADRENO_VSC_ADDR_MODE_CNTL && reg <= ADRENO_VSC_UNKNOWN_0BC6) ||
        (reg >= ADRENO_VSC_PIPE_CONFIG_BASE &&
         reg <= ADRENO_VSC_PIPE_DATA_LENGTH_END) ||
        reg == ADRENO_VSC_RESOLVE_CNTL) {
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
    trace_qcom8998_cp_load_state_request(count, header[0], header[1], header[2]);
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
        unit_words = type == 0 ? 4 :
                     type == 1 ? (source == ADRENO_LOAD_SOURCE_INDIRECT ?
                                  16 : 12) :
                     2;
    } else if (block >= 8 && block < ADRENO_LOAD_BLOCKS) {
        unit_words = type == 0 ? ADRENO_LOAD_MAX_WORDS : type == 1 ? 4 : 2;
    } else {
        /* SSBOs and other undocumented blocks need their own layout. */
        return false;
    }
    words = units * unit_words;
    if (source == ADRENO_LOAD_SOURCE_UBO) {
        unsigned slot = address >> 16;
        unsigned offset = address & 0xffff;

        /*
         * Source 3 binds constants to the shared UBO table. QCDX emits this
         * binding BEFORE uploading the current table; reading it here can
         * DMA through a stale address from the previous context. Resolve it
         * only when a draw actually reads the constant. EXT_SRC_ADDR packs
         * slot:16 and byte offset:16; one unit is a vec4.
         */
        if (type != 1 || block < 8 || block > 12 || address >> 32 ||
            slot >= ADRENO_UBO_STAGE_SLOTS || (offset & 15) || count != 3 ||
            units * 16 > 0x10000 - offset) {
            return false;
        }
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
    if (source != ADRENO_LOAD_SOURCE_UBO) {
        pending = g_new(uint32_t, words);
    }
    for (unsigned i = 0; pending && i < words; i++) {
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

        unit->ubo = source == ADRENO_LOAD_SOURCE_UBO;
        unit->ubo_offset = unit->ubo ? address + i * 16 : 0;
        if (unit->ubo) {
            memset(unit->words, 0, sizeof(unit->words));
        } else {
            memcpy(unit->words, pending + i * unit_words,
                   unit_words * sizeof(uint32_t));
        }
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

#define ADRENO_PAGE_CACHE_WAYS 16
#define ADRENO_PAGE_CACHE_MASK (ADRENO_PAGE_CACHE_WAYS - 1)

typedef struct Qcom8998PageCache {
    GHashTable *table;
    uint64_t pg[ADRENO_PAGE_CACHE_WAYS];
    uint8_t *data[ADRENO_PAGE_CACHE_WAYS];
} Qcom8998PageCache;

static inline void qcom8998_page_cache_init(Qcom8998PageCache *c,
                                            GHashTable *table)
{
    c->table = table;
    for (unsigned i = 0; i < ADRENO_PAGE_CACHE_WAYS; i++) {
        c->pg[i] = UINT64_MAX;
        c->data[i] = NULL;
    }
}

static inline uint64_t qcom8998_tile5_3_block_offset_fast(
    unsigned x_block, unsigned y_block, uint64_t macrotile_stride,
    unsigned bank_mask, unsigned hbb)
{
    uint64_t x_xor = (((uint64_t)(x_block & 1) * 7) ^ (x_block & 2) ^
                      ((uint64_t)(x_block >> 1) << 3)) << 8;
    uint64_t y_xor = ((((uint64_t)(y_block & 1) * 6) ^
                       (((y_block >> 1) & 1) * 3)) << 8) |
                     ((uint64_t)(y_block & (bank_mask << 2)) << (hbb - 3));
    return (uint64_t)(y_block >> 2) * macrotile_stride * 4096 + (x_xor ^ y_xor);
}

static inline void qcom8998_tile5_3_params(uint64_t pitch, unsigned cpp,
                                           unsigned hbb,
                                           uint64_t *macrotile_stride_out,
                                           unsigned *bank_mask_out)
{
    unsigned mt_bytes = cpp == 1 ? 128 : 256;
    uint64_t macrotile_stride = pitch / mt_bytes;
    unsigned bank_offset = cpp == 1 ? 1 : 0;
    unsigned bank_mask = 0;

    if (!(macrotile_stride & ((1ULL << (hbb - 12 + bank_offset)) - 1))) {
        bank_mask |= 1;
    }
    if (!(macrotile_stride & ((1ULL << (hbb - 11 + bank_offset)) - 1))) {
        bank_mask |= 2;
    }
    if (!(macrotile_stride & ((1ULL << (hbb - 10 + bank_offset)) - 1))) {
        bank_mask |= 4;
    }
    *macrotile_stride_out = macrotile_stride;
    *bank_mask_out = bank_mask;
}

static inline const uint8_t *qcom8998_texture_page(Qcom8998Display *s,
                                                   Qcom8998PageCache *cache,
                                                   uint64_t address);
static uint8_t *qcom8998_stage_write_page(Qcom8998Display *s,
                                          Qcom8998PageCache *cache,
                                          uint64_t address);
static bool qcom8998_publish_write_pages(GHashTable *pages);
static uint64_t qcom8998_tile5_3_block_offset(unsigned x_block,
                                              unsigned y_block,
                                              uint64_t pitch,
                                              unsigned cpp,
                                              unsigned hbb);
static inline unsigned qcom8998_tile5_3_pixel_index(unsigned lx, unsigned ly)
{
    return (lx & 1) | ((ly & 1) << 1) | (((lx & 2) >> 1) << 2) |
           (((ly & 2) >> 1) << 3) | (((lx & 0x1c) >> 2) << 4) |
           (((ly & 4) >> 2) << 7);
}

static inline unsigned qcom8998_rb_format_cpp(unsigned fmt)
{
    if (fmt == ADRENO_RB_RGBA8_UNORM) {
        return 4;
    }
    if (fmt == 0x0f) {
        return 2;
    }
    if (fmt == 0x02 || fmt == 0x03) {
        return 1;
    }
    return 0;
}

/*
 * GMEM is private: keep logical texels (1, 2, or 4 bytes per pixel) rather
 * than expose a guessed hardware tile swizzle. RB_CNTL defines the bin
 * dimensions; a fast clear can retain the previous sysmem MRT pitch.
 * RB_WINDOW_OFFSET locates the bin in framebuffer coordinates.
 */
static bool qcom8998_gmem_layout(Qcom8998Display *s, unsigned target,
                                 unsigned minimum, unsigned maximum,
                                 unsigned *byte_offset, unsigned *width,
                                 unsigned *fmt_out, unsigned *cpp_out)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    unsigned height = extract32(regs[ADRENO_RB_CNTL], 9, 8) << 5;
    unsigned origin = regs[0xe1d0];
    unsigned ox = origin & 0xffff, oy = origin >> 16;
    unsigned fmt = regs[mrt] & 0xff;
    unsigned swap = extract32(regs[mrt], 13, 2);
    unsigned cpp = qcom8998_rb_format_cpp(fmt);
    uint64_t offset;

    *width = extract32(regs[ADRENO_RB_CNTL], 0, 8) << 5;
    if (target >= 8 || !*width || !height || !cpp ||
        (cpp == 1 && swap) ||
        (regs[ADRENO_RB_CNTL] & ~0x1feffU) ||
        regs[ADRENO_RB_RAS_MSAA_CNTL] || regs[ADRENO_RB_DEST_MSAA_CNTL] ||
        ox + *width > 0x10000 || oy + height > 0x10000 ||
        (minimum & 0xffff) > (maximum & 0xffff) ||
        (minimum >> 16) > (maximum >> 16) ||
        (minimum & 0xffff) < ox || (minimum >> 16) < oy ||
        (maximum & 0xffff) >= ox + *width ||
        (maximum >> 16) >= oy + height ||
        (regs[mrt] & ~(ADRENO_RB_COLOR_SWAP_MASK | 0xffU)) != (2U << 8) ||
        (regs[ADRENO_RB_RENDER_CNTL] & (BIT(16 + target) | BIT(24 + target)))) {
        return false;
    }
    offset = qcom8998_reg_pair(regs, (mrt + 3) * 4);
    if ((offset & 3) || offset >= ADRENO_GMEM_SIZE ||
        (uint64_t)*width * height * cpp > ADRENO_GMEM_SIZE - offset) {
        return false;
    }
    *byte_offset = offset;
    if (fmt_out) {
        *fmt_out = fmt;
    }
    if (cpp_out) {
        *cpp_out = cpp;
    }
    return true;
}

static bool qcom8998_gmem_clear(Qcom8998Display *s, unsigned target,
                                unsigned minimum, unsigned maximum)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned offset, width, fmt, cpp;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;
    uint32_t clear_color = regs[ADRENO_RB_CLEAR_COLOR];
    uint8_t *gmem;

    if (!qcom8998_gmem_layout(s, target, minimum, maximum, &offset, &width,
                              &fmt, &cpp)) {
        trace_qcom8998_cp_draw_reject("gmem_clear", regs[ADRENO_RB_CNTL],
            target < 8 ? regs[ADRENO_RB_MRT_BUF_INFO +
                              target * ADRENO_RB_MRT_STRIDE] : target);
        return false;
    }
    gmem = (uint8_t *)s->gmem + offset;
    {
        unsigned x1 = minimum & 0xffff, x2 = maximum & 0xffff;
        unsigned y1 = minimum >> 16, y2 = maximum >> 16;
        unsigned copy_w = x2 - x1 + 1;

        if (cpp == 1) {
            uint8_t val = fmt == 2 ? (clear_color >> 24) : clear_color;

            for (unsigned y = y1; y <= y2; y++) {
                memset(gmem + (size_t)(y - oy) * width + (x1 - ox), val,
                       copy_w);
            }
        } else if (cpp == 2) {
            uint16_t val = clear_color & 0xffff;

            for (unsigned y = y1; y <= y2; y++) {
                uint16_t *dst = (uint16_t *)gmem +
                                (size_t)(y - oy) * width + (x1 - ox);

                for (unsigned i = 0; i < copy_w; i++) {
                    dst[i] = val;
                }
            }
        } else {
            for (unsigned y = y1; y <= y2; y++) {
                uint32_t *dst = (uint32_t *)gmem +
                                (size_t)(y - oy) * width + (x1 - ox);

                for (unsigned i = 0; i < copy_w; i++) {
                    dst[i] = clear_color;
                }
            }
        }
    }
    trace_qcom8998_gmem_transfer(true, target, offset, 0,
                                minimum, maximum);
    return true;
}

static bool qcom8998_gmem_resolve(Qcom8998Display *s, unsigned target,
                                  unsigned minimum, unsigned maximum)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned offset, width, fmt, cpp, mrt, swap, tile;
    unsigned x1 = minimum & 0xffff, y1 = minimum >> 16;
    unsigned x2 = maximum & 0xffff, y2 = maximum >> 16;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;
    uint64_t destination, pitch, layer, last;
    const uint8_t *gmem;

    if (!qcom8998_gmem_layout(s, target, minimum, maximum, &offset, &width,
                              &fmt, &cpp)) {
        trace_qcom8998_cp_draw_reject("gmem_resolve layout",
            regs[ADRENO_RB_CNTL], regs[ADRENO_RB_RESOLVE_CNTL]);
        return false;
    }
    tile = regs[ADRENO_RB_RESOLVE_CNTL] & 3;
    if (tile && (tile != 3 || (cpp != 1 && cpp != 4))) {
        trace_qcom8998_cp_draw_reject("gmem_resolve tile",
            regs[ADRENO_RB_RESOLVE_CNTL], fmt);
        return false;
    }
    mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    swap = extract32(regs[mrt], 13, 2);
    destination = qcom8998_reg_pair(regs, ADRENO_RB_BLIT_DST * 4);
    pitch = (uint64_t)regs[ADRENO_RB_BLIT_DST_PITCH] << 6;
    layer = (uint64_t)regs[ADRENO_RB_BLIT_DST_ARRAY_PITCH] << 6;
    last = y2 * pitch + (uint64_t)(x2 + 1) * cpp;
    gmem = (const uint8_t *)s->gmem + offset;
    /* Zero layer stride is valid for a single layer, as used by QCDX. */
    if ((destination & 3) || pitch < (uint64_t)(x2 + 1) * cpp ||
        (layer && last > layer) || destination > UINT64_MAX - last) {
        trace_qcom8998_cp_draw_reject("gmem_resolve dst", pitch, layer);
        return false;
    }
    if (tile == 3 || cpp != 4) {
        g_autoptr(GHashTable) write_pages = g_hash_table_new_full(
            g_direct_hash, g_direct_equal, NULL, g_free);
        Qcom8998PageCache write_cache;
        unsigned hbb = 13 + extract32(regs[ADRENO_RB_MODE_CNTL], 1, 2);
        unsigned bw = cpp == 1 ? 32 : 16;
        unsigned bh = cpp == 1 ? 8 : 4;
        uint64_t mt_stride = 0;
        unsigned bank_mask = 0;

        if (tile == 3) {
            unsigned align_w = cpp == 1 ? 128 : 256;
            unsigned align_h = cpp == 1 ? 32 : 16;

            if ((destination & 4095) || (pitch % align_w) ||
                (layer && (layer % (pitch * align_h)))) {
                trace_qcom8998_cp_draw_reject("gmem_resolve align",
                                              destination, pitch);
                return false;
            }
            qcom8998_tile5_3_params(pitch, cpp, hbb, &mt_stride, &bank_mask);
        }
        qcom8998_page_cache_init(&write_cache, write_pages);
        if (tile == 3) {
            for (unsigned y = y1; y <= y2; y++) {
                unsigned row = (y - oy) * width;
                unsigned yb = y / bh, ly = y % bh;
                unsigned x = x1;

                while (x <= x2) {
                    unsigned bx = x / bw;
                    unsigned x_end = MIN(x2 + 1, (bx + 1) * bw);
                    uint64_t baddr = destination +
                        qcom8998_tile5_3_block_offset_fast(bx, yb, mt_stride,
                                                           bank_mask, hbb);
                    uint8_t *ppage = qcom8998_stage_write_page(s, &write_cache,
                                                               baddr);
                    uint8_t *blk;

                    if (!ppage) {
                        return false;
                    }
                    blk = ppage + (baddr & 4095);
                    if (cpp == 1) {
                        for (unsigned xx = x; xx < x_end; xx++) {
                            unsigned pi = qcom8998_tile5_3_pixel_index(
                                xx - bx * bw, ly);

                            blk[pi] = gmem[row + xx - ox];
                        }
                    } else {
                        const uint32_t *gmem32 = (const uint32_t *)gmem;

                        for (unsigned xx = x; xx < x_end; xx++) {
                            unsigned pi = qcom8998_tile5_3_pixel_index(
                                xx - bx * bw, ly);

                            stl_le_p(blk + pi * 4,
                                     qcom8998_color_swap(gmem32[row + xx - ox],
                                                         swap));
                        }
                    }
                    x = x_end;
                }
            }
        } else {
            for (unsigned y = y1; y <= y2; y++) {
                unsigned row = (y - oy) * width;
                uint64_t paddr = destination + (uint64_t)y * pitch +
                                 (uint64_t)x1 * cpp;
                unsigned x = x1;

                while (x <= x2) {
                    unsigned page_off = paddr & 4095;
                    unsigned n = MIN(x2 - x + 1, (4096 - page_off) / cpp);
                    uint8_t *ppage = qcom8998_stage_write_page(s, &write_cache,
                                                               paddr);

                    if (!ppage) {
                        return false;
                    }
                    if (cpp == 1) {
                        memcpy(ppage + page_off, gmem + row + x - ox, n);
                    } else {
                        const uint16_t *src16 =
                            (const uint16_t *)gmem + row + x - ox;

                        for (unsigned i = 0; i < n; i++) {
                            stw_le_p(ppage + page_off + i * 2, src16[i]);
                        }
                    }
                    paddr += (uint64_t)n * cpp;
                    x += n;
                }
            }
        }
        if (!qcom8998_publish_write_pages(write_pages)) {
            return false;
        }
    } else {
        g_autoptr(GArray) spans = g_array_new(false, false,
                                              sizeof(Qcom8998GmemSpan));
        uint32_t pixels[(1U << SMMU_4K_SHIFT) / 4];

        for (unsigned y = y1; y <= y2; y++) {
            uint64_t address = destination + y * pitch + x1 * 4;
            unsigned soff = offset + ((y - oy) * width + x1 - ox) * 4;
            unsigned remaining = (x2 - x1 + 1) * 4;

            while (remaining) {
                Qcom8998GmemSpan span;
                unsigned page_left = (1U << SMMU_4K_SHIFT) -
                                     extract64(address, 0, SMMU_4K_SHIFT);

                span.length = MIN(remaining, page_left);
                span.offset = soff;
                if (!qcom8998_cp_translate(s, address, true, &span.address) ||
                    !qcom8998_ram_buffer(span.address, span.length, true)) {
                    return false;
                }
                g_array_append_val(spans, span);
                address += span.length;
                soff += span.length;
                remaining -= span.length;
            }
        }
        /* Preflight the complete destination before publishing any row. */
        for (unsigned i = 0; i < spans->len; i++) {
            const Qcom8998GmemSpan *span = &g_array_index(spans,
                                                          Qcom8998GmemSpan, i);

            for (unsigned p = 0; p < span->length / 4; p++) {
                pixels[p] = qcom8998_color_swap(s->gmem[span->offset / 4 + p],
                                                swap);
            }
            if (address_space_write(&address_space_memory, span->address,
                                    MEMTXATTRS_UNSPECIFIED, pixels,
                                    span->length) != MEMTX_OK) {
                return false;
            }
        }
    }
    trace_qcom8998_gmem_transfer(false, target, offset, destination,
                                minimum, maximum);
    return true;
}

/* Linear/TILE5_3 sysmem -> private GMEM, as used by QCDX's tile LOAD IB. */
static bool qcom8998_gmem_restore(Qcom8998Display *s, unsigned target,
                                  unsigned minimum, unsigned maximum)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned width = extract32(regs[ADRENO_RB_CNTL], 0, 8) << 5;
    unsigned height = extract32(regs[ADRENO_RB_CNTL], 9, 8) << 5;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;
    unsigned x1 = minimum & 0xffff, y1 = minimum >> 16;
    unsigned x2 = maximum & 0xffff, y2 = maximum >> 16;
    unsigned mrt, fmt, tile, swap, cpp;
    uint64_t offset, source, pitch, layer, last;
    unsigned copy_width;
    g_autofree uint8_t *pending = NULL;

    if (target >= 8) {
        return false;
    }
    mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    fmt = regs[mrt] & 0xff;
    tile = extract32(regs[mrt], 8, 2);
    swap = extract32(regs[mrt], 13, 2);
    cpp = qcom8998_rb_format_cpp(fmt);
    offset = qcom8998_reg_pair(regs, ADRENO_RB_BLIT_DST * 4);
    source = qcom8998_reg_pair(regs, (mrt + 3) * 4);
    pitch = (uint64_t)regs[mrt + 1] << 6;
    layer = (uint64_t)regs[mrt + 2] << 6;
    last = (uint64_t)y2 * pitch + ((uint64_t)x2 + 1) * cpp;

    if (!width || !height || !cpp || (cpp == 1 && swap) ||
        (tile && tile != 3) || (tile == 3 && cpp != 1 && cpp != 4) ||
        (regs[ADRENO_RB_CNTL] & ~(0x1feffU | ADRENO_RB_BYPASS)) ||
        regs[ADRENO_RB_RAS_MSAA_CNTL] || regs[ADRENO_RB_DEST_MSAA_CNTL] ||
        (regs[mrt] & ~(ADRENO_RB_COLOR_SWAP_MASK | 0x300U | 0xffU)) ||
        (regs[ADRENO_RB_RENDER_CNTL] & (BIT(16 + target) | BIT(24 + target))) ||
        ox + width > 0x10000 || oy + height > 0x10000 ||
        x1 > x2 || y1 > y2 || x1 < ox || y1 < oy ||
        x2 >= ox + width || y2 >= oy + height ||
        (offset & 3) || offset >= ADRENO_GMEM_SIZE ||
        (uint64_t)width * height * cpp > ADRENO_GMEM_SIZE - offset ||
        (uint64_t)regs[ADRENO_RB_BLIT_DST_PITCH] * 64 !=
            (uint64_t)width * cpp ||
        (regs[ADRENO_RB_BLIT_DST_ARRAY_PITCH] &&
         (uint64_t)regs[ADRENO_RB_BLIT_DST_ARRAY_PITCH] * 64 <
             (uint64_t)width * height * cpp) ||
        (source & 3) || pitch < ((uint64_t)x2 + 1) * cpp ||
        (layer && last > layer) || source > UINT64_MAX - last) {
        trace_qcom8998_cp_draw_reject("gmem_restore", regs[mrt],
                                      regs[ADRENO_RB_RENDER_CNTL]);
        return false;
    }
    copy_width = x2 - x1 + 1;
    pending = g_new(uint8_t, (size_t)copy_width * (y2 - y1 + 1) * cpp);
    if (tile == 3) {
        unsigned hbb = 13 + extract32(regs[ADRENO_RB_MODE_CNTL], 1, 2);
        unsigned align_w = cpp == 1 ? 128 : 256;
        unsigned align_h = cpp == 1 ? 32 : 16;
        unsigned bw = cpp == 1 ? 32 : 16;
        unsigned bh = cpp == 1 ? 8 : 4;
        uint64_t mt_stride = 0;
        unsigned bank_mask = 0;
        g_autoptr(GHashTable) src_pages = g_hash_table_new(g_direct_hash,
                                                           g_direct_equal);
        Qcom8998PageCache src_cache;

        if ((source & 4095) || (pitch % align_w) ||
            (layer && (layer % (pitch * align_h)))) {
            trace_qcom8998_cp_draw_reject("gmem_restore align", source, pitch);
            return false;
        }
        qcom8998_tile5_3_params(pitch, cpp, hbb, &mt_stride, &bank_mask);
        qcom8998_page_cache_init(&src_cache, src_pages);
        for (unsigned y = y1; y <= y2; y++) {
            unsigned yb = y / bh, ly = y % bh;
            uint8_t *dst_row = pending + (size_t)(y - y1) * copy_width * cpp;
            unsigned x = x1;

            while (x <= x2) {
                unsigned bx = x / bw;
                unsigned x_end = MIN(x2 + 1, (bx + 1) * bw);
                uint64_t baddr = source +
                    qcom8998_tile5_3_block_offset_fast(bx, yb, mt_stride,
                                                       bank_mask, hbb);
                const uint8_t *ppage = qcom8998_texture_page(s, &src_cache,
                                                             baddr);
                const uint8_t *blk;

                if (!ppage) {
                    return false;
                }
                blk = ppage + (baddr & 4095);
                if (cpp == 1) {
                    for (unsigned xx = x; xx < x_end; xx++) {
                        unsigned pi = qcom8998_tile5_3_pixel_index(
                            xx - bx * bw, ly);

                        dst_row[xx - x1] = blk[pi];
                    }
                } else {
                    uint32_t *dst32 = (uint32_t *)dst_row;

                    for (unsigned xx = x; xx < x_end; xx++) {
                        unsigned pi = qcom8998_tile5_3_pixel_index(
                            xx - bx * bw, ly);

                        dst32[xx - x1] = qcom8998_color_unswap(
                            ldl_le_p(blk + pi * 4), swap);
                    }
                }
                x = x_end;
            }
        }
    } else {
        for (unsigned y = y1; y <= y2; y++) {
            uint64_t address = source + (uint64_t)y * pitch +
                               (uint64_t)x1 * cpp;
            uint8_t *dst_row = pending + (size_t)(y - y1) * copy_width * cpp;
            unsigned x = 0;

            while (x < copy_width) {
                hwaddr physical;
                unsigned n = MIN(copy_width - x,
                                 (4096 - (address & 4095)) / cpp);
                const uint8_t *data;

                if (!qcom8998_cp_translate(s, address, false, &physical) ||
                    !(data = qcom8998_ram_ptr(physical, n * cpp, false))) {
                    return false;
                }
                if (cpp == 4) {
                    for (unsigned i = 0; i < n; i++) {
                        uint32_t color = qcom8998_color_unswap(
                            ldl_le_p(data + i * 4), swap);

                        ((uint32_t *)dst_row)[x + i] = color;
                    }
                } else {
                    memcpy(dst_row + x * cpp, data, n * cpp);
                }
                address += n * cpp;
                x += n;
            }
        }
    }
    /* A source fault in a later row must not partially overwrite GMEM. */
    for (unsigned y = y1; y <= y2; y++) {
        memcpy((uint8_t *)s->gmem + offset +
                   ((size_t)(y - oy) * width + x1 - ox) * cpp,
               pending + (size_t)(y - y1) * copy_width * cpp,
               (size_t)copy_width * cpp);
    }
    trace_qcom8998_gmem_restore(offset, source, minimum, maximum);
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
    unsigned mrt, width, height, swap, fmt, tile, cpp;
    bool ubwc;
    uint64_t base, pitch, layer_size, pixel_layer, last;
    uint32_t packed_color;
    uint32_t pixels[(1U << SMMU_4K_SHIFT) / sizeof(uint32_t)];
    g_autoptr(GArray) spans = g_array_new(false, false,
                                        sizeof(Qcom8998ClearSpan));

    if (target >= 8 || x2 < x1 || y2 < y1) {
        return false;
    }
    mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    fmt = regs[mrt] & 0xff;
    tile = extract32(regs[mrt], 8, 2);
    swap = extract32(regs[mrt], 13, 2);
    ubwc = !!(regs[ADRENO_RB_RENDER_CNTL] & BIT(16 + target));
    if ((fmt != ADRENO_RB_RGBA8_UNORM && fmt != 0x02 && fmt != 0x03 &&
         fmt != 0x0f) ||
        (regs[mrt] & ~(ADRENO_RB_COLOR_SWAP_MASK | 0x300U | 0xffU)) ||
        (tile && tile != 3) ||
        (regs[ADRENO_RB_RENDER_CNTL] & BIT(24 + target))) {
        return false;
    }
    cpp = fmt == ADRENO_RB_RGBA8_UNORM ? 4 : (fmt == 0x0f ? 2 : 1);
    if ((cpp == 1 && swap) || (tile == 3 && cpp != 1 && cpp != 4) ||
        (ubwc && (tile != 3 || cpp != 4))) {
        return false;
    }
    base = qcom8998_reg_pair(regs, (mrt + 3) * 4);
    pitch = (uint64_t)regs[mrt + 1] << 6;
    layer_size = (uint64_t)regs[mrt + 2] << 6;
    pixel_layer = layer_size;
    width = x2 - x1 + 1;
    height = y2 - y1 + 1;
    if (tile == 3 || cpp != 4) {
        g_autoptr(GHashTable) write_pages = g_hash_table_new_full(
            g_direct_hash, g_direct_equal, NULL, g_free);
        Qcom8998PageCache write_cache;
        unsigned hbb = 13 + extract32(regs[ADRENO_RB_MODE_CNTL], 1, 2);
        unsigned bw = cpp == 1 ? 32 : 16;
        unsigned bh = cpp == 1 ? 8 : 4;
        uint64_t flag_addr = 0, flag_pitch = 0, flag_size = 0;

        qcom8998_page_cache_init(&write_cache, write_pages);

        if (ubwc) {
            unsigned fb = 0xe243 + target * 4;
            uint64_t flag_array_pitch = (uint64_t)regs[fb + 3] << 6;

            flag_addr = qcom8998_reg_pair(regs, fb * 4);
            flag_pitch = (uint64_t)regs[fb + 2] << 6;
            if (flag_addr < base && base - flag_addr < layer_size) {
                flag_size = base - flag_addr;
                pixel_layer = layer_size - flag_size;
            } else {
                flag_size = flag_array_pitch;
            }
            if ((flag_addr & 4095) || !flag_pitch || (flag_pitch & 63) ||
                flag_pitch < DIV_ROUND_UP(pitch / cpp, 16) ||
                !flag_size || flag_size > ADRENO_CLEAR_MAX_BYTES ||
                flag_addr > UINT64_MAX - flag_size ||
                (uint64_t)(y2 / 4) * flag_pitch + (x2 / 16) >= flag_size) {
                return false;
            }
        }
        if ((base & 3) || !pitch || !pixel_layer || pixel_layer % pitch ||
            (uint64_t)(x2 + 1) * cpp > pitch ||
            (uint64_t)(y2 + 1) > pixel_layer / pitch ||
            pixel_layer > ADRENO_CLEAR_MAX_BYTES ||
            base > UINT64_MAX - pixel_layer) {
            return false;
        }
        if (tile == 3) {
            unsigned align_w = cpp == 1 ? 128 : 256;
            unsigned align_h = cpp == 1 ? 32 : 16;

            if ((base & 4095) || (pitch % align_w) ||
                (pixel_layer % (pitch * align_h))) {
                return false;
            }
        }
        packed_color = qcom8998_color_swap(color, swap);
        if (ubwc) {
            for (unsigned by = y1 / 4; by <= y2 / 4; by++) {
                for (unsigned bx = x1 / 16; bx <= x2 / 16; bx++) {
                    uint64_t faddr = flag_addr + (uint64_t)by * flag_pitch + bx;
                    uint8_t *fpage = qcom8998_stage_write_page(s, &write_cache,
                                                               faddr);
                    bool full = bx * 16 >= x1 && (bx + 1) * 16 - 1 <= x2 &&
                                by * 4 >= y1 && (by + 1) * 4 - 1 <= y2;
                    uint8_t old_flag;

                    if (!fpage) {
                        return false;
                    }
                    old_flag = fpage[faddr & 4095];
                    if (full && !packed_color) {
                        fpage[faddr & 4095] = 0x00;
                        continue;
                    }
                    if (!full && old_flag == 0x00) {
                        uint64_t baddr = base +
                            qcom8998_tile5_3_block_offset(bx, by, pitch, 4,
                                                          hbb);
                        uint8_t *bpage = qcom8998_stage_write_page(
                            s, &write_cache, baddr);

                        if (!bpage) {
                            return false;
                        }
                        memset(bpage + (baddr & 4095), 0, 256);
                    }
                    fpage[faddr & 4095] = 0xff;
                }
            }
        }
        for (unsigned y = y1; y <= y2; y++) {
            for (unsigned x = x1; x <= x2; x++) {
                uint64_t paddr;
                uint8_t *ppage;

                if (ubwc && !packed_color &&
                    (x / 16) * 16 >= x1 && (x / 16 + 1) * 16 - 1 <= x2 &&
                    (y / 4) * 4 >= y1 && (y / 4 + 1) * 4 - 1 <= y2) {
                    continue;
                }
                if (tile == 3) {
                    paddr = base +
                        qcom8998_tile5_3_block_offset(x / bw, y / bh, pitch,
                                                      cpp, hbb) +
                        (uint64_t)qcom8998_tile5_3_pixel_index(x % bw,
                                                               y % bh) * cpp;
                } else {
                    paddr = base + (uint64_t)y * pitch + (uint64_t)x * cpp;
                }
                ppage = qcom8998_stage_write_page(s, &write_cache, paddr);
                if (!ppage) {
                    return false;
                }
                if (cpp == 1) {
                    ppage[paddr & 4095] = fmt == 2 ? (color >> 24) : color;
                } else if (cpp == 2) {
                    stw_le_p(ppage + (paddr & 4095), color & 0xffff);
                } else {
                    stl_le_p(ppage + (paddr & 4095), packed_color);
                }
            }
        }
        return qcom8998_publish_write_pages(write_pages);
    }
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

static bool qcom8998_bypass_blit(Qcom8998Display *s, unsigned target,
                                 unsigned minimum, unsigned maximum)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned x1 = extract32(minimum, 0, 16);
    unsigned y1 = extract32(minimum, 16, 16);
    unsigned x2 = extract32(maximum, 0, 16);
    unsigned y2 = extract32(maximum, 16, 16);
    unsigned resolve_cntl = regs[ADRENO_RB_RESOLVE_CNTL];
    unsigned dst_tile = resolve_cntl & 3;
    unsigned mrt, fmt, cpp, src_tile;
    bool src_ubwc;
    uint64_t src_base, src_pitch, src_layer, src_pixel_layer, src_last;
    uint64_t dst_base, dst_pitch, dst_layer, dst_last;
    uint64_t flag_addr = 0, flag_pitch = 0, flag_size = 0;
    unsigned copy_w, copy_h;
    unsigned hbb = 13 + extract32(regs[ADRENO_RB_MODE_CNTL], 1, 2);
    g_autofree uint8_t *pending = NULL;
    g_autoptr(GHashTable) write_pages = g_hash_table_new_full(
        g_direct_hash, g_direct_equal, NULL, g_free);
    Qcom8998PageCache write_cache;

    if (target >= 8 || x2 < x1 || y2 < y1 ||
        regs[ADRENO_RB_RAS_MSAA_CNTL] || regs[ADRENO_RB_DEST_MSAA_CNTL] ||
        (resolve_cntl != 4 && resolve_cntl != 7)) {
        trace_qcom8998_cp_draw_reject("bypass_blit header", target,
                                      resolve_cntl);
        return false;
    }
    mrt = ADRENO_RB_MRT_BUF_INFO + target * ADRENO_RB_MRT_STRIDE;
    fmt = regs[mrt] & 0xff;
    src_tile = extract32(regs[mrt], 8, 2);
    src_ubwc = !!(regs[ADRENO_RB_RENDER_CNTL] & BIT(16 + target));
    cpp = qcom8998_rb_format_cpp(fmt);
    if (!cpp ||
        (regs[mrt] & ~(ADRENO_RB_COLOR_SWAP_MASK | 0x300U | 0xffU)) ||
        (src_tile && src_tile != 3) ||
        (src_tile == 3 && cpp != 1 && cpp != 4) ||
        (dst_tile == 3 && cpp != 1 && cpp != 4) ||
        (src_ubwc && (src_tile != 3 || cpp != 4)) ||
        (regs[ADRENO_RB_RENDER_CNTL] & BIT(24 + target))) {
        trace_qcom8998_cp_draw_reject("bypass_blit fmt", regs[mrt],
                                      regs[ADRENO_RB_RENDER_CNTL]);
        return false;
    }
    src_base = qcom8998_reg_pair(regs, (mrt + 3) * 4);
    src_pitch = (uint64_t)regs[mrt + 1] << 6;
    src_layer = (uint64_t)regs[mrt + 2] << 6;
    src_pixel_layer = src_layer;
    dst_base = qcom8998_reg_pair(regs, ADRENO_RB_BLIT_DST * 4);
    dst_pitch = (uint64_t)regs[ADRENO_RB_BLIT_DST_PITCH] << 6;
    dst_layer = (uint64_t)regs[ADRENO_RB_BLIT_DST_ARRAY_PITCH] << 6;
    copy_w = x2 - x1 + 1;
    copy_h = y2 - y1 + 1;
    if ((uint64_t)copy_w * copy_h * cpp > ADRENO_CLEAR_MAX_BYTES) {
        trace_qcom8998_cp_draw_reject("bypass_blit size", copy_w, copy_h);
        return false;
    }
    if (src_ubwc) {
        unsigned fb = 0xe243 + target * 4;
        uint64_t flag_array_pitch = (uint64_t)regs[fb + 3] << 6;

        flag_addr = qcom8998_reg_pair(regs, fb * 4);
        flag_pitch = (uint64_t)regs[fb + 2] << 6;
        if (flag_addr < src_base && src_base - flag_addr < src_layer) {
            flag_size = src_base - flag_addr;
            src_pixel_layer = src_layer - flag_size;
        } else {
            flag_size = flag_array_pitch;
        }
        if ((flag_addr & 4095) || !flag_pitch || (flag_pitch & 63) ||
            flag_pitch < DIV_ROUND_UP(src_pitch / cpp, 16) ||
            !flag_size || flag_size > ADRENO_CLEAR_MAX_BYTES ||
            flag_addr > UINT64_MAX - flag_size ||
            (uint64_t)(y2 / 4) * flag_pitch + (x2 / 16) >= flag_size) {
            trace_qcom8998_cp_draw_reject("bypass_blit ubwc", flag_pitch,
                                          flag_size);
            return false;
        }
    }
    src_last = (uint64_t)y2 * src_pitch + ((uint64_t)x2 + 1) * cpp;
    dst_last = (uint64_t)y2 * dst_pitch + ((uint64_t)x2 + 1) * cpp;
    if ((src_base & 3) || !src_pitch || src_pitch < ((uint64_t)x2 + 1) * cpp ||
        (src_pixel_layer && src_last > src_pixel_layer) ||
        src_base > UINT64_MAX - src_last ||
        (dst_base & 3) || !dst_pitch || dst_pitch < ((uint64_t)x2 + 1) * cpp ||
        (dst_layer && dst_last > dst_layer) ||
        dst_base > UINT64_MAX - dst_last) {
        trace_qcom8998_cp_draw_reject("bypass_blit bounds", src_pitch,
                                      dst_pitch);
        return false;
    }
    if (src_tile == 3) {
        unsigned align_w = cpp == 1 ? 128 : 256;
        unsigned align_h = cpp == 1 ? 32 : 16;

        if ((src_base & 4095) || (src_pitch % align_w) ||
            (src_pixel_layer && (src_pixel_layer % (src_pitch * align_h)))) {
            trace_qcom8998_cp_draw_reject("bypass_blit src_align", src_base,
                                          src_pitch);
            return false;
        }
    }
    if (dst_tile == 3) {
        unsigned align_w = cpp == 1 ? 128 : 256;
        unsigned align_h = cpp == 1 ? 32 : 16;

        if ((dst_base & 4095) || (dst_pitch % align_w) ||
            (dst_layer && (dst_layer % (dst_pitch * align_h)))) {
            trace_qcom8998_cp_draw_reject("bypass_blit dst_align", dst_base,
                                          dst_pitch);
            return false;
        }
    }

    pending = g_new(uint8_t, (size_t)copy_w * copy_h * cpp);
    if (src_tile == 3) {
        unsigned bw = cpp == 1 ? 32 : 16;
        unsigned bh = cpp == 1 ? 8 : 4;
        uint64_t mt_stride = 0;
        unsigned bank_mask = 0;
        g_autoptr(GHashTable) src_pages = g_hash_table_new(g_direct_hash,
                                                           g_direct_equal);
        Qcom8998PageCache src_cache;

        qcom8998_tile5_3_params(src_pitch, cpp, hbb, &mt_stride, &bank_mask);
        qcom8998_page_cache_init(&src_cache, src_pages);
        for (unsigned y = y1; y <= y2; y++) {
            unsigned yb = y / bh, ly = y % bh;
            uint8_t *dst_row = pending + (size_t)(y - y1) * copy_w * cpp;
            unsigned x = x1;

            while (x <= x2) {
                unsigned bx = x / bw;
                unsigned x_end = MIN(x2 + 1, (bx + 1) * bw);

                if (src_ubwc) {
                    uint64_t faddr = flag_addr + (uint64_t)yb * flag_pitch + bx;
                    const uint8_t *fpage = qcom8998_texture_page(s, &src_cache,
                                                                 faddr);
                    uint8_t flag;

                    if (!fpage) {
                        return false;
                    }
                    flag = fpage[faddr & 4095];
                    if (flag == 0x00) {
                        memset(dst_row + (size_t)(x - x1) * cpp, 0,
                               (size_t)(x_end - x) * cpp);
                        x = x_end;
                        continue;
                    }
                    if (flag != 0xff) {
                        trace_qcom8998_cp_draw_reject("bypass_blit ubwc flag",
                                                      flag, (yb << 16) | bx);
                        return false;
                    }
                }
                {
                    uint64_t baddr = src_base +
                        qcom8998_tile5_3_block_offset_fast(bx, yb, mt_stride,
                                                           bank_mask, hbb);
                    const uint8_t *ppage = qcom8998_texture_page(s, &src_cache,
                                                                 baddr);
                    const uint8_t *blk;

                    if (!ppage) {
                        return false;
                    }
                    blk = ppage + (baddr & 4095);
                    if (cpp == 1) {
                        for (unsigned xx = x; xx < x_end; xx++) {
                            dst_row[xx - x1] =
                                blk[qcom8998_tile5_3_pixel_index(xx - bx * bw,
                                                                 ly)];
                        }
                    } else {
                        uint32_t *dst32 = (uint32_t *)dst_row;

                        for (unsigned xx = x; xx < x_end; xx++) {
                            unsigned pi = qcom8998_tile5_3_pixel_index(
                                xx - bx * bw, ly);

                            dst32[xx - x1] = ldl_le_p(blk + pi * 4);
                        }
                    }
                }
                x = x_end;
            }
        }
    } else {
        for (unsigned y = y1; y <= y2; y++) {
            uint64_t addr = src_base + (uint64_t)y * src_pitch +
                            (uint64_t)x1 * cpp;
            uint8_t *dst_row = pending + (size_t)(y - y1) * copy_w * cpp;
            size_t remaining = (size_t)copy_w * cpp;
            size_t off = 0;

            while (remaining) {
                hwaddr phys;
                size_t n = MIN(remaining, 4096 - (addr & 4095));
                const uint8_t *data;

                if (!qcom8998_cp_translate(s, addr, false, &phys) ||
                    !(data = qcom8998_ram_ptr(phys, n, false))) {
                    return false;
                }
                memcpy(dst_row + off, data, n);
                addr += n;
                off += n;
                remaining -= n;
            }
        }
    }

    qcom8998_page_cache_init(&write_cache, write_pages);
    if (dst_tile == 3) {
        unsigned bw = cpp == 1 ? 32 : 16;
        unsigned bh = cpp == 1 ? 8 : 4;
        uint64_t mt_stride = 0;
        unsigned bank_mask = 0;

        qcom8998_tile5_3_params(dst_pitch, cpp, hbb, &mt_stride, &bank_mask);
        for (unsigned y = y1; y <= y2; y++) {
            unsigned yb = y / bh, ly = y % bh;
            const uint8_t *src_row = pending + (size_t)(y - y1) * copy_w * cpp;
            unsigned x = x1;

            while (x <= x2) {
                unsigned bx = x / bw;
                unsigned x_end = MIN(x2 + 1, (bx + 1) * bw);
                uint64_t baddr = dst_base +
                    qcom8998_tile5_3_block_offset_fast(bx, yb, mt_stride,
                                                       bank_mask, hbb);
                uint8_t *ppage = qcom8998_stage_write_page(s, &write_cache,
                                                           baddr);
                uint8_t *blk;

                if (!ppage) {
                    return false;
                }
                blk = ppage + (baddr & 4095);
                if (cpp == 1) {
                    for (unsigned xx = x; xx < x_end; xx++) {
                        blk[qcom8998_tile5_3_pixel_index(xx - bx * bw, ly)] =
                            src_row[xx - x1];
                    }
                } else {
                    const uint32_t *src32 = (const uint32_t *)src_row;

                    for (unsigned xx = x; xx < x_end; xx++) {
                        unsigned pi = qcom8998_tile5_3_pixel_index(
                            xx - bx * bw, ly);

                        stl_le_p(blk + pi * 4, src32[xx - x1]);
                    }
                }
                x = x_end;
            }
        }
    } else {
        for (unsigned y = y1; y <= y2; y++) {
            uint64_t addr = dst_base + (uint64_t)y * dst_pitch +
                            (uint64_t)x1 * cpp;
            const uint8_t *src_row = pending + (size_t)(y - y1) * copy_w * cpp;
            size_t remaining = (size_t)copy_w * cpp;
            size_t off = 0;

            while (remaining) {
                unsigned page_off = addr & 4095;
                size_t n = MIN(remaining, 4096 - page_off);
                uint8_t *ppage = qcom8998_stage_write_page(s, &write_cache,
                                                           addr);

                if (!ppage) {
                    return false;
                }
                memcpy(ppage + page_off, src_row + off, n);
                addr += n;
                off += n;
                remaining -= n;
            }
        }
    }
    if (!qcom8998_publish_write_pages(write_pages)) {
        return false;
    }
    trace_qcom8998_gmem_transfer(false, target, 0, dst_base, minimum, maximum);
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
             regs[ADRENO_RB_RESOLVE_CNTL] == 4 ||
             regs[ADRENO_RB_RESOLVE_CNTL] == 7) &&
            regs[ADRENO_RB_CLEAR_CNTL] ==
                (ADRENO_RB_FAST_CLEAR | ADRENO_RB_CLEAR_ALL_COMPONENTS)) {
            /* FAST_CLEAR also overrides the previous tile's store direction. */
            return qcom8998_gmem_clear(s, target, minimum, maximum);
        }
        if ((regs[ADRENO_RB_RESOLVE_CNTL] == 4 ||
             regs[ADRENO_RB_RESOLVE_CNTL] == 7) &&
            !regs[ADRENO_RB_CLEAR_CNTL]) {
            if (!qcom8998_gmem_resolve(s, target, minimum, maximum)) {
                return false;
            }
            s->gpu_dirty = true;
            return true;
        }
        if (!regs[ADRENO_RB_RESOLVE_CNTL] &&
            !regs[ADRENO_RB_CLEAR_CNTL]) {
            return qcom8998_gmem_restore(s, target, minimum, maximum);
        }
        trace_qcom8998_cp_draw_reject("ccu_clear gmem",
            regs[ADRENO_RB_RESOLVE_CNTL], regs[ADRENO_RB_CLEAR_CNTL]);
        return false;
    }
    if ((regs[ADRENO_RB_RESOLVE_CNTL] == 4 ||
         regs[ADRENO_RB_RESOLVE_CNTL] == 7) &&
        !regs[ADRENO_RB_CLEAR_CNTL]) {
        if (!qcom8998_bypass_blit(s, target, minimum, maximum)) {
            return false;
        }
        s->gpu_dirty = true;
        return true;
    }
    if (regs[ADRENO_RB_RAS_MSAA_CNTL] ||
        regs[ADRENO_RB_DEST_MSAA_CNTL] || regs[ADRENO_RB_RESOLVE_CNTL] ||
        regs[ADRENO_RB_CLEAR_CNTL] !=
            (ADRENO_RB_FAST_CLEAR | ADRENO_RB_CLEAR_ALL_COMPONENTS)) {
        trace_qcom8998_cp_draw_reject("ccu_clear bypass",
            regs[ADRENO_RB_RESOLVE_CNTL], regs[ADRENO_RB_CLEAR_CNTL]);
        return false;
    }
    if (!qcom8998_fill_rect(s, target, minimum, maximum,
                            regs[ADRENO_RB_CLEAR_COLOR])) {
        trace_qcom8998_cp_draw_reject("ccu_clear fill_rect",
            target, target < 8 ? regs[ADRENO_RB_MRT_BUF_INFO +
                                      target * ADRENO_RB_MRT_STRIDE] : 0);
        return false;
    }
    s->gpu_dirty = true;
    trace_qcom8998_ccu_clear(target,
        qcom8998_reg_pair(regs, (ADRENO_RB_MRT_BUF_INFO +
                         target * ADRENO_RB_MRT_STRIDE + 3) * 4),
        minimum & 0xffff, minimum >> 16,
        (maximum & 0xffff) - (minimum & 0xffff) + 1,
        (maximum >> 16) - (minimum >> 16) + 1,
        regs[ADRENO_RB_CLEAR_COLOR]);
    return true;
}

/* General A540 IR3 interpreter (Cat 0..6), bounded per invocation. */
#define ADRENO_SHADER_GPRS 244
#define ADRENO_SHADER_STEPS 2048
#define ADRENO_SHADER_MAX_INSNS 512
#define ADRENO_SHADER_MAX_CONSTS 1024

typedef struct Qcom8998ShaderProgram {
    uint64_t insns[ADRENO_SHADER_MAX_INSNS];
    uint64_t insn_valid[ADRENO_SHADER_MAX_INSNS / 64];
    unsigned insn_count;
    uint32_t consts[ADRENO_SHADER_MAX_CONSTS];
    uint64_t const_valid[ADRENO_SHADER_MAX_CONSTS / 64];
    unsigned const_count;
} Qcom8998ShaderProgram;

typedef struct Qcom8998TexInfo {
    bool valid;
    unsigned tile;
    unsigned fmt;
    unsigned swap;
    bool srgb;
    unsigned width;
    unsigned height;
    unsigned tex_type;
    uint64_t pitch;
    bool ubwc;
    uint64_t base;
    uint64_t fbase;
    uint64_t fpitch;
    unsigned cpp;
    unsigned bw;
    unsigned bh;
    unsigned hbb;
    uint64_t macrotile_stride;
    unsigned bank_mask;
    uint8_t swizzle[4];
    bool identity_swizzle;
} Qcom8998TexInfo;

typedef struct Qcom8998ShaderRegs {
    uint32_t full[ADRENO_SHADER_GPRS];
    uint16_t half[ADRENO_SHADER_GPRS];
    uint64_t full_valid[4];
    uint64_t half_valid[4];
    uint8_t pred[4];
    uint8_t pred_valid;
    int16_t address;
    bool address_valid;
    int16_t a1;
    bool a1_valid;
    uint32_t input[64];
    uint64_t input_valid;
    uint32_t flat_input[64];
    uint64_t flat_valid;
    bool killed;
    Qcom8998PageCache *tex_pages;
    Qcom8998ShaderProgram *program;
    const Qcom8998TexInfo *tex_info;
    unsigned tex_info_count;
    unsigned steps;
    uint32_t instruction[2];
} Qcom8998ShaderRegs;

static inline bool qcom8998_bit_test(const uint64_t *bits, unsigned idx)
{
    return !!(bits[idx >> 6] & (1ULL << (idx & 63)));
}

static inline void qcom8998_bit_set(uint64_t *bits, unsigned idx)
{
    bits[idx >> 6] |= 1ULL << (idx & 63);
}

static inline void qcom8998_shader_reset_pixel(Qcom8998ShaderRegs *r)
{
    r->full_valid[0] = 0;
    r->full_valid[1] = 0;
    r->full_valid[2] = 0;
    r->full_valid[3] = 0;
    r->half_valid[0] = 0;
    r->half_valid[1] = 0;
    r->half_valid[2] = 0;
    r->half_valid[3] = 0;
    r->pred_valid = 0;
    r->address_valid = false;
    r->a1_valid = false;
    r->input_valid = 0;
    r->flat_valid = 0;
    r->killed = false;
    r->steps = 0;
}

static inline uint32_t qcom8998_f16_to_f32(uint16_t h)
{
    uint32_t sign = (uint32_t)(h & 0x8000U) << 16;
    unsigned exp = (h >> 10) & 0x1fU;
    unsigned frac = h & 0x3ffU;

    if (exp == 0) {
        if (frac == 0) {
            return sign;
        }
        unsigned shift = clz32(frac) - 21;
        return sign | ((uint32_t)(113 - shift) << 23) |
               (((uint32_t)frac << shift) & 0x3ffU) << 13;
    }
    if (exp == 31) {
        if (frac == 0) {
            return sign | 0x7f800000U;
        }
        return sign | 0x7fc00000U | ((uint32_t)frac << 13);
    }
    return sign | ((uint32_t)(exp + 112) << 23) | ((uint32_t)frac << 13);
}

static inline uint16_t qcom8998_f32_to_f16(uint32_t f)
{
    uint16_t sign = (uint16_t)((f >> 16) & 0x8000U);
    int exp = (int)((f >> 23) & 0xffU) - 127;
    uint32_t frac = f & 0x7fffffU;

    if (exp == 128) {
        if (frac == 0) {
            return sign | 0x7c00U;
        }
        return sign | 0x7e00U | (uint16_t)(frac >> 13);
    }
    if (exp > 15) {
        return sign | 0x7c00U;
    }
    if (exp < -25) {
        return sign;
    }
    if (exp < -14) {
        uint32_t sig = frac | 0x800000U;
        unsigned shift = (unsigned)(-1 - exp); /* 14..24 */
        uint32_t half = 1U << (shift - 1);
        uint32_t rem = sig & ((1U << shift) - 1);
        uint32_t m = sig >> shift;

        if (rem > half || (rem == half && (m & 1))) {
            m++;
        }
        return sign | (uint16_t)m;
    }
    {
        uint32_t m = frac >> 13;
        uint32_t rem = frac & 0x1fffU;

        if (rem > 0x1000U || (rem == 0x1000U && (m & 1))) {
            m++;
            if (m == 0x400U) {
                m = 0;
                exp++;
                if (exp > 15) {
                    return sign | 0x7c00U;
                }
            }
        }
        return sign | (uint16_t)((exp + 15) << 10) | (uint16_t)m;
    }
}

static const float *qcom8998_srgb_lut(void)
{
    static float lut[256];
    static bool initialized;

    if (!initialized) {
        for (unsigned i = 0; i < 256; i++) {
            float v = (float)i / 255.0f;
            lut[i] = v <= 0.04045f ? v / 12.92f :
                     powf((v + 0.055f) / 1.055f, 2.4f);
        }
        initialized = true;
    }
    return lut;
}

static bool qcom8998_texture_word(Qcom8998Display *s, Qcom8998PageCache *pages,
                                  uint64_t address, unsigned cpp,
                                  uint32_t *value);

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
    if (type == 1 && unit->ubo) {
        const GArray *table = s->load_state[6][2];
        unsigned slot = unit->ubo_offset >> 16;
        unsigned offset = (unit->ubo_offset & 0xffff) + (index % 4) * 4;
        unsigned entry = (block - 8) * ADRENO_UBO_STAGE_STRIDE + slot;
        const Qcom8998StateUnit *descriptor;
        uint64_t buffer;

        if (!table || entry >= table->len) {
            return false;
        }
        descriptor = &g_array_index(table, Qcom8998StateUnit, entry);
        buffer = deposit64(descriptor->words[0], 32, 32, descriptor->words[1]);
        if (!descriptor->valid || !buffer || (buffer & 15) ||
            buffer > UINT64_MAX - offset - 4) {
            return false;
        }
        return qcom8998_cp_word(s, buffer + offset, value, false);
    }
    *value = unit->words[index % words];
    return true;
}

static bool qcom8998_shader_finite(uint32_t value)
{
    unsigned exponent = extract32(value, 23, 8);

    return exponent != 255;
}

static float qcom8998_u32_to_f(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static uint32_t qcom8998_f_to_u32(float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    return bits;
}

static uint32_t qcom8998_sat_f32(uint32_t bits)
{
    float f = qcom8998_u32_to_f(bits);

    if (isnan(f) || f <= 0.0f) {
        return 0;
    }
    if (f >= 1.0f) {
        return 0x3f800000;
    }
    return bits;
}

static void qcom8998_shader_prepare(Qcom8998Display *s, unsigned block,
                                    Qcom8998ShaderProgram *prog)
{
    const GArray *code = s->load_state[block][0];
    const GArray *consts = s->load_state[block][1];

    memset(prog->insn_valid, 0, sizeof(prog->insn_valid));
    memset(prog->const_valid, 0, sizeof(prog->const_valid));
    prog->insn_count = 0;
    prog->const_count = 0;

    if (code) {
        unsigned max_u = MIN(code->len, ADRENO_SHADER_MAX_INSNS / 16U);

        prog->insn_count = max_u * 16U;
        for (unsigned u = 0; u < max_u; u++) {
            const Qcom8998StateUnit *unit =
                &g_array_index(code, Qcom8998StateUnit, u);

            if (!unit->valid) {
                continue;
            }
            for (unsigned k = 0; k < 16; k++) {
                unsigned pc = u * 16 + k;

                prog->insns[pc] = deposit64(unit->words[k * 2], 32, 32,
                                            unit->words[k * 2 + 1]);
                qcom8998_bit_set(prog->insn_valid, pc);
            }
        }
    }
    if (consts) {
        unsigned max_u = MIN(consts->len, ADRENO_SHADER_MAX_CONSTS / 4U);

        prog->const_count = max_u * 4U;
        for (unsigned u = 0; u < max_u; u++) {
            const Qcom8998StateUnit *unit =
                &g_array_index(consts, Qcom8998StateUnit, u);

            if (!unit->valid || unit->ubo) {
                continue;
            }
            for (unsigned k = 0; k < 4; k++) {
                unsigned idx = u * 4 + k;

                prog->consts[idx] = unit->words[k];
                qcom8998_bit_set(prog->const_valid, idx);
            }
        }
    }
}

static inline bool qcom8998_shader_const(Qcom8998Display *s, unsigned block,
                                         const Qcom8998ShaderRegs *r,
                                         unsigned index, uint32_t *value)
{
    if (r->program && index < r->program->const_count) {
        if (qcom8998_bit_test(r->program->const_valid, index)) {
            *value = r->program->consts[index];
            return true;
        }
        if (!qcom8998_shader_word(s, block, 1, index, value)) {
            return false;
        }
        r->program->consts[index] = *value;
        qcom8998_bit_set(r->program->const_valid, index);
        return true;
    }
    return qcom8998_shader_word(s, block, 1, index, value);
}

static inline bool qcom8998_ir3_read_reg(const Qcom8998ShaderRegs *r,
                                         unsigned idx, bool full, uint32_t *out)
{
    if (idx >= 248 && idx <= 251) {
        unsigned p = idx - 248;

        *out = (r->pred_valid & BIT(p)) ? r->pred[p] : 0;
        return true;
    }
    if (idx >= ADRENO_SHADER_GPRS) {
        return false;
    }
    if (full) {
        if (!qcom8998_bit_test(r->full_valid, idx)) {
            return false;
        }
        *out = r->full[idx];
    } else {
        if (!qcom8998_bit_test(r->half_valid, idx)) {
            return false;
        }
        *out = r->half[idx];
    }
    return true;
}

static inline bool qcom8998_ir3_write_reg(Qcom8998ShaderRegs *r, unsigned idx,
                                          bool dest_half, bool is_float,
                                          uint32_t val)
{
    if (idx >= 248 && idx <= 251) {
        r->pred[idx - 248] = val & 1;
        r->pred_valid |= BIT(idx - 248);
        return true;
    }
    if (idx == 244) {
        r->address = (int16_t)val;
        r->address_valid = true;
        return true;
    }
    if (idx == 245) {
        r->a1 = (int16_t)val;
        r->a1_valid = true;
        return true;
    }
    if (idx >= 252 && idx <= 255) {
        /* r63.xyzw is the IR3 discard/sink register (e.g. bary.f (ei)r63.x). */
        return true;
    }
    if (idx >= ADRENO_SHADER_GPRS) {
        return false;
    }
    if (dest_half) {
        r->half[idx] = is_float ? qcom8998_f32_to_f16(val) : (uint16_t)val;
        qcom8998_bit_set(r->half_valid, idx);
    } else {
        r->full[idx] = val;
        qcom8998_bit_set(r->full_valid, idx);
    }
    return true;
}

static inline bool qcom8998_ir3_multisrc(Qcom8998Display *s, unsigned block,
                                         const Qcom8998ShaderRegs *r,
                                         unsigned enc, unsigned inc, bool full,
                                         unsigned kind, /* 0=float, 1=sint, 2=uint */
                                         uint32_t *out)
{
    static const float flut[12] = {
        0.0f, 0.5f, 1.0f, 2.0f,
        2.71828182845904523536f, 3.14159265358979323846f,
        0.31830988618379067154f, 0.69314718055994530942f,
        1.44269504088896340736f, 0.30102999566398119521f,
        3.32192809488736234787f, 4.0f,
    };
    unsigned sel = extract32(enc, 11, 3);
    unsigned absneg = extract32(enc, 14, 2);
    uint32_t val = 0;

    if (sel == 0) {
        unsigned idx = (enc & 0xff) + inc;

        if (!qcom8998_ir3_read_reg(r, idx, full, &val)) {
            return false;
        }
        if (!full) {
            if (kind == 0) {
                val = qcom8998_f16_to_f32((uint16_t)val);
            } else if (kind == 1) {
                val = (uint32_t)(int32_t)(int16_t)val;
            } else {
                val &= 0xffff;
            }
        }
    } else if (sel == 1) {
        int off = sextract32(enc, 0, 10) + (int)inc;
        int idx;

        if (!r->address_valid) {
            return false;
        }
        idx = r->address + off;
        if (idx < 0) {
            return false;
        }
        if (enc & BIT(10)) {
            if (!qcom8998_shader_const(s, block, r, idx, &val)) {
                return false;
            }
            if (!full && kind == 0) {
                val = qcom8998_f16_to_f32((uint16_t)val);
            }
        } else {
            if (!qcom8998_ir3_read_reg(r, idx, full, &val)) {
                return false;
            }
            if (!full && kind == 0) {
                val = qcom8998_f16_to_f32((uint16_t)val);
            } else if (!full && kind == 1) {
                val = (uint32_t)(int32_t)(int16_t)val;
            }
        }
    } else if ((sel & 3) == 2) {
        unsigned idx = (enc & 0x7ff) + inc;

        if (!qcom8998_shader_const(s, block, r, idx, &val)) {
            return false;
        }
        if (!full) {
            if (kind == 0) {
                val = qcom8998_f16_to_f32((uint16_t)val);
            } else if (kind == 1) {
                val = (uint32_t)(int32_t)(int16_t)val;
            } else {
                val &= 0xffff;
            }
        }
    } else if (sel == 4) {
        int32_t imm = sextract32(enc, 0, 11);

        val = kind == 0 ? qcom8998_f_to_u32((float)imm) : (uint32_t)imm;
    } else if (sel == 5) {
        unsigned fi = enc & 0x3ff;

        if (fi >= ARRAY_SIZE(flut)) {
            return false;
        }
        val = qcom8998_f_to_u32(flut[fi]);
    } else {
        return false;
    }

    if (kind == 0) {
        if (absneg & 2) {
            val &= ~BIT(31);
        }
        if (absneg & 1) {
            val ^= BIT(31);
        }
    } else if (kind == 1) {
        if (absneg & 2) {
            int32_t sv = (int32_t)val;
            val = sv < 0 ? (uint32_t)(-(int64_t)sv) : val;
        }
        if (absneg & 1) {
            val = (uint32_t)(-(int32_t)val);
        }
    } else {
        if (absneg & 1) {
            val = ~val;
        }
    }
    *out = val;
    return true;
}

static inline bool qcom8998_ir3_cat3_src(Qcom8998Display *s, unsigned block,
                                         const Qcom8998ShaderRegs *r,
                                         unsigned enc13, bool neg, unsigned inc,
                                         bool full, unsigned kind,
                                         uint32_t *out)
{
    unsigned sel = extract32(enc13, 11, 2);
    uint32_t val = 0;

    if (sel == 0) {
        unsigned idx = (enc13 & 0xff) + inc;

        if (!qcom8998_ir3_read_reg(r, idx, full, &val)) {
            return false;
        }
    } else if (sel == 1) {
        int idx;

        if (!r->address_valid) {
            return false;
        }
        idx = r->address + sextract32(enc13, 0, 10) + (int)inc;
        if (idx < 0) {
            return false;
        }
        if (enc13 & BIT(10)) {
            if (!qcom8998_shader_const(s, block, r, idx, &val)) {
                return false;
            }
        } else if (!qcom8998_ir3_read_reg(r, idx, full, &val)) {
            return false;
        }
    } else if (sel == 2) {
        unsigned idx = (enc13 & 0x7ff) + inc;

        if (!qcom8998_shader_const(s, block, r, idx, &val)) {
            return false;
        }
    } else {
        return false;
    }

    if (!full) {
        if (kind == 0) {
            val = qcom8998_f16_to_f32((uint16_t)val);
        } else if (kind == 1) {
            val = (uint32_t)(int32_t)(int16_t)val;
        } else {
            val &= 0xffff;
        }
    }
    if (neg) {
        if (kind == 0) {
            val ^= BIT(31);
        } else if (kind == 1) {
            val = (uint32_t)(-(int32_t)val);
        } else {
            val = ~val;
        }
    }
    *out = val;
    return true;
}

static int qcom8998_tex_wrap(int coord, int size, unsigned mode, bool *border)
{
    if (size <= 0) {
        return 0;
    }
    switch (mode) {
    case 0: /* REPEAT */
        coord %= size;
        return coord < 0 ? coord + size : coord;
    case 1: /* CLAMP_TO_EDGE */
        return CLAMP(coord, 0, size - 1);
    case 2: /* MIRROR_REPEAT */
        {
            int p = 2 * size;
            int m = coord % p;
            if (m < 0) {
                m += p;
            }
            return m < size ? m : (p - 1 - m);
        }
    case 3: /* CLAMP_TO_BORDER */
        if (coord < 0 || coord >= size) {
            *border = true;
            return 0;
        }
        return coord;
    default:
        return CLAMP(coord, 0, size - 1);
    }
}

static bool qcom8998_tex_prepare(Qcom8998Display *s, const uint32_t *tex,
                                 Qcom8998TexInfo *info)
{
    unsigned fmt = extract32(tex[0], 22, 8);
    unsigned cpp;

    memset(info, 0, sizeof(*info));
    switch (fmt) {
    case 0x02: /* A8_UNORM */
    case 0x03: /* R8_UNORM */
    case 0x05: /* R8_UINT */
        cpp = 1;
        break;
    case 0x0f: /* R8G8_UNORM */
        cpp = 2;
        break;
    case 0x30: /* R8G8B8A8_UNORM */
    case 0x33: /* R8G8B8A8_UINT */
    case 0x4a: /* R32_FLOAT */
    case 0x4b: /* R32_UINT */
    case 0x4c: /* R32_SINT */
        cpp = 4;
        break;
    default:
        return false;
    }

    info->tile = tex[0] & 3;
    info->fmt = fmt;
    info->swap = extract32(tex[0], 30, 2);
    info->srgb = !!(tex[0] & BIT(2));
    info->width = extract32(tex[1], 0, 15);
    info->height = extract32(tex[1], 15, 15);
    info->tex_type = extract32(tex[2], 29, 3);
    info->pitch = extract32(tex[2], 7, 22);
    info->ubwc = !!(tex[3] & BIT(28));
    info->base = deposit64(tex[4], 32, 17, tex[5] & 0x1ffff);
    info->hbb = 13 + extract32(s->bank[BANK_ADRENO].regs[ADRENO_TPL1_MODE_CNTL],
                               7, 2);
    info->cpp = cpp;
    info->bw = cpp == 1 ? 32 : 16;
    info->bh = cpp == 1 ? 8 : 4;
    info->identity_swizzle = true;
    for (unsigned i = 0; i < 4; i++) {
        info->swizzle[i] = extract32(tex[0], 4 + i * 3, 3);
        if (info->swizzle[i] != i) {
            info->identity_swizzle = false;
        }
    }
    if (info->tile == 3) {
        if (info->ubwc) {
            if (tex[7] & ~31U) {
                info->fbase = deposit64(tex[7] & ~31U, 32, 17,
                                        tex[8] & 0x1ffff);
                info->fpitch = (uint64_t)(tex[10] & 0x7f) << 6;
            } else {
                uint64_t fheight = ROUND_UP(DIV_ROUND_UP(info->height,
                                                         info->bh), 16);
                uint64_t fsize;

                info->fpitch = ROUND_UP(DIV_ROUND_UP(info->pitch / cpp,
                                                     info->bw), 64);
                fsize = ROUND_UP(info->fpitch * fheight, 4096);
                info->fbase = info->base;
                info->base += fsize;
            }
            if (!info->fbase || !info->fpitch) {
                return false;
            }
        }
        qcom8998_tile5_3_params(info->pitch, cpp, info->hbb,
                                &info->macrotile_stride, &info->bank_mask);
    } else if (info->tile != 0) {
        return false;
    }
    info->valid = true;
    return true;
}

static bool qcom8998_tex_fetch_texel(Qcom8998Display *s,
                                     Qcom8998PageCache *pages,
                                     const Qcom8998TexInfo *info,
                                     int x, int y,
                                     bool border, float out[4],
                                     uint32_t raw_out[4], bool *is_int)
{
    uint64_t paddr;
    uint32_t word = 0;
    float comps[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    uint32_t icomps[4] = { 0, 0, 0, 1 };

    *is_int = false;
    if (border || x < 0 || y < 0 ||
        x >= (int)info->width || y >= (int)info->height) {
        out[0] = out[1] = out[2] = out[3] = 0.0f;
        raw_out[0] = raw_out[1] = raw_out[2] = raw_out[3] = 0;
        return true;
    }

    if (info->tile == 3) {
        unsigned xb = (unsigned)x / info->bw;
        unsigned yb = (unsigned)y / info->bh;

        if (info->ubwc) {
            uint32_t fbyte = 0;

            if (!qcom8998_texture_word(s, pages,
                    info->fbase + (uint64_t)yb * info->fpitch + xb,
                    1, &fbyte)) {
                trace_qcom8998_cp_draw_reject("sam ubwc read", info->fbase,
                                              info->fpitch);
                return false;
            }
            if (fbyte == 0x00) {
                word = 0;
                goto decode;
            }
            if (fbyte != 0xff) {
                trace_qcom8998_cp_draw_reject("sam ubwc flag", fbyte,
                                              info->fbase);
                return false;
            }
        }
        paddr = info->base +
            qcom8998_tile5_3_block_offset_fast(xb, yb, info->macrotile_stride,
                                               info->bank_mask, info->hbb) +
            (uint64_t)qcom8998_tile5_3_pixel_index((unsigned)x % info->bw,
                                                   (unsigned)y % info->bh) *
            info->cpp;
    } else {
        paddr = info->base + (uint64_t)y * info->pitch +
                (uint64_t)x * info->cpp;
    }

    if (info->cpp == 2) {
        uint32_t lo = 0, hi = 0;

        if (!qcom8998_texture_word(s, pages, paddr, 1, &lo) ||
            !qcom8998_texture_word(s, pages, paddr + 1, 1, &hi)) {
            return false;
        }
        word = lo | (hi << 8);
    } else if (!qcom8998_texture_word(s, pages, paddr, info->cpp, &word)) {
        return false;
    }

decode:
    switch (info->fmt) {
    case 0x02: /* A8_UNORM */
        comps[0] = comps[1] = comps[2] = 0.0f;
        comps[3] = (float)(word & 0xff) * (1.0f / 255.0f);
        break;
    case 0x03: /* R8_UNORM */
        comps[0] = (float)(word & 0xff) * (1.0f / 255.0f);
        break;
    case 0x05: /* R8_UINT */
        *is_int = true;
        icomps[0] = word & 0xff;
        break;
    case 0x0f: /* R8G8_UNORM */
        comps[0] = (float)(word & 0xff) * (1.0f / 255.0f);
        comps[1] = (float)((word >> 8) & 0xff) * (1.0f / 255.0f);
        break;
    case 0x30: /* RGBA8_UNORM */
        {
            uint32_t c = qcom8998_color_unswap(word, info->swap);

            if (info->srgb) {
                const float *lut = qcom8998_srgb_lut();

                comps[0] = lut[c & 0xff];
                comps[1] = lut[(c >> 8) & 0xff];
                comps[2] = lut[(c >> 16) & 0xff];
                comps[3] = (float)((c >> 24) & 0xff) * (1.0f / 255.0f);
            } else {
                comps[0] = (float)(c & 0xff) * (1.0f / 255.0f);
                comps[1] = (float)((c >> 8) & 0xff) * (1.0f / 255.0f);
                comps[2] = (float)((c >> 16) & 0xff) * (1.0f / 255.0f);
                comps[3] = (float)((c >> 24) & 0xff) * (1.0f / 255.0f);
            }
        }
        break;
    case 0x33: /* RGBA8_UINT */
        {
            uint32_t c = qcom8998_color_unswap(word, info->swap);

            *is_int = true;
            icomps[0] = c & 0xff;
            icomps[1] = (c >> 8) & 0xff;
            icomps[2] = (c >> 16) & 0xff;
            icomps[3] = (c >> 24) & 0xff;
        }
        break;
    case 0x4a: /* R32_FLOAT */
        comps[0] = qcom8998_u32_to_f(word);
        break;
    case 0x4b:
    case 0x4c:
        *is_int = true;
        icomps[0] = word;
        break;
    }

    if (info->identity_swizzle && !*is_int) {
        out[0] = comps[0];
        out[1] = comps[1];
        out[2] = comps[2];
        out[3] = comps[3];
        raw_out[0] = qcom8998_f_to_u32(comps[0]);
        raw_out[1] = qcom8998_f_to_u32(comps[1]);
        raw_out[2] = qcom8998_f_to_u32(comps[2]);
        raw_out[3] = qcom8998_f_to_u32(comps[3]);
        return true;
    }

    for (unsigned i = 0; i < 4; i++) {
        unsigned sw = info->swizzle[i];

        if (*is_int) {
            raw_out[i] = sw < 4 ? icomps[sw] : (sw == 5 ? 1 : 0);
            out[i] = (float)raw_out[i];
        } else {
            out[i] = sw < 4 ? comps[sw] : (sw == 5 ? 1.0f : 0.0f);
            raw_out[i] = qcom8998_f_to_u32(out[i]);
        }
    }
    return true;
}

static bool qcom8998_sample_tex2d(Qcom8998Display *s, Qcom8998PageCache *pages,
                                  const uint32_t *samp,
                                  const Qcom8998TexInfo *info,
                                  float fu, float fv,
                                  float res[4], uint32_t raw_res[4],
                                  bool *is_int)
{
    unsigned width = info->width;
    unsigned height = info->height;
    bool unnorm = !!(samp[1] & BIT(5));
    bool linear = extract32(samp[0], 1, 2) == 1 ||
                  extract32(samp[0], 3, 2) == 1;
    unsigned wrap_s = extract32(samp[0], 5, 3);
    unsigned wrap_t = extract32(samp[0], 8, 3);
    float px = unnorm ? fu : fu * (float)width;
    float py = info->tex_type == 0 ? 0.0f : (unnorm ? fv : fv * (float)height);
    g_autoptr(GHashTable) local_pages = NULL;
    Qcom8998PageCache local_cache;

    if (!pages) {
        local_pages = g_hash_table_new(g_direct_hash, g_direct_equal);
        qcom8998_page_cache_init(&local_cache, local_pages);
        pages = &local_cache;
    }
    if (!linear) {
        bool b = false;
        int ix = qcom8998_tex_wrap((int)floorf(px), width, wrap_s, &b);
        int iy = qcom8998_tex_wrap((int)floorf(py), height, wrap_t, &b);

        return qcom8998_tex_fetch_texel(s, pages, info, ix, iy, b,
                                        res, raw_res, is_int);
    } else {
        float fx = px - 0.5f;
        float fy = py - 0.5f;
        int x0 = (int)floorf(fx);
        int y0 = (int)floorf(fy);
        float ax = fx - (float)x0;
        float ay = fy - (float)y0;
        float c00[4], c10[4], c01[4], c11[4];
        uint32_t rtmp[4];
        bool b00 = false, b10 = false, b01 = false, b11 = false;
        int wx0 = qcom8998_tex_wrap(x0, width, wrap_s, &b00);
        int wy0 = qcom8998_tex_wrap(y0, height, wrap_t, &b01);

        if (ax == 0.0f && ay == 0.0f) {
            return qcom8998_tex_fetch_texel(s, pages, info, wx0, wy0,
                                            b00 || b01, res, raw_res, is_int);
        }
        if (ay == 0.0f) {
            int wx1 = qcom8998_tex_wrap(x0 + 1, width, wrap_s, &b10);

            if (!qcom8998_tex_fetch_texel(s, pages, info, wx0, wy0, b00 || b01,
                                          c00, rtmp, is_int) ||
                !qcom8998_tex_fetch_texel(s, pages, info, wx1, wy0, b10 || b01,
                                          c10, rtmp, is_int)) {
                return false;
            }
            for (unsigned i = 0; i < 4; i++) {
                res[i] = c00[i] * (1.0f - ax) + c10[i] * ax;
                raw_res[i] = qcom8998_f_to_u32(res[i]);
            }
            return true;
        }
        if (ax == 0.0f) {
            int wy1 = qcom8998_tex_wrap(y0 + 1, height, wrap_t, &b11);

            if (!qcom8998_tex_fetch_texel(s, pages, info, wx0, wy0, b00 || b01,
                                          c00, rtmp, is_int) ||
                !qcom8998_tex_fetch_texel(s, pages, info, wx0, wy1, b00 || b11,
                                          c01, rtmp, is_int)) {
                return false;
            }
            for (unsigned i = 0; i < 4; i++) {
                res[i] = c00[i] * (1.0f - ay) + c01[i] * ay;
                raw_res[i] = qcom8998_f_to_u32(res[i]);
            }
            return true;
        }
        int wx1 = qcom8998_tex_wrap(x0 + 1, width, wrap_s, &b10);
        int wy1 = qcom8998_tex_wrap(y0 + 1, height, wrap_t, &b11);

        if (!qcom8998_tex_fetch_texel(s, pages, info, wx0, wy0, b00 || b01,
                                      c00, rtmp, is_int) ||
            !qcom8998_tex_fetch_texel(s, pages, info, wx1, wy0, b10 || b01,
                                      c10, rtmp, is_int) ||
            !qcom8998_tex_fetch_texel(s, pages, info, wx0, wy1, b00 || b11,
                                      c01, rtmp, is_int) ||
            !qcom8998_tex_fetch_texel(s, pages, info, wx1, wy1, b10 || b11,
                                      c11, rtmp, is_int)) {
            return false;
        }
        for (unsigned i = 0; i < 4; i++) {
            float top = c00[i] * (1.0f - ax) + c10[i] * ax;
            float bot = c01[i] * (1.0f - ax) + c11[i] * ax;

            res[i] = top * (1.0f - ay) + bot * ay;
            raw_res[i] = qcom8998_f_to_u32(res[i]);
        }
        return true;
    }
}

static bool qcom8998_fs_prefetch(Qcom8998Display *s,
                                 Qcom8998ShaderRegs *shader)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned cntl = regs[0xe5db];
    unsigned count = cntl & 7;
    const GArray *samplers = s->load_state[4][0];
    const GArray *textures = s->load_state[4][1];

    if (!count) {
        return true;
    }
    if (count > 4 || (cntl & ~0x1fU)) {
        trace_qcom8998_cp_draw_reject("fs prefetch cntl", cntl, regs[0xe5dc]);
        return false;
    }
    for (unsigned i = 0; i < count; i++) {
        uint32_t cmd = regs[0xe5dc + i];
        unsigned src = extract32(cmd, 0, 7);
        unsigned samp_id = extract32(cmd, 7, 4);
        unsigned tex_id = extract32(cmd, 11, 5);
        unsigned dst = extract32(cmd, 16, 6);
        unsigned wrmask = extract32(cmd, 22, 4);
        bool half = !!(cmd & BIT(26));
        unsigned opc = extract32(cmd, 29, 3);
        const uint32_t *samp;
        Qcom8998TexInfo local_info;
        const Qcom8998TexInfo *info;
        float fu, fv, res[4] = { 0, 0, 0, 1 };
        uint32_t raw_res[4] = { 0, 0, 0, 1 };
        bool is_int = false;

        if (opc != 1 || (cmd & (BIT(27) | BIT(28))) ||
            src + 1 >= ARRAY_SIZE(shader->input) ||
            !(shader->input_valid & BIT_ULL(src)) ||
            !(shader->input_valid & BIT_ULL(src + 1)) ||
            !samplers || samp_id >= samplers->len ||
            !g_array_index(samplers, Qcom8998StateUnit, samp_id).valid) {
            trace_qcom8998_cp_draw_reject("fs prefetch cmd", cmd, src);
            return false;
        }
        if (shader->tex_info && tex_id < shader->tex_info_count) {
            info = &shader->tex_info[tex_id];
            if (!info->valid) {
                trace_qcom8998_cp_draw_reject("fs prefetch tex", cmd, tex_id);
                return false;
            }
        } else {
            const uint32_t *tex;

            if (!textures || tex_id >= textures->len ||
                !g_array_index(textures, Qcom8998StateUnit, tex_id).valid) {
                trace_qcom8998_cp_draw_reject("fs prefetch cmd", cmd, src);
                return false;
            }
            tex = g_array_index(textures, Qcom8998StateUnit, tex_id).words;
            if (!qcom8998_tex_prepare(s, tex, &local_info)) {
                trace_qcom8998_cp_draw_reject("sam tex fmt", tex[0], tex_id);
                return false;
            }
            info = &local_info;
        }
        samp = g_array_index(samplers, Qcom8998StateUnit, samp_id).words;
        fu = qcom8998_u32_to_f(shader->input[src]);
        fv = qcom8998_u32_to_f(shader->input[src + 1]);
        if (!qcom8998_sample_tex2d(s, shader->tex_pages, samp, info, fu, fv,
                                   res, raw_res, &is_int)) {
            return false;
        }
        for (unsigned c = 0; c < 4; c++) {
            if (wrmask & BIT(c)) {
                if (!qcom8998_ir3_write_reg(shader, dst + c, half, true,
                                            qcom8998_f_to_u32(res[c]))) {
                    return false;
                }
            }
        }
    }
    return true;
}

static bool qcom8998_ir3_cat5_tex(Qcom8998Display *s, unsigned block,
                                  Qcom8998ShaderRegs *r, uint64_t instruction)
{
    unsigned opc = extract64(instruction, 54, 5);
    bool full_src = !!(instruction & BIT_ULL(0));
    unsigned src1 = extract64(instruction, 1, 8);
    unsigned samp_idx = extract64(instruction, 21, 4);
    unsigned tex_idx = extract64(instruction, 25, 7);
    unsigned dst = extract64(instruction, 32, 8);
    unsigned wrmask = extract64(instruction, 40, 4);
    unsigned dst_type = extract64(instruction, 44, 3);
    bool dst_half = dst_type == 0 || dst_type == 2 || dst_type == 4 ||
                    dst_type == 6;
    bool is_float_dst = dst_type == 0 || dst_type == 1;
    unsigned tex_block = block == 12 ? 4 : 0;
    const GArray *samplers = s->load_state[tex_block][0];
    const GArray *textures = s->load_state[tex_block][1];
    const uint32_t *samp = NULL, *tex = NULL;
    Qcom8998TexInfo local_info;
    const Qcom8998TexInfo *info = NULL;
    g_autoptr(GHashTable) local_pages = NULL;
    Qcom8998PageCache local_cache;
    Qcom8998PageCache *pages = r->tex_pages;
    float res[4] = { 0, 0, 0, 1 };
    uint32_t raw_res[4] = { 0, 0, 0, 1 };
    bool is_int = false;

    /* Supported: isam (0), isaml (1), isamm (2), sam (3), samb (4), saml (5). */
    if (opc > 5 || (instruction & (BIT_ULL(48) | BIT_ULL(51)))) {
        return false;
    }
    if (block == 12 && r->tex_info && tex_idx < r->tex_info_count &&
        r->tex_info[tex_idx].valid) {
        info = &r->tex_info[tex_idx];
    } else {
        if (!textures || tex_idx >= textures->len ||
            !g_array_index(textures, Qcom8998StateUnit, tex_idx).valid) {
            return false;
        }
        tex = g_array_index(textures, Qcom8998StateUnit, tex_idx).words;
    }
    if (opc >= 3) {
        if (!samplers || samp_idx >= samplers->len ||
            !g_array_index(samplers, Qcom8998StateUnit, samp_idx).valid) {
            return false;
        }
        samp = g_array_index(samplers, Qcom8998StateUnit, samp_idx).words;
    }
    if (!pages) {
        local_pages = g_hash_table_new(g_direct_hash, g_direct_equal);
        qcom8998_page_cache_init(&local_cache, local_pages);
        pages = &local_cache;
    }

    unsigned tex_type = info ? info->tex_type : extract32(tex[2], 29, 3);
    uint32_t u_raw = 0, v_raw = 0;

    if (!qcom8998_ir3_read_reg(r, src1, full_src, &u_raw)) {
        return false;
    }
    if (tex_type != 0 && tex_type != 4) { /* 2D */
        if (!qcom8998_ir3_read_reg(r, src1 + 1, full_src, &v_raw)) {
            return false;
        }
    }

    if (opc <= 2) { /* isam / isaml / isamm: integer texelFetch */
        int ix = full_src ? (int32_t)u_raw : (int16_t)u_raw;
        int iy = (tex_type == 0 || tex_type == 4) ? 0 :
                 (full_src ? (int32_t)v_raw : (int16_t)v_raw);

        if (tex_type == 4) { /* BUFFER texture */
            uint64_t base = info ? info->base :
                deposit64(tex[4], 32, 17, tex[5] & 0x1ffff);
            unsigned fmt = info ? info->fmt : extract32(tex[0], 22, 8);
            unsigned cpp = (fmt == 2 || fmt == 3 || fmt == 5) ? 1 : 4;
            uint32_t w = 0;

            if (ix < 0 || !qcom8998_texture_word(s, pages,
                    base + (uint64_t)ix * cpp, cpp, &w)) {
                return false;
            }
            if (fmt == 2 || fmt == 3) {
                res[0] = (float)(w & 0xff) * (1.0f / 255.0f);
                raw_res[0] = qcom8998_f_to_u32(res[0]);
            } else if (fmt == 5) {
                is_int = true;
                raw_res[0] = w & 0xff;
            } else {
                is_int = !is_float_dst;
                raw_res[0] = w;
                res[0] = qcom8998_u32_to_f(w);
            }
        } else {
            if (!info) {
                if (!qcom8998_tex_prepare(s, tex, &local_info)) {
                    return false;
                }
                info = &local_info;
            }
            if (!qcom8998_tex_fetch_texel(s, pages, info, ix, iy, false,
                                          res, raw_res, &is_int)) {
                return false;
            }
        }
    } else {
        float fu = full_src ? qcom8998_u32_to_f(u_raw) :
                   qcom8998_u32_to_f(qcom8998_f16_to_f32((uint16_t)u_raw));
        float fv = (tex_type == 0) ? 0.0f :
                   (full_src ? qcom8998_u32_to_f(v_raw) :
                    qcom8998_u32_to_f(qcom8998_f16_to_f32((uint16_t)v_raw)));

        if (!info) {
            if (!qcom8998_tex_prepare(s, tex, &local_info)) {
                return false;
            }
            info = &local_info;
        }
        if (!qcom8998_sample_tex2d(s, pages, samp, info, fu, fv,
                                   res, raw_res, &is_int)) {
            return false;
        }
    }

    unsigned out_idx = dst;
    for (unsigned c = 0; c < 4; c++) {
        if (wrmask & BIT(c)) {
            uint32_t out_val = is_float_dst ? qcom8998_f_to_u32(res[c]) :
                                              raw_res[c];

            if (!qcom8998_ir3_write_reg(r, out_idx++, dst_half, is_float_dst,
                                        out_val)) {
                return false;
            }
        }
    }
    return true;
}

static bool qcom8998_shader_execute(Qcom8998Display *s, unsigned block,
                                    Qcom8998ShaderRegs *r)
{
    unsigned pred_mode = 0; /* 0 = none, 1 = predt, 2 = predf */
    bool pred_val = true;
    int pc = 0;

    for (unsigned step = 0; step < ADRENO_SHADER_STEPS; step++) {
        uint32_t low, high;
        uint64_t instruction;
        unsigned category, dst;

        r->steps = step + 1;
        if (pc < 0) {
            return false;
        }
        if (r->program && (unsigned)pc < r->program->insn_count) {
            if (!qcom8998_bit_test(r->program->insn_valid, (unsigned)pc)) {
                return false;
            }
            instruction = r->program->insns[pc];
            low = (uint32_t)instruction;
            high = (uint32_t)(instruction >> 32);
        } else {
            if (!qcom8998_shader_word(s, block, 0, pc * 2, &low) ||
                !qcom8998_shader_word(s, block, 0, pc * 2 + 1, &high)) {
                return false;
            }
            instruction = deposit64(low, 32, 32, high);
        }
        r->instruction[0] = low;
        r->instruction[1] = high;
        category = extract64(instruction, 61, 3);
        dst = extract64(instruction, 32, 8);
        pc++;

        /* Handle predicated execution (predt/predf/prede). */
        if (category == 0 && (instruction & BIT_ULL(49))) {
            unsigned opc = extract64(instruction, 55, 4);

            if (opc == 13) { /* predt */
                pred_val = (r->pred_valid & BIT(0)) ? !!r->pred[0] : false;
                pred_mode = 1;
                continue;
            }
            if (opc == 14) { /* predf */
                pred_val = (r->pred_valid & BIT(0)) ? !!r->pred[0] : false;
                pred_mode = 2;
                continue;
            }
            if (opc == 15) { /* prede */
                pred_mode = 0;
                continue;
            }
        }
        if ((pred_mode == 1 && !pred_val) || (pred_mode == 2 && pred_val)) {
            continue;
        }

        if (category == 0) {
            unsigned opc_hi = extract64(instruction, 49, 1);
            unsigned opc = extract64(instruction, 55, 4);

            if (!opc_hi) {
                if (opc == 0) { /* nop */
                    continue;
                }
                if (opc == 6) { /* end */
                    return true;
                }
                if (opc == 5) { /* kill */
                    unsigned comp = extract64(instruction, 53, 2);
                    bool inv = !!(instruction & BIT_ULL(52));
                    bool cond = (r->pred_valid & BIT(comp)) ? !!r->pred[comp] :
                                                              false;

                    if (inv) {
                        cond = !cond;
                    }
                    if (cond) {
                        r->killed = true;
                        return true;
                    }
                    continue;
                }
                if (opc == 2) { /* jump */
                    pc += (int32_t)low - 1;
                    continue;
                }
                if (opc == 1) { /* br / bany / ball / brao / braa */
                    unsigned brtype = extract64(instruction, 37, 3);
                    unsigned c1 = extract64(instruction, 53, 2);
                    bool inv1 = !!(instruction & BIT_ULL(52));
                    bool p1 = ((r->pred_valid & BIT(c1)) ? !!r->pred[c1] :
                                                           false) ^ inv1;
                    bool take = false;

                    if (brtype == 0 || brtype == 4 || brtype == 5) {
                        take = p1;
                    } else if (brtype == 1 || brtype == 2) {
                        unsigned c2 = extract64(instruction, 46, 2);
                        bool inv2 = !!(instruction & BIT_ULL(45));
                        bool p2 = ((r->pred_valid & BIT(c2)) ? !!r->pred[c2] :
                                                               false) ^ inv2;
                        take = brtype == 1 ? (p1 || p2) : (p1 && p2);
                    } else {
                        return false;
                    }
                    if (take) {
                        pc += (int32_t)low - 1;
                    }
                    continue;
                }
            }
            return false;
        } else if (category == 1) {
            unsigned src_type = extract64(instruction, 50, 3);
            unsigned dst_type = extract64(instruction, 46, 3);
            unsigned round_mode = extract64(instruction, 55, 2);
            unsigned mode = extract64(instruction, 53, 2);
            unsigned repeat = extract64(instruction, 40, 2);
            unsigned sub = extract64(instruction, 57, 2);
            bool source_half = src_type == 0 || src_type == 2 ||
                               src_type == 4 || src_type == 6 ||
                               src_type == 7;
            bool dest_half = dst_type == 0 || dst_type == 2 ||
                             dst_type == 4 || dst_type == 6 ||
                             dst_type == 7;

            if (sub != 0 || (instruction & (BIT_ULL(42) | BIT_ULL(49))) ||
                mode == 3) {
                return false;
            }
            for (unsigned n = 0; n <= repeat; n++) {
                unsigned increment = (repeat && (instruction & BIT_ULL(43))) ?
                                     n : 0;
                uint32_t raw = 0, out_val = 0;
                int index;

                if (mode == 2) {
                    raw = low;
                } else if (mode == 1 || (low & BIT(11))) {
                    if (mode == 1) {
                        index = (low & 0x7ff) + increment;
                    } else {
                        if (!r->address_valid) {
                            return false;
                        }
                        index = r->address + sextract32(low, 0, 10) + increment;
                    }
                    if (index < 0) {
                        return false;
                    }
                    if (mode == 1 || (low & BIT(10))) {
                        if (!qcom8998_shader_const(s, block, r, index, &raw)) {
                            return false;
                        }
                    } else if (!qcom8998_ir3_read_reg(r, index, !source_half,
                                                      &raw)) {
                        return false;
                    }
                } else {
                    index = (low & 0xff) + increment;
                    if (!qcom8998_ir3_read_reg(r, index, !source_half, &raw)) {
                        return false;
                    }
                }

                if (src_type == dst_type) {
                    out_val = raw;
                } else {
                    float f = 0.0f;
                    int64_t sv = 0;
                    uint64_t uv = 0;
                    bool src_is_float = src_type == 0 || src_type == 1;
                    bool src_is_signed = src_type == 4 || src_type == 5 ||
                                         src_type == 7;

                    if (src_type == 0) {
                        f = qcom8998_u32_to_f(
                            qcom8998_f16_to_f32((uint16_t)raw));
                    } else if (src_type == 1) {
                        f = qcom8998_u32_to_f(raw);
                    } else if (src_type == 2) {
                        uv = raw & 0xffff;
                    } else if (src_type == 3) {
                        uv = raw;
                    } else if (src_type == 4) {
                        sv = (int16_t)raw;
                    } else if (src_type == 5) {
                        sv = (int32_t)raw;
                    } else if (src_type == 7) {
                        sv = (int8_t)raw;
                    } else {
                        uv = raw & 0xff;
                    }

                    if (src_is_float && dst_type >= 2) {
                        if (round_mode == 1) {
                            f = nearbyintf(f);
                        } else if (round_mode == 2) {
                            f = ceilf(f);
                        } else if (round_mode == 3) {
                            f = floorf(f);
                        }
                    }
                    if (dst_type == 0 || dst_type == 1) {
                        float df = src_is_float ? f :
                                   (src_is_signed ? (float)sv : (float)uv);
                        uint32_t fbits = qcom8998_f_to_u32(df);
                        out_val = dst_type == 0 ?
                            qcom8998_f32_to_f16(fbits) : fbits;
                    } else if (dst_type == 4 || dst_type == 5 ||
                               dst_type == 7) {
                        int64_t dv = src_is_float ?
                            (isnan(f) ? 0 : (int64_t)f) :
                            (src_is_signed ? sv : (int64_t)uv);
                        out_val = dst_type == 4 ?
                            (uint16_t)CLAMP(dv, INT16_MIN, INT16_MAX) :
                            dst_type == 7 ?
                            (uint8_t)CLAMP(dv, INT8_MIN, INT8_MAX) :
                            (uint32_t)CLAMP(dv, INT32_MIN, INT32_MAX);
                    } else {
                        uint64_t dv = src_is_float ?
                            (isnan(f) || f <= 0.0f ? 0 : (uint64_t)f) :
                            (src_is_signed ? (sv < 0 ? 0 : (uint64_t)sv) : uv);
                        out_val = dst_type == 2 ?
                            (uint16_t)MIN(dv, (uint64_t)UINT16_MAX) :
                            dst_type == 6 ? (uint8_t)MIN(dv, 255ULL) :
                            (uint32_t)MIN(dv, (uint64_t)UINT32_MAX);
                    }
                }
                if (!qcom8998_ir3_write_reg(r, dst + n, dest_half, false,
                                            out_val)) {
                    return false;
                }
            }
            continue;
        } else if (category == 2) {
            unsigned opcode = extract64(instruction, 53, 6);
            bool full = !!(instruction & BIT_ULL(52));
            bool dst_conv = !!(instruction & BIT_ULL(46));
            bool sat = !!(instruction & BIT_ULL(42));
            unsigned repeat = extract64(instruction, 40, 2);
            unsigned cond = extract64(instruction, 48, 3);
            bool dest_half = (full == dst_conv) && dst <= 0xf7;
            bool is_float = opcode <= 13 || opcode == 57;
            bool is_signed = opcode == 17 || opcode == 19 || opcode == 21 ||
                             opcode == 23 || opcode == 25 || opcode == 26 ||
                             opcode == 34 || opcode == 49 || opcode == 52 ||
                             opcode == 56;
            unsigned kind = is_float ? 0 : (is_signed ? 1 : 2);
            bool one_src = opcode == 4 || opcode == 6 ||
                           (opcode >= 9 && opcode <= 13) || opcode == 26 ||
                           opcode == 30 || opcode == 51 || opcode == 52 ||
                           opcode == 53 || opcode == 61;

            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc1 = (repeat && (instruction & BIT_ULL(43))) ? n : 0;
                unsigned inc2 = (repeat && (instruction & BIT_ULL(51))) ? n : 0;
                uint32_t a = 0, b = 0, value = 0;
                bool out_is_float = is_float && opcode != 5 && opcode != 7;

                if (opcode == 57) { /* bary.f */
                    unsigned input = (low & 0x7ff) + inc1;

                    if (block != 12) {
                        return false;
                    }
                    if (dst + n >= 252 && dst + n <= 255) {
                        continue; /* bary.f (ei)r63.x end-of-interpolation */
                    }
                    if ((low & 0xf800) != 0x2000 ||
                        input >= ARRAY_SIZE(r->input) ||
                        !(r->input_valid & BIT_ULL(input))) {
                        return false;
                    }
                    value = r->input[input];
                } else {
                    if (!qcom8998_ir3_multisrc(s, block, r, low & 0xffff, inc1,
                                               full, kind, &a)) {
                        return false;
                    }
                    if (!one_src &&
                        !qcom8998_ir3_multisrc(s, block, r, low >> 16, inc2,
                                               full, kind, &b)) {
                        return false;
                    }
                    switch (opcode) {
                    case 0: /* add.f */
                        value = qcom8998_f_to_u32(qcom8998_u32_to_f(a) +
                                                  qcom8998_u32_to_f(b));
                        break;
                    case 1: /* min.f */
                        {
                            float fa = qcom8998_u32_to_f(a);
                            float fb = qcom8998_u32_to_f(b);
                            value = qcom8998_f_to_u32(
                                isnan(fa) ? fb : isnan(fb) ? fa : MIN(fa, fb));
                        }
                        break;
                    case 2: /* max.f */
                        {
                            float fa = qcom8998_u32_to_f(a);
                            float fb = qcom8998_u32_to_f(b);
                            value = qcom8998_f_to_u32(
                                isnan(fa) ? fb : isnan(fb) ? fa : MAX(fa, fb));
                        }
                        break;
                    case 3: /* mul.f */
                        value = qcom8998_f_to_u32(qcom8998_u32_to_f(a) *
                                                  qcom8998_u32_to_f(b));
                        break;
                    case 4: /* sign.f */
                        {
                            float fa = qcom8998_u32_to_f(a);
                            value = qcom8998_f_to_u32(
                                fa > 0.0f ? 1.0f : fa < 0.0f ? -1.0f : 0.0f);
                        }
                        break;
                    case 5: /* cmps.f */
                    case 7: /* cmpv.f */
                        {
                            float fa = qcom8998_u32_to_f(a);
                            float fb = qcom8998_u32_to_f(b);
                            bool res = false;
                            switch (cond) {
                            case 0: res = fa < fb; break;
                            case 1: res = fa <= fb; break;
                            case 2: res = fa > fb; break;
                            case 3: res = fa >= fb; break;
                            case 4: res = fa == fb; break;
                            case 5: res = fa != fb; break;
                            default: return false;
                            }
                            value = res ? 1 : 0;
                        }
                        break;
                    case 6: /* absneg.f */
                        value = a;
                        break;
                    case 9: /* floor.f */
                        value = qcom8998_f_to_u32(floorf(qcom8998_u32_to_f(a)));
                        break;
                    case 10: /* ceil.f */
                        value = qcom8998_f_to_u32(ceilf(qcom8998_u32_to_f(a)));
                        break;
                    case 11: /* rndne.f */
                        value = qcom8998_f_to_u32(
                            nearbyintf(qcom8998_u32_to_f(a)));
                        break;
                    case 12: /* rndaz.f */
                        value = qcom8998_f_to_u32(roundf(qcom8998_u32_to_f(a)));
                        break;
                    case 13: /* trunc.f */
                        value = qcom8998_f_to_u32(truncf(qcom8998_u32_to_f(a)));
                        break;
                    case 16: /* add.u */
                    case 17: /* add.s */
                        value = a + b;
                        break;
                    case 18: /* sub.u */
                    case 19: /* sub.s */
                        value = a - b;
                        break;
                    case 20: /* cmps.u */
                    case 33: /* cmpv.u */
                        {
                            bool res = false;
                            switch (cond) {
                            case 0: res = a < b; break;
                            case 1: res = a <= b; break;
                            case 2: res = a > b; break;
                            case 3: res = a >= b; break;
                            case 4: res = a == b; break;
                            case 5: res = a != b; break;
                            default: return false;
                            }
                            value = res ? 1 : 0;
                        }
                        break;
                    case 21: /* cmps.s */
                    case 34: /* cmpv.s */
                        {
                            int32_t sa = (int32_t)a, sb = (int32_t)b;
                            bool res = false;
                            switch (cond) {
                            case 0: res = sa < sb; break;
                            case 1: res = sa <= sb; break;
                            case 2: res = sa > sb; break;
                            case 3: res = sa >= sb; break;
                            case 4: res = sa == sb; break;
                            case 5: res = sa != sb; break;
                            default: return false;
                            }
                            value = res ? 1 : 0;
                        }
                        break;
                    case 22: value = MIN(a, b); break;
                    case 23: value = (uint32_t)MIN((int32_t)a, (int32_t)b); break;
                    case 24: value = MAX(a, b); break;
                    case 25: value = (uint32_t)MAX((int32_t)a, (int32_t)b); break;
                    case 26: value = a; break; /* absneg.s */
                    case 28: value = a & b; break;
                    case 29: value = a | b; break;
                    case 30: value = ~a; break;
                    case 31: value = a ^ b; break;
                    case 48: value = (a & 0xffffff) * (b & 0xffffff); break;
                    case 49:
                        value = (uint32_t)(sextract32(a, 0, 24) *
                                           sextract32(b, 0, 24));
                        break;
                    case 50: /* mull.u: 16x16 -> 32 */
                        value = (a & 0xffff) * (b & 0xffff);
                        break;
                    case 51: value = revbit32(a); break;
                    case 52: value = a ? clz32(a) : 32; break;
                    case 53: value = a ? clz32(a) : 32; break;
                    case 54: value = a << (b & 31); break;
                    case 55: value = a >> (b & 31); break;
                    case 56: value = (uint32_t)((int32_t)a >> (b & 31)); break;
                    case 59: value = (a >> (b & 31)) & 1; break;
                    case 61: value = ctpop32(a); break;
                    default:
                        return false;
                    }
                }
                if (out_is_float && sat) {
                    value = qcom8998_sat_f32(value);
                }
                if (!qcom8998_ir3_write_reg(r, dst + n, dest_half,
                                            out_is_float, value)) {
                    return false;
                }
            }
            continue;
        } else if (category == 3) {
            unsigned opcode = extract64(instruction, 55, 4);
            unsigned repeat = extract64(instruction, 40, 2);
            unsigned src2 = extract64(instruction, 47, 8);
            bool dst_conv = !!(instruction & BIT_ULL(46));
            bool sat = !!(instruction & BIT_ULL(42));
            bool full = (opcode & 1) || opcode == 4;
            bool is_float = opcode == 6 || opcode == 7 ||
                            opcode == 12 || opcode == 13;
            bool is_signed = opcode == 2 || opcode == 3 || opcode == 5 ||
                             opcode == 10 || opcode == 11 ||
                             opcode == 14 || opcode == 15;
            unsigned kind = is_float ? 0 : (is_signed ? 1 : 2);
            bool dest_half = (full == dst_conv) && dst <= 0xf7;

            if (instruction & BIT_ULL(13)) {
                return false;
            }
            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc1 = (repeat && (instruction & BIT_ULL(43))) ? n : 0;
                unsigned inc2 = (repeat && (instruction & BIT_ULL(15))) ? n : 0;
                unsigned inc3 = (repeat && (instruction & BIT_ULL(29))) ? n : 0;
                uint32_t a = 0, b = 0, c = 0, value = 0;
                bool c_full = full || opcode <= 3;

                if (!qcom8998_ir3_cat3_src(s, block, r, low & 0x1fff,
                        !!(instruction & BIT_ULL(14)), inc1, full, kind, &a) ||
                    !qcom8998_ir3_cat3_src(s, block, r, src2,
                        !!(instruction & BIT_ULL(30)), inc2, full, kind, &b) ||
                    !qcom8998_ir3_cat3_src(s, block, r,
                        extract32(low, 16, 13), !!(instruction & BIT_ULL(31)),
                        inc3, c_full, kind, &c)) {
                    return false;
                }
                switch (opcode) {
                case 0: /* mad.u16 */
                    value = (a & 0xffff) * (b & 0xffff) + c;
                    break;
                case 1: /* madsh.u16 */
                    value = (((a & 0xffff) * (b & 0xffff)) << 16) + c;
                    break;
                case 2: /* mad.s16 */
                    value = (uint32_t)((int32_t)(int16_t)a *
                                       (int32_t)(int16_t)b + (int32_t)c);
                    break;
                case 3: /* madsh.m16 */
                    value = ((((a >> 16) & 0xffff) * (b & 0xffff)) << 16) + c;
                    break;
                case 4: /* mad.u24 */
                    value = (a & 0xffffff) * (b & 0xffffff) + c;
                    break;
                case 5: /* mad.s24 */
                    value = (uint32_t)(sextract32(a, 0, 24) *
                                       sextract32(b, 0, 24) + (int32_t)c);
                    break;
                case 6: /* mad.f16 */
                case 7: /* mad.f32 */
                    value = qcom8998_f_to_u32(qcom8998_u32_to_f(a) *
                                              qcom8998_u32_to_f(b) +
                                              qcom8998_u32_to_f(c));
                    break;
                case 8:  /* sel.b16 */
                case 9:  /* sel.b32 */
                case 10: /* sel.s16 */
                case 11: /* sel.s32 */
                    value = b ? a : c;
                    break;
                case 12: /* sel.f16 */
                case 13: /* sel.f32 */
                    {
                        float fb = qcom8998_u32_to_f(b);
                        value = (!isnan(fb) && fb != 0.0f) ? a : c;
                    }
                    break;
                default:
                    return false;
                }
                if (is_float && sat) {
                    value = qcom8998_sat_f32(value);
                }
                if (!qcom8998_ir3_write_reg(r, dst + n, dest_half, is_float,
                                            value)) {
                    return false;
                }
            }
            continue;
        } else if (category == 4) {
            unsigned opcode = extract64(instruction, 53, 6);
            bool full = !!(instruction & BIT_ULL(52));
            bool dst_conv = !!(instruction & BIT_ULL(46));
            bool sat = !!(instruction & BIT_ULL(42));
            unsigned repeat = extract64(instruction, 40, 2);
            bool dest_half = (full == dst_conv) && dst <= 0xf7;

            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc1 = (repeat && (instruction & BIT_ULL(43))) ? n : 0;
                uint32_t a = 0, value = 0;
                float fa, res;

                if (!qcom8998_ir3_multisrc(s, block, r, low & 0xffff, inc1,
                                           full, 0, &a)) {
                    return false;
                }
                fa = qcom8998_u32_to_f(a);
                switch (opcode) {
                case 0: res = 1.0f / fa; break;               /* rcp */
                case 1: case 9: res = 1.0f / sqrtf(fa); break;/* rsq/hrsq */
                case 2: case 10: res = log2f(fa); break;      /* log2 */
                case 3: case 11: res = exp2f(fa); break;      /* exp2 */
                case 4: res = sinf(fa); break;                /* sin */
                case 5: res = cosf(fa); break;                /* cos */
                case 6: res = sqrtf(fa); break;               /* sqrt */
                default: return false;
                }
                value = qcom8998_f_to_u32(res);
                if (sat) {
                    value = qcom8998_sat_f32(value);
                }
                if (!qcom8998_ir3_write_reg(r, dst + n, dest_half, true,
                                            value)) {
                    return false;
                }
            }
            continue;
        } else if (category == 5) {
            if (!qcom8998_ir3_cat5_tex(s, block, r, instruction)) {
                return false;
            }
            continue;
        } else if (category == 6) {
            unsigned opc = extract64(instruction, 54, 5);
            unsigned type = extract64(instruction, 49, 3);
            bool dest_half = type == 0 || type == 2 || type == 4 || type == 6;

            if (opc == 31) { /* ldlv: load flat/local varying */
                unsigned off = extract64(instruction, 1, 13);
                unsigned size = extract64(instruction, 24, 8);

                if (block != 12 || !size || off + size > ARRAY_SIZE(r->flat_input) ||
                    dst + size > ADRENO_SHADER_GPRS) {
                    return false;
                }
                for (unsigned c = 0; c < size; c++) {
                    bool fv = !!(r->flat_valid & BIT_ULL(off + c));
                    bool iv = !!(r->input_valid & BIT_ULL(off + c));
                    uint32_t v = fv ? r->flat_input[off + c] :
                                 (iv ? r->input[off + c] : 0);

                    if (!fv && !iv) {
                        return false;
                    }
                    if (!qcom8998_ir3_write_reg(r, dst + c, dest_half, false,
                                                v)) {
                        return false;
                    }
                }
                continue;
            }
            if (opc == 30) { /* A5xx LDC: load 1..4 32-bit words from UBO */
                unsigned sub = extract64(instruction, 52, 2);
                unsigned ubo_slot = extract64(instruction, 41, 8);
                unsigned src1 = extract64(instruction, 24, 8);
                bool src1_im = !!(instruction & BIT_ULL(23));
                unsigned ncomp = extract64(instruction, 12, 2) + 1;
                unsigned comp_off = extract64(instruction, 9, 2);
                const GArray *table = s->load_state[6][2];
                const Qcom8998StateUnit *descriptor;
                unsigned index, offset_vec4;
                uint64_t buffer, byte_off;

                if (sub != 2 || type != 3 ||
                    block < 8 || block > 12 ||
                    ubo_slot >= ADRENO_UBO_STAGE_SLOTS ||
                    dst + ncomp > ADRENO_SHADER_GPRS ||
                    (!src1_im && (src1 >= ADRENO_SHADER_GPRS ||
                                  !qcom8998_bit_test(r->full_valid, src1)))) {
                    return false;
                }
                index = (block - 8) * ADRENO_UBO_STAGE_STRIDE + ubo_slot;
                if (!table || index >= table->len) {
                    return false;
                }
                descriptor = &g_array_index(table, Qcom8998StateUnit, index);
                buffer = deposit64(descriptor->words[0], 32, 32,
                                   descriptor->words[1]);
                offset_vec4 = src1_im ? src1 : r->full[src1];
                if (!descriptor->valid || !buffer || (buffer & 15) ||
                    offset_vec4 > 0x100000) {
                    return false;
                }
                byte_off = (uint64_t)offset_vec4 * 16 +
                           (uint64_t)comp_off * 4;
                if (buffer > UINT64_MAX - byte_off - ncomp * 4) {
                    return false;
                }
                for (unsigned c = 0; c < ncomp; c++) {
                    uint32_t value;
                    uint64_t addr = buffer + byte_off + c * 4;

                    if (r->tex_pages ?
                        !qcom8998_texture_word(s, r->tex_pages, addr, 4,
                                               &value) :
                        !qcom8998_cp_word(s, addr, &value, false)) {
                        return false;
                    }
                    r->full[dst + c] = value;
                    qcom8998_bit_set(r->full_valid, dst + c);
                }
                continue;
            }
            return false;
        }
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
        qcom8998_bit_set(r->full_valid, vertex_reg);
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
    unsigned position = regs[0xe593] & 0xff;
    unsigned output = regs[0xe5cb] & 0xff;
    uint32_t color = 0;
    unsigned rect_count;
    unsigned rects[128][4];
    unsigned active_rects = 0;
    static const unsigned scissor[] = { 0xe0aa, 0xe0ca, 0xe0ea };
    static const unsigned disabled[] = {
        0xe001, 0xe002, 0xe003, 0xe090, 0xe0a2, 0xe0a3,
        0xe0a4, 0xe100, 0xe142, 0xe143, 0xe1c0,
        0xe2a1, 0xe385, 0xe400, 0xe408, 0xe409,
        0xe5db, 0xe704, 0xe705,
    };

    if (regs[0xe5db]) {
        return qcom8998_cp_textured_rectangle(s, count, payload);
    }

    /* A nonindexed, pretransformed screen rectangle list and constant FS. */
    if (!s->me_initialized || count != 3 || (payload[0] & ~0xc00U) != 0x88 ||
        payload[1] != 1 || payload[2] < 2 || (payload[2] & 1) ||
        payload[2] > ARRAY_SIZE(rects) * 2 ||
        s->render_mode[0] != ADRENO_RENDER_BYPASS ||
        regs[ADRENO_RB_CNTL] != ADRENO_RB_BYPASS ||
        (regs[ADRENO_RB_RENDER_CNTL] & ~(BIT(8) | BIT(16))) ||
        (regs[0xe0a0] & ~BIT(8)) ||
        (regs[0xe0a1] && regs[0xe0a1] != 0x20) ||
        (regs[0xe1a8] & BIT(8)) ||
        (regs[0xe1b1] & (BIT(0) | BIT(1) | BIT(6))) ||
        regs[0xe000] != 0x181 || regs[0xe146] != 1 || regs[0xe147] != 0xf ||
        regs[0xe150] != 0x780 || regs[0xe1a9] != 0xffff0000 ||
        !(regs[0xe584] & 1) || !(regs[0xe585] & 1) ||
        ((regs[0xe586] | regs[0xe587] | regs[0xe588]) & 1) ||
        regs[0xe5ac] != 1 || regs[0xe5ad] ||
        regs[0xe5c3] != 1 || regs[0xe5c4] ||
        regs[0xe592] != 1 || (regs[0xe593] & ~0xffU) != 0xf00 ||
        regs[0xe5a3] || (regs[0xe5cb] & ~0xffU) ||
        (regs[0xe5ca] & 0x3f) != 1 ||
        (regs[0xe5d3] != 0x30 && regs[0xe5d3] != 0x02 &&
         regs[0xe5d3] != 0x03 && regs[0xe5d3] != 0x0f) ||
        position > ADRENO_SHADER_GPRS - 4 ||
        output > ADRENO_SHADER_GPRS - 4) {
        trace_qcom8998_cp_draw_reject("rectangle header", regs[0xe152],
                                      regs[0xe5d3]);
        return false;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(disabled); i++) {
        if (regs[disabled[i]]) {
            trace_qcom8998_cp_draw_reject("rectangle state", disabled[i],
                                          regs[disabled[i]]);
            return false;
        }
    }
    rect_count = payload[2] / 2;
    for (unsigned r = 0; r < rect_count; r++) {
        unsigned coordinates[2][2];
        unsigned x1, y1, x2, y2;

        for (unsigned v = 0; v < 2; v++) {
            if (!qcom8998_shader_run(s, 8, r * 2 + v, regs[0xe401] & 0xff,
                                     &vertex)) {
                return false;
            }
            for (unsigned c = 0; c < 4; c++) {
                if (!qcom8998_bit_test(vertex.full_valid, position + c)) {
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
        if (x1 < x2 && y1 < y2) {
            rects[active_rects][0] = x1;
            rects[active_rects][1] = y1;
            rects[active_rects][2] = x2;
            rects[active_rects][3] = y2;
            active_rects++;
        }
    }
    if (!active_rects) {
        return true; /* No fragment invocation or attachment access. */
    }
    if (!qcom8998_shader_run(s, 12, 0, 0, &fragment)) {
        return false;
    }
    for (unsigned c = 0; c < 4; c++) {
        unsigned component = 0;
        unsigned fmt = regs[0xe5d3];

        if (qcom8998_bit_test(fragment.full_valid, output + c)) {
            if (!qcom8998_unorm8(fragment.full[output + c], &component)) {
                return false;
            }
        } else if (fmt == 0x30 || (fmt == 0x02 && c == 3) ||
                   (fmt == 0x03 && c == 0) || (fmt == 0x0f && c < 2)) {
            return false;
        }
        color |= component << (c * 8);
    }
    for (unsigned r = 0; r < active_rects; r++) {
        unsigned x1 = rects[r][0], y1 = rects[r][1];
        unsigned x2 = rects[r][2], y2 = rects[r][3];

        if (!qcom8998_fill_rect(s, 0, x1 | (y1 << 16),
                                (x2 - 1) | ((y2 - 1) << 16), color)) {
            return false;
        }
        trace_qcom8998_cp_rectangle(x1, y1, x2 - x1, y2 - y1, color);
    }
    s->gpu_dirty = true;
    return true;
}

typedef struct Qcom8998RasterVertex {
    int x, y; /* D3D 16.8 positions, bounded to +/-4096 pixels here. */
    float inv_w;
    uint32_t varyings[64];
    uint64_t varying_mask;
} Qcom8998RasterVertex;

static bool qcom8998_vertex_fetch(Qcom8998Display *s, unsigned index,
                                  unsigned instance, Qcom8998ShaderRegs *r)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned attributes = regs[0xe400];
    float_status fp = { 0 };

    Qcom8998PageCache *tex_pages = r->tex_pages;
    Qcom8998ShaderProgram *program = r->program;

    if (attributes > 32 || (regs[0xe401] & ~0xffffffU)) {
        return false;
    }
    qcom8998_shader_reset_pixel(r);
    r->tex_pages = tex_pages;
    r->program = program;
    r->tex_info = NULL;
    r->tex_info_count = 0;
    for (unsigned i = 0; i < 3; i++) {
        unsigned reg = extract32(regs[0xe401], i * 8, 8);

        if (reg != 0xfc) {
            if (reg >= ADRENO_SHADER_GPRS || i == 2) {
                return false;
            }
            r->full[reg] = i ? (instance + regs[0xe409]) :
                               (index + regs[0xe408]);
            qcom8998_bit_set(r->full_valid, reg);
        }
    }
    for (unsigned i = 0; i < attributes; i++) {
        unsigned decode = regs[0xe48a + i * 2];
        unsigned step_rate = regs[0xe48b + i * 2];
        bool instanced = !!(decode & BIT(17));
        unsigned fetch = 0xe40a + (decode & 31) * 4;
        unsigned format = extract32(decode, 20, 8);
        unsigned offset = extract32(decode, 5, 12);
        unsigned dest = regs[0xe4ca + i];
        unsigned out = extract32(dest, 4, 8);
        unsigned components, bytes, elem_idx;
        bool is_float = true;
        uint32_t value[4] = { 0, 0, 0, 0x3f800000 };
        uint64_t address, position;

        if ((decode & (BIT(18) | BIT(19))) ||
            (dest & ~0xfffU) || out > ADRENO_SHADER_GPRS - 4) {
            return false;
        }
        if (format == 0x7f || format == 0x74 || format == 0x67 ||
            format == 0x4a) {
            components = format == 0x7f ? 4 : format == 0x74 ? 3 :
                         format == 0x67 ? 2 : 1;
            bytes = components * 4;
        } else if (format == 0x4b || format == 0x4c ||
                   format == 0x68 || format == 0x69 ||
                   format == 0x75 || format == 0x76 ||
                   format == 0x80 || format == 0x81) {
            components = (format == 0x4b || format == 0x4c) ? 1 :
                         (format == 0x68 || format == 0x69) ? 2 :
                         (format == 0x75 || format == 0x76) ? 3 : 4;
            bytes = components * 4;
            is_float = false;
            value[3] = 1;
        } else if (format >= 0x43 && format <= 0x47) {
            components = 2;
            bytes = 4;
            is_float = format <= 0x45;
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format >= 0x60 && format <= 0x64) {
            components = 4;
            bytes = 8;
            is_float = format <= 0x62;
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format == 0x30 || format == 0x32 ||
                   format == 0x33 || format == 0x34) {
            components = 4;
            bytes = 4;
            is_float = format <= 0x32;
            if (!is_float) {
                value[3] = 1;
            }
        } else {
            trace_qcom8998_cp_draw_reject("vfd format", format, decode);
            return false;
        }
        elem_idx = instanced ?
                   ((step_rate ? instance / step_rate : 0) + regs[0xe409]) :
                   (index + regs[0xe408]);
        address = qcom8998_reg_pair(regs, fetch * 4);
        position = (uint64_t)elem_idx * regs[fetch + 3] + offset;
        if ((address & 3) || (position & 3) ||
            position + bytes > regs[fetch + 2] ||
            address > UINT64_MAX - position - bytes) {
            return false;
        }
        address += position;
        if (format == 0x30 || format == 0x32 ||
            format == 0x33 || format == 0x34) {
            uint32_t packed;

            if (!qcom8998_cp_word(s, address, &packed, false)) {
                return false;
            }
            packed = le32_to_cpu(qcom8998_color_swap(packed,
                                                     (decode >> 28) & 3));
            for (unsigned c = 0; c < 4; c++) {
                uint8_t b = extract32(packed, c * 8, 8);

                if (format == 0x30) {
                    value[c] = float32_div(uint32_to_float32(b, &fp),
                                           0x437f0000, &fp);
                } else if (format == 0x32) {
                    int8_t sb = (int8_t)b;
                    value[c] = sb == INT8_MIN ? 0xbf800000 :
                        float32_div(int32_to_float32(sb, &fp),
                                    0x42fe0000, &fp);
                } else if (format == 0x33) {
                    value[c] = b;
                } else {
                    value[c] = (uint32_t)(int32_t)(int8_t)b;
                }
            }
        } else if ((format >= 0x43 && format <= 0x47) ||
                   (format >= 0x60 && format <= 0x64)) {
            unsigned base_fmt = format >= 0x60 ? format - 0x60 : format - 0x43;

            for (unsigned w = 0; w < components / 2; w++) {
                uint32_t packed;

                if (!qcom8998_cp_word(s, address + w * 4, &packed, false)) {
                    return false;
                }
                for (unsigned c = 0; c < 2; c++) {
                    uint16_t raw = extract32(packed, c * 16, 16);
                    unsigned ci = w * 2 + c;

                    if (base_fmt == 3) { /* UINT */
                        value[ci] = raw;
                    } else if (base_fmt == 4) { /* SINT */
                        value[ci] = (uint32_t)(int32_t)(int16_t)raw;
                    } else if (base_fmt == 2) { /* FLOAT */
                        value[ci] = qcom8998_f16_to_f32(raw);
                    } else if (base_fmt == 0) { /* UNORM */
                        value[ci] = float32_div(uint32_to_float32(raw, &fp),
                                                0x477fff00, &fp);
                    } else { /* SNORM */
                        int16_t sv = (int16_t)raw;
                        value[ci] = sv == INT16_MIN ? 0xbf800000 :
                            float32_div(int32_to_float32(sv, &fp),
                                        0x46fffe00, &fp);
                    }
                }
            }
        } else {
            for (unsigned c = 0; c < components; c++) {
                if ((r->tex_pages ?
                     !qcom8998_texture_word(s, r->tex_pages, address + c * 4,
                                            4, &value[c]) :
                     !qcom8998_cp_word(s, address + c * 4, &value[c], false)) ||
                    (is_float && !qcom8998_shader_finite(value[c]))) {
                    return false;
                }
            }
        }
        for (unsigned c = 0; c < 4; c++) {
            if (dest & BIT(c)) {
                r->full[out + c] = value[c];
                qcom8998_bit_set(r->full_valid, out + c);
                r->half[out + c] = is_float ?
                    qcom8998_f32_to_f16(value[c]) :
                    (uint16_t)value[c];
                qcom8998_bit_set(r->half_valid, out + c);
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
    unsigned out_count = regs[0xe592];
    unsigned position = regs[0xe593] & 0xff;
    float_status fp = { 0 };
    float w, inv_w;

    if (!out_count || out_count > 32 || position > ADRENO_SHADER_GPRS - 4) {
        return false;
    }
    memset(v, 0, sizeof(*v));
    for (unsigned c = 0; c < 4; c++) {
        if (!qcom8998_bit_test(r->full_valid, position + c)) {
            return false;
        }
    }
    w = qcom8998_u32_to_f(r->full[position + 3]);
    if (!isfinite(w) || w == 0.0f) {
        trace_qcom8998_cp_draw_reject("clip Z/W", r->full[position + 2],
                                      r->full[position + 3]);
        return false;
    }
    inv_w = 1.0f / w;
    v->inv_w = inv_w;

    for (unsigned c = 0; c < 2; c++) {
        uint32_t offset = regs[0xe010 + c * 2];
        uint32_t scale = regs[0xe011 + c * 2];
        uint32_t ndc = w == 1.0f ? r->full[position + c] :
                       qcom8998_f_to_u32(
                           qcom8998_u32_to_f(r->full[position + c]) * inv_w);
        uint32_t coordinate;
        int fixed;

        if (!qcom8998_shader_finite(ndc) ||
            !qcom8998_shader_finite(offset) ||
            !qcom8998_shader_finite(scale)) {
            return false;
        }
        coordinate = float32_add(float32_mul(ndc, scale, &fp), offset, &fp);
        if (!qcom8998_shader_finite(coordinate) ||
            (coordinate & 0x7fffffff) > 0x46800000) {
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

    for (unsigned k = 1; k < out_count; k++) {
        uint32_t out_reg = regs[0xe593 + k / 2];
        unsigned regid = extract32(out_reg, (k & 1) ? 16 : 0, 8);
        unsigned mask = extract32(out_reg, (k & 1) ? 24 : 8, 4);
        unsigned loc = extract32(regs[0xe5a3 + k / 4], (k & 3) * 8, 8);

        for (unsigned b = 0; b < 4; b++) {
            if (mask & BIT(b)) {
                if (regid + b >= ADRENO_SHADER_GPRS || loc >= 64 ||
                    !qcom8998_bit_test(r->full_valid, regid + b)) {
                    return false;
                }
                v->varyings[loc] = r->full[regid + b];
                v->varying_mask |= BIT_ULL(loc);
                loc++;
            }
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

static float qcom8998_blend_factor(unsigned factor, unsigned c,
                                   const float src[4], const float dst[4],
                                   const float cst[4])
{
    switch (factor) {
    case 0: return 0.0f;
    case 1: return 1.0f;
    case 4: return src[c];
    case 5: return 1.0f - src[c];
    case 6: return src[3];
    case 7: return 1.0f - src[3];
    case 8: return dst[c];
    case 9: return 1.0f - dst[c];
    case 10: return dst[3];
    case 11: return 1.0f - dst[3];
    case 12: return cst[c];
    case 13: return 1.0f - cst[c];
    case 14: return cst[3];
    case 15: return 1.0f - cst[3];
    case 16: return c == 3 ? 1.0f : MIN(src[3], 1.0f - dst[3]);
    default: return 1.0f;
    }
}

static float qcom8998_blend_apply(unsigned op, float s_term, float d_term)
{
    switch (op) {
    case 0: return s_term + d_term;
    case 1: return s_term - d_term;
    case 2: return d_term - s_term;
    case 3: return MIN(s_term, d_term);
    case 4: return MAX(s_term, d_term);
    default: return s_term + d_term;
    }
}

static bool qcom8998_fragment_eval(Qcom8998Display *s,
                                   Qcom8998ShaderRegs *shader,
                                   unsigned fmt, float src[4])
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned output = regs[0xe5cb] & 0xff;
    bool half = !!(regs[0xe5cb] & BIT(8));

    if (output > ADRENO_SHADER_GPRS - 4 ||
        !qcom8998_fs_prefetch(s, shader) ||
        !qcom8998_shader_execute(s, 12, shader)) {
        trace_qcom8998_cp_draw_reject("fragment shader", shader->instruction[0],
                                      shader->instruction[1]);
        return false;
    }
    if (shader->killed) {
        return true;
    }
    for (unsigned c = 0; c < 4; c++) {
        uint32_t bits = 0;
        bool valid = false;

        if (half && qcom8998_bit_test(shader->half_valid, output + c)) {
            bits = qcom8998_f16_to_f32(shader->half[output + c]);
            valid = true;
        } else if (!half && qcom8998_bit_test(shader->full_valid, output + c)) {
            bits = shader->full[output + c];
            valid = true;
        } else if (qcom8998_bit_test(shader->full_valid, output + c)) {
            bits = shader->full[output + c];
            valid = true;
        } else if (qcom8998_bit_test(shader->half_valid, output + c)) {
            bits = qcom8998_f16_to_f32(shader->half[output + c]);
            valid = true;
        }
        if (!valid) {
            if (fmt == 0x30 || (fmt == 0x02 && c == 3) ||
                (fmt == 0x03 && c == 0) || (fmt == 0x0f && c < 2)) {
                return false;
            }
            bits = 0;
        }
        src[c] = qcom8998_u32_to_f(bits);
    }
    return true;
}

typedef struct Qcom8998BlendState {
    unsigned comp_mask;
    bool blend_rgb;
    bool blend_a;
    bool premul_over;
    unsigned sf_rgb, op_rgb, df_rgb;
    unsigned sf_a, op_a, df_a;
    float cst[4];
} Qcom8998BlendState;

static void qcom8998_blend_prepare(const uint32_t *regs, bool blend,
                                   Qcom8998BlendState *bs)
{
    uint32_t bc = regs[0xe151];

    bs->comp_mask = extract32(regs[0xe150], 7, 4);
    bs->blend_rgb = blend && !!(regs[0xe150] & BIT(0));
    bs->blend_a = blend && !!(regs[0xe150] & BIT(1));
    bs->sf_rgb = extract32(bc, 0, 5);
    bs->op_rgb = extract32(bc, 5, 3);
    bs->df_rgb = extract32(bc, 8, 5);
    bs->sf_a = extract32(bc, 16, 5);
    bs->op_a = extract32(bc, 21, 3);
    bs->df_a = extract32(bc, 24, 5);
    bs->premul_over = bs->blend_rgb && bs->blend_a &&
                      bs->sf_rgb == 1 && bs->op_rgb == 0 && bs->df_rgb == 7 &&
                      bs->sf_a == 1 && bs->op_a == 0 && bs->df_a == 7;
    if (bs->blend_rgb || bs->blend_a) {
        bs->cst[0] = qcom8998_u32_to_f(regs[0xe1a1]);
        bs->cst[1] = qcom8998_u32_to_f(regs[0xe1a3]);
        bs->cst[2] = qcom8998_u32_to_f(regs[0xe1a5]);
        bs->cst[3] = qcom8998_u32_to_f(regs[0xe1a7]);
    }
}

static inline bool qcom8998_fragment_blend(const Qcom8998BlendState *bs,
                                           const float src[4],
                                           uint32_t destination,
                                           uint32_t *color)
{
    unsigned c0, c1, c2, c3;

    if (!bs->blend_rgb && !bs->blend_a && bs->comp_mask == 0xf) {
        if (!qcom8998_unorm8(qcom8998_f_to_u32(src[0]), &c0) ||
            !qcom8998_unorm8(qcom8998_f_to_u32(src[1]), &c1) ||
            !qcom8998_unorm8(qcom8998_f_to_u32(src[2]), &c2) ||
            !qcom8998_unorm8(qcom8998_f_to_u32(src[3]), &c3)) {
            return false;
        }
        *color = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        return true;
    }
    if (bs->premul_over && bs->comp_mask == 0xf) {
        float inv_a = 1.0f - src[3];
        float v0 = src[0] + (float)(destination & 0xff) * (1.0f / 255.0f) * inv_a;
        float v1 = src[1] + (float)((destination >> 8) & 0xff) * (1.0f / 255.0f) * inv_a;
        float v2 = src[2] + (float)((destination >> 16) & 0xff) * (1.0f / 255.0f) * inv_a;
        float v3 = src[3] + (float)((destination >> 24) & 0xff) * (1.0f / 255.0f) * inv_a;

        if (!qcom8998_unorm8(qcom8998_f_to_u32(v0), &c0) ||
            !qcom8998_unorm8(qcom8998_f_to_u32(v1), &c1) ||
            !qcom8998_unorm8(qcom8998_f_to_u32(v2), &c2) ||
            !qcom8998_unorm8(qcom8998_f_to_u32(v3), &c3)) {
            return false;
        }
        *color = c0 | (c1 << 8) | (c2 << 16) | (c3 << 24);
        return true;
    }
    {
        float dst[4];

        if (bs->blend_rgb || bs->blend_a) {
            dst[0] = (float)(destination & 0xff) * (1.0f / 255.0f);
            dst[1] = (float)((destination >> 8) & 0xff) * (1.0f / 255.0f);
            dst[2] = (float)((destination >> 16) & 0xff) * (1.0f / 255.0f);
            dst[3] = (float)((destination >> 24) & 0xff) * (1.0f / 255.0f);
        }
        *color = 0;
        for (unsigned c = 0; c < 4; c++) {
            unsigned component;
            float value = src[c];

            if (!(bs->comp_mask & BIT(c))) {
                component = (destination >> (c * 8)) & 0xff;
                *color |= component << (c * 8);
                continue;
            }
            if (c < 3 ? bs->blend_rgb : bs->blend_a) {
                unsigned sf = c == 3 ? bs->sf_a : bs->sf_rgb;
                unsigned op = c == 3 ? bs->op_a : bs->op_rgb;
                unsigned df = c == 3 ? bs->df_a : bs->df_rgb;
                float f_src = qcom8998_blend_factor(sf, c, src, dst, bs->cst);
                float f_dst = qcom8998_blend_factor(df, c, src, dst, bs->cst);

                value = qcom8998_blend_apply(op, src[c] * f_src,
                                             dst[c] * f_dst);
            }
            if (!qcom8998_unorm8(qcom8998_f_to_u32(value), &component)) {
                return false;
            }
            *color |= component << (c * 8);
        }
        return true;
    }
}

static bool qcom8998_fragment_color(Qcom8998Display *s,
                                     Qcom8998ShaderRegs *shader,
                                     uint32_t destination,
                                     const Qcom8998BlendState *bs,
                                     unsigned fmt, uint32_t *color)
{
    float src[4] = { 0, 0, 0, 0 };

    if (!qcom8998_fragment_eval(s, shader, fmt, src)) {
        return false;
    }
    if (shader->killed) {
        return true;
    }
    return qcom8998_fragment_blend(bs, src, destination, color);
}

/* Cache only sampled pages (zero-copy direct RAM pointers), before publish. */
static inline const uint8_t *qcom8998_texture_page(Qcom8998Display *s,
                                                   Qcom8998PageCache *cache,
                                                   uint64_t address)
{
    uint64_t page = address & ~((1ULL << SMMU_4K_SHIFT) - 1);
    unsigned slot = (unsigned)(page >> SMMU_4K_SHIFT) & ADRENO_PAGE_CACHE_MASK;
    uint8_t *data;

    if (cache->pg[slot] == page) {
        return cache->data[slot];
    }
    gpointer key = (gpointer)(uintptr_t)(page >> SMMU_4K_SHIFT);

    data = g_hash_table_lookup(cache->table, key);
    if (!data) {
        hwaddr physical;

        if (!qcom8998_cp_translate(s, address, false, &physical)) {
            return NULL;
        }
        physical &= ~((1ULL << SMMU_4K_SHIFT) - 1);
        data = qcom8998_ram_ptr(physical, 1U << SMMU_4K_SHIFT, false);
        if (!data) {
            return NULL;
        }
        g_hash_table_insert(cache->table, key, data);
    }
    cache->pg[slot] = page;
    cache->data[slot] = data;
    return data;
}

static bool qcom8998_texture_word(Qcom8998Display *s, Qcom8998PageCache *cache,
                                   uint64_t address, unsigned cpp,
                                   uint32_t *value)
{
    const uint8_t *data = qcom8998_texture_page(s, cache, address);
    unsigned off = address & ((1U << SMMU_4K_SHIFT) - 1);

    if (!data) {
        return false;
    }
    *value = cpp == 1 ? data[off] : ldl_le_p(data + off);
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

typedef struct Qcom8998StagedPage {
    hwaddr physical;
    uint8_t data[1U << SMMU_4K_SHIFT];
} Qcom8998StagedPage;

static uint8_t *qcom8998_stage_write_page(Qcom8998Display *s,
                                          Qcom8998PageCache *cache,
                                          uint64_t address)
{
    uint64_t page = address & ~((1ULL << SMMU_4K_SHIFT) - 1);
    unsigned slot = (unsigned)(page >> SMMU_4K_SHIFT) & ADRENO_PAGE_CACHE_MASK;

    if (cache->pg[slot] == page) {
        return cache->data[slot];
    }
    gpointer key = (gpointer)(uintptr_t)(page >> SMMU_4K_SHIFT);
    Qcom8998StagedPage *entry = g_hash_table_lookup(cache->table, key);

    if (!entry) {
        hwaddr physical;
        const uint8_t *src;
        g_autofree Qcom8998StagedPage *pending = g_new(Qcom8998StagedPage, 1);

        if (!qcom8998_cp_translate(s, address, true, &physical)) {
            return NULL;
        }
        physical &= ~((1ULL << SMMU_4K_SHIFT) - 1);
        src = qcom8998_ram_ptr(physical, 1U << SMMU_4K_SHIFT, true);
        if (!src) {
            return NULL;
        }
        memcpy(pending->data, src, 1U << SMMU_4K_SHIFT);
        pending->physical = physical;
        entry = g_steal_pointer(&pending);
        g_hash_table_insert(cache->table, key, entry);
    }
    cache->pg[slot] = page;
    cache->data[slot] = entry->data;
    return entry->data;
}

static bool qcom8998_publish_write_pages(GHashTable *pages)
{
    GHashTableIter iter;
    gpointer key, value;

    g_hash_table_iter_init(&iter, pages);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        const Qcom8998StagedPage *entry = value;

        if (address_space_write(&address_space_memory, entry->physical,
                MEMTXATTRS_UNSPECIFIED, entry->data,
                1U << SMMU_4K_SHIFT) != MEMTX_OK) {
            return false;
        }
    }
    return true;
}

/*
 * A540 TILE5_3 (UBWC 1.0, 4-channel macrotile) block and pixel addressing,
 * matching Mesa freedreno's fd6_tiled_memcpy swizzle for cpp = 1 and cpp = 4.
 */
static uint64_t qcom8998_tile5_3_block_offset(unsigned x_block,
                                              unsigned y_block,
                                              uint64_t pitch,
                                              unsigned cpp,
                                              unsigned hbb)
{
    uint64_t macrotile_stride;
    unsigned bank_mask;

    qcom8998_tile5_3_params(pitch, cpp, hbb, &macrotile_stride, &bank_mask);
    return qcom8998_tile5_3_block_offset_fast(x_block, y_block,
                                              macrotile_stride, bank_mask, hbb);
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
    uint64_t address, pitch, layer, pixel_layer, dest;
    uint64_t flag_addr = 0, flag_pitch = 0, flag_size = 0;
    unsigned width, height, swap, dst_swap, cpp, x1, y1, x2, y2;
    unsigned tile = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 8, 2);
    bool r8 = regs[0xe5d3] == 3;
    bool ubwc = !!(regs[ADRENO_RB_RENDER_CNTL] & BIT(16));
    unsigned position = extract32(regs[0xe593], 16, 8);
    unsigned varying = regs[0xe593] & 0xff;
    unsigned *texel[2];
    float_status fp = { 0 };
    g_autoptr(GHashTable) pages = g_hash_table_new(g_direct_hash,
                                                   g_direct_equal);
    g_autoptr(GHashTable) write_pages = g_hash_table_new_full(g_direct_hash,
                                                              g_direct_equal,
                                                              NULL, g_free);
    Qcom8998PageCache tex_cache, write_cache;
    g_autoptr(GArray) spans = g_array_new(false, false,
                                        sizeof(Qcom8998GmemSpan));
    g_autofree uint32_t *pending = NULL;
    g_autofree uint8_t *raw_pixels = NULL;
    g_autofree unsigned *texel_x = NULL;
    g_autofree unsigned *texel_y = NULL;
    static const unsigned scissor[] = { 0xe0aa, 0xe0ca, 0xe0ea };
    static const unsigned disabled[] = {
        0xe001, 0xe002, 0xe003, 0xe090, 0xe0a2, 0xe0a3, 0xe0a4,
        0xe100, 0xe142, 0xe143, 0xe1c0, 0xe2a1,
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
        (regs[ADRENO_RB_RENDER_CNTL] & ~(BIT(8) | BIT(16))) ||
        (ubwc && (r8 || tile != 3)) ||
        regs[0xe000] != 0x181 || (regs[0xe0a0] & ~BIT(8)) ||
        (regs[0xe0a1] && regs[0xe0a1] != 0x20) ||
        (regs[0xe1a8] & BIT(8)) ||
        (regs[0xe1b1] & (BIT(0) | BIT(1) | BIT(6))) ||
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
    dst_swap = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 13, 2);
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
            if (!qcom8998_bit_test(vertex.full_valid, position + c)) {
                return false;
            }
        }
        if (vertex.full[position + 2] ||
            vertex.full[position + 3] != 0x3f800000) {
            return false;
        }
        for (unsigned c = 0; c < 2; c++) {
            if (!qcom8998_bit_test(vertex.full_valid, varying + c) ||
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
    pixel_layer = layer;
    if (ubwc) {
        uint64_t flag_array_pitch = (uint64_t)regs[0xe246] << 6;

        flag_addr = qcom8998_reg_pair(regs, 0xe243 * 4);
        flag_pitch = (uint64_t)regs[0xe245] << 6;
        if (flag_addr < dest && dest - flag_addr < layer) {
            flag_size = dest - flag_addr;
            pixel_layer = layer - flag_size;
        } else {
            flag_size = flag_array_pitch;
        }
        if ((flag_addr & 4095) || !flag_pitch || (flag_pitch & 63) ||
            flag_pitch < DIV_ROUND_UP(pitch / cpp, 16) ||
            !flag_size || flag_size > ADRENO_CLEAR_MAX_BYTES ||
            flag_addr > UINT64_MAX - flag_size ||
            ((uint64_t)((y2 - 1) / 4)) * flag_pitch +
            ((x2 - 1) / 16) >= flag_size) {
            trace_qcom8998_cp_draw_reject("ubwc flag layout", flag_pitch,
                                          flag_size);
            return false;
        }
    }
    if ((regs[ADRENO_RB_MRT_BUF_INFO] &
         ~(ADRENO_RB_COLOR_SWAP_MASK | 0x300U)) != (r8 ? 3 : 0x30) ||
        (r8 && dst_swap) ||
        (tile && tile != 3) ||
        regs[ADRENO_RB_RAS_MSAA_CNTL] || regs[ADRENO_RB_DEST_MSAA_CNTL] ||
        !pitch || !pixel_layer || pixel_layer % pitch ||
        pixel_layer > ADRENO_CLEAR_MAX_BYTES || x2 * cpp > pitch ||
        y2 > pixel_layer / pitch || (dest & 3) ||
        dest > UINT64_MAX - pixel_layer) {
        trace_qcom8998_cp_draw_reject("texture attachment",
            regs[ADRENO_RB_MRT_BUF_INFO], pixel_layer);
        return false;
    }
    if (tile == 3) {
        unsigned align_w = r8 ? 128 : 256;
        unsigned align_h = r8 ? 32 : 16;

        if ((dest & 4095) || (pitch % align_w) ||
            (pixel_layer % (pitch * align_h))) {
            trace_qcom8998_cp_draw_reject("tiled attachment", pitch,
                                          pixel_layer);
            return false;
        }
    }
    if (!tile) {
        if (r8 ? !qcom8998_texture_surface(s, dest, pitch, x1, y1, x2, y2, cpp,
                                            spans, &raw_pixels) :
                  !qcom8998_sysmem_surface(s, x1, y1, x2, y2, spans,
                                           &pending)) {
            return false;
        }
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
    qcom8998_page_cache_init(&tex_cache, pages);
    qcom8998_page_cache_init(&write_cache, write_pages);
    uint64_t src_pitch = extract32(texture[2], 7, 22);

    if (tile == 3) {
        unsigned hbb = 13 + extract32(regs[ADRENO_RB_MODE_CNTL], 1, 2);
        unsigned bw = r8 ? 32 : 16;
        unsigned bh = r8 ? 8 : 4;
        uint64_t dst_mt_stride;
        unsigned dst_bank_mask;

        qcom8998_tile5_3_params(pitch, cpp, hbb, &dst_mt_stride,
                                &dst_bank_mask);

        if (ubwc) {
            for (unsigned by = y1 / 4; by <= (y2 - 1) / 4; by++) {
                for (unsigned bx = x1 / 16; bx <= (x2 - 1) / 16; bx++) {
                    uint64_t faddr = flag_addr + (uint64_t)by * flag_pitch + bx;
                    uint8_t *fpage = qcom8998_stage_write_page(s, &write_cache,
                                                               faddr);
                    uint8_t old_flag;
                    bool full = bx * 16 >= x1 && (bx + 1) * 16 <= x2 &&
                                by * 4 >= y1 && (by + 1) * 4 <= y2;

                    if (!fpage) {
                        return false;
                    }
                    old_flag = fpage[faddr & 4095];
                    if (old_flag != 0x00 && old_flag != 0xff) {
                        trace_qcom8998_cp_draw_reject("ubwc flag", old_flag,
                                                      (by << 16) | bx);
                        return false;
                    }
                    if (!full && old_flag == 0x00) {
                        uint64_t baddr = dest +
                            qcom8998_tile5_3_block_offset_fast(bx, by,
                                dst_mt_stride, dst_bank_mask, hbb);
                        uint8_t *bpage = qcom8998_stage_write_page(
                            s, &write_cache, baddr);

                        if (!bpage) {
                            return false;
                        }
                        memset(bpage + (baddr & 4095), 0, 256);
                    }
                    fpage[faddr & 4095] = 0xff;
                }
            }
        }
        for (unsigned y = y1; y < y2; y++) {
            uint64_t src_row_addr = address + texel_y[y - y1] * src_pitch;
            unsigned yb = y / bh;
            unsigned ly = y % bh;

            for (unsigned x = x1; x < x2; x++) {
                uint32_t color;
                uint64_t paddr;
                uint8_t *ppage;

                if (!qcom8998_texture_word(s, &tex_cache,
                    src_row_addr + (uint64_t)texel_x[x - x1] * cpp,
                    cpp, &color)) {
                    return false;
                }
                paddr = dest +
                    qcom8998_tile5_3_block_offset_fast(x / bw, yb,
                        dst_mt_stride, dst_bank_mask, hbb) +
                    (uint64_t)qcom8998_tile5_3_pixel_index(x % bw, ly) * cpp;
                ppage = qcom8998_stage_write_page(s, &write_cache, paddr);
                if (!ppage) {
                    return false;
                }
                if (r8) {
                    ppage[paddr & 4095] = color;
                } else {
                    uint32_t out = swap == dst_swap ? color :
                        qcom8998_color_swap(
                            qcom8998_color_unswap(color, swap), dst_swap);

                    stl_le_p(ppage + (paddr & 4095), out);
                }
            }
        }
        if (!qcom8998_publish_write_pages(write_pages)) {
            return false;
        }
        s->gpu_dirty = true;
        trace_qcom8998_cp_texture_rectangle(x1, y1, x2 - x1, y2 - y1, address);
        return true;
    }
    bool unit_x = true;
    for (unsigned i = 1; i < x2 - x1; i++) {
        if (texel_x[i] != texel_x[i - 1] + 1) {
            unit_x = false;
            break;
        }
    }
    if (unit_x && !r8) {
        unsigned rw = x2 - x1;

        for (unsigned y = y1; y < y2; y++) {
            uint64_t src_addr = address + texel_y[y - y1] * src_pitch +
                                (uint64_t)texel_x[0] * 4;
            uint32_t *dst_row = pending + (size_t)(y - y1) * rw;
            unsigned rem = rw, dx = 0;

            while (rem) {
                const uint8_t *sp = qcom8998_texture_page(s, &tex_cache,
                                                          src_addr);
                unsigned off = src_addr & 4095;
                unsigned chunk = MIN(rem, (4096U - off) / 4);

                if (!sp) {
                    return false;
                }
                if (swap == 0) {
                    memcpy(dst_row + dx, sp + off, (size_t)chunk * 4);
                } else {
                    for (unsigned k = 0; k < chunk; k++) {
                        dst_row[dx + k] = qcom8998_color_unswap(
                            ldl_le_p(sp + off + k * 4), swap);
                    }
                }
                src_addr += (uint64_t)chunk * 4;
                dx += chunk;
                rem -= chunk;
            }
        }
    } else {
        for (unsigned y = y1; y < y2; y++) {
            uint64_t src_row_addr = address + texel_y[y - y1] * src_pitch;

            for (unsigned x = x1; x < x2; x++) {
                uint32_t color;

                if (!qcom8998_texture_word(s, &tex_cache,
                    src_row_addr + (uint64_t)texel_x[x - x1] * cpp,
                    cpp, &color)) {
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
    s->gpu_dirty = true;
    trace_qcom8998_cp_texture_rectangle(x1, y1, x2 - x1, y2 - y1, address);
    return true;
}

static bool qcom8998_cp_triangles(Qcom8998Display *s, unsigned count,
                                  const uint32_t *payload)
{
    const uint32_t *regs = s->bank[BANK_ADRENO].regs;
    unsigned base = 0, width, height, total = 0;
    unsigned prim = payload[0] & 0x3f;
    bool indexed = count == 7;
    unsigned idx_size = extract32(payload[0], 10, 2);
    unsigned idx_bytes = idx_size == 2 ? 4 : 2;
    unsigned fmt = regs[0xe5d3] & 0xff;
    unsigned cpp = fmt == 0x30 ? 4 : (fmt == 0x0f ? 2 : 1);
    unsigned tile = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 8, 2);
    unsigned dst_swap = extract32(regs[ADRENO_RB_MRT_BUF_INFO], 13, 2);
    bool ubwc = !!(regs[ADRENO_RB_RENDER_CNTL] & BIT(16));
    uint64_t indices = 0, end = 0;
    uint64_t pitch = 0, layer = 0, pixel_layer = 0, dest = 0;
    uint64_t flag_addr = 0, flag_pitch = 0, flag_size = 0;
    unsigned x1 = 0, y1 = 0, x2, y2;
    unsigned ox = regs[0xe1d0] & 0xffff, oy = regs[0xe1d0] >> 16;
    bool blend = !!(regs[0xe150] & 3);
    unsigned comp_mask = extract32(regs[0xe150], 7, 4);
    bool read_dst = blend || comp_mask != 0xf;
    bool sysmem = regs[ADRENO_RB_CNTL] == ADRENO_RB_BYPASS;
    bool tiled_sysmem = sysmem && tile == 3;
    bool restart_en = !!(regs[0xe384] & BIT(8));
    uint32_t restart_idx = idx_size == 2 ? regs[0xe38c] :
                                           (regs[0xe38c] & 0xffff);
    unsigned hbb = 13 + extract32(regs[ADRENO_RB_MODE_CNTL], 1, 2);
    uint64_t raster_budget = 64ULL << 20;
    uint64_t fragment_budget = 2ULL << 30;
    g_autoptr(GHashTable) tex_pages = g_hash_table_new(g_direct_hash,
                                                       g_direct_equal);
    g_autoptr(GHashTable) write_pages = g_hash_table_new_full(g_direct_hash,
                                                              g_direct_equal,
                                                              NULL, g_free);
    Qcom8998PageCache tex_cache, write_cache;
    Qcom8998ShaderProgram vs_prog, fs_prog;
    Qcom8998TexInfo fs_tex[32];
    Qcom8998BlendState blend_state, noblend_state;
    unsigned fs_tex_count = 0;
    uint64_t dst_mt_stride = 0;
    unsigned dst_bank_mask = 0;
    unsigned dst_bw = cpp == 1 ? 32 : 16;
    unsigned dst_bh = cpp == 1 ? 8 : 4;
    g_autofree uint8_t *pending = NULL;
    static const unsigned scissor[] = { 0xe0aa, 0xe0ca, 0xe0ea };

    qcom8998_page_cache_init(&tex_cache, tex_pages);
    qcom8998_page_cache_init(&write_cache, write_pages);

    bool binning_pass = s->vsc_binning ||
                        s->render_mode[0] == ADRENO_RENDER_BINNING ||
                        (regs[ADRENO_RB_RENDER_CNTL] & BIT(0)) ||
                        (regs[ADRENO_VPC_MODE_CNTL] & BIT(0));

    if (binning_pass) {
        /*
         * Hardware binning runs vertex-only visibility generation without
         * mutating color/GMEM/sysmem buffers. Rendering passes conservatively
         * treat every draw as visible and clip to each bin's scissor/offset.
         */
        return s->me_initialized && (count == 3 || count == 7);
    }

    if (!s->me_initialized || (count != 3 && count != 7) ||
        (prim != 4 && prim != 5 && prim != 6) ||
        (indexed ? ((payload[0] & ~0xf00U) != prim ||
                    (idx_size != 1 && idx_size != 2)) :
                   ((payload[0] & ~0xf00U) != (0x80U | prim))) ||
        ((payload[0] & 0x300) && !s->visibility_override && !s->bin_data_valid) ||
        !payload[1] || payload[1] > 4096 ||
        payload[2] < 3 || (prim == 4 && payload[2] % 3) ||
        payload[2] > 65536 ||
        regs[0xe001] || regs[0xe389] ||
        (regs[0xe0a0] & ~(BIT(3) | BIT(8))) ||
        (regs[0xe0a1] && regs[0xe0a1] != 0x20) ||
        regs[0xe0a2] || regs[0xe0a3] || (regs[0xe1a8] & BIT(8)) ||
        (regs[0xe1b1] & (BIT(0) | BIT(1) | BIT(6))) || regs[0xe1c0] ||
        regs[0xe100] || regs[0xe2a1] || regs[0xe385] ||
        (regs[0xe141] & ~(BIT(3) | BIT(8) | BIT(16))) ||
        (ubwc && (!tiled_sysmem || cpp != 4)) ||
        regs[0xe146] != 1 || regs[0xe147] != 0xf ||
        !(regs[0xe584] & 1) || !(regs[0xe585] & 1) ||
        !regs[0xe592] || regs[0xe592] > 32 ||
        (fmt != 0x30 && fmt != 0x02 && fmt != 0x03 && fmt != 0x0f) ||
        (regs[0xe5d3] != fmt) ||
        regs[0xe793] || regs[0xe794] || regs[0xe795] ||
        (regs[0xe5db] & ~0x1fU) || (regs[0xe5db] & 7) > 4 ||
        (regs[0xe384] & BIT(9)) || regs[0xe1b0] ||
        !(regs[0xe590] & BIT(2)) || !(regs[0xe5c0] & BIT(2)) ||
        /* QCDX uses both low-bit configurations 1 and 3 for the VS object. */
        (qcom8998_reg_pair(regs, 0xe5ac * 4) & ~2ULL) !=
            (s->shader_address[8] | 1) ||
        qcom8998_reg_pair(regs, 0xe5c3 * 4) != (s->shader_address[12] | 1)) {
        trace_qcom8998_cp_draw_reject("draw configuration", regs[0xe140],
                                      regs[0xe5d3]);
        return false;
    }
    if (sysmem) {
        pitch = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 1] << 6;
        layer = (uint64_t)regs[ADRENO_RB_MRT_BUF_INFO + 2] << 6;
        dest = qcom8998_reg_pair(regs, (ADRENO_RB_MRT_BUF_INFO + 3) * 4);
        pixel_layer = layer;
        if (ubwc) {
            uint64_t flag_array_pitch = (uint64_t)regs[0xe246] << 6;

            flag_addr = qcom8998_reg_pair(regs, 0xe243 * 4);
            flag_pitch = (uint64_t)regs[0xe245] << 6;
            if (flag_addr < dest && dest - flag_addr < layer) {
                flag_size = dest - flag_addr;
                pixel_layer = layer - flag_size;
            } else {
                flag_size = flag_array_pitch;
            }
            if ((flag_addr & 4095) || !flag_pitch || (flag_pitch & 63) ||
                flag_pitch < DIV_ROUND_UP(pitch / cpp, 16) ||
                !flag_size || flag_size > ADRENO_CLEAR_MAX_BYTES ||
                flag_addr > UINT64_MAX - flag_size) {
                trace_qcom8998_cp_draw_reject("tri ubwc flag", flag_pitch,
                                              flag_size);
                return false;
            }
        }
        if ((regs[ADRENO_RB_MRT_BUF_INFO] &
             ~(ADRENO_RB_COLOR_SWAP_MASK | 0x300U)) != fmt ||
            (cpp == 1 && dst_swap) || (tile && tile != 3) ||
            (tiled_sysmem && cpp != 1 && cpp != 4) ||
            regs[ADRENO_RB_RAS_MSAA_CNTL] ||
            regs[ADRENO_RB_DEST_MSAA_CNTL] || !pitch || !pixel_layer ||
            pitch / cpp > MDP_MAX_OUTPUT_SIZE || pixel_layer % pitch ||
            pixel_layer / pitch > MDP_MAX_OUTPUT_SIZE ||
            pixel_layer > ADRENO_CLEAR_MAX_BYTES || (dest & 3) ||
            dest > UINT64_MAX - pixel_layer) {
            trace_qcom8998_cp_draw_reject("linear layout", pitch, pixel_layer);
            return false;
        }
        if (tiled_sysmem) {
            unsigned align_w = cpp == 1 ? 128 : 256;
            unsigned align_h = cpp == 1 ? 32 : 16;

            if ((dest & 4095) || (pitch % align_w) ||
                (pixel_layer % (pitch * align_h))) {
                trace_qcom8998_cp_draw_reject("tiled layout", pitch,
                                              pixel_layer);
                return false;
            }
        }
        width = pitch / cpp;
        height = pixel_layer / pitch;
        ox = oy = base = 0; /* RB_WINDOW_OFFSET applies only to GMEM bins. */
    } else {
        unsigned gmem_fmt, gmem_cpp;

        height = extract32(regs[ADRENO_RB_CNTL], 9, 8) << 5;
        width = extract32(regs[ADRENO_RB_CNTL], 0, 8) << 5;
        if (!height || !width || ox + width > 0x10000 ||
            oy + height > 0x10000 ||
            !qcom8998_gmem_layout(s, 0, regs[0xe1d0],
            (ox + width - 1) | ((oy + height - 1) << 16), &base, &width,
            &gmem_fmt, &gmem_cpp) || gmem_fmt != fmt || gmem_cpp != cpp) {
            return false;
        }
    }
    x1 = ox;
    y1 = oy;
    x2 = ox + width;
    y2 = oy + height;
    for (unsigned c = 0; c < 2; c++) {
        float offset = qcom8998_u32_to_f(regs[0xe010 + c * 2]);
        float scale = fabsf(qcom8998_u32_to_f(regs[0xe011 + c * 2]));
        int vmin, vmax;

        if (!isfinite(offset) || !isfinite(scale) ||
            scale > MDP_MAX_OUTPUT_SIZE || fabsf(offset) > MDP_MAX_OUTPUT_SIZE) {
            return false;
        }
        vmin = MAX((int)floorf(offset - scale), 0);
        vmax = MAX((int)ceilf(offset + scale), 0);
        if (c) {
            y1 = MAX(y1, (unsigned)vmin);
            y2 = MIN(y2, (unsigned)vmax);
        } else {
            x1 = MAX(x1, (unsigned)vmin);
            x2 = MIN(x2, (unsigned)vmax);
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
    if (indexed) {
        uint64_t count_end = (uint64_t)payload[3] + payload[2];

        indices = deposit64(payload[4], 32, 32, payload[5]);
        end = count_end * idx_bytes;
        if ((indices & (idx_bytes - 1)) || count_end > payload[6] ||
            indices > UINT64_MAX - end) {
            trace_qcom8998_cp_draw_reject("index bounds", count_end,
                                          payload[6]);
            return false;
        }
        indices += (uint64_t)payload[3] * idx_bytes;
    }
    if (tiled_sysmem) {
        qcom8998_tile5_3_params(pitch, cpp, hbb, &dst_mt_stride,
                                &dst_bank_mask);
    }
    qcom8998_blend_prepare(regs, blend, &blend_state);
    qcom8998_blend_prepare(regs, false, &noblend_state);
    qcom8998_shader_prepare(s, 8, &vs_prog);
    qcom8998_shader_prepare(s, 12, &fs_prog);
    if (s->load_state[4][1]) {
        const GArray *textures = s->load_state[4][1];

        fs_tex_count = MIN(textures->len, (guint)ARRAY_SIZE(fs_tex));
        for (unsigned i = 0; i < fs_tex_count; i++) {
            const Qcom8998StateUnit *u =
                &g_array_index(textures, Qcom8998StateUnit, i);

            fs_tex[i].valid = false;
            if (u->valid) {
                qcom8998_tex_prepare(s, u->words, &fs_tex[i]);
            }
        }
    }
    if (!sysmem) {
        pending = g_memdup2((const uint8_t *)s->gmem + base,
                            (size_t)width * height * cpp);
    }
    for (unsigned inst = 0; inst < payload[1]; inst++) {
        unsigned strip_len = 0, strip_parity = 0;
        unsigned strip_idx[2] = { 0, 0 };
        unsigned fan_first = 0;
        unsigned vcount = payload[2];
        unsigned step = prim == 4 ? 3 : 1;

        for (unsigned pos = 0; pos < vcount; pos += step) {
            Qcom8998RasterVertex vertex[3];
            Qcom8998ShaderRegs shader;
            int64_t area;
            uint32_t color = 0;
            bool uniform = true;
            bool is_flat[64] = { 0 };
            bool constant[64] = { 0 };
            uint64_t vmask, nonconst_mask = 0;
            bool affine;
            int bx1, by1, bx2, by2;
            unsigned tri_idx[3];

            if (prim == 4) {
                bool skip = false;

                for (unsigned v = 0; v < 3; v++) {
                    if (indexed) {
                        uint64_t addr = indices + (uint64_t)(pos + v) * idx_bytes;
                        uint32_t packed;

                        if (!qcom8998_texture_word(s, &tex_cache, addr & ~3ULL,
                                                   4, &packed)) {
                            return false;
                        }
                        tri_idx[v] = idx_size == 2 ? packed :
                                     extract32(packed, (addr & 2) * 8, 16);
                        if (restart_en && tri_idx[v] == restart_idx) {
                            skip = true;
                        }
                    } else {
                        tri_idx[v] = pos + v;
                    }
                }
                if (skip) {
                    continue;
                }
            } else {
                unsigned cur_idx;

                if (indexed) {
                    uint64_t addr = indices + (uint64_t)pos * idx_bytes;
                    uint32_t packed;

                    if (!qcom8998_texture_word(s, &tex_cache, addr & ~3ULL,
                                               4, &packed)) {
                        return false;
                    }
                    cur_idx = idx_size == 2 ? packed :
                              extract32(packed, (addr & 2) * 8, 16);
                    if (restart_en && cur_idx == restart_idx) {
                        strip_len = 0;
                        strip_parity = 0;
                        continue;
                    }
                } else {
                    cur_idx = pos;
                }
                if (strip_len == 0) {
                    fan_first = cur_idx;
                    strip_idx[0] = cur_idx;
                    strip_len = 1;
                    continue;
                } else if (strip_len == 1) {
                    strip_idx[1] = cur_idx;
                    strip_len = 2;
                    continue;
                }
                if (prim == 5) {
                    tri_idx[0] = fan_first;
                    tri_idx[1] = strip_idx[1];
                    tri_idx[2] = cur_idx;
                    strip_idx[1] = cur_idx;
                } else {
                    if (strip_parity & 1) {
                        tri_idx[0] = strip_idx[0];
                        tri_idx[1] = cur_idx;
                        tri_idx[2] = strip_idx[1];
                    } else {
                        tri_idx[0] = strip_idx[0];
                        tri_idx[1] = strip_idx[1];
                        tri_idx[2] = cur_idx;
                    }
                    strip_idx[0] = strip_idx[1];
                    strip_idx[1] = cur_idx;
                    strip_parity++;
                }
            }

            for (unsigned v = 0; v < 3; v++) {
                shader.tex_pages = &tex_cache;
                shader.program = &vs_prog;
                shader.tex_info = NULL;
                shader.tex_info_count = 0;
                if (!qcom8998_vertex_fetch(s, tri_idx[v], inst, &shader) ||
                    !qcom8998_vertex_position(s, &shader, &vertex[v])) {
                    return false;
                }
            }
            area = qcom8998_edge(&vertex[0], &vertex[1],
                                 vertex[2].x, vertex[2].y);
            if (!area) {
                continue;
            }
            if (regs[0xe090] & 3) {
                bool is_cw = area > 0;
                bool is_front = (regs[0xe090] & BIT(2)) ? is_cw : !is_cw;

                if ((is_front && (regs[0xe090] & BIT(0))) ||
                    (!is_front && (regs[0xe090] & BIT(1)))) {
                    continue;
                }
            }
            if (x1 == x2 || y1 == y2) {
                continue;
            }
            if (area < 0) {
                Qcom8998RasterVertex swap = vertex[1];

                vertex[1] = vertex[2];
                vertex[2] = swap;
                area = -area;
            }
            vmask = vertex[0].varying_mask & vertex[1].varying_mask &
                    vertex[2].varying_mask;
            for (uint64_t m = vmask; m; m &= m - 1) {
                unsigned slot = ctz64(m);
                unsigned mode = extract32(regs[0xe282 + slot / 16],
                                          (slot % 16) * 2, 2);

                is_flat[slot] = (mode == 2);
                if (is_flat[slot]) {
                    constant[slot] = true;
                } else {
                    constant[slot] =
                        vertex[0].varyings[slot] == vertex[1].varyings[slot] &&
                        vertex[0].varyings[slot] == vertex[2].varyings[slot];
                    uniform &= constant[slot];
                    if (!constant[slot]) {
                        nonconst_mask |= BIT_ULL(slot);
                        for (unsigned v = 0; v < 3; v++) {
                            if (!qcom8998_shader_finite(
                                    vertex[v].varyings[slot])) {
                                return false;
                            }
                        }
                    }
                }
            }
            affine = vertex[0].inv_w == 1.0f && vertex[1].inv_w == 1.0f &&
                     vertex[2].inv_w == 1.0f;
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
                trace_qcom8998_cp_draw_reject("raster budget",
                                              bx2 - bx1, by2 - by1);
                return false;
            }
            raster_budget -= (uint64_t)(bx2 - bx1) * (by2 - by1);

            float uniform_src[4] = { 0, 0, 0, 0 };

            if (uniform) {
                qcom8998_shader_reset_pixel(&shader);
                shader.tex_pages = &tex_cache;
                shader.program = &fs_prog;
                shader.tex_info = fs_tex;
                shader.tex_info_count = fs_tex_count;
                for (uint64_t m = vmask; m; m &= m - 1) {
                    unsigned slot = ctz64(m);

                    shader.input[slot] = vertex[0].varyings[slot];
                    shader.flat_input[slot] = vertex[0].varyings[slot];
                }
                shader.input_valid = vmask;
                shader.flat_valid = vmask;
                if (!qcom8998_fragment_eval(s, &shader, fmt, uniform_src)) {
                    return false;
                }
                if (shader.killed) {
                    continue;
                }
                if (!read_dst &&
                    !qcom8998_fragment_blend(&noblend_state, uniform_src, 0,
                                             &color)) {
                    return false;
                }
            }

            float vvar[3][64];
            if (!uniform) {
                qcom8998_shader_reset_pixel(&shader);
                shader.tex_pages = &tex_cache;
                shader.program = &fs_prog;
                shader.tex_info = fs_tex;
                shader.tex_info_count = fs_tex_count;
                for (uint64_t m = vmask; m; m &= m - 1) {
                    unsigned slot = ctz64(m);

                    shader.flat_input[slot] = vertex[0].varyings[slot];
                    if (constant[slot]) {
                        shader.input[slot] = vertex[0].varyings[slot];
                    } else {
                        for (unsigned v = 0; v < 3; v++) {
                            vvar[v][slot] =
                                qcom8998_u32_to_f(vertex[v].varyings[slot]);
                        }
                    }
                }
            }

            double inv_area = 1.0 / (double)area;
            int64_t edge_row[3], step_x[3], step_y[3], bias[3];

            for (unsigned e = 0; e < 3; e++) {
                unsigned next = (e + 1) % 3;

                edge_row[e] = qcom8998_edge(&vertex[e], &vertex[next],
                                            bx1 * 256 + 128, by1 * 256 + 128);
                step_x[e] = -(int64_t)(vertex[next].y - vertex[e].y) * 256;
                step_y[e] = (int64_t)(vertex[next].x - vertex[e].x) * 256;
                bias[e] = qcom8998_top_left(&vertex[e], &vertex[next]) ? 0 : -1;
            }

            for (unsigned y = by1; y < by2; y++,
                 edge_row[0] += step_y[0], edge_row[1] += step_y[1],
                 edge_row[2] += step_y[2]) {
                int64_t k_lo = 0;
                int64_t k_hi = (int64_t)(bx2 - bx1);

                for (unsigned e = 0; e < 3; e++) {
                    int64_t v0 = edge_row[e] + bias[e];
                    int64_t dx = step_x[e];

                    if (dx == 0) {
                        if (v0 < 0) {
                            k_lo = k_hi;
                            break;
                        }
                    } else if (dx > 0) {
                        if (v0 < 0) {
                            int64_t need = (-v0 - 1) / dx + 1;

                            if (need > k_lo) {
                                k_lo = need;
                            }
                        }
                    } else {
                        if (v0 < 0) {
                            k_lo = k_hi;
                            break;
                        } else {
                            int64_t limit = v0 / (-dx) + 1;

                            if (limit < k_hi) {
                                k_hi = limit;
                            }
                        }
                    }
                }
                if (k_lo >= k_hi) {
                    continue;
                }
                unsigned rx1 = bx1 + (unsigned)k_lo;
                unsigned rx2 = bx1 + (unsigned)k_hi;
                int64_t edges[3] = {
                    edge_row[0] + k_lo * step_x[0],
                    edge_row[1] + k_lo * step_x[1],
                    edge_row[2] + k_lo * step_x[2],
                };

                for (unsigned x = rx1; x < rx2; x++,
                     edges[0] += step_x[0], edges[1] += step_x[1],
                     edges[2] += step_x[2]) {
                    uint32_t dst_pixel = 0;
                    uint64_t paddr = 0;
                    uint8_t *ppage = NULL;
                    if (sysmem) {
                        if (tiled_sysmem) {
                            uint64_t boff = qcom8998_tile5_3_block_offset_fast(
                                x / dst_bw, y / dst_bh, dst_mt_stride,
                                dst_bank_mask, hbb);

                            if (ubwc) {
                                uint64_t foff = (uint64_t)(y / 4) * flag_pitch +
                                                (x / 16);
                                uint8_t *fpage;
                                uint8_t old_flag;

                                if (foff >= flag_size) {
                                    return false;
                                }
                                fpage = qcom8998_stage_write_page(s,
                                    &write_cache, flag_addr + foff);
                                if (!fpage) {
                                    return false;
                                }
                                old_flag = fpage[(flag_addr + foff) & 4095];
                                if (old_flag != 0x00 && old_flag != 0xff) {
                                    trace_qcom8998_cp_draw_reject(
                                        "tri ubwc flag", old_flag,
                                        ((y / 4) << 16) | (x / 16));
                                    return false;
                                }
                                if (old_flag == 0x00) {
                                    uint8_t *bpage = qcom8998_stage_write_page(
                                        s, &write_cache, dest + boff);

                                    if (!bpage) {
                                        return false;
                                    }
                                    memset(bpage + ((dest + boff) & 4095), 0,
                                           256);
                                    fpage[(flag_addr + foff) & 4095] = 0xff;
                                }
                            }
                            paddr = dest + boff +
                                (uint64_t)qcom8998_tile5_3_pixel_index(
                                    x % dst_bw, y % dst_bh) * cpp;
                        } else {
                            paddr = dest + (uint64_t)y * pitch +
                                    (uint64_t)x * cpp;
                        }
                        ppage = qcom8998_stage_write_page(s, &write_cache,
                                                          paddr);
                        if (!ppage) {
                            return false;
                        }
                        if (read_dst) {
                            if (cpp == 1) {
                                uint8_t pv = ppage[paddr & 4095];
                                dst_pixel = fmt == 2 ? ((uint32_t)pv << 24) :
                                                       pv;
                            } else if (cpp == 2) {
                                dst_pixel = lduw_le_p(ppage + (paddr & 4095));
                            } else {
                                dst_pixel = qcom8998_color_unswap(
                                    ldl_le_p(ppage + (paddr & 4095)), dst_swap);
                            }
                        }
                    } else if (read_dst) {
                        unsigned idx = (y - oy) * width + x - ox;

                        if (cpp == 1) {
                            uint8_t pv = pending[idx];
                            dst_pixel = fmt == 2 ? ((uint32_t)pv << 24) : pv;
                        } else if (cpp == 2) {
                            dst_pixel = ((const uint16_t *)pending)[idx];
                        } else {
                            dst_pixel = ((const uint32_t *)pending)[idx];
                        }
                    }

                    if (uniform) {
                        if (read_dst &&
                            !qcom8998_fragment_blend(&blend_state, uniform_src,
                                                     dst_pixel, &color)) {
                            return false;
                        }
                    } else {
                        float fw[3];
                        uint64_t work;
                        int64_t opp[3] = { edges[1], edges[2], edges[0] };

                        qcom8998_shader_reset_pixel(&shader);
                        if (affine) {
                            fw[0] = (float)((double)opp[0] * inv_area);
                            fw[1] = (float)((double)opp[1] * inv_area);
                            fw[2] = (float)((double)opp[2] * inv_area);
                        } else {
                            double pw0 = (double)opp[0] *
                                         (double)vertex[0].inv_w;
                            double pw1 = (double)opp[1] *
                                         (double)vertex[1].inv_w;
                            double pw2 = (double)opp[2] *
                                         (double)vertex[2].inv_w;
                            double wsum = pw0 + pw1 + pw2;
                            double inv_wsum = wsum != 0.0 ? (1.0 / wsum) : 1.0;

                            fw[0] = (float)(pw0 * inv_wsum);
                            fw[1] = (float)(pw1 * inv_wsum);
                            fw[2] = (float)(pw2 * inv_wsum);
                        }
                        for (uint64_t m = nonconst_mask; m; m &= m - 1) {
                            unsigned slot = ctz64(m);
                            float fval = vvar[0][slot] * fw[0] +
                                         vvar[1][slot] * fw[1] +
                                         vvar[2][slot] * fw[2];

                            shader.input[slot] = qcom8998_f_to_u32(fval);
                        }
                        shader.input_valid = vmask;
                        shader.flat_valid = vmask;
                        if (!qcom8998_fragment_color(s, &shader, dst_pixel,
                                                     &blend_state, fmt,
                                                     &color)) {
                            return false;
                        }
                        work = shader.steps + 16;
                        if (work > fragment_budget) {
                            trace_qcom8998_cp_draw_reject("fragment budget",
                                                          total, work);
                            return false;
                        }
                        fragment_budget -= work;
                        if (shader.killed) {
                            continue;
                        }
                    }

                    if (sysmem) {
                        if (cpp == 1) {
                            ppage[paddr & 4095] =
                                fmt == 2 ? (color >> 24) : color;
                        } else if (cpp == 2) {
                            stw_le_p(ppage + (paddr & 4095), color & 0xffff);
                        } else {
                            stl_le_p(ppage + (paddr & 4095),
                                     qcom8998_color_swap(color, dst_swap));
                        }
                    } else {
                        unsigned idx = (y - oy) * width + x - ox;

                        if (cpp == 1) {
                            pending[idx] = fmt == 2 ? (color >> 24) : color;
                        } else if (cpp == 2) {
                            ((uint16_t *)pending)[idx] = color & 0xffff;
                        } else {
                            ((uint32_t *)pending)[idx] = color;
                        }
                    }
                    total++;
                }
            }
        }
    }
    if (sysmem) {
        if (!qcom8998_publish_write_pages(write_pages)) {
            return false;
        }
        s->gpu_dirty = true;
    } else {
        memcpy((uint8_t *)s->gmem + base, pending,
               (size_t)width * height * cpp);
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
    case ADRENO_EVENT_VSC_BINNING_START:
        if (count != 1) {
            return false;
        }
        s->vsc_binning = true;
        return true;
    case ADRENO_EVENT_VSC_BINNING_END:
        if (count != 1) {
            return false;
        }
        s->vsc_binning = false;
        return true;
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
    uint32_t payload[8];
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
        s->skip_ib2_global && s->skip_ib2_local && !s->visibility_override &&
        !s->bin_data_valid) {
        return false; /* Consuming an unbound visibility stream is unsupported. */
    }
    if (opcode == ADRENO_PM4_DRAW_INDX_OFFSET && s->me_initialized &&
        (count == 3 || count == 7) && qcom8998_cp_dispatch_draw_state(s)) {
        return true; /* The draw stays pending until all dirty groups return. */
    }
    if (opcode == ADRENO_PM4_NOP) {
        return true;
    }
    if (opcode == ADRENO_PM4_WAIT_MEM_WRITES || opcode == ADRENO_PM4_WAIT_FOR_ME) {
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
        {
            int64_t start = g_get_monotonic_time();
            bool result;
            uint32_t control = count ? payload[0] : 0;
            unsigned vertices = count > 2 ? payload[2] : 0;

            trace_qcom8998_cp_draw_time(true, control, vertices, 0);
            result = ((control & 0x3f) == 4 ||
                      (control & 0x3f) == 6) ?
                     qcom8998_cp_triangles(s, count, payload) :
                     qcom8998_cp_rectangle(s, count, payload);
            trace_qcom8998_cp_draw_time(false, control, vertices,
                                        g_get_monotonic_time() - start);
            return result;
        }
    case ADRENO_PM4_WAIT_REG_MEM:
        return qcom8998_cp_wait(s, count, payload);
    case ADRENO_PM4_REG_TEST:
        {
            unsigned reg, bit;
            uint32_t value;

            /* A5xx's one-word register test, with optional implicit WFM. */
            if (!s->me_initialized || count != 1 ||
                (payload[0] & ~(0x3ffffU | (31U << 20) | BIT(25)))) {
                return false;
            }
            reg = extract32(payload[0], 0, 18);
            bit = extract32(payload[0], 20, 5);
            if (reg >= ADRENO_VSC_BIN_VISIBILITY &&
                reg <= ADRENO_VSC_BIN_VISIBILITY_END) {
                /*
                 * Match the conservative visibility policy of the rasterizer:
                 * a bound bin executes its restore, draw and resolve streams.
                 * No visibility-stream contents or occlusion are synthesized.
                 */
                value = s->bin_data_valid ? UINT32_MAX : 0;
            } else if (qcom8998_cp_stored_register(reg)) {
                value = s->bank[BANK_ADRENO].regs[reg];
            } else {
                return false;
            }
            s->register_predicate = !!(value & BIT(bit));
            trace_qcom8998_cp_reg_test(reg, bit, value, s->register_predicate);
            return true;
        }
    case ADRENO_PM4_COND_REG_EXEC:
        /* Only the A5xx predicate-test form is implemented. */
        if (!s->me_initialized || count != 2 || payload[0] != 0x10000000 ||
            (payload[1] & ~0xffffffU) || payload[1] > available - count - 1) {
            return false;
        }
        if (!s->register_predicate) {
            *skip = payload[1];
        }
        return true;
    case ADRENO_PM4_SET_BIN_DATA5:
        if (!s->me_initialized || count != 5) {
            return false;
        }
        memcpy(s->bin_data, payload, sizeof(s->bin_data));
        s->bin_data_valid = true;
        return true;
    case ADRENO_PM4_SET_RENDER_MODE:
        /*
         * Accept sysmem bypass, hardware visibility binning, and GMEM tile
         * rendering modes (1-, 5-, or 8-word forms without preemption).
         * A bare 1-word GMEM selector without configured VSC_BIN_SIZE stays
         * rejected so unconfigured GMEM mode transitions remain pending.
         */
        if (!s->me_initialized ||
            (count != 1 && count != 5 && count != 8) ||
            (payload[0] != ADRENO_RENDER_BYPASS &&
             payload[0] != ADRENO_RENDER_BINNING &&
             payload[0] != ADRENO_RENDER_GMEM) ||
            (payload[0] == ADRENO_RENDER_BYPASS && count == 5 &&
             (payload[1] || payload[2] || payload[3] || payload[4] != 1)) ||
            (payload[0] == ADRENO_RENDER_GMEM && count == 1 &&
             !s->bank[BANK_ADRENO].regs[0x0bc2])) {
            return false;
        }
        memset(s->render_mode, 0, sizeof(s->render_mode));
        memcpy(s->render_mode, payload, count * sizeof(payload[0]));
        if (payload[0] != ADRENO_RENDER_GMEM) {
            s->bin_data_valid = false;
        }
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
    int64_t dispatch_start = g_get_monotonic_time();
    bool drew = false;

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
        bool draw_packet;

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
        draw_packet = (header & ADRENO_PM4_TYPE_MASK) == ADRENO_PM4_TYPE7 &&
                      extract32(header, 16, 7) == ADRENO_PM4_DRAW_INDX_OFFSET;
        /*
         * Software rasterization can dwarf the packet/word budgets. Give
         * CPUs and the display loop a turn before another costly draw, while
         * still allowing trailing fences and events to publish real work.
         */
        if (draw_packet && drew &&
            g_get_monotonic_time() - dispatch_start >= 2000) {
            trace_qcom8998_cp_yield(g_get_monotonic_time() - dispatch_start);
            goto defer;
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
        drew |= draw_packet;
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

    if (bank->index == BANK_ADRENO && offset == ADRENO_RBBM_STATUS) {
        Qcom8998Display *s = bank->display;
        bool pending = s->stream_count &&
            (s->stream_count > 1 ||
             bank->regs[ADRENO_CP_RB_RPTR / 4] !=
             bank->regs[ADRENO_CP_RB_WPTR / 4]);

        /* Deferred commands and memory waits have not retired yet. */
        value = pending && !bank->regs[ADRENO_CP_PFP_ME_CNTL / 4] ?
            ADRENO_RBBM_STATUS_GPU_BUSY | ADRENO_RBBM_STATUS_CP_BUSY |
            ADRENO_RBBM_STATUS_CP_BUSY_IGN_HYST : 0;
    }
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
          offset == ADRENO_RBBM_STATUS ||
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
    if (s->merged_panel) {
        qemu_graphic_console_close(s->merged_panel);
        s->merged_panel = NULL;
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
    s->register_predicate = false;
    s->register_protection = true;
    s->draw_predicate_global = false;
    s->draw_predicate_local = false;
    s->draw_predicate_pass = true;
    s->visibility_override = false;
    s->skip_ib2_global = false;
    s->skip_ib2_local = false;
    memset(s->autorefresh, 0, sizeof(s->autorefresh));
    memset(s->refresh_tick, 0, sizeof(s->refresh_tick));
    for (unsigned i = 0; i < MDSS_DSI_COUNT; i++) {
        s->panel_needs_full_update[i] = true;
    }
    s->merged_needs_full_update = true;
    s->gpu_dirty = false;
    memset(s->gmem, 0, ADRENO_GMEM_SIZE);
    memset(s->draw_state, 0, sizeof(s->draw_state));
    memset(s->render_mode, 0, sizeof(s->render_mode));
    s->vsc_binning = false;
    s->bin_data_valid = false;
    memset(s->bin_data, 0, sizeof(s->bin_data));
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
    for (unsigned i = 0; i < MDSS_DSI_COUNT; i++) {
        g_free(s->panel_scratch[i]);
        s->panel_scratch[i] = NULL;
        s->panel_scratch_pixels[i] = 0;
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
    DEFINE_PROP_BOOL("merged-view", Qcom8998Display, merged_view, true),
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

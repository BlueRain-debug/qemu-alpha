/*
 * Experimental SDM845 Adreno 630 direct-render IR3 shader and software
 * rasterization backend for local OEMSK850 bring-up.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Strictly independent from MSM8998 (qcom-8998-display.c).
 */
#include "qemu/osdep.h"
#include <math.h>
#include "fpu/softfloat.h"
#include "hw/display/qcom-845-display-internal.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "trace.h"

/* Adreno 630 context register dword indices (verified against live 17047 + a6xx.xml). */
#define A6XX_GRAS_CL_CNTL                0x8000
#define A6XX_GRAS_CL_VPORT_XOFFSET_0     0x8010
#define A6XX_GRAS_SU_CNTL                0x8090
#define A6XX_GRAS_SC_SCREEN_SCISSOR_TL_0 0x80b0
#define A6XX_GRAS_SC_VPORT_SCISSOR_TL_0  0x80d0
#define A6XX_GRAS_SC_WINDOW_SCISSOR_TL   0x80f0

#define A6XX_RB_MRT_0_CONTROL            0x8820
#define A6XX_RB_MRT_0_BLEND_CONTROL      0x8821
#define A6XX_RB_MRT_0_BUF_INFO           0x8822
#define A6XX_RB_MRT_0_PITCH              0x8823
#define A6XX_RB_MRT_0_ARRAY_PITCH        0x8824
#define A6XX_RB_MRT_0_BASE_LO            0x8825
#define A6XX_RB_MRT_0_BASE_HI            0x8826
#define A6XX_RB_BLEND_CONSTANT_RED_FP32  0x8860
#define A6XX_RB_RESOLVE_SYSTEM_BUFFER_INFO        0x88d7
#define A6XX_RB_RESOLVE_SYSTEM_BUFFER_BASE_LO     0x88d8
#define A6XX_RB_RESOLVE_SYSTEM_BUFFER_BASE_HI     0x88d9
#define A6XX_RB_RESOLVE_SYSTEM_BUFFER_PITCH       0x88da
#define A6XX_RB_RESOLVE_SYSTEM_BUFFER_ARRAY_PITCH 0x88db

#define A6XX_VPC_VARYING_INTERP_MODE_0   0x9200
#define A6XX_PC_RESTART_INDEX            0x9803
#define A6XX_PC_PRIM_VTX_CNTL            0x9804
#define A6XX_PC_PRIMITIVE_CNTL_0         0x9b00

#define A6XX_VFD_CONTROL_0               0xa000
#define A6XX_VFD_CONTROL_1               0xa001
#define A6XX_VFD_INDEX_OFFSET            0xa00e
#define A6XX_VFD_INSTANCE_START_OFFSET   0xa00f
#define A6XX_VFD_FETCH_0_BASE_LO         0xa010
#define A6XX_VFD_DECODE_0_INSTR          0xa090
#define A6XX_VFD_DEST_CNTL_0_INSTR       0xa0d0

#define A6XX_SP_VS_OUT_CNT               0xa802
#define A6XX_SP_VS_OUT_REG_0             0xa803
#define A6XX_SP_VS_VPC_DST_REG_0         0xa813
#define A6XX_SP_VS_OBJ_START_LO          0xa81c
#define A6XX_SP_FS_OBJ_START_LO          0xa983
#define A6XX_SP_FS_OUTPUT_CNTL1          0xa98d
#define A6XX_SP_FS_OUTPUT_REG_0          0xa98e
#define A6XX_SP_FS_MRT_REG_0             0xa996
#define A6XX_SP_FS_PREFETCH_CNTL         0xa99e
#define A6XX_SP_FS_PREFETCH_CMD_0        0xa99f
#define A6XX_SP_PS_SAMPLER_BASE_LO       0xa9e0
#define A6XX_SP_PS_TEXMEMOBJ_BASE_LO     0xa9e4
#define A6XX_SP_GFX_BINDLESS_BASE_0      0xab10
#define A6XX_HLSQ_BINDLESS_BASE_0        0xbb20

/* Software safety bounds. */
#define A6XX_MAX_OUTPUT_DIM     4096U
#define A6XX_MAX_SURFACE_BYTES  (64U << 20)
#define A6XX_SHADER_GPRS        256U
#define A6XX_SHADER_MAX_INSNS   512U
#define A6XX_SHADER_MAX_CONSTS  2048U
#define A6XX_SHADER_STEPS       4096U
#define A6XX_PAGE_SHIFT         12U
#define A6XX_PAGE_SIZE          (1U << A6XX_PAGE_SHIFT)
#define A6XX_PAGE_MASK          (A6XX_PAGE_SIZE - 1U)
#define A6XX_PAGE_CACHE_SLOTS   256U
#define A6XX_PAGE_CACHE_MASK    (A6XX_PAGE_CACHE_SLOTS - 1U)

typedef struct Qcom845PageCache {
    GHashTable *table;
    uint64_t pg[A6XX_PAGE_CACHE_SLOTS];
    uint8_t *data[A6XX_PAGE_CACHE_SLOTS];
} Qcom845PageCache;

typedef struct Qcom845StagedPage {
    hwaddr physical;
    uint8_t data[A6XX_PAGE_SIZE];
} Qcom845StagedPage;

typedef struct Qcom845ShaderProgram {
    uint64_t insns[A6XX_SHADER_MAX_INSNS];
    uint32_t insn_count;
    uint32_t consts[A6XX_SHADER_MAX_CONSTS];
    uint64_t const_valid[A6XX_SHADER_MAX_CONSTS / 64];
    uint32_t const_count;
} Qcom845ShaderProgram;

typedef struct Qcom845TexInfo {
    bool valid;
    bool srgb;
    bool identity_swizzle;
    unsigned tile;
    unsigned fmt;
    unsigned swap;
    unsigned tex_type;
    unsigned cpp;
    unsigned width;
    unsigned height;
    uint32_t buf_elems;
    uint64_t pitch;
    uint64_t base;
    unsigned swizzle[4];
} Qcom845TexInfo;

typedef struct Qcom845ShaderRegs {
    uint32_t full[A6XX_SHADER_GPRS];
    uint16_t half[A6XX_SHADER_GPRS];
    uint64_t full_valid[A6XX_SHADER_GPRS / 64];
    uint64_t half_valid[A6XX_SHADER_GPRS / 64];
    uint8_t pred[4];
    uint8_t pred_valid;
    int16_t a0;
    int16_t a1;
    bool a0_valid;
    bool a1_valid;
    bool killed;
    uint32_t input[64];
    uint32_t flat_input[64];
    uint64_t input_valid;
    uint64_t flat_valid;
    Qcom845PageCache *read_cache;
    Qcom845ShaderProgram *program;
    uint32_t last_insn[2];
} Qcom845ShaderRegs;

typedef struct Qcom845RasterVertex {
    int x; /* 24.8 fixed point */
    int y;
    float inv_w;
    bool clipped;
    uint32_t varyings[64];
    uint64_t varying_mask;
} Qcom845RasterVertex;

typedef struct Qcom845BlendState {
    bool blend_color;
    bool blend_alpha;
    unsigned comp_mask;
    unsigned rgb_src;
    unsigned rgb_op;
    unsigned rgb_dst;
    unsigned a_src;
    unsigned a_op;
    unsigned a_dst;
    float cst[4];
} Qcom845BlendState;

static inline void qcom845_bit_set(uint64_t *bits, unsigned idx)
{
    bits[idx >> 6] |= BIT_ULL(idx & 63);
}

static inline bool qcom845_bit_test(const uint64_t *bits, unsigned idx)
{
    return !!(bits[idx >> 6] & BIT_ULL(idx & 63));
}

static inline float qcom845_u32_to_f(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static inline uint32_t qcom845_f_to_u32(float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    return bits;
}

static inline bool qcom845_f32_finite(uint32_t bits)
{
    return extract32(bits, 23, 8) != 255;
}

static uint32_t qcom845_sat_f32(uint32_t bits)
{
    float f = qcom845_u32_to_f(bits);
    if (isnan(f) || f <= 0.0f) {
        return 0;
    }
    if (f >= 1.0f) {
        return 0x3f800000;
    }
    return bits;
}

static uint32_t qcom845_f16_to_f32(uint16_t h)
{
    float_status fp = { 0 };
    return float16_to_float32(h, true, &fp);
}

static uint16_t qcom845_f32_to_f16(uint32_t f)
{
    float_status fp = { 0 };
    return float32_to_float16(f, true, &fp);
}

static uint32_t qcom845_color_swap(uint32_t color, unsigned swap)
{
    static const unsigned order[4][4] = {
        { 0, 1, 2, 3 }, { 2, 1, 0, 3 },
        { 3, 0, 1, 2 }, { 3, 2, 1, 0 },
    };
    uint32_t packed = 0;

    swap &= 3;
    for (unsigned c = 0; c < 4; c++) {
        packed |= extract32(color, order[swap][c] * 8, 8) << (c * 8);
    }
    return packed;
}

static uint32_t qcom845_color_unswap(uint32_t color, unsigned swap)
{
    static const unsigned order[4][4] = {
        { 0, 1, 2, 3 }, { 2, 1, 0, 3 },
        { 3, 0, 1, 2 }, { 3, 2, 1, 0 },
    };
    uint32_t result = 0;

    swap &= 3;
    for (unsigned c = 0; c < 4; c++) {
        result |= extract32(color, c * 8, 8) << (order[swap][c] * 8);
    }
    return result;
}

static const float *qcom845_srgb_lut(void)
{
    static float lut[256];
    static bool initialized;

    if (!initialized) {
        for (unsigned i = 0; i < 256; i++) {
            float v = (float)i / 255.0f;
            lut[i] = (v <= 0.04045f) ? (v / 12.92f)
                                     : powf((v + 0.055f) / 1.055f, 2.4f);
        }
        initialized = true;
    }
    return lut;
}

static inline uint64_t qcom845_reg64(const uint32_t *regs, unsigned idx)
{
    return (uint64_t)regs[idx] | ((uint64_t)regs[idx + 1] << 32);
}

/* ---- Fast 4 KiB SMMU page cache ---- */

static void qcom845_page_cache_init(Qcom845PageCache *c, GHashTable *table)
{
    c->table = table;
    for (unsigned i = 0; i < A6XX_PAGE_CACHE_SLOTS; i++) {
        c->pg[i] = UINT64_MAX;
        c->data[i] = NULL;
    }
}

static uint8_t *qcom845_ram_page_ptr(hwaddr pa, bool write)
{
    hwaddr xlat = 0;
    hwaddr len = A6XX_PAGE_SIZE;
    MemoryRegion *mr = address_space_translate(&address_space_memory, pa,
                                               &xlat, &len, write,
                                               MEMTXATTRS_UNSPECIFIED);

    if (!mr || !memory_region_is_ram(mr) || len < A6XX_PAGE_SIZE) {
        return NULL;
    }
    if (write && memory_region_is_romd(mr)) {
        return NULL;
    }
    return (uint8_t *)memory_region_get_ram_ptr(mr) + xlat;
}

static const uint8_t *qcom845_read_page(Qcom845Display *s,
                                        Qcom845PageCache *cache,
                                        uint64_t iova)
{
    uint64_t page = iova & ~(uint64_t)A6XX_PAGE_MASK;
    unsigned slot = (unsigned)(page >> A6XX_PAGE_SHIFT) & A6XX_PAGE_CACHE_MASK;

    if (cache->pg[slot] == page) {
        return cache->data[slot];
    }
    gpointer key = (gpointer)(uintptr_t)(page >> A6XX_PAGE_SHIFT);
    uint8_t *data = g_hash_table_lookup(cache->table, key);

    if (!data) {
        hwaddr pa = page;
        if (s->gpu_smmu &&
            !qcom845_smmu_translate(s->gpu_smmu, 0, page, false, &pa)) {
            return NULL;
        }
        pa &= ~(hwaddr)A6XX_PAGE_MASK;
        data = qcom845_ram_page_ptr(pa, false);
        if (!data) {
            return NULL;
        }
        g_hash_table_insert(cache->table, key, data);
    }
    cache->pg[slot] = page;
    cache->data[slot] = data;
    return data;
}

static bool qcom845_cached_read(Qcom845Display *s, Qcom845PageCache *cache,
                                uint64_t iova, void *buf, size_t size)
{
    uint8_t *dst = buf;

    while (size) {
        unsigned off = iova & A6XX_PAGE_MASK;
        size_t chunk = MIN(size, A6XX_PAGE_SIZE - off);

        if (cache) {
            const uint8_t *pg = qcom845_read_page(s, cache, iova);
            if (!pg) {
                return false;
            }
            memcpy(dst, pg + off, chunk);
        } else {
            hwaddr pa = iova;
            if (s->gpu_smmu &&
                !qcom845_smmu_translate(s->gpu_smmu, 0, iova, false, &pa)) {
                return false;
            }
            if (address_space_read(&address_space_memory, pa,
                                   MEMTXATTRS_UNSPECIFIED, dst,
                                   chunk) != MEMTX_OK) {
                return false;
            }
        }
        iova += chunk;
        dst += chunk;
        size -= chunk;
    }
    return true;
}

static bool qcom845_cached_read32(Qcom845Display *s, Qcom845PageCache *cache,
                                  uint64_t iova, uint32_t *val)
{
    uint32_t le = 0;

    if (!qcom845_cached_read(s, cache, iova, &le, sizeof(le))) {
        return false;
    }
    *val = le32_to_cpu(le);
    return true;
}

static uint8_t *qcom845_stage_write_page(Qcom845Display *s,
                                         Qcom845PageCache *cache,
                                         uint64_t iova)
{
    uint64_t page = iova & ~(uint64_t)A6XX_PAGE_MASK;
    unsigned slot = (unsigned)(page >> A6XX_PAGE_SHIFT) & A6XX_PAGE_CACHE_MASK;

    if (cache->pg[slot] == page) {
        return cache->data[slot];
    }
    gpointer key = (gpointer)(uintptr_t)(page >> A6XX_PAGE_SHIFT);
    Qcom845StagedPage *entry = g_hash_table_lookup(cache->table, key);

    if (!entry) {
        hwaddr pa = page;
        const uint8_t *src;

        if (s->gpu_smmu &&
            !qcom845_smmu_translate(s->gpu_smmu, 0, page, true, &pa)) {
            return NULL;
        }
        pa &= ~(hwaddr)A6XX_PAGE_MASK;
        src = qcom845_ram_page_ptr(pa, true);
        if (!src) {
            return NULL;
        }
        entry = g_new(Qcom845StagedPage, 1);
        entry->physical = pa;
        memcpy(entry->data, src, A6XX_PAGE_SIZE);
        g_hash_table_insert(cache->table, key, entry);
    }
    cache->pg[slot] = page;
    cache->data[slot] = entry->data;
    return entry->data;
}

static bool qcom845_publish_write_pages(GHashTable *pages)
{
    GHashTableIter iter;
    gpointer key, value;

    g_hash_table_iter_init(&iter, pages);
    while (g_hash_table_iter_next(&iter, &key, &value)) {
        const Qcom845StagedPage *entry = value;

        if (address_space_write(&address_space_memory, entry->physical,
                                MEMTXATTRS_UNSPECIFIED, entry->data,
                                A6XX_PAGE_SIZE) != MEMTX_OK) {
            return false;
        }
    }
    return true;
}

/* ---- CP_LOAD_STATE6_GEOM (0x32) / CP_LOAD_STATE6_FRAG (0x34) ---- */

void qcom845_render_reset(Qcom845Display *s)
{
    for (unsigned b = 0; b < A6XX_LOAD_BLOCKS; b++) {
        for (unsigned t = 0; t < A6XX_LOAD_TYPES; t++) {
            if (s->load_state[b][t]) {
                g_array_set_size(s->load_state[b][t], 0);
            }
        }
    }
}

void qcom845_render_finalize(Qcom845Display *s)
{
    for (unsigned b = 0; b < A6XX_LOAD_BLOCKS; b++) {
        for (unsigned t = 0; t < A6XX_LOAD_TYPES; t++) {
            if (s->load_state[b][t]) {
                g_array_unref(s->load_state[b][t]);
                s->load_state[b][t] = NULL;
            }
        }
    }
}

void qcom845_render_load_state6(Qcom845Display *s, uint64_t stream_base,
                                uint32_t start, uint32_t ring_mask,
                                uint32_t pkt_pos, uint32_t pkt_dwords,
                                const uint32_t *w)
{
    if (pkt_dwords < 3) {
        return;
    }
    uint32_t w0 = w[0];
    unsigned dst_off = w0 & 0x3fff;
    unsigned type = (w0 >> 14) & 3;
    unsigned src = (w0 >> 16) & 3;
    unsigned blk = (w0 >> 18) & 0xf;
    unsigned num = (w0 >> 22) & 0x3ff;
    uint64_t addr = (uint64_t)w[1] | ((uint64_t)w[2] << 32);
    unsigned unit_words;

    if (blk >= A6XX_LOAD_BLOCKS || type >= A6XX_LOAD_TYPES ||
        num == 0 || dst_off + num > A6XX_LOAD_MAX_UNITS) {
        return;
    }

    if (blk < 8) {
        /* Texture/sampler state blocks: type 0 = 4dw sampler, type 1 = 16dw tex. */
        unit_words = (type == 0) ? 4 : (type == 1) ? 16 : 2;
    } else {
        /* Shader blocks: type 0 = 32dw (16 insns), type 1 = 4dw const, type 2 = 2dw UBO. */
        unit_words = (type == 0) ? 32 : (type == 1) ? 4 : 2;
    }

    GArray *arr = s->load_state[blk][type];
    if (!arr) {
        arr = g_array_new(false, true, sizeof(Qcom845StateUnit));
        s->load_state[blk][type] = arr;
    }
    if (arr->len < dst_off + num) {
        g_array_set_size(arr, dst_off + num);
    }

    if (src == 3) {
        /* SS6_UBO: lazy-bind constants from stage UBO table (load_state[blk][2]). */
        unsigned slot = w[1] >> 16;
        unsigned off = w[1] & 0xffff;

        for (unsigned i = 0; i < num; i++) {
            Qcom845StateUnit *u = &g_array_index(arr, Qcom845StateUnit,
                                                 dst_off + i);
            u->ubo = true;
            u->ubo_slot = slot;
            u->ubo_offset = off + i * 16;
            memset(u->words, 0, sizeof(u->words));
            u->valid = true;
        }
        return;
    }

    if (src == 0) {
        /* SS6_DIRECT: inline dwords immediately after w[0..2]. */
        unsigned total = num * unit_words;
        if (pkt_dwords < 3 + total) {
            return;
        }
        for (unsigned i = 0; i < num; i++) {
            Qcom845StateUnit *u = &g_array_index(arr, Qcom845StateUnit,
                                                 dst_off + i);
            u->ubo = false;
            u->ubo_slot = 0;
            u->ubo_offset = 0;
            for (unsigned k = 0; k < unit_words; k++) {
                uint32_t idx = (start + pkt_pos + 3 + i * unit_words + k) &
                               ring_mask;
                qcom845_gpu_read32(s, stream_base + (uint64_t)idx * 4,
                                   &u->words[k]);
            }
            u->valid = true;
        }
        return;
    }

    if (src == 2) {
        /* SS6_INDIRECT: read from GPU IOVA addr. */
        for (unsigned i = 0; i < num; i++) {
            Qcom845StateUnit *u = &g_array_index(arr, Qcom845StateUnit,
                                                 dst_off + i);
            u->ubo = false;
            u->ubo_slot = 0;
            u->ubo_offset = 0;
            for (unsigned k = 0; k < unit_words; k++) {
                qcom845_gpu_read32(s,
                    addr + (uint64_t)(i * unit_words + k) * 4, &u->words[k]);
            }
            u->valid = true;
        }
    }
}

/* ---- Shader constants, bindless descriptors, and textures ---- */

static bool qcom845_bindless_addr(const uint32_t *regs, unsigned base,
                                  unsigned idx, uint64_t *va,
                                  unsigned *desc_bytes)
{
    uint64_t reg64;

    if (base >= 5) {
        return false;
    }
    reg64 = qcom845_reg64(regs, A6XX_SP_GFX_BINDLESS_BASE_0 + base * 2);
    if (!reg64) {
        reg64 = qcom845_reg64(regs, A6XX_HLSQ_BINDLESS_BASE_0 + base * 2);
    }
    if (!reg64) {
        return false;
    }
    unsigned dsize = (2U << (reg64 & 3)) * 4;
    *desc_bytes = dsize;
    *va = (reg64 & ~3ULL) + (uint64_t)idx * dsize;
    return true;
}

static bool qcom845_shader_const_slow(Qcom845Display *s, unsigned blk,
                                      Qcom845PageCache *cache, unsigned idx,
                                      uint32_t *val)
{
    const GArray *consts = s->load_state[blk][1];
    unsigned unit_idx = idx / 4;

    if (!consts || unit_idx >= consts->len) {
        return false;
    }
    const Qcom845StateUnit *u = &g_array_index(consts, Qcom845StateUnit,
                                               unit_idx);
    if (!u->valid) {
        return false;
    }
    if (u->ubo) {
        const GArray *ubos = s->load_state[blk][2];
        if (!ubos || u->ubo_slot >= ubos->len) {
            return false;
        }
        const Qcom845StateUnit *d = &g_array_index(ubos, Qcom845StateUnit,
                                                   u->ubo_slot);
        if (!d->valid) {
            return false;
        }
        uint64_t va = (uint64_t)d->words[0] |
                      ((uint64_t)(d->words[1] & 0x1ffff) << 32);
        if (!va) {
            return false;
        }
        return qcom845_cached_read32(s, cache,
                                     va + u->ubo_offset + (idx % 4) * 4, val);
    }
    *val = u->words[idx % 4];
    return true;
}

static inline bool qcom845_shader_const(Qcom845Display *s, unsigned blk,
                                        const Qcom845ShaderRegs *r,
                                        unsigned idx, uint32_t *val)
{
    if (r->program && idx < r->program->const_count) {
        if (qcom845_bit_test(r->program->const_valid, idx)) {
            *val = r->program->consts[idx];
            return true;
        }
        if (!qcom845_shader_const_slow(s, blk, r->read_cache, idx, val)) {
            return false;
        }
        r->program->consts[idx] = *val;
        qcom845_bit_set(r->program->const_valid, idx);
        return true;
    }
    return qcom845_shader_const_slow(s, blk, r->read_cache, idx, val);
}

static void qcom845_shader_prepare(Qcom845Display *s, unsigned blk,
                                   uint64_t obj_addr,
                                   Qcom845PageCache *cache,
                                   Qcom845ShaderProgram *prog)
{
    const GArray *code = s->load_state[blk][0];
    const GArray *consts = s->load_state[blk][1];

    memset(prog->const_valid, 0, sizeof(prog->const_valid));
    prog->insn_count = 0;
    prog->const_count = 0;

    obj_addr &= ~3ULL;
    if (obj_addr) {
        for (unsigned pc = 0; pc < A6XX_SHADER_MAX_INSNS; pc++) {
            uint32_t lo = 0, hi = 0;
            if (!qcom845_cached_read32(s, cache, obj_addr + pc * 8, &lo) ||
                !qcom845_cached_read32(s, cache, obj_addr + pc * 8 + 4, &hi)) {
                break;
            }
            uint64_t insn = (uint64_t)lo | ((uint64_t)hi << 32);
            prog->insns[pc] = insn;
            prog->insn_count = pc + 1;
            /* Stop shortly after cat0 end (allowing trailing jump targets? No, scan further if needed). */
            if (pc > 0 && insn == 0 && prog->insns[pc - 1] == 0x0300000000000000ULL) {
                break;
            }
        }
    } else if (code) {
        unsigned max_u = MIN(code->len, A6XX_SHADER_MAX_INSNS / 16U);
        for (unsigned u = 0; u < max_u; u++) {
            const Qcom845StateUnit *unit =
                &g_array_index(code, Qcom845StateUnit, u);
            if (!unit->valid) {
                continue;
            }
            for (unsigned k = 0; k < 16; k++) {
                prog->insns[u * 16 + k] = (uint64_t)unit->words[k * 2] |
                                          ((uint64_t)unit->words[k * 2 + 1] << 32);
            }
            prog->insn_count = (u + 1) * 16;
        }
    }

    if (consts) {
        unsigned max_u = MIN(consts->len, A6XX_SHADER_MAX_CONSTS / 4U);
        prog->const_count = max_u * 4U;
        for (unsigned u = 0; u < max_u; u++) {
            const Qcom845StateUnit *unit =
                &g_array_index(consts, Qcom845StateUnit, u);
            if (!unit->valid || unit->ubo) {
                continue;
            }
            for (unsigned k = 0; k < 4; k++) {
                unsigned idx = u * 4 + k;
                prog->consts[idx] = unit->words[k];
                qcom845_bit_set(prog->const_valid, idx);
            }
        }
    }
}

static bool qcom845_tex_prepare(const uint32_t *tex, Qcom845TexInfo *info)
{
    unsigned fmt = extract32(tex[0], 22, 8);
    unsigned tex_type = extract32(tex[2], 29, 3);
    unsigned cpp;

    memset(info, 0, sizeof(*info));
    if (tex_type == 4) {
        /* TEX_BUFFER: support broad D3D11 buffer formats. */
        if (fmt >= 0x02 && fmt <= 0x06) {
            cpp = 1;
        } else if ((fmt >= 0x0f && fmt <= 0x12) || (fmt >= 0x15 && fmt <= 0x19)) {
            cpp = 2;
        } else if (fmt == 0x30 || fmt == 0x32 || fmt == 0x33 || fmt == 0x34 ||
                   (fmt >= 0x43 && fmt <= 0x4c)) {
            cpp = 4;
        } else if (fmt >= 0x60 && fmt <= 0x69) {
            cpp = 8;
        } else if (fmt >= 0x72 && fmt <= 0x76) {
            cpp = 12;
        } else if (fmt >= 0x7f && fmt <= 0x82) {
            cpp = 16;
        } else {
            return false;
        }
        info->buf_elems = extract32(tex[1], 0, 30);
    } else {
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
        info->width = extract32(tex[1], 0, 15);
        info->height = extract32(tex[1], 15, 15);
        info->pitch = extract32(tex[2], 7, 22);
    }

    info->tile = tex[0] & 3;
    if (tex_type != 4 && info->tile != 0) {
        return false;
    }
    info->fmt = fmt;
    info->swap = extract32(tex[0], 30, 2);
    info->srgb = !!(tex[0] & BIT(2));
    info->tex_type = tex_type;
    info->base = (uint64_t)(tex[4] & ~31U) |
                 ((uint64_t)(tex[5] & 0x1ffff) << 32);
    /* For TEX_BUFFER, BASE_LO can be byte/element aligned (<32B). */
    if (tex_type == 4) {
        info->base = (uint64_t)tex[4] | ((uint64_t)(tex[5] & 0x1ffff) << 32);
    }
    info->cpp = cpp;
    info->identity_swizzle = true;
    for (unsigned i = 0; i < 4; i++) {
        info->swizzle[i] = extract32(tex[0], 4 + i * 3, 3);
        if (info->swizzle[i] != i) {
            info->identity_swizzle = false;
        }
    }
    info->valid = info->base != 0;
    return info->valid;
}

static int qcom845_tex_wrap(int coord, int size, unsigned mode, bool *border)
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
    case 2: /* MIRROR_REPEAT */ {
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

static bool qcom845_tex_fetch_2d(Qcom845Display *s, Qcom845PageCache *cache,
                                 const Qcom845TexInfo *info, int x, int y,
                                 bool border, float out[4],
                                 uint32_t raw_out[4], bool *is_int)
{
    float comps[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    uint32_t icomps[4] = { 0, 0, 0, 1 };
    uint32_t word = 0;

    *is_int = false;
    if (border || x < 0 || y < 0 ||
        x >= (int)info->width || y >= (int)info->height) {
        out[0] = out[1] = out[2] = out[3] = 0.0f;
        raw_out[0] = raw_out[1] = raw_out[2] = raw_out[3] = 0;
        return true;
    }

    uint64_t addr = info->base + (uint64_t)y * info->pitch +
                    (uint64_t)x * info->cpp;
    uint8_t raw[4] = { 0 };
    if (!qcom845_cached_read(s, cache, addr, raw, info->cpp)) {
        return false;
    }
    word = (info->cpp == 1) ? raw[0]
         : (info->cpp == 2) ? lduw_le_p(raw)
                            : ldl_le_p(raw);

    switch (info->fmt) {
    case 0x02: /* A8_UNORM */
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
    case 0x30: /* RGBA8_UNORM */ {
        uint32_t c = qcom845_color_unswap(word, info->swap);
        if (info->srgb) {
            const float *lut = qcom845_srgb_lut();
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
        break;
    }
    case 0x33: /* RGBA8_UINT */ {
        uint32_t c = qcom845_color_unswap(word, info->swap);
        *is_int = true;
        icomps[0] = c & 0xff;
        icomps[1] = (c >> 8) & 0xff;
        icomps[2] = (c >> 16) & 0xff;
        icomps[3] = (c >> 24) & 0xff;
        break;
    }
    case 0x4a: /* R32_FLOAT */
        comps[0] = qcom845_u32_to_f(word);
        break;
    case 0x4b:
    case 0x4c:
        *is_int = true;
        icomps[0] = word;
        break;
    }

    if (info->identity_swizzle && !*is_int) {
        for (unsigned c = 0; c < 4; c++) {
            out[c] = comps[c];
            raw_out[c] = qcom845_f_to_u32(comps[c]);
        }
        return true;
    }

    for (unsigned i = 0; i < 4; i++) {
        unsigned sw = info->swizzle[i];
        if (*is_int) {
            raw_out[i] = (sw < 4) ? icomps[sw] : (sw == 5 ? 1 : 0);
            out[i] = (float)raw_out[i];
        } else {
            out[i] = (sw < 4) ? comps[sw] : (sw == 5 ? 1.0f : 0.0f);
            raw_out[i] = qcom845_f_to_u32(out[i]);
        }
    }
    return true;
}

static bool qcom845_tex_fetch_buf(Qcom845Display *s, Qcom845PageCache *cache,
                                  const Qcom845TexInfo *info, int ix,
                                  float res[4], uint32_t raw_res[4],
                                  bool *is_int)
{
    uint8_t raw[16] = { 0 };
    float fcomps[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    uint32_t icomps[4] = { 0, 0, 0, 1 };
    unsigned fmt = info->fmt;
    unsigned cpp = info->cpp;

    *is_int = false;
    if (ix < 0 || (info->buf_elems && (uint32_t)ix >= info->buf_elems)) {
        for (unsigned c = 0; c < 4; c++) {
            res[c] = 0.0f;
            raw_res[c] = 0;
        }
        return true;
    }
    if (!qcom845_cached_read(s, cache, info->base + (uint64_t)ix * cpp,
                             raw, cpp)) {
        return false;
    }

    if (fmt == 0x02) { /* A8_UNORM */
        fcomps[3] = raw[0] * (1.0f / 255.0f);
    } else if ((fmt >= 0x03 && fmt <= 0x06) || (fmt >= 0x0f && fmt <= 0x12) ||
               fmt == 0x30 || fmt == 0x32 || fmt == 0x33 || fmt == 0x34) {
        unsigned kind;
        if (cpp == 4) {
            uint32_t c = qcom845_color_unswap(ldl_le_p(raw), info->swap);
            stl_le_p(raw, c);
            kind = (fmt == 0x30) ? 0 : (fmt == 0x32) ? 1 : (fmt == 0x33) ? 2 : 3;
        } else {
            kind = (fmt >= 0x0f) ? (fmt - 0x0f) : (fmt - 0x03);
        }
        *is_int = kind >= 2;
        for (unsigned c = 0; c < cpp; c++) {
            if (kind == 0) {
                fcomps[c] = raw[c] * (1.0f / 255.0f);
            } else if (kind == 1) {
                int8_t sb = (int8_t)raw[c];
                fcomps[c] = (sb == INT8_MIN) ? -1.0f : sb / 127.0f;
            } else if (kind == 2) {
                icomps[c] = raw[c];
            } else {
                icomps[c] = (uint32_t)(int32_t)(int8_t)raw[c];
            }
        }
    } else if ((fmt >= 0x15 && fmt <= 0x19) || (fmt >= 0x43 && fmt <= 0x47) ||
               (fmt >= 0x60 && fmt <= 0x64)) {
        unsigned n = cpp / 2;
        unsigned kind = (fmt >= 0x60) ? (fmt - 0x60)
                      : (fmt >= 0x43) ? (fmt - 0x43)
                                      : (fmt - 0x15);
        *is_int = kind >= 3;
        for (unsigned c = 0; c < n; c++) {
            uint16_t v = lduw_le_p(raw + c * 2);
            if (kind == 3) {
                icomps[c] = v;
            } else if (kind == 4) {
                icomps[c] = (uint32_t)(int32_t)(int16_t)v;
            } else if (kind == 2) {
                fcomps[c] = qcom845_u32_to_f(qcom845_f16_to_f32(v));
            } else if (kind == 0) {
                fcomps[c] = v * (1.0f / 65535.0f);
            } else {
                int16_t sv = (int16_t)v;
                fcomps[c] = (sv == INT16_MIN) ? -1.0f : sv / 32767.0f;
            }
        }
    } else {
        unsigned n = cpp / 4;
        bool flt = (fmt == 0x4a || fmt == 0x67 || fmt == 0x74 ||
                    fmt == 0x7f || fmt == 0x82);
        *is_int = !flt;
        for (unsigned c = 0; c < n; c++) {
            uint32_t v = ldl_le_p(raw + c * 4);
            if (flt) {
                fcomps[c] = qcom845_u32_to_f(v);
            } else {
                icomps[c] = v;
            }
        }
    }

    for (unsigned c = 0; c < 4; c++) {
        unsigned sw = info->swizzle[c];
        if (*is_int) {
            raw_res[c] = (sw < 4) ? icomps[sw] : (sw == 5 ? 1 : 0);
            res[c] = qcom845_u32_to_f(raw_res[c]);
        } else {
            res[c] = (sw < 4) ? fcomps[sw] : (sw == 5 ? 1.0f : 0.0f);
            raw_res[c] = qcom845_f_to_u32(res[c]);
        }
    }
    return true;
}

static bool qcom845_sample_tex2d(Qcom845Display *s, Qcom845PageCache *cache,
                                 const uint32_t *samp,
                                 const Qcom845TexInfo *info,
                                 float fu, float fv,
                                 float res[4], uint32_t raw_res[4],
                                 bool *is_int)
{
    unsigned w = info->width;
    unsigned h = info->height;
    bool unnorm = !!(samp[1] & BIT(5));
    bool linear = extract32(samp[0], 1, 2) == 1 ||
                  extract32(samp[0], 3, 2) == 1;
    unsigned wrap_s = extract32(samp[0], 5, 3);
    unsigned wrap_t = extract32(samp[0], 8, 3);
    float px = unnorm ? fu : fu * (float)w;
    float py = (info->tex_type == 0) ? 0.0f : (unnorm ? fv : fv * (float)h);

    if (!linear) {
        bool b = false;
        int ix = qcom845_tex_wrap((int)floorf(px), w, wrap_s, &b);
        int iy = qcom845_tex_wrap((int)floorf(py), h, wrap_t, &b);
        return qcom845_tex_fetch_2d(s, cache, info, ix, iy, b,
                                    res, raw_res, is_int);
    }

    float fx = px - 0.5f;
    float fy = py - 0.5f;
    int x0 = (int)floorf(fx);
    int y0 = (int)floorf(fy);
    float ax = fx - (float)x0;
    float ay = fy - (float)y0;
    float c00[4], c10[4], c01[4], c11[4];
    uint32_t rtmp[4];
    bool b00 = false, b10 = false, b01 = false, b11 = false;
    int wx0 = qcom845_tex_wrap(x0, w, wrap_s, &b00);
    int wy0 = qcom845_tex_wrap(y0, h, wrap_t, &b01);

    if (ax == 0.0f && ay == 0.0f) {
        return qcom845_tex_fetch_2d(s, cache, info, wx0, wy0, b00 || b01,
                                    res, raw_res, is_int);
    }
    int wx1 = qcom845_tex_wrap(x0 + 1, w, wrap_s, &b10);
    int wy1 = qcom845_tex_wrap(y0 + 1, h, wrap_t, &b11);
    if (!qcom845_tex_fetch_2d(s, cache, info, wx0, wy0, b00 || b01, c00, rtmp, is_int) ||
        !qcom845_tex_fetch_2d(s, cache, info, wx1, wy0, b10 || b01, c10, rtmp, is_int) ||
        !qcom845_tex_fetch_2d(s, cache, info, wx0, wy1, b00 || b11, c01, rtmp, is_int) ||
        !qcom845_tex_fetch_2d(s, cache, info, wx1, wy1, b10 || b11, c11, rtmp, is_int)) {
        return false;
    }
    for (unsigned i = 0; i < 4; i++) {
        float top = c00[i] * (1.0f - ax) + c10[i] * ax;
        float bot = c01[i] * (1.0f - ax) + c11[i] * ax;
        res[i] = top * (1.0f - ay) + bot * ay;
        raw_res[i] = qcom845_f_to_u32(res[i]);
    }
    return true;
}

/* ---- IR3 Register File & Operand Helpers ---- */

static inline void qcom845_shader_reset_pixel(Qcom845ShaderRegs *r)
{
    memset(r->full_valid, 0, sizeof(r->full_valid));
    memset(r->half_valid, 0, sizeof(r->half_valid));
    r->pred_valid = 0;
    r->a0_valid = false;
    r->a1_valid = false;
    r->killed = false;
}

static inline bool qcom845_ir3_read_reg(const Qcom845ShaderRegs *r,
                                        unsigned idx, bool full, uint32_t *out)
{
    if (idx >= 248 && idx <= 251) {
        unsigned p = idx - 248;
        *out = (r->pred_valid & BIT(p)) ? r->pred[p] : 0;
        return true;
    }
    if (idx >= A6XX_SHADER_GPRS) {
        return false;
    }
    if (full) {
        if (!qcom845_bit_test(r->full_valid, idx)) {
            return false;
        }
        *out = r->full[idx];
    } else {
        if (!qcom845_bit_test(r->half_valid, idx)) {
            return false;
        }
        *out = r->half[idx];
    }
    return true;
}

static inline bool qcom845_ir3_write_reg(Qcom845ShaderRegs *r, unsigned idx,
                                         bool dest_half, bool is_float,
                                         uint32_t val)
{
    if (idx >= 248 && idx <= 251) {
        r->pred[idx - 248] = val & 1;
        r->pred_valid |= BIT(idx - 248);
        return true;
    }
    if (idx == 244) {
        r->a0 = (int16_t)val;
        r->a0_valid = true;
        return true;
    }
    if (idx == 245) {
        r->a1 = (int16_t)val;
        r->a1_valid = true;
        return true;
    }
    if (idx >= 252 && idx <= 255) {
        return true; /* r63.xyzw discard sink */
    }
    if (idx >= A6XX_SHADER_GPRS) {
        return false;
    }
    if (dest_half) {
        r->half[idx] = is_float ? qcom845_f32_to_f16(val) : (uint16_t)val;
        qcom845_bit_set(r->half_valid, idx);
    } else {
        r->full[idx] = val;
        qcom845_bit_set(r->full_valid, idx);
    }
    return true;
}

static inline bool qcom845_ir3_multisrc(Qcom845Display *s, unsigned blk,
                                        const Qcom845ShaderRegs *r,
                                        unsigned enc, unsigned inc, bool full,
                                        unsigned kind, uint32_t *out)
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
        if (!qcom845_ir3_read_reg(r, idx, full, &val)) {
            return false;
        }
        if (!full) {
            val = (kind == 0) ? qcom845_f16_to_f32((uint16_t)val)
                : (kind == 1) ? (uint32_t)(int32_t)(int16_t)val
                              : (val & 0xffff);
        }
    } else if (sel == 1) {
        int idx;
        if (!r->a0_valid) {
            return false;
        }
        idx = r->a0 + sextract32(enc, 0, 10) + (int)inc;
        if (idx < 0) {
            return false;
        }
        if (enc & BIT(10)) {
            if (!qcom845_shader_const(s, blk, r, idx, &val)) {
                return false;
            }
            if (!full && kind == 0) {
                val = qcom845_f16_to_f32((uint16_t)val);
            }
        } else {
            if (!qcom845_ir3_read_reg(r, idx, full, &val)) {
                return false;
            }
            if (!full) {
                val = (kind == 0) ? qcom845_f16_to_f32((uint16_t)val)
                    : (kind == 1) ? (uint32_t)(int32_t)(int16_t)val
                                  : (val & 0xffff);
            }
        }
    } else if ((sel & 3) == 2) {
        unsigned idx = (enc & 0x7ff) + inc;
        if (!qcom845_shader_const(s, blk, r, idx, &val)) {
            return false;
        }
        if (!full) {
            val = (kind == 0) ? qcom845_f16_to_f32((uint16_t)val)
                : (kind == 1) ? (uint32_t)(int32_t)(int16_t)val
                              : (val & 0xffff);
        }
    } else if (sel == 4) {
        int32_t imm = sextract32(enc, 0, 11);
        val = (kind == 0) ? qcom845_f_to_u32((float)imm) : (uint32_t)imm;
    } else if (sel == 5) {
        unsigned fi = enc & 0x3ff;
        if (fi >= ARRAY_SIZE(flut)) {
            return false;
        }
        val = qcom845_f_to_u32(flut[fi]);
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
            val = (sv < 0) ? (uint32_t)(-(int64_t)sv) : val;
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

static inline bool qcom845_ir3_cat3_src(Qcom845Display *s, unsigned blk,
                                        const Qcom845ShaderRegs *r,
                                        unsigned enc13, bool neg, unsigned inc,
                                        bool full, unsigned kind, uint32_t *out)
{
    unsigned sel = extract32(enc13, 11, 2);
    uint32_t val = 0;

    if (sel == 0) {
        if (!qcom845_ir3_read_reg(r, (enc13 & 0xff) + inc, full, &val)) {
            return false;
        }
    } else if (sel == 1) {
        int idx;
        if (!r->a0_valid) {
            return false;
        }
        idx = r->a0 + sextract32(enc13, 0, 10) + (int)inc;
        if (idx < 0) {
            return false;
        }
        if (enc13 & BIT(10)) {
            if (!qcom845_shader_const(s, blk, r, idx, &val)) {
                return false;
            }
        } else if (!qcom845_ir3_read_reg(r, idx, full, &val)) {
            return false;
        }
    } else if (sel == 2) {
        if (!qcom845_shader_const(s, blk, r, (enc13 & 0x7ff) + inc, &val)) {
            return false;
        }
    } else {
        return false;
    }

    if (!full) {
        val = (kind == 0) ? qcom845_f16_to_f32((uint16_t)val)
            : (kind == 1) ? (uint32_t)(int32_t)(int16_t)val
                          : (val & 0xffff);
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

static bool qcom845_load_tex_samp_nonbindless(Qcom845Display *s,
                                              Qcom845PageCache *cache,
                                              unsigned tex_blk,
                                              unsigned tex_idx,
                                              unsigned samp_idx,
                                              bool need_samp,
                                              uint32_t tex_dw[16],
                                              uint32_t samp_dw[4])
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    const GArray *textures = s->load_state[tex_blk][1];
    const GArray *samplers = s->load_state[tex_blk][0];

    if (textures && tex_idx < textures->len &&
        g_array_index(textures, Qcom845StateUnit, tex_idx).valid) {
        memcpy(tex_dw, g_array_index(textures, Qcom845StateUnit, tex_idx).words,
               16 * sizeof(uint32_t));
    } else {
        uint64_t tbase = qcom845_reg64(regs, A6XX_SP_PS_TEXMEMOBJ_BASE_LO);
        if (!tbase ||
            !qcom845_cached_read(s, cache, tbase + (uint64_t)tex_idx * 64,
                                 tex_dw, 64)) {
            return false;
        }
        for (unsigned i = 0; i < 16; i++) {
            tex_dw[i] = le32_to_cpu(tex_dw[i]);
        }
    }

    if (need_samp) {
        if (samplers && samp_idx < samplers->len &&
            g_array_index(samplers, Qcom845StateUnit, samp_idx).valid) {
            memcpy(samp_dw,
                   g_array_index(samplers, Qcom845StateUnit, samp_idx).words,
                   4 * sizeof(uint32_t));
        } else {
            uint64_t sbase = qcom845_reg64(regs, A6XX_SP_PS_SAMPLER_BASE_LO);
            if (!sbase ||
                !qcom845_cached_read(s, cache, sbase + (uint64_t)samp_idx * 16,
                                     samp_dw, 16)) {
                return false;
            }
            for (unsigned i = 0; i < 4; i++) {
                samp_dw[i] = le32_to_cpu(samp_dw[i]);
            }
        }
    }
    return true;
}

static bool qcom845_fs_prefetch(Qcom845Display *s, Qcom845ShaderRegs *r)
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    unsigned cntl = regs[A6XX_SP_FS_PREFETCH_CNTL];
    unsigned count = cntl & 7;

    if (!count) {
        return true;
    }
    if (count > 4) {
        trace_qcom845_render_reject("fs_prefetch_cntl", cntl, 0);
        return false;
    }
    for (unsigned i = 0; i < count; i++) {
        uint32_t cmd = regs[A6XX_SP_FS_PREFETCH_CMD_0 + i];
        unsigned src = extract32(cmd, 0, 7);
        unsigned samp_id = extract32(cmd, 7, 4);
        unsigned tex_id = extract32(cmd, 11, 5);
        unsigned dst = extract32(cmd, 16, 6);
        unsigned wrmask = extract32(cmd, 22, 4);
        bool half = !!(cmd & BIT(26));
        unsigned opc = extract32(cmd, 29, 3);
        uint32_t tex_dw[16], samp_dw[4];
        Qcom845TexInfo info;
        float res[4] = { 0, 0, 0, 1 };
        uint32_t raw_res[4] = { 0, 0, 0, 1 };
        bool is_int = false;

        if (opc != 1 || src + 1 >= 64 ||
            !(r->input_valid & BIT_ULL(src)) ||
            !(r->input_valid & BIT_ULL(src + 1))) {
            trace_qcom845_render_reject("fs_prefetch_cmd", cmd, src);
            return false;
        }
        if (!qcom845_load_tex_samp_nonbindless(s, r->read_cache, 4, tex_id,
                                               samp_id, true, tex_dw, samp_dw) ||
            !qcom845_tex_prepare(tex_dw, &info)) {
            trace_qcom845_render_reject("fs_prefetch_tex", tex_id, samp_id);
            return false;
        }
        float fu = qcom845_u32_to_f(r->input[src]);
        float fv = qcom845_u32_to_f(r->input[src + 1]);
        if (!qcom845_sample_tex2d(s, r->read_cache, samp_dw, &info, fu, fv,
                                  res, raw_res, &is_int)) {
            return false;
        }
        for (unsigned c = 0; c < 4; c++) {
            if (wrmask & BIT(c)) {
                if (!qcom845_ir3_write_reg(r, dst + c, half, true,
                                           qcom845_f_to_u32(res[c]))) {
                    return false;
                }
            }
        }
    }
    return true;
}

static bool qcom845_ir3_cat5(Qcom845Display *s, unsigned blk,
                             Qcom845ShaderRegs *r, uint64_t insn)
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    unsigned opc = extract64(insn, 54, 5);
    bool full_src = !!(insn & BIT_ULL(0));
    unsigned src1 = extract64(insn, 1, 8);
    unsigned dst = extract64(insn, 32, 8);
    unsigned wrmask = extract64(insn, 40, 4);
    unsigned dst_type = extract64(insn, 44, 3);
    bool dst_half = (dst_type == 0 || dst_type == 2 || dst_type == 4 ||
                     dst_type == 6);
    bool is_float_dst = (dst_type == 0 || dst_type == 1);
    bool s2en_bindless = !!(insn & BIT_ULL(51));
    bool need_samp = (opc >= 3);
    uint32_t tex_dw[16] = { 0 };
    uint32_t samp_dw[4] = { 0 };
    Qcom845TexInfo info;
    float res[4] = { 0, 0, 0, 1 };
    uint32_t raw_res[4] = { 0, 0, 0, 1 };
    bool is_int = false;

    if (opc > 5) {
        trace_qcom845_render_reject("cat5_opc", opc, insn);
        return false;
    }

    if (s2en_bindless) {
        unsigned desc_mode = extract64(insn, 29, 3);
        unsigned tex_base = (extract64(insn, 19, 2) << 1) |
                            extract64(insn, 47, 1);
        unsigned samp_base = tex_base;
        unsigned tex_idx = 0, samp_idx = 0;

        if (desc_mode == 7) {
            /* CAT5_BINDLESS_A1_IMM: samp_base=a1[2:0], tex_idx=a1[10:3], samp_idx=src3[7:0]. */
            if (!r->a1_valid) {
                trace_qcom845_render_reject("cat5_a1_invalid", insn, 0);
                return false;
            }
            uint16_t a1 = (uint16_t)r->a1;
            samp_base = a1 & 7;
            tex_idx = (a1 >> 3) & 0xff;
            samp_idx = extract64(insn, 21, 8);
        } else if (desc_mode == 6) {
            /* CAT5_BINDLESS_IMM */
            samp_idx = extract64(insn, 21, 4);
            tex_idx = extract64(insn, 25, 4);
        } else if (desc_mode == 5 || desc_mode == 2 ||
                   desc_mode == 1 || desc_mode == 3) {
            unsigned src3 = extract64(insn, 21, 8);
            uint32_t s3v = 0;
            if (!qcom845_ir3_read_reg(r, src3, true, &s3v)) {
                trace_qcom845_render_reject("cat5_src3", src3, desc_mode);
                return false;
            }
            if (desc_mode == 1 || desc_mode == 3) {
                if (!r->a1_valid) {
                    return false;
                }
                samp_base = ((uint16_t)r->a1) & 7;
            }
            if (need_samp) {
                samp_idx = s3v & 0xffff;
                tex_idx = s3v >> 16;
            } else {
                tex_idx = s3v & 0xffff;
            }
        } else {
            /* Non-bindless indirect s2en (desc_mode 0 / 4). */
            unsigned src3 = extract64(insn, 21, 8);
            uint32_t s3v = 0;
            if (!qcom845_ir3_read_reg(r, src3, false, &s3v)) {
                return false;
            }
            unsigned tex_blk = (blk == 12) ? 4 : 0;
            if (!qcom845_load_tex_samp_nonbindless(s, r->read_cache, tex_blk,
                    (s3v >> 8) & 0xff, s3v & 0xff, need_samp,
                    tex_dw, samp_dw)) {
                trace_qcom845_render_reject("cat5_s2en_load", s3v, tex_blk);
                return false;
            }
            goto have_descs;
        }

        uint64_t tva = 0, sva = 0;
        unsigned tdbytes = 0, sdbytes = 0;
        if (!qcom845_bindless_addr(regs, tex_base, tex_idx, &tva, &tdbytes) ||
            tdbytes < 64 ||
            !qcom845_cached_read(s, r->read_cache, tva, tex_dw, 64)) {
            trace_qcom845_render_reject("cat5_bindless_tex",
                                        ((uint64_t)tex_base << 32) | tex_idx, tva);
            return false;
        }
        for (unsigned i = 0; i < 16; i++) {
            tex_dw[i] = le32_to_cpu(tex_dw[i]);
        }
        if (need_samp) {
            if (!qcom845_bindless_addr(regs, samp_base, samp_idx, &sva, &sdbytes) ||
                sdbytes < 16 ||
                !qcom845_cached_read(s, r->read_cache, sva, samp_dw, 16)) {
                trace_qcom845_render_reject("cat5_bindless_samp",
                    ((uint64_t)samp_base << 32) | samp_idx, sva);
                return false;
            }
            for (unsigned i = 0; i < 4; i++) {
                samp_dw[i] = le32_to_cpu(samp_dw[i]);
            }
        }
    } else {
        unsigned samp_idx = extract64(insn, 21, 4);
        unsigned tex_idx = extract64(insn, 25, 7);
        unsigned tex_blk = (blk == 12) ? 4 : 0;
        if (!qcom845_load_tex_samp_nonbindless(s, r->read_cache, tex_blk,
                                               tex_idx, samp_idx, need_samp,
                                               tex_dw, samp_dw)) {
            trace_qcom845_render_reject("cat5_nonbindless", tex_idx, samp_idx);
            return false;
        }
    }

have_descs:
    if (!qcom845_tex_prepare(tex_dw, &info)) {
        trace_qcom845_render_reject("cat5_tex_prepare", tex_dw[0], tex_dw[2]);
        return false;
    }

    uint32_t u_raw = 0, v_raw = 0;
    if (!qcom845_ir3_read_reg(r, src1, full_src, &u_raw)) {
        trace_qcom845_render_reject("cat5_src1", src1, full_src);
        return false;
    }
    if (info.tex_type != 0 && info.tex_type != 4) {
        if (!qcom845_ir3_read_reg(r, src1 + 1, full_src, &v_raw)) {
            trace_qcom845_render_reject("cat5_src1p1", src1 + 1, info.tex_type);
            return false;
        }
    }

    if (opc <= 2) {
        int ix = full_src ? (int32_t)u_raw : (int16_t)u_raw;
        int iy = (info.tex_type == 0 || info.tex_type == 4) ? 0
               : (full_src ? (int32_t)v_raw : (int16_t)v_raw);
        if (info.tex_type == 4) {
            if (!qcom845_tex_fetch_buf(s, r->read_cache, &info, ix,
                                       res, raw_res, &is_int)) {
                trace_qcom845_render_reject("cat5_fetch_buf", info.base, ix);
                return false;
            }
        } else {
            if (!qcom845_tex_fetch_2d(s, r->read_cache, &info, ix, iy, false,
                                      res, raw_res, &is_int)) {
                trace_qcom845_render_reject("cat5_fetch_2d", info.base,
                                            ((uint64_t)iy << 32) | (uint32_t)ix);
                return false;
            }
        }
    } else {
        float fu = full_src ? qcom845_u32_to_f(u_raw)
                            : qcom845_u32_to_f(qcom845_f16_to_f32((uint16_t)u_raw));
        float fv = (info.tex_type == 0) ? 0.0f
                 : (full_src ? qcom845_u32_to_f(v_raw)
                             : qcom845_u32_to_f(qcom845_f16_to_f32((uint16_t)v_raw)));
        if (!qcom845_sample_tex2d(s, r->read_cache, samp_dw, &info, fu, fv,
                                  res, raw_res, &is_int)) {
            trace_qcom845_render_reject("cat5_sample", info.base, info.fmt);
            return false;
        }
    }

    for (unsigned c = 0; c < 4; c++) {
        if (wrmask & BIT(c)) {
            uint32_t out_val = is_float_dst ? qcom845_f_to_u32(res[c])
                                            : raw_res[c];
            if (!qcom845_ir3_write_reg(r, dst + c, dst_half, is_float_dst,
                                       out_val)) {
                return false;
            }
        }
    }
    return true;
}

static bool qcom845_ir3_cat6(Qcom845Display *s, unsigned blk,
                             Qcom845ShaderRegs *r, uint64_t insn)
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    unsigned opc54 = extract64(insn, 54, 5);
    unsigned sub52 = extract64(insn, 52, 2);
    unsigned type = extract64(insn, 49, 3);
    unsigned dst = extract64(insn, 32, 8);
    bool dest_half = (type == 0 || type == 2 || type == 4 || type == 6);

    if (opc54 == 0 && (sub52 == 2 || sub52 == 3) &&
        extract64(insn, 14, 6) == 0x1e) {
        /* A6xx ldc (sub==2) and ldc.k (sub==3). */
        unsigned base = extract64(insn, 1, 3);
        unsigned mode = extract64(insn, 6, 2);
        bool bindless = !!(insn & BIT_ULL(8));
        unsigned comp_off = extract64(insn, 9, 2);
        unsigned ncomp = extract64(insn, 12, 2) + 1;
        bool src1_im = !!(insn & BIT_ULL(23));
        unsigned src1 = extract64(insn, 24, 8);
        unsigned ubo_f = extract64(insn, 41, 8);
        uint32_t ubo_idx = 0, off_vec4 = 0;

        if (mode == 0) {
            ubo_idx = ubo_f;
        } else if (!qcom845_ir3_read_reg(r, ubo_f, true, &ubo_idx)) {
            return false;
        }
        if (src1_im) {
            off_vec4 = src1;
        } else if (!qcom845_ir3_read_reg(r, src1, true, &off_vec4)) {
            return false;
        }

        uint32_t d[2] = { 0, 0 };
        if (bindless) {
            uint64_t dva = 0;
            unsigned dbytes = 0;
            if (!qcom845_bindless_addr(regs, base, ubo_idx, &dva, &dbytes) ||
                !qcom845_cached_read32(s, r->read_cache, dva, &d[0]) ||
                !qcom845_cached_read32(s, r->read_cache, dva + 4, &d[1])) {
                trace_qcom845_render_reject("cat6_ldc_desc",
                    ((uint64_t)base << 32) | ubo_idx, dva);
                return false;
            }
        } else {
            const GArray *ubos = s->load_state[blk][2];
            if (!ubos || ubo_idx >= ubos->len ||
                !g_array_index(ubos, Qcom845StateUnit, ubo_idx).valid) {
                trace_qcom845_render_reject("cat6_ldc_ubo", blk, ubo_idx);
                return false;
            }
            d[0] = g_array_index(ubos, Qcom845StateUnit, ubo_idx).words[0];
            d[1] = g_array_index(ubos, Qcom845StateUnit, ubo_idx).words[1];
        }

        uint64_t ubo_va = (uint64_t)d[0] | ((uint64_t)(d[1] & 0x1ffff) << 32);
        if (!ubo_va) {
            trace_qcom845_render_reject("cat6_ldc_va0", d[0], d[1]);
            return false;
        }

        if (sub52 == 2) {
            /* ldc: load ncomp dwords into GPRs dst..dst+ncomp-1 */
            uint64_t byte_off = (uint64_t)off_vec4 * 16 + comp_off * 4;
            for (unsigned c = 0; c < ncomp; c++) {
                uint32_t v = 0;
                if (!qcom845_cached_read32(s, r->read_cache,
                                           ubo_va + byte_off + c * 4, &v) ||
                    !qcom845_ir3_write_reg(r, dst + c, false, false, v)) {
                    return false;
                }
            }
            return true;
        } else {
            /* ldc.k: load (dst + 1) vec4s into const file at a1.x */
            if (!r->a1_valid || !r->program) {
                return false;
            }
            unsigned nvec4 = dst + 1;
            unsigned cbase = (uint16_t)r->a1;
            for (unsigned i = 0; i < nvec4 * 4; i++) {
                uint32_t v = 0;
                if (cbase + i >= A6XX_SHADER_MAX_CONSTS ||
                    !qcom845_cached_read32(s, r->read_cache,
                        ubo_va + (uint64_t)off_vec4 * 16 + i * 4, &v)) {
                    return false;
                }
                r->program->consts[cbase + i] = v;
                qcom845_bit_set(r->program->const_valid, cbase + i);
                if (cbase + i >= r->program->const_count) {
                    r->program->const_count = cbase + i + 1;
                }
            }
            return true;
        }
    }

    if (opc54 == 31) {
        /* ldlv: load flat varying */
        unsigned off = extract64(insn, 1, 13);
        unsigned size = extract64(insn, 24, 8);
        if (blk != 12 || !size || off + size > 64) {
            return false;
        }
        for (unsigned c = 0; c < size; c++) {
            uint32_t v = (r->flat_valid & BIT_ULL(off + c)) ? r->flat_input[off + c]
                       : (r->input_valid & BIT_ULL(off + c)) ? r->input[off + c]
                                                             : 0;
            if (!qcom845_ir3_write_reg(r, dst + c, dest_half, false, v)) {
                return false;
            }
        }
        return true;
    }

    trace_qcom845_render_reject("cat6_unhandled", insn >> 32, insn & 0xffffffff);
    return false;
}

static uint32_t qcom845_cat1_convert(uint32_t raw, unsigned src_type,
                                     unsigned dst_type, unsigned round_mode)
{
    if (src_type == dst_type) {
        return raw;
    }
    float f = 0.0f;
    int64_t sv = 0;
    uint64_t uv = 0;
    bool src_is_float = (src_type == 0 || src_type == 1);
    bool src_is_signed = (src_type == 4 || src_type == 5 || src_type == 7);

    if (src_type == 0) {
        f = qcom845_u32_to_f(qcom845_f16_to_f32((uint16_t)raw));
    } else if (src_type == 1) {
        f = qcom845_u32_to_f(raw);
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
        float df = src_is_float ? f : (src_is_signed ? (float)sv : (float)uv);
        uint32_t fbits = qcom845_f_to_u32(df);
        return (dst_type == 0) ? qcom845_f32_to_f16(fbits) : fbits;
    }
    if (dst_type == 4 || dst_type == 5 || dst_type == 7) {
        int64_t dv = src_is_float ? (isnan(f) ? 0 : (int64_t)f)
                                  : (src_is_signed ? sv : (int64_t)uv);
        return (dst_type == 4) ? (uint16_t)CLAMP(dv, INT16_MIN, INT16_MAX)
             : (dst_type == 7) ? (uint8_t)CLAMP(dv, INT8_MIN, INT8_MAX)
                               : (uint32_t)CLAMP(dv, INT32_MIN, INT32_MAX);
    }
    uint64_t dv = src_is_float ? ((isnan(f) || f <= 0.0f) ? 0 : (uint64_t)f)
                               : (src_is_signed ? (sv < 0 ? 0 : (uint64_t)sv) : uv);
    return (dst_type == 2) ? (uint16_t)MIN(dv, (uint64_t)UINT16_MAX)
         : (dst_type == 6) ? (uint8_t)MIN(dv, 255ULL)
                           : (uint32_t)MIN(dv, (uint64_t)UINT32_MAX);
}

static bool qcom845_shader_execute(Qcom845Display *s, unsigned blk,
                                   Qcom845ShaderRegs *r)
{
    unsigned pred_mode = 0;
    bool pred_val = true;
    int pc = 0;

    for (unsigned step = 0; step < A6XX_SHADER_STEPS; step++) {
        if (pc < 0 || !r->program || (unsigned)pc >= r->program->insn_count) {
            return false;
        }
        uint64_t insn = r->program->insns[pc++];
        uint32_t low = (uint32_t)insn;
        uint32_t high = (uint32_t)(insn >> 32);
        unsigned cat = extract64(insn, 61, 3);
        unsigned dst = extract64(insn, 32, 8);

        r->last_insn[0] = low;
        r->last_insn[1] = high;

        if (cat == 0 && (insn & BIT_ULL(49))) {
            unsigned opc = extract64(insn, 55, 4);
            if (opc == 13) {
                pred_val = (r->pred_valid & BIT(0)) ? !!r->pred[0] : false;
                pred_mode = 1;
                continue;
            }
            if (opc == 14) {
                pred_val = (r->pred_valid & BIT(0)) ? !!r->pred[0] : false;
                pred_mode = 2;
                continue;
            }
            if (opc == 15) {
                pred_mode = 0;
                continue;
            }
        }
        if ((pred_mode == 1 && !pred_val) || (pred_mode == 2 && pred_val)) {
            continue;
        }

        if (cat == 0) {
            unsigned opc = extract64(insn, 55, 4);
            if (insn & BIT_ULL(49)) {
                return false;
            }
            if (opc == 0) { /* nop */
                continue;
            }
            if (opc == 6) { /* end */
                return true;
            }
            if (opc == 5) { /* kill */
                unsigned comp = extract64(insn, 53, 2);
                bool inv = !!(insn & BIT_ULL(52));
                bool cond = ((r->pred_valid & BIT(comp)) ? !!r->pred[comp] : false) ^ inv;
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
            if (opc == 1) { /* br/bany/ball */
                unsigned brtype = extract64(insn, 37, 3);
                unsigned c1 = extract64(insn, 53, 2);
                bool inv1 = !!(insn & BIT_ULL(52));
                bool p1 = ((r->pred_valid & BIT(c1)) ? !!r->pred[c1] : false) ^ inv1;
                bool take = false;
                if (brtype == 0 || brtype == 4 || brtype == 5) {
                    take = p1;
                } else if (brtype == 1 || brtype == 2) {
                    unsigned c2 = extract64(insn, 46, 2);
                    bool inv2 = !!(insn & BIT_ULL(45));
                    bool p2 = ((r->pred_valid & BIT(c2)) ? !!r->pred[c2] : false) ^ inv2;
                    take = (brtype == 1) ? (p1 || p2) : (p1 && p2);
                } else {
                    return false;
                }
                if (take) {
                    pc += (int32_t)low - 1;
                }
                continue;
            }
            return false;
        } else if (cat == 1) {
            unsigned src_type = extract64(insn, 50, 3);
            unsigned dst_type = extract64(insn, 46, 3);
            unsigned round_mode = extract64(insn, 55, 2);
            unsigned mode = extract64(insn, 53, 2);
            unsigned repeat = extract64(insn, 40, 2);
            unsigned sub = extract64(insn, 57, 2);
            bool source_half = (src_type == 0 || src_type == 2 ||
                                src_type == 4 || src_type == 6 || src_type == 7);
            bool dest_half = (dst_type == 0 || dst_type == 2 ||
                              dst_type == 4 || dst_type == 6 || dst_type == 7);

            if (sub == 2) {
                /* swz (repeat==0), gat (repeat==1), sct (repeat==2) */
                if (repeat == 0) {
                    uint32_t v0 = 0, v1 = 0;
                    unsigned s0 = low & 0xff;
                    unsigned s1 = (low >> 8) & 0xff;
                    unsigned d1 = (low >> 16) & 0xff;
                    if (!qcom845_ir3_read_reg(r, s0, !source_half, &v0) ||
                        !qcom845_ir3_read_reg(r, s1, !source_half, &v1)) {
                        return false;
                    }
                    v0 = qcom845_cat1_convert(v0, src_type, dst_type, round_mode);
                    v1 = qcom845_cat1_convert(v1, src_type, dst_type, round_mode);
                    if (!qcom845_ir3_write_reg(r, dst, dest_half, false, v0) ||
                        !qcom845_ir3_write_reg(r, d1, dest_half, false, v1)) {
                        return false;
                    }
                    continue;
                } else if (repeat == 1) {
                    uint32_t vals[4];
                    for (unsigned k = 0; k < 4; k++) {
                        unsigned sk = (low >> (k * 8)) & 0xff;
                        if (!qcom845_ir3_read_reg(r, sk, !source_half, &vals[k])) {
                            return false;
                        }
                        vals[k] = qcom845_cat1_convert(vals[k], src_type, dst_type,
                                                       round_mode);
                    }
                    for (unsigned k = 0; k < 4; k++) {
                        if (!qcom845_ir3_write_reg(r, dst + k, dest_half, false,
                                                   vals[k])) {
                            return false;
                        }
                    }
                    continue;
                } else if (repeat == 2) {
                    uint32_t vals[4];
                    for (unsigned k = 0; k < 4; k++) {
                        if (!qcom845_ir3_read_reg(r, (low & 0xff) + k,
                                                  !source_half, &vals[k])) {
                            return false;
                        }
                        vals[k] = qcom845_cat1_convert(vals[k], src_type, dst_type,
                                                       round_mode);
                    }
                    unsigned dregs[4] = {
                        dst, (low >> 8) & 0xff, (low >> 16) & 0xff,
                        (low >> 24) & 0xff,
                    };
                    for (unsigned k = 0; k < 4; k++) {
                        if (!qcom845_ir3_write_reg(r, dregs[k], dest_half, false,
                                                   vals[k])) {
                            return false;
                        }
                    }
                    continue;
                }
                return false;
            }
            if (sub != 0 || mode == 3) {
                return false;
            }
            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc = (repeat && (insn & BIT_ULL(43))) ? n : 0;
                uint32_t raw = 0;
                int index;

                if (mode == 2) {
                    raw = low;
                } else if (mode == 1 || (low & BIT(11))) {
                    if (mode == 1) {
                        index = (low & 0x7ff) + inc;
                    } else {
                        if (!r->a0_valid) {
                            return false;
                        }
                        index = r->a0 + sextract32(low, 0, 10) + (int)inc;
                    }
                    if (index < 0) {
                        return false;
                    }
                    if (mode == 1 || (low & BIT(10))) {
                        if (!qcom845_shader_const(s, blk, r, index, &raw)) {
                            return false;
                        }
                    } else if (!qcom845_ir3_read_reg(r, index, !source_half, &raw)) {
                        return false;
                    }
                } else {
                    if (!qcom845_ir3_read_reg(r, (low & 0xff) + inc,
                                              !source_half, &raw)) {
                        return false;
                    }
                }
                uint32_t out_val = qcom845_cat1_convert(raw, src_type, dst_type,
                                                        round_mode);
                if (!qcom845_ir3_write_reg(r, dst + n, dest_half, false, out_val)) {
                    return false;
                }
            }
        } else if (cat == 2) {
            unsigned opcode = extract64(insn, 53, 6);
            bool full = !!(insn & BIT_ULL(52));
            bool dst_conv = !!(insn & BIT_ULL(46));
            bool sat = !!(insn & BIT_ULL(42));
            unsigned repeat = extract64(insn, 40, 2);
            unsigned cond = extract64(insn, 48, 3);
            bool dest_half = (full == dst_conv) && dst <= 0xf7;
            bool is_float = (opcode <= 13 || opcode == 57);
            bool is_signed = (opcode == 17 || opcode == 19 || opcode == 21 ||
                              opcode == 23 || opcode == 25 || opcode == 26 ||
                              opcode == 34 || opcode == 49 || opcode == 52 ||
                              opcode == 56);
            unsigned kind = is_float ? 0 : (is_signed ? 1 : 2);
            bool one_src = (opcode == 4 || opcode == 6 ||
                            (opcode >= 9 && opcode <= 13) || opcode == 26 ||
                            opcode == 30 || opcode == 51 || opcode == 52 ||
                            opcode == 53 || opcode == 61);

            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc1 = (repeat && (insn & BIT_ULL(43))) ? n : 0;
                unsigned inc2 = (repeat && (insn & BIT_ULL(51))) ? n : 0;
                uint32_t a = 0, b = 0, value = 0;
                bool out_is_float = is_float && opcode != 5 && opcode != 7;

                if (opcode == 57) { /* bary.f */
                    unsigned input = (low & 0x7ff) + inc1;
                    if (blk != 12) {
                        return false;
                    }
                    if (dst + n >= 252 && dst + n <= 255) {
                        continue;
                    }
                    if (input >= 64 || !(r->input_valid & BIT_ULL(input))) {
                        return false;
                    }
                    value = r->input[input];
                } else {
                    if (!qcom845_ir3_multisrc(s, blk, r, low & 0xffff, inc1,
                                              full, kind, &a)) {
                        return false;
                    }
                    if (!one_src &&
                        !qcom845_ir3_multisrc(s, blk, r, low >> 16, inc2,
                                              full, kind, &b)) {
                        return false;
                    }
                    switch (opcode) {
                    case 0:
                        value = qcom845_f_to_u32(qcom845_u32_to_f(a) +
                                                 qcom845_u32_to_f(b));
                        break;
                    case 1: {
                        float fa = qcom845_u32_to_f(a), fb = qcom845_u32_to_f(b);
                        value = qcom845_f_to_u32(isnan(fa) ? fb : isnan(fb) ? fa : MIN(fa, fb));
                        break;
                    }
                    case 2: {
                        float fa = qcom845_u32_to_f(a), fb = qcom845_u32_to_f(b);
                        value = qcom845_f_to_u32(isnan(fa) ? fb : isnan(fb) ? fa : MAX(fa, fb));
                        break;
                    }
                    case 3:
                        value = qcom845_f_to_u32(qcom845_u32_to_f(a) *
                                                 qcom845_u32_to_f(b));
                        break;
                    case 4: {
                        float fa = qcom845_u32_to_f(a);
                        value = qcom845_f_to_u32(fa > 0.0f ? 1.0f : fa < 0.0f ? -1.0f : 0.0f);
                        break;
                    }
                    case 5:
                    case 7: {
                        float fa = qcom845_u32_to_f(a), fb = qcom845_u32_to_f(b);
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
                        break;
                    }
                    case 6: value = a; break;
                    case 9: value = qcom845_f_to_u32(floorf(qcom845_u32_to_f(a))); break;
                    case 10: value = qcom845_f_to_u32(ceilf(qcom845_u32_to_f(a))); break;
                    case 11: value = qcom845_f_to_u32(nearbyintf(qcom845_u32_to_f(a))); break;
                    case 12: value = qcom845_f_to_u32(roundf(qcom845_u32_to_f(a))); break;
                    case 13: value = qcom845_f_to_u32(truncf(qcom845_u32_to_f(a))); break;
                    case 16: case 17: value = a + b; break;
                    case 18: case 19: value = a - b; break;
                    case 20: case 33: {
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
                        break;
                    }
                    case 21: case 34: {
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
                        break;
                    }
                    case 22: value = MIN(a, b); break;
                    case 23: value = (uint32_t)MIN((int32_t)a, (int32_t)b); break;
                    case 24: value = MAX(a, b); break;
                    case 25: value = (uint32_t)MAX((int32_t)a, (int32_t)b); break;
                    case 26: value = a; break;
                    case 28: value = a & b; break;
                    case 29: value = a | b; break;
                    case 30: value = ~a; break;
                    case 31: value = a ^ b; break;
                    case 48: value = (a & 0xffffff) * (b & 0xffffff); break;
                    case 49: value = (uint32_t)(sextract32(a, 0, 24) * sextract32(b, 0, 24)); break;
                    case 50: value = (a & 0xffff) * (b & 0xffff); break;
                    case 51: value = revbit32(a); break;
                    case 52: case 53: value = a ? clz32(a) : 32; break;
                    case 54: value = a << (b & 31); break;
                    case 55: value = a >> (b & 31); break;
                    case 56: value = (uint32_t)((int32_t)a >> (b & 31)); break;
                    case 59: value = (a >> (b & 31)) & 1; break;
                    case 61: value = ctpop32(a); break;
                    default: return false;
                    }
                }
                if (out_is_float && sat) {
                    value = qcom845_sat_f32(value);
                }
                if (!qcom845_ir3_write_reg(r, dst + n, dest_half,
                                           out_is_float, value)) {
                    return false;
                }
            }
        } else if (cat == 3) {
            unsigned opcode = extract64(insn, 55, 4);
            unsigned repeat = extract64(insn, 40, 2);
            unsigned src2 = extract64(insn, 47, 8);
            bool dst_conv = !!(insn & BIT_ULL(46));
            bool sat = !!(insn & BIT_ULL(42));
            bool full = (opcode & 1) || opcode == 4;
            bool is_float = (opcode == 6 || opcode == 7 ||
                             opcode == 12 || opcode == 13);
            bool is_signed = (opcode == 2 || opcode == 3 || opcode == 5 ||
                              opcode == 10 || opcode == 11 ||
                              opcode == 14 || opcode == 15);
            unsigned kind = is_float ? 0 : (is_signed ? 1 : 2);
            bool dest_half = (full == dst_conv) && dst <= 0xf7;

            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc1 = (repeat && (insn & BIT_ULL(43))) ? n : 0;
                unsigned inc2 = (repeat && (insn & BIT_ULL(15))) ? n : 0;
                unsigned inc3 = (repeat && (insn & BIT_ULL(29))) ? n : 0;
                uint32_t a = 0, b = 0, c = 0, value = 0;
                bool c_full = full || opcode <= 3;

                if (!qcom845_ir3_cat3_src(s, blk, r, low & 0x1fff,
                        !!(insn & BIT_ULL(14)), inc1, full, kind, &a) ||
                    !qcom845_ir3_cat3_src(s, blk, r, src2,
                        !!(insn & BIT_ULL(30)), inc2, full, kind, &b) ||
                    !qcom845_ir3_cat3_src(s, blk, r, extract32(low, 16, 13),
                        !!(insn & BIT_ULL(31)), inc3, c_full, kind, &c)) {
                    return false;
                }
                switch (opcode) {
                case 0: value = (a & 0xffff) * (b & 0xffff) + c; break;
                case 1: value = (((a & 0xffff) * (b & 0xffff)) << 16) + c; break;
                case 2: value = (uint32_t)((int32_t)(int16_t)a * (int32_t)(int16_t)b + (int32_t)c); break;
                case 3: value = ((((a >> 16) & 0xffff) * (b & 0xffff)) << 16) + c; break;
                case 4: value = (a & 0xffffff) * (b & 0xffffff) + c; break;
                case 5: value = (uint32_t)(sextract32(a, 0, 24) * sextract32(b, 0, 24) + (int32_t)c); break;
                case 6: case 7:
                    value = qcom845_f_to_u32(qcom845_u32_to_f(a) *
                                             qcom845_u32_to_f(b) +
                                             qcom845_u32_to_f(c));
                    break;
                case 8: case 9: case 10: case 11:
                    value = b ? a : c;
                    break;
                case 12: case 13: {
                    float fb = qcom845_u32_to_f(b);
                    value = (!isnan(fb) && fb != 0.0f) ? a : c;
                    break;
                }
                default: return false;
                }
                if (is_float && sat) {
                    value = qcom845_sat_f32(value);
                }
                if (!qcom845_ir3_write_reg(r, dst + n, dest_half, is_float, value)) {
                    return false;
                }
            }
        } else if (cat == 4) {
            unsigned opcode = extract64(insn, 53, 6);
            bool full = !!(insn & BIT_ULL(52));
            bool dst_conv = !!(insn & BIT_ULL(46));
            bool sat = !!(insn & BIT_ULL(42));
            unsigned repeat = extract64(insn, 40, 2);
            bool dest_half = (full == dst_conv) && dst <= 0xf7;

            for (unsigned n = 0; n <= repeat; n++) {
                unsigned inc1 = (repeat && (insn & BIT_ULL(43))) ? n : 0;
                uint32_t a = 0;
                if (!qcom845_ir3_multisrc(s, blk, r, low & 0xffff, inc1,
                                          full, 0, &a)) {
                    return false;
                }
                float fa = qcom845_u32_to_f(a), res;
                switch (opcode) {
                case 0: res = 1.0f / fa; break;
                case 1: case 9: res = 1.0f / sqrtf(fa); break;
                case 2: case 10: res = log2f(fa); break;
                case 3: case 11: res = exp2f(fa); break;
                case 4: res = sinf(fa); break;
                case 5: res = cosf(fa); break;
                case 6: res = sqrtf(fa); break;
                default: return false;
                }
                uint32_t val = qcom845_f_to_u32(res);
                if (sat) {
                    val = qcom845_sat_f32(val);
                }
                if (!qcom845_ir3_write_reg(r, dst + n, dest_half, true, val)) {
                    return false;
                }
            }
        } else if (cat == 5) {
            if (!qcom845_ir3_cat5(s, blk, r, insn)) {
                return false;
            }
        } else if (cat == 6) {
            if (!qcom845_ir3_cat6(s, blk, r, insn)) {
                return false;
            }
        } else {
            return false;
        }
    }
    return false;
}

/* ---- VFD Vertex Fetch & VS Output Extraction ---- */

static bool qcom845_vertex_fetch_and_run(Qcom845Display *s, unsigned index,
                                         unsigned instance, bool raw_id,
                                         Qcom845ShaderRegs *r)
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    unsigned fetch_cnt = raw_id ? 0 : (regs[A6XX_VFD_CONTROL_0] & 0x3f);
    unsigned vtx_reg = regs[A6XX_VFD_CONTROL_1] & 0xff;
    unsigned inst_reg = (regs[A6XX_VFD_CONTROL_1] >> 8) & 0xff;
    uint32_t idx_off = raw_id ? 0 : regs[A6XX_VFD_INDEX_OFFSET];
    uint32_t inst_off = raw_id ? 0 : regs[A6XX_VFD_INSTANCE_START_OFFSET];
    float_status fp = { 0 };

    if (fetch_cnt > 32) {
        return false;
    }
    qcom845_shader_reset_pixel(r);
    if (vtx_reg != 0xfc && vtx_reg < A6XX_SHADER_GPRS) {
        r->full[vtx_reg] = index + idx_off;
        qcom845_bit_set(r->full_valid, vtx_reg);
    }
    if (inst_reg != 0xfc && inst_reg < A6XX_SHADER_GPRS) {
        r->full[inst_reg] = instance + inst_off;
        qcom845_bit_set(r->full_valid, inst_reg);
    }

    for (unsigned i = 0; i < fetch_cnt; i++) {
        uint32_t decode = regs[A6XX_VFD_DECODE_0_INSTR + i * 2];
        uint32_t step_rate = regs[A6XX_VFD_DECODE_0_INSTR + i * 2 + 1];
        uint32_t dest = regs[A6XX_VFD_DEST_CNTL_0_INSTR + i];
        unsigned wrmask = dest & 0xf;
        unsigned out = extract32(dest, 4, 8);

        if (!wrmask) {
            continue;
        }
        if (out > A6XX_SHADER_GPRS - 4) {
            return false;
        }
        unsigned buf_idx = decode & 0x1f;
        unsigned offset = extract32(decode, 5, 12);
        bool instanced = !!(decode & BIT(17));
        unsigned format = extract32(decode, 20, 8);
        unsigned swap = extract32(decode, 28, 2);
        unsigned fetch_reg = A6XX_VFD_FETCH_0_BASE_LO + buf_idx * 4;
        uint64_t base = qcom845_reg64(regs, fetch_reg);
        uint32_t buf_size = regs[fetch_reg + 2];
        uint32_t stride = regs[fetch_reg + 3];
        unsigned components, bytes;
        bool is_float = true;
        uint32_t value[4] = { 0, 0, 0, 0x3f800000 };

        if (format == 0x82 || format == 0x7f || format == 0x74 ||
            format == 0x67 || format == 0x4a) {
            components = (format == 0x82 || format == 0x7f) ? 4
                       : (format == 0x74) ? 3 : (format == 0x67) ? 2 : 1;
            bytes = components * 4;
        } else if (format == 0x48 || format == 0x49 || format == 0x4b ||
                   format == 0x4c || format == 0x65 || format == 0x66 ||
                   format == 0x68 || format == 0x69 || format == 0x72 ||
                   format == 0x73 || format == 0x75 || format == 0x76 ||
                   format == 0x80 || format == 0x81) {
            components = (format <= 0x4c) ? 1 : (format <= 0x69) ? 2
                       : (format <= 0x76) ? 3 : 4;
            bytes = components * 4;
            is_float = false;
            value[3] = 1;
        } else if (format >= 0x43 && format <= 0x47) {
            components = 2;
            bytes = 4;
            is_float = (format <= 0x45);
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format >= 0x60 && format <= 0x64) {
            components = 4;
            bytes = 8;
            is_float = (format <= 0x62);
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format == 0x30 || format == 0x32 ||
                   format == 0x33 || format == 0x34) {
            components = 4;
            bytes = 4;
            is_float = (format <= 0x32);
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format >= 0x03 && format <= 0x06) {
            components = 1;
            bytes = 1;
            is_float = (format <= 0x04);
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format >= 0x0f && format <= 0x12) {
            components = 2;
            bytes = 2;
            is_float = (format <= 0x10);
            if (!is_float) {
                value[3] = 1;
            }
        } else if (format >= 0x15 && format <= 0x19) {
            components = 1;
            bytes = 2;
            is_float = (format <= 0x17);
            if (!is_float) {
                value[3] = 1;
            }
        } else {
            trace_qcom845_render_reject("vfd_fmt", format, decode);
            return false;
        }

        uint32_t elem_idx = instanced
            ? ((step_rate ? instance / step_rate : 0) + inst_off)
            : (index + idx_off);
        uint64_t pos = (uint64_t)elem_idx * stride + offset;
        if (pos + bytes > buf_size) {
            trace_qcom845_render_reject("vfd_bounds", pos + bytes, buf_size);
            return false;
        }
        uint64_t addr = base + pos;

        if ((format >= 0x03 && format <= 0x06) ||
            (format >= 0x0f && format <= 0x12) ||
            format == 0x30 || format == 0x32 ||
            format == 0x33 || format == 0x34) {
            uint8_t raw[4] = { 0 };
            unsigned kind;
            if (!qcom845_cached_read(s, r->read_cache, addr, raw, bytes)) {
                return false;
            }
            if (bytes == 4) {
                uint32_t packed = qcom845_color_swap(ldl_le_p(raw), swap);
                stl_le_p(raw, packed);
                kind = (format == 0x30) ? 0 : (format == 0x32) ? 1
                     : (format == 0x33) ? 2 : 3;
            } else {
                kind = (format >= 0x0f) ? (format - 0x0f) : (format - 0x03);
            }
            for (unsigned c = 0; c < components; c++) {
                uint8_t b = raw[c];
                if (kind == 0) {
                    value[c] = qcom845_f_to_u32((float)b / 255.0f);
                } else if (kind == 1) {
                    int8_t sb = (int8_t)b;
                    value[c] = qcom845_f_to_u32(sb == INT8_MIN ? -1.0f : sb / 127.0f);
                } else if (kind == 2) {
                    value[c] = b;
                } else {
                    value[c] = (uint32_t)(int32_t)(int8_t)b;
                }
            }
        } else if ((format >= 0x15 && format <= 0x19) ||
                   (format >= 0x43 && format <= 0x47) ||
                   (format >= 0x60 && format <= 0x64)) {
            uint8_t raw[8] = { 0 };
            unsigned bfmt = (format >= 0x60) ? (format - 0x60)
                          : (format >= 0x43) ? (format - 0x43)
                                             : (format - 0x15);
            if (!qcom845_cached_read(s, r->read_cache, addr, raw, bytes)) {
                return false;
            }
            for (unsigned c = 0; c < components; c++) {
                uint16_t rv = lduw_le_p(raw + c * 2);
                if (bfmt == 3) {
                    value[c] = rv;
                } else if (bfmt == 4) {
                    value[c] = (uint32_t)(int32_t)(int16_t)rv;
                } else if (bfmt == 2) {
                    value[c] = qcom845_f16_to_f32(rv);
                } else if (bfmt == 0) {
                    value[c] = float32_div(uint32_to_float32(rv, &fp),
                                           0x477fff00, &fp);
                } else {
                    int16_t sv = (int16_t)rv;
                    value[c] = (sv == INT16_MIN) ? 0xbf800000
                             : float32_div(int32_to_float32(sv, &fp),
                                           0x46fffe00, &fp);
                }
            }
        } else {
            uint8_t raw[16];
            if (!qcom845_cached_read(s, r->read_cache, addr, raw, bytes)) {
                return false;
            }
            for (unsigned c = 0; c < components; c++) {
                value[c] = ldl_le_p(raw + c * 4);
            }
        }

        for (unsigned c = 0; c < 4; c++) {
            if (wrmask & BIT(c)) {
                r->full[out + c] = value[c];
                qcom845_bit_set(r->full_valid, out + c);
                r->half[out + c] = is_float ? qcom845_f32_to_f16(value[c])
                                            : (uint16_t)value[c];
                qcom845_bit_set(r->half_valid, out + c);
            }
        }
    }

    if (!qcom845_shader_execute(s, 8, r)) {
        trace_qcom845_render_reject("vs_exec", r->last_insn[0], r->last_insn[1]);
        return false;
    }
    return true;
}

static bool qcom845_extract_vertex(Qcom845Display *s,
                                   const Qcom845ShaderRegs *r,
                                   bool screen_space,
                                   Qcom845RasterVertex *v)
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    unsigned out_cnt = regs[A6XX_SP_VS_OUT_CNT] & 0x3f;
    unsigned pos_reg = A6XX_SHADER_GPRS;
    float_status fp = { 0 };

    if (!out_cnt || out_cnt > 32) {
        return false;
    }
    memset(v, 0, sizeof(*v));

    /* Scan SP_VS_OUT_REG / SP_VS_VPC_DST_REG: loc==0 is position, others varyings. */
    for (unsigned k = 0; k < out_cnt; k++) {
        uint32_t out_reg = regs[A6XX_SP_VS_OUT_REG_0 + k / 2];
        unsigned regid = extract32(out_reg, (k & 1) ? 16 : 0, 8);
        unsigned mask = extract32(out_reg, (k & 1) ? 24 : 8, 4);
        unsigned loc = extract32(regs[A6XX_SP_VS_VPC_DST_REG_0 + k / 4],
                                 (k & 3) * 8, 8);

        if (loc == 0 && pos_reg == A6XX_SHADER_GPRS) {
            pos_reg = regid;
            continue;
        }
        for (unsigned b = 0; b < 4; b++) {
            if (mask & BIT(b)) {
                if (regid + b >= A6XX_SHADER_GPRS || loc >= 64 ||
                    !qcom845_bit_test(r->full_valid, regid + b)) {
                    trace_qcom845_render_reject("vs_varying", regid + b, loc);
                    return false;
                }
                v->varyings[loc] = r->full[regid + b];
                v->varying_mask |= BIT_ULL(loc);
                loc++;
            }
        }
    }

    if (pos_reg > A6XX_SHADER_GPRS - 4) {
        trace_qcom845_render_reject("vs_no_pos", out_cnt, regs[A6XX_SP_VS_OUT_REG_0]);
        return false;
    }
    for (unsigned c = 0; c < 4; c++) {
        if (!qcom845_bit_test(r->full_valid, pos_reg + c)) {
            trace_qcom845_render_reject("vs_pos_invalid", pos_reg + c, 0);
            return false;
        }
    }

    if (screen_space) {
        v->inv_w = 1.0f;
        float fx = qcom845_u32_to_f(r->full[pos_reg]);
        float fy = qcom845_u32_to_f(r->full[pos_reg + 1]);
        if (!isfinite(fx) || !isfinite(fy)) {
            v->clipped = true;
            return true;
        }
        v->x = (int)lrintf(fx * 256.0f);
        v->y = (int)lrintf(fy * 256.0f);
        return true;
    }

    float w = qcom845_u32_to_f(r->full[pos_reg + 3]);
    if (!isfinite(w) || w == 0.0f) {
        v->clipped = true;
        return true;
    }
    float inv_w = 1.0f / w;
    v->inv_w = inv_w;

    for (unsigned c = 0; c < 2; c++) {
        uint32_t offset = regs[A6XX_GRAS_CL_VPORT_XOFFSET_0 + c * 2];
        uint32_t scale = regs[A6XX_GRAS_CL_VPORT_XOFFSET_0 + c * 2 + 1];
        uint32_t ndc = (w == 1.0f) ? r->full[pos_reg + c]
                     : qcom845_f_to_u32(qcom845_u32_to_f(r->full[pos_reg + c]) * inv_w);
        if (!qcom845_f32_finite(ndc) || !qcom845_f32_finite(offset) ||
            !qcom845_f32_finite(scale)) {
            v->clipped = true;
            return true;
        }
        uint32_t coord = float32_add(float32_mul(ndc, scale, &fp), offset, &fp);
        if (!qcom845_f32_finite(coord)) {
            v->clipped = true;
            return true;
        }
        int fixed = float32_to_int32_scalbn(coord, float_round_nearest_even,
                                            8, &fp);
        if (c) {
            v->y = fixed;
        } else {
            v->x = fixed;
        }
    }
    return true;
}

/* ---- Blending & Pixel Output ---- */

static void qcom845_blend_prepare(const uint32_t *regs, Qcom845BlendState *b)
{
    uint32_t ctrl = regs[A6XX_RB_MRT_0_CONTROL];
    uint32_t bc = regs[A6XX_RB_MRT_0_BLEND_CONTROL];

    b->blend_color = !!(ctrl & BIT(0));
    b->blend_alpha = !!(ctrl & BIT(1));
    b->comp_mask = extract32(ctrl, 7, 4);
    b->rgb_src = extract32(bc, 0, 5);
    b->rgb_op = extract32(bc, 5, 3);
    b->rgb_dst = extract32(bc, 8, 5);
    b->a_src = extract32(bc, 16, 5);
    b->a_op = extract32(bc, 21, 3);
    b->a_dst = extract32(bc, 24, 5);
    for (unsigned c = 0; c < 4; c++) {
        b->cst[c] = qcom845_u32_to_f(regs[A6XX_RB_BLEND_CONSTANT_RED_FP32 + c]);
    }
}

static float qcom845_blend_factor(unsigned factor, unsigned c,
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
    case 16: return (c == 3) ? 1.0f : MIN(src[3], 1.0f - dst[3]);
    default: return 1.0f;
    }
}

static float qcom845_blend_op(unsigned op, float s_term, float d_term)
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

static uint32_t qcom845_blend_pixel(const Qcom845BlendState *b,
                                    const float src[4], uint32_t dst_rgba)
{
    float dst[4] = {
        (float)(dst_rgba & 0xff) * (1.0f / 255.0f),
        (float)((dst_rgba >> 8) & 0xff) * (1.0f / 255.0f),
        (float)((dst_rgba >> 16) & 0xff) * (1.0f / 255.0f),
        (float)((dst_rgba >> 24) & 0xff) * (1.0f / 255.0f),
    };
    uint32_t out = 0;

    for (unsigned c = 0; c < 4; c++) {
        float v;
        if (!(b->comp_mask & BIT(c))) {
            v = dst[c];
        } else if (c < 3 ? b->blend_color : b->blend_alpha) {
            unsigned sf = (c < 3) ? b->rgb_src : b->a_src;
            unsigned op = (c < 3) ? b->rgb_op : b->a_op;
            unsigned df = (c < 3) ? b->rgb_dst : b->a_dst;
            float st = src[c] * qcom845_blend_factor(sf, c, src, dst, b->cst);
            float dt = dst[c] * qcom845_blend_factor(df, c, src, dst, b->cst);
            v = qcom845_blend_op(op, st, dt);
        } else {
            v = src[c];
        }
        v = isnan(v) ? 0.0f : CLAMP(v, 0.0f, 1.0f);
        unsigned q = (unsigned)lrintf(v * 255.0f);
        out |= (q & 0xffU) << (c * 8);
    }
    return out;
}

static bool qcom845_fragment_eval(Qcom845Display *s, Qcom845ShaderRegs *r,
                                  unsigned fmt, float src[4])
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    uint32_t out_reg = regs[A6XX_SP_FS_OUTPUT_REG_0];
    unsigned regid = out_reg & 0xff;
    bool half = !!(out_reg & BIT(8));

    if (!qcom845_fs_prefetch(s, r)) {
        return false;
    }
    if (!qcom845_shader_execute(s, 12, r)) {
        trace_qcom845_render_reject("fs_exec", r->last_insn[0], r->last_insn[1]);
        return false;
    }
    if (r->killed) {
        return true;
    }

    unsigned ncomp = (fmt == 0x30) ? 4 : (fmt == 0x0f) ? 2 : 1;
    src[0] = src[1] = src[2] = 0.0f;
    src[3] = 1.0f;
    for (unsigned c = 0; c < ncomp; c++) {
        uint32_t raw = 0;
        /* A8_UNORM (0x02): FS writes alpha into component 3 (or 0 if only 0 valid). */
        unsigned idx = (fmt == 0x02) ? (regid + 3) : (regid + c);
        if (fmt == 0x02 && !qcom845_ir3_read_reg(r, idx, !half, &raw)) {
            idx = regid;
        }
        if (!qcom845_ir3_read_reg(r, idx, !half, &raw)) {
            raw = 0;
        }
        float f = half ? qcom845_u32_to_f(qcom845_f16_to_f32((uint16_t)raw))
                       : qcom845_u32_to_f(raw);
        if (fmt == 0x02) {
            src[3] = f;
        } else {
            src[c] = f;
        }
    }
    return true;
}

static inline uint32_t qcom845_write_mrt_pixel(uint8_t *ppage, unsigned off,
                                               unsigned fmt, unsigned cpp,
                                               unsigned dst_swap,
                                               bool read_dst,
                                               const Qcom845BlendState *blend,
                                               const float src[4])
{
    uint32_t dst_rgba = 0;
    if (read_dst) {
        if (cpp == 1) {
            uint8_t pv = ppage[off];
            dst_rgba = (fmt == 0x02) ? ((uint32_t)pv << 24) : pv;
        } else if (cpp == 2) {
            dst_rgba = lduw_le_p(ppage + off);
        } else {
            dst_rgba = qcom845_color_unswap(ldl_le_p(ppage + off), dst_swap);
        }
    }
    uint32_t out_rgba = qcom845_blend_pixel(blend, src, dst_rgba);
    if (cpp == 1) {
        ppage[off] = (fmt == 0x02) ? ((out_rgba >> 24) & 0xff)
                                   : (out_rgba & 0xff);
    } else if (cpp == 2) {
        stw_le_p(ppage + off, out_rgba & 0xffff);
    } else {
        stl_le_p(ppage + off, qcom845_color_swap(out_rgba, dst_swap));
    }
    return out_rgba;
}

static inline int64_t qcom845_edge(const Qcom845RasterVertex *a,
                                   const Qcom845RasterVertex *b, int x, int y)
{
    return (int64_t)(b->x - a->x) * (y - a->y) -
           (int64_t)(b->y - a->y) * (x - a->x);
}

static inline bool qcom845_top_left(const Qcom845RasterVertex *a,
                                    const Qcom845RasterVertex *b)
{
    return b->y < a->y || (b->y == a->y && b->x > a->x);
}

/* ---- Main Entrypoint: CP_DRAW_INDX_OFFSET (0x38) ---- */

bool qcom845_render_draw(Qcom845Display *s, uint32_t n, const uint32_t *w)
{
    const uint32_t *regs = s->banks[BANK845_ADRENO].regs;
    if (n < 3) {
        return false;
    }

    /*
     * RM6_BIN_VISIBILITY (marker low nibble == 2) is a vertex-only binning
     * visibility pass and must not rasterize into color targets.
     */
    if ((s->cp_marker & 0xf) == 2) {
        return true;
    }

    unsigned prim = w[0] & 0x3f;
    unsigned inst_cnt = w[1];
    unsigned vtx_cnt = w[2];
    bool indexed = (n >= 7);
    unsigned idx_size = extract32(w[0], 10, 2);
    unsigned idx_bytes = (idx_size == 2) ? 4 : 2;
    bool is_rect = (prim == 8);

    if (!inst_cnt || inst_cnt > 4096 || !vtx_cnt || vtx_cnt > 65536) {
        return false;
    }
    if (!is_rect && prim != 4 && prim != 5 && prim != 6) {
        trace_qcom845_render_reject("prim", prim, w[0]);
        return false;
    }

    uint32_t mrt_info = regs[A6XX_RB_MRT_0_BUF_INFO];
    unsigned fmt = mrt_info & 0xff;
    unsigned tile = extract32(mrt_info, 8, 2);
    unsigned dst_swap = extract32(mrt_info, 13, 2);
    unsigned cpp = (fmt == 0x30) ? 4 : (fmt == 0x0f) ? 2
                 : (fmt == 0x02 || fmt == 0x03) ? 1 : 0;
    uint64_t pitch = (uint64_t)regs[A6XX_RB_MRT_0_PITCH] * 64;
    uint64_t layer = (uint64_t)regs[A6XX_RB_MRT_0_ARRAY_PITCH] * 64;
    uint64_t dest = qcom845_reg64(regs, A6XX_RB_MRT_0_BASE_LO);

    /*
     * Accept linear SYSMEM (tile == 0) as well as A630 GMEM bin-render passes
     * (tile == 2 / TILE6_2, marker low nibble == 4). During GMEM binning,
     * RB_MRT_0_BASE can remain stale across swapchain flips because hardware
     * writes to RB_MRT_0_BASE_GMEM and resolves via RB_RESOLVE_SYSTEM_BUFFER_*
     * in each bin's LOAD/STORE IB; prefer the linear resolve backing surface
     * when programmed, falling back to RB_MRT_0_BASE.
     */
    bool gmem_bin = (tile == 2 && (s->cp_marker & 0xf) == 4);
    if (gmem_bin) {
        uint64_t rdest = qcom845_reg64(regs,
                                       A6XX_RB_RESOLVE_SYSTEM_BUFFER_BASE_LO);
        uint32_t rinfo = regs[A6XX_RB_RESOLVE_SYSTEM_BUFFER_INFO];
        uint64_t rpitch =
            (uint64_t)(regs[A6XX_RB_RESOLVE_SYSTEM_BUFFER_PITCH] & 0xffff) * 64;

        if (rdest && (rinfo & 3) == 0 && rpitch) {
            dest = rdest;
            pitch = rpitch;
            layer = (uint64_t)(regs[A6XX_RB_RESOLVE_SYSTEM_BUFFER_ARRAY_PITCH] &
                               0x1fffffff) * 64;
            fmt = extract32(rinfo, 7, 8);
            dst_swap = extract32(rinfo, 5, 2);
            cpp = (fmt == 0x30) ? 4 : (fmt == 0x0f) ? 2
                : (fmt == 0x02 || fmt == 0x03) ? 1 : 0;
        }
    }
    if (!cpp || (tile != 0 && !gmem_bin) || !pitch || !dest) {
        trace_qcom845_render_reject("mrt_cfg", mrt_info, pitch);
        return false;
    }

    /* Compute clipped target bounds from scissor registers and MRT pitch/layer. */
    unsigned x1 = 0, y1 = 0;
    unsigned x2 = MIN((unsigned)(pitch / cpp), A6XX_MAX_OUTPUT_DIM);
    unsigned y2 = layer ? MIN((unsigned)(layer / pitch), A6XX_MAX_OUTPUT_DIM)
                        : A6XX_MAX_OUTPUT_DIM;

    static const unsigned sc_regs[3] = {
        A6XX_GRAS_SC_SCREEN_SCISSOR_TL_0,
        A6XX_GRAS_SC_VPORT_SCISSOR_TL_0,
        A6XX_GRAS_SC_WINDOW_SCISSOR_TL,
    };
    for (unsigned i = 0; i < 3; i++) {
        uint32_t tl = regs[sc_regs[i]];
        uint32_t br = regs[sc_regs[i] + 1];
        if (tl || br) {
            x1 = MAX(x1, tl & 0xffff);
            y1 = MAX(y1, tl >> 16);
            x2 = MIN(x2, (br & 0xffff) + 1);
            y2 = MIN(y2, (br >> 16) + 1);
        }
    }
    if (x1 >= x2 || y1 >= y2) {
        return true;
    }

    g_autoptr(GHashTable) read_tbl = g_hash_table_new(g_direct_hash,
                                                      g_direct_equal);
    g_autoptr(GHashTable) write_tbl = g_hash_table_new_full(g_direct_hash,
                                                            g_direct_equal,
                                                            NULL, g_free);
    Qcom845PageCache read_cache, write_cache;
    Qcom845ShaderProgram vs_prog, fs_prog;
    Qcom845BlendState blend;

    qcom845_page_cache_init(&read_cache, read_tbl);
    qcom845_page_cache_init(&write_cache, write_tbl);
    qcom845_blend_prepare(regs, &blend);
    bool read_dst = blend.blend_color || blend.blend_alpha ||
                    blend.comp_mask != 0xf;

    qcom845_shader_prepare(s, 8, qcom845_reg64(regs, A6XX_SP_VS_OBJ_START_LO),
                           &read_cache, &vs_prog);
    qcom845_shader_prepare(s, 12, qcom845_reg64(regs, A6XX_SP_FS_OBJ_START_LO),
                           &read_cache, &fs_prog);

    if (is_rect) {
        /* DI_PT_RECTLIST: 2 vertices defining screen-space axis-aligned rectangle. */
        if (vtx_cnt < 2) {
            return false;
        }
        bool screen_space = !!(regs[A6XX_GRAS_CL_CNTL] & BIT(0));
        Qcom845RasterVertex rv[2];
        Qcom845ShaderRegs sh;

        for (unsigned v = 0; v < 2; v++) {
            sh.read_cache = &read_cache;
            sh.program = &vs_prog;
            if (!qcom845_vertex_fetch_and_run(s, v, 0, true, &sh) ||
                !qcom845_extract_vertex(s, &sh, screen_space, &rv[v])) {
                return false;
            }
        }
        if (rv[0].clipped || rv[1].clipped) {
            return true;
        }
        int rx0 = rv[0].x / 256, ry0 = rv[0].y / 256;
        int rx1 = rv[1].x / 256, ry1 = rv[1].y / 256;
        int bx1 = MAX((int)x1, MIN(rx0, rx1));
        int by1 = MAX((int)y1, MIN(ry0, ry1));
        int bx2 = MIN((int)x2, MAX(rx0, rx1));
        int by2 = MIN((int)y2, MAX(ry0, ry1));
        if (bx1 >= bx2 || by1 >= by2 || rx0 == rx1 || ry0 == ry1) {
            return true;
        }

        uint64_t vmask = rv[0].varying_mask & rv[1].varying_mask;
        bool uniform = true;
        for (uint64_t m = vmask; m; m &= m - 1) {
            unsigned slot = ctz64(m);
            if (rv[0].varyings[slot] != rv[1].varyings[slot]) {
                uniform = false;
                break;
            }
        }

        float inv_dx = 1.0f / (float)(rx1 - rx0);
        float inv_dy = 1.0f / (float)(ry1 - ry0);
        float u_src[4] = { 0, 0, 0, 1 };

        if (uniform) {
            qcom845_shader_reset_pixel(&sh);
            sh.read_cache = &read_cache;
            sh.program = &fs_prog;
            for (uint64_t m = vmask; m; m &= m - 1) {
                unsigned slot = ctz64(m);
                sh.input[slot] = rv[0].varyings[slot];
                sh.flat_input[slot] = rv[0].varyings[slot];
            }
            sh.input_valid = vmask;
            sh.flat_valid = vmask;
            if (!qcom845_fragment_eval(s, &sh, fmt, u_src)) {
                return false;
            }
            if (sh.killed) {
                return true;
            }
        }

        uint32_t rect_nonblack = 0;
        for (int y = by1; y < by2; y++) {
            float ty = ((float)y + 0.5f - (float)ry0) * inv_dy;
            for (int x = bx1; x < bx2; x++) {
                float src[4];
                if (uniform) {
                    memcpy(src, u_src, sizeof(src));
                } else {
                    float tx = ((float)x + 0.5f - (float)rx0) * inv_dx;
                    qcom845_shader_reset_pixel(&sh);
                    sh.read_cache = &read_cache;
                    sh.program = &fs_prog;
                    for (uint64_t m = vmask; m; m &= m - 1) {
                        unsigned slot = ctz64(m);
                        float v0 = qcom845_u32_to_f(rv[0].varyings[slot]);
                        float v1 = qcom845_u32_to_f(rv[1].varyings[slot]);
                        /* Rect varying pairs: even slots interpolate along X, odd along Y. */
                        float t = (slot & 1) ? ty : tx;
                        sh.input[slot] = qcom845_f_to_u32(v0 + (v1 - v0) * t);
                        sh.flat_input[slot] = rv[0].varyings[slot];
                    }
                    sh.input_valid = vmask;
                    sh.flat_valid = vmask;
                    if (!qcom845_fragment_eval(s, &sh, fmt, src)) {
                        return false;
                    }
                    if (sh.killed) {
                        continue;
                    }
                }
                uint64_t paddr = dest + (uint64_t)y * pitch + (uint64_t)x * cpp;
                uint8_t *ppage = qcom845_stage_write_page(s, &write_cache, paddr);
                if (!ppage) {
                    return false;
                }
                uint32_t px = qcom845_write_mrt_pixel(ppage, paddr & A6XX_PAGE_MASK,
                                                      fmt, cpp, dst_swap, read_dst,
                                                      &blend, src);
                if (px & 0x00ffffffU) {
                    rect_nonblack++;
                }
            }
        }
        if (!qcom845_publish_write_pages(write_tbl)) {
            return false;
        }
        hwaddr dest_pa = dest;
        if (s->gpu_smmu) {
            qcom845_smmu_translate(s->gpu_smmu, 0, dest, false, &dest_pa);
        }
        trace_qcom845_render_done(dest, dest_pa, fmt,
                                  (uint32_t)(bx2 - bx1) * (by2 - by1),
                                  rect_nonblack);
        return true;
    }

    /* Triangle primitives: DI_PT_TRILIST (4), TRIFAN (5), TRISTRIP (6). */
    uint64_t idx_addr = 0;
    bool restart_en = !!(regs[A6XX_PC_PRIMITIVE_CNTL_0] & BIT(0));
    uint32_t restart_idx = (idx_size == 2) ? regs[A6XX_PC_RESTART_INDEX]
                                           : (regs[A6XX_PC_RESTART_INDEX] & 0xffff);
    if (indexed) {
        uint64_t base_ib = (uint64_t)w[4] | ((uint64_t)w[5] << 32);
        if ((uint64_t)w[3] + vtx_cnt > w[6]) {
            trace_qcom845_render_reject("ib_bounds", (uint64_t)w[3] + vtx_cnt, w[6]);
            return false;
        }
        idx_addr = base_ib + (uint64_t)w[3] * idx_bytes;
    }

    bool screen_space = !!(regs[A6XX_GRAS_CL_CNTL] & BIT(0));
    uint32_t su_cntl = regs[A6XX_GRAS_SU_CNTL];
    uint64_t raster_budget = 64ULL << 20;
    uint32_t total_pixels = 0;
    uint32_t total_nonblack = 0;

    for (unsigned inst = 0; inst < inst_cnt; inst++) {
        unsigned strip_len = 0, strip_parity = 0;
        unsigned strip_idx[2] = { 0, 0 };
        unsigned fan_first = 0;
        unsigned step = (prim == 4) ? 3 : 1;

        for (unsigned pos = 0; pos < vtx_cnt; pos += step) {
            unsigned tri_idx[3];
            if (prim == 4) {
                if (pos + 2 >= vtx_cnt) {
                    break;
                }
                bool skip = false;
                for (unsigned v = 0; v < 3; v++) {
                    if (indexed) {
                        uint64_t ia = idx_addr + (uint64_t)(pos + v) * idx_bytes;
                        uint8_t ib[4] = { 0 };
                        if (!qcom845_cached_read(s, &read_cache, ia, ib, idx_bytes)) {
                            return false;
                        }
                        tri_idx[v] = (idx_bytes == 4) ? ldl_le_p(ib) : lduw_le_p(ib);
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
                    uint64_t ia = idx_addr + (uint64_t)pos * idx_bytes;
                    uint8_t ib[4] = { 0 };
                    if (!qcom845_cached_read(s, &read_cache, ia, ib, idx_bytes)) {
                        return false;
                    }
                    cur_idx = (idx_bytes == 4) ? ldl_le_p(ib) : lduw_le_p(ib);
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
                }
                if (strip_len == 1) {
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

            Qcom845RasterVertex rv[3];
            Qcom845ShaderRegs sh;
            for (unsigned v = 0; v < 3; v++) {
                sh.read_cache = &read_cache;
                sh.program = &vs_prog;
                if (!qcom845_vertex_fetch_and_run(s, tri_idx[v], inst, false, &sh) ||
                    !qcom845_extract_vertex(s, &sh, screen_space, &rv[v])) {
                    return false;
                }
            }
            if (rv[0].clipped || rv[1].clipped || rv[2].clipped) {
                continue;
            }
            int64_t area = qcom845_edge(&rv[0], &rv[1], rv[2].x, rv[2].y);
            if (!area) {
                continue;
            }
            if (su_cntl & 3) {
                bool is_cw = area > 0;
                bool is_front = (su_cntl & BIT(2)) ? is_cw : !is_cw;
                if ((is_front && (su_cntl & BIT(0))) ||
                    (!is_front && (su_cntl & BIT(1)))) {
                    continue;
                }
            }
            if (area < 0) {
                Qcom845RasterVertex tmp = rv[1];
                rv[1] = rv[2];
                rv[2] = tmp;
                area = -area;
            }

            int bx1 = MAX((int)x1, MIN(rv[0].x, MIN(rv[1].x, rv[2].x)) / 256);
            int by1 = MAX((int)y1, MIN(rv[0].y, MIN(rv[1].y, rv[2].y)) / 256);
            int bx2 = MIN((int)x2, MAX(rv[0].x, MAX(rv[1].x, rv[2].x)) / 256 + 1);
            int by2 = MIN((int)y2, MAX(rv[0].y, MAX(rv[1].y, rv[2].y)) / 256 + 1);
            if (bx1 >= bx2 || by1 >= by2) {
                continue;
            }
            uint64_t barea = (uint64_t)(bx2 - bx1) * (by2 - by1);
            if (barea > raster_budget) {
                trace_qcom845_render_reject("raster_budget", bx2 - bx1, by2 - by1);
                return false;
            }
            raster_budget -= barea;

            uint64_t vmask = rv[0].varying_mask & rv[1].varying_mask &
                             rv[2].varying_mask;
            bool is_flat[64] = { 0 };
            bool constant[64] = { 0 };
            uint64_t nonconst_mask = 0;
            bool uniform = true;
            float vvar[3][64];

            for (uint64_t m = vmask; m; m &= m - 1) {
                unsigned slot = ctz64(m);
                unsigned mode = extract32(
                    regs[A6XX_VPC_VARYING_INTERP_MODE_0 + slot / 16],
                    (slot % 16) * 2, 2);
                is_flat[slot] = (mode == 2);
                if (is_flat[slot]) {
                    constant[slot] = true;
                } else {
                    constant[slot] = (rv[0].varyings[slot] == rv[1].varyings[slot] &&
                                      rv[0].varyings[slot] == rv[2].varyings[slot]);
                    uniform &= constant[slot];
                    if (!constant[slot]) {
                        nonconst_mask |= BIT_ULL(slot);
                        for (unsigned v = 0; v < 3; v++) {
                            vvar[v][slot] = qcom845_u32_to_f(rv[v].varyings[slot]);
                        }
                    }
                }
            }

            bool affine = (rv[0].inv_w == 1.0f && rv[1].inv_w == 1.0f &&
                           rv[2].inv_w == 1.0f);
            float uniform_src[4] = { 0, 0, 0, 1 };

            if (uniform) {
                qcom845_shader_reset_pixel(&sh);
                sh.read_cache = &read_cache;
                sh.program = &fs_prog;
                for (uint64_t m = vmask; m; m &= m - 1) {
                    unsigned slot = ctz64(m);
                    sh.input[slot] = rv[0].varyings[slot];
                    sh.flat_input[slot] = rv[0].varyings[slot];
                }
                sh.input_valid = vmask;
                sh.flat_valid = vmask;
                if (!qcom845_fragment_eval(s, &sh, fmt, uniform_src)) {
                    return false;
                }
                if (sh.killed) {
                    continue;
                }
            } else {
                sh.read_cache = &read_cache;
                sh.program = &fs_prog;
                for (uint64_t m = vmask; m; m &= m - 1) {
                    unsigned slot = ctz64(m);
                    sh.flat_input[slot] = rv[0].varyings[slot];
                    if (constant[slot]) {
                        sh.input[slot] = rv[0].varyings[slot];
                    }
                }
            }

            double inv_area = 1.0 / (double)area;
            int64_t edge_row[3], step_x[3], step_y[3], bias[3];
            for (unsigned e = 0; e < 3; e++) {
                unsigned next = (e + 1) % 3;
                edge_row[e] = qcom845_edge(&rv[e], &rv[next],
                                           bx1 * 256 + 128, by1 * 256 + 128);
                step_x[e] = -(int64_t)(rv[next].y - rv[e].y) * 256;
                step_y[e] = (int64_t)(rv[next].x - rv[e].x) * 256;
                bias[e] = qcom845_top_left(&rv[e], &rv[next]) ? 0 : -1;
            }

            for (int y = by1; y < by2; y++,
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
                        }
                        int64_t limit = v0 / (-dx) + 1;
                        if (limit < k_hi) {
                            k_hi = limit;
                        }
                    }
                }
                if (k_lo >= k_hi) {
                    continue;
                }

                int rx1 = bx1 + (int)k_lo;
                int rx2 = bx1 + (int)k_hi;
                int64_t edges[3] = {
                    edge_row[0] + k_lo * step_x[0],
                    edge_row[1] + k_lo * step_x[1],
                    edge_row[2] + k_lo * step_x[2],
                };

                for (int x = rx1; x < rx2; x++,
                     edges[0] += step_x[0], edges[1] += step_x[1],
                     edges[2] += step_x[2]) {
                    float src[4];
                    if (uniform) {
                        memcpy(src, uniform_src, sizeof(src));
                    } else {
                        float l0 = (float)((double)edges[1] * inv_area);
                        float l1 = (float)((double)edges[2] * inv_area);
                        float l2 = (float)((double)edges[0] * inv_area);
                        float w0 = l0, w1 = l1, w2 = l2;

                        if (!affine) {
                            float pw0 = l0 * rv[0].inv_w;
                            float pw1 = l1 * rv[1].inv_w;
                            float pw2 = l2 * rv[2].inv_w;
                            float denom = pw0 + pw1 + pw2;
                            if (denom != 0.0f) {
                                float inv_d = 1.0f / denom;
                                w0 = pw0 * inv_d;
                                w1 = pw1 * inv_d;
                                w2 = pw2 * inv_d;
                            }
                        }
                        qcom845_shader_reset_pixel(&sh);
                        sh.read_cache = &read_cache;
                        sh.program = &fs_prog;
                        for (uint64_t m = nonconst_mask; m; m &= m - 1) {
                            unsigned slot = ctz64(m);
                            float iv = w0 * vvar[0][slot] +
                                       w1 * vvar[1][slot] +
                                       w2 * vvar[2][slot];
                            sh.input[slot] = qcom845_f_to_u32(iv);
                        }
                        sh.input_valid = vmask;
                        sh.flat_valid = vmask;
                        if (!qcom845_fragment_eval(s, &sh, fmt, src)) {
                            return false;
                        }
                        if (sh.killed) {
                            continue;
                        }
                    }

                    uint64_t paddr = dest + (uint64_t)y * pitch +
                                     (uint64_t)x * cpp;
                    uint8_t *ppage = qcom845_stage_write_page(s, &write_cache,
                                                              paddr);
                    if (!ppage) {
                        trace_qcom845_render_reject("write_page", paddr, dest);
                        return false;
                    }
                    uint32_t px = qcom845_write_mrt_pixel(ppage,
                                                          paddr & A6XX_PAGE_MASK,
                                                          fmt, cpp, dst_swap,
                                                          read_dst, &blend, src);
                    total_pixels++;
                    if (px & 0x00ffffffU) {
                        total_nonblack++;
                    }
                }
            }
        }
    }

    if (!qcom845_publish_write_pages(write_tbl)) {
        return false;
    }
    hwaddr dest_pa = dest;
    if (s->gpu_smmu) {
        qcom845_smmu_translate(s->gpu_smmu, 0, dest, false, &dest_pa);
    }
    trace_qcom845_render_done(dest, dest_pa, fmt, total_pixels, total_nonblack);
    return true;
}

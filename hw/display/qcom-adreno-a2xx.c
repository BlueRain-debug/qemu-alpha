/*
 * Experimental A2xx straight-line shaders, rasterization and GMEM resolve.
 *
 * Instruction fields follow Mesa freedreno's MIT-licensed instr-a2xx.h;
 * register fields follow Linux's generated a2xx.xml.h. This is deliberately
 * a small interpreter, not a shader recognizer: it executes the uploaded
 * instructions and accesses guest data through the caller's RAM-only SMMU.
 * Unsupported instructions fail before the primitive's writes. A fault in a
 * later primitive can leave earlier primitives drawn, but the command remains
 * pending and does not complete its following fence.
 * Host floating point is sufficient for this local bring-up experiment; it
 * does not promise the hardware's exact rounding or exceptional arithmetic.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <float.h>
#include <math.h>
#include "qemu/bswap.h"
#include "qemu/bitops.h"
#include "hw/display/qcom-adreno-a2xx.h"
#include "trace.h"

/* Register numbers in this file are DWORD indices, not byte addresses. */
#define A2XX_ALU_CONSTANTS 0x4000
#define A2XX_FETCH_CONSTANTS 0x4800
#define A2XX_VS_CONSTANT_BASE 0x2307
#define A2XX_PS_CONSTANT_BASE 0x2308
#define A2XX_SURFACE_INFO 0x2000
#define A2XX_COLOR_INFO 0x2001
#define A2XX_SCREEN_SCISSOR_TL 0x200e
#define A2XX_SCREEN_SCISSOR_BR 0x200f
#define A2XX_WINDOW_OFFSET 0x2080
#define A2XX_WINDOW_SCISSOR_TL 0x2081
#define A2XX_WINDOW_SCISSOR_BR 0x2082
#define A2XX_COLOR_MASK 0x2104
#define A2XX_DEPTH_CONTROL 0x2200
#define A2XX_BLEND_CONTROL 0x2201
#define A2XX_COLOR_CONTROL 0x2202
#define A2XX_RASTER_CONTROL 0x2205
#define A2XX_CULL_FRONT (1U << 0)
#define A2XX_CULL_BACK (1U << 1)
#define A2XX_FRONT_CLOCKWISE (1U << 2)
#define A2XX_VIEWPORT_CONTROL 0x2206
#define A2XX_MODE_CONTROL 0x2208
#define A2XX_VERTEX_CONTROL 0x2302
#define A2XX_PIXEL_CENTER_HALF (1U << 0)
#define A2XX_COPY_CONTROL 0x2318
#define A2XX_COPY_DEST_BASE 0x2319
#define A2XX_COPY_DEST_PITCH 0x231a
#define A2XX_COPY_DEST_INFO 0x231b
#define A2XX_COPY_DEST_OFFSET 0x231c
#define A2XX_POSITION_EXPORT 62
#define A2XX_MAX_PARAMETERS 16
#define A2XX_MAX_DIMENSION 2048
#define A2XX_MAX_PIXELS (1024 * 1024)

typedef struct A2xxInvocation {
    float temporary[64][4];
    float exported[64][4];
    uint8_t export_mask[64];
    float fetch_index;
    bool fetch_index_valid;
    bool trace_texture;
} A2xxInvocation;

typedef struct A2xxPendingPixel {
    uint32_t address;
    uint32_t value;
} A2xxPendingPixel;

static bool a2xx_fail(QcomA2xxDrawContext *context, const char *reason,
                      unsigned detail)
{
    context->failure = reason;
    context->failure_detail = detail;
    return false;
}

static float a2xx_float(uint32_t word)
{
    float value;

    memcpy(&value, &word, sizeof(value));
    return value;
}

/* A 32x32 macro tile contains 8x16 micro tiles. The Y low bit, pipe,
 * and bank bits are interleaved with the byte offset within a micro tile.
 * This is the common 32-bit A2xx/Xenos texture layout (also documented in
 * Xenia's texture_address.h). For example, (2,2) in pitch 32 is byte 0x108.
 */
static uint32_t a2xx_tiled_offset(unsigned x, unsigned y, unsigned pitch)
{
    uint32_t outer = (((y >> 5) * (pitch >> 5)) + (x >> 5)) << 6;
    uint32_t inner = (((y >> 1) & 7) << 3) | (x & 7);
    uint32_t bytes = (outer | inner) << 2;
    unsigned pipe = ((x >> 3) & 3) ^ (((y >> 3) & 1) << 1);

    return (bytes & 15) | ((y & 1) << 4) | ((bytes & 16) << 1) |
           (pipe << 6) | ((bytes & 224) << 3) | ((y & 16) << 7) |
           ((bytes & ~255U) << 4);
}

static uint32_t a2xx_endian(uint32_t word, unsigned mode)
{
    switch (mode) {
    case 1: return ((word & 0xff00ff) << 8) | ((word >> 8) & 0xff00ff);
    case 2: return bswap32(word);
    case 3: return (word << 16) | (word >> 16);
    default: return word;
    }
}

static bool a2xx_fetch_result(float destination[4], const float value[4],
                              unsigned swizzle)
{
    for (unsigned component = 0; component < 4; component++) {
        unsigned selector = (swizzle >> (component * 3)) & 7;

        if (selector < 4) {
            destination[component] = value[selector];
        } else if (selector == 4 || selector == 5) {
            destination[component] = selector - 4;
        } else if (selector != 7) {
            return false;
        }
    }
    return true;
}

static bool a2xx_vertex_fetch(QcomA2xxDrawContext *context,
                               A2xxInvocation *invocation,
                               const uint32_t instruction[3])
{
    uint32_t first = instruction[0], second = instruction[1];
    uint32_t third = instruction[2];
    unsigned source = (first >> 5) & 63, destination = (first >> 12) & 63;
    unsigned fetch = ((first >> 20) & 31) * 6 + ((first >> 25) & 3) * 2;
    unsigned format = (second >> 16) & 63;
    unsigned components, fetch_bytes;
    const uint32_t *constant = context->registers + A2XX_FETCH_CONSTANTS + fetch;
    /* Subsequent mini-fetches reuse the full fetch's index, even when an
     * earlier attribute fetch has overwritten the source temporary. */
    bool mini = second & (1U << 30);
    float index = mini ? invocation->fetch_index :
                  invocation->temporary[source][first >> 30];
    uint64_t offset, address;
    float value[4] = {0, 0, 0, 1};

    if (((first >> 25) & 3) == 3 ||
        (first & ((1U << 11) | (1U << 18))) ||
        (second & 0xbf000000) || (third & 0xc0000000) ||
        (constant[0] & 3) != 3 || !isfinite(index) || index < 0 ||
        index > A2XX_MAX_PIXELS || floorf(index) != index ||
        (mini && !invocation->fetch_index_valid)) {
        return a2xx_fail(context, "vertex fetch state", first);
    }
    if (!mini) {
        invocation->fetch_index = index;
        invocation->fetch_index_valid = true;
    }
    switch (format) {
    case 36: components = 1; break; /* 32_FLOAT */
    case 37: components = 2; break;
    case 57: components = 3; break;
    case 38: components = 4; break;
    case 6: components = 4; break; /* Packed, unsigned normalized RGBA8. */
    default: return a2xx_fail(context, "vertex format", format);
    }
    offset = ((uint64_t)(unsigned)index * (third & 255) +
              ((third >> 8) & 0x3fffff)) * 4;
    fetch_bytes = format == 6 ? 4 : components * 4;
    /* Vertex fetch constant size is in bytes. Check the entire attribute. */
    address = (constant[0] & ~3U) + offset;
    if (offset + fetch_bytes > constant[1] ||
        address + fetch_bytes > (1ULL << 32)) {
        return a2xx_fail(context, "vertex buffer bounds", offset);
    }
    for (unsigned component = 0; component < (format == 6 ? 1 : components); component++) {
        uint32_t word;

        if (!context->read_word(context->opaque, address + component * 4,
                                 &word)) {
            return a2xx_fail(context, "vertex RAM translation", address);
        }
        if (format == 6) {
            if (second & 0x3000) {
                return a2xx_fail(context, "signed or integer RGBA vertex", second);
            }
            for (unsigned channel = 0; channel < 4; channel++) {
                value[channel] = ((word >> (channel * 8)) & 255) / 255.0f;
            }
        } else {
            /* SIGNED_RF_MODE selects signed normalized integer conversion;
             * it has no effect on the floating-point formats. GMEM restore
             * shaders leave it set on their FLOAT2 mini-fetch. */
            value[component] = a2xx_float(word);
        }
        if (!isfinite(value[component])) {
            return a2xx_fail(context, "nonfinite vertex", word);
        }
    }
    return a2xx_fetch_result(invocation->temporary[destination], value,
                              second & 0xfff) ||
           a2xx_fail(context, "vertex destination swizzle", second & 0xfff);
}

static bool a2xx_texture_texel(QcomA2xxDrawContext *context,
                                const uint32_t texture[6], unsigned pitch,
                                unsigned x, unsigned y, float value[4],
                                bool trace_sample)
{
    uint64_t address = texture[1] & 0xfffff000U;
    uint32_t packed;
    unsigned endian = (texture[1] >> 6) & 3;
    float components[4];

    address += texture[0] & (1U << 31) ? a2xx_tiled_offset(x, y, pitch) :
               (y * pitch + x) * 4;
    if (address > UINT32_MAX ||
        !context->read_word(context->opaque, address, &packed)) {
        return a2xx_fail(context, "texture RAM translation", address);
    }
    /* The observed QCDX CPU-upload descriptors request 8IN16 even though
     * their RGBA8 staging bytes already have the final component order.
     * Bypassing just that mode restores opaque black and transparent white.
     * The missing hardware/driver contract is not understood yet: keep this
     * local diagnostic opt-in, and preserve all descriptor modes by default.
     */
    if (context->upload_endian_workaround && endian == 1) {
        endian = 0;
    }
    if (trace_sample) {
        trace_qcom_mdp4_probe_texel(address, texture[1], packed,
                                    a2xx_endian(packed, endian));
    }
    packed = a2xx_endian(packed, endian);
    for (unsigned component = 0; component < 4; component++) {
        components[component] = ((packed >> (component * 8)) & 255) / 255.0f;
    }
    return a2xx_fetch_result(value, components, (texture[3] >> 1) & 0xfff) ||
           a2xx_fail(context, "texture constant swizzle", texture[3]);
}

static unsigned a2xx_texture_axis(int position, unsigned extent, unsigned clamp)
{
    if (clamp == 0) { /* Repeat, including taps across a bilinear seam. */
        int wrapped = position % (int)extent;

        return wrapped < 0 ? wrapped + extent : wrapped;
    }
    return CLAMP(position, 0, (int)extent - 1); /* Clamp to edge. */
}

static bool a2xx_texture_fetch(QcomA2xxDrawContext *context,
                                A2xxInvocation *invocation,
                                const uint32_t instruction[3])
{
    uint32_t first = instruction[0], second = instruction[1];
    unsigned source = (first >> 5) & 63, destination = (first >> 12) & 63;
    const uint32_t *texture = context->registers + A2XX_FETCH_CONSTANTS +
                              ((first >> 20) & 31) * 6;
    unsigned width = (texture[2] & 8191) + 1;
    unsigned height = ((texture[2] >> 13) & 8191) + 1;
    unsigned pitch = ((texture[0] >> 22) & 511) * 32;
    unsigned min_filter = (second >> 14) & 3;
    unsigned mag_filter = (second >> 12) & 3;
    unsigned clamp[2];
    float coordinates[2], value[4] = {0};

    if (min_filter == 3) {
        min_filter = (texture[3] >> 21) & 3;
    }
    if (mag_filter == 3) {
        mag_filter = (texture[3] >> 19) & 3;
    }
    if ((first & ((1U << 11) | (1U << 18))) ||
        (second & 0xe0000000) || instruction[2] != 2 ||
        (texture[0] & 0x3ff) != 2 || (texture[1] & 63) != 6 ||
        (texture[3] & 0x7e000) || (texture[5] & 0x600) != 0x200 ||
        min_filter > 1 || min_filter != mag_filter || !pitch || width > pitch ||
        width > A2XX_MAX_DIMENSION || height > A2XX_MAX_DIMENSION) {
        return a2xx_fail(context, "texture state or filtering", texture[1]);
    }
    for (unsigned axis = 0; axis < 2; axis++) {
        unsigned component = (first >> (26 + axis * 2)) & 3;
        unsigned extent = axis ? height : width;

        clamp[axis] = (texture[0] >> (10 + axis * 3)) & 7;

        coordinates[axis] = invocation->temporary[source][component];
        if (!(first & (1U << 25))) {
            coordinates[axis] *= extent;
        }
        if (!isfinite(coordinates[axis]) ||
            (clamp[axis] != 0 && clamp[axis] != 2)) {
            return a2xx_fail(context, "texture coordinate or clamp", clamp[axis]);
        }
        if (clamp[axis] == 0) {
            /* Reduce before converting to an integer, so even very large
             * finite shader coordinates cannot overflow the tap index. */
            coordinates[axis] = fmodf(coordinates[axis], extent);
            if (coordinates[axis] < 0) {
                coordinates[axis] += extent;
            }
        } else {
            coordinates[axis] = CLAMP(coordinates[axis], 0, extent);
        }
    }
    if (min_filter == 0) {
        unsigned x = a2xx_texture_axis(floorf(coordinates[0]), width, clamp[0]);
        unsigned y = a2xx_texture_axis(floorf(coordinates[1]), height, clamp[1]);

        if (!a2xx_texture_texel(context, texture, pitch, x, y, value,
                                 invocation->trace_texture)) {
            return false;
        }
    } else {
        int origin[2];
        float fraction[2];

        for (unsigned axis = 0; axis < 2; axis++) {
            float centered = coordinates[axis] - 0.5f;

            origin[axis] = floorf(centered);
            fraction[axis] = centered - origin[axis];
        }
        for (unsigned tap = 0; tap < 4; tap++) {
            unsigned x = a2xx_texture_axis(origin[0] + (int)(tap & 1),
                                             width, clamp[0]);
            unsigned y = a2xx_texture_axis(origin[1] + (int)(tap >> 1),
                                             height, clamp[1]);
            float weight = (tap & 1 ? fraction[0] : 1 - fraction[0]) *
                           (tap & 2 ? fraction[1] : 1 - fraction[1]);
            float sample[4] = {0};

            if (!a2xx_texture_texel(context, texture, pitch, x, y, sample,
                                     invocation->trace_texture)) {
                return false;
            }
            for (unsigned component = 0; component < 4; component++) {
                value[component] += weight * sample[component];
            }
        }
    }
    if (!a2xx_fetch_result(invocation->temporary[destination], value,
                            second & 0xfff)) {
        return a2xx_fail(context, "texture destination swizzle", second);
    }
    return true;
}

static bool a2xx_alu_source(QcomA2xxDrawContext *context,
                             A2xxInvocation *invocation, unsigned stage,
                             const uint32_t instruction[3], unsigned source,
                             float result[4])
{
    unsigned shift = source * 8;
    unsigned index = (instruction[2] >> shift) & 255;
    bool temporary = instruction[2] & (1U << (29 + source));
    unsigned swizzle = (instruction[1] >> shift) & 255;
    float input[4];

    if (temporary) {
        if (index & 64) {
            return a2xx_fail(context, "relative ALU register", index);
        }
        memcpy(input, invocation->temporary[index & 63], sizeof(input));
    } else {
        unsigned base = context->registers[stage ? A2XX_PS_CONSTANT_BASE :
                                                  A2XX_VS_CONSTANT_BASE] & 511;

        if (base + index >= 512) {
            return a2xx_fail(context, "ALU constant bounds", base + index);
        }
        for (unsigned component = 0; component < 4; component++) {
            input[component] = a2xx_float(context->registers[
                A2XX_ALU_CONSTANTS + (base + index) * 4 + component]);
        }
    }
    for (unsigned component = 0; component < 4; component++) {
        /* ALU swizzles encode component offsets; zero means identity. */
        result[component] = input[((swizzle >> (component * 2)) + component) & 3];
        if (temporary && (index & 128)) {
            result[component] = fabsf(result[component]);
        }
        if (instruction[1] & (1U << (24 + source))) {
            result[component] = -result[component];
        }
    }
    return true;
}

static bool a2xx_alu(QcomA2xxDrawContext *context, A2xxInvocation *invocation,
                      unsigned stage, const uint32_t instruction[3])
{
    uint32_t first = instruction[0];
    unsigned vector_mask = (first >> 16) & 15;
    unsigned scalar_mask = (first >> 20) & 15;
    unsigned opcode = (instruction[2] >> 24) & 31;
    bool exported = first & (1U << 15);
    unsigned destination = first & 63;
    unsigned scalar_destination = (first >> 8) & 63;
    unsigned constant_one = exported ? vector_mask & scalar_mask : 0;
    unsigned constant_zero = exported && (first & (1U << 14)) ?
                             (~(vector_mask | scalar_mask) & 15) : 0;
    float source1[4], source2[4], source3[4], result[4], scalar = 0, dot = 0;

    if ((first & ((1U << 6) | (1U << 7))) ||
        (!exported && (first & (1U << 14))) ||
        (instruction[1] & 0xf8000000)) {
        return a2xx_fail(context, "predicated or relative ALU", first);
    }
    if (exported) {
        vector_mask &= ~constant_one;
        scalar_mask &= ~constant_one;
        scalar_destination = destination;
    }
    if (!(vector_mask | scalar_mask | constant_one | constant_zero)) {
        return true; /* Unpredicated ALU with no destination writes is NOP. */
    }
    if (!a2xx_alu_source(context, invocation, stage, instruction, 2, source1) ||
        !a2xx_alu_source(context, invocation, stage, instruction, 1, source2) ||
        ((opcode == 11 || scalar_mask) &&
         !a2xx_alu_source(context, invocation, stage, instruction, 0, source3))) {
        return false;
    }
    if (scalar_mask) {
        switch (first >> 26) {
        case 0: scalar = source3[3] + source3[0]; break;
        case 2: scalar = source3[3] * source3[0]; break;
        case 5: scalar = fmaxf(source3[3], source3[0]); break;
        case 6: scalar = fminf(source3[3], source3[0]); break;
        case 19: scalar = 1.0f / source3[3]; break;
        default: return a2xx_fail(context, "scalar ALU opcode", first >> 26);
        }
        if (first & (1U << 25)) {
            scalar = CLAMP(scalar, 0, 1);
        }
        if (!isfinite(scalar)) {
            return a2xx_fail(context, "nonfinite scalar ALU", first >> 26);
        }
    }
    if (opcode == 15 || opcode == 16) {
        for (unsigned component = 0; component < (opcode == 15 ? 4 : 3); component++) {
            dot += source1[component] * source2[component];
        }
    }
    for (unsigned component = 0; component < 4; component++) {
        switch (opcode) {
        case 0: result[component] = source1[component] + source2[component]; break;
        case 1: result[component] = source1[component] * source2[component]; break;
        case 2: result[component] = fmaxf(source1[component], source2[component]); break;
        case 3: result[component] = fminf(source1[component], source2[component]); break;
        case 11: result[component] = source1[component] * source2[component] +
                                      source3[component]; break;
        case 15:
        case 16: result[component] = dot; break;
        default: return a2xx_fail(context, "vector ALU opcode", opcode);
        }
        if (first & (1U << 24)) {
            result[component] = CLAMP(result[component], 0, 1);
        }
        if (!isfinite(result[component])) {
            return a2xx_fail(context, "nonfinite ALU result", opcode);
        }
    }
    for (unsigned component = 0; component < 4; component++) {
        if (vector_mask & (1U << component)) {
            (exported ? invocation->exported : invocation->temporary)
                [destination][component] = result[component];
        }
        if (scalar_mask & (1U << component)) {
            (exported ? invocation->exported : invocation->temporary)
                [scalar_destination][component] = scalar;
        }
        if (constant_one & (1U << component)) {
            invocation->exported[destination][component] = 1;
        } else if (constant_zero & (1U << component)) {
            invocation->exported[destination][component] = 0;
        }
    }
    if (exported) {
        invocation->export_mask[destination] |= vector_mask | scalar_mask |
                                                 constant_one | constant_zero;
    }
    return true;
}

static bool a2xx_shader(QcomA2xxDrawContext *context, unsigned stage,
                         A2xxInvocation *invocation)
{
    const uint32_t *program = context->shader[stage];
    unsigned words = context->shader_words[stage];

    /* Two 48-bit control-flow instructions share three DWORDs. Execution
     * clauses refer to 96-bit fetch/ALU instructions in the same upload. */
    for (unsigned pair = 0; pair + 2 < words && pair < 384; pair += 3) {
        uint64_t control[2] = {
            program[pair] | ((uint64_t)(program[pair + 1] & 0xffff) << 32),
            (program[pair + 1] >> 16) | ((uint64_t)program[pair + 2] << 16),
        };

        for (unsigned half = 0; half < 2; half++) {
            unsigned opcode = control[half] >> 44;
            unsigned address = control[half] & 511;
            unsigned count = (control[half] >> 12) & 7;
            unsigned sequence = (control[half] >> 16) & 0xfff;

            if (opcode == 12 && ((control[half] >> 41) & 3) == 3) {
                return a2xx_fail(context, "shader memory export allocation", stage);
            }
            if (opcode == 0 || opcode == 12 || opcode == 15) {
                continue; /* NOP, position/parameter allocation, fetch hint. */
            }
            if ((opcode != 1 && opcode != 2) ||
                (address + count) * 3 > words) {
                return a2xx_fail(context, "shader control flow", opcode);
            }
            for (unsigned item = 0; item < count; item++) {
                const uint32_t *instruction = program + (address + item) * 3;
                bool success;

                if ((sequence >> (item * 2)) & 1) {
                    unsigned fetch_opcode = instruction[0] & 31;

                    if (stage == 0 && fetch_opcode == 0) {
                        success = a2xx_vertex_fetch(context, invocation, instruction);
                    } else if (stage == 1 && fetch_opcode == 1) {
                        success = a2xx_texture_fetch(context, invocation, instruction);
                    } else {
                        return a2xx_fail(context, "shader fetch opcode", fetch_opcode);
                    }
                } else {
                    success = a2xx_alu(context, invocation, stage, instruction);
                }
                if (!success) {
                    return false;
                }
            }
            if (opcode == 2) {
                return true;
            }
        }
    }
    return a2xx_fail(context, "unterminated shader", stage);
}

static float a2xx_blend_factor(unsigned factor, unsigned component,
                                const float source[4], const float destination[4])
{
    switch (factor) {
    case 0: return 0;
    case 1: return 1;
    case 4: return source[component];
    case 5: return 1 - source[component];
    case 6: return source[3];
    case 7: return 1 - source[3];
    case 8: return destination[component];
    case 9: return 1 - destination[component];
    case 10: return destination[3];
    case 11: return 1 - destination[3];
    default: return 0; /* Rejected by fixed-function validation. */
    }
}

static bool a2xx_valid_blend_factor(unsigned factor)
{
    /* Adreno's encoding leaves 2 and 3 reserved. */
    return factor <= 1 || (factor >= 4 && factor <= 11);
}

static bool a2xx_top_left(const float start[4], const float end[4])
{
    return end[1] < start[1] || (end[1] == start[1] && end[0] > start[0]);
}

static bool a2xx_primitive(QcomA2xxDrawContext *context,
                            const uint32_t indices[3], bool rectangle)
{
    const uint32_t *regs = context->registers;
    uint32_t color = regs[A2XX_COLOR_INFO];
    unsigned pitch = regs[A2XX_SURFACE_INFO] & 0x3fff;
    A2xxInvocation vertex[3] = {0};
    g_autofree A2xxPendingPixel *pending = NULL;
    unsigned pixel_count = 0;
    int left, top, right, bottom;
    float basis[2][2], determinant;
    float minimum[2] = {FLT_MAX, FLT_MAX}, maximum[2] = {-FLT_MAX, -FLT_MAX};
    bool resolve = regs[A2XX_MODE_CONTROL] == 6;
    bool gmem = !resolve && (color & 0xfffff000U) < context->gmem_bytes;
    bool blending = !resolve && !(regs[A2XX_COLOR_CONTROL] & 0x20);
    unsigned viewport = regs[A2XX_VIEWPORT_CONTROL];
    uint32_t blend = regs[A2XX_BLEND_CONTROL];
    float reciprocal_w[3] = {1, 1, 1};
    uint32_t copy_info = regs[A2XX_COPY_DEST_INFO];
    unsigned copy_pitch = regs[A2XX_COPY_DEST_PITCH] * 32;
    /* D3D9 centers are at integer coordinates; OGL centers are at .5.
     * Use the selected convention for both coverage and interpolation.
     * Always using .5 leaves the last row/column of half-offset D3D
     * fullscreen geometry uncovered and samples textures between texels.
     */
    float sample_offset = regs[A2XX_VERTEX_CONTROL] & A2XX_PIXEL_CENTER_HALF ?
                          0.5f : 0.0f;

    if ((color & 15) != 5 || (color & 0x980) ||
        (regs[A2XX_SURFACE_INFO] & ~0x3fffU) || !pitch ||
        pitch > A2XX_MAX_DIMENSION || (pitch & 31) ||
        (!resolve && regs[A2XX_MODE_CONTROL] != 4) ||
        (regs[A2XX_DEPTH_CONTROL] & 7) ||
        (regs[A2XX_COLOR_CONTROL] & 0x1ffd8) != 0xc00 ||
        (blending && ((blend & 0xe0e0e0e0U) ||
          !a2xx_valid_blend_factor(blend & 31) ||
          !a2xx_valid_blend_factor((blend >> 8) & 31) ||
          !a2xx_valid_blend_factor((blend >> 16) & 31) ||
          !a2xx_valid_blend_factor((blend >> 24) & 31))) ||
        (regs[A2XX_COLOR_MASK] & 15) != 15 ||
        (regs[A2XX_COLOR_MASK] & ~255U) ||
        (viewport != 0xb00 && viewport != 0x43f) ||
        (rectangle && (regs[A2XX_RASTER_CONTROL] &
                       (A2XX_CULL_FRONT | A2XX_CULL_BACK)))) {
        return a2xx_fail(context, "render target or fixed-function state", color);
    }
    if (resolve && (!rectangle || regs[A2XX_COPY_CONTROL] ||
        (color & 0xfffff000U) >= context->gmem_bytes ||
        (copy_info & ~0x108U) != 0x3c050 ||
        !copy_pitch || copy_pitch > A2XX_MAX_DIMENSION ||
        (regs[A2XX_COPY_DEST_OFFSET] & ~0x3ffffffU))) {
        return a2xx_fail(context, "GMEM resolve state", copy_info);
    }
    for (unsigned index = 0; index < 3; index++) {
        vertex[index].temporary[0][0] = indices[index];
        if (!a2xx_shader(context, 0, &vertex[index])) {
            return false;
        }
        if ((vertex[index].export_mask[A2XX_POSITION_EXPORT] & 3) != 3) {
            return a2xx_fail(context, "vertex position missing", index);
        }
        if (viewport == 0x43f) {
            float *position = vertex[index].exported[A2XX_POSITION_EXPORT];

            if (vertex[index].export_mask[A2XX_POSITION_EXPORT] != 15 ||
                position[3] <= 0) {
                return a2xx_fail(context, "vertex requires homogeneous clipping", index);
            }
            reciprocal_w[index] = 1.0f / position[3];
            for (unsigned axis = 0; axis < 3; axis++) {
                position[axis] = position[axis] * reciprocal_w[index] *
                    a2xx_float(regs[0x210f + axis * 2]) +
                    a2xx_float(regs[0x2110 + axis * 2]);
                if (!isfinite(position[axis])) {
                    return a2xx_fail(context, "nonfinite viewport result", index);
                }
            }
        }
        for (unsigned axis = 0; axis < 2; axis++) {
            float *coordinate = &vertex[index].exported[A2XX_POSITION_EXPORT][axis];

            /* The driver replays one scene for several GMEM tiles. Window
             * offset translates screen-space vertices into the current tile. */
            if (regs[A2XX_RASTER_CONTROL] & (1U << 16)) {
                *coordinate += sextract32(regs[A2XX_WINDOW_OFFSET], axis * 16, 15);
            }
            if (fabsf(*coordinate) > 32768) {
                return a2xx_fail(context, "vertex coordinate range", index);
            }
            minimum[axis] = fminf(minimum[axis], *coordinate);
            maximum[axis] = fmaxf(maximum[axis], *coordinate);
        }
    }
    /* RECTLIST supplies three distinct corners. The driver uses different
     * corner orders for blits and clears. Interpolate the plane of the
     * original triangle, including its extrapolation to the fourth corner. */
    for (unsigned index = 0; index < 3; index++) {
        for (unsigned axis = 0; axis < 2; axis++) {
            float coordinate = vertex[index].exported[A2XX_POSITION_EXPORT][axis];

            if (rectangle && coordinate != minimum[axis] && coordinate != maximum[axis]) {
                return a2xx_fail(context, "non-axis-aligned rectangle", index);
            }
            if (index) {
                basis[index - 1][axis] = coordinate -
                    vertex[0].exported[A2XX_POSITION_EXPORT][axis];
            }
        }
    }
    determinant = basis[0][0] * basis[1][1] - basis[1][0] * basis[0][1];
    if (!determinant) {
        return true; /* Degenerate triangles have no covered samples. */
    }
    if (!rectangle) {
        /* FACE=0 selects CCW front faces, FACE=1 selects CW. Screen Y
         * increases downwards, so a positive signed area is clockwise.
         * Culling happens after the viewport transform, before any stores. */
        bool front = (determinant > 0) ==
                     !!(regs[A2XX_RASTER_CONTROL] & A2XX_FRONT_CLOCKWISE);

        if (regs[A2XX_RASTER_CONTROL] &
            (front ? A2XX_CULL_FRONT : A2XX_CULL_BACK)) {
            return true;
        }
    }
    left = ceilf(CLAMP(minimum[0] - sample_offset, 0, A2XX_MAX_DIMENSION));
    right = ceilf(CLAMP(maximum[0] - sample_offset, 0, A2XX_MAX_DIMENSION));
    top = ceilf(CLAMP(minimum[1] - sample_offset, 0, A2XX_MAX_DIMENSION));
    bottom = ceilf(CLAMP(maximum[1] - sample_offset, 0, A2XX_MAX_DIMENSION));
    for (unsigned scissor = 0; scissor < 2; scissor++) {
        uint32_t tl = regs[scissor ? A2XX_WINDOW_SCISSOR_TL : A2XX_SCREEN_SCISSOR_TL];
        uint32_t br = regs[scissor ? A2XX_WINDOW_SCISSOR_BR : A2XX_SCREEN_SCISSOR_BR];

        /* Window coordinates are 14 bits. The driver's clear packets have
         * set padding bits 14/15; treating them as coordinates clips away
         * the entire clear. Screen scissor coordinates have 15 bits. */
        unsigned mask = scissor ? 0x3fff : 0x7fff;

        int tl_x = scissor ? tl & mask : sextract32(tl, 0, 15);
        int tl_y = scissor ? (tl >> 16) & mask : sextract32(tl, 16, 15);
        int br_x = scissor ? br & mask : sextract32(br, 0, 15);
        int br_y = scissor ? (br >> 16) & mask : sextract32(br, 16, 15);

        if (scissor && !(tl & (1U << 31))) {
            int offset_x = sextract32(regs[A2XX_WINDOW_OFFSET], 0, 15);
            int offset_y = sextract32(regs[A2XX_WINDOW_OFFSET], 16, 15);

            tl_x += offset_x;
            br_x += offset_x;
            tl_y += offset_y;
            br_y += offset_y;
        }
        left = MAX(left, tl_x);
        top = MAX(top, tl_y);
        right = MIN(right, br_x);
        bottom = MIN(bottom, br_y);
    }
    if (right <= left || bottom <= top) {
        return true; /* Fully clipped draw has no memory effects. */
    }
    if (right > pitch || (right - left) * (bottom - top) > A2XX_MAX_PIXELS) {
        return a2xx_fail(context, "rectangle bounds", right);
    }
    pending = g_new(A2xxPendingPixel, (right - left) * (bottom - top));
    for (int y = top; y < bottom; y++) {
        for (int x = left; x < right; x++) {
            A2xxInvocation pixel = {.trace_texture = pixel_count == 0};
            float delta_x = x + sample_offset - vertex[0].exported[A2XX_POSITION_EXPORT][0];
            float delta_y = y + sample_offset - vertex[0].exported[A2XX_POSITION_EXPORT][1];
            float weights[2] = {
                (delta_x * basis[1][1] - basis[1][0] * delta_y) / determinant,
                (basis[0][0] * delta_y - delta_x * basis[0][1]) / determinant,
            };
            uint64_t address = color & 0xfffff000U;
            uint32_t packed = 0;

            if (!rectangle) {
                float barycentric[3] = {1 - weights[0] - weights[1],
                                          weights[0], weights[1]};
                bool covered = true;

                for (unsigned edge = 0; edge < 3; edge++) {
                    unsigned first = (edge + 1) % 3, last = (edge + 2) % 3;
                    bool inclusive = determinant > 0 ?
                        a2xx_top_left(vertex[first].exported[A2XX_POSITION_EXPORT],
                                       vertex[last].exported[A2XX_POSITION_EXPORT]) :
                        a2xx_top_left(vertex[last].exported[A2XX_POSITION_EXPORT],
                                       vertex[first].exported[A2XX_POSITION_EXPORT]);

                    if (barycentric[edge] < 0 ||
                        (barycentric[edge] == 0 && !inclusive)) {
                        covered = false;
                        break;
                    }
                }
                if (!covered) {
                    continue;
                }
            }
            /* Perspective-correct interpolation after the viewport divide. */
            if (viewport == 0x43f) {
                float denominator = (1 - weights[0] - weights[1]) * reciprocal_w[0] +
                                    weights[0] * reciprocal_w[1] + weights[1] * reciprocal_w[2];

                weights[0] = weights[0] * reciprocal_w[1] / denominator;
                weights[1] = weights[1] * reciprocal_w[2] / denominator;
            }

            for (unsigned parameter = 0; !resolve && parameter < A2XX_MAX_PARAMETERS; parameter++) {
                for (unsigned component = 0; component < 4; component++) {
                    pixel.temporary[parameter][component] =
                        vertex[0].exported[parameter][component] +
                        weights[0] * (vertex[1].exported[parameter][component] -
                                      vertex[0].exported[parameter][component]) +
                        weights[1] * (vertex[2].exported[parameter][component] -
                                      vertex[0].exported[parameter][component]);
                }
            }
            if (resolve) {
                /* GMEM is on-chip storage, independent of the system-RAM
                 * LINEAR flag. Use one canonical layout for clears, restores,
                 * normal draws and resolves, even if that flag changes. */
                uint64_t source = (color & 0xfffff000U) + (y * pitch + x) * 4;
                unsigned destination_x = x + (regs[A2XX_COPY_DEST_OFFSET] & 8191);
                unsigned destination_y = y + ((regs[A2XX_COPY_DEST_OFFSET] >> 13) & 8191);
                uint32_t previous;

                if (source + 4 > context->gmem_bytes || destination_x >= copy_pitch ||
                    destination_y >= A2XX_MAX_DIMENSION) {
                    return a2xx_fail(context, "GMEM resolve bounds", source);
                }
                previous = context->gmem[source / 4];
                for (unsigned component = 0; component < 4; component++) {
                    unsigned input = ((color >> 9) & 3) == 1 && component < 3 ?
                                     2 - component : component;

                    pixel.exported[0][component] = ((previous >> (input * 8)) & 255) / 255.0f;
                }
                pixel.export_mask[0] = 15;
                address = (uint64_t)regs[A2XX_COPY_DEST_BASE] +
                    (copy_info & 8 ? (destination_y * copy_pitch + destination_x) * 4 :
                     a2xx_tiled_offset(destination_x, destination_y, copy_pitch));
            } else {
                if (!a2xx_shader(context, 1, &pixel)) {
                    return false;
                }
                address += gmem || (color & 0x40) ? (y * pitch + x) * 4 :
                           a2xx_tiled_offset(x, y, pitch);
            }
            if (pixel.export_mask[0] != 15) {
                return a2xx_fail(context, "pixel color missing", pixel.export_mask[0]);
            }
            if (address > UINT32_MAX || (gmem ?
                address + 4 > context->gmem_bytes :
                !context->writable_word(context->opaque, address))) {
                return a2xx_fail(context, "render target RAM translation", address);
            }
            if (blending) {
                uint32_t previous;
                float destination[4];

                if (gmem) {
                    previous = context->gmem[address / 4];
                } else if (!context->read_word(context->opaque, address, &previous)) {
                    return a2xx_fail(context, "blend destination RAM translation", address);
                }
                for (unsigned component = 0; component < 4; component++) {
                    unsigned input = ((color >> 9) & 3) == 1 && component < 3 ?
                                     2 - component : component;

                    destination[component] = ((previous >> (input * 8)) & 255) / 255.0f;
                }
                for (unsigned component = 0; component < 4; component++) {
                    unsigned factors = blend >> (component == 3 ? 16 : 0);

                    pixel.exported[0][component] =
                        pixel.exported[0][component] * a2xx_blend_factor(factors & 31,
                            component, pixel.exported[0], destination) +
                        destination[component] * a2xx_blend_factor((factors >> 8) & 31,
                            component, pixel.exported[0], destination);
                }
            }
            for (unsigned component = 0; component < 4; component++) {
                unsigned output = component;
                unsigned byte = floorf(CLAMP(pixel.exported[0][component], 0, 1) *
                                         255 + 0.5f);

                unsigned swap = resolve ? ((copy_info >> 8) & 3) : ((color >> 9) & 3);

                if (swap == 1 && component < 3) {
                    output = 2 - component; /* RGB -> BGR. */
                } else if (swap > 1) {
                    return a2xx_fail(context, "color component swap", color);
                }
                packed |= byte << (output * 8);
            }
            pending[pixel_count++] = (A2xxPendingPixel) {address, packed};
        }
    }
    /* All instructions and destination mappings have succeeded. System RAM
     * mappings are rechecked on write; a concurrent guest page-table change
     * may fault the commit, in which case its following fence stays pending. */
    for (unsigned index = 0; index < pixel_count; index++) {
        if (gmem) {
            context->gmem[pending[index].address / 4] = pending[index].value;
            continue;
        }
        if (!context->write_word(context->opaque, pending[index].address,
                                  pending[index].value)) {
            return a2xx_fail(context, "render target write", pending[index].address);
        }
    }
    if (pixel_count) {
        trace_qcom_mdp4_probe_primitive(rectangle | (resolve << 1), color, pitch,
            regs[A2XX_RASTER_CONTROL], regs[A2XX_COLOR_CONTROL], blend,
            copy_info, pixel_count, pending[0].address, pending[0].value);
    }
    return true;
}

bool qcom_a2xx_draw_rect(QcomA2xxDrawContext *context)
{
    static const uint32_t indices[3] = {0, 1, 2};

    return a2xx_primitive(context, indices, true);
}

bool qcom_a2xx_draw_triangles(QcomA2xxDrawContext *context,
                               uint32_t index_address, unsigned index_count,
                               unsigned index_bytes)
{
    uint32_t indices[192];

    if (!index_count || index_count > ARRAY_SIZE(indices) || index_count % 3 ||
        index_bytes != index_count * 2 || (index_address & 1) ||
        (uint64_t)index_address + index_bytes > (1ULL << 32)) {
        return a2xx_fail(context, "16-bit triangle index bounds", index_count);
    }
    for (unsigned index = 0; index < index_count; index++) {
        uint32_t address = index_address + index * 2, word;

        if (!context->read_word(context->opaque, address & ~3U, &word)) {
            return a2xx_fail(context, "index RAM translation", address);
        }
        indices[index] = (word >> ((address & 2) * 8)) & 0xffff;
    }
    for (unsigned index = 0; index < index_count; index += 3) {
        if (!a2xx_primitive(context, &indices[index], false)) {
            return false;
        }
    }
    return true;
}

/*
 * Check NEON widening and signed-to-unsigned saturating narrowing.
 *
 * The oracle uses scalar lane arithmetic, independently of the packed TCG
 * implementation. Exercise both source/destination overlap positions and
 * compare the entire FPSCR so unrelated flags and sticky QC are preserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <arm_neon.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef TCG_NEON_FREESTANDING
enum {
    SEMIHOST_WRITE0 = 4,
    SEMIHOST_EXIT_EXTENDED = 0x20,
    SEMIHOST_APPLICATION_EXIT = 0x20026,
};

static void semihost_call(uintptr_t operation, uintptr_t argument)
{
    register uintptr_t call asm("r0") = operation;
    register uintptr_t data asm("r1") = argument;
#ifdef __thumb__
    asm volatile("svc 0xab" : "+r"(call) : "r"(data) : "memory");
#else
    asm volatile("svc 0x123456" : "+r"(call) : "r"(data) : "memory");
#endif
}

static void test_report(const char *message)
{
    semihost_call(SEMIHOST_WRITE0, (uintptr_t)message);
    semihost_call(SEMIHOST_WRITE0, (uintptr_t)"\n");
}

static void __attribute__((noreturn)) test_exit(int status)
{
    uintptr_t arguments[] = { SEMIHOST_APPLICATION_EXIT, status };
    semihost_call(SEMIHOST_EXIT_EXTENDED, (uintptr_t)arguments);
    for (;;) {
    }
}
#else
#include <stdio.h>
#include <stdlib.h>
#define test_report puts
#define test_exit exit
#endif

#define FPSCR_QC (1u << 27)
/* Preserve an unrelated rounding-mode bit and cumulative exception flag. */
#define FPSCR_OTHER_FLAGS ((1u << 22) | 1u)

typedef union Vector {
    uint8x16_t wide;
    uint8x8_t narrow;
    uint8_t bytes[16];
    uint16_t halfwords[8];
    uint32_t words[4];
    uint64_t doublewords[2];
} Vector;

enum Overlap { SEPARATE, SOURCE_LOW, SOURCE_HIGH };
enum NarrowMode { SATURATE, SHIFT_SATURATE, ROUND_SHIFT_SATURATE };

static void write_fpscr(uint32_t value)
{
    asm volatile("vmsr fpscr, %0" : : "r"(value) : "memory");
}

static uint32_t read_fpscr(void)
{
    uint32_t value;
    asm volatile("vmrs %0, fpscr" : "=r"(value) : : "memory");
    return value;
}

/* The fixed q0 copies deliberately overlap either half of the source. */
#define NARROW_ASM(INSTRUCTION)                                       \
    do {                                                              \
        if (overlap == SOURCE_LOW) {                                  \
            asm volatile("vmov q0, %q1\n\t"                           \
                         INSTRUCTION " d0, q0\n\t"                    \
                         "vmov %P0, d0"                               \
                         : "=w"(output) : "w"(input)                   \
                         : "d0", "d1", "memory");                     \
        } else if (overlap == SOURCE_HIGH) {                           \
            asm volatile("vmov q0, %q1\n\t"                           \
                         INSTRUCTION " d1, q0\n\t"                    \
                         "vmov %P0, d1"                               \
                         : "=w"(output) : "w"(input)                   \
                         : "d0", "d1", "memory");                     \
        } else {                                                      \
            asm volatile(INSTRUCTION " %P0, %q1"                      \
                         : "=&w"(output) : "w"(input) : "memory");    \
        }                                                             \
    } while (0)

/* Immediate shifts must appear literally in the assembled instruction. */
#define SHIFTS_8(APPLY, BITS)                                         \
    APPLY(BITS, 1) APPLY(BITS, 2) APPLY(BITS, 3) APPLY(BITS, 4)          \
    APPLY(BITS, 5) APPLY(BITS, 6) APPLY(BITS, 7) APPLY(BITS, 8)
#define SHIFTS_16(APPLY, BITS)                                        \
    SHIFTS_8(APPLY, BITS)                                             \
    APPLY(BITS, 9) APPLY(BITS, 10) APPLY(BITS, 11) APPLY(BITS, 12)       \
    APPLY(BITS, 13) APPLY(BITS, 14) APPLY(BITS, 15) APPLY(BITS, 16)
#define SHIFTS_32(APPLY, BITS)                                        \
    SHIFTS_16(APPLY, BITS)                                            \
    APPLY(BITS, 17) APPLY(BITS, 18) APPLY(BITS, 19) APPLY(BITS, 20)      \
    APPLY(BITS, 21) APPLY(BITS, 22) APPLY(BITS, 23) APPLY(BITS, 24)      \
    APPLY(BITS, 25) APPLY(BITS, 26) APPLY(BITS, 27) APPLY(BITS, 28)      \
    APPLY(BITS, 29) APPLY(BITS, 30) APPLY(BITS, 31) APPLY(BITS, 32)

/* Append shifts after the source operand, retaining the overlap helpers. */
#define WIDEN_ASM(INSTRUCTION, SUFFIX)                                \
    do {                                                              \
        if (overlap == SOURCE_LOW || overlap == SOURCE_HIGH) {        \
            if (overlap == SOURCE_LOW) {                              \
                asm volatile("vmov d0, %P1\n\t"                       \
                             INSTRUCTION " q0, d0" SUFFIX "\n\t"     \
                             "vmov %q0, q0"                           \
                             : "=w"(output) : "w"(input)               \
                             : "d0", "d1", "memory");                 \
            } else {                                                  \
                asm volatile("vmov d1, %P1\n\t"                       \
                             INSTRUCTION " q0, d1" SUFFIX "\n\t"     \
                             "vmov %q0, q0"                           \
                             : "=w"(output) : "w"(input)               \
                             : "d0", "d1", "memory");                 \
            }                                                         \
        } else {                                                      \
            asm volatile(INSTRUCTION " %q0, %P1" SUFFIX               \
                         : "=&w"(output) : "w"(input) : "memory");    \
        }                                                             \
    } while (0)

/* Case-label X-macros cannot use an enclosing do-while statement. */
#define WIDEN_CASE(BITS, SHIFT)                                       \
    case SHIFT:                                                       \
        WIDEN_ASM("vshll.u" #BITS, ", #" #SHIFT);                     \
        break;

static uint8x16_t widen(uint8x8_t input, unsigned bits, unsigned shift,
                       enum Overlap overlap)
{
    uint8x16_t output;
    switch (bits) {
    case 8:
        switch (shift) {
        case 0:
            WIDEN_ASM("vmovl.u8", "");
            break;
        SHIFTS_8(WIDEN_CASE, 8)
        default:
            test_exit(2);
        }
        break;
    case 16:
        switch (shift) {
        case 0:
            WIDEN_ASM("vmovl.u16", "");
            break;
        SHIFTS_16(WIDEN_CASE, 16)
        default:
            test_exit(2);
        }
        break;
    case 32:
        switch (shift) {
        case 0:
            WIDEN_ASM("vmovl.u32", "");
            break;
        SHIFTS_32(WIDEN_CASE, 32)
        default:
            test_exit(2);
        }
        break;
    default:
        test_exit(2);
    }
    return output;
}

#define SHIFT_NARROW_ASM(INSTRUCTION, SHIFT)                           \
    do {                                                              \
        if (overlap == SOURCE_LOW || overlap == SOURCE_HIGH) {        \
            if (overlap == SOURCE_LOW) {                              \
                asm volatile("vmov q0, %q1\n\t"                       \
                             INSTRUCTION " d0, q0, #" #SHIFT "\n\t"   \
                             "vmov %P0, d0"                           \
                             : "=w"(output) : "w"(input)               \
                             : "d0", "d1", "memory");                 \
            } else {                                                  \
                asm volatile("vmov q0, %q1\n\t"                       \
                             INSTRUCTION " d1, q0, #" #SHIFT "\n\t"   \
                             "vmov %P0, d1"                           \
                             : "=w"(output) : "w"(input)               \
                             : "d0", "d1", "memory");                 \
            }                                                         \
        } else {                                                      \
            asm volatile(INSTRUCTION " %P0, %q1, #" #SHIFT             \
                         : "=&w"(output) : "w"(input) : "memory");    \
        }                                                             \
    } while (0)

#define NARROW_CASE(BITS, SHIFT)                                      \
    case SHIFT:                                                       \
        if (mode == ROUND_SHIFT_SATURATE) {                           \
            SHIFT_NARROW_ASM("vqrshrun.s" #BITS, SHIFT);               \
        } else {                                                      \
            SHIFT_NARROW_ASM("vqshrun.s" #BITS, SHIFT);                \
        }                                                             \
        break;

static uint8x8_t narrow(uint8x16_t input, unsigned bits, unsigned shift,
                       enum NarrowMode mode, enum Overlap overlap)
{
    uint8x8_t output;
    if (mode == SATURATE) {
        switch (bits) {
        case 16:
            NARROW_ASM("vqmovun.s16");
            break;
        case 32:
            NARROW_ASM("vqmovun.s32");
            break;
        case 64:
            NARROW_ASM("vqmovun.s64");
            break;
        default:
            test_exit(2);
        }
    } else {
        switch (bits) {
        case 16:
            switch (shift) {
            SHIFTS_8(NARROW_CASE, 16)
            default:
                test_exit(2);
            }
            break;
        case 32:
            switch (shift) {
            SHIFTS_16(NARROW_CASE, 32)
            default:
                test_exit(2);
            }
            break;
        case 64:
            switch (shift) {
            SHIFTS_32(NARROW_CASE, 64)
            default:
                test_exit(2);
            }
            break;
        default:
            test_exit(2);
        }
    }
    return output;
}

static uint64_t lane(const Vector *vector, unsigned bits, unsigned index)
{
    switch (bits) {
    case 8:
        return vector->bytes[index];
    case 16:
        return vector->halfwords[index];
    case 32:
        return vector->words[index];
    case 64:
        return vector->doublewords[index];
    default:
        test_exit(2);
    }
}

static void set_lane(Vector *vector, unsigned bits, unsigned index,
                     uint64_t value)
{
    switch (bits) {
    case 8:
        vector->bytes[index] = value;
        break;
    case 16:
        vector->halfwords[index] = value;
        break;
    case 32:
        vector->words[index] = value;
        break;
    case 64:
        vector->doublewords[index] = value;
        break;
    default:
        test_exit(2);
    }
}

static int64_t signed_lane(const Vector *vector, unsigned bits, unsigned index)
{
    switch (bits) {
    case 16:
        return (int16_t)vector->halfwords[index];
    case 32:
        return (int32_t)vector->words[index];
    case 64:
        return (int64_t)vector->doublewords[index];
    default:
        test_exit(2);
    }
}

static void check_widen(void)
{
    for (unsigned bits = 8; bits <= 32; bits *= 2) {
        unsigned count = bits == 8 ? 256 : bits == 16 ? 65536 : 2048;
        for (unsigned value = 0; value < count; value++) {
            Vector input = { 0 }, output;
            for (unsigned index = 0; index < 64 / bits; index++) {
                uint64_t sample = bits == 32 ?
                    UINT64_C(0x9e3779b9) * (value + index) :
                    value + index * 8191u;
                set_lane(&input, bits, index, sample);
            }
            for (unsigned shift = 0; shift <= bits; shift++) {
                for (unsigned overlap = SEPARATE;
                     overlap <= SOURCE_HIGH; overlap++) {
                    for (unsigned sticky = 0; sticky < 2; sticky++) {
                        uint32_t flags = FPSCR_OTHER_FLAGS |
                                         (sticky ? FPSCR_QC : 0);
                        write_fpscr(flags);
                        output.wide = widen(input.narrow, bits, shift, overlap);
                        if (read_fpscr() != flags) {
                            test_report("FAIL widening FPSCR");
                            test_exit(1);
                        }
                        for (unsigned index = 0; index < 64 / bits; index++) {
                            if (lane(&output, bits * 2, index) !=
                                lane(&input, bits, index) << shift) {
                                test_report("FAIL widening result");
                                test_exit(1);
                            }
                        }
                    }
                }
            }
        }
    }
    test_report("PASS widening: all shifts, both overlaps, FPSCR");
}

static void check_narrow(void)
{
    for (unsigned bits = 16; bits <= 64; bits *= 2) {
        unsigned count = bits == 16 ? 65536 : 2048;
        uint64_t maximum = (UINT64_C(1) << (bits / 2)) - 1;
        for (unsigned value = 0; value < count; value++) {
            Vector input = { 0 }, output;
            for (unsigned index = 0; index < 128 / bits; index++) {
                uint64_t sample;
                if (bits == 16) {
                    /* Full 16-bit domain; other lanes remain in range. */
                    sample = index == value % 8 ? value : index * 31;
                } else {
                    static const int64_t boundaries[] = {
                        0, -1, 1, 127, 128, 255, 256, 257, 32767, -32768,
                        65535, 65536, INT32_MIN, INT32_MAX,
                        INT64_MIN, INT64_MAX,
                    };
                    sample = value < 64 ?
                        (uint64_t)boundaries[(value + index) % 16] :
                        (UINT64_C(0x9e3779b97f4a7c15) * (value + index * 67));
                }
                set_lane(&input, bits, index, sample);
            }
            for (unsigned mode = SATURATE;
                 mode <= ROUND_SHIFT_SATURATE; mode++) {
                for (unsigned shift = mode == SATURATE ? 0 : 1;
                     shift <= (mode == SATURATE ? 0 : bits / 2); shift++) {
                    bool saturated = false;
                    uint64_t expected[8];
                    for (unsigned index = 0; index < 128 / bits; index++) {
                        int64_t original = signed_lane(&input, bits, index);
                        int64_t shifted = original >> shift;
                        if (mode == ROUND_SHIFT_SATURATE) {
                            shifted += ((uint64_t)original >> (shift - 1)) & 1;
                        }
                        if (shifted < 0) {
                            expected[index] = 0;
                            saturated = true;
                        } else if ((uint64_t)shifted > maximum) {
                            expected[index] = maximum;
                            saturated = true;
                        } else {
                            expected[index] = shifted;
                        }
                    }
                    for (unsigned overlap = SEPARATE;
                         overlap <= SOURCE_HIGH; overlap++) {
                        for (unsigned sticky = 0; sticky < 2; sticky++) {
                            uint32_t flags = FPSCR_OTHER_FLAGS |
                                             (sticky ? FPSCR_QC : 0);
                            uint32_t expected_flags = flags |
                                (saturated ? FPSCR_QC : 0);
                            write_fpscr(flags);
                            output.narrow = narrow(input.wide, bits, shift,
                                                   mode, overlap);
                            if (read_fpscr() != expected_flags) {
                                test_report("FAIL narrowing FPSCR/QC");
                                test_exit(1);
                            }
                            for (unsigned index = 0;
                                 index < 128 / bits; index++) {
                                if (lane(&output, bits / 2, index) !=
                                    expected[index]) {
                                    test_report("FAIL narrowing result");
                                    test_exit(1);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    test_report("PASS saturating narrowing: all shifts, overlaps, sticky QC");
}

static void check_prewiden(void)
{
    for (unsigned value = 0; value < 256; value++) {
        Vector left = { 0 }, right = { 0 }, wide = { 0 }, output;
        for (unsigned index = 0; index < 8; index++) {
            left.bytes[index] = value + index * 31;
            right.bytes[index] = 255 - value - index * 17;
            wide.halfwords[index] = value * 977 + index * 2000;
        }
        for (unsigned operation = 0; operation < 4; operation++) {
            uint16x8_t vector;
            switch (operation) {
            case 0:
                vector = vaddl_u8(left.narrow, right.narrow);
                break;
            case 1:
                vector = vsubl_u8(left.narrow, right.narrow);
                break;
            case 2:
                vector = vaddw_u8(vld1q_u16(wide.halfwords), right.narrow);
                break;
            default:
                vector = vsubw_u8(vld1q_u16(wide.halfwords), right.narrow);
                break;
            }
            output.wide = vreinterpretq_u8_u16(vector);
            for (unsigned index = 0; index < 8; index++) {
                unsigned first = operation < 2 ? left.bytes[index] :
                                                 wide.halfwords[index];
                unsigned second = right.bytes[index];
                uint16_t expected = operation & 1 ? first - second :
                                                    first + second;
                if (output.halfwords[index] != expected) {
                    test_report("FAIL prewiden arithmetic");
                    test_exit(1);
                }
            }
        }
    }
    test_report("PASS prewiden arithmetic");
}

int main(void)
{
    check_widen();
    check_narrow();
    check_prewiden();
    test_report("PASS all");
    test_exit(0);
}

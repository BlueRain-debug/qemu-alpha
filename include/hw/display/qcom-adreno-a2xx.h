/*
 * Experimental, bounded A2xx software drawing.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_DISPLAY_QCOM_ADRENO_A2XX_H
#define HW_DISPLAY_QCOM_ADRENO_A2XX_H

typedef struct QcomA2xxDrawContext {
    const uint32_t *registers;
    const uint32_t *shader[2];
    unsigned shader_words[2];
    uint32_t *gmem;
    size_t gmem_bytes;
    /* Opt-in diagnostic for QCDX CPU uploads; not a general endian rule. */
    bool upload_endian_workaround;
    void *opaque;
    bool (*read_word)(void *opaque, uint32_t address, uint32_t *value);
    /* Validate a primitive's destinations before committing its pixels. */
    bool (*writable_word)(void *opaque, uint32_t address);
    bool (*write_word)(void *opaque, uint32_t address, uint32_t value);
    const char *failure;
    unsigned failure_detail;
} QcomA2xxDrawContext;

bool qcom_a2xx_draw_rect(QcomA2xxDrawContext *context);
bool qcom_a2xx_draw_triangles(QcomA2xxDrawContext *context,
                               uint32_t index_address, unsigned index_count,
                               unsigned index_bytes);

#endif

/*
 * early boot framebuffer in guest ram
 * configured using fw_cfg
 *
 * Copyright Red Hat, Inc. 2017
 *
 * Author:
 *     Gerd Hoffmann <kraxel@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/loader.h"
#include "hw/display/ramfb.h"
#include "hw/display/bochs-vbe.h" /* for limits */
#include "ui/console.h"
#include "system/physmem.h"
#include "system/reset.h"
#include "exec/cpu-common.h"

struct QEMU_PACKED RAMFBCfg {
    uint64_t addr;
    uint32_t fourcc;
    uint32_t flags;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
};

typedef struct RAMFBCfg RAMFBCfg;

struct RAMFBState {
    DisplaySurface *ds;
    MemoryRegion *mr;
    ram_addr_t offset;
    hwaddr size;
    uint32_t stride, row_bytes;
    uint32_t width, height;
    struct RAMFBCfg cfg;
};

static void ramfb_unmap_display_surface(pixman_image_t *image, void *unused)
{
    void *data = pixman_image_get_data(image);
    uint32_t size = pixman_image_get_stride(image) *
        pixman_image_get_height(image);
    physical_memory_unmap(data, size, 0, 0);
}

static DisplaySurface *ramfb_create_display_surface(int width, int height,
                                                    pixman_format_code_t format,
                                                    hwaddr stride, hwaddr addr)
{
    DisplaySurface *surface;
    hwaddr size, mapsize, linesize;
    void *data;

    if (width < 16 || width > VBE_DISPI_MAX_XRES ||
        height < 16 || height > VBE_DISPI_MAX_YRES ||
        format == 0 /* unknown format */)
        return NULL;

    linesize = width * PIXMAN_FORMAT_BPP(format) / 8;
    if (stride == 0) {
        stride = linesize;
    }

    mapsize = size = stride * (height - 1) + linesize;
    data = physical_memory_map(addr, &mapsize, false);
    if (size != mapsize) {
        physical_memory_unmap(data, mapsize, 0, 0);
        return NULL;
    }

    surface = qemu_create_displaysurface_from(width, height,
                                              format, stride, data);
    pixman_image_set_destroy_function(surface->image,
                                      ramfb_unmap_display_surface, NULL);

    return surface;
}

static void ramfb_fw_cfg_write(void *dev, off_t offset, size_t len)
{
    RAMFBState *s = dev;
    DisplaySurface *surface;
    uint32_t fourcc, format, width, height;
    hwaddr stride, addr;

    width  = be32_to_cpu(s->cfg.width);
    height = be32_to_cpu(s->cfg.height);
    stride = be32_to_cpu(s->cfg.stride);
    fourcc = be32_to_cpu(s->cfg.fourcc);
    addr   = be64_to_cpu(s->cfg.addr);
    format = qemu_drm_format_to_pixman(fourcc);

    surface = ramfb_create_display_surface(width, height,
                                           format, stride, addr);
    if (!surface) {
        return;
    }

    if (s->mr) {
        memory_region_set_log(s->mr, false, DIRTY_MEMORY_VGA);
        memory_region_unref(s->mr);
    }
    s->mr = memory_region_from_host(surface_data(surface), &s->offset);
    if (s->mr) {
        memory_region_ref(s->mr);
        memory_region_set_log(s->mr, true, DIRTY_MEMORY_VGA);
    }
    s->stride = surface_stride(surface);
    s->row_bytes = width * PIXMAN_FORMAT_BPP(format) / 8;
    s->size = (hwaddr)s->stride * (height - 1) + s->row_bytes;
    s->width = width;
    s->height = height;
    qemu_free_displaysurface(s->ds);
    s->ds = surface;
}

void ramfb_display_update(QemuConsole *con, RAMFBState *s)
{
    bool full = s->ds != NULL;
    unsigned first = s->height, last = 0;

    if (!s->width || !s->height) {
        return;
    }

    if (s->ds) {
        qemu_console_set_surface(con, s->ds);
        s->ds = NULL;
    }

    if (s->mr) {
        g_autofree DirtyBitmapSnapshot *snap =
            memory_region_snapshot_and_clear_dirty(s->mr, s->offset, s->size,
                                                    DIRTY_MEMORY_VGA);

        for (unsigned y = 0; y < s->height; y++) {
            if (full || memory_region_snapshot_get_dirty(s->mr, snap,
                        s->offset + (hwaddr)y * s->stride, s->row_bytes)) {
                first = MIN(first, y);
                last = y + 1;
            }
        }
    } else {
        first = 0;
        last = s->height;
    }
    /* Avoid scaling the entire boot framebuffer for a small spinner update. */
    if (last > first) {
        qemu_console_update(con, 0, first, s->width, last - first);
    }
}

static int ramfb_post_load(void *opaque, int version_id)
{
    ramfb_fw_cfg_write(opaque, 0, 0);
    return 0;
}

const VMStateDescription ramfb_vmstate = {
    .name = "ramfb",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ramfb_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER_UNSAFE(cfg, RAMFBState, 0, sizeof(RAMFBCfg)),
        VMSTATE_END_OF_LIST()
    }
};

RAMFBState *ramfb_setup(bool romfile, Error **errp)
{
    FWCfgState *fw_cfg = fw_cfg_find();
    RAMFBState *s;

    if (!fw_cfg || !fw_cfg->dma_enabled) {
        error_setg(errp, "ramfb device requires fw_cfg with DMA");
        return NULL;
    }

    s = g_new0(RAMFBState, 1);

    if (romfile) {
        rom_add_vga("vgabios-ramfb.bin");
    }
    fw_cfg_add_file_callback(fw_cfg, "etc/ramfb",
                             NULL, ramfb_fw_cfg_write, s,
                             &s->cfg, sizeof(s->cfg), false);
    return s;
}

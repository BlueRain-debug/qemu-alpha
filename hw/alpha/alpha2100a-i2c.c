/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Experimental Lynx PCF8584 I2C master on ISA ports 0x530/0x531.
 * Register and transfer behavior: Philips PCF8584 datasheet, 1997-10-21.
 * DEC's published SDK iic_def.h/iic_driver.c was read to identify the
 * firmware interface; no DEC or other emulator implementation is copied.
 * Uses QEMU's I2C bus and EEPROM devices. Multi-master arbitration,
 * slave mode, electrical timing and migration are not implemented.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/isa/isa.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TYPE_LYNX_PCF8584 "alpha2100a-pcf8584"
OBJECT_DECLARE_SIMPLE_TYPE(LynxPCF8584, LYNX_PCF8584)
#define PIN 0x80
#define ESO 0x40
#define ENI 0x08
#define STA 0x04
#define STO 0x02
#define ACK 0x01
#define LRB 0x08
#define BB 0x01

struct LynxPCF8584 {
    ISADevice parent_obj;
    MemoryRegion io;
    I2CBus *bus;
    qemu_irq irq;
    QEMUTimer *timer;
    uint8_t control, status, own, clock, vector, data;
    uint8_t completion_status;
    bool master, receive, addressed;
};

static void pcf_irq(LynxPCF8584 *s)
{
    qemu_set_irq(s->irq, (s->control & ESO) && (s->control & ENI) &&
                        !(s->status & PIN));
}

static void pcf_complete(void *opaque)
{
    LynxPCF8584 *s = opaque;

    s->status = s->completion_status;
    pcf_irq(s);
}

static void pcf_schedule(LynxPCF8584 *s, bool nack)
{
    static const unsigned rates[] = { 90000, 45000, 11000, 1500 };

    s->status = PIN | (s->master ? 0 : BB);
    s->completion_status = (s->master ? 0 : BB) | (nack ? LRB : 0);
    pcf_irq(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              9 * NANOSECONDS_PER_SECOND / rates[s->clock & 3]);
}

static void pcf_stop(LynxPCF8584 *s)
{
    timer_del(s->timer);
    i2c_end_transfer(s->bus);
    s->master = s->addressed = s->receive = false;
    s->status = PIN | BB;
    pcf_irq(s);
}

static uint64_t pcf_read(void *opaque, hwaddr addr, unsigned size)
{
    LynxPCF8584 *s = opaque;
    uint8_t value;

    if (addr) {
        /* ESO=0 allows the low control bits to be tested during init. */
        value = (s->control & ESO) ? s->status :
                (s->status & PIN) | (s->control & 0x0f);
    } else if (!(s->control & ESO)) {
        switch (s->control & 0x30) {
        case 0x20: value = s->clock; break;
        case 0x10: value = s->vector; break;
        default: value = s->own; break;
        }
    } else {
        value = s->data;
        if (s->master && s->receive && s->addressed) {
            /* The first read discards the address and fetches byte zero. */
            s->data = i2c_recv(s->bus);
            if (!(s->control & ACK)) {
                i2c_nack(s->bus);
            }
            pcf_schedule(s, false);
        }
    }
    trace_alpha2100a_i2c(false, addr, value, s->status);
    return value;
}

static void pcf_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    LynxPCF8584 *s = opaque;

    trace_alpha2100a_i2c(true, addr, value, s->status);
    if (!addr) {
        if (!(s->control & ESO)) {
            switch (s->control & 0x30) {
            case 0x20: s->clock = value & 0x1f; break;
            case 0x10: s->vector = value; break;
            default: s->own = value & 0x7f; break;
            }
        } else {
            s->data = value;
            if (s->master && !s->receive) {
                pcf_schedule(s, !s->addressed || i2c_send(s->bus, value));
            }
        }
        return;
    }

    s->control = value;
    if (!(value & ESO)) {
        pcf_stop(s);
        return;
    }
    if (value & STO) {
        pcf_stop(s);
    } else if (value & STA) {
        s->master = true;
        s->receive = s->data & 1;
        s->addressed = !i2c_start_transfer(s->bus, s->data >> 1, s->receive);
        pcf_schedule(s, !s->addressed);
    } else if (value & PIN) {
        /* A control write acknowledges the previous byte interrupt. */
        s->status = PIN | (s->master ? 0 : BB);
        pcf_irq(s);
    } else {
        pcf_irq(s);
    }
}

static const MemoryRegionOps pcf_ops = {
    .read = pcf_read,
    .write = pcf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1, .valid.max_access_size = 1,
};

static void pcf_reset(DeviceState *dev)
{
    LynxPCF8584 *s = LYNX_PCF8584(dev);

    s->control = PIN;
    s->own = s->clock = s->vector = s->data = 0;
    pcf_stop(s);
}

static void pcf_realize(DeviceState *dev, Error **errp)
{
    LynxPCF8584 *s = LYNX_PCF8584(dev);

    s->bus = i2c_init_bus(dev, "lynx-i2c");
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pcf_complete, s);
    memory_region_init_io(&s->io, OBJECT(s), &pcf_ops, s, "lynx.pcf8584", 2);
    isa_register_ioport(ISA_DEVICE(s), &s->io, 0x530);
    qdev_init_gpio_out(dev, &s->irq, 1);
}

static void pcf_finalize(Object *obj)
{
    LynxPCF8584 *s = LYNX_PCF8584(obj);

    timer_free(s->timer);
}

static const VMStateDescription pcf_vmstate = {
    .name = "lynx-pcf8584",
    .unmigratable = true,
};

static void pcf_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = pcf_realize;
    device_class_set_legacy_reset(dc, pcf_reset);
    dc->vmsd = &pcf_vmstate;
    dc->user_creatable = false;
}

static const TypeInfo pcf_type = {
    .name = TYPE_LYNX_PCF8584,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(LynxPCF8584),
    .class_init = pcf_class_init,
    .instance_finalize = pcf_finalize,
};

static void pcf_register(void)
{
    type_register_static(&pcf_type);
}

type_init(pcf_register)

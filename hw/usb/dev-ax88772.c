/*
 * ASIX AX88772 USB 2.0 Ethernet controller.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Implements the original 0b95:7720 device, without checksum offload.
 * Vendor command and framing reference: Linux drivers/net/usb/asix*.c.
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/net/mii.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "net/eth.h"
#include "net/net.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "desc.h"
#include "trace.h"

#define TYPE_USB_AX88772 "usb-ax88772"
OBJECT_DECLARE_SIMPLE_TYPE(USBAX88772State, USB_AX88772)

#define AX_PHY_ADDR       0x10
#define AX_RX_START       0x0080
#define AX_RX_PHYSICAL    0x0020
#define AX_RX_MULTICAST   0x0010
#define AX_RX_BROADCAST   0x0008
#define AX_RX_ALLMULTI    0x0002
#define AX_RX_PROMISC     0x0001
#define AX_MEDIUM_RX      0x0100
#define AX_MAX_FRAME      1518
#define AX_TX_BUFSIZE     16384

struct USBAX88772State {
    USBDevice dev;
    NICConf conf;
    NICState *nic;
    USBEndpoint *bulk_in;
    USBEndpoint *intr;
    uint8_t mac[ETH_ALEN];
    uint16_t eeprom[256];
    uint16_t phy[32];
    uint16_t rx_ctl;
    uint16_t medium;
    uint8_t multicast[8];
    uint8_t ipg[3];
    uint8_t monitor;
    uint8_t gpio;
    uint8_t phy_select;
    uint8_t sw_reset;
    bool sw_mii;
    bool eeprom_write;
    bool intr_pending;
    uint8_t rx_buf[AX_MAX_FRAME + 5];
    uint32_t rx_len;
    uint32_t rx_pos;
    uint8_t tx_buf[AX_TX_BUFSIZE];
    uint32_t tx_len;
    bool tx_padding;
};

static const USBDescStrings ax_strings = {
    [1] = "ASIX Electronics Corp.",
    [2] = "AX88772 USB 2.0 Ethernet",
    [3] = "1",
};

#define AX_ENDPOINTS(packet, interval) { \
    { .bEndpointAddress = USB_DIR_IN | 1, \
      .bmAttributes = USB_ENDPOINT_XFER_INT, .wMaxPacketSize = 8, \
      .bInterval = interval }, \
    { .bEndpointAddress = USB_DIR_IN | 2, \
      .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = packet }, \
    { .bEndpointAddress = USB_DIR_OUT | 3, \
      .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = packet }, \
}

#define AX_CONFIG(packet, interval) { \
    .bNumInterfaces = 1, .bConfigurationValue = 1, \
    .bmAttributes = USB_CFG_ATT_ONE, .bMaxPower = 125, \
    .nif = 1, .ifs = (USBDescIface[]) { \
        { .bInterfaceNumber = 0, .bNumEndpoints = 3, \
          .bInterfaceClass = USB_CLASS_VENDOR_SPEC, \
          .eps = (USBDescEndpoint[]) AX_ENDPOINTS(packet, interval) }, \
    }, \
}

static const USBDescDevice ax_full = {
    .bcdUSB = 0x0200, .bMaxPacketSize0 = 64, .bNumConfigurations = 1,
    .confs = (USBDescConfig[]) { AX_CONFIG(64, 160) },
};

static const USBDescDevice ax_high = {
    .bcdUSB = 0x0200, .bMaxPacketSize0 = 64, .bNumConfigurations = 1,
    .confs = (USBDescConfig[]) { AX_CONFIG(512, 11) },
};

static const USBDesc ax_desc = {
    .id = { .idVendor = 0x0b95, .idProduct = 0x7720, .bcdDevice = 0x0001,
            .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3 },
    .full = &ax_full, .high = &ax_high, .str = ax_strings,
};

static bool ax_link_up(USBAX88772State *s)
{
    return !qemu_get_queue(s->nic)->link_down &&
           !(s->phy[MII_BMCR] & (MII_BMCR_PDOWN | MII_BMCR_ISOLATE)) &&
           !(s->sw_reset & 0x40);
}

static void ax_notify(USBAX88772State *s)
{
    trace_usb_ax88772_state("notify", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                           ax_link_up(s), s->rx_ctl, s->medium, s->sw_reset,
                           s->rx_len, s->rx_pos, s->tx_len);
    s->intr_pending = true;
    usb_wakeup(s->intr, 0);
}

static void ax_phy_reset(USBAX88772State *s)
{
    memset(s->phy, 0, sizeof(s->phy));
    s->phy[MII_BMCR] = MII_BMCR_SPEED100 | MII_BMCR_AUTOEN | MII_BMCR_FD;
    s->phy[MII_BMSR] = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD |
                      MII_BMSR_10T_FD | MII_BMSR_10T_HD | MII_BMSR_MFPS |
                      MII_BMSR_AUTONEG | MII_BMSR_EXTCAP;
    s->phy[MII_PHYID1] = 0x003b;
    s->phy[MII_PHYID2] = 0x1861;
    s->phy[MII_ANAR] = MII_ANAR_TXFD | MII_ANAR_TX | MII_ANAR_10FD |
                      MII_ANAR_10 | MII_ANAR_CSMACD | MII_ANAR_PAUSE;
    s->phy[MII_ANLPAR] = s->phy[MII_ANAR] | MII_ANLPAR_ACK;
    s->phy[MII_ANER] = MII_ANER_NWAY;
}

static uint16_t ax_phy_read(USBAX88772State *s, unsigned reg)
{
    uint16_t value = s->phy[reg];

    if (reg == MII_BMSR && ax_link_up(s)) {
        value |= MII_BMSR_LINK_ST | MII_BMSR_AN_COMP;
    }
    return value;
}

static void ax_reset(USBDevice *dev)
{
    USBAX88772State *s = USB_AX88772(dev);

    memcpy(s->mac, s->conf.macaddr.a, ETH_ALEN);
    s->rx_ctl = 0;
    s->medium = 0;
    s->rx_len = s->rx_pos = s->tx_len = 0;
    s->tx_padding = false;
    memset(s->multicast, 0, sizeof(s->multicast));
    s->ipg[0] = 0x15;
    s->ipg[1] = 0x0c;
    s->ipg[2] = 0x12;
    s->monitor = s->gpio = s->sw_reset = 0;
    s->phy_select = 1;
    s->sw_mii = s->eeprom_write = false;
    ax_phy_reset(s);
    trace_usb_ax88772_state("usb-reset", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                           ax_link_up(s), s->rx_ctl, s->medium, s->sw_reset,
                           s->rx_len, s->rx_pos, s->tx_len);
    ax_notify(s);
}

static void ax_control(USBDevice *dev, USBPacket *p, int request,
                       int value, int index, int length, uint8_t *data)
{
    USBAX88772State *s = USB_AX88772(dev);
    uint8_t reply[8] = { 0 };
    int size = 0;

    trace_usb_ax88772_control(request, value, index, length);
    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        if (request == (DeviceOutRequest | USB_REQ_SET_CONFIGURATION)) {
            ax_notify(s);
        }
        return;
    }
    switch (request) {
    case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
        if (value != 0 /* ENDPOINT_HALT */ || length ||
            (index != 0x81 && index != 0x82 && index != 0x03)) {
            goto stall;
        }
        if (index == 0x03) {
            s->tx_len = 0;
            s->tx_padding = false;
        } else if (index == 0x82) {
            s->rx_len = s->rx_pos = 0;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        } else {
            ax_notify(s);
        }
        usb_ep_set_halted(dev, index & USB_DIR_IN ? USB_TOKEN_IN : USB_TOKEN_OUT,
                          index & 0x0f, false);
        trace_usb_ax88772_state("clear-halt",
                               qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                               ax_link_up(s), s->rx_ctl, s->medium, s->sw_reset,
                               s->rx_len, s->rx_pos, s->tx_len);
        break;
    case VendorDeviceOutRequest | 0x06: /* software MII ownership */
        s->sw_mii = true;
        break;
    case VendorDeviceRequest | 0x07: /* PHY register */
        if (length != 2 || value != AX_PHY_ADDR || index >= 32) {
            goto stall;
        }
        stw_le_p(reply, ax_phy_read(s, index));
        size = 2;
        break;
    case VendorDeviceOutRequest | 0x08:
        if (length != 2 || value != AX_PHY_ADDR || index >= 32) {
            goto stall;
        }
        if (index == MII_BMCR) {
            uint16_t bmcr = lduw_le_p(data);
            if (bmcr & MII_BMCR_RESET) {
                ax_phy_reset(s);
            } else {
                s->phy[index] = bmcr & ~MII_BMCR_ANRESTART;
            }
            ax_notify(s);
        } else if (index != MII_BMSR && index != MII_PHYID1 &&
                   index != MII_PHYID2) {
            s->phy[index] = lduw_le_p(data);
        }
        break;
    case VendorDeviceRequest | 0x09: /* management ownership, chip code */
        reply[0] = s->sw_mii;
        size = 1;
        break;
    case VendorDeviceOutRequest | 0x0a:
        s->sw_mii = false;
        break;
    case VendorDeviceRequest | 0x0b: /* EEPROM word */
        if (value >= ARRAY_SIZE(s->eeprom)) {
            goto stall;
        }
        stw_le_p(reply, s->eeprom[value]);
        size = 2;
        break;
    case VendorDeviceOutRequest | 0x0c:
        if (value >= ARRAY_SIZE(s->eeprom) || !s->eeprom_write) {
            goto stall;
        }
        s->eeprom[value] = index;
        break;
    case VendorDeviceOutRequest | 0x0d:
    case VendorDeviceOutRequest | 0x0e:
        s->eeprom_write = (request & 0xff) == 0x0d;
        break;
    case VendorDeviceRequest | 0x0f:
        stw_le_p(reply, s->rx_ctl);
        size = 2;
        break;
    case VendorDeviceOutRequest | 0x10:
        s->rx_ctl = value;
        if (!(value & AX_RX_START)) {
            s->rx_len = s->rx_pos = 0;
        }
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case VendorDeviceRequest | 0x11:
        memcpy(reply, s->ipg, 3);
        size = 3;
        break;
    case VendorDeviceOutRequest | 0x12:
        s->ipg[0] = value;
        s->ipg[1] = value >> 8;
        s->ipg[2] = index;
        break;
    case VendorDeviceRequest | 0x13:
        memcpy(reply, s->mac, ETH_ALEN);
        size = ETH_ALEN;
        break;
    case VendorDeviceOutRequest | 0x14:
        if (length != ETH_ALEN) {
            goto stall;
        }
        memcpy(s->mac, data, ETH_ALEN);
        break;
    case VendorDeviceRequest | 0x15:
        memcpy(reply, s->multicast, sizeof(s->multicast));
        size = sizeof(s->multicast);
        break;
    case VendorDeviceOutRequest | 0x16:
        if (length != sizeof(s->multicast)) {
            goto stall;
        }
        memcpy(s->multicast, data, sizeof(s->multicast));
        break;
    case VendorDeviceOutRequest | 0x17: /* test mode, no physical PHY */
        break;
    case VendorDeviceRequest | 0x19:
        reply[0] = 0;
        reply[1] = AX_PHY_ADDR;
        size = 2;
        break;
    case VendorDeviceRequest | 0x1a:
        stw_le_p(reply, s->medium);
        size = 2;
        break;
    case VendorDeviceOutRequest | 0x1b:
        s->medium = value;
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case VendorDeviceRequest | 0x1c:
        reply[0] = s->monitor;
        size = 1;
        break;
    case VendorDeviceOutRequest | 0x1d:
        s->monitor = value;
        break;
    case VendorDeviceRequest | 0x1e:
        reply[0] = s->gpio;
        size = 1;
        break;
    case VendorDeviceOutRequest | 0x1f:
        s->gpio = value;
        break;
    case VendorDeviceOutRequest | 0x20:
        s->sw_reset = value;
        if (value & 3) {
            s->rx_len = s->rx_pos = s->tx_len = 0;
            s->tx_padding = false;
        }
        ax_notify(s);
        break;
    case VendorDeviceRequest | 0x21:
        reply[0] = s->phy_select;
        size = 1;
        break;
    case VendorDeviceOutRequest | 0x22:
        s->phy_select = value;
        break;
    default:
        goto stall;
    }
    if (request & (USB_DIR_IN << 8)) {
        p->actual_length = MIN(length, size);
        memcpy(data, reply, p->actual_length);
    }
    return;

stall:
    trace_usb_ax88772_error("control", 0, request, length, value, index);
    qemu_log_mask(LOG_UNIMP, "ax88772: unsupported control "
                  "%04x value %04x index %04x length %d\n",
                  request, value, index, length);
    p->status = USB_RET_STALL;
}

static bool ax_rx_enabled(USBAX88772State *s)
{
    return s->dev.config && ax_link_up(s) && (s->rx_ctl & AX_RX_START) &&
           (s->medium & AX_MEDIUM_RX);
}

static bool ax_accept(USBAX88772State *s, const uint8_t *buf)
{
    unsigned hash;

    if (s->rx_ctl & AX_RX_PROMISC) {
        return true;
    }
    if (is_broadcast_ether_addr(buf)) {
        return s->rx_ctl & AX_RX_BROADCAST;
    }
    if (is_multicast_ether_addr(buf)) {
        hash = net_crc32(buf, ETH_ALEN) >> 26;
        return (s->rx_ctl & AX_RX_ALLMULTI) ||
               ((s->rx_ctl & AX_RX_MULTICAST) &&
                (s->multicast[hash >> 3] & (1 << (hash & 7))));
    }
    hash = net_crc32(buf, ETH_ALEN) >> 26;
    return !memcmp(buf, s->mac, ETH_ALEN) ||
           ((s->rx_ctl & AX_RX_PHYSICAL) &&
            (s->multicast[hash >> 3] & (1 << (hash & 7))));
}

static ssize_t ax_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    USBAX88772State *s = qemu_get_nic_opaque(nc);
    unsigned padded = MAX(size, ETH_ZLEN);

    if (size < ETH_HLEN || size > AX_MAX_FRAME || !ax_rx_enabled(s) ||
        !ax_accept(s, buf)) {
        return size;
    }
    if (s->rx_len) {
        return 0;
    }
    stw_le_p(s->rx_buf, padded);
    stw_le_p(s->rx_buf + 2, ~padded);
    memcpy(s->rx_buf + 4, buf, size);
    memset(s->rx_buf + 4 + size, 0, ROUND_UP(padded, 2) - size);
    s->rx_len = 4 + ROUND_UP(padded, 2);
    s->rx_pos = 0;
    trace_usb_ax88772_packet(false, size);
    usb_wakeup(s->bulk_in, 0);
    return size;
}

static bool ax_tx_header_valid(const uint8_t *buf)
{
    unsigned len = lduw_le_p(buf);
    unsigned inverse = lduw_le_p(buf + 2);

    return (len ^ inverse) == 0xffff && len <= AX_MAX_FRAME;
}

static void ax_data(USBDevice *dev, USBPacket *p)
{
    USBAX88772State *s = USB_AX88772(dev);
    unsigned max_packet = dev->speed == USB_SPEED_HIGH ? 512 : 64;

    if (p->pid == USB_TOKEN_IN && p->ep->nr == 1) {
        uint8_t status[8] = { 0 };
        if (!s->intr_pending) {
            p->status = USB_RET_NAK;
            return;
        }
        status[0] = 0xa1;
        status[2] = ax_link_up(s);
        stw_le_p(status + 6, ax_phy_read(s, MII_BMSR));
        trace_usb_ax88772_state("interrupt", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                               ax_link_up(s), s->rx_ctl, s->medium, s->sw_reset,
                               s->rx_len, s->rx_pos, s->tx_len);
        usb_packet_copy(p, status, MIN(p->iov.size, sizeof(status)));
        s->intr_pending = false;
    } else if (p->pid == USB_TOKEN_IN && p->ep->nr == 2) {
        unsigned len;
        if (!s->rx_len) {
            p->status = USB_RET_NAK;
            return;
        }
        len = MIN(s->rx_len - s->rx_pos, p->iov.size);
        usb_packet_copy(p, s->rx_buf + s->rx_pos, len);
        s->rx_pos += len;
        if (s->rx_pos == s->rx_len && (!len || len % max_packet)) {
            s->rx_len = s->rx_pos = 0;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    } else if (p->pid == USB_TOKEN_OUT && p->ep->nr == 3) {
        if (p->iov.size > sizeof(s->tx_buf) - s->tx_len) {
            trace_usb_ax88772_error("tx-overflow", 3, p->pid, p->iov.size,
                                   s->tx_len, sizeof(s->tx_buf));
            s->tx_len = 0;
            s->tx_padding = false;
            p->status = USB_RET_STALL;
            return;
        }
        usb_packet_copy(p, s->tx_buf + s->tx_len, p->iov.size);
        s->tx_len += p->iov.size;
        while (s->tx_len >= 4) {
            unsigned len, inverse, framed;

            /*
             * Windows aligns odd-length frames to two bytes, including when
             * aggregating frames. Other drivers send an unpadded frame.
             * Only skip the optional byte if a valid header follows it;
             * retain this state when a USB packet splits that header.
             */
            if (s->tx_padding && !ax_tx_header_valid(s->tx_buf)) {
                if (s->tx_len < 5) {
                    break;
                }
                if (ax_tx_header_valid(s->tx_buf + 1)) {
                    trace_usb_ax88772_tx_padding(s->tx_len);
                    s->tx_len--;
                    memmove(s->tx_buf, s->tx_buf + 1, s->tx_len);
                }
            }
            s->tx_padding = false;
            len = lduw_le_p(s->tx_buf);
            inverse = lduw_le_p(s->tx_buf + 2);
            framed = 4 + len;
            if ((len ^ inverse) != 0xffff || len > AX_MAX_FRAME) {
                trace_usb_ax88772_error("tx-header", 3, p->pid, p->iov.size,
                                       len, inverse);
                s->tx_len = 0;
                s->tx_padding = false;
                p->status = USB_RET_STALL;
                return;
            }
            if (s->tx_len < framed) {
                break;
            }
            if (len >= ETH_HLEN && ax_link_up(s)) {
                trace_usb_ax88772_packet(true, len);
                qemu_send_packet(qemu_get_queue(s->nic), s->tx_buf + 4, len);
            }
            s->tx_len -= framed;
            memmove(s->tx_buf, s->tx_buf + framed, s->tx_len);
            s->tx_padding = len & 1;
        }
        if (p->iov.size % max_packet || !p->iov.size) {
            s->tx_len = 0;
            s->tx_padding = false;
        }
    } else {
        trace_usb_ax88772_error("endpoint", p->ep->nr, p->pid,
                               p->iov.size, s->rx_pos, s->tx_len);
        p->status = USB_RET_STALL;
    }
}

static void ax_link_changed(NetClientState *nc)
{
    USBAX88772State *s = qemu_get_nic_opaque(nc);
    ax_notify(s);
    qemu_flush_queued_packets(nc);
}

static void ax_cleanup(NetClientState *nc)
{
    USBAX88772State *s = qemu_get_nic_opaque(nc);
    s->nic = NULL;
}

static NetClientInfo ax_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = ax_receive,
    .link_status_changed = ax_link_changed,
    .cleanup = ax_cleanup,
};

static void ax_realize(USBDevice *dev, Error **errp)
{
    USBAX88772State *s = USB_AX88772(dev);
    int i;

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    s->bulk_in = usb_ep_get(dev, USB_TOKEN_IN, 2);
    s->intr = usb_ep_get(dev, USB_TOKEN_IN, 1);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ax_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->qdev.id,
                          &dev->qdev.mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    memset(s->eeprom, 0, sizeof(s->eeprom));
    s->eeprom[1] = 0x0b95;
    s->eeprom[2] = 0x7720;
    s->eeprom[3] = 1;
    for (i = 0; i < ETH_ALEN / 2; i++) {
        s->eeprom[4 + i] = lduw_le_p(s->conf.macaddr.a + i * 2);
    }
    s->eeprom[0x0f] = 0x0001; /* interrupt endpoint reports PHY BMSR */
    ax_reset(dev);
}

static void ax_unrealize(USBDevice *dev)
{
    USBAX88772State *s = USB_AX88772(dev);
    qemu_del_nic(s->nic);
}

static const Property ax_properties[] = {
    DEFINE_NIC_PROPERTIES(USBAX88772State, conf),
};

static const VMStateDescription ax_vmstate = {
    .name = TYPE_USB_AX88772,
    .unmigratable = 1,
};

static void ax_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->realize = ax_realize;
    uc->unrealize = ax_unrealize;
    uc->usb_desc = &ax_desc;
    uc->product_desc = "ASIX AX88772 USB Ethernet";
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = ax_reset;
    uc->handle_control = ax_control;
    uc->handle_data = ax_data;
    dc->vmsd = &ax_vmstate;
    dc->desc = "ASIX AX88772 USB 2.0 Ethernet adapter";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    device_class_set_props(dc, ax_properties);
}

static const TypeInfo ax_info = {
    .name = TYPE_USB_AX88772,
    .parent = TYPE_USB_DEVICE,
    .instance_size = sizeof(USBAX88772State),
    .class_init = ax_class_init,
};

static void ax_register_types(void)
{
    type_register_static(&ax_info);
}

type_init(ax_register_types)

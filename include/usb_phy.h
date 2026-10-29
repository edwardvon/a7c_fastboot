#ifndef A7C_USB_PHY_H
#define A7C_USB_PHY_H

#include "io.h"

#define USB_OTG_BASE        0xF2920000u
#define USB_OTG_STATUS      (USB_OTG_BASE + 0x010u)  /* bit0 VBUS, bit1 ID */
#define USB_OTG_CTRL0       (USB_OTG_BASE + 0x040u)
#define USB_OTG_CTRL1       (USB_OTG_BASE + 0x050u)

void usb_phy_init(void);
u32  usb_otg_status(void);

#endif /* A7C_USB_PHY_H */

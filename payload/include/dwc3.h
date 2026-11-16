#ifndef A7C_DWC3_H
#define A7C_DWC3_H

#include "io.h"

enum dwc3_status {
    DWC3_OK              =  0,
    DWC3_ERR_SOFTRESET   = -2,   /* DCTL.CSFTRST never cleared */
    DWC3_ERR_NOT_STARTED = -3,   /* DSTS.DEVCTRLHLT never cleared */
    DWC3_ERR_NPSID       = -4,   /* GSNPSID is not 0x5533xxxx */
    DWC3_ERR_TIMEOUT     = -5,   /* 端点命令的 CMDACT 未在轮询窗口内清零 */
};

/* Core bring-up to "device mode, high speed, ready to be told to connect".
 * Assumes usb_phy_init() already powered the PHY and ungated the clocks. */
int  dwc3_core_init(void);

int  dwc3_connect(void);
void dwc3_disconnect(void);

void dwc3_set_address(u32 addr);

/* ---- endpoint layer ----------------------------------------------------- */

int  dwc3_depcmd(u32 phys_ep, u32 cmd, u32 p0, u32 p1, u32 p2);
void dwc3_wait_tick(void);

int  dwc3_ep_start_config(void);
int  dwc3_ep_config(u32 phys_ep, u32 type, u32 maxpacket);

void dwc3_ep_enable(u32 phys_ep);
void dwc3_ep_disable(u32 phys_ep);

int  dwc3_ep_start_xfer(u32 phys_ep, u64 trb_addr, u32 *rsc_idx);
int  dwc3_ep_end_xfer(u32 phys_ep, u32 rsc_idx);

int  dwc3_ep_set_stall(u32 phys_ep);
int  dwc3_ep_clear_stall(u32 phys_ep);

/* 0 means the event buffer is empty. No legal event decodes to 0: a device
 * event always has bit0 set, an endpoint event always carries a non-zero
 * endpoint number or event type. */
u32  dwc3_event_poll(void);

#endif /* A7C_DWC3_H */

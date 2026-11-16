#ifndef A7C_USB_H
#define A7C_USB_H

#include "io.h"

/* ---- USB 2.0 chapter 9 -------------------------------------------------- */

#define USB_DIR_OUT                 0x00
#define USB_DIR_IN                  0x80

#define USB_TYPE_MASK               0x60
#define USB_TYPE_STANDARD           0x00

#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09
#define USB_REQ_GET_INTERFACE       0x0A
#define USB_REQ_SET_INTERFACE       0x0B

#define USB_DT_DEVICE               0x01
#define USB_DT_CONFIG               0x02
#define USB_DT_STRING               0x03
#define USB_DT_INTERFACE            0x04
#define USB_DT_ENDPOINT             0x05
#define USB_DT_DEVICE_QUALIFIER     0x06

#define USB_ENDPOINT_XFER_BULK      0x02

#define EP0_MAXPACKET               64u
#define BULK_MAXPACKET              512u
#define EP_BULK_OUT_ADDR            0x01
#define EP_BULK_IN_ADDR             0x81

struct usb_ctrlrequest {
    u8  bRequestType;
    u8  bRequest;
    u16 wValue;
    u16 wIndex;
    u16 wLength;
} __attribute__((packed));

/* ---- gadget ------------------------------------------------------------- */

enum usb_bulk_error {
    USB_BULK_ERR_NONE        = 0,
    USB_BULK_ERR_CFG_OUT     = 9,
    USB_BULK_ERR_CFG_IN      = 10,
    USB_BULK_ERR_OUT_CMD     = 11,
    USB_BULK_ERR_IN_CMD      = 12,
    USB_BULK_ERR_IN_TIMEOUT  = 13,
};

extern int usb_bulk_error;

/* Configures ep0 and arms the first SETUP. Must run before the D+ pullup. */
int  usb_gadget_init(void);

/* Waits for SET_CONFIGURATION, then hands over to fastboot_loop(). */
void usb_gadget_run(void);

void usb_event_pump(void);
void usb_bulk_enable_pending(void);
int  usb_is_configured(void);

u32  usb_bulk_recv(void *buf, u32 len);
u32  usb_bulk_send(const void *buf, u32 len);
void usb_bulk_abort(void);
int  usb_bulk_recover(void);

/* Monotonic counter bumped by every bus reset. Comparing a snapshot taken
 * before a transfer with a later one tells "the host reset us" apart from
 * "the transfer timed out", which usb_bulk_error alone cannot do (on_reset()
 * clears it). */
u32  usb_bulk_epoch(void);

/*
 * Snapshot taken at the bulk IN deadline (round 73).
 *
 * up_err=0x100 with uerr=0x0D/epch=0/xfer=0 only proves "nothing finished in
 * 0.5 s"; it cannot tell the two mechanisms apart. This does:
 *   hwo=1            the controller never fetched the TRB (STARTTRANSFER was
 *                    accepted but not consumed)
 *   hwo=0 & evt=0    fetched, yet no completion event ever arrived
 *   late>0           it completed, just after the deadline
 */
struct usb_bulk_diag {
    u32 hwo;    /* TRB HWO at the deadline: 1 = still owned by the controller */
    u32 tsz;    /* TRB.size then: == len when the transfer never started */
    u32 cmd;    /* DEPCMD(EP_IN) raw: bit10 CMDACT plus the status field */
    u32 gev;    /* GEVNTCOUNT(0) pending event bytes */
    u32 evt;    /* EP_IN xfer events seen during this send */
    u32 typ;    /* type of the last EP_IN dep event (0 = none) */
    u32 late;   /* ticks past the deadline when it did complete, 0 = never */

    /*
     * Round 82 (route B): what ENDTRANSFER did. usb_bulk_abort() used to throw
     * the command status away and assume the endpoint was freed, which is why a
     * timeout could be followed by "every later chunk fails immediately".
     *   end_st=0, end_hwo=0   the TRB was handed back, the abort worked
     *   end_st=0, end_hwo=1   command accepted, controller never released it
     *   end_st!=0             the endpoint command itself was rejected
     */
    u32 end_st;   /* ENDTRANSFER command status for EP_IN (0 = accepted) */
    u32 end_hwo;  /* TRB HWO after the abort wait: 1 = still controller-owned */
    u32 end_wt;   /* ticks spent waiting for that release (== limit means hung) */

    /*
     * Round 86: the 3882 contradiction. Round 85 read dg_xwt=3882 -- which is
     * almost exactly how long the ENDTRANSFER command itself takes -- while the
     * next instruction read HWO=1. The wait loop can only leave early on a
     * zero, so one single sample decided the exit and it disagreed with the
     * very next one.
     *   end_clr  ticks to a release confirmed by TWO consecutive zero reads;
     *            0 = HWO never read zero twice, i.e. the endpoint really is stuck
     *   end_re   ticks to the first read that put HWO back to 1 after such a
     *            release; 0 = the release held, !=0 = a stale transfer was
     *            still queued and re-armed the same TRB
     */
    u32 end_clr;
    u32 end_re;
};

const struct usb_bulk_diag *usb_bulk_diag(void);

#endif /* A7C_USB_H */

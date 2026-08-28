/*
 *	PearPC
 *	usb.cc
 *
 *	Copyright (C) 2003 Sebastian Biallas (sb@biallas.net)
 *
 *	References:
 *	[1] OpenHCI - Open Host Controller Interface Specification for USB
 *	              Revision 1.0a - hcir1_0a.pdf
 *	[2] Linux USB ohci-driver
 *	    (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 *	    (C) Copyright 2000-2001 David Brownell <dbrownell@users.sourceforge.net>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "debug/tracers.h"
#include "system/arch/sysendian.h"
#include "io/pci/pci.h"
#include "io/pic/pic.h"
#include "cpu/mem.h"
#include "cpu/cpu.h"
#include "system/systhread.h"
#include "usb.h"
#include "usbhid.h"

#include <cstring>

#define NUM_INTS 32		/* part of the OHCI standard */
#define MAX_ROOT_PORTS	15	/* maximum OHCI root hub ports */

struct ohci_hcca {
	uint32	int_table[NUM_INTS];	/* Interrupt ED table */
	uint16	frame_no;		/* current frame number */
	uint16	pad1;			/* set to 0 on each frame_no change */
	uint32	done_head;		/* info returned for an interrupt */
	uint8	reserved_for_hc[116];
} PACKED;

// [1].122
#define OHCI_REG_REVISION	0x00	// [1].123
#define OHCI_REG_CONTROL	0x04	// [1].123
#define OHCI_REG_CMDSTATUS	0x08	// [1].126
#define OHCI_REG_INTRSTATUS	0x0c	// [1].126
#define OHCI_REG_INTRENABLE	0x10	// [1].126
#define OHCI_REG_INTRDISABLE	0x14	// [1].126
#define OHCI_REG_HCCA		0x18	// [1].126
#define OHCI_REG_ED_PERIODCUR	0x1c	// [1].126
#define OHCI_REG_ED_CONTROL_HD	0x20	// [1].126
#define OHCI_REG_ED_CONTROL_CUR	0x24	// [1].126
#define OHCI_REG_ED_BULK_HD	0x28	// [1].126
#define OHCI_REG_ED_BULK_CUR	0x2c	// [1].126
#define OHCI_REG_DONEHEAD	0x30	// [1].126
#define OHCI_REG_FMINTERVAL	0x34	// [1].126
#define OHCI_REG_FMREMAIN	0x38	// [1].126
#define OHCI_REG_FMNUMBER	0x3c	// [1].126
#define OHCI_REG_PERIODICSTART	0x40	// [1].126
#define OHCI_REG_LSTHRESH	0x44	// [1].126
#define OHCI_REG_ROOTHUB_A	0x48
#define OHCI_REG_ROOTHUB_B	0x4c
#define OHCI_REG_ROOTHUB_STAT	0x50
#define OHCI_REG_ROOTHUB_PORTS	0x54

/*
 *	bits in ohci_hcregs.control
 */
#define OHCI_CTRL_CBSR	(3 << 0)	/* control/bulk service ratio */
#define OHCI_CTRL_PLE	(1 << 2)	/* periodic list enable */
#define OHCI_CTRL_IE	(1 << 3)	/* isochronous enable */
#define OHCI_CTRL_CLE	(1 << 4)	/* control list enable */
#define OHCI_CTRL_BLE	(1 << 5)	/* bulk list enable */
#define OHCI_CTRL_HCFS	(3 << 6)	/* host controller functional state */
#define OHCI_CTRL_IR	(1 << 8)	/* interrupt routing */
#define OHCI_CTRL_RWC	(1 << 9)	/* remote wakeup connected */
#define OHCI_CTRL_RWE	(1 << 10)	/* remote wakeup enable */

/* pre-shifted values for HCFS */
#	define OHCI_USB_RESET	(0 << 6)
#	define OHCI_USB_RESUME	(1 << 6)
#	define OHCI_USB_OPER	(2 << 6)
#	define OHCI_USB_SUSPEND	(3 << 6)

/*
 *	bits in ohci_hcregs.{intrstatus|intrenable|intrdisable}
 */
#define OHCI_INTR_SO	(1 << 0)	/* scheduling overrun */
#define OHCI_INTR_WDH	(1 << 1)	/* writeback of done_head */
#define OHCI_INTR_SF	(1 << 2)	/* start frame */
#define OHCI_INTR_RD	(1 << 3)	/* resume detect */
#define OHCI_INTR_UE	(1 << 4)	/* unrecoverable error */
#define OHCI_INTR_FNO	(1 << 5)	/* frame number overflow */
#define OHCI_INTR_RHSC	(1 << 6)	/* root hub status change */
#define OHCI_INTR_OC	(1 << 30)	/* ownership change */
#define OHCI_INTR_MIE	(1 << 31)	/* master interrupt enable */

/*
 *	bits in ohci_hcregs.cmdstatus
 */
#define OHCI_HCR	(1 << 0)	/* host controller reset */
#define OHCI_CLF  	(1 << 1)	/* control list filled */
#define OHCI_BLF  	(1 << 2)	/* bulk list filled */
#define OHCI_OCR  	(1 << 3)	/* ownership change request */
#define OHCI_SOC  	(3 << 16)	/* scheduling overrun count */

/*
 *	bits in ohci_hcregs.roothub.portstatus [1].142
 */
#define OHCI_RH_PS_CCS	(1 << 0)	/* current connect status */
#define OHCI_RH_PS_PES	(1 << 1)	/* port enable status */
#define OHCI_RH_PS_PSS	(1 << 2)	/* port suspend status */
#define OHCI_RH_PS_POCI	(1 << 3)	/* port overrun current indicator */
#define OHCI_RH_PS_PRS	(1 << 4)	/* port reset status */
#define OHCI_RH_PS_PPS	(1 << 8)	/* port power status */
#define OHCI_RH_PS_LSDA	(1 << 9)	/* low speed device attached */
#define OHCI_RH_PS_CSC	(1 << 16)	/* connect status change */
#define OHCI_RH_PS_PESC	(1 << 17)	/* port enable status change */
#define OHCI_RH_PS_PSSC	(1 << 18)	/* port suspend status change */
#define OHCI_RH_PS_OCIC	(1 << 19)	/* overrun current indicator change */
#define OHCI_RH_PS_PRSC	(1 << 20)	/* port reset status change */

/*
 *	bits in ohci_roothub_regs.status, [1].7.4.3.  Writes are set/clear
 *	commands, not a plain store.
 */
#define OHCI_RH_HS_LPS	(1 << 0)	/* w: ClearGlobalPower  r: local power status */
#define OHCI_RH_HS_OCI	(1 << 1)	/* r: over current indicator */
#define OHCI_RH_HS_DRWE	(1 << 15)	/* w: SetRemoteWakeupEnable  r: enabled */
#define OHCI_RH_HS_LPSC	(1 << 16)	/* w: SetGlobalPower */
#define OHCI_RH_HS_OCIC	(1 << 17)	/* over current indicator change */
#define OHCI_RH_HS_CRWE	(1u << 31)	/* w: ClearRemoteWakeupEnable */

struct ohci_hcregs {
	/* control and status registers */
	uint32	control;
	uint32	cmdstatus;
	uint32	intrstatus;
	uint32	intrenable;
	uint32	intrdisable;
	/* memory pointers */
	uint32	hcca;
	uint32	ed_periodcurrent;
	uint32	ed_controlhead;
	uint32	ed_controlcurrent;
	uint32	ed_bulkhead;
	uint32	ed_bulkcurrent;
	uint32	donehead;
	/* frame counters */
	uint32	fminterval;
	uint32	fmremaining;
	uint32	fmnumber;
	uint32	periodicstart;
	uint32	lsthresh;
	/* Root hub ports */
	struct	ohci_roothub_regs {
		uint32	a;
		uint32	b;
		uint32	status;
		uint32	portstatus[MAX_ROOT_PORTS];
	} roothub;
};
 
static inline const char *hc_regname(uint32 a)
{
	a >>= 2;
	if (a > 20) return "unknown";
	const char *names[] = {"revision","control","cmdstatus","intrstatus","intrenable",
	"intrdisable","hcca","ed_periodcurrent","ed_controlhead","ed_controlcurrent",
	"ed_bulkhead","ed_bulkcurrent","donehead","fminterval","fmremaining",
	"fmnumber","periodicstart", "lsthresh", "roothub.a", "roothub.b", "roothub.status"};
	return names[a];
}

/*
 *	Endpoint and transfer descriptors, [1].4.2 and [1].4.3.  Both are four
 *	little-endian words; the host controller owns them while the ED is on a
 *	list.
 */
#define OHCI_ED_SKIP		(1 << 14)
#define OHCI_ED_DIR_TD		0	/* direction comes from the TD	*/
#define OHCI_ED_DIR_OUT		1
#define OHCI_ED_DIR_IN		2
#define OHCI_ED_HALTED		1	/* bit 0 of HeadP		*/
#define OHCI_ED_TOGGLE		2	/* bit 1 of HeadP		*/

#define OHCI_TD_DP_SETUP	0
#define OHCI_TD_DP_OUT		1
#define OHCI_TD_DP_IN		2

#define OHCI_CC_NOERROR		0
#define OHCI_CC_STALL		4

/* HCCA layout, [1].4.4 */
#define OHCI_HCCA_FRAMENO	0x80
#define OHCI_HCCA_DONEHEAD	0x84

/* milliseconds between heartbeats, and therefore frames accounted per tick */
#define USB_FRAMES_PER_TICK	8
/* heartbeats of silence after which the controller stops touching guest memory */
#define USB_IDLE_TICKS		125

extern bool gSinglestep;

/* Serialises register access against the frame thread. */
extern sys_mutex gUSBRegMutex;
extern int gUSBRegWrites, gUSBFrames, gUSBEDs, gUSBTDs, gUSBCtrl, gUSBIntIn, gUSBReports;
int gHIDTraceArmed;
uint32 gHIDReportBuf;
int gHIDReportLen;
static volatile int gUSBPendingTicks;
extern int gUSBRegReads, gUSBIRQs, gUSBPortReads;
extern int gUSBReadHist[64];
int gUSBReadHist[64];
extern int gUSBWriteHist[64];
int gUSBWriteHist[64];
int gUSBRegReads, gUSBIRQs, gUSBPortReads;
struct USBRegLock {
	USBRegLock()  { sys_lock_mutex(gUSBRegMutex); }
	~USBRegLock() { sys_unlock_mutex(gUSBRegMutex); }
};

/*
 *
 */
class PCI_USB: public PCI_Device {
public:
	ohci_hcregs hcregs;
	uint rootport_count;

	/*
	 * A G4 Cube has no ADB port: Mac OS takes keyboard and mouse input from
	 * USB HID, so the root hub carries one of each.
	 */
	USBHIDDevice mDevices[USBHID_PORT_COUNT];

	/* Control transfers span several TDs; the reply is built at the SETUP
	 * stage and drained by the IN TDs that follow. */
	struct CtrlState {
		uint8	setup[8];
		uint8	buf[260];
		int	len;
		int	pos;
	} mCtrl[USBHID_PORT_COUNT];

	uint32	mDoneHead;	/* TDs finished this frame, newest first */
	int	mIdleTicks;	/* heartbeats since the guest last touched a register */
	bool	mIRQAsserted;	/* the level-triggered line we hold on the PIC */
	int	mReannounce;	/* ticks since the last connect re-announcement */
	int	mSFTick;	/* ticks since start-of-frame last raised an IRQ */
	bool	mSFDue;		/* start-of-frame may assert on this pass */
	int	mAttachDelay;	/* frames until the devices are plugged in, -1 = idle */

uint32 dmaRead32(uint32 addr)
{
	uint8 b[4];
	if (!ppc_dma_read(b, addr, 4)) return 0;
	return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
}

void dmaWrite32(uint32 addr, uint32 v)
{
	uint8 b[4] = { (uint8)v, (uint8)(v >> 8), (uint8)(v >> 16), (uint8)(v >> 24) };
	ppc_dma_write(addr, b, 4);
}

void dmaWrite16(uint32 addr, uint16 v)
{
	uint8 b[2] = { (uint8)v, (uint8)(v >> 8) };
	ppc_dma_write(addr, b, 2);
}

/* A TD buffer may straddle exactly one page boundary, [1].4.3.1.2. */
int bufLength(uint32 cbp, uint32 be)
{
	if (!cbp) return 0;
	if ((cbp & ~0xfffU) == (be & ~0xfffU)) return be - cbp + 1;
	return (0x1000 - (cbp & 0xfff)) + (be & 0xfff) + 1;
}

void bufRead(uint32 cbp, uint32 be, uint8 *dst, int len)
{
	if (len <= 0) return;
	int first = len;
	if ((cbp & ~0xfffU) != (be & ~0xfffU)) {
		first = 0x1000 - (cbp & 0xfff);
		if (first > len) first = len;
	}
	ppc_dma_read(dst, cbp, first);
	if (first < len) ppc_dma_read(dst + first, be & ~0xfffU, len - first);
}

void bufWrite(uint32 cbp, uint32 be, const uint8 *src, int len)
{
	if (len <= 0) return;
	int first = len;
	if ((cbp & ~0xfffU) != (be & ~0xfffU)) {
		first = 0x1000 - (cbp & 0xfff);
		if (first > len) first = len;
	}
	ppc_dma_write(cbp, src, first);
	if (first < len) ppc_dma_write(be & ~0xfffU, src + first, len - first);
}

/*
 * Only a port the host has enabled may answer.  This is what keeps the two
 * devices apart during enumeration, when both still have address 0.
 */
int findDevice(int addr)
{
	for (int i = 0; i < USBHID_PORT_COUNT && i < (int)rootport_count; i++) {
		if (!(hcregs.roothub.portstatus[i] & OHCI_RH_PS_PES)) continue;
		if (addr == 0) {
			if (mDevices[i].address == 0) return i;
		} else if (mDevices[i].address == addr) {
			return i;
		}
	}
	return -1;
}

void retireTD(uint32 td, uint32 t0, int cc, uint32 newCbp)
{
	gUSBTDs++;
	dmaWrite32(td, (t0 & 0x0fffffffU) | ((uint32)cc << 28));
	/*
	 * CurrentBufferPointer is zeroed when the whole buffer was consumed and
	 * otherwise left pointing at the next byte, [1].4.3.1.3.5 -- that is how
	 * the driver works out how much data actually arrived.  Leaving it at the
	 * start address makes every transfer look like it moved nothing, so HID
	 * reports read as empty and the driver eventually gives up on the pipe.
	 */
	dmaWrite32(td + 4, newCbp);
	dmaWrite32(td + 8, mDoneHead);		/* NextTD chains the done queue */
	mDoneHead = td;
}

/* Returns false when the endpoint NAKed and the ED must be left alone. */
bool processTD(int port, uint32 td, int pid, int endpoint)
{
	uint32 t0  = dmaRead32(td);
	uint32 cbp = dmaRead32(td + 4);
	uint32 be  = dmaRead32(td + 12);
	int len = bufLength(cbp, be);
	USBHIDDevice &d = mDevices[port];
	CtrlState &c = mCtrl[port];
	uint8 tmp[260];

	if (endpoint == 0) {
		switch (pid) {
		case OHCI_TD_DP_SETUP: {
			if (len > 8) len = 8;
			bufRead(cbp, be, c.setup, len);
			c.pos = 0;
			gUSBCtrl++;
			int n = usbhid_control(d, c.setup, c.buf, sizeof c.buf);
			{	/* Which request does the guest give up on? */
				static int t = 0;
				if (t < 90) {
					t++;
					fprintf(stderr, "[USB-CTRL] port%d bmRT=%02x bReq=%02x "
						"wVal=%04x wIdx=%04x wLen=%d -> %d%s\n",
						port, c.setup[0], c.setup[1],
						c.setup[2] | (c.setup[3] << 8),
						c.setup[4] | (c.setup[5] << 8),
						c.setup[6] | (c.setup[7] << 8), n,
						n < 0 ? "  STALL" : "");
				}
			}
			if (n < 0) {
				c.len = 0;
				retireTD(td, t0, OHCI_CC_STALL, 0);
				return true;
			}
			c.len = n;
			break;
		}
		case OHCI_TD_DP_IN: {
			int n = c.len - c.pos;
			if (n > len) n = len;
			if (n > 0) bufWrite(cbp, be, c.buf + c.pos, n);
			c.pos += n;
			/* a zero length IN is the status stage of an OUT transfer */
			if (len == 0) d.address = d.pendingAddress;
			retireTD(td, t0, OHCI_CC_NOERROR, n == len ? 0 : cbp + n);
			return true;
		}
		case OHCI_TD_DP_OUT:
			/* status stage of an IN transfer: SET_ADDRESS takes effect here */
			d.address = d.pendingAddress;
			break;
		}
		retireTD(td, t0, OHCI_CC_NOERROR, 0);
		return true;
	}

	/* interrupt endpoint */
	if (pid == OHCI_TD_DP_IN) {
		gUSBIntIn++;
		int n = usbhid_interrupt_in(d, tmp, len < (int)sizeof tmp ? len : (int)sizeof tmp);
		if (n < 0) return false;		/* NAK -- retry next frame */
		{	/* A report actually handed to the guest. */
			gUSBReports++;
			/* Record exactly where this report landed, so the trace can log
			 * only reads of these bytes rather than every byte that happens
			 * to equal the marker. */
			if (n > 2 && tmp[2] == 0x5a) {
				gHIDTraceArmed = 1;
				gHIDReportBuf = cbp;
				gHIDReportLen = n;
				fprintf(stderr, "[HIDARM] armed: buf=%08x len=%d bytes=%02x %02x %02x\n",
					cbp, n, tmp[0], tmp[1], tmp[2]);
			} else if (n > 2) {
				static int t = 0;
				if (t < 5) {
					t++;
					fprintf(stderr, "[HIDARM] NOT armed, report was %02x %02x %02x\n",
						tmp[0], tmp[1], tmp[2]);
				}
			}
			static int t = 0;
			/* Only log reports that actually carry movement or a button --
			 * the idle stream is all zeroes and would fill the log. */
			if (t < 20 && (n > 1 && (tmp[0] || tmp[1] || (n > 2 && tmp[2])))) {
				t++;
				fprintf(stderr, "[USB-RPT] port%d %d bytes: %02x %02x %02x (buf %d)\n",
					port, n, tmp[0], n > 1 ? tmp[1] : 0, n > 2 ? tmp[2] : 0, len);
			}
		}
		bufWrite(cbp, be, tmp, n);
		retireTD(td, t0, OHCI_CC_NOERROR, n == len ? 0 : cbp + n);
		return true;
	}
	retireTD(td, t0, OHCI_CC_NOERROR, 0);
	return true;
}

void processED(uint32 ed)
{
	uint32 w0 = dmaRead32(ed);
	if (w0 & OHCI_ED_SKIP) {
		static int t = 0;
		if (t < 6) { t++; fprintf(stderr, "[USB-ED] %08x SKIP (w0=%08x)\n", ed, w0); }
		return;
	}

	uint32 tailp = dmaRead32(ed + 4) & ~0xfU;
	uint32 headw = dmaRead32(ed + 8);
	if (headw & OHCI_ED_HALTED) {
		static int t = 0;
		if (t < 6) { t++; fprintf(stderr, "[USB-ED] %08x HALTED (headw=%08x)\n", ed, headw); }
		return;
	}

	int addr = w0 & 0x7f;
	int endpoint = (w0 >> 7) & 0xf;
	int dir = (w0 >> 11) & 3;
	gUSBEDs++;
	int port = findDevice(addr);
	if (port < 0) {
		/* Which endpoint is being polled that we cannot answer? */
		static int t = 0;
		if (t < 10) {
			t++;
			fprintf(stderr, "[USB-ED] no device: addr=%d ep=%d dir=%d "
				"(dev0=%d dev1=%d pes0=%d pes1=%d)\n",
				addr, endpoint, dir, mDevices[0].address, mDevices[1].address,
				!!(hcregs.roothub.portstatus[0] & OHCI_RH_PS_PES),
				!!(hcregs.roothub.portstatus[1] & OHCI_RH_PS_PES));
		}
		return;
	}
	{	/* And which ones we can. */
		static int t = 0;
		if (t < 10) {
			t++;
			fprintf(stderr, "[USB-ED] serving port%d addr=%d ep=%d dir=%d headp=%08x tailp=%08x\n",
				port, addr, endpoint, dir, headw & ~0xfU, tailp);
		}
	}

	uint32 headp = headw & ~0xfU;
	int guard = 0;
	while (headp && headp != tailp && guard++ < 64) {
		uint32 t0 = dmaRead32(headp);
		int pid = (dir == OHCI_ED_DIR_OUT) ? OHCI_TD_DP_OUT
			: (dir == OHCI_ED_DIR_IN) ? OHCI_TD_DP_IN
			: (int)((t0 >> 19) & 3);
		uint32 next = dmaRead32(headp + 8) & ~0xfU;
		if (!processTD(port, headp, pid, endpoint)) break;
		headp = next;
		/*
		 * Advance the endpoint's data toggle.  [1].4.2.2: the controller
		 * alternates DATA0/DATA1 across successful transfers and writes the
		 * next value back as HeadP's toggleCarry.  Leaving it fixed makes every
		 * report look like a retransmission of the one before it.
		 */
		headw ^= OHCI_ED_TOGGLE;
		dmaWrite32(ed + 8, headp | (headw & OHCI_ED_TOGGLE));
	}
}

void processList(uint32 head)
{
	int guard = 0;
	while (head && guard++ < 64) {
		{	/* What is actually queued on the list the guest points us at? */
			static int t = 0;
			if (t < 16) {
				t++;
				fprintf(stderr, "[USB-ED] head=%08x w0=%08x w1=%08x w2=%08x w3=%08x\n",
					head, dmaRead32(head), dmaRead32(head + 4),
					dmaRead32(head + 8), dmaRead32(head + 12));
			}
		}
		processED(head);
		head = dmaRead32(head + 12) & ~0xfU;
	}
}

void raiseIRQ()
{
	/*
	 * Start-of-frame must be delivered when the guest asks for it.  Apple's
	 * UIM drives its root hub interrupt endpoint from SF -- its own strings
	 * say "SimulateRootHubInt- queuing SF for non int case" -- so suppressing
	 * it leaves the hub driver deaf to port changes and nothing ever
	 * enumerates.  Raising it per USB frame would bury a guest running far
	 * slower than real time -- even at tick rate it stalls the boot -- and the
	 * UIM only queues SF when it wants a callback, not continuously.  So let
	 * SF assert on a slow cadence (see mSFDue) while every other source still
	 * asserts immediately.
	 */
	uint32 mask = hcregs.intrenable;
	if (!mSFDue) mask &= ~OHCI_INTR_SF;
	uint32 pending = hcregs.intrstatus & mask;
	mSFDue = false;
	if ((hcregs.intrenable & OHCI_INTR_MIE) && pending) {
		gUSBIRQs++;
		mIRQAsserted = true;
		pic_raise_interrupt(mConfig[0x3c]);
	} else if (mIRQAsserted) {
		/*
		 * A PCI interrupt is level triggered, and the OpenPIC keeps the source
		 * pending until the device drops the line -- asserting without ever
		 * de-asserting makes the guest re-enter its handler forever, which
		 * shows up as a spin in the Mac OS ROM around pc=0x68067e00.  Drop it
		 * as soon as no enabled status bit is left.
		 */
		mIRQAsserted = false;
		pic_cancel_interrupt(mConfig[0x3c]);
	}
}

/*
 * HccaFrameNumber must track HcFmNumber: Open Firmware declares the controller
 * broken if the register runs more than five frames ahead of the copy in
 * memory, so this is published whenever either changes.  Pad1 is zeroed
 * alongside it, as [1].4.4 requires.
 */
void writeFrameNumber()
{
	if (!hcregs.hcca) return;
	dmaWrite16(hcregs.hcca + OHCI_HCCA_FRAMENO, (uint16)hcregs.fmnumber);
	dmaWrite16(hcregs.hcca + OHCI_HCCA_FRAMENO + 2, 0);
}

/* One USB frame: advance the counter, run the lists, publish the done queue. */
void frameTick()
{
	/*
	 * Serialise against the guest.  Register access runs under gUSBRegMutex on
	 * the CPU thread while this runs on the frame thread, so without this the
	 * two race over hcregs, the done queue, the asserted-interrupt flag and
	 * the ED lists -- the same build then reaches the login screen on one run
	 * and leaves the guest spinning in its interrupt handler on the next.
	 *
	 * Skip the tick rather than wait for the lock: the guest polls these
	 * registers hundreds of thousands of times per boot, and blocking it
	 * behind a whole tick of list walking starves it badly enough that it
	 * never finishes starting up.  A dropped tick costs nothing -- the next
	 * one picks the work up.
	 */

	if ((hcregs.control & OHCI_CTRL_HCFS) != OHCI_USB_OPER) return;

	/*
	 * Go quiet when nobody is driving the controller: the guest programs an
	 * HCCA, then reclaims that memory, and a frame counter still being
	 * published into it corrupts the guest (extensions fail with "error type
	 * 10").  Only the accesses that touch guest memory may be held back,
	 * though -- HcFmNumber is free-running hardware and the Mac OS ROM's USB
	 * init polls it and nothing else, so freezing the counter deadlocks that
	 * wait permanently (reads of it deliberately do not clear mIdleTicks, so
	 * the poller could never wake the engine back up).
	 */
	/*
	 * Once a driver has enumerated a device it owns the controller and the
	 * lists must keep being serviced -- going quiet stops the interrupt
	 * endpoint from being polled, so HID reports never reach the guest and the
	 * pointer never moves.  The idle-out only guards the window before that,
	 * where the HCCA still belongs to whoever programmed it at boot.
	 */
	bool enumerated = false;
	for (uint i = 0; i < rootport_count; i++)
		if (mDevices[i].address) enumerated = true;
	if (enumerated) mIdleTicks = 0;
	bool idle = (++mIdleTicks > USB_IDLE_TICKS);

	if (!idle) gUSBFrames++;

	/*
	 * Plug the HID devices in once the guest has actually powered the ports,
	 * so the connect arrives while it is watching.  The port is reported
	 * connected but NOT enabled: a real one only enables after SetPortReset,
	 * and writePortStatus() handles that request.  Faking the enable made the
	 * guest fault once it started enumerating for real.  Register-only, so
	 * this stays on the always-run side.
	 */
	/*
	 * Re-announce a port that holds a device but has never been enabled.  The
	 * devices are plugged in as soon as the controller goes operational, which
	 * is minutes before the USB Manager and its hub driver exist, so the
	 * connect transition they are written to handle would otherwise happen
	 * with nobody listening -- afterwards the port merely looks statically
	 * occupied and nothing ever enumerates it.  Repeating the announcement
	 * until the port is enabled means the stack sees the event whenever it
	 * does come up.  Register-only, so this stays on the always-run side.
	 */
	if (++mReannounce > 500) {
		mReannounce = 0;
		bool announce = false;
		for (uint i = 0; i < rootport_count; i++) {
			uint32 &ps = hcregs.roothub.portstatus[i];
			/* NOTE: gating this on mDevices[i].address (skip already-enumerated
			 * devices) was tried and is worse -- the port reset clears the
			 * address first, so the gate never applies and the keyboard stops
			 * enumerating entirely.  Resets went 4 -> 7. */
			if ((ps & OHCI_RH_PS_CCS) && !(ps & OHCI_RH_PS_PES)) {
				ps |= OHCI_RH_PS_CSC;
				announce = true;
			}
		}
		if (announce) {
			hcregs.intrstatus |= OHCI_INTR_RHSC;
			raiseIRQ();
		}
	}

	if (mAttachDelay < 0) mAttachDelay = 60;
	if (mAttachDelay > 0 && --mAttachDelay == 0) {
		for (uint i = 0; i < rootport_count; i++) {
			hcregs.roothub.portstatus[i] |= OHCI_RH_PS_CCS
				| OHCI_RH_PS_LSDA | OHCI_RH_PS_CSC;
		}
		hcregs.intrstatus |= OHCI_INTR_RHSC;
		raiseIRQ();
	}

	/*
	 * A USB frame is 1 ms, but running the lists that often costs more than it
	 * is worth here, so the thread wakes every USB_FRAMES_PER_TICK ms and
	 * accounts for the frames that passed.  Drivers time the bus off
	 * FrameNumber -- advancing it too slowly stalls their timed waits -- while
	 * the periodic list still gets every one of its slots serviced.
	 */
	for (int f = 0; f < USB_FRAMES_PER_TICK; f++) {
		hcregs.fmnumber = (hcregs.fmnumber + 1) & 0xffff;
	}

	/*
	 * HccaFrameNumber must be published in lockstep with HcFmNumber.  The Mac
	 * OS ROM watchdog at ffdffe24 reads both and, when the register has run
	 * more than five frames ahead of the copy, treats the controller as wedged
	 * -- it saves the registers, writes HCR to HcCommandStatus and restores
	 * them, forever.  The HCCA belongs to that ROM code, so this write is part
	 * of the contract and must not be held back with the list traversal.
	 */
	writeFrameNumber();

	/*
	 * FrameRemaining reloads from FrameInterval every frame and its toggle bit
	 * flips, [1].7.3.2.  Drivers time the bus off this, and one that never
	 * moves leaves them spinning.
	 */
	hcregs.fmremaining = (hcregs.fminterval & 0x3fff)
		| ((hcregs.fmremaining ^ 0x80000000U) & 0x80000000U);

	/*
	 * Start of frame, [1].6.4.  The status bit is set so a driver polling for
	 * it makes progress, but no interrupt is raised: the guest runs far slower
	 * than real time, and one IRQ per frame drowns it in ISR entries.
	 */
	hcregs.intrstatus |= OHCI_INTR_SF;
	/* Let start-of-frame drive an interrupt roughly 15 times a second: often
	 * enough for the UIM's root hub simulation to make progress, rare enough
	 * that the guest is not buried in ISR entries. */
	if (++mSFTick >= 8) {
		mSFTick = 0;
		mSFDue = true;
	}

	/* Everything past here reads or writes guest memory. */
	if (idle) return;

	/*
	 * NOTE: this runs on the frame thread while the guest's register access
	 * runs under gUSBRegMutex on the CPU thread, so the two are not mutually
	 * excluded over hcregs, the lists or the done queue.  Serialising them
	 * properly was tried and is worse: taking the lock here starves the guest
	 * (it polls these registers hundreds of thousands of times per boot and
	 * never finishes starting up), and try-locking livelocks (the guest spins
	 * on HcFmNumber, so the frame thread can never get in to advance it).
	 * Fixing it needs finer-grained state, not a bigger lock.
	 */

	/*
	 * Walk the periodic list once per tick rather than once per emulated
	 * frame.  Servicing it eight times a pass completes up to eight transfers
	 * on the same endpoint at once, and the resulting burst of done-queue
	 * writes and interrupts drives the guest into its exception handler.  Once
	 * per tick is ~125Hz, comfortably faster than the 10ms interval the HID
	 * endpoints ask for.
	 */
	if ((hcregs.control & OHCI_CTRL_PLE) && hcregs.hcca) {
		uint32 ped = dmaRead32(hcregs.hcca + 4 * (hcregs.fmnumber & 0x1f)) & ~0xfU;
		processList(ped);
	}

	if (hcregs.control & OHCI_CTRL_CLE) {
		processList(hcregs.ed_controlhead & ~0xfU);
		hcregs.cmdstatus &= ~OHCI_CLF;
	}

	/*
	 * Publish the done queue.  Two things matter here.  The periodic list is
	 * walked at the top of this function, so anything it completed is already
	 * chained into mDoneHead -- clearing the chain at this point (as this code
	 * used to) threw away every interrupt transfer while leaving control
	 * transfers, which are processed just above, working perfectly.  Publishing
	 * is unconditional: holding a new head back until the driver acknowledges
	 * WDH is what [1].6.5 describes, but the guest can be waiting on a
	 * completion while it does so, and the two deadlock against each other.
	 */
	/*
	 * NOTE: unsynchronised against the guest, which races over hcregs, the
	 * done-queue head and the interrupt line and leaves the guest wedged
	 * partway through loading extensions on some runs.  Four ways of locking
	 * it were tried and every one is worse: the whole tick blocking starves
	 * the guest (it never finishes booting), the whole tick try-locked
	 * livelocks it (it spins on HcFmNumber so the frame thread never advances
	 * it), and even a short section around just this publish drops the CPU
	 * thread to ~1% of its dispatch rate, because raiseIRQ() reaches into the
	 * PIC and wakes the CPU while the lock is held.  This needs the shared
	 * fields made individually atomic, not a mutex.
	 */
	if (mDoneHead) {
		if (hcregs.hcca) dmaWrite32(hcregs.hcca + OHCI_HCCA_DONEHEAD, mDoneHead);
		hcregs.donehead = mDoneHead;
		hcregs.intrstatus |= OHCI_INTR_WDH;
		mDoneHead = 0;
	}
	raiseIRQ();
}

/* A port write is a set/clear request, not a plain register store, [1].7.4.4. */
void writePortStatus(uint port, uint32 data)
{
	{
		static int t = 0;
		if (t < 40) {
			t++;
			fprintf(stderr, "[USB-PORT] write port%u data=%08x (was %08x)\n",
				port, data, hcregs.roothub.portstatus[port]);
		}
	}
	uint32 &ps = hcregs.roothub.portstatus[port];
	bool changed = false;

	if (data & OHCI_RH_PS_CCS) ps &= ~OHCI_RH_PS_PES;		/* clear enable */
	if (data & OHCI_RH_PS_PES) {
		if (ps & OHCI_RH_PS_CCS) ps |= OHCI_RH_PS_PES;
	}
	if (data & (1 << 8)) ps |= OHCI_RH_PS_PPS;
	if (data & (1 << 9)) ps &= ~OHCI_RH_PS_PPS;
	if (data & OHCI_RH_PS_PRS) {
		/* Reset completes immediately: the device is emulated, so there is
		 * nothing to wait for.  A reset also re-enables the port and clears
		 * the device's address, exactly as a real one would. */
		if (ps & OHCI_RH_PS_CCS) {
			ps |= OHCI_RH_PS_PES;
			ps |= OHCI_RH_PS_PRSC;
			{
				static int t = 0;
				if (t < 30) {
					t++;
					fprintf(stderr, "[USB-RESET] >>> port%u reset, address %d -> 0\n",
						port, mDevices[port].address);
				}
			}
			mDevices[port].address = 0;
			mDevices[port].pendingAddress = 0;
			changed = true;
		}
	}
	/* write-1-to-clear on the change bits */
	ps &= ~(data & (OHCI_RH_PS_CSC | OHCI_RH_PS_PESC | OHCI_RH_PS_PSSC
			| OHCI_RH_PS_OCIC | OHCI_RH_PS_PRSC));
	if (data & (OHCI_RH_PS_CSC | OHCI_RH_PS_PESC | OHCI_RH_PS_PRSC)) changed = true;

	if (changed) {
		hcregs.intrstatus |= OHCI_INTR_RHSC;
		raiseIRQ();
	}
}

PCI_USB()
	:PCI_Device("pci-usb", 0x01, 0x06)
{
	mIORegSize[0] = 0x1000;
	mIORegType[0] = PCI_ADDRESS_SPACE_MEM;

	/* Apple KeyLargo OHCI.  Mac OS matches its USB driver on these IDs; with
	 * the original OPTi ones the controller is initialised by Open Firmware
	 * but no driver ever attaches, so the root hub is never enumerated. */
	/* Apple KeyLargo OHCI: Mac OS matches its USB driver on these IDs.  With
	 * the original OPTi ones Open Firmware initialises the controller and no
	 * guest driver ever attaches. */
	mConfig[0x00] = 0x6b;	// vendor ID: Apple
	mConfig[0x01] = 0x10;
	mConfig[0x02] = 0x19;	// unit ID: KeyLargo USB
	mConfig[0x03] = 0x00;
	
	mConfig[0x08] = 0x10;	// revision
	mConfig[0x09] = 0x10; 	// 
	mConfig[0x0a] = 0x03;	// 
	mConfig[0x0b] = 0x0c;	// 

	/*
	 * Memory space decoding and bus mastering.  This was left at 0, so the
	 * command register read back as "memory disabled, no bus master" -- a
	 * driver probing it sees a device it cannot use, and Mac OS reads exactly
	 * this register once and then leaves the controller alone.  An OHCI
	 * controller needs both: its registers are memory mapped and it DMAs the
	 * descriptor lists itself.
	 */
	mConfig[0x04] = 0x06;	// memory space enable | bus master enable
	mConfig[0x05] = 0x00;

	mConfig[0x0e] = 0x00;	// header-type

	/*
	 * PCI capability list with a power-management capability.  Apple's OHCI
	 * UIM walks it ("UIM - PCI Capabilities exists", "UIM - PCI PMC:"); with
	 * no list at all there is nothing for it to find.
	 */
	mConfig[0x06] = 0x10;	// status: capability list present
	mConfig[0x34] = 0x40;	// capabilities pointer
	mConfig[0x40] = 0x01;	// capability ID: power management
	mConfig[0x41] = 0x00;	// next: end of list
	mConfig[0x42] = 0x02;	// PMC: version 2, no PME
	mConfig[0x43] = 0x00;
	mConfig[0x44] = 0x00;	// PMCSR: D0
	mConfig[0x45] = 0x00;
	
	assignMemAddress(0, 0x80881000);
	
	/* The PCI interrupt map in promdt.cc routes this device to
	 * IO_PIC_IRQ_USB; raising anything else goes nowhere. */
	mConfig[0x3c] = IO_PIC_IRQ_USB;
	mConfig[0x3d] = 0x01;
	mConfig[0x3e] = 0x03;
	mConfig[0x3f] = 0x03;

	mIdleTicks = 0;
	mIRQAsserted = false;
	mReannounce = 0;
	mSFTick = 0;
	mSFDue = false;
	/*
	 * Two ports.  The keyboard on port 1 never finishes enumerating (addr1
	 * stays 0) and that is what drives the repeated re-scanning -- dropping to
	 * a single port does cut the port resets from ~7 to 2, but then the mouse
	 * never gets an address at all (7 control transfers and it stops), so it
	 * is a net loss.  Worth revisiting once the keyboard enumerates.
	 */
	rootport_count = USBHID_PORT_COUNT;
	usbhid_init(mDevices);
	memset(mCtrl, 0, sizeof mCtrl);
	mDoneHead = 0;
	reset();
}

void	reset()
{
	memset(&hcregs, 0, sizeof hcregs);
	hcregs.fminterval = 0x2edf;	// [1].134
	hcregs.lsthresh = 0x628;	// [1].137
	hcregs.roothub.a = 0		// [1].138
		| (1<<12) 		// No overcurrent protection supported
		| (0<<10)		// always 0
		| (1<<9)		// Ports are always powered on when the HC is powered on
		| (0<<8)		// all ports are powered at the same time.
		| (10 << 24)		// POTPGT: 20ms power on to power good
		| rootport_count;	// number of rootports

	/*
	 * Start with the ports empty and plug the devices in a moment after the
	 * controller goes operational.  A device that is already present when the
	 * driver first looks is easy to miss; a connect that happens while it is
	 * watching is the event it is written to handle.
	 */
	/*
	 * Come up with the ports powered but empty.  HcRhDescriptorA advertises
	 * NPS (ports are always powered), so a driver has no reason to issue
	 * SetGlobalPower and must never be made to wait for it.  The devices are
	 * plugged in a moment later by frameTick(), so the connect arrives as the
	 * transition the driver is written to handle rather than as state that was
	 * already there before it looked.
	 */
	/*
	 * A host controller reset must NOT disturb the root hub: [1].7.1.4 says
	 * HostControllerReset "does not affect the Root Hub and no subsequent
	 * reset signaling shall be asserted on its downstream ports".  Devices
	 * stay physically attached across it.  Wiping port status here made every
	 * HCR look like an unplug followed by a replug, so the driver tore the
	 * device down and enumerated it again -- and on the re-attach it reports
	 * "HIDCreateCursorDevice cursor already exists", leaving the live instance
	 * without a cursor device.  Reports still arrive (and buttons still reach
	 * the global MBState) but motion has nowhere to go.
	 *
	 * So keep connect/power/speed across a reset; only the port change bits
	 * and the enable are cleared, and an already-attached device is not
	 * re-announced.
	 */
	for (uint i = 0; i < rootport_count; i++) {
		uint32 &ps = hcregs.roothub.portstatus[i];
		bool attached = (ps & OHCI_RH_PS_CCS) != 0 || mAttachDelay == 0;
		ps = OHCI_RH_PS_PPS
		   | (attached ? (OHCI_RH_PS_CCS | OHCI_RH_PS_LSDA) : 0);
	}
	if (mAttachDelay != 0) mAttachDelay = -1;   /* only arm the first attach */
}

virtual void readConfig(uint reg)
{
	/* Who, if anyone, probes this device's PCI config space? */
	static int t = 0;
	if (t < 14) {
		t++;
		fprintf(stderr, "[USB-CFG] read reg %02x\n", reg);
	}
	PCI_Device::readConfig(reg);
}

void runDueFrameTicks()
{
	/* Called with the register lock held, so frameTick() and the guest's own
	 * register access can never overlap. */
	while (gUSBPendingTicks > 0) {
		gUSBPendingTicks--;
		frameTick();
	}
}

virtual bool readDeviceMem(uint r, uint32 address, uint32 &data, uint size)
{
	if (r != 0) return false;
	if (size != 4) return false;
	USBRegLock lock;
	runDueFrameTicks();	/* CPU thread: safe point to run the bus */
	gUSBRegReads++;
	if ((address >> 2) < 64) gUSBReadHist[address >> 2]++;
	/*
	 * Reading a register means a driver is really working the controller --
	 * except for the two the Open Firmware watchdog polls.  It samples
	 * FrameNumber and IntrStatus in lockstep (both around 619k reads across a
	 * boot), so counting either as activity keeps the engine running, and
	 * writing into the HCCA it programmed, long after Mac OS has reclaimed
	 * that memory.  Driving the controller always involves other registers.
	 */
	if (address != OHCI_REG_FMNUMBER && address != OHCI_REG_INTRSTATUS) mIdleTicks = 0;
	if (address == OHCI_REG_FMNUMBER) {
		/* Who is spinning on the frame counter? */
		static uint32 pcs[8];
		static int npc = 0;
		uint32 pc = ppc_cpu_get_pc(0);
		bool seen = false;
		for (int i = 0; i < npc; i++) if (pcs[i] == pc) { seen = true; break; }
		if (!seen && npc < 8) {
			pcs[npc++] = pc;
			fprintf(stderr, "[USB-FM] fmnumber read from pc=%08x\n", pc);
		}
	}
	if (address >= OHCI_REG_ROOTHUB_PORTS) {
		gUSBPortReads++;
		static int t = 0;
		if (t < 40) {
			t++;
			fprintf(stderr, "[USB-PORT] read port%u -> %08x  from pc=%08x\n",
				(address - OHCI_REG_ROOTHUB_PORTS) >> 2,
				hcregs.roothub.portstatus[(address - OHCI_REG_ROOTHUB_PORTS) >> 2],
				ppc_cpu_get_pc(0));
		}
	}
	IO_USB_TRACE("read(r=%d, a=%08x (%s), %d)\n", r, address, hc_regname(address), size);
	
	switch (address) {
	case OHCI_REG_REVISION:
		// [1].123
		data = 0x10;
		break;
	case OHCI_REG_CONTROL:
		data = hcregs.control;
		break;
	case OHCI_REG_CMDSTATUS:
		data = hcregs.cmdstatus;
		break;
	case OHCI_REG_INTRSTATUS:
		data = hcregs.intrstatus;
		break;
	case OHCI_REG_INTRENABLE:
		data = hcregs.intrenable;
		break;
	case OHCI_REG_INTRDISABLE:
		data = hcregs.intrdisable;
		break;
	case OHCI_REG_HCCA:
		data = hcregs.hcca;
		break;
	case OHCI_REG_ED_PERIODCUR:
		data = hcregs.ed_periodcurrent;
		break;
	case OHCI_REG_ED_CONTROL_HD:
		data = hcregs.ed_controlhead;
		break;
	case OHCI_REG_ED_CONTROL_CUR:
		data = hcregs.ed_controlcurrent;
		break;
	case OHCI_REG_ED_BULK_HD:
		data = hcregs.ed_bulkhead;
		break;
	case OHCI_REG_ED_BULK_CUR:
		data = hcregs.ed_bulkcurrent;
		break;
	case OHCI_REG_DONEHEAD:
		data = hcregs.donehead;
		break;
	case OHCI_REG_FMINTERVAL:
		data = hcregs.fminterval;
		break;
	case OHCI_REG_FMREMAIN:
		data = hcregs.fmremaining;
		if ((hcregs.fmremaining & 0x3fff) > 0) hcregs.fmremaining--;
		break;
	case OHCI_REG_FMNUMBER:
		data = hcregs.fmnumber;
		break;
	case OHCI_REG_PERIODICSTART:
		data = hcregs.periodicstart;
		break;
	case OHCI_REG_LSTHRESH:
		data = hcregs.lsthresh;
		break;
	case OHCI_REG_ROOTHUB_A:
		data = hcregs.roothub.a;
		break;
	case OHCI_REG_ROOTHUB_B:
		data = hcregs.roothub.b;
		break;
	case OHCI_REG_ROOTHUB_STAT:
		data = hcregs.roothub.status;
		break;
	default:
		address -= OHCI_REG_ROOTHUB_PORTS;
		address >>= 2;
		if (address < rootport_count) {
			data = hcregs.roothub.portstatus[address];
			break;
		}
		return false;
	}
	
//	gSinglestep = true;
	return true;
}

virtual bool writeDeviceMem(uint r, uint32 address, uint32 data, uint size)
{
	if (r != 0) return false;
	if (size != 4) return false;
	USBRegLock lock;
	runDueFrameTicks();	/* CPU thread: safe point to run the bus */
	gUSBRegWrites++;
	mIdleTicks = 0;
	if ((address >> 2) < 64) gUSBWriteHist[address >> 2]++;
	{
		/*
		 * Report the instruction at the PC as well: if attribution is
		 * instruction-accurate this must be a load/store, which validates the
		 * PC rather than assuming it.
		 */
		static int t = 0;
		if (t < 40) {
			t++;
			uint32 pc = ppc_cpu_get_pc(0);
			uint8 ib[4] = {0,0,0,0};
			ppc_dma_read(ib, pc, 4);
			uint32 insn = (ib[0]<<24)|(ib[1]<<16)|(ib[2]<<8)|ib[3];
			uint32 op = insn >> 26, xo = (insn >> 1) & 0x3ff;
			const char *kind =
				(op == 36 || op == 37 || op == 38 || op == 39 || op == 44 || op == 45) ? "store" :
				(op == 32 || op == 33 || op == 34 || op == 35 || op == 40 || op == 41) ? "load" :
				(op == 31 && (xo == 662 || xo == 725)) ? "stwbrx/sthbrx" :
				(op == 31 && (xo == 534 || xo == 790)) ? "lwbrx/lhbrx" :
				(op == 31 && (xo == 151 || xo == 183 || xo == 215)) ? "stwx" :
				(op == 31 && (xo == 23 || xo == 55 || xo == 87)) ? "lwzx" : "NOT-A-MEMORY-OP";
			fprintf(stderr, "[USB-W] reg %02x <- %08x pc=%08x insn=%08x (%s)\n",
				address, data, pc, insn, kind);
		}
	}		/* a driver is alive and using the controller */
	IO_USB_TRACE("write(r=%d, a=%08x (%s), data=%08x, %d)\n", r, address, hc_regname(address), data, size);

	switch (address) {
	case OHCI_REG_REVISION:
		// [1].123
		IO_USB_WARN("revision is read only.\n");
		return true;
	case OHCI_REG_CONTROL:
		hcregs.control = data;
		break;
	case OHCI_REG_CMDSTATUS:
		if (data & OHCI_HCR) {
			reset();
			return true;
		}
		hcregs.cmdstatus = data;
		break;
	case OHCI_REG_INTRSTATUS:
		hcregs.intrstatus &= ~data;	/* write 1 to clear, [1].126 */
		raiseIRQ();			/* may have dropped the last pending bit */
		break;
	case OHCI_REG_INTRENABLE:
		hcregs.intrenable |= data;
		/* Enabling an interrupt whose status bit is already set must assert it
		 * straight away, otherwise a driver that enables RHSC after the ports
		 * have come up waits for an edge that already happened. */
		raiseIRQ();
		break;
	case OHCI_REG_INTRDISABLE:
		hcregs.intrenable &= ~data;
		raiseIRQ();			/* masking may drop the line */
		break;
	case OHCI_REG_HCCA:
		hcregs.hcca = data;
		/* Publish at once: the counter has been running since power-on and a
		 * freshly programmed HCCA would otherwise look thousands of frames
		 * stale. */
		writeFrameNumber();
		break;
	case OHCI_REG_ED_PERIODCUR:
		hcregs.ed_periodcurrent = data;
		break;
	case OHCI_REG_ED_CONTROL_HD:
		hcregs.ed_controlhead = data;
		break;
	case OHCI_REG_ED_CONTROL_CUR:
		hcregs.ed_controlcurrent = data;
		break;
	case OHCI_REG_ED_BULK_HD:
		hcregs.ed_bulkhead = data;
		break;
	case OHCI_REG_ED_BULK_CUR:
		hcregs.ed_bulkcurrent = data;
		break;
	case OHCI_REG_DONEHEAD:
		hcregs.donehead = data;
		break;
	case OHCI_REG_FMINTERVAL:
		hcregs.fminterval = data;
		break;
	case OHCI_REG_FMREMAIN:
		hcregs.fmremaining = data;
		break;
	case OHCI_REG_FMNUMBER:
		hcregs.fmnumber = data;
		break;
	case OHCI_REG_PERIODICSTART:
		hcregs.periodicstart = data;
		break;
	case OHCI_REG_LSTHRESH:
		hcregs.lsthresh = data;
		break;
	case OHCI_REG_ROOTHUB_A:
		hcregs.roothub.a = data;
		break;
	case OHCI_REG_ROOTHUB_B:
		hcregs.roothub.b = data;
		break;
	case OHCI_REG_ROOTHUB_STAT:
		/*
		 * A set/clear command register, [1].7.4.3.  Storing the written value
		 * both discards the command and reports the command bits back on the
		 * next read.  SetGlobalPower is how the guest brings the ports up
		 * before it goes looking for a device.
		 */
		if (data & OHCI_RH_HS_LPSC) {
			for (uint i = 0; i < rootport_count; i++)
				hcregs.roothub.portstatus[i] |= OHCI_RH_PS_PPS;
		}
		if (data & OHCI_RH_HS_LPS) {
			for (uint i = 0; i < rootport_count; i++)
				hcregs.roothub.portstatus[i] &= ~OHCI_RH_PS_PPS;
		}
		if (data & OHCI_RH_HS_DRWE) hcregs.roothub.status |= OHCI_RH_HS_DRWE;
		if (data & OHCI_RH_HS_CRWE) hcregs.roothub.status &= ~OHCI_RH_HS_DRWE;
		if (data & OHCI_RH_HS_OCIC) hcregs.roothub.status &= ~OHCI_RH_HS_OCIC;
		break;
	default:
		address -= OHCI_REG_ROOTHUB_PORTS;
		address >>= 2;
		if (address < rootport_count) {
			writePortStatus(address, data);
			break;
		}
	}

//	gSinglestep = true;
	return true;
}

};


#include "configparser.h"

#define USB_KEY_INSTALLED	"pci_usb_installed"
/*
 * The transfer engine is off by default.  It keeps publishing the frame number
 * into the HCCA that Open Firmware programmed, and once Mac OS reclaims that
 * memory those writes corrupt it -- extensions then fail with "error type 10".
 * Until the guest's own driver owns the controller, leave the hardware passive.
 */
#define USB_KEY_HID		"pci_usb_hid"

/*
 * The host controller needs a heartbeat: interrupt endpoints are polled once
 * per frame, so without one the HID devices would never be able to report.
 * The CPU thread also touches these registers, hence the mutex.
 */
static PCI_USB *	gUSB;
static sys_thread	gUSBThread;
static sys_semaphore	gUSBSem;
sys_mutex		gUSBRegMutex;
int gUSBRegWrites, gUSBFrames, gUSBEDs, gUSBTDs, gUSBCtrl, gUSBIntIn, gUSBReports;
#define gUSBMutex gUSBRegMutex

static void *usbFrameLoop(void *)
{
	while (1) {
		sys_lock_semaphore(gUSBSem);
		/* Nominally 1 ms, but the guest is far slower than real time and an
		 * emulated frame costs real work, so run the bus at 8 ms.  HID
		 * endpoints ask to be polled every 10 ms. */
		sys_wait_semaphore_bounded(gUSBSem, 8);
		sys_unlock_semaphore(gUSBSem);
		if (!gUSB) continue;
		/*
		 * Only mark a tick as due.  frameTick() mutates hcregs, the ED lists,
		 * the done queue and the interrupt line, and running that here races
		 * the guest doing the same from the CPU thread -- which leaves it
		 * wedged partway through loading extensions on about half the runs.
		 * The CPU thread runs the tick instead, from the register path, the
		 * same way cuda_shim_apply() defers guest-memory writes to a safe
		 * point.  Cap the backlog so a stall cannot produce a burst.
		 */
		if (gUSBPendingTicks < 4) gUSBPendingTicks++;
	}
	return NULL;
}

unsigned long gMouseEventCalls = 0;	/* survives usbhid_init(), unlike per-device counters */

void usb_hid_mouse_event(int dx, int dy, bool button1, bool button2, bool button3)
{
	gMouseEventCalls++;
	if (!gUSB) return;
	sys_lock_mutex(gUSBMutex);
	usbhid_mouse_event(gUSB->mDevices[USBHID_PORT_MOUSE], dx, dy, button1, button2, button3);
	sys_unlock_mutex(gUSBMutex);
}

void usb_hid_key_event(uint8 adbKey, bool pressed)
{
	if (!gUSB) return;
	sys_lock_mutex(gUSBMutex);
	usbhid_key_event(gUSB->mDevices[USBHID_PORT_KEYBOARD], adbKey, pressed);
	sys_unlock_mutex(gUSBMutex);
}

void usb_debug_print()
{
	if (!gUSB) { fprintf(stderr, "[USB] not installed\n"); return; }
	fprintf(stderr, "[USB] most-read registers:");
	for (int pass = 0; pass < 8; pass++) {
		int best = -1, bestn = 0;
		for (int i = 0; i < 64; i++) {
			if (gUSBReadHist[i] > bestn) { bestn = gUSBReadHist[i]; best = i; }
		}
		if (best < 0) break;
		fprintf(stderr, " %s(%02x)=%d", hc_regname(best * 4), best * 4, bestn);
		gUSBReadHist[best] = 0;
	}
	fprintf(stderr, "\n");
	fprintf(stderr, "[USB] most-written registers:");
	for (int pass = 0; pass < 6; pass++) {
		int best = -1, bestn = 0;
		for (int i = 0; i < 64; i++) {
			if (gUSBWriteHist[i] > bestn) { bestn = gUSBWriteHist[i]; best = i; }
		}
		if (best < 0) break;
		fprintf(stderr, " %s(%02x)=%d", hc_regname(best * 4), best * 4, bestn);
		gUSBWriteHist[best] = 0;
	}
	fprintf(stderr, "\n");
	{
		/* Per-device traffic.  The mouse is inert while the keyboard works, so
		 * say whether the guest ever configured and polled the mouse at all --
		 * a driver that never bound it would leave these at zero. */
		USBHIDDevice &m = gUSB->mDevices[USBHID_PORT_MOUSE];
		USBHIDDevice &k = gUSB->mDevices[USBHID_PORT_KEYBOARD];
		for (unsigned ci = 0; ci < gMouseCtrlCount; ci++) {
			USBHIDCtrlRec &r = gMouseCtrlLog[ci];
			fprintf(stderr, "[MCTRL] %02x %02x wValue=%04x wIndex=%04x wLength=%d%s\n",
				r.bmRequestType, r.bRequest, r.wValue, r.wIndex, r.wLength,
				(r.wValue >> 8) == 0x22 ? "   <== REPORT DESCRIPTOR" : "");
		}
		fprintf(stderr, "[HIDDEV] hostMouseCalls=%lu (survives re-init)\n", gMouseEventCalls);
		fprintf(stderr, "[HIDDEV] mouse: addr=%d cfg=%d proto=%d idle=%d ctrl=%lu setProto=%lu setIdle=%lu polls=%lu reports=%lu events=%lu drains=%lu\n",
			m.address, m.config, m.protocol, m.idle, m.ctrlReqs, m.setProtocol, m.setIdle, m.intPolls, m.reportsOut, m.eventsIn, m.getReportDrains);
		fprintf(stderr, "[HIDDEV] keybd: addr=%d cfg=%d proto=%d idle=%d ctrl=%lu setProto=%lu setIdle=%lu polls=%lu reports=%lu events=%lu drains=%lu\n",
			k.address, k.config, k.protocol, k.idle, k.ctrlReqs, k.setProtocol, k.setIdle, k.intPolls, k.reportsOut, k.eventsIn, k.getReportDrains);
	}
	fprintf(stderr, "[USB] regWrites=%d frames=%d EDs=%d TDs=%d ctrl=%d intIn=%d "
		"reports=%d regReads=%d portReads=%d irqs=%d intrSt=%08x intrEn=%08x "
		"ctrl_reg=%08x hcca=%08x ctrlHead=%08x port0=%08x port1=%08x addr0=%d addr1=%d\n",
		gUSBRegWrites, gUSBFrames, gUSBEDs, gUSBTDs, gUSBCtrl, gUSBIntIn,
		gUSBReports, gUSBRegReads, gUSBPortReads, gUSBIRQs,
		gUSB->hcregs.intrstatus, gUSB->hcregs.intrenable,
		gUSB->hcregs.control, gUSB->hcregs.hcca, gUSB->hcregs.ed_controlhead,
		gUSB->hcregs.roothub.portstatus[0], gUSB->hcregs.roothub.portstatus[1],
		gUSB->mDevices[0].address, gUSB->mDevices[1].address);
}

void usb_init()
{
	if (gConfig->getConfigInt(USB_KEY_INSTALLED)) {
		gUSB = new PCI_USB();
		gPCI_Devices->insert(gUSB);
		sys_create_mutex(&gUSBMutex);
		if (gConfig->getConfigInt(USB_KEY_HID)) {
			sys_create_semaphore(&gUSBSem);
			sys_create_thread(&gUSBThread, 0, usbFrameLoop, NULL);
		}
	}
}

bool usb_hid_present()
{
	return gUSB != NULL && gConfig->getConfigInt(USB_KEY_INSTALLED)
		&& gConfig->getConfigInt(USB_KEY_HID);
}

void usb_done()
{
}

void usb_init_config()
{
	gConfig->acceptConfigEntryIntDef(USB_KEY_INSTALLED, 0);
	gConfig->acceptConfigEntryIntDef(USB_KEY_HID, 0);
}

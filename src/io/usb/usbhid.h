/*
 *	PearPC
 *	usbhid.h
 *
 *	USB HID (boot protocol) mouse and keyboard attached to the OHCI root hub.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 */

#ifndef __IO_USBHID_H__
#define __IO_USBHID_H__

#include "system/types.h"

/*
 * A G4 Cube has no ADB port -- its keyboard and mouse are USB -- so Mac OS
 * takes input from USB HID and ignores anything arriving over the emulated ADB
 * bus.  These two devices sit on the OHCI root hub.
 */

#define USBHID_PORT_MOUSE     0
#define USBHID_PORT_KEYBOARD  1
#define USBHID_PORT_COUNT     2

struct USBHIDDevice {
	bool	isKeyboard;
	uint8	address;		/* address assigned by SET_ADDRESS */
	uint8	pendingAddress;		/* applied at the status stage */
	uint8	config;
	uint8	protocol;		/* 0 = boot, 1 = report */
	uint8	idle;
	uint8	idleCount;		/* polls since the last idle-driven report */

	/* input accumulated since the last report was collected */
	int	dx, dy;
	uint8	buttons;
	uint8	modifiers;
	uint8	keys[6];
	bool	reportPending;
};

void usbhid_init(USBHIDDevice *devs);

/*
 * Handle a control transfer.  setup is the 8 byte SETUP packet.  For an IN
 * request the reply is written to data.  Returns the number of bytes to
 * return, or -1 to STALL.
 */
int usbhid_control(USBHIDDevice &d, const uint8 *setup, uint8 *data, int maxlen);

/*
 * Collect an interrupt-IN report.  Returns the byte count, or -1 when the
 * device has nothing to send (the host sees a NAK and retries next frame).
 */
int usbhid_interrupt_in(USBHIDDevice &d, uint8 *buf, int maxlen);

/* Host input, forwarded from the machine's event handler. */
void usbhid_mouse_event(USBHIDDevice &d, int dx, int dy, bool b1, bool b2, bool b3);
void usbhid_key_event(USBHIDDevice &d, uint8 adbKey, bool pressed);

#endif

/*
 *	PearPC
 *	usbhid.cc
 *
 *	USB HID (boot protocol) mouse and keyboard attached to the OHCI root hub.
 *
 *	References:
 *	[1] Universal Serial Bus Specification Revision 1.1, chapter 9
 *	[2] Device Class Definition for Human Interface Devices (HID) 1.11
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

#include "usbhid.h"

#include <cstring>

/* [1].9.3 bmRequestType */
#define REQ_DIR_IN		0x80
#define REQ_TYPE_MASK		0x60
#define REQ_TYPE_STANDARD	0x00
#define REQ_TYPE_CLASS		0x20

/* [1].9.4 standard requests */
#define REQ_GET_STATUS		0x00
#define REQ_CLEAR_FEATURE	0x01
#define REQ_SET_FEATURE		0x03
#define REQ_SET_ADDRESS		0x05
#define REQ_GET_DESCRIPTOR	0x06
#define REQ_SET_DESCRIPTOR	0x07
#define REQ_GET_CONFIGURATION	0x08
#define REQ_SET_CONFIGURATION	0x09
#define REQ_GET_INTERFACE	0x0a
#define REQ_SET_INTERFACE	0x0b

/* [2].7.2 HID class requests */
#define REQ_HID_GET_REPORT	0x01
#define REQ_HID_GET_IDLE	0x02
#define REQ_HID_GET_PROTOCOL	0x03
#define REQ_HID_SET_REPORT	0x09
#define REQ_HID_SET_IDLE	0x0a
#define REQ_HID_SET_PROTOCOL	0x0b

/* descriptor types */
#define DESC_DEVICE		1
#define DESC_CONFIG		2
#define DESC_STRING		3
#define DESC_HID		0x21
#define DESC_REPORT		0x22

/*
 * Boot-protocol report descriptors.  [2].B.1 and B.2 give these verbatim; a
 * host that only speaks the boot protocol never parses them, but Mac OS reads
 * them during enumeration and refuses the device if the read fails.
 */
static const uint8 gMouseReport[] = {
	0x05, 0x01,		/* Usage Page (Generic Desktop)		*/
	0x09, 0x02,		/* Usage (Mouse)			*/
	0xa1, 0x01,		/* Collection (Application)		*/
	0x09, 0x01,		/*   Usage (Pointer)			*/
	0xa1, 0x00,		/*   Collection (Physical)		*/
	0x05, 0x09,		/*     Usage Page (Button)		*/
	0x19, 0x01,		/*     Usage Minimum (1)		*/
	0x29, 0x03,		/*     Usage Maximum (3)		*/
	0x15, 0x00,		/*     Logical Minimum (0)		*/
	0x25, 0x01,		/*     Logical Maximum (1)		*/
	0x95, 0x03,		/*     Report Count (3)			*/
	0x75, 0x01,		/*     Report Size (1)			*/
	0x81, 0x02,		/*     Input (Data, Variable, Absolute)	*/
	0x95, 0x01,		/*     Report Count (1)			*/
	0x75, 0x05,		/*     Report Size (5)			*/
	0x81, 0x01,		/*     Input (Constant) -- padding	*/
	0x05, 0x01,		/*     Usage Page (Generic Desktop)	*/
	0x09, 0x30,		/*     Usage (X)			*/
	0x09, 0x31,		/*     Usage (Y)			*/
	0x15, 0x81,		/*     Logical Minimum (-127)		*/
	0x25, 0x7f,		/*     Logical Maximum (127)		*/
	/*
	 * Declare a physical range and unit as well.  Without them the device's
	 * resolution is undefined, and the Cursor Device Manager -- which is the
	 * one path that fails, while everything writing a low-memory global works
	 * -- scales motion by units-per-inch.  Physical == logical with an inch
	 * unit gives a plain 1:1 mapping.
	 */
	0x35, 0x81,		/*     Physical Minimum (-127)		*/
	0x45, 0x7f,		/*     Physical Maximum (127)		*/
	0x65, 0x13,		/*     Unit (English Linear, inch)	*/
	0x55, 0x00,		/*     Unit Exponent (0)		*/
	0x75, 0x08,		/*     Report Size (8)			*/
	0x95, 0x02,		/*     Report Count (2)			*/
	0x81, 0x06,		/*     Input (Data, Variable, Relative)	*/
	0x65, 0x00,		/*     Unit (None) -- close the unit	*/
	0xc0,			/*   End Collection			*/
	0xc0			/* End Collection			*/
};

static const uint8 gKeyboardReport[] = {
	0x05, 0x01,		/* Usage Page (Generic Desktop)		*/
	0x09, 0x06,		/* Usage (Keyboard)			*/
	0xa1, 0x01,		/* Collection (Application)		*/
	0x05, 0x07,		/*   Usage Page (Key Codes)		*/
	0x19, 0xe0,		/*   Usage Minimum (224)		*/
	0x29, 0xe7,		/*   Usage Maximum (231)		*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x01,		/*   Logical Maximum (1)		*/
	0x75, 0x01,		/*   Report Size (1)			*/
	0x95, 0x08,		/*   Report Count (8)			*/
	0x81, 0x02,		/*   Input (Data, Variable, Absolute)	*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x81, 0x01,		/*   Input (Constant) -- reserved	*/
	0x95, 0x05,		/*   Report Count (5)			*/
	0x75, 0x01,		/*   Report Size (1)			*/
	0x05, 0x08,		/*   Usage Page (LEDs)			*/
	0x19, 0x01,		/*   Usage Minimum (1)			*/
	0x29, 0x05,		/*   Usage Maximum (5)			*/
	0x91, 0x02,		/*   Output (Data, Variable, Absolute)	*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x75, 0x03,		/*   Report Size (3)			*/
	0x91, 0x01,		/*   Output (Constant) -- padding	*/
	0x95, 0x06,		/*   Report Count (6)			*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x65,		/*   Logical Maximum (101)		*/
	0x05, 0x07,		/*   Usage Page (Key Codes)		*/
	0x19, 0x00,		/*   Usage Minimum (0)			*/
	0x29, 0x65,		/*   Usage Maximum (101)		*/
	0x81, 0x00,		/*   Input (Data, Array)		*/
	0xc0			/* End Collection			*/
};

/* [1].9.6.1 -- low speed, so bMaxPacketSize0 is 8. */
static const uint8 gMouseDevDesc[] = {
	18, DESC_DEVICE, 0x00, 0x01, 0, 0, 0, 8,
	/*
	 * Identify as an Apple USB Mouse (05ac:0301) rather than the OPTi id the
	 * emulated OHCI used to carry.  Apple's USBHIDMouseModule has
	 * vendor-specific paths -- its own strings say "Do a SetIdle on non-Apple
	 * mice, as some 3rd party mice don't send reports on button up" -- so the
	 * Apple path is the one its own hardware exercises.
	 */
	0xac, 0x05,		/* idVendor  = 0x05ac Apple			*/
	0x01, 0x03,		/* idProduct = 0x0301 Apple USB Mouse		*/
	0x00, 0x01,		/* bcdDevice					*/
	1, 2, 0, 1
};

static const uint8 gKeyboardDevDesc[] = {
	18, DESC_DEVICE, 0x00, 0x01, 0, 0, 0, 8,
	0x45, 0x10,
	0x02, 0xc0,
	0x00, 0x01,
	1, 2, 0, 1
};

/* [1].9.6.3 configuration + interface + HID + endpoint */
static const uint8 gMouseCfgDesc[] = {
	9, DESC_CONFIG, 34, 0, 1, 1, 0, 0xa0, 50,
	/* Subclass 0 (no boot support) deliberately: advertising boot protocol
	 * makes Mac OS use the fixed boot report layout and never fetch the report
	 * descriptor, and while it then applies the buttons byte it ignores X and
	 * Y entirely.  Forcing report protocol makes it read the descriptor below,
	 * which describes the X and Y axes. */
	/*
	 * Boot subclass + mouse protocol, the configuration Apple's stack expects:
	 * USBHIDMouseModule -- the module that actually drives a mouse -- issues
	 * kSetProtocol and dispatches on bInterfaceProtocol.  Report protocol
	 * (subclass 0) does make Mac OS fetch the report descriptor, but the mouse
	 * module then never attaches and only the generic driver's button handling
	 * survives.
	 */
	9, 4, 0, 0, 1, 3 /* HID */, 1 /* boot */, 2 /* mouse */, 0,
	9, DESC_HID, 0x10, 0x01, 0, 1, DESC_REPORT, sizeof gMouseReport, 0,
	7, 5, 0x81 /* EP1 IN */, 3 /* interrupt */, 3, 0, 10
};

static const uint8 gKeyboardCfgDesc[] = {
	9, DESC_CONFIG, 34, 0, 1, 1, 0, 0xa0, 50,
	9, 4, 0, 0, 1, 3 /* HID */, 1 /* boot */, 1 /* keyboard */, 0,
	9, DESC_HID, 0x10, 0x01, 0, 1, DESC_REPORT, sizeof gKeyboardReport, 0,
	7, 5, 0x81, 3, 8, 0, 10
};

void usbhid_init(USBHIDDevice *devs)
{
	memset(devs, 0, sizeof(USBHIDDevice) * USBHID_PORT_COUNT);
	devs[USBHID_PORT_MOUSE].isKeyboard = false;
	devs[USBHID_PORT_KEYBOARD].isKeyboard = true;
}

static int copyOut(uint8 *dst, int maxlen, const uint8 *src, int len)
{
	if (len > maxlen) len = maxlen;
	memcpy(dst, src, len);
	return len;
}

int usbhid_control(USBHIDDevice &d, const uint8 *setup, uint8 *data, int maxlen)
{
	const uint8 bmRequestType = setup[0];
	const uint8 bRequest = setup[1];
	const uint16 wValue = setup[2] | (setup[3] << 8);
	const uint16 wLength = setup[6] | (setup[7] << 8);
	int want = wLength < (uint16)maxlen ? wLength : maxlen;

	if ((bmRequestType & REQ_TYPE_MASK) == REQ_TYPE_CLASS) {
		switch (bRequest) {
		case REQ_HID_SET_IDLE:
			d.idle = wValue >> 8;
			return 0;
		case REQ_HID_SET_PROTOCOL:
			d.protocol = wValue & 0xff;
			return 0;
		case REQ_HID_GET_PROTOCOL:
			if (want < 1) return 0;
			data[0] = d.protocol;
			return 1;
		case REQ_HID_GET_IDLE:
			if (want < 1) return 0;
			data[0] = d.idle;
			return 1;
		case REQ_HID_GET_REPORT: {
			/* An explicit poll: answer with the current state even when
			 * nothing has changed, unlike the interrupt endpoint. */
			int n = d.isKeyboard ? 8 : 3;
			if (n > want) n = want;
			memset(data, 0, n);
			if (d.isKeyboard) {
				if (n > 0) data[0] = d.modifiers;
				for (int i = 0; i < 6 && i + 2 < n; i++) data[i + 2] = d.keys[i];
			} else {
				if (n > 0) data[0] = d.buttons;
				if (n > 1) data[1] = (uint8)(d.dx < -127 ? -127 : d.dx > 127 ? 127 : d.dx);
				if (n > 2) data[2] = (uint8)(d.dy < -127 ? -127 : d.dy > 127 ? 127 : d.dy);
				d.dx = d.dy = 0;
				d.reportPending = false;
			}
			return n;
		}
		case REQ_HID_SET_REPORT:
			/* keyboard LEDs -- accepted and ignored */
			return 0;
		}
		return -1;
	}

	switch (bRequest) {
	case REQ_GET_DESCRIPTOR: {
		const uint8 type = wValue >> 8;
		switch (type) {
		case DESC_DEVICE:
			return d.isKeyboard
				? copyOut(data, want, gKeyboardDevDesc, sizeof gKeyboardDevDesc)
				: copyOut(data, want, gMouseDevDesc, sizeof gMouseDevDesc);
		case DESC_CONFIG:
			return d.isKeyboard
				? copyOut(data, want, gKeyboardCfgDesc, sizeof gKeyboardCfgDesc)
				: copyOut(data, want, gMouseCfgDesc, sizeof gMouseCfgDesc);
		case DESC_REPORT:
			return d.isKeyboard
				? copyOut(data, want, gKeyboardReport, sizeof gKeyboardReport)
				: copyOut(data, want, gMouseReport, sizeof gMouseReport);
		case DESC_HID:
			/* the HID descriptor embedded in the configuration */
			return d.isKeyboard
				? copyOut(data, want, gKeyboardCfgDesc + 18, 9)
				: copyOut(data, want, gMouseCfgDesc + 18, 9);
		case DESC_STRING: {
			/* index 0 is the language list, everything else a short label */
			static const uint8 lang[] = { 4, DESC_STRING, 0x09, 0x04 };
			if ((wValue & 0xff) == 0) return copyOut(data, want, lang, sizeof lang);
			const char *s = d.isKeyboard ? "Keyboard" : "Mouse";
			int n = (int)strlen(s);
			uint8 buf[2 + 32 * 2];
			buf[0] = (uint8)(2 + n * 2);
			buf[1] = DESC_STRING;
			for (int i = 0; i < n; i++) {
				buf[2 + i * 2] = (uint8)s[i];
				buf[3 + i * 2] = 0;
			}
			return copyOut(data, want, buf, buf[0]);
		}
		}
		return -1;
	}
	case REQ_SET_ADDRESS:
		/* [1].9.4.6 the new address takes effect after the status stage */
		d.pendingAddress = wValue & 0x7f;
		return 0;
	case REQ_SET_CONFIGURATION:
		d.config = wValue & 0xff;
		return 0;
	case REQ_GET_CONFIGURATION:
		if (want < 1) return 0;
		data[0] = d.config;
		return 1;
	case REQ_GET_STATUS:
		if (want < 2) return 0;
		data[0] = 1;	/* self powered */
		data[1] = 0;
		return 2;
	case REQ_CLEAR_FEATURE:
	case REQ_SET_FEATURE:
	case REQ_SET_INTERFACE:
		return 0;
	case REQ_GET_INTERFACE:
		if (want < 1) return 0;
		data[0] = 0;
		return 1;
	}
	return -1;
}

int usbhid_interrupt_in(USBHIDDevice &d, uint8 *buf, int maxlen)
{
	/*
	 * Honour SET_IDLE: a non-zero idle duration obliges the device to report
	 * even when nothing has changed ([HID1.11].7.2.4), and Mac OS asks for
	 * 24ms.  NAKing through that deadline makes it give up on the pipe and
	 * re-enumerate.  Answering *every* poll is equally bad -- it buries the
	 * guest -- so report on roughly the requested cadence: the periodic list
	 * is walked about every 8ms, hence every third poll.
	 *
	 * (Earlier attempts at this wedged the guest, but that was the frame
	 * thread racing the guest, since fixed by running frameTick() on the CPU
	 * thread.  It is stable now.)
	 */
	if (!d.reportPending) {
		if (!d.idle) return -1;			/* report on change only */
		if (++d.idleCount < 3) return -1;	/* not due yet */
		d.idleCount = 0;
	} else {
		d.idleCount = 0;
	}

	if (d.isKeyboard) {
		int n = maxlen < 8 ? maxlen : 8;
		memset(buf, 0, n);
		if (n > 0) buf[0] = d.modifiers;
		for (int i = 0; i < 6 && i + 2 < n; i++) buf[i + 2] = d.keys[i];
		d.reportPending = false;
		return n;
	}

	int dx = d.dx < -127 ? -127 : (d.dx > 127 ? 127 : d.dx);
	int dy = d.dy < -127 ? -127 : (d.dy > 127 ? 127 : d.dy);
	int n = maxlen < 3 ? maxlen : 3;
	if (n > 0) buf[0] = d.buttons;
	if (n > 1) buf[1] = (uint8)dx;
	if (n > 2) buf[2] = (uint8)dy;
	d.dx -= dx;
	d.dy -= dy;
	/* Only stop reporting once the accumulated movement has all been sent and
	 * no button is held, otherwise a fast drag loses its tail. */
	if (!d.dx && !d.dy) d.reportPending = false;
	return n;
}

void usbhid_mouse_event(USBHIDDevice &d, int dx, int dy, bool b1, bool b2, bool b3)
{
	d.dx += dx;
	d.dy += dy;
	if (d.dx > 1023) d.dx = 1023;
	if (d.dx < -1023) d.dx = -1023;
	if (d.dy > 1023) d.dy = 1023;
	if (d.dy < -1023) d.dy = -1023;
	d.buttons = (uint8)((b1 ? 1 : 0) | (b2 ? 2 : 0) | (b3 ? 4 : 0));
	d.reportPending = true;
}

/*
 * PearPC hands out ADB key codes; the boot keyboard protocol wants USB HID
 * usages.  Only the codes a startup screen needs are mapped -- the table can
 * grow as required.
 */
static uint8 adbToUsb(uint8 adb)
{
	static const uint8 map[128] = {
		/* 00 */ 0x04, 0x16, 0x07, 0x09, 0x0b, 0x0a, 0x1d, 0x1b,
		/* 08 */ 0x06, 0x19, 0x00, 0x05, 0x14, 0x1a, 0x08, 0x15,
		/* 10 */ 0x1c, 0x17, 0x1e, 0x1f, 0x20, 0x21, 0x23, 0x22,
		/* 18 */ 0x2e, 0x26, 0x24, 0x2d, 0x25, 0x27, 0x30, 0x12,
		/* 20 */ 0x18, 0x2f, 0x0c, 0x28, 0x0d, 0x0e, 0x33, 0x0f,
		/* 28 */ 0x34, 0x32, 0x31, 0x36, 0x38, 0x37, 0x2c, 0x2b,
		/* 30 */ 0x2b, 0x2c, 0x35, 0x2a, 0x00, 0x29, 0x00, 0x00,
		/* 38 */ 0x00, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* 40 */ 0x00, 0x63, 0x00, 0x55, 0x00, 0x57, 0x00, 0x53,
		/* 48 */ 0x00, 0x00, 0x00, 0x54, 0x58, 0x00, 0x56, 0x00,
		/* 50 */ 0x00, 0x67, 0x62, 0x59, 0x5a, 0x5b, 0x5c, 0x5d,
		/* 58 */ 0x5e, 0x5f, 0x00, 0x60, 0x61, 0x00, 0x00, 0x00,
		/* 60 */ 0x3e, 0x3f, 0x40, 0x3c, 0x41, 0x42, 0x00, 0x44,
		/* 68 */ 0x00, 0x46, 0x00, 0x47, 0x00, 0x45, 0x00, 0x43,
		/* 70 */ 0x00, 0x00, 0x49, 0x4a, 0x4b, 0x4c, 0x3d, 0x4d,
		/* 78 */ 0x3b, 0x4e, 0x3a, 0x4f, 0x51, 0x50, 0x52, 0x00
	};
	return map[adb & 0x7f];
}

void usbhid_key_event(USBHIDDevice &d, uint8 adbKey, bool pressed)
{
	/* modifiers live in their own byte, not the key array */
	uint8 mod = 0;
	switch (adbKey & 0x7f) {
	case 0x37: mod = 0x08; break;	/* left GUI (command)	*/
	case 0x38: mod = 0x02; break;	/* left shift		*/
	case 0x3a: mod = 0x04; break;	/* left alt (option)	*/
	case 0x3b: mod = 0x01; break;	/* left control		*/
	}
	if (mod) {
		if (pressed) d.modifiers |= mod;
		else d.modifiers &= ~mod;
		d.reportPending = true;
		return;
	}

	uint8 usb = adbToUsb(adbKey);
	if (!usb) return;
	if (pressed) {
		for (int i = 0; i < 6; i++) if (d.keys[i] == usb) return;
		for (int i = 0; i < 6; i++) {
			if (!d.keys[i]) { d.keys[i] = usb; break; }
		}
	} else {
		for (int i = 0; i < 6; i++) {
			if (d.keys[i] == usb) {
				for (int j = i; j < 5; j++) d.keys[j] = d.keys[j + 1];
				d.keys[5] = 0;
				break;
			}
		}
	}
	d.reportPending = true;
}

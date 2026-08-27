/*
 *	PearPC
 *	usb.h
 *
 *	Copyright (C) 2003 Sebastian Biallas (sb@biallas.net)
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

#ifndef __IO_USB_H__
#define __IO_USB_H__

#include "system/types.h"

void usb_init();
/* Host input, forwarded by the machine's event handler. */
void usb_hid_mouse_event(int dx, int dy, bool button1, bool button2, bool button3);
void usb_hid_key_event(uint8 adbKey, bool pressed);
void usb_debug_print();
/* True when the emulated OHCI root hub carries HID devices. */
bool usb_hid_present();
void usb_done();
void usb_init_config();

#endif


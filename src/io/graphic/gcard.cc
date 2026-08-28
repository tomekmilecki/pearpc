/*
 *	PearPC
 *	gcard.cc
 *
 *	Copyright (C) 2003, 2004 Sebastian Biallas (sb@biallas.net)
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

#include <cstdlib>
#include <cstring>

#include "debug/tracers.h"
#include "system/display.h"
#include "system/arch/sysendian.h"
#include "tools/snprintf.h"
#include "cpu/cpu.h"
#include "io/pic/pic.h"
#include "gcard.h"

struct VMode {
	int width, height, bytesPerPixel;
};

static VMode stdVModes[] = {
	{.width = 640, .height = 480, .bytesPerPixel = 2},
	{.width = 640, .height = 480, .bytesPerPixel = 4},
	{.width = 800, .height = 600, .bytesPerPixel = 2},
	{.width = 800, .height = 600, .bytesPerPixel = 4},
	{.width = 1024, .height = 768, .bytesPerPixel = 2},
	{.width = 1024, .height = 768, .bytesPerPixel = 4},
	{.width = 1152, .height = 864, .bytesPerPixel = 2},
	{.width = 1152, .height = 864, .bytesPerPixel = 4},
	{.width = 1280, .height = 720, .bytesPerPixel = 2},
	{.width = 1280, .height = 720, .bytesPerPixel = 4},
	{.width = 1280, .height = 768, .bytesPerPixel = 2},
	{.width = 1280, .height = 768, .bytesPerPixel = 4},
	{.width = 1280, .height = 960, .bytesPerPixel = 2},
	{.width = 1280, .height = 960, .bytesPerPixel = 4},
	{.width = 1280, .height = 1024, .bytesPerPixel = 2},
	{.width = 1280, .height = 1024, .bytesPerPixel = 4},
	{.width = 1360, .height = 768, .bytesPerPixel = 2},
	{.width = 1360, .height = 768, .bytesPerPixel = 4},
	{.width = 1600, .height = 900, .bytesPerPixel = 2},
	{.width = 1600, .height = 900, .bytesPerPixel = 4},
	{.width = 1600, .height = 1024, .bytesPerPixel = 2},
	{.width = 1600, .height = 1024, .bytesPerPixel = 4},
	{.width = 1600, .height = 1200, .bytesPerPixel = 2},
	{.width = 1600, .height = 1200, .bytesPerPixel = 4},
};

static Container *gGraphicModes;

PCI_GCard::PCI_GCard()
	:PCI_Device("pci-graphic", 0x00, 0x07)
{
	mIORegSize[0] = IO_GCARD_FRAMEBUFFER_PA_END - IO_GCARD_FRAMEBUFFER_PA_START;
	mIORegType[0] = PCI_ADDRESS_SPACE_MEM_PREFETCH;

//	mConfig[0x00] = 0x02;	// vendor ID
//	mConfig[0x01] = 0x10;
//	mConfig[0x02] = 0x45;	// unit ID
//	mConfig[0x03] = 0x52;
	mConfig[0x00] = 0x66;	// vendor ID
	mConfig[0x01] = 0x66;
	mConfig[0x02] = 0x66;	// unit ID
	mConfig[0x03] = 0x66;

	mConfig[0x08] = 0x00;	// revision
	mConfig[0x09] = 0x00;
	mConfig[0x0a] = 0x00;
	mConfig[0x0b] = 0x03;

	mConfig[0x0e] = 0x00;	// header-type
	
	assignMemAddress(0, IO_GCARD_FRAMEBUFFER_PA_START);
	
	mConfig[0x3c] = IO_PIC_IRQ_GCARD;
	mConfig[0x3d] = 1;
	mConfig[0x3e] = 0;
	mConfig[0x3f] = 0;
}

bool PCI_GCard::readDeviceMem(uint r, uint32 address, uint32 &data, uint size)
{
	IO_GRAPHIC_TRACE("read %d, %08x, %d\n", r, address, size);
	data = 0;
	return true;
}

bool PCI_GCard::writeDeviceMem(uint r, uint32 address, uint32 data, uint size)
{
	IO_GRAPHIC_TRACE("write %d, %08x, %08x, %d\n", r, address, data, size);
	return true;
}

#define MAYBE_PPC_HALF_TO_BE(a) ppc_half_from_LE(a)
#define MAYBE_PPC_WORD_TO_BE(a) ppc_word_from_LE(a)
#define MAYBE_PPC_DWORD_TO_BE(a) ppc_dword_from_LE(a)

/*#define MAYBE_PPC_HALF_TO_BE(a) (a)
#define MAYBE_PPC_WORD_TO_BE(a) (a)
#define MAYBE_PPC_DWORD_TO_BE(a) (a)*/


void FASTCALL gcard_write_1(uint32 addr, uint32 data)
{
	addr -= IO_GCARD_FRAMEBUFFER_PA_START;
	*(uint8*)(gFrameBuffer+addr) = data;
	damageFrameBuffer(addr);
}

void FASTCALL gcard_write_2(uint32 addr, uint32 data)
{
	addr -= IO_GCARD_FRAMEBUFFER_PA_START;
	*(uint16*)(gFrameBuffer+addr) = MAYBE_PPC_HALF_TO_BE(data);
	damageFrameBuffer(addr);
}

void FASTCALL gcard_write_4(uint32 addr, uint32 data)
{
	addr -= IO_GCARD_FRAMEBUFFER_PA_START;
	*(uint32*)(gFrameBuffer+addr) = MAYBE_PPC_WORD_TO_BE(data);
	damageFrameBuffer(addr);
}

void FASTCALL gcard_write_8(uint32 addr, uint64 data)
{
	addr -= IO_GCARD_FRAMEBUFFER_PA_START;
	*(uint64*)(gFrameBuffer+addr) = MAYBE_PPC_DWORD_TO_BE(data);
	damageFrameBuffer(addr);
}

void FASTCALL gcard_write_16(uint32 addr, uint128 *data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;
#if HOST_ENDIANESS == HOST_ENDIANESS_LE
	uint8 *src = (uint8 *)data;

	for (int i=0; i<16; i++) {
		gFrameBuffer[addr+15-i] = src[i];
	}
#elif HOST_ENDIANESS == HOST_ENDIANESS_BE
	memmove(gFrameBuffer+addr, data, 16);
#else
#error Unsupported endianess
#endif
	//*(uint64*)(gFrameBuffer+addr) = MAYBE_PPC_DWORD_TO_BE(data->h);
	//*(uint64*)(gFrameBuffer+addr+8) = MAYBE_PPC_DWORD_TO_BE(data->l);
	damageFrameBuffer(addr);
}

void FASTCALL gcard_write_16_native(uint32 addr, uint128 *data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;

	memmove(gFrameBuffer+addr, data, 16);

	damageFrameBuffer(addr);
}

void FASTCALL gcard_read_1(uint32 addr, uint32 &data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;
	data = (*(uint8*)(gFrameBuffer+addr));
}

void FASTCALL gcard_read_2(uint32 addr, uint32 &data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;
	data = MAYBE_PPC_HALF_TO_BE(*(uint16*)(gFrameBuffer+addr));
}

void FASTCALL gcard_read_4(uint32 addr, uint32 &data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;
	data = MAYBE_PPC_WORD_TO_BE(*(uint32*)(gFrameBuffer+addr));
}

void FASTCALL gcard_read_8(uint32 addr, uint64 &data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;
	data = MAYBE_PPC_DWORD_TO_BE(*(uint64*)(gFrameBuffer+addr));
}

void FASTCALL gcard_read_16(uint32 addr, uint128 *data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;
#if HOST_ENDIANESS == HOST_ENDIANESS_LE
	uint8 *store = (uint8 *)data;

	for (int i=0; i<16; i++) {
		store[i] = gFrameBuffer[addr+15-i];
	}
#elif HOST_ENDIANESS == HOST_ENDIANESS_BE
	memmove(data, gFrameBuffer+addr, 16);
#else
#error Unsupported endianess
#endif
	//data->h = MAYBE_PPC_DWORD_TO_BE(*(uint64*)(gFrameBuffer+addr));
	//data->l = MAYBE_PPC_DWORD_TO_BE(*(uint64*)(gFrameBuffer+addr+8));
}

void FASTCALL gcard_read_16_native(uint32 addr, uint128 *data)
{
	addr-= IO_GCARD_FRAMEBUFFER_PA_START;

	memcpy(data, gFrameBuffer+addr, 16);
}

static bool gVBLon = false;
/*
 * Mac OS clears CrsrNew and copies RawMouse into Mouse from the cursor VBL
 * task, so with no VBL interrupt the pointer can never move no matter what
 * feeds it.  VBL is only armed through OSI call 39, which the MacOnLinux video
 * NDRV alone issues; count both so a run can show whether the guest ever asks.
 */
static uint32 gVBLRaises = 0;
static uint32 gVBLCtrlCalls = 0;
static bool gVBLForce = false;
static int gCurrentGraphicMode;

void gcard_raise_interrupt()
{
	gVBLRaises++;
	if (!(gVBLon || gVBLForce)) return;
	/*
	 * Drop the line before asserting it again.  A PCI interrupt is level
	 * triggered and the OpenPIC keeps the source pending until the device
	 * de-asserts, so asserting without ever clearing leaves it latched after
	 * the first VBL: the guest sees no further edges, its 60Hz tick handler
	 * stops running, and Ticks (0x16a) freezes -- which stalls the clock, the
	 * UI redraw and event dispatch.  The same defect was fixed in the OHCI
	 * controller; the video card was missed.
	 */
	pic_cancel_interrupt(IO_PIC_IRQ_GCARD);
	pic_raise_interrupt(IO_PIC_IRQ_GCARD);
}

void gcard_osi(int cpu)
{
	uint32 func = ppc_cpu_get_gpr(cpu, 5);
	switch (func) {
	case 4:
		// cmount
		return;
	case 28: {
		// set_vmode
		uint vmode = ppc_cpu_get_gpr(cpu, 6)-1;
		if (vmode > gGraphicModes->count() || ppc_cpu_get_gpr(cpu, 7)) {
			ppc_cpu_set_gpr(cpu, 3, 1);
			return;
		}
		DisplayCharacteristics *chr = (DisplayCharacteristics *)(*gGraphicModes)[vmode];
		IO_GRAPHIC_TRACE("set mode %d\n", vmode);
		if (gDisplay->changeResolution(*chr)) {
			ppc_cpu_set_gpr(cpu, 3, 0);
			gcard_set_mode(*chr);
		} else {
			ppc_cpu_set_gpr(cpu, 3, 1);
		}
		return;
	}
	case 29: {
		// get_vmode_info
		int vmode = ppc_cpu_get_gpr(cpu, 6) - 1;
		if (vmode == -1) {
			vmode = gCurrentGraphicMode;
		}
		if (vmode > (int)gGraphicModes->count() || vmode < 0) {
			ppc_cpu_set_gpr(cpu, 3, 1);
			return;
		}
		DisplayCharacteristics *chr = ((DisplayCharacteristics *)(*gGraphicModes)[vmode]);
		ppc_cpu_set_gpr(cpu, 3, 0);
		ppc_cpu_set_gpr(cpu, 4, (gGraphicModes->count()<<16) | (vmode+1));
		ppc_cpu_set_gpr(cpu, 5, (1<<16) | 0);
		ppc_cpu_set_gpr(cpu, 6, (chr->width << 16) | chr->height);
		ppc_cpu_set_gpr(cpu, 7, chr->vsyncFrequency << 16);
		ppc_cpu_set_gpr(cpu, 8, chr->bytesPerPixel*8);
		ppc_cpu_set_gpr(cpu, 9, ((chr->scanLineLength)<<16) | 0);
		return;
	}
	case 31:
		// set_video_power
		ppc_cpu_set_gpr(cpu, 3, 0);
		return;
	case 39:
		IO_GRAPHIC_TRACE("video_ctrl: %d\n", ppc_cpu_get_gpr(cpu, 6));
		// video_ctrl
		gVBLCtrlCalls++;
		switch (ppc_cpu_get_gpr(cpu, 6)) {
		case 0:
			gVBLon = false;
			break;
		case 1:
			gVBLon = true;
			break;
		default:
			IO_GRAPHIC_ERR("39\n");
		}
		ppc_cpu_set_gpr(cpu, 3, 0);
		return;
	case 47: {
		// putchar - print guest kernel console output
		char c = (char)ppc_cpu_get_gpr(cpu, 6);
		fputc(c, stderr);
		return;
	}
	case 59: {
		// set_color
		uint32 r7 = ppc_cpu_get_gpr(cpu, 7);
		gDisplay->setColor(ppc_cpu_get_gpr(cpu, 6), MK_RGB((r7>>16)&0xff, (r7>>8)&0xff, r7&0xff));
		ppc_cpu_set_gpr(cpu, 3, 0);
		return;
	}
	case 64: {
		// get_color
		RGB c = gDisplay->getColor(ppc_cpu_get_gpr(cpu, 6));
		ppc_cpu_set_gpr(cpu, 3, (RGB_R(c) << 16) | (RGB_G(c) << 8) | (RGB_B(c)));
		return;
	}
	case 116:
		// hardware_cursor_bla
//		SINGLESTEP("hw cursor!! %d, %d, %d\n", gCPU.gpr[6], gCPU.gpr[7], gCPU.gpr[8]);
		IO_GRAPHIC_TRACE("hw cursor!! %d, %d, %d\n", ppc_cpu_get_gpr(cpu, 6), ppc_cpu_get_gpr(cpu, 7), ppc_cpu_get_gpr(cpu, 8));
		{	/* Is the guest driving a hardware cursor?  The SDL backend stores
			 * the position and never draws it, so if this is where the pointer
			 * lives it has been moving invisibly all along. */
			static int t = 0;
			static int lx = -1, ly = -1;
			int hx = (int)ppc_cpu_get_gpr(cpu, 6), hy = (int)ppc_cpu_get_gpr(cpu, 7);
			if (t < 30 && (hx != lx || hy != ly)) {
				t++; lx = hx; ly = hy;
				fprintf(stderr, "[HWCURSOR] x=%d y=%d visible=%d\n",
					hx, hy, (int)ppc_cpu_get_gpr(cpu, 8));
			}
		}
		gDisplay->setHWCursor(ppc_cpu_get_gpr(cpu, 6), ppc_cpu_get_gpr(cpu, 7), ppc_cpu_get_gpr(cpu, 8), NULL);
		return;
	}
	IO_GRAPHIC_ERR("unknown osi function\n");
}

/*
 * displayCharacteristicsFromString tries to create a(n unfinished) characteristic
 * from a String of the form [0-9]+x[0-9]+x(15|32)(@[0-9]+)?
 */
 
bool displayCharacteristicsFromString(DisplayCharacteristics &aChar, const String &s)
{
	String width, height, depth;
	String tmp, tmp2;
	if (!s.leftSplit('x', width, tmp)) return false;
	if (!width.toInt(aChar.width)) return false;
	if (!tmp.leftSplit('x', height, tmp2)) return false;
	if (!height.toInt(aChar.height)) return false;
	if (tmp2.leftSplit('@', depth, tmp)) {
		if (!depth.toInt(aChar.bytesPerPixel)) return false;	
		if (!tmp.toInt(aChar.vsyncFrequency)) return false;	
	} else {
		aChar.vsyncFrequency = -1;
		if (!tmp2.toInt(aChar.bytesPerPixel)) return false;
	}
	aChar.scanLineLength = -1;
	aChar.redShift = -1;
	aChar.redSize = -1;
	aChar.greenShift = -1;
	aChar.greenSize = -1;
	aChar.blueShift = -1;
	aChar.blueSize = -1;
	return true;
}

void gcard_add_characteristic(const DisplayCharacteristics &aChar)
{
	if (!gcard_supports_characteristic(aChar)) {
		DisplayCharacteristics *chr = new DisplayCharacteristics;
		*chr = aChar;
		gGraphicModes->insert(chr);
	}
}

bool gcard_supports_characteristic(const DisplayCharacteristics &aChar)
{
	return gGraphicModes->contains(&aChar);
}

/*
 *	gcard_finish_characteristic will fill out all fields 
 *	of aChar that aren't initialized yet (set to -1).
 */
bool gcard_finish_characteristic(DisplayCharacteristics &aChar)
{
	if (aChar.width == -1 || aChar.height == -1 || aChar.bytesPerPixel == -1) return false;
	if (aChar.vsyncFrequency == -1) aChar.vsyncFrequency = 60;
	if (aChar.scanLineLength == -1) aChar.scanLineLength = aChar.width * aChar.bytesPerPixel;
	switch (aChar.bytesPerPixel) {
	case 2:
		
		if (aChar.redShift == -1) aChar.redShift = 10;
		if (aChar.redSize == -1) aChar.redSize = 5;
		if (aChar.greenShift == -1) aChar.greenShift = 5;
		if (aChar.greenSize == -1) aChar.greenSize = 5;
		if (aChar.blueShift == -1) aChar.blueShift = 0;
		if (aChar.blueSize == -1) aChar.blueSize = 5;
		break;
	case 4:
		if (aChar.redShift == -1) aChar.redShift = 16;
		if (aChar.redSize == -1) aChar.redSize = 8;
		if (aChar.greenShift == -1) aChar.greenShift = 8;
		if (aChar.greenSize == -1) aChar.greenSize = 8;
		if (aChar.blueShift == -1) aChar.blueShift = 0;
		if (aChar.blueSize == -1) aChar.blueSize = 8;
		break;
	default:
		return false;
	}
	return true;
}

bool gcard_set_mode(DisplayCharacteristics &mode)
{
	uint tmp = gGraphicModes->getObjIdx(gGraphicModes->find(&mode));
	if (tmp == InvIdx) {
		return false;
	} else {
		gCurrentGraphicMode = tmp;
		return true;
	}
}

void gcard_init_modes()
{
	gGraphicModes = new Array(true);
	for (uint i=0; i < (sizeof stdVModes / sizeof stdVModes[0]); i++) {
		DisplayCharacteristics chr;
		chr.width = stdVModes[i].width;
		chr.height = stdVModes[i].height;
		chr.bytesPerPixel = stdVModes[i].bytesPerPixel;
		chr.scanLineLength = -1;
		chr.vsyncFrequency = -1;
		chr.redShift = -1;
		chr.redSize = -1;
		chr.greenShift = -1;
		chr.greenSize = -1;
		chr.blueShift = -1;
		chr.blueSize = -1;
		gcard_finish_characteristic(chr);
		gcard_add_characteristic(chr);
	}
}

void gcard_init_host_modes()
{
	const uint32 vramSize = IO_GCARD_FRAMEBUFFER_PA_END - IO_GCARD_FRAMEBUFFER_PA_START;
	Array modes(true);
	gDisplay->getHostCharacteristics(modes);
	foreach (DisplayCharacteristics, chr, modes, {
		// Normalize refresh rate to avoid duplicates at different Hz
		chr->vsyncFrequency = -1;
		gcard_finish_characteristic(*chr);
		// Skip modes that exceed the framebuffer size
		if ((uint32)chr->scanLineLength * chr->height > vramSize)
			continue;
		gcard_add_characteristic(*chr);
	});
}

void gcard_init()
{
	gPCI_Devices->insert(new PCI_GCard());
}

void gcard_done()
{
}

void gcard_init_config()
{
}

void gcard_debug_print()
{
	fprintf(stderr, "[VBL] raiseCalls=%u vblOn=%d osi39Calls=%u forced=%d\n",
		gVBLRaises, (int)gVBLon, gVBLCtrlCalls, (int)gVBLForce);
	{
		/* Is IRQ 23 (the video card) actually unmasked in the PIC?  The VBL
		 * cursor task never runs, and if Mac OS never enabled this line then
		 * every raise above is discarded and no VBL ever reaches the guest. */
		extern uint32 PIC_enable_low, PIC_enable_high;
		fprintf(stderr, "[VBL] PIC_enable_low=%08x high=%08x -- IRQ%d %s\n",
			PIC_enable_low, PIC_enable_high, IO_PIC_IRQ_GCARD,
			(IO_PIC_IRQ_GCARD < 32
				? (PIC_enable_low  & (1U << IO_PIC_IRQ_GCARD))
				: (PIC_enable_high & (1U << (IO_PIC_IRQ_GCARD - 32))))
				? "ENABLED" : "<== MASKED OFF, no VBL can reach the guest");
	}
	/*
	 * Is the guest's UI actually alive?  Ticks advancing only proves the timer
	 * interrupt fires -- it would keep counting with Mac OS wedged.  Checksum
	 * the framebuffer instead: if it never changes between samples, nothing is
	 * being drawn (not even the menu-bar clock), and "input does nothing" is a
	 * symptom of a hung system rather than an input bug.
	 */
	if (gFrameBuffer) {
		/* Every byte, not every 64th: a menu-bar clock is a handful of pixels
		 * and a sparse sample can miss it entirely.  Checksum the menu bar
		 * separately too -- that is where the clock lives, and it is the one
		 * thing a live Mac OS redraws without any user input. */
		uint32 sum = 0, bar = 0;
		for (uint32 off = 0; off < 640 * 480 * 2; off++)
			sum = sum * 31u + gFrameBuffer[off];
		for (uint32 y = 0; y < 20; y++)
			for (uint32 x = 0; x < 640; x++)
				bar = bar * 31u + gFrameBuffer[(y * 640 + x) * 2];
		fprintf(stderr, "[FBSUM] whole=%08x menubar=%08x\n", sum, bar);
	}
}

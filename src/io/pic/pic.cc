/*
 *	PearPC
 *	pic.cc
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

#include <cstdlib>
#include <cstring>

#include "tools/snprintf.h"
#include "system/arch/sysendian.h"
#include "cpu/cpu.h"
#include "pic.h"
#include "debug/tracers.h"
#include "system/systhread.h"

uint32 PIC_enable_low;
uint32 PIC_enable_high;
uint32 PIC_pending_low;
uint32 PIC_pending_high;
uint32 PIC_pending_level;

sys_mutex PIC_mutex;

static uint32 OpenPIC_ivpr[64];
static uint32 OpenPIC_idr[64];
static uint32 OpenPIC_ctpr;
static uint32 OpenPIC_spurious;
static bool OpenPIC_active;
static int OpenPIC_inService;

static void pic_renew_interrupts()
{
	/* Edge events raise the CPU when they arrive; only active level sources
	 * must be reasserted after an enable or acknowledge register write. */
	if (PIC_pending_level & PIC_enable_low) {
		ppc_cpu_raise_ext_exception();	
	} else {
		ppc_cpu_cancel_ext_exception();
	}
}

static bool openpic_pending(int intr)
{
	if (intr == OpenPIC_inService) return false;
	if (intr < 32) return (PIC_pending_low & (1U << intr)) != 0;
	return (PIC_pending_high & (1U << (intr - 32))) != 0;
}

/*
 * The guest enables VBL on the video card (OSI 39) but never unmasks the
 * card's line in the PIC, so every VBL raise is discarded and Mac OS's VBL
 * cursor task -- which is what copies RawMouse into Mouse, clears CrsrNew and
 * redraws the pointer -- never runs.  That leaves the pointer dead on every
 * transport while the keyboard, posted at interrupt time, still works.
 * Unmask on the guest's behalf, but only once it has asked for VBL.
 */
unsigned long gPicAckTotal = 0;
unsigned long gPicAckPerSrc[64];

/* Which sources does the guest actually take interrupts on?  If this reports
 * no acknowledged interrupts at all, Mac OS is polling its devices and no
 * OpenPIC source can ever deliver VBL, however it is unmasked. */
void pic_debug_print()
{
	fprintf(stderr, "[PICSTAT] acknowledged interrupts total=%lu ctpr=%u\n",
		gPicAckTotal, OpenPIC_ctpr);
	for (int i = 0; i < 64; i++) {
		if (!gPicAckPerSrc[i] && !(OpenPIC_ivpr[i] & 0xffff)) continue;
		fprintf(stderr, "[PICSTAT]   src%-2d ivpr=%08x %s acks=%lu\n",
			i, OpenPIC_ivpr[i],
			(OpenPIC_ivpr[i] & 0x80000000) ? "MASKED  " : "unmasked",
			gPicAckPerSrc[i]);
	}
}

uint32 pic_get_ivpr(int intr)
{
	return (intr >= 0 && intr < 64) ? OpenPIC_ivpr[intr] : 0;
}

void pic_force_enable(int intr)
{
	if (intr < 0 || intr >= 64) return;
	if (intr < 32) PIC_enable_low |= 1U << intr;
	else PIC_enable_high |= 1U << (intr - 32);
	/*
	 * Delivery is gated by the IVPR, not by PIC_enable_low: openpic_enabled()
	 * tests !(ivpr & 0x80000000) and priority > ctpr.  Setting the bookkeeping
	 * mask alone changed nothing, which is why the first attempt at this
	 * appeared to prove that unmasking does not help.  Clear the IVPR mask and
	 * raise the priority above ctpr so the source can actually be selected.
	 */
	uint32 before = OpenPIC_ivpr[intr];
	OpenPIC_ivpr[intr] &= ~0x80000000U;
	if (((OpenPIC_ivpr[intr] >> 16) & 0xf) <= OpenPIC_ctpr)
		OpenPIC_ivpr[intr] = (OpenPIC_ivpr[intr] & ~0x000f0000U) | 0x000f0000U;
	fprintf(stderr, "[PIC] force-enabled IRQ%d ivpr %08x -> %08x ctpr=%u (usb ivpr=%08x)\n",
		intr, before, OpenPIC_ivpr[intr], OpenPIC_ctpr, OpenPIC_ivpr[IO_PIC_IRQ_USB]);
}

static bool openpic_enabled(int intr)
{
	return intr >= 0 && intr < 64 && !(OpenPIC_ivpr[intr] & 0x80000000) &&
	       ((OpenPIC_ivpr[intr] >> 16) & 0xf) > OpenPIC_ctpr;
}

static void openpic_renew_interrupts()
{
	for (int intr = 0; intr < 64; ++intr) {
		if (openpic_pending(intr) && openpic_enabled(intr)) {
			ppc_cpu_raise_ext_exception();
			return;
		}
	}
	ppc_cpu_cancel_ext_exception();
}

static void openpic_reset()
{
	for (int intr = 0; intr < 64; ++intr) {
		OpenPIC_ivpr[intr] = 0xa0000000;
		OpenPIC_idr[intr] = 0;
	}
	OpenPIC_ctpr = 0xf;
	OpenPIC_spurious = 0xff;
	OpenPIC_inService = -1;
	PIC_enable_low = 0;
	PIC_enable_high = 0;
}

static void openpic_write(uint32 addr, uint32 data, int size)
{
	uint32 reg = addr - IO_OPENPIC_PA_START;
	OpenPIC_active = true;

	if (reg == 0x1020) {
		if (data & 0x80000000) openpic_reset();
	} else if (reg == 0x10e0) {
		OpenPIC_spurious = data & 0xff;
	} else if (reg >= 0x10000 && reg < 0x10800) {
		int intr = (reg & 0xffff) >> 5;
		switch (reg & 0x1f) {
		case 0x00:
			OpenPIC_ivpr[intr] = data;
			if (intr < 32) {
				if (data & 0x80000000) PIC_enable_low &= ~(1U << intr);
				else PIC_enable_low |= 1U << intr;
			} else {
				if (data & 0x80000000) PIC_enable_high &= ~(1U << (intr - 32));
				else PIC_enable_high |= 1U << (intr - 32);
			}
			break;
		case 0x10:
			OpenPIC_idr[intr] = data;
			break;
		}
	} else if ((reg & 0x3f000) == 0x20000) {
		switch (reg & 0xff0) {
		case 0x80:
			OpenPIC_ctpr = data & 0xf;
			break;
		case 0xb0:
			OpenPIC_inService = -1;
			break;
		}
	}
	openpic_renew_interrupts();
}

static void openpic_read(uint32 addr, uint32 &data, int size)
{
	uint32 reg = addr - IO_OPENPIC_PA_START;
	if (reg == 0x0000) {
		data = 0xffffffff;
	} else if (reg == 0x1000) {
		data = 0x003f0002;
	} else if (reg == 0x1020) {
		data = 0;
	} else if (reg == 0x1080 || reg == 0x1090) {
		data = 0;
	} else if (reg == 0x10e0) {
		data = OpenPIC_spurious;
	} else if (reg == 0x10f0) {
		data = 4160000;
	} else if (reg >= 0x10000 && reg < 0x10800) {
		int intr = (reg & 0xffff) >> 5;
		data = (reg & 0x10) ? OpenPIC_idr[intr] : OpenPIC_ivpr[intr];
	} else if ((reg & 0x3f000) == 0x20000) {
		switch (reg & 0xff0) {
		case 0x80:
			data = OpenPIC_ctpr;
			break;
		case 0x90:
			data = 0;
			break;
		case 0xa0: {
			int selected = -1;
			int priority = -1;
			for (int intr = 0; intr < 64; ++intr) {
				int candidatePriority = (OpenPIC_ivpr[intr] >> 16) & 0xf;
				if (openpic_pending(intr) && openpic_enabled(intr) && candidatePriority > priority) {
					selected = intr;
					priority = candidatePriority;
				}
			}
			if (selected >= 0) gPicAckTotal++;
			if (selected >= 0 && selected < 64) gPicAckPerSrc[selected]++;
			if (selected == IO_PIC_IRQ_GCARD) {
				/* Did the guest ever register a vector for the video source?
				 * ivpr==0 means no handler was installed, so delivering the
				 * interrupt hands Mac OS vector 0 and it is discarded. */
				static unsigned long n = 0;
				if ((n++ % 200) == 0)
					fprintf(stderr, "[PICACK] IRQ%d delivered #%lu vector=%02x ivpr=%08x\n",
						IO_PIC_IRQ_GCARD, n, OpenPIC_ivpr[selected] & 0xff,
						OpenPIC_ivpr[selected]);
			}
			if (selected < 0) {
				data = OpenPIC_spurious;
			} else {
				data = OpenPIC_ivpr[selected] & 0xff;
				OpenPIC_inService = selected;
				if (!(OpenPIC_ivpr[selected] & 0x00400000) || selected == IO_PIC_IRQ_CUDA) {
					if (selected < 32) {
						PIC_pending_low &= ~(1U << selected);
						PIC_pending_level &= ~(1U << selected);
					}
					else PIC_pending_high &= ~(1U << (selected - 32));
				}
			}
			openpic_renew_interrupts();
			break;
		}
		default:
			data = 0;
			break;
		}
	} else {
		data = 0xffffffff;
	}
}

void pic_write(uint32 addr, uint32 data, int size)
{
	if (addr >= IO_OPENPIC_PA_START && addr < IO_OPENPIC_PA_END) {
		openpic_write(addr, data, size);
		return;
	}
	IO_PIC_TRACE("write word @%08x: %08x (from %08x)\n", addr, data, ppc_cpu_get_pc(0));
	addr -= IO_PIC_PA_START;
	switch (addr) {
	case 0x24:
	case 0x14: {
		// enable /disable
		int o=0;
		if (addr == 0x14) {
			o = 32;
			PIC_enable_high = data;
		} else {
			PIC_enable_low = data;
			IO_PIC_TRACE("enable / disable\n");
		}
		int x = 1;
		for (int i=0; i<31; i++) {
			if (data & x) {
				IO_PIC_TRACE("enable %d\n", o+i);
//				gIRQ_Enable[o+i] = true;
			} else {
//				gIRQ_Enable[o+i] = false;
			}
			x<<=1;
		}
		break;
	}
	case 0x18:
		// ack irq
		IO_PIC_TRACE("ack high\n");
		PIC_pending_high &= ~data;
		break;
	case 0x28:
		// ack irq
		IO_PIC_TRACE("ack low\n");
		PIC_pending_low &= ~data;
		break;
	case 0x38: 
		IO_PIC_TRACE("sound\n");
		data = 0;
		break;
	default:
		/* Likewise: ignore a write to a register we do not model rather than
		 * killing the guest. */
		{
			static bool warned = false;
			if (!warned) {
				warned = true;
				IO_PIC_WARN("unimplemented register write: %08x (size %d, data %08x, from %08x); "
					"ignoring and continuing\n", addr, size, data, ppc_cpu_get_pc(0));
			}
		}
		break;
	}
	pic_renew_interrupts();
}

void pic_read(uint32 addr, uint32 &data, int size)
{
	if (addr >= IO_OPENPIC_PA_START && addr < IO_OPENPIC_PA_END) {
		openpic_read(addr, data, size);
		return;
	}
	IO_PIC_TRACE("read word @%08x (from %08x)\n", addr, ppc_cpu_get_pc(0));
	addr -= IO_PIC_PA_START;
	switch (addr) {
	case 0x24:
	case 0x14: {
		// enable /disable
		uint32 r;
		if (addr == 0x14) {
			r = PIC_enable_high;
		} else {
			r = PIC_enable_low;
		}
		IO_PIC_TRACE("enable / disable %08x\n", r);
		data = r;
		break;
	}
	case 0x10:
		IO_PIC_TRACE("interrupt high? (pending_high is %08x)\n", PIC_pending_high);
		data = PIC_pending_high;
		break;
	case 0x1c:
		IO_PIC_TRACE("level2\n");
		data = 0;
		break;
	case 0x20:
		IO_PIC_TRACE("interrupt low? (pending_low is %08x)\n", PIC_pending_low);
		data = PIC_pending_low;
		break;
	case 0x2c:
		// level
		IO_PIC_TRACE("level1 (%08x)\n", PIC_pending_level);
		data = PIC_pending_level;
		break;
	case 0x38:
		IO_PIC_TRACE("sound\n");
		data = 0;
		break;
	default:
		/*
		 * Real hardware returns something for a register the model does not
		 * implement; it does not halt the machine.  Killing the emulator here
		 * turns a harmless probe into a dead guest -- Mac OS reads 0x34 during
		 * startup and PearPC died on it.  Report once and read as zero.
		 */
		{
			static bool warned = false;
			if (!warned) {
				warned = true;
				IO_PIC_WARN("unimplemented register read: %08x (size %d, from %08x); "
					"reading 0 and continuing\n", addr, size, ppc_cpu_get_pc(0));
			}
		}
		data = 0;
	}
}

void pic_raise_interrupt(int intr)
{
	sys_lock_mutex(PIC_mutex);
	uint32 mask, pending;
	int intr_;
	if (intr > 31) {
		mask = PIC_enable_high;
		pending = PIC_pending_high;
		intr_ = intr-32;
	} else {
		mask = PIC_enable_low;
		pending = PIC_pending_low;
		intr_ = intr;
	}
	uint32 ibit = 1 << intr_;
	bool level = false;
	if (intr > 31) {
		PIC_pending_high |= ibit;
	} else {
		PIC_pending_low |= ibit;
		if (OpenPIC_active ? (OpenPIC_ivpr[intr] & 0x00400000) : (IO_PIC_LEVEL_TYPE & ibit)) {
			PIC_pending_level |= ibit;
			level = true;
		}
	}
	/*
	 *	edge type:
	 *	signal int if not masked and state raises from low to high
	 *
	 *	level type:
	 *	signal int if not masked and state high
	 */
	bool deliver = OpenPIC_active ? (openpic_pending(intr) && openpic_enabled(intr)) : ((mask & ibit) &&
	    (level || !(pending & ibit)));
	if (deliver) {
		IO_PIC_TRACE("*signal int: %d\n", intr);
		ppc_cpu_raise_ext_exception();
	} else {
		IO_PIC_TRACE("/signal int: %d\n", intr);
	}
	sys_unlock_mutex(PIC_mutex);
	ppc_cpu_wakeup();
}

void pic_cancel_interrupt(int intr)
{
	sys_lock_mutex(PIC_mutex);
	if (intr > 31) {
	        PIC_pending_high &= ~(1<<(intr-32));
	} else {
		PIC_pending_low &= ~(1<<intr);
		PIC_pending_level &= ~(1<<intr);
	}
	if (OpenPIC_active) openpic_renew_interrupts();
	else pic_renew_interrupts();
	sys_unlock_mutex(PIC_mutex);
}

void pic_init()
{
	PIC_pending_low = 0;
	PIC_pending_high = 0;
	PIC_enable_low = 0;
	PIC_enable_high = 0;
	OpenPIC_active = false;
	openpic_reset();
	sys_create_mutex(&PIC_mutex);
}

void pic_done()
{
	sys_destroy_mutex(PIC_mutex);
}

void pic_init_config()
{
}

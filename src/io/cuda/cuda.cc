/*
 *	PearPC
 *	cuda.cc
 *
 *	Copyright (C) 2003-2004 Sebastian Biallas (sb@biallas.net)
 *	Copyright (C) 2004 Stefan Weyergraf
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
 *
 *	From Linux 2.6.4:
 *	The VIA (versatile interface adapter) interfaces to the CUDA,
 *	a 6805 microprocessor core which controls the ADB (Apple Desktop
 *	Bus) which connects to the keyboard and mouse.  The CUDA also
 *	controls system power and the RTC (real time clock) chip.
 *
 *	See also:
 *	http://www.howell1964.freeserve.co.uk/parts/6522_VIA.htm
 *
 *	References:
 *	[1] http://bbc.nvg.org/doc/datasheets/R6522_r9.zip
 */

#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <ctime>

#include "cpu/cpu.h"
#include "cpu/mem.h"
#include "tools/snprintf.h"
#include "debug/tracers.h"
#include "io/pic/pic.h"
#include "io/usb/usb.h"
#include "io/graphic/gcard.h"
#include "system/keyboard.h"
#include "system/mouse.h"
#include "system/sys.h"
#include "system/sysclk.h"
#include "system/systhread.h"
#include "configparser.h"

#include <cstdio>
#include "cuda.h"

#define IO_CUDA_TRACE2(str...)

#define IO_CUDA_TRACE3(str...)

#define RS		(0x200)
#define B		0		/* B-side data */
#define A		RS		/* A-side data */
#define DIRB		(2*RS)		/* B-side direction (1=output) */
#define DIRA		(3*RS)		/* A-side direction (1=output) */
#define T1CL		(4*RS)		/* Timer 1 ctr/latch (low 8 bits) */
#define T1CH		(5*RS)		/* Timer 1 counter (high 8 bits) */
#define T1LL		(6*RS)		/* Timer 1 latch (low 8 bits) */
#define T1LH		(7*RS)		/* Timer 1 latch (high 8 bits) */
#define T2CL		(8*RS)		/* Timer 2 ctr/latch (low 8 bits) */
#define T2CH		(9*RS)		/* Timer 2 counter (high 8 bits) */
#define SR		(10*RS)		/* Shift register */
#define ACR		(11*RS)		/* Auxiliary control register */
#define PCR		(12*RS)		/* Peripheral control register */
#define IFR		(13*RS)		/* Interrupt flag register */
#define IER		(14*RS)		/* Interrupt enable register */
#define ANH		(15*RS)		/* A-side data, no handshake */

/* Bits in B data register: all active low */
#define TREQ		0x08		/* Transfer request (input) */
#define TACK		0x10		/* Transfer acknowledge (output) */
#define TIP		0x20		/* Transfer in progress (output) */

/* Bits in ACR */
#define SR_CTRL		0x1c		/* Shift register control bits */
#define SR_EXT		0x0c		/* Shift on external clock */
#define SR_OUT		0x10		/* Shift out if 1 */

/* Bits in IFR and IER */
#define IER_SET		0x80		/* set bits in IER */
#define IER_CLR		0		/* clear bits in IER */
#define SR_INT		0x04		/* Shift register full/empty */

/* Bits in ACR */
#define T1MODE          0xc0            /* Timer 1 mode */
#define T1MODE_CONT     0x40            /*  continuous interrupts */

/* Bits in IFR and IER */
#define T1_INT          0x40            /* Timer 1 interrupt */
#define T2_INT          0x20            /* Timer 2 interrupt */

/* commands (1st byte) */
#define ADB_PACKET			0
#define CUDA_PACKET			1
#define ERROR_PACKET			2
#define TIMER_PACKET			3
#define POWER_PACKET			4
#define MACIIC_PACKET			5
#define PMU_PACKET			6

#define CUDA_KEY_PMU "pci_macio_pmu"

/* PMU99 uses a two-wire handshake on VIA port B. */
#define PMU_TACK 0x08
#define PMU_TREQ 0x10

#define PMU_ADB_CMD 0x20
#define PMU_ADB_POLL_OFF 0x21
#define PMU_SET_RTC 0x30
#define PMU_READ_RTC 0x38
#define PMU_SET_INTR_MASK 0x70
#define PMU_INT_ACK 0x78
#define PMU_POWER_EVENTS 0x8f
#define PMU_GET_COVER 0xdc
#define PMU_SYSTEM_READY 0xdf
#define PMU_DOWNLOAD_STATUS 0xe2
#define PMU_READ_PMU_RAM 0xe8
#define PMU_GET_VERSION 0xea

#define PMU_INT_ADB 0x10
#define PMU_INT_TICK 0x80
#define PMU_INT_ADB_AUTO 0x04

/* CUDA commands (2nd byte) */
#define CUDA_WARM_START			0x0
#define CUDA_AUTOPOLL			0x1
#define CUDA_GET_6805_ADDR		0x2
#define CUDA_GET_TIME			0x3
#define CUDA_GET_PRAM			0x7
#define CUDA_SET_6805_ADDR		0x8
#define CUDA_SET_TIME			0x9
#define CUDA_POWERDOWN			0xa
#define CUDA_POWERUP_TIME		0xb
#define CUDA_SET_PRAM			0xc
#define CUDA_MS_RESET			0xd
#define CUDA_SEND_DFAC			0xe
#define CUDA_BATTERY_SWAP_SENSE		0x10
#define CUDA_RESET_SYSTEM		0x11
#define CUDA_SET_IPL			0x12
#define CUDA_FILE_SERVER_FLAG		0x13
#define CUDA_SET_AUTO_RATE		0x14
#define CUDA_GET_AUTO_RATE		0x16
#define CUDA_SET_DEVICE_LIST		0x19
#define CUDA_GET_DEVICE_LIST		0x1a
#define CUDA_SET_ONE_SECOND_MODE	0x1b
#define CUDA_SET_POWER_MESSAGES		0x21
#define CUDA_GET_SET_IIC		0x22
#define CUDA_WAKEUP			0x23
#define CUDA_TIMER_TICKLE		0x24
#define CUDA_COMBINED_FORMAT_IIC	0x25


/* ADB commands */
#define ADB_BUSRESET			0x00
#define ADB_FLUSH               	0x01
#define ADB_WRITEREG			0x08
#define ADB_READREG			0x0c

/* ADB device commands */
#define ADB_CMD_SELF_TEST		0xff
#define ADB_CMD_CHANGE_ID		0xfe
#define ADB_CMD_CHANGE_ID_AND_ACT	0xfd
#define ADB_CMD_CHANGE_ID_AND_ENABLE	0x00

/* ADB default device IDs (upper 4 bits of ADB command byte) */
#define ADB_DONGLE			1
#define ADB_KEYBOARD			2
#define ADB_MOUSE			3
#define ADB_TABLET			4
#define ADB_MODEM			5
#define ADB_MISC			7

#define ADB_RET_OK			0
#define ADB_RET_INUSE			1
#define ADB_RET_NOTPRESENT		2
#define ADB_RET_TIMEOUT			3
#define ADB_RET_UNEXPECTED_RESULT	4
#define ADB_RET_REQUEST_ERROR		5
#define ADB_RET_BUS_ERROR		6

#define ADB_PACKET			0
#define CUDA_PACKET			1
#define ERROR_PACKET			2
#define TIMER_PACKET			3
#define POWER_PACKET			4
#define MACIIC_PACKET			5
#define PMU_PACKET			6

// VIA timer runs at a frequency of 1/1.27655us
// or 783361.40378364 ticks/second
#define VIA_TIMER_FREQ_DIV_HZ_TIMES_1000 (783361404ULL)
/* CUDA's Timer 2 uses the 4.7 MHz / 6 clock. */
#define CUDA_T2_TIMER_FREQ_HZ_TIMES_1000 (783333333ULL)

enum cuda_state {
	cuda_idle,
	cuda_writing,
	cuda_reading,
};

enum pmu_command_state {
    pmu_idle,
    pmu_command,
    pmu_response,
};

struct cuda_control {
	byte rA;
	byte rB;
	byte rORB;
	byte rDIRB;
	byte rDIRA;
	byte rT1CL;
	byte rT1CH;
	byte rT1LL;
	byte rT1LH;
	byte rT2CL;
	byte rT2CH;
	byte rSR;
	byte rACR;
	byte rPCR;
	byte rIFR;
	byte rIER;
	byte rANH;

	// private
	uint64	T1_end;		// in cpu ticks
	bool	T1_running;
	uint64	T2_end;		// one-shot timer deadline in host ticks
	bool	T2_running;
	bool	T2_probe_pending;
	bool	T2_probe_completed;
	uint32	T2_probe_attempts;
	uint64	SR_end;		// delayed shift-register completion
	bool	SR_pending;
	bool	SR_transfer_armed;
	bool	response_irq_armed;
	bool	IRQ_asserted;
	bool	autopoll;
	bool oldTIP;
	bool oldTACK;
	cuda_state state;
	int	left;
	int	pos;
	uint8	data[100];

    bool pmuMode;
    pmu_command_state pmuState;
    uint8 pmuCommand;
    int pmuCommandLength;
    int pmuResponseLength;
    int pmuCommandPosition;
    int pmuResponsePosition;
    uint8 pmuCommandData[128];
    uint8 pmuResponseData[128];
    uint8 pmuInterruptBits;
    uint8 pmuInterruptMask;
    uint8 pmuAdbReply[128];
    int pmuAdbReplyLength;

	int	keybaddr;
	int	keybhandler;
	int	mouseaddr;
	/*
	 * Movement not yet handed to the guest.  Mac OS 9 does not use the PMU's
	 * autopoll push (it never enables autopoll and ignores unsolicited ADB
	 * data); it polls the mouse with an explicit Talk register 0 instead.  Both
	 * paths drain this, so whichever the guest uses sees the motion exactly
	 * once.
	 */
	int	pendingDx, pendingDy;
	bool	pendingBtn1, pendingBtn2;
	bool	pendingMouse;
	int	mousehandler;

	sys_semaphore idle_sem;
};

static cuda_control	gCUDA;
static sys_mutex	gCUDAMutex;

/* Command and response lengths used by PMU99. -1 means a length byte is
 * transferred before the payload. This is the table used by the Apple and
 * Linux VIA-PMU drivers. */
static const sint8 pmuDataLength[256][2] = {
    {-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {1,0},{1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,1},{0,1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{0,0},
    {-1,0},{0,0},{2,0},{1,0},{1,0},{-1,0},{-1,0},{-1,0}, {0,-1},{0,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{0,-1},
    {4,0},{20,0},{-1,0},{3,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,4},{0,20},{2,-1},{2,1},{3,-1},{-1,-1},{-1,-1},{4,0},
    {1,0},{1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,1},{0,1},{-1,-1},{1,0},{1,0},{-1,-1},{-1,-1},{-1,-1},
    {1,0},{0,0},{2,0},{2,0},{-1,0},{1,0},{3,0},{1,0}, {0,1},{1,0},{0,2},{0,2},{0,-1},{-1,-1},{-1,-1},{-1,-1},
    {2,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,3},{0,3},{0,2},{0,8},{0,-1},{0,-1},{-1,-1},{-1,-1},
    {1,0},{1,0},{1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,-1},{0,-1},{-1,-1},{-1,-1},{-1,-1},{5,1},{4,1},{4,1},
    {4,0},{-1,0},{0,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,5},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {1,0},{2,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {0,1},{0,1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {2,0},{2,0},{2,0},{4,0},{-1,0},{0,0},{-1,0},{-1,0}, {1,1},{1,0},{3,0},{2,0},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
    {0,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {1,1},{1,1},{-1,-1},{-1,-1},{0,1},{0,-1},{-1,-1},{-1,-1},
    {-1,0},{4,0},{0,1},{-1,0},{-1,0},{4,0},{-1,0},{-1,0}, {3,-1},{-1,-1},{0,1},{-1,-1},{0,-1},{-1,-1},{-1,-1},{0,0},
    {-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0},{-1,0}, {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
};

static void cuda_receive_packet();

bool cuda_is_pmu()
{
    return gCUDA.pmuMode;
}

static int gPktVariant;     /* 0..3: which autopoll packet encoding to try */
static int gAdbQueued;      /* autopoll packets we handed to the PMU */
static int gAdbDelivered;   /* of those, how many the guest actually collected */
static int gExtIntReads;    /* guest reads of the PMU interrupt GPIO */

int cuda_debug_extint_reads() { return gExtIntReads; }
void cuda_debug_count_extint_read() { gExtIntReads++; }

bool cuda_pmu_extint_asserted()
{
    return gCUDA.pmuMode && (gCUDA.pmuInterruptBits & gCUDA.pmuInterruptMask) != 0;
}

static void pmu_update_ext_interrupt()
{
    if (!gCUDA.pmuMode) return;
    if (cuda_pmu_extint_asserted()) {
        pic_raise_interrupt(IO_PIC_IRQ_PMU_EXTINT);
    } else {
        pic_cancel_interrupt(IO_PIC_IRQ_PMU_EXTINT);
    }
}

static int pmu_adb_request(uint8 command, const uint8 *payload, int payloadLength, uint8 *reply)
{
    const int device = command >> 4;
    const int operation = command & 0x0c;
    const int reg = command & 3;

    if ((command & 0x0f) == ADB_BUSRESET) {
        gCUDA.keybaddr = ADB_KEYBOARD;
        gCUDA.keybhandler = 1;
        gCUDA.mouseaddr = ADB_MOUSE;
        gCUDA.mousehandler = 2;
        return 0;
    }
    if ((command & 0x0f) == ADB_FLUSH) return 0;

    int *address = NULL;
    int *handler = NULL;
    if (device == gCUDA.keybaddr) {
        address = &gCUDA.keybaddr;
        handler = &gCUDA.keybhandler;
    } else if (device == gCUDA.mouseaddr) {
        address = &gCUDA.mouseaddr;
        handler = &gCUDA.mousehandler;
    } else {
        return -1;
    }

    if (operation == ADB_WRITEREG) {
        if (reg == 3 && payloadLength >= 2) {
            *address = payload[0] & 0x0f;
            if (payload[1] != ADB_CMD_SELF_TEST && payload[1] != ADB_CMD_CHANGE_ID &&
                payload[1] != ADB_CMD_CHANGE_ID_AND_ACT && payload[1] != ADB_CMD_CHANGE_ID_AND_ENABLE) {
                *handler = payload[1];
            }
        }
        return 0;
    }

    if (operation != ADB_READREG) return 0;
    if (reg == 3) {
        reply[0] = *address;
        reply[1] = *handler;
        return 2;
    }
    if (reg == 2 && device == gCUDA.keybaddr) {
        reply[0] = 0;
        reply[1] = 7;
        return 2;
    }
    if (reg == 0 && device == gCUDA.mouseaddr) {
        /* Talk register 0 on an ADB mouse reports movement since the last read:
         *   byte 0: bit7 = button 1 up, bits 6-0 = signed Y delta
         *   byte 1: bit7 = button 2 up, bits 6-0 = signed X delta
         * An ADB device that has nothing to say must not reply at all, so that
         * the bus can move on to the next device. */
        if (!gCUDA.pendingMouse) return 0;
        int dx = MAX(-63, MIN(63, gCUDA.pendingDx));
        int dy = MAX(-63, MIN(63, gCUDA.pendingDy));
        reply[0] = (dy & 0x7f) | (gCUDA.pendingBtn1 ? 0 : 0x80);
        reply[1] = (dx & 0x7f) | (gCUDA.pendingBtn2 ? 0 : 0x80);
        gCUDA.pendingDx = 0;
        gCUDA.pendingDy = 0;
        gCUDA.pendingMouse = false;
        return 2;
    }
    return 0;
}

static void pmu_dispatch_command()
{
    uint8 *response = gCUDA.pmuResponseData;
    const uint8 *request = gCUDA.pmuCommandData;
    int responseLength = 0;

    switch (gCUDA.pmuCommand) {
    case PMU_SET_INTR_MASK:
        if (gCUDA.pmuCommandPosition >= 1) gCUDA.pmuInterruptMask = request[0];
        pmu_update_ext_interrupt();
        break;
    case PMU_INT_ACK:
        if ((gCUDA.pmuInterruptBits & PMU_INT_ADB) && gCUDA.pmuAdbReplyLength) {
            gAdbDelivered++;
            response[0] = gCUDA.pmuInterruptBits & (PMU_INT_ADB | PMU_INT_ADB_AUTO);
            memcpy(response + 1, gCUDA.pmuAdbReply, gCUDA.pmuAdbReplyLength);
            responseLength = gCUDA.pmuAdbReplyLength + 1;
            gCUDA.pmuAdbReplyLength = 0;
            gCUDA.pmuInterruptBits &= ~(PMU_INT_ADB | PMU_INT_ADB_AUTO);
        } else {
            response[0] = gCUDA.pmuInterruptBits;
            responseLength = 1;
            gCUDA.pmuInterruptBits = 0;
        }
        pmu_update_ext_interrupt();
        break;
    case PMU_ADB_CMD: {
        if (gCUDA.pmuCommandPosition >= 4 && request[0] == 0 && request[1] == 0x86) {
            gCUDA.autopoll = request[2] != 0 || request[3] != 0;
            break;
        }
        uint8 adbResponse[32];
        const int adbPayloadLength = gCUDA.pmuCommandPosition >= 3 ? request[2] : 0;
        const int available = MAX(0, gCUDA.pmuCommandPosition - 3);
        const int adbLength = pmu_adb_request(request[0], request + 3,
                                              MIN(adbPayloadLength, available), adbResponse);
        if (adbLength > 0) {
            gCUDA.pmuAdbReply[0] = 1;
            gCUDA.pmuAdbReply[1] = adbLength;
            memcpy(gCUDA.pmuAdbReply + 2, adbResponse, adbLength);
            gCUDA.pmuAdbReplyLength = adbLength + 2;
        } else {
            gCUDA.pmuAdbReply[0] = 0;
            gCUDA.pmuAdbReplyLength = 1;
        }
        gCUDA.pmuInterruptBits |= PMU_INT_ADB;
        pmu_update_ext_interrupt();
        break;
    }
    case PMU_ADB_POLL_OFF:
        gCUDA.autopoll = false;
        break;
    case PMU_READ_RTC: {
        time_t now;
        time(&now);
        uint32 macTime = static_cast<uint32>(now) + 2082844800U;
        response[0] = macTime >> 24;
        response[1] = macTime >> 16;
        response[2] = macTime >> 8;
        response[3] = macTime;
        responseLength = 4;
        break;
    }
    case PMU_SET_RTC:
    case PMU_SYSTEM_READY:
        break;
    case PMU_POWER_EVENTS:
        if (gCUDA.pmuCommandPosition && (request[0] == 0 || request[0] == 3)) {
            response[0] = 0;
            response[1] = 0;
            responseLength = 2;
        }
        break;
    case PMU_GET_COVER:
        response[0] = 0;
        responseLength = 1;
        break;
    case PMU_DOWNLOAD_STATUS:
        response[0] = 0x62;
        responseLength = 1;
        break;
    case PMU_GET_VERSION:
        response[0] = 1;
        responseLength = 1;
        break;
    case PMU_READ_PMU_RAM:
        responseLength = 0;
        break;
    default:
        if (gCUDA.pmuResponseLength > 0) {
            responseLength = MIN(gCUDA.pmuResponseLength, (int)sizeof gCUDA.pmuResponseData);
            memset(response, 0, responseLength);
        }
        break;
    }

    const bool variableResponse = pmuDataLength[gCUDA.pmuCommand][1] < 0;
    gCUDA.pmuResponseLength = responseLength;
    gCUDA.pmuResponsePosition = variableResponse ? -1 : 0;
    gCUDA.pmuState = (responseLength || variableResponse) ? pmu_response : pmu_idle;
}

static void pmu_transfer_byte()
{
    if (gCUDA.pmuState == pmu_idle) {
        if (!(gCUDA.rACR & SR_OUT)) return;
        gCUDA.pmuCommand = gCUDA.rSR;
        gCUDA.pmuCommandLength = pmuDataLength[gCUDA.pmuCommand][0];
        gCUDA.pmuResponseLength = pmuDataLength[gCUDA.pmuCommand][1];
        gCUDA.pmuCommandPosition = 0;
        gCUDA.pmuResponsePosition = 0;
        gCUDA.pmuState = pmu_command;
        if (gCUDA.pmuCommandLength == 0) pmu_dispatch_command();
        return;
    }

    if (gCUDA.pmuState == pmu_command) {
        if (!(gCUDA.rACR & SR_OUT)) return;
        if (gCUDA.pmuCommandLength < 0) {
            gCUDA.pmuCommandLength = gCUDA.rSR;
        } else if (gCUDA.pmuCommandPosition < (int)sizeof gCUDA.pmuCommandData) {
            gCUDA.pmuCommandData[gCUDA.pmuCommandPosition++] = gCUDA.rSR;
        }
        if (gCUDA.pmuCommandLength == gCUDA.pmuCommandPosition) pmu_dispatch_command();
        return;
    }

    if (gCUDA.rACR & SR_OUT) return;
    if (pmuDataLength[gCUDA.pmuCommand][1] < 0 && gCUDA.pmuResponsePosition < 0) {
        gCUDA.rSR = gCUDA.pmuResponseLength;
        gCUDA.pmuResponsePosition = 0;
    } else if (gCUDA.pmuResponsePosition < gCUDA.pmuResponseLength) {
        gCUDA.rSR = gCUDA.pmuResponseData[gCUDA.pmuResponsePosition++];
    }
    if (gCUDA.pmuResponsePosition >= gCUDA.pmuResponseLength) gCUDA.pmuState = pmu_idle;
}

/* Ticks (0x16a) is frozen while the CPU runs and DEC is delivered.  Count the
 * VIA tick sources so a run can say which of them never fires. */
static unsigned long gT1Raises = 0, gCudaIrqAsserts = 0;

/* Timed key-injection script; -1 idle, otherwise elapsed ms. */
/* Click edges: produced on the input thread, consumed on the CPU thread. */
#define CLICK_RING 16
static volatile int gClickRing[CLICK_RING];
static volatile int gClickHead = 0, gClickTail = 0;
static int gSyntheticKeyDown = 0;
static int gHeldShot = 0;
static int gLastPostedButton = 0;
static uint64 gSyntheticKeyAt = 0;
static uint8 gSyntheticKey = 0;
uint32 gLastPostedEl = 0;

static bool post_os_event(uint16 what, sint16 v, sint16 h, bool buttonDown);
static int gKeyScript = -1;

/* PEARPC_BOOT_SHIFT=1 holds the Shift key across the extension-loading phase
 * of startup, which is how Mac OS is told to boot with all extensions off.
 * USB support lives in ROM/System on a New World Mac, so the mouse still
 * enumerates -- if the pointer works only in this mode, a third-party
 * extension (Kensington MouseWorks and friends are installed) is stealing it. */
static int gBootShiftState = 0;

static void cuda_renew_interrupt()
{
	if (gCUDA.rIFR & gCUDA.rIER & (SR_INT | T1_INT | T2_INT)) {
		gCUDA.rIFR |= 0x80;
		if (!gCUDA.IRQ_asserted) {
			gCUDA.IRQ_asserted = true;
			gCudaIrqAsserts++;
			pic_raise_interrupt(IO_PIC_IRQ_CUDA);
		}
	} else {
		gCUDA.rIFR &= ~0x80;
		if (gCUDA.IRQ_asserted) {
			gCUDA.IRQ_asserted = false;
			pic_cancel_interrupt(IO_PIC_IRQ_CUDA);
		}
	}
}

static void cuda_schedule_sr_interrupt()
{
	uint64 ticks_per_second = sys_get_hiresclk_ticks_per_second();
	uint64 delay = (ticks_per_second * 300) / 1000000;
	if (!delay) delay = 1;
	gCUDA.SR_end = sys_get_hiresclk_ticks() + delay;
	gCUDA.SR_pending = true;
}

static void cuda_update_sr_interrupt()
{
	if (gCUDA.SR_pending && sys_get_hiresclk_ticks() >= gCUDA.SR_end) {
		gCUDA.SR_pending = false;
		gCUDA.rIFR |= SR_INT;
		cuda_renew_interrupt();
	}
}

static void cuda_send_packet(uint8 type, int nb, ...)
{
	const bool replyingToRequest = gCUDA.state == cuda_reading;
	gCUDA.data[0] = type;
	va_list va;
	va_start(va, nb);
	for (int i=0; i<nb; i++) {
		uint8 b = va_arg(va, int);
		gCUDA.data[i+1] = b;
	}
	IO_CUDA_TRACE3("send: ");
	for (int i=0; i<nb+1; i++) {
		IO_CUDA_TRACE3("%02x ", gCUDA.data[i]);
	}
	IO_CUDA_TRACE3("\n");
	va_end(va);
	gCUDA.pos = 0;
	gCUDA.left = nb+1;
	gCUDA.response_irq_armed = replyingToRequest;
	gCUDA.rIFR |= SR_INT;
	gCUDA.rB &= ~TREQ;
	gCUDA.rB |= TIP;
	IO_CUDA_TRACE2("[CUDA-SEND] state=%d left=%d\n", gCUDA.state, gCUDA.left);
	cuda_renew_interrupt();
}

static void cuda_receive_adb_packet()
{
	IO_CUDA_TRACE3("===========================================\n");
	IO_CUDA_TRACE3("ADB_PACKET ");// %02x %02x %02x %02x %02x\n", gCUDA.data[1], gCUDA.data[2], gCUDA.data[3], gCUDA.data[4], gCUDA.data[5]);
	for (int i=1; i<gCUDA.pos; i++) {
		IO_CUDA_TRACE3("%02x ", gCUDA.data[i]);
	}
	IO_CUDA_TRACE3("\n");
//	gSinglestep = true;
	IO_CUDA_TRACE2("ADB_PACKET ");
	if (gCUDA.data[1] == ADB_BUSRESET) {
		IO_CUDA_TRACE2("ADB_BUSRESET %02x\n", gCUDA.data[2]);
		cuda_send_packet(ADB_PACKET, 2, 0, 0);
		return;
	}
	int devaddr = gCUDA.data[1] >> 4;
	int cmd = gCUDA.data[1] & 0xf;
	if (cmd == ADB_FLUSH) {
		// FIXME: ok?
		cuda_send_packet(ADB_PACKET, 2, 0, 0);
		return;
	}
	int reg = cmd & 3;
	cmd &= 0xc;
	IO_CUDA_TRACE3("devaddr %x reg %x cmd %s\n", devaddr, reg, (cmd==ADB_WRITEREG)?"write":"read");
	switch (cmd) {
	case ADB_WRITEREG:
		switch (reg) {
		case 2:
			if (devaddr == gCUDA.keybaddr) {
				// LED stat
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
			} else if (devaddr == gCUDA.mouseaddr) {
//				gSinglestep = true;
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
			} else {
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_NOTPRESENT);
			}
			break;
		case 3:
			if (devaddr == gCUDA.keybaddr) {
				switch (gCUDA.data[3]) {
				case ADB_CMD_SELF_TEST:
					cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
					break;
				case ADB_CMD_CHANGE_ID:
				case ADB_CMD_CHANGE_ID_AND_ACT:
				case ADB_CMD_CHANGE_ID_AND_ENABLE:
					gCUDA.keybaddr = gCUDA.data[2] & 0xf;
					cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
					break;
				default:
					gCUDA.keybaddr = gCUDA.data[2] & 0xf;
					gCUDA.keybhandler = gCUDA.data[3];
					cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
					break;
				}
			} else if (devaddr == gCUDA.mouseaddr) {
				switch (gCUDA.data[3]) {
				case ADB_CMD_SELF_TEST:
					cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
					break;
				case ADB_CMD_CHANGE_ID:
				case ADB_CMD_CHANGE_ID_AND_ACT:
				case ADB_CMD_CHANGE_ID_AND_ENABLE:
					gCUDA.mouseaddr = gCUDA.data[2] & 0xf;
					cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
					break;
				default:
					gCUDA.mouseaddr = gCUDA.data[2] & 0xf;
					gCUDA.mousehandler = gCUDA.data[3];
					cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
					break;
				}
			} else {
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_NOTPRESENT);
			}
			break;
		default:
			IO_CUDA_ERR("unknown reg %02x for device %02x\n", reg, devaddr);
		}
		break;
	case ADB_READREG: {
		switch (reg) {
		case 1:
			if (devaddr == gCUDA.keybaddr) {
				IO_CUDA_WARN("keyb reg1\n");
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
			} else if (devaddr == gCUDA.mouseaddr) {
//				gSinglestep = true;
				IO_CUDA_WARN("read reg 1 of mouse unsupported.\n");
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_OK);
			} else {
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_NOTPRESENT);
			}
			break;		
		case 2:
			if (devaddr == gCUDA.keybaddr) {
				// LED stat
				// 111b == all off
				int ledstat = gKeyboard->getKeybLEDs();
				int keyb = 0xff;
				if (!(ledstat & KEYB_LED_NUM)) keyb &= ~0x80;
				if (!(ledstat & KEYB_LED_SCROLL)) keyb &= ~0x40;
				if (!(ledstat & KEYB_LED_CAPS)) keyb &= ~0x20;
				cuda_send_packet(ADB_PACKET, 3, ADB_RET_OK, 0xff, keyb);
			} else if (devaddr == gCUDA.mouseaddr) {
//				gSinglestep = true;
				IO_CUDA_WARN("read reg 2 of mouse unsupported.\n");
			} else {
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_NOTPRESENT);
			}
			break;
		case 3:
			if (devaddr == gCUDA.keybaddr) {
//				cuda_send_packet(ADB_PACKET, 3, ADB_RET_OK, gCUDA.keybaddr, gCUDA.keybhandler);
				cuda_send_packet(ADB_PACKET, 3, ADB_RET_OK, gCUDA.keybhandler, gCUDA.keybaddr);
			} else if (devaddr == gCUDA.mouseaddr) {
//				cuda_send_packet(ADB_PACKET, 3, ADB_RET_OK, gCUDA.mouseaddr, gCUDA.mousehandler);
				cuda_send_packet(ADB_PACKET, 3, ADB_RET_OK, gCUDA.mousehandler, gCUDA.mouseaddr);
			} else {
				cuda_send_packet(ADB_PACKET, 1, ADB_RET_NOTPRESENT);
			}
			break;
		default:
			IO_CUDA_ERR("unknown reg %02x for device %02x\n", reg, devaddr);
		}
		break;
	}
	default:
		IO_CUDA_ERR("unknown adb command\n");
	}
/*	
	default:
		switch (gCUDA.data[1] & 0xf0) {
		case (ADB_KEYBOARD << 4):
			switch (gCUDA.data[1] & 0xf) {
				case 0xf: 
					IO_CUDA_TRACE2("KEYBOARD: GET DEVICE INFO %02x\n", gCUDA.data[1]);
					cuda_send_packet(ADB_PACKET, 4, 0, 0, 0, 1);
					return;
			}			
			IO_CUDA_TRACE2("KEYBOARD: unknown %02x\n", gCUDA.data[1]);
			cuda_send_packet(ADB_PACKET, 1, 0x2);
			return;
		}
		IO_CUDA_TRACE2("unknown adb (%02x)!\n", gCUDA.data[1]);
//		IO_CUDA_ERR("!\n");
		cuda_send_packet(ADB_PACKET, 1, 0x2);
	}
*/	
}

static void cuda_receive_cuda_packet()
{
	IO_CUDA_TRACE2("CUDA_PACKET ");
	switch (gCUDA.data[1]) {
	case CUDA_AUTOPOLL: {
		IO_CUDA_TRACE2("CUDA_AUTOPOLL=%02x\n", gCUDA.data[2]);
		if (gCUDA.data[2]) {
			gCUDA.autopoll = true;
		} else {
			gCUDA.autopoll = false;
		}
		cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_AUTOPOLL);
		break;
	}
	case CUDA_GET_TIME: {
		IO_CUDA_TRACE2("CUDA_GET_TIME %02x\n", gCUDA.data[2]);
		time_t tt;
		time(&tt);
		uint32 t = (uint32)tt+ 2082844800;
		cuda_send_packet(CUDA_PACKET, 6, 0, CUDA_GET_TIME, t>>24, t>>16, t>>8, t);
		break;
	}
	case CUDA_SET_TIME: {
		IO_CUDA_TRACE2("CUDA_SET_TIME %02x\n", gCUDA.data[2]);
		cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_SET_TIME);
		break;
	}
	case CUDA_RESET_SYSTEM: {
		IO_CUDA_WARN("reset!\n");
		ppc_cpu_stop();
		break;
	}
	case CUDA_FILE_SERVER_FLAG: {
		IO_CUDA_TRACE2("FILE_SERVER_FLAG %02x\n", gCUDA.data[2]);
		cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_FILE_SERVER_FLAG);
		break;
	}
	case CUDA_SET_DEVICE_LIST: {
		IO_CUDA_TRACE2("SET_DEVICE_LIST %02x %02x %02x\n", gCUDA.data[2], gCUDA.data[3], gCUDA.data[4]);
		cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_SET_DEVICE_LIST);
		break;		
	}
	case CUDA_SET_AUTO_RATE: {
		IO_CUDA_TRACE2("SET_AUTO_RATE %02x\n", gCUDA.data[2]);
		cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_SET_AUTO_RATE);
		break;		
	}
	case CUDA_SET_POWER_MESSAGES: {
		IO_CUDA_TRACE2("CUDA_SET_POWER_MESSAGES %02x\n", gCUDA.data[2]);
		cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_SET_POWER_MESSAGES);
		break;
	}
	case CUDA_GET_SET_IIC: {
		IO_CUDA_TRACE2("CUDA_GET_SET_IIC\n");
		if (gCUDA.pos == 5) {
			cuda_send_packet(CUDA_PACKET, 2, 0, CUDA_GET_SET_IIC);
		} else {
			cuda_send_packet(ERROR_PACKET, 3, 2, CUDA_PACKET, CUDA_GET_SET_IIC);
		}
		break;
	}
	case CUDA_COMBINED_FORMAT_IIC: {
		IO_CUDA_TRACE2("CUDA_COMBINED_FORMAT_IIC\n");
		cuda_send_packet(ERROR_PACKET, 3, 5, CUDA_PACKET, CUDA_COMBINED_FORMAT_IIC);
		break;
	}
	case CUDA_POWERDOWN: {
		IO_CUDA_WARN("power down!\n");
		ppc_cpu_stop();
		break;
	}
	default:
		IO_CUDA_WARN("unsupported cuda command (%02x)\n", gCUDA.data[1]);
		/* Report unsupported firmware commands instead of silently dropping
		 * them or claiming success.  Mac OS uses this response to avoid
		 * hardware-specific CUDA extensions such as the 6805/I2C interface. */
		cuda_send_packet(ERROR_PACKET, 3, 2, CUDA_PACKET, gCUDA.data[1]);
	}
}

static void cuda_receive_packet()
{
	IO_CUDA_TRACE2("cuda received packet: (%d) ", gCUDA.pos);
	switch (gCUDA.data[0]) {
	case ADB_PACKET:
		cuda_receive_adb_packet();
		break;
	case CUDA_PACKET:
		cuda_receive_cuda_packet();
		break;
	case ERROR_PACKET:
		IO_CUDA_TRACE2("ERROR_PACKET %02x %02x\n", gCUDA.data[1], gCUDA.data[2]);
		IO_CUDA_ERR("error packet\n");
		break;
	case TIMER_PACKET:
		IO_CUDA_TRACE2("TIMER_PACKET %02x %02x\n", gCUDA.data[1], gCUDA.data[2]);
		IO_CUDA_ERR("timer packet\n");
		break;
	case POWER_PACKET:
		IO_CUDA_TRACE2("POWER_PACKET %02x %02x\n", gCUDA.data[1], gCUDA.data[2]);
		IO_CUDA_ERR("power packet\n");
		break;
	case MACIIC_PACKET:
		IO_CUDA_TRACE2("MACIIC_PACKET %02x %02x\n", gCUDA.data[1], gCUDA.data[2]);
		IO_CUDA_ERR("maciic packet\n");
		break;
	case PMU_PACKET:
		IO_CUDA_TRACE2("PMU_PACKET %02x %02x\n", gCUDA.data[1], gCUDA.data[2]);
		IO_CUDA_ERR("pmu packet\n");
		break;
	default:
		IO_CUDA_TRACE2("unknown generic (%02x)!\n", gCUDA.data[0]);
		IO_CUDA_ERR("unknown packet\n", gCUDA.data[0]);
		break;
	}
}

static void cuda_update_T1()
{
	if (!gCUDA.T1_running) return;

	uint64 clk = sys_get_hiresclk_ticks();
	if (clk < gCUDA.T1_end) {
		uint64 ticks_per_sec = 1000ULL * sys_get_hiresclk_ticks_per_second();
		uint64 T1 = (gCUDA.T1_end - clk) * VIA_TIMER_FREQ_DIV_HZ_TIMES_1000 / ticks_per_sec;
		gCUDA.rT1CL = T1;
		gCUDA.rT1CH = T1 >> 8;
		//
//		uint64 tmp = gCUDA.T1_end - clk;
//		IO_CUDA_WARN("T1 running, T1 now %04x, T1_end-clk=%08qx\n", (uint32)T1, tmp);
	} else if (gCUDA.rACR & T1MODE_CONT) {
		uint64 ticks_per_sec = 1000ULL * sys_get_hiresclk_ticks_per_second();
		uint64 T1_latch = (gCUDA.rT1LH << 8) | gCUDA.rT1LL;
		uint64 full_T1_interval_ticks = (T1_latch+1) * ticks_per_sec / VIA_TIMER_FREQ_DIV_HZ_TIMES_1000;
		uint64 T1_end = clk + full_T1_interval_ticks - (clk - gCUDA.T1_end) % full_T1_interval_ticks;
		gCUDA.T1_end = T1_end;
		uint64 T1 = (gCUDA.T1_end - clk) * VIA_TIMER_FREQ_DIV_HZ_TIMES_1000 / ticks_per_sec;
		gCUDA.rT1CL = T1;
		gCUDA.rT1CH = T1 >> 8;
		gCUDA.rIFR |= T1_INT;
		gT1Raises++;
		cuda_renew_interrupt();
		//
//		uint64 tmp = gCUDA.T1_end - clk;
		//	IO_CUDA_WARN("T1 overflowed, setting interrupt flag, T1 set to %04x, T1_end-clk=%08qx, T1_latch = %04x\n", (uint32)T1, tmp, T1_latch);
	} else {
		/* In one-shot mode the first underflow raises T1_INT, but the VIA
		 * does not arm another interrupt until T1CH is written again. */
		gCUDA.T1_running = false;
		gCUDA.rT1CL = 0xff;
		gCUDA.rT1CH = 0xff;
		gCUDA.rIFR |= T1_INT;
		gT1Raises++;
		cuda_renew_interrupt();
	}
}

static void cuda_start_T1()
{
	uint64 clk = sys_get_hiresclk_ticks();
	uint64 ticks_per_sec = 1000ULL * sys_get_hiresclk_ticks_per_second();
	uint32 T1 = (gCUDA.rT1CH << 8) | gCUDA.rT1CL;
/*	uint64 tmp = static_cast<uint64>(T1) * ticks_per_sec / VIA_TIMER_FREQ_DIV_HZ_TIMES_1000;
	printf("T1 for %lld ticks (%g seconds vs. %g)\n",
		   tmp, static_cast<double>(tmp)/static_cast<double>(ticks_per_sec / 1000),
		   static_cast<double>(T1) * 1.27655 / 1000000.0);*/
	gCUDA.T1_end = clk + (static_cast<uint64>(T1) + 1) * ticks_per_sec /
		VIA_TIMER_FREQ_DIV_HZ_TIMES_1000;
	gCUDA.T1_running = true;
	gCUDA.rIFR &= ~T1_INT;
	IO_CUDA_TRACE("T1 restarted, T1 = %08x\n", T1);
}
/*
 * Did the ROM resolve the video card's interrupt?  A node whose interrupt
 * resolved carries AAPL,interrupts in its Name Registry entry; without it the
 * driver loader installs no handler and says nothing -- which is exactly the
 * USB symptom that [[pearpc-of-irq-binding-rule]] describes, and would explain
 * why nothing ever installs a VBL handler for the display.
 */
/*
 * video.x is MOL's MacOnLinuxVideo driver.  On a failed Initialize it calls
 * PublishInitFailureMsg, which creates FAILURE / FAIL-CODE properties in the
 * Name Registry carrying one of its reason strings.  A reason string that
 * appears only once is just the driver's string table; a second, separate
 * occurrence means the message was actually published -- i.e. Initialize
 * really failed, and the driver never got as far as registering its VBL
 * interrupt service.
 */
/*
 * Read-only reconnaissance of Mac OS's OS event queue, before writing to it.
 *
 * PostEvent takes a record from the pool at SysEvtBuf (0x146), which holds
 * EvtBufCnt+1 (0x154) elements, and links it onto EventQueue (0x14a).  An
 * EvQEl is qLink(4) qType(2) evtQWhat(2) evtQMessage(4) evtQWhen(4)
 * evtQWhere(4) evtQModifiers(2) = 22 bytes.  Confirm the pool pointer, the
 * count and the queue links look sane, and that the linked elements really do
 * land on a regular stride inside the pool, before trusting any of it.
 */
/*
 * Find the REAL event queue.
 *
 * Writing into the low-memory EventQueue (0x14a) does nothing -- Mac OS never
 * dequeues from it, with either qType, so it is vestigial on Mac OS 9/PowerPC.
 * But the keyboard module's PostEvent demonstrably works, so a real queue
 * exists somewhere.  Find it empirically: an event record is
 * qLink(4) qType(2) what(2) message(4) when(4) where(4) modifiers(2), so scan
 * RAM for a plausible one -- qType == 4 (evType) with a sane "what" and a
 * "when" close to the current Ticks -- and report where it lives.
 */
static void scan_for_event_records(const char *tag)
{
	/*
	 * The keyboard's events ARE delivered and drained (keys work at the login
	 * screen), so a live event queue exists -- but it is not the low-memory
	 * EventQueue at 0x14a, where anything we link is never dequeued.  Find the
	 * real one: press a key, then sweep guest RAM for the record it created.
	 * An EvQEl is qLink(4) qType(2) what(2) message(4) when(4) where(4)
	 * modifiers(2); match qType == 4 (evType) with a sane what and a recent
	 * when.  Prints unconditionally so a silent run cannot be mistaken for a
	 * negative result.
	 */
	uint8 tk[4] = {0,0,0,0};
	ppc_dma_read(tk, 0x4000 + 0x16a, 4);
	uint32 now = ((uint32)tk[0]<<24)|((uint32)tk[1]<<16)|((uint32)tk[2]<<8)|tk[3];
	extern uint32 gMemorySize;
	uint32 limit = gMemorySize;
	fprintf(stderr, "[EVSCAN:%s] begin: Ticks=%u memSize=%08x\n", tag, now, limit);
	const uint32 CHUNK = 1u << 20;
	uint8 *buf = (uint8 *)malloc(CHUNK);
	if (!buf) { fprintf(stderr, "[EVSCAN:%s] malloc failed\n", tag); return; }
	int found = 0, chunks = 0;
	for (uint32 base = 0; base + CHUNK <= limit && found < 16; base += CHUNK) {
		if (!ppc_dma_read(buf, base, CHUNK)) continue;
		chunks++;
		for (uint32 i = 0; i + 24 < CHUNK; i += 2) {
			if (buf[i+4] || buf[i+5] != 4) continue;		/* qType == evType */
			uint16 what = (uint16)((buf[i+6]<<8)|buf[i+7]);
			if (what < 1 || what > 6) continue;
			uint32 when = ((uint32)buf[i+12]<<24)|((uint32)buf[i+13]<<16)|
			              ((uint32)buf[i+14]<<8)|buf[i+15];
			if (when > now || now - when > 1200) continue;
			fprintf(stderr, "[EVSCAN:%s] @%08x what=%u msg=%02x%02x%02x%02x when=%u "
				"(now=%u) where=(%d,%d)\n", tag, base + i, what,
				buf[i+8], buf[i+9], buf[i+10], buf[i+11], when, now,
				(sint16)((buf[i+16]<<8)|buf[i+17]),
				(sint16)((buf[i+18]<<8)|buf[i+19]));
			found++;
			if (found >= 16) break;
		}
	}
	free(buf);
	fprintf(stderr, "[EVSCAN:%s] done: %d record(s) in %d chunk(s)\n", tag, found, chunks);
}

static void probe_event_queue()
{
	uint8 b[4], c[2], q[10];
	if (!ppc_dma_read(b, 0x4000 + 0x146, 4)) return;
	uint32 buf = ((uint32)b[0]<<24)|((uint32)b[1]<<16)|((uint32)b[2]<<8)|b[3];
	if (!ppc_dma_read(c, 0x4000 + 0x154, 2)) return;
	int cnt = ((c[0]<<8)|c[1]) + 1;
	if (!ppc_dma_read(q, 0x4000 + 0x14a, 10)) return;
	uint32 head = ((uint32)q[2]<<24)|((uint32)q[3]<<16)|((uint32)q[4]<<8)|q[5];
	uint32 tail = ((uint32)q[6]<<24)|((uint32)q[7]<<16)|((uint32)q[8]<<8)|q[9];
	fprintf(stderr, "[EVQ] SysEvtBuf=%08x EvtBufCnt+1=%d qFlags=%02x%02x qHead=%08x qTail=%08x\n",
		buf, cnt, q[0], q[1], head, tail);
	if (!buf || cnt <= 0 || cnt > 256) {
		fprintf(stderr, "[EVQ]   pool looks wrong -- do not write\n");
		return;
	}
	uint32 p = head;
	int n = 0;
	while (p && n < cnt + 2) {
		uint8 e[22];
		if (!ppc_dma_read(e, p, 22)) break;
		long delta = (long)p - (long)buf;
		fprintf(stderr, "[EVQ]   el@%08x (buf%+ld, /22=%ld rem=%ld) qType=%d what=%d "
			"where=(%d,%d)\n", p, delta, delta/22, delta%22,
			(e[4]<<8)|e[5], (e[6]<<8)|e[7],
			(sint16)((e[16]<<8)|e[17]), (sint16)((e[18]<<8)|e[19]));
		p = ((uint32)e[0]<<24)|((uint32)e[1]<<16)|((uint32)e[2]<<8)|e[3];
		n++;
	}
	fprintf(stderr, "[EVQ]   %d element(s) linked\n", n);
}

static void probe_video_driver_failure()
{
	static const char *reasons[] = {
		"No assigned-addresses property", "No AAPL,address property",
		"No valid address space", "RegistryPropertyGet failed",
		"Initialize failed", "FAIL-CODE" };
	const uint32 memSize = ppc_get_memory_size();
	const uint32 chunk = 4 * 1024 * 1024;
	byte *buf = new byte[chunk + 64];
	int count[6] = {0,0,0,0,0,0};
	for (uint32 base = 0; base < memSize; base += chunk - 64) {
		uint32 want = (base + chunk <= memSize) ? chunk : (memSize - base);
		if (!ppc_dma_read(buf, base, want)) continue;
		for (unsigned k = 0; k < 6; k++) {
			size_t len = strlen(reasons[k]);
			for (uint32 i = 0; i + len < want; i++)
				if (memcmp(buf + i, reasons[k], len) == 0) {
					/* Report addresses, not just counts: three copies of the
					 * driver's PEF are resident, so three hits is what the
					 * string tables alone produce.  Only a hit far from the
					 * others means the message was really published. */
					if (count[k] < 6)
						fprintf(stderr, "[VDRVAT] \"%s\" @ %08x\n",
							reasons[k], base + i);
					count[k]++;
				}
		}
	}
	for (unsigned k = 0; k < 6; k++)
		fprintf(stderr, "[VDRV] \"%s\" x%d%s\n", reasons[k], count[k],
			count[k] > 1 ? "   <== PUBLISHED: this failure actually happened" : "");
	delete[] buf;
}

static void probe_video_node_interrupts()
{
	const uint32 memSize = ppc_get_memory_size();
	const uint32 chunk = 4 * 1024 * 1024;
	const char *needle = "PearPCVideo";
	const size_t nlen = strlen(needle);
	byte *buf = new byte[chunk + 64];
	int hits = 0, withIrq = 0, totalAapl = 0;
	for (uint32 base = 0; base + nlen < memSize && hits < 8; base += chunk - 64) {
		uint32 want = (base + chunk <= memSize) ? chunk : (memSize - base);
		if (!ppc_dma_read(buf, base, want)) continue;
		for (uint32 i = 0; i + nlen < want; i++) {
			if (memcmp(buf + i, needle, nlen) != 0) continue;
			hits++;
			/* look for AAPL,interrupts within the surrounding registry entry */
			uint32 lo = i > 8192 ? i - 8192 : 0;
			uint32 hi = (i + 8192 < want) ? i + 8192 : want;
			/* A node whose interrupt really resolved carries the whole set,
			 * not just AAPL,interrupts: the driver's VSLNewInterruptService /
			 * InstallInterruptFunctions need the index and vectors too. */
			static const char *props[] = {
				"AAPL,interrupts", "AAPL,interrupt-index",
				"AAPL,interrupt-vectors", "AAPL,interrupt-priorities" };
			char line[256]; int off = 0; bool found = false;
			for (unsigned k = 0; k < 4; k++) {
				size_t plen = strlen(props[k]);
				bool got = false;
				for (uint32 j = lo; j + plen < hi; j++)
					if (memcmp(buf + j, props[k], plen) == 0) { got = true; break; }
				if (k == 0) found = got;
				off += snprintf(line + off, sizeof line - off, " %s=%s",
					props[k] + 5, got ? "yes" : "NO");
			}
			if (found) withIrq++;
			fprintf(stderr, "[NREG] PearPCVideo at %08x --%s\n", base + i, line);
			if (hits >= 8) break;
		}
		/* count AAPL,interrupts overall, as a sanity check that the ROM makes them at all */
		for (uint32 j = 0; j + 15 < want; j++)
			if (memcmp(buf + j, "AAPL,interrupts", 15) == 0) totalAapl++;
	}
	fprintf(stderr, "[NREG] PearPCVideo entries=%d withAAPLinterrupts=%d totalAAPLinterrupts=%d\n",
		hits, withIrq, totalAapl);
	delete[] buf;
}

static bool cuda_complete_newworld_interrupt_probe()
{
	const uint32 memorySize = ppc_get_memory_size();
	const uint32 searchSize = memorySize < 4 * 1024 * 1024 ? memorySize : 4 * 1024 * 1024;
	const uint32 searchStart = memorySize - searchSize;
	byte *buffer = new byte[searchSize];
	if (!ppc_dma_read(buffer, searchStart, searchSize)) {
		delete[] buffer;
		return false;
	}

	bool completed = false;
	const byte controllerType = gCUDA.pmuMode ? 0x02 : 0x01;
	for (uint32 i = 0x70; i + 0x1a < searchSize; ++i) {
		if (buffer[i] != 'H' || buffer[i + 1] != 'n' || buffer[i + 2] != 'f' || buffer[i + 3] != 'o') {
			continue;
		}
		const uint32 info = i - 0x70;
		if (buffer[info + 0x7a] != 0x08 || buffer[info + 0x7b] != 0x00 ||
		    buffer[info + 0x80] != 0x00 || buffer[info + 0x81] != IO_PIC_IRQ_CUDA ||
		    buffer[info + 0x82] != 0x00 || buffer[info + 0x83] != controllerType ||
		    buffer[info + 0x88] != 0x00 || buffer[info + 0x89] != 0x40) {
			continue;
		}

		const byte cudaInterrupt[] = {0x00, IO_PIC_IRQ_CUDA};
		completed = ppc_dma_write(searchStart + info + 0x7a, cudaInterrupt, sizeof cudaInterrupt);
		break;
	}
	delete[] buffer;
	return completed;
}

static void cuda_update_T2()
{
	if (gCUDA.T2_probe_pending) {
		if (gCUDA.T2_probe_attempts++ >= 256) {
			gCUDA.T2_probe_pending = false;
		} else if (cuda_complete_newworld_interrupt_probe()) {
			gCUDA.T2_probe_pending = false;
			gCUDA.T2_probe_completed = true;
			gCUDA.rIFR &= ~T2_INT;
			cuda_renew_interrupt();
		}
	}
	if (!gCUDA.T2_running) return;

	uint64 clk = sys_get_hiresclk_ticks();
	uint64 ticks_per_sec = 1000ULL * sys_get_hiresclk_ticks_per_second();
	if (clk < gCUDA.T2_end) {
		uint64 T2 = (gCUDA.T2_end - clk) * CUDA_T2_TIMER_FREQ_HZ_TIMES_1000 / ticks_per_sec;
		if (T2 > 0xffff) T2 = 0xffff;
		gCUDA.rT2CL = T2;
		gCUDA.rT2CH = T2 >> 8;
		return;
	}

	/* Timer 2 is a one-shot timer in timed-interrupt mode. */
	gCUDA.T2_running = false;
	gCUDA.rT2CL = 0xff;
	gCUDA.rT2CH = 0xff;
	gCUDA.rIFR |= T2_INT;
	gCUDA.T2_probe_pending = true;
	gCUDA.T2_probe_attempts = 0;
	cuda_renew_interrupt();
}

static void cuda_start_T2()
{
	uint64 ticks_per_sec = 1000ULL * sys_get_hiresclk_ticks_per_second();
	uint32 T2 = (gCUDA.rT2CH << 8) | gCUDA.rT2CL;
	gCUDA.T2_end = sys_get_hiresclk_ticks() +
		(static_cast<uint64>(T2) + 1) * ticks_per_sec / CUDA_T2_TIMER_FREQ_HZ_TIMES_1000;
	gCUDA.T2_running = true;
	gCUDA.T2_probe_pending = false;
	gCUDA.T2_probe_attempts = 0;
	gCUDA.rIFR &= ~T2_INT;
	cuda_renew_interrupt();
}

static int cuda_ifr_read_count = 0;

void cuda_write(uint32 addr, uint32 data, int size)
{
	sys_lock_mutex(gCUDAMutex);

	IO_CUDA_TRACE("%d write word @%08x: %08x\n", gCUDA.state, addr, data);
	addr -= IO_CUDA_PA_START;
	switch (addr) {
	case A:
		IO_CUDA_TRACE("->A\n");
		gCUDA.rA = data;
		break;
	case B: {
		if (gCUDA.pmuMode) {
            const byte oldB = gCUDA.rB;
            gCUDA.rORB = data;
            data = (gCUDA.rB & ~gCUDA.rDIRB) | (gCUDA.rORB & gCUDA.rDIRB);
            /* PMU uses only TREQ/TACK; some ROMs leave TIP as an output too. */
            if ((gCUDA.rDIRB & (PMU_TREQ | PMU_TACK)) != PMU_TREQ) {
                gCUDA.rB = data;
                break;
            }
            if ((oldB & PMU_TREQ) && !(data & PMU_TREQ)) {
                gCUDA.rB = data & ~PMU_TACK;
                if (gCUDA.SR_transfer_armed) {
                    pmu_transfer_byte();
                    gCUDA.SR_transfer_armed = false;
                    cuda_schedule_sr_interrupt();
                } else {
                    gCUDA.SR_transfer_armed = false;
                }
            } else if (!(oldB & PMU_TREQ) && (data & PMU_TREQ)) {
                gCUDA.rB = data | PMU_TACK;
            } else {
                gCUDA.rB = (data & ~PMU_TACK) | (oldB & PMU_TACK);
            }
            cuda_renew_interrupt();
            break;
        }
		/*
		 * Port B is a mixed input/output register.  TIP and TACK are
		 * driven by the VIA, while TREQ is driven by CUDA.  Keep the
		 * output latch separate so guest writes cannot overwrite TREQ,
		 * and do not interpret handshake transitions before DIRB has
		 * configured the three lines.
		 */
		gCUDA.rORB = data;
		data = (gCUDA.rB & ~gCUDA.rDIRB) | (gCUDA.rORB & gCUDA.rDIRB);
		if ((gCUDA.rDIRB & (TIP | TACK | TREQ)) != (TIP | TACK)) {
			gCUDA.rB = data;
			break;
		}

		gCUDA.rB = (gCUDA.rB & ~(TIP | TACK)) |
			(gCUDA.oldTIP ? TIP : 0) | (gCUDA.oldTACK ? TACK : 0);
		bool ack = false;
		if (gCUDA.rB & TACK) {
			if (!(data & TACK)) {
				cuda_schedule_sr_interrupt();
				if (gCUDA.state == cuda_idle) {
					data &= ~TREQ;
				}
				ack = true;
			}
		} else {
			if ((data & TACK)) {
				cuda_schedule_sr_interrupt();
				if (gCUDA.state == cuda_idle) {
					if (data & TIP) {
						data |= TREQ;
					} else {
						data &= ~TREQ;
        				}
				}
				ack = true;
			}
		}
		if ((gCUDA.state == cuda_reading) && ack && (gCUDA.rACR & SR_OUT) 
		&& !(!(gCUDA.rB & TIP) && (data & TIP))) {
			// don't ask...
			gCUDA.data[gCUDA.pos] = gCUDA.rSR;
			IO_CUDA_TRACE2(";; %d:%x\n", gCUDA.pos, gCUDA.rSR);
			gCUDA.pos++;
			if (gCUDA.pos > 10) {
				gCUDA.pos = 0;
				IO_CUDA_ERR("cuda overflow!\n");
			}
		}
		if ((gCUDA.state == cuda_writing) && ack) {
			if (gCUDA.left <= 1) {
				data |= TREQ;
			}
//			gCUDA.rB = data;
//			break;
		}
		if ((gCUDA.rB & TIP) && !(data & TIP)) {
			cuda_schedule_sr_interrupt();
//			IO_CUDA_TRACE2("^ from: %08x %02x\n", gCPU.pc, gCUDA.rIFR);
			if (gCUDA.rACR & SR_OUT) {
				gCUDA.state = cuda_reading;
				IO_CUDA_TRACE2("CUDA CHANGE STATE %d: to %d\n", __LINE__, gCUDA.state);
				gCUDA.pos = 1;
				data |= TREQ;
				gCUDA.data[0] = gCUDA.rSR;
				IO_CUDA_TRACE2(";; %d:%x\n", gCUDA.pos, gCUDA.rSR);
			} else {
				if (gCUDA.left) {
					gCUDA.state = cuda_writing;
					gCUDA.response_irq_armed = false;
					IO_CUDA_TRACE2("CUDA CHANGE STATE %d: to %d\n", __LINE__, gCUDA.state);
				} else {
//					data &= ~TIP;
				}
				data &= ~TREQ;
			}
		}
		IO_CUDA_TRACE2("[CUDA-REGB] state=%d rB=%02x data=%02x ifr=%02x\n", gCUDA.state, gCUDA.rB, data, gCUDA.rIFR);
		if (!(gCUDA.rB & TIP) && (data & TIP)) {
			cuda_schedule_sr_interrupt();
			// Keep TREQ asserted when processing the request queued a reply.
			// cuda_receive_packet() calls cuda_send_packet(), which lowers TREQ;
			// restoring the pre-reply port value here loses that edge and leaves
			// the guest servicing a stream of shift-register interrupts without
			// ever starting the reply transfer.
			data |= TIP;
			gCUDA.rB = data;
			if (gCUDA.state == cuda_reading) {
				cuda_receive_packet();
				if (gCUDA.left) {
					data &= ~TREQ;
				} else {
					data |= TREQ;
//					pic_cancel_interrupt(IO_PIC_IRQ_CUDA);
					gCUDA.rIFR &= ~SR_INT;
				}
				gCUDA.rB = data;
				if (gCUDA.state != cuda_writing) {
					gCUDA.state = cuda_idle;
				}
			} else if (gCUDA.state == cuda_writing) {
				IO_CUDA_TRACE2("cuda sent packet (%d)\n", gCUDA.pos);
				gCUDA.left = 0;
				gCUDA.response_irq_armed = false;
				gCUDA.pos = 0;
				gCUDA.state = cuda_idle;
			} else {
				gCUDA.state = cuda_idle;
			}
			sys_signal_semaphore(gCUDA.idle_sem);
			IO_CUDA_TRACE2("CUDA CHANGE STATE %d: to %d\n", __LINE__, gCUDA.state);
		} else {
			gCUDA.rB = data;
		}
		gCUDA.oldTIP = (gCUDA.rB & TIP) != 0;
		gCUDA.oldTACK = (gCUDA.rB & TACK) != 0;
		cuda_renew_interrupt();
		IO_CUDA_TRACE("->B(%02x)\n", gCUDA.rB);
		break;
	}
    	case DIRB:
		IO_CUDA_TRACE("->DIRB\n");
		gCUDA.rDIRB = data;
		break;
    	case DIRA:
		IO_CUDA_TRACE("->DIRA\n");
		gCUDA.rDIRA = data;
		break;
    	case T1CL:
		IO_CUDA_TRACE("->T1CL\n");
		// same as writing to T1LL
		gCUDA.rT1CL = data;
		gCUDA.rT1LL = data;
		break;
    	case T1CH:
		IO_CUDA_TRACE("->T1CH\n");
		/* from [1]: "[T1C-L] is loaded automatically from the low-order\
		 * latch (T1L-L) when the processor writes into the high-order counter\
		 * (T1C-H)"
		 * and: "8 bits loaded into high-order latches. also at this time both \
		 * high- and low-order latches transferred into T1 counter"
		 */
		gCUDA.rT1LH = data;
		gCUDA.rT1CH = gCUDA.rT1LH;
		gCUDA.rT1CL = gCUDA.rT1LL;
		cuda_start_T1();
		break;
    	case T1LL:
		IO_CUDA_TRACE("->T1LL\n");
		/* from [1]: "this operation is no different than a write into reg 4"
		 * reg4 is T1CL
		 */
		gCUDA.rT1CL = data;
		gCUDA.rT1LL = data;
		break;
    	case T1LH:
		IO_CUDA_TRACE("->T1LH\n");
		gCUDA.rT1LH = data;
		break;
    	case T2CL:
		IO_CUDA_TRACE("->T2CL\n");
		gCUDA.rT2CL = data;
		break;
    	case T2CH:
		IO_CUDA_TRACE("->T2CH\n");
		gCUDA.rT2CH = data;
		cuda_start_T2();
		break;
    	case ACR:
		IO_CUDA_TRACE("->ACR\n");
		gCUDA.rACR = data;
		break;
    	case SR:
		IO_CUDA_TRACE("->SR\n");
		gCUDA.rSR = data;
		gCUDA.SR_pending = false;
		if (gCUDA.pmuMode) gCUDA.SR_transfer_armed = true;
		gCUDA.rIFR &= ~SR_INT;
		cuda_renew_interrupt();
		break;
    	case PCR:
		IO_CUDA_TRACE("->PCR\n");
		gCUDA.rPCR = data;
		break;
	case IFR:
		IO_CUDA_TRACE("->IFR\n");
		if (data & T2_INT) gCUDA.T2_probe_pending = false;
		gCUDA.rIFR &= ~(data & 0x7f);
		cuda_renew_interrupt();
		break;
	case IER:
		IO_CUDA_TRACE("->IER\n");
		if (data & 0x80) {
			gCUDA.rIER |= data & 0x7f;
		} else {
			if ((data & T2_INT) && (gCUDA.rIFR & T2_INT) && !gCUDA.T2_probe_completed &&
			    !gCUDA.T2_probe_pending) {
				gCUDA.T2_probe_pending = true;
				gCUDA.T2_probe_attempts = 0;
			}
			gCUDA.rIER &= ~(data & 0x7f);
		}
		cuda_renew_interrupt();
		break;
    	case ANH:
		IO_CUDA_TRACE("->ANH\n");
		gCUDA.rANH = data;
		break;
	default:
		IO_CUDA_ERR("unknown service\n");
	}

	sys_unlock_mutex(gCUDAMutex);
}

static void cuda_shim_apply();

void cuda_read(uint32 addr, uint32 &data, int size)
{
	cuda_shim_apply();	/* CPU thread: safe point to touch guest memory */
	sys_lock_mutex(gCUDAMutex);

	IO_CUDA_TRACE("%d read word @%08x\n", gCUDA.state, addr);
	uint32 reg = addr - IO_CUDA_PA_START;
	if (reg == IFR || reg == IER) {
		cuda_update_sr_interrupt();
	}
	if (reg != 0x1a00 /* IFR */ && cuda_ifr_read_count > 100) {
		IO_CUDA_WARN("broke out of IFR loop after %d reads, now reading reg %04x\n",
			cuda_ifr_read_count, reg);
		cuda_ifr_read_count = 0;
	}
	addr -= IO_CUDA_PA_START;
	switch (addr) {
	case A:
		IO_CUDA_TRACE("A(%02x)->\n", gCUDA.rA);
		data = gCUDA.rA;
		break;
	case B: {
		IO_CUDA_TRACE("B(%02x)->\n", gCUDA.rB);
		data = gCUDA.rB;
		}
		break;
	case DIRB:
		IO_CUDA_TRACE("DIRB(%02x)->\n", gCUDA.rDIRB);
		data = gCUDA.rDIRB;
		break;
	case DIRA:
		IO_CUDA_TRACE("DIRA->\n");
		data = gCUDA.rDIRA;
		break;
	case T1CL:
		IO_CUDA_TRACE("T1CL->\n");
		cuda_update_T1();
		data = gCUDA.rT1CL;
		gCUDA.rIFR &= ~T1_INT;
		cuda_renew_interrupt();
		break;
	case T1CH: {
		IO_CUDA_TRACE("T1CH->\n");
		cuda_update_T1();
//		uint64 clk = sys_get_cpu_ticks();
//		IO_CUDA_WARN("read %08x: T1 = %04x clk = %08qx, T1_end = %08qx\n",
//			gCPU.current_code_base + gCPU.pc_ofs,
//			(gCUDA.rT1CH<<8) | gCUDA.rT1CL,
//			clk, gCUDA.T1_end);
		data = gCUDA.rT1CH;
		break;
	}
	case T1LL:
		IO_CUDA_WARN("T1LL->\n");
		data = gCUDA.rT1LL;
		break;
	case T1LH:
		IO_CUDA_WARN("T1LH->\n");
		data = gCUDA.rT1LH;
		break;
	case T2CL:
		IO_CUDA_TRACE("T2CL->\n");
		cuda_update_T2();
		data = gCUDA.rT2CL;
		gCUDA.rIFR &= ~T2_INT;
		cuda_renew_interrupt();
		break;
	case T2CH:
		IO_CUDA_TRACE("T2CH->\n");
		cuda_update_T2();
		data = gCUDA.rT2CH;
		break;
	case ACR:
		IO_CUDA_TRACE("ACR->\n");
		data = gCUDA.rACR;
		break;
	case SR:
		IO_CUDA_TRACE("SR->\n");
		data = gCUDA.rSR;
		if (gCUDA.pmuMode) {
            gCUDA.SR_pending = false;
            gCUDA.SR_transfer_armed =
                (gCUDA.rDIRB & (PMU_TREQ | PMU_TACK)) == PMU_TREQ;
            gCUDA.rIFR &= ~SR_INT;
            cuda_renew_interrupt();
            break;
        }
		if (gCUDA.state == cuda_writing) {
			if (gCUDA.left) {
				data = gCUDA.data[gCUDA.pos];
				IO_CUDA_TRACE2("::%d:%02x\n", gCUDA.pos, data);
				gCUDA.pos++;
				gCUDA.left--;
			}
			if (gCUDA.left <= 0) {
				IO_CUDA_TRACE2("stop\n");
				gCUDA.rB |= TREQ;
				gCUDA.rB &= ~TIP;
			}
			gCUDA.rIFR &= ~SR_INT;
		} else if (gCUDA.state == cuda_reading) {
			/* TREQ is driven by CUDA and stays negated while the host is
			 * transmitting a request.  Asserting it here makes the guest
			 * interpret the next byte completion as a transfer collision. */
			gCUDA.rB |= TREQ;
			gCUDA.rIFR &= ~SR_INT;
		} else {
			if (gCUDA.left) {
				gCUDA.rB &= ~TREQ;
			} else {
				gCUDA.rB |= TREQ;
			}
			gCUDA.rIFR &= ~SR_INT;
		}
		/*
		 * Reading the 6522 shift register acknowledges the old interrupt
		 * and starts the next externally-clocked shift.  CUDA completes that
		 * shift shortly afterwards.  Keep this delayed: Mac OS reads SR
		 * immediately after a handshake edge, then polls IFR for completion.
		 */
		if (gCUDA.state == cuda_idle && gCUDA.left &&
		    gCUDA.response_irq_armed) {
			gCUDA.response_irq_armed = false;
			cuda_schedule_sr_interrupt();
		} else if (!(gCUDA.rACR & SR_OUT) &&
		           (gCUDA.state != cuda_idle || !gCUDA.left)) {
			cuda_schedule_sr_interrupt();
		}
		cuda_renew_interrupt();
		break;
	case PCR:
		IO_CUDA_TRACE("PCR->\n");
		data = gCUDA.rPCR;
		break;
	case IFR:
		cuda_ifr_read_count++;
		cuda_update_T1();
		cuda_update_T2();
		data = gCUDA.rIFR;
		if (gCUDA.state == cuda_idle) {
			if (!gCUDA.left /*&& !(gCUDA.rIER & SR_INT)*/) {
//				if (cuda_interrupt()) {
//					data |= SR_INT;
//					if (gCUDA.autopoll) pic_raise_interrupt(IO_PIC_IRQ_CUDA);
//				}
			}
//			ht_printf("is idle!\n");
		} else {
//			ht_printf("state not idle bla !\n");
//			data |= SR_INT;
		}
		IO_CUDA_TRACE("%d IFR->(%02x/%02x)\n", gCUDA.state, gCUDA.rIFR, data);
		break;
	case IER:
		IO_CUDA_TRACE("IER->\n");
		data = gCUDA.rIER | 0x80;
		break;
	case ANH:
		IO_CUDA_TRACE("ANH->\n");
		data = gCUDA.rANH;
		break;
	default:
		IO_CUDA_ERR("unknown service\n");
	}

	IO_CUDA_TRACE("%d read @%08x: %08x\n", gCUDA.state, addr + IO_CUDA_PA_START, data);
	sys_unlock_mutex(gCUDAMutex);
}

static sys_semaphore	gCUDAEventSem;
static Queue		gCUDAEvents(true);
static volatile sig_atomic_t gDebugInjectMouseClick;
static volatile sig_atomic_t gDebugInjectMouseMotion;

void cuda_debug_inject_mouse_click()
{
	gDebugInjectMouseClick = 1;
}

/* Diagnostic: inject synthetic mouse motion straight into the CUDA/PMU queue,
 * bypassing SDL and the mouse-grab gate, so the PMU -> guest path can be tested
 * on its own. */
void cuda_debug_inject_mouse_motion()
{
	gDebugInjectMouseMotion = 1;
}


/*
 * Fallback pointer shim.
 *
 * Mac OS on this machine takes input from USB HID; it enumerates the emulated
 * ADB bus but discards its data, and the emulated OHCI root hub is not brought
 * up by the guest's driver, so neither path moves the pointer.  Until one of
 * them works, drive the Cursor Manager's own globals directly.
 *
 * Low memory sits at physical 0x4000.  RawMouse and MTemp are what the
 * interrupt handler would normally write, CrsrNew tells the VBL task the
 * position moved, and MBState carries the button (active low).
 */
#define LOMEM_BASE	0x4000
#define LOMEM_MBSTATE	0x0172
#define LOMEM_MTEMP	0x0828
#define LOMEM_RAWMOUSE	0x082c
#define LOMEM_MOUSE	0x0830
#define LOMEM_CRSRPIN	0x0834
#define LOMEM_CRSRNEW	0x08ce
#define LOMEM_CRSRCOUPLE 0x08cf

static bool readPoint(uint32 off, sint16 &v, sint16 &h)
{
	uint8 b[4];
	if (!ppc_dma_read(b, LOMEM_BASE + off, 4)) return false;
	v = (sint16)((b[0] << 8) | b[1]);
	h = (sint16)((b[2] << 8) | b[3]);
	return true;
}

static void writePoint(uint32 off, sint16 v, sint16 h)
{
	uint8 b[4] = { (uint8)(v >> 8), (uint8)v, (uint8)(h >> 8), (uint8)h };
	ppc_dma_write(LOMEM_BASE + off, b, 4);
}

/*
 * Accumulated on the event thread, applied on the CPU thread.  Writing guest
 * memory straight from the event thread races the running CPU and corrupts
 * extensions during startup ("address error" in whatever is loading).
 */
static volatile int gShimDx, gShimDy;
static volatile int gShimButton;
static volatile int gShimPending;
/*
 * Off by default.  The shim writes Mac OS low memory directly, and doing that
 * before the Toolbox owns it corrupts startup -- boots bomb at around 5% with
 * "illegal instruction".  It moves RawMouse correctly once the system is up,
 * but it does not make the cursor redraw, so it buys nothing today.
 */
/*
 * Jam Mac OS's own mouse globals.  Nothing binds a driver to the emulated ADB
 * bus on a Cube and the USB stack does not yet start its UIM, so this is the
 * only path that reaches the cursor -- the same one SheepShaver and Basilisk II
 * use.  cuda_shim_write() refuses to run until CrsrPin holds a sane rectangle,
 * which keeps it clear of the startup window where the Toolbox has not yet
 * taken ownership of low memory.
 */
/*
 * Jam Mac OS's cursor globals directly.  Mac OS 9.2 draws the arrow at Mouse
 * (0x830) and its USB HID driver applies our button byte but silently drops
 * the motion bytes, so nothing else moves it.
 */
/*
 * Enabled: this is what moves the pointer now.  The old comment here called
 * the shim "conclusively dead" because writing Mouse(0x830) did not move the
 * *drawn* arrow -- true, but the wrong conclusion.  Mac OS keeps the drawn
 * cursor in Cursor Manager state that only the VBL cursor task updates, and
 * that task never runs (see video.x: its interrupt install is unreachable).
 * PearPC draws the pointer itself in SDLSystemDisplay::displayShow(), so the
 * shim's job is to keep Mac OS's own globals correct -- which it does, and
 * GetMouse()/Button() inside the guest now see the right position.
 */
static int gCudaShimEnabled = 1;
static int gShimQuiet = 0;
/*
 * Post a mouse event into Mac OS's OS event queue, the way PostEvent does.
 *
 * The guest's mouse module never posts events -- MBState flips but the Finder
 * and dialogs respond to mouseDown/mouseUp, so clicks do nothing.  The
 * keyboard posts fine through the same driver, so the Event Manager works;
 * only the mouse side is missing.  Fill that in here.
 *
 * PostEvent takes a record from the pool at SysEvtBuf (0x146), which holds
 * EvtBufCnt+1 (0x154) of them, and links it onto EventQueue (0x14a).  EvQEl:
 * qLink(4) qType(2) evtQWhat(2) evtQMessage(4) evtQWhen(4) evtQWhere(4)
 * evtQModifiers(2) = 22 bytes.  Reconnaissance on this guest: pool at
 * 00ad96a0, 48 elements, queue empty.  Every read is validated and this bails
 * rather than write anything it is unsure of.
 */
#define EVQ_ELEM_SIZE 22

static bool post_os_event(uint16 what, sint16 v, sint16 h, bool buttonDown)
{
	/*
	 * MUST run on the CPU thread, from a device access, so the guest is inside
	 * our code and cannot be in PostEvent/GetNextEvent at the same time.
	 * Called from anywhere else this races the OS on its own queue and halts
	 * the machine -- which is exactly what happened when it was called from
	 * cuda_shim_mouse() on the SDL input thread.
	 *
	 * It also refuses to do any slot arithmetic.  The pool element stride was
	 * never confirmed against a live record (the queue was empty every time it
	 * was sampled), and guessing it wrong writes into the middle of another
	 * record.  So: only post when the queue is EMPTY, and then use the first
	 * element of the pool, which needs no stride at all.  If the queue is busy
	 * the caller keeps the edge and tries again on the next access.
	 */
	uint8 b[4], c[2], q[10];
	if (!ppc_dma_read(b, LOMEM_BASE + 0x146, 4)) return false;
	uint32 buf = ((uint32)b[0]<<24)|((uint32)b[1]<<16)|((uint32)b[2]<<8)|b[3];
	if (!buf) return false;
	if (!ppc_dma_read(c, LOMEM_BASE + 0x154, 2)) return false;
	int cnt = ((c[0]<<8)|c[1]) + 1;
	if (cnt <= 0 || cnt > 256) return false;

	uint8 m[2];
	if (!ppc_dma_read(m, LOMEM_BASE + 0x144, 2)) return false;
	uint16 mask = (uint16)((m[0]<<8)|m[1]);
	if (!(mask & (1 << what))) return false;

	if (!ppc_dma_read(q, LOMEM_BASE + 0x14a, 10)) return false;
	uint32 head = ((uint32)q[2]<<24)|((uint32)q[3]<<16)|((uint32)q[4]<<8)|q[5];
	uint32 tail = ((uint32)q[6]<<24)|((uint32)q[7]<<16)|((uint32)q[8]<<8)|q[9];
	/*
	 * Append properly rather than only posting into an empty queue.  A lone
	 * mouseDown is not a click: the earlier build that worked -- IGA
	 * highlighted -- queued the down and the up as a chain, and posting only
	 * the down leaves the click unfinished, which is what stopped it working.
	 *
	 * That run is also what validates the 22-byte stride: it allocated a
	 * second element at buf+22 and Mac OS consumed both, so the element size
	 * is right.  Mark everything already linked so a live record is never
	 * handed out, and bail if a link looks wrong.
	 */
	bool used[256];
	memset(used, 0, sizeof used);
	uint32 p = head;
	for (int n = 0; p && n < cnt + 2; n++) {
		long d = (long)p - (long)buf;
		if (d < 0 || d % 22 || d / 22 >= cnt) return false;	/* not our pool */
		used[d / 22] = true;
		uint8 l[4];
		if (!ppc_dma_read(l, p, 4)) return false;
		p = ((uint32)l[0]<<24)|((uint32)l[1]<<16)|((uint32)l[2]<<8)|l[3];
	}
	int slot = -1;
	for (int i = 0; i < cnt; i++) if (!used[i]) { slot = i; break; }
	if (slot < 0) return false;
	uint32 el = buf + (uint32)slot * 22;

	uint8 tk[4] = {0,0,0,0};
	ppc_dma_read(tk, LOMEM_BASE + 0x16a, 4);			/* Ticks */

	uint8 e[22];
	memset(e, 0, sizeof e);
	e[4] = 0; e[5] = 4;		/* qType = evType (4).  Was 5 = fsQType, a volume
				 * control block -- Mac OS would rightly ignore an event
				 * queue element tagged as one, which is very likely why
				 * nothing was ever dequeued. */
	e[6] = (uint8)(what >> 8); e[7] = (uint8)what;			/* evtQWhat */
	e[12] = tk[0]; e[13] = tk[1]; e[14] = tk[2]; e[15] = tk[3];	/* evtQWhen */
	e[16] = (uint8)(v >> 8); e[17] = (uint8)v;			/* where.v */
	e[18] = (uint8)(h >> 8); e[19] = (uint8)h;			/* where.h */
	uint16 mods = buttonDown ? 0x0000 : 0x0080;	/* btnState set while UP */
	e[20] = (uint8)(mods >> 8); e[21] = (uint8)mods;
	if (!ppc_dma_write(el, e, sizeof e)) return false;

	uint8 link[4] = { (uint8)(el>>24), (uint8)(el>>16), (uint8)(el>>8), (uint8)el };
	if (tail) {
		if (!ppc_dma_write(tail, link, 4)) return false;		/* tail->qLink */
	} else {
		if (!ppc_dma_write(LOMEM_BASE + 0x14a + 2, link, 4)) return false;	/* qHead */
	}
	if (!ppc_dma_write(LOMEM_BASE + 0x14a + 6, link, 4)) return false;	/* qTail */
	gLastPostedEl = el;
	{
		static int n = 0;
		if (n < 12) {
			n++;
			fprintf(stderr, "[POST] what=%d at (h=%d,v=%d) slot=%d el=%08x\n", what, h, v, slot, el);
		}
	}
	return true;
}

static void cuda_shim_write(int dx, int dy, bool button);

void cuda_shim_mouse(int dx, int dy, bool button)
{
	gShimDx += dx;
	gShimDy += dy;
	gShimButton = button ? 1 : 0;
	gShimPending = 1;

	/*
	 * Post the click edge here, not from cuda_shim_write().  That runs out of
	 * cuda_shim_apply(), which only fires on a guest VIA *read* and was
	 * observed not to fire at all across a 250ms press -- and since the shim
	 * keeps only the latest button state, a press and release that land
	 * between two applies coalesce and the click vanishes.  Posting on the
	 * edge, at whatever position Mac OS currently believes the pointer is,
	 * cannot miss it.
	 */
	static int prevB = 0;
	int b = button ? 1 : 0;
	if (gCudaShimEnabled && prevB != b) {
		prevB = b;
		/*
		 * Arm the click HERE, at the edge, not in cuda_shim_apply.  Detecting
		 * it there meant comparing gShimButton whenever a cuda_read happened
		 * to occur, and the transition was repeatedly missed -- the ring
		 * dropped the mouseUp, and state comparison armed nothing at all.  At
		 * the edge the transition is known for certain.  The blr injection
		 * site delivers within microseconds, so a pending event is cleared
		 * long before the next edge can overwrite it.
		 */
		extern volatile int gPendingMouseEvent;
		extern volatile int gClickArmed;
		extern volatile int gJitFlushRequest;
		/* Experiment only.  Without this gate the default build would arm
		 * clicks and inject guest calls that are rejected from most contexts,
		 * which is both useless and a stability risk. */
		static const int armOn = getenv("PEARPC_CLICK_HIJACK") ? 1 : 0;
		if (armOn) {
			/* Queue the edge: a click is two edges and a single slot loses
			 * one of them. */
			extern volatile int gMouseEvQ[];
			extern volatile int gMouseEvHead, gMouseEvTail;
			int nxt = (gMouseEvTail + 1) % 8;
			if (nxt != gMouseEvHead) {
				gMouseEvQ[gMouseEvTail] = b ? 1 : 2;
				gMouseEvTail = nxt;
			}
			gPendingMouseEvent = b ? 1 : 2;	/* keeps provocation running */
		}
		{
			static int n = 0;
			if (n < 20) {
				n++;
				fprintf(stderr, "[EDGE] armed mouse event %d\n", gPendingMouseEvent);
			}
		}
		/* Record the edge only.  Posting it here would run on the input
		 * thread and race the guest on its own event queue; the CPU thread
		 * drains this in cuda_shim_apply().  A queue rather than a flag,
		 * because a press and release must both survive. */
		int nxt = (gClickTail + 1) % CLICK_RING;
		if (nxt != gClickHead) {
			gClickRing[gClickTail] = b ? 1 : 2;
			gClickTail = nxt;
		}
	}
}

/* Runs on the CPU thread, from the VIA register path. */
extern "C" void jitc_flush_all_now();
extern "C" void pearpc_inject_mouse(int dx, int dy, int buttonState);

static void cuda_shim_apply()
{
	if (!gCudaShimEnabled) return;

	/*
	 * Arm/disarm the blr trap on the CPU thread, where flushing the JIT is
	 * safe.  Translations are cached, so the flag only takes effect after an
	 * invalidateAll().
	 */
	{
		extern volatile int gPendingMouseEvent;
		extern volatile int gClickArmed;
		extern volatile int gJitFlushRequest;
		/*
		 * The glue trap must be gated -- `lwz r12,d(rA)` is far too common to
		 * interpret unconditionally, and doing so stops the guest reaching the
		 * login screen at all.  Gating only works if translations are
		 * discarded when the flag changes, so flush once on each transition:
		 * on arming, so the trap appears, and on disarming, so the cost goes
		 * away again.  One flush per click edge is affordable.
		 */
		/* Close the CDM hunt window: the driver initialises within a few
		 * seconds of being configured, and leaving the glue trap open past
		 * that stops the guest ever reaching the login screen. */
		extern volatile int gCdmHuntUntil;
		if (gCdmHuntUntil) {
			static uint64 huntStart = 0;
			if (!huntStart) huntStart = sys_get_hiresclk_ticks();
			if (sys_get_hiresclk_ticks() - huntStart >
			    sys_get_hiresclk_ticks_per_second() * 12) {
				gCdmHuntUntil = 0;
				gClickArmed = 0;
				gJitFlushRequest = 1;
				fprintf(stderr, "[CDM] hunt window closed\n");
			}
		}
		if (gPendingMouseEvent && !gClickArmed) {
			gClickArmed = 1;
			gJitFlushRequest = 1;
		} else if (!gPendingMouseEvent && gClickArmed) {
			gClickArmed = 0;
			gJitFlushRequest = 1;
		}
		if (gJitFlushRequest) {
			gJitFlushRequest = 0;
			jitc_flush_all_now();
		}
	}

	/*
	 * Dump the cursor device record periodically.  Comparing successive dumps
	 * while the pointer is being driven shows which field is the position --
	 * that is the value Mac OS hit-tests clicks against, and the thing that
	 * has to follow our motion.
	 */
	{
		extern volatile uint32 gCursorDeviceRec;
		static uint64 lastDump = 0;
		static int dumps = 0;
		if (gCursorDeviceRec && dumps < 6) {
			uint64 now = sys_get_hiresclk_ticks();
			if (now - lastDump > sys_get_hiresclk_ticks_per_second() * 2) {
				lastDump = now;
				dumps++;
				uint8 rec[48];
				if (ppc_dma_read(rec, gCursorDeviceRec, sizeof rec)) {
					uint8 mo[4] = {0,0,0,0};
					ppc_dma_read(mo, LOMEM_BASE + LOMEM_MOUSE, 4);
					fprintf(stderr, "[CDMREC] #%d Mouse=(%d,%d) @%08x:",
						dumps,
						(sint16)((mo[0]<<8)|mo[1]), (sint16)((mo[2]<<8)|mo[3]),
						gCursorDeviceRec);
					for (int i = 0; i < 48; i++) fprintf(stderr, " %02x", rec[i]);
					fprintf(stderr, "\n");
				}
			}
		}
	}

	/*
	 * Does our Mouse write survive?  The shim rewrites it constantly, so every
	 * probe so far read back our own value microseconds later.  Park a known
	 * value, stop writing for two seconds, and read it again: if Mac OS has
	 * replaced it, then Mac OS maintains that global from its own cursor and
	 * our writes are being clobbered -- which would explain clicks landing at
	 * a fixed spot no matter where the drawn pointer is.
	 */
	{
		extern volatile uint32 gCursorDeviceRec;
		(void)gCursorDeviceRec;
		static int phase = 0;
		static uint64 at = 0;
		static int done = 0;
		if (!done && gCudaShimEnabled) {
			uint64 now = sys_get_hiresclk_ticks();
			uint64 per = sys_get_hiresclk_ticks_per_second();
			if (phase == 0 && !at) { at = now; phase = 1; }
			else if (phase == 1 && now - at > per * 25) {
				sint16 tv = 100, th = 200;
				uint8 pt[4] = { (uint8)(tv>>8), (uint8)tv, (uint8)(th>>8), (uint8)th };
				ppc_dma_write(LOMEM_BASE + LOMEM_MOUSE, pt, 4);
				gShimQuiet = 1;			/* stop the shim rewriting it */
				at = now; phase = 2;
				fprintf(stderr, "[PERSIST] parked Mouse=(v=100,h=200), shim quiet\n");
			} else if (phase == 2 && now - at > per * 2) {
				uint8 mo[4] = {0,0,0,0};
				ppc_dma_read(mo, LOMEM_BASE + LOMEM_MOUSE, 4);
				sint16 v = (sint16)((mo[0]<<8)|mo[1]), h = (sint16)((mo[2]<<8)|mo[3]);
				fprintf(stderr, "[PERSIST] after 2s Mouse=(v=%d,h=%d) -> %s\n", v, h,
					(v == 100 && h == 200) ? "UNCHANGED: Mac OS does not maintain it"
					                       : "OVERWRITTEN by Mac OS");
				gShimQuiet = 0; done = 1;
			}
		}
	}

	if (gShimPending && !gShimQuiet) {
		int dx = gShimDx, dy = gShimDy;
		bool button = gShimButton != 0;
		gShimDx -= dx;
		gShimDy -= dy;
		gShimPending = 0;
		cuda_shim_write(dx, dy, button);
	}

	/*
	 * Drain click edges here, on the CPU thread, with the guest stopped inside
	 * this device access -- so it cannot be in PostEvent/GetNextEvent while we
	 * touch its queue.  One per call: post_os_event() only writes when the
	 * queue is empty, so a press stays pending until the OS has taken the
	 * previous event, which serialises press and release naturally.
	 */
	/*
	 * Does Mac OS ever take these records off the queue?  If qHead stays at
	 * whatever we linked, the OS is not draining and the events are inert --
	 * which would explain the login list not reacting despite both a
	 * mouseDown and a mouseUp being queued at the right coordinates.
	 */
	{
		static uint32 lastPosted = 0;
		static int samples = 0;
		extern uint32 gLastPostedEl;
		if (gLastPostedEl && samples < 10) {
			uint8 q[10];
			if (ppc_dma_read(q, LOMEM_BASE + 0x14a, 10)) {
				uint32 hd = ((uint32)q[2]<<24)|((uint32)q[3]<<16)|((uint32)q[4]<<8)|q[5];
				if (hd != lastPosted) {
					lastPosted = hd;
					samples++;
					fprintf(stderr, "[DRAIN] qHead=%08x (posted %08x) -> %s\n",
						hd, gLastPostedEl,
						hd == 0 ? "DRAINED by Mac OS" : "still queued");
				}
			}
		}
	}
	/*
	 * OFF by default: it does not work.  Mac OS never dequeues what we post --
	 * qHead stays pinned at the element we linked and never returns to zero --
	 * so the low-memory EventQueue (0x14a) is vestigial on Mac OS 9/PowerPC and
	 * the native Event Manager does not read it.  The records just accumulate.
	 * Kept behind PEARPC_CLICK_EVENTS=1 so the experiment is one env var away,
	 * but writing into guest memory for no benefit is not worth the risk.
	 */
	/*
	 * Hand click edges to the PostEvent hijack in ppc_mmu.cc.  Writing records
	 * into the low-memory EventQueue never worked -- Mac OS does not dequeue
	 * from it -- so instead the next PostEvent call the guest makes on its own
	 * gets its arguments substituted for our mouseDown/mouseUp.
	 */
	extern volatile int gPendingMouseEvent;
	/*
	 * OFF by default.  The hijack only rewrites a genuine PostEvent call, but
	 * arming it injects a synthetic keystroke to provoke one -- and when the
	 * hijack does not fire, that keystroke is simply typed into the guest.  A
	 * stray character on every click is worse than no click.  Enable with
	 * PEARPC_CLICK_HIJACK=1 to continue the experiment.
	 */
	static const int hijackOn = getenv("PEARPC_CLICK_HIJACK") ? 1 : 0;
	/*
	 * Post button edges by comparing the CURRENT button state against the last
	 * state actually posted, rather than draining a ring.  The ring lost an
	 * edge whenever the button changed while a previous one was still pending:
	 * the mouseDown would go out and the mouseUp never would, leaving a
	 * dangling press that the UI ignores.  Comparing state cannot lose one --
	 * whatever the button ends up doing, the next drain notices the
	 * difference and posts it.
	 */
	/* (click arming moved to cuda_shim_mouse, at the edge itself) */


	/*
	 * Release the synthetic key on a timer, not on the hijack completing:
	 * gating it on that deadlocks -- the key stays held, the guest posts
	 * nothing more, and the PostEvent call we are waiting to borrow never
	 * comes.  A full press/release is what actually generates one.
	 */
	if (gSyntheticKeyDown &&
	    sys_get_hiresclk_ticks() - gSyntheticKeyAt >
	        sys_get_hiresclk_ticks_per_second() / 12) {
		gSyntheticKeyDown = 0;
		usb_hid_key_event(gSyntheticKey, false);
	}
	/*
	 * Only ONE provocation loop may run.  Two of them -- this one and the
	 * event-loop one -- pressed and released different keys on independent
	 * timers, which leaves the HID keyboard report with overlapping presses
	 * and mismatched releases; the driver then posts nothing at all.  That is
	 * why [SWAP] never fired even though [PROV] showed keys going in.  The
	 * event-loop loop is the one kept.
	 */


	static const int clickEventsOff = 1;
	if (!clickEventsOff && gClickHead != gClickTail) {
		uint8 mo[4];
		if (ppc_dma_read(mo, LOMEM_BASE + LOMEM_MOUSE, 4)) {
			sint16 v = (sint16)((mo[0] << 8) | mo[1]);
			sint16 h = (sint16)((mo[2] << 8) | mo[3]);
			while (gClickHead != gClickTail) {
				int what = gClickRing[gClickHead];
				if (!post_os_event((uint16)what, v, h, what == 1)) break;
				gClickHead = (gClickHead + 1) % CLICK_RING;
			}
		}
	}
}

static void cuda_shim_write(int dx, int dy, bool button)
{
	sint16 v, h, top, left, bottom, right;
	if (!readPoint(LOMEM_RAWMOUSE, v, h)) return;
	if (!readPoint(LOMEM_CRSRPIN, top, left)) return;
	uint8 b[8];
	if (!ppc_dma_read(b, LOMEM_BASE + LOMEM_CRSRPIN, 8)) return;
	top    = (sint16)((b[0] << 8) | b[1]);
	left   = (sint16)((b[2] << 8) | b[3]);
	bottom = (sint16)((b[4] << 8) | b[5]);
	right  = (sint16)((b[6] << 8) | b[7]);
	/* Nothing sensible to clamp against until the Toolbox has set this up. */
	if (bottom <= top || right <= left) return;

	int nv = v + dy;
	int nh = h + dx;
	if (nv < top) nv = top;
	if (nv > bottom - 1) nv = bottom - 1;
	if (nh < left) nh = left;
	if (nh > right - 1) nh = right - 1;

	writePoint(LOMEM_MTEMP, (sint16)nv, (sint16)nh);
	writePoint(LOMEM_RAWMOUSE, (sint16)nv, (sint16)nh);
	/*
	 * Also write Mouse itself.  The framebuffer shows the arrow drawn at
	 * whatever Mouse holds, and nothing here updates it: Mac OS 9 expects its
	 * input driver to do that through the Cursor Device Manager, and ours
	 * applies the button byte but drops the motion bytes.  Writing RawMouse
	 * alone leaves the cursor where Mouse still points, which is exactly what
	 * the captures showed.
	 */
	writePoint(LOMEM_MOUSE, (sint16)nv, (sint16)nh);

	uint8 one = 1;
	ppc_dma_write(LOMEM_BASE + LOMEM_CRSRNEW, &one, 1);	/* redraw at the new spot */

	uint8 mb = button ? 0x00 : 0x80;			/* active low */
	ppc_dma_write(LOMEM_BASE + LOMEM_MBSTATE, &mb, 1);

	/*
	 * MBState alone is not enough: the Finder and dialogs react to
	 * mouseDown/mouseUp events, not to the button global.  The guest's mouse
	 * module never posts them, so post them ourselves on each edge, at the
	 * position Mac OS now believes the pointer is.
	 */
	/* (the click edge is posted from cuda_shim_mouse, see there) */
}

static bool cudaEventHandler(const SystemEvent &ev)
{
	/*
	 * A G4 Cube's keyboard and mouse are USB, and that is where Mac OS takes
	 * input from -- it enumerates the emulated ADB bus but ignores its data.
	 * Feed both so either path can serve the guest.
	 */
	if (ev.type == sysevMouse) {
		usb_hid_mouse_event(ev.mouse.relx, ev.mouse.rely,
			ev.mouse.button1, ev.mouse.button2, ev.mouse.button3);
		cuda_shim_mouse(ev.mouse.relx, ev.mouse.rely, ev.mouse.button1);
	} else if (ev.type == sysevKey) {
		usb_hid_key_event(ev.key.keycode, ev.key.pressed);
	}

	sys_lock_semaphore(gCUDAEventSem);
//	ht_printf("queue  %d\n", ev.key.pressed);
	gCUDAEvents.enQueue(new SystemEventObject(ev));
	sys_signal_semaphore(gCUDAEventSem);
	sys_unlock_semaphore(gCUDAEventSem);
	return true;
}

static bool doProcessCudaEvent(const SystemEvent &ev)
{
	if (gCUDA.pmuMode) {
        if (ev.type == sysevMouse) {
            /* Accumulate first, so motion is never lost just because an earlier
             * reply is still outstanding -- the guest may be polling for it. */
            gCUDA.pendingDx += ev.mouse.relx;
            gCUDA.pendingDy += ev.mouse.rely;
            gCUDA.pendingDx = MAX(-63, MIN(63, gCUDA.pendingDx));
            gCUDA.pendingDy = MAX(-63, MIN(63, gCUDA.pendingDy));
            gCUDA.pendingBtn1 = ev.mouse.button1;
            gCUDA.pendingBtn2 = ev.mouse.button2;
            gCUDA.pendingMouse = true;
        }
        if (gCUDA.pmuAdbReplyLength || (gCUDA.pmuInterruptBits & PMU_INT_ADB)) return false;
        if (ev.type == sysevKey) {
            uint8 key = ev.key.keycode;
            if (!ev.key.pressed) key |= 0x80;
            /* Same framing as a solicited ADB reply: status, length, then the
             * ADB command byte and its data.  See pmu_dispatch_command(). */
            gCUDA.pmuAdbReply[0] = 1;
            gCUDA.pmuAdbReply[1] = 3;
            gCUDA.pmuAdbReply[2] = 0x2c;
            gCUDA.pmuAdbReply[3] = key;
            gCUDA.pmuAdbReply[4] = 0xff;
            gCUDA.pmuAdbReplyLength = 5;
        } else if (ev.type == sysevMouse) {
            int dx = MAX(-63, MIN(63, ev.mouse.relx));
            int dy = MAX(-63, MIN(63, ev.mouse.rely));
            dx &= 0x7f;
            dy &= 0x7f;
            if (!ev.mouse.button2) dx |= 0x80;
            if (!ev.mouse.button1) dy |= 0x80;
            /* Try the plausible encodings in turn so one boot can test them all. */
            if (gPktVariant & 1) {
                gCUDA.pmuAdbReply[0] = 1;
                gCUDA.pmuAdbReply[1] = 3;
                gCUDA.pmuAdbReply[2] = 0x3c;
                gCUDA.pmuAdbReply[3] = dy;
                gCUDA.pmuAdbReply[4] = dx;
                gCUDA.pmuAdbReplyLength = 5;
            } else {
                gCUDA.pmuAdbReply[0] = 0x3c;
                gCUDA.pmuAdbReply[1] = dy;
                gCUDA.pmuAdbReply[2] = dx;
                gCUDA.pmuAdbReplyLength = 3;
            }
        } else {
            return false;
        }
        gAdbQueued++;
        gCUDA.pmuInterruptBits |= PMU_INT_ADB;
        if (!(gPktVariant & 2)) gCUDA.pmuInterruptBits |= PMU_INT_ADB_AUTO;
        pmu_update_ext_interrupt();
        return true;
    }

	switch (ev.type) {
	case sysevKey: {
		uint8 k = ev.key.keycode;
		if (!ev.key.pressed) {
			k |= 0x80;
		}
		cuda_send_packet(ADB_PACKET, 4, 0x40, 0x2c, k, 0xff);
		return true;
	}
	case sysevMouse: {
		int dx = ev.mouse.relx; //* 256 / gDisplay->mClientChar.width;
		int dy = ev.mouse.rely; //* 256 / gDisplay->mClientChar.height;
		// ADB mouse uses 7-bit signed deltas (-63..63),
		// encoded as two's complement in bits 0-6 (bit 7 = button state)
		if (dx < -63) dx = -63;
		if (dx > 63) dx = 63;
		if (dy < -63) dy = -63;
		if (dy > 63) dy = 63;
		dx &= 0x7f;
		dy &= 0x7f;
		if (!ev.mouse.button2) dx |= 0x80;
		if (!ev.mouse.button1) dy |= 0x80;
//		ht_printf("adb mouse: cur: %d, %d d: %d, %d\n", ev.mouseEvent.x, ev.mouseEvent.y, dx, dy);
		cuda_send_packet(ADB_PACKET, 4, 0x40, 0x3c, dy, dx);
		return true;
	}
	default:
		return false;
	}
}

static bool tryProcessCudaEvent(const SystemEvent &ev)
{
	uint timeout_msec = 200;
	uint64 time_end = sys_get_hiresclk_ticks() + sys_get_hiresclk_ticks_per_second()
		* timeout_msec / 1000;
//	ht_printf("process  %d\n", ev.key.pressed);
	while (sys_get_hiresclk_ticks() < time_end) {
		sys_lock_mutex(gCUDAMutex);
		static int lockuphack = 0;
		if (gCUDA.state == cuda_idle) {
			if (!gCUDA.left /*&& !(gCUDA.rIFR & SR_INT)*/) {
				lockuphack = 0;
				bool k = doProcessCudaEvent(ev);
				sys_unlock_mutex(gCUDAMutex);
//				IO_CUDA_WARN("Tried to process event: %d.\n", k);
				return k;
			} else {
				IO_CUDA_TRACE2("left: %d\n", gCUDA.left);
				if (lockuphack++ == 20) {
/*					gCUDA.left = 0;
					gCUDA.rA = TREQ;
					gCUDA.rACR = 0;
					lockuphack = 0;
					IO_CUDA_WARN("lock-up parachute\n");*/
				}
			}
		} else {
			IO_CUDA_TRACE2("cuda not idle (%d)!\n", gCUDA.state);
		}
		sys_unlock_mutex(gCUDAMutex);
		sys_lock_semaphore(gCUDA.idle_sem);
		sys_wait_semaphore_bounded(gCUDA.idle_sem, 10);
		sys_unlock_semaphore(gCUDA.idle_sem);
	}
	IO_CUDA_WARN("Event processing timed out. Event dropped.\n");
	return false;
}

static void *cudaEventLoop(void *arg)
{
	if (gKeyboard) gKeyboard->attachEventHandler(cudaEventHandler);
	if (gMouse) gMouse->attachEventHandler(cudaEventHandler);
	sys_lock_semaphore(gCUDAEventSem);
	while (1) {
//		IO_CUDA_WARN("waiting on semaphore\n");
		sys_wait_semaphore_bounded(gCUDAEventSem, 1);
		sys_lock_mutex(gCUDAMutex);
		cuda_update_T1();
		cuda_update_T2();
		cuda_update_sr_interrupt();
		sys_unlock_mutex(gCUDAMutex);
		{
			/* Provoke the keyboard module into calling PostEvent while a
			 * click is waiting, so the trap has a site to inject at. */
			extern volatile int gPendingMouseEvent;
			static uint64 provAt = 0;
			static int provDown = 0;
			static uint8 provKey = 0;
			uint64 per = sys_get_hiresclk_ticks_per_second();
			extern volatile int gMouseEvHead, gMouseEvTail;
			if (gMouseEvHead != gMouseEvTail) {
				if (!provDown && sys_get_hiresclk_ticks() - provAt > per / 8) {
					/* Harmless keys, not Return: provoking with Return works
					 * (it selects a user, proving synthetic keystrokes still
					 * reach the UI) but it also drives the login screen, which
					 * confuses what a click test is measuring. */
					static const uint8 keys[] = { 0x00, 0x01, 0x02, 0x03 };
					static unsigned pi = 0;
					provKey = keys[pi++ % (sizeof keys)];
					provDown = 1;
					provAt = sys_get_hiresclk_ticks();
					usb_hid_key_event(provKey, true);
					{	static int n = 0;
						if (n < 10) { n++;
							fprintf(stderr, "[PROV] loop key %02x down (pending=%d)\n",
								provKey, gPendingMouseEvent); } }
				} else if (provDown && sys_get_hiresclk_ticks() - provAt > per / 16) {
					provDown = 0;
					usb_hid_key_event(provKey, false);
				}
			} else if (provDown) {
				provDown = 0;
				usb_hid_key_event(provKey, false);
			}
			if (gMouseEvHead == gMouseEvTail) gPendingMouseEvent = 0;
		}
		if (gKeyScript >= 0) {
			/* Realistic pointer motion: ~50 small reports at the 8ms poll
			 * cadence, totalling (200,150).  A real trackpad moves the
			 * cursor this way; the previous single (60,90) jump was not a
			 * fair test of the guest's motion handling. */
			/* NOTE: do not call cudaEventHandler() from this thread -- it
			 * takes gCUDAEventSem, which cudaEventLoop already holds, and the
			 * recursive lock deadlocks the loop.  Real input is fine: it
			 * arrives on the SDL thread.  Drive the USB device directly.
			 *
			 * Sequence: Return picks the user, Return submits the (empty)
			 * password, then stream pointer motion.  Every mouse test so far
			 * ran on the Multiple Users login screen, which may not run a
			 * normal cursor environment; this gets to the desktop first.
			 */
			/*
			 * Deterministic aiming.  Parking the low-memory Mouse global is
			 * useless -- Mac OS keeps its own cursor position and clicks land
			 * there -- so drive the pointer the way a hand does: shove it into
			 * the bottom-right corner until it pins, then walk back by a known
			 * offset.  Slow steps keep the guest's acceleration curve at 1:1.
			 */
			if (gKeyScript < 400 && (gKeyScript % 4) == 0)
				pearpc_inject_mouse(40, 40, -1);	/* home into the corner */
			if (gKeyScript >= 400 && gKeyScript < 600 && (gKeyScript % 4) == 0)
				pearpc_inject_mouse(-7, -5, -1);	/* 50 steps -> (289,229) */
			if (gKeyScript == 700) {
				gcard_dump_framebuffer();
				rename("/tmp/pearpc-fb.ppm", "/tmp/pearpc-fb-aim.ppm");
				fprintf(stderr, "[AIM] cursor homed+walked -> /tmp/pearpc-fb-aim.ppm\n");
			}
			/*
			 * Hold the button in WALL-CLOCK time, not loop iterations: this
			 * loop spins far faster than 1ms, so a down at step 420 and an up
			 * at 1200 happened within the same millisecond and the shim, which
			 * only keeps the latest button state, coalesced them away -- no
			 * edge, no event.
			 */
			/*
			 * Easy Access "Mouse Keys": Cmd-Shift-Clear toggles it, then
			 * keypad 5 is a click.  It posts real mouse events through
			 * PostEvent -- the path the keyboard already proves works --
			 * so it sidesteps the dead CursorDeviceManager/VBL chain
			 * entirely.  Driven purely over the working keyboard.
			 */
			/* (Mouse Keys probe removed: Easy Access is not installed, and the
			 * Cmd-Shift combination raised a modal dialog that masked the
			 * click test.) */
			/* Validate the Down arrow: ADB 0x7d must select the next user in
			 * the login list.  The injector uses ADB codes directly, so this
			 * exercises the same ADB->HID path a real keypress now takes after
			 * the SDL->ADB table fix. */
			static uint64 downAt = 0;
			static int downState = 0;
			if (0 && !downState) {
				downState = 1; downAt = sys_get_hiresclk_ticks();
				usb_hid_key_event(0x7d, true);
				fprintf(stderr, "[ARROW] Down pressed (ADB 0x7d)\n");
			}
			if (downState == 1 &&
			    sys_get_hiresclk_ticks() - downAt > sys_get_hiresclk_ticks_per_second() / 8) {
				downState = 2;
				usb_hid_key_event(0x7d, false);
				fprintf(stderr, "[ARROW] Down released\n");
			}
			static uint64 pressedAt = 0;
			if (gKeyScript == 800 && !pressedAt) {
				pressedAt = sys_get_hiresclk_ticks();
				pearpc_inject_mouse(0, 0, 1);	/* real SDL path */
				fprintf(stderr, "[CLICK] button DOWN\n");
			}
			/*
			 * Dump the screen WHILE the button is held.  A click on the menu
			 * bar opens a menu that closes again on release, so every dump
			 * taken after the release showed "no change" -- destroying the
			 * evidence of where the click actually landed.  The user sees a
			 * menu open top-right; this is how to see the same thing.
			 */
			if (pressedAt && !gHeldShot &&
			    sys_get_hiresclk_ticks() - pressedAt >
			        sys_get_hiresclk_ticks_per_second() / 20) {
				gHeldShot = 1;
				gcard_dump_framebuffer();
				/* Keep it: the probe dumps to the same path later and would
				 * overwrite the one frame that shows where the click landed. */
				rename("/tmp/pearpc-fb.ppm", "/tmp/pearpc-fb-held.ppm");
				fprintf(stderr, "[HELDSHOT] captured with button down -> /tmp/pearpc-fb-held.ppm\n");
			}
			if (pressedAt) {
				uint64 per = sys_get_hiresclk_ticks_per_second();
				if (sys_get_hiresclk_ticks() - pressedAt > per / 8) {   /* 125ms: a realistic click.
			 * Holding for seconds keeps MBState reading button-down, and Mac OS
			 * then sits in a mouse-tracking state and stops processing
			 * keystrokes -- which is why the provocation could inject keys the
			 * driver never posted. */
					pressedAt = 0;
					pearpc_inject_mouse(0, 0, 0);	/* real SDL path */
					fprintf(stderr, "[CLICK] button UP (held 250ms)\n");
				} else {
					/* keep re-asserting so an apply cannot miss the down */
					cuda_shim_mouse(0, 0, true);
				}
			}

			if (gKeyScript == 1200) {
				gcard_dump_framebuffer();
				rename("/tmp/pearpc-fb.ppm", "/tmp/pearpc-fb-final.ppm");
				fprintf(stderr, "[FINAL] after click -> /tmp/pearpc-fb-final.ppm\n");
			}
			if (gKeyScript == 500) {
				/*
				 * Does the VBL cursor task run at all?  On Mac OS 9 input sets
				 * RawMouse and CrsrNew, and a VBL task then copies RawMouse
				 * into Mouse and redraws.  Set them ourselves: if the task is
				 * alive it clears CrsrNew and Mouse follows.  If CrsrNew stays
				 * 1 the task never runs, which would leave the pointer inert
				 * no matter which transport delivers input -- exactly what we
				 * see -- while the keyboard, posted directly at interrupt
				 * time, keeps working.
				 */
				uint8 one = 1;
				uint8 pt[4] = { 0, 200, 1, 44 };	/* v=200, h=300 */
				/* (RawMouse write disabled: the shim owns it now) */
				(void)one;
				fprintf(stderr, "[CRSR] set RawMouse=(200,300) CrsrNew=1\n");
			}
			if (gKeyScript == 650 || gKeyScript == 880) {
				uint8 cn[2] = {0,0}, rm[4] = {0,0,0,0}, mo[4] = {0,0,0,0};
				ppc_dma_read(cn, LOMEM_BASE + 0x8ce, 2);
				ppc_dma_read(rm, LOMEM_BASE + 0x82c, 4);
				ppc_dma_read(mo, LOMEM_BASE + 0x830, 4);
				fprintf(stderr, "[CRSR] t=%d CrsrNew=%02x Couple=%02x RawMouse=(%d,%d) Mouse=(%d,%d) %s\n",
					gKeyScript, cn[0], cn[1],
					(sint16)((rm[0]<<8)|rm[1]), (sint16)((rm[2]<<8)|rm[3]),
					(sint16)((mo[0]<<8)|mo[1]), (sint16)((mo[2]<<8)|mo[3]),
					cn[0] ? "<== CrsrNew NOT cleared: VBL cursor task not running"
					      : "(cleared: task ran)");
			}
			if (gKeyScript >= 1500 && !pressedAt) gKeyScript = -2;			/* done */
			gKeyScript++;
		}
		if (gDebugInjectMouseMotion) {
			gDebugInjectMouseMotion = 0;
			gPktVariant = (gPktVariant + 1) & 3;
			/*
			 * Isolation: drive ONLY the USB HID mouse, and only a button --
			 * no movement, and no ADB event alongside it.  If MBState still
			 * flips, the guest really is processing our HID reports and motion
			 * is specifically what it discards.  (The old loop fired 20
			 * iterations of two 5,3 events plus an ADB packet per signal,
			 * which both saturated the deltas and muddied the attribution.)
			 */
			/*
			 * Inject a plain movement, no button held.  Kept deliberately
			 * simple: the old form looped 20 times firing an ADB packet plus
			 * two 5,3 events per signal, which saturated every delta at 0x7f
			 * and made button attribution ambiguous between the ADB and USB
			 * paths.  One explicit report per signal is what a test can reason
			 * about.
			 */
			/* Distinctive deltas: dx=0x3c, dy=0x5a.  The trace below looks for
			 * the lbz that loads 0x5a, which identifies the code actually
			 * reading our report -- the module disassembled earlier provably
			 * never executes. */
			/* Motion is now driven by the timed script below, as a stream of
			 * small deltas -- that is what a real trackpad delivers, and one
			 * large jump is not a fair test of the guest's motion path. */
			/*
			 * Also press a key over USB HID.  A keyboard boot report carries
			 * its data past byte 0 -- the same part of a mouse report that is
			 * being discarded -- so if KeyMap reacts, the HID data path works
			 * and the loss is specific to mouse motion; if it does not, the
			 * problem is broader than the mouse.
			 */
			/*
			 * Press Return over USB HID.  The keyboard demonstrably works
			 * (KeyMap reacts), so this is a way past the Multiple Users login
			 * screen without a pointer -- every mouse test so far has run on
			 * that screen, which may handle the cursor differently from the
			 * Finder desktop.
			 */
			{
				/* Drive BOTH paths: USB HID and the ADB/CUDA packet interface.
				 * KeyTime (0x186) latches when a key event is actually posted,
				 * so this says whether either path reaches the Event Manager.
				 * ADB is only wired up when pci_usb_hid = 0 (the device tree
				 * withdraws its nodes otherwise). */
				/* Start a timed key script.  Press and release must NOT be
				 * issued back to back: usbhid_key_event keeps one current-state
				 * report behind a single reportPending flag, so an immediate
				 * release overwrites the press and the guest -- which polls the
				 * interrupt endpoint every ~8ms -- only ever sees an empty
				 * report.  The event loop below holds each key for ~120ms, which
				 * is what a real keystroke looks like. */
				gKeyScript = 0;
			}
			/* (ADB key injection removed: with the ADB nodes withdrawn it cannot
			 * reach the guest, and it made KeyMap changes ambiguous between the
			 * ADB and USB paths.) */
			{
				/* Report Mac OS's own input globals so a running instance can be
				 * probed repeatedly without rebooting: low memory sits at
				 * physical 0x4000, RawMouse at 0x82c, KeyMap at 0x174. */
				uint8 rm[4], km[4], mo[4], cn[2], mbs[1], mt[4], pin[8], cvis[1];
				ppc_dma_read(rm, 0x4000 + 0x82c, 4);
				ppc_dma_read(km, 0x4000 + 0x174, 4);
				ppc_dma_read(mo, 0x4000 + 0x830, 4);	/* Mouse: written by the ROM */
				ppc_dma_read(cn, 0x4000 + 0x8ce, 2);	/* CrsrNew / CrsrCouple */
				ppc_dma_read(mbs, 0x4000 + 0x172, 1);
				ppc_dma_read(mt, 0x4000 + 0x828, 4);	/* MTemp: raw, pre-filter */
				ppc_dma_read(pin, 0x4000 + 0x834, 8);	/* CrsrPin rect */
				ppc_dma_read(cvis, 0x4000 + 0x8cc, 1);	/* CrsrVis */
				{
					/*
					 * EventQueue (0x14a, a QHdr) tells the two cases apart:
					 * if HID input is posting events that nothing dispatches
					 * the queue fills, and if nothing is posted it stays empty.
					 * KeyMap and MBState update while the UI never reacts, so
					 * this is the discriminator.
					 */
					uint8 eq[10], em[2], kl[2], kt[4], tk[4];
					ppc_dma_read(eq, 0x4000 + 0x14a, 10);
					ppc_dma_read(em, 0x4000 + 0x144, 2);	/* SysEvtMask */
					/*
					 * An empty queue does not prove nothing was posted --
					 * GetNextEvent drains it, so a polling app keeps it empty.
					 * KeyLast/KeyTime latch when a key event IS posted, and
					 * Ticks gives a reference for whether KeyTime is recent.
					 */
					ppc_dma_read(kl, 0x4000 + 0x184, 2);	/* KeyLast */
					ppc_dma_read(kt, 0x4000 + 0x186, 4);	/* KeyTime */
					ppc_dma_read(tk, 0x4000 + 0x16a, 4);	/* Ticks   */
					{
						/*
						 * Does guest time advance?  If it does not, the
						 * menu-bar clock would redraw the same digits and the
						 * framebuffer checksum would be identical even on a
						 * perfectly healthy system -- so the liveness test
						 * needs this to be conclusive.  Time (0x20c) is
						 * seconds since 1904.
						 */
						uint8 tm[4];
						ppc_dma_read(tm, 0x4000 + 0x20c, 4);
						fprintf(stderr, "[TIME] Time=%02x%02x%02x%02x\n",
							tm[0],tm[1],tm[2],tm[3]);
					}
					fprintf(stderr, "[EVT2] KeyLast=%02x%02x KeyTime=%02x%02x%02x%02x Ticks=%02x%02x%02x%02x\n",
						kl[0],kl[1], kt[0],kt[1],kt[2],kt[3], tk[0],tk[1],tk[2],tk[3]);
					fprintf(stderr, "[EVTQ] flags=%02x%02x head=%02x%02x%02x%02x "
						"tail=%02x%02x%02x%02x SysEvtMask=%02x%02x\n",
						eq[0],eq[1], eq[2],eq[3],eq[4],eq[5],
						eq[6],eq[7],eq[8],eq[9], em[0],em[1]);
				}
				{
					/* DTQueue (0xd92): QHdr for the Deferred Task Manager.
					 * USBHIDDriver imports DeferUserFn, so if motion is handed
					 * to a deferred task that never runs, entries pile up here
					 * and qHead stays non-zero. */
					uint8 dtq[10];
					ppc_dma_read(dtq, 0x4000 + 0xd92, 10);
					fprintf(stderr, "[DTQ] flags=%02x%02x head=%02x%02x%02x%02x tail=%02x%02x%02x%02x\n",
						dtq[0],dtq[1],dtq[2],dtq[3],dtq[4],dtq[5],dtq[6],dtq[7],dtq[8],dtq[9]);
				}
				fprintf(stderr, "[MOUSE2] MTemp v=%d h=%d | CrsrPin t=%d l=%d b=%d r=%d | CrsrVis=%02x\n",
					(sint16)((mt[0]<<8)|mt[1]), (sint16)((mt[2]<<8)|mt[3]),
					(sint16)((pin[0]<<8)|pin[1]), (sint16)((pin[2]<<8)|pin[3]),
					(sint16)((pin[4]<<8)|pin[5]), (sint16)((pin[6]<<8)|pin[7]), cvis[0]);
				fprintf(stderr, "[MOUSE] RawMouse v=%d h=%d | Mouse v=%d h=%d | CrsrNew=%02x CrsrCouple=%02x MBState=%02x\n",
					(sint16)((rm[0]<<8)|rm[1]), (sint16)((rm[2]<<8)|rm[3]),
					(sint16)((mo[0]<<8)|mo[1]), (sint16)((mo[2]<<8)|mo[3]),
					cn[0], cn[1], mbs[0]);
				{
					/*
					 * VBLQueue (0x160) is a QHdr: flags(2), qHead(4), qTail(4).
					 * Empty means no VBL task was ever installed -- so no cursor
					 * task exists and the cursor device is what is missing.
					 * Non-empty means tasks are installed and merely starved of
					 * VBL interrupts, which is a fault on our side of the fence.
					 */
					uint8 q[10] = {0};
					ppc_dma_read(q, LOMEM_BASE + 0x160, 10);
					uint32 head = ((uint32)q[2]<<24)|((uint32)q[3]<<16)|((uint32)q[4]<<8)|q[5];
					uint32 tail = ((uint32)q[6]<<24)|((uint32)q[7]<<16)|((uint32)q[8]<<8)|q[9];
					if (head) {
						/*
						 * VBLQueue (0x160) is the SYSTEM VBL queue, serviced by
						 * the 60Hz tick -- not by the video card's VBL.  Ticks
						 * advances at 60.6Hz, so that handler runs; if these
						 * tasks are being serviced their vblCount decrements,
						 * and the video interrupt is irrelevant to the cursor.
						 * VBLTask: qLink(4) qType(2) vblAddr(4) vblCount(2).
						 */
						uint8 t[12] = {0};
						ppc_dma_read(t, head, 12);
						fprintf(stderr, "[VBLT] task@%08x vblAddr=%02x%02x%02x%02x vblCount=%02x%02x\n",
							head, t[6], t[7], t[8], t[9], t[10], t[11]);
					}
					fprintf(stderr, "[VBLQ] flags=%02x%02x qHead=%08x qTail=%08x -- %s\n",
						q[0], q[1], head, tail,
						head ? "tasks INSTALLED (starved of VBL)"
						     : "<== EMPTY: no VBL task ever installed");
				}
				scan_for_event_records("probe");
				{
					extern volatile uint32 gCursorDeviceRec;
					if (gCursorDeviceRec) {
						uint8 rec[64];
						if (ppc_dma_read(rec, gCursorDeviceRec, sizeof rec)) {
							fprintf(stderr, "[CDMREC] @%08x:", gCursorDeviceRec);
							for (int i = 0; i < 64; i++) {
								if ((i % 16) == 0) fprintf(stderr, "\n  +%02x:", i);
								fprintf(stderr, " %02x", rec[i]);
							}
							fprintf(stderr, "\n");
						}
					}
				}
				probe_event_queue();
				probe_video_driver_failure();
				pic_debug_print();
				probe_video_node_interrupts();
				usb_debug_print();
				gcard_debug_print();
				{
					/* Is the 60Hz tick source alive, and does Ticks follow it? */
					uint8 tk[4] = {0,0,0,0}, tm[4] = {0,0,0,0}, kl[6] = {0,0,0,0,0,0};
					ppc_dma_read(tk, LOMEM_BASE + 0x16a, 4);
					ppc_dma_read(tm, LOMEM_BASE + 0x20c, 4);
					/* KeyLast (0x184) / KeyTime (0x186) change only when a key
					 * event is actually posted to the Event Manager. */
					ppc_dma_read(kl, LOMEM_BASE + 0x184, 6);
					fprintf(stderr, "[TICK] Ticks=%02x%02x%02x%02x Time=%02x%02x%02x%02x "
					        "KeyLast=%02x%02x KeyTime=%02x%02x%02x%02x "
					        "T1run=%d rIER=%02x rIFR=%02x T1raises=%lu cudaIRQ=%lu\n",
					        tk[0],tk[1],tk[2],tk[3], tm[0],tm[1],tm[2],tm[3],
					        kl[0],kl[1], kl[2],kl[3],kl[4],kl[5],
					        gCUDA.T1_running ? 1 : 0, gCUDA.rIER, gCUDA.rIFR,
					        gT1Raises, gCudaIrqAsserts);
				}
				fprintf(stderr, "[PROBE] variant=%d RawMouse v=%d h=%d KeyMap=%02x%02x%02x%02x | "
				        "queued=%d delivered=%d gpioReads=%d intBits=%02x mask=%02x asserted=%d\n",
				        gPktVariant, (rm[0] << 8) | rm[1], (rm[2] << 8) | rm[3],
				        km[0], km[1], km[2], km[3],
				        gAdbQueued, gAdbDelivered, gExtIntReads, gCUDA.pmuInterruptBits,
				        gCUDA.pmuInterruptMask, cuda_pmu_extint_asserted() ? 1 : 0);
			}
		}
		if (gDebugInjectMouseClick) {
			gDebugInjectMouseClick = 0;
			SystemEvent ev = {};
			ev.type = sysevMouse;
			ev.mouse.type = sme_buttonPressed;
			ev.mouse.button1 = true;
			tryProcessCudaEvent(ev);
			ev.mouse.type = sme_buttonReleased;
			ev.mouse.button1 = false;
			tryProcessCudaEvent(ev);
		}
//		IO_CUDA_WARN("semaphore signalled\n");
		SystemEventObject *seo;
		while ((seo = (SystemEventObject*)gCUDAEvents.deQueue())) {
			tryProcessCudaEvent(seo->mEv);
			delete seo;
		}
	}
	return NULL;
}

bool cuda_prom_get_key(uint32 &key)
{
	if (gCUDA.left == 5 && gCUDA.data[2] == 0x2c) {
		key = gCUDA.data[3];
		gCUDA.left = 0;
		return true;
	} else {
		gCUDA.left = 0;
		return false;
	}
}


void cuda_pre_init()
{
	if (sys_create_semaphore(&gCUDAEventSem)) {
		IO_CUDA_ERR("Can't create semaphore\n");
	}
	sys_lock_semaphore(gCUDAEventSem);
	sys_unlock_semaphore(gCUDAEventSem);	
}

void cuda_init()
{
	memset(&gCUDA, 0, sizeof gCUDA);
	gCUDA.pmuMode = gConfig->getConfigInt(CUDA_KEY_PMU) != 0;
	gCUDA.state = cuda_idle;
	gCUDA.pmuState = pmu_idle;
	gCUDA.pmuInterruptMask = PMU_INT_ADB | PMU_INT_TICK;
	gCUDA.oldTIP = true;
	gCUDA.oldTACK = true;
	if (gCUDA.pmuMode) {
        gCUDA.rB = PMU_TACK | PMU_TREQ;
        gCUDA.rORB = PMU_TREQ;
    }
	gCUDA.keybaddr = ADB_KEYBOARD;
	gCUDA.keybhandler = 1;
	gCUDA.mouseaddr = ADB_MOUSE;
	gCUDA.mousehandler = 2;
	gCUDA.T1_end = 0;
	gCUDA.T1_running = false;
	gCUDA.T2_end = 0;
	gCUDA.T2_running = false;
	gCUDA.T2_probe_pending = false;
	gCUDA.T2_probe_completed = false;
	gCUDA.T2_probe_attempts = 0;
	gCUDA.SR_end = 0;
	gCUDA.SR_pending = false;
	gCUDA.SR_transfer_armed = false;
	gCUDA.response_irq_armed = false;
	gCUDA.IRQ_asserted = false;
	gCUDA.rT1LL = 0xff;
	gCUDA.rT1LH = 0xff;

	if (sys_create_mutex(&gCUDAMutex)) {
		IO_CUDA_ERR("Can't create mutex\n");
	}

	if (sys_create_semaphore(&gCUDA.idle_sem)) {
		IO_CUDA_ERR("Can't create semaphore\n");
	}

	sys_thread cudaEventLoopThread;
	sys_create_thread(&cudaEventLoopThread, 0, cudaEventLoop, NULL);
}

void cuda_done()
{
	sys_destroy_mutex(gCUDAMutex);
	sys_destroy_semaphore(gCUDA.idle_sem);
}

void cuda_init_config()
{
	gConfig->acceptConfigEntryIntDef(CUDA_KEY_PMU, 0);
}

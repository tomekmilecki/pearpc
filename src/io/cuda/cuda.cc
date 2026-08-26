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
#include "system/keyboard.h"
#include "system/mouse.h"
#include "system/sys.h"
#include "system/sysclk.h"
#include "system/systhread.h"
#include "configparser.h"

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

static void cuda_renew_interrupt()
{
	if (gCUDA.rIFR & gCUDA.rIER & (SR_INT | T1_INT | T2_INT)) {
		gCUDA.rIFR |= 0x80;
		if (!gCUDA.IRQ_asserted) {
			gCUDA.IRQ_asserted = true;
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

void cuda_read(uint32 addr, uint32 &data, int size)
{
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

static bool cudaEventHandler(const SystemEvent &ev)
{
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
            gCUDA.pmuAdbReply[0] = 0x2c;
            gCUDA.pmuAdbReply[1] = key;
            gCUDA.pmuAdbReply[2] = 0xff;
            gCUDA.pmuAdbReplyLength = 3;
        } else if (ev.type == sysevMouse) {
            int dx = MAX(-63, MIN(63, ev.mouse.relx));
            int dy = MAX(-63, MIN(63, ev.mouse.rely));
            dx &= 0x7f;
            dy &= 0x7f;
            if (!ev.mouse.button2) dx |= 0x80;
            if (!ev.mouse.button1) dy |= 0x80;
            gCUDA.pmuAdbReply[0] = 0x3c;
            gCUDA.pmuAdbReply[1] = dy;
            gCUDA.pmuAdbReply[2] = dx;
            gCUDA.pmuAdbReplyLength = 3;
        } else {
            return false;
        }
        gCUDA.pmuInterruptBits |= PMU_INT_ADB | PMU_INT_ADB_AUTO;
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
		if (gDebugInjectMouseMotion) {
			gDebugInjectMouseMotion = 0;
			for (int i = 0; i < 20; i++) {
				SystemEvent ev = {};
				ev.type = sysevMouse;
				ev.mouse.type = sme_motionNotify;
				ev.mouse.relx = 5;
				ev.mouse.rely = 3;
				tryProcessCudaEvent(ev);
			}
			fprintf(stderr, "[INJECT] queued 20 motion events (+5,+3 each)\n");
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

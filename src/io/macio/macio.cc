/*
 *	PearPC
 *	pci.cc
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

#include "debug/tracers.h"
#include "cpu/mem.h"
#include "io/cuda/cuda.h"
#include "io/pic/pic.h"
#include "system/sysclk.h"
#include "macio.h"

#define MACIO_DBDMA_ADDRESS_CONTROL	0x00
#define MACIO_DBDMA_ADDRESS_STATUS	0x04
#define MACIO_DBDMA_ADDRESS_CMD_PTR_HI	0x08
#define MACIO_DBDMA_ADDRESS_CMD_PTR_LO	0x0c
#define MACIO_DBDMA_ADDRESS_INTR_SEL	0x10
#define MACIO_DBDMA_ADDRESS_BRANCH_SEL	0x14
#define MACIO_DBDMA_ADDRESS_WAIT_SEL	0x18
#define MACIO_DBDMA_ADDRESS_MODES	0x1c
#define MACIO_DBDMA_ADDRESS_DATA_PTR_HI	0x20
#define MACIO_DBDMA_ADDRESS_DATA_PTR_LO	0x24
#define MACIO_DBDMA_ADDRESS_ADDRESS_HI	0x2c

#define MACIO_DBDMA_BASE 0x8000
#define MACIO_DBDMA_CHANNEL_SIZE 0x100
#define MACIO_DBDMA_CHANNEL_COUNT 16

/*
 * Paddington/Heathrow exposes a small bank of board-control registers at
 * 0x17e00.  The Mac OS nanokernel enables its I/O paths through these byte
 * registers before the individual devices have been probed.  They do not
 * require device-side effects in the emulated machine, but must acknowledge
 * accesses instead of turning an otherwise harmless feature-bit write into a
 * fatal PCI error.
 */
#define MACIO_BOARD_CONTROL_BASE 0x17e00
#define MACIO_BOARD_CONTROL_SIZE 0x100

#define MACIO_GPIO_BASE 0x50
#define MACIO_GPIO_SIZE 0x30
#define MACIO_PMU_EXTINT_GPIO 0x59

/*
 * KeyLargo's global timer is a free-running 64-bit counter clocked at
 * 18.432 MHz.  Reading the low word latches the high word so software can
 * obtain a coherent value across a low-word rollover.
 */
#define MACIO_TIMER_FREQUENCY 18432000ULL
#define MACIO_WATCHDOG_LOW 0x15030
#define MACIO_WATCHDOG_HIGH 0x15034
#define MACIO_TIMER_LOW 0x15038
#define MACIO_TIMER_HIGH 0x1503c
#define MACIO_WATCHDOG_ENABLE 0x15048

/*
 * KeyLargo contains a Z85C30 ESCC.  The current interface uses 16-byte
 * spacing, while the CHRP legacy interface aliases the same four ports into
 * the first eight bytes at 0x12000.
 */
#define MACIO_ESCC_LEGACY_BASE 0x12000
#define MACIO_ESCC_LEGACY_SIZE 0x1000
#define MACIO_ESCC_BASE 0x13000
#define MACIO_ESCC_SIZE 0x1000

#define ESCC_CONTROL 0
#define ESCC_DATA 1

#define ESCC_WR_COMMAND 0
#define ESCC_WR_RX_CONTROL 3
#define ESCC_WR_TX_CONTROL_1 4
#define ESCC_WR_TX_CONTROL_2 5
#define ESCC_WR_MASTER_INTERRUPT 9
#define ESCC_WR_BAUD_LOW 12
#define ESCC_WR_BAUD_HIGH 13

#define ESCC_COMMAND_POINTER_MASK 0x07
#define ESCC_COMMAND_MASK 0x38
#define ESCC_COMMAND_HIGH_REGISTERS 0x08

#define ESCC_MASTER_RESET_MASK 0xc0
#define ESCC_MASTER_RESET_B 0x40
#define ESCC_MASTER_RESET_A 0x80
#define ESCC_MASTER_RESET_ALL 0xc0

#define ESCC_STATUS_RX_AVAILABLE 0x01
#define ESCC_STATUS_TX_EMPTY 0x04
#define ESCC_STATUS_SYNC 0x10
#define ESCC_STATUS_TX_UNDERRUN 0x40
#define ESCC_STATUS_BREAK_ABORT 0x80

/* OpenPIC sources for the two ESCC channels, matching the device tree
 * (ch-b@13000 -> 0x24, ch-a@13020 -> 0x25).  Channel index 0 is B, 1 is A. */
#define ESCC_IRQ_CHANNEL_B 0x24
#define ESCC_IRQ_CHANNEL_A 0x25

/* Cap on idle-line abort interrupts, so a driver that re-arms in a loop cannot
 * turn this into an interrupt storm. */
#define ESCC_MAX_IDLE_ABORTS 32
#define ESCC_SPECIAL_ALL_SENT 0x01
#define ESCC_SPECIAL_BITS_8 0x06
	
/*
 *	Channel control and status flags
 */
#define MACIO_DBDMA_RUN		0x8000
#define MACIO_DBDMA_PAUSE	0x4000
#define MACIO_DBDMA_FLUSH	0x2000
#define MACIO_DBDMA_WAKE	0x1000
#define MACIO_DBDMA_DEAD	0x800
#define MACIO_DBDMA_ACTIVE	0x400
#define MACIO_DBDMA_BT		0x100
#define MACIO_DBDMA_S7		0x80
#define MACIO_DBDMA_S6		0x40
#define MACIO_DBDMA_S5		0x20
#define MACIO_DBDMA_S4		0x10
#define MACIO_DBDMA_S3		0x8
#define MACIO_DBDMA_S2		0x4
#define MACIO_DBDMA_S1		0x2
#define MACIO_DBDMA_S0		0x1

/*
 *	commands
 */

#define MACIO_DBDMA_CMD_OUTPUT_MORE	0
#define MACIO_DBDMA_CMD_OUTPUT_LAST	1
#define MACIO_DBDMA_CMD_INPUT_MORE	2
#define MACIO_DBDMA_CMD_INPUT_LAST	3
#define MACIO_DBDMA_CMD_STORE_QUAD	4
#define MACIO_DBDMA_CMD_LOAD_QUAD	5
#define MACIO_DBDMA_CMD_NOP		6
#define MACIO_DBDMA_CMD_STOP		7

#define MACIO_DBDMA_COMMAND_MASK 0xf000
#define MACIO_DBDMA_KEY_MASK 0x0700
#define MACIO_DBDMA_INTERRUPT_MASK 0x0030
#define MACIO_DBDMA_BRANCH_MASK 0x000c
#define MACIO_DBDMA_WAIT_MASK 0x0003

/*
 *	keys
 */

#define  MACIO_DBDMA_KEY_STREAM0	0
#define  MACIO_DBDMA_KEY_STREAM1	1
#define  MACIO_DBDMA_KEY_STREAM2	2
#define  MACIO_DBDMA_KEY_STREAM3	3
#define  MACIO_DBDMA_KEY_REGS		5
#define  MACIO_DBDMA_KEY_SYSTEM		6
#define  MACIO_DBDMA_KEY_DEVICE		7

#define  MACIO_DBDMA_INT_NEVER		0
#define  MACIO_DBDMA_INT_IF_TRUE	1
#define  MACIO_DBDMA_INT_IF_FALSE	2
#define  MACIO_DBDMA_INT_ALWAYS		3

#define  MACIO_DBDMA_BRANCH_NEVER	0
#define  MACIO_DBDMA_BRANCH_IF_TRUE	1
#define  MACIO_DBDMA_BRANCH_IF_FALSE	2
#define  MACIO_DBDMA_BRANCH_ALWAYS	3

#define  MACIO_DBDMA_WAIT_NEVER		0
#define  MACIO_DBDMA_WAIT_IF_TRUE	1
#define  MACIO_DBDMA_WAIT_IF_FALSE	2
#define  MACIO_DBDMA_WAIT_ALWAYS	3

PCI_MacIO::PCI_MacIO()
	:PCI_Device("pci-macio", 0x01, 0x05),
    mTimerStartTicks(sys_get_hiresclk_ticks()),
    mTimerHighLatch(0),
    mWatchDogLow(0),
    mWatchDogHigh(0),
    mWatchDogEnable(0)
{
	mIORegSize[0] = 0x80000;
	mIORegType[0] = PCI_ADDRESS_SPACE_MEM;

	mConfig[0x00] = 0x6b;	// vendor ID
	mConfig[0x01] = 0x10;
	mConfig[0x02] = 0x22;	// KeyLargo unit ID
	mConfig[0x03] = 0x00;

	mConfig[0x08] = 0x03;	// revision
	mConfig[0x09] = 0x00; 	// programming interface code
	mConfig[0x0a] = 0x00;	// pci2pci
	mConfig[0x0b] = 0xff;	// bridge

	mConfig[0x0e] = 0x00;	// header-type

	assignMemAddress(0, 0x80800000);

	mConfig[0x3c] = 0x18;
	mConfig[0x3d] = 1;
	mConfig[0x3e] = 0;
	mConfig[0x3f] = 0;	

    for (uint channel = 0; channel < 2; ++channel) {
        ESCCChannel &escc = mESCCChannels[channel];
        escc.selectedRegister = 0;
        escc.receiveData = 0;
        escc.transmitData = 0;
        for (uint reg = 0; reg < 16; ++reg) {
            escc.readRegisters[reg] = 0;
            escc.writeRegisters[reg] = 0;
        }
        /* TX empty is the only guaranteed status bit after power-on. */
        escc.readRegisters[0] = ESCC_STATUS_TX_EMPTY;
    }

    for (uint channel = 0; channel < MACIO_DBDMA_CHANNEL_COUNT; ++channel) {
        for (uint reg = 0; reg < 12; ++reg) {
            mDBDMAChannels[channel].registers[reg] = 0;
        }
        mDBDMAChannels[channel].waitingForInput = false;
    }
}

static uint16 readLittleEndian16(const byte *data)
{
    return data[0] | (static_cast<uint16>(data[1]) << 8);
}

static uint32 readLittleEndian32(const byte *data)
{
    return data[0] | (static_cast<uint32>(data[1]) << 8) |
        (static_cast<uint32>(data[2]) << 16) | (static_cast<uint32>(data[3]) << 24);
}

static void writeLittleEndian16(byte *data, uint16 value)
{
    data[0] = value;
    data[1] = value >> 8;
}

static void writeLittleEndian32(byte *data, uint32 value)
{
    data[0] = value;
    data[1] = value >> 8;
    data[2] = value >> 16;
    data[3] = value >> 24;
}

bool PCI_MacIO::readDBDMACommand(uint32 address, DBDMACommand &command) const
{
    byte data[16];
    if (!ppc_dma_read(data, address, sizeof data)) return false;

    command.requestedCount = readLittleEndian16(data);
    command.command = readLittleEndian16(data + 2);
    command.physicalAddress = readLittleEndian32(data + 4);
    command.commandDependent = readLittleEndian32(data + 8);
    command.residualCount = readLittleEndian16(data + 12);
    command.transferStatus = readLittleEndian16(data + 14);
    return true;
}

bool PCI_MacIO::writeDBDMACommand(uint32 address, const DBDMACommand &command) const
{
    byte data[16];
    writeLittleEndian16(data, command.requestedCount);
    writeLittleEndian16(data + 2, command.command);
    writeLittleEndian32(data + 4, command.physicalAddress);
    writeLittleEndian32(data + 8, command.commandDependent);
    writeLittleEndian16(data + 12, command.residualCount);
    writeLittleEndian16(data + 14, command.transferStatus);
    return ppc_dma_write(address, data, sizeof data);
}

void PCI_MacIO::runDBDMAChannel(uint channel, bool flushInput)
{
    DBDMAChannel &dbdma = mDBDMAChannels[channel];
    uint32 &status = dbdma.registers[MACIO_DBDMA_ADDRESS_STATUS >> 2];
    uint32 &commandPointer = dbdma.registers[MACIO_DBDMA_ADDRESS_CMD_PTR_LO >> 2];

    auto selectionMatches = [&](uint registerAddress) {
        uint32 selection = dbdma.registers[registerAddress >> 2];
        uint32 mask = (selection >> 16) & 0x0f;
        return (status & mask) == ((selection & 0x0f) & mask);
    };

    auto shouldWait = [&](uint16 command) {
        switch (command & MACIO_DBDMA_WAIT_MASK) {
        case MACIO_DBDMA_WAIT_NEVER:
            return false;
        case MACIO_DBDMA_WAIT_ALWAYS:
            return true;
        case MACIO_DBDMA_WAIT_IF_TRUE:
            return selectionMatches(MACIO_DBDMA_ADDRESS_WAIT_SEL);
        case MACIO_DBDMA_WAIT_IF_FALSE:
            return !selectionMatches(MACIO_DBDMA_ADDRESS_WAIT_SEL);
        }
        return false;
    };

    auto raiseInterrupt = [&](uint16 command) {
        bool interrupt = false;
        switch ((command & MACIO_DBDMA_INTERRUPT_MASK) >> 4) {
        case MACIO_DBDMA_INT_IF_TRUE:
            interrupt = selectionMatches(MACIO_DBDMA_ADDRESS_INTR_SEL);
            break;
        case MACIO_DBDMA_INT_IF_FALSE:
            interrupt = !selectionMatches(MACIO_DBDMA_ADDRESS_INTR_SEL);
            break;
        case MACIO_DBDMA_INT_ALWAYS:
            interrupt = true;
            break;
        }
        if (interrupt) pic_raise_interrupt(channel);
    };

    auto advanceCommand = [&](const DBDMACommand &command) {
        bool branch = false;
        switch ((command.command & MACIO_DBDMA_BRANCH_MASK) >> 2) {
        case MACIO_DBDMA_BRANCH_IF_TRUE:
            branch = selectionMatches(MACIO_DBDMA_ADDRESS_BRANCH_SEL);
            break;
        case MACIO_DBDMA_BRANCH_IF_FALSE:
            branch = !selectionMatches(MACIO_DBDMA_ADDRESS_BRANCH_SEL);
            break;
        case MACIO_DBDMA_BRANCH_ALWAYS:
            branch = true;
            break;
        }
        if (branch) {
            commandPointer = command.commandDependent & ~0x0f;
            status |= MACIO_DBDMA_BT;
        } else {
            commandPointer += 16;
            status &= ~MACIO_DBDMA_BT;
        }
    };

    /* A malformed self-branch must not monopolize the CPU thread. */
    for (uint commandCount = 0; commandCount < 256; ++commandCount) {
        DBDMACommand command;
        if (!commandPointer || !readDBDMACommand(commandPointer, command)) {
            status &= ~(MACIO_DBDMA_ACTIVE | MACIO_DBDMA_FLUSH);
            status |= MACIO_DBDMA_DEAD;
            dbdma.waitingForInput = false;
            return;
        }

        status &= ~MACIO_DBDMA_WAKE;
        uint16 operation = (command.command & MACIO_DBDMA_COMMAND_MASK) >> 12;
        uint16 key = (command.command & MACIO_DBDMA_KEY_MASK) >> 8;

        if (operation == MACIO_DBDMA_CMD_STOP) {
            /*
             * BT reports that the last command executed took a branch.  Once the
             * channel has halted on a STOP it is idle, so the indication must not
             * persist: drivers poll the low status bits (which include BT) to wait
             * for the channel to quiesce, and a sticky BT strands them forever.
             */
            status &= ~(MACIO_DBDMA_ACTIVE | MACIO_DBDMA_DEAD | MACIO_DBDMA_FLUSH | MACIO_DBDMA_BT);
            dbdma.waitingForInput = false;
            return;
        }

        if (shouldWait(command.command)) return;

        if (operation == MACIO_DBDMA_CMD_INPUT_MORE || operation == MACIO_DBDMA_CMD_INPUT_LAST) {
            dbdma.waitingForInput = true;
            if (!flushInput) {
                /*
                 * The channel has loaded this command and is now waiting for the
                 * device to supply data.  Real hardware stamps the descriptor it
                 * is working on with the current channel status (nothing received
                 * yet, so the whole request is still residual); leaving it
                 * untouched makes it read back as all-zero, and drivers that poll
                 * the descriptor to see whether the channel picked it up spin
                 * forever.
                 */
                status &= ~(MACIO_DBDMA_ACTIVE | MACIO_DBDMA_FLUSH);
                command.residualCount = command.requestedCount;
                /*
                 * Drivers poll this word two different ways: one loop spins while
                 * (xferStatus & 0x2400) == 0x0400 (wants ACTIVE clear), the other
                 * while ((xferStatus<<16 | resCount) & 0x07ffffff) == 0 (wants some
                 * low bit set).  A descriptor with resCount == 0 satisfies neither
                 * unless a low status bit is set, so report BT as well.
                 */
                command.transferStatus = status | MACIO_DBDMA_BT;
                writeDBDMACommand(commandPointer, command);
                return;
            }

            /* No serial byte is pending, so flushing completes with all bytes residual. */
            command.residualCount = command.requestedCount;
            command.transferStatus = status;
            if (!writeDBDMACommand(commandPointer, command)) {
                status |= MACIO_DBDMA_DEAD;
                status &= ~MACIO_DBDMA_ACTIVE;
                return;
            }
            dbdma.waitingForInput = false;
            /* FLUSH applies only to the transfer that was active when requested. */
            flushInput = false;
            status &= ~MACIO_DBDMA_FLUSH;
            raiseInterrupt(command.command);
            advanceCommand(command);
            continue;
        }

        if (operation == MACIO_DBDMA_CMD_OUTPUT_MORE || operation == MACIO_DBDMA_CMD_OUTPUT_LAST) {
            /* The SCC transmit FIFO accepts the complete buffer immediately. */
            command.residualCount = 0;
            command.transferStatus = status;
            if (!writeDBDMACommand(commandPointer, command)) {
                status |= MACIO_DBDMA_DEAD;
                status &= ~MACIO_DBDMA_ACTIVE;
                return;
            }
            if (operation == MACIO_DBDMA_CMD_OUTPUT_LAST) status &= ~MACIO_DBDMA_FLUSH;
            raiseInterrupt(command.command);
            advanceCommand(command);
            continue;
        }

        if (operation == MACIO_DBDMA_CMD_STORE_QUAD || operation == MACIO_DBDMA_CMD_LOAD_QUAD) {
            if (key != MACIO_DBDMA_KEY_SYSTEM) {
                status |= MACIO_DBDMA_DEAD;
                status &= ~MACIO_DBDMA_ACTIVE;
                return;
            }

            uint transferSize = command.requestedCount & 7;
            if (transferSize & 4) {
                transferSize = 4;
                command.physicalAddress &= ~3U;
            } else if (transferSize & 2) {
                transferSize = 2;
                command.physicalAddress &= ~1U;
            } else {
                transferSize = 1;
            }

            byte value[4] = {0, 0, 0, 0};
            if (operation == MACIO_DBDMA_CMD_STORE_QUAD) {
                uint32 storedValue = command.commandDependent;
                if (transferSize == 2) storedValue >>= 16;
                if (transferSize == 1) storedValue >>= 24;
                writeLittleEndian32(value, storedValue);
                if (!ppc_dma_write(command.physicalAddress, value, transferSize)) {
                    status |= MACIO_DBDMA_DEAD;
                    status &= ~MACIO_DBDMA_ACTIVE;
                    return;
                }
            } else {
                if (!ppc_dma_read(value, command.physicalAddress, transferSize)) {
                    status |= MACIO_DBDMA_DEAD;
                    status &= ~MACIO_DBDMA_ACTIVE;
                    return;
                }
                uint32 loadedValue = readLittleEndian32(value);
                if (transferSize == 2) {
                    command.commandDependent = (loadedValue << 16) | (command.commandDependent & 0xffff);
                } else if (transferSize == 1) {
                    command.commandDependent = (loadedValue << 24) | (command.commandDependent & 0xffffff);
                } else {
                    command.commandDependent = loadedValue;
                }
            }

            command.transferStatus = status;
            if (!writeDBDMACommand(commandPointer, command)) {
                status |= MACIO_DBDMA_DEAD;
                status &= ~MACIO_DBDMA_ACTIVE;
                return;
            }
            status &= ~MACIO_DBDMA_FLUSH;
            raiseInterrupt(command.command);
            commandPointer += 16;
            status &= ~MACIO_DBDMA_BT;
            continue;
        }

        if (operation == MACIO_DBDMA_CMD_NOP) {
            command.transferStatus = status;
            if (!writeDBDMACommand(commandPointer, command)) {
                status |= MACIO_DBDMA_DEAD;
                status &= ~MACIO_DBDMA_ACTIVE;
                return;
            }
            raiseInterrupt(command.command);
            advanceCommand(command);
            continue;
        }

        status |= MACIO_DBDMA_DEAD;
        status &= ~MACIO_DBDMA_ACTIVE;
        return;
    }

    status |= MACIO_DBDMA_DEAD;
    status &= ~MACIO_DBDMA_ACTIVE;
}

bool PCI_MacIO::decodeESCCAddress(uint32 address, uint &channel, bool &dataRegister) const
{
    uint32 esccAddress;

    if (address >= MACIO_ESCC_BASE && address < MACIO_ESCC_BASE + MACIO_ESCC_SIZE) {
        esccAddress = address - MACIO_ESCC_BASE;
    } else if (address >= MACIO_ESCC_LEGACY_BASE &&
               address < MACIO_ESCC_LEGACY_BASE + MACIO_ESCC_LEGACY_SIZE) {
        static const uint8 legacyMap[] = {
            0x00, 0x20, 0x10, 0x30, 0x40, 0x50
        };
        uint32 legacyAddress = address - MACIO_ESCC_LEGACY_BASE;
        uint32 legacyPort = legacyAddress >> 1;
        if (legacyPort < sizeof legacyMap) {
            esccAddress = legacyMap[legacyPort];
        } else {
            /* Recovery/start/detect ports retain their original aliases. */
            esccAddress = legacyAddress;
        }
    } else {
        return false;
    }

    dataRegister = ((esccAddress >> 4) & 1) == ESCC_DATA;
    channel = (esccAddress >> 5) & 1;
    return true;
}

void PCI_MacIO::resetESCCChannel(uint channel, bool hardReset)
{
    ESCCChannel &escc = mESCCChannels[channel];

    escc.selectedRegister = 0;
    escc.receiveData = 0;
    escc.transmitData = 0;
    escc.writeRegisters[ESCC_WR_COMMAND] = 0;
    escc.writeRegisters[1] &= 0x24;
    escc.writeRegisters[ESCC_WR_RX_CONTROL] &= ~0x01;
    escc.writeRegisters[ESCC_WR_TX_CONTROL_1] |= 0x04;
    escc.writeRegisters[ESCC_WR_TX_CONTROL_2] &= 0x61;
    escc.writeRegisters[ESCC_WR_MASTER_INTERRUPT] &= ~0x20;
    escc.writeRegisters[10] &= 0x60;
    escc.writeRegisters[14] &= 0xc3;
    escc.writeRegisters[14] |= 0x20;
    escc.writeRegisters[15] = 0xf8;

    escc.readRegisters[0] &= 0xb8;
    escc.readRegisters[0] |= ESCC_STATUS_TX_EMPTY | ESCC_STATUS_TX_UNDERRUN;
    escc.readRegisters[1] &= ESCC_SPECIAL_ALL_SENT;
    escc.readRegisters[1] |= ESCC_SPECIAL_BITS_8;
    escc.readRegisters[3] = 0;
    escc.readRegisters[10] &= 0x40;

    if (hardReset) {
        escc.writeRegisters[ESCC_WR_MASTER_INTERRUPT] &= 0x03;
        escc.writeRegisters[ESCC_WR_MASTER_INTERRUPT] |= ESCC_MASTER_RESET_ALL;
        escc.writeRegisters[10] = 0;
        escc.writeRegisters[11] = 0x08;
        escc.writeRegisters[14] &= 0xc0;
        escc.writeRegisters[14] |= 0x30;
    }
}

uint8 PCI_MacIO::readESCCControl(uint channel)
{
    ESCCChannel &escc = mESCCChannels[channel];
    uint8 value = escc.readRegisters[escc.selectedRegister];
    escc.selectedRegister = 0;
    return value;
}

void PCI_MacIO::writeESCCControl(uint channel, uint8 value)
{
    ESCCChannel &escc = mESCCChannels[channel];
    uint selectedRegister = escc.selectedRegister;

    if (selectedRegister == ESCC_WR_COMMAND) {
        /*
         * WR0 D5-D3 carry a command.  "Reset External/Status Interrupts" (2) and
         * "Reset Highest IUS" (7) are how the guest acknowledges the idle-line
         * abort raised below; the OpenPIC source is level-triggered, so it has to
         * be deasserted here or the handler is re-entered forever.
         */
        uint8 command = value & 0x38;
        if (command == 0x10 || command == 0x38) {
            escc.readRegisters[0] &= ~ESCC_STATUS_BREAK_ABORT;
            pic_cancel_interrupt(channel ? ESCC_IRQ_CHANNEL_A : ESCC_IRQ_CHANNEL_B);
        }
        uint8 newRegister = value & ESCC_COMMAND_POINTER_MASK;
        if ((value & ESCC_COMMAND_MASK) == ESCC_COMMAND_HIGH_REGISTERS) {
            newRegister |= ESCC_COMMAND_HIGH_REGISTERS;
        }
        escc.selectedRegister = newRegister;
        return;
    }

    if (selectedRegister == ESCC_WR_MASTER_INTERRUPT) {
        switch (value & ESCC_MASTER_RESET_MASK) {
        case ESCC_MASTER_RESET_B:
            resetESCCChannel(0, false);
            return;
        case ESCC_MASTER_RESET_A:
            resetESCCChannel(1, false);
            return;
        case ESCC_MASTER_RESET_ALL:
            resetESCCChannel(0, true);
            resetESCCChannel(1, true);
            return;
        default:
            break;
        }
    }

    escc.writeRegisters[selectedRegister] = value;

    /*
     * Nothing is attached to the Cube's ESCC, so the receive line sits idle at
     * continuous marks.  A Z8530 in SDLC/HDLC mode reports that as an Abort and
     * raises an external/status interrupt (RR0 D7).  Without it a LocalTalk
     * driver that has enabled those interrupts waits forever for an event that
     * can never arrive.  Deliver it once the guest has actually armed:
     *   WR9 D1 = MIE, WR1 D0 = Ext Int Enable, WR15 D7 = Break/Abort IE.
     */
    if ((escc.writeRegisters[ESCC_WR_MASTER_INTERRUPT] & 0x02) &&
        (escc.writeRegisters[1] & 0x01) && (escc.writeRegisters[15] & 0x80) &&
        escc.abortsRaised < ESCC_MAX_IDLE_ABORTS) {
        escc.abortsRaised++;
        escc.readRegisters[0] |= ESCC_STATUS_BREAK_ABORT;
        pic_raise_interrupt(channel ? ESCC_IRQ_CHANNEL_A : ESCC_IRQ_CHANNEL_B);
    }
    if (selectedRegister == ESCC_WR_RX_CONTROL && (value & 0x10)) {
        escc.readRegisters[0] |= ESCC_STATUS_SYNC;
    } else if (selectedRegister == ESCC_WR_TX_CONTROL_1) {
        escc.readRegisters[1] |= ESCC_SPECIAL_ALL_SENT;
    } else if (selectedRegister == ESCC_WR_BAUD_LOW || selectedRegister == ESCC_WR_BAUD_HIGH) {
        escc.readRegisters[selectedRegister] = value;
    }
    escc.selectedRegister = 0;
}

uint64 PCI_MacIO::getTimerCounter() const
{
    uint64 elapsed = sys_get_hiresclk_ticks() - mTimerStartTicks;
    uint64 ticksPerSecond = sys_get_hiresclk_ticks_per_second();

    return (elapsed / ticksPerSecond) * MACIO_TIMER_FREQUENCY
        + ((elapsed % ticksPerSecond) * MACIO_TIMER_FREQUENCY) / ticksPerSecond;
}

bool PCI_MacIO::readDeviceMem(uint r, uint32 address, uint32 &data, uint size)
{
    uint esccChannel;
    bool esccDataRegister;
    if (r == 0 && size == 1 && decodeESCCAddress(address, esccChannel, esccDataRegister)) {
        ESCCChannel &escc = mESCCChannels[esccChannel];
        if (esccDataRegister) {
            data = escc.receiveData;
            escc.readRegisters[0] &= ~ESCC_STATUS_RX_AVAILABLE;
        } else {
            data = readESCCControl(esccChannel);
        }
        return true;
    }

    if (r == 0 && size == 4) {
        switch (address) {
        case MACIO_WATCHDOG_LOW:
            data = mWatchDogLow;
            return true;
        case MACIO_WATCHDOG_HIGH:
            data = mWatchDogHigh;
            return true;
        case MACIO_TIMER_LOW: {
            uint64 counter = getTimerCounter();
            mTimerHighLatch = counter >> 32;
            data = counter;
            return true;
        }
        case MACIO_TIMER_HIGH:
            data = mTimerHighLatch;
            return true;
        case MACIO_WATCHDOG_ENABLE:
            data = mWatchDogEnable;
            return true;
        }
    }

    if (r == 0 && address >= MACIO_GPIO_BASE && address < MACIO_GPIO_BASE + MACIO_GPIO_SIZE &&
        cuda_is_pmu()) {
        data = address == MACIO_PMU_EXTINT_GPIO && !cuda_pmu_extint_asserted() ? 0x02 : 0;
        return true;
    }

    if (r == 0 && address >= MACIO_BOARD_CONTROL_BASE
        && address < MACIO_BOARD_CONTROL_BASE + MACIO_BOARD_CONTROL_SIZE) {
        data = 0;
        return true;
    }

    if (r == 0 && address >= MACIO_DBDMA_BASE &&
        address < MACIO_DBDMA_BASE + MACIO_DBDMA_CHANNEL_SIZE * MACIO_DBDMA_CHANNEL_COUNT) {
        uint channel = (address - MACIO_DBDMA_BASE) / MACIO_DBDMA_CHANNEL_SIZE;
        address = (address - MACIO_DBDMA_BASE) % MACIO_DBDMA_CHANNEL_SIZE;
        if (size != 4) IO_MACIO_ERR("read with size != 4\n");
        if (size == 4 && !(address & 3) && address <= MACIO_DBDMA_ADDRESS_ADDRESS_HI) {
            if (address == MACIO_DBDMA_ADDRESS_CONTROL) {
                data = 0;
            } else {
                data = mDBDMAChannels[channel].registers[address >> 2];
            }
            return true;
        }
    }
	return false;
}

bool PCI_MacIO::writeDeviceMem(uint r, uint32 address, uint32 data, uint size)
{
    uint esccChannel;
    bool esccDataRegister;
    if (r == 0 && size == 1 && decodeESCCAddress(address, esccChannel, esccDataRegister)) {
        ESCCChannel &escc = mESCCChannels[esccChannel];
        if (esccDataRegister) {
            escc.transmitData = data;
            escc.readRegisters[0] |= ESCC_STATUS_TX_EMPTY;
            escc.readRegisters[1] |= ESCC_SPECIAL_ALL_SENT;
        } else {
            writeESCCControl(esccChannel, data);
        }
        return true;
    }

    if (r == 0 && size == 4) {
        switch (address) {
        case MACIO_WATCHDOG_LOW:
            mWatchDogLow = data;
            return true;
        case MACIO_WATCHDOG_HIGH:
            mWatchDogHigh = data;
            return true;
        case MACIO_WATCHDOG_ENABLE:
            mWatchDogEnable = data;
            return true;
        }
    }

    if (r == 0 && address >= MACIO_GPIO_BASE && address < MACIO_GPIO_BASE + MACIO_GPIO_SIZE &&
        cuda_is_pmu()) {
        return true;
    }

    if (r == 0 && address >= MACIO_BOARD_CONTROL_BASE
        && address < MACIO_BOARD_CONTROL_BASE + MACIO_BOARD_CONTROL_SIZE) {
        IO_MACIO_TRACE("board-control: write(%d) @%08x: %08x\n", size, address, data);
        return true;
    }

    if (r == 0 && address >= MACIO_DBDMA_BASE &&
        address < MACIO_DBDMA_BASE + MACIO_DBDMA_CHANNEL_SIZE * MACIO_DBDMA_CHANNEL_COUNT) {
        uint channel = (address - MACIO_DBDMA_BASE) / MACIO_DBDMA_CHANNEL_SIZE;
        address = (address - MACIO_DBDMA_BASE) % MACIO_DBDMA_CHANNEL_SIZE;
        if (size != 4) IO_MACIO_ERR("write with size != 4\n");
        if (size == 4 && !(address & 3) && address <= MACIO_DBDMA_ADDRESS_ADDRESS_HI) {
            DBDMAChannel &dbdma = mDBDMAChannels[channel];
            uint reg = address >> 2;

            /*
             * The command pointer may be reprogrammed whenever the channel is
             * not transferring.  Drivers quiesce the channel by flushing and
             * waiting for ACTIVE to clear -- they do not clear RUN first -- so
             * gating this write on RUN as well silently discards it and leaves
             * the channel pointing at nothing.
             */
            if (address == MACIO_DBDMA_ADDRESS_CMD_PTR_LO &&
                (dbdma.registers[MACIO_DBDMA_ADDRESS_STATUS >> 2] & MACIO_DBDMA_ACTIVE)) {
                return true;
            }

            dbdma.registers[reg] = data;
            if (address == MACIO_DBDMA_ADDRESS_CONTROL) {
                uint32 oldStatus = dbdma.registers[MACIO_DBDMA_ADDRESS_STATUS >> 2];
                uint32 mask = data >> 16;
                uint32 value = data & 0xffff;
                value &= MACIO_DBDMA_RUN | MACIO_DBDMA_PAUSE | MACIO_DBDMA_FLUSH |
                    MACIO_DBDMA_WAKE | 0xff;
                uint32 status = (value & mask) | (oldStatus & ~mask);

                if (status & MACIO_DBDMA_WAKE) status |= MACIO_DBDMA_ACTIVE;
                if (status & MACIO_DBDMA_RUN) {
                    status |= MACIO_DBDMA_ACTIVE;
                    status &= ~MACIO_DBDMA_DEAD;
                }
                if (status & MACIO_DBDMA_PAUSE) status &= ~MACIO_DBDMA_ACTIVE;
                if ((oldStatus & MACIO_DBDMA_RUN) && !(status & MACIO_DBDMA_RUN)) {
                    status &= ~(MACIO_DBDMA_ACTIVE | MACIO_DBDMA_DEAD);
                }
                dbdma.registers[MACIO_DBDMA_ADDRESS_STATUS >> 2] = status;
                if (!(status & MACIO_DBDMA_RUN)) {
                    dbdma.waitingForInput = false;
                } else if (status & MACIO_DBDMA_ACTIVE) {
                    runDBDMAChannel(channel, (status & MACIO_DBDMA_FLUSH) != 0);
                }
            } else if (address == MACIO_DBDMA_ADDRESS_CMD_PTR_LO) {
                dbdma.registers[reg] &= ~0x0f;
                dbdma.waitingForInput = false;
            }
            return true;
        }
    }
	return false;
}

void macio_init()
{
	gPCI_Devices->insert(new PCI_MacIO());
}

void macio_done()
{
}

void macio_init_config()
{
}

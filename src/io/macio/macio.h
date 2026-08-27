/*
 *	PearPC
 *	macio.h
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

#ifndef __IO_MACIO_H__
#define __IO_MACIO_H__

#include "io/pci/pci.h"

/* KeyLargo feature control registers: FCR0..FCR4 plus MediaBay, at 0x38. */
#define MACIO_FCR_COUNT 6

class PCI_MacIO: public PCI_Device {
private:
    struct ESCCChannel {
        uint8 selectedRegister;
        uint8 readRegisters[16];
        uint8 writeRegisters[16];
        uint8 receiveData;
        uint8 transmitData;
        /* Number of idle-line Break/Abort external-status interrupts delivered. */
        uint32 abortsRaised;
    };

    struct DBDMAChannel {
        uint32 registers[12];
        bool waitingForInput;
    };

    struct DBDMACommand {
        uint16 requestedCount;
        uint16 command;
        uint32 physicalAddress;
        uint32 commandDependent;
        uint16 residualCount;
        uint16 transferStatus;
    };

    uint64 mTimerStartTicks;
    uint32 mTimerHighLatch;
    uint32 mWatchDogLow;
    uint32 mWatchDogHigh;
    uint32 mWatchDogEnable;
    uint32 mFCR[MACIO_FCR_COUNT];
    ESCCChannel mESCCChannels[2];
    DBDMAChannel mDBDMAChannels[16];

    uint64 getTimerCounter() const;
    bool decodeESCCAddress(uint32 address, uint &channel, bool &dataRegister) const;
    void resetESCCChannel(uint channel, bool hardReset);
    uint8 readESCCControl(uint channel);
    void writeESCCControl(uint channel, uint8 value);
    bool readDBDMACommand(uint32 address, DBDMACommand &command) const;
    bool writeDBDMACommand(uint32 address, const DBDMACommand &command) const;
    void runDBDMAChannel(uint channel, bool flushInput);

public:
			PCI_MacIO();
	virtual bool	readDeviceMem(uint r, uint32 address, uint32 &data, uint size);
	virtual bool	writeDeviceMem(uint r, uint32 address, uint32 data, uint size);
};

void macio_init();
void macio_done();
void macio_init_config();

#endif

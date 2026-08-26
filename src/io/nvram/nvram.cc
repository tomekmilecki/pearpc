/*
 *	PearPC
 *	nvram.cc
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

#include <cstdio>
#include <cstring>

#include "debug/tracers.h"
#include "nvram.h"

#define NVRAM_IMAGE_SIZE 0x2000
#define NVRAM_FREE_PARTITION_NAME "wwwwwwwwwwww"
#define NVRAM_FREE_PARTITION_MAGIC 0x7f
#define NVRAM_PARTITION_HDR_SIZE 16
#define NVRAM_SYSTEM_PARTITION_MAGIC 0x70
#define NVRAM_VENDOR_PARTITION_MAGIC 0x7e
#define NVRAM_COMMON_PARTITION_SIZE 0x0800
#define NVRAM_MACOS_PARTITION_SIZE 0x0800

struct NVRAM {
	FILE *f;
};

// For reference:
struct NVRAMPartitionHeader {
	uint8	magic;
	uint8	chksum;
	uint16	length_div_16;
	char	name[12];
};

NVRAM gNVRAM;

void nvram_write(uint32 addr, uint32 data, int size)
{
	uint8 d = (uint8) data;
	addr -= IO_NVRAM_PA_START;
	IO_NVRAM_TRACE("write(%d): %08x at %08x (from @%08x, lr: %08x)\n", size, data, addr, gCPU.pc, gCPU.lr);
	if (addr & 0xf) IO_NVRAM_ERR("address not aligned\n");
	addr >>= 4;
	if (addr >= NVRAM_IMAGE_SIZE) IO_NVRAM_ERR("out of bounds\n");
	if (size != 1) IO_NVRAM_ERR("only supports byte writes\n");
	fseek(gNVRAM.f, addr, SEEK_SET);
	fwrite(&d, 1, 1, gNVRAM.f);
	fflush(gNVRAM.f);
}

void nvram_read(uint32 addr, uint32 &data, int size)
{
	uint8 d = 0;
	addr -= IO_NVRAM_PA_START;
	IO_NVRAM_TRACE("read(%d): at %08x (from @%08x, lr: %08x)\n", size, addr, gCPU.pc, gCPU.lr);
	if (addr & 0xf) IO_NVRAM_ERR("address not aligned\n");
	addr >>= 4;
	if (addr >= NVRAM_IMAGE_SIZE) IO_NVRAM_ERR("out of bounds\n");
	if (size != 1) IO_NVRAM_ERR("only supports byte reads\n");
	fseek(gNVRAM.f, addr, SEEK_SET);
	fread(&d, 1, 1, gNVRAM.f);
	data = d;
}

static uint8 calcChksum(byte *buf)
{
	uint8 x, y;
	y = 0;
	for (int i=0; i < NVRAM_PARTITION_HDR_SIZE; i++) {
		x = y + buf[i];
		if (x < y) x++;
		y = x;
	}
	return y;
}

static void nvram_add_partition(byte *buf, uint32 offset, uint32 length, const char *name, uint8 magic)
{
    byte *header = buf + offset;
    header[0] = magic;
    header[2] = length >> 12;
    header[3] = (length >> 4) & 0xff;
    strncpy((char *)&header[4], name, 12);
    header[1] = calcChksum(header);
}

static void nvram_create_default_image(byte *buf)
{
    memset(buf, 0, NVRAM_IMAGE_SIZE);

    // New World Macs store Open Firmware variables and PRAM/XPRAM in named
    // CHRP partitions. Classic Mac OS requires the APL,MacOS75 partition.
    nvram_add_partition(buf, 0, NVRAM_COMMON_PARTITION_SIZE, "common", NVRAM_SYSTEM_PARTITION_MAGIC);
    nvram_add_partition(buf, NVRAM_COMMON_PARTITION_SIZE, NVRAM_MACOS_PARTITION_SIZE, "APL,MacOS75",
        NVRAM_VENDOR_PARTITION_MAGIC);
    nvram_add_partition(buf, NVRAM_COMMON_PARTITION_SIZE + NVRAM_MACOS_PARTITION_SIZE,
        NVRAM_IMAGE_SIZE - NVRAM_COMMON_PARTITION_SIZE - NVRAM_MACOS_PARTITION_SIZE,
        NVRAM_FREE_PARTITION_NAME, NVRAM_FREE_PARTITION_MAGIC);
}

#define NRAM_KEY_FILE	"nvram_file"
#include "configparser.h"

void nvram_init()
{
	String filename;
	gConfig->getConfigString(NRAM_KEY_FILE, filename);
	
	gNVRAM.f = fopen(filename.contentChar(), "rb+");
	if (!gNVRAM.f) {
		gNVRAM.f = fopen(filename.contentChar(), "wb+");
		if (!gNVRAM.f) IO_NVRAM_ERR("couldn't create file '%y'\n", &filename);
		byte buf[NVRAM_IMAGE_SIZE];
        nvram_create_default_image(buf);
		
		if (fwrite(buf, sizeof buf, 1, gNVRAM.f) != 1) IO_NVRAM_ERR("can't write file '%y'\n", &filename);
		fflush(gNVRAM.f);
	}
}

void nvram_init_config()
{
	gConfig->acceptConfigEntryStringDef(NRAM_KEY_FILE, "nvram");
}

void nvram_done()
{
	fclose(gNVRAM.f);
}

/*
 *	PearPC
 *	promosi.cc
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

#ifndef __PROMOSI_H__
#define __PROMOSI_H__

#include "system/types.h"
#define PROM_MAGIC_OPCODE 0x00345678
#define PROM_RTAS_MAGIC_OPCODE 0x00345679
#define PROM_REAL_MODE_ENTRY 0xfffffff0

extern uint32 gPromOSIEntry;
extern uint32 gPromRTASEntry;

inline bool prom_osi_is_entry(uint32 pc)
{
    return pc == gPromOSIEntry || pc == PROM_REAL_MODE_ENTRY;
}

inline bool prom_rtas_is_entry(uint32 pc)
{
    return pc == gPromRTASEntry;
}

struct prom_args {
	uint32 service;
	uint32 nargs;
	uint32 nret;
	uint32 args[10];
};

void call_prom_osi();
void call_prom_rtas();


#endif

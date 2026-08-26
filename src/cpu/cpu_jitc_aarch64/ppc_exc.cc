/*
 *	PearPC
 *	ppc_exc.cc - AArch64 JIT exception handling (stub)
 *
 *	Copyright (C) 2026 Sebastian Biallas (sb@biallas.net)
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

#include "tools/snprintf.h"
#include "debug/tracers.h"
#include "info.h"
#include "ppc_cpu.h"
#include "ppc_exc.h"
#include "ppc_mmu.h"

/*
 *	The atomic exception raise/cancel functions
 *	(ppc_cpu_atomic_raise_dec_exception, ppc_cpu_atomic_raise_ext_exception,
 *	 ppc_cpu_atomic_raise_stop_exception, ppc_cpu_atomic_cancel_ext_exception)
 *	are defined in jitc_tools.S using AArch64 load-exclusive/store-exclusive
 *	instructions for proper atomicity.
 */

extern PPC_CPU_State *gCPU;

/*
 * Raise a PPC exception (copied from generic CPU).
 * Sets SRR0/SRR1, DAR/DSISR, clears MSR, invalidates TLB,
 * and sets npc to the exception vector address.
 *
 * For the JIT interpreter path (GEN_INTERPRET), this modifies
 * CPU state inline. The generated code checks the return value
 * of the interpreter function and dispatches to npc on exception.
 */
struct ExcRingEntry {
    uint32 type, pc, srr0, srr1, dar, dsisr, lr;
    bool valid;
};
static const int kExcRingSize = 40;
static ExcRingEntry gExcRing[kExcRingSize];
static uint64 gExcRingPos = 0;

extern "C" void ppc_exc_dump_ring()
{
    fprintf(stderr, "  --- last %d non-periodic exceptions (oldest first) ---\n", kExcRingSize);
    uint64 start = gExcRingPos > (uint64)kExcRingSize ? gExcRingPos - kExcRingSize : 0;
    for (uint64 i = start; i < gExcRingPos; i++) {
        const ExcRingEntry &e = gExcRing[i % kExcRingSize];
        if (!e.valid) continue;
        fprintf(stderr, "  [EXC] type=%03x pc=%08x srr0=%08x srr1=%08x dar=%08x dsisr=%08x lr=%08x\n",
                e.type, e.pc, e.srr0, e.srr1, e.dar, e.dsisr, e.lr);
    }
}

bool FASTCALL ppc_exception(PPC_CPU_State &aCPU, uint32 type, uint32 flags, uint32 a)
{
    // aCPU.pc must be set by the caller before ppc_exception() is
    // reachable. For the interpreter path, ppc_opc_gen_interpret stores
    // pc = current_code_base + pc_ofs. For native load/store, the asm
    // stub stores pc_ofs and computes pc on the slow path (TLB miss)
    // before calling the C++ helper. For ISI, ppc_new_pc_asm stores
    // pc directly.

    if (type != PPC_EXC_DEC) PPC_EXC_TRACE("@%08x: type = %08x (%08x, %08x)\n", aCPU.pc, type, flags, a);
    switch (type) {
    case PPC_EXC_DSI: {
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        aCPU.dar = a;
        aCPU.dsisr = flags;
        PPC_EXC_TRACE("DSI @%08x DAR=%08x DSISR=%08x [%s%s%s]\n",
            aCPU.pc, a, flags,
            (flags & PPC_EXC_DSISR_PAGE) ? "PAGE " : "",
            (flags & PPC_EXC_DSISR_PROT) ? "PROT " : "",
            (flags & PPC_EXC_DSISR_STORE) ? "STORE" : "LOAD");
        break;
    }
    case PPC_EXC_ISI:
        if (aCPU.pc == 0) {
            PPC_EXC_WARN("pc == 0 in ISI\n");
            SINGLESTEP("");
        }
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = (aCPU.msr & 0x87c0ffff) | flags;
        break;
    case PPC_EXC_DEC:
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    case PPC_EXC_EXT_INT:
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    case PPC_EXC_SC:
        aCPU.srr[0] = aCPU.npc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    case PPC_EXC_NO_FPU:
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    case PPC_EXC_NO_VEC:
        aCPU.srr[0] = aCPU.pc;
        /* Same SRR1 mask as every other exception (and as the generic
         * reference core).  The narrower mask used here previously dropped
         * the high MSR bits -- MSR_VEC among them -- so the handler's rfi
         * restored a truncated MSR. */
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    case PPC_EXC_PROGRAM:
        aCPU.srr[0] = (flags & PPC_EXC_PROGRAM_NEXT) ? aCPU.npc : aCPU.pc;
        aCPU.srr[1] = (aCPU.msr & 0x87c0ffff) | (flags & ~PPC_EXC_PROGRAM_NEXT);
        break;
    case PPC_EXC_FLOAT_ASSIST:
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    case PPC_EXC_MACHINE_CHECK:
        if (!(aCPU.msr & MSR_ME)) {
            PPC_EXC_ERR("machine check exception and MSR[ME]=0.\n");
        }
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = (aCPU.msr & 0x87c0ffff) | MSR_RI;
        break;
    case PPC_EXC_TRACE2:
        aCPU.srr[0] = aCPU.pc;
        aCPU.srr[1] = aCPU.msr & 0x87c0ffff;
        break;
    default:
        PPC_EXC_ERR("unknown\n");
        return false;
    }
    {
        /* Temporary: keep the most recent non-periodic exceptions for crash dumps. */
        if (type != PPC_EXC_DEC && type != PPC_EXC_EXT_INT) {
            ExcRingEntry &e = gExcRing[gExcRingPos % kExcRingSize];
            e.type = type; e.pc = aCPU.pc; e.srr0 = aCPU.srr[0]; e.srr1 = aCPU.srr[1];
            e.dar = aCPU.dar; e.dsisr = aCPU.dsisr; e.lr = aCPU.lr; e.valid = true;
            gExcRingPos++;
        }
    }
    ppc_mmu_tlb_invalidate(aCPU);
    aCPU.msr = 0;
    aCPU.npc = type;
    return true;
}

void ppc_cpu_raise_ext_exception()
{
    ppc_cpu_atomic_raise_ext_exception(*gCPU);
}

void ppc_cpu_cancel_ext_exception()
{
    ppc_cpu_atomic_cancel_ext_exception(*gCPU);
}

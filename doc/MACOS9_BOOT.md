# Booting Mac OS 9.2 (Power Mac G4 Cube) in PearPC

Status notes for the effort to boot `images/g4cubebackup.img` — a Mac OS 9.2 disk image from a
Power Mac G4 Cube — through to the Finder desktop on the aarch64 JIT (Apple Silicon).

**The image is known good: it boots on the original hardware.** Every remaining fault is therefore
PearPC's, not the image's.

**Current state: reaches the "Welcome to Mac OS" screen with no system error, then stalls before the
Finder draws the desktop.** Earlier states, for reference: bar stuck at ~10% (fixed by §2.4), then
~95% ending in a system error (fixed by §2.5).

Read [AGENT_DEBUGGING.md](AGENT_DEBUGGING.md) first for general methodology. The Golden Rule (the
generic interpreter is the reference implementation) applies here too.

---

## 1. Running the image

### The image must be padded

`ATADeviceFile` ([src/io/ide/ata.cc](../src/io/ide/ata.cc)) only accepts images whose size is an
exact multiple of **516096** bytes (16 heads x 63 spt x 512). `g4cubebackup.img` is 15,211,429,888
bytes — 16,384 over a multiple — so it is rejected outright:

```
*** FATAL: [IO/IDE] <Error> invalid format (filesize isn't a multiple of 516096)
```

Never run against the pristine image. Clone it (APFS copy-on-write, instant) and extend the clone to
the **full disk geometry** (see §5):

```bash
cp -c images/g4cubebackup.img /tmp/g4cubebackup-pearpc.img
python3 -c "import os; os.truncate('/tmp/g4cubebackup-pearpc.img', 20485398528)"
./src/ppc ppccfg.mac9
```

**Re-clone whenever boots start failing early.** Killing PearPC mid-write leaves the HFS+ volume
dirty; after a few such kills the boot dies with `*** FATAL: [IO/PROM] <Error> can't open boot file`.
That is a damaged test image, not a regression.

### Keep RAM at the 128 MB default

Raising `memory_size` makes things **worse**, despite page faults appearing to climb into the
128 MB ceiling:

| memory_size | Result |
|---|---|
| 128 MB (default) | reaches ~85,000 entry points (best) |
| 256 MB | stalls at ~54,700 |
| 512 MB | stalls at ~4,700 (breaks early ROM) |

### Keep the G4 PVR

`cpu_pvr = 0x000c0201` is required. Reporting a G3 (`0x00080200`) to sidestep AltiVec makes all runs
crash early — the ROM needs the G4 identity.

### Verifying progress — look at the framebuffer, not the log

`screencapture` does not work in an agent session (no display access) and the SDL window tends to
sit on another macOS Space, so host screenshots show wallpaper. Dump the guest framebuffer instead.

`framebuffer_dump_file` and `memdump_file` are written by `ppc_cpu_crash_dump()`, which the
`SIGILL`/`SIGSEGV`/`SIGBUS` handler calls — so you can force a dump from a *live* run:

```bash
kill -BUS <pid>
```

The framebuffer is **1024x768 XRGB** (`00 RR GG BB`), *not* BGRA — decoding it as BGRA yields an
olive/yellow image. `kill -USR1 <pid>` injects a synthetic mouse click.

```python
import struct, zlib, sys
W, H = 1024, 768
raw = open(sys.argv[1], 'rb').read()
rows = []
for y in range(H):
    px = raw[y*W*4:(y+1)*W*4]
    line = bytearray(b'\x00')
    for x in range(0, W*4, 4):
        line += bytes((px[x+1], px[x+2], px[x+3]))   # XRGB -> RGB
    rows.append(bytes(line))
def chunk(t, d):
    c = t + d
    return struct.pack('>I', len(d)) + c + struct.pack('>I', zlib.crc32(c) & 0xffffffff)
open(sys.argv[2], 'wb').write(b'\x89PNG\r\n\x1a\n'
    + chunk(b'IHDR', struct.pack('>IIBBBBB', W, H, 8, 2, 0, 0, 0))
    + chunk(b'IDAT', zlib.compress(b''.join(rows), 6)) + chunk(b'IEND', b''))
```

### Progress markers and retrying

`newEntry=` in the `[JITC] … dispatches:` lines is the best progress signal. When `newTrans`/
`newEntry` freeze while `dispatches` climbs into the billions, the guest is in a polling loop.

| `newEntry` | Meaning |
|---|---|
| ~24,000 | early ROM / PMU init |
| ~27,000–31,000 | common death point for the AltiVec loop (§3) |
| ~85,000 | progress bar ~95%, NanoKernel debugger stall (§4) |

Roughly half of all runs die early to §3, so **script the retries**: launch, poll `newEntry`,
restart on exit. Do not babysit single runs.

---

## 2. Fixes applied

### 2.1 DBDMA: `BT` must clear when the channel halts

`src/io/macio/macio.cc`, `MACIO_DBDMA_CMD_STOP` path. Drivers quiesce a channel by writing FLUSH and
spinning until the low status bits clear:

```
bl   read_status          ; ChannelStatus, offset 0x04
clrlwi. r0, r3, 18        ; r0 = status & 0x3FFF
bne  loop
```

`BT` (Branch Taken, `0x100`) is inside that mask. The STOP path cleared `ACTIVE|DEAD|FLUSH` but left
`BT` set, so a halted channel reported `0x8100` forever. Verified: status went from permanently
`0x8100` to reaching `low14=0000`.

### 2.2 DBDMA: `CommandPtrLo` writes gated only on `ACTIVE`

Writes were discarded while `RUN | ACTIVE`. Drivers quiesce by flushing and waiting for **ACTIVE**
to clear — they do not clear RUN, so status is `0x8000` and the write was silently dropped, leaving
the channel pointing at nothing. Verified: the guest now reads a real pointer instead of 0.

### 2.3 `PPC_EXC_NO_VEC` SRR1 mask parity

`src/cpu/cpu_jitc_aarch64/ppc_exc.cc` masked SRR1 with `0x0000ff73` where the generic reference (and
every other exception) uses `0x87c0ffff`, dropping the high MSR bits including MSR_VEC.

### 2.4 Do not advertise an ESCC — this was the big one

`src/io/prom/promdt.cc`, guarded by `const bool advertiseESCC = false;`.

**A Power Mac G4 Cube has no serial ports.** KeyLargo contains an ESCC cell, but advertising
`escc@13000` / `escc-legacy@12000` makes Mac OS open the port and poll a DBDMA receive ring that
nothing can ever feed. Removing those nodes moved the progress bar from **~10% to ~95%**.

Set `advertiseESCC = true` to restore the nodes.

### 2.5 Do not advertise AltiVec in the device tree — the second big one

`src/io/prom/promdt.cc`, guarded by `const bool advertiseAltiVec = false;`.

The `/cpus/PowerPC,G4` node published `graphics` and `data-streams`. In Apple's Open Firmware tree
**`graphics` is what tells Mac OS the CPU implements the AltiVec ("graphics") instruction group**, and
`data-streams` covers `dst`/`dstt`. Mac OS then used AltiVec inside **BlockMove** — the ROM memcpy at
`ffcf46xx`-`ffcf49xx`, whose `lvx` at `ffcf49dc` is the address that trapped f20 endlessly (§3).
That storm corrupted 68k state and produced the "address error" at Finder launch.

Guarding those two properties off removes the crash outright: boot reaches **"Welcome to Mac OS"**
with `DSErrCode=0`, both with extensions intact and with an empty Extensions folder.

Keep `cpu_pvr` at the G4 value. A G3 PVR does give `f20=0`, but the address error comes back — so
AltiVec is not the whole story, and the G3 identity is strictly worse.

**This is a trade-off, not a clean fix — read this before changing it.** Disassembling the
nanokernel's AltiVec-unavailable handler shows exactly why:

```
c0f14240: mtlr r1 / mfsprg0 r1 / mflr r8 / cmpwi r8,3872   ; 3872 = 0xF20, the vector number
c0f1425c: beq  0xc0f14180                                   ; -> AltiVec handler
...
c0f1418c: lwz  r6,-0x14(r1)      ; nanokernel context
c0f14190: lwz  r10,0xd8(r6)      ; a per-context "AltiVec available" flag
c0f1419c: beql 0xc0f1421c        ; if that flag == 0 ...
c0f141a0: oris r11,r11,0x200     ; ... otherwise srr1 |= MSR_VEC  (the grant)
c0f141a8: mtsrr1 r11
c0f141bc: bl   0xc0f23a60        ; vector context save/restore
```

and the `flag == 0` path never comes back — `c0f1421c` restores registers and does
`bl 0xc0f13d40 ; li r8,4 ; b 0xc0f12ab4`, abandoning the grant.

That flag is almost certainly derived from the `graphics` property. So removing the property stops
the *corruption* (BlockMove mostly avoids AltiVec) but also tells the nanokernel it has no vector
unit, and any remaining AltiVec use then loops forever because the grant is refused. Observed
behaviour is consequently variable: some runs reach "Welcome to Mac OS", others saturate the
exception ring with f20 at `ffcf48d8` around ~88,000 entry points.

AltiVec *execution* is not the problem: every AltiVec opcode — arithmetic (`ppc_opc_gen_group_v`)
**and** load/store (`GEN_INTERPRET_LOADSTORE`) — runs the interpreter, i.e. literally the same code
as the generic core. `lvsl`/`lvsr`/`vperm` are byte-identical between the two, and `test_altivec`
passes.

### The AltiVec trap is silently swallowed (real bug, deliberately left in)

`ppc_opc_group_v()` raises `PPC_EXC_NO_VEC` and **returns non-zero** when MSR_VEC is clear. But
`ppc_opc_gen_group_v()` calls plain `ppc_opc_gen_interpret()`, whose own comment says:

> "No exception check here. Non-load/store opcodes don't trigger DSI."

So the return value is discarded. `ppc_exception()` has already zeroed `msr` and set
`SRR0`/`SRR1`/`npc = 0xF20`, and the JIT then carries on executing the rest of the translated block
with that clobbered state. The FP path does this correctly via `ppc_opc_gen_check_fpu()`; the vector
path has no equivalent. The correct form is:

```c
ppc_opc_gen_interpret_loadstore(jitc, ppc_opc_group_v);   // checks W0, dispatches to npc
return flowEndBlock;
```

**This fix must be applied together with `advertiseAltiVec = true`** — the two are coupled, and
either one alone is worse than neither. The measured matrix:

| `advertiseAltiVec` | exception fix | Result |
|---|---|---|
| true | absent (upstream) | address error at ~95% |
| false | absent | "Welcome to Mac OS", but AltiVec still broken (f20 loops at ~88k on some runs) |
| false | present | address error at ~65k entries |
| **true** | **present** | **f20 = 0, DSErrCode = 0, no fatal** — AltiVec fully converges |

`f20` count is the proxy for whether the vector unit is working: 33–40 means the trap is looping,
0 means the nanokernel granted it and it stuck. **The tree ships the last row** (both correct).

With both applied the guest is *healthy but stalled*: the cursor is the spinning-quadrant **busy**
cursor (not a bomb), `DSErrCode=0`, no fatal error, and it sits at the ~10% progress bar for 40+
minutes executing 68k code at `pc=680a5150` (the emulator's `lbzx` dispatch). `newEntry` creeps
(104,714 → 105,189 over ~25 min) — which, as §3b warns, is not evidence of progress.

Note the `false/absent` row renders *further* (it reaches "Welcome to Mac OS", the last screen before
the Finder draws) but leaves AltiVec fundamentally broken; it is a rendering artefact of a
compromised machine, not a better boot. Both rows stall short of the desktop.

`advertiseAltiVec` and the Extensions folder also interact: **empty Extensions + both fixes** regresses
to `f20=33, DSERR=1`, while **extensions intact + both fixes** gives `f20=0, DSERR=0`. Test with the
extensions left alone.

### What the remaining stall is

`pc=68067f48` sits inside the ROM's **68k interpreter dispatch loop** (PA `0x00f67f40`):

```
c0f67f40: lha  r4,0(r3)          ; fetch 68k opcode word -- r3 IS the 68k PC
c0f67f48: rlwimi r29,r4,3,13,28  ; build the dispatch-table index from the opcode
c0f67f4c: mtlr r29
c0f67f50: lha  r27,2(r3)         ; fetch the extension word
c0f67f54: addi r24,r3,2          ; advance the 68k PC
c0f67f58: bclr 5,8               ; jump to the opcode handler
```

So the guest is executing 68k code (`r3` = 68k PC; `r27` = a fetched extension word — do **not**
mistake it for an address). Injecting mouse clicks with `kill -USR1` changes nothing, so it is not the
known mouse wait. The cursor is the spinning-quadrant busy cursor, so Mac OS believes it is working.

**The 68k PC is frozen.** No rebuild is needed to see this — the periodic `[JITC]` line already prints
`r3`, so just histogram it:

```bash
grep dispatches run.log | tail -400 | sed -n 's/.*r3=\([0-9a-f]*\).*/\1/p' | sort | uniq -c
```

Result: **400 of 400 samples read `r3=00e1905c`**. The 68k interpreter is re-executing one single 68k
instruction forever — a tight loop, definitively *not* slow progress. Sampled `pc` values stay inside
the dispatch loop (`68067f40`, `680a5150`, `680b37d0`, `68067f48`, `68066160`).

### Resolved: the stall was the SCC re-entrancy lock

Having the *emulator* resolve the 68k PC (rather than guessing the EA→PA skew from a dump) identified
it exactly:

```
[M68K] 68kPC=00e1905c op=4a2a ext=03a6 | A2=00eeb240 target_ea=00eeb5e6 -> pa=012475e6 byte=ff
```

`4A2A 03A6` is **`TST.B $03A6(A2)`**, and the polled byte at 68k EA `0x00eeb5e6` reads **`0xff` in
209/209 samples** — never cleared. That is the SCC/LocalTalk driver's re-entrancy lock: set before a
protected call, cleared on return. It never clears because the ESCC nodes were disabled (§2.4), so the
driver's call cannot complete.

**Restoring `advertiseESCC = true` eliminates this spin completely** (0 occurrences of the TST.B
loop). Note the EA→PA skew here is *not* the `+0x4000` seen elsewhere — `00e1905c` maps to
`0117505c`. Always let the emulator translate; never assume a fixed offset.

### Method note: beware sampling aliasing

Histogramming `r3` from the periodic `[JITC]` line at its fixed 10,000,000-dispatch interval gave
"400/400 samples at one address", which looked like a frozen PC and was **wrong** — it aliased against
a periodic guest loop. Re-sampling at a *prime* interval (997001) gave 47 distinct PCs. Use a prime
sampling interval for any periodic guest sampling.

### The two stalls are mutually exclusive — this is the crux

With everything else fixed, the boot has exactly one blocker, and the ESCC switch just moves it:

| `advertiseESCC` | Symptom |
|---|---|
| false | SCC driver's re-entrancy lock never clears — 68k spins on `TST.B $3A6(A2)`, byte stays `0xff`, because the driver's call cannot complete without hardware |
| **true** | The driver runs, and the boot stalls in the **serial DBDMA receive path** (`pc=00bb461c`, `r3=80808700` = channel 7 base) — the very loop this whole investigation started on |

So `advertiseESCC = true` is correct (KeyLargo really has an ESCC, and it clears the lock), and **the
single remaining blocker is the DBDMA receive path for a serial port with nothing attached.** On real
hardware the receive DMA is simply armed and never completes, and the driver proceeds; here the status
/descriptor values we report make it block.

Everything else is healthy in that configuration: `DSErrCode=0`, no SCC lock spin, and `f20≈26`, which
is *normal lazy vector-context switching* — not a runaway. Confirmed by reading the nanokernel's own
grant flag (`ctx = *(SPRG0-0x14)`, `flag = *(ctx+0xd8)`): it reads `0x07efb8e0`, i.e. **non-zero**, so
the grant path is taken. Only the very first trap of a boot sees `flag == 0`.

### The DBDMA handshake is correct — the driver is waiting for serial data

Correlating the driver's own pointer against the engine's, at each FLUSH write:

```
[DB-W] ch=7 FLUSH status=00008000 cmdPtr(phys)=010f5f30 | guest r21=00d99f30 r23=a4000300
```

Both end in `f30`: **EA `0x00d99f30` maps to PA `0x010f5f30`** — driver and engine are on the *same*
descriptor (skew here is `0x35C000`; again, not a fixed offset — always correlate rather than assume).
The value it reads, `0xa4000300`, satisfies **both** competing wait loops:

| test | computation | result |
|---|---|---|
| loop A: spins while `(w & 0x24000000) == 0x04000000` | `0xa4000300 & 0x24000000 = 0x24000000` | exits |
| loop B: spins while `(w & 0x07FFFFFF) == 0` | `= 0x04000300` | exits |

And the channel status reaches `0x8000` (`low14=0000`), so the status wait passes too. **The register
and descriptor handshake is therefore correct.** What remains is the driver's *outer* loop:

```
8648: lwz r0,0x1cc(r30) ; cmpwi r0,0 ; bne <back to receive processing>
```

It re-arms and re-flushes while a flag at `[r30+0x1cc]` stays non-zero — i.e. it is polling for
**received serial data that never arrives**, on a port with nothing attached. No status value can
satisfy that; the driver needs whatever event clears that flag (a completion/interrupt, or a timeout).
That is the single remaining blocker, and it is a missing *event*, not a wrong register value.

### The missing piece: the ESCC has no interrupt generation

`src/io/macio/macio.cc` models the ESCC as a passive register file — it **never calls
`pic_raise_interrupt()` anywhere**. Only the DBDMA engine raises interrupts, and only when a
descriptor asks for one; the serial descriptors here are `cmd=0x2000`, whose interrupt field is
`INT_NEVER`. So no serial event can ever reach the guest.

The guest, meanwhile, definitely expects them. Tracing writes to the Z8530 interrupt-enable registers
during the stall:

```
WR9  <- c0    ; hardware reset
WR9  <- 02    ; MIE - Master Interrupt Enable
WR1  <- e4    ; Rx interrupt mode + WAIT/DMA enables
WR15 <- a1    ; external/status interrupt enables
WR1  <- fd / 7d / fd / 7d ...   ; toggling bit 7 = WAIT/DMA Request Enable, every iteration
```

That is a driver arming SCC interrupts and re-arming the DMA request each time round its loop, then
waiting. Implementing this properly means modelling the Z8530 interrupt sources (Rx available,
Tx empty, external/status) against WR1/WR9/WR15 and the interrupt-acknowledge/RR2 vector path — real
work, and past experience in this file says a partial version will simply move the stall again.

Note also what the driver is *waiting for* is data on a port with nothing attached, so the honest
question is which event a Cube's ESCC produces in that situation (an external/status change, or a
receive timeout) — that is a hardware-behaviour question to settle before writing code.

### Implemented: idle-line Break/Abort interrupt (confirmed gap, but not sufficient)

The ESCC now generates one interrupt source, in `macio.cc`:

* **Why.** `WR1 = 0xfd` sets Rx interrupts to *special-condition only*, and `WR15 = 0xa1` enables
  **Break/Abort** external/status interrupts, with `WR9 = 0x02` (MIE). Nothing is attached to a Cube's
  ESCC, so the receive line idles at continuous marks — which a Z8530 in SDLC/HDLC mode reports as an
  Abort, raising an external/status interrupt (RR0 D7). That is defined hardware behaviour, not a
  guess.
* **What.** When the guest has armed MIE + Ext Int Enable + Break/Abort IE, set `RR0 D7` and raise the
  channel's OpenPIC source (ch B = `0x24`, ch A = `0x25`, matching the device tree). Capped at
  `ESCC_MAX_IDLE_ABORTS` so a driver re-arming in a loop cannot cause a storm.
* **Acknowledge.** The OpenPIC source is level-triggered, so WR0 commands *Reset External/Status
  Interrupts* (`(v & 0x38) == 0x10`) and *Reset Highest IUS* (`0x38`) clear `RR0 D7` and call
  `pic_cancel_interrupt()`. Without this the handler is re-entered forever.

**Result: delivered and acknowledged cleanly — no storm, `DSErrCode = 0`, no fatal — but it does not
unblock the boot.** The stall moves (to `pc=680b9000`) and the screen is unchanged. Kept because
"the ESCC cannot raise any interrupt at all" is a genuine defect regardless, and this regresses
nothing; but it is *not* the missing piece on its own.

### Still open

The knobs interact and outcomes vary run to run between: address error, AltiVec f20 loop, the SCC lock
spin, and "Welcome to Mac OS". No combination yet reaches the Finder desktop. Two things to resolve:

1. **Establish the EA→PA mapping for that 68k PC before decoding it.** `r3` is a guest EA and
   `memdump_file` is physical; this region has shown a `+0x4000` skew elsewhere. At identity the bytes
   are coherent *PPC* code (`bne cr1` / `addi r3,r8,-1` / `stw r3,0xc4(r31)`), which would mean the 68k
   interpreter is being fed PPC instructions — i.e. it was entered with a bogus PC. Confirm the mapping
   before trusting that reading.
2. **Find what re-enters the dispatcher with an unchanged PC.** A handler that faults and retries, or
   one that never advances the PC, both fit the observation.

---

## 3. The AltiVec exception loop (kills ~50% of runs)

The single most damaging remaining bug. The exception ring (§6) fills with:

```
[EXC] type=f20 pc=ffcf49dc srr0=ffcf49dc srr1=0000d032 lr=0005f0dc
[EXC] type=f20 pc=ffcf49dc srr0=ffcf49dc srr1=0000d032 lr=0005f0dc
...
```

`0xf20` is **AltiVec Unavailable** (`PPC_EXC_NO_VEC`). The instruction at `0xffcf49dc` is
`0x7c2018ce` = `lvx`, inside the ROM's AltiVec context-save helper. `srr1` shows MSR_VEC clear every
time, so the fault is legitimate — the guest really is resuming with AltiVec disabled.

Tracing MSR_VEC transitions shows a stable 4-step cycle that never converges:

```
[VEC] ON  pc=00f141b4 old=00000000 new=02000000 srr0=ffcf49dc srr1=0200d032 lr=00000f20
[VEC] ON  pc=00f13df4 old=00002000 new=0200f032 srr0=ffcf47a8 srr1=0200f032
[VEC] ON  pc=00f23cb8 old=00002000 new=02002000 srr0=6806e8c0 srr1=0202f032
[VEC] OFF pc=00f24524 old=02002000 new=0000d032 srr0=6806e1a0 srr1=0002d032
```

The handler *does* enable VEC and sets `srr0=ffcf49dc`, but the nanokernel eventually returns to the
task at `0x6806e1a0` with **VEC off** (`srr1` lacks `0x02000000`), which immediately re-faults.
So the nanokernel is failing to record that this task owns the vector unit.

When this loop does not simply hang, it degenerates into the crash previously tracked as "blocker A":
identical state every time, `pc=ffcf08a8 r3=68fff400 srr1=02082000`, which is the same ROM AltiVec
save routine dereferencing a translated pointer with MSR_IR/DR clear. The illegal opcode
`0xf027ec29` at `0x5f0f0` and the machine check are downstream symptoms, not causes — primary
opcode 60 is `ppc_opc_invalid` in the generic core too.

**`__VEC_EXC_OFF__` (in `ppc_vec.h`) suppresses this entirely** — defining it drops the f20 count to
zero. It is *not* enabled by default, because end-to-end it is not a win: with the storm suppressed
the guest never gets the trap that tells it to save/restore vector state, and runs then stall at
~70,800 entry points in the 68k emulator (`pc=68066160`) in roughly two of three attempts, versus the
~85,000 that storm-affected runs reach. It remains the fastest way to take AltiVec out of the picture
when bisecting something else.

**Importantly, the AltiVec storm and the §4 late stall are independent.** With `__VEC_EXC_OFF__`
enabled and the f20 count at zero, the late stall still ends with the identical ISI/DSI pair. Fixing
the AltiVec loop will therefore not fix §4.

**Ruled out as the cause:**

- `scripts/debug/stub_audit.py` reports no AltiVec stubs (only the harmless `dcbX` cache hints).
- `MSR_RFI_SAVE_MASK` is `0x87c0ff73`, which *does* include MSR_VEC, so `rfi` can restore it.
- `PPC_CPU_UNSUPPORTED_MSR_BITS` permits MSR_VEC, so `mtmsr` can set it.
- VRSAVE (SPR 256) *is* implemented — the native `mfspr`/`mtspr` codegen falls through to the
  interpreter for it (`ppc_alu.cc`), which reads/writes `aCPU.vrsave`.

The next place to look is the nanokernel's vector-unit ownership bookkeeping and whatever emulator
state it keys off.

---

## 3b. Making Mac OS tell you what went wrong

Two techniques turned a silent stall into a diagnosis:

**Empty the Extensions folder — do not remove it.** Renaming `System Folder/Extensions` aside leaves
Mac OS with no folder at all, and it then traps silently into the NanoKernel debugger (§4). Renaming
it aside *and creating an empty `Extensions` folder in its place* lets startup reach its own error
handling, which draws a real dialog:

```bash
cd "/Volumes/.../System Folder" && mv Extensions "Extensions (Off)" && mkdir Extensions
```

**Zoom into the cursor.** The top-left cursor glyph is drawn by the guest. Magnifying the top-left
28x28 pixels of the framebuffer dump shows a classic **bomb** when Mac OS has taken a system error —
which distinguishes "crashed" from "slow" at a glance, without any tracing.

With both applied, the guest displays:

> **Sorry, a system error occurred.** — **address error**
> *To temporarily turn off extensions, restart and hold down the shift key.*

behind a nearly-full progress bar. A 68k **address error** is the exception for a word/long access to
an odd address, i.e. a corrupted pointer in the 68k environment, raised around Finder launch.

**The AltiVec loop is implicated, despite an earlier misreading.** A `DSErrCode` watchpoint (§6)
shows the ring immediately before the error is full of `type=f20` exceptions at `ffcf49dc`. Enabling
`__VEC_EXC_OFF__` produces a byte-identical framebuffer histogram, which at first looked like
independence — but it is not exculpatory: with the trap suppressed the guest *never saves or restores
vector context at all*, so both settings corrupt vector state, just differently. Treat "same symptom
with `__VEC_EXC_OFF__`" as "still broken", not "unrelated". The image
truncation does not cause it either: `System` (11.4 MB + 8 MB rsrc), `Finder` (1.9 MB + 0.5 MB rsrc)
and `Mac OS ROM` (2.6 MB) all read back complete with no zero-filled regions.

**Beware of misreading `newEntry` creep as progress.** A crashed guest still slowly translates new
code — the 68k emulator keeps exercising fresh dispatch paths. One run was left for **49 minutes**:
`newEntry` climbed 104,390 -> 105,363 while the progress bar never moved off ~10% and the cursor was
a bomb the whole time. Judge progress from the framebuffer, not the counters.

---

## 3c. Current stall: "Welcome to Mac OS"

Deterministic, no system error, unchanged after 20 minutes. The exception ring ends:

```
[EXC] type=f20 pc=ffcf48d8 ...                        ; a few AltiVec traps remain (f20=10..33)
[EXC] type=400 pc=0080b0b8 srr1=4000d032 lr=ffcec400  ; ISI, instruction page fault
[EXC] type=700 pc=0080b0bc srr1=0004d032 lr=ffcec400  ; Program, PPC_EXC_PROGRAM_PRIV
```

But physical memory at those addresses holds an ordinary epilogue:

```
0080b0b0: lwz r0,0x8(r1) / mtlr r0 / lmw r24,-0x20(r1) / blr
```

`blr` is not privileged, so **the instruction fetched did not match physical memory** — the EA→PA
resolution for that page is wrong, or the page is mapped somewhere unexpected. Registers corroborate
corruption: `r1` (the stack pointer) holds `0x7c281280` and `r5` holds `0x7c3143a6`, which is
literally the first instruction of the `0xF20` exception vector — i.e. PPC instruction words are
sitting in data registers while the guest builds CFM/Mixed-Mode glue.

Ruled out for this one: `icbi` is implemented correctly (resolves EA→PA and calls
`jitcDestroyAndFreeClientPage`), and neither the aarch64 nor the x86_64 JIT invalidates on write —
both rely on the guest's `icbi`, so this is not an aarch64-only omission.

---

## 4. The earlier late stall — NanoKernel debugger

Past §3, the boot reaches ~85,000 entry points with the progress bar ~95%, then parks here:

```
f2751c: lwz r1,0x0(0) / addi r1,r1,1 / stw r1,0x0(0)   ; bump a counter at address 0
        bl  0xf26880                                    ; poll serial console for a character
        cmpwi r8,-1 / bne exit / b f2751c               ; spin while nothing arrives
```

`0xf26880` loads `r28 = *(r1+0xf700)`; with no ESCC that pointer is 0, so it returns `r8 = -1`
forever. This is the **NanoKernel debugger waiting for serial console input**, i.e. the system has
already trapped. Injecting mouse clicks does nothing (it is not the known mouse wait, despite
`r20` holding ASCII `"mous"`).

The exception ring shows exactly what precedes it:

```
[EXC] type=300 pc=6806ac20 dar=0790f000 dsisr=42000000   ; store page faults marching upward
[EXC] type=300 pc=00f15ed8 dar=0790f000 dsisr=42000000   ; handler fault (normal pattern)
[EXC] type=400 pc=00804774 srr1=4000d032 lr=ffcec400     ; ISI — instruction page fault
[EXC] type=300 pc=0080a39c dar=05c80000 dsisr=40000000   ; DSI the nanokernel cannot handle
```

Execution reaches `0x00804774` via `bctr` (it matches `ctr` in the dump, with `r12` holding the same
transition-vector target — a normal CFM cross-fragment indirect call), and that code page is not
resident. That ISI is handled; execution continues to `0x0080a39c`, where a *load* from `0x05c80000`
faults and the nanokernel gives up. `0x05c80000` is exactly one page past the nanokernel's
`0x05c7xxxx` data region, which smells like an off-by-one walking to the end of a 64 KB structure.

The DSI appears to be legitimate rather than a missed translation: `ppc_effective_to_physical()` in
`src/cpu/cpu_jitc_aarch64/ppc_mmu.cc` implements both the primary and the secondary (`~hash1`,
`PTE1_H` set) PTEG searches, matching the architecture and the generic core, so the guest genuinely
has no PTE for that address.

Note the DSI/handler-DSI pairs with matching DAR are the *normal* demand-paging pattern and appear
throughout a healthy boot; they are not themselves a fault.

---

## 4b. Reading guest state: Mac OS low memory is at physical 0x4000

The 68k low-memory globals are **not** at physical 0 (that holds the PPC exception vectors). They sit
at physical **0x4000**, so `physical = 0x4000 + lowmem_offset` in a `memdump_file`. Verified by
reading `FinderName` (0x02E0), which returns the Pascal string `"Finder"`.

At the address-error stop this yields:

| Global | Offset | Value | Meaning |
|---|---|---|---|
| `DSErrCode` | 0x0AF0 | **0x0002** | system error 2 = address error (matches the dialog) |
| `FinderName` | 0x02E0 | `"Finder"` | low memory is valid |
| `CurApName` | 0x0910 | length 0xFF (garbage) | the application has **not** finished launching |
| `CurrentA5` | 0x0904 | 0x05c7f190 | the 68k A5 world |
| `CurStackBase` | 0x0908 | 0x05c7209c | the 68k stack |
| `BootDrive` | 0x0210 | 0x8023 | boot volume is set |

So the failure is **during Finder launch**, before `CurApName` is filled in. Note that the
unhandleable DSI in §4 is at `dar=0x05c80000`, just past that same `0x05c7xxxx` A5 world — consistent
with a corrupted segment/jump table, which is also what produces odd addresses and hence an
"address error".

**Cleared of suspicion:**

- *Not the DBDMA fixes.* IDE does not use the MacIO DBDMA engine at all (`src/io/ide/ide.cc` has no
  reference to it), so §2.1/§2.2 cannot corrupt disk reads.
- *Not a JIT regression in covered paths.* All 11 tests in `test/run_tests.sh` pass, including
  `test_altivec`. On macOS the runner needs a GNU `timeout`; without one every test reports
  `FAIL (exit 127)`, which is a missing tool, not a real failure.
- *Not AltiVec, not truncation.* See §3b.

---

## 4c. Watching for the system error directly

`jitc.cc` polls `DSErrCode` (physical `0x4af0`) every million dispatches and, on the first plausible
error code (non-zero, `< 0x100` — the uninitialised value is `0xFFFF`), prints the CPU state and the
exception ring. That is how the error was pinned to dispatch ~503,000,000 with the f20 burst
immediately preceding it. Cheap enough to leave enabled.

What the handler actually does, from the MSR_VEC trace: the fault is at `ffcf49dc` with `srr1`
showing **PR set** (user mode) called from `lr=ffcf4730`. The nanokernel enables VEC and sets
`srr0=ffcf49dc`, but the final `rfi` returns to a *different* address (`0x6806e1a0`, inside the 68k
emulator) with **VEC off** — i.e. it abandons the operation and restarts it rather than resuming the
faulting `lvx`. That is the loop. Making Mac OS's per-task vector-unit ownership converge is the
open problem.

**Tried and rejected:** correcting `MSR_RFI_SAVE_MASK` from `0x87c0ff73` to the architectural
`0x87c0ffff` (`rfi` restores SRR1 bits 0, 5:9, 16:31; the old value silently dropped `MSR_PM` and two
reserved bits). The change is kept because it is spec-correct, but it changed nothing observable —
same error, same dispatch count, same framebuffer.

---

## 4d. Mouse and keyboard input (open)

Input does not work.  The host half is fixed and proven; the guest half is not.

### A fast harness — do not reboot per experiment

Rebooting to test one hypothesis costs ~10 minutes and roughly half of boots die to
the intermittent JIT fault (§3).  Instead keep **one** instance alive and probe it:

- `kill -USR2 <pid>` injects synthetic motion plus a keypress straight into the
  CUDA/PMU queue, bypassing SDL and the mouse-grab gate.
- The same handler reads Mac OS's own globals with `ppc_dma_read()` and prints
  them, so no memory dump and no kill is needed.
- It also cycles the packet encoding, so **one boot tests four variants**.

```
[PROBE] variant=1 RawMouse v=15 h=15 KeyMap=00000000 | queued=1 delivered=115 \
        gpioReads=0 intBits=14 mask=90 asserted=1
```

That turned a 40-minute sweep into 35 seconds.  Note the guest reaches a usable
Toolbox around 50,000 entry points — you do **not** need the login screen to test
input, which also dodges the late-boot crash.

### Host side: fixed

- `SDLSystemDisplay::setMouseGrab()` only *confined* the pointer.  The guest is
  driven by relative deltas, and a confined pointer stops producing them at the
  window edge.  Now also sets `SDL_SetWindowRelativeMouseMode()`.
- SDL3 reports `xrel`/`yrel` as **float** (they were `Sint32` in SDL2).  Truncating
  each event discarded every sub-pixel delta, so a **trackpad** produced no motion
  at all while a mouse mostly worked.  Deltas are now accumulated across events,
  and an event that rounds to (0,0) is dropped rather than spending the PMU's
  single ADB reply slot.
- Motion events had `x` and `y` transposed.

### Guest side: Mac OS receives the packets and discards them

Proven with counters, not inference:

- `delivered` increments on **every** probe (and was already >100 from boot), so
  Mac OS collects each packet via `PMU_INT_ACK`.
- `mask=90` — `PMU_INT_ADB` is unmasked; `asserted=1` — the interrupt is raised.
- `RawMouse` never leaves its startup value of `(15,15)`.

The low-memory base is confirmed each run by two globals whose values are
predictable: `CrsrPin` (0x834) must equal the screen bounds, and `ScrnBase`
(0x824) must equal `0x84000000`, PearPC's framebuffer address.  Both match, so
the reads are real.

### Ruled out

| Hypothesis | Result |
|---|---|
| Wrong ADB device address | No — enumeration assigns keyboard 2 / mouse 3, and `mouseaddr`/`keybaddr` agree |
| Wrong packet encoding | No — all four of `[0x3c,dy,dx]` and `[1,3,0x3c,dy,dx]`, each with and without `PMU_INT_ADB_AUTO`, are ignored |
| Mouse-specific bug | No — injected **keypresses** are ignored too (`KeyMap` stays all zeros) |
| Mac OS prefers USB HID | No — disabling `USB Device Extension` + `USB Software Locator` in the guest does **not** make it fall back to ADB |
| Autopoll gating in our code | No — `gCUDA.autopoll` is written but never read, so it gates nothing |

Mac OS also never polls the bus: after enumeration there are **zero** ADB requests,
and `gpioReads=0` (it never reads the PMU interrupt GPIO).  It never sends the
Linux-style autopoll enable (`PMU_ADB_CMD` with `0x86`) either.

### Two real gaps found on the way

- `pmu_adb_request()` never answered **Talk register 0** — the register carrying
  mouse movement — for any device; it fell through to "no data".  Implemented,
  along with pending-movement accumulation so motion is not lost while a reply is
  outstanding.  Correct behaviour, but Mac OS does not use this path.
- PearPC's USB is an **OHCI host-controller shell**: root-hub registers only, no
  device descriptors, no HID class, no attached devices.  A real G4 Cube has no
  ADB port at all — its keyboard and mouse are USB — so faithful input ultimately
  needs USB HID emulation on the root hub.

Do not simply remove the USB controller: `pci_usb_installed = 0` stalls the boot
at ~17,000 entry points, before Mac OS starts.

### Next steps

Either disassemble the guest's ADB/PMU driver to find what it requires before it
will dispatch received data, or implement a USB HID mouse on the emulated OHCI
root hub.  Three PMU commands remain unimplemented and are the cheapest lead:

```
cmd=8e len=5 payload: 00 ff ff ff ff     <- mask-shaped, looks like event/poll config
cmd=80 len=4 payload: e6 b5 97 f9
cmd=82 len=0
```

---

## 4e. USB HID: implemented, but the guest never enumerates it (open)

A G4 Cube's keyboard and mouse are USB, so this is the path that should work.
The device side is written and the host controller now has a transfer engine;
what is missing is the guest bringing up the root hub port.

### What was added

`src/io/usb/usbhid.{cc,h}` -- boot-protocol mouse and keyboard: descriptors
(device, configuration, interface, HID, endpoint, and both report descriptors),
the standard requests, the HID class requests, interrupt-IN reports with
movement accumulation, and an ADB-to-USB keycode table.

`src/io/usb/usb.cc` -- the OHCI transfer engine, which **did not exist**: the
file was a register file with no ED/TD traversal at all.  Added descriptor
walking for control and interrupt endpoints, the done queue, WDH/RHSC
interrupts, page-crossing TD buffers, per-port addressing so the two devices can
be told apart during enumeration, root hub reset/enable semantics, and a frame
timer.

Three genuine bugs found while doing it:

- `OHCI_RH_PS_LSDA` was `(1 << 8)`, identical to `PPS`; it is bit 9.
- The USB PCI interrupt line was `0x03`, but `promdt.cc` routes the device to
  `IO_PIC_IRQ_USB`, so every interrupt went nowhere.
- `intrstatus` was a plain store; it is write-1-to-clear.  `intrenable` and
  `intrdisable` are set/clear masks, not stores.

### How far the guest gets

With the PCI IDs left at the original OPTi values no driver attaches at all.
Changing them to Apple KeyLargo OHCI (`0x106b/0x0019`) makes Mac OS attach:

```
ctrl_reg=000000bc      operational, control+periodic+bulk lists enabled
hcca=00148200  ctrlHead=00114880
intrEn=8000007f -> RHSC enabled; the interrupt is raised and serviced
```

and then it stops:

```
portReads=1  TDs=0  ctrl=0  addr0=0 addr1=0
[USB-PORT] read port0 -> 00010301      (CCS|PPS|LSDA|CSC)
```

It reads the port status exactly once, never writes to it, and never enables the
port -- so no device is addressed and no TD is ever processed.

### PC attribution is instruction-accurate -- validated

`ppc_cpu_get_pc()` returns `pc_ofs + current_code_base`, and the aarch64 MMU
slow-path stubs write `pc_ofs` before calling out (`str w9, [x20, #pc_ofs]` in
`jitc_mmu.S`), which is the path MMIO takes.  Confirmed directly by reading the
instruction at each reported PC and checking it really is a memory op:

```
reg 30 <- 0        pc=00f15eb8  insn=92b3fffc  stw     (nanokernel, in RAM)
reg 20 <- 0        pc=ffdfbb78  insn=90850020  stw     (ROM)
reg 18 <- 00148200 pc=ffdfbc44  insn=906c0018  stw     (ROM)
reg 04 <- 000000bc pc=ffdfbd34  insn=906c0004  stw     (ROM)
```

**One trap when checking this yourself:** `ppc_dma_read()` takes a *physical*
address.  A ROM PC such as `0xffdfbb78` reads back as zeros unless you map it
through the ROM mirror first (`0xffcxxxxx` -> `0x00cxxxxx`).  Reading it raw and
concluding the PC is bogus is a mistake that cost time here -- as did an earlier
guess that the commented-out `gCPU.pc = ...` lines in `io.cc` were responsible;
`ppc_cpu_get_pc()` never reads that field.

### The decisive measurement: Mac OS never touches the controller

Capturing the guest PC on every OHCI register access settles it.  All of the
setup, and the single port read, come from ROM:

```
[USB-W] reg 20 <- 00000000  from pc=ffdfbb78     ed_controlhead
[USB-W] reg 18 <- 00148200  from pc=ffdfbc44     hcca
[USB-W] reg 04 <- 000000bc  from pc=ffdfbd34     control: operational
[USB-PORT] read port0 -> 00010303  from pc=ffe02fa0
```

`0xffdxxxxx`/`0xffexxxxx` is Open Firmware.  **Not one register access comes from
Mac OS.**  The heavy `fmnumber` polling (hundreds of thousands of reads) is the
same ROM code's watchdog idle loop.

So the problem is not the transfer engine, the descriptors, the port status, or
the interrupt: nothing in Mac OS ever binds to this controller, so nothing ever
enumerates the root hub.  Making the emulated device answer better cannot help
until something in the OS is driving it.  That is where to start next --
find why the USB extensions do not attach, rather than tuning the device.

### Two real bugs in the node/device, found late

**The node already published what I kept trying to add.**  Its `compatible` is
`"pci1045,c861\0pciclass,0c0310\0"` and it already had `assigned-addresses`.
`pciclass,0c0310` -- the string the driver matches on -- was there all along.
Two consequences worth knowing:

- An earlier experiment that "added `compatible`" actually added a **second**
  `compatible` property.  That duplicate is the likely reason it measured
  harmful; the verdict "compatible is harmful" was wrong.
- Changing the PCI IDs to Apple's left `compatible` claiming the old OPTi
  identity `pci1045,c861`, contradicting `vendor-id`/`device-id`.  The Name
  Registry binds on those names, so the node advertised two identities.  It is
  now a single consistent `"pci106b,19\0pciclass,0c0310\0"`.

**The PCI command register was never set**, so it read back `0x00`: memory space
decoding and bus mastering both disabled.  An OHCI controller needs both -- its
registers are memory mapped and it DMAs its own descriptor lists -- and a driver
probing that register sees a device it cannot use.  Now `0x06`.

Mac OS reads that register exactly once, from `pc=00026a80` (RAM, not ROM), and
never touches the device again.  That single probe is the only config-space
access the device ever receives, and it still happens with the command register
correct -- so whatever decides not to proceed is deciding on something else.

### The driver's own string table names what it matches on

Extract the extension and dump the strings around the KeyLargo logic (it is a
PEF container, `Joy!peff`):

```
0x00962c  pciclass,0c0310      <- the "compatible" value it matches
0x009645  OHCIUIM
0x009728  vendor-id / device-id / revision-id
0x00975e  assigned-addresses / AAPL,address / driver-ist / AAPL,clock-id
0x00981e  compatible
0x009829  Keylargo
```

`pciclass,0c0310` is the generic class-code form of `compatible` that Mac OS's
Name Registry uses, and our class-code is exactly `0x0c0310`.  That is the
string to publish -- **not** `pci106b,19`, which was a guess of mine and
measured actively harmful.  It is now published and is harmless, but on its own
it still does not make the driver bind.

The remaining properties in that list are the obvious next things to supply:
`driver-ist`, `AAPL,address`, `assigned-addresses`, `AAPL,SuspendablePorts`.

### What the guest's driver actually wants

Rather than guessing at the device, read the driver.  Mount the image and run
`strings` over `System Folder/Extensions/USB Device Extension` (395 KB data
fork; the resource fork is only 5 KB and has nothing useful):

```
UIM    - keylargo found            <- it explicitly looks for KeyLargo
Keylargo
CEGetDeviceCompatibleNames         <- it reads the "compatible" property
UIM    - PCI Capabilities exists   <- it walks the PCI capability list
UIM    - PCI Capabilities Ptr:
UIM    - PCI PMC:                  <- and reads the power-management capability
AAPL,clock-id
AAPL,address
AAPL,current-available
AAPL,SuspendablePorts
```

So the OHCI UIM expects an Apple KeyLargo-style controller: a `compatible`
property it can match, a PCI **capability list** with a power-management
capability (our config space has none -- status bit 4 clear, capability pointer
0), and several `AAPL,` properties.  Note it also expects to *find KeyLargo* --
on a real Cube the USB controllers are functions of KeyLargo, whereas ours is a
standalone PCI device on the bridge.  That structural difference is the most
likely reason nothing binds.

Each of these has now been tried individually:

| Change | Result |
|---|---|
| `AAPL,clock-id` = "usb0" | Harmless -- reaches the login screen -- but nothing binds |
| PCI capability list with a PM capability (status bit 4, cap pointer 0x40, PM cap) | Harmless, kept, but nothing binds |
| `compatible` = `"pci106b,19\0usb"` | **Harmful**: 3 of 3 boots failed to reach the login screen |
| Move `usb` beside `mac-io` under the host `pci` node | Breaks the boot (`MacOS: Fatal Error! (0xF3B37FDB)` after ~1,500 entry points) **if done alone** -- the host node's `interrupt-map` had only the video card in it, leaving the moved node unmappable.  Adding a `usb` entry to that map (child unit address `0x00003000` -> `IO_PIC_IRQ_USB`) makes the move clean: boots reach the login screen.  Kept, because it matches real hardware -- but it does **not** make the driver bind. |

Our `mac-io` node already advertises `compatible = "Keylargo"` and
`model = "KeyLargo"`, so the driver's KeyLargo probe is satisfied.  Note that
`mac-io`'s own `reg` says bus 1 even though it hangs below the host node, so the
parent/bus mismatch is not what matters here.

With the topology corrected, the capability list present and `AAPL,clock-id`
set, Mac OS **still** never touches the controller: every register access traces
to ROM.  Everything reachable from the device tree has now been tried.  What
remains is to find the driver's actual matching path -- disassembling
`USB Device Extension` around its `CEGetDeviceCompatibleNames` call and the
"keylargo found" branch would show exactly which node and properties it walks,
rather than inferring it from the outside.

### Tried and rejected

| Change | Result |
|---|---|
| SOF interrupt raised per frame | Guest drowns in ISR entries; boot dies.  The status bit is set, the interrupt is not raised. |
| Frame counter advanced 1:1 with real time | No change; the heavy `fmnumber` polling is the ROM's USB watchdog idle loop, not a stall |
| `HccaFrameNumber` published in step with `HcFmNumber` | Correct and kept -- Open Firmware declares the controller broken if the register runs more than 5 frames ahead (`addi r0,r4,5; cmp r6,r0; ble`) -- but not sufficient |
| `POTPGT` set to a real value | No change |
| Devices hot-plugged after the bus comes up rather than present from reset | No change |
| `compatible` property added to the `usb` node | **Harmful** -- retested on a clean disk, 3 of 3 boots failed to reach the login screen, where the same build without it reached it on the first attempt.  Reverted. |

### Also fixed: enabling an interrupt must assert a pending one

`intrenable` was a set-mask only.  A driver that enables RHSC *after* the ports
have already come up waits forever for an edge that has already happened, so the
write now calls `raiseIRQ()`.  This is correct behaviour regardless, and it is
what finally got the guest to read the port status in the engine-off
configuration.

### The engine is off by default

`pci_usb_hid` gates the frame thread and defaults to **0**.  With it running, the
thread keeps publishing the frame number into the HCCA that Open Firmware
programmed; once Mac OS reclaims that memory the writes corrupt it and
extensions fail with "error type 10".  Bisection confirmed this was the cause of
a run of failed boots.  Do not enable it until the guest's driver owns the
controller.

### A dead end worth recording: the low-memory pointer shim

Writing `RawMouse`/`MTemp` directly does move Mac OS's pointer coordinates --
verified, they track injected motion exactly -- but **the cursor does not
redraw**, so it achieves nothing visible.  It is also unsafe: applied from the
event thread it races the CPU, and applied at all before the Toolbox owns low
memory it corrupts startup (boots bomb at ~5%).  The code is left in place
behind `gCudaShimEnabled`, off.

### VBL is now delivered

`gcard_raise_interrupt()` existed but **nothing ever called it**, so vertical
blank interrupts were never generated even when the guest enabled them.  It is
now called from the redraw loop and `Ignore VBL` is `no`.  This matters because
Mac OS moves the cursor from a VBL task.  Tested stable on its own.

---

## 5. The image is truncated

The Apple Partition Map:

| # | Name | Type | Start | Size (sectors) |
|---|---|---|---|---|
| 1 | Apple | `Apple_partition_map` | 1 | 63 |
| 5 | MacOS | `Apple_HFS` | 704 | 40,009,226 |
| 6 | — | `Apple_Free` | 40,009,930 | 614 |

The disk ends at sector 40,010,544 = **20,485,398,528 bytes** (itself an exact multiple of 516096 —
strong evidence this is the true original size). The file is only **15,211,429,888 bytes**, so about
**5.3 GB is missing**, including HFS+'s alternate volume header. Content stored beyond 15.2 GB is
simply not in the backup. Restoring the full size on a clone makes the volume mountable on the host:

```bash
hdiutil attach -nomount -imagekey diskimage-class=CRawDiskImage /tmp/g4cubebackup-pearpc.img
fsck_hfs -fy /dev/disk4s5            # if the mount refuses with "Invalid argument"
mount -t hfs /dev/disk4s5 /tmp/g4mnt
# ... edit the guest System Folder ...
sync && diskutil unmount /tmp/g4mnt && hdiutil detach /dev/disk4
```

Use `diskutil unmount`, not `umount` (which reports "Resource busy").

---

## 6. Diagnostics available

`ppc_exception()` keeps a ring of the last 40 non-periodic exceptions (DEC and external interrupts
excluded, or the ring fills with timer noise). `ppc_cpu_crash_dump()` prints it, so `kill -BUS`
yields the fault history leading to any stall. This is how §3 and §4 were identified — add capped
`fprintf` traces (a `static int` counter with a limit) for anything more specific, and remove them
afterwards.

Several plausible-looking hypotheses in this investigation were wrong and were only settled by
tracing: a DBDMA fix aimed at a code path that never executed, and two different descriptor
status-word constants that each satisfied one driver wait loop while stranding another. Trace before
changing device semantics.

---

## 6b. Also ruled out (later work)

- **Disabling AppleTalk in the guest.** Renaming `System Folder/Preferences/AppleTalk Preferences`
  and `Remote Access` aside changes nothing — same screen, same stall (`DSErrCode=0`, f20≈25,
  ~101,900 entry points). The serial probe is not driven by those preferences.
- **Empty Extensions + the AltiVec fixes.** Regresses to `f20=33, DSERR=1`; test with Extensions
  left intact.
- **ESCC interrupts alone.** See §3d — correct and non-regressing, but not sufficient.

## 7. Ruled out

- **Extensions / Open Transport / AppleTalk.** Renaming `System Folder/Extensions` aside (the
  equivalent of a Shift-boot) reproduces the identical stalls, so no extension is responsible.
  AppleTalk is configured for Ethernet and no NIC is enabled.
- **The image itself.** It boots on original hardware.
- **Image truncation.** Restoring full geometry does not change any stall.
- **RAM size and CPU PVR.** See §1.

---

## 8. Building the generic interpreter reference

Out-of-tree `configure` is blocked by the in-tree build, so copy the tree:

```bash
cp -Rc /Users/tom/Code/pearpc /tmp/pearpc-generic && rm -rf /tmp/pearpc-generic/images
cd /tmp/pearpc-generic && make distclean
./configure --enable-ui=sdl --enable-cpu=generic && make -j8
```

Run with `ppccfg.mac9.generic` (separate image clone and NVRAM so both cores can run at once).

**It is not a usable reference for anything past early boot.** Measured: after **70 minutes** it had
only loaded BootX and printed `MacOS: unable to find a usable NVRAM partition` — still on PearPC's own
boot console, not even at the Mac OS startup screen, where the JIT arrives in about a minute. Use it
only for early-boot faults (it does not exhibit §3); do not plan on it reaching §4 or the Finder.

---

## 9. Working notes

The working tree carries a large set of uncommitted changes on top of upstream `2e27a1f`, with **no
stashes and no reflog history** beyond the initial clone. Earlier states are not recoverable — commit
before large refactors.

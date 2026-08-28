#!/bin/bash
#
# Launch Mac OS 9.2 (Power Mac G4 Cube image) under PearPC.
#
#   ./run-mac9.sh           launch, creating the working disk if needed
#   ./run-mac9.sh --reset    discard the working disk and rebuild it from the pristine image
#
# The pristine images/g4cubebackup.img is never written to.  Runs use an APFS
# copy-on-write clone in /tmp, so "--reset" is instant and always safe.
#
# Only ONE emulator may use the working disk at a time -- two instances sharing
# it corrupt the volume, which shows up as "can't open boot file".  This script
# refuses to start a second one, and repairs a dirty volume automatically.
#
set -e
cd "$(dirname "$0")"

PRISTINE=images/g4cubebackup.img
WORK=/tmp/g4cubebackup-pearpc.img
FULL_SIZE=20485398528          # true disk size from the Apple Partition Map
MNT=/tmp/g4mnt

# --- refuse to run two emulators against the same disk -----------------------
if pgrep -x ppc >/dev/null 2>&1 || pgrep -f "ppccfg.mac9" >/dev/null 2>&1; then
    echo "error: PearPC is already running against $WORK" >&2
    echo "       Two instances sharing one disk image corrupt it." >&2
    echo "       Stop it first:  pkill -f 'ppc ppccfg.mac9'" >&2
    exit 1
fi

detach_all() {
    for d in $(hdiutil info 2>/dev/null | awk -v img="$WORK" '$0 ~ img {found=1} /^\/dev\/disk/ && found {print $1; found=0}'); do
        hdiutil detach "$d" >/dev/null 2>&1 || true
    done
}

if [ "$1" = "--reset" ]; then
    echo "==> discarding $WORK"
    detach_all
    rm -f "$WORK"
fi

if [ ! -f "$WORK" ]; then
    [ -f "$PRISTINE" ] || { echo "error: $PRISTINE not found" >&2; exit 1; }

    # PearPC's ATADeviceFile only accepts images that are a whole number of
    # 16-head/63-spt cylinders (516096 bytes).  The backup is 16384 bytes over,
    # and is also ~5.3 GB short of the size its partition map declares, so
    # extend the clone to the real geometry.
    echo "==> cloning $PRISTINE -> $WORK (copy-on-write, instant)"
    cp -c "$PRISTINE" "$WORK"
    python3 -c "import os,sys; os.truncate(sys.argv[1], $FULL_SIZE)" "$WORK"
    NEEDS_EXT_FIX=1
fi

# --- repair the volume if a previous run left it dirty -----------------------
# A hard kill mid-write leaves HFS+ inconsistent; PearPC then fails with
# "can't open boot file".  Attaching + fsck is quick when the volume is clean.
echo "==> checking the guest volume"
detach_all
if hdiutil attach -nomount -imagekey diskimage-class=CRawDiskImage "$WORK" >/dev/null 2>&1; then
    sleep 1
    DEV=$(diskutil list 2>/dev/null | grep -B12 -m1 "Apple_HFS" >/dev/null 2>&1; true)
    DEV=""
    for d in /dev/disk4s5 /dev/disk5s5 /dev/disk6s5 /dev/disk7s5 /dev/disk8s5; do
        [ -e "$d" ] || continue
        if diskutil info "$d" 2>/dev/null | grep -q "Apple_HFS"; then DEV="$d"; break; fi
    done
    if [ -n "$DEV" ]; then
        fsck_hfs -fy "$DEV" >/dev/null 2>&1 && echo "    volume OK" || echo "    volume repaired"

        # Mac OS opens the serial port during startup and blocks waiting for
        # data that an emulated machine with nothing attached can never supply.
        # Move the serial/modem extensions aside; the Finder reports the missing
        # AppleTalk connection and carries on.
        if [ "$NEEDS_EXT_FIX" = "1" ]; then
            mkdir -p "$MNT"
            if mount -t hfs "$DEV" "$MNT" 2>/dev/null; then
                echo "==> disabling serial/modem extensions in the guest System Folder"
                EXT="$MNT/System Folder/Extensions"
                OFF="$MNT/System Folder/Extensions (Serial Off)"
                mkdir -p "$OFF"
                for f in "Setup Modem Selector" "OpenTpt Serial Arbitrator" "SerialShimLib" \
                         "Apple Modem Tool" "Internal V.90 Modem" "OpenTpt Remote Access" \
                         "Serial Tool" "XMODEM Tool"; do
                    [ -e "$EXT/$f" ] && mv "$EXT/$f" "$OFF/" && echo "    moved: $f"
                done
                # PEARPC_DISABLE_MOUSE_EXT=1 also moves the third-party mouse
                # drivers aside.  Kensington MouseWorks is installed on this
                # image and is a suspect for claiming the USB mouse and then
                # not driving it: the guest enumerates and polls our mouse but
                # ignores every report, while the keyboard works.
                if [ "$PEARPC_DISABLE_MOUSE_EXT" = "1" ]; then
                    echo "==> disabling third-party mouse extensions"
                    CPL="$MNT/System Folder/Control Panels"
                    for f in "Kensington USB Devices" "Kensington USB Shim" \
                             "Kensington Startup ADB" "InputSprocket Extension"; do
                        [ -e "$EXT/$f" ] && mv "$EXT/$f" "$OFF/" && echo "    moved: $f"
                    done
                    [ -e "$CPL/Kensington MouseWorks" ] && \
                        mv "$CPL/Kensington MouseWorks" "$OFF/" && \
                        echo "    moved: Kensington MouseWorks"
                fi
                sync
                diskutil unmount "$MNT" >/dev/null 2>&1 || true
            else
                echo "    warning: could not mount the guest volume; serial extensions left in place" >&2
            fi
        fi
    fi
    detach_all
    sleep 1
fi

[ -x src/ppc ] || { echo "error: src/ppc not built -- run 'make -j8' first" >&2; exit 1; }

echo "==> starting PearPC"
echo "    click once (or press F12) to grab the mouse -- that first click is"
echo "    consumed by the grab and does not reach Mac OS.  F12 releases it."
exec ./src/ppc ppccfg.mac9

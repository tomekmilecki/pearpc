/*
 *	PearPC
 *	sysdisplay.cc - screen access functions for SDL
 *
 *	Copyright (C)      2004 Jens v.d. Heydt (mailme@vdh-webservice.de)
 *	Copyright (C)      2004 John Kelley (pearpc@kelley.ca)
 *	Copyright (C) 1999-2002 Stefan Weyergraf
 *	Copyright (C) 1999-2004 Sebastian Biallas (sb@biallas.net)
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
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <SDL3/SDL.h>

#include "system/display.h"
#include "system/sysexcept.h"
#include "system/systhread.h"
#include "system/sysvaccel.h"
#include "system/types.h"

#include "cpu/mem.h"
#include "tools/data.h"
#include "tools/debug.h"
#include "tools/snprintf.h"

#include "configparser.h"

//#define DPRINTF(...)
#define DPRINTF(...) ht_printf("[Display/SDL]: " __VA_ARGS__)

#include "syssdl.h"


uint SDLSystemDisplay::bitsPerPixelToXBitmapPad(uint bitsPerPixel)
{
	if (bitsPerPixel <= 8) {
		return 8;
	} else if (bitsPerPixel <= 16) {
		return 16;
	} else {
		return 32;
	}
}

#define MASK(shift, size) (((1 << (size))-1)<<(shift))

// The guest sees a 16 MiB PCI framebuffer aperture and probes beyond the
// currently visible mode while detecting VRAM. Keep the full aperture backed.
static const size_t kEmulatedFramebufferSize = 16 * 1024 * 1024;

void SDLSystemDisplay::dumpDisplayChar(const DisplayCharacteristics &chr)
{
	fprintf(stderr, "\tdimensions:          %d x %d pixels\n", chr.width, chr.height);
	fprintf(stderr, "\tpixel size in bytes: %d\n", chr.bytesPerPixel);
	fprintf(stderr, "\tpixel size in bits:  %d\n", chr.bytesPerPixel*8);
	fprintf(stderr, "\tred_mask:            %08x (%d bits)\n", MASK(chr.redShift, chr.redSize), chr.redSize);
	fprintf(stderr, "\tgreen_mask:          %08x (%d bits)\n", MASK(chr.greenShift, chr.greenSize), chr.greenSize);
	fprintf(stderr, "\tblue_mask:           %08x (%d bits)\n", MASK(chr.blueShift, chr.blueSize), chr.blueSize);
	fprintf(stderr, "\tdepth:               %d\n", chr.redSize + chr.greenSize + chr.blueSize);
}

SDLSystemDisplay::SDLSystemDisplay(const char *title, const DisplayCharacteristics &chr, int redraw_ms)
: SystemDisplay(chr, redraw_ms)
{
	mTitle = strdup(title);

	gFrameBuffer = (byte*)malloc(kEmulatedFramebufferSize);
	memset(gFrameBuffer, 0, kEmulatedFramebufferSize);
	damageFrameBufferAll();

	mSDLFrameBuffer = NULL;
	mChangingScreen = false;

	sys_create_mutex(&mRedrawMutex);
}

void SDLSystemDisplay::finishMenu()
{
}

void SDLSystemDisplay::updateTitle()
{
	String key;
	int key_toggle_mouse_grab = gKeyboard->getKeyConfig().key_toggle_mouse_grab;
	SystemKeyboard::convertKeycodeToString(key, key_toggle_mouse_grab);
	String curTitle;
	curTitle.assignFormat("%s - [%s %s mouse]", mTitle, key.contentChar(), (isMouseGrabbed() ? "disables" : "enables"));
	if (gSDLWindow) {
		SDL_SetWindowTitle(gSDLWindow, curTitle.contentChar());
	}
}

int SDLSystemDisplay::toString(char *buf, int buflen) const
{
	return snprintf(buf, buflen, "SDL");
}

void SDLSystemDisplay::displayShow()
{
	if (!isExposed()) return;

	sys_lock_mutex(mRedrawMutex);

	/*
	 * The framebuffer is written by the CPU thread while SDL runs on the
	 * main thread.  The legacy min/max damage markers are not synchronized,
	 * so relying on them can permanently miss a completed burst of guest
	 * drawing.  SDL3 textures are already presented every redraw; upload the
	 * complete visible surface as well so the window always reflects VRAM.
	 */
	if (mSDLFrameBuffer) {
		healFrameBuffer();
		sys_convert_display(mClientChar, mSDLChar, gFrameBuffer,
			mSDLFrameBuffer, 0, mClientChar.height - 1);
		drawCursorOverlay();
		SDL_UpdateTexture(gSDLTexture, NULL, mSDLFrameBuffer,
			mClientChar.width * mSDLChar.bytesPerPixel);
	}

	// Always clear+render+present every frame to keep the
	// Metal compositor happy (backbuffer contents are undefined
	// after SDL_RenderPresent).
	SDL_RenderClear(gSDLRenderer);
	SDL_RenderTexture(gSDLRenderer, gSDLTexture, NULL, NULL);
	SDL_RenderPresent(gSDLRenderer);

	sys_unlock_mutex(mRedrawMutex);
}


/*
 * Composite the pointer ourselves.
 *
 * Mac OS normally draws the cursor from a VBL task, but the injected video.x
 * can never deliver VBL: the call that claims the display interrupt sits behind
 * a flag in the driver's globals that nothing in the image ever sets, so it is
 * unreachable dead code, and the driver does not even import
 * VSLDoInterruptService.  The guest therefore never redraws the pointer,
 * whichever transport delivers the motion.
 *
 * The CUDA shim keeps Mac OS's own globals correct (RawMouse, Mouse, MTemp,
 * MBState, clamped to CrsrPin), so GetMouse() and Button() see the right
 * thing; this draws what those globals say, using the guest's real cursor
 * bitmap from TheCrsr when it looks sane and a built-in arrow otherwise.
 */
void SDLSystemDisplay::drawCursorOverlay()
{
	if (!mSDLFrameBuffer) return;

	uint8 mo[4];
	if (!ppc_dma_read(mo, 0x4000 + 0x830, 4)) return;	/* Mouse */
	int cy = (sint16)((mo[0] << 8) | mo[1]);
	int cx = (sint16)((mo[2] << 8) | mo[3]);

	uint8 vis = 1;
	ppc_dma_read(&vis, 0x4000 + 0x8cc, 1);			/* CrsrVis */
	if (!vis) return;

	/* TheCrsr: data[32] mask[32] hotSpot(v,h) */
	uint8 c[68];
	bool haveCrsr = ppc_dma_read(c, 0x4000 + 0x844, 68);
	uint16 data[16], mask[16];
	int hy = 0, hx = 0;
	bool sane = false;
	if (haveCrsr) {
		int maskBits = 0;
		for (int i = 0; i < 16; i++) {
			data[i] = (uint16)((c[i*2] << 8) | c[i*2+1]);
			mask[i] = (uint16)((c[32+i*2] << 8) | c[32+i*2+1]);
			for (int b = 0; b < 16; b++) if (mask[i] & (1 << b)) maskBits++;
		}
		hy = (sint16)((c[64] << 8) | c[65]);
		hx = (sint16)((c[66] << 8) | c[67]);
		sane = maskBits > 4 && maskBits < 250 &&
		       hy >= 0 && hy < 16 && hx >= 0 && hx < 16;
	}
	if (!sane) {
		/* Built-in arrow, so a garbled TheCrsr never leaves the user blind. */
		static const uint16 aData[16] = {
			0x0000,0x4000,0x6000,0x7000,0x7800,0x7c00,0x7e00,0x7f00,
			0x7f80,0x7c00,0x6c00,0x4600,0x0600,0x0300,0x0300,0x0000 };
		static const uint16 aMask[16] = {
			0xc000,0xe000,0xf000,0xf800,0xfc00,0xfe00,0xff00,0xff80,
			0xffc0,0xffc0,0xfe00,0xef00,0xcf00,0x8780,0x0780,0x0380 };
		for (int i = 0; i < 16; i++) { data[i] = aData[i]; mask[i] = aMask[i]; }
		hy = hx = 0;
	}

	{
		static int n = 0;
		if (n < 8) {
			n++;
			fprintf(stderr, "[OVL] draw at (%d,%d) vis=%d sane=%d bpp=%d "
				"clientBpp=%d W=%d H=%d fb=%p\n",
				cx, cy, (int)vis, (int)sane, mSDLChar.bytesPerPixel,
				mClientChar.bytesPerPixel, mClientChar.width,
				mClientChar.height, (void *)mSDLFrameBuffer);
		}
	}
	const int bpp = mSDLChar.bytesPerPixel;
	const int W = mClientChar.width, H = mClientChar.height;
	uint8 *fb = (uint8 *)mSDLFrameBuffer;
	for (int row = 0; row < 16; row++) {
		int y = cy - hy + row;
		if (y < 0 || y >= H) continue;
		for (int col = 0; col < 16; col++) {
			if (!(mask[row] & (0x8000 >> col))) continue;
			int x = cx - hx + col;
			if (x < 0 || x >= W) continue;
			bool black = (data[row] & (0x8000 >> col)) != 0;
			uint8 *px = fb + (size_t)y * W * bpp + (size_t)x * bpp;
			uint32 v = black ? 0x00000000u : 0xffffffffu;
			for (int b = 0; b < bpp; b++) px[b] = (uint8)(v >> (8 * b));
		}
	}
}

void SDLSystemDisplay::convertCharacteristicsToHost(DisplayCharacteristics &aHostChar, const DisplayCharacteristics &aClientChar)
{
	aHostChar = aClientChar;
}

bool SDLSystemDisplay::changeResolution(const DisplayCharacteristics &aCharacteristics)
{
	// We absolutely have to make sure that SDL_calls are only used
	// in the thread, that did SDL_INIT and created Surfaces etc...
	// This function behaves as a forward-function for changeResolution calls.
	// It creates an SDL_Condition and pushes a userevent onto
	// the event queue. SDL_WaitCondition is used to wait for the event-thread
	// to do the actual work (in reacting on the event and calling changeResolutionREAL)
	// and finally signaling back to us that work is done.

	// AND: we have to check if the call came from another thread.
	// otherwise we would block and wait for our own thread to continue.-> endless loop

	mSDLChartemp = aCharacteristics;
	if (SDL_GetCurrentThreadID() != mEventThreadID) {
		SDL_Event ev;
		SDL_Mutex *tmpmutex;

		SDL_zero(ev);
		ev.type = SDL_EVENT_USER;
		ev.user.code = 1;

		tmpmutex = SDL_CreateMutex();
		mWaitcondition = SDL_CreateCondition();

		SDL_LockMutex(tmpmutex);
		SDL_PushEvent(&ev);

		SDL_WaitCondition(mWaitcondition, tmpmutex);

		SDL_UnlockMutex(tmpmutex);
		SDL_DestroyMutex(tmpmutex);
		SDL_DestroyCondition(mWaitcondition);
		return mChangeResRet;
	} else {
		// we can call it directly because we are in the same thread
		return changeResolutionREAL(aCharacteristics);
	}

}

bool SDLSystemDisplay::changeResolutionREAL(const DisplayCharacteristics &aCharacteristics)
{
	DisplayCharacteristics chr;

	DPRINTF("changeRes got called\n");

	convertCharacteristicsToHost(chr, aCharacteristics);

	DPRINTF("SDL: Changing resolution to %dx%dx%d\n", aCharacteristics.width, aCharacteristics.height, chr.bytesPerPixel * 8);

	mSDLChar = chr;
	mClientChar = aCharacteristics;

	sys_lock_mutex(mRedrawMutex);

	// Destroy old texture if it exists
	if (gSDLTexture) {
		SDL_DestroyTexture(gSDLTexture);
		gSDLTexture = NULL;
	}

	// Create window if it doesn't exist
	if (!gSDLWindow) {
		SDL_WindowFlags windowFlags = 0;
		if (mFullscreen) windowFlags |= SDL_WINDOW_FULLSCREEN;

		if (!SDL_CreateWindowAndRenderer(mTitle,
				aCharacteristics.width, aCharacteristics.height,
				windowFlags, &gSDLWindow, &gSDLRenderer)) {
			ht_printf("SDL: FATAL: can't create window: %s\n", SDL_GetError());
			exit(1);
		}
		SDL_SetRenderDrawColor(gSDLRenderer, 0, 0, 0, 255);
		SDL_ShowWindow(gSDLWindow);
		SDL_RaiseWindow(gSDLWindow);
	} else {
		SDL_SetWindowSize(gSDLWindow, aCharacteristics.width, aCharacteristics.height);
		if (mFullscreen) {
			SDL_SetWindowFullscreen(gSDLWindow, true);
		}
	}

	// Determine pixel format for SDL3 texture
	SDL_PixelFormat pixelFormat;
	switch (chr.bytesPerPixel) {
	case 2:
		pixelFormat = SDL_PIXELFORMAT_XRGB1555;
		mSDLChar.redSize = 5;
		mSDLChar.greenSize = 5;
		mSDLChar.blueSize = 5;
		mSDLChar.redShift = 10;
		mSDLChar.greenShift = 5;
		mSDLChar.blueShift = 0;
		break;
	case 4:
		pixelFormat = SDL_PIXELFORMAT_XRGB8888;
		mSDLChar.redSize = 8;
		mSDLChar.greenSize = 8;
		mSDLChar.blueSize = 8;
		mSDLChar.redShift = 16;
		mSDLChar.greenShift = 8;
		mSDLChar.blueShift = 0;
		break;
	default:
		ASSERT(0);
		break;
	}

	gSDLTexture = SDL_CreateTexture(gSDLRenderer, pixelFormat,
		SDL_TEXTUREACCESS_STREAMING,
		aCharacteristics.width, aCharacteristics.height);
	if (!gSDLTexture) {
		ht_printf("SDL: FATAL: can't create texture: %s\n", SDL_GetError());
		exit(1);
	}
	SDL_SetTextureBlendMode(gSDLTexture, SDL_BLENDMODE_NONE);
	SDL_SetTextureScaleMode(gSDLTexture, SDL_SCALEMODE_NEAREST);

	mFullscreenChanged = mFullscreen;

	gFrameBuffer = (byte*)realloc(gFrameBuffer, kEmulatedFramebufferSize);

	// Allocate host framebuffer for pixel format conversion
	free(mSDLFrameBuffer);
	mSDLFrameBuffer = (byte*)malloc(mClientChar.width *
		mClientChar.height * mSDLChar.bytesPerPixel);

	damageFrameBufferAll();
	sys_unlock_mutex(mRedrawMutex);
	return true;
}

void SDLSystemDisplay::getHostCharacteristics(Container &modes)
{
	// SDL3: enumerate display modes
	int count = 0;
	SDL_DisplayID displayID = SDL_GetPrimaryDisplay();
	const SDL_DisplayMode * const *sdlModes = SDL_GetFullscreenDisplayModes(displayID, &count);
	if (sdlModes) {
		for (int i = 0; i < count; i++) {
			const SDL_DisplayMode *mode = sdlModes[i];
			DisplayCharacteristics *dc = new DisplayCharacteristics;
			dc->width = mode->w;
			dc->height = mode->h;
			dc->bytesPerPixel = SDL_BYTESPERPIXEL(mode->format);
			dc->scanLineLength = -1;
			dc->vsyncFrequency = (int)mode->refresh_rate;
			dc->redShift = -1;
			dc->redSize = -1;
			dc->greenShift = -1;
			dc->greenSize = -1;
			dc->blueShift = -1;
			dc->blueSize = -1;
			modes.insert(dc);
		}
	}
}

void SDLSystemDisplay::setMouseGrab(bool enable)
{
	if (enable == isMouseGrabbed()) return;
	SystemDisplay::setMouseGrab(enable);
	if (gSDLWindow) {
		if (enable) {
			SDL_SetCursor(mInvisibleCursor);
			SDL_SetWindowMouseGrab(gSDLWindow, true);
			/*
			 * Confining the cursor is not enough: the guest is driven by
			 * relative deltas, and a merely-confined cursor stops producing
			 * them as soon as it reaches a window edge, which strands the
			 * guest pointer.  Relative mode keeps deltas flowing regardless.
			 */
			SDL_SetWindowRelativeMouseMode(gSDLWindow, true);
		} else {
			SDL_SetWindowRelativeMouseMode(gSDLWindow, false);
			SDL_SetCursor(mVisibleCursor);
			SDL_SetWindowMouseGrab(gSDLWindow, false);
		}
	}
}

void SDLSystemDisplay::initCursor()
{
	mVisibleCursor = SDL_GetDefaultCursor();
	// Create an invisible cursor
	SDL_Surface *surface = SDL_CreateSurface(16, 16, SDL_PIXELFORMAT_RGBA8888);
	if (surface) {
		memset(surface->pixels, 0, surface->h * surface->pitch);
		mInvisibleCursor = SDL_CreateColorCursor(surface, 0, 0);
		SDL_DestroySurface(surface);
	} else {
		mInvisibleCursor = mVisibleCursor;
	}
}

SystemDisplay *allocSystemDisplay(const char *title, const DisplayCharacteristics &chr, int redraw_ms)
{
	DPRINTF("Making new window %d x %d\n", chr.width, chr.height);
	return new SDLSystemDisplay(title, chr, redraw_ms);
}

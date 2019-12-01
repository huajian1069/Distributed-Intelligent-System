#include <stdio.h>
#include <errno.h>
#include "simio.h"
static void __inline__
doopen(PSIMIO psimio)
{
  	__asm__(".pword 0xDAC000" :: "r" (psimio));
}
int __attribute__((__weak__, __section__(".libc")))
open(const char *name, int access, int mode)
{
	SIMIO simio;

	simio.method = SIM_OPEN;
	simio.u.open.name = name;
	simio.u.open.access = access;
	simio.u.open.mode = mode;
	simio.u.open.handle = _FD_INVALID;
	doopen(&simio);

        if (simio.u.open.handle == _FD_INVALID)
	{
        	errno = EFOPEN;
	}
	return(simio.u.open.handle);
}


#include <stdio.h>
#include "simio.h"
static void __inline__
doclose(PSIMIO psimio)
{
  	__asm__(".pword 0xDAC000" :: "r" (psimio));
}
int __attribute__((__weak__, __section__(".libc")))
close(int handle)
{
	SIMIO simio;

	simio.method = SIM_CLOSE;
	simio.u.close.handle = handle;
	doclose(&simio);

	return(simio.u.close.handle);
}

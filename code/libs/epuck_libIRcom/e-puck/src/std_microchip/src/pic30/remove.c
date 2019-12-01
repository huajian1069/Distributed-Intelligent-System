#include <stdio.h>
#include "simio.h"
static void __inline__
doremove(PSIMIO psimio)
{
  	__asm__(".pword 0xDAC000" :: "r" (psimio));
}
int __attribute__((__weak__, __section__(".libc")))
remove(const char *filename)
{
	SIMIO simio;

	simio.method = SIM_UNLINK;
	simio.u.unlink.filename = filename;
	doremove(&simio);

	return(simio.u.unlink.rc);
}

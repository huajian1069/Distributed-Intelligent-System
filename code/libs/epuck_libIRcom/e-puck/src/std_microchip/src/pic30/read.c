#include <p30f6014.h>
#include <stdio.h>
#include "simio.h"

/*----------------------------------------------------------------------*/
/* Special function registers						*/
/*----------------------------------------------------------------------*/
#define	SPEN	0x8000u
#define	TXEN	0x0400u
#define	TRMT	0x0100u
#define	TXBF	0x0200u
#define	RCDA	0x0001u
/*----------------------------------------------------------------------*/
static void __inline__
doread(PSIMIO psimio)
{
  	__asm__(".pword 0xDAC000" :: "r" (psimio));
}
int __attribute__((__weak__, __section__(".libc")))
read(int handle, void *buffer, unsigned int len)
{
	int i;
	SIMIO simio;

	switch (handle)
	{
	case 0:
		if (_Files[0]->_Lockno == 0)
		{
			if ((U1MODE & SPEN) == 0)
			{
				U1BRG = 0;
				U1MODE |= SPEN;
			}
			for (i = len; i; --i)
			{
				int nTimeout;
	
				/*
				** Timeout is 16 cycles per 10-bit char
				*/
				nTimeout = 16*10;
				while (((U1STA & RCDA) == 0) && nTimeout)
				{
					--nTimeout;
				}
				if ((U1STA & RCDA) == 0)
				{
					break;
				}
				*(char*)buffer++ = U1RXREG;
			}
			len -= i;
			break;
		}
	default:
		simio.method = SIM_READ;
		simio.u.read.handle = handle;
		simio.u.read.buffer = buffer;
		simio.u.read.len = len;
		doread(&simio);

		len = simio.u.read.len;
		break;
	}
	return(len);
}

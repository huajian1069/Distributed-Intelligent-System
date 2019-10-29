#include <p30f6014.h>
#include <stdio.h>
#include "simio.h"
/*----------------------------------------------------------------------*/
/* Special function registers						*/
/*----------------------------------------------------------------------*/
#if 0
extern void	__sfr__(int address);	/* SFR address attribute	*/
extern volatile int  __attribute__((__sfr__(0x20C))) U1MOD;
extern volatile int  __attribute__((__sfr__(0x20E))) U1STA;
extern volatile char __attribute__((__sfr__(0x210))) TX1REG;
extern volatile char __attribute__((__sfr__(0x212))) RX1REG;
extern volatile int  __attribute__((__sfr__(0x214))) BRG1;
#endif
#define	SPEN	0x8000u
#define	TXEN	0x0400u
#define	TRMT	0x0100u
#define	TXBF	0x0200u
#define	RCDA	0x0001u
/*----------------------------------------------------------------------*/

static void __inline__
dowrite(PSIMIO psimio)
{
  	__asm__(".pword 0xDAC000" :: "r" (psimio));
}
int __attribute__((__weak__, __section__(".libc")))
write(int handle, void *buffer, unsigned int len)
{
	int i;
	SIMIO simio;

	switch (handle)
	{
	case 0:
	case 1:
	case 2:
		if ((U1MODE & SPEN) == 0)
		{
			U1BRG = 0;
			U1MODE |= SPEN;
		}
		if ((U1STA & TXEN) == 0)
		{
			U1STA |= TXEN;
		}
		for (i = len; i; --i)
		{
			while (U1STA & TXBF)
				;
			U1TXREG = *(char*)buffer++;
		}
		break;
	default:
		simio.method = SIM_WRITE;
		simio.u.write.handle = handle;
		simio.u.write.buffer = buffer;
		simio.u.write.len = len;
		dowrite(&simio);

		len = simio.u.write.len;
		break;
	}
	return(len);
}


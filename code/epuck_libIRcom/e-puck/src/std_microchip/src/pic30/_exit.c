#include <p30f6014.h>

#define	SPEN	0x8000u
#define	TXEN	0x0400u
#define	TRMT	0x0100u

/*
** Stub function called by the exit() standard C library
** function after it has done all it knows how to do.
**
** This function should terminate program execution.
*/

void 
__attribute__((__noreturn__, __weak__, __section__(".libc")))
_exit(int status __attribute__((unused)))
{
	/*
	** Flush stdout
	*/
	if ((U1MODE & SPEN) && (U1STA & TXEN))
	{
		while ((U1STA & TRMT) == 0)
			;
	}
	/*
	** Terminate
	*/
	for ( ; ; )
	{
  		__asm__ volatile (".pword 0xDA4000 ; break");
	}
}

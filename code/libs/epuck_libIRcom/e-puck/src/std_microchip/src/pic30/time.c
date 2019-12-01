#include <p30f6014.h>
#include <time.h>

#define	T2ON	0x8000u
#define	T32	0x0008u

time_t __attribute__((__weak__, __section__(".libc")))
time(time_t *pt)
{
	time_t t;

	if ((T2CON & T2ON) == 0)
	{
		T2CON |= T32;
		T2CON |= T2ON;
	}
	t = ((time_t *)(&TMR2))[0];
	if (pt)
	{
		*pt = t;
	}
	return(t);
}

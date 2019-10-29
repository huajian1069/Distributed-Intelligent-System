#include <stdlib.h>
#include <stdio.h>
int __attribute__((__weak__, __section__(".libc")))
system(const char * s)
{
	if (s != NULL)
	{
		printf("System(%s) called: Aborting\n",s);
  		__asm__ volatile (".pword 0xDA4000 ; break");
	}
	return(-1);
}

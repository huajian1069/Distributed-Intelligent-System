/*
** dsPIC-30 library helper
*/
long
__mulsi3(long A, long B)
{
	__asm__ volatile ("mul.su w1,w2,w4");
	__asm__ volatile ("mul.su w3,w0,w6");
	__asm__ volatile ("mul.uu w0,w2,w0");
	__asm__ volatile ("add w4,w1,w1");
	__asm__ volatile ("add w6,w1,w1");

	return(A);
}


/*
** dsPIC-30 library helper
*/
unsigned long long
__umuldi3(unsigned long long A, unsigned long long B)
{
	int i;
	unsigned long long PP;

	PP = 0;
	for (i = 0; i < 64; ++i)
	{
		if (B & 1)
		{
			PP += A;
		}
		B >>= 1;
		if (PP & 1)
		{
			B |= 0x8000000000000000LL;
		}
		PP >>= 1;
	}
	return(B);
}
long long
__muldi3(long long A, long long B)
{
	if (B < 0)
	{
		B = -B;
		A = -A;
	}
	return(__umuldi3(A, B));
}


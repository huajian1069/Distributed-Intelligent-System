char * __attribute__((__weak__, __section__(".libc")))
getenv(const char * s)
{
  /* no support for environment variables */
  return(0);
}

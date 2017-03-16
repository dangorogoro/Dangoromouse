#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
int _read_r (struct _reent *r, int file, char * ptr, int len)
{
	r = r;
	file = file;
	ptr = ptr;
	len = len;

	errno = EINVAL;
	return -1;
}

int _lseek_r (struct _reent *r, int file, int ptr, int dir)
{
	r = r;
	file = file;
	ptr = ptr;
	dir = dir;

	return 0;
}

int _write_r (struct _reent *r, int file, char * ptr, int len)
{  
	r = r;
	file = file;
	ptr = ptr;

#if 0
	int index;

	/* For example, output string by UART */
	for(index=0; index<len; index++)
	{
		if (ptr[index] == '\n')
		{
			uart_putc('\r');
		}  

		uart_putc(ptr[index]);
	}  
#endif   

	return len;
}

int _close_r (struct _reent *r, int file)
{
	return 0;
}

/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr asm ("sp");

caddr_t _sbrk_r (struct _reent *r, int incr)
{
	extern char   end asm ("end"); /* Defined by the linker.  */
	static char * heap_end;
	char *        prev_heap_end;

	if (heap_end == NULL)
		heap_end = & end;

	prev_heap_end = heap_end;

	if (heap_end + incr > stack_ptr)
	{
		/* Some of the libstdc++-v3 tests rely upon detecting
		 *         out of memory errors, so do not abort here.  */
#if 0
		extern void abort (void);

		_write (1, "_sbrk: Heap and stack collision\n", 32);

		abort ();
#else
		errno = ENOMEM;
		return (caddr_t) -1;
#endif
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _fstat_r (struct _reent *r, int file, struct stat * st)
{
	r = r; 
	file = file;

	memset (st, 0, sizeof (* st));
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty_r(struct _reent *r, int fd)
{
	r = r;
	fd = fd;

	return 1;
}


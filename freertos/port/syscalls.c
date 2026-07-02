/* Newlib retargeting stubs for the bare-metal QEMU target.
 *
 * _write routes stdout/stderr to UART0 (the log console); _sbrk provides a
 * simple bump allocator over the linker-defined heap region so that malloc
 * and printf's internal buffers work.
 */

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "mps2_uart.h"

extern char _heap_bottom;
extern char _heap_top;

void *_sbrk(ptrdiff_t incr)
{
    static char *heap_end = NULL;
    char *prev;

    if (heap_end == NULL)
    {
        heap_end = &_heap_bottom;
    }

    if ((heap_end + incr) > &_heap_top)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    prev = heap_end;
    heap_end += incr;
    return (void *)prev;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; ++i)
    {
        if (ptr[i] == '\n')
        {
            uart_putc(MPS2_UART0_BASE, '\r');
        }
        uart_putc(MPS2_UART0_BASE, ptr[i]);
    }
    return len;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    return 0;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

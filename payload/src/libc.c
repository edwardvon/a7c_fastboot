#include "io.h"

/*
 * The payload is freestanding; these exist because GCC may still turn a loop
 * into a call even with -ffreestanding, and because the ARM EABI runtime
 * helpers want the aeabi spellings.
 */

void *memset(void *dst, int c, unsigned int n)
{
    u8 *d = dst;

    while (n--)
        *d++ = (u8)c;
    return dst;
}

void *memcpy(void *dst, const void *src, unsigned int n)
{
    u8 *d = dst;
    const u8 *s = src;

    while (n--)
        *d++ = *s++;
    return dst;
}

void *memmove(void *dst, const void *src, unsigned int n)
{
    u8 *d = dst;
    const u8 *s = src;

    if (d < s) {
        while (n--)
            *d++ = *s++;
    } else {
        d += n;
        s += n;
        while (n--)
            *--d = *--s;
    }
    return dst;
}

int memcmp(const void *a, const void *b, unsigned int n)
{
    const u8 *p = a, *q = b;

    while (n--) {
        if (*p != *q)
            return (int)*p - (int)*q;
        p++;
        q++;
    }
    return 0;
}

unsigned int strlen(const char *s)
{
    unsigned int n = 0;

    while (s[n])
        n++;
    return n;
}

void *__aeabi_memset(void *dst, unsigned int n, int c) { return memset(dst, c, n); }
void *__aeabi_memcpy(void *dst, const void *src, unsigned int n) { return memcpy(dst, src, n); }
void *__aeabi_memmove(void *dst, const void *src, unsigned int n) { return memmove(dst, src, n); }
void  __aeabi_memclr(void *dst, unsigned int n) { (void)memset(dst, 0, n); }
void  __aeabi_memclr4(void *dst, unsigned int n) { (void)memset(dst, 0, n); }
void  __aeabi_memclr8(void *dst, unsigned int n) { (void)memset(dst, 0, n); }

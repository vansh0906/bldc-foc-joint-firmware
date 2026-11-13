/* The firmware links without a C library: the few memory routines it and the kernel need,
 * plus the ARM EABI aliases compilers emit for struct copies and clears. */
#include <stddef.h>
#include <stdint.h>

void *memcpy(void *d, const void *s, size_t n);
void *memmove(void *d, const void *s, size_t n);
void *memset(void *d, int c, size_t n);
int memcmp(const void *a, const void *b, size_t n);

void *memcpy(void *d, const void *s, size_t n)
{
    uint8_t *o = d;
    const uint8_t *i = s;
    while (n--) *o++ = *i++;
    return d;
}

void *memmove(void *d, const void *s, size_t n)
{
    uint8_t *o = d;
    const uint8_t *i = s;
    if (o < i) while (n--) *o++ = *i++;
    else { o += n; i += n; while (n--) *--o = *--i; }
    return d;
}

void *memset(void *d, int c, size_t n)
{
    uint8_t *o = d;
    while (n--) *o++ = (uint8_t)c;
    return d;
}

int memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *x = a, *y = b;
    for (; n; n--, x++, y++)
        if (*x != *y) return *x - *y;
    return 0;
}

void __aeabi_memcpy(void *d, const void *s, size_t n);
void __aeabi_memcpy4(void *d, const void *s, size_t n);
void __aeabi_memcpy8(void *d, const void *s, size_t n);
void __aeabi_memmove(void *d, const void *s, size_t n);
void __aeabi_memset(void *d, size_t n, int c);
void __aeabi_memclr(void *d, size_t n);
void __aeabi_memclr4(void *d, size_t n);
void __aeabi_memclr8(void *d, size_t n);
void __aeabi_memcpy(void *d, const void *s, size_t n) { memcpy(d, s, n); }
void __aeabi_memcpy4(void *d, const void *s, size_t n) { memcpy(d, s, n); }
void __aeabi_memcpy8(void *d, const void *s, size_t n) { memcpy(d, s, n); }
void __aeabi_memmove(void *d, const void *s, size_t n) { memmove(d, s, n); }
void __aeabi_memset(void *d, size_t n, int c) { memset(d, c, n); }
void __aeabi_memclr(void *d, size_t n) { memset(d, 0, n); }
void __aeabi_memclr4(void *d, size_t n) { memset(d, 0, n); }
void __aeabi_memclr8(void *d, size_t n) { memset(d, 0, n); }

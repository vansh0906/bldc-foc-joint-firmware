/* The target links without a C library (see mem.c); this is the part of <string.h> it uses. */
#ifndef PORT_STRING_H
#define PORT_STRING_H
#include <stddef.h>
void *memcpy(void *d, const void *s, size_t n);
void *memmove(void *d, const void *s, size_t n);
void *memset(void *d, int c, size_t n);
int memcmp(const void *a, const void *b, size_t n);
#endif

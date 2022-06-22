/* This is an example of a header file for platforms/compilers that do
 * not come with stdint.h/stddef.h/stdbool.h/string.h. To use it, define
 * PB_SYSTEM_HEADER as "pb_syshdr.h", including the quotes, and add the
 * extra folder to your include path.
 *
 * It is very likely that you will need to customize this file to suit
 * your platform. For any compiler that supports C99, this file should
 * not be necessary.
 * 
 * KUKA: Added VXWORKS support
 */

#ifndef _PB_SYSHDR_H_
#define _PB_SYSHDR_H_

/* KUKA VxWorks 6.8 support */
#ifdef VXWORKS
   #define HAVE_STRING_H
   #define HAVE_STDLIB_H
#ifdef _WRS_KERNEL
   #include <types/vxTypes.h> // int32_t, int64_t, ...
   #define HAVE_STDINT_H_ALTERNATIVE
   #define HAVE_STDDEF_H_ALTERNATIVE
#else
   #define HAVE_STDINT_H
   #define HAVE_STDDEF_H
#endif // _WRS_KERNEL
#endif // VXWORKS

/* KUKA: size_t is defined in stddef.h, stdlib.h or string.h */
#if !defined(HAVE_STDDEF_H) && !defined(HAVE_STDLIB_H) && !defined(HAVE_STRING_H)
typedef uint32_t size_t;
#endif



/* stdint.h subset */
#ifdef HAVE_STDINT_H
#include <stdint.h>
#else
#ifndef HAVE_STDINT_H_ALTERNATIVE
/* You will need to modify these to match the word size of your platform. */
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed long long int64_t;
typedef unsigned long long uint64_t;
#endif // HAVE_STDINT_H_ALTERNATIVE
#endif // HAVE_STDINT_H

/* stddef.h subset */
#ifdef HAVE_STDDEF_H
#include <stddef.h>
#else
#ifndef HAVE_STDDEF_H_ALTERNATIVE
#define offsetof(st, m) ((size_t)(&((st *)0)->m))
#ifndef NULL
#define NULL 0
#endif // NULL
#endif // HAVE_STDDEF_H_ALTERNATIVE
#endif // HAVE_STDDEF_H

/* stdbool.h subset */
#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#else

#ifndef __cplusplus
typedef int bool;
#define false 0
#define true 1
#endif
   
#endif // HAVE_STDBOOL_H

/* stdlib.h subset */
#ifdef PB_ENABLE_MALLOC
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#else
void *realloc(void *ptr, size_t size);
void free(void *ptr);
#endif // HAVE_STDLIB_H
#endif // PB_ENABLE_MALLOC

/* string.h subset */
#ifdef HAVE_STRING_H
#include <string.h>
#else

/* Implementations are from the Public Domain C Library (PDCLib). */
static size_t strlen( const char * s )
{
    size_t rc = 0;
    while ( s[rc] )
    {
        ++rc;
    }
    return rc;
}

static void * memcpy( void *s1, const void *s2, size_t n )
{
    char * dest = (char *) s1;
    const char * src = (const char *) s2;
    while ( n-- )
    {
        *dest++ = *src++;
    }
    return s1;
}

static void * memset( void * s, int c, size_t n )
{
    unsigned char * p = (unsigned char *) s;
    while ( n-- )
    {
        *p++ = (unsigned char) c;
    }
    return s;
}
#endif // HAVE_STRING_H

#endif // _PB_SYSHDR_H_

/* This is a headerfile customized for Microsoft Visual Studio 2010 
 * - based on example of a header file for platforms/compilers that do
 * not come with stdint.h/stddef.h/stdbool.h/string.h. To use it, define
 * PB_SYSTEM_HEADER as "pb_syshdr_win.h", including the quotes, and add the
 * extra folder to your include path.
 *
 *  Authorship: This file was altered/created by KUKA Roboter GmbH, Augsburg, Germany in 2014 
 *  
 */

#ifndef _PB_SYSHDR_WIN_H_
#define _PB_SYSHDR_WIN_H_

/* stdint.h subset */
#ifdef HAVE_STDINT_H
#include <stdint.h>
#else
/* You will need to modify these to match the word size of your platform. */
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed long long int64_t;
typedef unsigned long long uint64_t;

// from stdint.h
#define INT8_MAX	0x7f
#define INT16_MAX	0x7fff
#define INT32_MAX	0x7fffffff
#define UINT8_MAX	0xffU
#define UINT16_MAX	0xffffU
#define UINT32_MAX	0xffffffffU
#endif

/* stddef.h subset */
#ifdef HAVE_STDDEF_H
#include <stddef.h>
#else

//typedef uint32_t size_t;  // wird dieser typedef wirklich benötigt?!?
#ifndef offsetof
#define offsetof(st, m) ((size_t)(&((st *)0)->m))
#endif

#ifndef NULL
#define NULL 0
#endif

#endif

/* stdbool.h subset */
#ifdef HAVE_STDBOOL_H
#include <stdbool.h>
#else

#ifndef __cplusplus
typedef int bool;
#define false 0
#define true 1
#endif

#endif

/* stdlib.h subset */
#ifdef PB_ENABLE_MALLOC
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#else
void *realloc(void *ptr, size_t size);
void free(void *ptr);
#endif
#endif

/* string.h subset */
#ifdef HAVE_STRING_H
#include <string.h>
#else

#ifndef WIN32
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
#else
#include <string.h>
#endif

#endif

#endif

/* profil.h: gprof profiling header file

   Copyright 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.

This file is part of Cygwin.

This software is a copyrighted work licensed under the terms of the
Cygwin license.  Please consult the file "CYGWIN_LICENSE" for
details. */

/*
 * This file is taken from Cygwin distribution. Please keep it in sync.
 * The differences should be within __MINGW32__ guard.
 */

#ifndef __PROFIL_H__
#define __PROFIL_H__

/* profiling frequency.  (No larger than 1000) */
#define PROF_HZ			1000
#define __ASM __asm /*!< asm keyword for GNU Compiler */
#define __INLINE inline /*!< inline keyword for GNU Compiler */
#define __STATIC_INLINE static inline
/* convert an addr to an index */
#define PROFIDX(pc, base, scale)	\
  ({									\
    size_t i = (pc - base) / 2;				\
    if (sizeof (unsigned long long int) > sizeof (size_t))		\
      i = (unsigned long long int) i * scale / 65536;			\
    else								\
      i = i / 65536 * scale + i % 65536 * scale / 65536;		\
    i;									\
  })

/* convert an index into an address */
#define PROFADDR(idx, base, scale)		\
  ((base)					\
   + ((((unsigned long long)(idx) << 16)	\
       / (unsigned long long)(scale)) << 1))

/* convert a bin size into a scale */
#define PROFSCALE(range, bins)		(((bins) << 16) / ((range) >> 1))

typedef void *_WINHANDLE;

typedef enum {
  PROFILE_NOT_INIT = 0,
  PROFILE_ON,
  PROFILE_OFF
} PROFILE_State;

struct profinfo {
  PROFILE_State state; /* profiling state */
  u_short *counter;			/* profiling counters */
  size_t lowpc, highpc;		/* range to be profiled */
  u_int scale;			/* scale value of bins */
};

int profile_ctl(struct profinfo *, char *, size_t, size_t, u_int);
int profil(char *, size_t, size_t, u_int);
extern char Samples[];

/**
\brief Get Link Register
\details Returns the current value of the Link Register (LR).
\return LR Register value
*/
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_LR(void)
{
  register uint32_t result;

  __ASM volatile ("MOV %0, LR\n" : "=r" (result) );
  return(result);
}

#endif /* __PROFIL_H__ */

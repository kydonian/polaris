/***************************************************************

typedefs.h
Nathan Honka
1/13/2012

Type definitions for winch saver project.

***************************************************************/

#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

// Specify if the compiler is using the Embedded Application Binary
// Interface (EABI), which is indicated by the compiler option
// --abi=eabi.  If the other (default) option is specified, --abi=coff,
// do not define this macro.

// Why?  EABI allows us to use 64-bit floats.  COFF ABI only allows
// 32-bit, but the types are defined differently.
#define EABI

#ifdef EABI

typedef unsigned char			bool;
typedef unsigned char 			uint8;
typedef signed char 			int8;
typedef unsigned int			uint16;
typedef signed int				int16;
typedef unsigned long int		uint32;
typedef signed long int			int32;
typedef unsigned long long int	uint64;
typedef signed long long int	int64;

#else		// #ifdef EABI

typedef unsigned char	bool;
typedef unsigned char 	uint8;
typedef signed char 	int8;
typedef unsigned int	uint16;
typedef signed int		int16;
typedef unsigned long	uint32;
typedef signed long		int32;

#endif		// #ifdef EABI

#define TRUE 	(1==1)
#define FALSE 	(1==0)

#endif //__TYPEDEFS_H__

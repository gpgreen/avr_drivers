#ifndef DRIVER_H_
#define DRIVER_H_

#include <stdio.h>

#ifdef __AVR

#include <avr/pgmspace.h>

#define DEVICE_PRINT(MSG) (printf_P(PSTR("%s" MSG "\n"), k_name))
#define DEVICE_PRINT2(MSG, ...) (printf_P(PSTR("%s" MSG "\n"), k_name, __VA_ARGS__))

#else

#undef PDEBUG
#ifdef UM6_DEBUG
#  define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#endif

#endif

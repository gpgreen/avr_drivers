##########################################################################
# "THE ANY BEVERAGE-WARE LICENSE" (Revision 42 - based on beer-ware
# license):
# <dev@layer128.net> wrote this file. As long as you retain this notice
# you can do whatever you want with this stuff. If we meet some day, and
# you think this stuff is worth it, you can buy me a be(ve)er(age) in 
# return. (I don't like beer much.)
# 
# Matthias Kleemann
##########################################################################

set(avrcanlib_SOURCES
  canaero.c
  canaero_defs.h
  canaero.h
  can.c
  can.h
  can_delays.h
)

if (CANAERO)
   list(APPEND avrcanlib_SOURCES
     canaero_bss.c
     canaero_bss.h
     canaero_filters.c
     canaero_filters.h
     canaero_ids.c
     canaero_ids.h
     canaero_nis.c
     canaero_nis.h
   )
endif(CANAERO)

if (NMEA_2000)
  list(APPEND avrcanlib_SOURCES
    nmea2000.h
    nmea2000.c
    )
endif(NMEA_2000)

add_avr_library(avrcanlib ${avrcanlib_SOURCES})


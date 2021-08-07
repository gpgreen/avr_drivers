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

set(avrdrivers_SOURCES
  debounce.c
  debounce.h
  spi.c
  spi.h
  timer.c
  timer.h
  timer1.c
  timer1.h
  uart.c
  uart_def.h
  uart.h
  driver.h
)

if (AHRS_FIRMWARE)
   list(APPEND avrdrivers_SOURCES
     adxl345.c
     adxl345_def.h
     adxl345.h
     at90can.c
     at90can_def.h
     at90can.h
     bmp085.c
     bmp085_def.h
     bmp085.h
     l3g4200d.c
     l3g4200d_def.h
     l3g4200d.h
     watchdog.c
     watchdog.h
     )
endif(AHRS_FIRMWARE)

if (COMPASS_FIRMWARE)
  list(APPEND avrdrivers_SOURCES
    hmc5883l.c
    hmc5883l_def.h
    hmc5883l.h
  )
endif(COMPASS_FIRMWARE)

if (CANBUS_SHIELD_FIRMWARE)
  list(APPEND avrdrivers_SOURCES
    canserial.c
    canserial.h
    mcp2515.c
    mcp2515_def.h
    mcp2515.h
    )
endif(CANBUS_SHIELD_FIRMWARE)

if (PYPILOT_CONTROLLER_FIRMWARE)
  list(APPEND avrdrivers_SOURCES
    mcp2515.c
    mcp2515_def.h
    mcp2515.h
    watchdog.c
    watchdog.h
    )
endif(PYPILOT_CONTROLLER_FIRMWARE)

# Drivers not in default set
#   adxl345.c
#   adxl345_def.h
#   adxl345.h
#   at90can.c
#   at90can_def.h
#   at90can.h
#   bmp085.c
#   bmp085_def.h
#   bmp085.h
#   canserial.c
#   canserial.h
#   hmc5843.c
#   hmc5843_def.h
#   hmc5843.h
#   hmc5883l.c
#   hmc5883l_def.h
#   hmc5883l.h
#   l3g4200d.c
#   l3g4200d_def.h
#   l3g4200d.h
#   mcp2515.c
#   mcp2515_def.h
#   mcp2515.h
#   pca9545.c
#   pca9545_def.h
#   pca9545.h
#   scp1000.c
#   scp1000_def.h
#   scp1000.h
#   timer2.c
#   timer2.h
#   um6.c
#   um6_def.h
#   um6.h

add_avr_library(avrdrivers ${avrdrivers_SOURCES})

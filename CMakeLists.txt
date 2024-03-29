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

cmake_minimum_required(VERSION 2.8)

# build for the 'ahrs' firmware
option(AHRS_FIRMWARE "build for ahrs firmware" OFF)

# build for the 'compass' firmware
option(COMPASS_FIRMWARE "build for compass firmware" OFF)

# build for the 'pypilot-controller' firmware
option(PYPILOT_CONTROLLER_FIRMWARE "build for pypilot-controller firmware" OFF)

# add canaero routines to library
option(CANAERO "include canaero functions" OFF)

# add nmea2000 routines to library
option(NMEA_2000 "include nmea2000 functions" OFF)

### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior project() call. In that
# case they are already set and used.
##########################################################################

##########################################################################
# tools to be used for programming the AVR
##########################################################################
set(AVR_UPLOADTOOL avrdude)
set(AVR_PROGRAMMER usbtiny)
set(AVR_UPLOADTOOL_PORT usb)

##########################################################################
# AVR, clock speed and fuses needs to be set
##########################################################################

set(AVR_USE_E_FUSE on)

if (AHRS_FIRMWARE)
  set(MCU_SPEED "8000000UL")
  set(AVR_MCU at90can32)
  set(AVRDUDE_MCU c32)
  set(AVR_E_FUSE 0xfb)
  set(AVR_H_FUSE 0xd9)
  set(AVR_L_FUSE 0x5e)
endif(AHRS_FIRMWARE)

if (COMPASS_FIRMWARE)
  set(MCU_SPEED "16000000UL")
  set(AVR_MCU atmega644)
  set(AVRDUDE_MCU m44)
  set(AVR_E_FUSE 0xfc)
  set(AVR_H_FUSE 0xd9)
  set(AVR_L_FUSE 0x27)
endif(COMPASS_FIRMWARE)

if (CANBUS_SHIELD_FIRMWARE)
  set(MCU_SPEED "16000000UL")
  set(AVR_MCU atmega328p)
  set(AVRDUDE_MCU m328p)
  set(AVR_E_FUSE 0xfc)
  set(AVR_H_FUSE 0xd9)
  set(AVR_L_FUSE 0x27)
  set(AVR_PROGRAMMER stk500v1)
endif(CANBUS_SHIELD_FIRMWARE)

if (PYPILOT_CONTROLLER_FIRMWARE)
  set(MCU_SPEED "8000000UL")
  set(AVR_MCU atmega328p)
  set(AVRDUDE_MCU m328p)
  set(AVR_E_FUSE 0xfc)
  set(AVR_H_FUSE 0xd9)
  set(AVR_L_FUSE 0x27)
  set(AVR_PROGRAMMER stk500v1)
endif(PYPILOT_CONTROLLER_FIRMWARE)

### END TOOLCHAIN SETUP AREA #############################################

# Intentionally left blank, due to a different approach of using the
# toolchain file via -DCMAKE_TOOLCHAIN_FILE=path/to/generic-gcc-avr.cmake
# at the cmake command line call
#
# include(../generic-gcc-avr.cmake)

##########################################################################
# name your project
##########################################################################
project(avr_drivers)

##########################################################################
# status messages
##########################################################################
message(STATUS "Current uploadtool is: ${AVR_UPLOADTOOL}")
message(STATUS "Current programmer is: ${AVR_PROGRAMMER}")
message(STATUS "Current upload port is: ${AVR_UPLOADTOOL_PORT}")
message(STATUS "Current uploadtool options are: ${AVR_UPLOADTOOL_OPTIONS}")
message(STATUS "Current MCU is set to: ${AVR_MCU}")
message(STATUS "Current Programmer MCU is set to: ${AVRDUDE_MCU}")
message(STATUS "Current use E fuse is set to: ${AVR_USE_E_FUSE}")
message(STATUS "Current E_FUSE is set to: ${AVR_E_FUSE}")
message(STATUS "Current H_FUSE is set to: ${AVR_H_FUSE}")
message(STATUS "Current L_FUSE is set to: ${AVR_L_FUSE}")

##########################################################################
# set build type
##########################################################################
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

##########################################################################
# some cmake cross-compile necessities
##########################################################################
if(DEFINED ENV{AVR_FIND_ROOT_PATH})
    set(CMAKE_FIND_ROOT_PATH $ENV{AVR_FIND_ROOT_PATH})
else(DEFINED ENV{AVR_FIND_ROOT_PATH})
    if(EXISTS "/opt/local/avr")
      set(CMAKE_FIND_ROOT_PATH "/opt/local/avr")
    elseif(EXISTS "/usr/avr")
      set(CMAKE_FIND_ROOT_PATH "/usr/avr")
    elseif(EXISTS "/usr/lib/avr")
      set(CMAKE_FIND_ROOT_PATH "/usr/lib/avr")
    elseif(EXISTS "/usr/local/CrossPack-AVR")
      set(CMAKE_FIND_ROOT_PATH "/usr/local/CrossPack-AVR")
    else(EXISTS "/opt/local/avr")
      message(FATAL_ERROR "Please set AVR_FIND_ROOT_PATH in your environment.")
    endif(EXISTS "/opt/local/avr")
endif(DEFINED ENV{AVR_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# not added automatically, since CMAKE_SYSTEM_NAME is "generic"
set(CMAKE_SYSTEM_INCLUDE_PATH "${CMAKE_FIND_ROOT_PATH}/include")
set(CMAKE_SYSTEM_LIBRARY_PATH "${CMAKE_FIND_ROOT_PATH}/lib")

##########################################################################
# status messages for generating
##########################################################################
message(STATUS "Set CMAKE_FIND_ROOT_PATH to ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "Set CMAKE_SYSTEM_INCLUDE_PATH to ${CMAKE_SYSTEM_INCLUDE_PATH}")
message(STATUS "Set CMAKE_SYSTEM_LIBRARY_PATH to ${CMAKE_SYSTEM_LIBRARY_PATH}")

##########################################################################
# set compiler options for build types
##########################################################################
if(CMAKE_BUILD_TYPE MATCHES Release)
   set(CMAKE_C_FLAGS_RELEASE "-Os")
   set(CMAKE_CXX_FLAGS_RELEASE "-Os")
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
   set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
endif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)

if(CMAKE_BUILD_TYPE MATCHES Debug)
   set(CMAKE_C_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
   set(CMAKE_CXX_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
endif(CMAKE_BUILD_TYPE MATCHES Debug)

##########################################################################
# compiler options for all build types
##########################################################################
add_definitions("-DF_CPU=${MCU_SPEED}")
add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
add_definitions("-Wall")
#add_definitions("-Werror")
add_definitions("-pedantic")
add_definitions("-pedantic-errors")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-ffunction-sections")
add_definitions("-c")
add_definitions("-std=gnu99")

##########################################################################
# setup build for given firmware
##########################################################################
# build of the libraries for a given firmware require a path
# to that projects 'defs.h'
# default set it to empty
set(PROJECT_DEF_PATH "")

# set the include path to find 'ahrs/def.h'
if(AHRS_FIRMWARE)
  set(PROJECT_DEF_PATH "${CMAKE_SOURCE_DIR}/../../ahrs")
endif(AHRS_FIRMWARE)

# set the include path to find 'compass/def.h'
if(COMPASS_FIRMWARE)
  set(PROJECT_DEF_PATH "${CMAKE_SOURCE_DIR}/../../compass")
endif(COMPASS_FIRMWARE)

# set the include path to find 'canbus-shield/def.h'
if(CANBUS_SHIELD_FIRMWARE)
  set(PROJECT_DEF_PATH "${CMAKE_SOURCE_DIR}/../../canbus-shield")
endif(CANBUS_SHIELD_FIRMWARE)

# set the include path to find 'pypilot-controller/def.h'
if(PYPILOT_CONTROLLER_FIRMWARE)
  set(PROJECT_DEF_PATH "${CMAKE_SOURCE_DIR}/../../pypilot-controller")
endif(PYPILOT_CONTROLLER_FIRMWARE)

##########################################################################
# include search paths
##########################################################################
include_directories(
	"${PROJECT_DEF_PATH}"
	./canlibrary
	./drivers
	./i2cmaster
	./utils
)

##########################################################################
# building library and application in their subdirectories
##########################################################################
if(NOT COMPASS_FIRMWARE)
  add_subdirectory(canlibrary)
endif(NOT COMPASS_FIRMWARE)
add_subdirectory(drivers)
add_subdirectory(i2cmaster)
add_subdirectory(utils)

##########################################################################
# testing functions w/o source files - gets FATAL_ERROR
##########################################################################
# add_avr_executable(test_breaks_generation)
# add_avr_library(mytestlib_breaks_generation)

##########################################################################
# use default documentation target
##########################################################################
#set(DOXYGEN_CONF_IN "doxygen.conf")
#include("${PROJECT_SOURCE_DIR}/../Modules/defaultDocuTarget.cmake")



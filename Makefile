# This is the name of the file that shall be created. (It is also the name of my primary source file, without the file extension.)
TARGET = fanController

F_CPU = 8000000
MCU = attiny85

AVRDUDE_PROGRAMMER=dragon_isp
AVRDUDE_PORT=usb

NO_ARDUINO = true

# create a variable that only pertains to this project
MY_OWN_LIBRARY_DIR = 

# "EXTRAINCDIRS" is a variable used by the base makefile. The base makefile creates a -I compiler flag for every item in the "EXTRAINCDIRS" list.
EXTRAINCDIRS = $(MY_OWN_LIBRARY_DIR)

# specify *.S source files pertaining to my project
ASRC = 

# specify *.c source files pertaining to my project
SRC = fanController.c

# specify *.cpp source files pertaining to my project
PSRC = 

# specify additional (non-core) Arduino libraries to include
ARDLIBS = 

# include my base makefile
include AVR-makefile-base.mk


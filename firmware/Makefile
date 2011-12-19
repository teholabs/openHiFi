#******************************************************************************
# makefile for openHiFi project
# teho Labs/B. A. Bryce
# this makefile is designed to match the StellarisWare standard makefiles
# Please see the readme for licence details
#******************************************************************************

#Program name
NAME = openhifi

# Defines the part type that this project uses.
PART=PART_LM3S9B90

#include the toolchain settings 
include $.../toolchainsettings

#For readablity we repeat the contents of toolchainsettings here:

##start toolchain settings
##COMPORT should be COMx or /dev/ttyUSBx or similar for windows and linux respectivly
#COMPORT = /dev/ttyS3
#SERIALPROG = 0
#WINDOWS = 0

##Use forward slashes for the paths
##DIR_STELLARISWARE is typically C:/Users/USERNAME/Development/StellarisWare or /home/USERNAME/Development/StellarisWare
#DIR_STELLARISWARE = /home/USERNAME/Development/StellarisWare
#DIR_DRIVERLIB = $(DIR_STELLARISWARE)/driverlib
#DIR_USBLIB = $(DIR_STELLARISWARE)/usblib
#DIR_UTILS = $(DIR_STELLARISWARE)/utils

##end toolchain settings


#Toolchain Command Config

os:=${shell uname -s}

CYGWIN = 0
ifneq ($(findstring CYGWIN, ${os}), )
	CYGWIN = 1
endif

ifeq ($(SERIALPROG),1)
	LINKER = bootlink.ld
	ifeq ($(WINDOWS),1)
		FLASHCMD = lmflash -q manual -i serial -p $(COMPORT) -b 115200 -r --offset=0x1000 --xfer-size=32 $(NAME).bin
	else
		FLASHCMD = sflash -p 0x1000 -c $(COMPORT) -b 115200 -s 32 $(NAME).bin
	endif
else
	LINKER = link.ld

	ifeq ($(WINDOWS),1)
		FLASHCMD = perl ./do_flash.pl $(NAME).bin
	else 
		FLASHCMD = ./do_flash.pl $(NAME).bin
	endif

endif


ifeq ($(WINDOWS),1)	
	ifeq ($(CYGWIN),1)
		CLEANCMD = rm -rf ${COMPILER} ${wildcard *~}
		COPYCMD = cp ./${COMPILER}/$(NAME).bin ./
	else
		CLEANCMD = del /s /q ${COMPILER} ${wildcard *~}
		COPYCMD = copy ./${COMPILER}/$(NAME).bin ./
	endif	
else 
	CLEANCMD = rm -rf ${COMPILER} ${wildcard *~}
	COPYCMD = cp ./${COMPILER}/$(NAME).bin ./
endif


#Needed for standard StellarisWare makefile def
ROOT=$(DIR_STELLARISWARE)

#Common StellarisWare makefile definitions
include ${ROOT}/makedefs

# Source files not in local directory
VPATH=./fatfs/src

# Header files not in local directory
IPATH=$(DIR_STELLARISWARE)
IPATH+=./fatfs/src
IPATH+=./flac

# "make all"
all: ${COMPILER}
all: ${COMPILER}/openhifi.axf

# "make clean"
clean:
	$(CLEANCMD)

# "make flash"
flash: all 
	$(COPYCMD)
	$(FLASHCMD)

#command for make all
${COMPILER}:
	@mkdir ${COMPILER}

# Build the project out of these files/settings/defines
${COMPILER}/openhifi.axf: ${COMPILER}/xprintf.o
${COMPILER}/openhifi.axf: ${COMPILER}/ff.o
${COMPILER}/openhifi.axf: ${COMPILER}/fat_usbmsc.o
${COMPILER}/openhifi.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/openhifi.axf: ${COMPILER}/openhifi.o
${COMPILER}/openhifi.axf: ${ROOT}/usblib/${COMPILER}-cm3/libusb-cm3.a
${COMPILER}/openhifi.axf: ${ROOT}/driverlib/${COMPILER}-cm3/libdriver-cm3.a
${COMPILER}/openhifi.axf: ./flac/flaclib.a
${COMPILER}/openhifi.axf: $(LINKER)
SCATTERgcc_openhifi=$(LINKER)
ENTRY_openhifi=ResetISR
CFLAGSgcc=-DTARGET_IS_TEMPEST_RC5 -DBUILD_STANDALONE -D$(PART)

# Include the automatically generated dependency files
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif


# Project Name
TARGET = ouroboros

USE_DAISYSP_LGPL = 1

APP_TYPE=BOOT_QSPI
# without BOOT_SRAM: make program
# with BOOT_SRAM:
# hold boot, press reset and then
# > make program-boot
# press reset and then quickly press boot
# > make program-dfu

# Sources
CPP_SOURCES = main.cpp \
	lib/resampler.cpp \
	lib/tape.cpp \
	lib/tapehead.cpp \
	lib/balance2.cpp \
	lib/lfo.cpp \
	lib/lpf_biquad.cpp \
	lib/compressor.cpp \
	lib/daisy_midi.cpp \
	lib/wavheader.cpp \
	lib/utils.cpp \
	lib/noise.cpp \
	lib/chords.cpp
	
# C_SOURCES = audio.c

# Library Locations
LIBDAISY_DIR = libDaisy
DAISYSP_DIR = DaisySP
USE_FATFS = 1
LDFLAGS += -u _printf_float

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

.venv:
	uv venv 
	uv pip install -r requirements.txt

lib/fverb3.h:
	python3 dev/faust/faust.py --sram dev/faust/fverb3/fverb3.dsp fverb3.cpp FVerb3 
	mv fverb3.cpp lib/fverb3.h

# Project Name
TARGET = HelloDaisy3

# DEBUG = 1 / OPT = -Og for debugging, DEBUG = 0 / OPT = -O3 for performance
DEBUG = 1
OPT = -Og

# (optional) use the bootloader
# APP_TYPE=BOOT_SRAM

# Sources
CPP_SOURCES = src/main.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP
# USE_DAISYSP_LGPL=1

# Includes FatFS source files within project.
USE_FATFS = 1

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

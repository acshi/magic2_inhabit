.SUFFIXES:
.SUFFIXES:	.c .o

export ROOT_PATH    := $(CURDIR)
export APRIL_PATH   := $(ROOT_PATH)/april2
export BIN_PATH     := $(ROOT_PATH)/bin
export LIB_PATH     := $(ROOT_PATH)/lib
export BUILD_COMMON := $(ROOT_PATH)/Build.common

export CONFIG_DIR   := $(ROOT_PATH)/config
export SRC_PATH     := $(ROOT_PATH)/src
export LCM_PATH     := $(ROOT_PATH)/src/lcmtypes


### Build flags for all targets
#
export CFLAGS_STD := -std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable \
		-Wno-format-zero-length -pthread -fPIC -Werror \
		-I$(ROOT_PATH)/src -I$(ROOT_PATH)/april2/src
export LDFLAGS_STD :=
export DEPS_STD    :=

export CFLAGS_LCM  := `pkg-config --cflags lcm`
export LDFLAGS_LCM := `pkg-config --libs lcm`
export DEPS_LCM    :=

export CFLAGS_GTK  := `pkg-config --cflags gtk+-2.0`
export LDFLAGS_GTK := `pkg-config --libs gtk+-2.0`
export DEPS_GTK    :=

# libusb
export CFLAGS_USB := `pkg-config --cflags libusb-1.0`
export LDFLAGS_USB := `pkg-config --libs libusb-1.0`
export DEPS_USB    :=

# libpng
export CFLAGS_PNG := `pkg-config --cflags libpng`
export LDFLAGS_PNG := `pkg-config --libs libpng`



MAKE := $(MAKE) --no-print-directory


# more dependencies will be added to all and clean later
all:

clean: clean-targets

clean-targets:
	@rm -f bin/* lib/*

# This begins the recursive decent crawl over all the Rules.mk files.
# Add additional roots here as necessary.
include $(ROOT_PATH)/Rules.mk

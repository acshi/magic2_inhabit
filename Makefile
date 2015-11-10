
.SUFFIXES:
.SUFFIXES:	.c .o

ROOT_PATH   := $(CURDIR)
SRC_PATH    := $(ROOT_PATH)/src
BIN_PATH    := $(ROOT_PATH)/bin
LIB_PATH    := $(ROOT_PATH)/lib
CONFIG_DIR  := $(ROOT_PATH)/config
LCM_PATH     := $(ROOT_PATH)/src/lcmtypes

### Build flags for all targets
#
CFLAGS_STD  := 	-std=gnu99 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE \
		-D_REENTRANT -Wall -Wno-unused-parameter -Wno-unused-variable -Wno-format-zero-length \
		-pthread -fPIC -I src/ -I april2/src/ -Werror
LDFLAGS_STD   :=
LINK_LIBS_STD := -lpthread -lz -lm

### Build tools
#
CC              = gcc
AR              = @echo "[$(notdir $(@))]" && ar rc $@ $^
COMP            = @echo "	$@" 	   && $(CC) $(CFLAGS_STD) $(CFLAGS_TGT) -o $@ -c $<
LINK            = @echo "[$(notdir $(@))]" && $(CC) $(LDFLAGS_STD) $(LDFLAGS_TGT) -o $@ $^ $(LDFLAGS_TGT) $(LINK_LIBS_STD) $(LINK_LIBS_TGT)
COMPLINK        = @echo "[$(notdir $(@))]" && $(CC) $(CFLAGS_STD) $(CFLAGS_TGT) $(LDFLAGS_STD)  -o $@ $^ $(LINK_LIBS_TGT) $(LINK_LIBS_STD) $(LDFLAGS_TGT)
COMPLINKCPP     = @echo "[$(notdir $(@))]" && g++ $(CFLAGS_TGT) -o $@ $^ $(LINK_LIBS_TGT) $(LINK_LIBS_STD) $(LDFLAGS_TGT)

all:		targets

D	:= 	$(CURDIR)
include		Rules.mk

.PHONY:		targets
targets:	$(TGTS)

.PHONY:		clean
clean:
	@rm -f $(CLEAN)

%.o: %.c %.h
	$(COMP)

%.o: %.c
	$(COMP)

%: %.c
	$(COMPLINK)

%.o: %.cpp
	$(COMPLINKCPP)

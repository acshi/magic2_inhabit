#These ensure unique names.
sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)

# List all objects here, or use
# SRC_$(d)	:= $(shell ls $(d)/*.c)
# OBJS_$(d)	:= $(SRC_$(d):%.c=%.o)
OBJS_$(d)	:= $(d)/main.o

# List all targets, it's important for building and cleaning
TGTS_$(d)	:= $(BIN_PATH)/example-app

# List CFLAGS used when .o are compiled
$(OBJS_$(d)):   CFLAGS_TGT :=

# List LDFlags and libs youn depend on here
$(TGTS_$(d)):   LDFLAGS_TGT := $(LDFLAGS_LCM)
$(TGTS_$(d)):   $(LIBVX) $(LIBHTTPD) $(LIBMAGICLCM) $(LIBCOMMON)

# If more than one executable, you'll need to do something like:
# $(BIN_PATH)/example-app: $(d)/main.o
#  as well as possibly specify more libs to depend on
$(TGTS_$(d)):   $(OBJS_$(d))
	$(LINK)

# If it doesn't make it into TGTS, it won't build
TGTS		:= $(TGTS) $(TGTS_$(d))
CLEAN		:= $(CLEAN) $(TGTS_$(d)) $(OBJS_$(d))

# Don't mess w/ these
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))

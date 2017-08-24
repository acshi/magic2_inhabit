
export CFLAGS_LCMTYPES  = $(CFLAGS_APRIL_LCMTYPES)
export LDFLAGS_LCMTYPES = $(LIB_PATH)/libmagiclcmtypes.a $(LDFLAGS_APRIL_LCMTYPES)
export DEPS_LCMTYPES = $(LIB_PATH)/libmagiclcmtypes.a $(DEPS_APRIL_LCMTYPES)

.PHONY: local_lcmtypes local_lcmtypes_clean

local_lcmtypes: lcmtypes april_lcmtypes

local_lcmtypes:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lcmtypes -f Build.mk

local_lcmtypes_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/lcmtypes -f Build.mk clean

all: local_lcmtypes

clean: local_lcmtypes_clean

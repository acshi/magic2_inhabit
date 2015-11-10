
LCMTYPES  	:= $(shell ls $(D)/../../lcmtypes/*.lcm)
SRC_$(D)	:= $(LCMTYPES:%.lcm=%.c)
SRC_$(D)	:= $(notdir $(SRC_$(D)))
SRC_$(D) 	:= $(addprefix $(D)/,$(SRC_$(D)))
H_$(D)		:= $(SRC_$(D):%.c=%.h)

bd_$(D) = $(realpath $(D)/../../)

$(bd_$(D))/src/lcmtypes/%.c: $(bd_$(D))/lcmtypes/%.lcm
	@echo "    $<"
	lcm-gen -c --c-typeinfo --c-cpath $(dir $@) --c-hpath $(dir $@) --cinclude lcmtypes/ $<

$(bd_$(D))/src/lcmtypes/%.h: $(bd_$(D))/lcmtypes/%.lcm
	@echo "    $<"
	lcm-gen -c --c-typeinfo --c-cpath $(dir $@) --c-hpath $(dir $@) --cinclude lcmtypes/ $<

OBJS_$(D)	:= $(SRC_$(D):%.c=%.o)

$(OBJS_$(D)): $(SRC_$(D)) $(H_$(D))

LIBLCMTYPES_$(D) = $(LIB_PATH)/libtemplatelcmtypes.a
LIBLCMTYPES_SO_$(D) = $(LIB_PATH)/libtemplatelcmtypes.so

LDFLAGS_TEMPLATELCMTYPES = $(LIBLCMTYPES_$(D))

$(LIBLCMTYPES_$(D)): $(OBJS_$(D))
	        @echo "    $@"
		@ar rc $@ $^

$(LIBLCMTYPES_SO_$(D)): $(OBJS_$(D))
	        @echo "    $@"
		@gcc -shared -o $@ $^

TGTS		:= $(TGTS) $(SRC_$(D)) $(LIBLCMTYPES_$(D)) $(LIBLCMTYPES_SO)
CLEAN		:= $(CLEAN) $(OBJS_$(D))

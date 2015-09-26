sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)

LCMTYPES  	:= $(shell ls $(d)/../../lcmtypes/*.lcm)
SRC_$(d)	:= $(LCMTYPES:%.lcm=%.c)
SRC_$(d)	:= $(notdir $(SRC_$(d)))
SRC_$(d) 	:= $(addprefix $(d)/,$(SRC_$(d)))
H_$(d)		:= $(SRC_$(d):%.c=%.h)

bd_$(d) = $(dirstack_$(basename $(sp)))

$(bd_$(d))/src/lcmtypes/%.c: $(bd_$(d))/lcmtypes/%.lcm
	@echo "    $@"
	@lcm-gen -c --c-typeinfo --c-cpath $(dir $@) --c-hpath $(dir $@) --cinclude lcmtypes/ $<

$(bd_$(d))/src/lcmtypes/%.h: $(bd_$(d))/lcmtypes/%.lcm
	@echo "    $@"
	@lcm-gen -c --c-typeinfo --c-cpath $(dir $@) --c-hpath $(dir $@) --cinclude lcmtypes/ $<

OBJS_$(d)	:= $(SRC_$(d):%.c=%.o)

$(OBJS_$(d)): $(SRC_$(d)) $(H_$(d))


######################################################
# 	TODO XXX
# ###################################################
# list other *lcmtypes.a you depend on
$(OBJS_$(d)): $(LIB_PATH)/libaprillcmtypes.a

# TODO Define lib(project name)lcmtypes.a
LIBLCMTYPES_$(d) = $(LIB_PATH)/libTMPLlcmtypes.a
LIBLCMTYPES_SO_$(d) = $(LIB_PATH)/libTMPLlcmtypes.so

######## END #########################################

$(LIBLCMTYPES_$(d)): $(OBJS_$(d))
	$(AR)

$(LIBLCMTYPES_SO_$(d)): $(OBJS_$(d))
	$(AR)

TGTS		:= $(TGTS) $(SRC_$(d)) $(LIBLCMTYPES_$(d)) $(LIBLCMTYPES_SO)
CLEAN		:= $(CLEAN) $(OBJS_$(d))



d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))

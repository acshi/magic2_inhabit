sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)
# MODIFY BELOW THIS LINE

dir	:= $(d)/template_example
include		$(dir)/Rules.mk

dir	:= $(d)/lcmtypes
include		$(dir)/Rules.mk

# MODIFY ABOVE THIS LINE
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))


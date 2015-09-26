sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)
# MODIFY BELOW THIS LINE

#Submodules
dir	:= $(d)/april2
include		$(dir)/Rules.mk

dir	:= $(d)/src
include		$(dir)/Rules.mk

# MODIFY ABOVE THIS LINE
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))


#Submodules
D	:= $(D)/april2
include		$(D)/Rules.mk
D       := $(realpath $(dir $(D)))

D	:= $(D)/src
include		$(D)/Rules.mk
D       := $(realpath $(dir $(D)))


.PHONY: template template_clean

template: lcmtypes graph vx

template:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/template_example -f Build.mk

template_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/template_example -f Build.mk clean

all: template

clean: template_clean

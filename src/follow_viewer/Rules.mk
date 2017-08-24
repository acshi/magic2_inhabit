.PHONY: follow_viewer follow_viewer_clean

follow_viewer: lcmtypes graph vx

follow_viewer:
	@echo $@
	@echo c flags $(CFLAGS_VX)
	@$(MAKE) -C $(REAL_ROOT_PATH)/src/follow_viewer -f Build.mk

follow_viewer_clean:
	@echo $@
	@$(MAKE) -C $(REAL_ROOT_PATH)/src/follow_viewer -f Build.mk clean

all: follow_viewer

clean: follow_viewer_clean

.PHONY: tag_follower tag_follower_clean

tag_follower: lcmtypes graph vx

tag_follower:
	@echo $@
	@$(MAKE) -C $(REAL_ROOT_PATH)/src/tag_follower -f Build.mk

tag_follower_clean:
	@echo $@
	@$(MAKE) -C $(REAL_ROOT_PATH)/src/tag_follower -f Build.mk clean

all: tag_follower

clean: tag_follower_clean

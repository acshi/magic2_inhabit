SRCS = $(shell ls *.c)
OBJS = $(SRCS:%.c=%.o)
TARGET = $(BIN_PATH)/tag_follower

CFLAGS := $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_LCMTYPES)
LDFLAGS := $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_LCMTYPES)
DEPS := $(DEPS_STD) $(DEPS_VX) $(DEPS_LCMTYPES)

include $(BUILD_COMMON)


all: $(TARGET)
	@/bin/true

$(TARGET): $(OBJS) $(DEPS)
	@$(LD) -o $@ $^ $(LDFLAGS)

clean:
	@rm -rf *.o $(OBJS) $(TARGET)

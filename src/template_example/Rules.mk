
OBJS_$(D)	:= $(D)/main.o
TGTS_$(D)	:= $(BIN_PATH)/template_bin

$(TGTS_$(D)):   CFLAGS_TGT := $(CFLAGS_APRIL)
$(TGTS_$(D)):   LDFLAGS_TGT := $(LDFLAGS_LCM)
$(TGTS_$(D)):   $(LIBVX) $(LIBHTTPD) $(LIBLCM) $(LIBCOMMON)

$(TGTS_$(D)):   $(OBJS_$(D))
	$(LINK)

TGTS		:= $(TGTS) $(TGTS_$(D))
CLEAN		:= $(CLEAN) $(TGTS_$(D)) $(OBJS_$(D))

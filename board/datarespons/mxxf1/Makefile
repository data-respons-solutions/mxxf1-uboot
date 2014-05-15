

include $(TOPDIR)/config.mk

LIB    = $(obj)lib$(BOARD).o

COBJS  := mxxf1.o eeprom.o mxxf1_spl.o

SRCS   := $(COBJS:.o=.c)
OBJS   := $(addprefix $(obj),$(COBJS))

$(LIB):        $(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

BUILD_ROOT=../..
OBJS = main.o

BINARY_NAME = main
LEDCTRL_PATH = ./ledctrl

include $(BUILD_ROOT)/common_defs.mk

LIB_DIRS  += $(LEDCTRL_PATH)
LDLIBS += -lledctrl

.PHONY: all ledctrl $(BINARY_NAME)

all: $(BINARY_NAME)

ledctrl:
	make -C $(LEDCTRL_PATH)

$(BINARY_NAME): $(BINARY_NAME).bin ledctrl

clean:
	@rm -f $(OBJS) main.elf main.bin main.elf.map
	make -C $(LEDCTRL_PATH) clean

flash: $(BINARY_NAME)
	$(FLASH_BINARY)

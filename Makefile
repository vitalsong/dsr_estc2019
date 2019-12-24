BUILD_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))


SPL_PATH = $(BUILD_ROOT)/stm_spl/
COMMON_PATH = $(BUILD_ROOT)/common
LABS_DIR = $(BUILD_ROOT)/projects

.PHONY: labs spl common lab1 lab2 lab3 lab4 clean

all: labs

labs: lab1 lab2 lab3 lab4

spl:
	make -C $(SPL_PATH)

common:
	make -C $(COMMON_PATH)

lab1: spl common
	make -C $(LABS_DIR)/lab_1

lab2: spl common
	make -C $(LABS_DIR)/lab_2

lab3: spl common
	make -C $(LABS_DIR)/lab_3

lab4: spl common
	make -C $(LABS_DIR)/lab_4

clean:
	make -C $(SPL_PATH) clean
	make -C $(COMMON_PATH) clean
	make -C $(LABS_DIR)/lab_1 clean
	make -C $(LABS_DIR)/lab_2 clean
	make -C $(LABS_DIR)/lab_3 clean
	make -C $(LABS_DIR)/lab_4 clean

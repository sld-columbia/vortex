ROOT_DIR := $(realpath ..)
include $(ROOT_DIR)/config.mk

all: unittest kernel regression opencl riscv

unittest:
	$(MAKE) CXX=$(CXX) -C unittest

kernel:
	$(MAKE) CXX=$(CXX) -C kernel

regression:
	$(MAKE) CXX=$(CXX) -C regression

opencl:
ifneq ($(XLEN),64)
	$(MAKE) CXX=$(CXX) -C opencl
endif

riscv:
	$(MAKE) CXX=$(CXX) -C riscv

clean:
	$(MAKE) -C unittest clean
	$(MAKE) -C kernel clean
	$(MAKE) -C regression clean
	$(MAKE) -C opencl clean
	$(MAKE) -C riscv clean

.PHONY: all unittest kernel regression opencl riscv
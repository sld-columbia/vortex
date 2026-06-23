ROOT_DIR := $(realpath ..)
include $(ROOT_DIR)/config.mk

ESP_ROOT ?=
ESP_HEADER := $(ESP_ROOT)/accelerators/rtl/gt_vortex_rtl/sw/linux/include/gt_vortex_rtl.h

ifeq ($(strip $(ESP_ROOT)),)
ENABLE_ESP ?= 0
else ifeq ($(wildcard $(ESP_HEADER)),)
ENABLE_ESP ?= 0
else
ENABLE_ESP ?= 1
endif

RUNTIME_TARGETS := stub rtlsim simx opae xrt
ifeq ($(ENABLE_ESP),1)
RUNTIME_TARGETS += esp
endif

all: $(RUNTIME_TARGETS)

stub:
	$(MAKE) -C stub

simx:
	$(MAKE) -C simx

rtlsim:
	$(MAKE) -C rtlsim

opae:
	$(MAKE) -C opae

xrt:
	$(MAKE) -C xrt

esp:
	$(MAKE) -C esp ESP_ROOT=$(ESP_ROOT)

clean:
	$(MAKE) -C stub clean
	$(MAKE) -C simx clean
	$(MAKE) -C rtlsim clean
	$(MAKE) -C opae clean
	$(MAKE) -C xrt clean
	$(MAKE) -C esp clean

.PHONY: all stub simx rtlsim opae xrt esp clean

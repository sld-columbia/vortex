include ../common.mk

ESP_ROOT ?= $(realpath $(ROOT_DIR)/../../..)

DESTDIR ?= $(CURDIR)/..

SRC_DIR := $(VORTEX_HOME)/runtime/esp

CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors
CXXFLAGS += -I$(INC_DIR) -I$(COMMON_DIR) -I$(ROOT_DIR)/hw -I$(SIM_DIR)/rtlsim -I$(COMMON_DIR) -I$(SIM_DIR)/common
CXXFLAGS += -I$(ESP_ROOT)/soft/common/drivers/linux/include
CXXFLAGS += -I$(ESP_ROOT)/soft/common/drivers/common/include
CXXFLAGS += -I$(ESP_ROOT)/accelerators/rtl/gt_vortex_rtl/sw/linux/include
ifneq ($(strip $(DESIGN_PATH)),)
CXXFLAGS += -I$(DESIGN_PATH)/socgen/esp
endif
CXXFLAGS += -DXLEN_$(XLEN)

# Position independent code
CXXFLAGS += -fPIC

# Add external configuration
CXXFLAGS += $(CONFIGS)

LDFLAGS += -shared -pthread

SRCS := $(SRC_DIR)/vortex.cpp

# Debugging
ifdef DEBUG
	CXXFLAGS += -g -O0
else
	CXXFLAGS += -O2 -DNDEBUG
endif

PROJECT := libvortex-esp.so

all: $(DESTDIR)/$(PROJECT)

$(DESTDIR)/$(PROJECT): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $@

clean-runtime:
	rm -f $(DESTDIR)/$(PROJECT)

clean: clean-runtime

.PHONY: all clean-runtime clean
include ../common.mk

DESTDIR ?= $(CURDIR)/..

SRC_DIR := $(VORTEX_HOME)/runtime/rtlsim

CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors
CXXFLAGS += -I$(INC_DIR) -I$(COMMON_DIR) -I$(ROOT_DIR)/hw -I$(SIM_DIR)/rtlsim -I$(COMMON_DIR) -I$(SIM_DIR)/common
CXXFLAGS += -DXLEN_$(XLEN)

# Position independent code
CXXFLAGS += -fPIC

# Add external configuration
CXXFLAGS += $(CONFIGS)

LDFLAGS += -shared -pthread
LDFLAGS += -L$(DESTDIR) -lrtlsim

SRCS := $(SRC_DIR)/vortex.cpp

# Debugging
ifdef DEBUG
	CXXFLAGS += -g -O0
else
	CXXFLAGS += -O2 -DNDEBUG
endif

PROJECT := libvortex-rtlsim.so

all: $(DESTDIR)/$(PROJECT)

driver: $(DESTDIR)/librtlsim.so

.PHONY: force-stub-runtime
force-stub-runtime:

$(DESTDIR)/libvortex.so: force-stub-runtime
	DESTDIR=$(DESTDIR) $(MAKE) -B -C $(ROOT_DIR)/runtime/stub CONFIGS="$(CONFIGS)"

$(DESTDIR)/librtlsim.so:
	DESTDIR=$(DESTDIR) $(MAKE) -C $(ROOT_DIR)/sim/rtlsim $(DESTDIR)/librtlsim.so

$(DESTDIR)/$(PROJECT): $(SRCS) $(DESTDIR)/librtlsim.so $(DESTDIR)/libvortex.so
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $@

clean-driver:
	DESTDIR=$(DESTDIR) $(MAKE) -C $(ROOT_DIR)/sim/rtlsim clean-lib

clean-runtime:
	rm -f $(DESTDIR)/$(PROJECT)

clean: clean-driver clean-runtime

.PHONY: all driver force-stub-runtime clean-driver clean-runtime clean

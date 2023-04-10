RTLSIM_DIR = ../../sim/rtlsim

CXX = $(CROSS_COMPILE)g++

CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -Wfatal-errors

CXXFLAGS += -I../include -I../common -I../../hw -I$(RTLSIM_DIR) -I$(RTLSIM_DIR)/../common
CXXFLAGS += -I$(DRIVERS)/include -I$(DRIVERS)/../common/include

BUILD_DRIVERS=$(SOFT_BUILD)/drivers

# Position independent code
CXXFLAGS += -fPIC

# Add external configuration
CXXFLAGS += $(CONFIGS)

# Dump perf stats
CXXFLAGS += -DDUMP_PERF_STATS

LDFLAGS += -shared -pthread
LDFLAGS += -L. -L$(BUILD_DRIVERS)/libesp -lesp

SRCS = vortex.cpp ../common/vx_utils.cpp

# Debugigng
ifdef DEBUG
	CXXFLAGS += -g -O0
else    
	CXXFLAGS += -O2 -DNDEBUG
endif

# Enable perf counters
ifdef PERF
	CXXFLAGS += -DPERF_ENABLE
endif

PROJECT = libvortex.so

all: $(PROJECT)
	
$(PROJECT): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $(PROJECT)

clean:
	rm -rf $(PROJECT) *.o

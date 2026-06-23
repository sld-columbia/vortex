ROOT_DIR := $(realpath ../..)
include $(ROOT_DIR)/config.mk

all:
	$(MAKE) -C conform
	$(MAKE) -C hello	
	$(MAKE) -C fibonacci
	$(MAKE) -C fibonacci_all_64
	$(MAKE) -C mem
	$(MAKE) -C float

run-simx:
	$(MAKE) -C conform run-simx
	$(MAKE) -C hello run-simx
	$(MAKE) -C fibonacci run-simx
	$(MAKE) -C fibonacci_all_64 run-simx
	$(MAKE) -C mem run-simx
	$(MAKE) -C float run-simx

run-rtlsim:
	$(MAKE) -C conform run-rtlsim
	$(MAKE) -C hello run-rtlsim
	$(MAKE) -C fibonacci run-rtlsim
	$(MAKE) -C fibonacci_all_64 run-rtlsim
	$(MAKE) -C mem run-rtlsim
	$(MAKE) -C float run-rtlsim

clean:
	$(MAKE) -C conform clean
	$(MAKE) -C hello clean
	$(MAKE) -C fibonacci clean
	$(MAKE) -C fibonacci_all_64 clean
	$(MAKE) -C mem clean
	$(MAKE) -C float clean
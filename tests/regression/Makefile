ROOT_DIR := $(realpath ../..)
include $(ROOT_DIR)/config.mk

all:
	$(MAKE) CXX=$(CXX) -C basic
	$(MAKE) CXX=$(CXX) -C demo
	$(MAKE) CXX=$(CXX) -C dogfood
	$(MAKE) CXX=$(CXX) -C mstress
	$(MAKE) CXX=$(CXX) -C io_addr
	$(MAKE) CXX=$(CXX) -C printf
	$(MAKE) CXX=$(CXX) -C diverge
	$(MAKE) CXX=$(CXX) -C sort
	$(MAKE) CXX=$(CXX) -C fence
	$(MAKE) CXX=$(CXX) -C vecaddx
	$(MAKE) CXX=$(CXX) -C sgemmx
	$(MAKE) CXX=$(CXX) -C conv3x
	$(MAKE) CXX=$(CXX) -C sgemm2x
	$(MAKE) CXX=$(CXX) -C stencil3d
	$(MAKE) CXX=$(CXX) -C systolic_transformer

run-simx:
	$(MAKE) -C basic run-simx
	$(MAKE) -C demo run-simx
	$(MAKE) -C dogfood run-simx
	$(MAKE) -C mstress run-simx
	$(MAKE) -C io_addr run-simx
	$(MAKE) -C printf run-simx
	$(MAKE) -C diverge run-simx
	$(MAKE) -C sort run-simx
	$(MAKE) -C fence run-simx
	$(MAKE) -C vecaddx run-simx
	$(MAKE) -C sgemmx run-simx
	$(MAKE) -C conv3x run-simx
	$(MAKE) -C sgemm2x run-simx
	$(MAKE) -C stencil3d run-simx
	$(MAKE) -C systolic_transformer run-simx

run-rtlsim:
	$(MAKE) -C basic run-rtlsim
	$(MAKE) -C demo run-rtlsim
	$(MAKE) -C dogfood run-rtlsim
	$(MAKE) -C mstress run-rtlsim
	$(MAKE) -C io_addr run-rtlsim
	$(MAKE) -C printf run-rtlsim
	$(MAKE) -C diverge run-rtlsim
	$(MAKE) -C sort run-rtlsim
	$(MAKE) -C fence run-rtlsim
	$(MAKE) -C vecaddx run-rtlsim
	$(MAKE) -C sgemmx run-rtlsim
	$(MAKE) -C conv3x run-rtlsim
	$(MAKE) -C sgemm2x run-rtlsim
	$(MAKE) -C stencil3d run-rtlsim
	$(MAKE) -C systolic_transformer run-rtlsim

clean:
	$(MAKE) -C basic clean
	$(MAKE) -C demo clean
	$(MAKE) -C dogfood clean
	$(MAKE) -C mstress clean
	$(MAKE) -C io_addr clean
	$(MAKE) -C printf clean
	$(MAKE) -C diverge clean
	$(MAKE) -C sort clean
	$(MAKE) -C fence clean
	$(MAKE) -C vecaddx clean
	$(MAKE) -C sgemmx clean
	$(MAKE) -C conv3x clean
	$(MAKE) -C sgemm2x clean
	$(MAKE) -C stencil3d clean
	$(MAKE) -C systolic_transformer clean

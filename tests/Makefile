all: runtime regression opencl riscv unittest

runtime:
	$(MAKE) CXX=$(CXX) -C runtime

regression:
	$(MAKE) CXX=$(CXX) -C regression

opencl:
	$(MAKE) CXX=$(CXX) -C opencl

riscv:
	$(MAKE) CXX=$(CXX) -C riscv

unittest:
	$(MAKE) CXX=$(CXX) -C unittest

clean:
	$(MAKE) clean -C runtime
	$(MAKE) clean -C regression
	$(MAKE) clean -C opencl
	$(MAKE) clean -C riscv
	$(MAKE) clean -C unittest

.PHONY: all runtime regression opencl riscv unittest

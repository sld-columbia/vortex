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
	$(MAKE) CXX=$(CXX) -C no_mf_ext
	$(MAKE) CXX=$(CXX) -C no_smem
	$(MAKE) CXX=$(CXX) -C prefetch

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
	$(MAKE) -C no_mf_ext run-simx
	$(MAKE) -C no_smem run-simx
	$(MAKE) -C prefetch run-simx

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
	$(MAKE) -C no_mf_ext run-rtlsim
	$(MAKE) -C no_smem run-rtlsim
	$(MAKE) -C prefetch run-rtlsim

run-vlsim:
	$(MAKE) -C basic run-vlsim
	$(MAKE) -C demo run-vlsim
	$(MAKE) -C dogfood run-vlsim
	$(MAKE) -C mstress run-vlsim
	$(MAKE) -C io_addr run-vlsim
	$(MAKE) -C printf run-vlsim
	$(MAKE) -C diverge run-vlsim
	$(MAKE) -C sort run-vlsim
	$(MAKE) -C fence run-vlsim
	$(MAKE) -C no_mf_ext run-vlsim
	$(MAKE) -C no_smem run-vlsim
	$(MAKE) -C prefetch run-vlsim

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
	$(MAKE) -C no_mf_ext clean
	$(MAKE) -C no_smem clean
	$(MAKE) -C prefetch clean

clean-all:
	$(MAKE) -C basic clean-all
	$(MAKE) -C demo clean-all
	$(MAKE) -C dogfood clean-all
	$(MAKE) -C mstress clean-all
	$(MAKE) -C io_addr clean-all
	$(MAKE) -C printf clean-all
	$(MAKE) -C diverge clean-all
	$(MAKE) -C sort clean-all
	$(MAKE) -C fence clean-all
	$(MAKE) -C no_mf_ext clean-all
	$(MAKE) -C no_smem clean-all
	$(MAKE) -C prefetch clean-all

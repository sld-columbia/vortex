all: stub rtlsim simx vlsim

stub:
	$(MAKE) -C stub

fpga:
	$(MAKE) -C fpga

asesim:
	$(MAKE) -C asesim

vlsim:
	$(MAKE) -C vlsim

rtlsim:
	$(MAKE) -C rtlsim

simx:
	$(MAKE) -C simx

esp:
	$(MAKE) -C esp 

clean:
	$(MAKE) clean -C stub
	$(MAKE) clean -C fpga
	$(MAKE) clean -C asesim
	$(MAKE) clean -C vlsim
	$(MAKE) clean -C rtlsim
	$(MAKE) clean -C simx
	$(MAKE) clean -C esp

.PHONY: all stub fpga asesim vlsim rtlsim simx clean esp

SV_FILES = $(wildcard ./src/pkg/*.sv) $(wildcard ./src/*.sv)
TB_FILES = $(wildcard ./tb/*.sv)
ALL_FILES = $(SV_FILES) $(TB_FILES)

lint:
	@echo "Linting all SystemVerilog files..."
	@echo "----------------------------------------"
	verilator --lint-only -Wall -Wno-UNUSED -Wno-MULTIDRIVEN --timing $(ALL_FILES)
build:
	verilator --binary $(SV_FILES) ./tb/tb.sv --top tb -j 0 --trace -Wno-UNOPTFLAT -Wno-MULTIDRIVEN
run: build
	./obj_dir/Vtb

wave: run
	gtkwave --dark dump.vcd

clean:
	@echo "Cleaning up..."
	rm -rf obj_dir;
	rm dump.vcd
	rm pc.log

.PHONY: all lint build run wave clean

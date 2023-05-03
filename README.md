Build Notes

## VSCode Debugger Setup

New project setup - clone `.vscode` directory with `launch.json` and `settings.json` into project root

Run cmake with debug symbols

	cd build
	cmake -DCMAKE_BUILD_TYPE=Debug ..

Launch VSCode with build directory

	export PICO_SDK_PATH=/home/growlph/Code/pico/pico-sdk
	code

## Program RP2040 from CLI using debug probe

	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program body_emu.elf verify reset exit" -s tcl

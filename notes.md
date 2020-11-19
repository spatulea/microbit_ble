# Useful command lines:

to load .out (same as .elf) to microbit using openocd
openocd -f interface/cmsis-dap.cfg -f target/nrf51.cfg -c "program _build/nrf51822_xxaa_s110.out verify reset exit"

pca20006 is supposed to be nrf51822

arm GCC is in /usr/local/bin:
/usr/local/bin/arm-none-eabi-gcc

but they're actually symlinks to:
/usr/local/Cellar/arm-none-eabi-gcc/9-2019-q4-major/bin/arm-none-eabi-gcc


Process for replication:
1. Install arm compiler with homebrew... google for "brew install arm-none-eabi-gcc"
2. Install openocd from head (not release, since it's old) with: "brew install open-ocd --HEAD"
3. Install vscode and extensions:
    1. C/C++
    2. Cortex-Debug


openocd command to load softdevice:
openocd -f interface/cmsis-dap.cfg -f target/nrf51.cfg -c "program /Users/sebastian/gits/blinky/components/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.hex verify reset exit"

openocd command to load firmware image (dependent on softdevice present):
openocd -f interface/cmsis-dap.cfg -f target/nrf51.cfg -c "program _build/nrf51822_xxaa_s110.hex verify reset exit"


Compiler defines:
SOFTDEVICE_PRESENT
NRF51
S110
BLE_STACK_SUPPORT_REQD
BOARD_PCA20006
BSP_DEFINES_ONLY
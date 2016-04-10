cd ./sw/ext/luftboot/src
make clean
make LIBOPENCM3=/home/jingg/paparazzi/sw/ext/libopencm3 PREFIX=/home/jingg/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi LUFTBOOT_USE_48MHZ_INTERNAL_OSC=1
cd -
./ruan_tools/stlink/st-flash write sw/ext/luftboot/src/luftboot.bin 0x8000000

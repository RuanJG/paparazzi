cd ./sw/ext/luftboot/src/
make clean
make LIBOPENCM3='/home/mx6/paparazzi/sw/ext/libopencm3' PREFIX='/home/mx6/gcc-arm-none-eabi-4_9-2015q2/bin/arm-none-eabi' LUFTBOOT_USE_48MHZ_INTERNAL_OSC=1 all
cd -

app=$(find ./sw/ext/luftboot/src/ -name *.hex)
if [ $1'x' != 'x' ];then
	app=$1
fi
if [ $app'x' == 'x' ];then
	echo no *.hex find or input 
	exit 1
fi
sudo ./ruan_tools/stm32flash -w $app -v -g 0x0 /dev/ttyUSB0

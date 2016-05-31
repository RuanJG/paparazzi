# Hey Emacs, this is a -*- makefile -*-
#
# ruan_apogee_v0.makefile
#
#

BOARD=ruan_apogee
BOARD_VERSION=v0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/ruan_apogee.ld

HARD_FLOAT=yes

# default flash mode is via usb dfu bootloader
# possibilities: DFU-UTIL, SWD, STLINK
FLASH_MODE ?= STLINK

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 1
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= 3
SYS_TIME_LED       ?= 2

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART3
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B38400

#config sbus in ap xml file
#SBUS_PORT ?= UART6

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm


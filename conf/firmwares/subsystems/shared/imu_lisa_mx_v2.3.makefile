# Hey Emacs, this is a -*- makefile -*-
#
# LisaMX V2.3 IMU
# Use <define name="IMU_LISA_MX_DISABLE_MAG value="TRUE"/> to disable the mag.
#

include $(CFG_SHARED)/spi_master.makefile

ifeq ($(TARGET), ap)
  IMU_LISA_MX_CFLAGS  = -DUSE_IMU
endif

IMU_LISA_MX_CFLAGS += -DIMU_TYPE_H=\"boards/lisa_mx/imu_lisa_mx.h\"

IMU_LISA_MX_SRCS  = $(SRC_SUBSYSTEMS)/imu.c
IMU_LISA_MX_SRCS += $(SRC_BOARD)/imu_lisa_mx.c
IMU_LISA_MX_SRCS += peripherals/mpu60x0.c
IMU_LISA_MX_SRCS += peripherals/mpu60x0_spi.c

IMU_LISA_MX_SPI_DEV=spi2
IMU_LISA_MX_CFLAGS += -DUSE_SPI -DUSE_SPI2 -DIMU_LISA_MX_SPI_SLAVE_IDX=SPI_SLAVE2
IMU_LISA_MX_CFLAGS += -DIMU_LISA_MX_SPI_DEV=$(IMU_LISA_MX_SPI_DEV)

# Slave select configuration
# SLAVE2 is on PB12 (NSS) (MPU600 CS)
IMU_LISA_MX_CFLAGS += -DUSE_SPI_SLAVE2

# SLAVE3 is on PC13, which is the baro CS
IMU_LISA_MX_CFLAGS += -DUSE_SPI_SLAVE3

# add it for all targets except sim, fbw and nps
ifeq (,$(findstring $(TARGET),sim fbw nps))
$(TARGET).CFLAGS += $(IMU_LISA_MX_CFLAGS)
$(TARGET).srcs += $(IMU_LISA_MX_SRCS)
endif

#
# NPS simulator
#
include $(CFG_SHARED)/imu_nps.makefile

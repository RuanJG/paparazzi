/*
 * Copyright (C) 2013-2015 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu/imu_mpu60x0_i2c.c
 * Driver for IMU with only MPU60X0 via I2C.
 */

#include <math.h>
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"
#include "led.h"

/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined IMU_MPU60X0_LOWPASS_FILTER && !defined  IMU_MPU60X0_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define IMU_MPU60X0_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define IMU_MPU60X0_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define IMU_MPU60X0_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define IMU_MPU60X0_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define MPU60X0_HMC_LOWPASS_FILTER and MPU60X0_HMC_SMPLRT_DIV.
#endif
#endif
PRINT_CONFIG_VAR(IMU_MPU60X0_LOWPASS_FILTER)
PRINT_CONFIG_VAR(IMU_MPU60X0_SMPLRT_DIV)

#ifndef IMU_MPU60X0_GYRO_RANGE
#define IMU_MPU60X0_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif
PRINT_CONFIG_VAR(IMU_MPU60X0_GYRO_RANGE)

#ifndef IMU_MPU60X0_ACCEL_RANGE
#define IMU_MPU60X0_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif
PRINT_CONFIG_VAR(IMU_MPU60X0_ACCEL_RANGE)

#ifndef IMU_MPU60X0_I2C_ADDR
#define IMU_MPU60X0_I2C_ADDR MPU60X0_ADDR
#endif


//add by ruan
#ifndef GY86_DISABLE_INSIDE_HMC58xx
#define GY86_IMU_READ_HMC58xx
#endif

#ifdef GY86_IMU_READ_HMC58xx
#include "peripherals/hmc58xx_regs.h"
/* HMC58XX default conf */
#ifndef HMC58XX_DO
#define HMC58XX_DO 0x6 // Data Output Rate (6 -> 75Hz with HMC5883)
#endif
#ifndef HMC58XX_MS
#define HMC58XX_MS 0x0 // Measurement configuration
#endif
#ifndef HMC58XX_GN
#define HMC58XX_GN 0x1 // Gain configuration (1 -> +- 1 Gauss)
#endif
#ifndef HMC58XX_MD
#define HMC58XX_MD 0x0 // Continious measurement mode
#endif
#define HMC58XX_CRA ((HMC58XX_DO<<2)|(HMC58XX_MS))
#define HMC58XX_CRB (HMC58XX_GN<<5)
//startup delay time
#define GY86_MAG_STARTUP_DELAY 1.5
#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))
/*
example configure
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
*/
#endif


struct ImuMpu60x0 imu_mpu_i2c;
#ifdef GY86_IMU_READ_HMC58xx
bool_t imu_gy86_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu);
#endif

void imu_impl_init(void)
{
  mpu60x0_i2c_init(&imu_mpu_i2c.mpu, &(IMU_MPU60X0_I2C_DEV), IMU_MPU60X0_I2C_ADDR);
  // change the default configuration
  imu_mpu_i2c.mpu.config.smplrt_div = IMU_MPU60X0_SMPLRT_DIV;
  imu_mpu_i2c.mpu.config.dlpf_cfg = IMU_MPU60X0_LOWPASS_FILTER;
  imu_mpu_i2c.mpu.config.gyro_range = IMU_MPU60X0_GYRO_RANGE;
  imu_mpu_i2c.mpu.config.accel_range = IMU_MPU60X0_ACCEL_RANGE;

#ifdef GY86_IMU_READ_HMC58xx
  imu_mpu_i2c.mpu.config.nb_slaves = 1;
  /* read 15 bytes for status, accel, gyro + 6 bytes for mag slave */
  imu_mpu_i2c.mpu.config.nb_bytes = 21;
  imu_mpu_i2c.mpu.config.slaves[0].configure = &imu_gy86_configure_mag_slave;
  /* Set MPU I2C master clock */
  imu_mpu_i2c.mpu.config.i2c_mst_clk = MPU60X0_MST_CLK_400KHZ;
  /* Enable I2C slave0 delayed sample rate */
  imu_mpu_i2c.mpu.config.i2c_mst_delay = 1;
#endif
}

void imu_periodic(void)
{
  mpu60x0_i2c_periodic(&imu_mpu_i2c.mpu);
}

void imu_mpu_i2c_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU60X0 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_mpu_i2c.mpu);
  if (imu_mpu_i2c.mpu.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_mpu_i2c.mpu.data_rates.rates);
    VECT3_COPY(imu.accel_unscaled, imu_mpu_i2c.mpu.data_accel.vect);
    imu_mpu_i2c.mpu.data_available = FALSE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_MPU60X0_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_MPU60X0_ID, now_ts, &imu.accel);
#ifdef GY86_IMU_READ_HMC58xx
    /* HMC5883 has xzy order of axes in returned data */
    struct Int32Vect3 mag;
    mag.x = Int16FromBuf(imu_mpu_i2c.mpu.data_ext, 0);
    mag.z = Int16FromBuf(imu_mpu_i2c.mpu.data_ext, 2);
    mag.y = Int16FromBuf(imu_mpu_i2c.mpu.data_ext, 4);
    VECT3_COPY(imu.mag_unscaled, mag);
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_MPU60X0_ID, now_ts, &imu.mag);
#endif
  }
}


#ifdef GY86_IMU_READ_HMC58xx

#include "mcu_periph/sys_time.h"
static inline void mpu_set_and_wait(Mpu60x0ConfigSet mpu_set, void *mpu, uint8_t _reg, uint8_t _val)
{
  mpu_set(mpu, _reg, _val);
  while (imu_mpu_i2c.mpu.i2c_trans.status != I2CTransSuccess);
}
static void mpu_wait_slave4_ready(void)
{
	volatile uint32_t i;
	volatile float s;

	while (imu_mpu_i2c.mpu.i2c_trans.status != I2CTransSuccess);
	while( 1 ){
		imu_mpu_i2c.mpu.i2c_trans.buf[0] = MPU60X0_REG_I2C_MST_STATUS| MPU60X0_SPI_READ;
		i2c_transceive(imu_mpu_i2c.mpu.i2c_p, &(imu_mpu_i2c.mpu.i2c_trans), imu_mpu_i2c.mpu.i2c_trans.slave_addr, 1, 2);
		i=5;
		while( i-- > 0 ){
  			if(imu_mpu_i2c.mpu.i2c_trans.status != I2CTransSuccess){
				s = get_sys_time_float()+0.01 ;//10ms
				while( get_sys_time_float() >= s );
			}else
				break;
		}
		if( imu_mpu_i2c.mpu.i2c_trans.status == I2CTransSuccess ){
			if( bit_is_set(imu_mpu_i2c.mpu.i2c_trans.buf[1],MPU60X0_I2C_SLV4_DONE) ){
				break;
			}
		}
		s = get_sys_time_float()+0.01 ;//10ms
		while( get_sys_time_float() >= s );
	}
}

/** function to configure hmc5883 mag
 * @return TRUE if mag configuration finished
 */
bool_t imu_gy86_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu)
{
  // wait before starting the configuration of the HMC58xx mag
  // doing to early may void the mode configuration
  if (get_sys_time_float() < GY86_MAG_STARTUP_DELAY) {
    return FALSE;
  }

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_REG, HMC58XX_REG_CFGA);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_DO, HMC58XX_CRA);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_REG, HMC58XX_REG_CFGB);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_DO, HMC58XX_CRB);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_wait_slave4_ready();

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_ADDR, (HMC58XX_ADDR >> 1));
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_REG, HMC58XX_REG_MODE);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_DO, HMC58XX_MD);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV4_CTRL, (1 << 7)); // Slave 4 enable

  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV0_ADDR, (HMC58XX_ADDR >> 1) | MPU60X0_SPI_READ);
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV0_REG, HMC58XX_REG_DATXM);
  // Put the enable command as last.
  mpu_set_and_wait(mpu_set, mpu, MPU60X0_REG_I2C_SLV0_CTRL,
                   (1 << 7) |    // Slave 0 enable
                   (6 << 0));    // Read 6 bytes

  return TRUE;
}
#endif


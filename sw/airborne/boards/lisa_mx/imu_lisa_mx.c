/*
 * Copyright (C) 2014 KylinUAS <4054419@qq.com>
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
 * @file boards/lisa_mx/imu_lisa_mx.c
 *
 * Driver for the IMU on the LisaMX V2.3 board.
 *
 * Invensense MPU-6000 and Honeywell HMC-5883L
 * 
 */

#include "boards/lisa_mx/imu_lisa_mx.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/spi.h"
#include "peripherals/hmc58xx_regs.h"

/* defaults suitable for Lisa */
#ifndef IMU_LISA_MX_SPI_SLAVE_IDX
#define IMU_LISA_MX_SPI_SLAVE_IDX SPI_SLAVE2
#endif
PRINT_CONFIG_VAR(IMU_LISA_MX_SPI_SLAVE_IDX)

#ifndef IMU_LISA_MX_SPI_DEV
#define IMU_LISA_MX_SPI_DEV spi2
#endif
PRINT_CONFIG_VAR(IMU_LISA_MX_SPI_DEV)

/* MPU60x0 gyro/accel internal lowpass frequency */
#if !defined LISA_MX_LOWPASS_FILTER && !defined  LISA_MX_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define LISA_MX_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define LISA_MX_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define LISA_MX_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define LISA_MX_SMPLRT_DIV 3
PRINT_CONFIG_MSG("Gyro/Accel output rate is 2kHz at 8kHz internal sampling")
#else
#error Non-default PERIODIC_FREQUENCY: please define LISA_MX_LOWPASS_FILTER and LISA_MX_SMPLRT_DIV.
#endif
#endif

PRINT_CONFIG_VAR(LISA_MX_LOWPASS_FILTER)
PRINT_CONFIG_VAR(LISA_MX_SMPLRT_DIV)


#ifndef LISA_MX_GYRO_RANGE
#define LISA_MX_GYRO_RANGE MPU60X0_GYRO_RANGE_1000
#endif
PRINT_CONFIG_VAR(LISA_MX_GYRO_RANGE)

#ifndef LISA_MX_ACCEL_RANGE
#define LISA_MX_ACCEL_RANGE MPU60X0_ACCEL_RANGE_8G
#endif
PRINT_CONFIG_VAR(LISA_MX_ACCEL_RANGE)


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

/** delay in seconds before starting to configure HMC58xx mag slave */
#ifndef LISA_MX_MAG_STARTUP_DELAY
#define LISA_MX_MAG_STARTUP_DELAY 1.5
#endif

struct ImuLisaMx imu_lisa_mx;

void mpu_wait_slave4_ready(void);
void mpu_wait_slave4_ready_cb(struct spi_transaction *t);
bool_t imu_lisa_mx_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu);

void imu_impl_init(void)
{
  mpu60x0_spi_init(&imu_lisa_mx.mpu, &(IMU_LISA_MX_SPI_DEV), IMU_LISA_MX_SPI_SLAVE_IDX);
  // change the default configuration
  imu_lisa_mx.mpu.config.smplrt_div = LISA_MX_SMPLRT_DIV;
  imu_lisa_mx.mpu.config.dlpf_cfg = LISA_MX_LOWPASS_FILTER;
  imu_lisa_mx.mpu.config.gyro_range = LISA_MX_GYRO_RANGE;
  imu_lisa_mx.mpu.config.accel_range = LISA_MX_ACCEL_RANGE;

#if !IMU_LISA_MX_DISABLE_MAG
  /* read 15 bytes for status, accel, gyro + 6 bytes for mag slave */
  imu_lisa_mx.mpu.config.nb_bytes = 21;

  /* HMC5883 magnetometer as I2C slave */
  imu_lisa_mx.mpu.config.nb_slaves = 1;

  /* set function to configure mag */
  imu_lisa_mx.mpu.config.slaves[0].configure = &imu_lisa_mx_configure_mag_slave;


  /* Set MPU I2C master clock */
  imu_lisa_mx.mpu.config.i2c_mst_clk = MPU60X0_MST_CLK_400KHZ;
  /* Enable I2C slave0 delayed sample rate */
  imu_lisa_mx.mpu.config.i2c_mst_delay = 1;

  /* configure spi transaction for wait_slave4 */
  imu_lisa_mx.wait_slave4_trans.cpol = SPICpolIdleHigh;
  imu_lisa_mx.wait_slave4_trans.cpha = SPICphaEdge2;
  imu_lisa_mx.wait_slave4_trans.dss = SPIDss8bit;
  imu_lisa_mx.wait_slave4_trans.bitorder = SPIMSBFirst;
  imu_lisa_mx.wait_slave4_trans.cdiv = SPIDiv64;

  imu_lisa_mx.wait_slave4_trans.select = SPISelectUnselect;
  imu_lisa_mx.wait_slave4_trans.slave_idx = IMU_LISA_MX_SPI_SLAVE_IDX;
  imu_lisa_mx.wait_slave4_trans.output_length = 1;
  imu_lisa_mx.wait_slave4_trans.input_length = 2;
  imu_lisa_mx.wait_slave4_trans.before_cb = NULL;
  imu_lisa_mx.wait_slave4_trans.after_cb = mpu_wait_slave4_ready_cb;
  imu_lisa_mx.wait_slave4_trans.input_buf = &(imu_lisa_mx.wait_slave4_rx_buf[0]);
  imu_lisa_mx.wait_slave4_trans.output_buf = &(imu_lisa_mx.wait_slave4_tx_buf[0]);

  imu_lisa_mx.wait_slave4_trans.status = SPITransDone;
  imu_lisa_mx.slave4_ready = FALSE;
#endif
}


void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  mpu60x0_spi_periodic(&imu_lisa_mx.mpu);
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void imu_lisa_mx_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  // If the MPU6000 SPI transaction has succeeded: convert the data
  mpu60x0_spi_event(&imu_lisa_mx.mpu);
  if (imu_lisa_mx.mpu.data_available) {
#if !IMU_LISA_MX_DISABLE_MAG
    /* HMC5883 has xzy order of axes in returned data */
    struct Int32Vect3 mag;
    mag.x = Int16FromBuf(imu_lisa_mx.mpu.data_ext, 0);
    mag.z = Int16FromBuf(imu_lisa_mx.mpu.data_ext, 2);
    mag.y = Int16FromBuf(imu_lisa_mx.mpu.data_ext, 4);
#endif

    RATES_ASSIGN(imu.gyro_unscaled,imu_lisa_mx.mpu.data_rates.rates.q,-imu_lisa_mx.mpu.data_rates.rates.p,
                 imu_lisa_mx.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled,imu_lisa_mx.mpu.data_accel.vect.y,-imu_lisa_mx.mpu.data_accel.vect.x,
                 imu_lisa_mx.mpu.data_accel.vect.z);
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);

#if !IMU_LISA_MX_DISABLE_MAG
    VECT3_ASSIGN(imu.mag_unscaled, mag.y, -mag.x, mag.z);
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
#endif

    imu_lisa_mx.mpu.data_available = FALSE;
  }
}

// hack with waiting to avoid creating another event loop to check the mag config status
static inline void mpu_set_and_wait(Mpu60x0ConfigSet mpu_set, void *mpu, uint8_t _reg, uint8_t _val)
{
  mpu_set(mpu, _reg, _val);
  while (imu_lisa_mx.mpu.spi_trans.status != SPITransSuccess);
}

/** function to configure hmc5883 mag
 * @return TRUE if mag configuration finished
 */
bool_t imu_lisa_mx_configure_mag_slave(Mpu60x0ConfigSet mpu_set, void *mpu)
{
  // wait before starting the configuration of the HMC58xx mag
  // doing to early may void the mode configuration
  if (get_sys_time_float() < LISA_MX_MAG_STARTUP_DELAY) {
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

void mpu_wait_slave4_ready(void)
{
  while (!imu_lisa_mx.slave4_ready) {
    if (imu_lisa_mx.wait_slave4_trans.status == SPITransDone) {
      imu_lisa_mx.wait_slave4_tx_buf[0] = MPU60X0_REG_I2C_MST_STATUS | MPU60X0_SPI_READ;
      spi_submit(imu_lisa_mx.mpu.spi_p, &(imu_lisa_mx.wait_slave4_trans));
    }
  }
}

void mpu_wait_slave4_ready_cb(struct spi_transaction *t)
{
  if (bit_is_set(t->input_buf[1], MPU60X0_I2C_SLV4_DONE)) {
    imu_lisa_mx.slave4_ready = TRUE;
  } else {
    imu_lisa_mx.slave4_ready = FALSE;
  }
  t->status = SPITransDone;
}

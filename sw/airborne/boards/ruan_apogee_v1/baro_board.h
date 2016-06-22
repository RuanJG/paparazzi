
/*
 * board specific interface for the KroozSD board
 *
 * It uses the subsystems/sensors/baro_ms5611_i2c.c driver
 */

#ifndef BOARDS_RUAN_APOGEE_V1_BARO_H
#define BOARDS_RUAN_APOGEE_V1_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BOARD_MS5611_I2C
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_RUAN_APOGEE_V1_BARO_H */

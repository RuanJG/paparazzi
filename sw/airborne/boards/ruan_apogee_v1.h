#ifndef CONFIG_RUAN_APOGEE_v0_H
#define CONFIG_RUAN_APOGEE_v0_H

/*

my QFlightFMU board config for Apogee :

pB6,PB7: uart1(af7)
PB8,PB9 : can1(af9) / i2c1(af4) 
PB10,PB11: i2c2 (af4)/ uart3(af7)
C10,C11: uart3(af7) (modem txrx)
C6 tim8_1 ppm
C7 uart6_rx sbus

now use i2c1,uart1(GPS),uart3(debug,flash,modem), a3->adc

pwm pin list :
b8(can1,i2c1)
b9(can1,i2c1)
a0(tim5_1) pwm0
a1(tim5_2) pwm1
a2(tim5_3) pwm2
b1(tim3_4) pwm3
b0(tim3_3) pwm4
a3(tim5_4) (power) 
c7(tim3_2/tim8_2/uart6_rx sbus)
c6(ppm,tim3_1/tim8_1/uart6_tx sbus)

led: PC13 PC14 PC15


*/

#define BOARD_APOGEE

/* Apogee has a 16MHz external clock and 168MHz internal. */
#define EXT_CLK 8000000
#define AHB_CLK 168000000

/*
 * Onboard LEDs
 */

#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOC
#define LED_2_GPIO_PIN GPIO14
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOC
#define LED_3_GPIO_PIN GPIO15
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


/* Onboard ADCs */
#define USE_AD_TIM4 1
#define USE_ADC_1 1 // use for battery check

#if USE_ADC_1
#define AD1_1_CHANNEL 3
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOA
#define ADC_1_GPIO_PIN GPIO3
#endif
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

//#define DefaultVoltageOfAdc(adc) (0.006185*adc)
//v0 = 3.3*(adc/4096) ; I= v0/R1;  v = (R1+R2)*I;  ==> v = adc*( 3.3*(R1+R2) / (4096*R1) ) 
#define DefaultVoltageOfAdc(adc) (0.004834*adc)




/* UART */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6
//baud define in board makefile

// V0
#define UART3_GPIO_AF GPIO_AF7
#define UART3_GPIO_PORT_RX GPIOC
#define UART3_GPIO_RX GPIO11
#define UART3_GPIO_PORT_TX GPIOC
#define UART3_GPIO_TX GPIO10


//use imu tx rx
//#define UART3_GPIO_AF GPIO_AF7
//#define UART3_GPIO_PORT_RX GPIOB
//#define UART3_GPIO_RX GPIO11
//#define UART3_GPIO_PORT_TX GPIOB
//#define UART3_GPIO_TX GPIO10


#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define USE_UART6_TX FALSE
//#define UART6_GPIO_PORT_RX GPIOC
//#define UART6_GPIO_RX GPIO6

/*
#define UART4_GPIO_AF GPIO_AF8
#define UART4_GPIO_PORT_RX GPIOA
#define UART4_GPIO_RX GPIO1
#define UART4_GPIO_PORT_TX GPIOA
#define UART4_GPIO_TX GPIO0
*/

/* SPI */
/*
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

// SLAVE0 on SPI connector
#define SPI_SELECT_SLAVE0_PORT GPIOB
#define SPI_SELECT_SLAVE0_PIN GPIO9
// SLAVE1 on AUX1
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO1
// SLAVE2 on AUX2
#define SPI_SELECT_SLAVE2_PORT GPIOC
#define SPI_SELECT_SLAVE2_PIN GPIO5
// SLAVE3 on AUX3
#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO4
// SLAVE4 on AUX4
#define SPI_SELECT_SLAVE4_PORT GPIOB
#define SPI_SELECT_SLAVE4_PIN GPIO15
*/

/* I2C mapping */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
//#define I2C1_GPIO_SDA GPIO7
#define I2C1_GPIO_SDA GPIO9

/*
#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11
*/

/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif


/* PWM */
#define PWM_USE_TIM5 1
#define PWM_USE_TIM3 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
// enable PWM connectors by default

#ifndef USE_PWM0
#define USE_PWM0 1
#endif
#if USE_PWM0
#define PWM_SERVO_0 0
#define PWM_SERVO_0_TIMER TIM5
#define PWM_SERVO_0_GPIO GPIOA
#define PWM_SERVO_0_PIN GPIO0
#define PWM_SERVO_0_AF GPIO_AF2
#define PWM_SERVO_0_OC TIM_OC1
#define PWM_SERVO_0_OC_BIT (1<<0)
#else
#define PWM_SERVO_0_OC_BIT 0
#endif

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_TIMER TIM5
#define PWM_SERVO_1_GPIO GPIOA
#define PWM_SERVO_1_PIN GPIO1
#define PWM_SERVO_1_AF GPIO_AF2
#define PWM_SERVO_1_OC TIM_OC2
#define PWM_SERVO_1_OC_BIT (1<<1)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_TIMER TIM5
#define PWM_SERVO_2_GPIO GPIOA
#define PWM_SERVO_2_PIN GPIO2
#define PWM_SERVO_2_AF GPIO_AF2
#define PWM_SERVO_2_OC TIM_OC3
#define PWM_SERVO_2_OC_BIT (1<<2)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOB
#define PWM_SERVO_3_PIN GPIO1
#define PWM_SERVO_3_AF GPIO_AF2
#define PWM_SERVO_3_OC TIM_OC4
#define PWM_SERVO_3_OC_BIT (1<<3)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_GPIO GPIOB
#define PWM_SERVO_4_PIN GPIO0
#define PWM_SERVO_4_AF GPIO_AF2
#define PWM_SERVO_4_OC TIM_OC3
#define PWM_SERVO_4_OC_BIT (1<<2)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

///  adc battery det use A3
#define USE_PWM5 0

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_TIMER TIM5
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO3
#define PWM_SERVO_5_AF GPIO_AF2
#define PWM_SERVO_5_OC TIM_OC4
#define PWM_SERVO_5_OC_BIT (1<<3)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_5_OC_BIT)
#else
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_0_OC_BIT|PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT)
#endif
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)

/*
 * PPM
 */
#define USE_PPM_TIM8 1

#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM8_CC_IRQ
#define PPM_IRQ2            NVIC_TIM8_UP_TIM13_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOC
#define PPM_GPIO_PIN        GPIO6
#define PPM_GPIO_AF         GPIO_AF3



/*
 * IRQ Priorities
 */
#define RTOS_PRIO 2
#define NVIC_TIM_IRQ_PRIO (RTOS_PRIO+1)
#define NVIC_I2C_IRQ_PRIO (RTOS_PRIO+2)
#define NVIC_SPI_IRQ_PRIO (RTOS_PRIO+3)
#define NVIC_UART_IRQ_PRIO (RTOS_PRIO+4)
#define NVIC_USART_IRQ_PRIO (RTOS_PRIO+4)
#define NVIC_ADC_IRQ_PRIO (RTOS_PRIO+5)
#define NVIC_TIM6_DAC_IRQ_PRIO (RTOS_PRIO+6)


#endif /* CONFIG_APOGEE_1_00_H */

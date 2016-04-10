#ifndef CONFIG_RUAN_LISA_M_2_1_H
#define CONFIG_RUAN_LISA_M_2_1_H


#define BOARD_RUAN_LISA_M

/* Lisa/M has a 12MHz external clock and 72MHz internal. */
//#define EXT_CLK 12000000
#define EXT_CLK 8000000
#define AHB_CLK 72000000

/*
if uart hw_flow_control closed, cts rts ck can be use as gpio
else check the uartx cts rts pin

 USART1 :
#define GPIO_BANK_USART1_CK		 PA8 
#define GPIO_BANK_USART1_TX		 PA9 
#define GPIO_BANK_USART1_RX		PA10 
 USART2:
#define GPIO_BANK_USART2_TX		 PA2 
#define GPIO_BANK_USART2_RX		 PA3 
#define GPIO_BANK_USART2_CK		 PA4 
 UART3 :
#define GPIO_BANK_USART3_PR_TX		PC10 
#define GPIO_BANK_USART3_PR_RX		PC11 
#define GPIO_BANK_USART3_PR_CK		PC12 
 SPI1 GPIO 
#define GPIO_SPI1_NSS			 PA4 
#define GPIO_SPI1_SCK			 PA5 
#define GPIO_SPI1_MISO			 PA6 
#define GPIO_SPI1_MOSI			 PA7 

 SPI2 GPIO 
#define GPIO_SPI2_NSS			 PB12 
#define GPIO_SPI2_SCK			 PB13 
#define GPIO_SPI2_MISO			 PB14 
#define GPIO_SPI2_MOSI			 PB15 

***********************************
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10

#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO1

************************************

adc(timer1): B0 B1 A0(bat det) C5 

i2c1 : B6 B7 
i2c2 : B10 B11

pwm( tim3 tim4 ) : C6 C7 C8 C9 B8 B9

led: c13 

baro eoc pin : A11

spi slave: A15 A4 B12 C12 C4

empty: A12 A13 B... C... D...
*/



//****************************************************************** lisa_m_common.h 

/* SPI slave mapping 
*/
#define SPI_SELECT_SLAVE0_PORT GPIOA
#define SPI_SELECT_SLAVE0_PIN GPIO15

#define SPI_SELECT_SLAVE1_PORT GPIOA
#define SPI_SELECT_SLAVE1_PIN GPIO4

#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO12

#define SPI_SELECT_SLAVE3_PORT GPIOC
#define SPI_SELECT_SLAVE3_PIN GPIO13

#define SPI_SELECT_SLAVE4_PORT GPIOC
#define SPI_SELECT_SLAVE4_PIN GPIO12

#define SPI_SELECT_SLAVE5_PORT GPIOC
#define SPI_SELECT_SLAVE5_PIN GPIO4


/*
 * UART pin configuration
 *
 * sets on which pins the UARTs are connected
 */
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIO_BANK_USART1_RX
#define UART1_GPIO_RX GPIO_USART1_RX
#define UART1_GPIO_PORT_TX GPIO_BANK_USART1_TX
#define UART1_GPIO_TX GPIO_USART1_TX

#define UART2_GPIO_AF 0
#define UART2_GPIO_PORT_RX GPIO_BANK_USART2_RX
#define UART2_GPIO_RX GPIO_USART2_RX
#define UART2_GPIO_PORT_TX GPIO_BANK_USART2_TX
#define UART2_GPIO_TX GPIO_USART2_TX

#define UART3_GPIO_AF AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP
#define UART3_GPIO_PORT_RX GPIO_BANK_USART3_PR_RX
#define UART3_GPIO_RX GPIO_USART3_PR_RX
#define UART3_GPIO_PORT_TX GPIO_BANK_USART3_PR_TX
#define UART3_GPIO_TX GPIO_USART3_PR_TX



/* PPM
 *
 * Default is PPM config 2, input on GPIO01 (Servo pin 6)
 */

#ifndef PPM_CONFIG
#define PPM_CONFIG 2
#endif

#if PPM_CONFIG == 1
/* input on PA10 (UART1_RX) */
#define USE_PPM_TIM1 1
#define PPM_CHANNEL         TIM_IC3
#define PPM_TIMER_INPUT     TIM_IC_IN_TI3
#define PPM_IRQ             NVIC_TIM1_UP_IRQ
#define PPM_IRQ2            NVIC_TIM1_CC_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC3IE
#define PPM_CC_IF           TIM_SR_CC3IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10
#define PPM_GPIO_AF         0

#elif PPM_CONFIG == 2
/* input on PA1 (Servo 6 pin) */
#define USE_PPM_TIM2 1
#define PPM_CHANNEL         TIM_IC2
#define PPM_TIMER_INPUT     TIM_IC_IN_TI2
#define PPM_IRQ             NVIC_TIM2_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC2IE
#define PPM_CC_IF           TIM_SR_CC2IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO1
#define PPM_GPIO_AF         0
// Move default ADC timer
#if USE_AD_TIM2
#undef USE_AD_TIM2
#endif

#else
#error "Unknown PPM config"
#endif // PPM_CONFIG

/* Onboard ADCs */
/*
 * ADC
 */
#define USE_AD_TIM1 1

#if USE_ADC_1
#define AD1_1_CHANNEL 8
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOB
#define ADC_1_GPIO_PIN GPIO0
#endif

#if USE_ADC_2
#define AD1_2_CHANNEL 9
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT GPIOB
#define ADC_2_GPIO_PIN GPIO1
#endif

// Internal ADC for battery enabled by default
#ifndef USE_ADC_3
#define USE_ADC_3 1
#endif
#if USE_ADC_3
#define AD1_3_CHANNEL 0
#define ADC_3 AD1_3
#define ADC_3_GPIO_PORT GPIOA
#define ADC_3_GPIO_PIN GPIO0
#endif

#if USE_ADC_4
#define AD1_4_CHANNEL 15
#define ADC_4 AD1_4
#define ADC_4_GPIO_PORT GPIOC
#define ADC_4_GPIO_PIN GPIO5
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_3
#endif

//#define DefaultVoltageOfAdc(adc) (0.0059*adc)
#define DefaultVoltageOfAdc(adc) (0.0045*adc)



/*
 * I2C
 *
 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO6
#define I2C1_GPIO_SDA GPIO7

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11


/*
 * PWM
 *
 */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM4 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1
#define USE_PWM5 1
#define USE_PWM6 1

#define ACTUATORS_PWM_NB 6

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO6
#define PWM_SERVO_1_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_GPIO GPIOC
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM4
#define PWM_SERVO_5_GPIO GPIOB
#define PWM_SERVO_5_PIN GPIO8
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC3
#define PWM_SERVO_5_OC_BIT (1<<2)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM4
#define PWM_SERVO_6_GPIO GPIOB
#define PWM_SERVO_6_PIN GPIO9
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC4
#define PWM_SERVO_6_OC_BIT (1<<3)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif

/* servos 1-4 on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM4 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()










//******************************************************************************************************* lisa_m_2.1.h 



/*
 * Onboard LEDs
 */
///////#define LED_STP08
//#ifndef USE_LED_1
#define USE_LED_1 1
//#endif
#define LED_1_GPIO GPIOC
#define LED_1_GPIO_PIN GPIO13
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* green, shared with JTAG_TRST */
/*
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO4
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP {                            \
    rcc_periph_clock_enable(RCC_AFIO);                  \
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;  \
  }
 */




#endif /* CONFIG_LISA_M_2_1_H */

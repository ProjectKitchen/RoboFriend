/*
 * @file    GPIO.h
 * @version v1.0
 * @date    14.11.2018
 * @author  mzahedi
 * @brief   DESCRIPTION
 */

#ifndef GPIO_H_
#define GPIO_H_

// pin definitions for power management 

#define Hauptrelais     20
#define Serienschaltung 21
#define Relais12V       22
#define Relais5V        23

/******************************************************************* DEFINES */

/* I2C interface */
#define PIN_I2C_SCL     0  // D0
#define PIN_I2C_SDA     1  // D1

/* UART interface */
#define PIN_UART_RX     2  // D2
#define PIN_UART_TX     3  // D3

/* SPI interface */
/*
#define PIN_SPI_MISO    23  // B3
#define PIN_SPI_MOSI    22  // B2
#define PIN_SPI_SCK     21  // B1
#define PIN_SPI_SS      20  // B0
*/

/* ultrasonic sensors */
#define PIN_US1_ECHO    4  // D4
#define PIN_US1_TRIG    5  // D5
#define PIN_US2_ECHO    6  // D6
#define PIN_US2_TRIG    7  // D7
#define PIN_US3_ECHO    8  // E0
#define PIN_US3_TRIG    9  // E1
#define PIN_US4_ECHO    17 // C7
#define PIN_US4_TRIG    16 // C6

/* Motor right */
#define PIN_MT_RI_FWD   10 // C0
#define PIN_MT_RI_BCK   11 // C1
#define PIN_MT_RI_PWM   14 // C4
#define PIN_MT_RI_INC   24 // B4
#define PIN_MT_RI_DEC   25 // B5

/* Motor left */
#define PIN_MT_LE_FWD   13 // C2 TODO: invert logic in KiCad
#define PIN_MT_LE_BCK   12 // C3 TODO: invert logic in KiCad
#define PIN_MT_LE_PWM   15 // C5
#define PIN_MT_LE_INC   26 // B6
#define PIN_MT_LE_DEC   27 // B7

/* Overcurrent protection */
#define PIN_OC          19 // E7
#define PIN_OC_INT      18 // E6
#define PIN_OC_ADC      44 // F6

/* ADC and infrared sensors */
#define PIN_ADC_VBAT    38 // F0
#define PIN_ADC_IR1     39 // F1
#define PIN_ADC_IR2     40 // F2
#define PIN_ADC_IR3     41 // F3
#define PIN_ADC1        42 // F4
#define PIN_ADC2        43 // F5

#endif /* GPIO_H_ */

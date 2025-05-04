#ifndef PINOUT_H
#define PINOUT_H

//I2C
#define SDA 11
#define SCL 10

//SPI (not used)
//#define MOSI 8
//#define MISO 9
//#define SCK 10

//Motors
#define SPARK_MAX_PWM 12
#define FAN_PWM 36

//Laser
#define LASER 35

//Buzzer
#define BUZZER 1

//LED
#define STATUS_LED 16

//Encoder
#define ENC_A 38
#define ENC_B 37
//#define ENC_INDEX 4 not used

//IMU
//#define I_MOSI 8
//#define I_MISO 9
//#define I_SCK 10
//#define I_INT1 5
//#define I_INT2 6
//#define I_CS 7

//Motor Driver
//#define M_NFAULT 17
//#define M_NSLEEP 18
//#define M_IPROPI 38
//#define M_EN 39
//#define M_PH 40

//Interface
#define BTN0 45
#define BTN1 48
#define BTN2 3
#define BTN3 9

#endif

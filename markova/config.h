#ifndef CONFIG_H
#define CONFIG_H

#define PWM_FREQ 25000
#define ISENSE_RESISTANCE 1000

#define OLED_HEIGHT 64
#define OLED_WIDTH 128
#define OLED_WIRE &Wire
#define OLED_ADDR 0x3C

//Motion profile
#define SECONDS_PER_INTERVAL 0.01
#define INITIAL_DELAY 0

//Kinematics
#define MAX_VEL 2.77
#define MAX_ACC 2.25
#define MAX_JERK 5
#define STOP_RANGE 0.005

//Control
#define KP 2.5
#define KP_I 0.2485
#define KP_D 1.725
#define KV 0
#define KA 0
#define KJ 0
#define CONTRL_FREQ 2500 //in hertz
#define TICKS_PER_METER 8020.40658113
#define VEL_FREQ 2
#define ACC_FREQ 2

//IMU
#define ACCEL_G 9.80665
#define IMU_CLOCK_FREQ 100000

//Telemetry
#define AP_SSID "MCQUEEN"
#define AP_PASS "NATS2025"
#define NET_SSID "DOEGuest"
#define NET_PASS "NYC$itevent"
#define PORT 8080
#define IP_TARGET 192,168,4,2
#define TELEM_FREQ 50
#define TELEM_BUFF_SIZE 5

#define USE_AP true

//FSM states
#define IDLE 0
#define RUNMENU 1
#define DISTSET 2
#define OFFSET 3
#define SETTINGS 4
#define READY 5
#define RUNNING 6
#define STOPPED 7




#endif
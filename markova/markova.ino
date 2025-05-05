#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SparkFun_BMI270_Arduino_Library.h>
 
#include "pinout.h"
#include "config.h"

#include "BrushlessMotor.h"
#include "MotionProfile.h"
#include "Controller.h"
#include "Interface.h"
#include "utils.h"

MotionProfileParameters MOTIONPROFILEPARAMS;
ControllerPIDParams CONTROLLERPIDPARAMS;

BMI270 IMU;

FilterOnePole VELOCITYFILTER(LOWPASS, VEL_FREQ);
FilterOnePole ACCELERATIONFILTER(LOWPASS, ACC_FREQ);

volatile int TICKS;

bool IS_READY;
bool LASER_ON;
volatile bool TELEM_ENABLED;

float OFFSET_X;
float AIM_X;

int STATE;

float DELTA_X_OPTIONS[] = { 1, 0.5, 0.1, 0.01, 0.001 };
int DELTA_X_I;

float OFF_X_OPTIONS[] = { 10, 1, 0.1, 0.01, 0.001 };
int OFF_X_I;

int BTN_PINS[] = { BTN0, BTN1, BTN2, BTN3 };
bool BTN_PREV_STATES[] = { LOW, LOW, LOW, LOW };


BrushlessMotor MOTOR(
 SPARK_MAX_PWM, SPARKMAX_FULL_REVERSE, SPARKMAX_NEUTRAL_MIN, SPARKMAX_NEUTRAL_MAX,
 SPARKMAX_FULL_FORWARD, SPARKMAX_FREQUENCY, SPARKMAX_PERIOD
);

MotionProfile MOTIONPROFILE(
  &Serial, &MOTIONPROFILEPARAMS
);

Controller CONTROLLER(
  &Serial, &TICKS, &MOTIONPROFILE, &MOTOR, &CONTROLLERPIDPARAMS, &IMU, &VELOCITYFILTER, &ACCELERATIONFILTER, TICKS_PER_METER
);

Adafruit_SSD1306 OLED(OLED_WIDTH, OLED_HEIGHT, OLED_WIRE, -1);

Interface INTERFACE(&OLED, OLED_WIDTH, OLED_HEIGHT);

//Telemetry
WiFiUDP SERVER;
IPAddress TARGET(IP_TARGET);

char IP[32];

void beep() {
  digitalWrite(BUZZER, HIGH);
  vTaskDelay(100/portTICK_PERIOD_MS);
  digitalWrite(BUZZER, LOW);
}

void updateTelemetry(void* p) {
  float t_0 = (float)micros()/10e6;
  float period = 1/TELEM_FREQ;
  while (1) {
    yield();
    if (TELEM_ENABLED) {
      float t = (float)micros()/10e6;
      //Send telemetry
      if (t - t_0 > period) {
        char currentposition_text[10];
        char currentvelocity_text[10];
        char currentacceleration_text[10];
        char setpointposition_text[10];
        char setpointvelocity_text[10];
        char setpointacceleration_text[10];
        char manualsetpointposition_text[10];

        dtostrf(CONTROLLER.currentPos, 4, 4, currentposition_text);
        dtostrf(CONTROLLER.currentVel, 4, 4, currentvelocity_text);
        dtostrf(CONTROLLER.currentAcc, 4, 4, currentacceleration_text);
        dtostrf(CONTROLLER.profPosSet, 4, 4, setpointposition_text);
        dtostrf(CONTROLLER.profVelSet, 4, 4, setpointvelocity_text);
        dtostrf(CONTROLLER.profAccSet, 4, 4, setpointacceleration_text);
        dtostrf(CONTROLLER.posSetpoint, 4, 4, manualsetpointposition_text);

        char udp_outbound[100];
        memset(udp_outbound, 0, 100);
        
        snprintf(
          udp_outbound, 90, "%s,%s,%s,%s,%s,%s,%s\n", 
          currentposition_text, currentvelocity_text, currentacceleration_text,
          setpointposition_text, setpointvelocity_text, setpointacceleration_text, 
          manualsetpointposition_text
        );

        SERVER.beginPacket(TARGET, PORT);
        SERVER.print(udp_outbound);
        SERVER.endPacket();
      }
      //Read incoming
      char udp_inbound[TELEM_BUFF_SIZE];
      memset(udp_inbound, 0, TELEM_BUFF_SIZE);

      SERVER.parsePacket();

        //Incoming packet format
        //X-Y[31]
        //X -> var/func identifier
        //Y -> arg

      if (SERVER.read(udp_inbound, TELEM_BUFF_SIZE) > 0) {
        char mode = udp_inbound[0];
        float argf;
        float argd;
        memcpy(&argf, udp_inbound+1, 4);
        memcpy(&argd, udp_inbound+1, 4);
        switch (mode) {
          case 0:
            break;
          //kP
          case 1:
            CONTROLLERPIDPARAMS.kP = argf;
            beep();
            break;
          //kV
          case 2:
            CONTROLLERPIDPARAMS.kV = argf;
            beep();
            break;
          //kA
          case 3:
            CONTROLLERPIDPARAMS.kA = argf;
            beep();
            break;
          //kJ
          case 4:
            CONTROLLERPIDPARAMS.kJ = argf;
            beep();
            break;
          //kPI
          case 5:
            CONTROLLERPIDPARAMS.kPI = argf;
            beep();
            break;
          //mode
          case 6:
            CONTROLLER.mode = argd;
            beep();
            break;
          //posSetPoint (manual pid kp tune)
          case 7:
            CONTROLLER.posSetpoint = argf;
            beep();
            break;
          //dist
          case 8:
            MOTIONPROFILEPARAMS.dist = argf;
            beep();
            break;
          //max v
          case 9:
            MOTIONPROFILEPARAMS.vMax = argf;
            beep();
            break;
          //max a
          case 10:
            MOTIONPROFILEPARAMS.aMax = argf;
            beep();
            break;
          //max j
          case 11:
            MOTIONPROFILEPARAMS.jMax = argf;
            beep();
            break;
          //KP_D
          case 12:
            CONTROLLERPIDPARAMS.kPD = argf;
            beep();
            break;
          default:
            break;
        }
      }
    }
  }
}

//Interrupts
void encoderInterruptHandlerA() {
  if (digitalRead(ENC_A) != digitalRead(ENC_B)) {
    TICKS--;
  }
  else {
    TICKS++;
  }
}

void encoderInterruptHandlerB() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
    TICKS--;
  }
  else {
    TICKS++;
  }
}


void setup() {
  Wire.begin(SDA, SCL);
  //SPI.begin(I_SCK, I_MISO, I_MOSI, I_CS);
  Serial.begin(115200);

  //Init pins
  pinMode(LASER, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  
  pinMode(BTN0, INPUT);
  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT);

  //Init OLED
  OLED.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  vTaskDelay(100);

  //Start WiFi and server
  if (USE_AP) {
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress IP_buff = WiFi.softAPIP();
    sprintf(IP, "%d.%d.%d.%d", IP_buff[0], IP_buff[1], IP_buff[2], IP_buff[3]);
  }
  else {
    WiFi.begin(NET_SSID, NET_PASS);
    while(WiFi.status() != WL_CONNECTED){
        delay(100);
    }
    IPAddress IP_buff = WiFi.localIP();
    sprintf(IP, "%d.%d.%d.%d", IP_buff[0], IP_buff[1], IP_buff[2], IP_buff[3]);
  }

  SERVER.begin(PORT);

  xTaskCreatePinnedToCore(
    updateTelemetry,
    "Telemetry",
    10000,
    NULL,
    0,
    NULL,
    0
  );
  
  MOTIONPROFILEPARAMS = {
    8, MAX_VEL, MAX_ACC, MAX_JERK, SECONDS_PER_INTERVAL, INITIAL_DELAY
  };

  CONTROLLERPIDPARAMS = {
    KP, KP_I, KP_D, KV, KA, KJ, 1.0/CONTRL_FREQ
  };

  //Init objects to default states
  CONTROLLER.init();
  MOTIONPROFILE.clearMotionProfile();

  IS_READY = false;
  LASER_ON = true;
  TELEM_ENABLED = true;

  //Init ticks
  TICKS = 0;

  //Indexing
  DELTA_X_I = 0;
  OFF_X_I = 1;
  
  //Offets
  OFFSET_X = 0;
  AIM_X = 0;

  //Print start menu
  INTERFACE.mainMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, IS_READY);

  //Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderInterruptHandlerA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderInterruptHandlerB, RISING);

  //Beep buzzer
  beep();
  digitalWrite(LASER, HIGH);
   
  //Init state
  STATE = IDLE;  
}

float T_0;

void loop() {
  yield();
  int controllerReturn = CONTROLLER.update();
  switch(STATE) {
    case IDLE:
      //Enable
      if (get_btn_state(0, BTN_PINS, BTN_PREV_STATES)) {
        MOTIONPROFILE.generateMotionProfiles();
        IS_READY = true;
        digitalWrite(STATUS_LED, HIGH);
        INTERFACE.mainMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, IS_READY);
        STATE = READY;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Settings
      else if (get_btn_state(1, BTN_PINS, BTN_PREV_STATES)) {
        INTERFACE.settingMenu(IP, LASER_ON, TELEM_ENABLED, CONTROLLER.mode);
        STATE = SETTINGS;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Run Menu
      else if (get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        INTERFACE.runMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, MOTIONPROFILE.pathDuration);
        STATE = RUNMENU;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      break;

    case SETTINGS:
      //Telemetry Enable
      if (get_btn_state(0, BTN_PINS, BTN_PREV_STATES)) {
        TELEM_ENABLED = !TELEM_ENABLED;
        INTERFACE.settingMenu(IP, LASER_ON, TELEM_ENABLED, CONTROLLER.mode);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //LASER
      else if (get_btn_state(1, BTN_PINS, BTN_PREV_STATES)) {
        LASER_ON = !LASER_ON;
        digitalWrite(LASER, LASER_ON);
        INTERFACE.settingMenu(IP, LASER_ON, TELEM_ENABLED, CONTROLLER.mode);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Back to main menu
      else if (get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        INTERFACE.mainMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, IS_READY);
        STATE = IDLE;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      else if (get_btn_state(3, BTN_PINS, BTN_PREV_STATES)) {
        CONTROLLER.mode = !CONTROLLER.mode;
        INTERFACE.settingMenu(IP, LASER_ON, TELEM_ENABLED, CONTROLLER.mode);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      break;

    case RUNMENU:
      //Offset Menu
      if (get_btn_state(0, BTN_PINS, BTN_PREV_STATES)) {
        OFF_X_I = 0;
        INTERFACE.offsetMenu(OFFSET_X, OFF_X_OPTIONS[OFF_X_I]);
        STATE = OFFSET;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Distance Menu
      else if (get_btn_state(1, BTN_PINS, BTN_PREV_STATES)) {
        DELTA_X_I = 0;
        INTERFACE.distMenu(MOTIONPROFILEPARAMS.dist, DELTA_X_OPTIONS[DELTA_X_I]);
        STATE = DISTSET;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Run Menu
      else if (get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        INTERFACE.mainMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, IS_READY);
        STATE = IDLE;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      break;

    case DISTSET:
      //+
      if (get_btn_state(0, BTN_PINS, BTN_PREV_STATES)) {
        MOTIONPROFILEPARAMS.dist += DELTA_X_OPTIONS[DELTA_X_I];
        INTERFACE.distMenu(MOTIONPROFILEPARAMS.dist, DELTA_X_OPTIONS[DELTA_X_I]);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Change Increment
      else if (get_btn_state(1, BTN_PINS, BTN_PREV_STATES)) {
        DELTA_X_I++;
        DELTA_X_I %= 5;
        INTERFACE.distMenu(MOTIONPROFILEPARAMS.dist, DELTA_X_OPTIONS[DELTA_X_I]);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Run Menu
      else if (get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        INTERFACE.runMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, MOTIONPROFILE.pathDuration);
        STATE = RUNMENU;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //-
      else if (get_btn_state(3, BTN_PINS, BTN_PREV_STATES)) {
        MOTIONPROFILEPARAMS.dist -= DELTA_X_OPTIONS[DELTA_X_I];
        INTERFACE.distMenu(MOTIONPROFILEPARAMS.dist, DELTA_X_OPTIONS[DELTA_X_I]);
        vTaskDelay(10/portTICK_PERIOD_MS);
      } 
      break;

    case OFFSET:
      //+
      if (get_btn_state(0, BTN_PINS, BTN_PREV_STATES)) {
        OFFSET_X += OFF_X_OPTIONS[OFF_X_I];
        INTERFACE.offsetMenu(OFFSET_X, OFF_X_OPTIONS[OFF_X_I]);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Change Increment
      else if (get_btn_state(1, BTN_PINS, BTN_PREV_STATES)) {
        OFF_X_I++;
        OFF_X_I %= 5;
        INTERFACE.offsetMenu(OFFSET_X, OFF_X_OPTIONS[OFF_X_I]);
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Run Menu
      else if (get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        INTERFACE.runMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, MOTIONPROFILE.pathDuration);
        STATE = RUNMENU;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //-
      else if (get_btn_state(3, BTN_PINS, BTN_PREV_STATES)) {
        OFFSET_X -= OFF_X_OPTIONS[OFF_X_I];
        INTERFACE.offsetMenu(OFFSET_X, OFF_X_OPTIONS[OFF_X_I]);
        vTaskDelay(10/portTICK_PERIOD_MS);
      } 
      break;

    case READY:
      //Disable
      if (get_btn_state(0, BTN_PINS, BTN_PREV_STATES)) {
        IS_READY = false;
        digitalWrite(STATUS_LED, LOW);
        INTERFACE.mainMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, IS_READY);
        STATE = IDLE;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Settings
      else if (get_btn_state(1, BTN_PINS, BTN_PREV_STATES)) {
        IS_READY = false;
        digitalWrite(STATUS_LED, LOW);
        INTERFACE.settingMenu(IP, LASER_ON, TELEM_ENABLED, CONTROLLER.mode);
        STATE = SETTINGS;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Run Menu
      else if (get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        IS_READY = false;
        digitalWrite(STATUS_LED, LOW);
        INTERFACE.runMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, MOTIONPROFILE.pathDuration);
        STATE = RUNMENU;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      //Run!
      else if (get_btn_state(3, BTN_PINS, BTN_PREV_STATES)) {
        IS_READY = false;
        digitalWrite(STATUS_LED, LOW);
        INTERFACE.runScreen(MOTIONPROFILEPARAMS.dist, OFFSET_X);
        //MOTOR.enable();
        CONTROLLER.enable();
        CONTROLLER.start();
        T_0 = millis();
        STATE = RUNNING;
      }
      break;

    case RUNNING:
      if (!(controllerReturn) || get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        CONTROLLER.disable();
        MOTOR.disable();
        INTERFACE.endScreen(CONTROLLER.currentPos, (float)(millis() - T_0)/pow(10, 3));
        beep();
        STATE = STOPPED;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      break;
      
    case STOPPED:
      if(get_btn_state(2, BTN_PINS, BTN_PREV_STATES)) {
        CONTROLLER.init();
        MOTIONPROFILE.clearMotionProfile();
        INTERFACE.mainMenu(MOTIONPROFILEPARAMS.dist, OFFSET_X, AIM_X, IS_READY);
        STATE = IDLE;
        vTaskDelay(10/portTICK_PERIOD_MS);
      }
      break;
    default:
      break;
  }
}


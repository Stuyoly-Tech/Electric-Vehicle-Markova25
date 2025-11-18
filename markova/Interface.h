#ifndef INTERFACE_H
#define INTERFACE_H

#include <Adafruit_SSD1306.h>

#include "Controller.h"
#include "MotionProfile.h"

class Interface {
    private:
        Adafruit_SSD1306* display;
        int screenWidth, screenHeight;

        char rowVals[6][255];

        void printInterface(char* letterings);

    public:
        Interface(Adafruit_SSD1306* iDisplay, int iScreenWidth, int iScreenHeight);
        void mainMenu(float dist, float offset, float aim, bool isready);
        void runMenu(float dist, float offset, float aim, float runtime);
        void distMenu(float dist, float increment);
        void timeMenu(float time, float increment);
        void settingMenu(char* ip, bool laserstate, bool telemetrystate, int controllermode);
        void endScreen(float dist, float t);
        void runScreen(float dist, float offset);
};

#endif
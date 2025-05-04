#include <Adafruit_SSD1306.h>

#include "config.h"
#include "Interface.h"

void Interface::printInterface(char* letterings) {
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    //Draw bounding box
    display->setCursor(0, 0);
    display->drawRect(7, 0, screenWidth-16, screenHeight, 0xFF);
    //Print letters for buttons
    display->setCursor(0, 0);
    display->print(letterings[0]);
    display->setCursor(screenWidth-7, 0);
    display->print(letterings[1]);
    display->setCursor(0, screenHeight-7);
    display->print(letterings[2]);
    display->setCursor(screenWidth-7, screenHeight-7);
    display->print(letterings[3]);
    //Print stuff
    display->setTextSize(2);
    if (strlen(rowVals[0]) > 0) {
        display->setCursor(12, 0);
        display->print(rowVals[0]);
    }
    display->setTextSize(1);
    if (strlen(rowVals[1])) {
        display->setCursor(12, 16);
        display->print(rowVals[1]);
    }
    if (strlen(rowVals[2]) > 0) {
        display->setCursor(12, 24);
        display->print(rowVals[2]);
    }
    if (strlen(rowVals[3]) > 0) {
        display->setCursor(12, 32);
        display->print(rowVals[3]);
    }
    if (strlen(rowVals[4]) > 0) {
        display->setCursor(12, 40);
        display->print(rowVals[4]);
    }
    if (strlen(rowVals[5]) > 0) {
        display->setCursor(12, 56);
        display->print(rowVals[5]);
    }
    display->display();
    
}

Interface::Interface(Adafruit_SSD1306* iDisplay, int iScreenWidth, int iScreenHeight) {
    display = iDisplay;
    screenWidth = iScreenWidth;
    screenHeight = iScreenHeight;
    //Clear
    for (int i=0; i<6; i++) {
        memset(rowVals[i], 0, 255);
    }
}

void Interface::mainMenu(float dist, float offset, float aim, bool isready) {
    sprintf(rowVals[0], "MAIN");
    sprintf(rowVals[1], "DIST  : %.3fm", dist);
    sprintf(rowVals[2], "OFF_X : %.3fcm", offset);
    sprintf(rowVals[3], "AIM_X : %.3fcm", aim);
    sprintf(rowVals[4], "READY : %s", (isready) ? "TRUE" : "FALSE");
    sprintf(rowVals[5], "GOONMOBILE V1");
    printInterface("ESRD");
}

void Interface::runMenu(float dist, float offset, float aim, float runtime) {
    sprintf(rowVals[0], "RUNCONF");
    sprintf(rowVals[1], "DIST  : %.3fm", dist);
    sprintf(rowVals[2], "OFF_X : %.3fcm", offset);
    sprintf(rowVals[3], "AIM_X : %.4fcm", aim);
    sprintf(rowVals[4], "RTIME : %.2fs", runtime);
    sprintf(rowVals[5], "GOONMOBILE V1");
    printInterface("OD X");
}

void Interface::distMenu(float dist, float increment) {
  sprintf(rowVals[0], "DIST");
    sprintf(rowVals[1], "DIST : %.3fm", dist);
    sprintf(rowVals[2], "INCR : %.3fm", increment);
    sprintf(rowVals[3], "\0");
    sprintf(rowVals[4], "\0");
    sprintf(rowVals[5], "GOONMOBILE V1");
    printInterface("+I-X");
}

//In cm
void Interface::offsetMenu(float offset, float increment) {
    sprintf(rowVals[0], "OFFSET");
    sprintf(rowVals[1], "OFF_X : %.3fcm", offset);
    sprintf(rowVals[2], "INCR  : %.3fcm", increment);
    sprintf(rowVals[3], "\0");
    sprintf(rowVals[4], "\0");
    sprintf(rowVals[5], "GOONMOBILE V1");
    printInterface("+I-X");
}

void Interface::settingMenu(char* ip, bool laserstate, bool telemetrystate, int controllermode) {
    sprintf(rowVals[0], "SETTING");
    sprintf(rowVals[1], "TELEM : %s", (telemetrystate) ? "TRUE" : "FALSE");
    sprintf(rowVals[2], "%s", ip);
    sprintf(rowVals[3], "LASER : %s", (laserstate) ? "TRUE" : "FALSE");
    sprintf(rowVals[4], "CTRLM : %d", controllermode);
    sprintf(rowVals[5], "GOONMOBILE V1");
    printInterface("TLCX");
}

void Interface::endScreen(float dist, float t) {
    sprintf(rowVals[0], "END");
    sprintf(rowVals[1], "RUN ENDED");
    sprintf(rowVals[2], "DIST : %.3fm", dist);
    sprintf(rowVals[3], "TIME : %.2fs", t);
    sprintf(rowVals[4], "\0");
    sprintf(rowVals[5], "GOONMOBILE V1");
    printInterface("   X");
}


void Interface::runScreen(float dist, float offset) {
    sprintf(rowVals[0], "RUNNING");
    sprintf(rowVals[1], "\0");
    sprintf(rowVals[2], "DIST  : %.3fm", dist);
    sprintf(rowVals[3], "OFF_X : %.3fcm", offset);
    sprintf(rowVals[4], "\0");
    sprintf(rowVals[5], "MCQUEEN V1");
    printInterface("   X");
}
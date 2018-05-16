#include "graphics.h"

void clearScreen(Adafruit_SharpMem& screen) {
    screen.clearDisplay();
}

void drawBackground(Adafruit_SharpMem& screen, bool right) {
    int X = 0,
        Y = 0;

    if (right) {
        screen.drawXBitmap(X, Y, final_v2_right_bits, WIDTH, HEIGHT, BLACK);
    }
    else {
        screen.drawXBitmap(X, Y, final_v2_left_bits, WIDTH, HEIGHT, BLACK);
    }
}

void drawString(Adafruit_SharpMem& screen, const char * str, int x, int y, int size) {
    screen.setCursor(x, y);
    screen.setTextSize(size);

    for (unsigned int i = 0; i < sizeof(str)/sizeof(str[0]) + 1; ++i) {
        screen.write(str[i]);  // `.write` moves cursor for every character
    }
}

void calcXYHforBar(const double& percent, uint16_t& x, uint8_t& y, uint8_t& h) {
    x = 174 * percent + 121;
    // left side
    if (percent <= 27/172.0) {
        h = (uint8_t)(182824.4 * exponential(percent, 4) - 52442.6 * exponential(percent, 3) \
                      + 5229 * percent * percent + 152.9 * percent);
        y = (uint8_t)(-172461.3 * exponential(percent, 4) + 47923.9 * exponential(percent, 3) \
                      - 4535.9 * percent * percent - 199.0 * percent + 234);
    }
    // right side
    else if (percent >= 143/172.0) {
        h = (uint8_t)(40094.4 * exponential(percent, 4) - 148114.5 * exponential(percent, 3) \
                  + 204809.6 * percent * percent - 125997.4 * percent + 29209.9);
        y = 168;
    }
    // middle
    else {
        h = 64;
        y = 168;
    }
}

void drawSpeed(Adafruit_SharpMem& screen, const float& speedVal) {
    const int     speedX = -7;
    const uint8_t speedY = 112;

    char speedStr[16];
    sprintf(speedStr, "%0.1f", speedVal);

    screen.setFont(&FreeMono18pt7b);
    screen.fillRect(speedX, speedY - 42, 160, 56, WHITE);
    drawString(screen, speedStr, speedX, speedY, 2);
}

void drawLapTime(Adafruit_SharpMem& screen, const int& lapTimeSeconds) {
    const uint8_t x = 167,
                  y = 55;

    // format time
    int mins = lapTimeSeconds / 60;
    int secs = lapTimeSeconds % 60;

    char str[16] = {0};
    sprintf(str, "%02u:%02u", mins, secs);

    screen.fillRect(x, y - 25, 110, 30, WHITE);
    screen.setFont(&FreeMono9pt7b);
    drawString(screen, str, x, y, 2);
}

void drawCurrentValue(Adafruit_SharpMem& screen, const double& currentVal) {
    const uint16_t x = 174;
    const uint8_t  y = 137;

    // format current
    char str[16] = {0};
    sprintf(str, "%-4.2f", currentVal);

    screen.fillRect(x, y - 25, 110, 30, WHITE);
    screen.setFont(&FreeMono9pt7b);
    drawString(screen, str, x, y, 2);
}

void getStateString(const int& state, char* str) {
    // OFF:0, ACCCEL: 1, BRAKE:2, IDLE: 3, ERROR: 4, ENGAGE: 5
    switch (state) {
        case 0:
            strcpy(str, "OFF");
            break;
        case 1:
            strcpy(str, "ACCEL");
            break;
        case 2:
            strcpy(str, "BRAKE");
            break;
        case 3:
            strcpy(str, "IDLE");
            break;
        case 4:
            strcpy(str, "ERROR");
            break;
        case 5:
            strcpy(str, "ENGAGE");
            break;
        default:
            strcpy(str, "N/A");
            break;
    }
}

void drawMotor1State(Adafruit_SharpMem& screen, const int& motor1) {
    // motor states
    // OFF:0, ACCCEL: 1, BRAKE:2, IDLE: 3, ERROR: 4, ENGAGE: 5

    const int x = 29,
              y = 27;

    char stateStr[8] = {0};
    getStateString(motor1, stateStr);

    screen.setFont(&FreeMono9pt7b);
    screen.fillRect(x, y - 25, 40, 30, WHITE);
    drawString(screen, stateStr, x, y, 1);
}

void drawMotor2State(Adafruit_SharpMem& screen, const int& motor2) {
    const int x = 29,
              y = 39;

    char stateStr[8] = {0};
    getStateString(motor2, stateStr);

    screen.setFont(&FreeMono9pt7b);
    screen.fillRect(x, y - 25, 40, 30, WHITE);
    drawString(screen, stateStr, x, y, 1);
}

void drawCC(Adafruit_SharpMem& screen, const bool& active, const int& throttle) {    
    // cc draw
    uint8_t x = 10;
    uint8_t y = 211;
    if (active) {
        // Display on screen
        screen.setFont(&FreeMono12pt7b);
        char str[32];
        sprintf(str, "CC %d", throttle);
        drawString(screen, str, x, y, 1);
    }
    else {
        // remove from screen
        screen.fillRect(x, y - 25, 100, 30, WHITE);
    }
}



void drawTimeLeft(Adafruit_SharpMem& screen, const int& timeLeft) {
    const int     x = 180;
    const uint8_t y = 145;

    int mins = timeLeft / 60;  // only interested in minutes

    char str[16];
    sprintf(str, "%2d", timeLeft);

    screen.fillRect(x, y - 84, 156, 90, WHITE);
    screen.setFont(&FreeMono18pt7b);
    drawString(screen, str, x, y, 3);
}

void drawLapCount(Adafruit_SharpMem& screen, volatile const uint8_t& count, const uint8_t& max) {
    const uint8_t x = 30;
    const uint8_t y = 123;

    char str[16];
    sprintf(str, "%u/%u", count, max);

    screen.fillRect(x, y - 25, 110, 30, WHITE);
    screen.setFont(&FreeMono9pt7b);
    drawString(screen, str, x, y, 2);
}

void drawBestAndAvgLapTime(Adafruit_SharpMem& screen, volatile const int* lapTimes, const uint8_t& size) {
    long avg = 0;  // millis
    int min = 0xffff;  // millis
    uint8_t count = 0;
    for (int i = 0; i < size; ++i) {
        int elem = lapTimes[i];

        if (elem > 0) {
            ++count;

            avg += elem;
            if (min > elem) {
                min = elem;
            }
        }
    }
    avg = avg / count;
    if (min >= 0xfff0) min = 0;  // to get zeros if nothing is recorded yet


    // best lap time
    uint8_t x = 38;
    uint8_t y = 172;

    int mins = (min / 1000) / 60;
    int secs = (min / 1000) % 60;

    char str[16] = {0};
    sprintf(str, "%02u:%02u", mins, secs);

    screen.fillRect(x, y - 23, 110, 30, WHITE);
    screen.setFont(&FreeMono9pt7b);
    drawString(screen, str, x, y, 2);


    // avg lap time
    x = 62;
    y = 223;

    mins = (avg / 1000) / 60;
    secs = (avg / 1000) % 60;

    char str2[16] = {0};
    sprintf(str2, "%02u:%02u", mins, secs);

    screen.fillRect(x, y - 25, 110, 30, WHITE);
    drawString(screen, str2, x, y, 2);
}

void drawVoltageValue(Adafruit_SharpMem& screen, const double& voltageVal) {
    const uint8_t x = 32;
    const uint8_t y = 60;

    char str[16];
    sprintf(str, "%-4.2f", voltageVal);

    screen.fillRect(x, y - 25, 110, 30, WHITE);
    screen.setFont(&FreeMono9pt7b);
    drawString(screen, str, x, y, 2);
}
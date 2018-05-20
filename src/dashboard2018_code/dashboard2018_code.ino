#include "graphics.h"
#include "can.h"
#include "lights.h"
#include "helpers.h"
#include <TimerOne.h>

/* COMM PARAMETERS */
// SCREEN 1
#define S1_SCK  13
#define S1_MOSI 11
#define S1_CS   10

// SCREEN 2
#define S2_SCK  32
#define S2_MOSI 21
#define S2_CS   31

// LIGHTS
#define PIN_BACKLIGHT   35
#define PIN_FRONTLIGHT   36
#define PIN_SWHEEL_LIGHT  17


#define SPEED_SCALAR   10.0
#define CURRENT_SCALAR 10.0
#define VOLTAGE_SCALAR 10.0
#define ENERGY_SCALAR  10.0
#define MAX_CURRENT    10.0

#define LAP_BUTTON_COOLDOWN_SECS 5

// BUTTONS
// From steering wheel
#define PIN_THROTTLE        22
#define PIN_POT_LEFT        34
#define PIN_DEADMAN_SWITCH  15
#define PIN_BLINKER_L       20
#define PIN_BLINKER_R       24
#define PIN_LAP_BUTTON      39
#define PIN_HORN            23
#define PIN_CC              19
#define PIN_OPTIMAL_CURRENT 38
#define PIN_OPTIMAL_BRAKE   14
#define PIN_BLANK           27

// From dashboard
#define PIN_BRAKE_ENABLED    26
#define PIN_LIGHT_ENABLE     28
#define PIN_MAIN_SWITCH      37  // is this needed? everything turns off with the emergency switch anyways...
#define PIN_HAZARD_LIGHT     16
#define PIN_GEAR_2           29  // up
#define PIN_GEAR_1           25  // down
#define PIN_RESET            30
#define PIN_GEAR_AUTO_MANUAL 12

// precalculated bitmask (1<<bp) (bp = bit position) is supposed to be a little bit faster
#define GEAR_2_bp 1
#define GEAR_2_bm (1<<GEAR_2_bp)
#define GEAR_1_bp 0
#define GEAR_1_bm (1<<GEAR_1_bp)
#define GEAR_AUTO_MANUAL_bp 2
#define GEAR_AUTO_MANUAL_bm (1<<GEAR_AUTO_MANUAL_bp)

// Debug
bool debug = false;

// RACE INFO
#define TOTAL_LAPS 15
#define OPTIMAL_CURRENT_VAL 60  // these are calculated beforehand
#define OPTIMAL_BRAKE_VAL   60

/* PUBLIC VARIABLES */
// SCREEN
Adafruit_SharpMem screen1(S1_SCK, S1_MOSI, S1_CS, WIDTH, HEIGHT);
Adafruit_SharpMem screen2(S2_SCK, S2_MOSI, S2_CS, WIDTH, HEIGHT);
enum ORIENTATION { UP = 0, LEFT = 1, DOWN = 2, RIGHT = 3 };
// UP is where the flap is on the screen case

// LIGHTS
// Important to have the last argument in the initalization of the light objects
Adafruit_NeoPixel backLights(NUM_BACKLIGHTS, PIN_BACKLIGHT, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel frontLights(NUM_FRONTLIGHTS, PIN_FRONTLIGHT, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel swheelLights(NUM_SWHEEL_LIGHTS, PIN_SWHEEL_LIGHT, NEO_GRBW + NEO_KHZ800);

// CANbus
CAN_message_t txmsg, rxmsg1, rxmsg2;
TimerOne t1;

// TIMING
unsigned long millisStart = millis();
unsigned long millisEnd   = millis();
int timeElapsedLastLoop = 0;

int leftBlinkLightTotalElapsedTime  = 0;
int rightBlinkLightTotalElapsedTime = 0;
bool leftBlinkState  = false;
bool rightBlinkState = false;

unsigned long minuteCounterMillis = 0;
int secondCounter = 0;
int secondCounterPrev = 0;
const int MINUTES_TOTAL = 40;
int minutesRemaining = MINUTES_TOTAL;

volatile int lapTimeMillis = 0;
volatile int lapTimes[TOTAL_LAPS] = {0};

volatile bool    lapButtonPressedRecently = true;
volatile uint8_t lapButtonCooldown = LAP_BUTTON_COOLDOWN_SECS;

// RACE DATA
volatile uint8_t lapCount = 1;  // must be volatile to be changed in isr
double barWidth = 0;
double barWidth_prev = 0;

volatile bool newLap = false;  // true when lap count button is pressed

float motor1totalEnergy = 0;  // kJ
float motor1totalEnergy_prev = 0;
float motor2totalEnergy = 0;  // kJ
float motor2totalEnergy_prev = 0;

float motor1speed = 0;
float motor2speed = 0;

int motor1state = -1;
int motor2state = -1;

int motor1temp = 0;
int motor2temp = 0;

int motor1clutch = 0;
int motor2clutch = 0;

double motor1current = 0;
double motor2current = 0;

int throttle = 0;
int throttleRaw = 0;
static const int THROTTLE_HIGH = 100;
static const int REGEN_HIGH = 100;

int regen = 0;  // amount of regen braking
int regenRaw = 0;

// this will be a binary number where each bit represents one button's state
// the bits are as follows (LSB first in list):
// 0: gear1 on(1) / off(0)
// 1: gear2 on(1) / off(0)
// 2: auto(1) / manual(0)
volatile uint8_t buttons;

volatile bool brakeEnabled = false;

float voltage = 0;

const bool CC_ENABLED = false;
bool ccActive = false;

bool optimalCurrent = false;
bool optimalBrake = false;

// Deadman switch
volatile bool deadmanSwitchIsPressed = false;
volatile unsigned long millisAtLastChange = 0;

// Blinkers
volatile bool leftBlinkerPressed  = false;
volatile bool rightBlinkerPressed = false;
volatile bool hazardLightActive = false;

// Lights
volatile bool drivingLightsEnabled = false;  // this is set if driving lights should be turned on

/* INIT FUNCTIONS */
void initScreen1() {
    screen1.begin();
    screen1.setRotation(ORIENTATION::DOWN);
    screen1.setFont(&FreeMono9pt7b);
    screen1.setTextColor(BLACK, WHITE);
    clearScreen(screen1);

    screen1.refresh();
}

void initScreen2() {
    screen2.begin();
    screen2.setRotation(ORIENTATION::DOWN);
    screen2.setFont(&FreeMono9pt7b);
    screen2.setTextColor(BLACK, WHITE);
    clearScreen(screen2);

    screen2.refresh();
}

void initLights() {
    backLights.setBrightness(BRIGHTNESS_BACK);
    frontLights.setBrightness(BRIGHTNESS_FRONT);
    swheelLights.setBrightness(BRIGHTNESS_SWHEEL);

    backLights.begin();
    frontLights.begin();
    swheelLights.begin();

    turnOffStrip(backLights);
    turnOffStrip(frontLights);
    turnOffStrip(swheelLights);

    backLights.show();
    frontLights.show();
    swheelLights.show();
}

void initCANbus() {
    // can't get CAN to work locally, had to opt for CANtoUART library
    Serial3.begin(SERIAL_BAUDRATE);
}

void initSerial() {
    // to communicate with the computer
    Serial.begin(SERIAL_BAUDRATE);
}

void initTimer1() {
    const int T1_PERIOD_US = 100000; // 100 ms
    t1.initialize(100000);
    t1.attachInterrupt(t1_OVF_ISR);
    t1.start();
}

void initPins() {
    // Steering wheel buttons
    pinMode(PIN_THROTTLE, INPUT);  // pot 

    pinMode(PIN_DEADMAN_SWITCH, INPUT_PULLUP);  // push button
    attachInterrupt(digitalPinToInterrupt(PIN_DEADMAN_SWITCH), deadmanSwtichChanged_ISR, CHANGE);

    pinMode(PIN_LAP_BUTTON, INPUT_PULLUP);  // push button, debounced
    attachInterrupt(digitalPinToInterrupt(PIN_LAP_BUTTON), resetLapTimeAndIncrementLapCount_ISR, FALLING);

    pinMode(PIN_BLINKER_L, INPUT_PULLUP);  // push button
    attachInterrupt(digitalPinToInterrupt(PIN_BLINKER_L), leftBlinkerChanged_ISR, CHANGE);

    pinMode(PIN_BLINKER_R, INPUT_PULLUP);  // push button
    attachInterrupt(digitalPinToInterrupt(PIN_BLINKER_R), rightBlinkerChanged_ISR, CHANGE);

    pinMode(PIN_HORN, INPUT_PULLUP);  // push button, debounced
    // The horn is controlled directly from a relay in the dashbaord

    pinMode(PIN_CC, INPUT_PULLUP);  // push button, debounced
    attachInterrupt(digitalPinToInterrupt(PIN_CC), ccButtonPressed_ISR, FALLING);

    pinMode(PIN_OPTIMAL_CURRENT, INPUT_PULLUP);  // push button, debounced
    attachInterrupt(digitalPinToInterrupt(PIN_OPTIMAL_CURRENT), optimalCurrentButtonChanged_ISR, CHANGE);

    pinMode(PIN_OPTIMAL_BRAKE, INPUT_PULLUP);  // push button, debounced
    attachInterrupt(digitalPinToInterrupt(PIN_OPTIMAL_BRAKE), optimalBrakeButtonChanged_ISR, CHANGE);

    pinMode(PIN_BLANK, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BLANK), blankButtonPressed_ISR, FALLING);


    // Dashboard buttons
    pinMode(PIN_LIGHT_ENABLE, INPUT_PULLUP);  // flip switch
    attachInterrupt(digitalPinToInterrupt(PIN_LIGHT_ENABLE), lightsEnable_ISR, CHANGE);

    pinMode(PIN_BRAKE_ENABLED, INPUT_PULLUP);  // push button (at pedal)
    attachInterrupt(digitalPinToInterrupt(PIN_BRAKE_ENABLED), brakeButtonChanged_ISR, CHANGE);

    pinMode(PIN_HAZARD_LIGHT, INPUT_PULLUP);  // flip switch
    attachInterrupt(digitalPinToInterrupt(PIN_HAZARD_LIGHT), hazardLightButtonChanged_ISR, CHANGE);

    pinMode(PIN_GEAR_2, INPUT_PULLUP);  // flip switch, 3-way, up pos
    pinMode(PIN_GEAR_1, INPUT_PULLUP);  // flip switch, 3-way, down pos
    pinMode(PIN_GEAR_AUTO_MANUAL, INPUT_PULLUP);  // flip switch, 2-way
    attachInterrupt(digitalPinToInterrupt(PIN_GEAR_2), gear2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_GEAR_1), gear1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_GEAR_AUTO_MANUAL), gearAutoManual_ISR, CHANGE);

    pinMode(PIN_RESET, INPUT_PULLUP);  // push button
    attachInterrupt(digitalPinToInterrupt(PIN_RESET), reset, FALLING);
}

void reset() {
    lapCount = 1;

    for (int i = 0; i < TOTAL_LAPS; ++i) {
        lapTimes[i] = 0;
    }

    minutesRemaining = MINUTES_TOTAL;
    secondCounter = 0;
    minuteCounterMillis = 0;
    lapTimeMillis = 0;

    drawTimeLeft(screen2, minutesRemaining);
    drawLapTime(screen1, lapTimeMillis / 1000);

    newLap = true;
    screen2.refresh();
    Serial.print("  RESET!!  ");
}

void initScreensContent() {
    // Draw background only once
    drawBackground(screen1, true);
    drawBackground(screen2, false);

    drawTimeLeft(screen2, minutesRemaining);
    drawVoltageValue(screen2, voltage);
    drawCurrentValue(screen1, motor1current, motor2current);

    const char mins[] = "mins";
    screen2.setFont(&FreeMono9pt7b);
    drawString(screen2, mins, 260, 163, 1);

    const char kmh[] = "km/h";
    screen1.setFont(&FreeMono9pt7b);
    drawString(screen1, kmh, 90, 170, 1);
    drawSpeed(screen1, motor1speed, motor2speed);

    newLap = true;  // to get the left screen (screen2) to actually display something


    // setup for motor states
    const char m1[] = "M1:";
    const char m2[] = "M2:";
    screen1.setFont(&FreeMono9pt7b);
    drawString(screen1, m1, 6, 27, 1);
    drawString(screen1, m2, 6, 41, 1);
    
    drawMotor1State(screen1, motor1state);
    drawMotor2State(screen1, motor2state);

    // setup for motor temps
    const char t1[] = "T1:";
    const char t2[] = "T2:";
    screen2.setFont(&FreeMono12pt7b);
    drawString(screen2, t1, 204, 201, 1);
    drawString(screen2, t2, 204, 221, 1);

    drawTemperature(screen2, motor1temp, motor2temp);

    // setup for motor clutch
    const char c1[] = "C1:";
    const char c2[] = "C2:";
    screen1.setFont(&FreeMono12pt7b);
    drawString(screen1, c1, 6, 201, 1);
    drawString(screen1, c2, 6, 221, 1);

    drawClutch(screen1, motor1clutch, motor2clutch);

    // setup for total energy
    const char e1[] = "kJ1:";
    const char e2[] = "kJ2:";
    screen2.setFont(&FreeMono9pt7b);
    drawString(screen2, e1, 192, 35, 1);
    drawString(screen2, e2, 192, 55, 1);
    drawTotalEnegry(screen2, motor1totalEnergy, motor2totalEnergy);
}

void initSteeringWheelLightShow() {
    // light show at start-up
    for (int i = 0; i < swheelLights.numPixels(); ++i) {
        swheelLights.setPixelColor(swheelLights.numPixels() - i - 1, COLOR_SWHEELLIGHTS);
        swheelLights.show();
        delay(100);
    }
}

/* FUNCTIONS */
void readThrottle() {
    static const int THROTTLE_RAW_LOW  = 404;
    static const int THROTTLE_RAW_HIGH = 530;
    static const int THROTTLE_LOW  = 0;

    if (!(ccActive && CC_ENABLED)) {
        // only read new throttle command if cc is inactive (i.e. off)
        throttleRaw = analogRead(PIN_THROTTLE);
    }
    else {
        // is active (i.e. on)
        Serial.print(" CC ");
    }
    throttle = map(throttleRaw, THROTTLE_RAW_LOW, THROTTLE_RAW_HIGH, THROTTLE_LOW, THROTTLE_HIGH);

    static const int THROTTLE_THRESHOLD = 2;
    if (throttle <= THROTTLE_THRESHOLD) {
        throttle = THROTTLE_LOW;
    }
    else if (throttle >= THROTTLE_HIGH - THROTTLE_THRESHOLD) {
        throttle = THROTTLE_HIGH;
    }

    if (optimalCurrent) {
        throttle = OPTIMAL_CURRENT_VAL;
        Serial.print(" OPTIMAL CURRENT ");

        // to get cc to work corretly
        // +1 at end because map returned 29...
        throttleRaw = map(throttle, THROTTLE_LOW, THROTTLE_HIGH, THROTTLE_RAW_LOW, THROTTLE_RAW_HIGH) + 1;
    }

    Serial.print(" T: "); Serial.print(throttleRaw); Serial.print("-"); Serial.print(throttle); Serial.print(" ");    
}

void readRegen() {
    static const int REGEN_RAW_LOW  = 436;
    static const int REGEN_RAW_HIGH = 542;
    static const int REGEN_LOW  = 0;

    regenRaw = analogRead(PIN_POT_LEFT);
    regen = map(regenRaw, REGEN_RAW_LOW, REGEN_RAW_HIGH, REGEN_LOW, REGEN_HIGH);

    static const int REGEN_THRESHOLD = 3;
    if (regen <= REGEN_THRESHOLD) {
        regen = REGEN_LOW;
    }
    else if (regen >= REGEN_HIGH - REGEN_THRESHOLD) {
        regen = REGEN_HIGH;
    }

    if (optimalBrake) {
        regen = OPTIMAL_BRAKE_VAL;
        Serial.print(" OPTIMAL BRAKE ");
    }

    if (regen != 0) {
        ccActive = false;
    }

    if (regen > 0) {
        brakeEnabled = true;
        drivingLightsBack(backLights);
    }
    else {
        brakeEnabled = false;

        if (!drivingLightsEnabled) {
            turnOffStrip(backLights);
        }

    }

    Serial.print(" R: "); Serial.print(regenRaw); Serial.print("-"); Serial.print(regen); Serial.print(" ");    
}

void animateSliderBar(const double& barWidth, bool inverted) {
    // inverted: 0 (false): fill black, 1 (true): fill white
    // barWidth between 0..1
    static const uint8_t RBAR_HEIGHT = 108;
    static const uint8_t RBAR_WIDTH  = 123;

    // animate sliders for testing
    static const double BAR_UPDATE_MIN_DIFF = 0.04;
    // barWidth = (float)throttle / THROTTLE_HIGH;  // display throttle on on-screen bar
    // barWidth = current / MAX_CURRENT;  // display motor current on on-screen bar
    int barUpdateCount = 0;

    uint8_t y = 168;
    uint16_t x = 0;
    uint8_t h = 0;
    // have to be at least `BAR_UPDATE_MIN_DIFF` difference to be necessary to update
    if ( abs(barWidth - barWidth_prev) >= BAR_UPDATE_MIN_DIFF) {
        barUpdateCount = 1;
        for (double i = 0; i < 1; i += 1.0 / RBAR_WIDTH) {
            // leftmost
            calcXYHforBar(i, x, y, h);
            if (i < barWidth) {
                screen1.fillRect(x, y, 2, h, inverted);
            }
            else {
                screen1.fillRect(x, y, 2, h, !inverted);
            }
        }
    }

    if (barUpdateCount) {
        // we only want to update this value IF there has been a change, but not inside of for loop
        // since that updates it before the whole bar is drawn.
        barWidth_prev = barWidth;
        barUpdateCount = 0;
    }
}

void doLightStuffLikeBlinkLightsAndDrivingLightsOnOrOff() {
    // make lights blink with 700 ms timing
    // if hazardLightActive = true, both lights should blink
    if (leftBlinkerPressed || hazardLightActive) {
        leftBlinkLightTotalElapsedTime += timeElapsedLastLoop;
        if (leftBlinkLightTotalElapsedTime >= BLINKERS_PERIOD) {  // milli seconds
            leftBlinkLightTotalElapsedTime = 0;
            leftBlinkState = !leftBlinkState;
            if (leftBlinkState) {
                blinkLights(backLights, BLINK_LEFT_START_BACKLIGHTS, BLINK_LEFT_END_BACKLIGHTS);
                blinkLights(frontLights, BLINK_LEFT_START_FRONTLIGHTS, BLINK_LEFT_END_FRONTLIGHTS);
            }
            else {
                turnOffStrip(backLights, BLINK_LEFT_START_BACKLIGHTS, BLINK_LEFT_END_BACKLIGHTS);
                turnOffStrip(frontLights, BLINK_LEFT_START_FRONTLIGHTS, BLINK_LEFT_END_FRONTLIGHTS);
            }
        }
    }

    if (rightBlinkerPressed || hazardLightActive) {
        rightBlinkLightTotalElapsedTime += timeElapsedLastLoop;
        if (rightBlinkLightTotalElapsedTime >= BLINKERS_PERIOD) {  // milli seconds
            rightBlinkLightTotalElapsedTime = 0;
            rightBlinkState = !rightBlinkState;
            if (rightBlinkState) {
                blinkLights(backLights, BLINK_RIGHT_START_BACKLIGHTS, BLINK_RIGHT_END_BACKLIGHTS);
                blinkLights(frontLights, BLINK_RIGHT_START_FRONTLIGHTS, BLINK_RIGHT_END_FRONTLIGHTS);
            }
            else {
                turnOffStrip(backLights, BLINK_RIGHT_START_BACKLIGHTS, BLINK_RIGHT_END_BACKLIGHTS);
                turnOffStrip(frontLights, BLINK_RIGHT_START_FRONTLIGHTS, BLINK_RIGHT_END_FRONTLIGHTS);
            }
        }
    }

    if (drivingLightsEnabled && !(rightBlinkerPressed || leftBlinkerPressed || hazardLightActive)) {
        // this can be optimized by only updating IF the blinkers have been unpressed
        drivingLightsFront(frontLights);
        drivingLightsBack(backLights);
    }
    else if (!drivingLightsEnabled && !(rightBlinkerPressed || leftBlinkerPressed || hazardLightActive)) {
        turnOffStrip(frontLights);

        if(!brakeEnabled) {
            turnOffStrip(backLights);
        }
    }
}

void displayLapTime(const volatile int& lapTimeMillis) {
    // display lap time on screen
    // `lapTimeMillis % 1000` is always between 0 and 999. When the condition is true, it means
    // it's less than 150 ms since a new second and the lap time should update on screen.
    static const uint8_t LAP_TIME_MILLIS_UPDATE_THRESHOLD = 150;  // ms
    if (lapTimeMillis % 1000 < LAP_TIME_MILLIS_UPDATE_THRESHOLD) {
        drawLapTime(screen1, lapTimeMillis / 1000);
    }
}

void parseMotor1msg(const CAN_message_t& msg) {
    motor1state = msg.buf[0];
    motor1speed = msg.buf[6] / SPEED_SCALAR;
    motor1current = msg.buf[1] / CURRENT_SCALAR;
    motor1temp = msg.buf[7];

    motor1totalEnergy = (msg.buf[5] << 8 | msg.buf[4]) / ENERGY_SCALAR;
}

void parseMotor2msg(const CAN_message_t& msg) {
    motor2state = msg.buf[0];
    motor2speed = msg.buf[6] / SPEED_SCALAR;
    motor2current = msg.buf[1] / CURRENT_SCALAR;
    motor2temp = msg.buf[7];

    motor2totalEnergy = (msg.buf[5] << 8 | msg.buf[4]) / ENERGY_SCALAR;
}

void parseClutch1msg(const CAN_message_t& msg) {
    motor1clutch = msg.buf[2];
}

void parseClutch2msg(const CAN_message_t& msg) {
    motor2clutch = msg.buf[2];
}

/* MAIN PROGRAM */
void setup() {
    initScreen1();
    initScreen2();
    initSerial();
    initCANbus();
    initLights();
    initPins();
    initTimer1();
    initScreensContent();
    initSteeringWheelLightShow();

    debug = true; // give lots of information over serial
}

void loop() {
    timeElapsedLastLoop = millisEnd - millisStart;
    Serial.print(secondCounter);
    Serial.print(" Time: ");
    Serial.print(timeElapsedLastLoop);
    Serial.print(" ms  ");
    millisStart = millis();
    
    minuteCounterMillis += timeElapsedLastLoop;
    secondCounter = minuteCounterMillis / 1000;

    if (abs(secondCounter - secondCounterPrev) >= 1) {  // one second ellapsed
        secondCounterPrev = secondCounter;

        // below code is to make sure button in not accedently pressed multiple times in a row
        --lapButtonCooldown;
        if (lapButtonCooldown == 0) {
            lapButtonPressedRecently = false;
        }
    }

    // lap time
    lapTimeMillis += timeElapsedLastLoop;
    // reset lapTimeMillis on lap time button press interrupt


    // Throttle
    readThrottle();
    readRegen();

    // Only send if deadman switch was unpressed less than 2 secs ago
    static const int DEADMAN_SWITCH_UNPRESSED_THRESHOLD = 2000;  // ms
    if (abs(millisStart - millisAtLastChange) <= DEADMAN_SWITCH_UNPRESSED_THRESHOLD) {
        // Send message
        Serial.print(" CAN DRIVE   ");
        if (deadmanSwitchIsPressed) {
            millisAtLastChange = millis();  // have to update each time to be able to continue being within the boundaries
        }

        txmsg.id = DASHBOARD_CAN_ID;
        txmsg.len = 6;
        txmsg.buf[0] = 0;
        txmsg.buf[1] = buttons;
        txmsg.buf[2] = regen;
        txmsg.buf[3] = throttle;
        //sendCANoverUART(txmsg);  // this is done in the timer interrupt

        if (debug) {
            printTXmsg(txmsg);
        }
    }
    else {
        // to make sure no throttle or regen command is sent when the dms is unpressed
        txmsg.buf[0] = 0;
        txmsg.buf[1] = buttons;
        txmsg.buf[2] = 0;
        txmsg.buf[3] = 0;
        Serial.print(" CAN'T DRIVE ");
    }


    // Lights
    doLightStuffLikeBlinkLightsAndDrivingLightsOnOrOff();


    const int CAN_LEN = 25;
    char canBuffer[CAN_LEN * 2] = {0};
    readCANfromUARTtoBuffer(canBuffer);
    // Serial.print(canBuffer);
    parseUARTbufferToCANmessage(canBuffer, rxmsg1, rxmsg2);
    // Serial.print(canBuffer);
    // Serial.print(" ID1: ");  Serial.print(rxmsg1.id);
    //Serial.print("  ID2: "); Serial.print(rxmsg2.id);

    bool inverted = false;
    if (throttle > 0) {
        barWidth = (float)throttle / THROTTLE_HIGH;
    }
    else if (regen > 0) {
        barWidth = 1 - (float)regen / REGEN_HIGH;
        inverted = true;
    }
    else {
        barWidth = 0;
    }
    animateSliderBar(barWidth, inverted);
    displayLapTime(lapTimeMillis);


    bool motorCANreceived = false;
    CAN_message_t motorMsg;

    // get correct message
    if (rxmsg1.id == MOTOR_1_STATUS_CAN_ID) {  // || rxmsg2.id == MOTOR_1_STATUS_CAN_ID) {
        motorCANreceived = true;

        if (rxmsg1.id == MOTOR_1_STATUS_CAN_ID) {
            motorMsg = rxmsg1;
            parseMotor1msg(rxmsg1);
        }
        // if (rxmsg2.id == MOTOR_1_STATUS_CAN_ID) {
        //     motorMsg = rxmsg2;
        //     parseMotor1msg(rxmsg2);
        // }
    }

    if (rxmsg1.id == MOTOR_2_STATUS_CAN_ID) { // || rxmsg2.id == MOTOR_2_STATUS_CAN_ID) {
        motorCANreceived = true;

        if (rxmsg1.id == MOTOR_2_STATUS_CAN_ID) {
            motorMsg = rxmsg1;
            parseMotor2msg(rxmsg1);
        }
        // if (rxmsg2.id == MOTOR_2_STATUS_CAN_ID) {
        //     motorMsg = rxmsg2;
        //     parseMotor2msg(rxmsg2);
        // }
    }

    if (motorCANreceived) {
        drawSpeed(screen1, motor1speed, motor2speed);

        voltage = (motorMsg.buf[3] << 8 | motorMsg.buf[2]) / VOLTAGE_SCALAR;

        drawCurrentValue(screen1, motor1current, motor2current);
        drawVoltageValue(screen2, voltage);

        drawMotor1State(screen1, motor1state);
        drawMotor2State(screen1, motor2state);

        drawTemperature(screen2, motor1temp, motor2temp);

        static const int ENERGY_CHANGE_THRESHOLD = 2;  // kJ
        if (abs(motor1totalEnergy - motor1totalEnergy_prev) >= ENERGY_CHANGE_THRESHOLD ||
            abs(motor2totalEnergy - motor2totalEnergy_prev) >= ENERGY_CHANGE_THRESHOLD) {
            motor1totalEnergy_prev = motor1totalEnergy;
            motor2totalEnergy_prev = motor2totalEnergy;

            drawTotalEnegry(screen2, motor1totalEnergy, motor2totalEnergy);
            screen2.refresh();
        }
    }


    bool clutchCANreceive = false;
    if (rxmsg1.id == E_CLUTCH_1_CAN_ID) { // || rxmsg2.id == E_CLUTCH_1_CAN_ID) {
        clutchCANreceive = true;

        if (rxmsg1.id == E_CLUTCH_1_CAN_ID) {
            parseClutch1msg(rxmsg1);
        }
        // if (rxmsg2.id == E_CLUTCH_1_CAN_ID) {
        //     parseClutch1msg(rxmsg2);
        // }
    }

    if (rxmsg1.id == E_CLUTCH_2_CAN_ID) {  // || rxmsg2.id == E_CLUTCH_2_CAN_ID) {
        clutchCANreceive = true;

        if (rxmsg1.id == E_CLUTCH_2_CAN_ID) {
            parseClutch2msg(rxmsg1);
        }
        // if (rxmsg2.id == E_CLUTCH_2_CAN_ID) {
        //     parseClutch2msg(rxmsg2);
        // }
    }

    if (clutchCANreceive) {
        drawClutch(screen1, motor1clutch, motor2clutch);
    }


    // minutes remaining
    if (secondCounter >= 60) {
        secondCounter = 0;
        minuteCounterMillis = 0;
        --minutesRemaining;

        Serial.print(" Minutes remaining: "); Serial.print(minutesRemaining); Serial.print(" ");
        drawTimeLeft(screen2, minutesRemaining);
        screen2.refresh();
    }

    if (secondCounter >= 30) {
        // this is just to update the screen every half a minute 
        screen2.refresh(); 
    }


    // prints whole can message
    if (debug) {
        printEntireCANmsg(rxmsg1, 1);
        // printEntireCANmsg(rxmsg2, 2);
    }

    screen1.refresh();

    if (newLap) {  // this means the lap increment button has been pressed
        drawLapCount(screen2, lapCount, TOTAL_LAPS);
        drawBestAndAvgLapTime(screen2, lapTimes, TOTAL_LAPS);
        newLap = false;
        screen2.refresh();
    }

    Serial.println();  // to get a newline
    millisEnd = millis();
}

/* ISRs */
void resetLapTimeAndIncrementLapCount_ISR() {
    // if statement to make sure button in not accedently pressed multiple times in a row
    if (!lapButtonPressedRecently) {
        lapButtonPressedRecently = true;
        lapButtonCooldown = LAP_BUTTON_COOLDOWN_SECS;
        lapTimes[lapCount-1] = lapTimeMillis;
        //lapCount = (lapCount + TOTAL_LAPS) % TOTAL_LAPS + 1;  // to not crash after 10 laps
        ++lapCount;
        if (lapCount > TOTAL_LAPS) {
            reset();
        }
        lapTimeMillis = 0;
        newLap = true;
        Serial.print("  NEW LAP!  ");
    }
    else {
        Serial.print("  TOO SOON  ");
    }
}

void deadmanSwtichChanged_ISR() {
    int state = digitalRead(PIN_DEADMAN_SWITCH);

    if (state == HIGH) {  // RISING -- is unpressed
        deadmanSwitchIsPressed = false;

        millisAtLastChange = millis();
        deadmanSwitchNotPressed(swheelLights);
        Serial.print("  DMS UNPRESSED!  ");
    }
    else {  
        deadmanSwitchIsPressed = true;

        millisAtLastChange = millis();
        turnOffStrip(swheelLights);

        Serial.print("  DMS PRESSED!  ");
    }
}

void leftBlinkerChanged_ISR() {
    int state = digitalRead(PIN_BLINKER_L);

    if (state == HIGH) {  // RISING -- is unpressed
        leftBlinkerPressed = false;
        // to make sure lights are turned off when button is depressed
        turnOffStrip(backLights, BLINK_LEFT_START_BACKLIGHTS, BLINK_LEFT_END_BACKLIGHTS);
        turnOffStrip(frontLights, BLINK_LEFT_START_FRONTLIGHTS, BLINK_LEFT_END_FRONTLIGHTS);

        Serial.print("  BLINK L UNPRESSED  ");
    }
    else {
        leftBlinkerPressed = true;
        Serial.print("  BLINK L PRESSED  ");
    }
}

void rightBlinkerChanged_ISR() {
    int state = digitalRead(PIN_BLINKER_R);

    if (state == HIGH) {  // RISING -- is unpressed
        rightBlinkerPressed = false;
        // to make sure lights are turned off when button is depressed
        turnOffStrip(backLights, BLINK_RIGHT_START_BACKLIGHTS, BLINK_RIGHT_END_BACKLIGHTS);
        turnOffStrip(frontLights, BLINK_RIGHT_START_FRONTLIGHTS, BLINK_RIGHT_END_FRONTLIGHTS);

        Serial.print("  BLINK R UNPRESSED  ");
    }
    else {
        rightBlinkerPressed = true;
        Serial.print("  BLINK R PRESSED  ");
    }
}

void lightsEnable_ISR() {
    int state = digitalRead(PIN_LIGHT_ENABLE);

    if (state == HIGH) {  // RISING -- is unpressed
        drivingLightsEnabled = false;
        Serial.print("  LIGHTS OFF  ");
    }
    else {
        drivingLightsEnabled = true;
        Serial.print("  LIGHTS ON  ");
    }
}

void brakeButtonChanged_ISR() {
    // this button is reversed
    int state = digitalRead(PIN_BRAKE_ENABLED);

    if (state == HIGH) {  // RISING -- is unpressed
        // brake lights on
        backLights.setBrightness(BRIGHTNESS_BACK_BRAKE);
        brakeEnabled = true;
        drivingLightsBack(backLights);
        Serial.print("  BRAKES ON  ");
    }
    else {
        // brake lights off
        backLights.setBrightness(BRIGHTNESS_BACK);
        brakeEnabled = false;
        if (!drivingLightsEnabled) {
            turnOffStrip(backLights);
        }
        Serial.print("  BRAKES OFF  ");

        ccActive = false;
    }
}

void hazardLightButtonChanged_ISR() {
    int state = digitalRead(PIN_HAZARD_LIGHT);

    if (state == HIGH) {  // RISING -- is unpressed
        hazardLightActive = false;
        Serial.print("  HAZARD OFF  ");
    }
    else {
        hazardLightActive = true;
        Serial.print("  HAZARD ON  ");
    }
}

void ccButtonPressed_ISR() {
    // cc = cruise control
    // this should be OK since the button is debounced with a capactior
    ccActive = !ccActive;
    Serial.print(" CC TGL  ");
}

void optimalCurrentButtonChanged_ISR() {
    int state = digitalRead(PIN_OPTIMAL_CURRENT);

    if (state == HIGH) {  // RISING -- is unpressed
        optimalCurrent = false;
        Serial.print("  OPT-CURR OFF  ");
    }
    else {
        optimalCurrent = true;
        Serial.print("  OPT-CURR ON  ");
    }
}

void optimalBrakeButtonChanged_ISR() {
    int state = digitalRead(PIN_OPTIMAL_BRAKE);

    if (state == HIGH) {  // RISING -- is unpressed
        optimalBrake = false;
        Serial.print("  OPT-BRAKE OFF  ");
    }
    else {
        optimalBrake = true;
        Serial.print("  OPT-BRAKE ON  ");
    }
}

void blankButtonPressed_ISR() {
    reset();
    Serial.print("  BLANK  ");
}

void gear2_ISR() {
    // up
    int state = digitalRead(PIN_GEAR_2);

    if (state == HIGH) {  // RISING -- is unpressed
        buttons &= ~GEAR_2_bm;
        Serial.print("  GEAR 2 OFF  ");
    }
    else {
        buttons |= GEAR_2_bm;
        Serial.print("  GEAR 2 ON  ");
    }
}

void gear1_ISR() {
    // down
    int state = digitalRead(PIN_GEAR_1);

    if (state == HIGH) {  // RISING -- is unpressed
        buttons &= ~GEAR_1_bm;
        Serial.print("  GEAR 1 OFF  ");
    }
    else {
        buttons |= GEAR_1_bm;
        Serial.print("  GEAR 1 ON  ");
    }
}

void gearAutoManual_ISR() {
    int state = digitalRead(PIN_GEAR_AUTO_MANUAL);

    if (state == HIGH) {  // RISING -- is unpressed
        // manual
        buttons &= ~GEAR_AUTO_MANUAL_bm;
        Serial.print("  MANUAL GEAR  ");
    }
    else {
        // auto
        buttons |= GEAR_AUTO_MANUAL_bm;
        Serial.print("  AUTO GEAR  ");
    }
}

void t1_OVF_ISR() {
    // if (deadmanSwitchIsPressed) {
    sendCANoverUART(txmsg);
    Serial.print("  SENT CAN  ");
}
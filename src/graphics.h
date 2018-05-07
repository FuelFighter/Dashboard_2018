#include <Adafruit_SharpMem.h>
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include "helpers.h"

/* SCREEN INCLUDES */
#include "final_v2_left.h"
#include "final_v2_right.h"
#include "fonts/FreeMono9pt7b.h"
#include "fonts/FreeMono12pt7b.h"
#include "fonts/FreeMono18pt7b.h"
#include "fonts/FreeMono24pt7b.h"

/* SCREEN PARAMETERS */
#define WIDTH  320
#define HEIGHT 240
#define BLACK 0
#define WHITE 1


/* FUNCTION PROTOTYPES */
// Clears specified screen
void clearScreen(Adafruit_SharpMem& screen);

// Draws the background bitmap defined in a .h file.
// The .h file must be exported from GIMP as .xbm and then renamed to .h. 
// `right` is to select which background to use.
void drawBackground(Adafruit_SharpMem& screen, bool right=true);

// Draws a string at (x, y).
// The size of the text will be `size` times the font size specified
// by the font .h file. 
// For some reason, `x` and `y` are the bottom left corner of the text string,
// so remember to take that into account when inputting the coords.
void drawString(Adafruit_SharpMem& screen, const char * str, int x, int y, int size=1);

// DRAW FUNCTIONS FOR RIGHT SCREEN
// calculates the x- and y-value and height for the slider bar
// saves values in `x`, `y` and `h`. 
void calcXYHforBar(const double& percent, uint16_t& x, uint8_t& y, uint8_t& h);
void drawSpeed(Adafruit_SharpMem& screen, const float& speedVal);
void drawLapTime(Adafruit_SharpMem& screen, const int& lapTimeSeconds);
void drawCurrentValue(Adafruit_SharpMem& screen, const double& currentVal);


// DRAW FUNCTINOS FOR LEFT SCREEN
// timeLeft in seconds
void drawTimeLeft(Adafruit_SharpMem& screen, const int& timeLeft);
void drawLapCount(Adafruit_SharpMem& screen, volatile const uint8_t& count, const uint8_t& max=10);
void drawBestAndAvgLapTime(Adafruit_SharpMem& screen, volatile const int* lapTimes, const uint8_t& size);
void drawVoltageValue(Adafruit_SharpMem& screen, const double& voltageVal);
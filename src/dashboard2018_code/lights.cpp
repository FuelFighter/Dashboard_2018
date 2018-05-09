#include "lights.h"

void turnOffStrip(Adafruit_NeoPixel& strip, const uint16_t& start, uint16_t end) {
    if (end == 0) {
        end = strip.numPixels();
    }

    for (uint16_t i = start; i < end; ++i) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, 0));
    }

    strip.show();
}


void blinkLights(Adafruit_NeoPixel& strip, const uint16_t& start, uint16_t end) {
    if (end == 0) {
        end = strip.numPixels();
    }

    for (uint16_t i = start; i < end; ++i) {
        strip.setPixelColor(i, COLOR_BLINKLIGHTS);
    }
    strip.show();
}

void drivingLightsFront(Adafruit_NeoPixel& frontLights) {
    for (uint16_t i = 0; i < frontLights.numPixels(); ++i) {
        frontLights.setPixelColor(i, COLOR_FRONTLIGHTS);
    }

    frontLights.show();
}

void drivingLightsBack(Adafruit_NeoPixel& backLights) {
    for (uint16_t i = 0; i < backLights.numPixels(); ++i) {
        backLights.setPixelColor(i, COLOR_BACKLIGHTS);
    }

    backLights.show();
}

void deadmanSwitchNotPressed(Adafruit_NeoPixel& shweelLights) {
    for (uint16_t i = 0; i < shweelLights.numPixels(); ++i) {
        shweelLights.setPixelColor(i, COLOR_ERROR);
    }

    shweelLights.show();
}
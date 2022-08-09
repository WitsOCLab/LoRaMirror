#pragma once

#include <Arduino.h>

#define stickX A0
#define stickY A1

class Joystick
{
public:
    enum class StickState
    {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        CENTERED,
        CLICKED,
        HELD,
        CLEARED
    };
    void updateStick();
    StickState getStickState(bool clear = true);
    int8_t getStickX();
    int8_t getStickY();
    void setDeadzone(int deadzone);
    int getDeadzone();
    void centerStick();
    void setStickClicked();
    void setStickHeld();
    void clearStick();

private:
    int xRaw = 0;
    int yRaw = 0;
    int xCenter = 0;
    int yCenter = 0;
    int8_t xScaled = 0;
    int8_t yScaled = 0;
    int deadzone = 20;
    StickState stickState = StickState::CENTERED;
};
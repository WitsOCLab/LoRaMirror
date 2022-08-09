#include "Joystick.h"

void Joystick::updateStick()
{
    int x = analogRead(stickX);
    x = map(x, 0, 4095, 0, 200);
    int y = analogRead(stickY);
    y = map(y, 0, 4095, 0, 200);
    x -= xCenter;
    y -= yCenter;

    // Add deadzone
    if (abs(x) < deadzone)
    {
        x = 0;
    }
    if (abs(y) < deadzone)
    {
        y = 0;
    }

    if ((stickState != StickState::CLICKED) && (stickState != StickState::HELD))
    {
        if (stickState != StickState::CLEARED)
        {
            if (y > 0 && abs(y) > abs(x))
            {
                stickState = StickState::UP;
            }
            else if (y < 0 && abs(y) > abs(x))
            {
                stickState = StickState::DOWN;
            }
            else if (x > 0 && abs(x) > abs(y))
            {
                stickState = StickState::RIGHT;
            }
            else if (x < 0 && abs(x) > abs(y))
            {
                stickState = StickState::LEFT;
            }
            else
            {
                stickState = StickState::CENTERED;
            }
        }
        else
        {
            if (y == 0 && x == 0)
            {
                stickState = StickState::CENTERED;
            }
        }
    }

    // Update the scaled values
    if (x > 100)
    {
        x = 100;
    }
    else if (x < -100)
    {
        x = -100;
    }
    xScaled = x;

    if (y > 100)
    {
        y = 100;
    }
    else if (y < -100)
    {
        y = -100;
    }
    yScaled = y;
}

void Joystick::centerStick()
{
    xCenter = analogRead(stickX);
    xCenter = map(xCenter, 0, 4095, 0, 200);
    yCenter = analogRead(stickY);
    yCenter = map(yCenter, 0, 4095, 0, 200);
}

int Joystick::getDeadzone()
{
    return deadzone;
}

void Joystick::setDeadzone(int deadzone)
{
    this->deadzone = deadzone;
}

Joystick::StickState Joystick::getStickState(bool clear)
{
    StickState tempstate = stickState;
    if (clear)
    {
        stickState = StickState::CLEARED;
    }
    return tempstate;
}

int8_t Joystick::getStickX()
{
    return xScaled;
}

int8_t Joystick::getStickY()
{
    return yScaled;
}

void Joystick::setStickClicked()
{
    stickState = StickState::CLICKED;
}

void Joystick::setStickHeld()
{
    stickState = StickState::HELD;
}

void Joystick::clearStick() {
    stickState = StickState::CLEARED;
}
#pragma once
#include <Arduino.h>

enum direction
{
    DIRECTION_RIGHT,
    DIRECTION_FRONT,
    DIRECTION_LEFT,
    DIRECTION_TURN_AROUND
};
struct walls
{
    bool right;
    bool left;
    bool front;
};

bool checkWall(unsigned int distance);
direction decide(walls currentWalls);
#include "Maze.h"

static const unsigned int WALL_THRESHOLD = 20;

bool checkWall(unsigned int distance)
{
    return distance <= WALL_THRESHOLD;
}

direction decide(walls currentWalls)
{
    if (!currentWalls.right)
    {
        return DIRECTION_RIGHT;
    }
    else if (!currentWalls.front)
    {
        return DIRECTION_FRONT;
    }
    else if (!currentWalls.left)
    {
        return DIRECTION_LEFT;
    }
    return DIRECTION_TURN_AROUND;
}
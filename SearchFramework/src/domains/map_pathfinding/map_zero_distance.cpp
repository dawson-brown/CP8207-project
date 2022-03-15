#include "map_zero_distance.h"

#include <stdlib.h>

MapZeroDistance::MapZeroDistance()
{
}

MapZeroDistance::~MapZeroDistance()
{
}

void MapZeroDistance::setGoal(const MapLocation &state)
{
    setGoal(state.x, state.y);
}

void MapZeroDistance::setGoal(uint16_t x_loc, uint16_t y_loc)
{
    goal.x = x_loc;
    goal.y = y_loc;
}

double MapZeroDistance::computeHValue(const MapLocation& state) const
{
    return 0;
}

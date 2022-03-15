#include "map_euclidean_distance.h"

#include <stdlib.h>
#include <math.h>

MapEuclideanDistance::MapEuclideanDistance()
{
}

MapEuclideanDistance::~MapEuclideanDistance()
{
}

void MapEuclideanDistance::setGoal(const MapLocation &state)
{
    setGoal(state.x, state.y);
}

void MapEuclideanDistance::setGoal(uint16_t x_loc, uint16_t y_loc)
{
    goal.x = x_loc;
    goal.y = y_loc;
}

double MapEuclideanDistance::computeHValue(const MapLocation& state) const
{
    double tmp = pow(goal.x - state.x, 2) + pow(goal.y - state.y, 2);
    return sqrt(tmp);
}

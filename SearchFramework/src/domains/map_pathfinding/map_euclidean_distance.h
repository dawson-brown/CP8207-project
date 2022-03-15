

#ifndef MAP_EUCLIDEAN_DISTANCE_H_
#define MAP_EUCLIDEAN_DISTANCE_H_

#include "map_location.h"
#include "../../generic_defs/heuristic.h"

/**
 * A class that defines the euclidean heuristic function for a map pathfinding domain.
 *
 * @class MapEuclideanDistance
 */
class MapEuclideanDistance: public Heuristic<MapLocation>
{
public:
    /**
     * Constructor for a MapZeroDistance heuristic function. By default, sets the goal at (0, 0)
     */
    MapEuclideanDistance();

    /**
     * Destructor for a ZeroDistance heuristic function. Does nothing.
     */
    virtual ~MapEuclideanDistance();

    /**
     * Sets the goal location to the given map location.
     * @param state The new goal.
     */
    void setGoal(const MapLocation &state);

    /**
     * Sets the goal to use the given coordinates.
     *
     * @param x_loc The x location of the goal.
     * @param y_loc The y location of the goal.
     */
    void setGoal(uint16_t x_loc, uint16_t y_loc);

protected:
    // Overloaded methods
    virtual double computeHValue(const MapLocation &state) const;

    MapLocation goal; ///< The goal location currently being used.
};

#endif /* MAP_EUCLIDEAN_DISTANCE_H_ */

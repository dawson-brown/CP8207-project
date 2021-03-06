/*
 * tile_correct_placement.h
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-17
 *      Author: Dawson Brown
 */

#ifndef TILECORRECTPLACEMENT_H_
#define TILECORRECTPLACEMENT_H_

#include "tile_puzzle_transitions.h"
#include "../../generic_defs/heuristic.h"
#include <vector>

/**
 * Manhattan distance calculator for tile puzzle states.
 *
 * @class TileManhattanDistance
 */
class TileCorrectPlacement: public Heuristic<TilePuzzleState>
{
public:
    /**
     * Constructor that stores the given goal and tile move costs.
     *
     * @param g The goal state.
     * @param ops The transition function used to get the tile move costs.
     */
    TileCorrectPlacement(const TilePuzzleState &g, const TilePuzzleTransitions &ops);

    /**
     * Destructor for the tile Manhattan distance heuristic. Destructor is empty.
     */
    virtual ~TileCorrectPlacement();

    /**
     * Sets the goal to the given state and the tile move costs using the given transition function.
     *
     * @param g The new goal state.
     * @param ops The transition function used to get the tile move costs.
     */
    void setGoal(const TilePuzzleState &g, const TilePuzzleTransitions &ops);

protected:
    // Overloaded methods
    virtual double computeHValue(const TilePuzzleState &state) const;

    TilePuzzleState goal; ///< The goal state.
    unsigned num_rows; ///< The number of rows in the puzzle.
    unsigned num_cols; ///< The number of columns in the puzzle.
    unsigned puzzle_size; ///< The total number of locations in the puzzle.

    std::vector<std::vector<double> > tile_h_value; ///< The heuristic impact of the current tile in the current position. The first index (for the blank) is unused.
    std::vector<double> tile_move_cost; ///< The cost of moving each tile.
};

#endif /* TILECORRECTPLACEMENT_H_ */
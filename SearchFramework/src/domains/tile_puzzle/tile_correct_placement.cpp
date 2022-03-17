/*
 * tile_correct_placement.cpp
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-17
 *      Author: Dawson Brown
 */

#include "tile_correct_placement.h"
#include <cmath>

using std::vector;
using std::abs;
using std::cout;
using std::endl;

TileCorrectPlacement::TileCorrectPlacement(const TilePuzzleState &g, const TilePuzzleTransitions &ops)
        : num_rows(0), num_cols(0), puzzle_size(0)
{
    setGoal(g, ops);
}

TileCorrectPlacement::~TileCorrectPlacement()
{
}

void TileCorrectPlacement::setGoal(const TilePuzzleState& g, const TilePuzzleTransitions &ops)
{
    goal = g;
    num_rows = g.num_rows;
    num_cols = g.num_cols;
    puzzle_size = num_rows*num_cols;

    tile_move_cost.resize(puzzle_size);

    for(unsigned i = 1; i < puzzle_size; i++) {
        tile_move_cost[i] = ops.getTileMoveCost(i);
    }

    tile_h_value.resize(puzzle_size, vector<double>(puzzle_size, 0.0));

    for(unsigned goal_pos = 0; goal_pos < puzzle_size; goal_pos++) {
        unsigned tile = goal.permutation[goal_pos];

        if(tile == 0)
            continue;

        for(unsigned pos = 0; pos < puzzle_size; pos++) {
            tile_h_value[tile][pos] = abs((int) (goal_pos % num_cols) - (int) (pos % num_cols)); // column difference
            tile_h_value[tile][pos] += abs((int) (goal_pos / num_cols) - (int) (pos / num_cols)); // row difference
            tile_h_value[tile][pos] *= tile_move_cost[tile]; // increase by the tile move cost
        }
    }
}

double TileCorrectPlacement::computeHValue(const TilePuzzleState& state) const
{
    double h_value = 0.0;

    for(unsigned pos = 0; pos < state.permutation.size(); pos++) {
        // printf("%d ", state.permutation[pos]);
        if (pos != state.permutation[pos])
            h_value += 1.0;
    }
    // printf("\nh: %f\n\n", h_value);
    return h_value;
}

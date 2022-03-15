/**
 * The file you can use to do your experiments in the sliding tile puzzle.

 * @file tile_experiments.cpp
 */

#include <stdio.h>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <inttypes.h>
#include <chrono>

#include "../domains/tile_puzzle/tile_puzzle_transitions.h"
#include "../domains/tile_puzzle/tile_manhattan_distance.h"
#include "../generic_defs/single_goal_test.h"
// #include "../algorithms/best_first_search/awa_star.h"
#include "../algorithms/best_first_search/weighted_a_star.h"
#include "../algorithms/best_first_search/e_weighted_a_star.h"
#include "../generic_defs/permutation_hash_function.h"
#include "../utils/combinatorics.h"

#include <cstdint>

using namespace std;

int main(int argc, char **argv)
{
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;

    TilePuzzleTransitions tile_ops(3, 4, TileCostType::unit);
    TilePuzzleState goal_state(3, 4);
    SingleGoalTest<TilePuzzleState> goal_test(goal_state);
    PermutationHashFunction<TilePuzzleState> tile_hash;
    TileManhattanDistance manhattan(goal_state, tile_ops);

    // weighted A*
    WAStar<TilePuzzleState, BlankSlide> wa_star(10);
    wa_star.setTransitionSystem(&tile_ops);
    wa_star.setGoalTest(&goal_test);
    wa_star.setHashFunction(&tile_hash);
    wa_star.setHeuristic(&manhattan);

    // Epsilon weighted A*
    EpsilonWAStar<TilePuzzleState, BlankSlide> e_wa_star(10, 0.1);
    e_wa_star.setTransitionSystem(&tile_ops);
    e_wa_star.setGoalTest(&goal_test);
    e_wa_star.setHashFunction(&tile_hash);
    e_wa_star.setHeuristic(&manhattan);


    vector<vector<unsigned> > starts;

    vector<BlankSlide> wa_solution;
    vector<uint64_t> wa_expansions;
    vector<uint64_t> wa_lengths;

    vector<BlankSlide> e_wa_solution;
    vector<uint64_t> e_wa_expansions;
    vector<uint64_t> e_wa_lengths;

    read_in_permutations("../src/domains/tile_puzzle/tile_files/3x4_puzzle.probs", starts);

    for(unsigned i = 0; i < starts.size(); i++) {

        TilePuzzleState start_state(starts[i], 3, 4);
        // a_star.getPlan(start_state, a_solution);
        // a_lengths.push_back(a_star.getLastPlan().size());
        // a_expansions.push_back(a_star.getGoalTestCount());

        wa_star.getPlan(start_state, wa_solution);
        wa_lengths.push_back(wa_star.getLastPlan().size());
        wa_expansions.push_back(wa_star.getGoalTestCount());

        e_wa_star.getPlan(start_state, e_wa_solution);
        e_wa_lengths.push_back(e_wa_star.getLastPlan().size());
        e_wa_expansions.push_back(e_wa_star.getGoalTestCount());

    }

    // for (unsigned i = 0; i<wa_lengths.size(); i++){
    //     printf("Lenghts.... A*: %ld -- Weighted: %ld  --  Epsilon: %ld\n", a_lengths[i], wa_lengths[i], e_wa_lengths[i]);
    //     printf("Expansions.... A*: %ld -- Weighted: %ld  --  Epsilon: %ld\n\n", a_expansions[i], wa_expansions[i], e_wa_expansions[i]);
    // }

    for (unsigned i = 0; i<wa_lengths.size(); i++){
        printf("Lenghts.... Epsilon: %ld\n", wa_lengths[i]);
        printf("Expansions.... Epsilon: %ld\n\n", wa_lengths[i]);
    }

    return 0;
}
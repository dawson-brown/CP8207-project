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
#include "../domains/tile_puzzle/tile_correct_placement.h"
#include "../generic_defs/single_goal_test.h"
#include "../algorithms/best_first_search/awa_star.h"
#include "../algorithms/best_first_search/epsilon_awa_star.h"
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
    TileCorrectPlacement correct_placement(goal_state, tile_ops);

    // Anytime weighted A*
    // AWAStar<TilePuzzleState, BlankSlide> awa_star(25);
    // awa_star.setTransitionSystem(&tile_ops);
    // awa_star.setGoalTest(&goal_test);
    // awa_star.setHashFunction(&tile_hash);
    // awa_star.setHeuristic(&correct_placement);

    // Epsilon-AWA*
    EpsilonAWAStar<TilePuzzleState, BlankSlide> e_awa_star(0.1, 10);
    e_awa_star.setTransitionSystem(&tile_ops);
    e_awa_star.setGoalTest(&goal_test);
    e_awa_star.setHashFunction(&tile_hash);
    e_awa_star.setHeuristic(&manhattan);

    // Epsilon weighted A*
    // EpsilonWAStar<TilePuzzleState, BlankSlide> e_wa_star(10, 0.1);
    // e_wa_star.setTransitionSystem(&tile_ops);
    // e_wa_star.setGoalTest(&goal_test);
    // e_wa_star.setHashFunction(&tile_hash);
    // e_wa_star.setHeuristic(&manhattan);


    vector<vector<unsigned> > starts;

    vector<BlankSlide> awa_solution;
    vector<uint64_t> awa_expansions;
    vector<vector<double>> awa_costs;

    vector<BlankSlide> e_awa_solution;
    vector<uint64_t> e_awa_expansions;
    vector<vector<double>> e_awa_costs;

    read_in_permutations("../src/domains/tile_puzzle/tile_files/3x4_puzzle.probs", starts);

    for(unsigned i = 0; i < starts.size(); i++) {

        printf("Puzzle: %d\n", i);

        TilePuzzleState start_state(starts[i], 3, 4);
        // awa_star.getPlan(start_state, awa_solution);
        // awa_costs.push_back(awa_star.getSolutionCosts());

        SearchTermType res = e_awa_star.getPlan(start_state, e_awa_solution);

        if (res == SearchTermType::res_limit) 
            printf("Ran out of resources\n");
        e_awa_costs.push_back(e_awa_star.getSolutionCosts());
        // awa_expansions.push_back(awa_star.getGoalTestCount());

        // wa_star.getPlan(start_state, wa_solution);
        // wa_lengths.push_back(wa_star.getLastPlan().size());
        // wa_expansions.push_back(wa_star.getGoalTestCount());

        // e_wa_star.getPlan(start_state, e_wa_solution);
        // e_wa_lengths.push_back(e_wa_star.getLastPlan().size());
        // e_wa_expansions.push_back(e_wa_star.getGoalTestCount());

    }

    // for (unsigned i = 0; i<wa_lengths.size(); i++){
    //     printf("Lenghts.... A*: %ld -- Weighted: %ld  --  Epsilon: %ld\n", a_lengths[i], wa_lengths[i], e_wa_lengths[i]);
    //     printf("Expansions.... A*: %ld -- Weighted: %ld  --  Epsilon: %ld\n\n", a_expansions[i], wa_expansions[i], e_wa_expansions[i]);
    // }

    for (unsigned i = 0; i<1; i++){
        // printf("%ld solutions found\n", awa_costs[i].size());
        // for (unsigned j = 0; j<awa_costs[i].size(); j++){
        //     printf("Cost.... AWA: %f\n", awa_costs[i][j]);
        // }

        printf("\n%ld solutions found\n", e_awa_costs[i].size());
        for (unsigned j = 0; j<e_awa_costs[i].size(); j++){
            printf("Cost -- E-AWA*: %f\n", e_awa_costs[i][j]);
        }
        printf("\n===================\n\n");
    }

    return 0;
}

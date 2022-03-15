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
#include "../algorithms/best_first_search/a_star.h"
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

    AStar<TilePuzzleState, BlankSlide> a_star;

    TilePuzzleTransitions tile_ops(3, 4, TileCostType::inverse);
    a_star.setTransitionSystem(&tile_ops);

    TilePuzzleState goal_state(3, 4);

    SingleGoalTest<TilePuzzleState> goal_test(goal_state);
    a_star.setGoalTest(&goal_test);

    PermutationHashFunction<TilePuzzleState> tile_hash;
    a_star.setHashFunction(&tile_hash);

    TileManhattanDistance manhattan(goal_state, tile_ops);
    a_star.setHeuristic(&manhattan);

    vector<BlankSlide> solution;

    vector<vector<unsigned> > starts;
    
    vector<uint64_t> expansions;
    vector<uint64_t> lengths;

    read_in_permutations("../src/domains/tile_puzzle/tile_files/3x4_puzzle.probs", starts);


    for(unsigned i = 0; i < starts.size(); i++) {
        TilePuzzleState start_state(starts[i], 3, 4);

        // auto t1 = high_resolution_clock::now();
        a_star.getPlan(start_state, solution);
        // auto t2 = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = t2 - t1;

        // prints stats (using goal test count as measure of number of expansions)
        // cout << a_star.getLastPlanCost()
        //         << endl;

        lengths.push_back(a_star.getLastPlan().size());
        expansions.push_back(a_star.getGoalTestCount());

    }

    double avg = 0;
    printf("Expansions: \n");
    for (unsigned int i=0; i<expansions.size(); i++){
        printf("%" PRIu64 "\n", expansions[i]);
        avg += expansions[i];
    }
    printf("average cost: %f\n", avg/expansions.size());

    sort(expansions.begin(), expansions.end(), greater<uint64_t>());
    printf("median cost: %" PRIu64 "\n", expansions[50]);

    printf("\nLengths:\n");
    for (unsigned int i=0; i<lengths.size(); i++){
        printf("%" PRIu64 "\n", lengths[i]);
    }
    return 0;
}

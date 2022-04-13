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
#include "../algorithms/best_first_search/beta_awa_star.h"
#include "../algorithms/best_first_search/weighted_a_star.h"
#include "../algorithms/best_first_search/e_weighted_a_star.h"
#include "../generic_defs/permutation_hash_function.h"
#include "../utils/combinatorics.h"

#include <cstdint>

using namespace std;

int main(int argc, char **argv)
{

    TilePuzzleTransitions tile_ops(3, 4, TileCostType::unit);
    TilePuzzleState goal_state(3, 4);
    SingleGoalTest<TilePuzzleState> goal_test(goal_state);
    PermutationHashFunction<TilePuzzleState> tile_hash;
    TileManhattanDistance manhattan(goal_state, tile_ops);
    TileCorrectPlacement correct_placement(goal_state, tile_ops);

    // // Anytime weighted A*
    // AWAStar<TilePuzzleState, BlankSlide> awa_star_2(2);
    // awa_star_2.setTransitionSystem(&tile_ops);
    // awa_star_2.setGoalTest(&goal_test);
    // awa_star_2.setHashFunction(&tile_hash);
    // awa_star_2.setHeuristic(&manhattan);
    // vector<BlankSlide> awa_2_solution;
    // vector<uint64_t> awa_2_expansions;
    // vector<vector<double>> awa_2_costs;
    // vector<unsigned> awa_2_storage;
    // vector<double> awa_2_cost;

    // AWAStar<TilePuzzleState, BlankSlide> awa_star_5(5);
    // awa_star_5.setTransitionSystem(&tile_ops);
    // awa_star_5.setGoalTest(&goal_test);
    // awa_star_5.setHashFunction(&tile_hash);
    // awa_star_5.setHeuristic(&manhattan);
    // vector<BlankSlide> awa_5_solution;
    // vector<uint64_t> awa_5_expansions;
    // vector<vector<double>> awa_5_costs;
    // vector<unsigned> awa_5_storage;
    // vector<double> awa_5_cost;

    // Epsilon-AWA*
    EpsilonAWAStar<TilePuzzleState, BlankSlide> e_awa_star(0.1, 5);
    e_awa_star.setTransitionSystem(&tile_ops);
    e_awa_star.setGoalTest(&goal_test);
    e_awa_star.setHashFunction(&tile_hash);
    e_awa_star.setHeuristic(&manhattan);
    vector<BlankSlide> e_awa_solution;
    vector<uint64_t> e_awa_expansions;
    vector<vector<double>> e_awa_costs;
    vector<unsigned> e_awa_storage;
    vector<double> e_awa_cost;

    //BETA-AWA*
    BetaAWAStar<TilePuzzleState, BlankSlide> beta_awa_star(0.1, 1.3, 5.0, 0.6);
    beta_awa_star.setTransitionSystem(&tile_ops);
    beta_awa_star.setGoalTest(&goal_test);
    beta_awa_star.setHashFunction(&tile_hash);
    beta_awa_star.setHeuristic(&manhattan);
    vector<BlankSlide> beta_awa_solution;
    vector<uint64_t> beta_awa_expansions;
    vector<vector<double>> beta_awa_costs;
    vector<unsigned> beta_awa_storage;
    vector<double> beta_awa_cost;


    vector<vector<unsigned> > starts;
    read_in_permutations("../src/domains/tile_puzzle/tile_files/3x4_puzzle.probs", starts);
    unsigned experiments = starts.size();

    for(unsigned i = 0; i < experiments; i++) {

        TilePuzzleState start_state(starts[i], 3, 4);

        // // AWA*
        // awa_star_2.getPlan(start_state, awa_2_solution);
        // awa_2_costs.push_back(awa_star_2.getSolutionCosts());
        // awa_2_expansions.push_back(awa_star_2.getGoalTestCount());
        // awa_2_storage.push_back(awa_star_2.getStateGenCount());

        // awa_star_5.getPlan(start_state, awa_5_solution);
        // awa_5_costs.push_back(awa_star_5.getSolutionCosts());
        // awa_5_expansions.push_back(awa_star_5.getGoalTestCount());
        // awa_5_storage.push_back(awa_star_5.getStateGenCount());

        // e-AWA*
        // e_awa_star.getPlan(start_state, e_awa_solution);
        // e_awa_costs.push_back(e_awa_star.getSolutionCosts());
        // e_awa_expansions.push_back(e_awa_star.getGoalTestCount());
        // e_awa_storage.push_back(e_awa_star.getStateGenCount());

        // beta-AWA*
        beta_awa_star.getPlan(start_state, beta_awa_solution);
        beta_awa_costs.push_back(beta_awa_star.getSolutionCosts());
        beta_awa_expansions.push_back(beta_awa_star.getGoalTestCount());
        beta_awa_storage.push_back(beta_awa_star.getStateGenCount());

        // WA*
        // wa_star.getPlan(start_state, wa_solution);
        // wa_lengths.push_back(wa_star.getLastPlan().size());
        // wa_expansions.push_back(wa_star.getGoalTestCount());

        // e-WA*
        // e_wa_star.getPlan(start_state, e_wa_solution);
        // e_wa_lengths.push_back(e_wa_star.getLastPlan().size());
        // e_wa_expansions.push_back(e_wa_star.getGoalTestCount());

    }

    double num_experiments = (double) experiments; 

    // // AWA*
    // printf("2-AWA*\n");
    size_t total_found = 0;
    uint64_t total_exp = 0;
    unsigned total_stored = 0;
    // for (unsigned i = 0; i<experiments; i++){

    //     total_found += awa_2_costs[i].size();
    //     total_exp += awa_2_expansions[i];
    //     total_stored += awa_2_storage[i];

    // }
    // printf("Average found: %lf\n", (double) total_found / num_experiments );
    // printf("Average stored: %lf\n", (double) total_stored / num_experiments);
    // printf("Average expanded: %lf\n", (double) total_exp / num_experiments);
    // printf("\n===================\n\n");

    // printf("5-AWA*\n");
    // total_found = 0;
    // total_exp = 0;
    // total_stored = 0;
    // for (unsigned i = 0; i<experiments; i++){

    //     total_found += awa_5_costs[i].size();
    //     total_exp += awa_5_expansions[i];
    //     total_stored += awa_5_storage[i];

    // }
    // printf("Average found: %lf\n", (double) total_found / num_experiments );
    // printf("Average stored: %lf\n", (double) total_stored / num_experiments);
    // printf("Average expanded: %lf\n", (double) total_exp / num_experiments);
    // printf("\n===================\n\n");

    // e-AWA*
    // printf("e-AWA*\n");
    // total_found = 0;
    // total_exp = 0;
    // total_stored = 0;
    // for (unsigned i = 0; i<experiments; i++){

    //     total_found += e_awa_costs[i].size();
    //     total_exp += e_awa_expansions[i];
    //     total_stored += e_awa_storage[i];

    // }
    // printf("Average found: %lf\n", (double) total_found / num_experiments );
    // printf("Average stored: %lf\n", (double) total_stored / num_experiments);
    // printf("Average expanded: %lf\n", (double) total_exp / num_experiments);
    // printf("\n===================\n\n");


    // beta-AWA*
    printf("beta-AWA*\n");
    total_found = 0;
    total_exp = 0;
    total_stored = 0;
    for (unsigned i = 0; i<experiments; i++){

        total_found += beta_awa_costs[i].size();
        total_exp += beta_awa_expansions[i];
        total_stored += beta_awa_storage[i];

    }
    printf("Average found: %lf\n", (double) total_found / num_experiments );
    printf("Average stored: %lf\n", (double) total_stored / num_experiments);
    printf("Average expanded: %lf\n", (double) total_exp / num_experiments);
    printf("\n===================\n\n");

    return 0;
}

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
    AWAStar<TilePuzzleState, BlankSlide> awa_star(10);
    awa_star.setTransitionSystem(&tile_ops);
    awa_star.setGoalTest(&goal_test);
    awa_star.setHashFunction(&tile_hash);
    awa_star.setHeuristic(&manhattan);
    vector<BlankSlide> awa_solution;
    vector<uint64_t> awa_expansions;
    vector<vector<double>> awa_costs;
    vector<unsigned> awa_storage;
    vector<double> awa_cost;
    // vector<vector<double>> awa_time_between_sols;

    // // Epsilon-AWA*
    // EpsilonAWAStar<TilePuzzleState, BlankSlide> e_awa_star(0.3, 10.0);
    // e_awa_star.setTransitionSystem(&tile_ops);
    // e_awa_star.setGoalTest(&goal_test);
    // e_awa_star.setHashFunction(&tile_hash);
    // e_awa_star.setHeuristic(&manhattan);
    // vector<BlankSlide> e_awa_solution;
    // vector<uint64_t> e_awa_expansions;
    // vector<vector<double>> e_awa_costs;
    // vector<unsigned> e_awa_storage;
    // vector<double> e_awa_cost;

    // //BETA-AWA*
    // BetaAWAStar<TilePuzzleState, BlankSlide> beta_awa_star(0.3, 10.0, 5.0, 0.6);
    // beta_awa_star.setTransitionSystem(&tile_ops);
    // beta_awa_star.setGoalTest(&goal_test);
    // beta_awa_star.setHashFunction(&tile_hash);
    // beta_awa_star.setHeuristic(&manhattan);
    // vector<BlankSlide> beta_awa_solution;
    // vector<uint64_t> beta_awa_expansions;
    // vector<vector<double>> beta_awa_costs;
    // vector<unsigned> beta_awa_storage;
    // vector<double> beta_awa_cost;

    // std::clock_t start;
    // double total_time = 0.0;

    vector<double> optimal_costs = {
        37.00,
        41.00,
        31.00,
        26.00,
        31.00,
        37.00,
        38.00,
        37.00,
        36.00,
        38.00,
        28.00,
        37.00,
        32.00,
        48.00,
        36.00,
        26.00,
        37.00,
        34.00,
        39.00,
        39.00,
        32.00,
        29.00,
        38.00,
        26.00,
        36.00,
        37.00,
        36.00,
        39.00,
        40.00,
        30.00,
        36.00,
        37.00,
        25.00,
        38.00,
        39.00,
        30.00,
        37.00,
        22.00,
        35.00,
        33.00,
        34.00,
        40.00,
        34.00,
        37.00,
        34.00,
        39.00,
        24.00,
        31.00,
        42.00,
        40.00,
        40.00,
        32.00,
        34.00,
        33.00,
        31.00,
        25.00,
        36.00,
        39.00,
        32.00,
        37.00,
        34.00,
        38.00,
        31.00,
        37.00,
        29.00,
        34.00,
        29.00,
        38.00,
        40.00,
        33.00,
        42.00,
        31.00,
        35.00,
        38.00,
        33.00,
        36.00,
        32.00,
        28.00,
        42.00,
        26.00,
        27.00,
        42.00,
        35.00,
        34.00,
        31.00,
        30.00,
        37.00,
        36.00,
        36.00,
        43.00,
        38.00,
        40.00,
        29.00,
        36.00,
        30.00,
        36.00,
        34.00,
        31.00,
        38.00,
        36.00
    };

    vector<vector<unsigned> > starts;
    read_in_permutations("../src/domains/tile_puzzle/tile_files/3x4_puzzle.probs", starts);
    unsigned experiments = starts.size();
    // unsigned experiments = 4;

    for(unsigned i = 0; i < experiments; i++) {

        printf("Puzzle: %d\n", i);
        TilePuzzleState start_state(starts[i], 3, 4);

        // AWA*
        // start = clock();
        awa_star.getPlan(start_state, awa_solution);
        // total_time += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        // awa_costs.push_back(awa_star.getSolutionCosts());
        awa_costs.push_back(awa_star.getIntervalCosts());
        // awa_expansions.push_back(awa_star.getGoalTestCount());
        // awa_storage.push_back(awa_star.getStateGenCount());
        // awa_time_between_sols.push_back(awa_star.getTimeBetweenSolutions());

        // // e-AWA*
        // start = clock();
        // e_awa_star.getPlan(start_state, e_awa_solution);
        // total_time += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        // e_awa_costs.push_back(e_awa_star.getIntervalCosts());
        // e_awa_expansions.push_back(e_awa_star.getGoalTestCount());
        // e_awa_storage.push_back(e_awa_star.getStateGenCount());

        // // beta-AWA*
        // start = clock();
        // beta_awa_star.getPlan(start_state, beta_awa_solution);
        // total_time += ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        // beta_awa_costs.push_back(beta_awa_star.getIntervalCosts());
        // beta_awa_expansions.push_back(beta_awa_star.getGoalTestCount());
        // beta_awa_storage.push_back(beta_awa_star.getStateGenCount());

        // WA*
        // wa_star.getPlan(start_state, wa_solution);
        // wa_lengths.push_back(wa_star.getLastPlan().size());
        // wa_expansions.push_back(wa_star.getGoalTestCount());

        // e-WA*
        // e_wa_star.getPlan(start_state, e_wa_solution);
        // e_wa_lengths.push_back(e_wa_star.getLastPlan().size());
        // e_wa_expansions.push_back(e_wa_star.getGoalTestCount());

    }

    // double num_experiments = (double) experiments; 

    // AWA*
    // printf("e-AWA*\n");
    // printf("Average time to optaim solution: %lf\n", total_time / num_experiments);
    // double total_found = 0;
    // uint64_t total_exp = 0;
    // unsigned total_stored = 0;

    // for (unsigned i = 0; i<experiments; i++){
    //     for (unsigned j=0; j<e_awa_costs[i].size(); j++)
    //         printf("%lf\n", e_awa_costs[i][j]);
    //     printf("\n");
    // }

    unsigned longest = 0;
    for (unsigned i = 0; i<experiments; i++){

        // total_found += awa_costs[i].size();
        // total_exp += awa_expansions[i];
        // total_stored += awa_storage[i];
        if (awa_costs[i].size() > longest) {
            longest = awa_costs[i].size();
        }

        for (unsigned j = 0; j < awa_costs[i].size(); j++) {
            // printf("%lf,", awa_costs[i][j]);
            awa_costs[i][j] = optimal_costs[i] / awa_costs[i][j];
        }
        // printf("\n\n");

    }


    vector<double> avgs(longest, 0);
    for (unsigned i = 0; i<experiments; i++){
        for (unsigned j = 0; j < awa_costs[i].size(); j++)
            avgs[j] += awa_costs[i][j];
        for (unsigned j = awa_costs[i].size(); j < longest; j++)
            avgs[j] += 1.0;
    }

    for (unsigned j = 0; j < longest; j++)
        avgs[j] = avgs[j] / experiments;

    for (unsigned j = 0; j < longest; j++)
        printf("%lf\n", avgs[j]);


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
    // printf("beta-AWA*\n");
    // total_found = 0;
    // total_exp = 0;
    // total_stored = 0;
    // for (unsigned i = 0; i<experiments; i++){

    //     total_found += beta_awa_costs[i].size();
    //     total_exp += beta_awa_expansions[i];
    //     total_stored += beta_awa_storage[i];

    // }
    // printf("Average found: %lf\n", (double) total_found / num_experiments );
    // printf("Average stored: %lf\n", (double) total_stored / num_experiments);
    // printf("Average expanded: %lf\n", (double) total_exp / num_experiments);
    // printf("\n===================\n\n");

    return 0;
}

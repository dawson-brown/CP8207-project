/**
 * The file you can use to do your experiments for pathfinding in the given Starcraft map.

 * @file starcraft_map_experiments.cpp
 */

#include <stdio.h>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <inttypes.h>

#include "../domains/map_pathfinding/map_pathfinding_transitions.h"
#include "../domains/map_pathfinding/map_manhattan_distance.h"
#include "../domains/map_pathfinding/map_zero_distance.h"
#include "../domains/map_pathfinding/map_euclidean_distance.h"
#include "../domains/map_pathfinding/map_loc_hash_function.h"
#include "../domains/map_pathfinding/map_utils.h"
#include "../generic_defs/non_goal_heuristic.h"
#include "../generic_defs/single_goal_test.h"
#include "../algorithms/best_first_search/a_star.h"
#include "../utils/string_utils.h"

using namespace std;

int main(int argc, char **argv)
{
    AStar<MapLocation, MapDir> a_star;

    MapPathfindingTransitions map_ops;
    a_star.setTransitionSystem(&map_ops);

    SingleGoalTest<MapLocation> goal_test(MapLocation(0, 0));
    a_star.setGoalTest(&goal_test);

    MapLocHashFunction map_hash;
    a_star.setHashFunction(&map_hash);

    MapEuclideanDistance heuristic;

    vector<MapLocation> starts;
    vector<MapLocation> goals;
    vector<MapDir> solution;

    // Starcraft map tests
    map_ops.loadMap("../src/domains/map_pathfinding/map_files/starcraft_bgh.map");
    map_ops.set4Connected();
    map_hash.setMapDimensions(map_ops);

    a_star.setHeuristic(&heuristic);

    starts.clear();
    goals.clear();
    read_in_pathfinding_probs("../src/domains/map_pathfinding/map_files/starcraft_bgh.probs", starts, goals);
    assert(starts.size() == goals.size());

    vector<uint64_t> expansions;

    for(unsigned i = 0; i < starts.size(); i++) {
        goal_test.setGoal(goals[i]);
        heuristic.setGoal(goals[i]);

        a_star.getPlan(starts[i], solution);

        // prints stats (using goal test count as measure of number of expansions)
        cout << a_star.getLastPlanCost() << "\t" << a_star.getGoalTestCount() << "\t" << a_star.getUniqueGoalTests()
                << endl;

        expansions.push_back(a_star.getGoalTestCount());
    }

    double avg = 0;
    for (unsigned int i=0; i<expansions.size(); i++){
        printf("%" PRIu64 "\n", expansions[i]);
        avg += expansions[i];
    }
    printf("average: %f\n", avg/expansions.size());

    sort(expansions.begin(), expansions.end(), greater<uint64_t>());
    printf("median: %" PRIu64 "\n", expansions[50]);

    return 0;
}

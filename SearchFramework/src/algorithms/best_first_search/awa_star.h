/*
 * awa_star.h
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-10
 *      Author: Dawson Brown
 */

#ifndef AWA_STAR_H_
#define AWA_STAR_H_

#include <float.h>
#include "weighted_a_star.h"
#include "a_star.h"
#include "e_weighted_a_star.h"
#include "best_first_search.h"
#include "open_closed_list.h"
#include <ctime>


template<class state_t, class action_t>
class AWAStar : public WAStar<state_t, action_t>
{
public:
    AWAStar(double weight) : WAStar<state_t, action_t>(weight) {};
    // AWAStar(double weight, double epsilon) : EpsilonWAStar<state_t, action_t>(weight, epsilon) {};
    virtual ~AWAStar();

    /**
     * Returns the list of found solution costs.
     *
     * @return all found solution costs.
     */
    std::vector<double> getSolutionCosts();

    /**
     * @brief get the time it took between each solution
     * 
     */
    std::vector<double> getTimeBetweenSolutions();

    /**
     * Returns the list of costs found at fixed time intervals
     *
     * @return list of solution costs
     */
    std::vector<double> getIntervalCosts(); 


protected:
    virtual void resetEngine();
    virtual SearchTermType searchForPlan(const state_t &init_state);
    std::vector<double> incumbent_g_costs;
    std::vector<double> time_between_solutions;
    std::vector<double> interval_costs;
    virtual double nodeEvalAdmissible(const state_t &state, double g_cost, double h_cost);

    /**
     * Checks whether or not the current node should be generated or not
     *
     * @return bool
     */
    virtual bool skipNodeForGeneration(state_t child_state, double child_g, double child_h);

};

template<class state_t, class action_t>
inline AWAStar<state_t, action_t>::~AWAStar()
{
}

template<class state_t, class action_t>
std::vector<double> AWAStar<state_t, action_t>::getSolutionCosts()
{
    return incumbent_g_costs;
}

template<class state_t, class action_t>
std::vector<double> AWAStar<state_t, action_t>::getTimeBetweenSolutions()
{
    return time_between_solutions;
}

template<class state_t, class action_t>
std::vector<double> AWAStar<state_t, action_t>::getIntervalCosts()
{
    return interval_costs;
}

template<class state_t, class action_t>
void AWAStar<state_t, action_t>::resetEngine()
{
    interval_costs.clear();
    incumbent_g_costs.clear();
    time_between_solutions.clear();
    BestFirstSearch<state_t, action_t>::resetEngine();
}

template<class state_t, class action_t>
bool AWAStar<state_t, action_t>::skipNodeForGeneration(state_t child_state, double child_g, double child_h)
{   
    if (incumbent_g_costs.size() < 1)
        return false;

    double child_eval_admissible = nodeEvalAdmissible(child_state, child_g, child_h);
    if (child_eval_admissible > incumbent_g_costs[incumbent_g_costs.size()-1])
        return true;

    return false;
}

template<class state_t, class action_t>
SearchTermType AWAStar<state_t, action_t>::searchForPlan(const state_t& init_state)
{

/* timer decl */
    std::clock_t start;
    double duration;

    BfsExpansionResult exp_result;

/* timer start */
    // start = std::clock();

    this->heur_func->prepareToCompute();
    double init_h = this->heur_func->getHValue(init_state);

    this->pubIncrementHCompCount();

    double init_eval = this->nodeEval(init_state, 0.0, init_h);

    this->open_closed_list.addInitialNodeToOpen(init_state, this->getTransitionSystem()->getDummyAction(), this->hash_func->getStateHash(init_state),
            init_h, init_eval);

/* convergence timer */
    start = std::clock();

/* convergence expansions */
    // unsigned conv_exp = 50000;

    while( true ){

        exp_result = this->nodeExpansion();

    /* convergence expansions */
        // if (this->getGoalTestCount() >= conv_exp) {
        //     // printf("Interval\n");
        //     if (incumbent_g_costs.size() > 0)
        //         interval_costs.push_back(incumbent_g_costs.back());
        //     else
        //         interval_costs.push_back(DBL_MAX);
        //     conv_exp += 50000;
        // }

    /* convergence timer */
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        if (duration >= 1.0) {
            if (incumbent_g_costs.size() > 0)
                interval_costs.push_back(incumbent_g_costs.back());
            else
                interval_costs.push_back(DBL_MAX);
            start = std::clock();
        }

        if (exp_result == BfsExpansionResult::goal_found) {

        /* timer end */
            // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
            // time_between_solutions.push_back(duration);

            incumbent_g_costs.push_back(this->getLastPlanCost());

        /* timer start */
            // start = std::clock();

        } else if (exp_result == BfsExpansionResult::res_limit){
            return SearchTermType::res_limit;
        }

        if (exp_result == BfsExpansionResult::empty_open){
        
        /* timer end */
            // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
            // time_between_solutions.push_back(duration);

            incumbent_g_costs.push_back(this->getLastPlanCost());
            break;
        }

    /* convergence time limit */
        if (interval_costs.size() >= 60) {
            break;
        }

    /* convergence expansion limit */
        // if (this->getGoalTestCount() >= 3000000) {
        //     break;
        // }    
    }

    return SearchTermType::completed;

}

template<class state_t, class action_t>
inline double AWAStar<state_t, action_t>::nodeEvalAdmissible(const state_t& state, double g_cost, double h_cost)
{
    return g_cost + h_cost;
}


#endif /* A_STAR_H_ */

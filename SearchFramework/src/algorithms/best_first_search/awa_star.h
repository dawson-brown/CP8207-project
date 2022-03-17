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

#include "weighted_a_star.h"
#include "a_star.h"
#include "e_weighted_a_star.h"
#include "best_first_search.h"


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


protected:
    virtual void resetEngine();
    virtual SearchTermType searchForPlan(const state_t &init_state);
    std::vector<double> incumbent_g_costs;
    virtual double nodeEvalAdmissible(const state_t &state, double g_cost, double h_cost);

    /**
     * Checks whether or not the current node should be generated or not
     *
     * @return bool
     */
    virtual bool skipNodeForGeneration(state_t child_state, double child_g);

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
void AWAStar<state_t, action_t>::resetEngine()
{
    incumbent_g_costs.clear();
    BestFirstSearch<state_t, action_t>::resetEngine();
}

template<class state_t, class action_t>
bool AWAStar<state_t, action_t>::skipNodeForGeneration(state_t child_state, double child_g)
{   
    if (incumbent_g_costs.size() < 1)
        return false;

    this->heur_func->prepareToCompute();
    double child_h = this->heur_func->getHValue(child_state);
    double child_eval_admissible = nodeEvalAdmissible(child_state, child_g, child_h);

    if (child_eval_admissible > incumbent_g_costs[incumbent_g_costs.size()-1])
        return true;

    return false;
}

template<class state_t, class action_t>
SearchTermType AWAStar<state_t, action_t>::searchForPlan(const state_t& init_state)
{

    BfsExpansionResult exp_result;

    this->heur_func->prepareToCompute();
    double init_h = this->heur_func->getHValue(init_state);

    this->pubIncrementHCompCount();

    double init_eval = this->nodeEval(init_state, 0.0, init_h);

    this->open_closed_list.addInitialNodeToOpen(init_state, this->getTransitionSystem()->getDummyAction(), this->hash_func->getStateHash(init_state),
            init_h, init_eval);

    while( true ){

        exp_result = this->nodeExpansion();

        if (exp_result == BfsExpansionResult::empty_open){
            break;
        }

        if (exp_result == BfsExpansionResult::goal_found) {
            incumbent_g_costs.push_back(this->getLastPlanCost());
        } else if (exp_result == BfsExpansionResult::res_limit){
            break;
        }
    }


    if(exp_result == BfsExpansionResult::res_limit) {
        return SearchTermType::res_limit;
    }
    return SearchTermType::completed;

}

template<class state_t, class action_t>
inline double AWAStar<state_t, action_t>::nodeEvalAdmissible(const state_t& state, double g_cost, double h_cost)
{
    return g_cost + h_cost;
}


#endif /* A_STAR_H_ */

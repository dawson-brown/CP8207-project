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
     * Returns the list of lower bounds on solution costs.
     *
     * @return all lower bounds after each iteration.
     */
    // std::vector<double> getLowerBounds(); 


protected:
    virtual void resetEngine();
    virtual SearchTermType searchForPlan(const state_t &init_state);
    std::vector<double> incumbent_g_costs;
    // std::vector<double> lower_bounds;
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

// template<class state_t, class action_t>
// std::vector<double> AWAStar<state_t, action_t>::getLowerBounds()
// {
//     return lower_bounds;
// }

template<class state_t, class action_t>
void AWAStar<state_t, action_t>::resetEngine()
{
    // lower_bounds.clear();
    incumbent_g_costs.clear();
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

    BfsExpansionResult exp_result;

    this->heur_func->prepareToCompute();
    double init_h = this->heur_func->getHValue(init_state);

    this->pubIncrementHCompCount();

    double init_eval = this->nodeEval(init_state, 0.0, init_h);
    // lower_bounds.push_back(init_h);

    this->open_closed_list.addInitialNodeToOpen(init_state, this->getTransitionSystem()->getDummyAction(), this->hash_func->getStateHash(init_state),
            init_h, init_eval);

    while( true ){

        exp_result = this->nodeExpansion();

        if (exp_result == BfsExpansionResult::goal_found) {
            incumbent_g_costs.push_back(this->getLastPlanCost());

            // // // comment out for time complexity eval
            // double min_f = DBL_MAX;
            // std::vector<NodeID> open_list = this->open_closed_list.getOpenListHeap();
            // for (size_t i=0; i<open_list.size(); i++) {
            //     NodeID node_id = open_list[i];
            //     BFSNode<state_t, action_t> node = this->open_closed_list.getNode(node_id);
            //     double f_cost = nodeEvalAdmissible(node.state, node.g_cost, node.h_value);
            //     if (f_cost < min_f) {
            //         min_f = f_cost;
            //     }
            // }
            // lower_bounds.push_back(incumbent_g_costs.back() - min_f);

        } else if (exp_result == BfsExpansionResult::res_limit){
            return SearchTermType::res_limit;
        }

        if (exp_result == BfsExpansionResult::empty_open){
            incumbent_g_costs.push_back(this->getLastPlanCost());
            break;
        }
    }

    return SearchTermType::completed;

}

template<class state_t, class action_t>
inline double AWAStar<state_t, action_t>::nodeEvalAdmissible(const state_t& state, double g_cost, double h_cost)
{
    return g_cost + h_cost;
}


#endif /* A_STAR_H_ */

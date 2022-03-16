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
     * Expands a single node and returns the result of the expansion (regarding if a solution as found or not).
     *
     * @return The result of the node expansion.
     */
    // virtual BfsExpansionResult nodeExpansion();

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

// template<class state_t, class action_t>
// BfsExpansionResult AWAStar<state_t, action_t>::nodeExpansion()
// {
//     if(this->open_closed_list.isOpenEmpty())
//         return BfsExpansionResult::empty_open;

//     NodeID to_expand_id = this->getNodeForExpansion();

//     BFSNode<state_t, action_t> to_expand_node = this->open_closed_list.getNode(to_expand_id);
//     //std::cout << "Expanding " << to_expand_id << " - " << to_expand_node.state << "," << to_expand_node.gen_action << std::endl;

//     if(this->hitGoalTestLimit())
//         return BfsExpansionResult::res_limit;

//     this->incrementGoalTestCount();
//     if(!to_expand_node.reopened)
//         this->unique_goal_tests++; // change this to a function

//     if(this->goal_test->isGoal(to_expand_node.state)) { // checks for goal
//         this->extractSolutionPath(to_expand_id);
//         return BfsExpansionResult::goal_found;
//     }

//     double parent_g = to_expand_node.g_cost;

//     if(this->hitSuccFuncLimit())
//         return BfsExpansionResult::res_limit;

//     this->incrementSuccFuccCalls();

//     this->app_actions.clear();
//     this->getTransitionSystem()->getActions(to_expand_node.state, this->app_actions);
//     this->increaseActionGenCount(this->app_actions.size());

//     for(unsigned i = 0; i < this->app_actions.size(); i++) {

//         double edge_cost = this->getTransitionSystem()->getActionCost(to_expand_node.state, this->app_actions[i]);
//         double child_g = parent_g + edge_cost;

//         state_t child_state = to_expand_node.state;
//         this->pubIncrementHCompCount();
//         this->heur_func->prepareToCompute();
//         double child_h = this->heur_func->getHValue(child_state);
//         double child_eval_admissible = nodeEvalAdmissible(child_state, child_g, child_h);

//         if (child_eval_admissible >= incumbent_g_costs[incumbent_g_costs.size()-1])
//             continue;

//         this->getTransitionSystem()->applyAction(child_state, this->app_actions[i]);
//         this->incrementStateGenCount();

//         StateHash child_hash = this->hash_func->getStateHash(child_state);
//         NodeID child_id;
//         StateLocation child_loc = this->open_closed_list.getStateLocation(child_state, child_hash, child_id);

//         if(child_loc == StateLocation::open || child_loc == StateLocation::closed) {
//             if(fp_less(child_g, this->open_closed_list.getNode(child_id).g_cost)) {
//                 this->open_closed_list.getNode(child_id).g_cost = child_g;
//                 this->open_closed_list.getNode(child_id).eval = nodeEval(child_state, child_g,
//                         this->open_closed_list.getNode(child_id).h_value);
//                 this->open_closed_list.getNode(child_id).parent_id = to_expand_id;
//                 this->open_closed_list.getNode(child_id).gen_action = this->app_actions[i];

//                 if(child_loc == StateLocation::open)
//                     this->open_closed_list.openNodeEvalChanged(child_id);
//                 else
//                     this->open_closed_list.reopenNode(child_id);
//             }
//         } else {

//             if(this->hitHCompLimit())
//                 return BfsExpansionResult::res_limit;

//             double child_eval = nodeEval(child_state, child_g, child_h);

//             //std::cout << "New Child " << child_state << " eval " << child_eval << std::endl;
//             this->open_closed_list.addNewNodeToOpen(child_state, this->app_actions[i], child_hash, child_g, child_h, child_eval,
//                     to_expand_id);
//         }
//     }

//     return BfsExpansionResult::no_solution;
// }

template<class state_t, class action_t>
bool AWAStar<state_t, action_t>::skipNodeForGeneration(state_t child_state, double child_g)
{   
    if (incumbent_g_costs.size() < 1)
        return false;

    this->heur_func->prepareToCompute();
    double child_h = this->heur_func->getHValue(child_state);
    double child_eval_admissible = nodeEvalAdmissible(child_state, child_g, child_h);

    if (child_eval_admissible >= incumbent_g_costs[incumbent_g_costs.size()-1])
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



// template<class state_t, class action_t>
// SearchTermType AWAStar<state_t, action_t>::getPlan(const state_t& init_state, std::vector<action_t>& sol_plan)
// {

//     resetEngine();

//     if(alg_status == SearchStatus::not_ready)
//         return SearchTermType::engine_not_ready;

//     alg_status = SearchStatus::active;

//     while(open_closed_list.isOpenEmpty()) {
//         SearchTermType term = searchForPlan(init_state);
//     }

//     alg_status = SearchStatus::terminated;

//     sol_plan.clear();
//     if(incumbent_plan.size() > 0) {
//         for(unsigned i = 0; i < incumbent_plan.size(); i++) {
//             sol_plan.push_back(incumbent_plan[i]);
//         }
//     }

//     return term;
// }


#endif /* A_STAR_H_ */

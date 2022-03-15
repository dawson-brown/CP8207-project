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

protected:
    virtual SearchTermType searchForPlan(const state_t &init_state);
    std::vector<unsigned> incumbent_lengths;

};

template<class state_t, class action_t>
inline AWAStar<state_t, action_t>::~AWAStar()
{
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
            incumbent_lengths.push_back(this->getLastPlanCost());
        } else if (exp_result == BfsExpansionResult::res_limit){
            break;
        }
    }


    if(exp_result == BfsExpansionResult::res_limit)
        return SearchTermType::res_limit;
    return SearchTermType::completed;

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

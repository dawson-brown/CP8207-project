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
class AWAStar : public BestFirstSearch<state_t, action_t> 
{
public:
    AWAStar(double weight) : WAStar<state_t, action_t>(weight) {};
    AWAStar(double weight, double epsilon) : EpsilonWAStar<state_t, action_t>(weight, epsilon) {
        mid_search = false;
    };
    virtual ~AWAStar();   
    virtual SearchTermType searchForPlan(const state_t &init_state);
    SearchTermType getPlan(const state_t &init_state, std::vector<action_t> &sol_plan); 

private:
    bool mid_search;

};

template<class state_t, class action_t>
inline AWAStar<state_t, action_t>::~AWAStar()
{
}

template<class state_t, class action_t>
SearchTermType AWAStar<state_t, action_t>::searchForPlan(const state_t& init_state)
{
    BfsExpansionResult exp_result = BfsExpansionResult::no_solution;

    if (mid_search == false) {

        heur_func->prepareToCompute();
        double init_h = heur_func->getHValue(init_state);

        incrementHCompCount();

        double init_eval = nodeEval(init_state, 0.0, init_h);

        open_closed_list.addInitialNodeToOpen(init_state, op_system->getDummyAction(), hash_func->getStateHash(init_state),
                init_h, init_eval);
    } else {
        // TODO: the meat of AWA* i guess? 
    }

    while(exp_result == BfsExpansionResult::no_solution)
        exp_result = nodeExpansion();


    if (mid_search == false){
        mid_search = true;
    }


    if(exp_result == BfsExpansionResult::res_limit)
        return SearchTermType::res_limit;
    return SearchTermType::completed;

}

template<class state_t, class action_t>
SearchTermType AWAStar<state_t, action_t>::getPlan(const state_t& init_state, std::vector<action_t>& sol_plan)
{

    resetEngine();

    if(alg_status == SearchStatus::not_ready)
        return SearchTermType::engine_not_ready;

    alg_status = SearchStatus::active;

    while(open_closed_list.isOpenEmpty()) {
        SearchTermType term = searchForPlan(init_state);
    }

    alg_status = SearchStatus::terminated;

    sol_plan.clear();
    if(incumbent_plan.size() > 0) {
        for(unsigned i = 0; i < incumbent_plan.size(); i++) {
            sol_plan.push_back(incumbent_plan[i]);
        }
    }

    return term;
}


#endif /* A_STAR_H_ */

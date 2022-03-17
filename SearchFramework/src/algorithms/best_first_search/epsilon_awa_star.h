/*
 * awa_star.h
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-10
 *      Author: Dawson Brown
 */

#ifndef E_AWA_STAR_H_
#define E_AWA_STAR_H_

#include "awa_star.h"
#include "e_weighted_a_star.h"
#include "open_closed_list.h"


template<class state_t, class action_t>
class EpsilonAWAStar : public AWAStar<state_t, action_t>
{

public:
    EpsilonAWAStar(double epsilon, double weight) : AWAStar<state_t, action_t>(weight) {
        srand (time(NULL));
        this->epsilon = epsilon;
    };
    // AWAStar(double weight, double epsilon) : EpsilonWAStar<state_t, action_t>(weight, epsilon) {};
    virtual ~EpsilonAWAStar();


protected:
    double epsilon;

    /**
     * @brief Get the Node For Expansion
     * 
     * @return NodeID 
     */
    virtual NodeID getNodeForExpansion();


};

template<class state_t, class action_t>
inline EpsilonAWAStar<state_t, action_t>::~EpsilonAWAStar()
{
}


template<class state_t, class action_t>
NodeID EpsilonAWAStar<state_t, action_t>::getNodeForExpansion()
{
    // bool keep_searching = true;
    // NodeID to_expand_id;

    // while (keep_searching) {
    //     double choice = (double)rand() / RAND_MAX;
    //     if (choice <= this->epsilon) {
    //         to_expand_id = this->open_closed_list.getRandomNodeAndClose();
    //         BFSNode<state_t, action_t> to_expand_node = this->open_closed_list.getNode(to_expand_id);
    //         double node_eval = this->nodeEvalAdmissible(to_expand_node.state, to_expand_node.g_cost, to_expand_node.h_value);
    //         keep_searching = this->incumbent_g_costs.size() > 0 ? node_eval > this->incumbent_g_costs[this->incumbent_g_costs.size()-1] : false;
    //     } else {
    //         to_expand_id = this->open_closed_list.getBestNodeAndClose();
    //         keep_searching = false;
    //     }
    // }

    NodeID to_expand_id;
    double choice = (double)rand() / RAND_MAX;
    if (choice <= this->epsilon)
        to_expand_id = this->open_closed_list.getRandomNodeAndClose();
    else
        to_expand_id = this->open_closed_list.getBestNodeAndClose();

    return to_expand_id;
}

#endif /* A_STAR_H_ */

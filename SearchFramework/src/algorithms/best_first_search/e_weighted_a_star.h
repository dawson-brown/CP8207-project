/*
 * weighted_a_star.h
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-10
 *      Author: Dawson Brown
 */

#ifndef E_WEIGHTED_A_STAR_H_
#define E_WEIGHTED_A_STAR_H_

#include "weighted_a_star.h"

template<class state_t, class action_t>
class EpsilonWAStar: public WAStar<state_t, action_t>
{
public:
    EpsilonWAStar(double h_weight, double epsilon) : WAStar<state_t, action_t>(h_weight) {
        this->epsilon = epsilon;
    };
    virtual ~EpsilonWAStar();



protected:
    double epsilon;

    /**
     * @brief Get the Node For Expansion
     * 
     * @return NodeID 
     */
    virtual NodeID getNodeForExpansion();
};

// template<class state_t, class action_t>
// inline EpsilonWAStar<state_t, action_t>::EpsilonWAStar(double h_weight, double epsilon)
// {
//     this->weight = h_weight;
//     this->epsilon = epsilon;
// }

template<class state_t, class action_t>
inline EpsilonWAStar<state_t, action_t>::~EpsilonWAStar()
{
}

template<class state_t, class action_t>
NodeID EpsilonWAStar<state_t, action_t>::getNodeForExpansion()
{
    NodeID to_expand_id;
    double choice = (double)rand() / RAND_MAX;
    if (choice <= this->epsilon)
        to_expand_id = this->open_closed_list.getRandomNodeAndClose();
    else
        to_expand_id = this->open_closed_list.getBestNodeAndClose();

    return to_expand_id;
}

#endif
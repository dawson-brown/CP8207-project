/*
 * weighted_a_star.h
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-10
 *      Author: Dawson Brown
 */

#ifndef WEIGHTED_A_STAR_H_
#define WEIGHTED_A_STAR_H_

#include "a_star.h"

template<class state_t, class action_t>
class WAStar: public AStar<state_t, action_t>
{
public:
    WAStar(double h_weight);
    virtual ~WAStar();

protected:
    double weight;
    virtual double nodeEval(const state_t &state, double g_cost, double h_cost);
};

template<class state_t, class action_t>
inline WAStar<state_t, action_t>::WAStar(double h_weight)
{
    this.weight = h_weight;
}

template<class state_t, class action_t>
inline WAStar<state_t, action_t>::~WAStar()
{
}

template<class state_t, class action_t>
inline double WAStar<state_t, action_t>::nodeEval(const state_t& state, double g_cost, double h_cost)
{
    return g_cost + this.weight*h_cost;
}

#endif /* A_STAR_H_ */

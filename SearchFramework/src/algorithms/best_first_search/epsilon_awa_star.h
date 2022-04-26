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

    void setEpsilon(double epsilon);


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
void EpsilonAWAStar<state_t, action_t>::setEpsilon(double epsilon)
{
    this->epsilon = epsilon;
}

template<class state_t, class action_t>
NodeID EpsilonAWAStar<state_t, action_t>::getNodeForExpansion()
{
    double choice = (double)rand() / RAND_MAX;
    if (choice <= this->epsilon)
        return  this->open_closed_list.getNodeAndClose(
            rand() % this->open_closed_list.openListSize()
        );
    else
        return this->open_closed_list.getNodeAndClose(0);

}

#endif /* A_STAR_H_ */

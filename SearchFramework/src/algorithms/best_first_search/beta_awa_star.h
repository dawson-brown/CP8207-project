/*
 * awa_star.h
 *
 * LICENSE HERE
 *
 *  Created on: 2022-03-10
 *      Author: Dawson Brown
 */

#ifndef BETA_AWA_STAR_H_
#define BETA_AWA_STAR_H_

#include "awa_star.h"
#include "open_closed_list.h"


template<class state_t, class action_t>
class BetaAWAStar : public AWAStar<state_t, action_t>
{

public:
    BetaAWAStar(double epsilon, double weight, double alpha, double beta) : AWAStar<state_t, action_t>(weight) {
        srand (time(NULL));
        this->epsilon = epsilon;
        this->alpha_distribution = std::gamma_distribution<double>(alpha,1.0);
        this->beta_distribution = std::gamma_distribution<double>(beta,1.0);
    };
    // AWAStar(double weight, double epsilon) : EpsilonWAStar<state_t, action_t>(weight, epsilon) {};
    virtual ~BetaAWAStar();

    void setEpsilon(double epsilon);


protected:
    double epsilon;
    std::default_random_engine generator;
    std::gamma_distribution<double> alpha_distribution;
    std::gamma_distribution<double> beta_distribution;

    /**
     * @brief Get the Node For Expansion
     * 
     * @return NodeID 
     */
    virtual NodeID getNodeForExpansion();


};

template<class state_t, class action_t>
inline BetaAWAStar<state_t, action_t>::~BetaAWAStar()
{
}

template<class state_t, class action_t>
void BetaAWAStar<state_t, action_t>::setEpsilon(double epsilon)
{
    this->epsilon = epsilon;
}

template<class state_t, class action_t>
NodeID BetaAWAStar<state_t, action_t>::getNodeForExpansion()
{   
    double max_row = floor(log2(this->open_closed_list.openListSize()));

    double choice = (double)rand() / RAND_MAX;
    if (choice <= this->epsilon) {
        double alpha_sample = alpha_distribution(generator);
        double beta_sample = beta_distribution(generator);
        double row = round( (alpha_sample/(alpha_sample+beta_sample)) * max_row );

        unsigned start_of_row = (unsigned) pow(2, row) - 1;
        unsigned end_of_row = 2*start_of_row;

        unsigned sample_i = start_of_row + rand() % (( end_of_row + 1 ) - start_of_row);
        if (sample_i >= this->open_closed_list.openListSize()) // noticed too late that this biases the last element of the last row
            sample_i = this->open_closed_list.openListSize()-1;

        return this->open_closed_list.getNodeAndClose(sample_i);
    }   
    else
        return this->open_closed_list.getNodeAndClose(0);
}

#endif /* A_STAR_H_ */

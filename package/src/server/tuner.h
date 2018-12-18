#pragma once

#include <vector>

/*
 * The Client class supports using multiple different tuning algorithms that
 * must use this as a base class.
 *
 * For convience, the tuners take parameters and give feedback as nice vectors
 * of floats, not that std_msgs crap.
 */
class Tuner {
public:
    typedef std::vector<double> parameters;
    typedef std::vector<double> feedback;

    // Called once per client
    virtual bool init(int num_params, const feedback &goals) = 0;

    // Called once after init() to get the inital starting guess to send
    virtual parameters get_inital() = 0;

    // Called over and over again with each new set of feedback
    virtual parameters iterate(const feedback &feedback) = 0;
};

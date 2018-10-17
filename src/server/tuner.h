#pragma once

#include <vector>

class Tuner {
public:
    typedef std::vector<double> parameters;
    typedef std::vector<double> feedback;

    virtual bool init(int num_params, const feedback &goals) = 0;
    virtual parameters iterate(const feedback &feedback) = 0;
    virtual parameters get_inital() = 0;
};

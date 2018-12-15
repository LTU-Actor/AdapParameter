#pragma once

#include "tuner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <numeric>
#include <vector>
#include <functional>

class TunerGradients : public Tuner {
public:
    bool init(int num_params, const feedback &goals)
    {
        params.resize(num_params);
        std::fill(params.begin(), params.end(), 0.5);
        this->goals = goals;
        single_tuners.reserve(num_params);
        for(int i=0; i<num_params; i++)
            single_tuners.emplace_back(SingleGradient(params.at(i)));
        current_tuner = single_tuners.begin();
        return true;
    }

    parameters iterate(const feedback &feedback)
    {
        double error = calc_error(feedback);

        current_tuner->feedback(error);
        current_tuner++;
        if (current_tuner == single_tuners.end()) current_tuner = single_tuners.begin();
        current_tuner->set_next(error);

        return params;
    }

    parameters get_inital() { return params; }

private:
    class SingleGradient {
    public:
        SingleGradient(double &parameter) : parameter_ref(parameter)
        {
            double nan = std::numeric_limits<double>::quiet_NaN();
            starting_parameter = nan;
            starting_error = nan;
            to_apply = 0.5;
        }

        void feedback(double error)
        {
            if (error < 0) error = -error;
            const float parameter = parameter_ref.get();

            double ret;
            double slope = (starting_error - error) / (starting_parameter - parameter);
            if (std::isnan(slope))
            {
                if (parameter < .9)
                    ret = parameter + 0.05;
                else
                    ret = parameter - 0.05;
            }
            else if (std::abs(slope) < alpha)
                ret = 2 * parameter - starting_parameter;
            else
                ret = parameter - slope * alpha * error;

            if (ret > 1)
                ret = 1;
            else if (ret < 0)
                ret = 0;

            const double return_to_center = error * alpha;
            ret = ret * (1 - return_to_center) + .5 * (return_to_center);

            to_apply = ret;
        }

        void set_next(double error)
        {
            if (error < 0) error = -error;
            starting_error = error;
            starting_parameter = parameter_ref.get();
            parameter_ref.get() = to_apply;
        }

        SingleGradient(const SingleGradient &) = default;
    private:

        double to_apply;
        double slope = 0;
        double alpha = 0.1;
        double starting_parameter;
        double starting_error;
        std::reference_wrapper<double> parameter_ref;
    };

    // return distance from goals
    double calc_error(feedback fb)
    {
        for (feedback::iterator f = fb.begin(), g = goals.begin();
             f != fb.end() && g != goals.end(); f++, g++)
            *f = *g - *f;

        double sum = 0;
        for (const auto s : fb) sum += s * s;

        return std::sqrt(sum);
    }

    feedback goals;
    parameters params;
    std::vector<SingleGradient> single_tuners;
    std::vector<SingleGradient>::iterator current_tuner;
};

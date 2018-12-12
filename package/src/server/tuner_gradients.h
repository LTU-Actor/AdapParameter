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
        past_errors = Statistics();
        current_tuner = single_tuners.begin();
        return true;
    }

    parameters iterate(const feedback &feedback)
    {
        double error = calc_error(feedback);

        current_tuner->feedback(error);
        current_tuner++;
        if (current_tuner == single_tuners.end()) current_tuner = single_tuners.begin();
        current_tuner->apply(error);

        return params;
    }

    parameters get_inital() { return params; }

private:
    class SingleGradient {
    public:
        SingleGradient(double &parameter) : parameter_m1(parameter), parameter(parameter)
        {
            double nan = std::numeric_limits<double>::quiet_NaN();
            parameter_m2 = nan;
            error_last = nan;
        }

        double feedback(double error)
        {
            if (error < 0) error *= -1;

            double ret;
            double slope = calc_slope(error);

            if (std::isnan(slope))
            {
                if (parameter_m1 < .9)
                    ret = parameter_m1 + 0.05;
                else if (parameter_m1 > .1)
                    ret = parameter_m1 - 0.05;
                else
                    ret = 0.5;
            }
            else
                ret = parameter_m1 - slope * alpha * error;

            if (ret > 1)
                ret = 1;
            else if (ret < 0)
                ret = 0;

            ret = ret * .95 + .5 * .05;

            parameter_m2 = parameter_m1;
            parameter_m1 = ret;

            to_apply = ret;
            return ret;
        }

        void apply(double error)
        {
            error_last = error;
            parameter.get() = to_apply;
        }

        SingleGradient(const SingleGradient &) = default;
    private:

        double to_apply;
        double slope = 0;
        double alpha = 0.1;

        double calc_slope(double error)
        {
            if (parameter_m1 == parameter_m2)
                return 0;
            return (error_last - error) / (parameter_m2 - parameter_m1);
        }

        double parameter_m1; // parameter last time
        double parameter_m2; // parameter before last
        double error_last;
        std::reference_wrapper<double> parameter;
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

    struct Statistics {
        void push(double d)
        {
            data[index] = d;
            index = (index + 1) % data.size();
        }

        double avg_diff()
        {
            if (std::any_of(data.begin(), data.end(),
                            [](double d) { return std::isnan(d); }))
                return std::numeric_limits<double>::quiet_NaN();

            auto bb = index;
            auto be = (index + 25) % data.size() + 1;
            auto eb = (index + 74) % data.size();
            auto ee = (index + 99) % data.size() + 1;

            double sum_begin, sum_end;

            if (bb < be)
            {
                sum_begin =
                    std::accumulate(data.begin() + bb, data.begin() + be, 0.0f);
            }
            else
            {
                sum_begin =
                    std::accumulate(data.begin() + bb, data.end(), 0.0f)
                    + std::accumulate(data.begin(), data.begin() + be, 0.0f);
            }


            if (eb < ee)
            {
                sum_end =
                    std::accumulate(data.begin() + eb, data.begin() + ee, 0.0f);
            }
            else
            {
                sum_end =
                    std::accumulate(data.begin() + eb, data.end(), 0.0f)
                    + std::accumulate(data.begin(), data.begin() + ee, 0.0f);
            }

            return sum_end - sum_begin;
        }

        Statistics() { data.fill(std::numeric_limits<double>::quiet_NaN()); }

        std::array<double, 100> data;
        int index = 0;
    } past_errors;

    feedback goals;
    parameters params;
    std::vector<SingleGradient> single_tuners;
    std::vector<SingleGradient>::iterator current_tuner;
};

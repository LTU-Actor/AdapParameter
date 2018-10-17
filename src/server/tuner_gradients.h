#pragma once

#include "tuner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <numeric>
#include <vector>

class TunerGradients : public Tuner {
public:
    bool init(int num_params, const feedback &goals)
    {
        params.resize(num_params);
        std::fill(params.begin(), params.end(), 0.5);
        this->goals = goals;
        target_param = params.begin();
        single_tuner = SingleGradient(*target_param);
        past_errors = Statistics();
        return true;
    }

    parameters iterate(const feedback &feedback)
    {
        double error = calc_error(feedback);
        *target_param = single_tuner.iterate(error);


        past_errors.push(error);
        double avg_diff = past_errors.avg_diff();
        if (avg_diff < 0 && avg_diff > -0.0005)
        {
            ROS_INFO_STREAM("NEXT: " << avg_diff);
            target_param++;
            if (target_param == params.end()) target_param = params.begin();
            single_tuner = SingleGradient(*target_param);
            past_errors = Statistics();
        }

        return params;
    }

    parameters get_inital() { return params; }

private:
    class SingleGradient {
    public:
        SingleGradient(double inital_parameter) : parameter_m1(inital_parameter)
        {
            double nan = std::numeric_limits<double>::quiet_NaN();
            parameter_m2 = nan;
            error_last = nan;
        }

        SingleGradient() {} // Do nothing. Object is is useless and will throw
                            // floating point exception on use.

        double iterate(double error)
        {
            if (error < 0) error *= -1;

            double ret;
            double slope = calc_slope(error);

            if (std::isnan(slope))
                ret = parameter_m1 + 0.05;
            else
                ret = parameter_m1 - slope * alpha;

            error_last = error;
            parameter_m2 = parameter_m1;
            parameter_m1 = ret;
            return ret;
        }

    private:
        static constexpr double alpha = 0.001;

        double calc_slope(double error)
        {
            if(parameter_m1 == parameter_m2) return 0;
            return (error_last - error) / (parameter_m2 - parameter_m1);
        }

        double parameter_m1; // parameter last time
        double parameter_m2; // parameter before last
        double error_last;
    };

    // return distance from goals
    double calc_error(feedback fb)
    {
        for (feedback::iterator f = fb.begin(), g = goals.begin();
             f != fb.end() && g != goals.end(); f++, g++)
        { *f = *g - *f; } double sum = 0;
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
            auto be = (index + 24) % data.size() + 1;
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

    SingleGradient single_tuner;
    parameters::iterator target_param;
};

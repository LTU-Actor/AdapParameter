#include "tuner_gradients.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <numeric>
#include <vector>

bool
TunerGradients::init(int num_params, const Tuner::feedback &goals)
{
    this->goals = goals;

    // Start out with all params at 0.5
    params.resize(num_params);
    std::fill(params.begin(), params.end(), 0.5);

    // Create all slope decent objects and start working with the first
    single_tuners.reserve(num_params);
    for (int i = 0; i < num_params; i++)
        single_tuners.emplace_back(SingleGradient(params.at(i)));
    current_tuner = single_tuners.begin();
    return true;
}

Tuner::parameters
TunerGradients::iterate(const Tuner::feedback &feedback)
{
    double error = calc_error(feedback);

    // Immedeately after set_next, check if it got better or worse and calculate
    // the next setpoint. Don't set yet.
    current_tuner->calculate(error);

    // Go to the next parameter.
    current_tuner++;
    if (current_tuner == single_tuners.end())
        current_tuner = single_tuners.begin();

    // Set the calculated value
    current_tuner->set_next(error);

    // Return our new guess
    return params;
}

Tuner::parameters
TunerGradients::get_inital()
{
    return params;
}

TunerGradients::SingleGradient::SingleGradient(double &parameter)
  : parameter_ref(parameter)
{
    // Makes checking for edge cases when calculate or set_next has not been
    // set.
    double nan = std::numeric_limits<double>::quiet_NaN();
    starting_parameter = nan;
    starting_error = nan;
    to_apply = nan;
}

void
TunerGradients::SingleGradient::calculate(double error)
{
    if (error < 0) error = -error;
    const float parameter = parameter_ref.get();

    // How much change out last guess made
    double slope = (starting_error - error) / (starting_parameter - parameter);

    // For the rest of this function, cap error to [-1,1] to limit excessively
    // large jumps
    if (error > 1)
        error = 1;
    else if (error < -1)
        error = -1;

    // Either set_next has not run, or starting_parameter == parameter
    if (std::isnan(slope))
    {
        if (parameter < .9)
            to_apply = parameter + 0.05 * error;
        else
            to_apply = parameter - 0.05 * error;
    }
    // If slope is super small, make a bigger jump in the same direction
    else if (std::abs(slope) < alpha * error)
        to_apply = 2 * parameter - starting_parameter;
    // Otherwise (normally), make a jump proportional to our slope.
    // Slow down as we get closer to optimal.
    else
        to_apply = parameter - slope * alpha * error;

    // Limit output to [-1,1]
    if (to_apply > 1)
        to_apply = 1;
    else if (to_apply < 0)
        to_apply = 0;

    // Slowly get pulled back to center, especially when we have high error.
    // This is the only mechanism currently to get out of being stuck at either
    // extreme end. For example, when a binary threshold is at 0, we will always
    // get back maximum error. This error will never change, so slope will
    // always be 0, and we will be stuck.
    const double return_to_center = error * alpha;
    to_apply = to_apply * (1 - return_to_center) + .5 * (return_to_center);
}

void
TunerGradients::SingleGradient::set_next(double error)
{
    if (error < 0) error = -error;
    starting_error = error;
    starting_parameter = parameter_ref.get();
    if (!std::isnan(to_apply)) parameter_ref.get() = to_apply;
}

// return n-dimensional distance from goals
double
TunerGradients::calc_error(Tuner::feedback fb)
{
    for (Tuner::feedback::iterator f = fb.begin(), g = goals.begin();
         f != fb.end() && g != goals.end(); f++, g++)
        *f = *g - *f;

    double sum = 0;
    for (const auto s : fb) sum += s * s;

    return std::sqrt(sum);
}

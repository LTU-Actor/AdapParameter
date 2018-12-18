#pragma once

#include "tuner.h"

#include <functional>

/*
 * Create a SingleGradient tuner per parameter from the client that works
 * separately. Each tune will change only one parameter at a time, and then move
 * to the next.
 *
 * SingleGradient is a slope decent optimizer that has the possibility of
 * getting stuck in one of two ways:
 *  1. In a local minima
 *  2. When there is no change in feedback from a change
 *
 * In the second case, it is possible to take larger and larger jumps in the
 * same direction until we get a change.
 */
class TunerGradients : public Tuner {
public:
    // Interface from the virtual Tuner class
    bool init(int num_params, const feedback &goals);
    parameters get_inital();
    parameters iterate(const feedback &feedback);

private:
    class SingleGradient {
        // This seems to work well for most applications. It is still fast
        // to tune, but does not overshoot much on noisy feedback.
        constexpr static double alpha = 0.1;

    public:
        // Remembers a reference to a parameter to make updates to
        SingleGradient(double &parameter);

        // Takes the feedback from immediately after the feedback given to
        // set_next. It will calculate the next jump to take, but will wait to
        // publish it until next cycle.
        void calculate(double error);

        // Sets the calculated value, and remembers the starting error given
        void set_next(double error);

    private:
        double to_apply;
        double starting_parameter;
        double starting_error;
        std::reference_wrapper<double> parameter_ref;
    };

    // return distance from goals
    double calc_error(feedback fb);

    feedback goals;
    parameters params;
    std::vector<SingleGradient> single_tuners;
    std::vector<SingleGradient>::iterator current_tuner;
};

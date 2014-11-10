#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

namespace s8 {
    namespace pid {
        class PIDController {
            double sum_errors;
            double prev_error;
            int hz;

        public:
            //These are public so that they can be set by parameter system.
            double kp;
            double ki;
            double kd;

        public:
            PIDController(int hz) : hz(hz) {
                reset();
            }

            void reset() {
                sum_errors = prev_error = 0.0;
            }

            void update(double & value, double diff) {
                p_controller(value, diff);
                i_controller(value, diff);
                d_controller(value, diff);
            }

            void p_controller(double & value, double diff) {
                value += kp * diff;
            }

            void i_controller(double & value, double diff) {
                value += ki * sum_errors;
                sum_errors += diff / hz; //Formula is diff * dt. "/ hz" is the same as "* (1.0 / hz)" and "(1.0 / hz)" = dt.
            }

            void d_controller(double & value, double diff) {
                value += kd * (diff - prev_error) * hz; // Formula is kd * d / dt. "* hz" is the same as "/ (1.0 / hz)" and "(1.0 / hz)" = dt.
                prev_error = diff;
            }

            double get_prev_error() {
                return prev_error;
            }

            double get_sum_errors() {
                return sum_errors;
            }
        };
    }
}

#endif

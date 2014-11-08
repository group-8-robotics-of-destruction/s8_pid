#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

namespace s8 {
    namespace pid {
        class PIDController {
            double sum_errors;
            double prev_error;
            double delta_time;

        public:
            //These are public so that they can be set by parameter system.
            double kp;
            double ki;
            double kd;

        public:
            PIDController(int hz) : delta_time(1.0 / hz) {
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
                sum_errors += diff * delta_time;
            }

            void d_controller(double & value, double diff) {
                value += kd * (diff - prev_error) * delta_time;
                prev_error = diff;
            }
        };
    }
}

#endif

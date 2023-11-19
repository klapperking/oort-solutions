use oort_api::prelude::*;
use crate::maths_rs::abs;

pub struct Ship {
    kp: f64,
    ki: f64,
    kd: f64,

    lim_max: f64,
    lim_min: f64,

    sample_time: f64,
    integrator: f64,
    differentiator: f64,
    prev_error: f64,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            kp: 6.0,
            ki: 0.5,
            kd: 130.0,

            lim_max: max_angular_acceleration(),
            lim_min: -max_angular_acceleration(),

            sample_time: 1.0,
            integrator: 0.0,
            differentiator: 0.0,
            prev_error: 0.0,

        }
    }

    pub fn pid_controller(&mut self) {

    }


    pub fn tick(&mut self) {

        let error = angle_diff(heading(), (target() - position()).angle());

        // p
        let proportional = self.kp * error;

        // i
        self.integrator += 0.5 * self.ki * self.sample_time * (error - self.prev_error);

        let mut max_integrator = 0.0;
        let mut min_integrator = 0.0;

        if self.lim_max > proportional {
            max_integrator = self.lim_max - proportional;
        }

        if self.lim_min < proportional {
            min_integrator = self.lim_min - proportional;
        }


        if self.integrator > max_integrator {
            self.integrator = max_integrator;
        } else if self.integrator < min_integrator {
            self.integrator = min_integrator;
        }

        // d
        self.differentiator = (2.0 * self.kd * (error - self.prev_error)
                            + (2.0 * TAU - self.sample_time) * self.differentiator)
                            / (2.0 * TAU + self.sample_time);

        let mut control_signal = proportional + self.integrator + self.differentiator;

        if control_signal > self.lim_max {
            control_signal = self.lim_max;
        } else if control_signal < self.lim_min {
            control_signal = self.lim_min;
        }

        // update for next call
        self.prev_error = error;

        // Change angular_acceleration with control_signal
        torque(control_signal);

        if abs(error) < PI / 4.0 {
            activate_ability(Ability::Boost);
        }
        accelerate(target() - position());

    }
}

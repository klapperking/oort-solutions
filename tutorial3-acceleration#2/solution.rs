use oort_api::prelude::*;
use crate::maths_rs::abs;

// General PID-Controller
pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,

    signal_max: f64,
    signal_min: f64,

    sample_time: f64,
    integrator: f64,
    differentiator: f64,
    integrator_tau: f64,
    prev_error: f64,
    prev_measurement: f64,
}

// default PIDController-instance
impl Default for PIDController {

    fn default() -> Self {
        PIDController {
            kp: 10.0,
            ki: 0.1,
            kd: 195.0,

            signal_max: max_angular_acceleration(),
            signal_min: -max_angular_acceleration(),

            sample_time: 1.0,
            integrator: 0.0,
            integrator_tau: TAU, // TODO: Arbitrary constant here, choose wiser
            differentiator: 0.0,
            prev_error: 0.0,
            prev_measurement: 0.0,
        }
    }
}

impl PIDController {

    // integrator anti-windup & clamping
    fn clamp_integrator(&mut self, p_term: f64) {

        // Integrator Anti-Wind-Up (TODO: Review limits and find better anti-windup)
        let mut max_integrator = 0.0;
        let mut min_integrator = 0.0;

        if self.signal_max > p_term {
            let max_integrator = self.signal_max - p_term;
        }
        if self.signal_min < p_term {
            let min_integrator = self.signal_min - p_term;
        }

        // Integrator Clamping with anti-wind-up limits
        if self.integrator > max_integrator {
            self.integrator = max_integrator;
        } else if self.integrator < min_integrator {
            self.integrator = min_integrator;
        }
    }

    // calculate control signal
    pub fn control(&mut self, measurement: f64, error: f64) -> f64 {

        // p-term
        let proportional = self.kp * error;

        // i-term
        self.integrator = 0.5 * self.ki * self.sample_time * (error - self.prev_error);
        self.clamp_integrator(proportional);

        // d-term (using derivate-on-measurement)
        self.differentiator = (2.0 * -self.kd * (measurement - self.prev_measurement)
                            + (2.0 * self.integrator_tau - self.sample_time) * self.differentiator)
                            / (2.0 * self.integrator_tau + self.sample_time);

        let mut control_signal = proportional + self.integrator + self.differentiator;

        // limit control signal
        if control_signal > self.signal_max {
            control_signal = self.signal_max;
        } else if control_signal < self.signal_min {
            control_signal = self.signal_min;
        }

        // update for feedback loop
        self.prev_error = error;
        self.prev_measurement = measurement;

        return control_signal;
    }
}

pub struct Ship {
    controller: Box<PIDController>,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            controller: Box::new(PIDController::default()),
        }
    }

    pub fn tick(&mut self) {

        // for now we control our heading rather than acceleration
        let current_angle = heading();
        let error = angle_diff(current_angle, (target() - position()).angle());

        // set control acceleration using control signal
        let control_signal = self.controller.control(current_angle, error);
        torque(control_signal);

        // start moving: if angle error is sufficiently small, boost as well

        /*
          TODO: Use acceleration to move influence velocity vector in controlled manner.
              Possibily control forward and lateral acceleration in the same controller?
          TODO: Control for Velocity-Angle error rather than just heading.
              In the context of aligning oneself with a target to shoot: What makes more sense?
        */

        accelerate(target() - position());

        if abs(error) < (PI / 3.0) {
            activate_ability(Ability::Boost);
        }

        // Debugging
        draw_line(target(), position(), 255);

        debug!("Heading: {}", heading());
        debug!("Target Angle {}", (target() - position()).angle());
        debug!("Angle Diff {}", error);
        debug!("Control signal {}", control_signal);
    }
}

// Tutorial: Acceleration 2
// Fly through the target circle. The target is in a random
// location given by the "target" function.
//
// You can add vectors together: vec2(a, b) + vec2(c, d) == vec2(a + c, b + d)
// And subtract them: vec2(a, b) - vec2(c, d) == vec2(a - c, b - d)
use oort_api::prelude::*;

pub struct Ship {}

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }

    pub fn tick(&mut self) {

        // activate_ability(Ability::Boost);

        accelerate(target() - position());

        // let optimal_direction = target() - position();

        // can optimize by better target radius approximation?
        let target_pos_edge_dir = (target().rotate(0.1)) - position();
        let target_neg_edge_dir = (target().rotate(-0.1)) - position();

        debug!("Cone left limit {}", target_neg_edge_dir.angle());
        debug!("Cone right limit {}", target_pos_edge_dir.angle());
        debug!("direction vetor {}", (target() - position()).angle());
        debug!("Current heading {}", heading());

        draw_line(target().rotate(0.1), position(), 255);
        draw_line(target().rotate(-0.1), position(), 255);

        // accelerate and boost towards target
        // adjust heading towards angle using side acceleration?
    }
}

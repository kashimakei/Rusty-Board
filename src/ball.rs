use crate::math::Vec2;
use crate::branch::Branch;
use pyo3::prelude::*;
use std::f32::consts::PI;

#[pyclass]
#[derive(Clone, Copy, Debug)]
pub struct Ball {
    #[pyo3(get, set)]
    pub pos: Vec2,
    #[pyo3(get, set)]
    pub old_pos: Vec2,
    #[pyo3(get, set)]
    pub acc: Vec2,
    #[pyo3(get, set)]
    pub radius: f32,
    #[pyo3(get, set)]
    pub mass: f32,
    #[pyo3(get, set)]
    pub restitution: f32,
    /// True if the ball was touching the plank on the previous frame (for edge-detection)
    pub touching_plank: bool,
    /// Cooldown in seconds before the boing sound can re-trigger for this ball
    pub boing_cooldown: f32,
}

#[pymethods]
impl Ball {
    #[new]
    pub fn new(pos: Vec2, radius: f32, density: f32, restitution: f32) -> Self {
        let mass = PI * radius * radius * density;
        Self {
            pos,
            old_pos: pos,
            acc: Vec2::zero(),
            radius,
            mass,
            restitution,
            touching_plank: false,
            boing_cooldown: 0.0,
        }
    }

    pub fn update(&mut self, dt: f32, friction: f32) {
        let vel = self.pos.sub_val(self.old_pos).mul_val(friction);
        self.old_pos = self.pos;
        self.pos = self.pos.add_val(vel).add_val(self.acc.mul_val(dt * dt));
        self.acc = Vec2::zero();
        if self.boing_cooldown > 0.0 {
            self.boing_cooldown -= dt;
        }
    }
}

// These physics functions aren't directly part of PyO3 classes, but could be exported as python module level funcs if desired.
// For now they are Rust functions that modify the pyclasses.

/// Returns the peak normal impact speed (−vel·n) across all hit segments this sub-step,
/// or `None` if the ball did not touch the plank at all.
pub fn resolve_ball_plank_collision(ball: &mut Ball, plank: &mut Branch) -> Option<f32> {
    let mut hit = false;
    let mut max_impact_speed: f32 = 0.0;

    for c in &plank.constraints {
        let node_a_pos = plank.nodes[c.node_a].pos;
        let node_b_pos = plank.nodes[c.node_b].pos;

        let ab = node_b_pos.sub_val(node_a_pos);
        let ap = ball.pos.sub_val(node_a_pos);

        let t = ap.dot_val(ab) / ab.length_sq_val();
        let t_clamped = t.clamp(0.0, 1.0);

        let nearest = node_a_pos.add_val(ab.mul_val(t_clamped));
        let dist_vec = ball.pos.sub_val(nearest);
        let dist = dist_vec.length_val();

        if dist < ball.radius {
            hit = true;
            let normal = dist_vec.normalize_val();
            let overlap = ball.radius - dist;

            ball.pos = ball.pos.add_val(normal.mul_val(overlap));

            let vel = ball.pos.sub_val(ball.old_pos);
            let vel_dot_n = vel.dot_val(normal);

            if vel_dot_n < 0.0 {
                // Track the hardest hit across all segments
                max_impact_speed = max_impact_speed.max(-vel_dot_n);

                let impulse_mag = vel_dot_n * (1.0 + ball.restitution);
                let impulse = normal.mul_val(impulse_mag);
                ball.old_pos = ball.pos.sub_val(vel.sub_val(impulse));

                // Reaction on plank nodes - scale by ball mass
                let node_impact = overlap * 0.5 * (ball.mass / 10.0).clamp(0.5, 2.0);
                if !plank.nodes[c.node_a].pinned {
                    plank.nodes[c.node_a].pos = plank.nodes[c.node_a].pos.sub_val(normal.mul_val(node_impact * (1.0 - t_clamped)));
                }
                if !plank.nodes[c.node_b].pinned {
                    plank.nodes[c.node_b].pos = plank.nodes[c.node_b].pos.sub_val(normal.mul_val(node_impact * t_clamped));
                }
            }
        }
    }

    if hit { Some(max_impact_speed) } else { None }
}

pub fn resolve_ball_ball_collision(b1: &mut Ball, b2: &mut Ball) {
    let dist_vec = b1.pos.sub_val(b2.pos);
    let dist_sq = dist_vec.length_sq_val();
    let min_dist = b1.radius + b2.radius;

    if dist_sq < min_dist * min_dist {
        let dist = dist_sq.sqrt();
        let normal = dist_vec.mul_val(1.0 / dist);
        let overlap = min_dist - dist;

        // Static separation based on mass
        let m_total = b1.mass + b2.mass;
        let w1 = b2.mass / m_total;
        let w2 = b1.mass / m_total;

        b1.pos = b1.pos.add_val(normal.mul_val(overlap * w1));
        b2.pos = b2.pos.sub_val(normal.mul_val(overlap * w2));

        // Dynamic impulse (Elastic momentum exchange)
        let v1 = b1.pos.sub_val(b1.old_pos);
        let v2 = b2.pos.sub_val(b2.old_pos);
        let relative_vel = v1.sub_val(v2);
        let vel_dot_n = relative_vel.dot_val(normal);

        if vel_dot_n < 0.0 {
            let restitution = (b1.restitution + b2.restitution) * 0.5;
            let impulse_mag = -(1.0 + restitution) * vel_dot_n / (1.0 / b1.mass + 1.0 / b2.mass);
            let impulse = normal.mul_val(impulse_mag);

            b1.old_pos = b1.pos.sub_val(v1.add_val(impulse.mul_val(1.0 / b1.mass)));
            b2.old_pos = b2.pos.sub_val(v2.sub_val(impulse.mul_val(1.0 / b2.mass)));
        }
    }
}

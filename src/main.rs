use macroquad::prelude::*;
use macroquad::ui::{hash, root_ui, widgets};
use std::f32::consts::PI;

// --- TUNABLE PARAMETERS ---
const GRAVITY: f32 = 800.0;
const FRICTION: f32 = 0.999;
const BALL_DENSITY: f32 = 0.01;      // mass = PI * r^2 * density
const BALL_FLUBBER: f32 = 0.75;
const PLANK_STIFFNESS: f32 = 0.95;
const PLANK_DAMPING: f32 = 0.999;
const SUB_STEPS: usize = 12;

// --- DYNAMIC PARAMETERS ---
const DEFAULT_PLANK_LENGTH: f32 = 400.0;
const PLANK_SEGMENT_DENSITY: f32 = 30.0; // pixels per segment
const SPAWN_X: f32 = 350.0;
const SPAWN_Y: f32 = -50.0;

// --- MATH UTILITIES ---

#[derive(Clone, Copy, Debug, PartialEq)]
struct Vec2 {
    x: f32,
    y: f32,
}

impl Vec2 {
    fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }

    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }

    fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar)
    }

    fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    fn length_sq(self) -> f32 {
        self.dot(self)
    }

    fn length(self) -> f32 {
        self.length_sq().sqrt()
    }

    fn normalize(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            self.mul(1.0 / len)
        } else {
            Self::zero()
        }
    }
}

// --- PHYSICS ENTITIES ---

#[derive(Clone, Copy, Debug)]
struct Node {
    pos: Vec2,
    old_pos: Vec2,
    acc: Vec2,
    radius: f32,
    pinned: bool,
}

impl Node {
    fn new(pos: Vec2, radius: f32, pinned: bool) -> Self {
        Self {
            pos,
            old_pos: pos,
            acc: Vec2::zero(),
            radius,
            pinned,
        }
    }

    fn update(&mut self, dt: f32, friction: f32) {
        if self.pinned { return; }

        let vel = self.pos.sub(self.old_pos).mul(friction);
        self.old_pos = self.pos;
        self.pos = self.pos.add(vel).add(self.acc.mul(dt * dt));
        self.acc = Vec2::zero();
    }
}

struct Constraint {
    node_a: usize,
    node_b: usize,
    target_length: f32,
    stiffness: f32,
}

impl Constraint {
    fn new(node_a: usize, node_b: usize, target_length: f32, stiffness: f32) -> Self {
        Self { node_a, node_b, target_length, stiffness }
    }
}

struct SoftBody {
    nodes: Vec<Node>,
    constraints: Vec<Constraint>,
}

impl SoftBody {
    fn new_truss_plank(start: Vec2, length: f32, depth: f32, segments: usize, stiffness: f32) -> Self {
        let mut nodes = Vec::new();
        let mut constraints = Vec::new();
        let segment_len = length / segments as f32;

        for i in 0..=segments {
            let pos = Vec2::new(start.x + i as f32 * segment_len, start.y);
            let pinned = i == 0 || i == 1; 
            nodes.push(Node::new(pos, 3.0, pinned));
        }
        for i in 0..=segments {
            let pos = Vec2::new(start.x + i as f32 * segment_len, start.y + depth);
            let pinned = i == 0 || i == 1;
            nodes.push(Node::new(pos, 3.0, pinned));
        }

        let bottom_offset = segments + 1;

        for i in 0..segments {
            let t1 = i;
            let t2 = i + 1;
            let b1 = i + bottom_offset;
            let b2 = i + 1 + bottom_offset;

            constraints.push(Constraint::new(t1, t2, segment_len, stiffness));
            constraints.push(Constraint::new(b1, b2, segment_len, stiffness));
            constraints.push(Constraint::new(t1, b1, depth, stiffness));
            
            let diag_len = (segment_len * segment_len + depth * depth).sqrt();
            constraints.push(Constraint::new(t1, b2, diag_len, stiffness));
            constraints.push(Constraint::new(b1, t2, diag_len, stiffness));
        }
        constraints.push(Constraint::new(segments, segments + bottom_offset, depth, stiffness));

        Self { nodes, constraints }
    }

    fn update_constraints(&mut self, iterations: usize) {
        for _ in 0..iterations {
            for c in &self.constraints {
                let delta = self.nodes[c.node_b].pos.sub(self.nodes[c.node_a].pos);
                let current_len = delta.length();
                if current_len == 0.0 { continue; }
                
                let diff = (current_len - c.target_length) / current_len;
                let offset = delta.mul(diff * 0.5 * c.stiffness);

                if !self.nodes[c.node_a].pinned {
                    self.nodes[c.node_a].pos = self.nodes[c.node_a].pos.add(offset);
                }
                if !self.nodes[c.node_b].pinned {
                    self.nodes[c.node_b].pos = self.nodes[c.node_b].pos.sub(offset);
                }
            }
        }
    }
}

struct Ball {
    pos: Vec2,
    old_pos: Vec2,
    acc: Vec2,
    radius: f32,
    mass: f32,
    restitution: f32,
}

impl Ball {
    fn new(pos: Vec2, radius: f32, density: f32, restitution: f32) -> Self {
        let mass = PI * radius * radius * density;
        Self {
            pos,
            old_pos: pos,
            acc: Vec2::zero(),
            radius,
            mass,
            restitution,
        }
    }

    fn update(&mut self, dt: f32, friction: f32) {
        let vel = self.pos.sub(self.old_pos).mul(friction);
        self.old_pos = self.pos;
        self.pos = self.pos.add(vel).add(self.acc.mul(dt * dt));
        self.acc = Vec2::zero();
    }
}

// --- COLLISION LOGIC ---

fn resolve_ball_plank_collision(ball: &mut Ball, plank: &mut SoftBody) {
    for c in &plank.constraints {
        let node_a_pos = plank.nodes[c.node_a].pos;
        let node_b_pos = plank.nodes[c.node_b].pos;

        let ab = node_b_pos.sub(node_a_pos);
        let ap = ball.pos.sub(node_a_pos);

        let t = ap.dot(ab) / ab.length_sq();
        let t_clamped = t.clamp(0.0, 1.0);

        let nearest = node_a_pos.add(ab.mul(t_clamped));
        let dist_vec = ball.pos.sub(nearest);
        let dist = dist_vec.length();

        if dist < ball.radius {
            let normal = dist_vec.normalize();
            let overlap = ball.radius - dist;

            ball.pos = ball.pos.add(normal.mul(overlap));
            
            let vel = ball.pos.sub(ball.old_pos);
            let vel_dot_n = vel.dot(normal);
            
            if vel_dot_n < 0.0 {
                let impulse_mag = vel_dot_n * (1.0 + ball.restitution);
                let impulse = normal.mul(impulse_mag);
                ball.old_pos = ball.pos.sub(vel.sub(impulse));
                
                // Reaction on plank nodes - scale by ball mass
                let node_impact = overlap * 0.5 * (ball.mass / 10.0).clamp(0.5, 2.0);
                if !plank.nodes[c.node_a].pinned {
                    plank.nodes[c.node_a].pos = plank.nodes[c.node_a].pos.sub(normal.mul(node_impact * (1.0 - t_clamped)));
                }
                if !plank.nodes[c.node_b].pinned {
                    plank.nodes[c.node_b].pos = plank.nodes[c.node_b].pos.sub(normal.mul(node_impact * t_clamped));
                }
            }
        }
    }
}

fn resolve_ball_ball_collision(b1: &mut Ball, b2: &mut Ball) {
    let dist_vec = b1.pos.sub(b2.pos);
    let dist_sq = dist_vec.length_sq();
    let min_dist = b1.radius + b2.radius;

    if dist_sq < min_dist * min_dist {
        let dist = dist_sq.sqrt();
        let normal = dist_vec.mul(1.0 / dist);
        let overlap = min_dist - dist;

        // Static separation based on mass
        let m_total = b1.mass + b2.mass;
        let w1 = b2.mass / m_total;
        let w2 = b1.mass / m_total;

        b1.pos = b1.pos.add(normal.mul(overlap * w1));
        b2.pos = b2.pos.sub(normal.mul(overlap * w2));

        // Dynamic impulse (Elastic momentum exchange)
        let v1 = b1.pos.sub(b1.old_pos);
        let v2 = b2.pos.sub(b2.old_pos);
        let relative_vel = v1.sub(v2);
        let vel_dot_n = relative_vel.dot(normal);

        if vel_dot_n < 0.0 {
            let restitution = (b1.restitution + b2.restitution) * 0.5;
            let impulse_mag = -(1.0 + restitution) * vel_dot_n / (1.0 / b1.mass + 1.0 / b2.mass);
            let impulse = normal.mul(impulse_mag);

            b1.old_pos = b1.pos.sub(v1.add(impulse.mul(1.0 / b1.mass)));
            b2.old_pos = b2.pos.sub(v2.sub(impulse.mul(1.0 / b2.mass)));
        }
    }
}

// --- MAIN LOOP ---

#[macroquad::main("Diving Board Sim (Final)")]
async fn main() {
    let mut plank_length = DEFAULT_PLANK_LENGTH;
    let mut segments = (plank_length / PLANK_SEGMENT_DENSITY) as usize;
    let mut plank = SoftBody::new_truss_plank(Vec2::new(100.0, 300.0), plank_length, 25.0, segments, PLANK_STIFFNESS);
    let mut balls: Vec<Ball> = Vec::new();
    
    let mut gravity_val = GRAVITY;
    let mut gloop_val = 1.0 - FRICTION;
    let mut density_val = BALL_DENSITY;
    let mut flubber_val = BALL_FLUBBER;
    let mut stiffness_val = PLANK_STIFFNESS;
    let mut spawn_x_val = SPAWN_X;
    let mut ball_size_val = 15.0;
    
    let dt = 1.0 / 60.0;
    let sub_dt = dt / SUB_STEPS as f32;

    let mut spawn_timer = 0.0;
    let mut balls_per_second = 1.0;
    let mut random_size = true;

    loop {
        clear_background(Color::new(0.05, 0.05, 0.1, 1.0));

        // --- UI ---
        widgets::Window::new(hash!(), vec2(20.0, 60.0), vec2(320.0, 450.0))
            .label("Simulation Controls")
            .ui(&mut *root_ui(), |ui| {
                ui.slider(hash!(), "Balls/Sec", 0.2..2.5, &mut balls_per_second);
                
                let board_start = 100.0;
                let board_end = board_start + plank_length;
                ui.slider(hash!(), "Spawn X", board_start..board_end, &mut spawn_x_val);
                spawn_x_val = spawn_x_val.clamp(board_start, board_end);
                
                let old_len = plank_length;
                ui.slider(hash!(), "Board Length", 100.0..700.0, &mut plank_length);
                if (plank_length - old_len).abs() > 1.0 {
                    segments = (plank_length / PLANK_SEGMENT_DENSITY).max(2.0) as usize;
                    plank = SoftBody::new_truss_plank(Vec2::new(100.0, 300.0), plank_length, 25.0, segments, stiffness_val);
                }

                ui.slider(hash!(), "Gravity", 0.0..2000.0, &mut gravity_val);
                ui.slider(hash!(), "Gloopification", 0.0..0.05, &mut gloop_val);
                ui.slider(hash!(), "Density", 0.001..0.05, &mut density_val);
                ui.slider(hash!(), "Flubberization", 0.1..2.0, &mut flubber_val);
                
                let old_stiffness = stiffness_val;
                ui.slider(hash!(), "Stiffness", 0.1..1.0, &mut stiffness_val);
                if (stiffness_val - old_stiffness).abs() > 0.001 {
                    for c in &mut plank.constraints {
                        c.stiffness = stiffness_val;
                    }
                }

                ui.slider(hash!(), "Ball Size", 5.0..40.0, &mut ball_size_val);
                ui.checkbox(hash!(), "Random Size", &mut random_size);
                
                if ui.button(None, "Clear Balls") {
                    balls.clear();
                }
                if ui.button(None, "Reset Everything") {
                    plank_length = DEFAULT_PLANK_LENGTH;
                    stiffness_val = PLANK_STIFFNESS;
                    segments = (plank_length / PLANK_SEGMENT_DENSITY) as usize;
                    plank = SoftBody::new_truss_plank(Vec2::new(board_start, 300.0), plank_length, 25.0, segments, stiffness_val);
                    balls_per_second = 1.0;
                    spawn_x_val = SPAWN_X;
                    ball_size_val = 15.0;
                    gloop_val = 1.0 - FRICTION;
                    flubber_val = BALL_FLUBBER;
                    balls.clear();
                }
            });

        // --- SPAWN LOGIC ---
        spawn_timer += get_frame_time();
        let interval = 1.0 / balls_per_second;
        while spawn_timer >= interval {
            spawn_timer -= interval;
            let radius = if random_size { rand::gen_range(10.0, 30.0) } else { ball_size_val };
            balls.push(Ball::new(
                Vec2::new(spawn_x_val + rand::gen_range(-20.0, 20.0), SPAWN_Y),
                radius,
                density_val,
                flubber_val
            ));
        }

        if is_key_pressed(KeyCode::Space) {
             balls.push(Ball::new(Vec2::new(spawn_x_val, 100.0), ball_size_val, density_val, flubber_val));
        }
        if is_key_pressed(KeyCode::R) {
            segments = (plank_length / PLANK_SEGMENT_DENSITY) as usize;
            plank = SoftBody::new_truss_plank(Vec2::new(100.0, 300.0), plank_length, 25.0, segments, stiffness_val);
            balls.clear();
        }

        // --- UPDATE ---
        let gravity_vec = Vec2::new(0.0, gravity_val);
        for _ in 0..SUB_STEPS {
            // Update Plank
            for node in &mut plank.nodes {
                node.acc = node.acc.add(gravity_vec);
                node.update(sub_dt, PLANK_DAMPING);
            }
            plank.update_constraints(4);

            // Update Balls
            for i in 0..balls.len() {
                // Ball-Plank
                balls[i].acc = balls[i].acc.add(gravity_vec);
                balls[i].update(sub_dt, 1.0 - gloop_val);
                resolve_ball_plank_collision(&mut balls[i], &mut plank);
            }

            // Ball-Ball
            for i in 0..balls.len() {
                for j in i + 1..balls.len() {
                    let (b1, b2) = {
                        if i < j {
                            let (left, right) = balls.split_at_mut(j);
                            (&mut left[i], &mut right[0])
                        } else {
                            continue;
                        }
                    };
                    resolve_ball_ball_collision(b1, b2);
                }
            }
        }

        balls.retain(|b| b.pos.y < screen_height() + 100.0 && b.pos.x > -100.0 && b.pos.x < screen_width() + 100.0);

        // --- DRAW ---
        for c in &plank.constraints {
            let n1 = &plank.nodes[c.node_a];
            let n2 = &plank.nodes[c.node_b];
            let color = if n1.pinned && n2.pinned { RED } else { Color::new(0.2, 0.4, 0.8, 0.5) };
            draw_line(n1.pos.x, n1.pos.y, n2.pos.x, n2.pos.y, 1.5, color);
        }

        for i in 0..plank.nodes.len() / 2 - 1 {
             let t1 = &plank.nodes[i];
             let t2 = &plank.nodes[i+1];
             let b1 = &plank.nodes[i + plank.nodes.len()/2];
             let b2 = &plank.nodes[i + 1 + plank.nodes.len()/2];
             draw_line(t1.pos.x, t1.pos.y, t2.pos.x, t2.pos.y, 4.0, SKYBLUE);
             draw_line(b1.pos.x, b1.pos.y, b2.pos.x, b2.pos.y, 4.0, SKYBLUE);
        }

        for ball in &balls {
            // Color based on mass/radius
            let hue = (ball.radius / 30.0) * 0.6; // Scale radius to color range
            let color = Color::from_vec(vec4(hue, 0.8, 1.0 - hue, 1.0));
            draw_circle(ball.pos.x, ball.pos.y, ball.radius, color);
            draw_circle_lines(ball.pos.x, ball.pos.y, ball.radius, 2.0, WHITE);
        }

        draw_text("RUSTY BOARD: MULTI-BODY SYMPHONY", 20.0, 30.0, 30.0, WHITE);
        draw_text(&format!("Bodies: {} | FPS: {}", balls.len() + plank.nodes.len(), get_fps()), 20.0, 60.0, 20.0, LIGHTGRAY);

        next_frame().await
    }
}

// --- TESTS ---

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec2_math() {
        let v1 = Vec2::new(3.0, 4.0);
        assert_eq!(v1.length(), 5.0);
        let v2 = Vec2::new(1.0, 1.0);
        assert_eq!(v1.dot(v2), 7.0);
    }

    #[test]
    fn test_mass_scaling() {
        let b = Ball::new(Vec2::zero(), 10.0, 0.01, 0.5);
        // mass = PI * 10^2 * 0.01 = 3.14159
        assert!((b.mass - PI).abs() < 0.01);
    }
}

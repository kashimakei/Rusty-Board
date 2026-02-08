use macroquad::prelude::*;

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

struct SoftBody {
    nodes: Vec<Node>,
    constraints: Vec<Constraint>,
}

impl SoftBody {
    fn new_plank(start: Vec2, length: f32, segments: usize, stiffness: f32) -> Self {
        let mut nodes = Vec::new();
        let mut constraints = Vec::new();
        let segment_len = length / segments as f32;

        for i in 0..=segments {
            let pos = Vec2::new(start.x + i as f32 * segment_len, start.y);
            let pinned = i == 0 || i == 1; // Pin first two for "cantilever" effect
            nodes.push(Node::new(pos, 5.0, pinned));

            if i > 0 {
                constraints.push(Constraint {
                    node_a: i - 1,
                    node_b: i,
                    target_length: segment_len,
                    stiffness,
                });
            }
        }

        Self { nodes, constraints }
    }

    fn update_constraints(&mut self) {
        // Dot products are used here implicitly via vector subtraction and normalization
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

struct Ball {
    pos: Vec2,
    old_pos: Vec2,
    acc: Vec2,
    radius: f32,
    restitution: f32,
}

impl Ball {
    fn new(pos: Vec2, radius: f32, restitution: f32) -> Self {
        Self {
            pos,
            old_pos: pos,
            acc: Vec2::zero(),
            radius,
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

// --- COLLISION LOGIC (EXTENSIVE DOT PRODUCTS) ---

fn resolve_collision(ball: &mut Ball, plank: &mut SoftBody) {
    for i in 0..plank.constraints.len() {
        let node_a_pos = plank.nodes[plank.constraints[i].node_a].pos;
        let node_b_pos = plank.nodes[plank.constraints[i].node_b].pos;

        // Line segment vector
        let ab = node_b_pos.sub(node_a_pos);
        // Vector from A to ball
        let ap = ball.pos.sub(node_a_pos);

        // Project ap onto ab using DOT PRODUCT
        // t is the normalized distance along the segment [0, 1]
        let t = ap.dot(ab) / ab.length_sq();
        let t_clamped = t.clamp(0.0, 1.0);

        // Nearest point on segment
        let nearest = node_a_pos.add(ab.mul(t_clamped));
        let dist_vec = ball.pos.sub(nearest);
        let dist = dist_vec.length();

        if dist < ball.radius {
            // Collision normal
            let normal = dist_vec.normalize();
            let overlap = ball.radius - dist;

            // Simple restitution: Push ball out along normal
            ball.pos = ball.pos.add(normal.mul(overlap));
            
            // Reflect velocity using DOT PRODUCT: v_new = v - (1 + r) * (v . n) * n
            let vel = ball.pos.sub(ball.old_pos);
            let vel_dot_n = vel.dot(normal);
            
            if vel_dot_n < 0.0 {
                let impulse = normal.mul(vel_dot_n * (1.0 + ball.restitution));
                ball.old_pos = ball.pos.sub(vel.sub(impulse));
            }

            // Also push the plank nodes a bit
            let node_impact = overlap * 0.5;
            if !plank.nodes[plank.constraints[i].node_a].pinned {
                plank.nodes[plank.constraints[i].node_a].pos = plank.nodes[plank.constraints[i].node_a].pos.sub(normal.mul(node_impact * (1.0 - t_clamped)));
            }
            if !plank.nodes[plank.constraints[i].node_b].pinned {
                plank.nodes[plank.constraints[i].node_b].pos = plank.nodes[plank.constraints[i].node_b].pos.sub(normal.mul(node_impact * t_clamped));
            }
        }
    }
}

// --- MAIN LOOP ---

#[macroquad::main("Diving Board Sim")]
async fn main() {
    let mut plank = SoftBody::new_plank(Vec2::new(100.0, 300.0), 300.0, 15, 0.9);
    let mut ball = Ball::new(Vec2::new(350.0, 100.0), 15.0, 0.7);
    
    let gravity = Vec2::new(0.0, 500.0);
    let friction = 0.999;
    let dt = 1.0 / 60.0;
    let sub_steps = 8;
    let sub_dt = dt / sub_steps as f32;

    loop {
        clear_background(DARKGRAY);

        // --- UPDATE ---
        for _ in 0..sub_steps {
            // Gravity
            ball.acc = ball.acc.add(gravity);
            for node in &mut plank.nodes {
                node.acc = node.acc.add(gravity);
            }

            // Integration
            ball.update(sub_dt, friction);
            for node in &mut plank.nodes {
                node.update(sub_dt, friction);
            }

            // Constraints & Collisions
            for _ in 0..5 {
                plank.update_constraints();
            }
            resolve_collision(&mut ball, &mut plank);
        }

        // --- DRAW ---
        // Draw Plank
        for i in 0..plank.constraints.len() {
            let n1 = plank.nodes[plank.constraints[i].node_a];
            let n2 = plank.nodes[plank.constraints[i].node_b];
            draw_line(n1.pos.x, n1.pos.y, n2.pos.x, n2.pos.y, 4.0, WHITE);
        }
        for node in &plank.nodes {
            draw_circle(node.pos.x, node.pos.y, node.radius, if node.pinned { RED } else { BLUE });
        }

        // Draw Ball
        draw_circle(ball.pos.x, ball.pos.y, ball.radius, YELLOW);

        draw_text("Space to Drop Ball | R to Reset", 20.0, 20.0, 20.0, WHITE);

        if is_key_pressed(KeyCode::Space) {
            ball.pos = Vec2::new(350.0, 100.0);
            ball.old_pos = Vec2::new(350.0, 100.0);
        }
        if is_key_pressed(KeyCode::R) {
            plank = SoftBody::new_plank(Vec2::new(100.0, 300.0), 300.0, 15, 0.9);
            ball = Ball::new(Vec2::new(350.0, 100.0), 15.0, 0.7);
        }

        next_frame().await
    }
}

use crate::math::Vec2;
use pyo3::prelude::*;
use std::f32::consts::PI;

#[pyclass]
#[derive(Clone, Copy, Debug)]
pub struct Node {
    #[pyo3(get, set)]
    pub pos: Vec2,
    #[pyo3(get, set)]
    pub old_pos: Vec2,
    #[pyo3(get, set)]
    pub acc: Vec2,
    #[pyo3(get, set)]
    pub radius: f32,
    #[pyo3(get, set)]
    pub pinned: bool,
}

#[pymethods]
impl Node {
    #[new]
    pub fn new(pos: Vec2, radius: f32, pinned: bool) -> Self {
        Self {
            pos,
            old_pos: pos,
            acc: Vec2::zero(),
            radius,
            pinned,
        }
    }

    pub fn update(&mut self, dt: f32, friction: f32) {
        if self.pinned { return; }
        let vel = self.pos.sub_val(self.old_pos).mul_val(friction);
        self.old_pos = self.pos;
        self.pos = self.pos.add_val(vel).add_val(self.acc.mul_val(dt * dt));
        self.acc = Vec2::zero();
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
pub struct Constraint {
    #[pyo3(get, set)]
    pub node_a: usize,
    #[pyo3(get, set)]
    pub node_b: usize,
    #[pyo3(get, set)]
    pub target_length: f32,
    #[pyo3(get, set)]
    pub stiffness: f32,
}

#[pymethods]
impl Constraint {
    #[new]
    pub fn new(node_a: usize, node_b: usize, target_length: f32, stiffness: f32) -> Self {
        Self { node_a, node_b, target_length, stiffness }
    }
}

#[pyclass]
pub struct Branch {
    #[pyo3(get, set)]
    pub nodes: Vec<Node>,
    #[pyo3(get, set)]
    pub constraints: Vec<Constraint>,
}

#[pymethods]
impl Branch {
    #[new]
    #[pyo3(signature = (start, length, base_depth, segments, stiffness, gnarl=1.0))]
    pub fn new_tree_branch(start: Vec2, length: f32, base_depth: f32, segments: usize, stiffness: f32, gnarl: f32) -> Self {
        let mut nodes = Vec::new();
        let mut constraints = Vec::new();
        let segment_len = length / segments as f32;

        for i in 0..=segments {
            let t = i as f32 / segments as f32;
            let bend = (t * PI * 1.5 * gnarl).sin() * (30.0 * gnarl) + (t * PI * 4.0 * gnarl).sin() * (10.0 * gnarl) + (t * t * 40.0 * gnarl);
            
            let pos_top = Vec2::new(start.x + i as f32 * segment_len, start.y + bend);
            let pinned = i == 0 || i == 1; 
            nodes.push(Node::new(pos_top, 3.0, pinned));
        }

        for i in 0..=segments {
            let t = i as f32 / segments as f32;
            let current_depth = base_depth * (1.0 - 0.75 * t).max(0.1);
            let bend = (t * PI * 1.5 * gnarl).sin() * (30.0 * gnarl) + (t * PI * 4.0 * gnarl).sin() * (10.0 * gnarl) + (t * t * 40.0 * gnarl);
            
            let pos_bottom = Vec2::new(start.x + i as f32 * segment_len, start.y + bend + current_depth);
            let pinned = i == 0 || i == 1; 
            nodes.push(Node::new(pos_bottom, 3.0, pinned));
        }

        let bottom_offset = segments + 1;

        let get_dist = |nodes: &Vec<Node>, a: usize, b: usize| -> f32 {
            nodes[a].pos.sub_val(nodes[b].pos).length_val()
        };

        for i in 0..segments {
            let t1 = i;
            let t2 = i + 1;
            let b1 = i + bottom_offset;
            let b2 = i + 1 + bottom_offset;

            constraints.push(Constraint::new(t1, t2, get_dist(&nodes, t1, t2), stiffness));
            constraints.push(Constraint::new(b1, b2, get_dist(&nodes, b1, b2), stiffness));
            constraints.push(Constraint::new(t1, b1, get_dist(&nodes, t1, b1), stiffness));
            
            constraints.push(Constraint::new(t1, b2, get_dist(&nodes, t1, b2), stiffness));
            constraints.push(Constraint::new(b1, t2, get_dist(&nodes, b1, t2), stiffness));
        }
        constraints.push(Constraint::new(segments, segments + bottom_offset, get_dist(&nodes, segments, segments + bottom_offset), stiffness));

        Self { nodes, constraints }
    }

    pub fn update_constraints(&mut self, iterations: usize) {
        for _ in 0..iterations {
            for c in &self.constraints {
                let delta = self.nodes[c.node_b].pos.sub_val(self.nodes[c.node_a].pos);
                let current_len = delta.length_val();
                if current_len == 0.0 { continue; }
                
                let diff = (current_len - c.target_length) / current_len;
                let offset = delta.mul_val(diff * 0.5 * c.stiffness);

                if !self.nodes[c.node_a].pinned {
                    self.nodes[c.node_a].pos = self.nodes[c.node_a].pos.add_val(offset);
                }
                if !self.nodes[c.node_b].pinned {
                    self.nodes[c.node_b].pos = self.nodes[c.node_b].pos.sub_val(offset);
                }
            }
        }
    }

    pub fn update_physics(&mut self, sub_steps: usize, dt: f32, gravity: f32, global_damping: f32) {
        let sub_dt = dt / sub_steps as f32;
        let gravity_vec = Vec2::new(0.0, gravity);

        for _ in 0..sub_steps {
            for node in &mut self.nodes {
                node.acc = node.acc.add_val(gravity_vec);
                node.update(sub_dt, global_damping);
            }
            self.update_constraints(4);
        }
    }
}

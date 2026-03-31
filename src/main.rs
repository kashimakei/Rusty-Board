use macroquad::prelude::*;
use macroquad::ui::{hash, root_ui, widgets};
use macroquad::audio::{load_sound, play_sound_once};

// --- TUNABLE PARAMETERS ---
const GRAVITY: f32 = 800.0;
const FRICTION: f32 = 0.999;
const BALL_DENSITY: f32 = 0.05;     // mass = PI * r^2 * density
const BALL_FLUBBER: f32 = 0.85;
const PLANK_STIFFNESS: f32 = 0.95;
const PLANK_DAMPING: f32 = 0.999;
const SUB_STEPS: usize = 12;

// --- DYNAMIC PARAMETERS ---
const DEFAULT_PLANK_LENGTH: f32 = 700.0;
const PLANK_SEGMENT_DENSITY: f32 = 30.0; // pixels per segment
const SPAWN_X: f32 = 350.0;
const SPAWN_Y: f32 = -50.0;

mod math;
mod branch;
mod ball;

use math::Vec2;
use branch::Branch as SoftBody;
use ball::{Ball, resolve_ball_plank_collision, resolve_ball_ball_collision};

// --- MAIN LOOP ---

#[macroquad::main("Tree Branch Sim (Final)")]
async fn main() {
    request_new_screen_size(1280.0, 720.0);
    
    let pop_sound = load_sound("src/assets/pop.wav").await.unwrap_or_else(|_| panic!("Failed to load pop.wav"));
    let boing_sound = load_sound("src/assets/Boing.wav").await.unwrap_or_else(|_| panic!("Failed to load Boing.wav"));
    let wobble_sound = load_sound("src/assets/wobble.wav").await.unwrap_or_else(|_| panic!("Failed to load wobble.wav"));
    let boingee_sound = load_sound("src/assets/Boingee.wav").await.unwrap_or_else(|_| panic!("Failed to load Boingee.wav"));
    let mut plank_length = DEFAULT_PLANK_LENGTH;
    let mut segments = (plank_length / PLANK_SEGMENT_DENSITY) as usize;
    let mut plank = SoftBody::new_tree_branch(Vec2::new(100.0, 300.0), plank_length, 45.0, segments, PLANK_STIFFNESS, 1.0);
    let mut balls: Vec<Ball> = Vec::new();
    
    let mut gravity_val = GRAVITY;
    let mut gloop_val = 1.0 - FRICTION;
    let mut density_val = BALL_DENSITY;
    let mut flubber_val = BALL_FLUBBER;
    let mut stiffness_val = PLANK_STIFFNESS;
    let mut spawn_x_val = SPAWN_X;
    let mut ball_size_val = 15.0;
    let mut wobble_threshold_val = 800.0;
    
    let dt = 1.0 / 60.0;
    let sub_dt = dt / SUB_STEPS as f32;

    let mut spawn_timer = 0.0;
    let mut balls_per_second = 1.5;
    let mut random_size = true;
    let mut wobble_playing = false;
    let mut wobble_timer = 0.0;
    let mut next_spawn_target = 1.0;

    loop {
        clear_background(Color::new(0.05, 0.05, 0.1, 1.0));

        // --- UI ---
        widgets::Window::new(hash!(), vec2(screen_width(), 100.0), vec2(400.0, 450.0))
            .label("Simulation Controls")
            .ui(&mut *root_ui(), |ui| {
                ui.slider(hash!(), "Balls/Sec", 0.2..3.5, &mut balls_per_second);
                
                let board_start = 200.0;
                let board_end = board_start + plank_length;
                ui.slider(hash!(), "Spawn X", board_start..board_end, &mut spawn_x_val);
                spawn_x_val = spawn_x_val.clamp(board_start, board_end);
                
                let old_len = plank_length;
                ui.slider(hash!(), "Board Length", 100.0..1500.0, &mut plank_length);
                if (plank_length - old_len).abs() > 1.0 {
                    segments = (plank_length / PLANK_SEGMENT_DENSITY).max(2.0) as usize;
                    plank = SoftBody::new_tree_branch(Vec2::new(100.0, 300.0), plank_length, 45.0, segments, stiffness_val, 1.0);
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

                ui.slider(hash!(), "Ball Size", 5.0..50.0, &mut ball_size_val);
                ui.checkbox(hash!(), "Random Size", &mut random_size);
                ui.slider(hash!(), "Wobble Threshold", 100.0..5000.0, &mut wobble_threshold_val);
                
                if ui.button(None, "Clear Balls") {
                    balls.clear();
                }
                if ui.button(None, "Reset Everything") {
                    plank_length = DEFAULT_PLANK_LENGTH;
                    stiffness_val = PLANK_STIFFNESS;
                    segments = (plank_length / PLANK_SEGMENT_DENSITY) as usize;
                    plank = SoftBody::new_tree_branch(Vec2::new(board_start, 300.0), plank_length, 45.0, segments, stiffness_val, 1.0);
                    balls_per_second = 1.0;
                    spawn_x_val = SPAWN_X;
                    ball_size_val = 15.0;
                    gloop_val = 1.0 - FRICTION;
                    flubber_val = BALL_FLUBBER;
                    balls.clear();
                }
            });

        // --- SPAWN LOGIC ---
        spawn_timer += get_frame_time() * balls_per_second;
        while spawn_timer >= next_spawn_target {
            spawn_timer -= next_spawn_target;
            
            // Exponential distribution for realistic "popcorn" burst spacing (mean = 1.0)
            let u = rand::gen_range(0.01f32, 1.0f32);
            next_spawn_target = -u.ln();

            let radius = if random_size { rand::gen_range(10.0, 30.0) } else { ball_size_val };
            play_sound_once(&pop_sound);
            balls.push(Ball::new(
                Vec2::new(spawn_x_val + rand::gen_range(-20.0, 20.0), SPAWN_Y),
                radius,
                density_val,
                flubber_val
            ));
        }

        if is_key_pressed(KeyCode::Space) {
             play_sound_once(&pop_sound);
             balls.push(Ball::new(Vec2::new(spawn_x_val, 100.0), ball_size_val, density_val, flubber_val));
        }
        if is_key_pressed(KeyCode::R) {
            segments = (plank_length / PLANK_SEGMENT_DENSITY) as usize;
            plank = SoftBody::new_tree_branch(Vec2::new(100.0, 300.0), plank_length, 45.0, segments, stiffness_val, 1.0);
            balls.clear();
        }

        // --- UPDATE ---
        let gravity_vec = Vec2::new(0.0, gravity_val);
        let mut ball_impacts: Vec<Option<f32>> = vec![None; balls.len()];
        for _ in 0..SUB_STEPS {
            // Update Plank
            for node in &mut plank.nodes {
                node.acc = node.acc.add_val(gravity_vec);
                node.update(sub_dt, PLANK_DAMPING);
            }
            plank.update_constraints(4);

            // Update Balls
            for i in 0..balls.len() {
                // Ball-Plank
                balls[i].acc = balls[i].acc.add_val(gravity_vec);
                balls[i].update(sub_dt, 1.0 - gloop_val);
                let impact = resolve_ball_plank_collision(&mut balls[i], &mut plank);
                if let Some(speed) = impact {
                    let current = ball_impacts[i].unwrap_or(0.0);
                    ball_impacts[i] = Some(current.max(speed));
                }
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

        // --- BOING TRIGGER (once per frame, on first-contact transition, with cooldown) ---
        const BOING_COOLDOWN_SECS: f32 = 0.25;
        for i in 0..balls.len() {
            let impact = ball_impacts[i];
            let hit = impact.is_some();
            if hit && !balls[i].touching_plank && balls[i].boing_cooldown <= 0.0 {
                let speed = impact.unwrap();
                if speed > 1.25 {
                    play_sound_once(&boingee_sound);
                    balls[i].boing_cooldown = BOING_COOLDOWN_SECS;
                } else if speed > 0.9 {
                    play_sound_once(&boing_sound);
                    balls[i].boing_cooldown = BOING_COOLDOWN_SECS;
                }
            }
            balls[i].touching_plank = hit;
        }

        balls.retain(|b| b.pos.y < screen_height() + 100.0 && b.pos.x > -100.0 && b.pos.x < screen_width() + 100.0);

        // --- WOBBLE LOGIC ---
        let top_nodes_len = plank.nodes.len() / 2;
        let mut mean_radius = 100000.0;
        let mut sign_flips = 0;
        
        if top_nodes_len >= 3 {
            let mut total_r = 0.0;
            let mut last_sign = 0;
            
            for i in 1..(top_nodes_len - 1) {
                let pos_a = plank.nodes[i-1].pos;
                let pos_b = plank.nodes[i].pos;
                let pos_c = plank.nodes[i+1].pos;

                let ab = pos_b.sub_val(pos_a);
                let bc = pos_c.sub_val(pos_b);
                
                // 2D cross product Z component to find bend direction
                let cross_z = ab.x * bc.y - ab.y * bc.x;
                
                // Deadzone to ignore noisy micro-jitters (straight lines)
                let current_sign = if cross_z > 5.0 { 1 } else if cross_z < -5.0 { -1 } else { 0 };
                
                if current_sign != 0 {
                    if last_sign != 0 && current_sign != last_sign {
                        sign_flips += 1;
                    }
                    last_sign = current_sign;
                }

                let a = ab.length_val();
                let b = bc.length_val();
                let c = pos_a.sub_val(pos_c).length_val();

                let s = (a + b + c) / 2.0;
                let area_sq = s * (s - a) * (s - b) * (s - c);
                let r = if area_sq > 0.1 {
                    (a * b * c) / (4.0 * area_sq.sqrt())
                } else {
                    100000.0
                };
                total_r += r;
            }
            mean_radius = total_r / (top_nodes_len - 2) as f32;
        }

        if mean_radius < wobble_threshold_val && sign_flips >= 1 {
            if !wobble_playing {
                play_sound_once(&wobble_sound);
                wobble_timer = 2.332; // Duration of wobble.wav
                wobble_playing = true;
            }
        }

        if wobble_playing {
            wobble_timer -= get_frame_time();
            if wobble_timer <= 0.0 {
                wobble_playing = false;
            }
        }

        // --- DRAW ---
        for c in &plank.constraints {
            let n1 = &plank.nodes[c.node_a];
            let n2 = &plank.nodes[c.node_b];
            let color = if n1.pinned && n2.pinned { RED } else { Color::new(0.3, 0.15, 0.05, 0.5) };
            draw_line(n1.pos.x, n1.pos.y, n2.pos.x, n2.pos.y, 1.5, color);
        }

        for i in 0..plank.nodes.len() / 2 - 1 {
             let t1 = &plank.nodes[i];
             let t2 = &plank.nodes[i+1];
             let b1 = &plank.nodes[i + plank.nodes.len()/2];
             let b2 = &plank.nodes[i + 1 + plank.nodes.len()/2];
             draw_line(t1.pos.x, t1.pos.y, t2.pos.x, t2.pos.y, 4.0, Color::new(0.4, 0.2, 0.0, 1.0));
             draw_line(b1.pos.x, b1.pos.y, b2.pos.x, b2.pos.y, 4.0, Color::new(0.4, 0.2, 0.0, 1.0));
        }

        for ball in &balls {
            // Color based on mass/radius
            let hue = (ball.radius / 30.0) * 0.6; // Scale radius to color range
            let color = Color::from_vec(vec4(hue, 0.8, 1.0 - hue, 1.0));
            draw_circle(ball.pos.x, ball.pos.y, ball.radius, color);
            draw_circle_lines(ball.pos.x, ball.pos.y, ball.radius, 2.0, WHITE);
        }

        draw_text("RUSTY BRANCH: MULTI-BODY SYMPHONY", 20.0, 30.0, 30.0, WHITE);
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
        assert_eq!(v1.length_val(), 5.0);
        let v2 = Vec2::new(1.0, 1.0);
        assert_eq!(v1.dot_val(v2), 7.0);
    }

    #[test]
    fn test_mass_scaling() {
        let b = Ball::new(Vec2::zero(), 10.0, 0.01, 0.5);
        assert!((b.mass - std::f32::consts::PI).abs() < 0.01);
    }
}

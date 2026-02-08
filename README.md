# Rusty-Board: Diving Board Simulation

A lightweight 2D physics simulation of a ball bouncing on a cantilevered soft-body plank (diving board), built with Rust and [macroquad](https://macroquad.rs/).

## Physics Features
- **Soft Body Simulation**: The plank is a mass-spring system using Verlet integration for stability.
- **Cantilevered Logic**: One end of the plank is pinned to simulate a fixed attachment point.
- **Math-Heavy**: Extensive use of **dot products** for:
  - Vector projection (collision detection).
  - Normal calculation.
  - Impulse-based velocity reflection.
- **Parameters**: Implementation includes mass, gravity, friction, restitution (bounciness), and dampening.

## Controls
- **[Space]**: Respawn the ball at the top.
- **[R]**: Reset the simulation (plank and ball).

## Quick Start (One-Time Setup)

To get started immediately on Linux or macOS, run the following command in your terminal:
```bash
./setup.sh
```
This script will:
1. Install necessary system dependencies (X11, OpenGL headers, etc.).
2. Install the Rust toolchain if it's not already present.
3. Build the project.

### Windows Setup
1. Install [Rust](https://rustup.rs/).
2. No extra dependencies are usually required for Windows (macroquad handles it via Miniquad).
3. Run `cargo run` in your terminal.

## Running the App
Once set up, simply run:
```bash
cargo run
```

## License
MIT

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

## Getting Started

### Prerequisites
- [Rust](https://rustup.rs/) installed.
- System dependencies for macroquad (Linux):
  ```bash
  sudo apt install pkg-config libx11-dev libxi-dev libgl1-mesa-dev libasound2-dev
  ```

### Running the App
```bash
cargo run
```

## License
MIT

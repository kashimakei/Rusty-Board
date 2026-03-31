pub mod math;
pub mod branch;
pub mod ball;

use pyo3::prelude::*;
use math::Vec2;
use branch::{Branch, Node, Constraint};
use ball::Ball;

#[pymodule]
fn rusty_board(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add math submodule
    let math_mod = pyo3::types::PyModule::new_bound(py, "math")?;
    math_mod.add_class::<Vec2>()?;
    m.add_submodule(&math_mod)?;

    // Add branch submodule
    let branch_mod = pyo3::types::PyModule::new_bound(py, "branch")?;
    branch_mod.add_class::<Node>()?;
    branch_mod.add_class::<Constraint>()?;
    branch_mod.add_class::<Branch>()?;
    m.add_submodule(&branch_mod)?;

    // Add ball submodule
    let ball_mod = pyo3::types::PyModule::new_bound(py, "ball")?;
    ball_mod.add_class::<Ball>()?;
    m.add_submodule(&ball_mod)?;

    // Optionally expose top-level convenience classes that could also just be imported from submodules.
    // E.g., from rusty_board import Vec2
    m.add_class::<Vec2>()?;
    m.add_class::<Node>()?;
    m.add_class::<Constraint>()?;
    m.add_class::<Branch>()?;
    m.add_class::<Ball>()?;

    Ok(())
}

"""
Quick smoke test for the rusty_board Python module.
Run after: maturin develop --release
"""
import rusty_board

# --- Test Vec2 ---
v = rusty_board.Vec2(3.0, 4.0)
assert abs(v.length() - 5.0) < 0.001, f"Expected 5.0, got {v.length()}"
print(f"Vec2 length test passed: {v.length():.4f}")

# --- Test Branch generation ---
branch = rusty_board.Branch(
    start=rusty_board.Vec2(100.0, 300.0),
    length=600.0,
    base_depth=40.0,
    segments=20,
    stiffness=0.95,
    gnarl=1.0,
)

total_nodes = len(branch.nodes)
total_constraints = len(branch.constraints)
print(f"Branch created: {total_nodes} nodes, {total_constraints} constraints")

# Confirm top-row pinned nodes are at indices 0 and 1
assert branch.nodes[0].pinned, "Node 0 should be pinned"
assert branch.nodes[1].pinned, "Node 1 should be pinned"
assert not branch.nodes[5].pinned, "Node 5 should NOT be pinned"

# --- Test physics update ---
branch.update_physics(sub_steps=12, dt=1/60, gravity=800.0, global_damping=0.999)
pos_after = branch.nodes[10].pos
print(f"Node 10 pos after physics tick: ({pos_after.x:.2f}, {pos_after.y:.2f})")

# --- Test gnarl variation ---
gnarly = rusty_board.Branch(
    start=rusty_board.Vec2(0.0, 0.0),
    length=300.0,
    base_depth=20.0,
    segments=10,
    stiffness=0.9,
    gnarl=2.5,
)
print(f"High-gnarl branch: {len(gnarly.nodes)} nodes")

# --- Test Ball ---
ball = rusty_board.Ball(
    pos=rusty_board.Vec2(200.0, 50.0),
    radius=15.0,
    density=0.05,
    restitution=0.85,
)
print(f"Ball mass: {ball.mass:.4f}, radius: {ball.radius}")
ball.update(dt=1/60, friction=0.999)
print(f"Ball pos after update: ({ball.pos.x:.2f}, {ball.pos.y:.2f})")

print("\nAll tests passed!")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_drawer")

    # Outer Tray (Base Cabinet/Housing)
    outer_tray = model.part("outer_tray")
    # Bottom: Z from 0.0 to 0.01
    outer_tray.visual(Box((0.40, 0.50, 0.01)), origin=Origin((0.0, 0.0, 0.005)), name="bottom", color=(0.3, 0.3, 0.3))
    # Side walls: Z from 0.0 to 0.20
    outer_tray.visual(Box((0.01, 0.50, 0.20)), origin=Origin((-0.195, 0.0, 0.10)), name="left_wall", color=(0.3, 0.3, 0.3))
    outer_tray.visual(Box((0.01, 0.50, 0.20)), origin=Origin((0.195, 0.0, 0.10)), name="right_wall", color=(0.3, 0.3, 0.3))
    # Back wall
    outer_tray.visual(Box((0.38, 0.01, 0.20)), origin=Origin((0.0, -0.245, 0.10)), name="back_wall", color=(0.3, 0.3, 0.3))

    # Middle Tray (Intermediate Stage)
    middle_tray = model.part("middle_tray")
    # Bottom: Z from 0.011 to 0.021
    middle_tray.visual(Box((0.378, 0.48, 0.01)), origin=Origin((0.0, 0.005, 0.016)), name="bottom", color=(0.6, 0.6, 0.6))
    # Side walls: Z from 0.011 to 0.189
    middle_tray.visual(Box((0.01, 0.48, 0.178)), origin=Origin((-0.184, 0.005, 0.10)), name="left_wall", color=(0.6, 0.6, 0.6))
    middle_tray.visual(Box((0.01, 0.48, 0.178)), origin=Origin((0.184, 0.005, 0.10)), name="right_wall", color=(0.6, 0.6, 0.6))
    # Back wall
    middle_tray.visual(Box((0.358, 0.01, 0.178)), origin=Origin((0.0, -0.230, 0.10)), name="back_wall", color=(0.6, 0.6, 0.6))

    # Inner Tray (Final Drawer)
    inner_tray = model.part("inner_tray")
    # Bottom: Z from 0.022 to 0.032
    inner_tray.visual(Box((0.356, 0.46, 0.01)), origin=Origin((0.0, 0.015, 0.027)), name="bottom", color=(0.8, 0.8, 0.8))
    # Side walls: Z from 0.022 to 0.178
    inner_tray.visual(Box((0.01, 0.45, 0.156)), origin=Origin((-0.173, 0.010, 0.10)), name="left_wall", color=(0.8, 0.8, 0.8))
    inner_tray.visual(Box((0.01, 0.45, 0.156)), origin=Origin((0.173, 0.010, 0.10)), name="right_wall", color=(0.8, 0.8, 0.8))
    # Back wall
    inner_tray.visual(Box((0.336, 0.01, 0.156)), origin=Origin((0.0, -0.210, 0.10)), name="back_wall", color=(0.8, 0.8, 0.8))
    # Front wall
    inner_tray.visual(Box((0.356, 0.01, 0.156)), origin=Origin((0.0, 0.240, 0.10)), name="front_wall", color=(0.8, 0.8, 0.8))
    # Handle
    inner_tray.visual(Box((0.10, 0.01, 0.02)), origin=Origin((0.0, 0.250, 0.10)), name="handle", color=(0.1, 0.1, 0.1))

    # Articulations
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_tray,
        child=middle_tray,
        origin=Origin((0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.35),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_tray,
        child=inner_tray,
        origin=Origin((0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.35),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_tray")
    middle = object_model.get_part("middle_tray")
    inner = object_model.get_part("inner_tray")
    j1 = object_model.get_articulation("outer_to_middle")
    j2 = object_model.get_articulation("middle_to_inner")

    # Allow isolated parts due to sliding clearances
    ctx.allow_isolated_part("middle_tray", reason="Modeled with a small clearance gap to represent sliding.")
    ctx.allow_isolated_part("inner_tray", reason="Modeled with a small clearance gap to represent sliding.")

    # Exact checks at rest
    ctx.expect_within(middle, outer, axes="x")
    ctx.expect_within(inner, middle, axes="x")

    # Check Z gaps
    ctx.expect_gap(middle, outer, axis="z", positive_elem="bottom", negative_elem="bottom", min_gap=0.0005, max_gap=0.0015)
    ctx.expect_gap(inner, middle, axis="z", positive_elem="bottom", negative_elem="bottom", min_gap=0.0005, max_gap=0.0015)

    # Check retained insertion at rest
    ctx.expect_overlap(middle, outer, axes="y", min_overlap=0.4)
    ctx.expect_overlap(inner, middle, axes="y", min_overlap=0.4)

    # Check at full extension
    with ctx.pose({j1: 0.35, j2: 0.35}):
        ctx.expect_within(middle, outer, axes="x")
        ctx.expect_within(inner, middle, axes="x")
        ctx.expect_overlap(middle, outer, axes="y", min_overlap=0.1)
        ctx.expect_overlap(inner, middle, axes="y", min_overlap=0.1)

    return ctx.report()

object_model = build_object_model()

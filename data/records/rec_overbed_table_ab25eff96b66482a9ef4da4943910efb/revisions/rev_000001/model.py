from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehab_overbed_table")

    base = model.part("base")
    # Base legs and crossbar
    base.visual(Box((0.05, 0.6, 0.05)), origin=Origin(xyz=(-0.4, 0.0, 0.105)), name="left_leg")
    base.visual(Box((0.05, 0.6, 0.05)), origin=Origin(xyz=(0.4, 0.0, 0.105)), name="right_leg")
    base.visual(Box((0.75, 0.05, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.105)), name="crossbar")
    
    # Twin uprights (outer sleeves)
    base.visual(Box((0.05, 0.05, 0.7)), origin=Origin(xyz=(-0.4, 0.0, 0.48)), name="left_upright")
    base.visual(Box((0.05, 0.05, 0.7)), origin=Origin(xyz=(0.4, 0.0, 0.48)), name="right_upright")

    bridge = model.part("bridge")
    # Inner columns
    bridge.visual(Box((0.04, 0.04, 0.8)), origin=Origin(xyz=(-0.4, 0.0, 0.53)), name="left_inner_column")
    bridge.visual(Box((0.04, 0.04, 0.8)), origin=Origin(xyz=(0.4, 0.0, 0.53)), name="right_inner_column")
    # Bridge beam
    bridge.visual(Box((0.84, 0.1, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.955)), name="bridge_beam")

    model.articulation(
        "bridge_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.5, lower=0.0, upper=0.4),
    )

    tabletop = model.part("tabletop")
    tabletop.visual(Box((0.9, 0.5, 0.02)), origin=Origin(xyz=(0.0, -0.25, 0.01)), name="panel")

    model.articulation(
        "tabletop_tilt",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=tabletop,
        origin=Origin(xyz=(0.0, 0.05, 0.98)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=1.0),
    )

    # Casters
    for prefix, x, y in [
        ("fl", -0.4, -0.25),
        ("fr", 0.4, -0.25),
        ("rl", -0.4, 0.25),
        ("rr", 0.4, 0.25),
    ]:
        swivel = model.part(f"caster_swivel_{prefix}")
        swivel.visual(Box((0.04, 0.04, 0.05)), origin=Origin(xyz=(0.0, -0.01, -0.025)), name="bracket")
        
        model.articulation(
            f"swivel_{prefix}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=swivel,
            origin=Origin(xyz=(x, y, 0.08)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0),
        )

        wheel = model.part(f"caster_wheel_{prefix}")
        wheel.visual(
            Cylinder(radius=0.03, length=0.02),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
            name="tire",
        )

        model.articulation(
            f"wheel_{prefix}",
            ArticulationType.CONTINUOUS,
            parent=swivel,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.02, -0.05)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    tabletop = object_model.get_part("tabletop")
    
    for prefix in ["fl", "fr", "rl", "rr"]:
        ctx.allow_overlap(
            f"caster_swivel_{prefix}",
            f"caster_wheel_{prefix}",
            reason="Wheel is mounted inside the caster bracket.",
        )
    
    # Allow overlap between inner columns and uprights
    ctx.allow_overlap(base, bridge, elem_a="left_upright", elem_b="left_inner_column", reason="Telescoping column slides inside upright proxy")
    ctx.allow_overlap(base, bridge, elem_a="right_upright", elem_b="right_inner_column", reason="Telescoping column slides inside upright proxy")
    
    ctx.expect_within(bridge, base, axes="xy", inner_elem="left_inner_column", outer_elem="left_upright", margin=0.001)
    ctx.expect_within(bridge, base, axes="xy", inner_elem="right_inner_column", outer_elem="right_upright", margin=0.001)
    
    ctx.expect_overlap(bridge, base, axes="z", elem_a="left_inner_column", elem_b="left_upright", min_overlap=0.1)
    ctx.expect_overlap(bridge, base, axes="z", elem_a="right_inner_column", elem_b="right_upright", min_overlap=0.1)

    ctx.expect_contact(tabletop, bridge, elem_a="panel", elem_b="bridge_beam")

    with ctx.pose({"bridge_lift": 0.4}):
        ctx.expect_overlap(bridge, base, axes="z", elem_a="left_inner_column", elem_b="left_upright", min_overlap=0.1)
        ctx.expect_overlap(bridge, base, axes="z", elem_a="right_inner_column", elem_b="right_upright", min_overlap=0.1)

    return ctx.report()


object_model = build_object_model()

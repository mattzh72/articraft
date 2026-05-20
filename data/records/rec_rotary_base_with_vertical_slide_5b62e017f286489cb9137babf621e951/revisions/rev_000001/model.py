from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_lift_module")

    mat_dark = Material(name="dark_metal", rgba=(0.2, 0.2, 0.2, 1.0))
    mat_light = Material(name="light_metal", rgba=(0.6, 0.6, 0.6, 1.0))
    mat_rail = Material(name="rail_metal", rgba=(0.8, 0.8, 0.8, 1.0))
    mat_accent = Material(name="accent", rgba=(1.0, 0.4, 0.0, 1.0))
    mat_accent_dark = Material(name="accent_dark", rgba=(0.8, 0.3, 0.0, 1.0))

    # 1. Base
    base = model.part("base")
    base.visual(
        Box((0.20, 0.20, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_body",
        material=mat_dark,
    )

    # 2. Rotary Stage
    rotary_stage = model.part("rotary_stage")
    # Turntable
    rotary_stage.visual(
        Cylinder(radius=0.09, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="turntable",
        material=mat_light,
    )
    # Vertical Column
    rotary_stage.visual(
        Box((0.04, 0.04, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        name="column",
        material=mat_light,
    )
    # Linear Rail on the front of the column
    rotary_stage.visual(
        Box((0.02, 0.01, 0.30)),
        origin=Origin(xyz=(0.0, -0.025, 0.17)),
        name="rail",
        material=mat_rail,
    )
    # Top plate
    rotary_stage.visual(
        Box((0.06, 0.06, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        name="top_plate",
        material=mat_dark,
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-3.14, upper=3.14),
    )

    # 3. Lift Carriage
    lift_carriage = model.part("lift_carriage")
    lift_carriage.visual(
        Box((0.08, 0.04, 0.08)),
        # The joint origin is at the front face of the rail.
        # The carriage extends forward (-Y) from the joint.
        origin=Origin(xyz=(0.0, -0.02, 0.04)),
        name="carriage_body",
        material=mat_accent,
    )
    
    # Carriage mount detail that visually wraps the rail slightly
    lift_carriage.visual(
        Box((0.04, 0.01, 0.08)),
        origin=Origin(xyz=(0.0, 0.005, 0.04)),
        name="carriage_mount",
        material=mat_accent_dark,
    )

    model.articulation(
        "lift_joint",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=lift_carriage,
        origin=Origin(xyz=(0.0, -0.03, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=0.22),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    rotary_stage = object_model.get_part("rotary_stage")
    lift_carriage = object_model.get_part("lift_carriage")
    
    yaw_joint = object_model.get_articulation("yaw_joint")
    lift_joint = object_model.get_articulation("lift_joint")

    # Allow overlap between the carriage mount and the rail since it's a proxy for a linear bearing wrapping the rail
    ctx.allow_overlap(
        lift_carriage,
        rotary_stage,
        elem_a="carriage_mount",
        elem_b="rail",
        reason="The carriage mount wraps around the linear rail, modeled here as a simple overlap proxy."
    )

    # Check that rotary stage sits on the base
    ctx.expect_contact(rotary_stage, base, elem_a="turntable", elem_b="base_body")
    
    # Check that lift carriage remains on the rail
    ctx.expect_overlap(lift_carriage, rotary_stage, axes="z", elem_a="carriage_body", elem_b="rail", min_overlap=0.079)

    # Check limits
    with ctx.pose({lift_joint: 0.22}):
        # At max lift, the carriage should be at the top of the column, touching the top plate
        ctx.expect_contact(lift_carriage, rotary_stage, elem_a="carriage_body", elem_b="top_plate")

    return ctx.report()

object_model = build_object_model()

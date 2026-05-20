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
    model = ArticulatedObject(name="display_easel")

    # Mast (root part)
    mast = model.part("mast")
    mast.visual(
        Box((0.04, 0.04, 1.8)),
        origin=Origin(xyz=(0.0, 0.0, 1.1)),
        name="pole",
    )
    mast.visual(
        Cylinder(radius=0.06, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.8)),
        name="crown",
    )
    mast.visual(
        Box((0.6, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.045, 0.6)),
        name="lower_ledge",
    )
    mast.visual(
        Box((0.6, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.08, 0.63)),
        name="lower_ledge_lip",
    )

    # Top clamp (slides on mast)
    top_clamp = model.part("top_clamp")
    # Sleeve elements
    top_clamp.visual(
        Box((0.08, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
        name="clamp_front",
    )
    top_clamp.visual(
        Box((0.08, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, -0.03, 0.0)),
        name="clamp_back",
    )
    top_clamp.visual(
        Box((0.02, 0.04, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        name="clamp_right",
    )
    top_clamp.visual(
        Box((0.02, 0.04, 0.05)),
        origin=Origin(xyz=(-0.03, 0.0, 0.0)),
        name="clamp_left",
    )
    # Lip to hold canvas
    top_clamp.visual(
        Box((0.08, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.05, -0.025)),
        name="clamp_lip",
    )
    # Tightening knob
    top_clamp.visual(
        Cylinder(radius=0.015, length=0.02),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        name="clamp_knob",
    )

    model.articulation(
        "mast_to_top_clamp",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=top_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.7)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=1.1),
    )

    # Legs
    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Box((0.03, 0.03, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
        name="leg_pole",
    )
    rear_leg.visual(
        Box((0.035, 0.035, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.91)),
        name="foot",
    )
    model.articulation(
        "mast_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.073, 0.8)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.6),
    )

    front_left_leg = model.part("front_left_leg")
    front_left_leg.visual(
        Box((0.03, 0.03, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
        name="leg_pole",
    )
    front_left_leg.visual(
        Box((0.035, 0.035, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.91)),
        name="foot",
    )
    model.articulation(
        "mast_to_front_left_leg",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=front_left_leg,
        origin=Origin(xyz=(0.0632, 0.0365, 0.8)),
        axis=(0.5, -0.866, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.6),
    )

    front_right_leg = model.part("front_right_leg")
    front_right_leg.visual(
        Box((0.03, 0.03, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
        name="leg_pole",
    )
    front_right_leg.visual(
        Box((0.035, 0.035, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.91)),
        name="foot",
    )
    model.articulation(
        "mast_to_front_right_leg",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=front_right_leg,
        origin=Origin(xyz=(-0.0632, 0.0365, 0.8)),
        axis=(0.5, 0.866, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.6),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    top_clamp = object_model.get_part("top_clamp")
    rear_leg = object_model.get_part("rear_leg")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")

    ctx.allow_overlap(
        top_clamp,
        mast,
        reason="Top clamp sleeve intentionally fits flush against the mast.",
    )
    ctx.allow_overlap(
        rear_leg,
        mast,
        reason="Legs are hinged to the crown and have an intentional slight overlap for seating.",
    )
    ctx.allow_overlap(
        front_left_leg,
        mast,
        reason="Legs are hinged to the crown and have an intentional slight overlap for seating.",
    )
    ctx.allow_overlap(
        front_right_leg,
        mast,
        reason="Legs are hinged to the crown and have an intentional slight overlap for seating.",
    )

    # Check top clamp slides and stays centered
    ctx.expect_within(top_clamp, mast, axes="xy", margin=0.045)
    ctx.expect_overlap(top_clamp, mast, axes="z")
    
    with ctx.pose(mast_to_top_clamp=1.0):
        ctx.expect_within(top_clamp, mast, axes="xy", margin=0.045)
        ctx.expect_overlap(top_clamp, mast, axes="z")

    # Check legs rotate outward
    rest_rear_aabb = ctx.part_world_aabb(rear_leg)
    rest_fl_aabb = ctx.part_world_aabb(front_left_leg)
    rest_fr_aabb = ctx.part_world_aabb(front_right_leg)

    with ctx.pose(mast_to_rear_leg=0.6, mast_to_front_left_leg=0.6, mast_to_front_right_leg=0.6):
        ext_rear_aabb = ctx.part_world_aabb(rear_leg)
        ext_fl_aabb = ctx.part_world_aabb(front_left_leg)
        ext_fr_aabb = ctx.part_world_aabb(front_right_leg)
        
        if rest_rear_aabb and ext_rear_aabb:
            ctx.check("rear_leg_moves_outward", ext_rear_aabb[0][1] < rest_rear_aabb[0][1] - 0.1)
        if rest_fl_aabb and ext_fl_aabb:
            ctx.check("front_left_leg_moves_outward", ext_fl_aabb[1][0] > rest_fl_aabb[1][0] + 0.1 and ext_fl_aabb[1][1] > rest_fl_aabb[1][1] + 0.1)
        if rest_fr_aabb and ext_fr_aabb:
            ctx.check("front_right_leg_moves_outward", ext_fr_aabb[0][0] < rest_fr_aabb[0][0] - 0.1 and ext_fr_aabb[1][1] > rest_fr_aabb[1][1] + 0.1)

    return ctx.report()

object_model = build_object_model()

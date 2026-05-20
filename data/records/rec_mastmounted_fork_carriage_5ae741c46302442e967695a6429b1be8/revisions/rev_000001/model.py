from __future__ import annotations

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


STEEL = Material("painted_steel", rgba=(0.14, 0.16, 0.17, 1.0))
WEAR_STEEL = Material("polished_wear_steel", rgba=(0.62, 0.64, 0.62, 1.0))
SAFETY_ORANGE = Material("safety_orange", rgba=(0.95, 0.42, 0.06, 1.0))
DARK_TINE = Material("dark_fork_steel", rgba=(0.08, 0.085, 0.08, 1.0))
RUBBER = Material("black_guide_pads", rgba=(0.015, 0.015, 0.012, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_forklift_mast")

    mast = model.part("mast")

    # Root mast frame: +Z is the lift axis, +X is the fork/front side, +Y is mast width.
    rail_z = 0.95
    rail_h = 1.85
    for side, y_sign in (("0", -1.0), ("1", 1.0)):
        web_y = y_sign * 0.315
        flange_y = y_sign * 0.276
        # Each upright is a C-channel, open toward the centre of the mast.
        mast.visual(
            Box((0.160, 0.025, rail_h)),
            origin=Origin(xyz=(0.0, web_y, rail_z)),
            material=STEEL,
            name=f"upright_web_{side}",
        )
        mast.visual(
            Box((0.032, 0.080, rail_h)),
            origin=Origin(xyz=(0.064, flange_y, rail_z)),
            material=STEEL,
            name="front_flange_0" if side == "0" else "front_flange_1",
        )
        mast.visual(
            Box((0.032, 0.080, rail_h)),
            origin=Origin(xyz=(-0.064, flange_y, rail_z)),
            material=STEEL,
            name=f"rear_flange_{side}",
        )
        mast.visual(
            Box((0.018, 0.006, 1.55)),
            origin=Origin(xyz=(0.083, y_sign * 0.234, 0.96)),
            material=WEAR_STEEL,
            name=f"inner_wear_strip_{side}",
        )

    mast.visual(
        Box((0.220, 0.700, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 1.91)),
        material=STEEL,
        name="top_crosshead",
    )
    mast.visual(
        Box((0.190, 0.650, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=STEEL,
        name="bottom_tie",
    )
    mast.visual(
        Box((0.300, 0.170, 0.035)),
        origin=Origin(xyz=(0.0, -0.315, 0.018)),
        material=STEEL,
        name="base_foot_0",
    )
    mast.visual(
        Box((0.300, 0.170, 0.035)),
        origin=Origin(xyz=(0.0, 0.315, 0.018)),
        material=STEEL,
        name="base_foot_1",
    )
    mast.visual(
        Cylinder(radius=0.035, length=0.56),
        origin=Origin(xyz=(-0.015, 0.0, 1.84), rpy=(1.5708, 0.0, 0.0)),
        material=WEAR_STEEL,
        name="crosshead_pin",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.050, 0.500, 0.420)),
        origin=Origin(xyz=(0.130, 0.0, 0.220)),
        material=SAFETY_ORANGE,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.090, 0.570, 0.080)),
        origin=Origin(xyz=(0.155, 0.0, 0.430)),
        material=SAFETY_ORANGE,
        name="upper_rail",
    )
    carriage.visual(
        Box((0.090, 0.570, 0.080)),
        origin=Origin(xyz=(0.155, 0.0, 0.040)),
        material=SAFETY_ORANGE,
        name="lower_rail",
    )
    carriage.visual(
        Box((0.080, 0.045, 0.450)),
        origin=Origin(xyz=(0.130, -0.276, 0.225)),
        material=SAFETY_ORANGE,
        name="side_cheek_0",
    )
    carriage.visual(
        Box((0.080, 0.045, 0.450)),
        origin=Origin(xyz=(0.130, 0.276, 0.225)),
        material=SAFETY_ORANGE,
        name="side_cheek_1",
    )
    carriage.visual(
        Box((0.025, 0.035, 0.380)),
        origin=Origin(xyz=(0.1045, -0.232, 0.220)),
        material=RUBBER,
        name="guide_pad_0",
    )
    carriage.visual(
        Box((0.025, 0.035, 0.380)),
        origin=Origin(xyz=(0.1045, 0.232, 0.220)),
        material=RUBBER,
        name="guide_pad_1",
    )

    for side, y in (("0", -0.165), ("1", 0.165)):
        carriage.visual(
            Box((0.085, 0.080, 0.360)),
            origin=Origin(xyz=(0.180, y, 0.090)),
            material=DARK_TINE,
            name=f"fork_shank_{side}",
        )
        carriage.visual(
            Box((0.760, 0.075, 0.055)),
            origin=Origin(xyz=(0.560, y, -0.060)),
            material=DARK_TINE,
            name=f"fork_tine_{side}",
        )
        carriage.visual(
            Box((0.120, 0.074, 0.032)),
            origin=Origin(xyz=(0.930, y, -0.072)),
            material=DARK_TINE,
            name=f"fork_toe_{side}",
        )

    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.55, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("lift")

    ctx.check(
        "single moving lift joint",
        len(object_model.articulations) == 1 and lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.check(
        "lift travels along mast axis",
        lift.axis == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper >= 0.9,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        min_gap=0.001,
        max_gap=0.030,
        positive_elem="guide_pad_0",
        negative_elem="front_flange_0",
        name="carriage guide pad clears mast rail",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.95}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            min_gap=0.060,
            positive_elem="top_crosshead",
            negative_elem="upper_rail",
            name="raised carriage remains below crosshead",
        )

    ctx.check(
        "carriage raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.90,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()

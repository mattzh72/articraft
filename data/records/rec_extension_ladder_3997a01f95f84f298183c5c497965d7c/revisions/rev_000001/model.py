from __future__ import annotations

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


BASE_LENGTH = 4.20
BASE_WIDTH = 0.465
BASE_RAIL_WIDTH = 0.075
BASE_RAIL_DEPTH = 0.026
BASE_RAIL_X = 0.195
BASE_RUNG_RADIUS = 0.016
BASE_RUNG_LENGTH = 0.340
BASE_RUNG_ZS = (
    0.42,
    0.725,
    1.03,
    1.335,
    1.64,
    1.945,
    2.25,
    2.555,
    2.86,
    3.165,
    3.47,
    3.775,
)

FLY_LENGTH = 3.60
FLY_RAIL_WIDTH = 0.060
FLY_RAIL_DEPTH = 0.020
FLY_RAIL_X = 0.165
FLY_SECTION_Y = 0.032
FLY_RUNG_RADIUS = 0.014
FLY_RUNG_LENGTH = 0.296
FLY_RUNG_ZS = (
    0.40,
    0.705,
    1.01,
    1.315,
    1.62,
    1.925,
    2.23,
    2.535,
    2.84,
    3.145,
)

SLIDE_SEAT_Z = 0.95
SLIDE_TRAVEL = 1.75

ARM_LENGTH = 0.60
ARM_UPPER_LIMIT = 1.22


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((min_pt[i] + max_pt[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    fiberglass = model.material("fiberglass", rgba=(0.93, 0.70, 0.18, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.76, 0.79, 1.0))
    zinc = model.material("zinc", rgba=(0.62, 0.65, 0.69, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.24, 0.25, 0.28, 1.0))

    base = model.part("base")
    for x_pos in (-BASE_RAIL_X, BASE_RAIL_X):
        base.visual(
            Box((BASE_RAIL_WIDTH, BASE_RAIL_DEPTH, BASE_LENGTH)),
            origin=Origin(xyz=(x_pos, 0.0, BASE_LENGTH * 0.5)),
            material=fiberglass,
            name=f"base_rail_{0 if x_pos < 0.0 else 1}",
        )
        base.visual(
            Box((0.055, 0.018, 0.050)),
            origin=Origin(xyz=(x_pos, 0.015, 0.055)),
            material=dark_grey,
            name=f"shoe_lug_{0 if x_pos < 0.0 else 1}",
        )

    for index, z_pos in enumerate(BASE_RUNG_ZS):
        base.visual(
            Cylinder(radius=BASE_RUNG_RADIUS, length=BASE_RUNG_LENGTH),
            origin=Origin(xyz=(0.0, 0.002, z_pos), rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
            material=aluminum,
            name=f"base_rung_{index}",
        )

    base.visual(
        Box((0.028, 0.014, 0.110)),
        origin=Origin(xyz=(-0.236, 0.006, BASE_LENGTH - 0.200)),
        material=zinc,
        name="guide_block_0",
    )
    base.visual(
        Box((0.028, 0.014, 0.110)),
        origin=Origin(xyz=(0.236, 0.006, BASE_LENGTH - 0.200)),
        material=zinc,
        name="guide_block_1",
    )
    base.visual(
        Box((0.028, 0.014, 0.110)),
        origin=Origin(xyz=(-0.236, 0.006, BASE_LENGTH - 1.300)),
        material=zinc,
        name="guide_block_2",
    )
    base.visual(
        Box((0.028, 0.014, 0.110)),
        origin=Origin(xyz=(0.236, 0.006, BASE_LENGTH - 1.300)),
        material=zinc,
        name="guide_block_3",
    )

    fly = model.part("fly")
    for x_pos in (-FLY_RAIL_X, FLY_RAIL_X):
        fly.visual(
            Box((FLY_RAIL_WIDTH, FLY_RAIL_DEPTH, FLY_LENGTH)),
            origin=Origin(xyz=(x_pos, FLY_SECTION_Y, FLY_LENGTH * 0.5)),
            material=fiberglass,
            name=f"fly_rail_{0 if x_pos < 0.0 else 1}",
        )
        fly.visual(
            Box((0.040, 0.011, 0.120)),
            origin=Origin(xyz=(x_pos, 0.0185, 0.58)),
            material=dark_grey,
            name=f"slide_pad_{0 if x_pos < 0.0 else 1}",
        )
        fly.visual(
            Box((0.040, 0.011, 0.120)),
            origin=Origin(xyz=(x_pos, 0.0185, 2.38)),
            material=dark_grey,
            name=f"slide_pad_{2 if x_pos < 0.0 else 3}",
        )
        fly.visual(
            Box((0.050, 0.018, 0.060)),
            origin=Origin(xyz=(x_pos, 0.051, FLY_LENGTH - 0.040)),
            material=dark_grey,
            name=f"arm_lug_{0 if x_pos < 0.0 else 1}",
        )

    for index, z_pos in enumerate(FLY_RUNG_ZS):
        fly.visual(
            Cylinder(radius=FLY_RUNG_RADIUS, length=FLY_RUNG_LENGTH),
            origin=Origin(xyz=(0.0, FLY_SECTION_Y, z_pos), rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
            material=aluminum,
            name=f"fly_rung_{index}",
        )

    fly.visual(
        Box((0.300, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, FLY_SECTION_Y, 0.240)),
        material=zinc,
        name="lower_stop",
    )
    fly.visual(
        Box((0.170, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.048, 0.240)),
        material=zinc,
        name="lock_bar",
    )

    arm_0 = model.part("stand_off_arm_0")
    arm_0.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
        material=zinc,
        name="hinge_barrel",
    )
    arm_0.visual(
        Cylinder(radius=0.010, length=ARM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -ARM_LENGTH * 0.5)),
        material=aluminum,
        name="arm_tube",
    )
    arm_0.visual(
        Box((0.110, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, 0.022, -ARM_LENGTH - 0.020)),
        material=rubber,
        name="arm_pad",
    )

    arm_1 = model.part("stand_off_arm_1")
    arm_1.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
        material=zinc,
        name="hinge_barrel",
    )
    arm_1.visual(
        Cylinder(radius=0.010, length=ARM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -ARM_LENGTH * 0.5)),
        material=aluminum,
        name="arm_tube",
    )
    arm_1.visual(
        Box((0.110, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, 0.022, -ARM_LENGTH - 0.020)),
        material=rubber,
        name="arm_pad",
    )

    shoe_0 = model.part("shoe_0")
    shoe_0.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
        material=zinc,
        name="pivot_barrel",
    )
    shoe_0.visual(
        Box((0.034, 0.038, 0.042)),
        origin=Origin(xyz=(0.0, 0.020, -0.018)),
        material=dark_grey,
        name="pivot_block",
    )
    shoe_0.visual(
        Box((0.095, 0.080, 0.028)),
        origin=Origin(xyz=(0.0, 0.050, -0.032)),
        material=rubber,
        name="shoe_pad",
    )

    shoe_1 = model.part("shoe_1")
    shoe_1.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
        material=zinc,
        name="pivot_barrel",
    )
    shoe_1.visual(
        Box((0.034, 0.038, 0.042)),
        origin=Origin(xyz=(0.0, 0.020, -0.018)),
        material=dark_grey,
        name="pivot_block",
    )
    shoe_1.visual(
        Box((0.095, 0.080, 0.028)),
        origin=Origin(xyz=(0.0, 0.050, -0.032)),
        material=rubber,
        name="shoe_pad",
    )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.40,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "stand_off_arm_0_hinge",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=arm_0,
        origin=Origin(xyz=(-FLY_RAIL_X, 0.070, FLY_LENGTH - 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=ARM_UPPER_LIMIT,
        ),
    )
    model.articulation(
        "stand_off_arm_1_hinge",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=arm_1,
        origin=Origin(xyz=(FLY_RAIL_X, 0.070, FLY_LENGTH - 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=ARM_UPPER_LIMIT,
        ),
    )
    model.articulation(
        "shoe_0_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoe_0,
        origin=Origin(xyz=(-BASE_RAIL_X, 0.034, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "shoe_1_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoe_1,
        origin=Origin(xyz=(BASE_RAIL_X, 0.034, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    arm_0 = object_model.get_part("stand_off_arm_0")
    shoe_0 = object_model.get_part("shoe_0")
    shoe_1 = object_model.get_part("shoe_1")

    fly_slide = object_model.get_articulation("fly_slide")
    arm_hinge = object_model.get_articulation("stand_off_arm_0_hinge")
    shoe_0_pivot = object_model.get_articulation("shoe_0_pivot")
    shoe_1_pivot = object_model.get_articulation("shoe_1_pivot")

    ctx.expect_within(
        fly,
        base,
        axes="x",
        margin=0.030,
        name="fly stays between the base rails in width",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=1.45,
        name="fly section retains insertion in the base section",
    )

    fly_rest = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: SLIDE_TRAVEL}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.45,
            name="extended fly still overlaps the base section along the rails",
        )
        fly_extended = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        fly_rest is not None
        and fly_extended is not None
        and fly_extended[2] > fly_rest[2] + 1.70,
        details=f"rest={fly_rest}, extended={fly_extended}",
    )

    arm_closed_center = _center_from_aabb(ctx.part_world_aabb(arm_0))
    with ctx.pose({arm_hinge: ARM_UPPER_LIMIT}):
        arm_open_center = _center_from_aabb(ctx.part_world_aabb(arm_0))

    ctx.check(
        "upper stand-off arm swings outward",
        arm_closed_center is not None
        and arm_open_center is not None
        and arm_open_center[1] > arm_closed_center[1] + 0.22,
        details=f"closed={arm_closed_center}, open={arm_open_center}",
    )

    with ctx.pose({shoe_0_pivot: 0.60, shoe_1_pivot: -0.50}):
        shoe_0_center = _center_from_aabb(ctx.part_world_aabb(shoe_0))
        shoe_1_center = _center_from_aabb(ctx.part_world_aabb(shoe_1))

    ctx.check(
        "lower shoes rotate independently",
        shoe_0_center is not None
        and shoe_1_center is not None
        and abs(shoe_0_center[2] - shoe_1_center[2]) > 0.03,
        details=f"shoe_0={shoe_0_center}, shoe_1={shoe_1_center}",
    )

    return ctx.report()


object_model = build_object_model()

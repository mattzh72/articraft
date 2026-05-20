from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bridge_backed_parallel_gripper")

    anodized = model.material("dark_anodized_aluminum", color=(0.12, 0.14, 0.16, 1.0))
    steel = model.material("brushed_steel", color=(0.60, 0.62, 0.60, 1.0))
    rail = model.material("polished_guide_steel", color=(0.74, 0.76, 0.73, 1.0))
    carriage_blue = model.material("blue_carriage_blocks", color=(0.05, 0.20, 0.44, 1.0))
    rubber = model.material("matte_black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    bronze = model.material("bronze_bearing_inserts", color=(0.80, 0.52, 0.22, 1.0))

    frame = model.part("rear_frame")

    # The fixed rear frame is a bridge: two horizontal back bars tied by end
    # posts and a center web.  The short pneumatic/electric housing and the
    # guide rails overlap the bridge so the fixed part is one supported body.
    frame.visual(
        Box((0.440, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.060, 0.155)),
        material=anodized,
        name="rear_top_bridge",
    )
    frame.visual(
        Box((0.440, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.060, 0.025)),
        material=anodized,
        name="rear_bottom_bridge",
    )
    for idx, x in enumerate((-0.206, 0.206)):
        frame.visual(
            Box((0.028, 0.030, 0.150)),
            origin=Origin(xyz=(x, -0.060, 0.090)),
            material=anodized,
            name=f"rear_post_{idx}",
        )
    frame.visual(
        Box((0.065, 0.032, 0.150)),
        origin=Origin(xyz=(0.0, -0.060, 0.090)),
        material=anodized,
        name="center_web",
    )
    frame.visual(
        Box((0.118, 0.100, 0.086)),
        origin=Origin(xyz=(0.0, -0.018, 0.090)),
        material=steel,
        name="center_housing",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.050),
        origin=Origin(xyz=(0.0, -0.090, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="rear_actuator_cap",
    )

    # Two outward guideways start at the center housing and run along the grip
    # axis toward the mirrored jaw carriages.  Each guideway has upper and lower
    # hard rails backed by a shallow bridge plate.
    frame.visual(
        Box((0.168, 0.045, 0.010)),
        origin=Origin(xyz=(-0.143, -0.0125, 0.090)),
        material=anodized,
        name="guide_0_backplate",
    )
    frame.visual(
        Box((0.168, 0.020, 0.018)),
        origin=Origin(xyz=(-0.143, 0.000, 0.122)),
        material=rail,
        name="guide_0_upper_rail",
    )
    frame.visual(
        Box((0.168, 0.020, 0.018)),
        origin=Origin(xyz=(-0.143, 0.000, 0.058)),
        material=rail,
        name="guide_0_lower_rail",
    )
    frame.visual(
        Box((0.016, 0.028, 0.092)),
        origin=Origin(xyz=(-0.233, 0.000, 0.090)),
        material=anodized,
        name="guide_0_end_stop",
    )
    frame.visual(
        Box((0.168, 0.045, 0.010)),
        origin=Origin(xyz=(0.143, -0.0125, 0.090)),
        material=anodized,
        name="guide_1_backplate",
    )
    frame.visual(
        Box((0.168, 0.020, 0.018)),
        origin=Origin(xyz=(0.143, 0.000, 0.122)),
        material=rail,
        name="guide_1_upper_rail",
    )
    frame.visual(
        Box((0.168, 0.020, 0.018)),
        origin=Origin(xyz=(0.143, 0.000, 0.058)),
        material=rail,
        name="guide_1_lower_rail",
    )
    frame.visual(
        Box((0.016, 0.028, 0.092)),
        origin=Origin(xyz=(0.233, 0.000, 0.090)),
        material=anodized,
        name="guide_1_end_stop",
    )

    def make_jaw(name: str, *, side: float) -> object:
        jaw = model.part(name)
        inward = -side
        jaw.visual(
            Box((0.055, 0.036, 0.095)),
            origin=Origin(xyz=(0.0, 0.028, 0.0)),
            material=carriage_blue,
            name="slide_block",
        )
        jaw.visual(
            Box((0.062, 0.010, 0.024)),
            origin=Origin(xyz=(0.0, 0.051, 0.032)),
            material=bronze,
            name="upper_bearing",
        )
        jaw.visual(
            Box((0.062, 0.010, 0.024)),
            origin=Origin(xyz=(0.0, 0.051, -0.032)),
            material=bronze,
            name="lower_bearing",
        )
        jaw.visual(
            Box((0.065, 0.078, 0.026)),
            origin=Origin(xyz=(inward * 0.035, 0.083, 0.010)),
            material=carriage_blue,
            name="jaw_arm",
        )
        jaw.visual(
            Box((0.025, 0.032, 0.125)),
            origin=Origin(xyz=(inward * 0.075, 0.122, -0.004)),
            material=carriage_blue,
            name="jaw_finger",
        )
        jaw.visual(
            Box((0.008, 0.034, 0.075)),
            origin=Origin(xyz=(inward * 0.091, 0.123, -0.005)),
            material=rubber,
            name="grip_pad",
        )
        return jaw

    jaw_0 = make_jaw("jaw_carriage_0", side=-1.0)
    jaw_1 = make_jaw("jaw_carriage_1", side=1.0)

    model.articulation(
        "frame_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=jaw_0,
        origin=Origin(xyz=(-0.145, 0.0, 0.090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.18, lower=0.0, upper=0.045),
    )
    model.articulation(
        "frame_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=jaw_1,
        origin=Origin(xyz=(0.145, 0.0, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.18, lower=0.0, upper=0.045),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("rear_frame")
    jaw_0 = object_model.get_part("jaw_carriage_0")
    jaw_1 = object_model.get_part("jaw_carriage_1")
    slide_0 = object_model.get_articulation("frame_to_jaw_0")
    slide_1 = object_model.get_articulation("frame_to_jaw_1")

    ctx.check(
        "two independent prismatic jaw slides",
        slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC
        and slide_0.mimic is None
        and slide_1.mimic is None,
        details="The two mirrored jaw carriages should be separate prismatic joints, not a fused or mimic-only jaw.",
    )
    ctx.expect_gap(
        jaw_0,
        frame,
        axis="y",
        positive_elem="slide_block",
        negative_elem="guide_0_upper_rail",
        min_gap=0.0,
        max_gap=0.0002,
        name="jaw 0 rides close to its upper guide rail",
    )
    ctx.expect_gap(
        jaw_1,
        frame,
        axis="y",
        positive_elem="slide_block",
        negative_elem="guide_1_upper_rail",
        min_gap=0.0,
        max_gap=0.0002,
        name="jaw 1 rides close to its upper guide rail",
    )
    ctx.expect_overlap(
        jaw_0,
        frame,
        axes="xz",
        elem_a="slide_block",
        elem_b="guide_0_upper_rail",
        min_overlap=0.018,
        name="jaw 0 slide block is engaged on guideway",
    )
    ctx.expect_overlap(
        jaw_1,
        frame,
        axes="xz",
        elem_a="slide_block",
        elem_b="guide_1_upper_rail",
        min_overlap=0.018,
        name="jaw 1 slide block is engaged on guideway",
    )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({slide_0: 0.045, slide_1: 0.045}):
        ctx.expect_overlap(
            jaw_0,
            frame,
            axes="xz",
            elem_a="slide_block",
            elem_b="guide_0_upper_rail",
            min_overlap=0.018,
            name="jaw 0 remains captured at full travel",
        )
        ctx.expect_overlap(
            jaw_1,
            frame,
            axes="xz",
            elem_a="slide_block",
            elem_b="guide_1_upper_rail",
            min_overlap=0.018,
            name="jaw 1 remains captured at full travel",
        )
        open_0 = ctx.part_world_position(jaw_0)
        open_1 = ctx.part_world_position(jaw_1)

    ctx.check(
        "jaws open outward along grip axis",
        rest_0 is not None
        and rest_1 is not None
        and open_0 is not None
        and open_1 is not None
        and open_0[0] < rest_0[0] - 0.040
        and open_1[0] > rest_1[0] + 0.040,
        details=f"rest_0={rest_0}, open_0={open_0}, rest_1={rest_1}, open_1={open_1}",
    )

    return ctx.report()


object_model = build_object_model()

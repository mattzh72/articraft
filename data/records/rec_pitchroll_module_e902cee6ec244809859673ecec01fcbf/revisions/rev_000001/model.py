from __future__ import annotations

import math

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


X_AXIS = (0.0, math.pi / 2.0, 0.0)
Y_AXIS = (-math.pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gimbal_cube")

    black = model.material("hard_black_anodized", rgba=(0.015, 0.017, 0.020, 1.0))
    gunmetal = model.material("satin_gunmetal", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bronze = model.material("bearing_bronze", rgba=(0.78, 0.55, 0.26, 1.0))
    dark_bolt = model.material("black_fasteners", rgba=(0.005, 0.005, 0.006, 1.0))

    shaft_carrier = model.part("shaft_carrier")
    shaft_carrier.visual(
        Box((0.225, 0.035, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=steel,
        name="top_bridge",
    )
    for index, x in enumerate((-0.106, 0.106)):
        shaft_carrier.visual(
            Box((0.020, 0.035, 0.105)),
            origin=Origin(xyz=(x, 0.0, 0.057)),
            material=steel,
            name=f"upright_{index}",
        )
        shaft_carrier.visual(
            Box((0.032, 0.045, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=steel,
            name=f"bearing_block_{index}",
        )
        shaft_carrier.visual(
            Cylinder(radius=0.011, length=0.050),
            origin=Origin(xyz=(0.080 if x > 0.0 else -0.080, 0.0, 0.0), rpy=X_AXIS),
            material=steel,
            name=f"main_shaft_{index}",
        )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((0.082, 0.180, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=black,
        name="top_rail",
    )
    outer_frame.visual(
        Box((0.082, 0.180, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.077)),
        material=black,
        name="bottom_rail",
    )
    outer_frame.visual(
        Box((0.082, 0.026, 0.128)),
        origin=Origin(xyz=(0.0, 0.077, 0.0)),
        material=black,
        name="side_rail_0",
    )
    outer_frame.visual(
        Box((0.082, 0.026, 0.128)),
        origin=Origin(xyz=(0.0, -0.077, 0.0)),
        material=black,
        name="side_rail_1",
    )

    for prefix, x in (("front", 0.045), ("rear", -0.045)):
        outer_frame.visual(
            Cylinder(radius=0.023, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=X_AXIS),
            material=black,
            name=f"{prefix}_collar",
        )
        for z, suffix in ((0.043, "top"), (-0.043, "bottom")):
            outer_frame.visual(
                Box((0.012, 0.010, 0.050)),
                origin=Origin(xyz=(x * 0.87, 0.0, z)),
                material=black,
                name=f"{prefix}_{suffix}_spoke",
            )
        for y, suffix in ((0.043, "side_0"), (-0.043, "side_1")):
            outer_frame.visual(
                Box((0.012, 0.050, 0.010)),
                origin=Origin(xyz=(x * 0.87, y, 0.0)),
                material=black,
                name=f"{prefix}_{suffix}_spoke",
            )

    for index, y in enumerate((0.067, -0.067)):
        outer_frame.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=Y_AXIS),
            material=bronze,
            name=f"pitch_bushing_{index}",
        )

    for index, (y, z) in enumerate(
        (
            (0.065, 0.078),
            (-0.065, 0.078),
            (0.065, -0.078),
            (-0.065, -0.078),
            (0.079, 0.052),
            (-0.079, 0.052),
            (0.079, -0.052),
            (-0.079, -0.052),
        )
    ):
        outer_frame.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(0.034, y, z), rpy=X_AXIS),
            material=dark_bolt,
            name=f"screw_{index}",
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.044, length=0.040),
        origin=Origin(rpy=X_AXIS),
        material=gunmetal,
        name="barrel_shell",
    )
    for index, x in enumerate((0.019, -0.019)):
        cradle.visual(
            Cylinder(radius=0.046, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=X_AXIS),
            material=black,
            name=f"barrel_band_{index}",
        )
    cradle.visual(
        Box((0.008, 0.052, 0.052)),
        origin=Origin(xyz=(0.023, 0.0, 0.0)),
        material=steel,
        name="center_flange",
    )
    cradle.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=X_AXIS),
        material=steel,
        name="flange_boss",
    )
    for index, (y, z) in enumerate(
        ((0.016, 0.016), (-0.016, 0.016), (0.016, -0.016), (-0.016, -0.016))
    ):
        cradle.visual(
            Cylinder(radius=0.003, length=0.003),
            origin=Origin(xyz=(0.0285, y, z), rpy=X_AXIS),
            material=dark_bolt,
            name=f"flange_bolt_{index}",
        )
    for index, y in enumerate((0.052, -0.052)):
        cradle.visual(
            Cylinder(radius=0.0065, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=Y_AXIS),
            material=steel,
            name=f"pitch_pin_{index}",
        )
        cradle.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, 0.043 if y > 0.0 else -0.043, 0.0), rpy=Y_AXIS),
            material=steel,
            name=f"pin_collar_{index}",
        )

    model.articulation(
        "shaft_to_frame",
        ArticulationType.REVOLUTE,
        parent=shaft_carrier,
        child=outer_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-0.60, upper=0.60),
    )
    model.articulation(
        "frame_to_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carrier = object_model.get_part("shaft_carrier")
    frame = object_model.get_part("outer_frame")
    cradle = object_model.get_part("cradle")
    roll = object_model.get_articulation("shaft_to_frame")
    pitch = object_model.get_articulation("frame_to_cradle")

    def _center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.check(
        "outer frame rolls on the main shaft axis",
        roll.axis == (1.0, 0.0, 0.0)
        and roll.motion_limits is not None
        and roll.motion_limits.lower < 0.0 < roll.motion_limits.upper,
        details=f"axis={roll.axis}, limits={roll.motion_limits}",
    )
    ctx.check(
        "inner cradle pitches on a perpendicular cross-axis",
        pitch.axis == (0.0, 1.0, 0.0)
        and pitch.motion_limits is not None
        and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper,
        details=f"axis={pitch.axis}, limits={pitch.motion_limits}",
    )

    ctx.expect_gap(
        frame,
        cradle,
        axis="z",
        positive_elem="top_rail",
        negative_elem="barrel_shell",
        min_gap=0.012,
        max_gap=0.030,
        name="top nested clearance around barrel",
    )
    ctx.expect_gap(
        cradle,
        frame,
        axis="z",
        positive_elem="barrel_shell",
        negative_elem="bottom_rail",
        min_gap=0.012,
        max_gap=0.030,
        name="bottom nested clearance around barrel",
    )
    ctx.expect_gap(
        frame,
        cradle,
        axis="y",
        positive_elem="side_rail_0",
        negative_elem="barrel_shell",
        min_gap=0.012,
        max_gap=0.030,
        name="side nested clearance around barrel",
    )
    ctx.expect_gap(
        cradle,
        frame,
        axis="y",
        positive_elem="barrel_shell",
        negative_elem="side_rail_1",
        min_gap=0.012,
        max_gap=0.030,
        name="opposite side nested clearance around barrel",
    )
    ctx.expect_gap(
        frame,
        cradle,
        axis="x",
        positive_elem="front_collar",
        negative_elem="flange_boss",
        min_gap=0.001,
        max_gap=0.010,
        name="front axial clearance before flange",
    )
    ctx.expect_gap(
        cradle,
        frame,
        axis="x",
        positive_elem="barrel_band_1",
        negative_elem="rear_collar",
        min_gap=0.008,
        max_gap=0.020,
        name="rear axial clearance behind barrel",
    )

    rest_top = _center(ctx.part_element_world_aabb(frame, elem="top_rail"))
    with ctx.pose({roll: 0.55}):
        rolled_top = _center(ctx.part_element_world_aabb(frame, elem="top_rail"))
    ctx.check(
        "roll pose visibly swings square frame",
        rest_top is not None
        and rolled_top is not None
        and rolled_top[1] < rest_top[1] - 0.025
        and rolled_top[2] < rest_top[2] - 0.006,
        details=f"rest_top={rest_top}, rolled_top={rolled_top}",
    )

    rest_flange = _center(ctx.part_element_world_aabb(cradle, elem="center_flange"))
    with ctx.pose({pitch: 0.50}):
        pitched_flange = _center(ctx.part_element_world_aabb(cradle, elem="center_flange"))
    ctx.check(
        "pitch pose tips the central flange",
        rest_flange is not None
        and pitched_flange is not None
        and pitched_flange[2] < rest_flange[2] - 0.008,
        details=f"rest_flange={rest_flange}, pitched_flange={pitched_flange}",
    )

    return ctx.report()


object_model = build_object_model()

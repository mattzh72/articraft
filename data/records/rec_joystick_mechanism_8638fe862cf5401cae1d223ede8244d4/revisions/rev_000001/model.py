from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_pedestal_joystick")

    painted_steel = model.material("painted_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.66, 0.08, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    nameplate_blue = model.material("nameplate_blue", rgba=(0.05, 0.12, 0.20, 1.0))

    lever_profile = [
        (0.0, 0.00),
        (0.038, 0.00),
        (0.032, 0.18),
        (0.023, 0.50),
        (0.017, 0.76),
        (0.0, 0.76),
    ]
    lever_mesh = mesh_from_geometry(
        LatheGeometry(lever_profile, segments=40),
        "tapered_control_lever",
    )

    base = model.part("base_housing")
    base.visual(
        Box((0.70, 0.48, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=painted_steel,
        name="rectangular_housing",
    )
    base.visual(
        Box((0.44, 0.30, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=painted_steel,
        name="raised_pedestal",
    )
    base.visual(
        Box((0.13, 0.10, 0.46)),
        origin=Origin(xyz=(0.0, 0.285, 0.41)),
        material=painted_steel,
        name="bearing_tower_0",
    )
    base.visual(
        Box((0.13, 0.10, 0.46)),
        origin=Origin(xyz=(0.0, -0.285, 0.41)),
        material=painted_steel,
        name="bearing_tower_1",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.105),
        origin=Origin(xyz=(0.0, 0.285, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pitch_bearing_0",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.105),
        origin=Origin(xyz=(0.0, -0.285, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pitch_bearing_1",
    )
    base.visual(
        Box((0.11, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=safety_yellow,
        name="top_warning_strip",
    )
    base.visual(
        Box((0.33, 0.012, 0.075)),
        origin=Origin(xyz=(0.14, -0.241, 0.092)),
        material=nameplate_blue,
        name="front_nameplate",
    )
    for index, (x, y) in enumerate(
        [(-0.26, -0.17), (-0.26, 0.17), (0.26, -0.17), (0.26, 0.17)]
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.187)),
            material=dark_steel,
            name=f"anchor_bolt_{index}",
        )
    for index, (y, yaw) in enumerate(((0.225, 0.0), (-0.225, math.pi))):
        base.visual(
            Box((0.105, 0.055, 0.34)),
            origin=Origin(xyz=(-0.065, y, 0.36), rpy=(0.0, math.radians(14.0), yaw)),
            material=painted_steel,
            name=f"tower_web_{index}",
        )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(0.0, 0.285, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_trunnion_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(0.0, -0.285, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_trunnion_1",
    )
    outer_yoke.visual(
        Box((0.060, 0.085, 0.235)),
        origin=Origin(xyz=(0.205, 0.0, 0.1725)),
        material=safety_yellow,
        name="side_cheek_0",
    )
    outer_yoke.visual(
        Box((0.060, 0.085, 0.235)),
        origin=Origin(xyz=(-0.205, 0.0, 0.1725)),
        material=safety_yellow,
        name="side_cheek_1",
    )
    outer_yoke.visual(
        Box((0.060, 0.085, 0.155)),
        origin=Origin(xyz=(0.205, 0.0, -0.1325)),
        material=safety_yellow,
        name="lower_cheek_0",
    )
    outer_yoke.visual(
        Box((0.060, 0.085, 0.155)),
        origin=Origin(xyz=(-0.205, 0.0, -0.1325)),
        material=safety_yellow,
        name="lower_cheek_1",
    )
    outer_yoke.visual(
        Box((0.060, 0.085, 0.085)),
        origin=Origin(xyz=(0.205, 0.0, 0.315)),
        material=safety_yellow,
        name="upper_cap_0",
    )
    outer_yoke.visual(
        Box((0.060, 0.085, 0.085)),
        origin=Origin(xyz=(-0.205, 0.0, 0.315)),
        material=safety_yellow,
        name="upper_cap_1",
    )
    outer_yoke.visual(
        Box((0.47, 0.390, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=safety_yellow,
        name="bottom_crossbar",
    )
    outer_yoke.visual(
        Cylinder(radius=0.060, length=0.085),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="roll_bearing_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.060, length=0.085),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="roll_bearing_1",
    )
    outer_yoke.visual(
        Box((0.060, 0.040, 0.220)),
        origin=Origin(xyz=(0.0, 0.170, -0.105)),
        material=safety_yellow,
        name="trunnion_web_0",
    )
    outer_yoke.visual(
        Box((0.060, 0.040, 0.220)),
        origin=Origin(xyz=(0.0, -0.170, -0.105)),
        material=safety_yellow,
        name="trunnion_web_1",
    )
    outer_yoke.visual(
        Box((0.065, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.185, 0.000)),
        material=safety_yellow,
        name="trunnion_neck_0",
    )
    outer_yoke.visual(
        Box((0.065, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, -0.185, 0.000)),
        material=safety_yellow,
        name="trunnion_neck_1",
    )

    inner_frame = model.part("inner_frame")
    inner_frame.visual(
        Cylinder(radius=0.024, length=0.52),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_axle",
    )
    inner_frame.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="center_hub",
    )
    inner_frame.visual(
        Box((0.070, 0.045, 0.255)),
        origin=Origin(xyz=(0.0, 0.120, -0.020)),
        material=dark_steel,
        name="inner_side_0",
    )
    inner_frame.visual(
        Box((0.070, 0.045, 0.255)),
        origin=Origin(xyz=(0.0, -0.120, -0.020)),
        material=dark_steel,
        name="inner_side_1",
    )
    inner_frame.visual(
        Box((0.075, 0.285, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=dark_steel,
        name="inner_bottom_bridge",
    )
    inner_frame.visual(
        Box((0.075, 0.285, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_steel,
        name="inner_top_tie",
    )
    inner_frame.visual(
        Cylinder(radius=0.052, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=machined_steel,
        name="lever_socket",
    )
    inner_frame.visual(
        lever_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=machined_steel,
        name="tapered_lever",
    )
    inner_frame.visual(
        Sphere(radius=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.892)),
        material=black_rubber,
        name="handle_grip",
    )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=math.radians(-22.0),
            upper=math.radians(22.0),
        ),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.4,
            lower=math.radians(-26.0),
            upper=math.radians(26.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    outer = object_model.get_part("outer_yoke")
    inner = object_model.get_part("inner_frame")
    pitch = object_model.get_articulation("pitch_axis")
    roll = object_model.get_articulation("roll_axis")

    ctx.allow_overlap(
        base,
        outer,
        elem_a="pitch_bearing_0",
        elem_b="pitch_trunnion_0",
        reason="The pitch trunnion is intentionally captured inside the solid-proxy side bearing.",
    )
    ctx.allow_overlap(
        base,
        outer,
        elem_a="pitch_bearing_1",
        elem_b="pitch_trunnion_1",
        reason="The pitch trunnion is intentionally captured inside the solid-proxy side bearing.",
    )
    ctx.allow_overlap(
        outer,
        inner,
        elem_a="roll_bearing_0",
        elem_b="roll_axle",
        reason="The roll axle is intentionally seated through the yoke bearing boss.",
    )
    ctx.allow_overlap(
        outer,
        inner,
        elem_a="roll_bearing_1",
        elem_b="roll_axle",
        reason="The roll axle is intentionally seated through the yoke bearing boss.",
    )

    ctx.expect_overlap(
        base,
        outer,
        axes="yz",
        elem_a="pitch_bearing_0",
        elem_b="pitch_trunnion_0",
        min_overlap=0.045,
        name="positive pitch bearing captures shaft",
    )
    ctx.expect_overlap(
        base,
        outer,
        axes="yz",
        elem_a="pitch_bearing_1",
        elem_b="pitch_trunnion_1",
        min_overlap=0.045,
        name="negative pitch bearing captures shaft",
    )
    ctx.expect_overlap(
        outer,
        inner,
        axes="xz",
        elem_a="roll_bearing_0",
        elem_b="roll_axle",
        min_overlap=0.035,
        name="front roll boss captures axle",
    )
    ctx.expect_overlap(
        outer,
        inner,
        axes="xz",
        elem_a="roll_bearing_1",
        elem_b="roll_axle",
        min_overlap=0.035,
        name="rear roll boss captures axle",
    )
    ctx.expect_gap(
        inner,
        base,
        axis="z",
        min_gap=0.06,
        positive_elem="inner_bottom_bridge",
        negative_elem="raised_pedestal",
        name="cardan frame clears pedestal top",
    )

    def _z_center(aabb):
        return (aabb[0][2] + aabb[1][2]) * 0.5

    def _x_center(aabb):
        return (aabb[0][0] + aabb[1][0]) * 0.5

    def _y_center(aabb):
        return (aabb[0][1] + aabb[1][1]) * 0.5

    rest_tip = ctx.part_element_world_aabb(inner, elem="handle_grip")
    with ctx.pose({pitch: math.radians(18.0)}):
        pitched_tip = ctx.part_element_world_aabb(inner, elem="handle_grip")
    ctx.check(
        "pitch joint moves lever fore-aft",
        rest_tip is not None
        and pitched_tip is not None
        and _x_center(pitched_tip) > _x_center(rest_tip) + 0.18
        and _z_center(pitched_tip) < _z_center(rest_tip) - 0.025,
        details=f"rest_tip={rest_tip}, pitched_tip={pitched_tip}",
    )

    with ctx.pose({roll: math.radians(18.0)}):
        rolled_tip = ctx.part_element_world_aabb(inner, elem="handle_grip")
    ctx.check(
        "roll joint moves lever side-to-side",
        rest_tip is not None
        and rolled_tip is not None
        and _y_center(rolled_tip) < _y_center(rest_tip) - 0.18
        and _z_center(rolled_tip) < _z_center(rest_tip) - 0.025,
        details=f"rest_tip={rest_tip}, rolled_tip={rolled_tip}",
    )

    return ctx.report()


object_model = build_object_model()

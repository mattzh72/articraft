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


def _add_leg_visuals(part, metal, rubber) -> None:
    leg_drop = math.radians(13.0)
    leg_dir_x = math.cos(leg_drop)
    leg_dir_z = -math.sin(leg_drop)
    tube_length = 0.39
    tube_start = (0.028, 0.0, -0.003)
    tube_center = (
        tube_start[0] + leg_dir_x * tube_length * 0.5,
        0.0,
        tube_start[2] + leg_dir_z * tube_length * 0.5,
    )

    part.visual(
        Box((0.056, 0.024, 0.028)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=metal,
        name="hinge_block",
    )
    part.visual(
        Cylinder(radius=0.012, length=tube_length),
        origin=Origin(
            xyz=tube_center,
            rpy=(0.0, math.pi * 0.5 + leg_drop, 0.0),
        ),
        material=metal,
        name="leg_tube",
    )
    part.visual(
        Box((0.076, 0.052, 0.016)),
        origin=Origin(xyz=(0.384, 0.0, -0.087)),
        material=rubber,
        name="foot",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_c_stand")

    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.62, 0.71, 0.78, 0.65))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.056, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_steel,
        name="crown",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark_steel,
        name="base_socket",
    )
    stand.visual(
        Cylinder(radius=0.016, length=1.350),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=steel,
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.495)),
        material=dark_steel,
        name="head",
    )

    hinge_radius = 0.056
    hinge_z = 0.095
    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        _add_leg_visuals(leg, steel, rubber)
        model.articulation(
            f"stand_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), hinge_z),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=2.0,
                lower=0.0,
                upper=1.55,
            ),
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="pan_hub",
    )
    yoke.visual(
        Box((0.040, 0.280, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_steel,
        name="spine",
    )
    yoke.visual(
        Box((0.040, 0.322, 0.036)),
        origin=Origin(xyz=(0.100, 0.0, 0.245)),
        material=dark_steel,
        name="crossbar",
    )
    yoke.visual(
        Box((0.028, 0.026, 0.300)),
        origin=Origin(xyz=(0.100, 0.148, 0.105)),
        material=dark_steel,
        name="arm_0",
    )
    yoke.visual(
        Box((0.028, 0.026, 0.300)),
        origin=Origin(xyz=(0.100, -0.148, 0.105)),
        material=dark_steel,
        name="arm_1",
    )
    yoke.visual(
        Box((0.180, 0.026, 0.040)),
        origin=Origin(xyz=(0.090, 0.148, 0.055)),
        material=dark_steel,
        name="brace_0",
    )
    yoke.visual(
        Box((0.180, 0.026, 0.040)),
        origin=Origin(xyz=(0.090, -0.148, 0.055)),
        material=dark_steel,
        name="brace_1",
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.018, length=0.270),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="trunnion_axle",
    )
    lamp.visual(
        Cylinder(radius=0.100, length=0.220),
        origin=Origin(xyz=(0.055, 0.000, 0.065), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=black,
        name="body",
    )
    lamp.visual(
        Cylinder(radius=0.115, length=0.040),
        origin=Origin(xyz=(0.185, 0.000, 0.065), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.102, length=0.006),
        origin=Origin(xyz=(0.207, 0.000, 0.065), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=lens_glass,
        name="lens",
    )
    lamp.visual(
        Box((0.074, 0.150, 0.118)),
        origin=Origin(xyz=(-0.020, 0.000, 0.065)),
        material=black,
        name="rear_housing",
    )
    lamp.visual(
        Box((0.028, 0.150, 0.028)),
        origin=Origin(xyz=(0.030, 0.000, 0.159)),
        material=dark_steel,
        name="top_handle",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.525)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.100, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    lamp = object_model.get_part("lamp")
    leg_0 = object_model.get_part("leg_0")

    leg_joint = object_model.get_articulation("stand_to_leg_0")
    pan_joint = object_model.get_articulation("stand_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_lamp")

    for index in range(3):
        ctx.expect_gap(
            stand,
            object_model.get_part(f"leg_{index}"),
            axis="z",
            positive_elem="crown",
            negative_elem="foot",
            min_gap=0.045,
            max_gap=0.075,
            name=f"leg_{index} foot sits below the crown in the deployed pose",
        )

    crown_aabb = ctx.part_element_world_aabb(stand, elem="crown")
    foot_aabb = ctx.part_element_world_aabb(leg_0, elem="foot")
    crown_top = crown_aabb[1][2] if crown_aabb is not None else None
    foot_center = _aabb_center(foot_aabb)
    ctx.check(
        "deployed leg reaches a stable studio footprint",
        crown_top is not None
        and foot_center is not None
        and math.hypot(foot_center[0], foot_center[1]) > 0.34
        and 0.0 <= foot_aabb[0][2] <= 0.01,
        details=f"foot_center={foot_center}, foot_aabb={foot_aabb}, crown_top={crown_top}",
    )

    with ctx.pose({leg_joint: 1.35}):
        folded_foot_aabb = ctx.part_element_world_aabb(leg_0, elem="foot")
    ctx.check(
        "folding a leg lifts its foot above the crown",
        crown_top is not None
        and folded_foot_aabb is not None
        and folded_foot_aabb[0][2] > crown_top + 0.05,
        details=f"folded_foot_aabb={folded_foot_aabb}, crown_top={crown_top}",
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(lamp, elem="lens"))
    with ctx.pose({pan_joint: math.pi * 0.5}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(lamp, elem="lens"))
    ctx.check(
        "positive pan swings the beam around the vertical column",
        rest_lens is not None
        and panned_lens is not None
        and rest_lens[0] > 0.16
        and panned_lens[1] > 0.16
        and abs(panned_lens[0]) < 0.05,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt_joint: 0.55}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(lamp, elem="lens"))
    ctx.check(
        "positive tilt raises the front of the lamp in the yoke",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.08
        and tilted_lens[0] < rest_lens[0] - 0.02,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()

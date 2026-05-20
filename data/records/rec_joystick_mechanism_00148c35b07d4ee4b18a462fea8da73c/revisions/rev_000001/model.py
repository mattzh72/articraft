from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pedestal_cardan_joystick")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.21, 0.23, 1.0))
    blue_gray = model.material("blue_gray_housing", rgba=(0.10, 0.16, 0.22, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    bare_steel = model.material("machined_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    safety_plate = model.material("warm_guard_plate", rgba=(0.75, 0.55, 0.18, 1.0))
    grip_rubber = model.material("black_rubber_grip", rgba=(0.012, 0.012, 0.010, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.56, 0.40, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=blue_gray,
        name="base_housing",
    )
    base.visual(
        Box((0.46, 0.30, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=painted_steel,
        name="top_cover",
    )
    base.visual(
        Box((0.66, 0.075, 0.026)),
        origin=Origin(xyz=(0.0, 0.225, 0.013)),
        material=painted_steel,
        name="front_flange",
    )
    base.visual(
        Box((0.66, 0.075, 0.026)),
        origin=Origin(xyz=(0.0, -0.225, 0.013)),
        material=painted_steel,
        name="rear_flange",
    )
    base.visual(
        Box((0.18, 0.082, 0.395)),
        origin=Origin(xyz=(0.0, 0.190, 0.387)),
        material=painted_steel,
        name="front_pedestal",
    )
    base.visual(
        Box((0.18, 0.082, 0.395)),
        origin=Origin(xyz=(0.0, -0.190, 0.387)),
        material=painted_steel,
        name="rear_pedestal",
    )
    base.visual(
        Cylinder(radius=0.061, length=0.028),
        origin=Origin(xyz=(0.0, 0.236, 0.480), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_bearing_cap",
    )
    base.visual(
        Cylinder(radius=0.061, length=0.028),
        origin=Origin(xyz=(0.0, -0.236, 0.480), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing_cap",
    )
    base.visual(
        Box((0.035, 0.250, 0.115)),
        origin=Origin(xyz=(0.325, 0.0, 0.257)),
        material=safety_plate,
        name="guard_plate_0",
    )
    base.visual(
        Box((0.035, 0.250, 0.115)),
        origin=Origin(xyz=(-0.325, 0.0, 0.257)),
        material=safety_plate,
        name="guard_plate_1",
    )
    base.visual(
        Box((0.110, 0.250, 0.030)),
        origin=Origin(xyz=(0.275, 0.0, 0.207)),
        material=safety_plate,
        name="guard_foot_0",
    )
    base.visual(
        Box((0.110, 0.250, 0.030)),
        origin=Origin(xyz=(-0.275, 0.0, 0.207)),
        material=safety_plate,
        name="guard_foot_1",
    )
    for ix, x in enumerate((-0.255, 0.255)):
        for iy, y in enumerate((-0.225, 0.225)):
            base.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x, y, 0.031)),
                material=dark_steel,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        Cylinder(radius=0.023, length=0.298),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="pitch_shaft",
    )
    outer_yoke.visual(
        Box((0.145, 0.125, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=dark_steel,
        name="shaft_saddle",
    )
    outer_yoke.visual(
        Box((0.380, 0.130, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=painted_steel,
        name="lower_bridge",
    )
    outer_yoke.visual(
        Box((0.047, 0.130, 0.345)),
        origin=Origin(xyz=(0.166, 0.0, 0.090)),
        material=painted_steel,
        name="bearing_cheek_0",
    )
    outer_yoke.visual(
        Box((0.047, 0.130, 0.345)),
        origin=Origin(xyz=(-0.166, 0.0, 0.090)),
        material=painted_steel,
        name="bearing_cheek_1",
    )
    outer_yoke.visual(
        Cylinder(radius=0.047, length=0.036),
        origin=Origin(xyz=(0.194, 0.0, 0.130), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_cap_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.047, length=0.036),
        origin=Origin(xyz=(-0.194, 0.0, 0.130), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_cap_1",
    )
    outer_yoke.visual(
        Box((0.030, 0.118, 0.145)),
        origin=Origin(xyz=(0.105, 0.0, 0.015)),
        material=painted_steel,
        name="web_rib_0",
    )
    outer_yoke.visual(
        Box((0.030, 0.118, 0.145)),
        origin=Origin(xyz=(-0.105, 0.0, 0.015)),
        material=painted_steel,
        name="web_rib_1",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.017, length=0.246),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=bare_steel,
        name="roll_shaft",
    )
    inner_cradle.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.1325, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_cap_0",
    )
    inner_cradle.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(-0.1325, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_cap_1",
    )
    inner_cradle.visual(
        Sphere(radius=0.045),
        material=bare_steel,
        name="central_hub",
    )
    inner_cradle.visual(
        Box((0.162, 0.034, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=painted_steel,
        name="cradle_crossbar",
    )
    inner_cradle.visual(
        Box((0.128, 0.026, 0.112)),
        origin=Origin(xyz=(0.0, 0.062, 0.012)),
        material=painted_steel,
        name="cradle_plate_0",
    )
    inner_cradle.visual(
        Box((0.128, 0.026, 0.112)),
        origin=Origin(xyz=(0.0, -0.062, 0.012)),
        material=painted_steel,
        name="cradle_plate_1",
    )
    socket_profile = [
        (0.0, 0.000),
        (0.067, 0.000),
        (0.063, 0.027),
        (0.048, 0.078),
        (0.031, 0.122),
        (0.0, 0.122),
    ]
    inner_cradle.visual(
        mesh_from_geometry(LatheGeometry(socket_profile, segments=48), "lever_socket"),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=dark_steel,
        name="lever_socket",
    )
    lever_profile = [
        (0.0, 0.000),
        (0.030, 0.000),
        (0.024, 0.095),
        (0.014, 0.675),
        (0.018, 0.720),
        (0.0, 0.720),
    ]
    inner_cradle.visual(
        mesh_from_geometry(LatheGeometry(lever_profile, segments=48), "tapered_lever"),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=bare_steel,
        name="tapered_lever",
    )
    inner_cradle.visual(
        Sphere(radius=0.047),
        origin=Origin(xyz=(0.0, 0.0, 0.812)),
        material=grip_rubber,
        name="rubber_grip",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-0.34, upper=0.34),
    )
    model.articulation(
        "yoke_to_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.6, lower=-0.38, upper=0.38),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_cradle = object_model.get_part("inner_cradle")
    pitch = object_model.get_articulation("base_to_yoke")
    roll = object_model.get_articulation("yoke_to_cradle")

    ctx.expect_contact(
        base,
        outer_yoke,
        elem_a="front_pedestal",
        elem_b="pitch_shaft",
        contact_tol=0.001,
        name="front bearing seats pitch shaft",
    )
    ctx.expect_contact(
        outer_yoke,
        base,
        elem_a="pitch_shaft",
        elem_b="rear_pedestal",
        contact_tol=0.001,
        name="rear bearing seats pitch shaft",
    )
    ctx.expect_contact(
        outer_yoke,
        inner_cradle,
        elem_a="bearing_cheek_0",
        elem_b="roll_cap_0",
        contact_tol=0.001,
        name="right cheek seats roll cap",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_yoke,
        elem_a="roll_cap_1",
        elem_b="bearing_cheek_1",
        contact_tol=0.001,
        name="left cheek seats roll cap",
    )

    rest_inner_pos = ctx.part_world_position(inner_cradle)
    rest_lever_aabb = ctx.part_element_world_aabb(inner_cradle, elem="tapered_lever")
    with ctx.pose({pitch: 0.34}):
        ctx.expect_gap(
            outer_yoke,
            base,
            axis="z",
            positive_elem="lower_bridge",
            negative_elem="top_cover",
            min_gap=0.070,
            name="pitched yoke stays above cover",
        )
        pitched_inner_pos = ctx.part_world_position(inner_cradle)
    with ctx.pose({pitch: -0.34}):
        ctx.expect_gap(
            outer_yoke,
            base,
            axis="z",
            positive_elem="lower_bridge",
            negative_elem="top_cover",
            min_gap=0.070,
            name="reverse pitched yoke stays above cover",
        )
    with ctx.pose({roll: 0.38}):
        ctx.expect_gap(
            inner_cradle,
            outer_yoke,
            axis="z",
            positive_elem="lever_socket",
            negative_elem="lower_bridge",
            min_gap=0.045,
            name="rolled socket clears lower bridge",
        )
        rolled_lever_aabb = ctx.part_element_world_aabb(inner_cradle, elem="tapered_lever")

    ctx.check(
        "pitch joint moves inner cradle forward",
        rest_inner_pos is not None
        and pitched_inner_pos is not None
        and pitched_inner_pos[0] > rest_inner_pos[0] + 0.035,
        details=f"rest={rest_inner_pos}, pitched={pitched_inner_pos}",
    )
    ctx.check(
        "roll joint sweeps lever sideways",
        rest_lever_aabb is not None
        and rolled_lever_aabb is not None
        and (rolled_lever_aabb[1][1] - rolled_lever_aabb[0][1])
        > (rest_lever_aabb[1][1] - rest_lever_aabb[0][1]) + 0.18,
        details=f"rest={rest_lever_aabb}, rolled={rolled_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

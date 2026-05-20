from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_wand")

    satin_red = model.material("satin_red", rgba=(0.72, 0.05, 0.04, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.035, 0.038, 0.045, 1.0))
    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.62, 0.62, 0.58, 1.0))
    brushed_tube = model.material("brushed_tube", rgba=(0.78, 0.79, 0.76, 1.0))
    translucent_smoke = model.material("translucent_smoke", rgba=(0.48, 0.62, 0.70, 0.45))
    blue_brush = model.material("blue_brush", rgba=(0.06, 0.22, 0.85, 1.0))

    body = model.part("main_body")
    # The body frame is the upper wand hinge.  The motor, dust cup, handle, and
    # fixed protective guard are all stationary structure carried by this link.
    body.visual(
        Cylinder(radius=0.092, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=translucent_smoke,
        name="dust_cup",
    )
    motor_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.135, length=0.17, radial_segments=32, height_segments=8).rotate_x(
            -math.pi / 2
        ),
        "motor_housing",
    )
    body.visual(
        motor_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=satin_red,
        name="motor_housing",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.11),
        origin=Origin(xyz=(0.105, 0.0, 0.39), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_graphite,
        name="rear_filter_cap",
    )
    body.visual(
        Box((0.070, 0.200, 0.070)),
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        material=dark_graphite,
        name="socket_bridge",
    )
    for side, y in (("0", -0.061), ("1", 0.061)):
        body.visual(
            Box((0.100, 0.032, 0.060)),
            origin=Origin(xyz=(0.015, y, -0.005)),
            material=dark_graphite,
            name=f"socket_lug_{side}",
        )
    handle = wire_from_points(
        [
            (-0.115, 0.0, 0.300),
            (-0.250, 0.0, 0.420),
            (-0.220, 0.0, 0.610),
            (-0.045, 0.0, 0.500),
        ],
        radius=0.015,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.050,
    )
    body.visual(
        mesh_from_geometry(handle, "rear_handle"),
        material=dark_graphite,
        name="rear_handle",
    )
    body.visual(
        Box((0.045, 0.026, 0.020)),
        origin=Origin(xyz=(-0.177, 0.0, 0.345), rpy=(0.0, 0.0, -0.35)),
        material=blue_brush,
        name="power_trigger",
    )
    guard = wire_from_points(
        [
            (-0.060, -0.100, 0.018),
            (0.070, -0.098, -0.050),
            (0.072, -0.098, -0.360),
            (0.072, 0.098, -0.360),
            (0.070, 0.098, -0.050),
            (-0.060, 0.100, 0.018),
        ],
        radius=0.007,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.030,
    )
    body.visual(
        mesh_from_geometry(guard, "guard_hoop"),
        material=warm_gray,
        name="guard_hoop",
    )

    wand_0 = model.part("wand_0")
    wand_0.visual(
        Cylinder(radius=0.022, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, -0.213)),
        material=brushed_tube,
        name="upper_tube",
    )
    wand_0.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_graphite,
        name="upper_hinge_pin",
    )
    wand_0.visual(
        Cylinder(radius=0.027, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
        material=dark_graphite,
        name="lower_collar",
    )
    for side, y in (("0", -0.036), ("1", 0.036)):
        wand_0.visual(
            Box((0.055, 0.026, 0.070)),
            origin=Origin(xyz=(0.0, y, -0.430)),
            material=dark_graphite,
            name=f"elbow_yoke_{side}",
        )

    wand_1 = model.part("wand_1")
    wand_1.visual(
        Cylinder(radius=0.017, length=0.046),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_graphite,
        name="elbow_hub",
    )
    wand_1.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=dark_graphite,
        name="elbow_neck",
    )
    wand_1.visual(
        Cylinder(radius=0.020, length=0.450),
        origin=Origin(xyz=(0.0, 0.0, -0.2425)),
        material=brushed_tube,
        name="lower_tube",
    )
    wand_1.visual(
        Box((0.060, 0.130, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.465)),
        material=dark_graphite,
        name="nozzle_bridge",
    )
    for side, y in (("0", -0.047), ("1", 0.047)):
        wand_1.visual(
            Box((0.060, 0.034, 0.070)),
            origin=Origin(xyz=(0.0, y, -0.495)),
            material=dark_graphite,
            name=f"nozzle_yoke_{side}",
        )

    nozzle = model.part("floor_nozzle")
    nozzle_shell = ExtrudeGeometry(
        rounded_rect_profile(0.300, 0.420, 0.040, corner_segments=8),
        0.075,
        center=True,
    )
    nozzle.visual(
        mesh_from_geometry(nozzle_shell, "nozzle_shell"),
        origin=Origin(xyz=(0.100, 0.0, -0.065)),
        material=warm_gray,
        name="nozzle_shell",
    )
    nozzle.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_graphite,
        name="pitch_hinge_barrel",
    )
    nozzle.visual(
        Box((0.050, 0.052, 0.026)),
        origin=Origin(xyz=(0.010, 0.0, -0.031)),
        material=dark_graphite,
        name="hinge_saddle",
    )
    nozzle.visual(
        Box((0.032, 0.438, 0.035)),
        origin=Origin(xyz=(0.235, 0.0, -0.035)),
        material=matte_black,
        name="front_bumper",
    )
    nozzle.visual(
        Box((0.040, 0.330, 0.022)),
        origin=Origin(xyz=(0.125, 0.0, -0.107)),
        material=blue_brush,
        name="brush_strip",
    )
    for side, y in (("0", -0.183), ("1", 0.183)):
        nozzle.visual(
            Box((0.052, 0.045, 0.022)),
            origin=Origin(xyz=(-0.015, y, -0.111)),
            material=matte_black,
            name=f"rear_roller_{side}",
        )

    model.articulation(
        "body_to_wand_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand_0,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "wand_0_to_wand_1",
        ArticulationType.REVOLUTE,
        parent=wand_0,
        child=wand_1,
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "wand_1_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=wand_1,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, -0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("main_body")
    wand_0 = object_model.get_part("wand_0")
    wand_1 = object_model.get_part("wand_1")
    nozzle = object_model.get_part("floor_nozzle")
    body_joint = object_model.get_articulation("body_to_wand_0")
    elbow_joint = object_model.get_articulation("wand_0_to_wand_1")
    nozzle_joint = object_model.get_articulation("wand_1_to_nozzle")

    ctx.check(
        "wand has two elbow revolutes and nozzle pitch",
        body_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and nozzle_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"types={[body_joint.articulation_type, elbow_joint.articulation_type, nozzle_joint.articulation_type]}",
    )
    ctx.expect_contact(
        body,
        wand_0,
        elem_a="socket_lug_0",
        elem_b="upper_hinge_pin",
        contact_tol=0.001,
        name="upper hinge pin is captured by body yoke",
    )
    ctx.expect_contact(
        wand_0,
        wand_1,
        elem_a="elbow_yoke_0",
        elem_b="elbow_hub",
        contact_tol=0.014,
        name="middle elbow hardware is colocated",
    )
    ctx.expect_contact(
        wand_1,
        nozzle,
        elem_a="nozzle_yoke_0",
        elem_b="pitch_hinge_barrel",
        contact_tol=0.018,
        name="nozzle pitch hinge is captured by yoke",
    )

    guard_box = ctx.part_element_world_aabb(body, elem="guard_hoop")
    wand_box = ctx.part_element_world_aabb(wand_0, elem="upper_tube")
    ctx.check(
        "fixed guard surrounds first moving stage at rest",
        guard_box is not None
        and wand_box is not None
        and guard_box[0][1] < wand_box[0][1] - 0.035
        and guard_box[1][1] > wand_box[1][1] + 0.035
        and guard_box[0][2] < wand_box[1][2] - 0.25
        and guard_box[1][2] > wand_box[1][2] - 0.05,
        details=f"guard={guard_box}, wand={wand_box}",
    )

    rest_nozzle = ctx.part_world_aabb(nozzle)
    with ctx.pose({body_joint: 0.25, elbow_joint: -0.45, nozzle_joint: 0.45}):
        posed_nozzle = ctx.part_world_aabb(nozzle)
    ctx.check(
        "articulated wand changes floor nozzle pose",
        rest_nozzle is not None
        and posed_nozzle is not None
        and abs(posed_nozzle[1][2] - rest_nozzle[1][2]) > 0.08,
        details=f"rest={rest_nozzle}, posed={posed_nozzle}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float, name: str):
    """CadQuery rounded rectangular solid, used for the armored turret shell."""
    body = cq.Workplane("XY").box(*size).edges().fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.0015, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilized_weapon_station")

    olive = Material("matte_olive_drab", rgba=(0.22, 0.27, 0.19, 1.0))
    dark = Material("dark_phosphate", rgba=(0.055, 0.060, 0.060, 1.0))
    rubber = Material("flat_black_rubber", rgba=(0.012, 0.012, 0.010, 1.0))
    glass = Material("smoked_sensor_glass", rgba=(0.02, 0.05, 0.075, 0.86))
    brass = Material("worn_bronze_bearing", rgba=(0.45, 0.35, 0.16, 1.0))

    # Fixed lower structure: a square deck plate and broad rotating bearing.
    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Box((1.05, 0.86, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark,
        name="base_plate",
    )
    yaw_base.visual(
        Cylinder(radius=0.36, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=olive,
        name="pedestal_drum",
    )
    yaw_base.visual(
        Cylinder(radius=0.43, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=brass,
        name="yaw_bearing",
    )
    for i, (x, y) in enumerate(
        ((0.42, 0.32), (-0.42, 0.32), (0.42, -0.32), (-0.42, -0.32))
    ):
        yaw_base.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x, y, 0.069)),
            material=rubber,
            name=f"deck_bolt_{i}",
        )

    # Rotating turret: broad armored shell, trunnion cheeks, and rounded sensor pod.
    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.30, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=olive,
        name="yaw_skirt",
    )
    turret.visual(
        _rounded_box((0.86, 0.62, 0.32), 0.055, "rounded_turret_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=olive,
        name="armored_shell",
    )
    turret.visual(
        Box((0.34, 0.14, 0.25)),
        origin=Origin(xyz=(0.45, 0.37, 0.28)),
        material=olive,
        name="trunnion_cheek_0",
    )
    turret.visual(
        Box((0.34, 0.14, 0.25)),
        origin=Origin(xyz=(0.45, -0.37, 0.28)),
        material=olive,
        name="trunnion_cheek_1",
    )
    turret.visual(
        Cylinder(radius=0.12, length=0.035),
        origin=Origin(xyz=(0.54, 0.4575, 0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pitch_bearing_0",
    )
    turret.visual(
        Cylinder(radius=0.12, length=0.035),
        origin=Origin(xyz=(0.54, -0.4575, 0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pitch_bearing_1",
    )
    turret.visual(
        Box((0.28, 0.05, 0.13)),
        origin=Origin(xyz=(-0.18, 0.335, 0.22)),
        material=dark,
        name="side_access_panel",
    )
    turret.visual(
        Box((0.30, 0.18, 0.035)),
        origin=Origin(xyz=(-0.10, 0.0, 0.397)),
        material=dark,
        name="top_hatch",
    )
    turret.visual(
        Box((0.16, 0.12, 0.13)),
        origin=Origin(xyz=(0.04, 0.29, 0.435)),
        material=olive,
        name="sensor_pylon",
    )
    turret.visual(
        Cylinder(radius=0.115, length=0.29),
        origin=Origin(xyz=(0.10, 0.29, 0.555), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="sensor_pod_body",
    )
    turret.visual(
        Sphere(radius=0.115),
        origin=Origin(xyz=(-0.045, 0.29, 0.555)),
        material=dark,
        name="sensor_pod_rear",
    )
    turret.visual(
        Sphere(radius=0.115),
        origin=Origin(xyz=(0.245, 0.29, 0.555)),
        material=dark,
        name="sensor_pod_nose",
    )
    turret.visual(
        Cylinder(radius=0.078, length=0.014),
        origin=Origin(xyz=(0.358, 0.29, 0.555), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="sensor_lens",
    )

    # Pitching cradle and gun: all geometry is local to the pitch axis.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.065, length=0.60),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="trunnion_tube",
    )
    cradle.visual(
        Box((0.22, 0.22, 0.14)),
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
        material=olive,
        name="cradle_block",
    )
    cradle.visual(
        Box((0.50, 0.12, 0.085)),
        origin=Origin(xyz=(0.39, 0.0, -0.07)),
        material=olive,
        name="lower_slide",
    )
    cradle.visual(
        Box((0.18, 0.08, 0.10)),
        origin=Origin(xyz=(0.24, 0.0, -0.02)),
        material=olive,
        name="slide_web",
    )
    cradle.visual(
        Cylinder(radius=0.046, length=0.82),
        origin=Origin(xyz=(0.56, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.056, length=0.13),
        origin=Origin(xyz=(0.99, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="muzzle",
    )
    cradle.visual(
        Box((0.16, 0.14, 0.06)),
        origin=Origin(xyz=(0.18, -0.18, 0.03)),
        material=olive,
        name="guard_hinge_mount",
    )

    # Separate folding guard arm on its own small side hinge.
    guard_arm = model.part("guard_arm")
    guard_arm.visual(
        Cylinder(radius=0.030, length=0.11),
        origin=Origin(),
        material=brass,
        name="hinge_barrel",
    )
    guard_arm.visual(
        Box((0.59, 0.035, 0.040)),
        origin=Origin(xyz=(0.322, 0.0, 0.0)),
        material=olive,
        name="arm_bar",
    )
    guard_arm.visual(
        Cylinder(radius=0.033, length=0.060),
        origin=Origin(xyz=(0.625, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rounded_tip",
    )

    model.articulation(
        "yaw",
        ArticulationType.CONTINUOUS,
        parent=yaw_base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.54, 0.0, 0.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=-0.35, upper=0.55),
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=guard_arm,
        origin=Origin(xyz=(0.18, -0.28, 0.03)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yaw_base = object_model.get_part("yaw_base")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    guard_arm = object_model.get_part("guard_arm")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    guard_hinge = object_model.get_articulation("guard_hinge")

    ctx.check(
        "yaw joint is vertical",
        abs(yaw.axis[2]) > 0.99,
        details=f"yaw axis={yaw.axis}",
    )
    ctx.check(
        "pitch joint is horizontal",
        abs(pitch.axis[1]) > 0.99 and abs(pitch.axis[2]) < 0.01,
        details=f"pitch axis={pitch.axis}",
    )
    ctx.check(
        "guard has its own side hinge",
        abs(guard_hinge.axis[2]) > 0.99,
        details=f"guard axis={guard_hinge.axis}",
    )

    ctx.expect_gap(
        turret,
        yaw_base,
        axis="z",
        positive_elem="yaw_skirt",
        negative_elem="yaw_bearing",
        max_gap=0.003,
        max_penetration=0.0,
        name="turret sits on yaw bearing",
    )
    ctx.expect_contact(
        guard_arm,
        cradle,
        elem_a="hinge_barrel",
        elem_b="guard_hinge_mount",
        contact_tol=0.003,
        name="guard hinge barrel is mounted beside cradle",
    )

    rest_cradle = ctx.part_world_position(cradle)
    with ctx.pose({yaw: 0.80}):
        yawed_cradle = ctx.part_world_position(cradle)
    ctx.check(
        "yaw rotates the forward cradle around the base",
        rest_cradle is not None
        and yawed_cradle is not None
        and abs(yawed_cradle[1] - rest_cradle[1]) > 0.20,
        details=f"rest={rest_cradle}, yawed={yawed_cradle}",
    )

    rest_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle")
    with ctx.pose({pitch: 0.50}):
        raised_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle")
    rest_muzzle_z = (rest_muzzle[0][2] + rest_muzzle[1][2]) / 2.0 if rest_muzzle else None
    raised_muzzle_z = (
        (raised_muzzle[0][2] + raised_muzzle[1][2]) / 2.0 if raised_muzzle else None
    )
    ctx.check(
        "positive pitch elevates the gun cradle",
        rest_muzzle_z is not None
        and raised_muzzle_z is not None
        and raised_muzzle_z > rest_muzzle_z + 0.15,
        details=f"rest_z={rest_muzzle_z}, raised_z={raised_muzzle_z}",
    )

    rest_arm = ctx.part_element_world_aabb(guard_arm, elem="arm_bar")
    with ctx.pose({guard_hinge: 1.10}):
        folded_arm = ctx.part_element_world_aabb(guard_arm, elem="arm_bar")
    rest_arm_y = (rest_arm[0][1] + rest_arm[1][1]) / 2.0 if rest_arm else None
    folded_arm_y = (folded_arm[0][1] + folded_arm[1][1]) / 2.0 if folded_arm else None
    ctx.check(
        "guard arm folds outward from the cradle side",
        rest_arm_y is not None
        and folded_arm_y is not None
        and folded_arm_y < rest_arm_y - 0.18,
        details=f"rest_y={rest_arm_y}, folded_y={folded_arm_y}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

HERE = Path(__file__).resolve().parent

LEG_BARREL_RADIUS = 0.015
LEG_BARREL_LENGTH = 0.050
LEG_LENGTH = 0.48
LEG_DROP = math.radians(17.5)
LEG_FOOT_RADIUS = 0.019
LEG_JOINT_Z = 0.16325027138372117
LEG_SOCKET_RADIUS = 0.072
ARM_HINGE_RADIUS = 0.016
POLE_RADIUS = 0.024


def _radial_xyz(radius: float, yaw: float, z: float, tangential: float = 0.0) -> tuple[float, float, float]:
    return (
        radius * math.cos(yaw) - tangential * math.sin(yaw),
        radius * math.sin(yaw) + tangential * math.cos(yaw),
        z,
    )


def _add_leg_geometry(part, tube_material, foot_material) -> None:
    foot_x = LEG_BARREL_RADIUS + LEG_LENGTH * math.cos(LEG_DROP)
    foot_z = -LEG_LENGTH * math.sin(LEG_DROP)
    tube_center_x = LEG_BARREL_RADIUS + 0.5 * LEG_LENGTH * math.cos(LEG_DROP)
    tube_center_z = -0.5 * LEG_LENGTH * math.sin(LEG_DROP)

    part.visual(
        Cylinder(radius=LEG_BARREL_RADIUS, length=LEG_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tube_material,
        name="hinge_barrel",
    )
    part.visual(
        Cylinder(radius=0.0115, length=LEG_LENGTH),
        origin=Origin(
            xyz=(tube_center_x, 0.0, tube_center_z),
            rpy=(0.0, math.pi / 2.0 + LEG_DROP, 0.0),
        ),
        material=tube_material,
        name="leg_tube",
    )
    part.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(LEG_BARREL_RADIUS + 0.09, 0.0, -0.03)),
        material=tube_material,
        name="knee_cap",
    )
    part.visual(
        Sphere(radius=LEG_FOOT_RADIUS),
        origin=Origin(xyz=(foot_x, 0.0, foot_z)),
        material=foot_material,
        name="foot_cap",
    )


def _add_leg_socket(base, prefix: str, yaw: float, material) -> None:
    lug_center = _radial_xyz(
        LEG_SOCKET_RADIUS - (LEG_BARREL_RADIUS + 0.009),
        yaw,
        LEG_JOINT_Z,
    )
    base.visual(
        Box((0.018, 0.058, 0.050)),
        origin=Origin(xyz=lug_center, rpy=(0.0, 0.0, yaw)),
        material=material,
        name=f"{prefix}_socket",
    )
    gusset_center = _radial_xyz(0.041, yaw, 0.126)
    base.visual(
        Box((0.042, 0.028, 0.028)),
        origin=Origin(xyz=gusset_center, rpy=(0.0, 0.0, yaw)),
        material=material,
        name=f"{prefix}_gusset",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pole_lamp")

    matte_black = model.material("matte_black", rgba=(0.14, 0.15, 0.16, 1.0))
    warm_white = model.material("warm_white", rgba=(0.89, 0.88, 0.84, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.86, 0.90, 0.94, 0.35))
    brass = model.material("brass", rgba=(0.70, 0.60, 0.34, 1.0))
    charcoal_rubber = model.material("charcoal_rubber", rgba=(0.18, 0.18, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.042, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=matte_black,
        name="sleeve_shell",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=matte_black,
        name="tripod_crown",
    )
    base.visual(
        Box((0.016, 0.096, 0.136)),
        origin=Origin(xyz=(-0.032, 0.0, 0.285)),
        material=matte_black,
        name="collar_cheek_left",
    )
    base.visual(
        Box((0.016, 0.096, 0.136)),
        origin=Origin(xyz=(0.032, 0.0, 0.285)),
        material=matte_black,
        name="collar_cheek_right",
    )
    base.visual(
        Box((0.050, 0.016, 0.136)),
        origin=Origin(xyz=(0.0, -0.032, 0.285)),
        material=matte_black,
        name="collar_backstrap",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.062),
        origin=Origin(xyz=(0.055, 0.0, 0.280), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="thumbscrew_shaft",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.097, 0.0, 0.280), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="thumbscrew_knob",
    )
    _add_leg_socket(base, "front_leg", 0.0, matte_black)
    _add_leg_socket(base, "left_leg", 2.0 * math.pi / 3.0, matte_black)
    _add_leg_socket(base, "right_leg", -2.0 * math.pi / 3.0, matte_black)

    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=POLE_RADIUS, length=1.520),
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        material=warm_white,
        name="pole_tube",
    )
    pole.visual(
        Cylinder(radius=0.035, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        material=matte_black,
        name="arm_collar_ring",
    )
    pole.visual(
        Box((0.014, 0.086, 0.100)),
        origin=Origin(xyz=(-0.031, 0.0, 0.960)),
        material=matte_black,
        name="arm_collar_left",
    )
    pole.visual(
        Box((0.014, 0.086, 0.100)),
        origin=Origin(xyz=(0.028, 0.0, 0.960)),
        material=matte_black,
        name="arm_collar_right",
    )
    pole.visual(
        Box((0.046, 0.014, 0.100)),
        origin=Origin(xyz=(0.0, -0.030, 0.960)),
        material=matte_black,
        name="arm_collar_backstrap",
    )
    pole.visual(
        Box((0.016, 0.064, 0.052)),
        origin=Origin(xyz=(0.028, 0.0, 0.960)),
        material=satin_steel,
        name="arm_pivot_lug",
    )

    leg_front = model.part("leg_front")
    leg_left = model.part("leg_left")
    leg_right = model.part("leg_right")
    _add_leg_geometry(leg_front, matte_black, charcoal_rubber)
    _add_leg_geometry(leg_left, matte_black, charcoal_rubber)
    _add_leg_geometry(leg_right, matte_black, charcoal_rubber)

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=ARM_HINGE_RADIUS, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="arm_hinge_barrel",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.462),
        origin=Origin(xyz=(0.252, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="extension_tube",
    )
    arm.visual(
        Box((0.034, 0.034, 0.034)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=satin_steel,
        name="arm_root_block",
    )
    arm.visual(
        Box((0.054, 0.050, 0.036)),
        origin=Origin(xyz=(0.449, 0.0, 0.0)),
        material=matte_black,
        name="yoke_core",
    )
    arm.visual(
        Box((0.022, 0.016, 0.118)),
        origin=Origin(xyz=(0.494, 0.080, 0.0)),
        material=matte_black,
        name="yoke_left",
    )
    arm.visual(
        Box((0.022, 0.016, 0.118)),
        origin=Origin(xyz=(0.494, -0.080, 0.0)),
        material=matte_black,
        name="yoke_right",
    )
    arm.visual(
        Box((0.074, 0.050, 0.018)),
        origin=Origin(xyz=(0.446, 0.049, 0.0)),
        material=matte_black,
        name="yoke_link_left",
    )
    arm.visual(
        Box((0.074, 0.050, 0.018)),
        origin=Origin(xyz=(0.446, -0.049, 0.0)),
        material=matte_black,
        name="yoke_link_right",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.010, length=0.144),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pivot_barrel",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.050),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="neck",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.044),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.090, length=0.096),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="head_housing",
    )
    head.visual(
        Cylinder(radius=0.094, length=0.014),
        origin=Origin(xyz=(0.163, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="bezel_ring",
    )
    head.visual(
        Cylinder(radius=0.080, length=0.008),
        origin=Origin(xyz=(0.174, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_glass,
        name="lens",
    )

    leg_limits = MotionLimits(effort=40.0, velocity=1.2, lower=-0.12, upper=0.18)
    model.articulation(
        "front_leg_spread",
        ArticulationType.REVOLUTE,
        parent=base,
        child=leg_front,
        origin=Origin(xyz=_radial_xyz(LEG_SOCKET_RADIUS, 0.0, LEG_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=leg_limits,
    )
    model.articulation(
        "left_leg_spread",
        ArticulationType.REVOLUTE,
        parent=base,
        child=leg_left,
        origin=Origin(
            xyz=_radial_xyz(LEG_SOCKET_RADIUS, 2.0 * math.pi / 3.0, LEG_JOINT_Z),
            rpy=(0.0, 0.0, 2.0 * math.pi / 3.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=leg_limits,
    )
    model.articulation(
        "right_leg_spread",
        ArticulationType.REVOLUTE,
        parent=base,
        child=leg_right,
        origin=Origin(
            xyz=_radial_xyz(LEG_SOCKET_RADIUS, -2.0 * math.pi / 3.0, LEG_JOINT_Z),
            rpy=(0.0, 0.0, -2.0 * math.pi / 3.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=leg_limits,
    )
    model.articulation(
        "pole_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pole,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.18),
    )
    model.articulation(
        "arm_pitch",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=arm,
        origin=Origin(xyz=(0.052, 0.0, 0.960)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.1, lower=-0.45, upper=0.85),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.495, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.95, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base")
    pole = object_model.get_part("pole")
    leg_front = object_model.get_part("leg_front")
    leg_left = object_model.get_part("leg_left")
    leg_right = object_model.get_part("leg_right")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")

    front_leg_spread = object_model.get_articulation("front_leg_spread")
    left_leg_spread = object_model.get_articulation("left_leg_spread")
    right_leg_spread = object_model.get_articulation("right_leg_spread")
    pole_slide = object_model.get_articulation("pole_slide")
    arm_pitch = object_model.get_articulation("arm_pitch")
    head_tilt = object_model.get_articulation("head_tilt")

    sleeve_shell = base.get_visual("sleeve_shell")
    tripod_crown = base.get_visual("tripod_crown")
    thumbscrew_shaft = base.get_visual("thumbscrew_shaft")
    front_socket = base.get_visual("front_leg_socket")
    left_socket = base.get_visual("left_leg_socket")
    right_socket = base.get_visual("right_leg_socket")
    pole_tube = pole.get_visual("pole_tube")
    arm_pivot_lug = pole.get_visual("arm_pivot_lug")
    front_barrel = leg_front.get_visual("hinge_barrel")
    left_barrel = leg_left.get_visual("hinge_barrel")
    right_barrel = leg_right.get_visual("hinge_barrel")
    front_foot = leg_front.get_visual("foot_cap")
    left_foot = leg_left.get_visual("foot_cap")
    right_foot = leg_right.get_visual("foot_cap")
    arm_hinge_barrel = arm.get_visual("arm_hinge_barrel")
    yoke_left = arm.get_visual("yoke_left")
    yoke_right = arm.get_visual("yoke_right")
    pivot_barrel = head.get_visual("pivot_barrel")
    head_housing = head.get_visual("head_housing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(base, pole, reason="telescoping pole intentionally nests inside the sleeve collar")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(leg_front, base, elem_a=front_barrel, elem_b=front_socket)
    ctx.expect_contact(leg_left, base, elem_a=left_barrel, elem_b=left_socket)
    ctx.expect_contact(leg_right, base, elem_a=right_barrel, elem_b=right_socket)
    ctx.expect_gap(
        leg_front,
        base,
        axis="x",
        min_gap=0.42,
        positive_elem=front_foot,
        negative_elem=tripod_crown,
    )
    ctx.expect_gap(
        leg_left,
        base,
        axis="y",
        min_gap=0.36,
        positive_elem=left_foot,
        negative_elem=tripod_crown,
    )
    ctx.expect_gap(
        base,
        leg_right,
        axis="y",
        min_gap=0.36,
        positive_elem=tripod_crown,
        negative_elem=right_foot,
    )

    ctx.expect_origin_distance(pole, base, axes="xy", max_dist=0.002)
    ctx.expect_overlap(pole, base, axes="xy", min_overlap=0.045, elem_a=pole_tube, elem_b=sleeve_shell)
    ctx.expect_contact(base, pole, elem_a=thumbscrew_shaft, elem_b=pole_tube)

    ctx.expect_contact(arm, pole, elem_a=arm_hinge_barrel, elem_b=arm_pivot_lug)
    ctx.expect_gap(
        head,
        arm,
        axis="x",
        min_gap=0.02,
        positive_elem=head_housing,
        negative_elem=yoke_left,
    )
    ctx.expect_contact(head, arm, elem_a=pivot_barrel, elem_b=yoke_left)
    ctx.expect_contact(head, arm, elem_a=pivot_barrel, elem_b=yoke_right)

    with ctx.pose({front_leg_spread: 0.12, left_leg_spread: 0.12, right_leg_spread: 0.12}):
        ctx.expect_gap(
            leg_front,
            base,
            axis="x",
            min_gap=0.45,
            positive_elem=front_foot,
            negative_elem=tripod_crown,
        )
        ctx.expect_gap(
            leg_left,
            base,
            axis="y",
            min_gap=0.38,
            positive_elem=left_foot,
            negative_elem=tripod_crown,
        )

    with ctx.pose({pole_slide: 0.16}):
        ctx.expect_overlap(pole, base, axes="xy", min_overlap=0.045, elem_a=pole_tube, elem_b=sleeve_shell)
        ctx.expect_contact(base, pole, elem_a=thumbscrew_shaft, elem_b=pole_tube)

    with ctx.pose({arm_pitch: 0.55, head_tilt: -0.50}):
        ctx.expect_contact(arm, pole, elem_a=arm_hinge_barrel, elem_b=arm_pivot_lug)
        ctx.expect_contact(head, arm, elem_a=pivot_barrel, elem_b=yoke_left)
        ctx.expect_contact(head, arm, elem_a=pivot_barrel, elem_b=yoke_right)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

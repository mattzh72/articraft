from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        fallback = "/tmp" if os.path.isdir("/tmp") else "/"
        os.chdir(fallback)
        return fallback


os.getcwd = _safe_getcwd
os.getcwd()

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, name: str, a, b, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_shade_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.022, 0.000),
            (0.034, 0.018),
            (0.055, 0.070),
            (0.079, 0.154),
            (0.104, 0.252),
        ],
        [
            (0.014, 0.008),
            (0.026, 0.022),
            (0.047, 0.074),
            (0.071, 0.156),
            (0.097, 0.246),
        ],
        segments=64,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_floor_lamp", assets=ASSETS)

    dark_wood = model.material("dark_wood", rgba=(0.34, 0.24, 0.17, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.16, 0.17, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.63, 0.51, 0.31, 1.0))
    linen = model.material("linen", rgba=(0.89, 0.86, 0.79, 1.0))
    diffuser = model.material("diffuser", rgba=(0.98, 0.94, 0.84, 0.55))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=dark_wood,
        name="hub_body",
    )
    stand.visual(
        Cylinder(radius=0.023, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=warm_brass,
        name="hub_collar",
    )
    stand.visual(
        Cylinder(radius=0.015, length=0.99),
        origin=Origin(xyz=(0.0, 0.0, 1.035)),
        material=satin_black,
        name="pole",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.525)),
        material=warm_brass,
        name="pole_tip",
    )
    stand.visual(
        Box((0.026, 0.032, 0.010)),
        origin=Origin(xyz=(0.009, 0.0, 1.541)),
        material=warm_brass,
        name="yoke_arm",
    )
    stand.visual(
        Box((0.020, 0.008, 0.030)),
        origin=Origin(xyz=(0.032, 0.020, 1.555)),
        material=warm_brass,
        name="left_cheek",
    )
    stand.visual(
        Box((0.020, 0.008, 0.030)),
        origin=Origin(xyz=(0.032, -0.020, 1.555)),
        material=warm_brass,
        name="right_cheek",
    )

    leg_angles = {
        "front": 0.0,
        "rear_left": 2.0 * math.pi / 3.0,
        "rear_right": 4.0 * math.pi / 3.0,
    }
    for leg_name, angle in leg_angles.items():
        top = (0.028 * math.cos(angle), 0.028 * math.sin(angle), 0.415)
        foot = (0.280 * math.cos(angle), 0.280 * math.sin(angle), 0.018)
        _add_member(stand, f"{leg_name}_leg", top, foot, 0.0125, dark_wood)
        stand.visual(
            Cylinder(radius=0.028, length=0.020),
            origin=Origin(xyz=(foot[0], foot[1], 0.010)),
            material=satin_black,
            name=f"{leg_name}_foot",
        )

    stand.inertial = Inertial.from_geometry(
        Box((0.64, 0.64, 1.60)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
    )

    head = model.part("shade_head")
    head.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.044),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="neck",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linen,
        name="shade_rear",
    )
    head.visual(
        _save_mesh("tripod_floor_lamp_shade_shell.obj", _build_shade_shell()),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linen,
        name="shade_shell",
    )
    head.visual(
        Cylinder(radius=0.109, length=0.012),
        origin=Origin(xyz=(0.312, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="shade_rim",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.066),
        origin=Origin(xyz=(0.101, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="socket",
    )
    head.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.134, 0.0, 0.0)),
        material=diffuser,
        name="lamp_bulb",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.24, 0.24)),
        mass=0.9,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.032, 0.0, 1.558)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.65,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    stand = object_model.get_part("stand")
    head = object_model.get_part("shade_head")
    head_tilt = object_model.get_articulation("head_tilt")

    hub_body = stand.get_visual("hub_body")
    pole = stand.get_visual("pole")
    yoke_arm = stand.get_visual("yoke_arm")
    left_cheek = stand.get_visual("left_cheek")
    right_cheek = stand.get_visual("right_cheek")
    front_leg = stand.get_visual("front_leg")
    front_foot = stand.get_visual("front_foot")
    rear_left_foot = stand.get_visual("rear_left_foot")
    rear_right_foot = stand.get_visual("rear_right_foot")

    hinge_barrel = head.get_visual("hinge_barrel")
    neck = head.get_visual("neck")
    shade_rear = head.get_visual("shade_rear")
    shade_shell = head.get_visual("shade_shell")
    shade_rim = head.get_visual("shade_rim")
    lamp_bulb = head.get_visual("lamp_bulb")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)

    ctx.expect_contact(
        stand,
        stand,
        elem_a=front_leg,
        elem_b=hub_body,
        name="front tripod leg meets central hub",
    )
    ctx.expect_contact(
        head,
        head,
        elem_a=neck,
        elem_b=shade_rear,
        name="neck connects into rear shade collar",
    )

    ctx.expect_gap(
        stand,
        stand,
        axis="x",
        positive_elem=front_foot,
        negative_elem=rear_left_foot,
        min_gap=0.34,
        name="tripod stance spreads front foot ahead of rear foot",
    )
    ctx.expect_gap(
        stand,
        stand,
        axis="y",
        positive_elem=rear_left_foot,
        negative_elem=rear_right_foot,
        min_gap=0.40,
        name="rear tripod feet stay widely splayed",
    )
    ctx.expect_gap(
        head,
        stand,
        axis="z",
        positive_elem=shade_shell,
        negative_elem=hub_body,
        min_gap=0.80,
        name="shade head sits well above the central hub",
    )
    ctx.expect_gap(
        head,
        stand,
        axis="x",
        positive_elem=lamp_bulb,
        negative_elem=pole,
        min_gap=0.08,
        name="shade projects forward of the pole",
    )
    ctx.expect_gap(
        stand,
        stand,
        axis="z",
        positive_elem=hub_body,
        negative_elem=front_foot,
        min_gap=0.36,
        name="tripod feet sit far below the hub",
    )
    ctx.expect_contact(
        head,
        stand,
        elem_a=hinge_barrel,
        elem_b=left_cheek,
        name="hinge barrel seats against left yoke cheek",
    )
    ctx.expect_contact(
        head,
        stand,
        elem_a=hinge_barrel,
        elem_b=right_cheek,
        name="hinge barrel seats against right yoke cheek",
    )

    limits = head_tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        shade_rim_rest = ctx.part_element_world_aabb(head, elem="shade_rim")
        if shade_rim_rest is None:
            ctx.fail("shade_rim_rest_aabb_available", "shade rim world AABB could not be resolved")

        with ctx.pose({head_tilt: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_lower_no_floating")
            ctx.expect_gap(
                head,
                stand,
                axis="z",
                positive_elem=shade_rim,
                negative_elem=yoke_arm,
                min_gap=0.08,
                name="shade clears the yoke when tilted upward",
            )
            ctx.expect_contact(
                head,
                stand,
                elem_a=hinge_barrel,
                elem_b=left_cheek,
                name="hinge stays seated at upward tilt",
            )
            shade_rim_up = ctx.part_element_world_aabb(head, elem="shade_rim")
            if shade_rim_rest is not None and shade_rim_up is not None:
                ctx.check(
                    "negative tilt raises shade rim",
                    shade_rim_up[1][2] > shade_rim_rest[1][2] + 0.09,
                    details=(
                        f"upper rim z did not rise enough: rest={shade_rim_rest[1][2]:.3f}, "
                        f"up={shade_rim_up[1][2]:.3f}"
                    ),
                )

        with ctx.pose({head_tilt: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_upper_no_floating")
            ctx.expect_gap(
                stand,
                head,
                axis="z",
                positive_elem=yoke_arm,
                negative_elem=shade_rim,
                min_gap=0.02,
                name="shade can dip downward below the yoke arm",
            )
            ctx.expect_contact(
                head,
                stand,
                elem_a=hinge_barrel,
                elem_b=right_cheek,
                name="hinge stays seated at downward tilt",
            )
            shade_rim_down = ctx.part_element_world_aabb(head, elem="shade_rim")
            if shade_rim_rest is not None and shade_rim_down is not None:
                ctx.check(
                    "positive tilt lowers shade rim",
                    shade_rim_down[0][2] < shade_rim_rest[0][2] - 0.07,
                    details=(
                        f"lower rim z did not drop enough: rest={shade_rim_rest[0][2]:.3f}, "
                        f"down={shade_rim_down[0][2]:.3f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

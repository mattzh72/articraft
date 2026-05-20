from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except Exception:
            pass
        return "/"


os.getcwd = _safe_getcwd
os.environ["PWD"] = _safe_getcwd()

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ROTOR_CENTER_X = 0.44
ROTOR_CENTER_Z = -0.242
ROTOR_BLADE_PITCH = -0.14


def _polar_xy(radius: float, angle: float, tangential: float = 0.0) -> tuple[float, float]:
    return (
        (radius * math.cos(angle)) - (tangential * math.sin(angle)),
        (radius * math.sin(angle)) + (tangential * math.cos(angle)),
    )


def _add_rotor_visuals(rotor, *, sweep_sign: float) -> None:
    satin_metal = "satin_metal"
    blade_finish = "blade_finish"
    blade_z = -0.024

    rotor.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=satin_metal,
        name="mount_collar",
    )
    rotor.visual(
        Cylinder(radius=0.062, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=satin_metal,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=satin_metal,
        name="hub_cap",
    )

    blade_specs = [
        ("blade_east", 0.0),
        ("blade_north", 0.5 * math.pi),
        ("blade_west", math.pi),
        ("blade_south", -0.5 * math.pi),
    ]
    for blade_name, yaw in blade_specs:
        arm_x, arm_y = _polar_xy(0.106, yaw)
        rotor.visual(
            Box((0.090, 0.018, 0.008)),
            origin=Origin(xyz=(arm_x, arm_y, blade_z), rpy=(0.0, 0.0, yaw)),
            material=satin_metal,
            name=f"arm_{blade_name.split('_', 1)[1]}",
        )

    for blade_name, yaw in blade_specs:
        blade_x, blade_y = _polar_xy(0.204, yaw, tangential=0.012 * sweep_sign)
        rotor.visual(
            Box((0.212, 0.058, 0.007)),
            origin=Origin(
                xyz=(blade_x, blade_y, blade_z),
                rpy=(0.0, ROTOR_BLADE_PITCH, yaw + (0.10 * sweep_sign)),
            ),
            material=blade_finish,
            name=blade_name,
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.070),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rotor_ceiling_fan")

    model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("bronze", rgba=(0.39, 0.32, 0.24, 1.0))
    model.material("satin_metal", rgba=(0.59, 0.60, 0.63, 1.0))
    model.material("blade_finish", rgba=(0.63, 0.47, 0.31, 1.0))

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.076, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material="graphite",
        name="canopy",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material="graphite",
        name="downrod",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material="bronze",
        name="downrod_tip",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.250),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
    )

    fan_frame = model.part("fan_frame")
    fan_frame.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material="bronze",
        name="top_collar",
    )
    fan_frame.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material="bronze",
        name="swivel_knuckle",
    )
    fan_frame.visual(
        Box((1.000, 0.082, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material="graphite",
        name="bridge_bar",
    )
    fan_frame.visual(
        Cylinder(radius=0.082, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.096)),
        material="bronze",
        name="center_body",
    )
    fan_frame.visual(
        Cylinder(radius=0.075, length=0.118),
        origin=Origin(xyz=(-ROTOR_CENTER_X, 0.0, -0.165)),
        material="bronze",
        name="left_motor_pod",
    )
    fan_frame.visual(
        Cylinder(radius=0.055, length=0.032),
        origin=Origin(xyz=(-ROTOR_CENTER_X, 0.0, -0.208)),
        material="graphite",
        name="left_motor_cap",
    )
    fan_frame.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(-ROTOR_CENTER_X, 0.0, -0.233)),
        material="satin_metal",
        name="left_axle_sleeve",
    )
    fan_frame.visual(
        Cylinder(radius=0.075, length=0.118),
        origin=Origin(xyz=(ROTOR_CENTER_X, 0.0, -0.165)),
        material="bronze",
        name="right_motor_pod",
    )
    fan_frame.visual(
        Cylinder(radius=0.055, length=0.032),
        origin=Origin(xyz=(ROTOR_CENTER_X, 0.0, -0.208)),
        material="graphite",
        name="right_motor_cap",
    )
    fan_frame.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(ROTOR_CENTER_X, 0.0, -0.233)),
        material="satin_metal",
        name="right_axle_sleeve",
    )
    fan_frame.inertial = Inertial.from_geometry(
        Box((1.04, 0.18, 0.26)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
    )

    left_rotor = model.part("left_rotor")
    _add_rotor_visuals(left_rotor, sweep_sign=-1.0)

    right_rotor = model.part("right_rotor")
    _add_rotor_visuals(right_rotor, sweep_sign=1.0)

    model.articulation(
        "mount_to_frame",
        ArticulationType.FIXED,
        parent=ceiling_mount,
        child=fan_frame,
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
    )
    model.articulation(
        "left_rotor_axle",
        ArticulationType.CONTINUOUS,
        parent=fan_frame,
        child=left_rotor,
        origin=Origin(xyz=(-ROTOR_CENTER_X, 0.0, ROTOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=14.0),
    )
    model.articulation(
        "right_rotor_axle",
        ArticulationType.CONTINUOUS,
        parent=fan_frame,
        child=right_rotor,
        origin=Origin(xyz=(ROTOR_CENTER_X, 0.0, ROTOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=14.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_mount = object_model.get_part("ceiling_mount")
    fan_frame = object_model.get_part("fan_frame")
    left_rotor = object_model.get_part("left_rotor")
    right_rotor = object_model.get_part("right_rotor")
    left_axle = object_model.get_articulation("left_rotor_axle")
    right_axle = object_model.get_articulation("right_rotor_axle")

    downrod_tip = ceiling_mount.get_visual("downrod_tip")
    top_collar = fan_frame.get_visual("top_collar")
    left_axle_sleeve = fan_frame.get_visual("left_axle_sleeve")
    right_axle_sleeve = fan_frame.get_visual("right_axle_sleeve")
    left_mount_collar = left_rotor.get_visual("mount_collar")
    right_mount_collar = right_rotor.get_visual("mount_collar")
    left_blade_east = left_rotor.get_visual("blade_east")
    right_blade_east = right_rotor.get_visual("blade_east")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_overlap(
        fan_frame,
        ceiling_mount,
        axes="xy",
        min_overlap=0.030,
        elem_a=top_collar,
        elem_b=downrod_tip,
    )
    ctx.expect_contact(fan_frame, ceiling_mount, elem_a=top_collar, elem_b=downrod_tip)

    ctx.expect_overlap(
        left_rotor,
        fan_frame,
        axes="xy",
        min_overlap=0.020,
        elem_a=left_mount_collar,
        elem_b=left_axle_sleeve,
    )
    ctx.expect_contact(left_rotor, fan_frame, elem_a=left_mount_collar, elem_b=left_axle_sleeve)

    ctx.expect_overlap(
        right_rotor,
        fan_frame,
        axes="xy",
        min_overlap=0.020,
        elem_a=right_mount_collar,
        elem_b=right_axle_sleeve,
    )
    ctx.expect_contact(right_rotor, fan_frame, elem_a=right_mount_collar, elem_b=right_axle_sleeve)

    ctx.expect_origin_distance(left_rotor, right_rotor, axes="yz", max_dist=0.001)
    ctx.expect_origin_gap(right_rotor, left_rotor, axis="x", min_gap=0.86, max_gap=0.90)
    ctx.expect_gap(right_rotor, left_rotor, axis="x", min_gap=0.12)

    ctx.check(
        "left_rotor_axis_up",
        tuple(left_axle.axis) == (0.0, 0.0, 1.0),
        details=f"Expected left rotor axle axis (0, 0, 1), got {left_axle.axis}.",
    )
    ctx.check(
        "right_rotor_axis_down",
        tuple(right_axle.axis) == (0.0, 0.0, -1.0),
        details=f"Expected right rotor axle axis (0, 0, -1), got {right_axle.axis}.",
    )

    for pose_name, angle in (
        ("quarter_turn", math.pi / 2.0),
        ("oblique_turn", 1.10),
        ("reverse_turn", -0.95),
    ):
        with ctx.pose({left_axle: angle, right_axle: angle}):
            ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
            ctx.expect_gap(
                right_rotor,
                left_rotor,
                axis="x",
                min_gap=0.12,
                positive_elem=right_blade_east,
                negative_elem=left_blade_east,
                name=f"{pose_name}_blade_clearance",
            )
            ctx.expect_contact(left_rotor, fan_frame, elem_a=left_mount_collar, elem_b=left_axle_sleeve)
            ctx.expect_contact(right_rotor, fan_frame, elem_a=right_mount_collar, elem_b=right_axle_sleeve)

    with ctx.pose({left_axle: 1.0, right_axle: 1.0}):
        left_blade_aabb = ctx.part_element_world_aabb(left_rotor, elem=left_blade_east)
        right_blade_aabb = ctx.part_element_world_aabb(right_rotor, elem=right_blade_east)
        if left_blade_aabb is None or right_blade_aabb is None:
            ctx.fail("blade_pose_measurements", "Expected east blade AABBs to be available in posed checks.")
        else:
            left_center_y = 0.5 * (left_blade_aabb[0][1] + left_blade_aabb[1][1])
            right_center_y = 0.5 * (right_blade_aabb[0][1] + right_blade_aabb[1][1])
            ctx.check(
                "left_rotor_positive_angle_sweeps_positive_y",
                left_center_y > 0.10,
                details=f"Expected left rotor east blade center y > 0.10, got {left_center_y:.3f}.",
            )
            ctx.check(
                "right_rotor_positive_angle_sweeps_negative_y",
                right_center_y < -0.10,
                details=f"Expected right rotor east blade center y < -0.10, got {right_center_y:.3f}.",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

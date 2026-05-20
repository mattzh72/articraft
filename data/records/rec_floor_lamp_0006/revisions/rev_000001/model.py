from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

HERE = Path(__file__).resolve().parent

try:
    os.chdir(HERE)
except FileNotFoundError:
    pass

_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        fallback = str(HERE)
        os.chdir(fallback)
        return fallback


os.getcwd = _safe_getcwd

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_shade_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.042, 0.000),
            (0.050, 0.036),
            (0.066, 0.102),
            (0.082, 0.176),
        ],
        [
            (0.036, 0.000),
            (0.044, 0.036),
            (0.060, 0.102),
            (0.076, 0.176),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    warm_bulb = model.material("warm_bulb", rgba=(0.96, 0.89, 0.66, 0.92))
    cream_shade = model.material("cream_shade", rgba=(0.90, 0.87, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.22, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=matte_black,
        name="base_plinth",
    )
    base.visual(
        Box((0.27, 0.15, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_steel,
        name="base_trim",
    )
    base.visual(
        Box((0.070, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=satin_steel,
        name="column_socket",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.062)),
        mass=8.4,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_steel,
        name="column_spigot",
    )
    column.visual(
        Cylinder(radius=0.014, length=0.880),
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        material=matte_black,
        name="column_tube",
    )
    column.visual(
        Box((0.050, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        material=satin_steel,
        name="shoulder_block",
    )
    column.visual(
        Box((0.058, 0.006, 0.046)),
        origin=Origin(xyz=(0.029, -0.019, 0.920)),
        material=satin_steel,
        name="shoulder_left_cheek",
    )
    column.visual(
        Box((0.058, 0.006, 0.046)),
        origin=Origin(xyz=(0.029, 0.019, 0.920)),
        material=satin_steel,
        name="shoulder_right_cheek",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.060, 0.070, 0.950)),
        mass=2.1,
        origin=Origin(xyz=(0.015, 0.0, 0.475)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="shoulder_hub",
    )
    arm.visual(
        Box((0.020, 0.032, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=satin_steel,
        name="shoulder_collar",
    )
    arm.visual(
        Box((0.500, 0.026, 0.018)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=matte_black,
        name="arm_beam",
    )
    arm.visual(
        Box((0.050, 0.050, 0.026)),
        origin=Origin(xyz=(0.555, 0.0, 0.0)),
        material=satin_steel,
        name="elbow_block",
    )
    arm.visual(
        Box((0.060, 0.006, 0.040)),
        origin=Origin(xyz=(0.590, -0.019, 0.0)),
        material=satin_steel,
        name="elbow_left_cheek",
    )
    arm.visual(
        Box((0.060, 0.006, 0.040)),
        origin=Origin(xyz=(0.590, 0.019, 0.0)),
        material=satin_steel,
        name="elbow_right_cheek",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.620, 0.050, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="elbow_hub",
    )
    shade.visual(
        Box((0.040, 0.030, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=satin_steel,
        name="neck_link",
    )
    shade.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.076, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="socket",
    )
    shade.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream_shade,
        name="shade_collar",
    )
    shade.visual(
        _save_mesh(_build_shade_shell(), "shade_shell.obj"),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream_shade,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="bulb_stem",
    )
    shade.visual(
        Sphere(radius=0.027),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=warm_bulb,
        name="bulb_globe",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.29, 0.17, 0.17)),
        mass=0.75,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
    )
    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=arm,
        origin=Origin(xyz=(0.047, 0.0, 0.920)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.4,
            lower=-0.30,
            upper=0.85,
        ),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(0.610, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, seed=0)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("column_to_arm")
    elbow = object_model.get_articulation("arm_to_shade")

    base_plinth = base.get_visual("base_plinth")
    column_socket = base.get_visual("column_socket")
    column_spigot = column.get_visual("column_spigot")
    column_tube = column.get_visual("column_tube")
    shoulder_left_cheek = column.get_visual("shoulder_left_cheek")
    shoulder_right_cheek = column.get_visual("shoulder_right_cheek")
    shoulder_hub = arm.get_visual("shoulder_hub")
    arm_beam = arm.get_visual("arm_beam")
    elbow_left_cheek = arm.get_visual("elbow_left_cheek")
    elbow_right_cheek = arm.get_visual("elbow_right_cheek")
    elbow_hub = shade.get_visual("elbow_hub")
    bulb_globe = shade.get_visual("bulb_globe")
    shade_shell = shade.get_visual("shade_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=24)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(
        column,
        base,
        elem_a=column_spigot,
        elem_b=column_socket,
        name="column_is_seated_in_base_socket",
    )
    ctx.expect_contact(
        arm,
        column,
        elem_a=shoulder_hub,
        elem_b=shoulder_left_cheek,
        name="shoulder_hub_contacts_left_cheek",
    )
    ctx.expect_contact(
        arm,
        column,
        elem_a=shoulder_hub,
        elem_b=shoulder_right_cheek,
        name="shoulder_hub_contacts_right_cheek",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a=elbow_hub,
        elem_b=elbow_left_cheek,
        name="elbow_hub_contacts_left_cheek",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a=elbow_hub,
        elem_b=elbow_right_cheek,
        name="elbow_hub_contacts_right_cheek",
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem=column_socket,
        outer_elem=base_plinth,
        name="column_is_centered_on_rectangular_base",
    )
    ctx.expect_origin_gap(
        arm,
        base,
        axis="z",
        min_gap=0.96,
        max_gap=1.01,
        name="arm_hinge_sits_at_floor_lamp_height",
    )
    ctx.expect_gap(
        arm,
        column,
        axis="x",
        min_gap=0.04,
        positive_elem=arm_beam,
        negative_elem=column_tube,
        name="arm_projects_forward_of_column",
    )
    ctx.expect_gap(
        shade,
        arm,
        axis="x",
        min_gap=0.10,
        positive_elem=shade_shell,
        negative_elem=arm_beam,
        name="conical_shade_extends_beyond_arm_tip",
    )
    ctx.expect_within(
        shade,
        shade,
        axes="yz",
        inner_elem=bulb_globe,
        outer_elem=shade_shell,
        name="bulb_globe_stays_inside_shade_profile",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=0.78,
        positive_elem=shade_shell,
        negative_elem=base_plinth,
        name="rest_pose_keeps_shade_clear_of_base",
    )

    shoulder_limits = shoulder.motion_limits
    elbow_limits = elbow.motion_limits
    if shoulder_limits is not None and shoulder_limits.lower is not None and shoulder_limits.upper is not None:
        with ctx.pose({shoulder: shoulder_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shoulder_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shoulder_lower_no_floating")
        with ctx.pose({shoulder: shoulder_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shoulder_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shoulder_upper_no_floating")
    if elbow_limits is not None and elbow_limits.lower is not None and elbow_limits.upper is not None:
        with ctx.pose({elbow: elbow_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="elbow_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="elbow_lower_no_floating")
        with ctx.pose({elbow: elbow_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="elbow_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="elbow_upper_no_floating")

    with ctx.pose({shoulder: 0.70, elbow: -0.20}):
        ctx.expect_contact(arm, column, elem_a=shoulder_hub, elem_b=shoulder_left_cheek)
        ctx.expect_contact(shade, arm, elem_a=elbow_hub, elem_b=elbow_left_cheek)
        ctx.expect_gap(
            shade,
            column,
            axis="x",
            min_gap=0.36,
            positive_elem=shade_shell,
            negative_elem=column_tube,
            name="raised_pose_keeps_shade_forward_of_column",
        )
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=1.00,
            positive_elem=shade_shell,
            negative_elem=base_plinth,
            name="raised_pose_keeps_shade_high_above_base",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

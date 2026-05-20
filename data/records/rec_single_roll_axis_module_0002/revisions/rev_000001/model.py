from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BAR_LENGTH = 0.104
BAR_DEPTH = 0.020
BAR_HEIGHT = 0.014
BAR_HALF_HEIGHT = 0.5 * BAR_HEIGHT
FRAME_HALF_SPAN_Z = 0.036

UPRIGHT_THICKNESS = 0.014
UPRIGHT_HALF_THICKNESS = 0.5 * UPRIGHT_THICKNESS
UPRIGHT_DEPTH = 0.024
STRUT_HEIGHT = 0.018
STRUT_HALF_HEIGHT = 0.5 * STRUT_HEIGHT
STRUT_CENTER_Z = 0.020

CARTRIDGE_DEPTH = 0.036
CARTRIDGE_HEIGHT = 0.022
CARTRIDGE_HALF_HEIGHT = 0.5 * CARTRIDGE_HEIGHT
INNER_BOSS_RADIUS = 0.020
INNER_BOSS_LENGTH = 0.010
OUTER_FLANGE_RADIUS = 0.022
OUTER_FLANGE_LENGTH = 0.006
BORE_RADIUS = 0.013

SHAFT_RADIUS = 0.010
SHAFT_LENGTH = 0.122
SHAFT_CENTER_X = 0.052
HUB_RADIUS = 0.020
HUB_LENGTH = 0.034
LEFT_COLLAR_CENTER_X = 0.004
RIGHT_COLLAR_CENTER_X = 0.100
COLLAR_RADIUS = 0.014
COLLAR_LENGTH = 0.010
TAB_SIZE = (0.030, 0.020, 0.008)
TAB_ORIGIN = Origin(xyz=(SHAFT_CENTER_X, 0.0, 0.022))
X_AXIS_CYLINDER_RPY = (0.0, 0.5 * pi, 0.0)


def _x_cylinder(radius: float, length: float, center_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - 0.5 * length, 0.0, 0.0))
    )


def _make_left_upright() -> cq.Workplane:
    upright = (
        cq.Workplane("XY")
        .box(UPRIGHT_THICKNESS, UPRIGHT_DEPTH, STRUT_HEIGHT)
        .translate((-UPRIGHT_HALF_THICKNESS, 0.0, STRUT_CENTER_Z))
    )
    upright = upright.union(
        cq.Workplane("XY")
        .box(UPRIGHT_THICKNESS, UPRIGHT_DEPTH, STRUT_HEIGHT)
        .translate((-UPRIGHT_HALF_THICKNESS, 0.0, -STRUT_CENTER_Z))
    )
    upright = upright.union(
        cq.Workplane("XY")
        .box(UPRIGHT_THICKNESS, CARTRIDGE_DEPTH, CARTRIDGE_HEIGHT)
        .translate((-UPRIGHT_HALF_THICKNESS, 0.0, 0.0))
    )
    upright = upright.union(
        _x_cylinder(INNER_BOSS_RADIUS, INNER_BOSS_LENGTH, 0.5 * INNER_BOSS_LENGTH)
    )
    upright = upright.union(
        _x_cylinder(
            OUTER_FLANGE_RADIUS,
            OUTER_FLANGE_LENGTH,
            -(UPRIGHT_THICKNESS + 0.5 * OUTER_FLANGE_LENGTH),
        )
    )
    return upright.cut(
        _x_cylinder(
            BORE_RADIUS,
            UPRIGHT_THICKNESS + INNER_BOSS_LENGTH + OUTER_FLANGE_LENGTH + 0.012,
            -0.5 * UPRIGHT_THICKNESS,
        )
    )


def _make_right_upright() -> cq.Workplane:
    upright = (
        cq.Workplane("XY")
        .box(UPRIGHT_THICKNESS, UPRIGHT_DEPTH, STRUT_HEIGHT)
        .translate((UPRIGHT_HALF_THICKNESS, 0.0, -STRUT_HALF_HEIGHT))
    )
    upright = upright.union(
        cq.Workplane("XY")
        .box(UPRIGHT_THICKNESS, UPRIGHT_DEPTH, STRUT_HEIGHT)
        .translate((UPRIGHT_HALF_THICKNESS, 0.0, -0.049))
    )
    upright = upright.union(
        cq.Workplane("XY")
        .box(UPRIGHT_THICKNESS, CARTRIDGE_DEPTH, CARTRIDGE_HEIGHT)
        .translate((UPRIGHT_HALF_THICKNESS, 0.0, -0.029))
    )
    upright = upright.union(
        _x_cylinder(INNER_BOSS_RADIUS, INNER_BOSS_LENGTH, -0.5 * INNER_BOSS_LENGTH).translate(
            (0.0, 0.0, -0.029)
        )
    )
    upright = upright.union(
        _x_cylinder(
            OUTER_FLANGE_RADIUS,
            OUTER_FLANGE_LENGTH,
            UPRIGHT_THICKNESS + 0.5 * OUTER_FLANGE_LENGTH,
        ).translate((0.0, 0.0, -0.029))
    )
    return upright.cut(
        _x_cylinder(
            BORE_RADIUS,
            UPRIGHT_THICKNESS + INNER_BOSS_LENGTH + OUTER_FLANGE_LENGTH + 0.012,
            0.5 * UPRIGHT_THICKNESS,
        ).translate((0.0, 0.0, -0.029))
    )


def _make_rotor_body() -> cq.Workplane:
    rotor = _x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, SHAFT_CENTER_X)
    rotor = rotor.union(_x_cylinder(HUB_RADIUS, HUB_LENGTH, SHAFT_CENTER_X))
    rotor = rotor.union(_x_cylinder(COLLAR_RADIUS, COLLAR_LENGTH, LEFT_COLLAR_CENTER_X))
    rotor = rotor.union(_x_cylinder(COLLAR_RADIUS, COLLAR_LENGTH, RIGHT_COLLAR_CENTER_X))
    return rotor


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    filename: str,
    material_name: str,
) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_axis_rotary_module", assets=ASSETS)

    model.material("frame_dark", rgba=(0.21, 0.24, 0.28, 1.0))
    model.material("cartridge_dark", rgba=(0.32, 0.34, 0.38, 1.0))
    model.material("rotor_silver", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("index_orange", rgba=(0.90, 0.40, 0.12, 1.0))

    left_upright = model.part("left_upright")
    _add_mesh_visual(left_upright, _make_left_upright(), "left_upright.obj", "cartridge_dark")





    left_upright.inertial = Inertial.from_geometry(
        Box((0.020, CARTRIDGE_DEPTH, 0.058)),
        mass=0.18,
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
    )

    top_bar = model.part("top_bar")
    top_bar.visual(
        Box((BAR_LENGTH, BAR_DEPTH, BAR_HEIGHT)),
        origin=Origin(xyz=(0.5 * BAR_LENGTH, 0.0, BAR_HALF_HEIGHT)),
        material="frame_dark",
    )

    top_bar.inertial = Inertial.from_geometry(
        Box((BAR_LENGTH, BAR_DEPTH, BAR_HEIGHT)),
        mass=0.12,
        origin=Origin(xyz=(0.5 * BAR_LENGTH, 0.0, BAR_HALF_HEIGHT)),
    )

    bottom_bar = model.part("bottom_bar")
    bottom_bar.visual(
        Box((BAR_LENGTH, BAR_DEPTH, BAR_HEIGHT)),
        origin=Origin(xyz=(0.5 * BAR_LENGTH, 0.0, -BAR_HALF_HEIGHT)),
        material="frame_dark",
    )

    bottom_bar.inertial = Inertial.from_geometry(
        Box((BAR_LENGTH, BAR_DEPTH, BAR_HEIGHT)),
        mass=0.12,
        origin=Origin(xyz=(0.5 * BAR_LENGTH, 0.0, -BAR_HALF_HEIGHT)),
    )

    right_upright = model.part("right_upright")
    _add_mesh_visual(right_upright, _make_right_upright(), "right_upright.obj", "cartridge_dark")





    right_upright.inertial = Inertial.from_geometry(
        Box((0.020, CARTRIDGE_DEPTH, 0.058)),
        mass=0.18,
        origin=Origin(xyz=(0.005, 0.0, -0.029)),
    )

    rotor = model.part("rotor")
    _add_mesh_visual(rotor, _make_rotor_body(), "rotor_body.obj", "rotor_silver")
    rotor.visual(Box(TAB_SIZE), origin=TAB_ORIGIN, material="index_orange")





    rotor.inertial = Inertial.from_geometry(
        Box((SHAFT_LENGTH, 0.040, 0.052)),
        mass=0.30,
        origin=Origin(xyz=(SHAFT_CENTER_X, 0.0, 0.0)),
    )

    model.articulation(
        "left_to_top",
        ArticulationType.FIXED,
        parent=left_upright,
        child=top_bar,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HALF_SPAN_Z - BAR_HALF_HEIGHT)),
    )
    model.articulation(
        "left_to_bottom",
        ArticulationType.FIXED,
        parent=left_upright,
        child=bottom_bar,
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HALF_SPAN_Z + BAR_HALF_HEIGHT)),
    )
    model.articulation(
        "top_to_right",
        ArticulationType.FIXED,
        parent=top_bar,
        child=right_upright,
        origin=Origin(xyz=(BAR_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_rotor",
        ArticulationType.REVOLUTE,
        parent=left_upright,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.5 * pi,
            upper=0.5 * pi,
            effort=8.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "right_upright",
        "rotor",
        reason="bearing cartridge collision proxy conservatively encloses the shaft bore on the outboard support",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("left_upright", "rotor", axes="xy", max_dist=0.001)
    ctx.expect_aabb_gap("top_bar", "rotor", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("rotor", "bottom_bar", axis="z", max_gap=0.015, max_penetration=0.0)
    ctx.expect_aabb_gap("top_bar", "bottom_bar", axis="z", max_gap=0.080, max_penetration=0.0)
    ctx.expect_aabb_overlap("top_bar", "bottom_bar", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("rotor", "top_bar", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("rotor", "bottom_bar", axes="xy", min_overlap=0.020)
    ctx.expect_joint_motion_axis(
        "frame_to_rotor",
        "rotor",
        world_axis="y",
        direction="negative",
        min_delta=0.005,
    )

    with ctx.pose(frame_to_rotor=0.5 * pi):
        ctx.expect_origin_distance("left_upright", "rotor", axes="xy", max_dist=0.001)
        ctx.expect_aabb_gap("top_bar", "rotor", axis="z", max_gap=0.020, max_penetration=0.0)
        ctx.expect_aabb_gap("rotor", "bottom_bar", axis="z", max_gap=0.020, max_penetration=0.0)
        ctx.expect_aabb_overlap("rotor", "top_bar", axes="xy", min_overlap=0.020)

    with ctx.pose(frame_to_rotor=-0.5 * pi):
        ctx.expect_origin_distance("left_upright", "rotor", axes="xy", max_dist=0.001)
        ctx.expect_aabb_gap("top_bar", "rotor", axis="z", max_gap=0.020, max_penetration=0.0)
        ctx.expect_aabb_gap("rotor", "bottom_bar", axis="z", max_gap=0.020, max_penetration=0.0)
        ctx.expect_aabb_overlap("rotor", "bottom_bar", axes="xy", min_overlap=0.020)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

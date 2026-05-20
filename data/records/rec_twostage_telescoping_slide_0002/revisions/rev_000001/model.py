from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
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
PLATE_LENGTH = 0.180
PLATE_WIDTH = 0.062
PLATE_THICKNESS = 0.006

OUTER_LENGTH = 0.240
OUTER_WIDTH = 0.018
OUTER_HEIGHT = 0.026
OUTER_WALL = 0.0022
OUTER_END_MARGIN = 0.012

MIDDLE_LENGTH = 0.180
MIDDLE_WIDTH = 0.013
MIDDLE_HEIGHT = 0.018
MIDDLE_WALL = 0.0018
MIDDLE_END_MARGIN = 0.010

INNER_LENGTH = 0.130
INNER_WIDTH = 0.009
INNER_HEIGHT = 0.0105

OUTER_STAGE_Z = 0.0030
INNER_STAGE_Z = 0.0035
OUTER_INSERT = 0.016
INNER_INSERT = 0.014
OUTER_TRAVEL = 0.110
INNER_TRAVEL = 0.090


def _base_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            PLATE_LENGTH,
            PLATE_WIDTH,
            PLATE_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, -PLATE_THICKNESS))
        .edges("|Z")
        .fillet(0.004)
    )
    return (
        plate.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (0.032, -0.022),
                (0.032, 0.022),
                (0.148, -0.022),
                (0.148, 0.022),
            ]
        )
        .slot2D(0.018, 0.0052, 90)
        .cutThruAll()
    )


def _c_rail_shape(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    end_margin: float,
    open_side: str,
    window_centers: tuple[float, ...],
) -> cq.Workplane:
    rail = cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(False, True, False),
    )
    cavity_center_y = wall / 2.0 if open_side == "+y" else -wall / 2.0
    rail = rail.cut(
        cq.Workplane("XY")
        .transformed(offset=(length / 2.0, cavity_center_y, height / 2.0))
        .box(length - 2.0 * end_margin, width - wall, height - 2.0 * wall)
    )

    window_center_y = -width / 2.0 + wall / 2.0
    if open_side == "-y":
        window_center_y = width / 2.0 - wall / 2.0

    for x_center in window_centers:
        rail = rail.cut(
            cq.Workplane("XY")
            .transformed(offset=(x_center, window_center_y, height * 0.50))
            .box(0.036, wall + 0.0010, 0.010)
        )

    return rail


def _inner_rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(
        INNER_LENGTH,
        INNER_WIDTH,
        INNER_HEIGHT,
        centered=(False, True, False),
    )
    nose = (
        cq.Workplane("XY")
        .transformed(
            offset=(INNER_LENGTH - 0.018, 0.0, INNER_HEIGHT * 0.55),
        )
        .box(0.036, INNER_WIDTH * 0.92, INNER_HEIGHT * 0.36)
    )
    rail = rail.union(nose)
    return (
        rail.faces(">Z")
        .workplane()
        .pushPoints([(0.038, 0.0), (0.092, 0.0)])
        .slot2D(0.014, 0.0045, 90)
        .cutBlind(-0.0026)
    )


def _add_mesh_box_proxy_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    box_size: tuple[float, float, float],
    box_center: tuple[float, float, float],
    mass: float,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, mesh_name, assets=ASSETS), material=material)
    proxy_origin = Origin(xyz=box_center)

    part.inertial = Inertial.from_geometry(Box(box_size), mass=mass, origin=proxy_origin)
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide", assets=ASSETS)

    model.material("zinc", rgba=(0.80, 0.82, 0.86, 1.0))
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("dark_steel", rgba=(0.39, 0.41, 0.45, 1.0))

    base_plate = _add_mesh_box_proxy_part(
        model,
        name="base_plate",
        shape=_base_plate_shape(),
        mesh_name="base_plate.obj",
        material="zinc",
        box_size=(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
        box_center=(PLATE_LENGTH / 2.0, 0.0, -PLATE_THICKNESS / 2.0),
        mass=0.75,
    )
    outer_rail = _add_mesh_box_proxy_part(
        model,
        name="outer_rail",
        shape=_c_rail_shape(
            length=OUTER_LENGTH,
            width=OUTER_WIDTH,
            height=OUTER_HEIGHT,
            wall=OUTER_WALL,
            end_margin=OUTER_END_MARGIN,
            open_side="+y",
            window_centers=(0.070, 0.150),
        ),
        mesh_name="outer_rail.obj",
        material="rail_steel",
        box_size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT),
        box_center=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0),
        mass=0.42,
    )
    middle_rail = _add_mesh_box_proxy_part(
        model,
        name="middle_rail",
        shape=_c_rail_shape(
            length=MIDDLE_LENGTH,
            width=MIDDLE_WIDTH,
            height=MIDDLE_HEIGHT,
            wall=MIDDLE_WALL,
            end_margin=MIDDLE_END_MARGIN,
            open_side="-y",
            window_centers=(0.055, 0.120),
        ),
        mesh_name="middle_rail.obj",
        material="dark_steel",
        box_size=(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT),
        box_center=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0),
        mass=0.28,
    )
    inner_rail = _add_mesh_box_proxy_part(
        model,
        name="inner_rail",
        shape=_inner_rail_shape(),
        mesh_name="inner_rail.obj",
        material="rail_steel",
        box_size=(INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT),
        box_center=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0),
        mass=0.18,
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base_plate,
        child=outer_rail,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_rail,
        child=middle_rail,
        origin=Origin(xyz=(OUTER_INSERT, 0.0, OUTER_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TRAVEL,
            effort=120.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_rail,
        child=inner_rail,
        origin=Origin(xyz=(INNER_INSERT, 0.0, INNER_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=90.0,
            velocity=0.45,
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
        "outer_rail",
        "middle_rail",
        reason="Nested telescoping members use conservative full-envelope box proxies.",
    )
    ctx.allow_overlap(
        "middle_rail",
        "inner_rail",
        reason="The inner member travels inside the middle rail while box proxies stay conservative.",
    )
    ctx.allow_overlap(
        "outer_rail",
        "inner_rail",
        reason="Retracted nested stages share the outer rail envelope in AABB overlap QC.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("outer_rail", "base_plate", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_gap("outer_rail", "base_plate", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_gap("middle_rail", "base_plate", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("inner_rail", "base_plate", axis="z", max_gap=0.015, max_penetration=0.0)
    ctx.expect_aabb_overlap("middle_rail", "outer_rail", axes="xy", min_overlap=0.006)
    ctx.expect_aabb_overlap("inner_rail", "middle_rail", axes="xy", min_overlap=0.004)
    ctx.expect_joint_motion_axis(
        "outer_to_middle",
        "middle_rail",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_inner",
        "inner_rail",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(outer_to_middle=OUTER_TRAVEL):
        ctx.expect_aabb_overlap("middle_rail", "outer_rail", axes="xy", min_overlap=0.006)
        ctx.expect_aabb_overlap("inner_rail", "middle_rail", axes="xy", min_overlap=0.004)

    with ctx.pose(outer_to_middle=OUTER_TRAVEL, middle_to_inner=INNER_TRAVEL):
        ctx.expect_aabb_overlap("inner_rail", "middle_rail", axes="xy", min_overlap=0.004)
        ctx.expect_aabb_gap("inner_rail", "base_plate", axis="z", max_gap=0.015, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

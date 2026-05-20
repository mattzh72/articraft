from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

import cadquery as cq


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


BASE_X = 0.44
BASE_Y = 0.30
BASE_Z = 0.06
PIVOT_Z = 0.19
TABLE_JOINT_Z = 0.080
TABLE_RADIUS = 0.105
TABLE_THICKNESS = 0.032


def _bolt_circle_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / count),
            radius * math.sin((2.0 * math.pi * i) / count),
        )
        for i in range(count)
    ]


def _build_base_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_Z)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, BASE_Z / 2.0))
    )

    left_bearing_block = (
        cq.Workplane("XY")
        .box(0.08, 0.13, 0.18)
        .edges("|Z")
        .fillet(0.008)
        .translate((-0.15, 0.0, 0.150))
    )
    right_bearing_block = (
        cq.Workplane("XY")
        .box(0.08, 0.13, 0.18)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.15, 0.0, 0.150))
    )

    left_gusset = (
        cq.Workplane("XY")
        .box(0.11, 0.16, 0.07)
        .edges("|Z")
        .fillet(0.010)
        .translate((-0.13, 0.0, 0.095))
    )
    right_gusset = (
        cq.Workplane("XY")
        .box(0.11, 0.16, 0.07)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.13, 0.0, 0.095))
    )

    front_rib = (
        cq.Workplane("XY")
        .box(0.14, 0.06, 0.025)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.085, 0.0425))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(0.18, 0.08, 0.030)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, -0.085, 0.045))
    )

    return (
        plinth.union(left_bearing_block)
        .union(right_bearing_block)
        .union(left_gusset)
        .union(right_gusset)
        .union(front_rib)
        .union(rear_rib)
    )


def _build_trunnion_shape() -> cq.Workplane:
    side_web = (
        cq.Workplane("XZ")
        .moveTo(-0.155, -0.018)
        .lineTo(-0.155, 0.018)
        .lineTo(-0.11, 0.038)
        .lineTo(-0.055, 0.060)
        .lineTo(0.055, 0.060)
        .lineTo(0.11, 0.038)
        .lineTo(0.155, 0.018)
        .lineTo(0.155, -0.018)
        .lineTo(0.09, -0.006)
        .lineTo(-0.09, -0.006)
        .close()
        .extrude(0.10)
        .translate((0.0, -0.05, 0.0))
    )

    hub_shaft = (
        cq.Workplane("YZ")
        .circle(0.028)
        .extrude(0.40)
        .translate((-0.20, 0.0, 0.0))
    )

    lower_spindle_body = (
        cq.Workplane("XY")
        .circle(0.055)
        .extrude(0.024)
        .translate((0.0, 0.0, -0.004))
    )

    rotary_housing = (
        cq.Workplane("XY")
        .circle(0.080)
        .extrude(0.066)
        .translate((0.0, 0.0, 0.006))
    )

    mounting_flange = (
        cq.Workplane("XY")
        .circle(0.094)
        .circle(0.062)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.060))
    )

    trunnion = (
        side_web.union(hub_shaft)
        .union(lower_spindle_body)
        .union(rotary_housing)
        .union(mounting_flange)
    )

    spindle_pocket = (
        cq.Workplane("XY")
        .circle(0.044)
        .extrude(0.014)
        .translate((0.0, 0.0, 0.058))
    )
    center_bore = (
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(0.080)
        .translate((0.0, 0.0, -0.004))
    )

    return trunnion.cut(spindle_pocket).cut(center_bore)


def _build_turntable_shape() -> cq.Workplane:
    bolt_points = _bolt_circle_points(0.070, 8)

    platter = cq.Workplane("XY").circle(TABLE_RADIUS).extrude(TABLE_THICKNESS)
    platter = platter.faces(">Z").workplane().circle(0.088).cutBlind(-0.003)
    platter = platter.faces(">Z").workplane().circle(0.020).cutBlind(-0.010)
    platter = platter.faces(">Z").workplane().pushPoints(bolt_points).hole(0.008)
    platter = platter.faces(">Z").workplane().slot2D(0.140, 0.014).cutBlind(-0.008)
    platter = (
        platter.faces(">Z")
        .workplane()
        .transformed(rotate=(0.0, 0.0, 90.0))
        .slot2D(0.140, 0.014)
        .cutBlind(-0.008)
    )

    spindle = (
        cq.Workplane("XY")
        .circle(0.042)
        .extrude(0.014)
        .translate((0.0, 0.0, -0.014))
    )

    return platter.union(spindle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_trunnion_table", assets=ASSETS)

    cast_gray = model.material("cast_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    machined_gray = model.material("machined_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    faceplate_steel = model.material("faceplate_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.15, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base.obj", assets=ASSETS),
        material=cast_gray,
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, 0.21)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    trunnion = model.part("trunnion")
    trunnion.visual(
        mesh_from_cadquery(_build_trunnion_shape(), "trunnion.obj", assets=ASSETS),
        material=machined_gray,
    )
    trunnion.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
    )
    trunnion.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
    )
    trunnion.inertial = Inertial.from_geometry(
        Box((0.42, 0.12, 0.11)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(_build_turntable_shape(), "turntable.obj", assets=ASSETS),
        material=faceplate_steel,
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=TABLE_RADIUS, length=TABLE_THICKNESS + 0.014),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent="base",
        child="trunnion",
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.0,
            lower=-0.35,
            upper=math.pi / 2.0,
        ),
    )

    model.articulation(
        "table_spin",
        ArticulationType.CONTINUOUS,
        parent="trunnion",
        child="turntable",
        origin=Origin(xyz=(0.0, 0.0, TABLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.100)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("base", "trunnion")
    ctx.expect_aabb_contact("trunnion", "turntable")
    ctx.expect_aabb_overlap("base", "trunnion", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("base", "turntable", axes="xy", min_overlap=0.16)
    ctx.expect_origin_distance("turntable", "trunnion", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("turntable", "base", axes="x", max_dist=0.010)
    ctx.expect_origin_distance("turntable", "base", axes="y", max_dist=0.005)
    ctx.expect_joint_motion_axis(
        "trunnion_tilt",
        "turntable",
        world_axis="y",
        direction="negative",
        min_delta=0.050,
    )
    ctx.expect_aabb_gap(
        "turntable",
        "trunnion",
        axis="z",
        max_gap=0.003,
        max_penetration=0.020,
    )

    for spin in (0.0, math.pi / 2.0, math.pi):
        with ctx.pose(table_spin=spin):
            ctx.expect_origin_distance("turntable", "trunnion", axes="xy", max_dist=0.005)
            ctx.expect_origin_distance("turntable", "base", axes="x", max_dist=0.010)
            ctx.expect_origin_distance("turntable", "base", axes="y", max_dist=0.005)
            ctx.expect_aabb_overlap("turntable", "trunnion", axes="xy", min_overlap=0.12)
            ctx.expect_aabb_gap(
                "turntable",
                "trunnion",
                axis="z",
                max_gap=0.003,
                max_penetration=0.020,
            )

    for tilt in (-0.25, 1.40):
        with ctx.pose(trunnion_tilt=tilt):
            ctx.expect_aabb_contact("base", "trunnion")
            ctx.expect_origin_distance("turntable", "base", axes="x", max_dist=0.010)
            ctx.expect_aabb_overlap("base", "turntable", axes="xy", min_overlap=0.020)
        with ctx.pose(trunnion_tilt=tilt, table_spin=math.pi / 2.0):
            ctx.expect_aabb_contact("base", "trunnion")
            ctx.expect_origin_distance("turntable", "base", axes="x", max_dist=0.010)
            ctx.expect_aabb_overlap("base", "turntable", axes="xy", min_overlap=0.020)

    with ctx.pose(trunnion_tilt=-0.25):
        ctx.expect_origin_distance("turntable", "base", axes="y", min_dist=0.015, max_dist=0.030)
    with ctx.pose(trunnion_tilt=-0.25, table_spin=math.pi / 2.0):
        ctx.expect_origin_distance("turntable", "base", axes="y", min_dist=0.015, max_dist=0.030)
    with ctx.pose(trunnion_tilt=1.40):
        ctx.expect_origin_distance("turntable", "base", axes="y", min_dist=0.070, max_dist=0.085)
    with ctx.pose(trunnion_tilt=1.40, table_spin=math.pi / 2.0):
        ctx.expect_origin_distance("turntable", "base", axes="y", min_dist=0.070, max_dist=0.085)
    with ctx.pose(trunnion_tilt=math.pi / 2.0):
        ctx.expect_origin_distance("turntable", "base", axes="x", max_dist=0.010)
        ctx.expect_origin_distance("turntable", "base", axes="y", min_dist=0.080, max_dist=0.095)
        ctx.expect_aabb_overlap("base", "turntable", axes="xy", min_overlap=0.015)
    with ctx.pose(trunnion_tilt=math.pi / 2.0, table_spin=math.pi / 2.0):
        ctx.expect_origin_distance("turntable", "base", axes="x", max_dist=0.010)
        ctx.expect_origin_distance("turntable", "base", axes="y", min_dist=0.080, max_dist=0.095)
        ctx.expect_aabb_overlap("base", "turntable", axes="xy", min_overlap=0.015)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()

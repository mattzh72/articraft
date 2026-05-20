from __future__ import annotations

import math

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
PRIMARY_AXIS_LIMIT = 2.6
SECONDARY_AXIS_LIMIT = 3.0


def _polar_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + (2.0 * math.pi * idx / count)),
            radius * math.sin(phase + (2.0 * math.pi * idx / count)),
        )
        for idx in range(count)
    ]


def _cylinder(radius: float, height: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z_center - (height / 2.0)))
    )


def _ring(
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    *,
    hole_radius: float | None = None,
    hole_diameter: float | None = None,
    hole_count: int = 0,
    phase: float = 0.0,
) -> cq.Workplane:
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    if hole_count and hole_radius is not None and hole_diameter is not None:
        ring = (
            ring.faces(">Z")
            .workplane(centerOption="CenterOfMass")
            .pushPoints(_polar_points(hole_radius, hole_count, phase))
            .hole(hole_diameter)
        )
    return ring.translate((0.0, 0.0, z_center - (height / 2.0)))


def _ring_plate_with_windows(
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    *,
    window_radius: float,
    window_center_radius: float,
    window_count: int,
    phase: float = 0.0,
) -> cq.Workplane:
    plate = _ring(outer_radius, inner_radius, height, z_center)
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_polar_points(window_center_radius, window_count, phase))
        .circle(window_radius)
        .cutBlind(-height)
    )


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _add_annular_box_collisions(
    part,
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    z_center: float = 0.0,
) -> None:
    wall = outer_radius - inner_radius
    x_center = inner_radius + (wall / 2.0)
    y_center = inner_radius + (wall / 2.0)
    long_span = 2.0 * outer_radius
    short_span = 2.0 * inner_radius






def _build_pedestal_shape() -> cq.Workplane:
    foot = _cylinder(radius=0.160, height=0.022, z_center=-0.149)
    lower_body = _cylinder(radius=0.136, height=0.086, z_center=-0.105)
    upper_body = _cylinder(radius=0.118, height=0.048, z_center=-0.038)
    fixed_race = _ring(
        outer_radius=0.190,
        inner_radius=0.118,
        height=0.018,
        z_center=-0.009,
        hole_radius=0.154,
        hole_diameter=0.008,
        hole_count=12,
        phase=math.pi / 12.0,
    )
    primary_spindle = _cylinder(radius=0.015, height=0.040, z_center=0.000)
    spindle_cap = _ring(
        outer_radius=0.041,
        inner_radius=0.018,
        height=0.008,
        z_center=0.022,
    )
    return _union_all(
        foot,
        lower_body,
        upper_body,
        fixed_race,
        primary_spindle,
        spindle_cap,
    )


def _build_slew_shape() -> cq.Workplane:
    lower_bearing_ring = _ring(
        outer_radius=0.176,
        inner_radius=0.104,
        height=0.022,
        z_center=0.012,
        hole_radius=0.145,
        hole_diameter=0.008,
        hole_count=12,
    )
    transfer_plate = _ring_plate_with_windows(
        outer_radius=0.146,
        inner_radius=0.050,
        height=0.018,
        z_center=0.034,
        window_radius=0.020,
        window_center_radius=0.090,
        window_count=6,
        phase=math.pi / 6.0,
    )
    lower_hub = _ring(
        outer_radius=0.050,
        inner_radius=0.018,
        height=0.032,
        z_center=0.000,
    )
    secondary_support_flange = _ring(
        outer_radius=0.090,
        inner_radius=0.048,
        height=0.010,
        z_center=0.046,
        hole_radius=0.068,
        hole_diameter=0.006,
        hole_count=8,
        phase=math.pi / 8.0,
    )
    secondary_spindle = _cylinder(radius=0.014, height=0.038, z_center=0.060)
    upper_guard = _ring(
        outer_radius=0.060,
        inner_radius=0.026,
        height=0.010,
        z_center=0.078,
    )
    return _union_all(
        lower_bearing_ring,
        transfer_plate,
        lower_hub,
        secondary_support_flange,
        secondary_spindle,
        upper_guard,
    )


def _build_platter_shape() -> cq.Workplane:
    secondary_collar = _ring(
        outer_radius=0.048,
        inner_radius=0.018,
        height=0.030,
        z_center=0.000,
    )
    tooling_disc = _cylinder(radius=0.122, height=0.020, z_center=0.026)
    tooling_disc = (
        tooling_disc.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_polar_points(0.085, 8))
        .hole(0.007)
    )
    lightening_ring = _ring_plate_with_windows(
        outer_radius=0.106,
        inner_radius=0.040,
        height=0.012,
        z_center=0.042,
        window_radius=0.012,
        window_center_radius=0.062,
        window_count=6,
    )
    center_cap = _cylinder(radius=0.030, height=0.010, z_center=0.052)
    return _union_all(
        secondary_collar,
        tooling_disc,
        lightening_ring,
        center_cap,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_industrial_turntable", assets=ASSETS)

    model.material("machine_base", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("bearing_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("tooling_orange", rgba=(0.83, 0.39, 0.10, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_mesh = mesh_from_cadquery(_build_pedestal_shape(), "pedestal_base.obj", assets=ASSETS)
    pedestal_base.visual(pedestal_mesh, material="machine_base")





    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.180),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    slew_carrier = model.part("slew_carrier")
    slew_mesh = mesh_from_cadquery(_build_slew_shape(), "slew_carrier.obj", assets=ASSETS)
    slew_carrier.visual(slew_mesh, material="bearing_steel")


    _add_annular_box_collisions(
        slew_carrier,
        inner_radius=0.018,
        outer_radius=0.050,
        height=0.032,
        z_center=0.000,
    )


    slew_carrier.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.110),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    top_platter = model.part("top_platter")
    platter_mesh = mesh_from_cadquery(_build_platter_shape(), "top_platter.obj", assets=ASSETS)
    top_platter.visual(platter_mesh, material="tooling_orange")
    _add_annular_box_collisions(
        top_platter,
        inner_radius=0.018,
        outer_radius=0.048,
        height=0.030,
        z_center=0.000,
    )



    top_platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.122, length=0.065),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    model.articulation(
        "base_to_slew",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=slew_carrier,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PRIMARY_AXIS_LIMIT,
            upper=PRIMARY_AXIS_LIMIT,
            effort=80.0,
            velocity=1.2,
        ),
    )

    model.articulation(
        "slew_to_platter",
        ArticulationType.REVOLUTE,
        parent=slew_carrier,
        child=top_platter,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-SECONDARY_AXIS_LIMIT,
            upper=SECONDARY_AXIS_LIMIT,
            effort=45.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("slew_carrier", "pedestal_base", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("top_platter", "pedestal_base", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("top_platter", "slew_carrier", axes="xy", max_dist=0.005)

    ctx.expect_aabb_overlap("slew_carrier", "pedestal_base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_overlap("top_platter", "pedestal_base", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_overlap("top_platter", "slew_carrier", axes="xy", min_overlap=0.12)

    ctx.expect_aabb_gap("slew_carrier", "pedestal_base", axis="z", max_gap=0.006, max_penetration=0.045)
    ctx.expect_aabb_gap("top_platter", "slew_carrier", axis="z", max_gap=0.006, max_penetration=0.040)
    ctx.expect_aabb_gap("top_platter", "pedestal_base", axis="z", max_gap=0.030, max_penetration=0.0)
    ctx.expect_origin_gap("top_platter", "pedestal_base", axis="z", min_gap=0.010)

    with ctx.pose(base_to_slew=PRIMARY_AXIS_LIMIT):
        ctx.expect_origin_distance("slew_carrier", "pedestal_base", axes="xy", max_dist=0.005)
        ctx.expect_aabb_overlap("slew_carrier", "pedestal_base", axes="xy", min_overlap=0.18)
        ctx.expect_aabb_gap("slew_carrier", "pedestal_base", axis="z", max_gap=0.006, max_penetration=0.045)
        ctx.expect_origin_gap("top_platter", "pedestal_base", axis="z", min_gap=0.010)

    with ctx.pose(base_to_slew=-PRIMARY_AXIS_LIMIT, slew_to_platter=SECONDARY_AXIS_LIMIT):
        ctx.expect_origin_distance("top_platter", "slew_carrier", axes="xy", max_dist=0.005)
        ctx.expect_aabb_overlap("top_platter", "slew_carrier", axes="xy", min_overlap=0.12)
        ctx.expect_aabb_gap("top_platter", "slew_carrier", axis="z", max_gap=0.006, max_penetration=0.040)
        ctx.expect_origin_gap("top_platter", "pedestal_base", axis="z", min_gap=0.010)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

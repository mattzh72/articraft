from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _hollow_column(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
):
    """Thin-walled revolved metal sleeve with real clearance through its bore."""
    return _mesh(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _seat_cushion_mesh():
    # A rounded, gently crowned upholstered stool pad, revolved as one solid.
    return _mesh(
        LatheGeometry(
            [
                (0.000, 0.040),
                (0.176, 0.040),
                (0.216, 0.048),
                (0.238, 0.070),
                (0.235, 0.105),
                (0.205, 0.126),
                (0.118, 0.134),
                (0.000, 0.130),
            ],
            segments=96,
        ),
        "seat_cushion",
    )


def _dish_base_mesh():
    # Wide convex pedestal base with a soft rolled top edge, not a plain disk.
    return _mesh(
        LatheGeometry(
            [
                (0.000, 0.006),
                (0.205, 0.006),
                (0.268, 0.011),
                (0.292, 0.024),
                (0.282, 0.042),
                (0.226, 0.054),
                (0.072, 0.057),
                (0.000, 0.054),
            ],
            segments=96,
        ),
        "weighted_base",
    )


def _lever_rod_mesh():
    return _mesh(
        tube_from_spline_points(
            [
                (0.0, -0.010, 0.000),
                (0.0, -0.070, -0.010),
                (0.0, -0.132, -0.024),
                (0.0, -0.182, -0.034),
            ],
            radius=0.006,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
        "lever_rod",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospitality_swivel_bar_stool")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.13, 0.14, 0.15, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    upholstery = model.material("charcoal_vinyl", rgba=(0.035, 0.038, 0.042, 1.0))
    seam = model.material("soft_seam", rgba=(0.010, 0.011, 0.012, 1.0))

    base = model.part("base")
    base.visual(_dish_base_mesh(), material=chrome, name="weighted_base")
    base.visual(
        Cylinder(radius=0.274, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black_rubber,
        name="floor_grip",
    )
    base.visual(
        _mesh(TorusGeometry(radius=0.260, tube=0.007, radial_segments=18, tubular_segments=96), "base_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=chrome,
        name="rolled_rim",
    )
    base.visual(
        _hollow_column(
            outer_radius=0.052,
            inner_radius=0.039,
            z0=0.045,
            z1=0.365,
            name="outer_sleeve",
        ),
        material=chrome,
        name="outer_sleeve",
    )
    base.visual(
        _hollow_column(
            outer_radius=0.066,
            inner_radius=0.039,
            z0=0.338,
            z1=0.385,
            name="top_collar",
        ),
        material=brushed,
        name="top_collar",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=brushed,
        name="lower_boss",
    )

    lift_stage = model.part("lift_stage")
    lift_stage.visual(
        Cylinder(radius=0.031, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed,
        name="inner_piston",
    )
    lift_stage.visual(
        Cylinder(radius=0.045, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brushed,
        name="lower_stop",
    )
    lift_stage.visual(
        Cylinder(radius=0.080, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=dark_metal,
        name="upper_turntable",
    )
    lift_stage.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=brushed,
        name="footrest_collar",
    )
    lift_stage.visual(
        _mesh(
            TorusGeometry(radius=0.205, tube=0.012, radial_segments=18, tubular_segments=112),
            "footrest_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=chrome,
        name="footrest_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius_mid = 0.132
        lift_stage.visual(
            Cylinder(radius=0.010, length=0.150),
            origin=Origin(
                xyz=(radius_mid * math.cos(angle), radius_mid * math.sin(angle), 0.055),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=chrome,
            name=f"footrest_spoke_{index}",
        )
    lift_stage.visual(
        Cylinder(radius=0.095, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.349)),
        material=brushed,
        name="bearing_lip",
    )

    seat = model.part("seat")
    seat.visual(_seat_cushion_mesh(), material=upholstery, name="seat_cushion")
    seat.visual(
        _mesh(TorusGeometry(radius=0.235, tube=0.007, radial_segments=14, tubular_segments=96), "seat_piping"),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=seam,
        name="seat_piping",
    )
    seat.visual(
        _mesh(TorusGeometry(radius=0.125, tube=0.0025, radial_segments=10, tubular_segments=72), "top_stitching"),
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        material=seam,
        name="top_stitching",
    )
    seat.visual(
        Cylinder(radius=0.082, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_metal,
        name="seat_bearing",
    )
    seat.visual(
        Cylinder(radius=0.150, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brushed,
        name="mounting_plate",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        seat.visual(
            Box((0.118, 0.028, 0.012)),
            origin=Origin(
                xyz=(0.070 * math.cos(angle), 0.070 * math.sin(angle), 0.037),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"underbrace_{index}",
        )
    seat.visual(
        Box((0.160, 0.055, 0.014)),
        origin=Origin(xyz=(0.120, -0.102, 0.027)),
        material=dark_metal,
        name="lever_bridge",
    )
    seat.visual(
        Box((0.012, 0.045, 0.040)),
        origin=Origin(xyz=(0.080, -0.145, 0.020)),
        material=dark_metal,
        name="lever_ear_0",
    )
    seat.visual(
        Box((0.012, 0.045, 0.040)),
        origin=Origin(xyz=(0.160, -0.145, 0.020)),
        material=dark_metal,
        name="lever_ear_1",
    )

    lever = model.part("height_lever")
    lever.visual(
        Cylinder(radius=0.010, length=0.068),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="pivot_barrel",
    )
    lever.visual(_lever_rod_mesh(), material=brushed, name="lever_rod")
    lever.visual(
        Cylinder(radius=0.012, length=0.076),
        origin=Origin(xyz=(0.0, -0.194, -0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="handle_grip",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.22, lower=0.0, upper=0.200),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=lift_stage,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=3.2),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.120, -0.150, 0.016)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift = object_model.get_part("lift_stage")
    seat = object_model.get_part("seat")
    lever = object_model.get_part("height_lever")
    height_slide = object_model.get_articulation("height_slide")
    swivel = object_model.get_articulation("seat_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.check(
        "swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"seat_swivel type={swivel.articulation_type}",
    )
    ctx.check(
        "height travel is bar stool scale",
        height_slide.motion_limits is not None
        and height_slide.motion_limits.upper is not None
        and height_slide.motion_limits.upper >= 0.18,
        details=f"height limits={height_slide.motion_limits}",
    )
    ctx.expect_within(
        lift,
        base,
        axes="xy",
        inner_elem="inner_piston",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="piston centered in sleeve",
    )
    ctx.expect_overlap(
        lift,
        base,
        axes="z",
        elem_a="inner_piston",
        elem_b="outer_sleeve",
        min_overlap=0.20,
        name="lowered piston remains inserted",
    )
    ctx.expect_gap(
        seat,
        lift,
        axis="z",
        positive_elem="seat_bearing",
        negative_elem="upper_turntable",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat bearing sits on turntable",
    )

    rest_seat = ctx.part_world_position(seat)
    with ctx.pose({height_slide: 0.20}):
        raised_seat = ctx.part_world_position(seat)
        ctx.expect_within(
            lift,
            base,
            axes="xy",
            inner_elem="inner_piston",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="raised piston stays centered",
        )
        ctx.expect_overlap(
            lift,
            base,
            axes="z",
            elem_a="inner_piston",
            elem_b="outer_sleeve",
            min_overlap=0.035,
            name="raised piston retains sleeve insertion",
        )
    ctx.check(
        "height adjustment raises seat",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.18,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(lever, elem="handle_grip")
    with ctx.pose({lever_pivot: 0.45}):
        pulled_handle_aabb = ctx.part_element_world_aabb(lever, elem="handle_grip")
    rest_handle_z = None if rest_handle_aabb is None else (rest_handle_aabb[0][2] + rest_handle_aabb[1][2]) * 0.5
    pulled_handle_z = None if pulled_handle_aabb is None else (pulled_handle_aabb[0][2] + pulled_handle_aabb[1][2]) * 0.5
    ctx.check(
        "height lever pulls upward",
        rest_handle_z is not None and pulled_handle_z is not None and pulled_handle_z > rest_handle_z + 0.04,
        details=f"rest_z={rest_handle_z}, pulled_z={pulled_handle_z}",
    )

    rest_handle_xy = None
    swivel_handle_xy = None
    if rest_handle_aabb is not None:
        rest_handle_xy = (
            (rest_handle_aabb[0][0] + rest_handle_aabb[1][0]) * 0.5,
            (rest_handle_aabb[0][1] + rest_handle_aabb[1][1]) * 0.5,
        )
    with ctx.pose({swivel: math.pi / 2.0}):
        swivel_handle_aabb = ctx.part_element_world_aabb(lever, elem="handle_grip")
        if swivel_handle_aabb is not None:
            swivel_handle_xy = (
                (swivel_handle_aabb[0][0] + swivel_handle_aabb[1][0]) * 0.5,
                (swivel_handle_aabb[0][1] + swivel_handle_aabb[1][1]) * 0.5,
            )
    ctx.check(
        "swivel rotates seat assembly",
        rest_handle_xy is not None
        and swivel_handle_xy is not None
        and abs(rest_handle_xy[0] - swivel_handle_xy[0]) > 0.20
        and abs(rest_handle_xy[1] - swivel_handle_xy[1]) > 0.20,
        details=f"rest_xy={rest_handle_xy}, swivel_xy={swivel_handle_xy}",
    )

    return ctx.report()


object_model = build_object_model()

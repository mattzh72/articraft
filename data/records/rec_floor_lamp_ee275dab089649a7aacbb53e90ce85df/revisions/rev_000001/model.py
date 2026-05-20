from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_points(radius: float, y: float, segments: int = 18) -> list[tuple[float, float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), y, radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _build_base_geometry():
    profile = [
        (0.0, 0.0),
        (0.135, 0.0),
        (0.158, 0.004),
        (0.170, 0.012),
        (0.170, 0.028),
        (0.155, 0.038),
        (0.032, 0.044),
        (0.025, 0.050),
        (0.0, 0.050),
    ]
    return LatheGeometry(profile, segments=72)


def _build_swivel_geometry():
    swivel = CylinderGeometry(radius=0.019, height=0.036).translate(0.0, 0.0, 0.018)
    swivel.merge(CylinderGeometry(radius=0.010, height=0.050).translate(0.0, 0.0, 0.061))
    swivel.merge(
        CylinderGeometry(radius=0.009, height=0.072)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.036, 0.055)
    )
    swivel.merge(
        tube_from_spline_points(
            [(0.0, 0.0, 0.055), (0.0, 0.014, 0.078), (0.0, 0.028, 0.110)],
            radius=0.005,
            samples_per_segment=10,
            radial_segments=14,
        )
    )
    swivel.merge(
        tube_from_spline_points(
            [(-0.090, 0.060, 0.055), (-0.090, 0.028, 0.110), (0.090, 0.028, 0.110), (0.090, 0.060, 0.055)],
            radius=0.006,
            samples_per_segment=10,
            radial_segments=16,
        )
    )
    swivel.merge(
        CylinderGeometry(radius=0.014, height=0.006)
        .rotate_y(pi / 2.0)
        .translate(-0.090, 0.060, 0.055)
    )
    swivel.merge(
        CylinderGeometry(radius=0.014, height=0.006)
        .rotate_y(pi / 2.0)
        .translate(0.090, 0.060, 0.055)
    )
    return swivel


def _build_shade_geometry():
    cage = (
        CylinderGeometry(radius=0.012, height=0.066)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.053, 0.0)
    )

    for ring_radius, ring_y, wire_radius in (
        (0.040, 0.130, 0.004),
        (0.062, 0.220, 0.0035),
        (0.082, 0.320, 0.004),
    ):
        cage.merge(
            tube_from_spline_points(
                _circle_points(ring_radius, ring_y, segments=18),
                radius=wire_radius,
                closed_spline=True,
                cap_ends=False,
                samples_per_segment=4,
                radial_segments=16,
            )
        )

    rib_angles = [2.0 * pi * index / 8.0 for index in range(8)]
    for angle in rib_angles:
        cage.merge(
            tube_from_spline_points(
                [
                    (0.012 * cos(angle), 0.020, 0.012 * sin(angle)),
                    (0.024 * cos(angle), 0.078, 0.024 * sin(angle)),
                    (0.040 * cos(angle), 0.130, 0.040 * sin(angle)),
                    (0.054 * cos(angle), 0.180, 0.054 * sin(angle)),
                    (0.062 * cos(angle), 0.220, 0.062 * sin(angle)),
                    (0.074 * cos(angle), 0.272, 0.074 * sin(angle)),
                    (0.082 * cos(angle), 0.320, 0.082 * sin(angle)),
                ],
                radius=0.0028,
                samples_per_segment=10,
                radial_segments=12,
            )
        )

    return cage


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.19, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    black_enamel = model.material("black_enamel", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(_save_mesh("pharmacy_lamp_base", _build_base_geometry()), material=cast_iron, name="base_casting")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.050),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brushed_steel,
        name="lower_collar",
    )
    column.visual(
        Cylinder(radius=0.013, length=1.400),
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        material=brushed_steel,
        name="column_tube",
    )
    column.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.418)),
        material=brushed_steel,
        name="upper_collar",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 1.430)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
    )

    swivel_head = model.part("swivel_head")
    swivel_head.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=black_enamel,
        name="swivel_base",
    )
    swivel_head.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black_enamel,
        name="swivel_stem",
    )
    swivel_head.visual(
        Box((0.016, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, 0.030, 0.066)),
        material=black_enamel,
        name="yoke_stalk",
    )
    swivel_head.visual(
        Cylinder(radius=0.006, length=0.106),
        origin=Origin(xyz=(0.0, 0.045, 0.086), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_enamel,
        name="top_yoke_bridge",
    )
    swivel_head.visual(
        Box((0.006, 0.024, 0.038)),
        origin=Origin(xyz=(-0.050, 0.057, 0.073)),
        material=black_enamel,
        name="left_yoke_plate",
    )
    swivel_head.visual(
        Box((0.006, 0.024, 0.038)),
        origin=Origin(xyz=(0.050, 0.057, 0.073)),
        material=black_enamel,
        name="right_yoke_plate",
    )
    swivel_head.inertial = Inertial.from_geometry(
        Box((0.110, 0.090, 0.100)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.030, 0.055)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.008, length=0.094),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black_enamel,
        name="pivot_barrel",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_enamel,
        name="shade_neck",
    )
    shade.visual(
        _save_mesh("pharmacy_lamp_cage_shade_v4", _build_shade_geometry()),
        material=black_enamel,
        name="shade_cage",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.190, 0.330, 0.190)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.150, 0.0)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )
    model.articulation(
        "column_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=swivel_head,
        origin=Origin(xyz=(0.0, 0.0, 1.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=shade,
        origin=Origin(xyz=(0.0, 0.060, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    swivel_head = object_model.get_part("swivel_head")
    shade = object_model.get_part("shade")

    column_swivel = object_model.get_articulation("column_swivel")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.fail_if_articulation_overlaps(max_pose_samples=36)

    ctx.expect_contact(base, column, name="base_column_contact")
    ctx.expect_contact(column, swivel_head, name="column_swivel_contact")
    ctx.expect_contact(swivel_head, shade, name="shade_pivot_contact")
    ctx.expect_origin_gap(swivel_head, base, axis="z", min_gap=1.40, name="lamp_height_above_base")

    ctx.check(
        "column_swivel_axis_is_vertical",
        column_swivel.axis == (0.0, 0.0, 1.0),
        f"Expected vertical swivel axis, got {column_swivel.axis}",
    )
    ctx.check(
        "shade_tilt_axis_is_lateral",
        shade_tilt.axis == (1.0, 0.0, 0.0),
        f"Expected lateral tilt axis, got {shade_tilt.axis}",
    )
    ctx.check(
        "shade_tilt_limits_realistic",
        shade_tilt.motion_limits is not None
        and shade_tilt.motion_limits.lower is not None
        and shade_tilt.motion_limits.upper is not None
        and shade_tilt.motion_limits.lower <= -0.50
        and shade_tilt.motion_limits.upper >= 0.70,
        f"Unexpected tilt limits: {shade_tilt.motion_limits}",
    )

    base_size = _aabb_size(ctx.part_world_aabb(base))
    column_size = _aabb_size(ctx.part_world_aabb(column))
    shade_rest_aabb = ctx.part_element_world_aabb(shade, elem="shade_cage")
    shade_rest_size = _aabb_size(shade_rest_aabb)

    ctx.check(
        "base_diameter_realistic",
        base_size is not None and 0.32 <= base_size[0] <= 0.36 and 0.32 <= base_size[1] <= 0.36,
        f"Unexpected base footprint: {base_size}",
    )
    ctx.check(
        "column_is_slender_and_tall",
        column_size is not None and column_size[2] >= 1.40 and column_size[0] <= 0.04 and column_size[1] <= 0.04,
        f"Unexpected column envelope: {column_size}",
    )
    ctx.check(
        "shade_reads_as_cage_head",
        shade_rest_size is not None and shade_rest_size[1] >= 0.28 and shade_rest_size[0] >= 0.16 and shade_rest_size[2] >= 0.16,
        f"Unexpected shade size: {shade_rest_size}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    quarter_turn_shade_pos = None
    with ctx.pose({column_swivel: pi / 2.0}):
        quarter_turn_shade_pos = ctx.part_world_position(shade)
        ctx.expect_contact(column, swivel_head, name="column_swivel_quarter_contact")
        ctx.expect_contact(swivel_head, shade, name="shade_quarter_turn_contact")
        ctx.fail_if_isolated_parts(name="column_swivel_quarter_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="column_swivel_quarter_no_overlap")

    ctx.check(
        "column_swivel_moves_head_around_tip",
        rest_shade_pos is not None
        and quarter_turn_shade_pos is not None
        and abs(quarter_turn_shade_pos[0]) >= 0.05
        and abs(rest_shade_pos[1] - quarter_turn_shade_pos[1]) >= 0.05,
        f"Rest/quarter-turn shade positions: {rest_shade_pos}, {quarter_turn_shade_pos}",
    )

    lower_tilt_aabb = None
    upper_tilt_aabb = None
    limits = shade_tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({shade_tilt: limits.lower}):
            lower_tilt_aabb = ctx.part_element_world_aabb(shade, elem="shade_cage")
            ctx.expect_contact(swivel_head, shade, name="shade_tilt_lower_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_tilt_lower_no_floating")
        with ctx.pose({shade_tilt: limits.upper}):
            upper_tilt_aabb = ctx.part_element_world_aabb(shade, elem="shade_cage")
            ctx.expect_contact(swivel_head, shade, name="shade_tilt_upper_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_tilt_upper_no_floating")

    ctx.check(
        "shade_tilt_changes_head_pitch",
        lower_tilt_aabb is not None
        and upper_tilt_aabb is not None
        and upper_tilt_aabb[1][2] - lower_tilt_aabb[1][2] >= 0.10,
        f"Lower/upper tilt AABBs: {lower_tilt_aabb}, {upper_tilt_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

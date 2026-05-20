from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lathed_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 56,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _add_vertical_bolt(
    part,
    *,
    x: float,
    y: float,
    z_bottom: float,
    shank_length: float,
    shank_radius: float,
    head_length: float,
    head_radius: float,
    material,
    name_prefix: str,
) -> None:
    part.visual(
        Cylinder(radius=shank_radius, length=shank_length),
        origin=Origin(xyz=(x, y, z_bottom + 0.5 * shank_length)),
        material=material,
        name=f"{name_prefix}_shank",
    )
    part.visual(
        Cylinder(radius=head_radius, length=head_length),
        origin=Origin(xyz=(x, y, z_bottom + shank_length + 0.5 * head_length)),
        material=material,
        name=f"{name_prefix}_head",
    )


def _add_radial_bolts(
    part,
    *,
    radius: float,
    count: int,
    phase: float,
    z_bottom: float,
    shank_length: float,
    shank_radius: float,
    head_length: float,
    head_radius: float,
    material,
    prefix: str,
) -> None:
    for index in range(count):
        angle = phase + ((2.0 * math.pi * index) / count)
        _add_vertical_bolt(
            part,
            x=radius * math.cos(angle),
            y=radius * math.sin(angle),
            z_bottom=z_bottom,
            shank_length=shank_length,
            shank_radius=shank_radius,
            head_length=head_length,
            head_radius=head_radius,
            material=material,
            name_prefix=f"{prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_swivel_bar_stool")

    powder_coat = model.material("powder_coat", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    seat_black = model.material("seat_black", rgba=(0.12, 0.12, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.73, 0.14, 1.0))

    foot_ring_mesh = _mesh(
        "foot_ring",
        TorusGeometry(
            radius=0.170,
            tube=0.013,
            radial_segments=16,
            tubular_segments=56,
        ),
    )
    guard_collar_mesh = _mesh(
        "guard_collar",
        _lathed_shell(
            outer_radius=0.072,
            inner_radius=0.050,
            z0=0.566,
            z1=0.638,
            segments=64,
        ),
    )
    bearing_skirt_mesh = _mesh(
        "bearing_skirt",
        _lathed_shell(
            outer_radius=0.086,
            inner_radius=0.074,
            z0=0.014,
            z1=0.060,
            segments=64,
        ),
    )

    support_frame = model.part("support_frame")
    support_frame.visual(
        Cylinder(radius=0.260, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=powder_coat,
        name="base_plate",
    )
    support_frame.visual(
        Cylinder(radius=0.180, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=powder_coat,
        name="ballast_disc",
    )
    support_frame.visual(
        Cylinder(radius=0.100, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=steel,
        name="base_flange",
    )
    support_frame.visual(
        Cylinder(radius=0.060, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=powder_coat,
        name="lower_reinforcement_sleeve",
    )
    support_frame.visual(
        Cylinder(radius=0.045, length=0.548),
        origin=Origin(xyz=(0.0, 0.0, 0.336)),
        material=powder_coat,
        name="center_column",
    )
    for name, xyz, size, rpy in [
        ("base_gusset_pos_x", (0.077, 0.0, 0.114), (0.110, 0.012, 0.120), (0.0, 0.0, 0.0)),
        ("base_gusset_neg_x", (-0.077, 0.0, 0.114), (0.110, 0.012, 0.120), (0.0, 0.0, 0.0)),
        ("base_gusset_pos_y", (0.0, 0.077, 0.114), (0.012, 0.110, 0.120), (0.0, 0.0, 0.0)),
        ("base_gusset_neg_y", (0.0, -0.077, 0.114), (0.012, 0.110, 0.120), (0.0, 0.0, 0.0)),
    ]:
        support_frame.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=steel,
            name=name,
        )
    support_frame.visual(
        Cylinder(radius=0.062, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.559)),
        material=steel,
        name="upper_reinforcement_sleeve",
    )
    for name, xyz, size in [
        ("upper_rib_pos_x", (0.044, 0.0, 0.574), (0.050, 0.010, 0.090)),
        ("upper_rib_neg_x", (-0.044, 0.0, 0.574), (0.050, 0.010, 0.090)),
        ("upper_rib_pos_y", (0.0, 0.044, 0.574), (0.010, 0.050, 0.090)),
        ("upper_rib_neg_y", (0.0, -0.044, 0.574), (0.010, 0.050, 0.090)),
    ]:
        support_frame.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=steel,
            name=name,
        )
    support_frame.visual(
        Cylinder(radius=0.092, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.617)),
        material=steel,
        name="top_flange",
    )
    support_frame.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=steel,
        name="swivel_hub_core",
    )
    support_frame.visual(
        Cylinder(radius=0.078, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.631)),
        material=steel,
        name="lower_bearing_plate",
    )
    support_frame.visual(
        guard_collar_mesh,
        material=powder_coat,
        name="guard_collar",
    )
    support_frame.visual(
        Box((0.018, 0.034, 0.028)),
        origin=Origin(xyz=(-0.182, 0.0, 0.652)),
        material=steel,
        name="stop_bar_pos",
    )
    support_frame.visual(
        Box((0.018, 0.034, 0.028)),
        origin=Origin(xyz=(0.182, 0.0, 0.652)),
        material=steel,
        name="stop_bar_neg",
    )
    support_frame.visual(
        Box((0.144, 0.018, 0.016)),
        origin=Origin(xyz=(-0.110, 0.0, 0.608)),
        material=steel,
        name="stop_bar_pos_tie",
    )
    support_frame.visual(
        Box((0.144, 0.018, 0.016)),
        origin=Origin(xyz=(0.110, 0.0, 0.608)),
        material=steel,
        name="stop_bar_neg_tie",
    )
    support_frame.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(-0.182, 0.0, 0.630)),
        material=steel,
        name="stop_bar_pos_post",
    )
    support_frame.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(0.182, 0.0, 0.630)),
        material=steel,
        name="stop_bar_neg_post",
    )
    support_frame.visual(
        Box((0.030, 0.118, 0.020)),
        origin=Origin(xyz=(-0.060, -0.081, 0.622)),
        material=steel,
        name="lockout_receiver_post",
    )
    support_frame.visual(
        Box((0.040, 0.024, 0.010)),
        origin=Origin(xyz=(-0.065, -0.122, 0.627)),
        material=steel,
        name="lockout_receiver",
    )
    _add_radial_bolts(
        support_frame,
        radius=0.070,
        count=4,
        phase=math.pi / 4.0,
        z_bottom=0.604,
        shank_length=0.026,
        shank_radius=0.004,
        head_length=0.006,
        head_radius=0.007,
        material=steel,
        prefix="swivel_stack_bolt",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.52, 0.52, 0.66)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
    )

    footrest = model.part("footrest")
    footrest.visual(
        foot_ring_mesh,
        material=steel,
        name="foot_ring",
    )
    footrest.visual(
        Box((0.116, 0.018, 0.022)),
        origin=Origin(xyz=(0.101, 0.0, 0.0)),
        material=steel,
        name="right_bracket",
    )
    footrest.visual(
        Box((0.116, 0.018, 0.022)),
        origin=Origin(xyz=(-0.101, 0.0, 0.0)),
        material=steel,
        name="left_bracket",
    )
    footrest.visual(
        Box((0.018, 0.116, 0.022)),
        origin=Origin(xyz=(0.0, 0.101, 0.0)),
        material=steel,
        name="front_bracket",
    )
    footrest.visual(
        Box((0.018, 0.116, 0.022)),
        origin=Origin(xyz=(0.0, -0.101, 0.0)),
        material=steel,
        name="rear_bracket",
    )
    footrest.inertial = Inertial.from_geometry(
        Cylinder(radius=0.185, length=0.030),
        mass=4.5,
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.078, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=steel,
        name="upper_bearing_plate",
    )
    seat_stage.visual(
        bearing_skirt_mesh,
        material=powder_coat,
        name="bearing_skirt",
    )
    seat_stage.visual(
        Cylinder(radius=0.055, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=steel,
        name="hub_sleeve",
    )
    for name, xyz, size in [
        ("gusset_pos_x", (0.060, 0.0, 0.050), (0.120, 0.012, 0.072)),
        ("gusset_neg_x", (-0.060, 0.0, 0.050), (0.120, 0.012, 0.072)),
        ("gusset_pos_y", (0.0, 0.060, 0.050), (0.012, 0.120, 0.072)),
        ("gusset_neg_y", (0.0, -0.060, 0.050), (0.012, 0.120, 0.072)),
    ]:
        seat_stage.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=steel,
            name=name,
        )
    seat_stage.visual(
        Box((0.280, 0.280, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=steel,
        name="seat_mount_plate",
    )
    seat_stage.visual(
        Cylinder(radius=0.190, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=powder_coat,
        name="seat_pan",
    )
    seat_stage.visual(
        Cylinder(radius=0.176, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material=seat_black,
        name="seat_cushion",
    )
    seat_stage.visual(
        Box((0.018, 0.092, 0.016)),
        origin=Origin(xyz=(0.0, 0.111, 0.012)),
        material=steel,
        name="stop_arm",
    )
    seat_stage.visual(
        Box((0.014, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.156, 0.012)),
        material=safety_yellow,
        name="stop_lug",
    )
    seat_stage.visual(
        Box((0.062, 0.050, 0.040)),
        origin=Origin(xyz=(-0.065, -0.114, 0.056)),
        material=steel,
        name="lockout_bracket",
    )
    seat_stage.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(-0.069, -0.122, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lockout_guide",
    )
    seat_stage.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(-0.112, -0.122, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_yellow,
        name="lockout_handle",
    )
    seat_stage.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(-0.065, -0.122, 0.018)),
        material=safety_yellow,
        name="lockout_pin",
    )
    _add_radial_bolts(
        seat_stage,
        radius=0.162,
        count=4,
        phase=math.pi / 4.0,
        z_bottom=0.074,
        shank_length=0.034,
        shank_radius=0.004,
        head_length=0.006,
        head_radius=0.008,
        material=steel,
        prefix="seat_pan_bolt",
    )
    seat_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.180),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    model.articulation(
        "support_to_footrest",
        ArticulationType.FIXED,
        parent=support_frame,
        child=footrest,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.638)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=1.6,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    footrest = object_model.get_part("footrest")
    seat_stage = object_model.get_part("seat_stage")
    seat_swivel = object_model.get_articulation("seat_swivel")

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

    limits = seat_swivel.motion_limits
    axis_is_vertical = tuple(round(value, 6) for value in seat_swivel.axis) == (0.0, 0.0, 1.0)
    limits_are_expected = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(limits.lower, -math.pi / 2.0, abs_tol=1e-6)
        and math.isclose(limits.upper, math.pi / 2.0, abs_tol=1e-6)
    )
    ctx.check(
        "seat_swivel_is_vertical_and_limited",
        axis_is_vertical and limits_are_expected,
        details="Seat swivel should rotate about +Z with ±90 degree safety stops.",
    )

    ctx.expect_contact(
        footrest,
        support_frame,
        elem_a="front_bracket",
        elem_b="center_column",
        name="footrest_is_carried_by_column",
    )
    ctx.expect_contact(
        seat_stage,
        support_frame,
        elem_a="upper_bearing_plate",
        elem_b="lower_bearing_plate",
        name="seat_stage_bearing_stack_is_supported",
    )
    ctx.expect_overlap(
        seat_stage,
        support_frame,
        elem_a="upper_bearing_plate",
        elem_b="lower_bearing_plate",
        axes="xy",
        min_overlap=0.150,
        name="seat_swivel_is_centered_on_bearing_stack",
    )
    ctx.expect_origin_gap(
        seat_stage,
        support_frame,
        axis="z",
        min_gap=0.62,
        max_gap=0.66,
        name="seat_height_matches_bar_stool_range",
    )
    ctx.expect_overlap(
        seat_stage,
        support_frame,
        elem_a="lockout_pin",
        elem_b="lockout_receiver",
        axes="xy",
        min_overlap=0.010,
        name="lockout_pin_is_indexed_over_receiver",
    )
    ctx.expect_gap(
        seat_stage,
        support_frame,
        axis="z",
        positive_elem="lockout_pin",
        negative_elem="lockout_receiver",
        min_gap=0.001,
        max_gap=0.004,
        name="lockout_pin_sits_just_above_receiver_slot",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({seat_swivel: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_positive_stop")
            ctx.expect_contact(
                seat_stage,
                support_frame,
                elem_a="stop_lug",
                elem_b="stop_bar_pos",
                name="positive_overtravel_stop_contacts",
            )
    if limits is not None and limits.lower is not None:
        with ctx.pose({seat_swivel: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_negative_stop")
            ctx.expect_contact(
                seat_stage,
                support_frame,
                elem_a="stop_lug",
                elem_b="stop_bar_neg",
                name="negative_overtravel_stop_contacts",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

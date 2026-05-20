from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * cos(angle), radius * sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_bar_stool", assets=ASSETS)

    matte_upholstery = model.material("matte_upholstery", rgba=(0.72, 0.70, 0.66, 1.0))
    matte_trim = model.material("matte_trim", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.30, 0.31, 0.33, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.73, 0.75, 1.0))
    soft_black = model.material("soft_black", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.223, length=0.043),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
    )
    base_profile = [
        (0.0, 0.0),
        (0.110, 0.0),
        (0.186, 0.003),
        (0.220, 0.011),
        (0.223, 0.024),
        (0.205, 0.033),
        (0.082, 0.035),
        (0.0, 0.035),
    ]
    base.visual(
        _save_mesh("bar_stool_base_plate.obj", LatheGeometry(base_profile, segments=72)),
        material=satin_graphite,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.168, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=soft_black,
        name="glide_ring",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=matte_trim,
        name="top_pad",
    )

    column = model.part("column")
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.560),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
    )
    lower_collar_profile = [
        (0.034, 0.0),
        (0.074, 0.0),
        (0.078, 0.006),
        (0.066, 0.020),
        (0.055, 0.038),
        (0.048, 0.055),
        (0.046, 0.060),
        (0.034, 0.018),
        (0.034, 0.0),
    ]
    column.visual(
        _save_mesh("bar_stool_lower_collar.obj", LatheGeometry(lower_collar_profile, segments=72)),
        material=satin_graphite,
        name="lower_collar",
    )
    column.visual(
        _save_mesh(
            "bar_stool_outer_sleeve.obj",
            LatheGeometry.from_shell_profiles(
                [(0.046, 0.055), (0.046, 0.516)],
                [(0.0315, 0.055), (0.0315, 0.516)],
                segments=72,
            ),
        ),
        material=satin_aluminum,
        name="outer_sleeve",
    )
    column.visual(
        _save_mesh(
            "bar_stool_upper_break_ring.obj",
            LatheGeometry.from_shell_profiles(
                [(0.050, 0.516), (0.050, 0.524)],
                [(0.033, 0.516), (0.033, 0.524)],
                segments=72,
            ),
        ),
        material=matte_trim,
        name="upper_break_ring",
    )
    column.visual(
        _save_mesh(
            "bar_stool_guide_bushing.obj",
            LatheGeometry.from_shell_profiles(
                [(0.052, 0.524), (0.052, 0.548)],
                [(0.0315, 0.524), (0.0315, 0.548)],
                segments=72,
            ),
        ),
        material=satin_graphite,
        name="guide_bushing",
    )
    column.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.551)),
        material=satin_graphite,
        name="top_bearing_race",
    )
    footrest_geom = TorusGeometry(
        0.170,
        0.012,
        radial_segments=18,
        tubular_segments=54,
    ).translate(0.0, 0.0, 0.285)
    footrest_geom.merge(
        TorusGeometry(
            0.058,
            0.014,
            radial_segments=16,
            tubular_segments=42,
        ).translate(0.0, 0.0, 0.238)
    )
    for brace_angle in (pi / 6.0, 5.0 * pi / 6.0, 3.0 * pi / 2.0):
        brace = tube_from_spline_points(
            [
                _polar(0.060, brace_angle, 0.238),
                _polar(0.116, brace_angle, 0.255),
                _polar(0.162, brace_angle, 0.285),
            ],
            radius=0.010,
            samples_per_segment=10,
            radial_segments=16,
        )
        footrest_geom.merge(brace)
    column.visual(
        _save_mesh("bar_stool_footrest_assembly.obj", footrest_geom),
        material=satin_aluminum,
        name="footrest_assembly",
    )

    seat_support = model.part("seat_support")
    seat_support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.030),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )
    seat_support.visual(
        Cylinder(radius=0.056, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_aluminum,
        name="lower_swivel_ring",
    )
    seat_support.visual(
        Cylinder(radius=0.042, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_graphite,
        name="center_hub",
    )
    seat_support.visual(
        Cylinder(radius=0.118, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=satin_graphite,
        name="support_disc",
    )
    for screw_index, screw_angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        sx, sy, sz = _polar(0.050, screw_angle, 0.0205)
        seat_support.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(sx, sy, sz)),
            material=satin_aluminum,
            name=f"disc_fastener_{screw_index}",
        )
    seat_support.visual(
        Box((0.020, 0.024, 0.016)),
        origin=Origin(xyz=(0.118, 0.0, 0.011)),
        material=satin_graphite,
        name="lever_mount",
    )
    seat_support.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(xyz=(0.118, 0.0, 0.011), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="lever_pivot_pin",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.186, length=0.106),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
    )
    seat_shell_profile = [
        (0.0, 0.0),
        (0.090, 0.0),
        (0.135, 0.004),
        (0.158, 0.014),
        (0.163, 0.026),
        (0.156, 0.032),
        (0.0, 0.032),
    ]
    seat_cushion_profile = [
        (0.0, 0.030),
        (0.090, 0.032),
        (0.150, 0.040),
        (0.175, 0.058),
        (0.186, 0.080),
        (0.174, 0.100),
        (0.106, 0.106),
        (0.0, 0.100),
    ]
    seat.visual(
        _save_mesh("bar_stool_seat_shell.obj", LatheGeometry(seat_shell_profile, segments=72)),
        material=matte_trim,
        name="seat_shell",
    )
    seat.visual(
        _save_mesh("bar_stool_seat_cushion.obj", LatheGeometry(seat_cushion_profile, segments=72)),
        material=matte_upholstery,
        name="seat_cushion",
    )
    seat.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=matte_upholstery,
        name="center_softening",
    )

    adjustment_lever = model.part("adjustment_lever")
    adjustment_lever.inertial = Inertial.from_geometry(
        Box((0.090, 0.024, 0.024)),
        mass=0.25,
        origin=Origin(xyz=(0.045, 0.0, -0.018)),
    )
    adjustment_lever.visual(
        _save_mesh(
            "bar_stool_adjustment_lever_body.obj",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (0.012, 0.0, -0.006),
                    (0.044, 0.0, -0.019),
                    (0.078, 0.0, -0.026),
                ],
                radius=0.006,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=satin_aluminum,
        name="lever_body",
    )
    adjustment_lever.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.082, 0.0, -0.026)),
        material=soft_black,
        name="lever_tip",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )
    model.articulation(
        "spindle_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat_support,
        origin=Origin(xyz=(0.0, 0.0, 0.554)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    model.articulation(
        "support_to_seat",
        ArticulationType.FIXED,
        parent=seat_support,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat_support,
        child=adjustment_lever,
        origin=Origin(xyz=(0.118, 0.0, 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=-0.35, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    seat_support = object_model.get_part("seat_support")
    seat = object_model.get_part("seat")
    adjustment_lever = object_model.get_part("adjustment_lever")

    spindle_swivel = object_model.get_articulation("spindle_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    top_pad = base.get_visual("top_pad")
    lower_collar = column.get_visual("lower_collar")
    upper_break_ring = column.get_visual("upper_break_ring")
    top_bearing_race = column.get_visual("top_bearing_race")
    lower_swivel_ring = seat_support.get_visual("lower_swivel_ring")
    support_disc = seat_support.get_visual("support_disc")
    lever_mount = seat_support.get_visual("lever_mount")
    lever_pivot_pin = seat_support.get_visual("lever_pivot_pin")
    seat_shell = seat.get_visual("seat_shell")
    lever_body = adjustment_lever.get_visual("lever_body")

    ctx.allow_overlap(
        adjustment_lever,
        seat_support,
        elem_a=lever_body,
        elem_b=lever_mount,
        reason="The gas-lift lever body nests into a recessed hinge bracket at the pivot.",
    )
    ctx.allow_overlap(
        adjustment_lever,
        seat_support,
        elem_a=lever_body,
        elem_b=lever_pivot_pin,
        reason="The lever rotates around a retained pivot pin that passes through the lever root.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_origin_distance(base, column, axes="xy", max_dist=0.001, name="column_centered_on_base")
    ctx.expect_origin_distance(base, seat, axes="xy", max_dist=0.0015, name="seat_centered_on_pedestal")
    ctx.expect_origin_gap(seat, base, axis="z", min_gap=0.60, max_gap=0.66, name="bar_height_origin_gap")
    ctx.expect_contact(column, base, elem_a=lower_collar, elem_b=top_pad, name="column_seated_on_base_pad")
    ctx.expect_contact(
        seat_support,
        column,
        elem_a=lower_swivel_ring,
        elem_b=top_bearing_race,
        name="visible_swivel_bearing_contact",
    )
    ctx.expect_contact(
        seat_support,
        seat,
        elem_a=support_disc,
        elem_b=seat_shell,
        name="seat_shell_supported",
    )
    ctx.expect_gap(seat_support, column, axis="z", positive_elem=support_disc, negative_elem=upper_break_ring, min_gap=0.03, max_gap=0.05, name="swivel_interface_exposed")
    ctx.expect_overlap(seat, base, axes="xy", min_overlap=0.36, name="seat_footprint_over_base")
    ctx.expect_overlap(
        seat,
        seat_support,
        axes="xy",
        elem_a=seat_shell,
        elem_b=support_disc,
        min_overlap=0.18,
        name="support_disc_under_seat",
    )

    with ctx.pose({spindle_swivel: pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="swivel_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="swivel_quarter_turn_no_floating")
        ctx.expect_contact(
            seat_support,
            column,
            elem_a=lower_swivel_ring,
            elem_b=top_bearing_race,
            name="swivel_quarter_turn_bearing_contact",
        )

    lever_limits = lever_pivot.motion_limits
    if lever_limits is not None and lever_limits.lower is not None and lever_limits.upper is not None:
        with ctx.pose({lever_pivot: lever_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lever_lower_pose_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="lever_lower_pose_no_floating")
        with ctx.pose({lever_pivot: lever_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lever_upper_pose_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="lever_upper_pose_no_floating")

    if lever_limits is not None and lever_limits.lower is not None:
        with ctx.pose(
            {
                spindle_swivel: pi / 3.0,
                lever_pivot: lever_limits.lower,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_operating_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_operating_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

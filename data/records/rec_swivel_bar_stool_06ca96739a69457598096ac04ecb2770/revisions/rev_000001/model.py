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
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _section_loop(
    width: float,
    depth: float,
    z: float,
    *,
    exponent: float = 2.2,
    segments: int = 48,
) -> tuple[tuple[float, float, float], ...]:
    return tuple((x, y, z) for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_swivel_bar_stool")

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.24, 0.26, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    service_panel = model.material("service_panel", rgba=(0.44, 0.28, 0.16, 1.0))
    seat_pan_black = model.material("seat_pan_black", rgba=(0.10, 0.11, 0.12, 1.0))
    vinyl_brown = model.material("vinyl_brown", rgba=(0.36, 0.23, 0.15, 1.0))

    base_cover_mesh = _save_mesh(
        "retrofit_stool_base_cover",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.135, 0.000),
                (0.205, 0.008),
                (0.245, 0.020),
                (0.272, 0.034),
                (0.272, 0.044),
                (0.188, 0.050),
                (0.0, 0.050),
            ],
            segments=72,
        ),
    )
    seat_pan_mesh = _save_mesh(
        "retrofit_stool_seat_pan",
        LatheGeometry(
            [
                (0.0, 0.046),
                (0.060, 0.046),
                (0.112, 0.048),
                (0.155, 0.050),
                (0.172, 0.055),
                (0.176, 0.064),
                (0.0, 0.064),
            ],
            segments=72,
        ),
    )
    foot_ring_mesh = _save_mesh(
        "retrofit_stool_foot_ring",
        TorusGeometry(radius=0.170, tube=0.012, radial_segments=18, tubular_segments=64),
    )
    foot_ring_brace_mesh = _save_mesh(
        "retrofit_stool_foot_ring_brace",
        tube_from_spline_points(
            [
                (0.056, 0.000, 0.284),
                (0.112, 0.000, 0.294),
                (0.160, 0.000, 0.300),
            ],
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    seat_brace_mesh = _save_mesh(
        "retrofit_stool_seat_brace",
        tube_from_spline_points(
            [
                (0.045, 0.000, 0.048),
                (0.085, 0.000, 0.056),
                (0.128, 0.000, 0.062),
            ],
            radius=0.009,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    seat_cushion_mesh = _save_mesh(
        "retrofit_stool_seat_cushion",
        section_loft(
            [
                _section_loop(0.322, 0.322, 0.000, exponent=2.2),
                _section_loop(0.350, 0.350, 0.014, exponent=2.1),
                _section_loop(0.336, 0.336, 0.028, exponent=2.0),
            ]
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(base_cover_mesh, material=cast_iron, name="base_cover")
    pedestal_base.visual(
        Cylinder(radius=0.048, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=machine_gray,
        name="column_tube",
    )
    pedestal_base.visual(
        Cylinder(radius=0.082, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=cast_iron,
        name="service_drum",
    )
    pedestal_base.visual(
        Cylinder(radius=0.090, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=zinc_steel,
        name="lower_adapter_flange",
    )
    pedestal_base.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=machine_gray,
        name="upper_neck",
    )
    pedestal_base.visual(
        Cylinder(radius=0.095, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.663)),
        material=cast_iron,
        name="bearing_housing",
    )
    pedestal_base.visual(
        Cylinder(radius=0.120, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.694)),
        material=zinc_steel,
        name="bearing_top_plate",
    )
    pedestal_base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=zinc_steel,
        name="foot_ring",
    )
    for brace_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal_base.visual(
            foot_ring_brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=machine_gray,
            name=f"foot_ring_brace_{brace_index}",
        )

    pedestal_base.visual(
        Box((0.010, 0.095, 0.150)),
        origin=Origin(xyz=(0.078, 0.0, 0.205)),
        material=service_panel,
        name="service_hatch_lower",
    )
    for bolt_index, (y_pos, z_pos) in enumerate(
        (
            (-0.031, 0.155),
            (0.031, 0.155),
            (-0.031, 0.255),
            (0.031, 0.255),
        )
    ):
        pedestal_base.visual(
            Cylinder(radius=0.0065, length=0.010),
            origin=Origin(xyz=(0.088, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc_steel,
            name=f"service_hatch_lower_bolt_{bolt_index}",
        )
    for bolt_index in range(6):
        angle = (2.0 * math.pi * bolt_index) / 6.0
        pedestal_base.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(
                xyz=(0.071 * math.cos(angle), 0.071 * math.sin(angle), 0.067),
            ),
            material=zinc_steel,
            name=f"lower_adapter_bolt_{bolt_index}",
        )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.544, 0.544, 0.700)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        Cylinder(radius=0.105, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=zinc_steel,
        name="rotor_collar",
    )
    seat_stage.visual(
        Cylinder(radius=0.122, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=zinc_steel,
        name="rotor_flange",
    )
    seat_stage.visual(
        Cylinder(radius=0.058, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=machine_gray,
        name="adapter_tube",
    )
    for brace_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        seat_stage.visual(
            seat_brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=machine_gray,
            name=f"seat_brace_{brace_index}",
        )
    seat_stage.visual(seat_pan_mesh, material=seat_pan_black, name="seat_pan")
    seat_stage.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=vinyl_brown,
        name="seat_cushion",
    )
    seat_stage.visual(
        Box((0.080, 0.010, 0.055)),
        origin=Origin(xyz=(0.0, 0.056, 0.045)),
        material=service_panel,
        name="service_hatch_upper",
    )
    for bolt_index, (x_pos, z_pos) in enumerate(
        (
            (-0.026, 0.029),
            (0.026, 0.029),
            (-0.026, 0.061),
            (0.026, 0.061),
        )
    ):
        seat_stage.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(xyz=(x_pos, 0.066, z_pos), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc_steel,
            name=f"service_hatch_upper_bolt_{bolt_index}",
        )
    for bolt_index in range(6):
        angle = (2.0 * math.pi * bolt_index) / 6.0
        seat_stage.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(
                xyz=(0.085 * math.cos(angle), 0.085 * math.sin(angle), 0.035),
            ),
            material=zinc_steel,
            name=f"rotor_flange_bolt_{bolt_index}",
        )
    seat_stage.inertial = Inertial.from_geometry(
        Box((0.370, 0.370, 0.100)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    model.articulation(
        "pedestal_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    seat_stage = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("pedestal_to_seat_swivel")

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

    ctx.check(
        "swivel_joint_is_vertical_continuous",
        swivel.axis == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower is None
        and swivel.motion_limits.upper is None,
        details="Seat swivel should be a centered continuous vertical-axis rotation.",
    )
    ctx.expect_origin_distance(
        seat_stage,
        pedestal_base,
        axes="xy",
        max_dist=0.001,
        name="seat_stage_centered_over_pedestal",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_contact(
            seat_stage,
            pedestal_base,
            elem_a="rotor_collar",
            elem_b="bearing_top_plate",
            contact_tol=0.0005,
            name="bearing_stack_support_closed_pose",
        )
        ctx.expect_overlap(
            seat_stage,
            pedestal_base,
            axes="xy",
            elem_a="rotor_collar",
            elem_b="bearing_top_plate",
            min_overlap=0.190,
            name="bearing_stack_has_full_support_footprint",
        )
        ctx.expect_gap(
            seat_stage,
            pedestal_base,
            axis="z",
            positive_elem="seat_pan",
            negative_elem="bearing_top_plate",
            min_gap=0.030,
            max_gap=0.070,
            name="seat_pan_clears_fixed_bearing_stack",
        )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_contact(
            seat_stage,
            pedestal_base,
            elem_a="rotor_collar",
            elem_b="bearing_top_plate",
            contact_tol=0.0005,
            name="bearing_stack_support_quarter_turn_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

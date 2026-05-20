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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    place_on_surface,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annular_mesh(name: str, outer_radius: float, inner_radius: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=56),
            [_circle_profile(inner_radius, segments=56)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_swivel_bar_stool")

    powder_coat = model.material("powder_coat", rgba=(0.22, 0.23, 0.24, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    polymer = model.material("polymer", rgba=(0.17, 0.18, 0.20, 1.0))
    wear_poly = model.material("wear_poly", rgba=(0.64, 0.56, 0.23, 1.0))
    hardware = model.material("hardware", rgba=(0.48, 0.50, 0.53, 1.0))

    bearing_housing_mesh = _annular_mesh("bearing_housing_ring", 0.105, 0.045, 0.060)
    thrust_plate_mesh = _annular_mesh("thrust_plate_ring", 0.112, 0.050, 0.020)
    lower_bearing_mesh = _annular_mesh("lower_bearing_plate", 0.102, 0.047, 0.012)
    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.160, tube=0.012, radial_segments=18, tubular_segments=48),
        "foot_ring",
    )
    seat_pad_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.130, 0.0),
                (0.165, 0.004),
                (0.178, 0.018),
                (0.180, 0.034),
                (0.170, 0.049),
                (0.145, 0.056),
                (0.0, 0.056),
            ],
            segments=72,
        ),
        "seat_pad_profile",
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.230, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=powder_coat,
        name="base_plate",
    )
    pedestal_base.visual(
        Cylinder(radius=0.140, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=steel_dark,
        name="base_sleeve",
    )
    pedestal_base.visual(
        Cylinder(radius=0.085, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=powder_coat,
        name="column_shroud",
    )
    pedestal_base.visual(
        Cylinder(radius=0.065, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.327)),
        material=powder_coat,
        name="main_column",
    )
    pedestal_base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=machined_steel,
        name="foot_ring",
    )
    for index in range(4):
        angle = index * (math.pi / 2.0)
        pedestal_base.visual(
            Box((0.118, 0.018, 0.030)),
            origin=Origin(
                xyz=(0.093 * math.cos(angle), 0.093 * math.sin(angle), 0.290),
                rpy=(0.0, 0.0, angle),
            ),
            material=machined_steel,
            name=f"foot_ring_bracket_{index + 1}",
        )
    pedestal_base.visual(
        bearing_housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.592)),
        material=powder_coat,
        name="bearing_housing",
    )
    pedestal_base.visual(
        thrust_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.632)),
        material=machined_steel,
        name="upper_thrust_plate",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        pedestal_base.visual(
            Cylinder(radius=0.016, length=0.020),
            origin=Origin(
                xyz=(0.074 * math.cos(angle), 0.074 * math.sin(angle), 0.632),
            ),
            material=hardware,
            name=f"bearing_mount_boss_{index + 1}",
        )
    for index in range(4):
        angle = (math.pi / 4.0) + index * (math.pi / 2.0)
        pedestal_base.visual(
            Cylinder(radius=0.028, length=0.012),
            origin=Origin(
                xyz=(0.145 * math.cos(angle), 0.145 * math.sin(angle), 0.006),
            ),
            material=wear_poly,
            name=f"wear_pad_{index + 1}",
        )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.230, length=0.642),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.321)),
    )

    service_cover = model.part("service_cover")
    service_cover.visual(
        Box((0.014, 0.100, 0.160)),
        origin=Origin(xyz=(0.007, 0.0, 0.080)),
        material=polymer,
        name="cover_panel",
    )
    service_cover.visual(
        Cylinder(radius=0.008, length=0.130),
        origin=Origin(xyz=(0.016, -0.042, 0.080)),
        material=hardware,
        name="cover_hinge_barrel",
    )
    service_cover.visual(
        Box((0.022, 0.030, 0.012)),
        origin=Origin(xyz=(0.019, 0.0, 0.132)),
        material=hardware,
        name="latch_pull",
    )
    for index, z_pos in enumerate((0.035, 0.065, 0.095, 0.125), start=1):
        service_cover.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.014, 0.028 if index % 2 else -0.028, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"cover_fastener_{index}",
        )
    service_cover.inertial = Inertial.from_geometry(
        Box((0.024, 0.100, 0.160)),
        mass=0.35,
        origin=Origin(xyz=(0.012, 0.0, 0.080)),
    )

    seat_stage = model.part("seat_stage")
    seat_stage.visual(
        lower_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=machined_steel,
        name="lower_bearing_plate",
    )
    seat_stage.visual(
        Cylinder(radius=0.038, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=machined_steel,
        name="swivel_spindle",
    )
    seat_stage.visual(
        Cylinder(radius=0.058, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=powder_coat,
        name="bearing_cap",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        seat_stage.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(0.074 * math.cos(angle), 0.074 * math.sin(angle), 0.009)),
            material=hardware,
            name=f"cartridge_fastener_{index + 1}",
        )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        seat_stage.visual(
            Box((0.130, 0.028, 0.068)),
            origin=Origin(
                xyz=(0.065 * math.cos(angle), 0.065 * math.sin(angle), 0.046),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_coat,
            name=f"seat_gusset_{index + 1}",
        )
    seat_stage.visual(
        Cylinder(radius=0.155, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=steel_dark,
        name="support_plate",
    )
    seat_stage.visual(
        Cylinder(radius=0.190, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=powder_coat,
        name="seat_pan",
    )
    seat_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.124),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        seat_pad_mesh,
        material=polymer,
        name="seat_pad_shell",
    )
    seat_pad.inertial = Inertial.from_geometry(
        Cylinder(radius=0.185, length=0.055),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    model.articulation(
        "pedestal_to_service_cover",
        ArticulationType.FIXED,
        parent=pedestal_base,
        child=service_cover,
        origin=place_on_surface(
            service_cover,
            pedestal_base,
            point_hint=(0.085, 0.0, 0.145),
            child_axis="+x",
            clearance=0.0,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
    )
    model.articulation(
        "pedestal_to_seat_stage",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.642)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=4.0),
    )
    model.articulation(
        "seat_stage_to_seat_pad",
        ArticulationType.FIXED,
        parent=seat_stage,
        child=seat_pad,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    service_cover = object_model.get_part("service_cover")
    seat_stage = object_model.get_part("seat_stage")
    seat_pad = object_model.get_part("seat_pad")
    swivel = object_model.get_articulation("pedestal_to_seat_stage")

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

    ctx.expect_contact(
        service_cover,
        pedestal_base,
        elem_a="cover_panel",
        elem_b="column_shroud",
        contact_tol=0.001,
        name="service_cover_is_mounted_to_column",
    )
    ctx.expect_overlap(
        service_cover,
        pedestal_base,
        axes="yz",
        elem_a="cover_panel",
        elem_b="column_shroud",
        min_overlap=0.080,
        name="service_cover_has_real_mounting_footprint",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_origin_distance(
            seat_stage,
            pedestal_base,
            axes="xy",
            max_dist=0.001,
            name="seat_stage_stays_centered_over_pedestal",
        )
        ctx.expect_origin_gap(
            seat_stage,
            pedestal_base,
            axis="z",
            min_gap=0.640,
            max_gap=0.644,
            name="seat_height_matches_bar_stool_scale",
        )
        ctx.expect_gap(
            seat_stage,
            pedestal_base,
            axis="z",
            positive_elem="lower_bearing_plate",
            negative_elem="upper_thrust_plate",
            max_gap=0.003,
            max_penetration=0.0005,
            name="bearing_stack_is_supported_with_service_clearance",
        )
        ctx.expect_overlap(
            seat_stage,
            pedestal_base,
            axes="xy",
            elem_a="lower_bearing_plate",
            elem_b="upper_thrust_plate",
            min_overlap=0.090,
            name="bearing_stack_has_centered_support_area",
        )
        ctx.expect_contact(
            seat_pad,
            seat_stage,
            elem_a="seat_pad_shell",
            elem_b="seat_pan",
            contact_tol=0.001,
            name="seat_pad_is_bolted_to_pan",
        )
        ctx.expect_within(
            seat_pad,
            seat_stage,
            axes="xy",
            inner_elem="seat_pad_shell",
            outer_elem="seat_pan",
            margin=0.0,
            name="seat_pad_stays_within_pan_perimeter",
        )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_origin_distance(
            seat_stage,
            pedestal_base,
            axes="xy",
            max_dist=0.001,
            name="seat_swivel_remains_concentric_when_turned",
        )
        ctx.expect_gap(
            seat_stage,
            pedestal_base,
            axis="z",
            positive_elem="lower_bearing_plate",
            negative_elem="upper_thrust_plate",
            max_gap=0.003,
            max_penetration=0.0005,
            name="bearing_stack_stays_supported_through_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

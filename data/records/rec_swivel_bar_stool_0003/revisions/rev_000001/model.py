from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_bar_stool", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.15, 0.16, 0.17, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.35, 0.32, 0.28, 1.0))
    warm_leather = model.material("warm_leather", rgba=(0.49, 0.43, 0.36, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.11, 0.12, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    pedestal_shell = save_mesh(
        "stool_pedestal_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.060, 0.000),
                (0.145, 0.003),
                (0.205, 0.010),
                (0.223, 0.020),
                (0.215, 0.031),
                (0.187, 0.039),
                (0.124, 0.044),
                (0.058, 0.046),
                (0.0, 0.046),
            ],
            segments=72,
        ),
    )
    seat_shell_mesh = save_mesh(
        "stool_seat_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.070, 0.000),
                (0.130, 0.003),
                (0.160, 0.010),
                (0.175, 0.018),
                (0.172, 0.024),
                (0.146, 0.028),
                (0.090, 0.030),
                (0.0, 0.030),
            ],
            segments=72,
        ),
    )
    seat_support_transition_mesh = save_mesh(
        "stool_support_transition.obj",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.036, 0.000),
                (0.050, 0.004),
                (0.070, 0.012),
                (0.094, 0.020),
                (0.112, 0.028),
                (0.118, 0.034),
                (0.0, 0.034),
            ],
            segments=72,
        ),
    )
    cushion_mesh = save_mesh(
        "stool_cushion.obj",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.102, 0.0015),
                (0.146, 0.006),
                (0.170, 0.013),
                (0.181, 0.021),
                (0.176, 0.029),
                (0.156, 0.034),
                (0.102, 0.038),
                (0.0, 0.036),
            ],
            segments=72,
        ),
    )
    footrest_ring_mesh = save_mesh(
        "stool_footrest_ring.obj",
        TorusGeometry(radius=0.142, tube=0.012, radial_segments=20, tubular_segments=84),
    )
    seam_welt_mesh = save_mesh(
        "stool_cushion_welt.obj",
        TorusGeometry(radius=0.163, tube=0.0028, radial_segments=16, tubular_segments=80),
    )
    pedestal = model.part("pedestal")
    pedestal.visual(pedestal_shell, material=matte_graphite, name="pedestal_shell")
    pedestal.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=matte_graphite,
        name="base_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.040, length=0.490),
        origin=Origin(xyz=(0.0, 0.0, 0.291)),
        material=matte_graphite,
        name="column_lower",
    )
    pedestal.visual(
        Cylinder(radius=0.034, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.613)),
        material=satin_aluminum,
        name="column_upper",
    )
    pedestal.visual(
        footrest_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=dark_bronze,
        name="footrest_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.061, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=dark_bronze,
        name="footrest_hub",
    )
    for index in range(3):
        pedestal.visual(
            Cylinder(radius=0.008, length=0.100),
            origin=Origin(
                xyz=(0.100, 0.0, 0.315),
                rpy=(0.0, math.pi / 2.0, index * math.tau / 3.0),
            ),
            material=dark_bronze,
            name=f"footrest_arm_{index}",
        )
    pedestal.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.684)),
        material=soft_black,
        name="bearing_shroud",
    )
    pedestal.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.699)),
        material=soft_black,
        name="swivel_lower",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.223, length=0.708),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.354)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_aluminum,
        name="swivel_upper",
    )
    seat.visual(
        Cylinder(radius=0.037, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_aluminum,
        name="support_hub",
    )
    seat.visual(
        Cylinder(radius=0.105, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_aluminum,
        name="support_plate",
    )
    seat.visual(
        seat_support_transition_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=matte_graphite,
        name="support_transition",
    )
    seat.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=matte_graphite,
        name="seat_shell",
    )
    seat.visual(
        cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=warm_leather,
        name="seat_cushion",
    )
    seat.visual(
        seam_welt_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_bronze,
        name="cushion_welt",
    )
    for index in range(3):
        angle = index * math.tau / 3.0 + math.pi / 6.0
        seat.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(
                xyz=(0.062 * math.cos(angle), 0.062 * math.sin(angle), 0.027),
            ),
            material=soft_black,
            name=f"fastener_{index}",
        )
    seat.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.110, 0.0, 0.033), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="lift_lever",
    )
    seat.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.136, 0.0, 0.028)),
        material=soft_black,
        name="lift_lever_tip",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.181, length=0.089),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0445)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.708)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    seat_swivel = object_model.get_articulation("seat_swivel")
    swivel_lower = pedestal.get_visual("swivel_lower")
    footrest_ring = pedestal.get_visual("footrest_ring")
    swivel_upper = seat.get_visual("swivel_upper")
    support_plate = seat.get_visual("support_plate")
    seat_cushion = seat.get_visual("seat_cushion")
    lift_lever = seat.get_visual("lift_lever")

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

    ctx.expect_origin_distance(
        seat,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="seat_is_centered_over_pedestal",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        min_overlap=0.108,
        elem_a=swivel_upper,
        elem_b=swivel_lower,
        name="swivel_interface_has_broad_shared_footprint",
    )
    ctx.expect_within(
        seat,
        pedestal,
        axes="xy",
        margin=0.002,
        inner_elem=swivel_upper,
        outer_elem=swivel_lower,
        name="upper_swivel_ring_stays_within_lower_housing",
    )
    ctx.expect_contact(
        seat,
        pedestal,
        elem_a=swivel_upper,
        elem_b=swivel_lower,
        contact_tol=1e-6,
        name="swivel_seam_is_tight_but_readable",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        min_overlap=0.090,
        elem_a=support_plate,
        elem_b=swivel_lower,
        name="seat_support_plate_reads_as_mounted_over_column",
    )

    pedestal_aabb = ctx.part_world_aabb(pedestal)
    seat_aabb = ctx.part_world_aabb(seat)
    footrest_aabb = ctx.part_element_world_aabb(pedestal, elem=footrest_ring)
    cushion_aabb = ctx.part_element_world_aabb(seat, elem=seat_cushion)

    if pedestal_aabb is not None and seat_aabb is not None and footrest_aabb is not None and cushion_aabb is not None:
        base_span_x = pedestal_aabb[1][0] - pedestal_aabb[0][0]
        seat_span_x = cushion_aabb[1][0] - cushion_aabb[0][0]
        overall_height = seat_aabb[1][2] - pedestal_aabb[0][2]
        footrest_height = 0.5 * (footrest_aabb[0][2] + footrest_aabb[1][2])
        ctx.check(
            "bar_stool_overall_height_is_believable",
            0.74 <= overall_height <= 0.84,
            details=f"overall_height={overall_height:.4f}",
        )
        ctx.check(
            "pedestal_base_is_wider_than_seat",
            base_span_x >= seat_span_x + 0.05,
            details=f"base_span_x={base_span_x:.4f}, seat_span_x={seat_span_x:.4f}",
        )
        ctx.check(
            "footrest_sits_at_bar_stool_height",
            0.28 <= footrest_height <= 0.35,
            details=f"footrest_height={footrest_height:.4f}",
        )

    def center_of(aabb):
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    lever_rest_aabb = ctx.part_element_world_aabb(seat, elem=lift_lever)
    if lever_rest_aabb is not None:
        lever_rest = center_of(lever_rest_aabb)
        with ctx.pose({seat_swivel: math.pi / 2.0}):
            ctx.expect_contact(
                seat,
                pedestal,
                elem_a=swivel_upper,
                elem_b=swivel_lower,
                contact_tol=1e-6,
                name="swivel_seam_stays_controlled_when_rotated",
            )
            ctx.expect_overlap(
                seat,
                pedestal,
                axes="xy",
                min_overlap=0.108,
                elem_a=swivel_upper,
                elem_b=swivel_lower,
                name="swivel_footprint_stays_registered_when_rotated",
            )
            lever_rotated_aabb = ctx.part_element_world_aabb(seat, elem=lift_lever)
            if lever_rotated_aabb is not None:
                lever_rotated = center_of(lever_rotated_aabb)
                rest_radius = math.hypot(lever_rest[0], lever_rest[1])
                rotated_radius = math.hypot(lever_rotated[0], lever_rotated[1])
                ctx.check(
                    "seat_swivel_rotates_asymmetric_lever_around_column",
                    abs(lever_rotated[0]) < 0.035
                    and lever_rotated[1] > 0.08
                    and abs(rest_radius - rotated_radius) < 0.01,
                    details=(
                        f"lever_rest={lever_rest}, lever_rotated={lever_rotated}, "
                        f"rest_radius={rest_radius:.4f}, rotated_radius={rotated_radius:.4f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_swivel_bar_stool")

    powder_coat = model.material("powder_coat", rgba=(0.24, 0.26, 0.28, 1.0))
    seat_polymer = model.material("seat_polymer", rgba=(0.31, 0.33, 0.35, 1.0))
    marine_stainless = model.material("marine_stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal_shell = _save_mesh(
        "pedestal_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.223, 0.0),
                (0.223, 0.014),
                (0.186, 0.030),
                (0.135, 0.058),
                (0.088, 0.112),
                (0.060, 0.220),
                (0.052, 0.600),
                (0.058, 0.666),
                (0.074, 0.710),
                (0.080, 0.736),
                (0.074, 0.740),
                (0.0, 0.740),
            ],
            segments=72,
        ),
    )
    seat_shell = _save_mesh(
        "seat_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0, 0.072),
                (0.050, 0.078),
                (0.120, 0.082),
                (0.180, 0.077),
                (0.205, 0.066),
                (0.200, 0.055),
                (0.188, 0.046),
                (0.180, 0.036),
                (0.174, 0.024),
                (0.104, 0.018),
                (0.098, 0.000),
                (0.098, -0.052),
            ],
            [
                (0.0, 0.052),
                (0.050, 0.058),
                (0.118, 0.062),
                (0.172, 0.058),
                (0.186, 0.051),
                (0.178, 0.044),
                (0.166, 0.036),
                (0.160, 0.030),
                (0.154, 0.024),
                (0.094, 0.024),
                (0.090, 0.006),
                (0.090, -0.046),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    foot_ring_mesh = _save_mesh(
        "foot_ring",
        TorusGeometry(radius=0.150, tube=0.014, radial_segments=16, tubular_segments=56),
    )

    pedestal = model.part("pedestal_base")
    pedestal.visual(pedestal_shell, material=powder_coat, name="pedestal_shell")
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=marine_stainless,
        name="foot_ring",
    )
    pedestal.visual(
        Box((0.080, 0.018, 0.012)),
        origin=Origin(xyz=(0.096, 0.0, 0.305)),
        material=marine_stainless,
        name="foot_ring_bracket_pos_x",
    )
    pedestal.visual(
        Box((0.080, 0.018, 0.012)),
        origin=Origin(xyz=(-0.096, 0.0, 0.305)),
        material=marine_stainless,
        name="foot_ring_bracket_neg_x",
    )
    pedestal.visual(
        Box((0.080, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.096, 0.305), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=marine_stainless,
        name="foot_ring_bracket_pos_y",
    )
    pedestal.visual(
        Box((0.080, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.096, 0.305), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=marine_stainless,
        name="foot_ring_bracket_neg_y",
    )
    pedestal.visual(
        Cylinder(radius=0.066, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.703)),
        material=marine_stainless,
        name="lower_bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.078, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
        material=gasket_black,
        name="weather_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.086, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.746)),
        material=marine_stainless,
        name="thrust_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.020, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.718)),
        material=marine_stainless,
        name="bearing_spindle_cap",
    )
    for index in range(4):
        angle = math.tau * index / 4.0 + (math.pi / 4.0)
        pedestal.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(
                xyz=(0.160 * math.cos(angle), 0.160 * math.sin(angle), 0.056)
            ),
            material=marine_stainless,
            name=f"anchor_cap_{index:02d}",
        )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.223, length=0.752),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.376)),
    )

    seat = model.part("seat_stage")
    seat.visual(seat_shell, material=seat_polymer, name="seat_shell")
    seat.visual(
        Cylinder(radius=0.080, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=seat_polymer,
        name="hub_sleeve",
    )
    seat.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=powder_coat,
        name="support_boss",
    )
    seat.visual(
        Cylinder(radius=0.084, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=marine_stainless,
        name="bearing_flange",
    )
    for index in range(6):
        angle = math.tau * index / 6.0
        seat.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(
                xyz=(0.066 * math.cos(angle), 0.066 * math.sin(angle), 0.015)
            ),
            material=marine_stainless,
            name=f"fastener_cap_{index:02d}",
        )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.140),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.752)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_base")
    seat = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("seat_swivel")

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
        "seat swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        f"expected a continuous centered swivel, got {swivel.articulation_type}",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_origin_distance(
            seat,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="seat stays centered on pedestal",
        )
        ctx.expect_origin_gap(
            seat,
            pedestal,
            axis="z",
            min_gap=0.72,
            max_gap=0.78,
            name="swivel plane sits at bar stool height",
        )
        ctx.expect_contact(
            seat,
            pedestal,
            elem_a="bearing_flange",
            elem_b="thrust_plate",
            contact_tol=0.0006,
            name="bearing stack supports the seat",
        )
        ctx.expect_overlap(
            seat,
            pedestal,
            axes="xy",
            elem_a="bearing_flange",
            elem_b="thrust_plate",
            min_overlap=0.165,
            name="bearing races overlap in plan",
        )
        ctx.expect_within(
            pedestal,
            seat,
            axes="xy",
            inner_elem="weather_collar",
            outer_elem="bearing_flange",
            name="weather collar stays inside the seat flange overhang",
        )
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="bearing_flange",
            negative_elem="weather_collar",
            min_gap=0.010,
            max_gap=0.014,
            name="sealed collar sits below the rotating flange",
        )
        ctx.expect_contact(
            seat,
            seat,
            elem_a="seat_shell",
            elem_b="hub_sleeve",
            contact_tol=0.0006,
            name="seat shell is structurally tied into the hub sleeve",
        )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_origin_distance(
            seat,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="seat remains centered while rotated",
        )
        ctx.expect_contact(
            seat,
            pedestal,
            elem_a="bearing_flange",
            elem_b="thrust_plate",
            contact_tol=0.0006,
            name="rotated seat still rides on thrust plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

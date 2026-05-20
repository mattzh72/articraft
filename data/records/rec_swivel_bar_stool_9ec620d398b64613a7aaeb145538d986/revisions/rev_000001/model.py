from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    powder_coat = model.material("powder_coat", rgba=(0.17, 0.18, 0.19, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.11, 0.11, 0.12, 1.0))
    board_brown = model.material("board_brown", rgba=(0.32, 0.24, 0.16, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    pedestal_base = model.part("pedestal_base")
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.44, 0.44, 0.69)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
    )

    base_profile = [
        (0.0, 0.000),
        (0.080, 0.002),
        (0.150, 0.004),
        (0.205, 0.008),
        (0.215, 0.012),
        (0.215, 0.018),
        (0.175, 0.022),
        (0.085, 0.024),
        (0.0, 0.024),
    ]
    pedestal_base.visual(
        save_mesh("stool_base_shell", LatheGeometry(base_profile, segments=72)),
        material=powder_coat,
        name="base_shell",
    )
    pedestal_base.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=powder_coat,
        name="base_collar",
    )
    pedestal_base.visual(
        Cylinder(radius=0.038, length=0.630),
        origin=Origin(xyz=(0.0, 0.0, 0.339)),
        material=powder_coat,
        name="column_tube",
    )
    pedestal_base.visual(
        Cylinder(radius=0.046, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.626)),
        material=dark_steel,
        name="swivel_clamp_collar",
    )
    pedestal_base.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.045, 0.626), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_steel,
        name="clamp_bolt_front",
    )
    pedestal_base.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, -0.045, 0.626), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_steel,
        name="clamp_bolt_rear",
    )

    footrest_hoop = tube_from_spline_points(
        [
            (-0.092, 0.060, 0.0),
            (-0.138, 0.110, 0.0),
            (0.0, 0.168, 0.0),
            (0.138, 0.110, 0.0),
            (0.092, 0.060, 0.0),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    footrest_left_strut = tube_from_spline_points(
        [
            (-0.092, 0.060, 0.0),
            (-0.060, 0.030, 0.028),
            (-0.034, 0.0, 0.060),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    footrest_right_strut = tube_from_spline_points(
        [
            (0.092, 0.060, 0.0),
            (0.060, 0.030, 0.028),
            (0.034, 0.0, 0.060),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    footrest_geom = footrest_hoop.copy()
    footrest_geom.merge(footrest_left_strut)
    footrest_geom.merge(footrest_right_strut)
    footrest_geom.translate(0.0, 0.0, 0.300)
    pedestal_base.visual(
        save_mesh("stool_footrest", footrest_geom),
        material=powder_coat,
        name="footrest_assembly",
    )

    pedestal_base.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        material=powder_coat,
        name="top_cap",
    )
    pedestal_base.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.664)),
        material=bearing_steel,
        name="lower_thrust_race",
    )

    swivel_stage = model.part("swivel_stage")
    swivel_stage.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.05)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    swivel_stage.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=bearing_steel,
        name="upper_thrust_race",
    )
    swivel_stage.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="swivel_hub",
    )
    swivel_stage.visual(
        Box((0.200, 0.200, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="mounting_plate",
    )
    swivel_stage.visual(
        Box((0.160, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="plate_stiffener_x",
    )
    swivel_stage.visual(
        Box((0.035, 0.160, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="plate_stiffener_y",
    )
    for x in (-0.060, 0.060):
        for y in (-0.060, 0.060):
            swivel_stage.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(x, y, 0.024)),
                material=bearing_steel,
            )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.182, length=0.078),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )
    seat.visual(
        Cylinder(radius=0.170, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=board_brown,
        name="seat_mount_board",
    )
    for x in (-0.060, 0.060):
        for y in (-0.060, 0.060):
            seat.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=Origin(xyz=(x, y, 0.003)),
                material=bearing_steel,
            )
    cushion_profile = [
        (0.0, 0.018),
        (0.120, 0.018),
        (0.165, 0.020),
        (0.178, 0.036),
        (0.182, 0.052),
        (0.145, 0.068),
        (0.070, 0.076),
        (0.0, 0.078),
    ]
    seat.visual(
        save_mesh("stool_seat_cushion", LatheGeometry(cushion_profile, segments=72)),
        material=vinyl_black,
        name="seat_cushion",
    )

    model.articulation(
        "pedestal_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=swivel_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )
    model.articulation(
        "swivel_to_seat",
        ArticulationType.FIXED,
        parent=swivel_stage,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    swivel_stage = object_model.get_part("swivel_stage")
    seat = object_model.get_part("seat")
    pedestal_to_swivel = object_model.get_articulation("pedestal_to_swivel")
    swivel_to_seat = object_model.get_articulation("swivel_to_seat")

    lower_thrust_race = pedestal_base.get_visual("lower_thrust_race")
    upper_thrust_race = swivel_stage.get_visual("upper_thrust_race")
    mounting_plate = swivel_stage.get_visual("mounting_plate")
    seat_mount_board = seat.get_visual("seat_mount_board")

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
        "swivel articulation type",
        pedestal_to_swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous, got {pedestal_to_swivel.articulation_type}",
    )
    ctx.check(
        "swivel articulation axis",
        tuple(round(v, 6) for v in pedestal_to_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {pedestal_to_swivel.axis}",
    )
    ctx.check(
        "seat mount articulation type",
        swivel_to_seat.articulation_type == ArticulationType.FIXED,
        details=f"expected fixed, got {swivel_to_seat.articulation_type}",
    )

    ctx.expect_contact(
        swivel_stage,
        pedestal_base,
        elem_a=upper_thrust_race,
        elem_b=lower_thrust_race,
        contact_tol=1e-4,
        name="bearing stack seated on pedestal",
    )
    ctx.expect_contact(
        seat,
        swivel_stage,
        elem_a=seat_mount_board,
        elem_b=mounting_plate,
        contact_tol=1e-4,
        name="seat board seated on mounting plate",
    )
    ctx.expect_origin_distance(
        seat,
        pedestal_base,
        axes="xy",
        max_dist=0.001,
        name="seat centered over pedestal at rest",
    )

    with ctx.pose({pedestal_to_swivel: 1.7}):
        ctx.expect_origin_distance(
            seat,
            pedestal_base,
            axes="xy",
            max_dist=0.001,
            name="seat stays centered while swiveling",
        )
        ctx.expect_contact(
            swivel_stage,
            pedestal_base,
            elem_a=upper_thrust_race,
            elem_b=lower_thrust_race,
            contact_tol=1e-4,
            name="bearing stack stays seated while swiveling",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_articulated_vacuum")

    painted_metal = model.material("painted_metal", rgba=(0.74, 0.77, 0.81, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.09, 0.10, 0.11, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.55, 0.68, 0.78, 0.32))
    wand_aluminum = model.material("wand_aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    soft_rubber = model.material("soft_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")

    body_shell = section_loft(
        [
            _yz_section(-0.01, 0.078, 0.150, 0.022, z_center=0.105),
            _yz_section(-0.10, 0.122, 0.245, 0.036, z_center=0.170),
            _yz_section(-0.21, 0.120, 0.285, 0.040, z_center=0.218),
            _yz_section(-0.31, 0.082, 0.170, 0.028, z_center=0.185),
        ]
    )
    body.visual(_mesh("vacuum_body_shell", body_shell), material=painted_metal, name="body_shell")

    dust_bin_shell = LatheGeometry.from_shell_profiles(
        [
            (0.020, 0.000),
            (0.046, 0.010),
            (0.055, 0.060),
            (0.054, 0.150),
            (0.046, 0.198),
            (0.038, 0.214),
        ],
        [
            (0.000, 0.003),
            (0.041, 0.014),
            (0.049, 0.060),
            (0.048, 0.150),
            (0.041, 0.204),
        ],
        segments=48,
        end_cap="round",
    )
    body.visual(
        _mesh("vacuum_dust_bin", dust_bin_shell),
        origin=Origin(xyz=(-0.050, 0.0, 0.010)),
        material=smoke_clear,
        name="dust_bin",
    )
    body.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(-0.050, 0.0, 0.205)),
        material=graphite_polymer,
        name="dust_bin_lid",
    )
    body.visual(
        Box((0.110, 0.078, 0.110)),
        origin=Origin(xyz=(-0.190, 0.0, 0.052)),
        material=graphite_polymer,
        name="battery_pack",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.118),
        origin=Origin(xyz=(-0.285, 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_polymer,
        name="rear_motor_cap",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, -0.020, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="socket_left_cheek",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.020, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="socket_right_cheek",
    )
    body.visual(
        Box((0.016, 0.056, 0.042)),
        origin=Origin(xyz=(-0.028, 0.0, 0.008)),
        material=graphite_polymer,
        name="socket_housing",
    )
    handle_loop = tube_from_spline_points(
        [
            (-0.225, 0.0, 0.145),
            (-0.305, 0.0, 0.245),
            (-0.265, 0.0, 0.375),
            (-0.160, 0.0, 0.410),
            (-0.075, 0.0, 0.330),
            (-0.055, 0.0, 0.225),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
    )
    body.visual(_mesh("vacuum_handle_loop", handle_loop), material=painted_metal, name="handle_loop")
    grip_sleeve = tube_from_spline_points(
        [
            (-0.250, 0.0, 0.300),
            (-0.195, 0.0, 0.360),
            (-0.115, 0.0, 0.345),
            (-0.082, 0.0, 0.300),
        ],
        radius=0.021,
        samples_per_segment=14,
        radial_segments=18,
    )
    body.visual(_mesh("vacuum_handle_grip", grip_sleeve), material=soft_rubber, name="handle_grip")
    body.visual(
        Box((0.040, 0.024, 0.060)),
        origin=Origin(xyz=(-0.080, 0.0, 0.175)),
        material=satin_black,
        name="cyclone_bridge",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.360, 0.150, 0.430)),
        mass=3.6,
        origin=Origin(xyz=(-0.170, 0.0, 0.190)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="proximal_collar",
    )
    upper_wand_tube = section_loft(
        [
            _yz_section(0.014, 0.036, 0.030, 0.010),
            _yz_section(0.160, 0.034, 0.028, 0.009),
            _yz_section(0.330, 0.032, 0.026, 0.008),
        ]
    )
    upper_wand.visual(_mesh("vacuum_upper_wand_tube", upper_wand_tube), material=wand_aluminum, name="tube")
    upper_wand.visual(
        Box((0.085, 0.042, 0.034)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=graphite_polymer,
        name="proximal_sleeve",
    )
    upper_wand.visual(
        Box((0.024, 0.050, 0.038)),
        origin=Origin(xyz=(0.318, 0.0, 0.0)),
        material=graphite_polymer,
        name="distal_bridge",
    )
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.352, -0.018, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="distal_left_cheek",
    )
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.352, 0.018, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="distal_right_cheek",
    )
    upper_wand.visual(
        Box((0.024, 0.010, 0.014)),
        origin=Origin(xyz=(0.338, -0.018, 0.0)),
        material=graphite_polymer,
        name="distal_left_rib",
    )
    upper_wand.visual(
        Box((0.024, 0.010, 0.014)),
        origin=Origin(xyz=(0.338, 0.018, 0.0)),
        material=graphite_polymer,
        name="distal_right_rib",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.380, 0.060, 0.060)),
        mass=0.85,
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="proximal_collar",
    )
    lower_wand_tube = section_loft(
        [
            _yz_section(0.012, 0.032, 0.026, 0.009),
            _yz_section(0.190, 0.030, 0.024, 0.008),
            _yz_section(0.372, 0.028, 0.022, 0.007),
        ]
    )
    lower_wand.visual(_mesh("vacuum_lower_wand_tube", lower_wand_tube), material=wand_aluminum, name="tube")
    lower_wand.visual(
        Box((0.070, 0.038, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=graphite_polymer,
        name="coupler_sleeve",
    )
    lower_wand.visual(
        Box((0.024, 0.048, 0.036)),
        origin=Origin(xyz=(0.368, 0.0, 0.0)),
        material=graphite_polymer,
        name="distal_bridge",
    )
    lower_wand.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.400, -0.017, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="distal_left_cheek",
    )
    lower_wand.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.400, 0.017, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="distal_right_cheek",
    )
    lower_wand.visual(
        Box((0.026, 0.010, 0.014)),
        origin=Origin(xyz=(0.386, -0.017, 0.0)),
        material=graphite_polymer,
        name="distal_left_rib",
    )
    lower_wand.visual(
        Box((0.026, 0.010, 0.014)),
        origin=Origin(xyz=(0.386, 0.017, 0.0)),
        material=graphite_polymer,
        name="distal_right_rib",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.420, 0.055, 0.055)),
        mass=0.92,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="pivot_collar",
    )
    nozzle_shell = section_loft(
        [
            _yz_section(0.030, 0.084, 0.050, 0.012, z_center=-0.010),
            _yz_section(0.140, 0.250, 0.046, 0.014, z_center=-0.017),
            _yz_section(0.260, 0.310, 0.040, 0.014, z_center=-0.020),
            _yz_section(0.335, 0.230, 0.036, 0.012, z_center=-0.021),
        ]
    )
    floor_nozzle.visual(
        _mesh("vacuum_floor_nozzle_shell", nozzle_shell),
        material=graphite_polymer,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Box((0.030, 0.020, 0.038)),
        origin=Origin(xyz=(0.020, 0.0, -0.010)),
        material=graphite_polymer,
        name="pivot_neck",
    )
    floor_nozzle.visual(
        Box((0.300, 0.118, 0.012)),
        origin=Origin(xyz=(0.175, 0.0, -0.032)),
        material=satin_black,
        name="sole_plate",
    )
    floor_nozzle.visual(
        Box((0.024, 0.258, 0.018)),
        origin=Origin(xyz=(0.318, 0.0, -0.027)),
        material=soft_rubber,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.028, 0.190, 0.016)),
        origin=Origin(xyz=(0.036, 0.0, -0.028)),
        material=soft_rubber,
        name="rear_skirt",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.350, 0.310, 0.060)),
        mass=1.25,
        origin=Origin(xyz=(0.175, 0.0, -0.018)),
    )

    body_to_upper = model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(rpy=(0.0, 1.00, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.85,
        ),
    )

    upper_to_lower = model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.352, 0.0, 0.0), rpy=(0.0, 0.22, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=1.4,
            lower=-0.10,
            upper=1.05,
        ),
    )

    lower_to_nozzle = model.articulation(
        "lower_wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(0.0, -1.22, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_to_upper = object_model.get_articulation("body_to_upper_wand")
    upper_to_lower = object_model.get_articulation("upper_to_lower_wand")
    lower_to_nozzle = object_model.get_articulation("lower_wand_to_nozzle")

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

    ctx.expect_contact(body, upper_wand, name="body_contacts_upper_wand_at_socket")
    ctx.expect_contact(upper_wand, lower_wand, name="upper_wand_contacts_lower_wand_at_hinge")
    ctx.expect_contact(lower_wand, floor_nozzle, name="lower_wand_contacts_floor_nozzle_at_pivot")
    ctx.expect_origin_distance(body, floor_nozzle, axes="xz", min_dist=0.70, name="vacuum_has_real_stick_length")

    rest_nozzle_position = ctx.part_world_position(floor_nozzle)
    with ctx.pose({body_to_upper: 0.45, upper_to_lower: 0.60, lower_to_nozzle: 0.08}):
        folded_nozzle_position = ctx.part_world_position(floor_nozzle)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_folded_pose")
        ctx.check(
            "wand_folds_upward",
            rest_nozzle_position is not None
            and folded_nozzle_position is not None
            and folded_nozzle_position[2] > rest_nozzle_position[2] + 0.15,
            details=(
                f"rest nozzle position={rest_nozzle_position}, "
                f"folded nozzle position={folded_nozzle_position}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

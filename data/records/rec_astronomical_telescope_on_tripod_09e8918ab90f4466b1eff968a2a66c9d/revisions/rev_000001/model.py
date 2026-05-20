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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    radial_segments: int = 18,
) :
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=radial_segments,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=max(radius * 2.0, 0.01),
            corner_segments=8,
        ),
        name,
    )


def _axis_offset(distance: float, polar_angle: float) -> tuple[float, float, float]:
    return (
        -math.sin(polar_angle) * distance,
        0.0,
        math.cos(polar_angle) * distance,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maksutov_fork_equatorial_mount")

    mount_black = model.material("mount_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tube_blue = model.material("tube_blue", rgba=(0.15, 0.19, 0.30, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.70, 0.74, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.34, 0.38, 0.45))

    polar_angle = math.radians(35.0)
    ra_origin = (0.0, 0.0, 0.22)

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=mount_black,
        name="top_platform",
    )
    tripod.visual(
        Cylinder(radius=0.085, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.935)),
        material=mount_black,
        name="shoulder_casting",
    )
    tripod.visual(
        Cylinder(radius=0.042, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=steel,
        name="center_post",
    )
    tripod.visual(
        Cylinder(radius=0.018, length=0.33),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=steel,
        name="spreader_column",
    )
    tripod.visual(
        Cylinder(radius=0.085, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
        material=mount_black,
        name="spreader_hub",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        c = math.cos(angle)
        s = math.sin(angle)
        tripod.visual(
            _tube_mesh(
                f"tripod_leg_{index}",
                [
                    (0.085 * c, 0.085 * s, 0.93),
                    (0.23 * c, 0.23 * s, 0.50),
                    (0.49 * c, 0.49 * s, 0.02),
                ],
                radius=0.016,
            ),
            material=steel,
            name=f"leg_{index}",
        )
        tripod.visual(
            Box((0.055, 0.035, 0.02)),
            origin=Origin(xyz=(0.50 * c, 0.50 * s, 0.01), rpy=(0.0, 0.0, angle)),
            material=mount_black,
            name=f"foot_{index}",
        )
        tripod.visual(
            _tube_mesh(
                f"tripod_spreader_{index}",
                [
                    (0.055 * c, 0.055 * s, 0.57),
                    (0.22 * c, 0.22 * s, 0.48),
                ],
                radius=0.007,
                radial_segments=14,
            ),
            material=mount_black,
            name=f"spreader_arm_{index}",
        )
    tripod.inertial = Inertial.from_geometry(
        Box((1.05, 1.05, 1.00)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.10, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=mount_black,
        name="base_plate",
    )
    mount_head.visual(
        Box((0.19, 0.17, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.08)),
        material=mount_black,
        name="wedge_block",
    )
    mount_head.visual(
        Box((0.10, 0.12, 0.10)),
        origin=Origin(xyz=(-0.055, 0.0, 0.07)),
        material=mount_black,
        name="drive_box",
    )
    main_housing_center = tuple(
        ra_origin[i] - _axis_offset(0.114, polar_angle)[i] for i in range(3)
    )
    bearing_center = tuple(
        ra_origin[i] - _axis_offset(0.012, polar_angle)[i] for i in range(3)
    )
    mount_head.visual(
        Cylinder(radius=0.048, length=0.18),
        origin=Origin(xyz=main_housing_center, rpy=(0.0, -polar_angle, 0.0)),
        material=mount_black,
        name="polar_housing",
    )
    mount_head.visual(
        Cylinder(radius=0.062, length=0.024),
        origin=Origin(xyz=bearing_center, rpy=(0.0, -polar_angle, 0.0)),
        material=mount_black,
        name="ra_bearing",
    )
    mount_head.visual(
        Box((0.05, 0.13, 0.05)),
        origin=Origin(xyz=(-0.095, 0.0, 0.105)),
        material=mount_black,
        name="electronics_pod",
    )
    mount_head.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, 0.25)),
        mass=6.0,
        origin=Origin(xyz=(0.02, 0.0, 0.11)),
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.062, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=mount_black,
        name="ra_shaft",
    )
    fork.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=mount_black,
        name="ra_drum",
    )
    fork.visual(
        Box((0.24, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, -0.025, 0.065)),
        material=mount_black,
        name="fork_base",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        x = 0.126 * sign
        fork.visual(
            Box((0.018, 0.032, 0.33)),
            origin=Origin(xyz=(x, -0.082, 0.225)),
            material=mount_black,
            name=f"{side}_spine",
        )
        fork.visual(
            Box((0.018, 0.17, 0.075)),
            origin=Origin(xyz=(x, -0.002, 0.255)),
            material=mount_black,
            name=f"{side}_bearing_carrier",
        )
        fork.visual(
            Box((0.018, 0.15, 0.028)),
            origin=Origin(xyz=(x, -0.002, 0.385)),
            material=mount_black,
            name=f"{side}_top_arm",
        )
        fork.visual(
            Box((0.018, 0.12, 0.028)),
            origin=Origin(xyz=(x, -0.020, 0.125)),
            material=mount_black,
            name=f"{side}_bottom_arm",
        )
    fork.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, 0.32)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.02, 0.15)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        Cylinder(radius=0.088, length=0.32),
        origin=Origin(xyz=(0.0, 0.06, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=tube_blue,
        name="main_tube",
    )
    optical_tube.visual(
        Cylinder(radius=0.096, length=0.04),
        origin=Origin(xyz=(0.0, 0.24, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_gray,
        name="front_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.091, length=0.004),
        origin=Origin(xyz=(0.0, 0.262, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="corrector_plate",
    )
    optical_tube.visual(
        Cylinder(radius=0.098, length=0.008),
        origin=Origin(xyz=(0.0, 0.268, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_gray,
        name="retaining_ring",
    )
    optical_tube.visual(
        Cylinder(radius=0.099, length=0.05),
        origin=Origin(xyz=(0.0, -0.125, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_black,
        name="rear_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(xyz=(0.0, -0.17, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_black,
        name="visual_back",
    )
    optical_tube.visual(
        Box((0.055, 0.18, 0.012)),
        origin=Origin(xyz=(0.0, 0.015, -0.094)),
        material=mount_black,
        name="dovetail_rail",
    )
    optical_tube.visual(
        Box((0.042, 0.035, 0.024)),
        origin=Origin(xyz=(0.0, -0.11, 0.062)),
        material=mount_black,
        name="focus_knob_block",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        optical_tube.visual(
            Box((0.030, 0.09, 0.030)),
            origin=Origin(xyz=(0.085 * sign, 0.0, 0.0)),
            material=mount_black,
            name=f"{side}_trunnion_plate",
        )
        optical_tube.visual(
            Cylinder(radius=0.016, length=0.022),
            origin=Origin(
                xyz=(0.106 * sign, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_gray,
            name=f"{side}_trunnion",
        )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.24, 0.45, 0.22)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
    )

    model.articulation(
        "tripod_to_mount_head",
        ArticulationType.FIXED,
        parent=tripod,
        child=mount_head,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
    )
    model.articulation(
        "hour_angle_axis",
        ArticulationType.REVOLUTE,
        parent=mount_head,
        child=fork,
        origin=Origin(xyz=ra_origin, rpy=(0.0, -polar_angle, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.7,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "declination_axis",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.7,
            lower=-1.15,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    mount_head = object_model.get_part("mount_head")
    fork = object_model.get_part("fork")
    optical_tube = object_model.get_part("optical_tube")
    hour_angle_axis = object_model.get_articulation("hour_angle_axis")
    declination_axis = object_model.get_articulation("declination_axis")

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
    ctx.allow_overlap(
        fork,
        optical_tube,
        elem_a="left_bearing_carrier",
        elem_b="left_trunnion",
        reason="Left declination trunnion shaft seats inside a closed bearing carrier whose bore is not explicitly modeled.",
    )
    ctx.allow_overlap(
        fork,
        optical_tube,
        elem_a="right_bearing_carrier",
        elem_b="right_trunnion",
        reason="Right declination trunnion shaft seats inside a closed bearing carrier whose bore is not explicitly modeled.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(mount_head, tripod, name="mount_head_seats_on_tripod")
    ctx.expect_contact(fork, mount_head, name="fork_contacts_ra_bearing")
    ctx.expect_contact(optical_tube, fork, name="tube_contacts_dec_bearings")

    ctx.check(
        "hour_angle_axis_is_revolute",
        hour_angle_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(hour_angle_axis.axis) == (0.0, 0.0, 1.0)
        and hour_angle_axis.motion_limits is not None
        and hour_angle_axis.motion_limits.lower is not None
        and hour_angle_axis.motion_limits.upper is not None
        and hour_angle_axis.motion_limits.lower <= -3.0
        and hour_angle_axis.motion_limits.upper >= 3.0,
        "Hour-angle axis should be a wide-travel revolute joint aligned to local +Z.",
    )
    ctx.check(
        "declination_axis_is_revolute",
        declination_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(declination_axis.axis) == (1.0, 0.0, 0.0)
        and declination_axis.motion_limits is not None
        and declination_axis.motion_limits.lower is not None
        and declination_axis.motion_limits.upper is not None
        and declination_axis.motion_limits.lower < 0.0
        and declination_axis.motion_limits.upper > 0.0,
        "Declination axis should be a bidirectional revolute joint aligned to local +X.",
    )

    with ctx.pose({hour_angle_axis: 0.9}):
        ctx.expect_contact(fork, mount_head, name="ra_bearing_contact_in_slew")

    with ctx.pose({declination_axis: 0.75}):
        ctx.expect_contact(optical_tube, fork, name="dec_bearing_contact_in_tilt")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rotate_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sct_equatorial_fork")

    tripod_black = model.material("tripod_black", rgba=(0.15, 0.15, 0.16, 1.0))
    mount_charcoal = model.material("mount_charcoal", rgba=(0.21, 0.22, 0.24, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    tube_orange = model.material("tube_orange", rgba=(0.82, 0.43, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.32, 0.38, 0.35))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    polar_rest = math.radians(38.0)

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.095, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=tripod_black,
        name="head_core",
    )
    tripod.visual(
        Box((0.24, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=mount_charcoal,
        name="head_plate",
    )
    tripod.visual(
        Cylinder(radius=0.044, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=tripod_black,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=mount_charcoal,
        name="accessory_tray",
    )
    tripod.visual(
        Cylinder(radius=0.030, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=tripod_black,
        name="tray_hub",
    )

    leg_angles = (math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.055 * c, 0.055 * s, 0.745),
                (0.215 * c, 0.215 * s, 0.415),
                (0.450 * c, 0.450 * s, 0.030),
            ],
            radius=0.015,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        tripod.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=tripod_black,
            name=f"leg_{index}",
        )

        brace_mesh = tube_from_spline_points(
            [
                (0.050 * c, 0.050 * s, 0.430),
                (0.135 * c, 0.135 * s, 0.405),
                (0.205 * c, 0.205 * s, 0.370),
            ],
            radius=0.007,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        tripod.visual(
            mesh_from_geometry(brace_mesh, f"tripod_brace_{index}"),
            material=satin_metal,
            name=f"brace_{index}",
        )
        tripod.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.450 * c, 0.450 * s, 0.030)),
            material=rubber,
            name=f"foot_{index}",
        )

    tripod.inertial = Inertial.from_geometry(
        Box((0.94, 0.94, 0.82)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
    )

    wedge = model.part("wedge")
    wedge.visual(
        Cylinder(radius=0.030, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_charcoal,
        name="hinge_barrel",
    )

    top_plate_center = _rotate_x((0.0, 0.145, 0.030), polar_rest)
    wedge.visual(
        Box((0.19, 0.24, 0.018)),
        origin=Origin(xyz=top_plate_center, rpy=(polar_rest, 0.0, 0.0)),
        material=mount_charcoal,
        name="top_plate",
    )

    front_plate_center = _rotate_x((0.0, 0.050, 0.026), polar_rest)
    wedge.visual(
        Box((0.14, 0.080, 0.016)),
        origin=Origin(xyz=front_plate_center, rpy=(polar_rest, 0.0, 0.0)),
        material=mount_charcoal,
        name="front_bridge",
    )

    for index, side_x in enumerate((-0.095, 0.095)):
        side_strut = tube_from_spline_points(
            [
                _rotate_x((side_x, 0.000, 0.026), polar_rest),
                _rotate_x((side_x, 0.084, 0.044), polar_rest),
                _rotate_x((side_x, 0.162, 0.018), polar_rest),
            ],
            radius=0.011,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        )
        wedge.visual(
            mesh_from_geometry(side_strut, f"wedge_side_strut_{index}"),
            material=mount_charcoal,
            name=f"side_strut_{index}",
        )

    rear_adjust_block_center = _rotate_x((0.0, 0.235, 0.014), polar_rest)
    wedge.visual(
        Box((0.10, 0.040, 0.028)),
        origin=Origin(xyz=rear_adjust_block_center, rpy=(polar_rest, 0.0, 0.0)),
        material=tripod_black,
        name="rear_adjust_block",
    )
    wedge.visual(
        Cylinder(radius=0.017, length=0.12),
        origin=Origin(
            xyz=_rotate_x((0.0, 0.215, -0.005), polar_rest),
            rpy=(polar_rest, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="latitude_screw",
    )
    wedge.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=_rotate_x((0.070, 0.215, -0.005), polar_rest)),
        material=tripod_black,
        name="latitude_knob",
    )
    wedge.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(
            xyz=_rotate_x((0.044, 0.215, -0.005), polar_rest),
            rpy=(polar_rest, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="latitude_handle_stem",
    )
    wedge.inertial = Inertial.from_geometry(
        Box((0.23, 0.30, 0.24)),
        mass=2.4,
        origin=Origin(xyz=_rotate_x((0.0, 0.115, 0.040), polar_rest)),
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.080, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=tripod_black,
        name="ra_base",
    )
    fork.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=mount_charcoal,
        name="ra_collar",
    )
    fork.visual(
        Box((0.078, 0.082, 0.260)),
        origin=Origin(xyz=(0.110, 0.0, 0.190)),
        material=mount_charcoal,
        name="arm_shell",
    )
    fork.visual(
        Box((0.052, 0.076, 0.130)),
        origin=Origin(xyz=(0.078, 0.0, 0.095)),
        material=mount_charcoal,
        name="arm_root_bridge",
    )
    fork.visual(
        Box((0.090, 0.084, 0.140)),
        origin=Origin(xyz=(0.112, 0.0, 0.370)),
        material=mount_charcoal,
        name="arm_head",
    )
    fork.visual(
        Box((0.072, 0.050, 0.092)),
        origin=Origin(xyz=(0.112, -0.016, 0.455)),
        material=mount_charcoal,
        name="dec_clamp",
    )
    fork.visual(
        Box((0.046, 0.060, 0.040)),
        origin=Origin(xyz=(0.076, 0.0, 0.412)),
        material=tripod_black,
        name="dec_motor_cover",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, 0.56)),
        mass=4.2,
        origin=Origin(xyz=(0.070, 0.0, 0.270)),
    )

    ota = model.part("optical_tube")
    ota.visual(
        Box((0.060, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, -0.056, -0.020)),
        material=satin_metal,
        name="saddle_block",
    )
    ota.visual(
        Box((0.250, 0.092, 0.090)),
        origin=Origin(xyz=(-0.125, -0.104, -0.060)),
        material=tripod_black,
        name="side_rail",
    )
    ota.visual(
        Cylinder(radius=0.092, length=0.340),
        origin=Origin(xyz=(-0.210, -0.150, -0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_orange,
        name="tube_shell",
    )
    ota.visual(
        Cylinder(radius=0.100, length=0.040),
        origin=Origin(xyz=(-0.380, -0.150, -0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_charcoal,
        name="front_cell",
    )
    ota.visual(
        Cylinder(radius=0.088, length=0.004),
        origin=Origin(xyz=(-0.402, -0.150, -0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="corrector_glass",
    )
    ota.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(-0.394, -0.150, -0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tripod_black,
        name="secondary_housing",
    )
    ota.visual(
        Cylinder(radius=0.097, length=0.100),
        origin=Origin(xyz=(-0.080, -0.150, -0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_charcoal,
        name="rear_cell",
    )
    ota.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(-0.010, -0.150, -0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="visual_back",
    )
    ota.visual(
        Box((0.040, 0.044, 0.060)),
        origin=Origin(xyz=(0.020, -0.150, -0.060)),
        material=tripod_black,
        name="diagonal_body",
    )
    ota.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.020, -0.150, -0.010)),
        material=tripod_black,
        name="eyepiece_barrel",
    )
    ota.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.020, -0.150, 0.038)),
        material=rubber,
        name="eyecup",
    )
    ota.inertial = Inertial.from_geometry(
        Box((0.48, 0.21, 0.22)),
        mass=5.1,
        origin=Origin(xyz=(-0.150, -0.120, -0.095)),
    )

    model.articulation(
        "tripod_to_wedge_polar",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=wedge,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=-0.20,
            upper=0.30,
        ),
    )

    model.articulation(
        "wedge_to_fork_ra",
        ArticulationType.CONTINUOUS,
        parent=wedge,
        child=fork,
        origin=Origin(
            xyz=_rotate_x((0.0, 0.145, 0.039), polar_rest),
            rpy=(polar_rest, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
        ),
    )

    model.articulation(
        "fork_to_ota_dec",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=ota,
        origin=Origin(xyz=(0.112, 0.0, 0.455)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.8,
            lower=-0.35,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    wedge = object_model.get_part("wedge")
    fork = object_model.get_part("fork")
    ota = object_model.get_part("optical_tube")
    polar = object_model.get_articulation("tripod_to_wedge_polar")
    ra = object_model.get_articulation("wedge_to_fork_ra")
    dec = object_model.get_articulation("fork_to_ota_dec")

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
        ota,
        elem_a="arm_head",
        elem_b="saddle_block",
        reason=(
            "The declination carrier housing and OTA saddle are simplified solid "
            "proxies for a nested clamp/trunnion stack at the single-arm fork tip."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        wedge,
        tripod,
        elem_a="hinge_barrel",
        elem_b="head_plate",
        name="wedge hinge barrel sits on tripod head plate",
    )
    ctx.expect_contact(
        fork,
        wedge,
        elem_a="ra_base",
        elem_b="top_plate",
        name="RA base seats on wedge top plate",
    )
    ctx.expect_contact(
        ota,
        fork,
        elem_a="saddle_block",
        elem_b="dec_clamp",
        name="OTA saddle seats against declination clamp",
    )

    ctx.check(
        "equatorial mount uses polar, RA, and declination articulations",
        polar.articulation_type == ArticulationType.REVOLUTE
        and tuple(polar.axis) == (1.0, 0.0, 0.0)
        and ra.articulation_type == ArticulationType.CONTINUOUS
        and tuple(ra.axis) == (0.0, 0.0, 1.0)
        and ra.motion_limits is not None
        and ra.motion_limits.lower is None
        and ra.motion_limits.upper is None
        and dec.articulation_type == ArticulationType.REVOLUTE
        and tuple(dec.axis) == (0.0, 1.0, 0.0),
        details=(
            f"polar={polar.articulation_type, polar.axis}, "
            f"ra={ra.articulation_type, ra.axis, ra.motion_limits}, "
            f"dec={dec.articulation_type, dec.axis}"
        ),
    )

    rest_fork_pos = ctx.part_world_position(fork)
    with ctx.pose({polar: 0.18}):
        raised_fork_pos = ctx.part_world_position(fork)
    ctx.check(
        "positive polar adjustment raises the RA axis",
        rest_fork_pos is not None
        and raised_fork_pos is not None
        and raised_fork_pos[2] > rest_fork_pos[2] + 0.012,
        details=f"rest={rest_fork_pos}, raised={raised_fork_pos}",
    )

    rest_ota_pos = ctx.part_world_position(ota)
    with ctx.pose({ra: 1.0}):
        ra_swept_ota_pos = ctx.part_world_position(ota)
    ra_shift = None
    if rest_ota_pos is not None and ra_swept_ota_pos is not None:
        ra_shift = math.hypot(
            ra_swept_ota_pos[0] - rest_ota_pos[0],
            ra_swept_ota_pos[1] - rest_ota_pos[1],
        )
    ctx.check(
        "RA articulation sweeps the fork around the polar axis",
        ra_shift is not None and ra_shift > 0.08,
        details=f"rest={rest_ota_pos}, swept={ra_swept_ota_pos}, horizontal_shift={ra_shift}",
    )

    rest_front_aabb = ctx.part_element_world_aabb(ota, elem="front_cell")
    with ctx.pose({dec: 0.55}):
        raised_front_aabb = ctx.part_element_world_aabb(ota, elem="front_cell")
    ctx.check(
        "positive declination raises the telescope front cell",
        rest_front_aabb is not None
        and raised_front_aabb is not None
        and raised_front_aabb[1][2] > rest_front_aabb[1][2] + 0.050,
        details=f"rest={rest_front_aabb}, raised={raised_front_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
    wrap_profile_onto_surface,
)


DOME_RADIUS = 2.0
BASE_RADIUS = 2.25
BASE_HEIGHT = 1.50
TRACK_DECK_Z = 1.62
SUPPORT_BALL_RADIUS = 0.035
GUIDE_RAIL_RADIUS = 0.032
SHUTTER_BEARING_RADIUS = 0.028
GUIDE_RAIL_PROUD = 0.060
SLIT_RAIL_X = 1.94
SLIT_RAIL_HALF_SPAN = 0.82
UPPER_SHUTTER_CLOSED_Z = 2.62
LOWER_SHUTTER_CLOSED_Z = 1.66


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _sphere_front_point(y: float, z: float, *, proud: float = 0.0) -> tuple[float, float, float]:
    x = math.sqrt(max(0.0, (DOME_RADIUS * DOME_RADIUS) - (y * y) - (z * z)))
    scale = (DOME_RADIUS + proud) / DOME_RADIUS
    return (x * scale, y * scale, z * scale)


def _sphere_rear_point(y: float, z: float, *, proud: float = 0.0) -> tuple[float, float, float]:
    x, yy, zz = _sphere_front_point(y, z, proud=proud)
    return (-x, yy, zz)


def _radial_offset(point: tuple[float, float, float], offset: float) -> tuple[float, float, float]:
    x, y, z = point
    mag = math.sqrt((x * x) + (y * y) + (z * z))
    scale = (mag + offset) / mag
    return (x * scale, y * scale, z * scale)


def _spherical_rail_path(
    y: float,
    z_start: float,
    z_end: float,
    *,
    proud: float,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    return [
        _sphere_front_point(
            y,
            z_start + (z_end - z_start) * (index / (samples - 1)),
            proud=proud,
        )
        for index in range(samples)
    ]


def _spherical_cross_path(
    z: float,
    y_start: float,
    y_end: float,
    *,
    proud: float,
    samples: int = 11,
) -> list[tuple[float, float, float]]:
    return [
        _sphere_front_point(
            y_start + (y_end - y_start) * (index / (samples - 1)),
            z,
            proud=proud,
        )
        for index in range(samples)
    ]


def _wrapped_panel_mesh(
    name: str,
    profile,
    *,
    direction: tuple[float, float, float],
    thickness: float,
    visible_relief: float = 0.0,
    hole_profiles=(),
):
    return _mesh(
        name,
        wrap_profile_onto_surface(
            profile,
            Sphere(radius=DOME_RADIUS),
            thickness=thickness,
            hole_profiles=hole_profiles,
            direction=direction,
            mapping="intrinsic",
            visible_relief=visible_relief,
            surface_max_edge=0.10,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_observatory_dome")

    concrete = model.material("concrete", rgba=(0.68, 0.69, 0.71, 1.0))
    structural_gray = model.material("structural_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    machine_dark = model.material("machine_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dome_white = model.material("dome_white", rgba=(0.92, 0.94, 0.95, 1.0))
    shutter_white = model.material("shutter_white", rgba=(0.87, 0.89, 0.91, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.09, 0.10, 1.0))
    datum_orange = model.material("datum_orange", rgba=(0.93, 0.45, 0.12, 1.0))

    base_pier = model.part("base_pier")
    base_pier.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=concrete,
        name="pier_drum",
    )
    base_pier.visual(
        Cylinder(radius=2.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.04)),
        material=structural_gray,
        name="pier_cap",
    )
    base_pier.visual(
        Box((0.90, 1.15, 1.00)),
        origin=Origin(xyz=(-1.72, 0.0, 0.50)),
        material=structural_gray,
        name="service_annex",
    )
    base_pier.visual(
        Box((0.18, 0.62, 0.96)),
        origin=Origin(xyz=(2.16, 0.0, 0.48)),
        material=machine_dark,
        name="access_door_frame",
    )
    base_pier.inertial = Inertial.from_geometry(
        Box((4.8, 4.8, 1.65)),
        mass=9000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
    )

    ring_track = model.part("ring_track")
    ring_track.visual(
        Cylinder(radius=2.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, TRACK_DECK_Z)),
        material=machine_dark,
        name="track_deck",
    )
    ring_track.visual(
        _mesh(
            "azimuth_inner_encoder_ring",
            TorusGeometry(radius=1.88, tube=0.018, radial_segments=16, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, TRACK_DECK_Z + 0.030)),
        material=structural_gray,
        name="encoder_ring",
    )
    ring_track.visual(
        Box((0.18, 0.32, 0.03)),
        origin=Origin(xyz=(2.15, 0.0, TRACK_DECK_Z + 0.055)),
        material=datum_orange,
        name="azimuth_datum_bar",
    )
    ring_track.visual(
        Box((0.02, 0.26, 0.01)),
        origin=Origin(xyz=(2.24, 0.0, TRACK_DECK_Z + 0.065)),
        material=seal_black,
        name="azimuth_index_mark",
    )
    for index, theta in enumerate([index * math.tau / 12.0 for index in range(12)]):
        c = math.cos(theta)
        s = math.sin(theta)
        yaw = theta + (math.pi / 2.0)
        ring_track.visual(
            Box((0.14, 0.08, 0.02)),
            origin=Origin(xyz=(1.91 * c, 1.91 * s, 1.595), rpy=(0.0, 0.0, yaw)),
            material=structural_gray,
            name=f"bogie_bracket_{index:02d}",
        )
        ring_track.visual(
            Sphere(radius=SUPPORT_BALL_RADIUS),
            origin=Origin(xyz=(2.03 * c, 2.03 * s, 1.66)),
            material=bearing_steel,
            name=f"support_ball_{index:02d}",
        )
    for index, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(theta)
        s = math.sin(theta)
        ring_track.visual(
            Box((0.14, 0.14, 0.04)),
            origin=Origin(xyz=(1.55 * c, 1.55 * s, 1.61)),
            material=datum_orange,
            name=f"leveling_pad_{index:02d}",
        )
        ring_track.visual(
            Cylinder(radius=0.020, length=0.04),
            origin=Origin(xyz=(1.55 * c, 1.55 * s, 1.64)),
            material=bearing_steel,
            name=f"leveling_jack_{index:02d}",
        )
    ring_track.inertial = Inertial.from_geometry(
        Cylinder(radius=2.12, length=0.24),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 1.66)),
    )

    model.articulation(
        "pier_to_ring_track",
        ArticulationType.FIXED,
        parent=base_pier,
        child=ring_track,
        origin=Origin(),
    )

    dome = model.part("dome")
    dome.visual(
        _mesh(
            "dome_bearing_ring",
            TorusGeometry(radius=2.03, tube=SUPPORT_BALL_RADIUS, radial_segments=18, tubular_segments=80),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.029468)),
        material=machine_dark,
        name="bearing_ring",
    )
    dome.visual(
        Cylinder(radius=1.98, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dome_white,
        name="rotation_skirt",
    )
    dome.visual(
        _mesh(
            "dome_lower_trim",
            TorusGeometry(radius=1.90, tube=0.022, radial_segments=16, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=machine_dark,
        name="lower_trim",
    )
    dome.visual(
        Sphere(radius=1.72),
        origin=Origin(xyz=(-0.38, 0.0, 1.94)),
        material=dome_white,
        name="rear_shell",
    )
    dome.visual(
        Box((1.35, 2.80, 0.56)),
        origin=Origin(xyz=(-0.72, 0.0, 0.52)),
        material=dome_white,
        name="rear_lower_shell",
    )
    dome.visual(
        Box((1.28, 0.36, 2.16)),
        origin=Origin(xyz=(0.92, 0.93, 1.62)),
        material=dome_white,
        name="left_cheek_shell",
    )
    dome.visual(
        Box((1.28, 0.36, 2.16)),
        origin=Origin(xyz=(0.92, -0.93, 1.62)),
        material=dome_white,
        name="right_cheek_shell",
    )
    dome.visual(
        Box((1.40, 1.90, 0.24)),
        origin=Origin(xyz=(0.86, 0.0, 3.00)),
        material=dome_white,
        name="crown_shell",
    )
    dome.visual(
        Box((0.40, 1.24, 0.20)),
        origin=Origin(xyz=(1.36, 0.0, 3.02)),
        material=machine_dark,
        name="top_collar",
    )
    dome.visual(
        Box((0.38, 0.24, 1.86)),
        origin=Origin(xyz=(1.37, 0.69, 2.08)),
        material=machine_dark,
        name="left_collar",
    )
    dome.visual(
        Box((0.38, 0.24, 1.86)),
        origin=Origin(xyz=(1.37, -0.69, 2.08)),
        material=machine_dark,
        name="right_collar",
    )
    dome.visual(
        Box((0.20, 0.025, 0.055)),
        origin=Origin(xyz=(2.00, 0.0, 0.18)),
        material=datum_orange,
        name="azimuth_pointer",
    )
    for index, theta in enumerate((math.pi / 6.0, math.pi / 2.0, 5.0 * math.pi / 6.0, 7.0 * math.pi / 6.0, 3.0 * math.pi / 2.0, 11.0 * math.pi / 6.0)):
        c = math.cos(theta)
        s = math.sin(theta)
        dome.visual(
            Box((0.05, 0.16, 0.08)),
            origin=Origin(xyz=(1.995 * c, 1.995 * s, 0.035), rpy=(0.0, 0.0, theta)),
            material=machine_dark,
            name=f"bearing_bridge_{index}",
        )
    dome.inertial = Inertial.from_geometry(
        Box((4.2, 4.2, 2.5)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    azimuth = model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=ring_track,
        child=dome,
        origin=Origin(xyz=(0.0, 0.0, 1.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.25),
    )

    slit_frame = model.part("slit_frame")
    slit_frame.visual(
        Box((0.10, 0.12, 1.86)),
        origin=Origin(xyz=(1.61, 0.69, 2.08)),
        material=machine_dark,
        name="left_jamb",
    )
    slit_frame.visual(
        Box((0.10, 0.12, 1.86)),
        origin=Origin(xyz=(1.61, -0.69, 2.08)),
        material=machine_dark,
        name="right_jamb",
    )
    slit_frame.visual(
        Box((0.12, 1.26, 0.12)),
        origin=Origin(xyz=(1.63, 0.0, 1.09)),
        material=machine_dark,
        name="bottom_sill",
    )
    slit_frame.visual(
        Box((0.06, 0.06, 4.10)),
        origin=Origin(xyz=(1.69, SLIT_RAIL_HALF_SPAN, 2.78)),
        material=structural_gray,
        name="left_guide_rail",
    )
    slit_frame.visual(
        Box((0.06, 0.06, 4.10)),
        origin=Origin(xyz=(1.69, -SLIT_RAIL_HALF_SPAN, 2.78)),
        material=structural_gray,
        name="right_guide_rail",
    )
    slit_frame.visual(
        Box((0.06, 1.70, 0.08)),
        origin=Origin(xyz=(1.69, 0.0, 4.79)),
        material=structural_gray,
        name="upper_rail_bridge",
    )
    slit_frame.visual(
        Box((0.06, 1.70, 0.08)),
        origin=Origin(xyz=(1.69, 0.0, 0.77)),
        material=structural_gray,
        name="lower_rail_bridge",
    )
    for side, tag in ((1.0, "left"), (-1.0, "right")):
        for idx, z_value in enumerate((1.35, 2.25, 2.95)):
            slit_frame.visual(
                Box((0.08, 0.06, 0.10)),
                origin=Origin(xyz=(1.62, side * 0.74, z_value)),
                material=structural_gray,
                name=f"{tag}_rail_standoff_{idx}",
            )
        for row, z_value in enumerate((2.10, 2.64)):
            slit_frame.visual(
                Box((0.04, 0.10, 0.18)),
                origin=Origin(xyz=(1.66, side * 0.76, z_value)),
                material=datum_orange,
                name=f"{tag}_datum_pad_{row}",
            )
            slit_frame.visual(
                Box((0.012, 0.10, 0.03)),
                origin=Origin(xyz=(1.686, side * 0.76, z_value + 0.06)),
                material=seal_black,
                name=f"{tag}_index_mark_{row}",
            )
        for row, z_value in enumerate((1.46, 2.86)):
            slit_frame.visual(
                Cylinder(radius=0.014, length=0.06),
                origin=Origin(xyz=(1.66, side * 0.73, z_value), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bearing_steel,
                name=f"{tag}_adjuster_{row}",
            )
    slit_frame.inertial = Inertial.from_geometry(
        Box((0.30, 1.90, 4.20)),
        mass=220.0,
        origin=Origin(xyz=(1.65, 0.0, 2.45)),
    )

    model.articulation(
        "dome_to_slit_frame",
        ArticulationType.FIXED,
        parent=dome,
        child=slit_frame,
        origin=Origin(),
    )

    upper_shutter = model.part("upper_shutter")
    upper_shutter.visual(
        Box((0.05, 0.86, 1.04)),
        origin=Origin(),
        material=shutter_white,
        name="upper_skin",
    )
    upper_shutter.visual(
        Box((0.05, 0.08, 1.04)),
        origin=Origin(xyz=(0.0, 0.43, 0.0)),
        material=machine_dark,
        name="upper_left_carriage",
    )
    upper_shutter.visual(
        Box((0.05, 0.08, 1.04)),
        origin=Origin(xyz=(0.0, -0.43, 0.0)),
        material=machine_dark,
        name="upper_right_carriage",
    )
    for name, side, z_value in (
        ("upper_left_lower_bearing", 1.0, -0.35),
        ("upper_left_upper_bearing", 1.0, 0.35),
        ("upper_right_lower_bearing", -1.0, -0.35),
        ("upper_right_upper_bearing", -1.0, 0.35),
    ):
        upper_shutter.visual(
            Sphere(radius=SHUTTER_BEARING_RADIUS),
            origin=Origin(xyz=(-0.005, side * SLIT_RAIL_HALF_SPAN, z_value)),
            material=bearing_steel,
            name=name,
        )
        upper_shutter.visual(
            Box((0.04, 0.40, 0.04)),
            origin=Origin(xyz=(0.0, side * 0.625, z_value)),
            material=machine_dark,
            name=f"{name}_arm",
        )
    upper_shutter.inertial = Inertial.from_geometry(
        Box((0.10, 0.96, 1.10)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    upper_shutter_joint = model.articulation(
        "slit_frame_to_upper_shutter",
        ArticulationType.PRISMATIC,
        parent=slit_frame,
        child=upper_shutter,
        origin=Origin(xyz=(1.75, 0.0, UPPER_SHUTTER_CLOSED_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.25,
            lower=0.0,
            upper=1.00,
        ),
    )

    lower_shutter = model.part("lower_shutter")
    lower_shutter.visual(
        Box((0.05, 0.86, 0.84)),
        origin=Origin(),
        material=shutter_white,
        name="lower_skin",
    )
    lower_shutter.visual(
        Box((0.05, 0.08, 0.84)),
        origin=Origin(xyz=(0.0, 0.43, 0.0)),
        material=machine_dark,
        name="lower_left_carriage",
    )
    lower_shutter.visual(
        Box((0.05, 0.08, 0.84)),
        origin=Origin(xyz=(0.0, -0.43, 0.0)),
        material=machine_dark,
        name="lower_right_carriage",
    )
    for name, side, z_value in (
        ("lower_left_lower_bearing", 1.0, -0.17),
        ("lower_left_upper_bearing", 1.0, 0.17),
        ("lower_right_lower_bearing", -1.0, -0.17),
        ("lower_right_upper_bearing", -1.0, 0.17),
    ):
        lower_shutter.visual(
            Sphere(radius=SHUTTER_BEARING_RADIUS),
            origin=Origin(xyz=(-0.005, side * SLIT_RAIL_HALF_SPAN, z_value)),
            material=bearing_steel,
            name=name,
        )
        lower_shutter.visual(
            Box((0.04, 0.40, 0.04)),
            origin=Origin(xyz=(0.0, side * 0.625, z_value)),
            material=machine_dark,
            name=f"{name}_arm",
        )
    lower_shutter.inertial = Inertial.from_geometry(
        Box((0.10, 0.96, 0.90)),
        mass=70.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    lower_shutter_joint = model.articulation(
        "slit_frame_to_lower_shutter",
        ArticulationType.PRISMATIC,
        parent=slit_frame,
        child=lower_shutter,
        origin=Origin(xyz=(1.75, 0.0, LOWER_SHUTTER_CLOSED_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.20,
            lower=0.0,
            upper=0.55,
        ),
    )

    model.meta["prompt_articulations"] = {
        "azimuth_rotation": azimuth.name,
        "upper_shutter": upper_shutter_joint.name,
        "lower_shutter": lower_shutter_joint.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_pier = object_model.get_part("base_pier")
    ring_track = object_model.get_part("ring_track")
    dome = object_model.get_part("dome")
    slit_frame = object_model.get_part("slit_frame")
    upper_shutter = object_model.get_part("upper_shutter")
    lower_shutter = object_model.get_part("lower_shutter")
    azimuth = object_model.get_articulation("azimuth_rotation")
    upper_joint = object_model.get_articulation("slit_frame_to_upper_shutter")
    lower_joint = object_model.get_articulation("slit_frame_to_lower_shutter")

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
        ring_track,
        base_pier,
        contact_tol=0.003,
        name="ring_track_seated_on_pier",
    )
    ctx.expect_contact(
        dome,
        ring_track,
        elem_a="bearing_ring",
        elem_b="support_ball_00",
        contact_tol=0.004,
        name="dome_bearing_ring_supported",
    )
    ctx.expect_contact(
        slit_frame,
        dome,
        contact_tol=0.004,
        name="slit_frame_mounted_to_shell",
    )
    ctx.expect_contact(
        upper_shutter,
        slit_frame,
        elem_a="upper_left_lower_bearing",
        elem_b="left_guide_rail",
        contact_tol=0.004,
        name="upper_left_bearing_on_rail",
    )
    ctx.expect_contact(
        upper_shutter,
        slit_frame,
        elem_a="upper_right_lower_bearing",
        elem_b="right_guide_rail",
        contact_tol=0.004,
        name="upper_right_bearing_on_rail",
    )
    ctx.expect_contact(
        lower_shutter,
        slit_frame,
        elem_a="lower_left_upper_bearing",
        elem_b="left_guide_rail",
        contact_tol=0.004,
        name="lower_left_bearing_on_rail",
    )
    ctx.expect_contact(
        lower_shutter,
        slit_frame,
        elem_a="lower_right_upper_bearing",
        elem_b="right_guide_rail",
        contact_tol=0.004,
        name="lower_right_bearing_on_rail",
    )

    with ctx.pose({azimuth: 1.2}):
        ctx.expect_contact(
            dome,
            ring_track,
            elem_a="bearing_ring",
            elem_b="support_ball_00",
            contact_tol=0.004,
            name="dome_support_persists_when_rotated",
        )

    with ctx.pose({upper_joint: 0.0, lower_joint: 0.0}):
        ctx.expect_gap(
            upper_shutter,
            lower_shutter,
            axis="z",
            positive_elem="upper_skin",
            negative_elem="lower_skin",
            max_gap=0.04,
            max_penetration=0.0,
            name="closed_slit_has_controlled_mid_seam",
        )
        ctx.expect_overlap(
            dome,
            ring_track,
            axes="xy",
            elem_a="azimuth_pointer",
            elem_b="azimuth_datum_bar",
            min_overlap=0.01,
            name="azimuth_pointer_aligned_to_datum",
        )

    with ctx.pose({upper_joint: 1.0, lower_joint: 0.45}):
        ctx.expect_gap(
            upper_shutter,
            lower_shutter,
            axis="z",
            positive_elem="upper_skin",
            negative_elem="lower_skin",
            min_gap=1.0,
            name="opened_slit_clear_aperture",
        )
        ctx.expect_contact(
            upper_shutter,
            slit_frame,
            elem_a="upper_left_upper_bearing",
            elem_b="left_guide_rail",
            contact_tol=0.006,
            name="upper_shutter_remains_supported_open",
        )
        ctx.expect_contact(
            lower_shutter,
            slit_frame,
            elem_a="lower_right_lower_bearing",
            elem_b="right_guide_rail",
            contact_tol=0.006,
            name="lower_shutter_remains_supported_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _rounded_rect_section(
    width: float,
    length: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            length,
            radius,
            corner_segments=8,
        )
    ]


def _build_pad_mesh():
    return section_loft(
        [
            _rounded_rect_section(0.205, 0.145, 0.030, 0.006),
            _rounded_rect_section(0.198, 0.138, 0.033, 0.035),
            _rounded_rect_section(0.182, 0.122, 0.036, 0.058),
        ]
    )


def _build_tire_mesh(radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.56, -half_width * 0.96),
        (radius * 0.82, -half_width),
        (radius * 0.94, -half_width * 0.72),
        (radius, 0.0),
        (radius * 0.94, half_width * 0.72),
        (radius * 0.82, half_width),
        (radius * 0.56, half_width * 0.96),
        (radius * 0.45, half_width * 0.28),
        (radius * 0.43, 0.0),
        (radius * 0.45, -half_width * 0.28),
        (radius * 0.56, -half_width * 0.96),
    ]
    return LatheGeometry(profile, segments=56, closed=True).rotate_y(math.pi / 2.0)


def _build_caster_socket_mesh(
    *,
    outer_radius: float,
    cavity_radius: float,
    stem_clear_radius: float,
    height: float,
    lip_height: float,
):
    half_height = height * 0.5
    lip_top = -half_height + lip_height
    outer_profile = [
        (outer_radius, -half_height),
        (outer_radius, lip_top),
        (outer_radius, half_height),
    ]
    inner_profile = [
        (stem_clear_radius, -half_height),
        (stem_clear_radius, lip_top),
        (cavity_radius, lip_top),
        (cavity_radius, half_height),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _add_wheel_visuals(
    part,
    *,
    tire_mesh,
    tire_radius: float,
    hub_length: float,
    rim_radius: float,
    hub_radius: float,
    rubber,
    rim_material,
) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=rim_radius, length=hub_length * 0.22),
        origin=Origin(
            xyz=(-hub_length * 0.22, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rim_material,
        name="left_rim",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=hub_length * 0.22),
        origin=Origin(
            xyz=(hub_length * 0.22, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rim_material,
        name="right_rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=spin_origin,
        material=rim_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.12, length=hub_length * 0.34),
        origin=spin_origin,
        material=rim_material,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_knee_scooter")

    frame_paint = model.material("frame_paint", rgba=(0.60, 0.63, 0.67, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    pad_black = model.material("pad_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.79, 0.81, 0.84, 1.0))
    fork_dark = model.material("fork_dark", rgba=(0.26, 0.27, 0.29, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))

    rear_tire_mesh = _mesh("rear_wheel_tire", _build_tire_mesh(0.100, 0.030))
    front_tire_mesh = _mesh("front_caster_tire", _build_tire_mesh(0.050, 0.024))
    pad_mesh = _mesh("knee_pad_cushion", _build_pad_mesh())
    caster_socket_mesh = _mesh(
        "caster_socket",
        _build_caster_socket_mesh(
            outer_radius=0.023,
            cavity_radius=0.015,
            stem_clear_radius=0.0105,
            height=0.028,
            lip_height=0.0035,
        ),
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.40, 0.66, 0.86)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.04, 0.43)),
    )

    frame.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.0, -0.220, 0.126), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="rear_cross_tube",
    )
    frame.visual(
        _mesh(
            "left_lower_rail",
            tube_from_spline_points(
                [
                    (-0.075, -0.220, 0.126),
                    (-0.075, -0.020, 0.122),
                    (-0.082, 0.130, 0.128),
                    (-0.064, 0.238, 0.134),
                ],
                radius=0.012,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_lower_rail",
    )
    frame.visual(
        _mesh(
            "right_lower_rail",
            tube_from_spline_points(
                _mirror_x(
                    [
                        (-0.075, -0.220, 0.126),
                        (-0.075, -0.020, 0.122),
                        (-0.082, 0.130, 0.128),
                        (-0.064, 0.238, 0.134),
                    ]
                ),
                radius=0.012,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_lower_rail",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.196),
        origin=Origin(xyz=(0.0, 0.250, 0.146), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="front_cross_tube",
    )
    frame.visual(
        Box((0.012, 0.030, 0.026)),
        origin=Origin(xyz=(-0.085, -0.220, 0.101)),
        material=charcoal,
        name="left_rear_bracket",
    )
    frame.visual(
        Box((0.012, 0.030, 0.026)),
        origin=Origin(xyz=(0.085, -0.220, 0.101)),
        material=charcoal,
        name="right_rear_bracket",
    )
    frame.visual(
        caster_socket_mesh,
        origin=Origin(xyz=(-0.090, 0.250, 0.120)),
        material=charcoal,
        name="left_caster_socket",
    )
    frame.visual(
        caster_socket_mesh,
        origin=Origin(xyz=(0.090, 0.250, 0.120)),
        material=charcoal,
        name="right_caster_socket",
    )
    for brace_name, points in [
        (
            "left_rear_pad_brace",
            [(-0.075, -0.020, 0.122), (-0.062, -0.020, 0.250), (-0.055, -0.020, 0.382)],
        ),
        (
            "left_front_pad_brace",
            [(-0.082, 0.130, 0.128), (-0.063, 0.090, 0.255), (-0.055, 0.080, 0.382)],
        ),
        (
            "right_rear_pad_brace",
            _mirror_x([(-0.075, -0.020, 0.122), (-0.062, -0.020, 0.250), (-0.055, -0.020, 0.382)]),
        ),
        (
            "right_front_pad_brace",
            _mirror_x([(-0.082, 0.130, 0.128), (-0.063, 0.090, 0.255), (-0.055, 0.080, 0.382)]),
        ),
    ]:
        frame.visual(
            _mesh(
                brace_name,
                tube_from_spline_points(
                    points,
                    radius=0.010,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=brace_name,
        )
    frame.visual(
        Box((0.152, 0.158, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, 0.387)),
        material=charcoal,
        name="knee_pad_support",
    )
    frame.visual(
        _mesh(
            "handle_post_tube",
            tube_from_spline_points(
                [
                    (0.0, 0.250, 0.146),
                    (0.0, 0.220, 0.360),
                    (0.0, 0.185, 0.620),
                    (0.0, 0.160, 0.816),
                ],
                radius=0.016,
                samples_per_segment=14,
                radial_segments=20,
            ),
        ),
        material=frame_paint,
        name="handle_post",
    )
    frame.visual(
        Box((0.120, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, 0.160, 0.804)),
        material=charcoal,
        name="handlebar_clamp",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.036),
        origin=Origin(xyz=(-0.048, 0.160, 0.820)),
        material=charcoal,
        name="left_handlebar_riser",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.036),
        origin=Origin(xyz=(0.048, 0.160, 0.820)),
        material=charcoal,
        name="right_handlebar_riser",
    )
    frame.visual(
        _mesh(
            "handlebar_tube",
            tube_from_spline_points(
                [
                    (-0.135, 0.155, 0.826),
                    (-0.080, 0.160, 0.838),
                    (0.000, 0.160, 0.842),
                    (0.080, 0.160, 0.838),
                    (0.135, 0.155, 0.826),
                ],
                radius=0.012,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="handlebar",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(-0.165, 0.152, 0.822), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.165, 0.152, 0.822), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    knee_pad = model.part("knee_pad")
    knee_pad.visual(
        Box((0.166, 0.126, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=charcoal,
        name="base_plate",
    )
    knee_pad.visual(
        pad_mesh,
        material=pad_black,
        name="cushion",
    )
    knee_pad.inertial = Inertial.from_geometry(
        Box((0.205, 0.145, 0.058)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        tire_mesh=rear_tire_mesh,
        tire_radius=0.100,
        hub_length=0.036,
        rim_radius=0.060,
        hub_radius=0.030,
        rubber=rubber,
        rim_material=rim_silver,
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.036),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        tire_mesh=rear_tire_mesh,
        tire_radius=0.100,
        hub_length=0.036,
        rim_radius=0.060,
        hub_radius=0.030,
        rubber=rubber,
        rim_material=rim_silver,
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.036),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    front_left_caster = model.part("front_left_caster")
    front_left_caster.visual(
        Cylinder(radius=0.009, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=fork_dark,
        name="stem",
    )
    front_left_caster.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=fork_dark,
        name="retainer_collar",
    )
    front_left_caster.visual(
        Box((0.040, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, -0.020)),
        material=fork_dark,
        name="fork_crown",
    )
    front_left_caster.visual(
        Box((0.008, 0.012, 0.066)),
        origin=Origin(xyz=(-0.024, -0.024, -0.057)),
        material=fork_dark,
        name="left_fork_arm",
    )
    front_left_caster.visual(
        Box((0.008, 0.012, 0.066)),
        origin=Origin(xyz=(0.024, -0.024, -0.057)),
        material=fork_dark,
        name="right_fork_arm",
    )
    front_left_caster.visual(
        Box((0.056, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.024, -0.024)),
        material=fork_dark,
        name="fork_bridge",
    )
    front_left_caster.inertial = Inertial.from_geometry(
        Box((0.056, 0.034, 0.104)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.016, -0.035)),
    )

    front_right_caster = model.part("front_right_caster")
    front_right_caster.visual(
        Cylinder(radius=0.009, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=fork_dark,
        name="stem",
    )
    front_right_caster.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=fork_dark,
        name="retainer_collar",
    )
    front_right_caster.visual(
        Box((0.040, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, -0.020)),
        material=fork_dark,
        name="fork_crown",
    )
    front_right_caster.visual(
        Box((0.008, 0.012, 0.066)),
        origin=Origin(xyz=(-0.024, -0.024, -0.057)),
        material=fork_dark,
        name="left_fork_arm",
    )
    front_right_caster.visual(
        Box((0.008, 0.012, 0.066)),
        origin=Origin(xyz=(0.024, -0.024, -0.057)),
        material=fork_dark,
        name="right_fork_arm",
    )
    front_right_caster.visual(
        Box((0.056, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.024, -0.024)),
        material=fork_dark,
        name="fork_bridge",
    )
    front_right_caster.inertial = Inertial.from_geometry(
        Box((0.056, 0.034, 0.104)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.016, -0.035)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        tire_mesh=front_tire_mesh,
        tire_radius=0.050,
        hub_length=0.040,
        rim_radius=0.030,
        hub_radius=0.017,
        rubber=rubber,
        rim_material=rim_silver,
    )
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.040),
        mass=0.42,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        tire_mesh=front_tire_mesh,
        tire_radius=0.050,
        hub_length=0.040,
        rim_radius=0.030,
        hub_radius=0.017,
        rubber=rubber,
        rim_material=rim_silver,
    )
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.040),
        mass=0.42,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_knee_pad",
        ArticulationType.FIXED,
        parent=frame,
        child=knee_pad,
        origin=Origin(xyz=(0.0, 0.010, 0.392)),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.109, -0.220, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(0.109, -0.220, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_front_left_caster",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_left_caster,
        origin=Origin(xyz=(-0.090, 0.250, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_front_right_caster",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_right_caster,
        origin=Origin(xyz=(0.090, 0.250, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "front_left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, -0.024, -0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, -0.024, -0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    frame = object_model.get_part("frame")
    knee_pad = object_model.get_part("knee_pad")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    front_left_swivel = object_model.get_articulation("frame_to_front_left_caster")
    front_right_swivel = object_model.get_articulation("frame_to_front_right_caster")
    front_left_wheel_spin = object_model.get_articulation("front_left_caster_to_wheel")
    front_right_wheel_spin = object_model.get_articulation("front_right_caster_to_wheel")

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
        "wheel spin axes align with scooter axle direction",
        rear_left_spin.axis == (1.0, 0.0, 0.0)
        and rear_right_spin.axis == (1.0, 0.0, 0.0)
        and front_left_wheel_spin.axis == (1.0, 0.0, 0.0)
        and front_right_wheel_spin.axis == (1.0, 0.0, 0.0),
        "All wheel joints should spin around the left-right axle axis.",
    )
    ctx.check(
        "caster stems swivel vertically",
        front_left_swivel.axis == (0.0, 0.0, 1.0)
        and front_right_swivel.axis == (0.0, 0.0, 1.0)
        and front_left_swivel.joint_type == ArticulationType.CONTINUOUS
        and front_right_swivel.joint_type == ArticulationType.CONTINUOUS,
        "Front casters should be continuous swivel joints on vertical stems.",
    )

    ctx.expect_contact(knee_pad, frame, elem_a="base_plate", elem_b="knee_pad_support")
    ctx.expect_overlap(knee_pad, frame, axes="xy", min_overlap=0.12)

    ctx.expect_contact(rear_left_wheel, frame, elem_a="hub", elem_b="left_rear_bracket")
    ctx.expect_contact(rear_right_wheel, frame, elem_a="hub", elem_b="right_rear_bracket")
    ctx.expect_contact(
        front_left_caster,
        frame,
        elem_a="retainer_collar",
        elem_b="left_caster_socket",
        contact_tol=0.001,
    )
    ctx.expect_contact(
        front_right_caster,
        frame,
        elem_a="retainer_collar",
        elem_b="right_caster_socket",
        contact_tol=0.001,
    )
    ctx.expect_contact(front_left_wheel, front_left_caster, elem_a="hub")
    ctx.expect_contact(front_right_wheel, front_right_caster, elem_a="hub")

    ctx.expect_origin_gap(front_left_caster, rear_left_wheel, axis="y", min_gap=0.42)
    ctx.expect_origin_gap(knee_pad, rear_left_wheel, axis="z", min_gap=0.28)

    with ctx.pose(
        {
            front_left_swivel: 1.10,
            front_right_swivel: -1.10,
            rear_left_spin: 1.40,
            rear_right_spin: -0.85,
            front_left_wheel_spin: 2.10,
            front_right_wheel_spin: -1.75,
        }
    ):
        ctx.expect_contact(
            front_left_caster,
            frame,
            elem_a="retainer_collar",
            elem_b="left_caster_socket",
            contact_tol=0.001,
            name="left caster stays clipped into frame mount while swiveling",
        )
        ctx.expect_contact(
            front_right_caster,
            frame,
            elem_a="retainer_collar",
            elem_b="right_caster_socket",
            contact_tol=0.001,
            name="right caster stays clipped into frame mount while swiveling",
        )
        ctx.expect_contact(front_left_wheel, front_left_caster, elem_a="hub")
        ctx.expect_contact(front_right_wheel, front_right_caster, elem_a="hub")
        ctx.expect_contact(rear_left_wheel, frame, elem_a="hub", elem_b="left_rear_bracket")
        ctx.expect_contact(rear_right_wheel, frame, elem_a="hub", elem_b="right_rear_bracket")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

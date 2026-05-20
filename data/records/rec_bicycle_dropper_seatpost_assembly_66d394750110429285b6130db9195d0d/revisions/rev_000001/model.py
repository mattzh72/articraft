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
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _section_loop(
    y_pos: float,
    *,
    width: float,
    thickness: float,
    z_center: float,
    exponent: float,
    segments: int = 40,
) -> tuple[tuple[float, float, float], ...]:
    profile = superellipse_profile(width, thickness, exponent=exponent, segments=segments)
    return tuple((x, y_pos, z_center + z) for x, z in profile)


def _saddle_body_mesh(
    name: str,
    *,
    section_specs: tuple[tuple[float, float, float, float], ...],
    exponent: float,
) :
    sections = [
        _section_loop(
            y_pos,
            width=width,
            thickness=thickness,
            z_center=z_center,
            exponent=exponent,
        )
        for y_pos, width, thickness, z_center in section_specs
    ]
    return _mesh(name, section_loft(sections))


def _rail_mesh(name: str, x_pos: float):
    return _mesh(
        name,
        tube_from_spline_points(
            [
                (x_pos, -0.108, 0.043),
                (x_pos, -0.060, 0.033),
                (x_pos, 0.000, 0.033),
                (x_pos, 0.056, 0.034),
                (x_pos, 0.108, 0.042),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.17, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.29, 0.31, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    seat_shell = model.material("seat_shell", rgba=(0.09, 0.09, 0.10, 1.0))
    seat_pad = model.material("seat_pad", rgba=(0.18, 0.18, 0.19, 1.0))

    outer_radius = 0.01745
    sleeve_inner_radius = 0.0146
    outer_height = 0.272
    stanchion_radius = 0.0125
    stanchion_length = 0.400
    travel = 0.170
    pivot_z = 0.176

    outer_body = model.part("outer_body")
    outer_shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, 0.000),
            (outer_radius, 0.218),
            (0.0188, 0.238),
            (0.0208, 0.252),
            (0.0208, 0.272),
        ],
        [
            (sleeve_inner_radius, 0.000),
            (sleeve_inner_radius, 0.238),
            (0.0140, 0.252),
            (0.0140, 0.272),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    outer_body.visual(
        _mesh("dropper_outer_sleeve", outer_shell),
        material=anodized_black,
        name="outer_sleeve",
    )
    outer_body.visual(
        Box((0.018, 0.010, 0.018)),
        origin=Origin(xyz=(0.024, 0.011, 0.262)),
        material=anodized_black,
        name="binder_ear_front",
    )
    outer_body.visual(
        Box((0.018, 0.010, 0.018)),
        origin=Origin(xyz=(0.024, -0.011, 0.262)),
        material=anodized_black,
        name="binder_ear_rear",
    )
    outer_body.visual(
        Cylinder(radius=0.0034, length=0.032),
        origin=Origin(
            xyz=(0.033, 0.000, 0.262),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="binder_bolt",
    )
    outer_body.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(0.033, 0.019, 0.262)),
        material=steel,
        name="binder_bolt_head",
    )
    outer_body.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(0.033, -0.019, 0.262)),
        material=steel,
        name="binder_nut",
    )
    outer_body.visual(
        Box((0.010, 0.015, 0.010)),
        origin=Origin(xyz=(0.000, -0.0195, 0.082)),
        material=gunmetal,
        name="hydraulic_port_base",
    )
    outer_body.visual(
        Cylinder(radius=0.0038, length=0.022),
        origin=Origin(
            xyz=(0.000, -0.0305, 0.082),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hydraulic_port_fitting",
    )
    outer_body.visual(
        Cylinder(radius=0.0024, length=0.014),
        origin=Origin(
            xyz=(0.000, -0.0485, 0.082),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hydraulic_port_stub",
    )
    outer_body.visual(
        Box((0.010, 0.005, 0.022)),
        origin=Origin(xyz=(0.000, 0.0150, 0.252)),
        material=satin_black,
        name="guide_pad_front",
    )
    outer_body.visual(
        Box((0.010, 0.005, 0.022)),
        origin=Origin(xyz=(0.000, -0.0150, 0.252)),
        material=satin_black,
        name="guide_pad_rear",
    )
    outer_body.visual(
        Box((0.005, 0.010, 0.022)),
        origin=Origin(xyz=(0.0150, 0.000, 0.252)),
        material=satin_black,
        name="guide_pad_right",
    )
    outer_body.visual(
        Box((0.005, 0.010, 0.022)),
        origin=Origin(xyz=(-0.0150, 0.000, 0.252)),
        material=satin_black,
        name="guide_pad_left",
    )
    outer_body.inertial = Inertial.from_geometry(
        Box((0.070, 0.090, outer_height)),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=stanchion_radius, length=stanchion_length),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=polished_alloy,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0160, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=gunmetal,
        name="crown_collar",
    )
    inner_post.visual(
        Cylinder(radius=0.0100, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.159)),
        material=gunmetal,
        name="head_neck",
    )
    inner_post.visual(
        Box((0.060, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=gunmetal,
        name="head_bridge",
    )
    inner_post.visual(
        Box((0.018, 0.055, 0.009)),
        origin=Origin(xyz=(-0.033, 0.0, 0.2000)),
        material=gunmetal,
        name="lower_clamp_left",
    )
    inner_post.visual(
        Box((0.018, 0.055, 0.009)),
        origin=Origin(xyz=(0.033, 0.0, 0.2000)),
        material=gunmetal,
        name="lower_clamp_right",
    )
    for name, start, end in (
        (
            "left_support_strut",
            (-0.022, 0.0, 0.170),
            (-0.030, 0.0, 0.196),
        ),
        (
            "right_support_strut",
            (0.022, 0.0, 0.170),
            (0.030, 0.0, 0.196),
        ),
    ):
        inner_post.visual(
            Cylinder(radius=0.0040, length=_distance(start, end)),
            origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_cylinder(start, end)),
            material=gunmetal,
            name=name,
        )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.250)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    model.articulation(
        "outer_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, outer_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.25,
            lower=0.0,
            upper=travel,
        ),
    )

    saddle = model.part("saddle")
    saddle.visual(
        Cylinder(radius=0.0042, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    saddle.visual(
        Box((0.008, 0.016, 0.052)),
        origin=Origin(xyz=(-0.010, 0.0, 0.026)),
        material=gunmetal,
        name="left_yoke_plate",
    )
    saddle.visual(
        Box((0.008, 0.016, 0.052)),
        origin=Origin(xyz=(0.010, 0.0, 0.026)),
        material=gunmetal,
        name="right_yoke_plate",
    )
    saddle.visual(
        Box((0.012, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.007, 0.017)),
        material=gunmetal,
        name="front_tilt_rib",
    )
    saddle.visual(
        Box((0.012, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, -0.007, 0.017)),
        material=gunmetal,
        name="rear_tilt_rib",
    )
    saddle.visual(
        Box((0.072, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=gunmetal,
        name="upper_yoke_bridge",
    )
    saddle.visual(
        Box((0.020, 0.055, 0.008)),
        origin=Origin(xyz=(-0.033, 0.0, 0.040)),
        material=gunmetal,
        name="upper_clamp_left",
    )
    saddle.visual(
        Box((0.020, 0.055, 0.008)),
        origin=Origin(xyz=(0.033, 0.0, 0.040)),
        material=gunmetal,
        name="upper_clamp_right",
    )
    saddle.visual(
        Cylinder(radius=0.0036, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_bolt",
    )
    saddle.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(0.046, 0.0, 0.043)),
        material=steel,
        name="clamp_bolt_head",
    )
    saddle.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(-0.046, 0.0, 0.043)),
        material=steel,
        name="clamp_bolt_nut",
    )
    saddle.visual(
        _rail_mesh("dropper_left_rail", -0.033),
        material=steel,
        name="left_rail",
    )
    saddle.visual(
        _rail_mesh("dropper_right_rail", 0.033),
        material=steel,
        name="right_rail",
    )
    saddle.visual(
        Box((0.062, 0.095, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=seat_shell,
        name="center_support",
    )
    saddle.visual(
        Box((0.112, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, -0.078, 0.052)),
        material=seat_shell,
        name="rear_rail_mount",
    )
    saddle.visual(
        Box((0.086, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, 0.072, 0.050)),
        material=seat_shell,
        name="front_rail_mount",
    )
    saddle.visual(
        _saddle_body_mesh(
            "dropper_saddle_shell",
            section_specs=(
                (-0.125, 0.136, 0.016, 0.059),
                (-0.082, 0.123, 0.017, 0.058),
                (-0.020, 0.094, 0.015, 0.054),
                (0.052, 0.058, 0.013, 0.049),
                (0.120, 0.030, 0.010, 0.045),
            ),
            exponent=2.3,
        ),
        material=seat_shell,
        name="saddle_shell",
    )
    saddle.visual(
        _saddle_body_mesh(
            "dropper_saddle_pad",
            section_specs=(
                (-0.128, 0.145, 0.038, 0.080),
                (-0.086, 0.132, 0.040, 0.079),
                (-0.022, 0.098, 0.034, 0.072),
                (0.054, 0.062, 0.028, 0.065),
                (0.122, 0.034, 0.020, 0.058),
            ),
            exponent=2.8,
        ),
        material=seat_pad,
        name="saddle_pad",
    )
    saddle.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.116, 0.058)),
        material=seat_pad,
        name="nose_probe",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.160, 0.280, 0.110)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "inner_post_to_saddle",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=-0.30,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    inner_post = object_model.get_part("inner_post")
    saddle = object_model.get_part("saddle")
    extension = object_model.get_articulation("outer_to_inner_post")
    tilt = object_model.get_articulation("inner_post_to_saddle")

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

    ctx.expect_within(
        inner_post,
        outer_body,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_sleeve",
        margin=0.0015,
        name="stanchion stays centered in the outer sleeve",
    )
    ctx.expect_overlap(
        inner_post,
        outer_body,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_sleeve",
        min_overlap=0.220,
        name="collapsed post remains deeply inserted in the outer sleeve",
    )
    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="left_rail",
        elem_b="lower_clamp_left",
        contact_tol=0.0015,
        name="left saddle rail sits in the lower clamp cradle",
    )
    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="right_rail",
        elem_b="lower_clamp_right",
        contact_tol=0.0015,
        name="right saddle rail sits in the lower clamp cradle",
    )

    rest_inner_pos = ctx.part_world_position(inner_post)
    upper_extension = extension.motion_limits.upper if extension.motion_limits else None
    if upper_extension is not None:
        with ctx.pose({extension: upper_extension}):
            ctx.expect_within(
                inner_post,
                outer_body,
                axes="xy",
                inner_elem="stanchion",
                outer_elem="outer_sleeve",
                margin=0.0015,
                name="extended stanchion stays centered in the outer sleeve",
            )
            ctx.expect_overlap(
                inner_post,
                outer_body,
                axes="z",
                elem_a="stanchion",
                elem_b="outer_sleeve",
                min_overlap=0.080,
                name="extended post retains insertion in the outer sleeve",
            )
            extended_inner_pos = ctx.part_world_position(inner_post)
        ctx.check(
            "dropper post extends upward",
            rest_inner_pos is not None
            and extended_inner_pos is not None
            and extended_inner_pos[2] > rest_inner_pos[2] + 0.16,
            details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
        )

    def _z_center(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    nose_rest = ctx.part_element_world_aabb(saddle, elem="nose_probe")
    with ctx.pose({tilt: 0.20}):
        nose_up = ctx.part_element_world_aabb(saddle, elem="nose_probe")
    with ctx.pose({tilt: -0.20}):
        nose_down = ctx.part_element_world_aabb(saddle, elem="nose_probe")

    nose_rest_z = _z_center(nose_rest)
    nose_up_z = _z_center(nose_up)
    nose_down_z = _z_center(nose_down)
    ctx.check(
        "positive tilt raises the saddle nose",
        nose_rest_z is not None and nose_up_z is not None and nose_up_z > nose_rest_z + 0.015,
        details=f"rest={nose_rest_z}, up={nose_up_z}",
    )
    ctx.check(
        "negative tilt lowers the saddle nose",
        nose_rest_z is not None and nose_down_z is not None and nose_down_z < nose_rest_z - 0.015,
        details=f"rest={nose_rest_z}, down={nose_down_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

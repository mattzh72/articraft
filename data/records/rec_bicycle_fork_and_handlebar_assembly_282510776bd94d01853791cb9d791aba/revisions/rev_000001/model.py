from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shell_ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 64,
):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return _mesh(name, geom)


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _lerp_point(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _ellipse_section(
    center: tuple[float, float, float],
    direction: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    samples: int = 16,
) -> list[tuple[float, float, float]]:
    ref = (0.0, 1.0, 0.0)
    ref_proj = (
        ref[0] - _dot(ref, direction) * direction[0],
        ref[1] - _dot(ref, direction) * direction[1],
        ref[2] - _dot(ref, direction) * direction[2],
    )
    if math.sqrt(_dot(ref_proj, ref_proj)) < 1e-6:
        ref = (1.0, 0.0, 0.0)
        ref_proj = (
            ref[0] - _dot(ref, direction) * direction[0],
            ref[1] - _dot(ref, direction) * direction[1],
            ref[2] - _dot(ref, direction) * direction[2],
        )
    u_axis = _normalize(ref_proj)
    v_axis = _normalize(_cross(direction, u_axis))
    half_width = width * 0.5
    half_depth = depth * 0.5
    section: list[tuple[float, float, float]] = []
    for index in range(samples):
        angle = math.tau * index / samples
        c = math.cos(angle)
        s = math.sin(angle)
        section.append(
            (
                center[0] + u_axis[0] * half_width * c + v_axis[0] * half_depth * s,
                center[1] + u_axis[1] * half_width * c + v_axis[1] * half_depth * s,
                center[2] + u_axis[2] * half_width * c + v_axis[2] * half_depth * s,
            )
        )
    return section


def _blade_mesh(
    name: str,
    *,
    top: tuple[float, float, float],
    bottom: tuple[float, float, float],
    top_width: float,
    top_depth: float,
    bottom_width: float,
    bottom_depth: float,
):
    direction = _normalize((bottom[0] - top[0], bottom[1] - top[1], bottom[2] - top[2]))
    mid = _lerp_point(top, bottom, 0.56)
    geom = section_loft(
        [
            _ellipse_section(top, direction, width=top_width, depth=top_depth),
            _ellipse_section(
                mid,
                direction,
                width=(top_width + bottom_width) * 0.5,
                depth=(top_depth + bottom_depth) * 0.5,
            ),
            _ellipse_section(bottom, direction, width=bottom_width, depth=bottom_depth),
        ]
    )
    return _mesh(name, geom)


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_fork_with_quill_stem")

    headset_black = model.material("headset_black", rgba=(0.12, 0.12, 0.13, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    bar_tape = model.material("bar_tape", rgba=(0.10, 0.10, 0.10, 1.0))

    head_tube = model.part("head_tube_fixture")
    head_tube.visual(
        _shell_ring_mesh(
            "head_tube_shell",
            outer_radius=0.0175,
            inner_radius=0.0155,
            z0=-0.075,
            z1=0.075,
        ),
        material=headset_black,
        name="head_tube_shell",
    )
    head_tube.visual(
        _shell_ring_mesh(
            "top_headset_cup",
            outer_radius=0.0210,
            inner_radius=0.0158,
            z0=0.075,
            z1=0.087,
        ),
        material=satin_silver,
        name="top_headset_cup",
    )
    head_tube.visual(
        _shell_ring_mesh(
            "bottom_headset_cup",
            outer_radius=0.0210,
            inner_radius=0.0158,
            z0=-0.087,
            z1=-0.075,
        ),
        material=satin_silver,
        name="bottom_headset_cup",
    )
    head_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.174),
        mass=0.85,
        origin=Origin(),
    )

    steering = model.part("steering_assembly")
    steering.inertial = Inertial.from_geometry(
        Box((0.31, 0.42, 0.68)),
        mass=2.2,
        origin=Origin(xyz=(0.08, 0.0, -0.11)),
    )

    steering.visual(
        Cylinder(radius=0.0127, length=0.244),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=fork_steel,
        name="steerer_tube",
    )
    steering.visual(
        _shell_ring_mesh(
            "crown_race_ring",
            outer_radius=0.0184,
            inner_radius=0.0132,
            z0=-0.099,
            z1=-0.087,
        ),
        material=polished_steel,
        name="crown_race_ring",
    )
    steering.visual(
        _shell_ring_mesh(
            "locknut_ring",
            outer_radius=0.0184,
            inner_radius=0.0132,
            z0=0.087,
            z1=0.099,
        ),
        material=polished_steel,
        name="locknut_ring",
    )
    for ring_index in range(7):
        steering.visual(
            Cylinder(radius=0.01355, length=0.0023),
            origin=Origin(xyz=(0.0, 0.0, 0.057 + 0.0073 * ring_index)),
            material=polished_steel,
        )

    crown_geom = (
        CapsuleGeometry(radius=0.0105, length=0.072)
        .rotate_x(math.pi / 2.0)
        .scale(1.28, 1.0, 0.78)
        .translate(0.002, 0.0, -0.110)
    )
    steering.visual(
        _mesh("fork_crown", crown_geom),
        material=satin_silver,
        name="fork_crown",
    )
    steering.visual(
        Cylinder(radius=0.0152, length=0.018),
        origin=Origin(xyz=(0.001, 0.0, -0.101)),
        material=satin_silver,
        name="crown_socket",
    )

    left_blade_top = (0.008, 0.036, -0.116)
    left_blade_bottom = (0.042, 0.050, -0.470)
    right_blade_top = (0.008, -0.036, -0.116)
    right_blade_bottom = (0.042, -0.050, -0.470)
    steering.visual(
        _blade_mesh(
            "left_fork_blade",
            top=left_blade_top,
            bottom=left_blade_bottom,
            top_width=0.016,
            top_depth=0.010,
            bottom_width=0.011,
            bottom_depth=0.007,
        ),
        material=fork_steel,
        name="left_fork_blade",
    )
    steering.visual(
        _blade_mesh(
            "right_fork_blade",
            top=right_blade_top,
            bottom=right_blade_bottom,
            top_width=0.016,
            top_depth=0.010,
            bottom_width=0.011,
            bottom_depth=0.007,
        ),
        material=fork_steel,
        name="right_fork_blade",
    )
    steering.visual(
        Box((0.007, 0.020, 0.028)),
        origin=Origin(xyz=(0.044, 0.052, -0.484)),
        material=satin_silver,
        name="left_dropout",
    )
    steering.visual(
        Box((0.007, 0.020, 0.028)),
        origin=Origin(xyz=(0.044, -0.052, -0.484)),
        material=satin_silver,
        name="right_dropout",
    )

    stem_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.082),
            (0.0, 0.0, 0.110),
            (0.024, 0.0, 0.126),
            (0.058, 0.0, 0.140),
            (0.096, 0.0, 0.149),
        ],
        radius=0.0105,
        samples_per_segment=14,
        radial_segments=18,
    )
    steering.visual(
        _mesh("quill_stem_body", stem_geom),
        material=satin_silver,
        name="quill_stem_body",
    )
    steering.visual(
        Cylinder(radius=0.0140, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=satin_silver,
        name="stem_quill_top",
    )
    steering.visual(
        Cylinder(radius=0.0175, length=0.052),
        origin=Origin(xyz=(0.104, 0.0, 0.149), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="clamp_collar",
    )
    steering.visual(
        Box((0.020, 0.020, 0.025)),
        origin=Origin(xyz=(0.094, 0.0, 0.136)),
        material=satin_silver,
        name="clamp_ears",
    )
    steering.visual(
        Cylinder(radius=0.0034, length=0.024),
        origin=Origin(xyz=(0.092, 0.0, 0.134), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="pinch_bolt",
    )
    steering.visual(
        Cylinder(radius=0.0027, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=polished_steel,
        name="expander_bolt_shaft",
    )
    steering.visual(
        Cylinder(radius=0.0068, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=polished_steel,
        name="expander_bolt_head",
    )

    bar_points = [
        (0.240, 0.195, 0.018),
        (0.204, 0.187, 0.050),
        (0.165, 0.172, 0.085),
        (0.132, 0.140, 0.120),
        (0.112, 0.090, 0.146),
        (0.104, 0.040, 0.149),
        (0.104, 0.0, 0.149),
        (0.104, -0.040, 0.149),
        (0.112, -0.090, 0.146),
        (0.132, -0.140, 0.120),
        (0.165, -0.172, 0.085),
        (0.204, -0.187, 0.050),
        (0.240, -0.195, 0.018),
    ]
    handlebar_geom = tube_from_spline_points(
        bar_points,
        radius=0.0112,
        samples_per_segment=18,
        radial_segments=18,
    )
    steering.visual(
        _mesh("pursuit_handlebar", handlebar_geom),
        material=bar_tape,
        name="pursuit_handlebar",
    )
    steering.visual(
        Cylinder(radius=0.0133, length=0.090),
        origin=Origin(xyz=(0.104, 0.0, 0.149), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="center_clamp_section",
    )
    steering.visual(
        Sphere(radius=0.0116),
        origin=Origin(xyz=bar_points[0]),
        material=bar_tape,
        name="left_bar_end",
    )
    steering.visual(
        Sphere(radius=0.0116),
        origin=Origin(xyz=bar_points[-1]),
        material=bar_tape,
        name="right_bar_end",
    )

    model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube_fixture")
    steering = object_model.get_part("steering_assembly")
    steer_joint = object_model.get_articulation("steering_axis")
    head_tube.get_visual("head_tube_shell")
    head_tube.get_visual("top_headset_cup")
    steering.get_visual("steerer_tube")
    steering.get_visual("fork_crown")
    steering.get_visual("pursuit_handlebar")
    steering.get_visual("expander_bolt_head")
    steering.get_visual("clamp_collar")

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
        "steering_axis_is_vertical",
        tuple(round(value, 6) for value in steer_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Unexpected steering axis: {steer_joint.axis}",
    )
    limits = steer_joint.motion_limits
    ctx.check(
        "steering_limits_track_like",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -0.60
        and limits.upper >= 0.60,
        details=f"Unexpected steering limits: {limits}",
    )

    ctx.expect_contact(
        steering,
        head_tube,
        elem_a="crown_race_ring",
        elem_b="bottom_headset_cup",
        name="lower_headset_bearing_contact",
    )
    ctx.expect_contact(
        steering,
        head_tube,
        elem_a="locknut_ring",
        elem_b="top_headset_cup",
        name="upper_headset_locknut_contact",
    )
    ctx.expect_within(
        steering,
        head_tube,
        axes="xy",
        inner_elem="steerer_tube",
        outer_elem="head_tube_shell",
        margin=0.0005,
        name="steerer_runs_inside_head_tube",
    )
    ctx.expect_gap(
        head_tube,
        steering,
        axis="z",
        positive_elem="bottom_headset_cup",
        negative_elem="fork_crown",
        min_gap=0.010,
        max_gap=0.025,
        name="crown_sits_below_head_tube",
    )

    rest_left = ctx.part_element_world_aabb(steering, elem="left_bar_end")
    assert rest_left is not None
    rest_left_center = _aabb_center(rest_left)
    with ctx.pose({steer_joint: math.radians(22.0)}):
        turned_left = ctx.part_element_world_aabb(steering, elem="left_bar_end")
        assert turned_left is not None
        turned_left_center = _aabb_center(turned_left)
        ctx.expect_contact(
            steering,
            head_tube,
            elem_a="crown_race_ring",
            elem_b="bottom_headset_cup",
            name="lower_headset_contact_at_left_lock",
        )
        ctx.expect_within(
            steering,
            head_tube,
            axes="xy",
            inner_elem="steerer_tube",
            outer_elem="head_tube_shell",
            margin=0.0005,
            name="steerer_within_head_tube_at_left_lock",
        )
        ctx.check(
            "positive_steer_swings_left_bar_outward",
            turned_left_center[0] < rest_left_center[0] - 0.03
            and turned_left_center[1] > rest_left_center[1] + 0.02,
            details=(
                f"Left bar end rest={rest_left_center}, "
                f"turned={turned_left_center}"
            ),
        )

    with ctx.pose({steer_joint: math.radians(-22.0)}):
        turned_right = ctx.part_element_world_aabb(steering, elem="left_bar_end")
        assert turned_right is not None
        turned_right_center = _aabb_center(turned_right)
        ctx.expect_contact(
            steering,
            head_tube,
            elem_a="locknut_ring",
            elem_b="top_headset_cup",
            name="upper_headset_contact_at_right_lock",
        )
        ctx.expect_within(
            steering,
            head_tube,
            axes="xy",
            inner_elem="steerer_tube",
            outer_elem="head_tube_shell",
            margin=0.0005,
            name="steerer_within_head_tube_at_right_lock",
        )
        ctx.check(
            "negative_steer_sweeps_left_bar_forward_inboard",
            turned_right_center[0] > rest_left_center[0] + 0.03
            and turned_right_center[1] < rest_left_center[1] - 0.02,
            details=(
                f"Left bar end rest={rest_left_center}, "
                f"turned={turned_right_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

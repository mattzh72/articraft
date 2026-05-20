from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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
    wire_from_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2((dx * dx + dy * dy) ** 0.5, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    *,
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z_center + z)
        for z, y in rounded_rect_profile(height_z, width_y, radius)
    ]


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="urban_commuter_fork_assembly")

    frame_silver = model.material("frame_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    fork_paint = model.material("fork_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    rack_paint = model.material("rack_paint", rgba=(0.11, 0.12, 0.12, 1.0))
    alloy = model.material("alloy", rgba=(0.79, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    bar_black = model.material("bar_black", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    frame_stub = model.part("frame_stub")
    frame_stub.inertial = Inertial.from_geometry(
        Box((0.16, 0.18, 0.24)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.03, 0.06)),
    )

    head_tube_shell = LatheGeometry.from_shell_profiles(
        [
            (0.028, 0.000),
            (0.028, 0.010),
            (0.026, 0.010),
            (0.026, 0.170),
            (0.028, 0.170),
            (0.028, 0.180),
        ],
        [
            (0.019, 0.000),
            (0.019, 0.180),
        ],
        segments=56,
    )
    frame_stub.visual(
        mesh_from_geometry(head_tube_shell, "head_tube_shell"),
        material=frame_silver,
        name="head_tube_shell",
    )

    frame_stub.visual(
        Box((0.018, 0.028, 0.022)),
        origin=Origin(xyz=(0.027, -0.014, 0.156)),
        material=frame_silver,
        name="top_tube_lug",
    )
    _add_member(
        frame_stub,
        (0.030, -0.026, 0.156),
        (0.040, -0.116, 0.150),
        radius=0.013,
        material=frame_silver,
        name="top_tube_stub",
    )
    frame_stub.visual(
        Box((0.024, 0.036, 0.030)),
        origin=Origin(xyz=(0.031, -0.018, 0.043)),
        material=frame_silver,
        name="down_tube_lug",
    )
    _add_member(
        frame_stub,
        (0.036, -0.032, 0.032),
        (0.050, -0.145, -0.050),
        radius=0.018,
        material=frame_silver,
        name="down_tube_stub",
    )
    headset_cup_ring = LatheGeometry.from_shell_profiles(
        [
            (0.031, 0.000),
            (0.031, 0.006),
        ],
        [
            (0.0156, 0.000),
            (0.0156, 0.006),
        ],
        segments=48,
    )
    frame_stub.visual(
        mesh_from_geometry(headset_cup_ring, "lower_cup_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="lower_cup_lip",
    )
    frame_stub.visual(
        mesh_from_geometry(headset_cup_ring, "upper_cup_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=steel,
        name="upper_cup_lip",
    )

    front_end = model.part("front_end")
    front_end.inertial = Inertial.from_geometry(
        Box((0.68, 0.34, 0.82)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.06, -0.02)),
    )

    front_end.visual(
        Cylinder(radius=0.0145, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=fork_paint,
        name="steerer_tube",
    )
    front_end.visual(
        Cylinder(radius=0.0245, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="lower_headset_race",
    )
    front_end.visual(
        Cylinder(radius=0.0235, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        material=steel,
        name="upper_locknut",
    )
    front_end.visual(
        Cylinder(radius=0.0160, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=steel,
        name="threaded_section",
    )
    for idx, z in enumerate((0.196, 0.214, 0.232, 0.250, 0.268), start=1):
        front_end.visual(
            Cylinder(radius=0.0175, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=f"thread_ridge_{idx}",
        )

    crown_mesh = section_loft(
        [
            _yz_section(x=-0.056, width_y=0.028, height_z=0.020, radius=0.006, z_center=-0.040),
            _yz_section(x=0.000, width_y=0.054, height_z=0.032, radius=0.010, z_center=-0.038),
            _yz_section(x=0.056, width_y=0.028, height_z=0.020, radius=0.006, z_center=-0.040),
        ]
    )
    front_end.visual(
        mesh_from_geometry(crown_mesh, "fork_crown"),
        material=fork_paint,
        name="fork_crown",
    )
    front_end.visual(
        Box((0.040, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=fork_paint,
        name="crown_column",
    )

    left_blade_top = (0.048, 0.000, -0.050)
    left_blade_bottom = (0.050, 0.052, -0.360)
    right_blade_top = (-0.048, 0.000, -0.050)
    right_blade_bottom = (-0.050, 0.052, -0.360)
    _add_member(
        front_end,
        left_blade_top,
        left_blade_bottom,
        radius=0.0115,
        material=fork_paint,
        name="left_blade",
    )
    _add_member(
        front_end,
        right_blade_top,
        right_blade_bottom,
        radius=0.0115,
        material=fork_paint,
        name="right_blade",
    )
    _add_member(
        front_end,
        (0.048, 0.004, -0.092),
        (0.050, 0.020, -0.168),
        radius=0.0135,
        material=fork_paint,
        name="left_blade_reinforcement",
    )
    _add_member(
        front_end,
        (-0.048, 0.004, -0.092),
        (-0.050, 0.020, -0.168),
        radius=0.0135,
        material=fork_paint,
        name="right_blade_reinforcement",
    )

    front_end.visual(
        Box((0.024, 0.020, 0.050)),
        origin=Origin(xyz=(0.050, 0.052, -0.385)),
        material=fork_paint,
        name="left_dropout",
    )
    front_end.visual(
        Box((0.024, 0.020, 0.050)),
        origin=Origin(xyz=(-0.050, 0.052, -0.385)),
        material=fork_paint,
        name="right_dropout",
    )
    front_end.visual(
        Cylinder(radius=0.005, length=0.120),
        origin=Origin(xyz=(0.0, 0.052, -0.390), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="front_axle",
    )

    left_low_rider = wire_from_points(
        [
            (0.090, 0.030, -0.230),
            (0.090, 0.100, -0.230),
            (0.090, 0.148, -0.095),
            (0.090, 0.080, -0.090),
        ],
        radius=0.006,
        radial_segments=16,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.012,
        corner_segments=8,
    )
    right_low_rider = wire_from_points(
        [
            (-0.090, 0.030, -0.230),
            (-0.090, 0.100, -0.230),
            (-0.090, 0.148, -0.095),
            (-0.090, 0.080, -0.090),
        ],
        radius=0.006,
        radial_segments=16,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.012,
        corner_segments=8,
    )
    front_end.visual(
        mesh_from_geometry(left_low_rider, "left_low_rider_loop"),
        material=rack_paint,
        name="left_low_rider_loop",
    )
    front_end.visual(
        mesh_from_geometry(right_low_rider, "right_low_rider_loop"),
        material=rack_paint,
        name="right_low_rider_loop",
    )
    for x in (0.0725, -0.0725):
        front_end.visual(
            Cylinder(radius=0.006, length=0.035),
            origin=Origin(xyz=(x, 0.032, -0.222), rpy=(0.0, pi / 2.0, 0.0)),
            material=rack_paint,
            name=f"{'left' if x > 0 else 'right'}_mid_blade_mount",
        )
        front_end.visual(
            Cylinder(radius=0.0055, length=0.032),
            origin=Origin(xyz=(x, 0.048, -0.148), rpy=(0.0, pi / 2.0, 0.0)),
            material=rack_paint,
            name=f"{'left' if x > 0 else 'right'}_upper_blade_mount",
        )

    platform_z = 0.025
    front_end.visual(
        Box((0.170, 0.200, 0.008)),
        origin=Origin(xyz=(0.0, 0.160, platform_z)),
        material=rack_paint,
        name="rack_platform",
    )
    for x, y in ((0.070, 0.105), (0.070, 0.195), (-0.070, 0.105), (-0.070, 0.195)):
        front_end.visual(
            Cylinder(radius=0.006, length=0.114),
            origin=Origin(xyz=(x, y, -0.036), rpy=(0.0, 0.0, 0.0)),
            material=rack_paint,
            name=f"platform_standoff_{'l' if x > 0 else 'r'}_{'rear' if y < 0.15 else 'front'}",
        )
    for y in (0.105, 0.195):
        front_end.visual(
            Cylinder(radius=0.005, length=0.140),
            origin=Origin(xyz=(0.0, y, -0.093), rpy=(0.0, pi / 2.0, 0.0)),
            material=rack_paint,
            name=f"platform_crossbrace_{'rear' if y < 0.15 else 'front'}",
        )
    _add_member(
        front_end,
        (0.070, 0.105, -0.093),
        (0.0565, 0.048, -0.148),
        radius=0.005,
        material=rack_paint,
        name="left_rear_platform_brace",
    )
    _add_member(
        front_end,
        (-0.070, 0.105, -0.093),
        (-0.0565, 0.048, -0.148),
        radius=0.005,
        material=rack_paint,
        name="right_rear_platform_brace",
    )
    _add_member(
        front_end,
        (0.070, 0.195, -0.093),
        (0.090, 0.148, -0.095),
        radius=0.005,
        material=rack_paint,
        name="left_front_platform_brace",
    )
    _add_member(
        front_end,
        (-0.070, 0.195, -0.093),
        (-0.090, 0.148, -0.095),
        radius=0.005,
        material=rack_paint,
        name="right_front_platform_brace",
    )

    front_end.visual(
        Cylinder(radius=0.0115, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=alloy,
        name="quill_shaft",
    )
    _add_member(
        front_end,
        (0.0, 0.0, 0.275),
        (0.0, 0.078, 0.325),
        radius=0.011,
        material=alloy,
        name="stem_extension",
    )
    front_end.visual(
        Box((0.068, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, 0.090, 0.336)),
        material=alloy,
        name="stem_clamp",
    )
    front_end.visual(
        Box((0.030, 0.025, 0.022)),
        origin=Origin(xyz=(0.0, 0.078, 0.312)),
        material=alloy,
        name="stem_shoulder",
    )
    for z in (0.324, 0.348):
        front_end.visual(
            Cylinder(radius=0.0042, length=0.082),
            origin=Origin(xyz=(0.0, 0.090, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"clamp_bolt_{int(round(z * 1000))}",
        )

    handlebar_geom = tube_from_spline_points(
        [
            (-0.300, 0.018, 0.376),
            (-0.235, 0.035, 0.372),
            (-0.160, 0.058, 0.358),
            (-0.075, 0.084, 0.342),
            (0.000, 0.090, 0.336),
            (0.075, 0.084, 0.342),
            (0.160, 0.058, 0.358),
            (0.235, 0.035, 0.372),
            (0.300, 0.018, 0.376),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
    )
    front_end.visual(
        mesh_from_geometry(handlebar_geom, "handlebar_bar"),
        material=bar_black,
        name="handlebar_bar",
    )
    front_end.visual(
        Cylinder(radius=0.0165, length=0.090),
        origin=Origin(xyz=(-0.255, 0.028, 0.373), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    front_end.visual(
        Cylinder(radius=0.0165, length=0.090),
        origin=Origin(xyz=(0.255, 0.028, 0.373), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=frame_stub,
        child=front_end,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_stub = object_model.get_part("frame_stub")
    front_end = object_model.get_part("front_end")
    steering = object_model.get_articulation("steering")

    frame_stub.get_visual("head_tube_shell")
    front_end.get_visual("steerer_tube")
    front_end.get_visual("fork_crown")
    front_end.get_visual("front_axle")
    front_end.get_visual("rack_platform")
    front_end.get_visual("handlebar_bar")

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

    # Keep pose-specific checks lean.
    # Do not add blanket lower/upper pose sweeps or
    # `fail_if_parts_overlap_in_sampled_poses(...)` by default.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.

    ctx.expect_contact(front_end, frame_stub)
    ctx.check(
        "steering_axis_vertical",
        tuple(float(v) for v in steering.axis) == (0.0, 0.0, 1.0),
        f"axis={steering.axis}",
    )
    limits = steering.motion_limits
    ctx.check(
        "steering_limits_realistic",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -0.60
        and limits.upper >= 0.60,
        f"limits={limits}",
    )

    axle_aabb = ctx.part_element_world_aabb(front_end, elem="front_axle")
    platform_aabb = ctx.part_element_world_aabb(front_end, elem="rack_platform")
    handlebar_aabb = ctx.part_element_world_aabb(front_end, elem="handlebar_bar")
    assert axle_aabb is not None
    assert platform_aabb is not None
    assert handlebar_aabb is not None

    axle_center = _aabb_center(axle_aabb)
    platform_clearance = platform_aabb[0][2] - axle_aabb[1][2]
    bar_width = handlebar_aabb[1][0] - handlebar_aabb[0][0]
    ctx.check(
        "rack_platform_above_axle",
        platform_clearance > 0.34,
        f"platform_clearance={platform_clearance:.4f}",
    )
    ctx.check(
        "riser_handlebar_width_realistic",
        0.56 <= bar_width <= 0.64,
        f"bar_width={bar_width:.4f}",
    )
    ctx.check(
        "axle_sits_forward_of_steerer",
        axle_center[1] > 0.040,
        f"axle_center={axle_center}",
    )

    with ctx.pose({steering: 0.35}):
        turned_axle_aabb = ctx.part_element_world_aabb(front_end, elem="front_axle")
        assert turned_axle_aabb is not None
        turned_axle_center = _aabb_center(turned_axle_aabb)
        ctx.expect_contact(front_end, frame_stub)
        ctx.check(
            "steering_rotates_about_steerer",
            turned_axle_center[0] < axle_center[0] - 0.010
            and turned_axle_center[1] < axle_center[1] - 0.001,
            f"rest={axle_center}, turned={turned_axle_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

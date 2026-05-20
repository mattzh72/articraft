from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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
    sweep_profile_along_spline,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_bike_front_fork")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.20, 0.19, 1.0))
    fork_paint = model.material("fork_paint", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.69, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    head_angle = radians(18.0)
    head_tube_length = 0.180
    steerer_radius = 0.0143

    frame_stub = model.part("frame_stub")
    frame_stub.inertial = Inertial.from_geometry(
        Box((0.18, 0.36, 0.42)),
        mass=4.5,
        origin=Origin(xyz=(0.0, -0.14, 0.03)),
    )

    head_tube_shell = LatheGeometry.from_shell_profiles(
        [
            (0.031, 0.000),
            (0.029, 0.014),
            (0.028, 0.090),
            (0.029, 0.166),
            (0.031, head_tube_length),
        ],
        [
            (0.020, 0.000),
            (0.020, head_tube_length),
        ],
        segments=56,
    )
    frame_stub.visual(
        _save_mesh("head_tube_shell", head_tube_shell),
        origin=Origin(rpy=(head_angle, 0.0, 0.0)),
        material=frame_paint,
        name="head_tube_shell",
    )

    for side, sign in (("left", 1.0), ("right", -1.0)):
        frame_stub.visual(
            Box((0.012, 0.100, 0.090)),
            origin=Origin(xyz=(sign * 0.025, -0.072, 0.148)),
            material=frame_paint,
            name=f"{side}_upper_lug",
        )
        frame_stub.visual(
            Box((0.012, 0.140, 0.140)),
            origin=Origin(xyz=(sign * 0.025, -0.120, -0.015)),
            material=frame_paint,
            name=f"{side}_lower_lug",
        )
        frame_stub.visual(
            Box((0.012, 0.060, 0.070)),
            origin=Origin(xyz=(sign * 0.025, -0.080, 0.040)),
            material=frame_paint,
            name=f"{side}_lower_gusset",
        )
        frame_stub.visual(
            Box((0.012, 0.080, 0.120)),
            origin=Origin(xyz=(sign * 0.025, -0.087, 0.089)),
            material=frame_paint,
            name=f"{side}_side_web",
        )

    frame_stub.visual(
        Cylinder(radius=0.020, length=0.210),
        origin=Origin(xyz=(0.0, -0.205, 0.160), rpy=(1.50, 0.0, 0.0)),
        material=frame_paint,
        name="top_tube_stub",
    )

    frame_stub.visual(
        Cylinder(radius=0.023, length=0.270),
        origin=Origin(xyz=(0.0, -0.175, -0.090), rpy=(2.60, 0.0, 0.0)),
        material=frame_paint,
        name="down_tube_stub",
    )

    steer_assembly = model.part("steer_assembly")
    steer_assembly.inertial = Inertial.from_geometry(
        Box((0.78, 0.48, 0.82)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.07, -0.08)),
    )

    steer_assembly.visual(
        Cylinder(radius=steerer_radius, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=steel,
        name="steerer_tube",
    )
    lower_headset_ring = LatheGeometry.from_shell_profiles(
        [
            (0.036, 0.000),
            (0.034, 0.003),
            (0.034, 0.007),
            (0.036, 0.010),
        ],
        [
            (0.031, 0.000),
            (0.031, 0.010),
        ],
        segments=48,
    )
    steer_assembly.visual(
        _save_mesh("lower_headset_ring", lower_headset_ring),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="lower_headset_ring",
    )
    steer_assembly.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, head_tube_length + 0.014)),
        material=steel,
        name="upper_spacer_lower",
    )
    steer_assembly.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, head_tube_length + 0.024)),
        material=steel,
        name="upper_spacer_upper",
    )

    steer_assembly.visual(
        Box((0.082, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, 0.022, -0.020)),
        material=fork_paint,
        name="crown_yoke",
    )
    steer_assembly.visual(
        Cylinder(radius=0.019, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="crown_race_seat",
    )
    steer_assembly.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, head_tube_length + 0.016)),
        material=steel,
        name="upper_dust_cap",
    )

    blade_profile = rounded_rect_profile(0.028, 0.017, 0.005, corner_segments=6)
    left_blade_points = [
        (0.038, 0.018, -0.018),
        (0.050, 0.030, -0.090),
        (0.064, 0.046, -0.190),
        (0.077, 0.055, -0.300),
        (0.088, 0.060, -0.420),
    ]
    right_blade_points = _mirror_x(left_blade_points)
    left_blade = sweep_profile_along_spline(
        left_blade_points,
        profile=blade_profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    right_blade = sweep_profile_along_spline(
        right_blade_points,
        profile=blade_profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    steer_assembly.visual(
        _save_mesh("left_fork_blade", left_blade),
        material=fork_paint,
        name="left_fork_blade",
    )
    steer_assembly.visual(
        _save_mesh("right_fork_blade", right_blade),
        material=fork_paint,
        name="right_fork_blade",
    )

    steer_assembly.visual(
        Box((0.014, 0.010, 0.054)),
        origin=Origin(xyz=(0.089, 0.060, -0.447)),
        material=fork_paint,
        name="left_dropout",
    )
    steer_assembly.visual(
        Box((0.014, 0.010, 0.054)),
        origin=Origin(xyz=(-0.089, 0.060, -0.447)),
        material=fork_paint,
        name="right_dropout",
    )

    steer_assembly.visual(
        Box((0.012, 0.028, 0.018)),
        origin=Origin(xyz=(0.084, 0.049, -0.255)),
        material=steel,
        name="left_lowrider_tab",
    )
    steer_assembly.visual(
        Box((0.012, 0.028, 0.018)),
        origin=Origin(xyz=(-0.084, 0.049, -0.255)),
        material=steel,
        name="right_lowrider_tab",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        for bolt_index, bolt_y in enumerate((0.042, 0.056), start=1):
            steer_assembly.visual(
                Cylinder(radius=0.0045, length=0.008),
                origin=Origin(xyz=(sign * 0.092, bolt_y, -0.255), rpy=(0.0, pi / 2.0, 0.0)),
                material=steel,
                name=f"{side}_rack_bolt_{bolt_index}",
            )

    rack_perimeter = wire_from_points(
        [
            (-0.135, 0.038, -0.110),
            (-0.135, 0.248, -0.110),
            (0.135, 0.248, -0.110),
            (0.135, 0.038, -0.110),
        ],
        radius=0.007,
        radial_segments=16,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.028,
        corner_segments=10,
    )
    steer_assembly.visual(
        _save_mesh("rack_perimeter", rack_perimeter),
        material=fork_paint,
        name="rack_perimeter",
    )

    rack_front_cross = tube_from_spline_points(
        [(-0.125, 0.245, -0.110), (0.125, 0.245, -0.110)],
        radius=0.006,
        samples_per_segment=2,
        radial_segments=14,
    )
    rack_rear_cross = tube_from_spline_points(
        [(-0.125, 0.041, -0.110), (0.125, 0.041, -0.110)],
        radius=0.006,
        samples_per_segment=2,
        radial_segments=14,
    )
    steer_assembly.visual(
        _save_mesh("rack_front_cross", rack_front_cross),
        material=fork_paint,
        name="rack_front_cross",
    )
    steer_assembly.visual(
        _save_mesh("rack_rear_cross", rack_rear_cross),
        material=fork_paint,
        name="rack_rear_cross",
    )

    for slat_index, slat_x in enumerate((-0.055, 0.0, 0.055), start=1):
        steer_assembly.visual(
            Box((0.012, 0.214, 0.008)),
            origin=Origin(xyz=(slat_x, 0.143, -0.110)),
            material=steel,
            name=f"rack_slat_{slat_index}",
        )

    strut_specs = [
        ("left_rear_strut", [(0.084, 0.047, -0.245), (0.112, 0.050, -0.180), (0.135, 0.044, -0.110)]),
        ("left_front_strut", [(0.084, 0.051, -0.245), (0.120, 0.130, -0.180), (0.135, 0.242, -0.110)]),
        ("right_rear_strut", _mirror_x([(0.084, 0.047, -0.245), (0.112, 0.050, -0.180), (0.135, 0.044, -0.110)])),
        ("right_front_strut", _mirror_x([(0.084, 0.051, -0.245), (0.120, 0.130, -0.180), (0.135, 0.242, -0.110)])),
    ]
    for name, points in strut_specs:
        steer_assembly.visual(
            _save_mesh(name, tube_from_spline_points(points, radius=0.006, samples_per_segment=10, radial_segments=14)),
            material=fork_paint,
            name=name,
        )

    steer_assembly.visual(
        Cylinder(radius=0.021, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.236)),
        material=satin_black,
        name="stem_steerer_clamp",
    )
    steer_assembly.visual(
        Box((0.034, 0.050, 0.044)),
        origin=Origin(xyz=(0.0, 0.018, 0.254)),
        material=satin_black,
        name="stem_riser_block",
    )
    steer_assembly.visual(
        Box((0.040, 0.090, 0.030)),
        origin=Origin(xyz=(0.0, 0.044, 0.272)),
        material=satin_black,
        name="stem_body",
    )
    steer_assembly.visual(
        Cylinder(radius=0.019, length=0.060),
        origin=Origin(xyz=(0.0, 0.094, 0.286), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="face_collar",
    )
    steer_assembly.visual(
        Box((0.016, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.106, 0.286)),
        material=steel,
        name="face_plate",
    )

    handlebar = tube_from_spline_points(
        [
            (-0.370, 0.066, 0.292),
            (-0.255, 0.080, 0.290),
            (-0.110, 0.092, 0.288),
            (0.0, 0.094, 0.286),
            (0.110, 0.092, 0.288),
            (0.255, 0.080, 0.290),
            (0.370, 0.066, 0.292),
        ],
        radius=0.011,
        samples_per_segment=16,
        radial_segments=18,
    )
    steer_assembly.visual(
        _save_mesh("bar_span", handlebar),
        material=satin_black,
        name="bar_span",
    )
    steer_assembly.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(0.337, 0.070, 0.291), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    steer_assembly.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(-0.337, 0.070, 0.291), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )

    model.articulation(
        "head_tube_steer",
        ArticulationType.REVOLUTE,
        parent=frame_stub,
        child=steer_assembly,
        origin=Origin(rpy=(head_angle, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=-0.72, upper=0.72),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_stub = object_model.get_part("frame_stub")
    steer_assembly = object_model.get_part("steer_assembly")
    steer_joint = object_model.get_articulation("head_tube_steer")

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
        steer_assembly,
        frame_stub,
        elem_a="lower_headset_ring",
        elem_b="head_tube_shell",
        name="lower headset seats against the head tube",
    )
    ctx.expect_overlap(
        steer_assembly,
        frame_stub,
        axes="xy",
        elem_a="steerer_tube",
        elem_b="head_tube_shell",
        min_overlap=0.026,
        name="steerer stays coaxial within the head tube footprint",
    )
    ctx.expect_gap(
        steer_assembly,
        frame_stub,
        axis="y",
        positive_elem="rack_front_cross",
        negative_elem="head_tube_shell",
        min_gap=0.10,
        name="front rack platform projects ahead of the head tube",
    )

    rest_rack_center = _aabb_center(ctx.part_element_world_aabb(steer_assembly, elem="rack_front_cross"))
    rest_grip_center = _aabb_center(ctx.part_element_world_aabb(steer_assembly, elem="right_grip"))
    upper_limit = steer_joint.motion_limits.upper if steer_joint.motion_limits is not None else None
    turned_rack_center = None
    turned_grip_center = None
    if upper_limit is not None:
        with ctx.pose({steer_joint: upper_limit}):
            turned_rack_center = _aabb_center(ctx.part_element_world_aabb(steer_assembly, elem="rack_front_cross"))
            turned_grip_center = _aabb_center(ctx.part_element_world_aabb(steer_assembly, elem="right_grip"))

    ctx.check(
        "positive steering swings the rack left around the head-tube axis",
        rest_rack_center is not None
        and turned_rack_center is not None
        and turned_rack_center[0] < rest_rack_center[0] - 0.08,
        details=f"rest_rack={rest_rack_center}, turned_rack={turned_rack_center}",
    )
    ctx.check(
        "positive steering carries the right grip forward and inboard",
        rest_grip_center is not None
        and turned_grip_center is not None
        and turned_grip_center[0] < rest_grip_center[0] - 0.08
        and turned_grip_center[1] > rest_grip_center[1] + 0.12,
        details=f"rest_grip={rest_grip_center}, turned_grip={turned_grip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

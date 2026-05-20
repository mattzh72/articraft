from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import dist, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, f"road_bike_fork_v10_{name}")


def _rounded_loop_xy(
    width: float,
    depth: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    chamfer = min(half_w, half_d) * 0.55
    return [
        (center_x + half_w - chamfer, center_y + half_d, z),
        (center_x + half_w, center_y + half_d - chamfer, z),
        (center_x + half_w, center_y - half_d + chamfer, z),
        (center_x + half_w - chamfer, center_y - half_d, z),
        (center_x - half_w + chamfer, center_y - half_d, z),
        (center_x - half_w, center_y - half_d + chamfer, z),
        (center_x - half_w, center_y + half_d - chamfer, z),
        (center_x - half_w + chamfer, center_y + half_d, z),
    ]


def _rounded_loop_xz(
    width: float,
    height: float,
    y: float,
    *,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    chamfer = min(half_w, half_h) * 0.55
    return [
        (center_x + half_w - chamfer, y, center_z + half_h),
        (center_x + half_w, y, center_z + half_h - chamfer),
        (center_x + half_w, y, center_z - half_h + chamfer),
        (center_x + half_w - chamfer, y, center_z - half_h),
        (center_x - half_w + chamfer, y, center_z - half_h),
        (center_x - half_w, y, center_z - half_h + chamfer),
        (center_x - half_w, y, center_z + half_h - chamfer),
        (center_x - half_w + chamfer, y, center_z + half_h),
    ]


def _build_blade_mesh(side_sign: float):
    sections = [
        _rounded_loop_xy(0.024, 0.018, -0.042, center_x=side_sign * 0.040, center_y=0.000),
        _rounded_loop_xy(0.022, 0.016, -0.105, center_x=side_sign * 0.041, center_y=0.002),
        _rounded_loop_xy(0.019, 0.013, -0.185, center_x=side_sign * 0.043, center_y=0.009),
        _rounded_loop_xy(0.015, 0.010, -0.285, center_x=side_sign * 0.046, center_y=0.020),
        _rounded_loop_xy(0.012, 0.008, -0.360, center_x=side_sign * 0.049, center_y=0.028),
        _rounded_loop_xy(0.010, 0.006, -0.395, center_x=side_sign * 0.050, center_y=0.032),
    ]
    return section_loft(sections)


def _build_crown_mesh():
    return section_loft(
        [
            _rounded_loop_xy(0.122, 0.034, -0.048, center_y=0.001),
            _rounded_loop_xy(0.102, 0.038, -0.035, center_y=0.002),
            _rounded_loop_xy(0.072, 0.034, -0.022, center_y=0.001),
            _rounded_loop_xy(0.044, 0.028, -0.008, center_y=0.000),
        ]
    )


def _build_stem_body_mesh():
    return section_loft(
        [
            _rounded_loop_xz(0.052, 0.046, 0.008, center_z=0.199),
            _rounded_loop_xz(0.044, 0.036, 0.050, center_z=0.197),
            _rounded_loop_xz(0.042, 0.030, 0.082, center_z=0.197),
            _rounded_loop_xz(0.050, 0.028, 0.100, center_z=0.198),
        ]
    )


def _build_head_tube_shell():
    outer_profile = [
        (0.030, 0.000),
        (0.029, 0.016),
        (0.0275, 0.075),
        (0.0265, 0.145),
        (0.026, 0.160),
    ]
    inner_profile = [
        (0.0212, 0.000),
        (0.0206, 0.016),
        (0.0192, 0.075),
        (0.0178, 0.145),
        (0.0175, 0.160),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_head_tube_lower_cup():
    return LatheGeometry.from_shell_profiles(
        [(0.032, -0.018), (0.031, -0.008), (0.030, 0.000)],
        [(0.026, -0.018), (0.026, -0.008), (0.026, 0.000)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_head_tube_upper_cup():
    return LatheGeometry.from_shell_profiles(
        [(0.026, 0.160), (0.029, 0.170), (0.031, 0.182)],
        [(0.0225, 0.160), (0.0225, 0.170), (0.0225, 0.182)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_headset_lower_ring():
    outer_profile = [
        (0.026, -0.010),
        (0.027, -0.004),
        (0.026, 0.000),
    ]
    inner_profile = [
        (0.0190, -0.010),
        (0.0190, -0.004),
        (0.0190, 0.000),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_headset_upper_ring():
    outer_profile = [
        (0.0225, 0.160),
        (0.0235, 0.166),
        (0.0225, 0.172),
    ]
    inner_profile = [
        (0.0145, 0.160),
        (0.0145, 0.166),
        (0.0145, 0.172),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_steerer_mesh():
    return LatheGeometry(
        [
            (0.0185, -0.030),
            (0.0185, 0.060),
            (0.0172, 0.098),
            (0.0158, 0.140),
            (0.0142, 0.205),
            (0.0142, 0.252),
        ],
        segments=56,
    )


def _build_top_cap_mesh():
    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.010, 0.000),
            (0.016, 0.002),
            (0.016, 0.006),
            (0.013, 0.010),
            (0.000, 0.011),
        ],
        segments=48,
    )


def _build_spacer_ring():
    return LatheGeometry.from_shell_profiles(
        [(0.0225, 0.160), (0.0225, 0.252)],
        [(0.0142, 0.160), (0.0142, 0.252)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _build_handlebar_mesh():
    return wire_from_points(
        [
            (-0.220, 0.112, 0.070),
            (-0.215, 0.124, 0.094),
            (-0.205, 0.133, 0.126),
            (-0.182, 0.130, 0.160),
            (-0.152, 0.112, 0.188),
            (-0.118, 0.095, 0.198),
            (-0.070, 0.090, 0.198),
            (0.070, 0.090, 0.198),
            (0.118, 0.095, 0.198),
            (0.152, 0.112, 0.188),
            (0.182, 0.130, 0.160),
            (0.205, 0.133, 0.126),
            (0.215, 0.124, 0.094),
            (0.220, 0.112, 0.070),
        ],
        radius=0.012,
        radial_segments=18,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=12,
    )


def _build_crown_bridge_mesh():
    return wire_from_points(
        [
            (-0.042, -0.004, -0.067),
            (-0.020, -0.015, -0.056),
            (0.020, -0.015, -0.056),
            (0.042, -0.004, -0.067),
        ],
        radius=0.0055,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.012,
        corner_segments=8,
    )


def _has_visual(part, visual_name: str) -> bool:
    return any(visual.name == visual_name for visual in part.visuals)


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_fork_and_cockpit")

    head_tilt = radians(17.0)

    carbon_black = model.material("carbon_black", rgba=(0.09, 0.09, 0.10, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    alloy = model.material("alloy", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.33, 0.35, 0.38, 1.0))
    support_grey = model.material("support_grey", rgba=(0.42, 0.44, 0.47, 1.0))

    head_tube = model.part("head_tube")
    head_tube.visual(
        _save_mesh("head_tube_shell", _build_head_tube_shell()),
        origin=Origin(rpy=(head_tilt, 0.0, 0.0)),
        material=support_grey,
        name="head_tube_shell",
    )
    head_tube.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.190)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.004, 0.080), rpy=(head_tilt, 0.0, 0.0)),
    )

    steering = model.part("steering_assembly")
    steering.visual(
        _save_mesh("tapered_steerer", _build_steerer_mesh()),
        material=dark_alloy,
        name="tapered_steerer",
    )
    steering.visual(
        _save_mesh("fork_crown", _build_crown_mesh()),
        material=carbon_black,
        name="fork_crown",
    )
    steering.visual(
        _save_mesh("left_blade", _build_blade_mesh(-1.0)),
        material=carbon_black,
        name="left_blade",
    )
    steering.visual(
        _save_mesh("right_blade", _build_blade_mesh(1.0)),
        material=carbon_black,
        name="right_blade",
    )
    steering.visual(
        _save_mesh("crown_bridge", _build_crown_bridge_mesh()),
        material=carbon_black,
        name="crown_bridge",
    )
    steering.visual(
        _save_mesh("stem_body", _build_stem_body_mesh()),
        material=satin_black,
        name="stem_body",
    )
    steering.visual(
        Box((0.046, 0.036, 0.052)),
        origin=Origin(xyz=(0.0, 0.010, 0.199)),
        material=satin_black,
        name="steerer_clamp",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.094, 0.198), rpy=(0.0, radians(90.0), 0.0)),
        material=satin_black,
        name="bar_clamp",
    )
    steering.visual(
        Box((0.048, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, 0.110, 0.198)),
        material=satin_black,
        name="faceplate",
    )
    steering.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.0, 0.119, 0.212), rpy=(-radians(90.0), 0.0, 0.0)),
        material=alloy,
        name="faceplate_bolt_upper",
    )
    steering.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.0, 0.119, 0.184), rpy=(-radians(90.0), 0.0, 0.0)),
        material=alloy,
        name="faceplate_bolt_lower",
    )
    steering.visual(
        _save_mesh("handlebar", _build_handlebar_mesh()),
        material=satin_black,
        name="handlebar",
    )
    steering.visual(
        Sphere(radius=0.0115),
        origin=Origin(xyz=(-0.220, 0.112, 0.070)),
        material=satin_black,
        name="left_bar_end",
    )
    steering.visual(
        Sphere(radius=0.0115),
        origin=Origin(xyz=(0.220, 0.112, 0.070)),
        material=satin_black,
        name="right_bar_end",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.257)),
        material=alloy,
        name="top_cap",
    )
    steering.visual(
        Cylinder(radius=0.0045, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        material=dark_alloy,
        name="top_cap_bolt",
    )
    steering.visual(
        Box((0.018, 0.005, 0.032)),
        origin=Origin(xyz=(-0.050, 0.032, -0.394)),
        material=dark_alloy,
        name="left_dropout",
    )
    steering.visual(
        Box((0.018, 0.005, 0.032)),
        origin=Origin(xyz=(0.050, 0.032, -0.394)),
        material=dark_alloy,
        name="right_dropout",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.46, 0.16, 0.69)),
        mass=2.20,
        origin=Origin(xyz=(0.0, 0.085, -0.070)),
    )

    model.articulation(
        "steering_rotation",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(rpy=(head_tilt, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=3.0,
            lower=-0.75,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steering = object_model.get_part("steering_assembly")
    steering_rotation = object_model.get_articulation("steering_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0015)
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

    required_visuals = (
        "head_tube_shell",
        "tapered_steerer",
        "fork_crown",
        "left_blade",
        "right_blade",
        "crown_bridge",
        "stem_body",
        "faceplate",
        "faceplate_bolt_upper",
        "faceplate_bolt_lower",
        "handlebar",
        "left_dropout",
        "right_dropout",
        "left_bar_end",
        "right_bar_end",
        "top_cap",
    )
    ctx.check(
        "prompt visuals present",
        all(
            _has_visual(head_tube if visual_name == "head_tube_shell" else steering, visual_name)
            for visual_name in required_visuals
        ),
        details="Expected head tube shell, fork details, cockpit, and visible bolts are missing.",
    )
    ctx.check(
        "steering joint axis and limits",
        steering_rotation.axis == (0.0, 0.0, 1.0)
        and steering_rotation.motion_limits is not None
        and steering_rotation.motion_limits.lower is not None
        and steering_rotation.motion_limits.upper is not None
        and steering_rotation.motion_limits.lower <= -0.70
        and steering_rotation.motion_limits.upper >= 0.70,
        details="Steering should revolve smoothly about the steerer axis with realistic left-right range.",
    )
    ctx.expect_contact(
        steering,
        head_tube,
        contact_tol=0.0015,
        name="steering assembly seats on headset faces",
    )
    ctx.expect_overlap(
        steering,
        head_tube,
        axes="xy",
        min_overlap=0.040,
        name="steering assembly stays centered over head tube",
    )

    left_rest_aabb = ctx.part_element_world_aabb(steering, elem="left_bar_end")
    right_rest_aabb = ctx.part_element_world_aabb(steering, elem="right_bar_end")
    left_rest = _aabb_center(left_rest_aabb) if left_rest_aabb is not None else None
    right_rest = _aabb_center(right_rest_aabb) if right_rest_aabb is not None else None
    ctx.check(
        "bar end trackers resolve",
        left_rest is not None and right_rest is not None,
        details="Named bar-end visuals must exist so steering motion can be verified.",
    )
    if left_rest is not None and right_rest is not None:
        with ctx.pose({steering_rotation: 0.45}):
            ctx.expect_contact(
                steering,
                head_tube,
                contact_tol=0.0015,
                name="steering contact retained while turned",
            )
            left_turn_aabb = ctx.part_element_world_aabb(steering, elem="left_bar_end")
            right_turn_aabb = ctx.part_element_world_aabb(steering, elem="right_bar_end")
            left_turn = _aabb_center(left_turn_aabb) if left_turn_aabb is not None else None
            right_turn = _aabb_center(right_turn_aabb) if right_turn_aabb is not None else None
            ctx.check(
                "handlebar swings with steering articulation",
                left_turn is not None
                and right_turn is not None
                and dist(left_rest, left_turn) > 0.060
                and dist(right_rest, right_turn) > 0.060,
                details="Both bar ends should sweep a clear arc when the steering rotates.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

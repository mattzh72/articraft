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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


HEAD_TILT = radians(18.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_superellipse(
    width: float,
    depth: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    exponent: float = 3.0,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (cx + x, cy + y, z)
        for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)
    ]


def _xz_roundrect(
    width: float,
    height: float,
    y: float,
    *,
    cx: float = 0.0,
    cz: float = 0.0,
    radius: float | None = None,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    corner_radius = radius if radius is not None else min(width, height) * 0.24
    return [
        (cx + x, y, cz + z)
        for x, z in rounded_rect_profile(
            width,
            height,
            corner_radius,
            corner_segments=corner_segments,
        )
    ]


def _loft(sections: list[list[tuple[float, float, float]]]):
    return repair_loft(section_loft(sections), repair="mesh")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gravel_bike_fork_front_end")

    carbon = model.material("carbon", rgba=(0.12, 0.13, 0.14, 1.0))
    stem_black = model.material("stem_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hood_black = model.material("hood_black", rgba=(0.06, 0.06, 0.07, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.23, 0.24, 0.26, 1.0))
    frame_graphite = model.material("frame_graphite", rgba=(0.15, 0.16, 0.18, 1.0))

    frame_stub = model.part("frame_stub")
    frame_stub.inertial = Inertial.from_geometry(
        Box((0.13, 0.10, 0.23)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    head_tube_shell = LatheGeometry.from_shell_profiles(
        [
            (0.040, -0.078),
            (0.038, -0.040),
            (0.034, 0.040),
            (0.030, 0.078),
        ],
        [
            (0.026, -0.078),
            (0.024, -0.040),
            (0.021, 0.040),
            (0.0185, 0.078),
        ],
        segments=56,
    ).rotate_x(HEAD_TILT)
    frame_stub.visual(
        _save_mesh("frame_head_tube_shell", head_tube_shell),
        material=frame_graphite,
        name="head_tube_shell",
    )

    lower_headset = LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.090),
            (0.036, -0.082),
            (0.035, -0.072),
        ],
        [
            (0.025, -0.090),
            (0.025, -0.082),
            (0.025, -0.072),
        ],
        segments=48,
    ).rotate_x(HEAD_TILT)
    frame_stub.visual(
        _save_mesh("frame_lower_headset", lower_headset),
        material=dark_alloy,
        name="lower_headset",
    )

    upper_headset = LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.072),
            (0.025, 0.082),
            (0.023, 0.094),
        ],
        [
            (0.018, 0.072),
            (0.018, 0.082),
            (0.018, 0.094),
        ],
        segments=48,
    ).rotate_x(HEAD_TILT)
    frame_stub.visual(
        _save_mesh("frame_upper_headset", upper_headset),
        material=dark_alloy,
        name="upper_headset",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.74, 0.22, 0.66)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.07, -0.10)),
    )

    steerer_geom = LatheGeometry(
        [
            (0.020, -0.115),
            (0.020, -0.020),
            (0.0185, 0.020),
            (0.0160, 0.060),
            (0.0143, 0.090),
            (0.0143, 0.165),
        ],
        segments=48,
    )
    fork.visual(
        _save_mesh("fork_steerer_tube", steerer_geom),
        material=carbon,
        name="steerer_tube",
    )

    crown_geom = _loft(
        [
            _xy_superellipse(0.132, 0.066, -0.145, cy=0.018, exponent=3.6),
            _xy_superellipse(0.102, 0.056, -0.118, cy=0.012, exponent=3.5),
            _xy_superellipse(0.060, 0.044, -0.092, cy=0.005, exponent=3.2),
        ]
    )
    fork.visual(
        _save_mesh("fork_crown_shell", crown_geom),
        material=carbon,
        name="crown_shell",
    )

    blade_sections = [
        (-0.126, 0.040, 0.013, 0.034, 0.022),
        (-0.270, 0.046, 0.032, 0.026, 0.017),
        (-0.416, 0.055, 0.055, 0.020, 0.012),
    ]
    left_blade_geom = _loft(
        [
            _xy_superellipse(width, depth, z, cx=x, cy=y, exponent=3.3, segments=40)
            for z, x, y, width, depth in blade_sections
        ]
    )
    fork.visual(
        _save_mesh("fork_left_blade", left_blade_geom),
        material=carbon,
        name="left_blade",
    )

    right_blade_geom = _loft(
        [
            _xy_superellipse(width, depth, z, cx=-x, cy=y, exponent=3.3, segments=40)
            for z, x, y, width, depth in blade_sections
        ]
    )
    fork.visual(
        _save_mesh("fork_right_blade", right_blade_geom),
        material=carbon,
        name="right_blade",
    )

    fork.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=dark_alloy,
        name="crown_race",
    )

    fork.visual(
        Box((0.020, 0.022, 0.026)),
        origin=Origin(xyz=(0.055, 0.055, -0.414)),
        material=dark_alloy,
        name="left_dropout",
    )
    fork.visual(
        Box((0.020, 0.022, 0.026)),
        origin=Origin(xyz=(-0.055, 0.055, -0.414)),
        material=dark_alloy,
        name="right_dropout",
    )
    fork.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.100507)),
        material=dark_alloy,
        name="compression_ring",
    )

    steerer_clamp = LatheGeometry.from_shell_profiles(
        [
            (0.026, -0.021),
            (0.026, 0.021),
        ],
        [
            (0.0143, -0.021),
            (0.0143, 0.021),
        ],
        segments=48,
    )
    fork.visual(
        _save_mesh("stem_steerer_clamp", steerer_clamp),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=stem_black,
        name="steerer_clamp",
    )
    fork.visual(
        Box((0.033, 0.034, 0.034)),
        origin=Origin(xyz=(0.0, 0.012, 0.131)),
        material=stem_black,
        name="steerer_bridge",
    )

    left_rib = _loft(
        [
            _xz_roundrect(0.014, 0.030, 0.018, cx=0.020, cz=-0.010, radius=0.004),
            _xz_roundrect(0.012, 0.026, 0.045, cx=0.022, cz=-0.007, radius=0.004),
            _xz_roundrect(0.012, 0.020, 0.076, cx=0.023, cz=-0.003, radius=0.004),
        ]
    )
    fork.visual(
        _save_mesh("stem_left_rib", left_rib),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=stem_black,
        name="left_rib",
    )
    right_rib = _loft(
        [
            _xz_roundrect(0.014, 0.030, 0.018, cx=-0.020, cz=-0.010, radius=0.004),
            _xz_roundrect(0.012, 0.026, 0.045, cx=-0.022, cz=-0.007, radius=0.004),
            _xz_roundrect(0.012, 0.020, 0.076, cx=-0.023, cz=-0.003, radius=0.004),
        ]
    )
    fork.visual(
        _save_mesh("stem_right_rib", right_rib),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=stem_black,
        name="right_rib",
    )
    fork.visual(
        Box((0.030, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.048, 0.115)),
        material=stem_black,
        name="lower_bridge",
    )
    fork.visual(
        Box((0.022, 0.054, 0.016)),
        origin=Origin(xyz=(0.0, 0.076, 0.126)),
        material=stem_black,
        name="nose_bridge",
    )

    bar_clamp = LatheGeometry.from_shell_profiles(
        [
            (0.0275, -0.026),
            (0.0275, 0.026),
        ],
        [
            (0.0159, -0.026),
            (0.0159, 0.026),
        ],
        segments=48,
    ).rotate_y(pi / 2.0).translate(0.0, 0.096, 0.006)
    fork.visual(
        _save_mesh("stem_bar_clamp", bar_clamp),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=stem_black,
        name="bar_clamp",
    )

    clamp_face = LatheGeometry.from_shell_profiles(
        [
            (0.030, -0.008),
            (0.030, 0.008),
        ],
        [
            (0.0159, -0.008),
            (0.0159, 0.008),
        ],
        segments=48,
    ).rotate_y(pi / 2.0).translate(0.0, 0.114, 0.006)
    fork.visual(
        _save_mesh("stem_clamp_face", clamp_face),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=stem_black,
        name="clamp_face",
    )
    fork.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.121, 0.134), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stem_black,
        name="center_pinch_collar",
    )
    fork.visual(
        Cylinder(radius=0.0032, length=0.018),
        origin=Origin(xyz=(0.0, 0.124, 0.134), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="center_pinch_bolt",
    )

    for x_sign in (-1.0, 1.0):
        for z_sign in (-1.0, 1.0):
            fork.visual(
                Cylinder(radius=0.0038, length=0.022),
                origin=Origin(
                    xyz=(0.021 * x_sign, 0.113, 0.134 + 0.019 * z_sign),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=dark_alloy,
                name=f"faceplate_bolt_{'r' if x_sign > 0 else 'l'}_{'u' if z_sign > 0 else 'd'}",
            )

    handlebar_path = [
        (-0.330, 0.050, -0.132),
        (-0.312, 0.038, -0.086),
        (-0.284, 0.050, -0.028),
        (-0.234, 0.082, 0.008),
        (-0.170, 0.060, 0.018),
        (-0.090, 0.016, 0.004),
        (-0.040, 0.000, 0.000),
        (0.040, 0.000, 0.000),
        (0.090, 0.016, 0.004),
        (0.170, 0.060, 0.018),
        (0.234, 0.082, 0.008),
        (0.284, 0.050, -0.028),
        (0.312, 0.038, -0.086),
        (0.330, 0.050, -0.132),
    ]
    bar_mount_origin = Origin(xyz=(0.0, 0.096, 0.134))
    fork.visual(
        _save_mesh(
            "gravel_handlebar_tube",
            tube_from_spline_points(
                handlebar_path,
                radius=0.0115,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        origin=bar_mount_origin,
        material=carbon,
        name="bar_tube",
    )
    fork.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.330, 0.146, 0.002)),
        material=hood_black,
        name="left_end_plug",
    )
    fork.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.330, 0.146, 0.002)),
        material=hood_black,
        name="right_end_plug",
    )
    fork.visual(
        Box((0.038, 0.030, 0.082)),
        origin=Origin(xyz=(-0.232, 0.175, 0.122), rpy=(-0.34, 0.0, 0.12)),
        material=hood_black,
        name="left_hood",
    )
    fork.visual(
        Box((0.038, 0.030, 0.082)),
        origin=Origin(xyz=(0.232, 0.175, 0.122), rpy=(-0.34, 0.0, -0.12)),
        material=hood_black,
        name="right_hood",
    )

    model.articulation(
        "steer",
        ArticulationType.REVOLUTE,
        parent=frame_stub,
        child=fork,
        origin=Origin(rpy=(HEAD_TILT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=3.0,
            lower=-0.60,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_stub = object_model.get_part("frame_stub")
    fork = object_model.get_part("fork")
    steer = object_model.get_articulation("steer")

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
        frame_stub,
        fork,
        elem_a="lower_headset",
        elem_b="crown_race",
        reason="The headset bearing seat is represented as simplified nested rings at the fork crown.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(frame_stub, fork, axes="xy", elem_a="lower_headset", elem_b="crown_race", min_overlap=0.030)
    ctx.expect_overlap(frame_stub, fork, axes="xy", elem_a="head_tube_shell", elem_b="steerer_tube", min_overlap=0.035)
    ctx.expect_overlap(fork, fork, axes="xy", elem_a="steerer_clamp", elem_b="steerer_tube", min_overlap=0.028, name="steerer_clamp_wraps_steerer")
    ctx.expect_overlap(fork, fork, axes="xz", elem_a="bar_clamp", elem_b="bar_tube", min_overlap=0.030, name="bar_clamp_wraps_handlebar")

    limits = steer.motion_limits
    ctx.check(
        "steering_joint_limits_are_gravel_like",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower <= -0.5 and limits.upper >= 0.5,
        f"Unexpected steering limits: {limits}",
    )
    ctx.check(
        "steering_axis_is_head_tube_tilted",
        abs(steer.origin.rpy[0] - HEAD_TILT) < 1e-6,
        f"Expected head-tube tilt {HEAD_TILT}, got {steer.origin.rpy}",
    )

    def _aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    left_rest_aabb = ctx.part_element_world_aabb(fork, elem="left_end_plug")
    assert left_rest_aabb is not None
    left_rest = _aabb_center(left_rest_aabb)

    with ctx.pose({steer: 0.45}):
        left_turned_aabb = ctx.part_element_world_aabb(fork, elem="left_end_plug")
        assert left_turned_aabb is not None
        left_turned = _aabb_center(left_turned_aabb)
        ctx.check(
            "steering_turns_left_bar_end_back",
            left_turned[1] < left_rest[1] - 0.05,
            f"Expected left bar end to sweep rearward under positive steer: rest={left_rest}, turned={left_turned}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_left_lock")

    with ctx.pose({steer: -0.45}):
        left_turned_aabb = ctx.part_element_world_aabb(fork, elem="left_end_plug")
        assert left_turned_aabb is not None
        left_turned = _aabb_center(left_turned_aabb)
        ctx.check(
            "steering_turns_left_bar_end_forward",
            left_turned[1] > left_rest[1] + 0.05,
            f"Expected left bar end to sweep forward under negative steer: rest={left_rest}, turned={left_turned}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_right_lock")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

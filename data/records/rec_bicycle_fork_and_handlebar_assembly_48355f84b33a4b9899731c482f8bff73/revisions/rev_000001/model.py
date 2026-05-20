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


def _rounded_section(
    width: float,
    depth: float,
    z: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    radius = min(width, depth) * 0.22
    return [
        (center[0] + x, center[1] + y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 56,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_fork_stem_and_bars")

    fork_black = model.material("fork_black", rgba=(0.10, 0.10, 0.11, 1.0))
    stem_alloy = model.material("stem_alloy", rgba=(0.24, 0.25, 0.28, 1.0))
    bar_chrome = model.material("bar_chrome", rgba=(0.76, 0.77, 0.80, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.14, 0.14, 0.15, 1.0))

    steerer_radius = 0.014
    head_tube_outer = 0.029
    headset_inner = 0.017
    head_tube_height = 0.120
    bar_clamp_z = 0.185
    bar_clamp_y = 0.052

    head_tube = model.part("head_tube")
    head_tube.visual(
        _shell_mesh(
            "head_tube_shell",
            outer_radius=head_tube_outer,
            inner_radius=headset_inner,
            z0=0.0,
            z1=head_tube_height,
        ),
        material=fork_black,
        name="head_tube_shell",
    )
    head_tube.inertial = Inertial.from_geometry(
        Box((0.064, 0.064, head_tube_height)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, head_tube_height * 0.5)),
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=steerer_radius, length=0.186),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=fork_black,
        name="steerer",
    )
    fork.visual(
        _shell_mesh(
            "crown_race_ring",
            outer_radius=head_tube_outer,
            inner_radius=headset_inner,
            z0=-0.008,
            z1=0.0,
        ),
        material=fork_black,
        name="crown_race",
    )
    fork.visual(
        Box((0.050, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=fork_black,
        name="crown_center",
    )
    fork.visual(
        Box((0.038, 0.030, 0.018)),
        origin=Origin(xyz=(-0.034, 0.0, -0.013)),
        material=fork_black,
        name="left_crown_wing",
    )
    fork.visual(
        Box((0.038, 0.030, 0.018)),
        origin=Origin(xyz=(0.034, 0.0, -0.013)),
        material=fork_black,
        name="right_crown_wing",
    )

    blade_top = _rounded_section(0.020, 0.030, -0.004, corner_segments=6)
    blade_mid = _rounded_section(0.017, 0.025, -0.165, center=(0.0, 0.008), corner_segments=6)
    blade_bottom = _rounded_section(0.014, 0.020, -0.325, center=(0.0, 0.016), corner_segments=6)
    blade_geom = section_loft([blade_top, blade_mid, blade_bottom])
    fork.visual(
        mesh_from_geometry(blade_geom, "left_fork_blade"),
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        material=fork_black,
        name="left_blade",
    )
    fork.visual(
        mesh_from_geometry(blade_geom, "right_fork_blade"),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=fork_black,
        name="right_blade",
    )
    fork.visual(
        Box((0.022, 0.008, 0.028)),
        origin=Origin(xyz=(-0.052, 0.020, -0.337)),
        material=hardware_dark,
        name="left_dropout",
    )
    fork.visual(
        Box((0.022, 0.008, 0.028)),
        origin=Origin(xyz=(0.052, 0.020, -0.337)),
        material=hardware_dark,
        name="right_dropout",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.130, 0.090, 0.530)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.010, -0.070)),
    )

    fork.visual(
        Box((0.010, 0.046, 0.044)),
        origin=Origin(xyz=(-0.019, 0.0, 0.147)),
        material=stem_alloy,
        name="left_steerer_cheek",
    )
    fork.visual(
        Box((0.010, 0.046, 0.044)),
        origin=Origin(xyz=(0.019, 0.0, 0.147)),
        material=stem_alloy,
        name="right_steerer_cheek",
    )
    fork.visual(
        Box((0.048, 0.010, 0.044)),
        origin=Origin(xyz=(0.0, -0.019, 0.147)),
        material=stem_alloy,
        name="rear_bridge",
    )
    fork.visual(
        Box((0.036, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, 0.037, 0.167)),
        material=stem_alloy,
        name="stem_body",
    )
    stem_spine_sections = [
        _rounded_section(0.050, 0.022, 0.142, center=(0.0, 0.010), corner_segments=5),
        _rounded_section(0.044, 0.028, 0.168, center=(0.0, 0.028), corner_segments=5),
        _rounded_section(0.038, 0.024, 0.194, center=(0.0, 0.044), corner_segments=5),
    ]
    fork.visual(
        mesh_from_geometry(section_loft(stem_spine_sections), "stem_spine_fairing_r1"),
        material=stem_alloy,
        name="stem_spine",
    )
    fork.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, bar_clamp_y, 0.1998)),
        material=stem_alloy,
        name="top_saddle",
    )
    fork.visual(
        Box((0.050, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, bar_clamp_y, 0.1702)),
        material=stem_alloy,
        name="bottom_saddle",
    )
    fork.visual(
        Box((0.050, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.064, 0.1998)),
        material=stem_alloy,
        name="upper_face_bridge",
    )
    fork.visual(
        Box((0.050, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.060, 0.1702)),
        material=stem_alloy,
        name="lower_face_bridge",
    )
    fork.visual(
        Box((0.050, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, 0.073, bar_clamp_z)),
        material=stem_alloy,
        name="front_faceplate",
    )
    fork.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _rounded_section(0.058, 0.012, 0.170, center=(0.0, 0.060), corner_segments=5),
                    _rounded_section(0.060, 0.010, 0.185, center=(0.0, 0.072), corner_segments=5),
                    _rounded_section(0.058, 0.012, 0.200, center=(0.0, 0.060), corner_segments=5),
                ]
            ),
            "stem_faceplate_body_r1",
        ),
        material=stem_alloy,
        name="faceplate_body",
    )
    fork.visual(
        Cylinder(radius=0.003, length=0.048),
        origin=Origin(xyz=(0.0, -0.022, 0.146), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_dark,
        name="steerer_pinch_bolt",
    )
    fork.visual(
        Cylinder(radius=0.003, length=0.066),
        origin=Origin(xyz=(0.0, 0.073, bar_clamp_z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_dark,
        name="bar_clamp_bolt",
    )

    main_bar_points = [
        (-0.380, -0.052, 0.060),
        (-0.300, -0.047, 0.060),
        (-0.225, -0.038, 0.050),
        (-0.155, -0.030, 0.038),
        (-0.120, -0.024, 0.024),
        (-0.080, -0.010, 0.010),
        (-0.060, 0.000, 0.000),
        (-0.030, 0.000, 0.000),
        (0.030, 0.000, 0.000),
        (0.060, 0.000, 0.000),
        (0.080, -0.010, 0.010),
        (0.120, -0.024, 0.024),
        (0.155, -0.030, 0.038),
        (0.225, -0.038, 0.050),
        (0.300, -0.047, 0.060),
        (0.380, -0.052, 0.060),
    ]
    crossbar_points = [
        (-0.120, -0.024, 0.024),
        (0.120, -0.024, 0.024),
    ]
    fork.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                main_bar_points,
                radius=0.011,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "fork_main_bar_r3",
        ),
        origin=Origin(xyz=(0.0, bar_clamp_y, bar_clamp_z)),
        material=bar_chrome,
        name="main_bar",
    )
    fork.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                crossbar_points,
                radius=0.009,
                samples_per_segment=2,
                radial_segments=16,
                cap_ends=True,
            ),
            "fork_crossbar_r3",
        ),
        origin=Origin(xyz=(0.0, bar_clamp_y, bar_clamp_z)),
        material=bar_chrome,
        name="crossbar",
    )
    fork.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(-0.338, 0.003, 0.245), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_dark,
        name="left_grip",
    )
    fork.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(0.338, 0.003, 0.245), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware_dark,
        name="right_grip",
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-math.radians(80.0),
            upper=math.radians(80.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("fork")
    steering = object_model.get_articulation("steering")
    fork.get_visual("main_bar")
    fork.get_visual("crossbar")
    fork.get_visual("left_blade")
    fork.get_visual("right_blade")
    bar_clamp_z = 0.185

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        head_tube,
        fork,
        reason="Fork steerer and crown race intentionally pass through the head tube headset.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        head_tube,
        fork,
        axis="z",
        positive_elem="head_tube_shell",
        negative_elem="crown_race",
        max_gap=0.0005,
        max_penetration=0.0,
        name="lower_headset_seat",
    )
    ctx.expect_overlap(
        fork,
        head_tube,
        axes="xy",
        elem_a="crown_race",
        elem_b="head_tube_shell",
        min_overlap=0.050,
        name="headset_coaxial_overlap",
    )
    ctx.expect_within(
        fork,
        head_tube,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube_shell",
        margin=0.0,
        name="steerer_within_head_tube",
    )
    ctx.expect_contact(
        fork,
        fork,
        elem_a="left_steerer_cheek",
        elem_b="steerer",
        contact_tol=0.0006,
        name="left_stem_cheek_contacts_steerer",
    )
    ctx.expect_contact(
        fork,
        fork,
        elem_a="right_steerer_cheek",
        elem_b="steerer",
        contact_tol=0.0006,
        name="right_stem_cheek_contacts_steerer",
    )

    steering_limits = steering.motion_limits
    if steering_limits is not None and steering_limits.lower is not None and steering_limits.upper is not None:
        with ctx.pose({steering: steering_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_lower_no_floating")
            ctx.expect_within(
                fork,
                head_tube,
                axes="xy",
                inner_elem="steerer",
                outer_elem="head_tube_shell",
                margin=0.0,
                name="steering_lower_steerer_within_head_tube",
            )
        with ctx.pose({steering: steering_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_upper_no_floating")
            ctx.expect_gap(
                head_tube,
                fork,
                axis="z",
                positive_elem="head_tube_shell",
                negative_elem="crown_race",
                max_gap=0.0005,
                max_penetration=0.0,
                name="steering_upper_lower_headset_contact",
            )

    main_bar_rest_aabb = ctx.part_element_world_aabb(fork, elem="main_bar")
    assert main_bar_rest_aabb is not None
    with ctx.pose({steering: math.radians(55.0)}):
        main_bar_turned_aabb = ctx.part_element_world_aabb(fork, elem="main_bar")
        assert main_bar_turned_aabb is not None
        assert main_bar_turned_aabb[1][1] > main_bar_rest_aabb[1][1] + 0.24

    crossbar_rest_aabb = ctx.part_element_world_aabb(fork, elem="crossbar")
    assert crossbar_rest_aabb is not None
    crossbar_rest_center_z = (crossbar_rest_aabb[0][2] + crossbar_rest_aabb[1][2]) * 0.5
    ctx.check(
        "crossbar_above_stem_clamp",
        crossbar_rest_center_z > bar_clamp_z + 0.020,
        details=f"crossbar_center_z={crossbar_rest_center_z:.4f}",
    )
    ctx.check(
        "crossbar_span_width",
        (crossbar_rest_aabb[1][0] - crossbar_rest_aabb[0][0]) > 0.22,
        details=f"crossbar_span={(crossbar_rest_aabb[1][0] - crossbar_rest_aabb[0][0]):.4f}",
    )
    ctx.check(
        "bar_wider_than_crown",
        (main_bar_rest_aabb[1][0] - main_bar_rest_aabb[0][0]) > 0.70,
        details=f"bar_width={(main_bar_rest_aabb[1][0] - main_bar_rest_aabb[0][0]):.4f}",
    )
    assert crossbar_rest_center_z > bar_clamp_z + 0.020
    assert crossbar_rest_aabb[1][0] - crossbar_rest_aabb[0][0] > 0.22

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

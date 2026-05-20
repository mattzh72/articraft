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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_bell_shell_mesh():
    outer_profile = [
        (0.30, -1.20),
        (0.29, -1.14),
        (0.27, -1.01),
        (0.24, -0.84),
        (0.20, -0.65),
        (0.16, -0.48),
        (0.12, -0.38),
        (0.09, -0.34),
    ]
    inner_profile = [
        (0.24, -1.15),
        (0.23, -1.09),
        (0.22, -0.98),
        (0.19, -0.82),
        (0.16, -0.67),
        (0.12, -0.52),
        (0.08, -0.40),
        (0.03, -0.34),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_yoke_arch_mesh():
    return sweep_profile_along_spline(
        [
            (-0.10, 0.0, -0.05),
            (-0.07, 0.0, -0.10),
            (0.0, 0.0, -0.16),
            (0.07, 0.0, -0.10),
            (0.10, 0.0, -0.05),
        ],
        profile=rounded_rect_profile(0.04, 0.08, radius=0.010, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
    )


def _circle_profile(radius: float, segments: int = 24):
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_hanger_cheek_mesh():
    cheek = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.03, 0.28, radius=0.006, corner_segments=6),
        [_circle_profile(0.014, segments=32)],
        height=0.012,
        center=True,
    )
    cheek.rotate_x(math.pi / 2.0)
    return cheek


def _aabb_center(aabb, axis: int) -> float:
    return 0.5 * (aabb[0][axis] + aabb[1][axis])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_frame_bell_cote")

    stone = model.material("stone", rgba=(0.67, 0.67, 0.64, 1.0))
    weathered_oak = model.material("weathered_oak", rgba=(0.42, 0.31, 0.20, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.30, 0.20, 0.12, 1.0))
    wrought_iron = model.material("wrought_iron", rgba=(0.18, 0.18, 0.20, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.63, 0.43, 0.18, 1.0))

    frame = model.part("frame")
    post_run_x = 1.50
    post_run_z = 3.25
    post_length = math.sqrt(post_run_x**2 + post_run_z**2)
    post_angle = math.atan2(post_run_x, post_run_z)

    frame.visual(
        Box((3.60, 1.30, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=stone,
        name="plinth",
    )
    frame.visual(
        Box((2.95, 0.28, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=weathered_oak,
        name="tie_beam",
    )
    frame.visual(
        Box((0.20, 0.24, post_length)),
        origin=Origin(xyz=(-0.75, 0.0, 1.975), rpy=(0.0, post_angle, 0.0)),
        material=weathered_oak,
        name="left_post",
    )
    frame.visual(
        Box((0.20, 0.24, post_length)),
        origin=Origin(xyz=(0.75, 0.0, 1.975), rpy=(0.0, -post_angle, 0.0)),
        material=weathered_oak,
        name="right_post",
    )
    frame.visual(
        Box((0.24, 1.02, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 3.60)),
        material=weathered_oak,
        name="ridge_beam",
    )
    frame.visual(
        Box((0.16, 0.10, 0.42)),
        origin=Origin(xyz=(0.0, 0.24, 3.285)),
        material=wrought_iron,
        name="front_bearing_block",
    )
    frame.visual(
        Box((0.16, 0.10, 0.42)),
        origin=Origin(xyz=(0.0, -0.24, 3.285)),
        material=wrought_iron,
        name="rear_bearing_block",
    )
    frame.inertial = Inertial.from_geometry(
        Box((3.60, 1.30, 3.82)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.91)),
    )

    bell = model.part("bell")
    bell.visual(
        Box((0.14, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=dark_oak,
        name="headstock",
    )
    bell.visual(
        _save_mesh("bell_cote_yoke_arch_v3", _build_yoke_arch_mesh()),
        material=dark_oak,
        name="yoke_arch",
    )
    bell.visual(
        Cylinder(radius=0.03, length=0.38),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="pivot_axle",
    )
    bell.visual(
        Box((0.08, 0.12, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
        material=dark_oak,
        name="crown_block",
    )
    bell.visual(
        _save_mesh("bell_cote_bell_shell_v3", _build_bell_shell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=bell_bronze,
        name="bell_shell",
    )
    bell.visual(
        _save_mesh("bell_cote_hanger_cheek_v3", _build_hanger_cheek_mesh()),
        origin=Origin(xyz=(0.0, 0.056, -0.40)),
        material=wrought_iron,
        name="hanger_cheek_front",
    )
    bell.visual(
        _save_mesh("bell_cote_hanger_cheek_v4", _build_hanger_cheek_mesh()),
        origin=Origin(xyz=(0.0, -0.056, -0.40)),
        material=wrought_iron,
        name="hanger_cheek_rear",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.32, length=1.10),
        mass=130.0,
        origin=Origin(xyz=(0.0, 0.0, -0.60)),
    )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.008, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="clapper_pin",
    )
    clapper.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="retaining_washer_front",
    )
    clapper.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="retaining_washer_rear",
    )
    clapper.visual(
        Cylinder(radius=0.016, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=wrought_iron,
        name="upper_shank",
    )
    clapper.visual(
        Cylinder(radius=0.020, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
        material=wrought_iron,
        name="lower_shank",
    )
    clapper.visual(
        Sphere(radius=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
        material=wrought_iron,
        name="clapper_bulb",
    )
    clapper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.82),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.41)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 3.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.8,
            lower=-0.20,
            upper=0.20,
        ),
    )
    model.articulation(
        "clapper_pivot",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=2.0,
            lower=-0.24,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    bell_swing = object_model.get_articulation("bell_swing")
    clapper_pivot = object_model.get_articulation("clapper_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        frame,
        bell,
        elem_b="pivot_axle",
        contact_tol=0.001,
        name="bell_axle_bears_on_frame",
    )
    ctx.expect_gap(
        bell,
        frame,
        axis="z",
        negative_elem="plinth",
        min_gap=1.40,
        name="bell_hangs_above_plinth",
    )
    ctx.expect_gap(
        clapper,
        frame,
        axis="z",
        negative_elem="plinth",
        min_gap=1.20,
        name="clapper_hangs_above_plinth",
    )
    ctx.expect_overlap(
        bell,
        frame,
        axes="xy",
        min_overlap=0.45,
        name="bell_within_cote_plan",
    )
    ctx.expect_within(
        clapper,
        bell,
        axes="xy",
        outer_elem="bell_shell",
        margin=0.04,
        name="clapper_within_bell_at_rest",
    )
    ctx.expect_contact(
        bell,
        clapper,
        contact_tol=1e-6,
        name="clapper_supported_by_hanger",
    )
    ctx.expect_origin_distance(
        bell,
        clapper,
        axes="xy",
        max_dist=0.001,
        name="clapper_hangs_from_bell_centerline",
    )

    bell_shell_rest = ctx.part_element_world_aabb(bell, elem="bell_shell")
    clapper_bulb_rest = ctx.part_element_world_aabb(clapper, elem="clapper_bulb")
    assert bell_shell_rest is not None
    assert clapper_bulb_rest is not None

    bell_limits = bell_swing.motion_limits
    assert bell_limits is not None and bell_limits.lower is not None and bell_limits.upper is not None
    with ctx.pose({bell_swing: bell_limits.lower}):
        bell_shell_lower = ctx.part_element_world_aabb(bell, elem="bell_shell")
        assert bell_shell_lower is not None
        ctx.check(
            "bell_lower_swings_toward_positive_x",
            _aabb_center(bell_shell_lower, 0) > _aabb_center(bell_shell_rest, 0) + 0.05,
            "Bell shell did not shift toward +X at lower bell-swing limit.",
        )
        ctx.expect_contact(
            frame,
            bell,
            elem_b="pivot_axle",
            contact_tol=0.001,
            name="bell_lower_pivot_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="bell_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="bell_lower_no_floating")

    with ctx.pose({bell_swing: bell_limits.upper}):
        bell_shell_upper = ctx.part_element_world_aabb(bell, elem="bell_shell")
        assert bell_shell_upper is not None
        ctx.check(
            "bell_upper_swings_toward_negative_x",
            _aabb_center(bell_shell_upper, 0) < _aabb_center(bell_shell_rest, 0) - 0.05,
            "Bell shell did not shift toward -X at upper bell-swing limit.",
        )
        ctx.expect_contact(
            frame,
            bell,
            elem_b="pivot_axle",
            contact_tol=0.001,
            name="bell_upper_pivot_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="bell_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="bell_upper_no_floating")

    clapper_limits = clapper_pivot.motion_limits
    assert (
        clapper_limits is not None
        and clapper_limits.lower is not None
        and clapper_limits.upper is not None
    )
    with ctx.pose({clapper_pivot: clapper_limits.lower}):
        clapper_bulb_lower = ctx.part_element_world_aabb(clapper, elem="clapper_bulb")
        assert clapper_bulb_lower is not None
        ctx.check(
            "clapper_lower_swings_toward_positive_x",
            _aabb_center(clapper_bulb_lower, 0) > _aabb_center(clapper_bulb_rest, 0) + 0.08,
            "Clapper bulb did not shift toward +X at lower clapper limit.",
        )
        ctx.expect_within(
            clapper,
            bell,
            axes="xy",
            outer_elem="bell_shell",
            margin=0.08,
            name="clapper_lower_within_bell",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="clapper_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="clapper_lower_no_floating")

    with ctx.pose({clapper_pivot: clapper_limits.upper}):
        clapper_bulb_upper = ctx.part_element_world_aabb(clapper, elem="clapper_bulb")
        assert clapper_bulb_upper is not None
        ctx.check(
            "clapper_upper_swings_toward_negative_x",
            _aabb_center(clapper_bulb_upper, 0) < _aabb_center(clapper_bulb_rest, 0) - 0.08,
            "Clapper bulb did not shift toward -X at upper clapper limit.",
        )
        ctx.expect_within(
            clapper,
            bell,
            axes="xy",
            outer_elem="bell_shell",
            margin=0.08,
            name="clapper_upper_within_bell",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="clapper_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="clapper_upper_no_floating")

    with ctx.pose({bell_swing: bell_limits.upper, clapper_pivot: clapper_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _tube_mesh(name: str, points: list[tuple[float, float, float]], *, radius: float, samples: int = 18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _shell_tube(name: str, *, outer_radius: float, inner_radius: float, length: float):
    half = length * 0.5
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
        name,
    )


def _wheel_visual(
    part,
    *,
    name_prefix: str,
    center: tuple[float, float, float],
    radius: float,
    width: float,
    tire_material,
    hub_material,
) -> None:
    x, y, z = center
    spin_origin = Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=tire_material,
        name=f"{name_prefix}_tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.42, length=width * 1.12),
        origin=spin_origin,
        material=hub_material,
        name=f"{name_prefix}_hub",
    )
    part.visual(
        Cylinder(radius=radius * 0.18, length=width * 1.22),
        origin=spin_origin,
        material=hub_material,
        name=f"{name_prefix}_axle_cap",
    )


def _aabb_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][1] + aabb[1][1]) * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    powder_black = model.material("powder_black", rgba=(0.20, 0.21, 0.22, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.11, 0.11, 0.12, 1.0))
    foam_gray = model.material("foam_gray", rgba=(0.19, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.76, 0.34, 0.60)),
        mass=8.5,
        origin=Origin(xyz=(0.02, 0.0, 0.30)),
    )
    frame.visual(
        _tube_mesh(
            "frame_main_spine",
            [(-0.16, 0.0, 0.18), (-0.04, 0.0, 0.24), (0.10, 0.0, 0.30), (0.24, 0.0, 0.33)],
            radius=0.022,
        ),
        material=powder_black,
        name="main_spine",
    )
    frame.visual(
        _tube_mesh(
            "frame_deck_post",
            [(-0.05, 0.0, 0.246), (-0.08, 0.0, 0.38), (-0.062, 0.0, 0.4745)],
            radius=0.018,
        ),
        material=powder_black,
        name="deck_post",
    )
    rear_stay_points = [(-0.11, 0.0, 0.21), (-0.17, 0.032, 0.18), (-0.225, 0.094, 0.142)]
    frame.visual(
        _tube_mesh("frame_left_rear_stay", rear_stay_points, radius=0.017, samples=14),
        material=powder_black,
        name="left_rear_stay",
    )
    frame.visual(
        _tube_mesh("frame_right_rear_stay", _mirror_y(rear_stay_points), radius=0.017, samples=14),
        material=powder_black,
        name="right_rear_stay",
    )
    frame.visual(
        Box((0.09, 0.13, 0.038)),
        origin=Origin(xyz=(-0.17, 0.0, 0.150)),
        material=powder_black,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.050, 0.022, 0.050)),
        origin=Origin(xyz=(0.232, 0.033, 0.346)),
        material=powder_black,
        name="left_head_gusset",
    )
    frame.visual(
        Box((0.050, 0.022, 0.050)),
        origin=Origin(xyz=(0.232, -0.033, 0.346)),
        material=powder_black,
        name="right_head_gusset",
    )
    frame.visual(
        _tube_mesh(
            "frame_left_head_brace",
            [(0.22, 0.028, 0.33), (0.262, 0.032, 0.365), (0.298, 0.035, 0.396)],
            radius=0.013,
            samples=8,
        ),
        material=powder_black,
        name="left_head_brace",
    )
    frame.visual(
        _tube_mesh(
            "frame_right_head_brace",
            _mirror_y([(0.22, 0.028, 0.33), (0.262, 0.032, 0.365), (0.298, 0.035, 0.396)]),
            radius=0.013,
            samples=8,
        ),
        material=powder_black,
        name="right_head_brace",
    )
    frame.visual(
        _shell_tube("frame_head_tube", outer_radius=0.035, inner_radius=0.026, length=0.16),
        origin=Origin(xyz=(0.30, 0.0, 0.44)),
        material=powder_black,
        name="head_tube",
    )
    frame.visual(
        Box((0.010, 0.016, 0.072)),
        origin=Origin(xyz=(-0.236, 0.108, 0.10)),
        material=satin_silver,
        name="left_rear_plate",
    )
    frame.visual(
        Box((0.018, 0.020, 0.022)),
        origin=Origin(xyz=(-0.230, 0.106, 0.136)),
        material=powder_black,
        name="left_dropout_gusset",
    )
    frame.visual(
        Box((0.010, 0.016, 0.072)),
        origin=Origin(xyz=(-0.236, -0.108, 0.10)),
        material=satin_silver,
        name="right_rear_plate",
    )
    frame.visual(
        Box((0.018, 0.020, 0.022)),
        origin=Origin(xyz=(-0.230, -0.106, 0.136)),
        material=powder_black,
        name="right_dropout_gusset",
    )
    frame.visual(
        Box((0.22, 0.12, 0.012)),
        origin=Origin(xyz=(-0.06, 0.0, 0.484)),
        material=satin_silver,
        name="deck_support_plate",
    )

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.36, 0.20, 0.09)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )
    deck.visual(
        Box((0.30, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_silver,
        name="mount_plate",
    )
    deck.visual(
        Box((0.30, 0.16, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=powder_black,
        name="pad_base",
    )
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.34, 0.18, 0.035, corner_segments=10),
                0.055,
                center=True,
            ),
            "knee_pad_cushion",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0535)),
        material=cushion_black,
        name="cushion",
    )

    rear_axle = model.part("rear_axle")
    rear_axle.inertial = Inertial.from_geometry(
        Box((0.08, 0.39, 0.22)),
        mass=2.6,
        origin=Origin(),
    )
    rear_axle.visual(
        Cylinder(radius=0.009, length=0.39),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="axle_rod",
    )
    rear_axle.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="left_bushing",
    )
    rear_axle.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="right_bushing",
    )
    _wheel_visual(
        rear_axle,
        name_prefix="left_rear_wheel",
        center=(0.0, 0.152, 0.0),
        radius=0.102,
        width=0.036,
        tire_material=dark_rubber,
        hub_material=satin_silver,
    )
    _wheel_visual(
        rear_axle,
        name_prefix="right_rear_wheel",
        center=(0.0, -0.152, 0.0),
        radius=0.102,
        width=0.036,
        tire_material=dark_rubber,
        hub_material=satin_silver,
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.20, 0.26, 0.86)),
        mass=3.8,
        origin=Origin(xyz=(0.04, 0.0, 0.04)),
    )
    front_fork.visual(
        Cylinder(radius=0.020, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=satin_silver,
        name="steerer_core",
    )
    front_fork.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=satin_silver,
        name="upper_headset",
    )
    front_fork.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=satin_silver,
        name="lower_headset",
    )
    front_fork.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=satin_silver,
        name="sleeve_bridge",
    )
    front_fork.visual(
        _shell_tube("front_fork_upper_sleeve", outer_radius=0.028, inner_radius=0.021, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=satin_silver,
        name="upper_sleeve",
    )
    front_fork.visual(
        _shell_tube("front_fork_top_collar", outer_radius=0.033, inner_radius=0.0205, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.347)),
        material=powder_black,
        name="top_collar",
    )
    front_fork.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.034, 0.0, 0.312), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_black,
        name="clamp_stem",
    )
    front_fork.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.050, 0.0, 0.312)),
        material=powder_black,
        name="clamp_knob",
    )
    front_fork.visual(
        Box((0.08, 0.14, 0.045)),
        origin=Origin(xyz=(0.012, 0.0, -0.12)),
        material=powder_black,
        name="fork_crown",
    )
    fork_blade_points = [(0.005, 0.055, -0.11), (0.03, 0.065, -0.22), (0.078, 0.080, -0.34)]
    front_fork.visual(
        _tube_mesh("front_fork_left_blade", fork_blade_points, radius=0.014, samples=14),
        material=powder_black,
        name="left_blade",
    )
    front_fork.visual(
        _tube_mesh("front_fork_right_blade", _mirror_y(fork_blade_points), radius=0.014, samples=14),
        material=powder_black,
        name="right_blade",
    )
    front_fork.visual(
        Cylinder(radius=0.010, length=0.24),
        origin=Origin(xyz=(0.078, 0.0, -0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="front_axle",
    )
    _wheel_visual(
        front_fork,
        name_prefix="left_front_wheel",
        center=(0.078, 0.096, -0.34),
        radius=0.100,
        width=0.040,
        tire_material=dark_rubber,
        hub_material=satin_silver,
    )
    _wheel_visual(
        front_fork,
        name_prefix="right_front_wheel",
        center=(0.078, -0.096, -0.34),
        radius=0.100,
        width=0.040,
        tire_material=dark_rubber,
        hub_material=satin_silver,
    )

    handlebar_post = model.part("handlebar_post")
    handlebar_post.inertial = Inertial.from_geometry(
        Box((0.18, 0.52, 0.90)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )
    handlebar_post.visual(
        _shell_tube("handlebar_stop_collar", outer_radius=0.034, inner_radius=0.0195, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=powder_black,
        name="stop_collar",
    )
    handlebar_post.visual(
        Cylinder(radius=0.0195, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=powder_black,
        name="collar_core",
    )
    handlebar_post.visual(
        Cylinder(radius=0.018, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=satin_silver,
        name="inner_tube",
    )
    handlebar_post.visual(
        Box((0.050, 0.074, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.530)),
        material=powder_black,
        name="bar_clamp",
    )
    handlebar_post.visual(
        _tube_mesh(
            "handlebar_crossbar",
            [(0.0, -0.22, 0.50), (0.02, -0.10, 0.54), (0.0, 0.0, 0.56), (0.02, 0.10, 0.54), (0.0, 0.22, 0.50)],
            radius=0.014,
        ),
        material=satin_silver,
        name="crossbar",
    )
    handlebar_post.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.0, 0.235, 0.50), rpy=(pi / 2.0, 0.0, 0.0)),
        material=foam_gray,
        name="left_grip",
    )
    handlebar_post.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.0, -0.235, 0.50), rpy=(pi / 2.0, 0.0, 0.0)),
        material=foam_gray,
        name="right_grip",
    )

    model.articulation(
        "frame_to_deck",
        ArticulationType.FIXED,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(-0.06, 0.0, 0.49)),
    )
    model.articulation(
        "rear_axle_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_axle,
        origin=Origin(xyz=(-0.22, 0.0, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=16.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.30, 0.0, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "handlebar_raise",
        ArticulationType.PRISMATIC,
        parent=front_fork,
        child=handlebar_post,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    deck = object_model.get_part("deck")
    rear_axle = object_model.get_part("rear_axle")
    front_fork = object_model.get_part("front_fork")
    handlebar_post = object_model.get_part("handlebar_post")
    rear_axle_spin = object_model.get_articulation("rear_axle_spin")
    front_steer = object_model.get_articulation("front_steer")
    handlebar_raise = object_model.get_articulation("handlebar_raise")

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

    ctx.expect_contact(deck, frame, elem_a="mount_plate", elem_b="deck_support_plate", name="deck is mounted on the support plate")
    ctx.expect_contact(front_fork, frame, elem_a="upper_headset", elem_b="head_tube", name="front fork headset seats on head tube")
    ctx.expect_contact(rear_axle, frame, elem_a="left_bushing", elem_b="left_rear_plate", name="left rear bushing seats against dropout plate")
    ctx.expect_contact(rear_axle, frame, elem_a="right_bushing", elem_b="right_rear_plate", name="right rear bushing seats against dropout plate")
    ctx.expect_contact(handlebar_post, front_fork, elem_a="stop_collar", elem_b="top_collar", name="handlebar post rests on the stem collar at minimum height")

    ctx.check(
        "rear axle joint is a lateral revolute",
        rear_axle_spin.articulation_type == ArticulationType.REVOLUTE and tuple(rear_axle_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={rear_axle_spin.articulation_type}, axis={rear_axle_spin.axis}",
    )
    ctx.check(
        "front steering joint turns about vertical axis",
        front_steer.articulation_type == ArticulationType.REVOLUTE and tuple(front_steer.axis) == (0.0, 0.0, 1.0),
        details=f"type={front_steer.articulation_type}, axis={front_steer.axis}",
    )
    ctx.check(
        "handlebar post uses vertical prismatic travel",
        handlebar_raise.articulation_type == ArticulationType.PRISMATIC and tuple(handlebar_raise.axis) == (0.0, 0.0, 1.0),
        details=f"type={handlebar_raise.articulation_type}, axis={handlebar_raise.axis}",
    )

    ctx.expect_within(
        handlebar_post,
        front_fork,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="upper_sleeve",
        margin=0.012,
        name="handlebar post stays centered in the fork stem",
    )
    ctx.expect_overlap(
        handlebar_post,
        front_fork,
        axes="z",
        elem_a="inner_tube",
        elem_b="upper_sleeve",
        min_overlap=0.16,
        name="handlebar post has deep insertion at minimum height",
    )

    rest_post_position = ctx.part_world_position(handlebar_post)
    rest_axle_center_y = _aabb_center_y(ctx.part_element_world_aabb(front_fork, elem="front_axle"))
    with ctx.pose({handlebar_raise: 0.14}):
        ctx.expect_within(
            handlebar_post,
            front_fork,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="upper_sleeve",
            margin=0.012,
            name="raised handlebar post stays centered in the stem",
        )
        ctx.expect_overlap(
            handlebar_post,
            front_fork,
            axes="z",
            elem_a="inner_tube",
            elem_b="upper_sleeve",
            min_overlap=0.06,
            name="raised handlebar post retains insertion in the stem",
        )
        raised_post_position = ctx.part_world_position(handlebar_post)
    ctx.check(
        "handlebar post extends upward when raised",
        rest_post_position is not None
        and raised_post_position is not None
        and raised_post_position[2] > rest_post_position[2] + 0.10,
        details=f"rest={rest_post_position}, raised={raised_post_position}",
    )

    with ctx.pose({front_steer: 0.35}):
        steered_axle_center_y = _aabb_center_y(ctx.part_element_world_aabb(front_fork, elem="front_axle"))
    ctx.check(
        "front fork visibly yaws when steered",
        rest_axle_center_y is not None and steered_axle_center_y is not None and steered_axle_center_y > rest_axle_center_y + 0.015,
        details=f"rest_y={rest_axle_center_y}, steered_y={steered_axle_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

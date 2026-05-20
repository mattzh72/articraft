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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_knee_scooter")

    frame_blue = model.material("frame_blue", rgba=(0.18, 0.28, 0.44, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))

    wheel_radius = 0.105
    wheel_width = 0.040
    axle_length = 0.068
    track_half = 0.135
    rear_wheel_y = -0.170
    front_wheel_y = 0.310
    steer_origin = (0.0, 0.240, 0.310)
    fold_origin_local = (0.0, 0.0, 0.107)

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def _section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]

    def _add_wheel(part, mesh_name: str) -> None:
        half_width = wheel_width * 0.5
        tire_profile = [
            (wheel_radius * 0.60, -half_width * 0.98),
            (wheel_radius * 0.82, -half_width),
            (wheel_radius * 0.95, -half_width * 0.70),
            (wheel_radius, -half_width * 0.18),
            (wheel_radius, half_width * 0.18),
            (wheel_radius * 0.95, half_width * 0.70),
            (wheel_radius * 0.82, half_width),
            (wheel_radius * 0.60, half_width * 0.98),
            (wheel_radius * 0.52, half_width * 0.30),
            (wheel_radius * 0.50, 0.0),
            (wheel_radius * 0.52, -half_width * 0.30),
            (wheel_radius * 0.60, -half_width * 0.98),
        ]
        spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
        tire_mesh = _mesh(
            f"{mesh_name}_tire",
            LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0),
        )
        part.visual(tire_mesh, material=rubber, name="tire")
        part.visual(
            Cylinder(radius=wheel_radius * 0.72, length=wheel_width * 0.82),
            origin=spin_origin,
            material=aluminum,
            name="rim_body",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.77, length=wheel_width * 0.16),
            origin=Origin(xyz=(wheel_width * 0.21, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="rim_face_outer",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.77, length=wheel_width * 0.16),
            origin=Origin(xyz=(-wheel_width * 0.21, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="rim_face_inner",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.26, length=wheel_width * 0.96),
            origin=spin_origin,
            material=steel_dark,
            name="hub_core",
        )
        part.visual(
            Cylinder(radius=0.007, length=axle_length),
            origin=spin_origin,
            material=steel_dark,
            name="axle_core",
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.34, 0.72, 0.42)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.02, 0.24)),
    )

    deck_mesh = _mesh(
        "deck_shell",
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.180, 0.340, 0.028), 0.014),
    )
    frame.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, -0.020, 0.158)),
        material=deck_gray,
        name="deck_shell",
    )
    frame.visual(
        Box((0.148, 0.190, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, 0.174)),
        material=rubber,
        name="deck_grip",
    )
    frame.visual(
        Box((0.202, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, rear_wheel_y, wheel_radius)),
        material=steel_dark,
        name="rear_axle_beam",
    )
    rear_left_stay = tube_from_spline_points(
        [
            (0.052, -0.040, 0.166),
            (0.062, -0.095, 0.148),
            (0.074, rear_wheel_y, wheel_radius + 0.010),
        ],
        radius=0.012,
        samples_per_segment=10,
        radial_segments=14,
    )
    rear_right_stay = tube_from_spline_points(
        _mirror_x(
            [
                (0.052, -0.040, 0.166),
                (0.062, -0.095, 0.148),
                (0.074, rear_wheel_y, wheel_radius + 0.010),
            ]
        ),
        radius=0.012,
        samples_per_segment=10,
        radial_segments=14,
    )
    frame.visual(_mesh("rear_left_stay", rear_left_stay), material=frame_blue, name="rear_left_stay")
    frame.visual(_mesh("rear_right_stay", rear_right_stay), material=frame_blue, name="rear_right_stay")

    main_spine = tube_from_spline_points(
        [
            (0.0, -0.150, 0.135),
            (0.0, -0.030, 0.185),
            (0.0, 0.120, 0.235),
            (0.0, 0.145, 0.252),
        ],
        radius=0.026,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.visual(_mesh("main_spine", main_spine), material=frame_blue, name="main_spine")
    frame.visual(
        Box((0.056, 0.058, 0.052)),
        origin=Origin(xyz=(0.0, 0.160, 0.272)),
        material=frame_blue,
        name="neck_connector",
    )

    knee_post = tube_from_spline_points(
        [
            (0.0, -0.010, 0.178),
            (0.0, -0.028, 0.285),
            (0.0, -0.035, 0.385),
        ],
        radius=0.020,
        samples_per_segment=12,
        radial_segments=18,
    )
    frame.visual(_mesh("knee_post", knee_post), material=frame_blue, name="knee_post")
    frame.visual(
        Box((0.120, 0.120, 0.044)),
        origin=Origin(xyz=(0.0, -0.035, 0.404)),
        material=steel_dark,
        name="knee_pad_bracket",
    )
    knee_pad_mesh = _mesh(
        "knee_pad",
        section_loft(
            [
                _section(0.180, 0.110, 0.026, 0.0),
                _section(0.220, 0.145, 0.036, 0.028),
                _section(0.185, 0.125, 0.030, 0.058),
            ]
        ),
    )
    frame.visual(
        knee_pad_mesh,
        origin=Origin(xyz=(0.0, -0.035, 0.420)),
        material=pad_vinyl,
        name="knee_pad",
    )

    frame.visual(
        Box((0.070, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, 0.190, 0.285)),
        material=frame_blue,
        name="head_block",
    )

    steering_fork = model.part("steering_fork")
    steering_fork.inertial = Inertial.from_geometry(
        Box((0.26, 0.18, 0.56)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.010, -0.010)),
    )
    steering_fork.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=aluminum,
        name="upper_bearing",
    )
    steering_fork.visual(
        Cylinder(radius=0.016, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=aluminum,
        name="steering_column",
    )
    steering_fork.visual(
        Box((0.050, 0.032, 0.084)),
        origin=Origin(xyz=(0.0, 0.016, -0.070)),
        material=steel_dark,
        name="fork_spine",
    )
    steering_fork.visual(
        Box((0.170, 0.036, 0.026)),
        origin=Origin(xyz=(0.0, 0.032, -0.105)),
        material=steel_dark,
        name="fork_crown",
    )
    steering_fork.visual(
        Box((0.160, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, -0.180)),
        material=steel_dark,
        name="front_axle_beam",
    )
    steering_fork.visual(
        Box((0.024, 0.022, 0.090)),
        origin=Origin(xyz=(0.090, 0.050, -0.150)),
        material=frame_blue,
        name="fork_left_leg",
    )
    steering_fork.visual(
        Box((0.024, 0.022, 0.090)),
        origin=Origin(xyz=(-0.090, 0.050, -0.150)),
        material=frame_blue,
        name="fork_right_leg",
    )
    steering_fork.visual(
        Box((0.040, 0.022, 0.070)),
        origin=Origin(xyz=(0.0, 0.050, -0.160)),
        material=frame_blue,
        name="fork_center_brace",
    )
    steering_fork.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.095, front_wheel_y - steer_origin[1], wheel_radius - steer_origin[2])),
        material=steel_dark,
        name="left_dropout",
    )
    steering_fork.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(-0.095, front_wheel_y - steer_origin[1], wheel_radius - steer_origin[2])),
        material=steel_dark,
        name="right_dropout",
    )
    steering_fork.visual(
        Box((0.070, 0.056, 0.040)),
        origin=Origin(xyz=(0.0, -0.004, 0.062)),
        material=steel_dark,
        name="clamp_body",
    )
    steering_fork.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=aluminum,
        name="clamp_riser",
    )
    steering_fork.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, -0.002, 0.081)),
        material=aluminum,
        name="clamp_saddle",
    )
    steering_fork.visual(
        Box((0.014, 0.044, 0.046)),
        origin=Origin(xyz=(0.023, 0.0, 0.113)),
        material=aluminum,
        name="clamp_cheek_left",
    )
    steering_fork.visual(
        Box((0.014, 0.044, 0.046)),
        origin=Origin(xyz=(-0.023, 0.0, 0.113)),
        material=aluminum,
        name="clamp_cheek_right",
    )

    upper_stem = model.part("upper_stem")
    upper_stem.inertial = Inertial.from_geometry(
        Box((0.52, 0.16, 0.62)),
        mass=1.9,
        origin=Origin(xyz=(0.0, -0.050, 0.300)),
    )
    upper_stem.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    upper_stem.visual(
        Box((0.028, 0.036, 0.022)),
        origin=Origin(xyz=(0.0, 0.002, -0.006)),
        material=aluminum,
        name="hinge_tongue",
    )
    upper_stem.visual(
        Box((0.026, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.010, 0.030)),
        material=steel_dark,
        name="stem_sleeve",
    )
    stem_tube = tube_from_spline_points(
        [
            (0.0, -0.012, 0.020),
            (0.0, -0.022, 0.180),
            (0.0, -0.050, 0.370),
            (0.0, -0.090, 0.560),
        ],
        radius=0.016,
        samples_per_segment=18,
        radial_segments=18,
    )
    upper_stem.visual(_mesh("upper_stem_tube", stem_tube), material=frame_blue, name="stem_tube")
    handlebar = tube_from_spline_points(
        [
            (-0.230, -0.090, 0.490),
            (-0.160, -0.096, 0.545),
            (-0.060, -0.102, 0.565),
            (0.060, -0.102, 0.565),
            (0.160, -0.096, 0.545),
            (0.230, -0.090, 0.490),
        ],
        radius=0.013,
        samples_per_segment=16,
        radial_segments=16,
    )
    upper_stem.visual(_mesh("handlebar", handlebar), material=frame_blue, name="handlebar")
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.255, -0.090, 0.490), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(-0.255, -0.090, 0.490), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(rear_left_wheel, "rear_left_wheel")

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(rear_right_wheel, "rear_right_wheel")

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(front_left_wheel, "front_left_wheel")

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(front_right_wheel, "front_right_wheel")

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_fork,
        origin=Origin(xyz=steer_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=steering_fork,
        child=upper_stem,
        origin=Origin(xyz=fold_origin_local),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-1.35, upper=0.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(track_half, rear_wheel_y, wheel_radius)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-track_half, rear_wheel_y, wheel_radius)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(track_half, front_wheel_y - steer_origin[1], wheel_radius - steer_origin[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(-track_half, front_wheel_y - steer_origin[1], wheel_radius - steer_origin[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    frame = object_model.get_part("frame")
    steering_fork = object_model.get_part("steering_fork")
    upper_stem = object_model.get_part("upper_stem")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    steering_yaw = object_model.get_articulation("steering_yaw")
    stem_fold = object_model.get_articulation("stem_fold")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")

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

    ctx.expect_gap(steering_fork, frame, axis="y", positive_elem="upper_bearing", negative_elem="head_block", min_gap=0.0, max_gap=0.0)
    ctx.expect_overlap(steering_fork, frame, axes="x", elem_a="upper_bearing", elem_b="head_block", min_overlap=0.03)
    ctx.expect_overlap(steering_fork, frame, axes="z", elem_a="upper_bearing", elem_b="head_block", min_overlap=0.012)
    ctx.expect_contact(upper_stem, steering_fork, elem_a="hinge_tongue", elem_b="clamp_saddle")
    ctx.expect_contact(rear_left_wheel, frame, elem_a="axle_core", elem_b="rear_axle_beam")
    ctx.expect_contact(rear_right_wheel, frame, elem_a="axle_core", elem_b="rear_axle_beam")
    ctx.expect_contact(front_left_wheel, steering_fork, elem_a="axle_core", elem_b="left_dropout")
    ctx.expect_contact(front_right_wheel, steering_fork, elem_a="axle_core", elem_b="right_dropout")

    ctx.check(
        "steering joint is z-yaw revolute",
        steering_yaw.articulation_type == ArticulationType.REVOLUTE and tuple(steering_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={steering_yaw.articulation_type} axis={steering_yaw.axis}",
    )
    ctx.check(
        "fold hinge is x-axis revolute",
        stem_fold.articulation_type == ArticulationType.REVOLUTE and tuple(stem_fold.axis) == (1.0, 0.0, 0.0),
        details=f"type={stem_fold.articulation_type} axis={stem_fold.axis}",
    )
    ctx.check(
        "all wheel joints spin on axle axis",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0)
            for joint in (
                rear_left_wheel_spin,
                rear_right_wheel_spin,
                front_left_wheel_spin,
                front_right_wheel_spin,
            )
        ),
        details="One or more wheel joints do not spin continuously about the lateral axle axis.",
    )

    front_left_rest = ctx.part_world_position(front_left_wheel)
    front_right_rest = ctx.part_world_position(front_right_wheel)
    assert front_left_rest is not None
    assert front_right_rest is not None
    with ctx.pose({steering_yaw: 0.45}):
        front_left_steered = ctx.part_world_position(front_left_wheel)
        front_right_steered = ctx.part_world_position(front_right_wheel)
        assert front_left_steered is not None
        assert front_right_steered is not None
        ctx.check(
            "fork yaws the front wheel pair",
            front_left_steered[1] > front_left_rest[1] + 0.03 and front_right_steered[1] < front_right_rest[1] - 0.03,
            details=(
                f"left_y {front_left_rest[1]:.3f}->{front_left_steered[1]:.3f}, "
                f"right_y {front_right_rest[1]:.3f}->{front_right_steered[1]:.3f}"
            ),
        )
        ctx.expect_contact(front_left_wheel, steering_fork, elem_a="axle_core", elem_b="left_dropout")
        ctx.expect_contact(front_right_wheel, steering_fork, elem_a="axle_core", elem_b="right_dropout")

    stem_rest_aabb = ctx.part_world_aabb(upper_stem)
    assert stem_rest_aabb is not None
    with ctx.pose({stem_fold: -1.20}):
        stem_folded_aabb = ctx.part_world_aabb(upper_stem)
        assert stem_folded_aabb is not None
        ctx.check(
            "upper stem folds forward from clamp block",
            stem_folded_aabb[1][1] > stem_rest_aabb[1][1] + 0.08 and stem_folded_aabb[1][2] < stem_rest_aabb[1][2] - 0.20,
            details=(
                f"rest max={stem_rest_aabb[1]}, folded max={stem_folded_aabb[1]}"
            ),
        )
        ctx.expect_within(upper_stem, steering_fork, axes="x", inner_elem="hinge_barrel", outer_elem="clamp_saddle")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

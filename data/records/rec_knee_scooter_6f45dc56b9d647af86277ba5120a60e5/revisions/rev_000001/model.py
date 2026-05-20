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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _add_wheel_visuals(
    part,
    *,
    mesh_name: str,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    hub_radius: float,
    rubber,
    rim_metal,
    hub_metal,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.50, -half_width * 0.96),
        (tire_radius * 0.74, -half_width),
        (tire_radius * 0.92, -half_width * 0.70),
        (tire_radius, -half_width * 0.18),
        (tire_radius, half_width * 0.18),
        (tire_radius * 0.92, half_width * 0.70),
        (tire_radius * 0.74, half_width),
        (tire_radius * 0.50, half_width * 0.96),
        (tire_radius * 0.43, half_width * 0.24),
        (tire_radius * 0.40, 0.0),
        (tire_radius * 0.43, -half_width * 0.24),
        (tire_radius * 0.50, -half_width * 0.96),
    ]
    part.visual(
        _mesh(mesh_name, LatheGeometry(tire_profile, segments=56).rotate_x(pi / 2.0)),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.92),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=tire_width * 1.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.42, length=tire_width * 1.25),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_knee_scooter")

    frame_finish = model.material("frame_finish", rgba=(0.26, 0.28, 0.30, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.13, 0.14, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    foam = model.material("foam", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.48, 0.05, 0.04)),
        origin=Origin(xyz=(0.00, 0.00, 0.22)),
        material=frame_finish,
        name="main_beam",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.284),
        origin=Origin(xyz=(-0.22, 0.0, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="rear_axle_tube",
    )
    frame.visual(
        _mesh(
            "frame_left_rear_strut",
            tube_from_spline_points(
                [(-0.09, 0.025, 0.205), (-0.17, 0.085, 0.16), (-0.22, 0.118, 0.10)],
                radius=0.014,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_finish,
        name="left_rear_strut",
    )
    frame.visual(
        _mesh(
            "frame_right_rear_strut",
            tube_from_spline_points(
                _mirror_y([(-0.09, 0.025, 0.205), (-0.17, 0.085, 0.16), (-0.22, 0.118, 0.10)]),
                radius=0.014,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_finish,
        name="right_rear_strut",
    )
    frame.visual(
        Box((0.074, 0.07, 0.055)),
        origin=Origin(xyz=(0.221, 0.0, 0.1825)),
        material=frame_finish,
        name="front_hinge_block",
    )
    frame.visual(
        Box((0.032, 0.022, 0.09)),
        origin=Origin(xyz=(0.275, 0.055, 0.205)),
        material=frame_finish,
        name="left_hinge_plate",
    )
    frame.visual(
        Box((0.032, 0.022, 0.09)),
        origin=Origin(xyz=(0.275, -0.055, 0.205)),
        material=frame_finish,
        name="right_hinge_plate",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.275, 0.055, 0.19), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="left_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.275, -0.055, 0.19), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="right_hinge_barrel",
    )
    frame.visual(
        Box((0.05, 0.04, 0.25)),
        origin=Origin(xyz=(-0.03, 0.0, 0.345)),
        material=frame_finish,
        name="pad_post",
    )
    frame.visual(
        Box((0.18, 0.14, 0.02)),
        origin=Origin(xyz=(-0.03, 0.0, 0.48)),
        material=satin_black,
        name="pad_bracket",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.66, 0.40, 0.56)),
        mass=5.2,
        origin=Origin(xyz=(0.03, 0.0, 0.28)),
    )

    knee_pad = model.part("knee_pad")
    knee_pad.visual(
        _mesh(
            "knee_pad_plate",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.24, 0.14, 0.018, corner_segments=8),
                0.01,
            ),
        ),
        material=satin_black,
        name="pad_plate",
    )
    knee_pad.visual(
        _mesh(
            "knee_pad_cushion",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.28, 0.16, 0.040, corner_segments=10),
                0.055,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=foam,
        name="pad_cushion",
    )
    knee_pad.visual(
        _mesh(
            "knee_pad_front_bolster",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.12, 0.16, 0.030, corner_segments=8),
                0.025,
            ),
        ),
        origin=Origin(xyz=(0.10, 0.0, 0.055)),
        material=foam,
        name="front_bolster",
    )
    knee_pad.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.09)),
        mass=1.1,
        origin=Origin(xyz=(0.02, 0.0, 0.04)),
    )

    lower_column = model.part("lower_column")
    column_shell = LatheGeometry.from_shell_profiles(
        [
            (0.030, 0.08),
            (0.030, 0.30),
            (0.033, 0.30),
            (0.033, 0.34),
            (0.025, 0.34),
            (0.025, 0.62),
        ],
        [
            (0.022, 0.08),
            (0.022, 0.30),
            (0.022, 0.34),
            (0.0205, 0.34),
            (0.0205, 0.62),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    ).translate(0.04, 0.0, 0.0)
    lower_column.visual(
        _mesh("lower_column_shell", column_shell),
        material=frame_finish,
        name="outer_sleeve",
    )
    lower_column.visual(
        Cylinder(radius=0.016, length=0.07),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="hinge_barrel",
    )
    lower_column.visual(
        Box((0.028, 0.05, 0.08)),
        origin=Origin(xyz=(0.028, 0.0, 0.045)),
        material=frame_finish,
        name="hinge_cheek",
    )
    lower_column.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(xyz=(0.075, 0.0, 0.32), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="clamp_bolt",
    )
    lower_column.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.64)),
        mass=1.7,
        origin=Origin(xyz=(0.04, 0.0, 0.32)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.019, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=aluminum,
        name="inner_tube",
    )
    upper_mast.visual(
        Cylinder(radius=0.031, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=frame_finish,
        name="mast_stop_collar",
    )
    upper_mast.visual(
        Cylinder(radius=0.024, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=frame_finish,
        name="stem_clamp",
    )
    upper_mast.visual(
        Cylinder(radius=0.020, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=frame_finish,
        name="riser",
    )
    upper_mast.visual(
        _mesh(
            "handlebar_tube",
            tube_from_spline_points(
                [
                    (-0.015, -0.21, 0.50),
                    (-0.008, -0.12, 0.515),
                    (0.0, 0.0, 0.52),
                    (-0.008, 0.12, 0.515),
                    (-0.015, 0.21, 0.50),
                ],
                radius=0.013,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_finish,
        name="handlebar",
    )
    upper_mast.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.015, 0.235, 0.50), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    upper_mast.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.015, -0.235, 0.50), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Box((0.18, 0.56, 0.78)),
        mass=1.3,
        origin=Origin(xyz=(-0.005, 0.0, 0.26)),
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.020, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=aluminum,
        name="steerer_tube",
    )
    fork.visual(
        Box((0.07, 0.06, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, -0.006)),
        material=frame_finish,
        name="steering_connector",
    )
    fork.visual(
        Box((0.11, 0.11, 0.035)),
        origin=Origin(xyz=(0.10, 0.0, -0.018)),
        material=frame_finish,
        name="upper_crown",
    )
    fork.visual(
        Box((0.06, 0.10, 0.028)),
        origin=Origin(xyz=(0.085, 0.0, -0.07)),
        material=frame_finish,
        name="lower_bridge",
    )
    fork.visual(
        _mesh(
            "fork_left_leg",
            tube_from_spline_points(
                [(0.055, 0.05, -0.012), (0.095, 0.072, -0.10), (0.12, 0.08, -0.18)],
                radius=0.012,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_finish,
        name="left_leg",
    )
    fork.visual(
        _mesh(
            "fork_right_leg",
            tube_from_spline_points(
                _mirror_y([(0.055, 0.05, -0.012), (0.095, 0.072, -0.10), (0.12, 0.08, -0.18)]),
                radius=0.012,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_finish,
        name="right_leg",
    )
    fork.visual(
        Cylinder(radius=0.010, length=0.20),
        origin=Origin(xyz=(0.12, 0.0, -0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle",
    )
    fork.visual(
        Box((0.045, 0.16, 0.014)),
        origin=Origin(xyz=(0.083, 0.0, -0.102)),
        material=frame_finish,
        name="fender_bridge",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.26, 0.24, 0.26)),
        mass=1.2,
        origin=Origin(xyz=(0.07, 0.0, -0.10)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        mesh_name="front_left_wheel_tire",
        tire_radius=0.10,
        tire_width=0.04,
        rim_radius=0.075,
        hub_radius=0.022,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=satin_black,
    )
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.04),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        mesh_name="front_right_wheel_tire",
        tire_radius=0.10,
        tire_width=0.04,
        rim_radius=0.075,
        hub_radius=0.022,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=satin_black,
    )
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.04),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        mesh_name="rear_left_wheel_tire",
        tire_radius=0.10,
        tire_width=0.04,
        rim_radius=0.075,
        hub_radius=0.022,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=satin_black,
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.04),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        mesh_name="rear_right_wheel_tire",
        tire_radius=0.10,
        tire_width=0.04,
        rim_radius=0.075,
        hub_radius=0.022,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=satin_black,
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.04),
        mass=0.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_knee_pad",
        ArticulationType.FIXED,
        parent=frame,
        child=knee_pad,
        origin=Origin(xyz=(-0.03, 0.0, 0.49)),
    )
    model.articulation(
        "frame_to_lower_column_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_column,
        origin=Origin(xyz=(0.275, 0.0, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.8, lower=0.0, upper=1.10),
    )
    model.articulation(
        "lower_column_to_fork_steer",
        ArticulationType.REVOLUTE,
        parent=lower_column,
        child=fork,
        origin=Origin(xyz=(0.04, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "lower_column_to_upper_mast",
        ArticulationType.PRISMATIC,
        parent=lower_column,
        child=upper_mast,
        origin=Origin(xyz=(0.04, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.20, lower=0.0, upper=0.16),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.12, 0.122, -0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.12, -0.122, -0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.22, 0.165, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.22, -0.165, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    knee_pad = object_model.get_part("knee_pad")
    lower_column = object_model.get_part("lower_column")
    upper_mast = object_model.get_part("upper_mast")
    fork = object_model.get_part("fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    fold_joint = object_model.get_articulation("frame_to_lower_column_fold")
    steer_joint = object_model.get_articulation("lower_column_to_fork_steer")
    mast_joint = object_model.get_articulation("lower_column_to_upper_mast")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_spin = object_model.get_articulation("front_right_wheel_spin")
    rear_left_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_spin = object_model.get_articulation("rear_right_wheel_spin")

    ctx.expect_contact(
        knee_pad,
        frame,
        elem_a="pad_plate",
        elem_b="pad_bracket",
        name="knee pad sits on the support bracket",
    )

    for joint_name, joint, expected_type, expected_axis in [
        ("fold", fold_joint, ArticulationType.REVOLUTE, (0.0, 1.0, 0.0)),
        ("steer", steer_joint, ArticulationType.REVOLUTE, (0.0, 0.0, 1.0)),
        ("mast", mast_joint, ArticulationType.PRISMATIC, (0.0, 0.0, 1.0)),
        ("front_left_spin", front_left_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
        ("front_right_spin", front_right_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
        ("rear_left_spin", rear_left_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
        ("rear_right_spin", rear_right_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
    ]:
        ctx.check(
            f"{joint_name} articulation is configured correctly",
            joint.joint_type == expected_type and tuple(joint.axis) == expected_axis,
            details=f"type={joint.joint_type}, axis={joint.axis}",
        )

    mast_rest = ctx.part_world_position(upper_mast)
    wheel_rest = ctx.part_world_position(front_left_wheel)
    folded_rest = ctx.part_world_position(upper_mast)
    with ctx.pose({mast_joint: 0.16}):
        ctx.expect_within(
            upper_mast,
            lower_column,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="mast stays centered in the outer sleeve",
        )
        ctx.expect_overlap(
            upper_mast,
            lower_column,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.07,
            name="mast keeps retained insertion at full extension",
        )
        mast_extended = ctx.part_world_position(upper_mast)
    ctx.check(
        "mast extends upward",
        mast_rest is not None and mast_extended is not None and mast_extended[2] > mast_rest[2] + 0.12,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    with ctx.pose({steer_joint: 0.45}):
        wheel_steered = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front fork steering yaws the wheel pair",
        wheel_rest is not None and wheel_steered is not None and wheel_steered[1] > wheel_rest[1] + 0.025,
        details=f"rest={wheel_rest}, steered={wheel_steered}",
    )

    with ctx.pose({fold_joint: 1.0}):
        folded_pose = ctx.part_world_position(upper_mast)
    ctx.check(
        "fold joint swings the upright forward for storage",
        folded_rest is not None
        and folded_pose is not None
        and folded_pose[0] > folded_rest[0] + 0.15
        and folded_pose[2] < folded_rest[2] - 0.10,
        details=f"rest={folded_rest}, folded={folded_pose}",
    )

    ctx.expect_overlap(
        front_left_wheel,
        fork,
        axes="z",
        elem_a="tire",
        elem_b="front_axle",
        min_overlap=0.015,
        name="front left wheel shares axle height with the fork",
    )
    ctx.expect_overlap(
        front_right_wheel,
        fork,
        axes="z",
        elem_a="tire",
        elem_b="front_axle",
        min_overlap=0.015,
        name="front right wheel shares axle height with the fork",
    )
    ctx.expect_overlap(
        rear_left_wheel,
        frame,
        axes="z",
        elem_a="tire",
        elem_b="rear_axle_tube",
        min_overlap=0.015,
        name="rear left wheel sits on the rear axle line",
    )
    ctx.expect_overlap(
        rear_right_wheel,
        frame,
        axes="z",
        elem_a="tire",
        elem_b="rear_axle_tube",
        min_overlap=0.015,
        name="rear right wheel sits on the rear axle line",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

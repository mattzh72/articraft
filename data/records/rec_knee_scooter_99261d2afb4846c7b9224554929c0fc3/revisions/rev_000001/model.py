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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    rim_width: float,
    hub_radius: float,
    hub_width: float,
    rubber,
    metal,
    dark_metal,
) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=rim_width),
        origin=spin_origin,
        material=metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=spin_origin,
        material=dark_metal,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    frame_paint = model.material("frame_paint", rgba=(0.58, 0.73, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.74, 0.77, 0.80, 1.0))
    pad_black = model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.86, 0.42, 0.78)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    spine = tube_from_spline_points(
        [
            (-0.22, 0.0, 0.09),
            (-0.19, 0.0, 0.38),
            (-0.04, 0.0, 0.50),
            (0.14, 0.0, 0.58),
        ],
        radius=0.023,
        samples_per_segment=16,
        radial_segments=18,
    )
    neck = tube_from_spline_points(
        [
            (0.14, 0.0, 0.58),
            (0.22, 0.0, 0.63),
            (0.28, 0.0, 0.665),
        ],
        radius=0.020,
        samples_per_segment=12,
        radial_segments=18,
    )
    frame.visual(mesh_from_geometry(spine, "frame_spine"), material=frame_paint, name="spine")
    frame.visual(mesh_from_geometry(neck, "frame_neck"), material=frame_paint, name="neck")
    frame.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.30, 0.0, 0.665)),
        material=dark_steel,
        name="steering_boss",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.380),
        origin=Origin(xyz=(-0.22, 0.0, 0.09), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.240),
        origin=Origin(xyz=(-0.02, 0.0, 0.57)),
        material=frame_paint,
        name="pad_post",
    )
    frame.visual(
        Box((0.22, 0.14, 0.020)),
        origin=Origin(xyz=(-0.02, 0.0, 0.70)),
        material=dark_steel,
        name="pad_plate",
    )
    frame.visual(
        Box((0.30, 0.18, 0.050)),
        origin=Origin(xyz=(-0.02, 0.0, 0.735)),
        material=pad_black,
        name="knee_pad_lower",
    )
    frame.visual(
        Box((0.24, 0.14, 0.040)),
        origin=Origin(xyz=(-0.02, 0.0, 0.780)),
        material=pad_black,
        name="knee_pad_upper",
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.40, 0.26, 1.02)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )
    front_fork.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="steerer_base",
    )
    front_fork.visual(
        Cylinder(radius=0.022, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=frame_paint,
        name="steerer_column",
    )
    front_fork.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(0.0, 0.034, -0.030)),
        material=frame_paint,
        name="left_crown_strut",
    )
    front_fork.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(0.0, -0.034, -0.030)),
        material=frame_paint,
        name="right_crown_strut",
    )
    handlebar = tube_from_spline_points(
        [
            (0.0, -0.19, 0.32),
            (0.0, -0.10, 0.34),
            (0.0, 0.00, 0.35),
            (0.0, 0.10, 0.34),
            (0.0, 0.19, 0.32),
        ],
        radius=0.014,
        samples_per_segment=12,
        radial_segments=18,
    )
    front_fork.visual(
        mesh_from_geometry(handlebar, "fork_handlebar"),
        material=frame_paint,
        name="handlebar",
    )
    front_fork.visual(
        Box((0.032, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=dark_steel,
        name="stem_clamp",
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.0, 0.215, 0.320), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.0, -0.215, 0.320), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    front_fork.visual(
        Box((0.026, 0.016, 0.125)),
        origin=Origin(xyz=(0.0, 0.047, -0.040), rpy=(-0.21, 0.0, 0.0)),
        material=frame_paint,
        name="left_yoke",
    )
    front_fork.visual(
        Box((0.026, 0.016, 0.125)),
        origin=Origin(xyz=(0.0, -0.047, -0.040), rpy=(0.21, 0.0, 0.0)),
        material=frame_paint,
        name="right_yoke",
    )
    front_fork.visual(
        Box((0.034, 0.126, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=frame_paint,
        name="fork_crown",
    )
    front_fork.visual(
        Box((0.028, 0.016, 0.440)),
        origin=Origin(xyz=(0.0, 0.053, -0.320)),
        material=frame_paint,
        name="left_leg",
    )
    front_fork.visual(
        Box((0.028, 0.016, 0.440)),
        origin=Origin(xyz=(0.0, -0.053, -0.320)),
        material=frame_paint,
        name="right_leg",
    )
    front_fork.visual(
        Box((0.028, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.057, -0.575)),
        material=frame_paint,
        name="left_dropout",
    )
    front_fork.visual(
        Box((0.028, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, -0.057, -0.575)),
        material=frame_paint,
        name="right_dropout",
    )
    front_fork.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.051, -0.595), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_axle_stub",
    )
    front_fork.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, -0.051, -0.595), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_axle_stub",
    )
    front_fork.visual(
        Box((0.028, 0.122, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.470)),
        material=frame_paint,
        name="fork_bridge",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.036),
        mass=1.6,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_wheel,
        tire_radius=0.100,
        tire_width=0.036,
        rim_radius=0.074,
        rim_width=0.038,
        hub_radius=0.026,
        hub_width=0.090,
        rubber=rubber,
        metal=satin_steel,
        dark_metal=dark_steel,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.034),
        mass=1.2,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_left_wheel,
        tire_radius=0.090,
        tire_width=0.034,
        rim_radius=0.066,
        rim_width=0.036,
        hub_radius=0.024,
        hub_width=0.050,
        rubber=rubber,
        metal=satin_steel,
        dark_metal=dark_steel,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.034),
        mass=1.2,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_right_wheel,
        tire_radius=0.090,
        tire_width=0.034,
        rim_radius=0.066,
        rim_width=0.036,
        hub_radius=0.024,
        hub_width=0.050,
        rubber=rubber,
        metal=satin_steel,
        dark_metal=dark_steel,
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.30, 0.0, 0.695)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.595)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.22, 0.215, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.22, -0.215, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")

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

    ctx.expect_contact(front_fork, frame, name="front_fork_supported_on_steering_head")
    ctx.expect_contact(front_wheel, front_fork, name="front_wheel_captured_between_fork_legs")
    ctx.expect_contact(rear_left_wheel, frame, name="left_rear_wheel_supported_by_rear_axle")
    ctx.expect_contact(rear_right_wheel, frame, name="right_rear_wheel_supported_by_rear_axle")

    ctx.expect_origin_gap(
        front_wheel,
        rear_left_wheel,
        axis="x",
        min_gap=0.48,
        max_gap=0.58,
        name="front_wheel_leads_rear_axle",
    )
    ctx.expect_origin_gap(
        rear_left_wheel,
        rear_right_wheel,
        axis="y",
        min_gap=0.40,
        max_gap=0.46,
        name="rear_track_is_stable_and_spread",
    )
    ctx.expect_gap(
        frame,
        rear_left_wheel,
        axis="z",
        positive_elem="knee_pad_lower",
        min_gap=0.50,
        max_gap=0.70,
        name="knee_pad_sits_well_above_rear_wheels",
    )

    steering_limits = steering_yaw.motion_limits
    ctx.check(
        "steering_joint_has_vertical_yaw_axis",
        steering_yaw.axis == (0.0, 0.0, 1.0)
        and steering_limits is not None
        and steering_limits.lower is not None
        and steering_limits.upper is not None
        and steering_limits.lower < 0.0 < steering_limits.upper,
        f"axis={steering_yaw.axis}, limits={steering_limits}",
    )
    for joint_name, wheel_joint in (
        ("front", front_wheel_spin),
        ("rear_left", rear_left_wheel_spin),
        ("rear_right", rear_right_wheel_spin),
    ):
        ctx.check(
            f"{joint_name}_wheel_spins_about_axle_axis",
            wheel_joint.axis == (0.0, 1.0, 0.0),
            f"{wheel_joint.name} axis was {wheel_joint.axis}",
        )

    for pose_name, steer_angle in (
        ("full_left", -0.65),
        ("full_right", 0.65),
    ):
        with ctx.pose({steering_yaw: steer_angle}):
            ctx.expect_contact(
                front_fork,
                frame,
                name=f"steering_head_stays_supported_at_{pose_name}",
            )
            ctx.expect_contact(
                front_wheel,
                front_fork,
                name=f"front_wheel_stays_captured_at_{pose_name}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"no_part_overlaps_at_{pose_name}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

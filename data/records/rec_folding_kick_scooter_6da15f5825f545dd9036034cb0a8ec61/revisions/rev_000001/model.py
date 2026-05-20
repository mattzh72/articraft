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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = sqrt(dx * dx + dy * dy)
    yaw = 0.0 if length_xy < 1e-9 else atan2(dy, dx)
    pitch = 0.0 if abs(dz) < 1e-9 and length_xy < 1e-9 else atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_wheel(
    part,
    *,
    radius: float,
    tire_width: float,
    hub_width: float,
    rubber,
    metal,
    hub_name: str,
    mesh_name: str,
) -> None:
    axle_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    inner_tire_radius = radius * 0.72
    tire_half = tire_width * 0.5
    tire_profile = [
        (inner_tire_radius, -tire_half * 0.94),
        (radius * 0.84, -tire_half),
        (radius * 0.96, -tire_half * 0.72),
        (radius, -tire_half * 0.25),
        (radius, tire_half * 0.25),
        (radius * 0.96, tire_half * 0.72),
        (radius * 0.84, tire_half),
        (inner_tire_radius, tire_half * 0.94),
        (inner_tire_radius * 0.96, tire_half * 0.45),
        (inner_tire_radius * 0.96, -tire_half * 0.45),
        (inner_tire_radius, -tire_half * 0.94),
    ]
    part.visual(
        mesh_from_geometry(LatheGeometry(tire_profile, segments=56).rotate_x(pi / 2.0), mesh_name),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=inner_tire_radius * 1.03, length=tire_width * 0.78),
        origin=axle_origin,
        material=metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=inner_tire_radius * 0.74, length=tire_width * 0.18),
        origin=axle_origin,
        material=metal,
        name="spider",
    )
    part.visual(
        Cylinder(radius=inner_tire_radius * 0.46, length=hub_width),
        origin=axle_origin,
        material=metal,
        name=hub_name,
    )


def _add_axle(
    part,
    *,
    shaft_radius: float,
    shaft_length: float,
    cap_radius: float,
    cap_length: float,
    metal,
) -> None:
    axle_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=shaft_radius, length=shaft_length),
        origin=axle_origin,
        material=metal,
        name="axle_shaft",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stairclimber_scooter")

    powder_black = model.material("powder_black", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.36, 0.38, 0.41, 1.0))
    grip_black = model.material("grip_black", rgba=(0.05, 0.05, 0.05, 1.0))
    accent_red = model.material("accent_red", rgba=(0.72, 0.12, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    deck_length = 0.58
    deck_width = 0.15
    deck_thickness = 0.020
    wheel_radius = 0.105
    wheel_tire_width = 0.028
    wheel_hub_width = 0.040
    axle_z = -0.118
    front_axle_x = 0.285
    rear_axle_x = -0.225

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.76, 0.44, 0.96)),
        mass=8.5,
        origin=Origin(xyz=(0.02, 0.0, 0.33)),
    )

    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(deck_length, deck_width, 0.024), deck_thickness),
        "scooter_deck_shell",
    )
    frame.visual(deck_mesh, material=powder_black, name="deck_shell")
    frame.visual(
        Box((0.40, 0.108, 0.003)),
        origin=Origin(xyz=(-0.01, 0.0, deck_thickness * 0.5 + 0.0015)),
        material=grip_black,
        name="grip_pad",
    )
    frame.visual(
        Box((0.11, 0.09, 0.012)),
        origin=Origin(xyz=(0.185, 0.0, -0.002)),
        material=dark_alloy,
        name="stem_base_block",
    )
    frame.visual(
        Box((0.14, 0.10, 0.024)),
        origin=Origin(xyz=(-0.175, 0.0, 0.001)),
        material=dark_alloy,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.268, 0.060, -0.023)),
        material=accent_red,
        name="left_aux_mount_front_ear",
    )
    frame.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.236, 0.060, -0.023)),
        material=accent_red,
        name="left_aux_mount_rear_ear",
    )
    frame.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.268, -0.060, -0.023)),
        material=accent_red,
        name="right_aux_mount_front_ear",
    )
    frame.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.236, -0.060, -0.023)),
        material=accent_red,
        name="right_aux_mount_rear_ear",
    )
    frame.visual(
        Box((0.065, 0.082, 0.040)),
        origin=Origin(xyz=(0.205, 0.0, 0.044)),
        material=dark_alloy,
        name="head_block",
    )
    _add_member(frame, (0.180, 0.0, 0.000), (0.070, 0.0, 0.860), 0.020, brushed_aluminum, name="stem_tube")
    _add_member(frame, (0.125, 0.0, 0.000), (0.115, 0.0, 0.320), 0.014, brushed_aluminum, name="stem_brace")
    _add_member(frame, (0.070, 0.0, 0.860), (0.070, 0.0, 0.905), 0.015, dark_alloy, name="handlebar_neck")
    handlebar_geom = tube_from_spline_points(
        [
            (0.060, -0.215, 0.875),
            (0.064, -0.145, 0.892),
            (0.070, -0.070, 0.905),
            (0.070, 0.070, 0.905),
            (0.064, 0.145, 0.892),
            (0.060, 0.215, 0.875),
        ],
        radius=0.014,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.visual(mesh_from_geometry(handlebar_geom, "scooter_handlebar"), material=dark_alloy, name="handlebar")
    frame.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(0.060, 0.235, 0.875), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(0.060, -0.235, 0.875), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    frame.visual(
        Box((0.050, 0.060, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, 0.030)),
        material=dark_alloy,
        name="fork_crown",
    )
    _add_member(frame, (0.215, 0.046, 0.050), (front_axle_x, 0.046, axle_z + 0.024), 0.006, brushed_aluminum, name="front_left_fork_leg")
    _add_member(frame, (0.215, -0.046, 0.050), (front_axle_x, -0.046, axle_z + 0.024), 0.006, brushed_aluminum, name="front_right_fork_leg")
    frame.visual(
        Box((0.018, 0.006, 0.032)),
        origin=Origin(xyz=(front_axle_x, 0.037, axle_z + 0.016)),
        material=brushed_aluminum,
        name="front_left_dropout",
    )
    frame.visual(
        Box((0.018, 0.006, 0.032)),
        origin=Origin(xyz=(front_axle_x, -0.037, axle_z + 0.016)),
        material=brushed_aluminum,
        name="front_right_dropout",
    )
    _add_member(frame, (-0.125, 0.046, -0.015), (rear_axle_x, 0.046, axle_z + 0.022), 0.006, powder_black, name="rear_left_stay")
    _add_member(frame, (-0.125, -0.046, -0.015), (rear_axle_x, -0.046, axle_z + 0.022), 0.006, powder_black, name="rear_right_stay")
    frame.visual(
        Box((0.018, 0.006, 0.032)),
        origin=Origin(xyz=(rear_axle_x, 0.037, axle_z + 0.016)),
        material=powder_black,
        name="rear_left_dropout",
    )
    frame.visual(
        Box((0.018, 0.006, 0.032)),
        origin=Origin(xyz=(rear_axle_x, -0.037, axle_z + 0.016)),
        material=powder_black,
        name="rear_right_dropout",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_tire_width),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel(
        front_wheel,
        radius=wheel_radius,
        tire_width=wheel_tire_width,
        hub_width=0.048,
        rubber=rubber,
        metal=dark_alloy,
        hub_name="hub",
        mesh_name="front_primary_tire",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.032),
        mass=1.1,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel(
        rear_wheel,
        radius=wheel_radius,
        tire_width=0.032,
        hub_width=0.050,
        rubber=rubber,
        metal=dark_alloy,
        hub_name="hub",
        mesh_name="rear_primary_tire",
    )

    front_axle = model.part("front_axle")
    front_axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.068),
        mass=0.20,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_axle(
        front_axle,
        shaft_radius=0.006,
        shaft_length=0.068,
        cap_radius=0.009,
        cap_length=0.006,
        metal=brushed_aluminum,
    )

    rear_axle = model.part("rear_axle")
    rear_axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.068),
        mass=0.22,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_axle(
        rear_axle,
        shaft_radius=0.006,
        shaft_length=0.068,
        cap_radius=0.009,
        cap_length=0.006,
        metal=dark_alloy,
    )

    left_aux_arm = model.part("left_aux_arm")
    left_aux_arm.inertial = Inertial.from_geometry(
        Box((0.04, 0.05, 0.13)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.018, -0.055)),
    )
    left_aux_arm.visual(
        Box((0.022, 0.012, 0.012)),
        material=accent_red,
        name="hinge_block",
    )
    left_aux_arm.visual(
        Box((0.014, 0.016, 0.060)),
        origin=Origin(xyz=(0.000, 0.010, -0.032)),
        material=powder_black,
        name="arm_spine",
    )
    left_aux_arm.visual(
        Box((0.014, 0.036, 0.008)),
        origin=Origin(xyz=(0.000, 0.020, -0.066)),
        material=powder_black,
        name="wheel_yoke",
    )
    left_aux_arm.visual(
        Box((0.010, 0.004, 0.072)),
        origin=Origin(xyz=(0.000, 0.004, -0.104)),
        material=powder_black,
        name="inner_fork",
    )
    left_aux_arm.visual(
        Box((0.010, 0.004, 0.072)),
        origin=Origin(xyz=(0.000, 0.036, -0.104)),
        material=powder_black,
        name="outer_fork",
    )
    left_aux_arm.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, 0.006, -0.140), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="inner_axle_stub",
    )
    left_aux_arm.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, 0.034, -0.140), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="outer_axle_stub",
    )

    right_aux_arm = model.part("right_aux_arm")
    right_aux_arm.inertial = Inertial.from_geometry(
        Box((0.04, 0.05, 0.13)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.018, -0.055)),
    )
    right_aux_arm.visual(
        Box((0.022, 0.012, 0.012)),
        material=accent_red,
        name="hinge_block",
    )
    right_aux_arm.visual(
        Box((0.014, 0.016, 0.060)),
        origin=Origin(xyz=(0.000, -0.010, -0.032)),
        material=powder_black,
        name="arm_spine",
    )
    right_aux_arm.visual(
        Box((0.014, 0.036, 0.008)),
        origin=Origin(xyz=(0.000, -0.020, -0.066)),
        material=powder_black,
        name="wheel_yoke",
    )
    right_aux_arm.visual(
        Box((0.010, 0.004, 0.072)),
        origin=Origin(xyz=(0.000, -0.004, -0.104)),
        material=powder_black,
        name="inner_fork",
    )
    right_aux_arm.visual(
        Box((0.010, 0.004, 0.072)),
        origin=Origin(xyz=(0.000, -0.036, -0.104)),
        material=powder_black,
        name="outer_fork",
    )
    right_aux_arm.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, -0.006, -0.140), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="inner_axle_stub",
    )
    right_aux_arm.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, -0.034, -0.140), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="outer_axle_stub",
    )

    left_aux_wheel = model.part("left_aux_wheel")
    left_aux_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.020),
        mass=0.35,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel(
        left_aux_wheel,
        radius=0.060,
        tire_width=0.018,
        hub_width=0.024,
        rubber=rubber,
        metal=dark_alloy,
        hub_name="hub",
        mesh_name="left_aux_tire",
    )

    right_aux_wheel = model.part("right_aux_wheel")
    right_aux_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.020),
        mass=0.35,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel(
        right_aux_wheel,
        radius=0.060,
        tire_width=0.018,
        hub_width=0.024,
        rubber=rubber,
        metal=dark_alloy,
        hub_name="hub",
        mesh_name="right_aux_tire",
    )

    front_spin = model.articulation(
        "frame_to_front_axle",
        ArticulationType.FIXED,
        parent=frame,
        child=front_axle,
        origin=Origin(xyz=(front_axle_x, 0.0, axle_z)),
    )
    rear_spin = model.articulation(
        "frame_to_rear_axle",
        ArticulationType.FIXED,
        parent=frame,
        child=rear_axle,
        origin=Origin(xyz=(rear_axle_x, 0.0, axle_z)),
    )
    front_spin = model.articulation(
        "front_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_wheel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    rear_spin = model.articulation(
        "rear_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=rear_axle,
        child=rear_wheel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=30.0),
    )
    left_fold = model.articulation(
        "left_aux_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_aux_arm,
        origin=Origin(xyz=(-0.252, 0.060, -0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=0.80),
    )
    right_fold = model.articulation(
        "right_aux_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_aux_arm,
        origin=Origin(xyz=(-0.252, -0.060, -0.041)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=0.80),
    )
    model.articulation(
        "left_aux_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=left_aux_arm,
        child=left_aux_wheel,
        origin=Origin(xyz=(0.000, 0.020, -0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "right_aux_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=right_aux_arm,
        child=right_aux_wheel,
        origin=Origin(xyz=(0.000, -0.020, -0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    front_axle = object_model.get_part("front_axle")
    rear_axle = object_model.get_part("rear_axle")
    left_aux_arm = object_model.get_part("left_aux_arm")
    right_aux_arm = object_model.get_part("right_aux_arm")
    left_aux_wheel = object_model.get_part("left_aux_wheel")
    right_aux_wheel = object_model.get_part("right_aux_wheel")
    front_spin = object_model.get_articulation("front_wheel_axle")
    rear_spin = object_model.get_articulation("rear_wheel_axle")
    left_fold = object_model.get_articulation("left_aux_fold")
    right_fold = object_model.get_articulation("right_aux_fold")
    left_aux_spin = object_model.get_articulation("left_aux_wheel_axle")
    right_aux_spin = object_model.get_articulation("right_aux_wheel_axle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        front_axle,
        front_wheel,
        reason="The front wheel is modeled with a solid center proxy, so the axle shaft intentionally occupies the wheel bore region.",
    )
    ctx.allow_overlap(
        rear_axle,
        rear_wheel,
        reason="The rear wheel is modeled with a solid center proxy, so the axle shaft intentionally occupies the wheel bore region.",
    )

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

    for part_name in (
        "frame",
        "front_axle",
        "rear_axle",
        "front_wheel",
        "rear_wheel",
        "left_aux_arm",
        "right_aux_arm",
        "left_aux_wheel",
        "right_aux_wheel",
    ):
        ctx.check(f"{part_name} present", object_model.get_part(part_name) is not None)

    ctx.expect_contact(front_axle, frame, name="front axle is clamped by the fork dropouts")
    ctx.expect_contact(rear_axle, frame, name="rear axle is clamped by the rear dropouts")
    ctx.expect_contact(left_aux_arm, frame, name="left stair wheel arm is mounted to the rear deck corner")
    ctx.expect_contact(right_aux_arm, frame, name="right stair wheel arm is mounted to the rear deck corner")
    ctx.expect_contact(left_aux_wheel, left_aux_arm, name="left stair wheel sits in its yoke")
    ctx.expect_contact(right_aux_wheel, right_aux_arm, name="right stair wheel sits in its yoke")
    ctx.expect_within(
        front_axle,
        front_wheel,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="hub",
        margin=0.001,
        name="front axle stays centered inside the front hub",
    )
    ctx.expect_within(
        rear_axle,
        rear_wheel,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="hub",
        margin=0.001,
        name="rear axle stays centered inside the rear hub",
    )

    ctx.check(
        "primary wheels spin on transverse axles",
        tuple(front_spin.axis) == (0.0, 1.0, 0.0) and tuple(rear_spin.axis) == (0.0, 1.0, 0.0),
        details=f"front={front_spin.axis}, rear={rear_spin.axis}",
    )
    ctx.check(
        "auxiliary wheels spin on transverse axles",
        tuple(left_aux_spin.axis) == (0.0, 1.0, 0.0) and tuple(right_aux_spin.axis) == (0.0, 1.0, 0.0),
        details=f"left={left_aux_spin.axis}, right={right_aux_spin.axis}",
    )
    ctx.check(
        "rear corner hinges are mirrored longitudinal folds",
        tuple(left_fold.axis) == (1.0, 0.0, 0.0) and tuple(right_fold.axis) == (-1.0, 0.0, 0.0),
        details=f"left={left_fold.axis}, right={right_fold.axis}",
    )

    left_deployed = ctx.part_world_position(left_aux_wheel)
    right_deployed = ctx.part_world_position(right_aux_wheel)
    left_upper = 0.0 if left_fold.motion_limits is None or left_fold.motion_limits.upper is None else left_fold.motion_limits.upper
    right_upper = 0.0 if right_fold.motion_limits is None or right_fold.motion_limits.upper is None else right_fold.motion_limits.upper

    with ctx.pose({left_fold: left_upper, right_fold: right_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps when stair-assist wheels are folded")
        ctx.expect_gap(
            frame,
            left_aux_wheel,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="tire",
            min_gap=0.0,
            name="left folded stair wheel stays below the deck",
        )
        ctx.expect_gap(
            frame,
            right_aux_wheel,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="tire",
            min_gap=0.0,
            name="right folded stair wheel stays below the deck",
        )
        left_folded = ctx.part_world_position(left_aux_wheel)
        right_folded = ctx.part_world_position(right_aux_wheel)

    ctx.check(
        "left stair wheel folds upward when stowed",
        left_deployed is not None
        and left_folded is not None
        and left_folded[2] > left_deployed[2] + 0.05,
        details=f"deployed={left_deployed}, folded={left_folded}",
    )
    ctx.check(
        "right stair wheel folds upward when stowed",
        right_deployed is not None
        and right_folded is not None
        and right_folded[2] > right_deployed[2] + 0.05,
        details=f"deployed={right_deployed}, folded={right_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

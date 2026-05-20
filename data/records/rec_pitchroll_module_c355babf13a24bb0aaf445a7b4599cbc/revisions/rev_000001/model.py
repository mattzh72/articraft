from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _box(center: tuple[float, float, float], size: tuple[float, float, float]):
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Solid.makeBox(
        sx,
        sy,
        sz,
        cq.Vector(cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0),
    )


def _cyl_x(center: tuple[float, float, float], radius: float, length: float):
    cx, cy, cz = center
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(cx - length / 2.0, cy, cz),
        cq.Vector(1.0, 0.0, 0.0),
    )


def _cyl_y(center: tuple[float, float, float], radius: float, length: float):
    cx, cy, cz = center
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(cx, cy - length / 2.0, cz),
        cq.Vector(0.0, 1.0, 0.0),
    )


def _ring_x(
    center: tuple[float, float, float],
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    outer = _cyl_x(center, outer_radius, length)
    inner = _cyl_x(center, inner_radius, length + 0.004)
    return outer.cut(inner)


def _make_base_frame():
    base = _box(center=(-0.046, 0.000, -0.086), size=(0.118, 0.158, 0.014))
    rear_bridge = _box(center=(-0.088, 0.000, -0.031), size=(0.018, 0.108, 0.080))

    left_cheek = _box(center=(-0.020, 0.068, -0.020), size=(0.072, 0.012, 0.110))
    right_cheek = _box(center=(-0.020, -0.068, -0.020), size=(0.072, 0.012, 0.110))

    left_boss = _cyl_y(center=(0.000, 0.068, 0.000), radius=0.022, length=0.018)
    right_boss = _cyl_y(center=(0.000, -0.068, 0.000), radius=0.022, length=0.018)

    left_foot = _box(center=(-0.018, 0.048, -0.075), size=(0.040, 0.020, 0.022))
    right_foot = _box(center=(-0.018, -0.048, -0.075), size=(0.040, 0.020, 0.022))

    frame = (
        base.fuse(rear_bridge)
        .fuse(left_cheek)
        .fuse(right_cheek)
        .fuse(left_boss)
        .fuse(right_boss)
        .fuse(left_foot)
        .fuse(right_foot)
    )

    left_window = _box(center=(-0.012, 0.068, -0.018), size=(0.046, 0.016, 0.074))
    right_window = _box(center=(-0.012, -0.068, -0.018), size=(0.046, 0.016, 0.074))
    frame = frame.cut(left_window).cut(right_window)

    trunnion_bore = _cyl_y(center=(0.000, 0.000, 0.000), radius=0.012, length=0.190)
    frame = frame.cut(trunnion_bore)

    return frame


def _make_pitch_yoke():
    left_trunnion = _cyl_y(center=(0.000, 0.057, 0.000), radius=0.010, length=0.060)
    right_trunnion = _cyl_y(center=(0.000, -0.057, 0.000), radius=0.010, length=0.060)
    left_collar = _cyl_y(center=(0.000, 0.081, 0.000), radius=0.017, length=0.008)
    right_collar = _cyl_y(center=(0.000, -0.081, 0.000), radius=0.017, length=0.008)

    left_plate = _box(center=(0.032, 0.040, 0.000), size=(0.044, 0.014, 0.034))
    right_plate = _box(center=(0.032, -0.040, 0.000), size=(0.044, 0.014, 0.034))
    left_nose_link = _box(center=(0.070, 0.025, 0.000), size=(0.036, 0.010, 0.020))
    right_nose_link = _box(center=(0.070, -0.025, 0.000), size=(0.036, 0.010, 0.020))

    rear_ring = _ring_x(center=(0.088, 0.000, 0.000), outer_radius=0.026, inner_radius=0.019, length=0.008)
    front_ring = _ring_x(center=(0.100, 0.000, 0.000), outer_radius=0.026, inner_radius=0.019, length=0.008)
    top_bridge = _box(center=(0.094, 0.000, 0.022), size=(0.030, 0.012, 0.010))
    bottom_bridge = _box(center=(0.094, 0.000, -0.022), size=(0.030, 0.012, 0.010))

    return (
        left_trunnion.fuse(right_trunnion)
        .fuse(left_collar)
        .fuse(right_collar)
        .fuse(left_plate)
        .fuse(right_plate)
        .fuse(left_nose_link)
        .fuse(right_nose_link)
        .fuse(rear_ring)
        .fuse(front_ring)
        .fuse(top_bridge)
        .fuse(bottom_bridge)
    )


def _make_roll_cartridge():
    spindle = _cyl_x(center=(0.000, 0.000, 0.000), radius=0.014, length=0.028)
    rear_journal = _cyl_x(center=(-0.006, 0.000, 0.000), radius=0.019, length=0.008)
    front_journal = _cyl_x(center=(0.006, 0.000, 0.000), radius=0.019, length=0.008)
    rear_cap = _cyl_x(center=(-0.020, 0.000, 0.000), radius=0.016, length=0.010)
    main_body = _cyl_x(center=(0.052, 0.000, 0.000), radius=0.023, length=0.068)
    front_bezel = _cyl_x(center=(0.091, 0.000, 0.000), radius=0.026, length=0.010)

    cartridge = (
        spindle.fuse(rear_journal)
        .fuse(front_journal)
        .fuse(rear_cap)
        .fuse(main_body)
        .fuse(front_bezel)
    )

    sensor_face = _cyl_x(center=(0.094, 0.000, 0.000), radius=0.012, length=0.006)
    return cartridge.cut(sensor_face)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_and_spindle_sensor_bracket")

    frame_material = model.material("frame_powdercoat", rgba=(0.22, 0.24, 0.26, 1.0))
    yoke_material = model.material("yoke_alloy", rgba=(0.69, 0.71, 0.74, 1.0))
    cartridge_material = model.material("sensor_cartridge", rgba=(0.14, 0.17, 0.21, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_make_base_frame(), "base_frame"),
        material=frame_material,
        name="frame_shell",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.112, 0.144, 0.120)),
        mass=2.0,
        origin=Origin(xyz=(-0.020, 0.000, -0.035)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_make_pitch_yoke(), "pitch_yoke"),
        material=yoke_material,
        name="yoke_shell",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.126, 0.170, 0.058)),
        mass=0.95,
        origin=Origin(xyz=(0.050, 0.000, 0.000)),
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        mesh_from_cadquery(_make_roll_cartridge(), "roll_cartridge"),
        material=cartridge_material,
        name="cartridge_shell",
    )
    roll_cartridge.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.106),
        mass=0.62,
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=pitch_yoke,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-0.95,
            upper=0.75,
        ),
    )

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_cartridge,
        origin=Origin(xyz=(0.094, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_cartridge = object_model.get_part("roll_cartridge")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        base_frame,
        pitch_yoke,
        reason=(
            "Pitch trunnion pins are intentionally nested into the frame-side trunnion "
            "bosses as a simplified bearing representation."
        ),
    )
    ctx.allow_overlap(
        pitch_yoke,
        roll_cartridge,
        reason=(
            "The roll spindle is represented as journals nested in the yoke's coaxial "
            "bearing rings rather than with separate inner races."
        ),
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "pitch_joint_axis_is_lateral",
        tuple(pitch_joint.axis) == (0.0, 1.0, 0.0),
        f"expected pitch axis (0, 1, 0), got {pitch_joint.axis}",
    )
    ctx.check(
        "roll_joint_axis_is_forward",
        tuple(roll_joint.axis) == (1.0, 0.0, 0.0),
        f"expected roll axis (1, 0, 0), got {roll_joint.axis}",
    )

    ctx.expect_contact(
        pitch_yoke,
        base_frame,
        name="pitch_trunnions_seat_in_frame_cheeks",
    )
    ctx.expect_contact(
        roll_cartridge,
        pitch_yoke,
        name="roll_journals_seat_in_coaxial_bearings",
    )
    ctx.expect_gap(
        roll_cartridge,
        base_frame,
        axis="x",
        min_gap=0.020,
        name="cartridge_projects_forward_of_base_frame",
    )

    rest_position = ctx.part_world_position(roll_cartridge)
    with ctx.pose({pitch_joint: -0.45}):
        pitched_position = ctx.part_world_position(roll_cartridge)
    if rest_position is None or pitched_position is None:
        ctx.fail("pitch_motion_pose_available", "could not resolve cartridge world position")
    else:
        ctx.check(
            "pitch_motion_lifts_the_nose",
            pitched_position[2] > rest_position[2] + 0.030,
            f"expected pitch to raise cartridge center; rest={rest_position}, pitched={pitched_position}",
        )

    with ctx.pose({roll_joint: 1.20}):
        rolled_position = ctx.part_world_position(roll_cartridge)
    if rest_position is None or rolled_position is None:
        ctx.fail("roll_motion_pose_available", "could not resolve cartridge world position")
    else:
        center_shift = math.dist(rest_position, rolled_position)
        ctx.check(
            "roll_motion_keeps_spindle_center_fixed",
            center_shift <= 1e-6,
            f"roll articulation displaced cartridge center by {center_shift:.6g} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

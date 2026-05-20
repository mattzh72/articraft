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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_X = 0.18
BASE_PLATE_Y = 0.14
BASE_PLATE_Z = 0.02
BASE_COLUMN_Z = 0.055
BASE_SEAT_Z = 0.01
BASE_TOTAL_HEIGHT = BASE_PLATE_Z + BASE_COLUMN_Z + BASE_SEAT_Z

UPPER_ELBOW_X = 0.195
UPPER_ELBOW_Z = 0.085
FOREARM_WRIST_X = 0.23
WRIST_CARTRIDGE_X = 0.055


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _cylinder_x(radius: float, length: float, x0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, 0.0, 0.0))


def _cylinder_y(radius: float, length: float, y0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((0.0, y0, 0.0))


def make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z)
        .translate((0.0, 0.0, BASE_PLATE_Z / 2.0))
        .edges("|Z")
        .fillet(0.01)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.065, -0.045),
                (-0.065, 0.045),
                (0.065, -0.045),
                (0.065, 0.045),
            ]
        )
        .hole(0.012)
    )
    column = _cylinder_z(0.05, BASE_COLUMN_Z, BASE_PLATE_Z)
    seat = _cylinder_z(0.062, BASE_SEAT_Z, BASE_PLATE_Z + BASE_COLUMN_Z)
    return plate.union(column).union(seat)


def make_upper_arm_shape() -> cq.Workplane:
    turret = _cylinder_z(0.046, 0.105, 0.0)
    top_cap = _cylinder_z(0.038, 0.012, 0.105)
    shoulder_web = _box((0.07, 0.074, 0.07), (0.035, 0.0, 0.058))
    arm_beam = (
        _box((0.14, 0.052, 0.06), (0.07, 0.0, UPPER_ELBOW_Z))
        .edges("|X")
        .fillet(0.01)
    )
    elbow_cheek_neg = _box((0.035, 0.018, 0.06), (0.1725, -0.031, UPPER_ELBOW_Z)).edges("|X").fillet(0.004)
    elbow_cheek_pos = _box((0.035, 0.018, 0.06), (0.1725, 0.031, UPPER_ELBOW_Z)).edges("|X").fillet(0.004)
    elbow_center_plate = _box((0.02, 0.028, 0.028), (0.185, 0.0, UPPER_ELBOW_Z))
    return (
        turret.union(top_cap)
        .union(shoulder_web)
        .union(arm_beam)
        .union(elbow_cheek_neg)
        .union(elbow_cheek_pos)
        .union(elbow_center_plate)
    )


def make_forearm_shape() -> cq.Workplane:
    elbow_tongue = _box((0.055, 0.022, 0.042), (0.0275, 0.0, 0.0)).edges("|X").fillet(0.005)
    tapered_beam = (
        cq.Workplane("YZ")
        .workplane(offset=0.055)
        .rect(0.028, 0.044)
        .workplane(offset=0.13)
        .rect(0.024, 0.038)
        .loft(combine=True)
    )
    underside_rib = _box((0.11, 0.012, 0.015), (0.125, 0.0, -0.022)).edges("|X").fillet(0.003)
    wrist_collar = _cylinder_x(0.032, 0.04, 0.19)
    return elbow_tongue.union(tapered_beam).union(underside_rib).union(wrist_collar)


def make_wrist_cartridge_shape() -> cq.Workplane:
    rear_ring = _cylinder_x(0.039, 0.012, 0.0)
    main_body = _cylinder_x(0.031, 0.035, 0.012)
    nose = _cylinder_x(0.024, 0.008, 0.047)
    return rear_ring.union(main_body).union(nose)


def make_tool_flange_shape() -> cq.Workplane:
    flange = _cylinder_x(0.041, 0.012, 0.0).union(_cylinder_x(0.018, 0.018, 0.0))
    flange = flange.faces("<X").workplane(centerOption="CenterOfMass").circle(0.011).cutThruAll()
    flange = (
        flange.faces("<X")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.026, 0.0, 360.0, 4)
        .circle(0.003)
        .cutThruAll()
    )
    return flange


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_robot_arm")

    graphite = model.material("graphite", color=(0.20, 0.22, 0.24))
    arm_paint = model.material("arm_paint", color=(0.84, 0.85, 0.87))
    dark_trim = model.material("dark_trim", color=(0.11, 0.12, 0.13))
    machined = model.material("machined", color=(0.73, 0.74, 0.77))

    bench_base = model.part("bench_base")
    bench_base.visual(
        mesh_from_cadquery(make_base_shape(), "bench_base"),
        material=graphite,
        name="base_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(make_upper_arm_shape(), "upper_arm"),
        material=arm_paint,
        name="upper_arm_shell",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(make_forearm_shape(), "forearm"),
        material=arm_paint,
        name="forearm_shell",
    )

    wrist_roll = model.part("wrist_roll")
    wrist_roll.visual(
        mesh_from_cadquery(make_wrist_cartridge_shape(), "wrist_roll"),
        material=dark_trim,
        name="wrist_cartridge_shell",
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        mesh_from_cadquery(make_tool_flange_shape(), "tool_flange"),
        material=machined,
        name="tool_flange_shell",
    )
    tool_flange.visual(
        Box((0.008, 0.012, 0.01)),
        origin=Origin(xyz=(0.006, 0.0, 0.036)),
        material=dark_trim,
        name="roll_index_key",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=bench_base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ELBOW_X, 0.0, UPPER_ELBOW_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.0,
            lower=-1.35,
            upper=1.10,
        ),
    )
    model.articulation(
        "wrist_roll_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_roll,
        origin=Origin(xyz=(FOREARM_WRIST_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.5,
            lower=-3.1,
            upper=3.1,
        ),
    )
    model.articulation(
        "wrist_to_flange",
        ArticulationType.FIXED,
        parent=wrist_roll,
        child=tool_flange,
        origin=Origin(xyz=(WRIST_CARTRIDGE_X, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench_base = object_model.get_part("bench_base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_roll = object_model.get_part("wrist_roll")
    tool_flange = object_model.get_part("tool_flange")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll_joint = object_model.get_articulation("wrist_roll_joint")
    wrist_to_flange = object_model.get_articulation("wrist_to_flange")

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

    for part_name, part in (
        ("bench_base", bench_base),
        ("upper_arm", upper_arm),
        ("forearm", forearm),
        ("wrist_roll", wrist_roll),
        ("tool_flange", tool_flange),
    ):
        ctx.check(f"{part_name}_present", part.name == part_name, f"missing or wrong part for {part_name}")

    ctx.check(
        "shoulder_axis_vertical",
        shoulder_yaw.axis == (0.0, 0.0, 1.0),
        f"expected shoulder yaw axis to be vertical, got {shoulder_yaw.axis}",
    )
    ctx.check(
        "elbow_axis_horizontal",
        elbow_pitch.axis == (0.0, 1.0, 0.0),
        f"expected elbow axis to run across the arm, got {elbow_pitch.axis}",
    )
    ctx.check(
        "wrist_axis_forearm_aligned",
        wrist_roll_joint.axis == (1.0, 0.0, 0.0),
        f"expected wrist roll axis along the forearm, got {wrist_roll_joint.axis}",
    )
    ctx.check(
        "tool_flange_fixed_to_wrist",
        wrist_to_flange.articulation_type == ArticulationType.FIXED,
        "tool flange should be rigidly carried by the wrist cartridge",
    )

    ctx.expect_contact(bench_base, upper_arm, name="shoulder_bearing_contact")
    ctx.expect_contact(upper_arm, forearm, name="elbow_bearing_contact")
    ctx.expect_contact(forearm, wrist_roll, name="wrist_cartridge_mount_contact")
    ctx.expect_contact(wrist_roll, tool_flange, name="tool_flange_mount_contact")

    ctx.expect_origin_gap(
        upper_arm,
        bench_base,
        axis="z",
        min_gap=BASE_TOTAL_HEIGHT - 0.001,
        max_gap=BASE_TOTAL_HEIGHT + 0.001,
        name="shoulder_origin_height",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=UPPER_ELBOW_X - 0.001,
        max_gap=UPPER_ELBOW_X + 0.001,
        name="elbow_reach_from_pedestal",
    )
    ctx.expect_origin_gap(
        wrist_roll,
        forearm,
        axis="x",
        min_gap=FOREARM_WRIST_X - 0.001,
        max_gap=FOREARM_WRIST_X + 0.001,
        name="wrist_mount_at_forearm_tip",
    )

    rest_flange_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "tool_flange_rest_pose_forward_reach",
        rest_flange_pos is not None and rest_flange_pos[0] > 0.45 and 0.16 < rest_flange_pos[2] < 0.18,
        f"unexpected flange rest pose position: {rest_flange_pos}",
    )

    with ctx.pose(shoulder_yaw=0.75):
        yawed_flange_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "shoulder_yaw_swings_arm_in_plan",
        rest_flange_pos is not None
        and yawed_flange_pos is not None
        and abs(yawed_flange_pos[1] - rest_flange_pos[1]) > 0.20
        and abs(yawed_flange_pos[2] - rest_flange_pos[2]) < 0.005,
        f"shoulder yaw did not swing the arm laterally as expected: rest={rest_flange_pos}, yawed={yawed_flange_pos}",
    )

    with ctx.pose(elbow_pitch=-0.9):
        raised_flange_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "elbow_pitch_raises_tool",
        rest_flange_pos is not None
        and raised_flange_pos is not None
        and raised_flange_pos[2] > rest_flange_pos[2] + 0.12,
        f"elbow pitch should raise the tool flange: rest={rest_flange_pos}, raised={raised_flange_pos}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    key_rest = _aabb_center(ctx.part_element_world_aabb(tool_flange, elem="roll_index_key"))
    with ctx.pose(wrist_roll_joint=1.2):
        key_rolled = _aabb_center(ctx.part_element_world_aabb(tool_flange, elem="roll_index_key"))
    ctx.check(
        "wrist_roll_rotates_index_key",
        key_rest is not None
        and key_rolled is not None
        and abs(key_rolled[0] - key_rest[0]) < 0.002
        and abs(key_rolled[1] - key_rest[1]) > 0.015
        and abs(key_rolled[2] - key_rest[2]) > 0.015,
        f"wrist roll should rotate the flange key around the forearm axis: rest={key_rest}, rolled={key_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robotic_arm")

    base_gray = model.material("base_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    arm_blue = model.material("arm_blue", rgba=(0.18, 0.37, 0.65, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.24, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_gray,
        name="floor_foot",
    )
    pedestal_base.visual(
        Cylinder(radius=0.12, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=base_gray,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=dark_gray,
        name="shoulder_base_housing",
    )
    pedestal_base.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=steel,
        name="shoulder_bearing_ring",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 0.60)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.135, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel,
        name="shoulder_turntable",
    )
    upper_arm.visual(
        Box((0.14, 0.20, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.08)),
        material=dark_gray,
        name="shoulder_motor_pack",
    )
    upper_arm.visual(
        Box((0.30, 0.11, 0.10)),
        origin=Origin(xyz=(0.16, 0.0, 0.08)),
        material=arm_blue,
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Cylinder(radius=0.07, length=0.12),
        origin=Origin(xyz=(0.33, 0.0, 0.09), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="elbow_housing",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.16)),
        mass=26.0,
        origin=Origin(xyz=(0.17, 0.0, 0.08)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.08, 0.14, 0.14)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=dark_gray,
        name="elbow_drive_block",
    )
    forearm.visual(
        Box((0.25, 0.09, 0.09)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=arm_blue,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.18, 0.05, 0.04)),
        origin=Origin(xyz=(0.18, 0.0, -0.045)),
        material=steel,
        name="forearm_stiffener",
    )
    forearm.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="wrist_roll_housing",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.36, 0.16, 0.16)),
        mass=16.0,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.04, length=0.06),
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="roll_coupler",
    )
    wrist_head.visual(
        Box((0.10, 0.12, 0.10)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=dark_gray,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.02, 0.09, 0.10)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.06, 0.035, 0.08)),
        origin=Origin(xyz=(0.12, 0.0, -0.06)),
        material=arm_blue,
        name="tool_pad",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.18)),
        mass=6.0,
        origin=Origin(xyz=(0.09, 0.0, -0.01)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.8, lower=-pi, upper=pi),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.40, 0.0, 0.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=2.0, lower=-1.2, upper=1.35),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.5, lower=-pi, upper=pi),
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
    pedestal_base = object_model.get_part("pedestal_base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.expect_origin_gap(
        upper_arm,
        pedestal_base,
        axis="z",
        min_gap=0.55,
        max_gap=0.61,
        name="shoulder axis sits well above the floor line",
    )
    ctx.expect_contact(
        upper_arm,
        pedestal_base,
        name="upper arm mounts onto the pedestal shoulder bearing",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        name="forearm nests directly against the elbow joint housing",
    )
    ctx.expect_contact(
        wrist_head,
        forearm,
        name="wrist head remains seated on the forearm roll housing",
    )

    rest_pos = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder_yaw: 0.8}):
        yawed_pos = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder yaw keeps the arm rooted while rotating in place",
        rest_pos is not None
        and yawed_pos is not None
        and abs(rest_pos[2] - yawed_pos[2]) < 1e-6
        and abs(rest_pos[0] - yawed_pos[0]) < 1e-6
        and abs(rest_pos[1] - yawed_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    wrist_rest = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 0.95}):
        wrist_lifted = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch lifts the forearm and wrist upward",
        wrist_rest is not None
        and wrist_lifted is not None
        and wrist_lifted[2] > wrist_rest[2] + 0.18,
        details=f"rest={wrist_rest}, lifted={wrist_lifted}",
    )

    with ctx.pose({shoulder_yaw: 1.0}):
        wrist_swung = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw sweeps the arm around the pedestal",
        wrist_rest is not None
        and wrist_swung is not None
        and abs(wrist_swung[1] - wrist_rest[1]) > 0.40,
        details=f"rest={wrist_rest}, swung={wrist_swung}",
    )

    tool_pad_rest = ctx.part_element_world_aabb(wrist_head, elem="tool_pad")
    with ctx.pose({wrist_roll: pi / 2.0}):
        tool_pad_rolled = ctx.part_element_world_aabb(wrist_head, elem="tool_pad")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            0.5 * (mins[0] + maxs[0]),
            0.5 * (mins[1] + maxs[1]),
            0.5 * (mins[2] + maxs[2]),
        )

    tool_pad_rest_center = _aabb_center(tool_pad_rest)
    tool_pad_rolled_center = _aabb_center(tool_pad_rolled)
    ctx.check(
        "wrist roll visibly reorients the off-axis tool pad",
        tool_pad_rest_center is not None
        and tool_pad_rolled_center is not None
        and abs(tool_pad_rolled_center[1] - tool_pad_rest_center[1]) > 0.03
        and abs(tool_pad_rolled_center[2] - tool_pad_rest_center[2]) > 0.01,
        details=f"rest={tool_pad_rest_center}, rolled={tool_pad_rolled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

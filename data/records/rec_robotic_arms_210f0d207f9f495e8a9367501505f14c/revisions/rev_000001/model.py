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

    base_dark = model.material("base_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    joint_dark = model.material("joint_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    wrist_gray = model.material("wrist_gray", rgba=(0.62, 0.64, 0.67, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.46, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=base_dark,
        name="base_plinth",
    )
    base.visual(
        Box((0.32, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=base_dark,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=joint_dark,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=joint_dark,
        name="slew_ring",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.46, 0.48)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    shoulder_stage = model.part("shoulder_stage")
    shoulder_stage.visual(
        Cylinder(radius=0.14, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=joint_dark,
        name="shoulder_turntable",
    )
    shoulder_stage.visual(
        Box((0.24, 0.24, 0.20)),
        origin=Origin(xyz=(0.10, 0.0, 0.16)),
        material=arm_gray,
        name="shoulder_housing",
    )
    shoulder_stage.visual(
        Box((0.40, 0.16, 0.14)),
        origin=Origin(xyz=(0.41, 0.0, 0.16)),
        material=arm_gray,
        name="upper_arm_beam",
    )
    shoulder_stage.visual(
        Cylinder(radius=0.08, length=0.22),
        origin=Origin(xyz=(0.62, 0.0, 0.16), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_dark,
        name="elbow_hub",
    )
    shoulder_stage.inertial = Inertial.from_geometry(
        Box((0.74, 0.26, 0.28)),
        mass=32.0,
        origin=Origin(xyz=(0.32, 0.0, 0.14)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.10, 0.10, 0.10)),
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
        material=joint_dark,
        name="elbow_bridge",
    )
    forearm.visual(
        Box((0.36, 0.12, 0.12)),
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        material=arm_gray,
        name="forearm_beam",
    )
    forearm.visual(
        Cylinder(radius=0.065, length=0.18),
        origin=Origin(xyz=(0.63, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_dark,
        name="wrist_sleeve",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.74, 0.18, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.05, length=0.11),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_dark,
        name="wrist_roll_cap",
    )
    wrist_head.visual(
        Box((0.12, 0.14, 0.12)),
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        material=wrist_gray,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.04, 0.08, 0.10)),
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
        material=wrist_gray,
        name="wrist_nose",
    )
    wrist_head.visual(
        Box((0.06, 0.08, 0.04)),
        origin=Origin(xyz=(0.29, 0.0, -0.08)),
        material=joint_dark,
        name="tool_mount",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.36, 0.16, 0.18)),
        mass=6.0,
        origin=Origin(xyz=(0.24, 0.0, -0.02)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder_stage,
        child=forearm,
        origin=Origin(xyz=(0.62, 0.0, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-1.2, upper=1.15),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.63, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder_stage = object_model.get_part("shoulder_stage")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    ctx.expect_gap(
        shoulder_stage,
        base,
        axis="z",
        positive_elem="shoulder_turntable",
        negative_elem="slew_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder turntable seats on the pedestal ring",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder_joint: pi / 2.0}):
        yawed_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings the arm around the pedestal",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and yawed_wrist_pos[1] > 0.55
        and abs(yawed_wrist_pos[0]) < 0.08,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({elbow_joint: 0.9}):
        raised_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow positive motion raises the wrist head",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.28,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    wrist_origin_rest = ctx.part_world_position(wrist_head)
    tool_aabb_rest = ctx.part_element_world_aabb(wrist_head, elem="tool_mount")
    with ctx.pose({wrist_joint: pi / 2.0}):
        wrist_origin_rolled = ctx.part_world_position(wrist_head)
        tool_aabb_rolled = ctx.part_element_world_aabb(wrist_head, elem="tool_mount")

    def _center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    tool_center_rest = _center(tool_aabb_rest)
    tool_center_rolled = _center(tool_aabb_rolled)
    ctx.check(
        "wrist roll carries the tool mount around the forearm axis",
        wrist_origin_rest is not None
        and wrist_origin_rolled is not None
        and tool_center_rest is not None
        and tool_center_rolled is not None
        and abs(wrist_origin_rest[0] - wrist_origin_rolled[0]) < 1e-6
        and tool_center_rest[2] < wrist_origin_rest[2] - 0.05
        and tool_center_rolled[1] > wrist_origin_rolled[1] + 0.05,
        details=(
            f"wrist_rest={wrist_origin_rest}, wrist_rolled={wrist_origin_rolled}, "
            f"tool_rest={tool_center_rest}, tool_rolled={tool_center_rolled}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

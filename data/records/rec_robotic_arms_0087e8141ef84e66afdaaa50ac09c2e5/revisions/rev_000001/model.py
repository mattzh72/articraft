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
    model = ArticulatedObject(name="pick_and_place_robotic_arm")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.91, 0.46, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    tool_black = model.material("tool_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="base_foot",
    )
    pedestal_base.visual(
        Cylinder(radius=0.072, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=graphite,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.10, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=dark_steel,
        name="shoulder_seat",
    )
    pedestal_base.visual(
        Box((0.20, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=graphite,
        name="column_cap",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.42),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )

    shoulder_assembly = model.part("shoulder_assembly")
    shoulder_assembly.visual(
        Cylinder(radius=0.085, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machine_gray,
        name="turret",
    )
    shoulder_assembly.visual(
        Box((0.16, 0.18, 0.16)),
        origin=Origin(xyz=(0.06, 0.0, 0.08)),
        material=machine_gray,
        name="shoulder_housing",
    )
    shoulder_assembly.visual(
        Box((0.32, 0.09, 0.08)),
        origin=Origin(xyz=(0.24, 0.0, 0.05)),
        material=accent_orange,
        name="upper_arm_body",
    )
    shoulder_assembly.visual(
        Box((0.20, 0.11, 0.04)),
        origin=Origin(xyz=(0.22, 0.0, 0.10)),
        material=machine_gray,
        name="upper_arm_cover",
    )
    shoulder_assembly.visual(
        Box((0.06, 0.012, 0.12)),
        origin=Origin(xyz=(0.40, 0.043, 0.04)),
        material=machine_gray,
        name="elbow_left_cheek",
    )
    shoulder_assembly.visual(
        Box((0.06, 0.012, 0.12)),
        origin=Origin(xyz=(0.40, -0.043, 0.04)),
        material=machine_gray,
        name="elbow_right_cheek",
    )
    shoulder_assembly.visual(
        Cylinder(radius=0.018, length=0.074),
        origin=Origin(xyz=(0.375, 0.0, 0.04), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_pin_boss",
    )
    shoulder_assembly.inertial = Inertial.from_geometry(
        Box((0.45, 0.19, 0.16)),
        mass=34.0,
        origin=Origin(xyz=(0.20, 0.0, 0.07)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.06, 0.065, 0.10)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=machine_gray,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.28, 0.08, 0.07)),
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
        material=accent_orange,
        name="forearm_body",
    )
    forearm.visual(
        Box((0.18, 0.05, 0.04)),
        origin=Origin(xyz=(0.17, 0.0, -0.045)),
        material=machine_gray,
        name="forearm_rib",
    )
    forearm.visual(
        Box((0.08, 0.07, 0.012)),
        origin=Origin(xyz=(0.34, 0.0, 0.036)),
        material=machine_gray,
        name="wrist_top_guard",
    )
    forearm.visual(
        Box((0.08, 0.07, 0.012)),
        origin=Origin(xyz=(0.34, 0.0, -0.036)),
        material=machine_gray,
        name="wrist_bottom_guard",
    )
    forearm.visual(
        Box((0.04, 0.09, 0.05)),
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_mount_block",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.12)),
        mass=20.0,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.024, length=0.06),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_barrel",
    )
    wrist_head.visual(
        Box((0.07, 0.055, 0.05)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=machine_gray,
        name="wrist_shell",
    )
    wrist_head.visual(
        Box((0.012, 0.04, 0.04)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=dark_steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.086, 0.0, -0.039)),
        material=tool_black,
        name="vacuum_stem",
    )
    wrist_head.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.086, 0.0, -0.056)),
        material=tool_black,
        name="vacuum_pad",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.11, 0.06, 0.07)),
        mass=4.0,
        origin=Origin(xyz=(0.04, 0.0, -0.01)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=shoulder_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.6,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder_assembly,
        child=forearm,
        origin=Origin(xyz=(0.40, 0.0, 0.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.25,
        ),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    shoulder_assembly = object_model.get_part("shoulder_assembly")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    ctx.check(
        "shoulder rotates about vertical axis",
        shoulder_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={shoulder_joint.axis}",
    )
    ctx.check(
        "elbow pitches about horizontal axis",
        elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={elbow_joint.axis}",
    )
    ctx.check(
        "wrist rolls about forearm axis",
        wrist_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={wrist_joint.axis}",
    )

    ctx.expect_gap(
        shoulder_assembly,
        pedestal_base,
        axis="z",
        positive_elem="turret",
        negative_elem="shoulder_seat",
        max_gap=0.001,
        max_penetration=1e-6,
        name="shoulder turret seats on pedestal",
    )
    ctx.expect_overlap(
        shoulder_assembly,
        pedestal_base,
        axes="xy",
        elem_a="turret",
        elem_b="shoulder_seat",
        min_overlap=0.16,
        name="shoulder turret stays centered on pedestal",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder_joint: 0.9}):
        yawed_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings the working envelope in plan view",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and abs(yawed_wrist_pos[1] - rest_wrist_pos[1]) > 0.28,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({elbow_joint: 0.85}):
        raised_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow lift raises the wrist head",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.12,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    def _part_size(part) -> tuple[float, float, float] | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return tuple(aabb[1][index] - aabb[0][index] for index in range(3))

    shoulder_size = _part_size(shoulder_assembly)
    forearm_size = _part_size(forearm)
    wrist_size = _part_size(wrist_head)
    ctx.check(
        "wrist head remains compact relative to the arm links",
        shoulder_size is not None
        and forearm_size is not None
        and wrist_size is not None
        and wrist_size[0] < shoulder_size[0] * 0.40
        and wrist_size[0] < forearm_size[0] * 0.50
        and wrist_size[1] < 0.08
        and wrist_size[2] < 0.10,
        details=f"shoulder={shoulder_size}, forearm={forearm_size}, wrist={wrist_size}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

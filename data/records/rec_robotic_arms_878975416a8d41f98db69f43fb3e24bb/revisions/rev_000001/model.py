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


def _x_cylinder(part, *, radius: float, length: float, xyz: tuple[float, float, float], material, name: str | None = None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(part, *, radius: float, length: float, xyz: tuple[float, float, float], material, name: str | None = None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_pick_place_arm")

    base_paint = model.material("base_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.83, 0.84, 0.86, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.32, 0.35, 0.38, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.58, 0.60, 0.63, 1.0))
    accent = model.material("accent", rgba=(0.92, 0.59, 0.16, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.22, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_paint,
        name="base_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=base_paint,
        name="lower_pedestal",
    )
    pedestal.visual(
        Cylinder(radius=0.10, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=steel_dark,
        name="main_column",
    )
    pedestal.visual(
        Cylinder(radius=0.13, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=steel_mid,
        name="shoulder_table",
    )
    pedestal.visual(
        Box((0.12, 0.07, 0.10)),
        origin=Origin(xyz=(-0.06, 0.0, 0.11)),
        material=steel_dark,
        name="service_access",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.44, 0.44, 0.38)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=arm_paint,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.09, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=steel_mid,
        name="shoulder_cap",
    )
    upper_arm.visual(
        Box((0.10, 0.14, 0.12)),
        origin=Origin(xyz=(-0.02, 0.0, 0.06)),
        material=steel_dark,
        name="shoulder_drive",
    )
    upper_arm.visual(
        Box((0.12, 0.16, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, 0.04)),
        material=arm_paint,
        name="shoulder_bridge",
    )
    _x_cylinder(
        upper_arm,
        radius=0.018,
        length=0.28,
        xyz=(0.20, 0.055, 0.04),
        material=steel_dark,
        name="left_upper_rail",
    )
    _x_cylinder(
        upper_arm,
        radius=0.018,
        length=0.28,
        xyz=(0.20, -0.055, 0.04),
        material=steel_dark,
        name="right_upper_rail",
    )
    upper_arm.visual(
        Box((0.06, 0.024, 0.12)),
        origin=Origin(xyz=(0.34, 0.058, 0.04)),
        material=arm_paint,
        name="left_elbow_cheek",
    )
    upper_arm.visual(
        Box((0.06, 0.024, 0.12)),
        origin=Origin(xyz=(0.34, -0.058, 0.04)),
        material=arm_paint,
        name="right_elbow_cheek",
    )
    upper_arm.visual(
        Box((0.09, 0.03, 0.05)),
        origin=Origin(xyz=(0.12, 0.0, 0.095)),
        material=accent,
        name="upper_status_block",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.44, 0.20, 0.14)),
        mass=18.0,
        origin=Origin(xyz=(0.14, 0.0, 0.06)),
    )

    forearm = model.part("forearm")
    _y_cylinder(
        forearm,
        radius=0.034,
        length=0.09,
        xyz=(0.0, 0.0, 0.0),
        material=steel_mid,
        name="elbow_torque_tube",
    )
    forearm.visual(
        Box((0.09, 0.07, 0.09)),
        origin=Origin(xyz=(0.045, 0.0, 0.015)),
        material=steel_dark,
        name="elbow_drive",
    )
    _x_cylinder(
        forearm,
        radius=0.014,
        length=0.30,
        xyz=(0.17, 0.038, 0.0),
        material=steel_dark,
        name="left_forearm_rail",
    )
    _x_cylinder(
        forearm,
        radius=0.014,
        length=0.30,
        xyz=(0.17, -0.038, 0.0),
        material=steel_dark,
        name="right_forearm_rail",
    )
    _x_cylinder(
        forearm,
        radius=0.009,
        length=0.28,
        xyz=(0.17, 0.0, -0.03),
        material=arm_paint,
        name="lower_tie",
    )
    _x_cylinder(
        forearm,
        radius=0.009,
        length=0.24,
        xyz=(0.20, 0.0, 0.035),
        material=accent,
        name="upper_tie",
    )
    forearm.visual(
        Box((0.05, 0.09, 0.08)),
        origin=Origin(xyz=(0.335, 0.0, 0.0)),
        material=arm_paint,
        name="wrist_mount",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.12)),
        mass=9.5,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    _x_cylinder(
        wrist_head,
        radius=0.032,
        length=0.08,
        xyz=(0.04, 0.0, 0.0),
        material=steel_mid,
        name="wrist_drive",
    )
    wrist_head.visual(
        Box((0.06, 0.09, 0.08)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=arm_paint,
        name="wrist_body",
    )
    _x_cylinder(
        wrist_head,
        radius=0.045,
        length=0.014,
        xyz=(0.127, 0.0, 0.0),
        material=steel_mid,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.020, 0.050, 0.050)),
        origin=Origin(xyz=(0.141, 0.0, 0.0)),
        material=steel_dark,
        name="tool_nose",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.10, 0.10)),
        mass=2.8,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.37, 0.0, 0.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-0.30, upper=2.20),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.5, lower=-3.0, upper=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    shoulder = object_model.get_articulation("pedestal_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist = object_model.get_articulation("forearm_to_wrist")

    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="z",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_table",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder turntable seats on pedestal",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=0.34,
        max_gap=0.40,
        name="elbow sits near the upper arm tip",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow: 1.20}):
        raised_wrist_pos = ctx.part_world_position(wrist_head)
        ctx.expect_gap(
            wrist_head,
            pedestal,
            axis="z",
            min_gap=0.10,
            name="raised wrist clears pedestal",
        )
    ctx.check(
        "elbow raises the wrist",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.18,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    with ctx.pose({shoulder: 1.0}):
        swung_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder swings the arm laterally",
        rest_wrist_pos is not None
        and swung_wrist_pos is not None
        and swung_wrist_pos[1] > rest_wrist_pos[1] + 0.30,
        details=f"rest={rest_wrist_pos}, swung={swung_wrist_pos}",
    )

    with ctx.pose({wrist: 2.0}):
        rolled_wrist_origin = ctx.part_world_position(wrist_head)
    ctx.check(
        "wrist roll stays centered on the forearm axis",
        rest_wrist_pos is not None
        and rolled_wrist_origin is not None
        and abs(rolled_wrist_origin[0] - rest_wrist_pos[0]) < 1e-6
        and abs(rolled_wrist_origin[1] - rest_wrist_pos[1]) < 1e-6
        and abs(rolled_wrist_origin[2] - rest_wrist_pos[2]) < 1e-6,
        details=f"rest={rest_wrist_pos}, rolled={rolled_wrist_origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

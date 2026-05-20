from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _cyl_x(part, *, radius: float, length: float, xyz, material: str, name: str) -> None:
    """Add a cylinder whose axis follows local +X."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, *, radius: float, length: float, xyz, material: str, name: str) -> None:
    """Add a cylinder whose axis follows local +Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _diagonal_box(part, start, end, *, thickness: float, material: str, name: str) -> None:
    """A slim rectangular truss member between two points in the local XY plane."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    length = sqrt(dx * dx + dy * dy)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, 0.0, atan2(dy, dx)),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_pick_place_arm")

    model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("painted_base", rgba=(0.12, 0.14, 0.16, 1.0))
    model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("blue_anodized", rgba=(0.08, 0.22, 0.75, 1.0))
    model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("warning_yellow", rgba=(0.95, 0.70, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.28, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="painted_base",
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.29),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material="painted_base",
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.16, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3225)),
        material="dark_steel",
        name="top_bearing_ring",
    )
    for idx, (x, y) in enumerate(
        ((0.19, 0.19), (-0.19, 0.19), (-0.19, -0.19), (0.19, -0.19))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.051)),
            material="dark_steel",
            name=f"anchor_bolt_{idx}",
        )

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Cylinder(radius=0.155, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="dark_steel",
        name="rotary_table",
    )
    shoulder_housing.visual(
        Box((0.16, 0.22, 0.31)),
        origin=Origin(xyz=(-0.005, 0.0, 0.235)),
        material="blue_anodized",
        name="upright_spine",
    )
    shoulder_housing.visual(
        Box((0.10, 0.24, 0.18)),
        origin=Origin(xyz=(0.10, 0.0, 0.34)),
        material="blue_anodized",
        name="front_mount",
    )
    shoulder_housing.visual(
        Box((0.06, 0.12, 0.05)),
        origin=Origin(xyz=(-0.115, 0.0, 0.33)),
        material="dark_steel",
        name="rear_motor_cap",
    )
    _cyl_y(
        shoulder_housing,
        radius=0.045,
        length=0.25,
        xyz=(0.105, 0.0, 0.37),
        material="dark_steel",
        name="shoulder_cross_boss",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.08, 0.20, 0.18)),
        origin=Origin(xyz=(0.19, 0.0, 0.37)),
        material="brushed_aluminum",
        name="root_plate",
    )
    _cyl_y(
        upper_arm,
        radius=0.044,
        length=0.20,
        xyz=(0.23, 0.0, 0.37),
        material="dark_steel",
        name="root_cross_tube",
    )
    for side, y in enumerate((-0.065, 0.065)):
        _cyl_x(
            upper_arm,
            radius=0.018,
            length=0.49,
            xyz=(0.475, y, 0.37),
            material="brushed_aluminum",
            name=f"side_rail_{side}",
        )
    _cyl_y(
        upper_arm,
        radius=0.014,
        length=0.16,
        xyz=(0.36, 0.0, 0.37),
        material="brushed_aluminum",
        name="rail_spacer_0",
    )
    _cyl_y(
        upper_arm,
        radius=0.014,
        length=0.16,
        xyz=(0.59, 0.0, 0.37),
        material="brushed_aluminum",
        name="rail_spacer_1",
    )
    _diagonal_box(
        upper_arm,
        (0.31, -0.065, 0.37),
        (0.64, 0.065, 0.37),
        thickness=0.014,
        material="warning_yellow",
        name="truss_brace_0",
    )
    _diagonal_box(
        upper_arm,
        (0.31, 0.065, 0.37),
        (0.64, -0.065, 0.37),
        thickness=0.014,
        material="warning_yellow",
        name="truss_brace_1",
    )
    for side, y in enumerate((-0.09, 0.09)):
        _cyl_y(
            upper_arm,
            radius=0.065,
            length=0.04,
            xyz=(0.78, y, 0.37),
            material="dark_steel",
            name=f"elbow_boss_{side}",
        )

    forearm = model.part("forearm")
    _cyl_y(
        forearm,
        radius=0.055,
        length=0.140,
        xyz=(0.0, 0.0, 0.0),
        material="dark_steel",
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.10, 0.13, 0.07)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material="brushed_aluminum",
        name="elbow_clamp",
    )
    for side, y in enumerate((-0.045, 0.045)):
        _cyl_x(
            forearm,
            radius=0.015,
            length=0.42,
            xyz=(0.29, y, 0.0),
            material="brushed_aluminum",
            name=f"forearm_rail_{side}",
        )
    _diagonal_box(
        forearm,
        (0.13, -0.045, 0.0),
        (0.43, 0.045, 0.0),
        thickness=0.011,
        material="warning_yellow",
        name="forearm_brace_0",
    )
    _diagonal_box(
        forearm,
        (0.13, 0.045, 0.0),
        (0.43, -0.045, 0.0),
        thickness=0.011,
        material="warning_yellow",
        name="forearm_brace_1",
    )
    _cyl_y(
        forearm,
        radius=0.012,
        length=0.115,
        xyz=(0.25, 0.0, 0.0),
        material="brushed_aluminum",
        name="forearm_spacer_0",
    )
    _cyl_x(
        forearm,
        radius=0.045,
        length=0.12,
        xyz=(0.54, 0.0, 0.0),
        material="dark_steel",
        name="wrist_sleeve",
    )

    wrist_head = model.part("wrist_head")
    _cyl_x(
        wrist_head,
        radius=0.040,
        length=0.09,
        xyz=(0.045, 0.0, 0.0),
        material="dark_steel",
        name="roll_shaft",
    )
    wrist_head.visual(
        Box((0.055, 0.11, 0.11)),
        origin=Origin(xyz=(0.1175, 0.0, 0.0)),
        material="blue_anodized",
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.035, 0.020, 0.035)),
        origin=Origin(xyz=(0.118, 0.0, 0.0725)),
        material="warning_yellow",
        name="index_mark",
    )
    _cyl_x(
        wrist_head,
        radius=0.020,
        length=0.08,
        xyz=(0.185, 0.0, 0.0),
        material="black_rubber",
        name="vacuum_nozzle",
    )
    _cyl_x(
        wrist_head,
        radius=0.034,
        length=0.018,
        xyz=(0.234, 0.0, 0.0),
        material="black_rubber",
        name="suction_cup",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.8, lower=-pi, upper=pi),
    )
    model.articulation(
        "housing_to_upper_arm",
        ArticulationType.FIXED,
        parent=shoulder_housing,
        child=upper_arm,
        origin=Origin(),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.78, 0.0, 0.37)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=-1.1, upper=1.6),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder_housing")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_head")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    moving_revolutes = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "three actuated revolute joints",
        len(moving_revolutes) == 3,
        details=f"revolute joints={[joint.name for joint in moving_revolutes]}",
    )

    with ctx.pose({shoulder_yaw: 0.0, elbow_pitch: 0.0, wrist_roll: 0.0}):
        ctx.expect_contact(
            shoulder,
            base,
            elem_a="rotary_table",
            elem_b="top_bearing_ring",
            contact_tol=0.001,
            name="rotary table seats on pedestal bearing",
        )
        ctx.expect_contact(
            upper_arm,
            shoulder,
            elem_a="root_plate",
            elem_b="front_mount",
            contact_tol=0.001,
            name="upper arm bolts to shoulder housing",
        )
        ctx.expect_gap(
            upper_arm,
            forearm,
            axis="y",
            positive_elem="elbow_boss_1",
            negative_elem="elbow_barrel",
            max_gap=0.001,
            max_penetration=0.0001,
            name="elbow bearing cheek supports barrel",
        )
        ctx.expect_contact(
            wrist,
            forearm,
            elem_a="roll_shaft",
            elem_b="wrist_sleeve",
            contact_tol=0.001,
            name="wrist shaft seats against forearm sleeve",
        )

    rest_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({elbow_pitch: 0.9}):
        raised_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "positive elbow pitch raises wrist",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.20,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    rest_forearm_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder_yaw: 0.8}):
        yawed_forearm_pos = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder yaw swings arm around pedestal",
        rest_forearm_pos is not None
        and yawed_forearm_pos is not None
        and yawed_forearm_pos[1] > rest_forearm_pos[1] + 0.45,
        details=f"rest={rest_forearm_pos}, yawed={yawed_forearm_pos}",
    )

    rest_mark = ctx.part_element_world_aabb(wrist, elem="index_mark")
    with ctx.pose({wrist_roll: 1.2}):
        rolled_mark = ctx.part_element_world_aabb(wrist, elem="index_mark")
    if rest_mark is not None and rolled_mark is not None:
        rest_center_y = (rest_mark[0][1] + rest_mark[1][1]) / 2.0
        rolled_center_y = (rolled_mark[0][1] + rolled_mark[1][1]) / 2.0
        rest_center_z = (rest_mark[0][2] + rest_mark[1][2]) / 2.0
        rolled_center_z = (rolled_mark[0][2] + rolled_mark[1][2]) / 2.0
        roll_ok = (
            abs(rolled_center_y - rest_center_y) > 0.04
            and abs(rolled_center_z - rest_center_z) > 0.015
        )
    else:
        roll_ok = False
    ctx.check(
        "wrist roll rotates off-axis index mark",
        roll_ok,
        details=f"rest_mark={rest_mark}, rolled_mark={rolled_mark}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_pick_place_robot_arm")

    base_grey = model.material("base_grey", rgba=(0.34, 0.37, 0.40, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.89, 0.46, 0.16, 1.0))
    wrist_silver = model.material("wrist_silver", rgba=(0.72, 0.75, 0.79, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.26, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=base_grey,
        name="pedestal_foot",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=housing_dark,
        name="pedestal_column",
    )
    base.visual(
        Box((0.14, 0.13, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=base_grey,
        name="shoulder_housing",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=housing_dark,
        name="shoulder_skirt",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=trim_black,
        name="shoulder_top",
    )
    base.visual(
        Box((0.09, 0.05, 0.05)),
        origin=Origin(xyz=(-0.015, -0.055, 0.265)),
        material=housing_dark,
        name="rear_service_box",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.26, 0.20, 0.35)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )

    shoulder_height = 0.35

    upper_arm = model.part("upper_arm")
    upper_arm_shell = section_loft(
        [
            _yz_section(0.0, 0.090, 0.110, 0.022, z_center=0.060),
            _yz_section(0.090, 0.082, 0.095, 0.020, z_center=0.050),
            _yz_section(0.195, 0.064, 0.074, 0.016, z_center=0.038),
        ]
    )
    upper_arm.visual(
        mesh_from_geometry(upper_arm_shell, "upper_arm_shell"),
        material=arm_orange,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Cylinder(radius=0.048, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=housing_dark,
        name="shoulder_motor_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.044, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=housing_dark,
        name="shoulder_bearing_plate",
    )
    upper_arm.visual(
        Cylinder(radius=0.040, length=0.072),
        origin=Origin(xyz=(0.155, 0.0, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_dark,
        name="elbow_joint_housing",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.12)),
        mass=4.8,
        origin=Origin(xyz=(0.10, 0.0, 0.060)),
    )

    forearm = model.part("forearm")
    forearm_shell = section_loft(
        [
            _yz_section(0.0, 0.060, 0.070, 0.015, z_center=0.0),
            _yz_section(0.075, 0.056, 0.062, 0.013, z_center=-0.002),
            _yz_section(0.160, 0.050, 0.054, 0.012, z_center=-0.004),
        ]
    )
    forearm.visual(
        mesh_from_geometry(forearm_shell, "forearm_shell"),
        material=arm_orange,
        name="forearm_shell",
    )
    forearm.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.140, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_dark,
        name="wrist_drive_housing",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.17, 0.06, 0.07)),
        mass=2.8,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wrist_silver,
        name="wrist_roll_housing",
    )
    wrist_head.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_dark,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.022, 0.030, 0.018)),
        origin=Origin(xyz=(0.074, 0.0, -0.013)),
        material=trim_black,
        name="tool_block",
    )
    wrist_head.visual(
        Box((0.012, 0.040, 0.040)),
        origin=Origin(xyz=(0.091, 0.0, 0.0)),
        material=wrist_silver,
        name="face_plate",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.10, 0.05, 0.05)),
        mass=1.1,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.195, 0.0, 0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.2,
            lower=-math.radians(20.0),
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "forearm_to_wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.160, 0.0, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.0,
            lower=-math.radians(180.0),
            upper=math.radians(180.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    shoulder = object_model.get_articulation("base_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_roll = object_model.get_articulation("forearm_to_wrist_roll")

    ctx.expect_gap(
        upper_arm,
        base,
        axis="z",
        positive_elem="shoulder_bearing_plate",
        negative_elem="shoulder_top",
        max_penetration=1e-6,
        max_gap=0.001,
        name="upper arm bearing plate seats on the shoulder top cap",
    )
    ctx.expect_overlap(
        upper_arm,
        base,
        axes="xy",
        elem_a="upper_arm_shell",
        elem_b="shoulder_top",
        min_overlap=0.050,
        name="upper arm sits over the shoulder bearing",
    )
    ctx.expect_gap(
        wrist_head,
        base,
        axis="x",
        min_gap=0.22,
        name="wrist head projects forward of the pedestal in home pose",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder: math.radians(55.0)}):
        yawed_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw sweeps the arm laterally",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and yawed_wrist_pos[1] > rest_wrist_pos[1] + 0.10,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({elbow: math.radians(70.0)}):
        lifted_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow lift raises the wrist head",
        rest_wrist_pos is not None
        and lifted_wrist_pos is not None
        and lifted_wrist_pos[2] > rest_wrist_pos[2] + 0.08,
        details=f"rest={rest_wrist_pos}, lifted={lifted_wrist_pos}",
    )

    tool_block_rest = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_block"))
    with ctx.pose({wrist_roll: math.radians(90.0)}):
        tool_block_rolled = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_block"))
    ctx.check(
        "wrist roll spins the tool block about the forearm axis",
        tool_block_rest is not None
        and tool_block_rolled is not None
        and tool_block_rolled[1] > tool_block_rest[1] + 0.010,
        details=f"rest={tool_block_rest}, rolled={tool_block_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

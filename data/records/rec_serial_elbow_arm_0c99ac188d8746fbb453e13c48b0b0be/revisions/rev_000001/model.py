from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_elbow_arm")

    dark_steel = model.material("dark_blued_steel", rgba=(0.05, 0.07, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    tool_gray = model.material("tool_face_gray", rgba=(0.38, 0.40, 0.42, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.48, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=satin_steel,
        name="round_column",
    )
    pedestal.visual(
        Box((0.20, 0.20, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.645)),
        material=dark_steel,
        name="fork_base",
    )
    for cheek_name, cap_name, y in (
        ("shoulder_cheek_near", "shoulder_bearing_cap_near", -0.065),
        ("shoulder_cheek_far", "shoulder_bearing_cap_far", 0.065),
    ):
        pedestal.visual(
            Box((0.150, 0.030, 0.240)),
            origin=Origin(xyz=(0.0, y, 0.750)),
            material=dark_steel,
            name=cheek_name,
        )
        pedestal.visual(
            Cylinder(radius=0.045, length=0.020),
            origin=Origin(xyz=(0.0, y * 1.23, 0.750), rpy=(-pi / 2, 0.0, 0.0)),
            material=black_rubber,
            name=cap_name,
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.055, length=0.100),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=satin_steel,
        name="shoulder_hub",
    )
    upper_link.visual(
        Box((0.380, 0.060, 0.055)),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material=safety_yellow,
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.080, 0.180, 0.055)),
        origin=Origin(xyz=(0.435, 0.0, 0.0)),
        material=safety_yellow,
        name="elbow_yoke_bridge",
    )
    for cheek_name, cap_name, y in (
        ("elbow_cheek_near", "elbow_bearing_cap_near", -0.060),
        ("elbow_cheek_far", "elbow_bearing_cap_far", 0.060),
    ):
        upper_link.visual(
            Box((0.160, 0.030, 0.140)),
            origin=Origin(xyz=(0.550, y, 0.0)),
            material=safety_yellow,
            name=cheek_name,
        )
        upper_link.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(0.550, y * 1.40, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=black_rubber,
            name=cap_name,
        )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=0.048, length=0.090),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=satin_steel,
        name="elbow_hub",
    )
    distal_link.visual(
        Box((0.360, 0.050, 0.045)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=safety_yellow,
        name="distal_beam",
    )
    distal_link.visual(
        Box((0.050, 0.175, 0.120)),
        origin=Origin(xyz=(0.425, 0.0, 0.0)),
        material=tool_gray,
        name="tool_block",
    )
    distal_link.visual(
        Box((0.010, 0.155, 0.095)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=black_rubber,
        name="tool_face",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.750)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.4, lower=-1.10, upper=1.00),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=distal_link,
        origin=Origin(xyz=(0.550, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.8, lower=-1.55, upper=1.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    distal_link = object_model.get_part("distal_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

    def _parallel_y(axis: tuple[float, float, float]) -> bool:
        return abs(axis[0]) < 1e-6 and abs(abs(axis[1]) - 1.0) < 1e-6 and abs(axis[2]) < 1e-6

    ctx.check(
        "two serial revolute joints",
        shoulder_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and shoulder_joint.parent == "pedestal"
        and shoulder_joint.child == "upper_link"
        and elbow_joint.parent == "upper_link"
        and elbow_joint.child == "distal_link",
        details=f"shoulder={shoulder_joint}, elbow={elbow_joint}",
    )
    ctx.check(
        "joint axes are parallel",
        _parallel_y(shoulder_joint.axis) and _parallel_y(elbow_joint.axis),
        details=f"shoulder_axis={shoulder_joint.axis}, elbow_axis={elbow_joint.axis}",
    )

    ctx.expect_contact(
        upper_link,
        pedestal,
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_near",
        name="shoulder hub is carried by near pedestal cheek",
    )
    ctx.expect_contact(
        upper_link,
        pedestal,
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_far",
        name="shoulder hub is carried by far pedestal cheek",
    )
    ctx.expect_contact(
        distal_link,
        upper_link,
        elem_a="elbow_hub",
        elem_b="elbow_cheek_near",
        name="elbow hub is carried by near upper-link cheek",
    )
    ctx.expect_contact(
        distal_link,
        upper_link,
        elem_a="elbow_hub",
        elem_b="elbow_cheek_far",
        name="elbow hub is carried by far upper-link cheek",
    )

    rest_elbow_pos = ctx.part_world_position(distal_link)
    with ctx.pose({shoulder_joint: -0.55}):
        raised_elbow_pos = ctx.part_world_position(distal_link)
    ctx.check(
        "shoulder joint raises elbow in vertical plane",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.20
        and abs(raised_elbow_pos[1] - rest_elbow_pos[1]) < 1e-6,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_tool_aabb = ctx.part_element_world_aabb(distal_link, elem="tool_face")
    with ctx.pose({elbow_joint: -0.75}):
        raised_tool_aabb = ctx.part_element_world_aabb(distal_link, elem="tool_face")
    rest_tool_z = rest_tool_aabb[1][2] if rest_tool_aabb is not None else None
    raised_tool_z = raised_tool_aabb[1][2] if raised_tool_aabb is not None else None
    ctx.check(
        "elbow joint swings tool face upward",
        rest_tool_z is not None and raised_tool_z is not None and raised_tool_z > rest_tool_z + 0.20,
        details=f"rest_tool_z={rest_tool_z}, raised_tool_z={raised_tool_z}",
    )

    return ctx.report()


object_model = build_object_model()

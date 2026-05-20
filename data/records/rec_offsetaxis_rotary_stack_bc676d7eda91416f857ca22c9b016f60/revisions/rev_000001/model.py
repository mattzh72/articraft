from __future__ import annotations

import math

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
    model = ArticulatedObject(name="underslung_offset_rotary_assembly")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.19, 0.22, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.035, 0.037, 0.04, 1.0))
    lower_blue = model.material("lower_blue", rgba=(0.05, 0.22, 0.60, 1.0))
    upper_orange = model.material("upper_orange", rgba=(0.88, 0.42, 0.10, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.015, 0.015, 0.014, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.72, 0.20, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=painted_steel,
        name="top_bridge",
    )
    bridge.visual(
        Box((0.095, 0.160, 0.200)),
        origin=Origin(xyz=(-0.20, 0.0, 0.425)),
        material=painted_steel,
        name="lower_hanger",
    )
    bridge.visual(
        Box((0.080, 0.140, 0.105)),
        origin=Origin(xyz=(0.17, 0.0, 0.4725)),
        material=painted_steel,
        name="upper_hanger",
    )
    bridge.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(-0.20, 0.0, 0.340)),
        material=dark_bearing,
        name="lower_bearing_face",
    )
    bridge.visual(
        Cylinder(radius=0.050, length=0.032),
        origin=Origin(xyz=(0.17, 0.0, 0.436)),
        material=dark_bearing,
        name="upper_bearing_face",
    )
    bridge.visual(
        Box((0.22, 0.028, 0.052)),
        origin=Origin(xyz=(-0.20, 0.094, 0.425)),
        material=painted_steel,
        name="lower_front_web",
    )
    bridge.visual(
        Box((0.22, 0.028, 0.052)),
        origin=Origin(xyz=(-0.20, -0.094, 0.425)),
        material=painted_steel,
        name="lower_rear_web",
    )
    bridge.visual(
        Box((0.18, 0.024, 0.044)),
        origin=Origin(xyz=(0.17, 0.082, 0.472)),
        material=painted_steel,
        name="upper_front_web",
    )
    bridge.visual(
        Box((0.18, 0.024, 0.044)),
        origin=Origin(xyz=(0.17, -0.082, 0.472)),
        material=painted_steel,
        name="upper_rear_web",
    )
    for i, (x, y) in enumerate(
        ((-0.31, -0.075), (-0.31, 0.075), (0.31, -0.075), (0.31, 0.075))
    ):
        bridge.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.582)),
            material=bolt_black,
            name=f"bridge_bolt_{i}",
        )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.047, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=shaft_steel,
        name="lower_thrust_washer",
    )
    lower_stage.visual(
        Cylinder(radius=0.024, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=shaft_steel,
        name="lower_shaft",
    )
    lower_stage.visual(
        Cylinder(radius=0.052, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.123)),
        material=shaft_steel,
        name="lower_hub",
    )
    lower_stage.visual(
        Cylinder(radius=0.155, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.159)),
        material=lower_blue,
        name="lower_rotary_table",
    )
    lower_stage.visual(
        Box((0.260, 0.052, 0.026)),
        origin=Origin(xyz=(0.130, 0.0, -0.159)),
        material=lower_blue,
        name="lower_radial_arm",
    )
    lower_stage.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.265, 0.0, -0.159)),
        material=shaft_steel,
        name="lower_end_pad",
    )
    lower_stage.visual(
        Box((0.070, 0.024, 0.014)),
        origin=Origin(xyz=(-0.070, 0.084, -0.138)),
        material=shaft_steel,
        name="lower_index_lug",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=shaft_steel,
        name="upper_thrust_washer",
    )
    upper_stage.visual(
        Cylinder(radius=0.018, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, -0.0325)),
        material=shaft_steel,
        name="upper_shaft",
    )
    upper_stage.visual(
        Cylinder(radius=0.040, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=shaft_steel,
        name="upper_hub",
    )
    upper_stage.visual(
        Cylinder(radius=0.090, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
        material=upper_orange,
        name="upper_rotary_plate",
    )
    upper_stage.visual(
        Box((0.150, 0.040, 0.020)),
        origin=Origin(xyz=(0.075, 0.0, -0.108)),
        material=upper_orange,
        name="upper_radial_arm",
    )
    upper_stage.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.152, 0.0, -0.108)),
        material=shaft_steel,
        name="upper_end_pad",
    )

    model.articulation(
        "bridge_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=lower_stage,
        origin=Origin(xyz=(-0.20, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "bridge_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=upper_stage,
        origin=Origin(xyz=(0.17, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("bridge_to_lower_stage")
    upper_joint = object_model.get_articulation("bridge_to_upper_stage")

    ctx.check(
        "two independent revolute stages",
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE
        and lower_joint.child == "lower_stage"
        and upper_joint.child == "upper_stage",
        details=f"lower={lower_joint}, upper={upper_joint}",
    )
    ctx.check(
        "supported axes are parallel",
        tuple(lower_joint.axis) == tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower_joint.axis}, upper_axis={upper_joint.axis}",
    )
    ctx.expect_origin_distance(
        lower_stage,
        upper_stage,
        axes="x",
        min_dist=0.35,
        name="stages have offset parallel axes",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="z",
        min_gap=0.09,
        max_gap=0.11,
        name="upper stage axis hangs above lower stage axis",
    )
    ctx.expect_contact(
        lower_stage,
        bridge,
        elem_a="lower_thrust_washer",
        elem_b="lower_bearing_face",
        contact_tol=0.0005,
        name="lower stage is seated against its bearing",
    )
    ctx.expect_contact(
        upper_stage,
        bridge,
        elem_a="upper_thrust_washer",
        elem_b="upper_bearing_face",
        contact_tol=0.0005,
        name="upper stage is seated against its bearing",
    )

    lower_rest = ctx.part_element_world_aabb(lower_stage, elem="lower_end_pad")
    upper_rest = ctx.part_element_world_aabb(upper_stage, elem="upper_end_pad")
    with ctx.pose({lower_joint: math.pi / 2.0}):
        lower_turned = ctx.part_element_world_aabb(lower_stage, elem="lower_end_pad")
        upper_during_lower_turn = ctx.part_element_world_aabb(upper_stage, elem="upper_end_pad")
    with ctx.pose({upper_joint: math.pi / 2.0}):
        upper_turned = ctx.part_element_world_aabb(upper_stage, elem="upper_end_pad")

    def _center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    lower_rest_xy = _center_xy(lower_rest)
    lower_turned_xy = _center_xy(lower_turned)
    upper_rest_xy = _center_xy(upper_rest)
    upper_turned_xy = _center_xy(upper_turned)
    upper_during_lower_turn_xy = _center_xy(upper_during_lower_turn)

    ctx.check(
        "lower joint turns lower stage only",
        lower_rest_xy is not None
        and lower_turned_xy is not None
        and upper_rest_xy is not None
        and upper_during_lower_turn_xy is not None
        and lower_turned_xy[1] > lower_rest_xy[1] + 0.20
        and abs(upper_during_lower_turn_xy[0] - upper_rest_xy[0]) < 0.001
        and abs(upper_during_lower_turn_xy[1] - upper_rest_xy[1]) < 0.001,
        details=(
            f"lower_rest={lower_rest_xy}, lower_turned={lower_turned_xy}, "
            f"upper_rest={upper_rest_xy}, upper_during_lower={upper_during_lower_turn_xy}"
        ),
    )
    ctx.check(
        "upper joint turns smaller stage independently",
        upper_rest_xy is not None
        and upper_turned_xy is not None
        and upper_turned_xy[1] > upper_rest_xy[1] + 0.12,
        details=f"upper_rest={upper_rest_xy}, upper_turned={upper_turned_xy}",
    )

    return ctx.report()


object_model = build_object_model()

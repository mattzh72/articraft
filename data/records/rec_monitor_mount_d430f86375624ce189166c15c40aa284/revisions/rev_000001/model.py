from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_clamped_monitor_mount")

    dark = model.material("satin_black_powdercoat", color=(0.02, 0.022, 0.024, 1.0))
    metal = model.material("brushed_aluminum", color=(0.62, 0.64, 0.64, 1.0))
    rubber = model.material("matte_rubber", color=(0.005, 0.005, 0.004, 1.0))
    screw_metal = model.material("dark_zinc_screw", color=(0.12, 0.12, 0.115, 1.0))
    desk = model.material("pale_desk_edge_reference", color=(0.70, 0.52, 0.34, 1.0))
    hole_black = model.material("black_recesses", color=(0.0, 0.0, 0.0, 1.0))

    clamp_base = model.part("clamp_base")
    # Root frame is the shoulder bearing top plane.  The clamp hangs below it and
    # visibly bites onto a short desk-edge reference block.
    clamp_base.visual(
        Cylinder(radius=0.058, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark,
        name="shoulder_bearing",
    )
    clamp_base.visual(
        Cylinder(radius=0.035, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=dark,
        name="upright_post",
    )
    clamp_base.visual(
        Box((0.220, 0.160, 0.035)),
        origin=Origin(xyz=(-0.030, 0.0, -0.395)),
        material=dark,
        name="top_jaw",
    )
    clamp_base.visual(
        Box((0.040, 0.160, 0.220)),
        origin=Origin(xyz=(-0.130, 0.0, -0.490)),
        material=dark,
        name="clamp_spine",
    )
    clamp_base.visual(
        Box((0.180, 0.140, 0.035)),
        origin=Origin(xyz=(-0.040, 0.0, -0.600)),
        material=dark,
        name="lower_jaw",
    )
    clamp_base.visual(
        Box((0.200, 0.170, 0.050)),
        origin=Origin(xyz=(-0.020, 0.0, -0.4375)),
        material=desk,
        name="desk_edge",
    )
    clamp_base.visual(
        Cylinder(radius=0.012, length=0.176),
        origin=Origin(xyz=(0.030, 0.0, -0.554)),
        material=screw_metal,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(0.030, 0.0, -0.652)),
        material=rubber,
        name="clamp_knob",
    )
    clamp_base.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.030, 0.0, -0.466)),
        material=rubber,
        name="pressure_pad",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.052, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="shoulder_hub",
    )
    lower_arm.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=dark,
        name="shoulder_cap",
    )
    lower_arm.visual(
        Box((0.340, 0.055, 0.045)),
        origin=Origin(xyz=(0.180, 0.0, 0.0375)),
        material=metal,
        name="arm_beam",
    )
    lower_arm.visual(
        Box((0.260, 0.064, 0.012)),
        origin=Origin(xyz=(0.180, 0.0, 0.020)),
        material=dark,
        name="cable_channel",
    )
    lower_arm.visual(
        Cylinder(radius=0.052, length=0.060),
        origin=Origin(xyz=(0.360, 0.0, 0.030)),
        material=dark,
        name="elbow_hub",
    )
    lower_arm.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.360, 0.0, 0.055)),
        material=dark,
        name="elbow_cap",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="elbow_hub",
    )
    upper_arm.visual(
        Box((0.320, 0.052, 0.044)),
        origin=Origin(xyz=(0.170, 0.0, 0.030)),
        material=metal,
        name="arm_beam",
    )
    upper_arm.visual(
        Box((0.230, 0.060, 0.010)),
        origin=Origin(xyz=(0.170, 0.0, 0.055)),
        material=dark,
        name="top_cable_cover",
    )
    upper_arm.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.340, 0.0, 0.030)),
        material=dark,
        name="head_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.340, 0.0, 0.054)),
        material=dark,
        name="head_cap",
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Cylinder(radius=0.045, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=dark,
        name="swivel_bearing",
    )
    swivel_yoke.visual(
        Box((0.090, 0.060, 0.040)),
        origin=Origin(xyz=(0.045, 0.0, 0.035)),
        material=dark,
        name="neck_block",
    )
    swivel_yoke.visual(
        Box((0.035, 0.108, 0.035)),
        origin=Origin(xyz=(0.078, 0.0, 0.026)),
        material=dark,
        name="yoke_bridge",
    )
    swivel_yoke.visual(
        Box((0.055, 0.014, 0.090)),
        origin=Origin(xyz=(0.115, 0.047, 0.040)),
        material=dark,
        name="yoke_cheek_0",
    )
    swivel_yoke.visual(
        Box((0.055, 0.014, 0.090)),
        origin=Origin(xyz=(0.115, -0.047, 0.040)),
        material=dark,
        name="yoke_cheek_1",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        Cylinder(radius=0.017, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="tilt_barrel",
    )
    vesa_head.visual(
        Box((0.050, 0.040, 0.035)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=dark,
        name="tilt_boss",
    )
    vesa_head.visual(
        Box((0.014, 0.140, 0.140)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=dark,
        name="vesa_plate",
    )
    vesa_head.visual(
        Box((0.010, 0.060, 0.060)),
        origin=Origin(xyz=(0.039, 0.0, 0.0)),
        material=screw_metal,
        name="center_reinforcement",
    )
    for row, y in enumerate((-0.050, 0.050)):
        for col, z in enumerate((-0.050, 0.050)):
            vesa_head.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(
                    xyz=(0.058, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hole_black,
                name=f"vesa_hole_{row}_{col}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.7, upper=1.7),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.360, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-2.3, upper=2.3),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=swivel_yoke,
        origin=Origin(xyz=(0.340, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=vesa_head,
        origin=Origin(xyz=(0.115, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    swivel_yoke = object_model.get_part("swivel_yoke")
    vesa_head = object_model.get_part("vesa_head")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four requested revolute mechanisms",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, head_swivel, head_tilt)),
        details="Shoulder, elbow, head swivel, and head tilt must all be revolute joints.",
    )
    ctx.check(
        "vertical yaw axes",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0)
        and head_swivel.axis == (0.0, 0.0, 1.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, swivel={head_swivel.axis}",
    )
    ctx.check(
        "horizontal tilt axis",
        head_tilt.axis == (0.0, 1.0, 0.0),
        details=f"tilt axis={head_tilt.axis}",
    )

    ctx.expect_gap(
        lower_arm,
        clamp_base,
        axis="z",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder bearing stack is seated",
    )
    ctx.expect_gap(
        upper_arm,
        lower_arm,
        axis="z",
        positive_elem="elbow_hub",
        negative_elem="elbow_hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="elbow bearing stack is seated",
    )
    ctx.expect_gap(
        swivel_yoke,
        upper_arm,
        axis="z",
        positive_elem="swivel_bearing",
        negative_elem="head_hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="head swivel stack is seated",
    )
    ctx.expect_within(
        vesa_head,
        swivel_yoke,
        axes="yz",
        inner_elem="tilt_barrel",
        margin=0.002,
        name="tilt barrel sits inside the yoke span",
    )

    rest_upper = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.75}):
        swung_upper = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder swings lower arm in plan",
        rest_upper is not None
        and swung_upper is not None
        and abs(swung_upper[1] - rest_upper[1]) > 0.20,
        details=f"rest={rest_upper}, swung={swung_upper}",
    )

    rest_head = ctx.part_world_position(vesa_head)
    with ctx.pose({elbow: 0.75}):
        elbow_head = ctx.part_world_position(vesa_head)
    ctx.check(
        "elbow swings upper arm in plan",
        rest_head is not None
        and elbow_head is not None
        and abs(elbow_head[1] - rest_head[1]) > 0.25,
        details=f"rest={rest_head}, elbow_pose={elbow_head}",
    )

    with ctx.pose({head_swivel: 0.80}):
        swivel_head_pos = ctx.part_world_position(vesa_head)
    ctx.check(
        "head swivel yaws the VESA head",
        rest_head is not None
        and swivel_head_pos is not None
        and abs(swivel_head_pos[1] - rest_head[1]) > 0.06,
        details=f"rest={rest_head}, swivel_pose={swivel_head_pos}",
    )

    rest_aabb = ctx.part_world_aabb(vesa_head)
    with ctx.pose({head_tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(vesa_head)

    def x_extent(aabb):
        return None if aabb is None else aabb[1][0] - aabb[0][0]

    ctx.check(
        "head tilt pitches the plate about a horizontal pin",
        rest_aabb is not None
        and tilted_aabb is not None
        and x_extent(tilted_aabb) is not None
        and x_extent(rest_aabb) is not None
        and x_extent(tilted_aabb) > x_extent(rest_aabb) + 0.025,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

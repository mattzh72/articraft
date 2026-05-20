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
    model = ArticulatedObject(name="under_shelf_tv_wall_arm")

    powder_black = model.material("powder_black", color=(0.015, 0.016, 0.017, 1.0))
    satin_black = model.material("satin_black", color=(0.055, 0.058, 0.060, 1.0))
    dark_metal = model.material("dark_metal", color=(0.20, 0.21, 0.22, 1.0))
    fastener = model.material("fastener", color=(0.55, 0.56, 0.55, 1.0))
    rubber = model.material("rubber", color=(0.01, 0.01, 0.012, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.46, 0.22, 0.035)),
        origin=Origin(xyz=(0.05, 0.0, 0.105)),
        material=powder_black,
        name="top_plate",
    )
    support.visual(
        Box((0.46, 0.018, 0.060)),
        origin=Origin(xyz=(0.05, 0.119, 0.075)),
        material=powder_black,
        name="front_lip",
    )
    support.visual(
        Box((0.46, 0.018, 0.060)),
        origin=Origin(xyz=(0.05, -0.119, 0.075)),
        material=powder_black,
        name="rear_lip",
    )
    support.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="drop_post",
    )
    support.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=dark_metal,
        name="shoulder_boss",
    )
    for i, (x, y) in enumerate(((-0.12, -0.065), (-0.12, 0.065), (0.18, -0.065), (0.18, 0.065))):
        support.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.120)),
            material=fastener,
            name=f"mount_screw_{i}",
        )

    link_0 = model.part("link_0")
    link_0.visual(
        Cylinder(radius=0.047, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_metal,
        name="shoulder_hub",
    )
    link_0.visual(
        Cylinder(radius=0.044, length=0.040),
        origin=Origin(xyz=(0.320, 0.0, -0.030)),
        material=dark_metal,
        name="elbow_upper_hub",
    )
    for i, y in enumerate((-0.040, 0.040)):
        link_0.visual(
            Box((0.320, 0.022, 0.020)),
            origin=Origin(xyz=(0.160, y, -0.038)),
            material=satin_black,
            name=f"rail_{i}",
        )
    link_0.visual(
        Box((0.230, 0.082, 0.012)),
        origin=Origin(xyz=(0.160, 0.0, -0.038)),
        material=satin_black,
        name="center_rib",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=0.042, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=dark_metal,
        name="elbow_lower_hub",
    )
    link_1.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(-0.280, 0.0, -0.070)),
        material=dark_metal,
        name="head_hub",
    )
    for i, y in enumerate((-0.035, 0.035)):
        link_1.visual(
            Box((0.280, 0.020, 0.020)),
            origin=Origin(xyz=(-0.140, y, -0.078)),
            material=satin_black,
            name=f"rail_{i}",
        )
    link_1.visual(
        Box((0.195, 0.072, 0.012)),
        origin=Origin(xyz=(-0.140, 0.0, -0.078)),
        material=satin_black,
        name="center_rib",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=dark_metal,
        name="yaw_collar",
    )
    swivel.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.178)),
        material=dark_metal,
        name="drop_stem",
    )
    swivel.visual(
        Box((0.060, 0.175, 0.030)),
        origin=Origin(xyz=(0.025, 0.0, -0.235)),
        material=satin_black,
        name="yoke_cross",
    )
    for y, ear_name, cap_name in (
        (-0.080, "tilt_ear_0", "tilt_cap_0"),
        (0.080, "tilt_ear_1", "tilt_cap_1"),
    ):
        swivel.visual(
            Box((0.080, 0.015, 0.110)),
            origin=Origin(xyz=(0.035, y, -0.185)),
            material=satin_black,
            name=ear_name,
        )
        swivel.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.055, -0.0935 if y < 0 else 0.0935, -0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener,
            name=cap_name,
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.021, length=0.145),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_bar",
    )
    head.visual(
        Box((0.040, 0.050, 0.030)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=dark_metal,
        name="neck_block",
    )
    head.visual(
        Box((0.014, 0.135, 0.120)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=powder_black,
        name="vesa_plate",
    )
    for i, (y, z) in enumerate(((-0.0375, -0.0375), (-0.0375, 0.0375), (0.0375, -0.0375), (0.0375, 0.0375))):
        head.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.055, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"vesa_pad_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-2.65, upper=2.65),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.0, lower=-2.55, upper=2.55),
    )
    model.articulation(
        "swivel_yaw",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=swivel,
        origin=Origin(xyz=(-0.280, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.055, 0.0, -0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.45, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    swivel_yaw = object_model.get_articulation("swivel_yaw")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four revolute axes",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    ctx.expect_contact(
        support,
        link_0,
        elem_a="shoulder_boss",
        elem_b="shoulder_hub",
        contact_tol=0.001,
        name="shoulder stack is seated",
    )
    ctx.expect_contact(
        link_0,
        link_1,
        elem_a="elbow_upper_hub",
        elem_b="elbow_lower_hub",
        contact_tol=0.001,
        name="elbow stack is seated",
    )
    ctx.expect_contact(
        link_1,
        swivel,
        elem_a="head_hub",
        elem_b="yaw_collar",
        contact_tol=0.001,
        name="head swivel is seated",
    )
    ctx.expect_contact(
        swivel,
        head,
        elem_a="tilt_ear_0",
        elem_b="tilt_bar",
        contact_tol=0.001,
        name="tilt bar is carried by yoke",
    )

    ctx.expect_overlap(
        head,
        support,
        axes="xy",
        min_overlap=0.045,
        name="folded head nests below support footprint",
    )

    rest_elbow = ctx.part_world_position(link_1)
    with ctx.pose({shoulder: 1.0}):
        moved_elbow = ctx.part_world_position(link_1)
    ctx.check(
        "shoulder swings first link",
        rest_elbow is not None and moved_elbow is not None and moved_elbow[1] > rest_elbow[1] + 0.18,
        details=f"rest={rest_elbow}, moved={moved_elbow}",
    )

    rest_swivel = ctx.part_world_position(swivel)
    with ctx.pose({elbow: 1.0}):
        moved_swivel = ctx.part_world_position(swivel)
    ctx.check(
        "elbow unfolds second link",
        rest_swivel is not None and moved_swivel is not None and moved_swivel[1] < rest_swivel[1] - 0.18,
        details=f"rest={rest_swivel}, moved={moved_swivel}",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({swivel_yaw: 0.9}):
        yawed_head = ctx.part_world_position(head)
    ctx.check(
        "head swivel yaws carrier",
        rest_head is not None and yawed_head is not None and yawed_head[1] > rest_head[1] + 0.030,
        details=f"rest={rest_head}, yawed={yawed_head}",
    )

    rest_aabb = ctx.part_world_aabb(head)
    with ctx.pose({head_tilt: 0.45}):
        tilted_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head tilt raises plate edge",
        rest_aabb is not None and tilted_aabb is not None and tilted_aabb[1][2] > rest_aabb[1][2] + 0.010,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

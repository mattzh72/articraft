from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_bench_cantilever_arm")

    dark = model.material("dark_powder_coat", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("black_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    arm_yellow = model.material("safety_yellow_casting", rgba=(0.95, 0.63, 0.08, 1.0))
    face_grey = model.material("machined_face", rgba=(0.62, 0.64, 0.63, 1.0))
    bolt = model.material("dark_fasteners", rgba=(0.025, 0.025, 0.028, 1.0))

    shoulder_z = 0.195
    elbow_x = 0.380
    elbow_z = 0.055
    wrist_x = 0.320
    wrist_z = 0.055

    base = model.part("base")
    base.visual(
        Box((0.34, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="bench_plate",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark,
        name="short_column",
    )
    base.visual(
        Box((0.20, 0.026, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark,
        name="long_gusset",
    )
    base.visual(
        Box((0.026, 0.16, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark,
        name="cross_gusset",
    )
    base.visual(
        Cylinder(radius=0.076, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=black,
        name="top_bearing",
    )

    link_0 = model.part("link_0")
    link_0.visual(
        Cylinder(radius=0.070, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=black,
        name="shoulder_bearing",
    )
    link_0.visual(
        Box((0.360, 0.065, 0.038)),
        origin=Origin(xyz=(0.180, 0.0, 0.034)),
        material=arm_yellow,
        name="first_casting",
    )
    link_0.visual(
        Cylinder(radius=0.055, length=0.055),
        origin=Origin(xyz=(elbow_x, 0.0, 0.0275)),
        material=black,
        name="elbow_bearing",
    )
    link_0.visual(
        Box((0.070, 0.088, 0.018)),
        origin=Origin(xyz=(0.205, 0.0, 0.060)),
        material=arm_yellow,
        name="raised_rib",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black,
        name="proximal_bearing",
    )
    link_1.visual(
        Box((0.305, 0.060, 0.038)),
        origin=Origin(xyz=(0.153, 0.0, 0.032)),
        material=arm_yellow,
        name="second_casting",
    )
    link_1.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(xyz=(wrist_x, 0.0, 0.0275)),
        material=black,
        name="wrist_bearing",
    )
    link_1.visual(
        Box((0.058, 0.078, 0.016)),
        origin=Origin(xyz=(0.165, 0.0, 0.056)),
        material=arm_yellow,
        name="forearm_rib",
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        Cylinder(radius=0.043, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=black,
        name="wrist_turntable",
    )
    wrist_face.visual(
        Box((0.075, 0.055, 0.035)),
        origin=Origin(xyz=(0.040, 0.0, 0.025)),
        material=face_grey,
        name="neck_block",
    )
    wrist_face.visual(
        Box((0.022, 0.140, 0.100)),
        origin=Origin(xyz=(0.085, 0.0, 0.062)),
        material=face_grey,
        name="tool_face",
    )
    wrist_face.visual(
        Cylinder(radius=0.037, length=0.013),
        origin=Origin(xyz=(0.1015, 0.0, 0.062), rpy=(0.0, pi / 2.0, 0.0)),
        material=face_grey,
        name="tool_disk",
    )
    for idx, (y, z) in enumerate(
        ((-0.045, 0.035), (0.045, 0.035), (-0.045, 0.089), (0.045, 0.089))
    ):
        wrist_face.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.0995, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt,
            name=f"face_bolt_{idx}",
        )

    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(elbow_x, 0.0, elbow_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.2, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "link_1_to_wrist_face",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=wrist_face,
        origin=Origin(xyz=(wrist_x, 0.0, wrist_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    wrist = object_model.get_part("wrist_face")
    shoulder = object_model.get_articulation("base_to_link_0")
    elbow = object_model.get_articulation("link_0_to_link_1")
    wrist_joint = object_model.get_articulation("link_1_to_wrist_face")

    serial_joints = (shoulder, elbow, wrist_joint)
    ctx.check(
        "serial three revolute arm",
        len(serial_joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in serial_joints)
        and [j.parent for j in serial_joints] == ["base", "link_0", "link_1"]
        and [j.child for j in serial_joints] == ["link_0", "link_1", "wrist_face"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in serial_joints]}",
    )

    ctx.expect_gap(
        link_0,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="shoulder_bearing",
        negative_elem="top_bearing",
        name="shoulder turntable seats on column",
    )
    ctx.expect_overlap(
        link_0,
        base,
        axes="xy",
        min_overlap=0.080,
        elem_a="shoulder_bearing",
        elem_b="top_bearing",
        name="shoulder bearing is centered over column",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="proximal_bearing",
        negative_elem="elbow_bearing",
        name="elbow bearing stack is seated",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xy",
        min_overlap=0.070,
        elem_a="proximal_bearing",
        elem_b="elbow_bearing",
        name="elbow joint shares a vertical axis",
    )
    ctx.expect_gap(
        wrist,
        link_1,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="wrist_turntable",
        negative_elem="wrist_bearing",
        name="wrist turntable seats on forearm",
    )
    ctx.expect_overlap(
        wrist,
        link_1,
        axes="xy",
        min_overlap=0.060,
        elem_a="wrist_turntable",
        elem_b="wrist_bearing",
        name="wrist joint shares a vertical axis",
    )

    base_aabb = ctx.part_world_aabb(base)
    wrist_aabb = ctx.part_world_aabb(wrist)
    ctx.check(
        "arm overhangs one side of base",
        base_aabb is not None
        and wrist_aabb is not None
        and wrist_aabb[1][0] > base_aabb[1][0] + 0.45,
        details=f"base_aabb={base_aabb}, wrist_aabb={wrist_aabb}",
    )

    rest_face = ctx.part_element_world_aabb(wrist, elem="tool_face")
    with ctx.pose({wrist_joint: 0.70}):
        turned_face = ctx.part_element_world_aabb(wrist, elem="tool_face")
    rest_y = None if rest_face is None else (rest_face[0][1] + rest_face[1][1]) / 2.0
    turned_y = None if turned_face is None else (turned_face[0][1] + turned_face[1][1]) / 2.0
    ctx.check(
        "wrist face rotates about its vertical joint",
        rest_y is not None and turned_y is not None and abs(turned_y - rest_y) > 0.030,
        details=f"rest_face={rest_face}, turned_face={turned_face}",
    )

    return ctx.report()


object_model = build_object_model()

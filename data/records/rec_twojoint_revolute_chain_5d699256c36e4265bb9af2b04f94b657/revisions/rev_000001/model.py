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
    model = ArticulatedObject(name="bench_top_two_joint_chain")

    dark_paint = Material("matte_black_paint", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber = Material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    clear_plate = Material("clear_polycarbonate", rgba=(0.45, 0.75, 1.0, 0.36))
    screw_metal = Material("dark_screw_heads", rgba=(0.12, 0.12, 0.12, 1.0))

    pin_rpy = (-math.pi / 2.0, 0.0, 0.0)
    joint0_z = 0.20
    link1_len = 0.36

    base = model.part("base")
    base.visual(
        Box((0.38, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=rubber,
        name="base_foot",
    )
    base.visual(
        Box((0.09, 0.13, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_paint,
        name="upright_block",
    )
    base.visual(
        Box((0.110, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, 0.050, joint0_z)),
        material=clear_plate,
        name="joint_plate_0",
    )
    base.visual(
        Box((0.110, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, -0.050, joint0_z)),
        material=clear_plate,
        name="joint_plate_1",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.061, joint0_z), rpy=pin_rpy),
        material=screw_metal,
        name="shoulder_pin_cap_0",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.061, joint0_z), rpy=pin_rpy),
        material=screw_metal,
        name="shoulder_pin_cap_1",
    )
    for ix, x in enumerate((-0.135, 0.135)):
        for iy, y in enumerate((-0.070, 0.070)):
            base.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(x, y, 0.036)),
                material=screw_metal,
                name=f"foot_screw_{ix}_{iy}",
            )

    link_0 = model.part("link_0")
    link_0.visual(
        Cylinder(radius=0.030, length=0.094),
        origin=Origin(rpy=pin_rpy),
        material=aluminum,
        name="shoulder_hub",
    )
    link_0.visual(
        Box((0.296, 0.032, 0.024)),
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
        material=aluminum,
        name="main_link",
    )
    link_0.visual(
        Box((0.070, 0.032, 0.024)),
        origin=Origin(xyz=(link1_len - 0.065, 0.0295, 0.0)),
        material=aluminum,
        name="distal_web_0",
    )
    link_0.visual(
        Box((0.070, 0.032, 0.024)),
        origin=Origin(xyz=(link1_len - 0.065, -0.0295, 0.0)),
        material=aluminum,
        name="distal_web_1",
    )
    link_0.visual(
        Box((0.110, 0.012, 0.082)),
        origin=Origin(xyz=(link1_len, 0.050, 0.0)),
        material=clear_plate,
        name="elbow_plate_0",
    )
    link_0.visual(
        Box((0.110, 0.012, 0.082)),
        origin=Origin(xyz=(link1_len, -0.050, 0.0)),
        material=clear_plate,
        name="elbow_plate_1",
    )
    link_0.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(link1_len, 0.061, 0.0), rpy=pin_rpy),
        material=screw_metal,
        name="elbow_pin_cap_0",
    )
    link_0.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(link1_len, -0.061, 0.0), rpy=pin_rpy),
        material=screw_metal,
        name="elbow_pin_cap_1",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=0.026, length=0.090),
        origin=Origin(rpy=pin_rpy),
        material=aluminum,
        name="elbow_hub",
    )
    link_1.visual(
        Box((0.220, 0.030, 0.022)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=aluminum,
        name="distal_link",
    )
    link_1.visual(
        Box((0.056, 0.058, 0.024)),
        origin=Origin(xyz=(0.257, 0.0, 0.0)),
        material=aluminum,
        name="end_tab_neck",
    )
    link_1.visual(
        Cylinder(radius=0.028, length=0.058),
        origin=Origin(xyz=(0.286, 0.0, 0.0), rpy=pin_rpy),
        material=aluminum,
        name="end_tab_round",
    )
    link_1.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.286, 0.0, 0.0), rpy=pin_rpy),
        material=screw_metal,
        name="tab_hole_shadow",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, joint0_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link1_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-1.45, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")

    revolute_count = sum(
        1
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check("two revolute joints", revolute_count == 2, details=f"count={revolute_count}")
    ctx.check(
        "parallel hinge axes",
        tuple(shoulder.axis) == (0.0, -1.0, 0.0) and tuple(elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )
    ctx.allow_overlap(
        link_0,
        base,
        elem_a="shoulder_hub",
        elem_b="joint_plate_0",
        reason="The shoulder axle is intentionally captured through the clear side plate.",
    )
    ctx.allow_overlap(
        link_0,
        base,
        elem_a="shoulder_hub",
        elem_b="joint_plate_1",
        reason="The shoulder axle is intentionally captured through the clear side plate.",
    )
    ctx.allow_overlap(
        link_1,
        link_0,
        elem_a="elbow_hub",
        elem_b="elbow_plate_0",
        reason="The elbow axle is intentionally captured through the clear side plate.",
    )
    ctx.allow_overlap(
        link_1,
        link_0,
        elem_a="elbow_hub",
        elem_b="elbow_plate_1",
        reason="The elbow axle is intentionally captured through the clear side plate.",
    )
    ctx.expect_gap(
        base,
        link_0,
        axis="y",
        positive_elem="joint_plate_0",
        negative_elem="shoulder_hub",
        max_penetration=0.006,
        max_gap=0.001,
        name="shoulder axle seats in upper side plate",
    )
    ctx.expect_gap(
        link_0,
        base,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="joint_plate_1",
        max_penetration=0.006,
        max_gap=0.001,
        name="shoulder axle seats in lower side plate",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="y",
        positive_elem="elbow_plate_0",
        negative_elem="elbow_hub",
        max_penetration=0.004,
        max_gap=0.001,
        name="elbow axle seats in upper side plate",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_plate_1",
        max_penetration=0.004,
        max_gap=0.001,
        name="elbow axle seats in lower side plate",
    )
    ctx.expect_overlap(
        link_0,
        base,
        axes="xz",
        elem_a="shoulder_hub",
        elem_b="joint_plate_0",
        min_overlap=0.025,
        name="shoulder hub sits inside base plates",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xz",
        elem_a="elbow_hub",
        elem_b="elbow_plate_0",
        min_overlap=0.020,
        name="elbow hub sits inside distal plates",
    )

    rest_aabb = ctx.part_world_aabb(link_1)
    with ctx.pose({shoulder: 0.65, elbow: 0.55}):
        raised_aabb = ctx.part_world_aabb(link_1)
    rest_z = rest_aabb[1][2] if rest_aabb is not None else None
    raised_z = raised_aabb[1][2] if raised_aabb is not None else None
    ctx.check(
        "positive joint motion stays in vertical plane",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.06,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _body_shape() -> cq.Workplane:
    deck_plate = cq.Workplane("XY").circle(0.031).extrude(0.006)
    barrel = cq.Workplane("XY").workplane(offset=0.006).circle(0.025).extrude(0.044)
    top_cap = cq.Workplane("XY").workplane(offset=0.050).circle(0.0205).extrude(0.015)

    spout_outer = (
        cq.Workplane("YZ")
        .circle(0.0135)
        .extrude(0.064)
        .translate((0.006, 0.0, 0.042))
    )
    spout_bore = (
        cq.Workplane("YZ")
        .circle(0.01005)
        .extrude(0.064)
        .translate((0.006, 0.0, 0.042))
    )

    lever_boss = (
        cq.Workplane("XZ")
        .circle(0.009)
        .extrude(0.013)
        .translate((0.0, 0.020, 0.047))
    )

    body = deck_plate.union(barrel).union(top_cap).union(spout_outer).union(lever_boss)
    return body.cut(spout_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    satin = model.material("satin", rgba=(0.73, 0.75, 0.78, 1.0))
    nozzle_black = model.material("nozzle_black", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "faucet_body"),
        material=chrome,
        name="body_shell",
    )
    body.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.028, 0.047)),
        material=chrome,
        name="lever_mount",
    )

    lever = model.part("lever")
    lever.visual(
        Box((0.010, 0.009, 0.010)),
        origin=Origin(xyz=(0.0, 0.0045, -0.002)),
        material=satin,
        name="lever_root",
    )
    lever.visual(
        Box((0.008, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, 0.023, -0.001)),
        material=satin,
        name="lever_arm",
    )
    lever.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.0, 0.042, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="lever_tip",
    )

    spray_head = model.part("spray_head")
    spray_head.visual(
        Cylinder(radius=0.0096, length=0.062),
        origin=Origin(xyz=(-0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hose_shank",
    )
    spray_head.visual(
        Cylinder(radius=0.0112, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="head_collar",
    )
    spray_head.visual(
        Cylinder(radius=0.0102, length=0.002),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_black,
        name="spray_face",
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, 0.033, 0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "body_to_spray_head",
        ArticulationType.PRISMATIC,
        parent=body,
        child=spray_head,
        origin=Origin(xyz=(0.070, 0.0, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.20,
            lower=0.0,
            upper=0.042,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    spray_head = object_model.get_part("spray_head")
    lever_joint = object_model.get_articulation("body_to_lever")
    spray_joint = object_model.get_articulation("body_to_spray_head")

    ctx.expect_gap(
        lever,
        body,
        axis="y",
        positive_elem="lever_root",
        negative_elem="lever_mount",
        max_gap=0.001,
        max_penetration=0.0005,
        name="lever root sits at the side pivot mount",
    )
    ctx.expect_gap(
        spray_head,
        body,
        axis="x",
        positive_elem="head_collar",
        negative_elem="body_shell",
        max_gap=0.0015,
        max_penetration=0.0015,
        name="spray head docks flush with the spout tip",
    )
    ctx.expect_within(
        spray_head,
        body,
        axes="yz",
        inner_elem="hose_shank",
        outer_elem="body_shell",
        margin=0.0015,
        name="hose shank stays centered inside the spout bore at rest",
    )
    ctx.expect_overlap(
        spray_head,
        body,
        axes="x",
        elem_a="hose_shank",
        elem_b="body_shell",
        min_overlap=0.055,
        name="docked spray head remains deeply inserted in the spout",
    )

    rest_head_pos = ctx.part_world_position(spray_head)
    with ctx.pose({spray_joint: 0.042}):
        ctx.expect_within(
            spray_head,
            body,
            axes="yz",
            inner_elem="hose_shank",
            outer_elem="body_shell",
            margin=0.0015,
            name="extended spray head stays aligned with the spout bore",
        )
        ctx.expect_overlap(
            spray_head,
            body,
            axes="x",
            elem_a="hose_shank",
            elem_b="body_shell",
            min_overlap=0.018,
            name="extended spray head keeps retained insertion in the spout",
        )
        extended_head_pos = ctx.part_world_position(spray_head)

    ctx.check(
        "spray head pulls outward along the spout axis",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[0] > rest_head_pos[0] + 0.035,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(lever, elem="lever_tip")
    with ctx.pose({lever_joint: 1.0}):
        lifted_tip_aabb = ctx.part_element_world_aabb(lever, elem="lever_tip")

    ctx.check(
        "lever tip lifts when the handle rotates open",
        rest_tip_aabb is not None
        and lifted_tip_aabb is not None
        and lifted_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.020,
        details=f"rest={rest_tip_aabb}, lifted={lifted_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

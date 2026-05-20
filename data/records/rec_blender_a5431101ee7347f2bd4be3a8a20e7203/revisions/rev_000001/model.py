from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_LENGTH = 0.24
BASE_DEPTH = 0.19
BASE_LOWER_HEIGHT = 0.078
BASE_UPPER_HEIGHT = 0.046
TOP_DECK_THICKNESS = 0.006
TOP_DECK_Z = BASE_LOWER_HEIGHT + BASE_UPPER_HEIGHT + TOP_DECK_THICKNESS

PITCHER_HEIGHT = 0.215
PITCHER_X = -0.035

DOOR_LENGTH = 0.038
DOOR_WIDTH = 0.040
DOOR_THICKNESS = 0.005
DOOR_HINGE_X = 0.046


def _build_pitcher_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.145, 0.118, PITCHER_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .shell(-0.0045)
    )

    foot = (
        cq.Workplane("XY")
        .box(0.108, 0.094, 0.020, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .circle(0.025)
        .cutBlind(-0.014)
    )

    spout = cq.Workplane("XY").box(0.018, 0.040, 0.008).translate((0.076, 0.0, 0.206))

    grip = cq.Workplane("XY").box(0.018, 0.022, 0.118).translate((0.0, 0.093, 0.118))
    upper_bridge = cq.Workplane("XY").box(0.020, 0.050, 0.018).translate((0.0, 0.079, 0.176))
    lower_bridge = cq.Workplane("XY").box(0.020, 0.048, 0.018).translate((0.0, 0.078, 0.070))

    return (
        body.union(foot)
        .union(spout)
        .union(grip)
        .union(upper_bridge)
        .union(lower_bridge)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_blender")

    body_matte = model.material("body_matte", rgba=(0.86, 0.88, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    pitcher_clear = model.material("pitcher_clear", rgba=(0.82, 0.90, 0.96, 0.55))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_DEPTH, BASE_LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_HEIGHT / 2.0)),
        material=body_matte,
        name="body_lower",
    )
    base.visual(
        Box((0.206, 0.162, BASE_UPPER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_HEIGHT + BASE_UPPER_HEIGHT / 2.0)),
        material=body_matte,
        name="body_upper",
    )
    base.visual(
        Box((0.180, 0.140, TOP_DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_HEIGHT + BASE_UPPER_HEIGHT + TOP_DECK_THICKNESS / 2.0)),
        material=trim_dark,
        name="top_deck",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(PITCHER_X, 0.0, TOP_DECK_Z + 0.002)),
        material=rubber,
        name="pitcher_pad",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.027,
                body_style="tapered",
                top_diameter=0.040,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "front_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="dial_knob",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((DOOR_LENGTH, DOOR_WIDTH, DOOR_THICKNESS)),
        origin=Origin(xyz=(DOOR_LENGTH / 2.0, 0.0, DOOR_THICKNESS / 2.0)),
        material=trim_dark,
        name="door_panel",
    )
    service_door.visual(
        Box((0.010, DOOR_WIDTH * 0.64, 0.007)),
        origin=Origin(xyz=(DOOR_LENGTH - 0.005, 0.0, 0.0035)),
        material=trim_dark,
        name="door_pull",
    )
    service_door.visual(
        Cylinder(radius=0.003, length=DOOR_WIDTH * 0.62),
        origin=Origin(xyz=(0.003, 0.0, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="door_barrel",
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        mesh_from_cadquery(_build_pitcher_shape(), "pitcher"),
        material=pitcher_clear,
        name="pitcher_body",
    )
    pitcher.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=steel,
        name="blade_spindle",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.140, 0.114, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rubber,
        name="lid_plate",
    )
    lid.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=rubber,
        name="lid_cap",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.011, length=0.010),
        material=steel,
        name="blade_hub",
    )
    blade.visual(
        Box((0.040, 0.008, 0.002)),
        origin=Origin(xyz=(0.018, 0.0, 0.004), rpy=(0.0, math.radians(16.0), math.radians(24.0))),
        material=steel,
        name="blade_0",
    )
    blade.visual(
        Box((0.040, 0.008, 0.002)),
        origin=Origin(xyz=(-0.018, 0.0, 0.004), rpy=(0.0, math.radians(-16.0), math.radians(204.0))),
        material=steel,
        name="blade_1",
    )
    blade.visual(
        Box((0.036, 0.007, 0.002)),
        origin=Origin(xyz=(0.0, 0.018, 0.003), rpy=(0.0, math.radians(-14.0), math.radians(114.0))),
        material=steel,
        name="blade_2",
    )
    blade.visual(
        Box((0.036, 0.007, 0.002)),
        origin=Origin(xyz=(0.0, -0.018, 0.003), rpy=(0.0, math.radians(14.0), math.radians(-66.0))),
        material=steel,
        name="blade_3",
    )

    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(BASE_LENGTH / 2.0, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    model.articulation(
        "base_to_service_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=service_door,
        origin=Origin(xyz=(DOOR_HINGE_X, 0.0, TOP_DECK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(PITCHER_X, 0.0, TOP_DECK_Z)),
    )
    model.articulation(
        "pitcher_to_lid",
        ArticulationType.FIXED,
        parent=pitcher,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, PITCHER_HEIGHT)),
    )
    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    dial = object_model.get_part("dial")
    door = object_model.get_part("service_door")
    pitcher = object_model.get_part("pitcher")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")

    door_hinge = object_model.get_articulation("base_to_service_door")
    dial_joint = object_model.get_articulation("base_to_dial")
    blade_joint = object_model.get_articulation("pitcher_to_blade")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            base,
            axis="z",
            positive_elem="door_panel",
            negative_elem="top_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name="service door sits flush with the top deck",
        )
        ctx.expect_overlap(
            door,
            base,
            axes="xy",
            elem_a="door_panel",
            elem_b="top_deck",
            min_overlap=0.020,
            name="service door covers the service opening footprint",
        )

    ctx.expect_gap(
        dial,
        base,
        axis="x",
        positive_elem="dial_knob",
        negative_elem="body_lower",
        max_gap=0.002,
        max_penetration=0.0,
        name="front dial mounts flush to the base face",
    )
    ctx.expect_overlap(
        dial,
        base,
        axes="yz",
        elem_a="dial_knob",
        elem_b="body_lower",
        min_overlap=0.032,
        name="front dial stays centered on the front panel",
    )
    ctx.expect_contact(
        pitcher,
        base,
        elem_a="pitcher_body",
        elem_b="pitcher_pad",
        name="pitcher lands on the top deck pad",
    )
    ctx.expect_gap(
        lid,
        pitcher,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="pitcher_body",
        max_gap=0.002,
        max_penetration=0.0,
        name="lid seats on the pitcher rim",
    )
    ctx.expect_contact(
        blade,
        pitcher,
        elem_a="blade_hub",
        elem_b="blade_spindle",
        contact_tol=0.001,
        name="blade hub seats on the spindle mount",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: math.radians(75.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "service door opens upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.028,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.check(
        "blade uses continuous rotation",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()

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


def _spout_shell() -> cq.Workplane:
    outer_radius = 0.0175

    def tube_between(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        radius: float,
    ) -> cq.Workplane:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        direction = (dx / length, dy / length, dz / length)
        midpoint = (
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
            0.5 * (start[2] + end[2]),
        )
        return (
            cq.Workplane("XY")
            .cylinder(length, radius, centered=(True, True, True), direct=direction)
            .translate(midpoint)
        )

    crown_0 = (0.0, 0.0, 0.255)
    crown_1 = (0.090, 0.0, 0.305)
    nozzle_top = (0.210, 0.0, 0.137)

    collar = cq.Workplane("XY").cylinder(0.015, 0.026, centered=(True, True, False))
    outer = (
        collar.union(
            cq.Workplane("XY")
            .cylinder(0.240, outer_radius, centered=(True, True, False))
            .translate((0.0, 0.0, 0.015))
        )
        .union(tube_between(crown_0, crown_1, outer_radius))
        .union(tube_between(crown_1, nozzle_top, outer_radius))
    )

    for point in (crown_0, crown_1, nozzle_top):
        outer = outer.union(cq.Workplane("XY").sphere(outer_radius).translate(point))

    return outer


def _nozzle_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.0195)
        .circle(0.0160)
        .extrude(0.047)
        .translate((0.21, 0.0, 0.090))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_kitchen_faucet")

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.14, 0.15, 0.16, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.33, 0.35, 0.38, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=chrome,
        name="deck_flange",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=chrome,
        name="body_shell",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.026, 0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_boss",
    )

    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_spout_shell(), "spout_shell"),
        material=chrome,
        name="spout_shell",
    )
    spout.visual(
        mesh_from_cadquery(_nozzle_shell(), "nozzle_shell"),
        material=chrome,
        name="nozzle_shell",
    )

    side_lever = model.part("side_lever")
    side_lever.visual(
        Box((0.014, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(-0.18, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_block",
    )
    side_lever.visual(
        Box((0.012, 0.078, 0.010)),
        origin=Origin(xyz=(0.0, 0.062, 0.004), rpy=(-0.18, 0.0, 0.0)),
        material=satin_metal,
        name="lever_arm",
    )
    side_lever.visual(
        Cylinder(radius=0.0075, length=0.032),
        origin=Origin(xyz=(0.0, 0.094, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="grip_tip",
    )

    spray_head = model.part("spray_head")
    spray_head.visual(
        Cylinder(radius=0.0075, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=hose_gray,
        name="hose",
    )
    spray_head.visual(
        Cylinder(radius=0.0150, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=chrome,
        name="upper_neck",
    )
    spray_head.visual(
        Cylinder(radius=0.0190, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=chrome,
        name="seat_lip",
    )
    spray_head.visual(
        Cylinder(radius=0.0180, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0575)),
        material=chrome,
        name="head_shell",
    )
    spray_head.visual(
        Cylinder(radius=0.0160, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=dark_insert,
        name="spray_ring",
    )
    spray_head.visual(
        Cylinder(radius=0.0135, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.093)),
        material=dark_insert,
        name="nozzle_face",
    )

    model.articulation(
        "base_to_spout",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0),
    )
    model.articulation(
        "base_to_side_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_lever,
        origin=Origin(xyz=(0.0, 0.026, 0.068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=math.radians(-12.0),
            upper=math.radians(42.0),
        ),
    )
    model.articulation(
        "spout_to_spray_head",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(xyz=(0.21, 0.0, 0.090)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.35,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("side_lever")
    spray_head = object_model.get_part("spray_head")

    spout_joint = object_model.get_articulation("base_to_spout")
    lever_joint = object_model.get_articulation("base_to_side_lever")
    spray_joint = object_model.get_articulation("spout_to_spray_head")

    ctx.allow_overlap(
        spout,
        spray_head,
        elem_a="spout_shell",
        elem_b="hose",
        reason="The visible nozzle is hollow, but the hidden hose path inside the arched spout is simplified as a solid outer shell.",
    )

    ctx.expect_overlap(
        spout,
        base,
        axes="xy",
        min_overlap=0.035,
        name="spout swivel stays centered over the faucet body",
    )
    ctx.expect_within(
        spray_head,
        spout,
        axes="xy",
        inner_elem="upper_neck",
        outer_elem="nozzle_shell",
        margin=0.0,
        name="spray head neck stays inside the nozzle footprint",
    )
    ctx.expect_overlap(
        spray_head,
        spout,
        axes="z",
        elem_a="upper_neck",
        elem_b="nozzle_shell",
        min_overlap=0.009,
        name="spray head seats into the nozzle shell",
    )

    rest_nozzle = ctx.part_world_position(spray_head)
    with ctx.pose({spout_joint: math.pi / 2.0}):
        quarter_turn_nozzle = ctx.part_world_position(spray_head)
    ctx.check(
        "spout quarter turn swings the nozzle around the base axis",
        rest_nozzle is not None
        and quarter_turn_nozzle is not None
        and rest_nozzle[0] > 0.18
        and abs(rest_nozzle[1]) < 0.02
        and quarter_turn_nozzle[1] > 0.18
        and abs(quarter_turn_nozzle[0]) < 0.02
        and abs(quarter_turn_nozzle[2] - rest_nozzle[2]) < 0.002,
        details=f"rest={rest_nozzle}, quarter_turn={quarter_turn_nozzle}",
    )

    lever_rest = ctx.part_element_world_aabb(lever, elem="grip_tip")
    with ctx.pose({lever_joint: math.radians(40.0)}):
        lever_raised = ctx.part_element_world_aabb(lever, elem="grip_tip")
    ctx.check(
        "side lever lifts upward about its short pivot",
        lever_rest is not None
        and lever_raised is not None
        and lever_raised[1][2] > lever_rest[1][2] + 0.030,
        details=f"rest={lever_rest}, raised={lever_raised}",
    )

    spray_rest = ctx.part_world_position(spray_head)
    with ctx.pose({spray_joint: 0.12}):
        ctx.expect_within(
            spray_head,
            spout,
            axes="xy",
            inner_elem="hose",
            outer_elem="nozzle_shell",
            margin=0.0,
            name="extended hose stays centered in the nozzle neck",
        )
        ctx.expect_overlap(
            spray_head,
            spout,
            axes="z",
            elem_a="hose",
            elem_b="nozzle_shell",
            min_overlap=0.020,
            name="extended hose remains inserted in the nozzle neck",
        )
        spray_extended = ctx.part_world_position(spray_head)
    ctx.check(
        "spray head pulls down from the spout end",
        spray_rest is not None
        and spray_extended is not None
        and spray_extended[2] < spray_rest[2] - 0.10,
        details=f"rest={spray_rest}, extended={spray_extended}",
    )

    return ctx.report()


object_model = build_object_model()

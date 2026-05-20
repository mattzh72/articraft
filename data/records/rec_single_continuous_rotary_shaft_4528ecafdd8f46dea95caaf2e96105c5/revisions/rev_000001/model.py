from __future__ import annotations

import cadquery as cq
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
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pottery_wheel")

    base = model.part("base")

    # Table top with a central hole for the shaft
    table_top_cq = (
        cq.Workplane("XY")
        .box(0.55, 0.75, 0.05)
        .faces(">Z").workplane()
        .circle(0.03).cutThruAll()
        .translate((0, 0, 0.275))
    )
    base.visual(
        mesh_from_cadquery(table_top_cq, "table_top"),
        name="table_top",
        material=Material("table_mat", color=(0.8, 0.8, 0.8))
    )

    # 4 Legs
    leg_positions = [
        (0.225, 0.325),
        (-0.225, 0.325),
        (0.225, -0.325),
        (-0.225, -0.325),
    ]
    for i, (x, y) in enumerate(leg_positions):
        base.visual(
            Box((0.05, 0.05, 0.26)),
            origin=Origin(xyz=(x, y, 0.13)),
            name=f"leg_{i}",
            material=Material(f"leg_mat_{i}", color=(0.2, 0.2, 0.2))
        )

    # Motor housing under the table, extending to the floor
    base.visual(
        Box((0.2, 0.3, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        name="motor_housing",
        material=Material("motor_mat", color=(0.15, 0.15, 0.15))
    )

    # Splash pan sitting on the table top
    splash_pan_cq = (
        cq.Workplane("XY")
        .circle(0.25).extrude(0.15)
        .faces(">Z").workplane()
        .circle(0.24).cutBlind(-0.14)
        .faces("<Z").workplane()
        .circle(0.03).cutThruAll()
        .translate((0, 0, 0.29))  # Overlaps table top by 1cm for connectivity
    )
    base.visual(
        mesh_from_cadquery(splash_pan_cq, "splash_pan"),
        name="splash_pan",
        material=Material("pan_mat", color=(0.9, 0.8, 0.2))
    )

    # Pedal base on the floor
    base.visual(
        Box((0.12, 0.25, 0.04)),
        origin=Origin(xyz=(0.4, 0.0, 0.02)),
        name="pedal_base",
        material=Material("pedal_base_mat", color=(0.2, 0.2, 0.2))
    )

    # Hinge barrel for pedal
    base.visual(
        Cylinder(radius=0.01, length=0.10),
        origin=Origin(xyz=(0.4, -0.1, 0.04), rpy=(0.0, 1.5708, 0.0)),
        name="pedal_hinge",
        material=Material("pedal_base_mat", color=(0.2, 0.2, 0.2))
    )

    # Cable connecting motor housing to pedal base
    base.visual(
        Box((0.26, 0.02, 0.02)),
        origin=Origin(xyz=(0.22, 0.0, 0.01)),
        name="cable",
        material=Material("cable_mat", color=(0.1, 0.1, 0.1))
    )

    # Rotor part (shaft + wheelhead)
    rotor = model.part("rotor")
    
    # Shaft passing through table top and splash pan
    rotor.visual(
        Cylinder(radius=0.02, length=0.138),
        origin=Origin(xyz=(0.0, 0.0, 0.321)),
        name="shaft",
        material=Material("shaft_mat", color=(0.7, 0.7, 0.7))
    )
    
    # Wheelhead top plate
    rotor.visual(
        Cylinder(radius=0.15, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        name="wheelhead",
        material=Material("wheelhead_mat", color=(0.85, 0.85, 0.85))
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )

    # Foot pedal
    pedal = model.part("pedal")
    pedal.visual(
        Box((0.10, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.1, 0.01)),
        name="pad",
        material=Material("pad_mat", color=(0.1, 0.1, 0.1))
    )

    model.articulation(
        "pedal_press",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedal,
        origin=Origin(xyz=(0.4, -0.1, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.3)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotor = object_model.get_part("rotor")
    pedal = object_model.get_part("pedal")

    ctx.allow_overlap(
        rotor, base,
        elem_a="shaft",
        elem_b="motor_housing",
        reason="Shaft is inserted into the motor housing"
    )

    ctx.allow_overlap(
        pedal, base,
        elem_a="pad",
        elem_b="pedal_hinge",
        reason="Pedal pad wraps around the hinge barrel"
    )

    ctx.expect_within(
        rotor, base,
        axes="xy",
        inner_elem="wheelhead",
        outer_elem="splash_pan",
        name="wheelhead is horizontally contained within splash pan"
    )

    ctx.expect_gap(
        rotor, base,
        axis="z",
        positive_elem="wheelhead",
        negative_elem="table_top",
        min_gap=0.05,
        name="wheelhead sits above the table top"
    )

    ctx.expect_gap(
        pedal, base,
        axis="z",
        positive_elem="pad",
        negative_elem="pedal_base",
        min_gap=0.0,
        max_gap=0.005,
        name="pedal pad rests on pedal base"
    )

    with ctx.pose(pedal_press=0.3):
        pad_aabb = ctx.part_element_world_aabb(pedal, elem="pad")
        if pad_aabb:
            ctx.check(
                "pedal tilts up when pressed",
                pad_aabb[1][2] > 0.08,
                details=f"pad max Z is {pad_aabb[1][2]}"
            )
        else:
            ctx.fail("pedal tilts up when pressed", "could not get pad AABB")

    return ctx.report()


object_model = build_object_model()

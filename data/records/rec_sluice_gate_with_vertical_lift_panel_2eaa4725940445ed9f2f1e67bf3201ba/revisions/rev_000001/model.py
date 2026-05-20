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


OPENING_WIDTH = 1.20
GATE_WIDTH = 1.18
GATE_HEIGHT = 1.85
GATE_THICKNESS = 0.028
GATE_TRAVEL = 0.95
GATE_CENTER_Z = 1.095

RAIL_WIDTH = 0.10
RAIL_DEPTH = 0.12
RAIL_LENGTH = 2.35
RAIL_WEB_THICKNESS = 0.016
RAIL_FLANGE_THICKNESS = 0.016
RAIL_CENTER_Z = 1.345
RAIL_X = 0.655

FRAME_WIDTH = 1.90
FRAME_DEPTH = 0.16
FRAME_CENTER_Y = -0.18
SILL_HEIGHT = 0.22
PIER_WIDTH = 0.28
PIER_HEIGHT = 2.24
PIER_CENTER_Z = 1.33
PIER_X = 0.81
TOP_BEAM_HEIGHT = 0.24
TOP_BEAM_CENTER_Z = 2.54

HOUSING_WIDTH = 0.46
HOUSING_DEPTH = 0.22
HOUSING_HEIGHT = 0.26
HOUSING_CENTER_Y = 0.05
HOUSING_CENTER_Z = 2.80
HOUSING_FRONT_Y = HOUSING_CENTER_Y + HOUSING_DEPTH / 2.0

WHEEL_BOSS_RADIUS = 0.038
WHEEL_BOSS_LENGTH = 0.07
HANDWHEEL_CENTER_Y = HOUSING_FRONT_Y + WHEEL_BOSS_LENGTH + 0.016

DOOR_WIDTH = 0.16
DOOR_HEIGHT = 0.16
DOOR_THICKNESS = 0.014
DOOR_HINGE_X = 0.06
DOOR_CENTER_Z = HOUSING_CENTER_Z


def _build_handwheel_shape() -> cq.Shape:
    rim = (
        cq.Workplane("XZ")
        .circle(0.24)
        .circle(0.20)
        .extrude(0.011, both=True)
    )
    hub = (
        cq.Workplane("XZ")
        .circle(0.055)
        .circle(0.026)
        .extrude(0.018, both=True)
    )

    spoke_0 = cq.Workplane("XY").box(0.42, 0.016, 0.034)
    spoke_1 = cq.Workplane("XY").box(0.034, 0.016, 0.42)
    spoke_2 = spoke_0.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 45.0)
    spoke_3 = spoke_0.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -45.0)

    return rim.union(hub).union(spoke_0).union(spoke_1).union(spoke_2).union(spoke_3)


def _add_guide_rail(model: ArticulatedObject, name: str) -> None:
    rail = model.part(name)

    rail.visual(
        Box((RAIL_WEB_THICKNESS, RAIL_DEPTH, RAIL_LENGTH)),
        origin=Origin(
            xyz=(
                -RAIL_WIDTH / 2.0 + RAIL_WEB_THICKNESS / 2.0,
                0.0,
                0.0,
            )
        ),
        material="structural_steel",
        name="web",
    )
    rail.visual(
        Box((RAIL_WIDTH, RAIL_FLANGE_THICKNESS, RAIL_LENGTH)),
        origin=Origin(xyz=(0.0, RAIL_DEPTH / 2.0 - RAIL_FLANGE_THICKNESS / 2.0, 0.0)),
        material="structural_steel",
        name="front_flange",
    )
    rail.visual(
        Box((RAIL_WIDTH, RAIL_FLANGE_THICKNESS, RAIL_LENGTH)),
        origin=Origin(xyz=(0.0, -RAIL_DEPTH / 2.0 + RAIL_FLANGE_THICKNESS / 2.0, 0.0)),
        material="structural_steel",
        name="rear_flange",
    )

    for index, z in enumerate((-0.82, 0.00, 0.82)):
        rail.visual(
            Box((0.035, 0.048, 0.10)),
            origin=Origin(xyz=(-0.0325, -0.076, z)),
            material="structural_steel",
            name=f"mount_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_channel_sluice_gate")

    model.material("structural_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    model.material("gate_coating", rgba=(0.20, 0.34, 0.46, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("operator_wheel", rgba=(0.80, 0.13, 0.10, 1.0))
    model.material("seal_black", rgba=(0.08, 0.09, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_CENTER_Y, SILL_HEIGHT / 2.0)),
        material="structural_steel",
        name="sill",
    )
    frame.visual(
        Box((PIER_WIDTH, FRAME_DEPTH, PIER_HEIGHT)),
        origin=Origin(xyz=(-PIER_X, FRAME_CENTER_Y, PIER_CENTER_Z)),
        material="structural_steel",
        name="pier_0",
    )
    frame.visual(
        Box((PIER_WIDTH, FRAME_DEPTH, PIER_HEIGHT)),
        origin=Origin(xyz=(PIER_X, FRAME_CENTER_Y, PIER_CENTER_Z)),
        material="structural_steel",
        name="pier_1",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_CENTER_Y, TOP_BEAM_CENTER_Z)),
        material="structural_steel",
        name="top_beam",
    )
    frame.visual(
        Box((0.34, 0.14, 0.22)),
        origin=Origin(xyz=(0.0, -0.04, 2.69)),
        material="structural_steel",
        name="gear_pedestal",
    )
    frame.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, HOUSING_CENTER_Z)),
        material="machined_steel",
        name="housing_shell",
    )
    frame.visual(
        Cylinder(radius=WHEEL_BOSS_RADIUS, length=WHEEL_BOSS_LENGTH),
        origin=Origin(
            xyz=(0.0, HOUSING_FRONT_Y + WHEEL_BOSS_LENGTH / 2.0, HOUSING_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="machined_steel",
        name="wheel_boss",
    )
    frame.visual(
        Box((0.016, 0.020, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_HINGE_X - 0.020,
                HOUSING_FRONT_Y + 0.010,
                DOOR_CENTER_Z,
            )
        ),
        material="machined_steel",
        name="door_jamb",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(
            xyz=(DOOR_HINGE_X, HOUSING_FRONT_Y + 0.008, DOOR_CENTER_Z + 0.056),
        ),
        material="machined_steel",
        name="hinge_knuckle_0",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(
            xyz=(DOOR_HINGE_X, HOUSING_FRONT_Y + 0.008, DOOR_CENTER_Z - 0.056),
        ),
        material="machined_steel",
        name="hinge_knuckle_1",
    )

    _add_guide_rail(model, "guide_rail_0")
    _add_guide_rail(model, "guide_rail_1")

    gate = model.part("gate")
    gate.visual(
        Box((GATE_WIDTH, GATE_THICKNESS, GATE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material="gate_coating",
        name="leaf",
    )
    gate.visual(
        Box((0.09, 0.030, 1.56)),
        origin=Origin(xyz=(-0.30, 0.038, 0.0)),
        material="structural_steel",
        name="stiffener_0",
    )
    gate.visual(
        Box((0.09, 0.030, 1.56)),
        origin=Origin(xyz=(0.30, 0.038, 0.0)),
        material="structural_steel",
        name="stiffener_1",
    )
    gate.visual(
        Box((0.94, 0.050, 0.12)),
        origin=Origin(xyz=(0.0, 0.048, -0.79)),
        material="structural_steel",
        name="bottom_stiffener",
    )
    gate.visual(
        Box((0.10, 0.050, 0.16)),
        origin=Origin(xyz=(0.0, 0.048, 0.845)),
        material="structural_steel",
        name="lifting_lug",
    )
    gate.visual(
        Box((0.04, 0.050, 1.72)),
        origin=Origin(xyz=(-GATE_WIDTH / 2.0 + 0.02, 0.036, 0.0)),
        material="seal_black",
        name="seal_0",
    )
    gate.visual(
        Box((0.04, 0.050, 1.72)),
        origin=Origin(xyz=(GATE_WIDTH / 2.0 - 0.02, 0.036, 0.0)),
        material="seal_black",
        name="seal_1",
    )
    gate.visual(
        Box((0.03, 0.10, 1.72)),
        origin=Origin(xyz=(-0.590, 0.0, 0.0)),
        material="machined_steel",
        name="guide_shoe_0",
    )
    gate.visual(
        Box((0.03, 0.10, 1.72)),
        origin=Origin(xyz=(0.590, 0.0, 0.0)),
        material="machined_steel",
        name="guide_shoe_1",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_build_handwheel_shape(), "handwheel"),
        material="operator_wheel",
        name="wheel",
    )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.020, DOOR_THICKNESS / 2.0, 0.0)),
        material="gate_coating",
        name="panel",
    )
    inspection_door.visual(
        Box((0.030, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.025, DOOR_THICKNESS / 2.0, 0.0)),
        material="machined_steel",
        name="hinge_leaf",
    )
    inspection_door.visual(
        Box((0.014, DOOR_THICKNESS, 0.070)),
        origin=Origin(xyz=(0.016, DOOR_THICKNESS / 2.0, 0.0)),
        material="machined_steel",
        name="hinge_bridge",
    )
    inspection_door.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material="machined_steel",
        name="hinge_barrel",
    )
    inspection_door.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(
            xyz=(DOOR_WIDTH * 0.72, DOOR_THICKNESS + 0.010, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="machined_steel",
        name="latch",
    )

    model.articulation(
        "frame_to_guide_rail_0",
        ArticulationType.FIXED,
        parent=frame,
        child="guide_rail_0",
        origin=Origin(xyz=(-RAIL_X, 0.0, RAIL_CENTER_Z)),
    )
    model.articulation(
        "frame_to_guide_rail_1",
        ArticulationType.FIXED,
        parent=frame,
        child="guide_rail_1",
        origin=Origin(xyz=(RAIL_X, 0.0, RAIL_CENTER_Z), rpy=(0.0, math.pi, 0.0)),
    )
    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, GATE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=GATE_TRAVEL,
            effort=25000.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, HANDWHEEL_CENTER_Y, HOUSING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=3.0),
    )
    model.articulation(
        "frame_to_inspection_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=inspection_door,
        origin=Origin(xyz=(DOOR_HINGE_X, HOUSING_FRONT_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=20.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate")
    rail_0 = object_model.get_part("guide_rail_0")
    rail_1 = object_model.get_part("guide_rail_1")
    handwheel = object_model.get_part("handwheel")
    inspection_door = object_model.get_part("inspection_door")

    gate_slide = object_model.get_articulation("frame_to_gate")
    door_hinge = object_model.get_articulation("frame_to_inspection_door")

    ctx.expect_gap(
        gate,
        rail_0,
        axis="x",
        min_gap=0.010,
        max_gap=0.025,
        positive_elem="leaf",
        name="gate clears guide_rail_0 at rest",
    )
    ctx.expect_gap(
        rail_1,
        gate,
        axis="x",
        min_gap=0.010,
        max_gap=0.025,
        negative_elem="leaf",
        name="gate clears guide_rail_1 at rest",
    )
    ctx.expect_overlap(
        gate,
        rail_0,
        axes="z",
        min_overlap=1.70,
        name="closed gate remains deeply engaged in the guide rail",
    )
    ctx.expect_gap(
        handwheel,
        frame,
        axis="y",
        min_gap=0.060,
        max_gap=0.090,
        positive_elem="wheel",
        negative_elem="housing_shell",
        name="handwheel stands off in front of the gearbox housing",
    )
    ctx.expect_gap(
        inspection_door,
        frame,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="panel",
        negative_elem="housing_shell",
        name="inspection door closes flush on the housing face",
    )

    closed_gate_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: GATE_TRAVEL}):
        ctx.expect_gap(
            gate,
            rail_0,
            axis="x",
            min_gap=0.010,
            max_gap=0.025,
            positive_elem="leaf",
            name="raised gate clears guide_rail_0",
        )
        ctx.expect_gap(
            rail_1,
            gate,
            axis="x",
            min_gap=0.010,
            max_gap=0.025,
            negative_elem="leaf",
            name="raised gate clears guide_rail_1",
        )
        ctx.expect_overlap(
            gate,
            rail_0,
            axes="z",
            min_overlap=1.40,
            name="raised gate still remains guided by the rail",
        )
        raised_gate_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate lifts upward",
        closed_gate_pos is not None
        and raised_gate_pos is not None
        and raised_gate_pos[2] > closed_gate_pos[2] + 0.80,
        details=f"closed={closed_gate_pos}, raised={raised_gate_pos}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(inspection_door, elem="panel")
    with ctx.pose({door_hinge: 0.95}):
        open_door_aabb = ctx.part_element_world_aabb(inspection_door, elem="panel")

    ctx.check(
        "inspection door swings outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.06,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

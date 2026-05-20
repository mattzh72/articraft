from __future__ import annotations

import math

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


LINK_LENGTH = 0.34
LINK_WIDTH = 0.092
LINK_THICKNESS = 0.034
LAYER_STEP = 0.058
PIVOT_HOLE_RADIUS = 0.024
PIN_RADIUS = 0.014


def _link_plate(length: float, width: float, thickness: float, direction: int) -> cq.Workplane:
    """Dog-bone folding-arm plate with real pivot clearance holes."""
    sx = float(direction)
    half_depth = thickness / 2.0
    web = cq.Workplane("XY").center(sx * length / 2.0, 0.0).rect(length, width * 0.68).extrude(half_depth, both=True)
    proximal = cq.Workplane("XY").circle(width / 2.0).extrude(half_depth, both=True)
    distal = cq.Workplane("XY").center(sx * length, 0.0).circle(width / 2.0).extrude(half_depth, both=True)
    plate = proximal.union(web).union(distal)

    for x in (0.0, sx * length):
        cutter = cq.Workplane("XY").center(x, 0.0).circle(PIVOT_HOLE_RADIUS).extrude(thickness * 3.0, both=True)
        plate = plate.cut(cutter)

    return plate.edges("|Z").fillet(0.004)


def _add_section_visuals(
    part,
    *,
    direction: int,
    plate_material: Material,
    steel: Material,
    mesh_name: str,
    has_distal_pin: bool,
    has_tool_lug: bool = False,
) -> None:
    sx = float(direction)
    part.visual(
        mesh_from_cadquery(_link_plate(LINK_LENGTH, LINK_WIDTH, LINK_THICKNESS, direction), mesh_name),
        material=plate_material,
        name="plate",
    )

    # Thin darker reinforcing ribs make each member read as a light fabricated
    # arm link rather than a solid plank.  They are deliberately seated into the
    # plate so the part remains one connected manufactured piece.
    rib_length = LINK_LENGTH - 0.09
    for y, name in ((LINK_WIDTH * 0.31, "rib_a"), (-LINK_WIDTH * 0.31, "rib_b")):
        part.visual(
            Box((rib_length, 0.010, 0.012)),
            origin=Origin(xyz=(sx * LINK_LENGTH / 2.0, y, LINK_THICKNESS / 2.0 + 0.003)),
            material=steel,
            name=name,
        )

    if has_distal_pin:
        pin_length = LAYER_STEP + LINK_THICKNESS + 0.010
        washer_length = 0.010
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=pin_length),
            origin=Origin(xyz=(sx * LINK_LENGTH, 0.0, LAYER_STEP / 2.0), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name="distal_pin",
        )
        part.visual(
            Cylinder(radius=0.031, length=0.010),
            origin=Origin(xyz=(sx * LINK_LENGTH, 0.0, -LINK_THICKNESS / 2.0), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name="pin_head",
        )
        part.visual(
            Cylinder(radius=0.031, length=washer_length),
            origin=Origin(xyz=(sx * LINK_LENGTH, 0.0, LAYER_STEP - LINK_THICKNESS / 2.0 - washer_length / 2.0)),
            material=steel,
            name="upper_washer",
        )

    if has_tool_lug:
        part.visual(
            Box((0.076, 0.070, 0.028)),
            origin=Origin(xyz=(sx * (LINK_LENGTH + 0.020), 0.0, 0.0)),
            material=steel,
            name="tool_lug",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deployable_stacked_arm")

    graphite = model.material("graphite_powdercoat", rgba=(0.055, 0.060, 0.065, 1.0))
    black = model.material("matte_black", rgba=(0.010, 0.011, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.33, 0.08, 1.0))
    blue = model.material("deep_blue", rgba=(0.05, 0.22, 0.42, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.18, 0.036)),
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.018)),
        material=graphite,
        name="deck",
    )
    base.visual(
        Box((0.50, 0.215, 0.018)),
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.009)),
        material=black,
        name="foot_plate",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=graphite,
        name="shoulder_pedestal",
    )
    base.visual(
        Cylinder(radius=PIN_RADIUS, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=steel,
        name="shoulder_pin",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=steel,
        name="shoulder_pin_head",
    )

    section_0 = model.part("section_0")
    _add_section_visuals(
        section_0,
        direction=1,
        plate_material=orange,
        steel=steel,
        mesh_name="section_0_plate",
        has_distal_pin=True,
    )

    section_1 = model.part("section_1")
    _add_section_visuals(
        section_1,
        direction=-1,
        plate_material=blue,
        steel=steel,
        mesh_name="section_1_plate",
        has_distal_pin=True,
    )

    section_2 = model.part("section_2")
    _add_section_visuals(
        section_2,
        direction=1,
        plate_material=orange,
        steel=steel,
        mesh_name="section_2_plate",
        has_distal_pin=True,
    )

    section_3 = model.part("section_3")
    _add_section_visuals(
        section_3,
        direction=-1,
        plate_material=blue,
        steel=steel,
        mesh_name="section_3_plate",
        has_distal_pin=False,
        has_tool_lug=True,
    )

    hinge_limits = MotionLimits(effort=28.0, velocity=1.6, lower=0.0, upper=math.pi)
    model.articulation(
        "base_to_section_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=section_0,
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "section_0_to_section_1",
        ArticulationType.REVOLUTE,
        parent=section_0,
        child=section_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, LAYER_STEP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "section_1_to_section_2",
        ArticulationType.REVOLUTE,
        parent=section_1,
        child=section_2,
        origin=Origin(xyz=(-LINK_LENGTH, 0.0, LAYER_STEP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "section_2_to_section_3",
        ArticulationType.REVOLUTE,
        parent=section_2,
        child=section_3,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, LAYER_STEP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sections = [object_model.get_part(f"section_{i}") for i in range(4)]
    joints = [
        object_model.get_articulation("section_0_to_section_1"),
        object_model.get_articulation("section_1_to_section_2"),
        object_model.get_articulation("section_2_to_section_3"),
    ]

    ctx.check("four hinged arm sections", len(sections) == 4)

    # Compact/default pose: all dog-bone plates lie over the same base footprint
    # but are separated into distinct mechanical layers.
    for lower, upper, index in zip(sections[:-1], sections[1:], range(3)):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.014,
            max_gap=0.030,
            positive_elem="plate",
            negative_elem="plate",
            name=f"section_{index + 1}_stack_clearance",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            min_overlap=0.08,
            elem_a="plate",
            elem_b="plate",
            name=f"section_{index + 1}_compact_footprint",
        )

    with ctx.pose({joint: math.pi for joint in joints}):
        ctx.expect_origin_gap(
            sections[-1],
            sections[0],
            axis="x",
            min_gap=0.95,
            name="unfolded_tip_reaches_outward",
        )

    return ctx.report()


object_model = build_object_model()

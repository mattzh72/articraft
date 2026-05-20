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


LOW_LAYER_Z = 0.000
HIGH_LAYER_Z = 0.016
PLATE_THICKNESS = 0.008
BOSS_THICKNESS = 0.014
LINK_WIDTH = 0.018
BOSS_RADIUS = 0.018
PIN_HOLE_RADIUS = 0.007
PIN_RADIUS = 0.0072
PIN_HEAD_RADIUS = 0.012
PIN_HEAD_THICKNESS = 0.004


def _capsule_plate(
    length: float,
    *,
    width: float = LINK_WIDTH,
    thickness: float = PLATE_THICKNESS,
    boss_points: tuple[tuple[float, float], ...] = (),
    hole_points: tuple[tuple[float, float], ...] = (),
) -> cq.Workplane:
    """Flat narrow lever plate with rounded ends, raised bosses, and pin holes."""
    end_radius = width / 2.0
    plate = (
        cq.Workplane("XY")
        .center(length / 2.0, 0.0)
        .rect(length, width)
        .extrude(thickness)
        .union(cq.Workplane("XY").center(0.0, 0.0).circle(end_radius).extrude(thickness))
        .union(cq.Workplane("XY").center(length, 0.0).circle(end_radius).extrude(thickness))
        .translate((0.0, 0.0, -thickness / 2.0))
    )

    if boss_points:
        bosses = (
            cq.Workplane("XY")
            .pushPoints(list(boss_points))
            .circle(BOSS_RADIUS)
            .extrude(BOSS_THICKNESS)
            .translate((0.0, 0.0, -BOSS_THICKNESS / 2.0))
        )
        plate = plate.union(bosses)

    if hole_points:
        cutter_depth = BOSS_THICKNESS * 5.0
        holes = (
            cq.Workplane("XY")
            .pushPoints(list(hole_points))
            .circle(PIN_HOLE_RADIUS)
            .extrude(cutter_depth)
            .translate((0.0, 0.0, -cutter_depth / 2.0))
        )
        plate = plate.cut(holes)

    return plate


def _slot_cutter(center_x: float, slot_length: float, slot_width: float, depth: float) -> cq.Workplane:
    """Capsule-shaped through cutter for the tab slot."""
    straight_length = max(slot_length - slot_width, 0.001)
    radius = slot_width / 2.0
    return (
        cq.Workplane("XY")
        .center(center_x, 0.0)
        .rect(straight_length, slot_width)
        .extrude(depth)
        .union(
            cq.Workplane("XY")
            .center(center_x - straight_length / 2.0, 0.0)
            .circle(radius)
            .extrude(depth)
        )
        .union(
            cq.Workplane("XY")
            .center(center_x + straight_length / 2.0, 0.0)
            .circle(radius)
            .extrude(depth)
        )
        .translate((0.0, 0.0, -depth / 2.0))
    )


def _slotted_end_member(length: float) -> cq.Workplane:
    """Last lever member with a wider slotted pull tab at its distal end."""
    tab_length = 0.060
    tab_width = 0.038
    tab_start = length - tab_length
    tab_end = length

    body = _capsule_plate(
        tab_start + LINK_WIDTH * 0.25,
        boss_points=((0.0, 0.0),),
        hole_points=((0.0, 0.0),),
    )

    tab = (
        cq.Workplane("XY")
        .center((tab_start + tab_end) / 2.0, 0.0)
        .rect(tab_end - tab_start, tab_width)
        .extrude(PLATE_THICKNESS)
        .union(
            cq.Workplane("XY")
            .center(tab_start, 0.0)
            .circle(tab_width / 2.0)
            .extrude(PLATE_THICKNESS)
        )
        .union(
            cq.Workplane("XY")
            .center(tab_end, 0.0)
            .circle(tab_width / 2.0)
            .extrude(PLATE_THICKNESS)
        )
        .translate((0.0, 0.0, -PLATE_THICKNESS / 2.0))
    )

    body = body.union(tab)
    body = body.cut(_slot_cutter(center_x=length - 0.030, slot_length=0.040, slot_width=0.010, depth=0.060))
    return body


def _add_stacked_pin_visuals(part, x: float, *, owner_layer: str, material) -> None:
    """Add a vertical rivet/pin stack at a lever's distal revolute joint."""
    shaft_center_z = 0.008
    shaft_length = 0.039
    if owner_layer == "low":
        lower_head_z = -BOSS_THICKNESS / 2.0 - PIN_HEAD_THICKNESS / 2.0 + 0.0002
        # Leave a hairline clearance above the high child layer to avoid
        # unintended inter-part penetration while still reading as a captured pin.
        upper_head_z = HIGH_LAYER_Z + BOSS_THICKNESS / 2.0 + 0.0006 + PIN_HEAD_THICKNESS / 2.0
    else:
        # The high parent owns this pin, so the upper head lightly seats on its
        # own boss, while the lower head floats just below the low child boss.
        lower_head_z = -BOSS_THICKNESS / 2.0 - 0.0006 - PIN_HEAD_THICKNESS / 2.0
        upper_head_z = HIGH_LAYER_Z + BOSS_THICKNESS / 2.0 - 0.0002 + PIN_HEAD_THICKNESS / 2.0

    part.visual(
        Cylinder(radius=PIN_RADIUS, length=shaft_length),
        origin=Origin(xyz=(x, 0.0, shaft_center_z)),
        material=material,
        name="pivot_pin_shaft",
    )
    part.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(xyz=(x, 0.0, lower_head_z)),
        material=material,
        name="lower_pin_head",
    )
    part.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(xyz=(x, 0.0, upper_head_z)),
        material=material,
        name="upper_pin_head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_over_center_lever_train")

    root_mat = model.material("blackened_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    base_mat = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    lever_mat = model.material("satin_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    lever_dark_mat = model.material("blue_grey_steel", rgba=(0.33, 0.42, 0.48, 1.0))
    tab_mat = model.material("brushed_bronze", rgba=(0.72, 0.48, 0.24, 1.0))
    pin_mat = model.material("polished_pin_heads", rgba=(0.88, 0.86, 0.78, 1.0))

    # The root bracket is a fixed clevis-like saddle.  Its two cheeks flank the
    # first low-layer lever boss while a vertical pin marks the first pivot axis.
    root = model.part("root_bracket")
    root.visual(
        Box((0.130, 0.105, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.006)),
        material=base_mat,
        name="base_plate",
    )
    for y in (-0.030, 0.030):
        root.visual(
            Box((0.055, 0.015, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.012 + 0.035 / 2.0 - 0.0004)),
            material=root_mat,
            name=f"clevis_cheek_{'neg' if y < 0 else 'pos'}",
        )
    root.visual(
        Cylinder(radius=PIN_RADIUS, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.012 + 0.033 / 2.0)),
        material=pin_mat,
        name="root_pin_shaft",
    )
    root.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0468)),
        material=pin_mat,
        name="root_pin_cap",
    )
    for x in (-0.043, 0.055):
        for y in (-0.039, 0.039):
            root.visual(
                Cylinder(radius=0.0045, length=0.003),
                origin=Origin(xyz=(x, y, 0.0133)),
                material=pin_mat,
                name=f"mount_bolt_{x:+.3f}_{y:+.3f}",
            )

    l0 = 0.150
    l1 = 0.125
    l2 = 0.140

    lever_0 = model.part("lever_0")
    lever_0.visual(
        mesh_from_cadquery(
            _capsule_plate(l0, boss_points=((0.0, 0.0), (l0, 0.0)), hole_points=((0.0, 0.0), (l0, 0.0))),
            "lever_0_plate",
        ),
        material=lever_dark_mat,
        name="lever_body",
    )
    _add_stacked_pin_visuals(lever_0, l0, owner_layer="low", material=pin_mat)

    lever_1 = model.part("lever_1")
    lever_1.visual(
        mesh_from_cadquery(
            _capsule_plate(l1, boss_points=((0.0, 0.0), (l1, 0.0)), hole_points=((0.0, 0.0), (l1, 0.0))),
            "lever_1_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, HIGH_LAYER_Z)),
        material=lever_mat,
        name="lever_body",
    )
    _add_stacked_pin_visuals(lever_1, l1, owner_layer="high", material=pin_mat)

    lever_2 = model.part("lever_2")
    lever_2.visual(
        mesh_from_cadquery(_slotted_end_member(l2), "lever_2_slotted_member"),
        material=tab_mat,
        name="slotted_member",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=root,
        child=lever_0,
        origin=Origin(xyz=(0.0, 0.0, 0.026), rpy=(0.0, 0.0, math.radians(12.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=math.radians(-25.0), upper=math.radians(35.0), effort=18.0, velocity=2.0),
    )
    model.articulation(
        "center_pivot",
        ArticulationType.REVOLUTE,
        parent=lever_0,
        child=lever_1,
        origin=Origin(xyz=(l0, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(-35.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=math.radians(-42.0), upper=math.radians(46.0), effort=15.0, velocity=2.4),
    )
    model.articulation(
        "end_pivot",
        ArticulationType.REVOLUTE,
        parent=lever_1,
        child=lever_2,
        origin=Origin(xyz=(l1, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(48.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=math.radians(-40.0), upper=math.radians(44.0), effort=12.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_bracket")
    lever_0 = object_model.get_part("lever_0")
    lever_1 = object_model.get_part("lever_1")
    lever_2 = object_model.get_part("lever_2")
    root_pivot = object_model.get_articulation("root_pivot")
    center_pivot = object_model.get_articulation("center_pivot")
    end_pivot = object_model.get_articulation("end_pivot")

    joints = (root_pivot, center_pivot, end_pivot)
    ctx.check(
        "three serial revolute pivots",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and root_pivot.parent == "root_bracket"
        and root_pivot.child == "lever_0"
        and center_pivot.parent == "lever_0"
        and center_pivot.child == "lever_1"
        and end_pivot.parent == "lever_1"
        and end_pivot.child == "lever_2",
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "all pivot axes are normal to the lever plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    # The pin shafts are intentionally modeled as a tiny press-fit through the
    # visual bores.  This makes the support path explicit while keeping the
    # overlap local to the captured pin-and-hole interfaces.
    ctx.allow_overlap(
        root,
        lever_0,
        elem_a="root_pin_shaft",
        elem_b="lever_body",
        reason="The fixed bracket pin is intentionally captured through the first lever bore with a tiny press-fit.",
    )
    ctx.allow_overlap(
        lever_0,
        lever_1,
        elem_a="pivot_pin_shaft",
        elem_b="lever_body",
        reason="The first link's rivet shaft is intentionally captured through the second link bore.",
    )
    ctx.allow_overlap(
        lever_1,
        lever_2,
        elem_a="pivot_pin_shaft",
        elem_b="slotted_member",
        reason="The second link's rivet shaft is intentionally captured through the slotted member's pivot bore.",
    )

    ctx.expect_overlap(
        root,
        lever_0,
        axes="xyz",
        min_overlap=0.006,
        elem_a="root_pin_shaft",
        elem_b="lever_body",
        name="root pin passes through first lever boss",
    )
    ctx.expect_overlap(
        lever_0,
        lever_1,
        axes="xyz",
        min_overlap=0.006,
        elem_a="pivot_pin_shaft",
        elem_b="lever_body",
        name="center pin passes through second lever boss",
    )
    ctx.expect_overlap(
        lever_1,
        lever_2,
        axes="xyz",
        min_overlap=0.006,
        elem_a="pivot_pin_shaft",
        elem_b="slotted_member",
        name="end pin passes through slotted member boss",
    )
    ctx.expect_gap(
        lever_0,
        root,
        axis="z",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="lever_body",
        negative_elem="base_plate",
        name="first lever clears the fixed base plate",
    )
    ctx.expect_gap(
        lever_1,
        lever_0,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="lever_body",
        negative_elem="lever_body",
        name="first and second levers are visibly stacked",
    )
    ctx.expect_gap(
        lever_1,
        lever_2,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="lever_body",
        negative_elem="slotted_member",
        name="second and last levers are visibly stacked",
    )

    return ctx.report()


object_model = build_object_model()

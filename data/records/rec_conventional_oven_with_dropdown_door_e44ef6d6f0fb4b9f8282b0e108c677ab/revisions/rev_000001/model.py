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
    model = ArticulatedObject(name="wide_range_cooker_rotisserie")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = model.material("black_enamel", rgba=(0.035, 0.035, 0.038, 1.0))
    cast_iron = model.material("matte_cast_iron", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoky_glass", rgba=(0.08, 0.13, 0.16, 0.42))
    handle_metal = model.material("polished_handle", rgba=(0.86, 0.86, 0.82, 1.0))

    body = model.part("body")

    # A freestanding 1.2 m range body, built as connected panels around a real
    # hollow oven mouth instead of a single solid block.
    body.visual(Box((1.20, 0.68, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.100)), material=stainless, name="bottom_plinth")
    body.visual(Box((1.20, 0.68, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.865)), material=stainless, name="cooktop_deck")
    body.visual(Box((0.050, 0.68, 0.820)), origin=Origin(xyz=(-0.575, 0.0, 0.480)), material=stainless, name="side_panel_0")
    body.visual(Box((0.050, 0.68, 0.820)), origin=Origin(xyz=(0.575, 0.0, 0.480)), material=stainless, name="side_panel_1")
    body.visual(Box((1.20, 0.045, 0.820)), origin=Origin(xyz=(0.0, 0.318, 0.480)), material=stainless, name="rear_panel")

    # Front surround and control fascia sit just behind the hinged door.
    body.visual(Box((1.20, 0.038, 0.165)), origin=Origin(xyz=(0.0, -0.333, 0.780)), material=stainless, name="front_fascia")
    body.visual(Box((0.115, 0.038, 0.565)), origin=Origin(xyz=(-0.5425, -0.333, 0.435)), material=stainless, name="front_post_0")
    body.visual(Box((0.115, 0.038, 0.565)), origin=Origin(xyz=(0.5425, -0.333, 0.435)), material=stainless, name="front_post_1")
    body.visual(Box((1.20, 0.038, 0.085)), origin=Origin(xyz=(0.0, -0.333, 0.177)), material=stainless, name="front_sill")

    # Dark enamel liner surfaces inside the chamber make the cavity readable
    # through the transparent door glass.
    body.visual(Box((1.03, 0.018, 0.500)), origin=Origin(xyz=(0.0, 0.287, 0.455)), material=dark_steel, name="oven_back_liner")
    body.visual(Box((1.12, 0.555, 0.016)), origin=Origin(xyz=(0.0, -0.020, 0.205)), material=dark_steel, name="oven_floor")
    body.visual(Box((1.12, 0.555, 0.016)), origin=Origin(xyz=(0.0, -0.020, 0.692)), material=dark_steel, name="oven_ceiling")

    # A visible full-width lower hinge track and the two support shelves that
    # cradle the rotisserie spit inside the oven.
    body.visual(Cylinder(radius=0.010, length=1.12), origin=Origin(xyz=(0.0, -0.350, 0.185), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="body_hinge_barrel")
    body.visual(Box((0.060, 0.085, 0.008)), origin=Origin(xyz=(-0.528, -0.040, 0.454)), material=handle_metal, name="spit_shelf_0")
    body.visual(Box((0.060, 0.085, 0.008)), origin=Origin(xyz=(0.528, -0.040, 0.454)), material=handle_metal, name="spit_shelf_1")

    # Freestanding feet.
    for i, (x, y) in enumerate(((-0.48, -0.24), (0.48, -0.24), (-0.48, 0.24), (0.48, 0.24))):
        body.visual(Cylinder(radius=0.035, length=0.070), origin=Origin(xyz=(x, y, 0.035)), material=dark_steel, name=f"foot_{i}")

    # Five burner positions on the wide cooktop, each with a burner cap and a
    # simple cast-iron cross grate that visibly sits on the top plate.
    burners = [(-0.38, -0.18), (0.0, -0.18), (0.38, -0.18), (-0.24, 0.17), (0.24, 0.17)]
    for i, (x, y) in enumerate(burners):
        radius = 0.072 if i == 1 else 0.060
        body.visual(Cylinder(radius=radius, length=0.012), origin=Origin(xyz=(x, y, 0.904)), material=cast_iron, name=f"burner_cap_{i}")
        body.visual(Box((radius * 2.45, 0.018, 0.016)), origin=Origin(xyz=(x, y, 0.918)), material=cast_iron, name=f"burner_grate_x_{i}")
        body.visual(Box((0.018, radius * 2.45, 0.016)), origin=Origin(xyz=(x, y, 0.918)), material=cast_iron, name=f"burner_grate_y_{i}")

    door = model.part("door")
    # The child frame lies on the bottom hinge line; the closed door rises along
    # local +Z and drops outward toward -Y for positive revolute motion.
    door.visual(Cylinder(radius=0.026, length=1.10), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="door_hinge_barrel")
    door.visual(Box((1.06, 0.050, 0.085)), origin=Origin(xyz=(0.0, 0.0, 0.0425)), material=stainless, name="door_bottom_rail")
    door.visual(Box((1.06, 0.050, 0.085)), origin=Origin(xyz=(0.0, 0.0, 0.4675)), material=stainless, name="door_top_rail")
    door.visual(Box((0.090, 0.050, 0.510)), origin=Origin(xyz=(-0.485, 0.0, 0.255)), material=stainless, name="door_side_0")
    door.visual(Box((0.090, 0.050, 0.510)), origin=Origin(xyz=(0.485, 0.0, 0.255)), material=stainless, name="door_side_1")
    door.visual(Box((0.900, 0.012, 0.375)), origin=Origin(xyz=(0.0, -0.030, 0.255)), material=glass, name="glass_panel")
    door.visual(Cylinder(radius=0.014, length=0.080), origin=Origin(xyz=(-0.410, -0.058, 0.400), rpy=(math.pi / 2, 0.0, 0.0)), material=handle_metal, name="handle_standoff_0")
    door.visual(Cylinder(radius=0.014, length=0.080), origin=Origin(xyz=(0.410, -0.058, 0.400), rpy=(math.pi / 2, 0.0, 0.0)), material=handle_metal, name="handle_standoff_1")
    door.visual(Cylinder(radius=0.018, length=0.920), origin=Origin(xyz=(0.0, -0.098, 0.400), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="bar_handle")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.386, 0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    spit = model.part("spit")
    spit.visual(Cylinder(radius=0.012, length=1.080), origin=Origin(rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="spit_rod")
    spit.visual(Cylinder(radius=0.024, length=0.038), origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="fork_collar_0")
    spit.visual(Cylinder(radius=0.024, length=0.038), origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="fork_collar_1")
    spit.visual(Cylinder(radius=0.004, length=0.145), origin=Origin(xyz=(-0.130, -0.014, 0.014), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="fork_tine_0")
    spit.visual(Cylinder(radius=0.004, length=0.145), origin=Origin(xyz=(-0.130, 0.014, 0.014), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="fork_tine_1")
    spit.visual(Cylinder(radius=0.004, length=0.145), origin=Origin(xyz=(0.130, -0.014, 0.014), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="fork_tine_2")
    spit.visual(Cylinder(radius=0.004, length=0.145), origin=Origin(xyz=(0.130, 0.014, 0.014), rpy=(0.0, math.pi / 2, 0.0)), material=handle_metal, name="fork_tine_3")

    model.articulation(
        "body_to_spit",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spit,
        origin=Origin(xyz=(0.0, -0.040, 0.470)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )

    for i, x in enumerate((-0.43, -0.215, 0.0, 0.215, 0.43)):
        knob = model.part(f"knob_{i}")
        knob.visual(Cylinder(radius=0.032, length=0.006), origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=stainless, name="knob_skirt")
        knob.visual(Cylinder(radius=0.026, length=0.024), origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=stainless, name="knob_cap")
        knob.visual(Box((0.006, 0.006, 0.024)), origin=Origin(xyz=(0.0, -0.026, 0.012)), material=dark_steel, name="knob_marker")
        model.articulation(
            f"body_to_knob_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, -0.352, 0.805)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-2.45, upper=2.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    spit = object_model.get_part("spit")
    door_joint = object_model.get_articulation("body_to_door")
    spit_joint = object_model.get_articulation("body_to_spit")

    ctx.expect_contact(
        door,
        body,
        elem_a="door_hinge_barrel",
        elem_b="body_hinge_barrel",
        contact_tol=0.002,
        name="full-width bottom hinge barrels meet",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_fascia",
        negative_elem="door_top_rail",
        min_gap=0.004,
        max_gap=0.025,
        name="closed door sits just proud of the fascia",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="x",
        elem_a="glass_panel",
        elem_b="front_fascia",
        min_overlap=0.80,
        name="wide glass door spans the oven opening",
    )

    ctx.expect_contact(
        spit,
        body,
        elem_a="spit_rod",
        elem_b="spit_shelf_0",
        contact_tol=0.002,
        name="spit rests in one side support",
    )
    ctx.expect_contact(
        spit,
        body,
        elem_a="spit_rod",
        elem_b="spit_shelf_1",
        contact_tol=0.002,
        name="spit rests in opposite side support",
    )
    ctx.expect_overlap(
        spit,
        body,
        axes="x",
        elem_a="spit_rod",
        elem_b="oven_floor",
        min_overlap=1.00,
        name="spit spans nearly the full oven chamber width",
    )
    ctx.expect_gap(
        spit,
        body,
        axis="z",
        positive_elem="spit_rod",
        negative_elem="oven_floor",
        min_gap=0.22,
        max_gap=0.27,
        name="rotisserie spit is at chamber mid-height above the floor",
    )
    ctx.expect_gap(
        body,
        spit,
        axis="z",
        positive_elem="oven_ceiling",
        negative_elem="spit_rod",
        min_gap=0.18,
        max_gap=0.23,
        name="rotisserie spit clears the chamber ceiling",
    )

    knob_joints = [object_model.get_articulation(f"body_to_knob_{i}") for i in range(5)]
    ctx.check(
        "five articulated front control knobs",
        all(j.motion_limits is not None for j in knob_joints),
        details="Expected five individual revolute knob controls on the fascia.",
    )

    def _coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door drops outward and downward on bottom hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and _coord(open_door_aabb[0], 1) < _coord(closed_door_aabb[0], 1) - 0.24
        and _coord(open_door_aabb[1], 2) < _coord(closed_door_aabb[1], 2) - 0.25,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_tine = ctx.part_element_world_aabb(spit, elem="fork_tine_0")
    with ctx.pose({spit_joint: math.pi / 2}):
        rotated_tine = ctx.part_element_world_aabb(spit, elem="fork_tine_0")
    ctx.check(
        "rotisserie fork rotates about the spit rod",
        closed_tine is not None
        and rotated_tine is not None
        and abs(
            ((_coord(rotated_tine[0], 2) + _coord(rotated_tine[1], 2)) * 0.5)
            - ((_coord(closed_tine[0], 2) + _coord(closed_tine[1], 2)) * 0.5)
        )
        > 0.015,
        details=f"closed_tine={closed_tine}, rotated_tine={rotated_tine}",
    )

    return ctx.report()


object_model = build_object_model()

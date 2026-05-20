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
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_X = 0.32
BODY_Y = 0.28
BODY_H = 0.16
HINGE_X = 0.150
HINGE_Z = 0.177
PANEL_X = -0.173


def _rounded_box_mesh(size_x: float, size_y: float, size_z: float, radius: float, name: str):
    shape = (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z)
        .edges("|Z")
        .fillet(radius)
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0012, angular_tolerance=0.08)


def _rect_ring_mesh(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    name: str,
):
    ring = (
        cq.Workplane("XY")
        .rect(outer_x, outer_y)
        .rect(inner_x, inner_y)
        .extrude(height)
    )
    return mesh_from_cadquery(ring, name, tolerance=0.001, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    warm_white = model.material("warm_white", rgba=(0.92, 0.89, 0.82, 1.0))
    seam_black = model.material("seam_black", rgba=(0.02, 0.022, 0.025, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.085, 0.09, 1.0))
    metal = model.material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.20, 0.21, 1.0))
    knob_gray = model.material("knob_gray", rgba=(0.74, 0.73, 0.70, 1.0))
    red = model.material("red_pointer", rgba=(0.86, 0.08, 0.04, 1.0))
    amber = model.material("cook_amber", rgba=(1.0, 0.56, 0.07, 1.0))
    green = model.material("warm_green", rgba=(0.16, 0.72, 0.27, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_box_mesh(BODY_X, BODY_Y, BODY_H, 0.055, "body_shell_mesh"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material=warm_white,
        name="body_shell",
    )

    # Slightly inset/dark front control field; it is volume-embedded into the
    # outer housing so the dial reads as mounted below the latch.
    body.visual(
        Box((0.008, 0.160, 0.106)),
        origin=Origin(xyz=(-0.163, 0.0, 0.075)),
        material=control_black,
        name="control_panel",
    )
    body.visual(
        Box((0.007, 0.070, 0.014)),
        origin=Origin(xyz=(-0.160, 0.0, 0.140)),
        material=seam_black,
        name="latch_receiver",
    )

    # Inner pot rim and dark well are visible when the rear-hinged lid opens.
    pot_rim = (
        cq.Workplane("XY")
        .circle(0.106)
        .circle(0.088)
        .extrude(0.007)
    )
    body.visual(
        mesh_from_cadquery(pot_rim, "inner_pot_rim_mesh", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=metal,
        name="inner_pot_rim",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=seam_black,
        name="inner_well",
    )

    # Small cook/warm dots flank the selector without adding extra moving controls.
    for y, mat, visual_name in ((-0.045, amber, "cook_dot"), (0.045, green, "warm_dot")):
        body.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(-0.167, y, 0.037), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mat,
            name=visual_name,
        )

    # Non-overlapping outer hinge barrels are fixed to the rear housing.
    for y, visual_name in ((-0.091, "hinge_barrel_0"), (0.091, "hinge_barrel_1")):
        body.visual(
            Cylinder(radius=0.011, length=0.054),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name=visual_name,
        )
        body.visual(
            Box((0.025, 0.054, 0.010)),
            origin=Origin(xyz=(HINGE_X - 0.010, y, HINGE_Z - 0.012)),
            material=dark_gray,
            name=f"hinge_leaf_{0 if y < 0 else 1}",
        )

    for x in (-0.095, 0.095):
        for y in (-0.085, 0.085):
            body.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(x, y, -0.002)),
                material=seam_black,
                name=f"foot_{'front' if x < 0 else 'rear'}_{'a' if y < 0 else 'b'}",
            )

    lid = model.part("lid")
    lid_shape = (
        cq.Workplane("XY")
        .box(0.280, 0.260, 0.050)
        .edges("|Z")
        .fillet(0.045)
        .translate((-0.160, 0.0, 0.010))
    )
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid_shell_mesh", tolerance=0.0012, angular_tolerance=0.08),
        material=warm_white,
        name="lid_shell",
    )
    seam_ring = (
        cq.Workplane("XY")
        .rect(0.304, 0.262)
        .rect(0.276, 0.236)
        .extrude(0.004)
        .translate((-0.150, 0.0, -0.017))
    )
    lid.visual(
        mesh_from_cadquery(seam_ring, "lid_seam_mesh", tolerance=0.001),
        material=seam_black,
        name="seam_gasket",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(-0.105, 0.070, 0.037)),
        material=dark_gray,
        name="steam_vent",
    )
    for y in (0.061, 0.070, 0.079):
        lid.visual(
            Box((0.026, 0.003, 0.003)),
            origin=Origin(xyz=(-0.105, y, 0.041)),
            material=seam_black,
            name=f"vent_slot_{int(round(y * 1000))}",
        )
    lid.visual(
        Box((0.026, 0.078, 0.024)),
        origin=Origin(xyz=(-0.313, 0.0, 0.000)),
        material=dark_gray,
        name="lid_latch",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.026, 0.104, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, -0.012)),
        material=dark_gray,
        name="hinge_leaf",
    )

    selector_dial = model.part("selector_dial")
    dial_geometry = KnobGeometry(
        0.060,
        0.024,
        body_style="skirted",
        top_diameter=0.047,
        edge_radius=0.001,
        skirt=KnobSkirt(0.068, 0.005, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
    )
    selector_dial.visual(
        mesh_from_geometry(dial_geometry, "selector_dial_mesh"),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_gray,
        name="dial_cap",
    )
    selector_dial.visual(
        Box((0.003, 0.006, 0.030)),
        origin=Origin(xyz=(-0.025, 0.0, 0.012)),
        material=red,
        name="dial_pointer",
    )
    selector_dial.visual(
        Cylinder(radius=0.010, length=0.007),
        origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="dial_axle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # The closed lid extends along local -X from the rear hinge line.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(PANEL_X, 0.0, 0.075)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    selector_dial = object_model.get_part("selector_dial")
    hinge = object_model.get_articulation("body_to_lid")
    selector_joint = object_model.get_articulation("body_to_selector")

    ctx.allow_overlap(
        body,
        selector_dial,
        elem_a="control_panel",
        elem_b="dial_axle",
        reason="The selector dial's central axle is intentionally seated slightly inside the front control panel socket.",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_dims = tuple(body_aabb[1][i] - body_aabb[0][i] for i in range(3))
        ctx.check(
            "compact kitchen appliance scale",
            0.25 <= body_dims[0] <= 0.38
            and 0.22 <= body_dims[1] <= 0.34
            and 0.14 <= body_dims[2] <= 0.24,
            details=f"body_dims={body_dims}",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="seam_gasket",
        negative_elem="body_shell",
        max_gap=0.003,
        max_penetration=0.00001,
        name="closed lid seam sits just above housing",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="seam_gasket",
        elem_b="body_shell",
        min_overlap=0.20,
        name="lid footprint covers the cooker body",
    )

    ctx.expect_gap(
        body,
        selector_dial,
        axis="x",
        positive_elem="control_panel",
        negative_elem="dial_cap",
        max_gap=0.004,
        max_penetration=0.0,
        name="selector dial seats on the front control panel",
    )
    ctx.expect_overlap(
        selector_dial,
        body,
        axes="yz",
        elem_a="dial_cap",
        elem_b="control_panel",
        min_overlap=0.045,
        name="selector dial is centered in the control area",
    )
    ctx.expect_overlap(
        selector_dial,
        body,
        axes="x",
        elem_a="dial_axle",
        elem_b="control_panel",
        min_overlap=0.0005,
        name="selector axle remains inserted in the panel",
    )
    ctx.expect_within(
        selector_dial,
        body,
        axes="yz",
        inner_elem="dial_axle",
        outer_elem="control_panel",
        margin=0.0,
        name="selector axle is centered within the control panel",
    )

    dial_bb = ctx.part_element_world_aabb(selector_dial, elem="dial_cap")
    latch_bb = ctx.part_element_world_aabb(body, elem="latch_receiver")
    if dial_bb is not None and latch_bb is not None:
        ctx.check(
            "selector dial sits below the latch",
            dial_bb[1][2] < latch_bb[0][2] - 0.010,
            details=f"dial={dial_bb}, latch={latch_bb}",
        )

    rest_latch = ctx.part_element_world_aabb(lid, elem="lid_latch")
    with ctx.pose({hinge: 1.10}):
        open_latch = ctx.part_element_world_aabb(lid, elem="lid_latch")
    if rest_latch is not None and open_latch is not None:
        ctx.check(
            "rear hinge lifts the front latch upward",
            open_latch[0][2] > rest_latch[0][2] + 0.12,
            details=f"rest={rest_latch}, open={open_latch}",
        )

    ctx.check(
        "selector joint is continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={selector_joint.articulation_type}",
    )
    ctx.check(
        "lid hinge has realistic opening limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and 1.0 <= hinge.motion_limits.upper <= 1.5,
        details=f"limits={hinge.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()

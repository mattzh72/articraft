from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.34
BODY_WIDTH = 0.44
BODY_HEIGHT = 0.72
RIM_DEPTH = 0.38
RIM_WIDTH = 0.48
RIM_TOP_Z = 0.735
HINGE_X = RIM_DEPTH / 2.0


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular slab centered at the local origin."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _body_shell() -> cq.Workplane:
    wall = 0.025
    bottom = 0.055

    outer = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.022)
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(BODY_DEPTH - 2.0 * wall, BODY_WIDTH - 2.0 * wall, BODY_HEIGHT + 0.08)
        .translate((0.0, 0.0, bottom + (BODY_HEIGHT + 0.08) / 2.0))
    )
    shell = outer.cut(inner_cut)

    rim_outer = (
        cq.Workplane("XY")
        .box(RIM_DEPTH, RIM_WIDTH, 0.045)
        .translate((0.0, 0.0, RIM_TOP_Z - 0.045 / 2.0))
        .edges("|Z")
        .fillet(0.026)
    )
    rim_inner = (
        cq.Workplane("XY")
        .box(BODY_DEPTH - 0.050, BODY_WIDTH - 0.055, 0.075)
        .translate((0.0, 0.0, RIM_TOP_Z - 0.045 / 2.0))
    )
    rim = rim_outer.cut(rim_inner)

    return shell.union(rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_waste_bin")

    yellow = model.material("clinical_yellow_plastic", rgba=(0.92, 0.76, 0.12, 1.0))
    dark = model.material("dark_gray_rubber", rgba=(0.05, 0.055, 0.055, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.36, 0.37, 0.36, 1.0))
    red = model.material("warning_red", rgba=(0.78, 0.05, 0.035, 1.0))
    white = model.material("clean_white", rgba=(0.96, 0.96, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.001),
        material=yellow,
        name="body_shell",
    )
    body.visual(
        Box((0.405, 0.500, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="base_plinth",
    )
    body.visual(
        Box((0.060, 0.035, 0.120)),
        origin=Origin(xyz=(-0.205, 0.205, 0.080)),
        material=hinge_gray,
        name="pivot_bracket_0",
    )
    body.visual(
        Box((0.060, 0.035, 0.120)),
        origin=Origin(xyz=(-0.205, -0.205, 0.080)),
        material=hinge_gray,
        name="pivot_bracket_1",
    )
    for idx, y in enumerate((-0.160, 0.0, 0.160)):
        body.visual(
            Box((0.034, 0.060, 0.050)),
            origin=Origin(xyz=(0.188, y, 0.710)),
            material=hinge_gray,
            name=f"rear_hinge_block_{idx}",
        )
    body.visual(
        Box((0.004, 0.205, 0.125)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 - 0.002, 0.0, 0.430)),
        material=red,
        name="warning_label",
    )
    body.visual(
        Box((0.003, 0.118, 0.020)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 - 0.0055, 0.0, 0.430)),
        material=white,
        name="label_cross_bar",
    )
    body.visual(
        Box((0.003, 0.030, 0.092)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 - 0.006, 0.0, 0.430)),
        material=white,
        name="label_cross_stem",
    )

    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_cadquery(_rounded_box((0.185, 0.360, 0.024), 0.026), "pedal_plate"),
        origin=Origin(xyz=(-0.0925, 0.0, -0.025)),
        material=dark,
        name="pedal_plate",
    )
    pedal.visual(
        Cylinder(radius=0.018, length=0.375),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="pedal_axle",
    )
    for idx, x in enumerate((-0.155, -0.120, -0.085, -0.050)):
        pedal.visual(
            Box((0.012, 0.305, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.010)),
            material=hinge_gray,
            name=f"pedal_tread_{idx}",
        )

    lid = model.part("lid")
    lid_depth = 0.390
    lid_width = 0.500
    lid_thickness = 0.035
    lid.visual(
        mesh_from_cadquery(
            _rounded_box((lid_depth, lid_width, lid_thickness), 0.030),
            "lid_panel",
            tolerance=0.001,
        ),
        origin=Origin(xyz=(-lid_depth / 2.0, 0.0, lid_thickness / 2.0)),
        material=yellow,
        name="lid_panel",
    )
    lid.visual(
        Box((0.018, 0.420, 0.040)),
        origin=Origin(xyz=(-0.397, 0.0, -0.016)),
        material=yellow,
        name="front_skirt",
    )
    lid.visual(
        Box((0.340, 0.018, 0.040)),
        origin=Origin(xyz=(-0.200, 0.257, -0.016)),
        material=yellow,
        name="side_skirt_0",
    )
    lid.visual(
        Box((0.340, 0.018, 0.040)),
        origin=Origin(xyz=(-0.200, -0.257, -0.016)),
        material=yellow,
        name="side_skirt_1",
    )
    for idx, (y, length) in enumerate(((-0.172, 0.095), (0.0, 0.095), (0.172, 0.095))):
        lid.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(-0.004, y, 0.019), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_gray,
            name=f"lid_hinge_barrel_{idx}",
        )

    pedal_joint = model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(-0.205, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5, lower=-0.22, upper=0.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, RIM_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.85),
        mimic=Mimic(pedal_joint.name, multiplier=-3.8, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    pedal_joint = object_model.get_articulation("body_to_pedal")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.28,
        name="flap lid covers the bin opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="lid panel rests on the top rim",
    )
    ctx.expect_gap(
        body,
        pedal,
        axis="x",
        positive_elem="body_shell",
        negative_elem="pedal_plate",
        min_gap=0.0,
        max_gap=0.040,
        name="pedal plate sits just in front of body",
    )
    ctx.expect_contact(
        pedal,
        body,
        elem_a="pedal_axle",
        elem_b="pivot_bracket_0",
        contact_tol=0.004,
        name="pedal axle reaches one pivot bracket",
    )
    ctx.expect_contact(
        pedal,
        body,
        elem_a="pedal_axle",
        elem_b="pivot_bracket_1",
        contact_tol=0.004,
        name="pedal axle reaches opposite pivot bracket",
    )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    with ctx.pose({pedal_joint: -0.20}):
        pressed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")

    ctx.check(
        "pedal motion opens the rear-hinged lid",
        rest_lid_aabb is not None
        and pressed_lid_aabb is not None
        and pressed_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.14,
        details=f"rest_lid_aabb={rest_lid_aabb}, pressed_lid_aabb={pressed_lid_aabb}",
    )
    ctx.check(
        "pedal free edge rotates downward",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.020,
        details=f"rest_pedal_aabb={rest_pedal_aabb}, pressed_pedal_aabb={pressed_pedal_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

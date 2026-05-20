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


CASE_LENGTH = 0.70
CASE_DEPTH = 0.22
LOWER_HEIGHT = 0.060
LID_THICKNESS = 0.030
WALL = 0.014
HINGE_Y = CASE_DEPTH / 2.0 + 0.010
HINGE_Z = LOWER_HEIGHT + 0.004
FRONT_Y = -CASE_DEPTH / 2.0


def _rounded_box(
    length: float,
    depth: float,
    height: float,
    *,
    center: tuple[float, float, float],
    radius: float,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, depth, height)
    if radius > 0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(center)


def _lower_shell_shape() -> cq.Workplane:
    outer = _rounded_box(
        CASE_LENGTH,
        CASE_DEPTH,
        LOWER_HEIGHT,
        center=(0.0, 0.0, LOWER_HEIGHT / 2.0),
        radius=0.026,
    )
    cutter_height = LOWER_HEIGHT + 0.024
    cutter = _rounded_box(
        CASE_LENGTH - 2.0 * WALL,
        CASE_DEPTH - 2.0 * WALL,
        cutter_height,
        center=(0.0, 0.0, 0.012 + cutter_height / 2.0),
        radius=0.017,
    )
    return outer.cut(cutter)


def _lid_panel_shape() -> cq.Workplane:
    # The lid's part frame is the rear hinge axis.  At q=0 it spans forward
    # along local -Y, with the flat bottom just seated on the lower shell rim.
    panel_center_y = -HINGE_Y
    panel_center_z = LOWER_HEIGHT + LID_THICKNESS / 2.0 - HINGE_Z
    panel = _rounded_box(
        CASE_LENGTH,
        CASE_DEPTH,
        LID_THICKNESS,
        center=(0.0, panel_center_y, panel_center_z),
        radius=0.024,
    )
    # A very shallow flat inset on the top keeps the silhouette flat while
    # giving the molded case surface some visible treatment.
    inset = _rounded_box(
        CASE_LENGTH - 0.095,
        CASE_DEPTH - 0.070,
        0.003,
        center=(0.0, panel_center_y, panel_center_z + LID_THICKNESS / 2.0 + 0.001),
        radius=0.018,
    )
    return panel.union(inset)


def _lid_lip_shape() -> cq.Workplane:
    panel_center_y = -HINGE_Y
    lip_center_z = LOWER_HEIGHT - HINGE_Z - 0.006
    return _rounded_box(
        CASE_LENGTH - 2.0 * WALL - 0.020,
        CASE_DEPTH - 2.0 * WALL - 0.008,
        0.014,
        center=(0.0, panel_center_y, lip_center_z),
        radius=0.012,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clarinet_case")

    molded_black = Material("molded_black", rgba=(0.015, 0.014, 0.012, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    plush = Material("burgundy_plush", rgba=(0.22, 0.015, 0.045, 1.0))
    brass = Material("aged_brass", rgba=(0.77, 0.56, 0.22, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_cadquery(_lower_shell_shape(), "lower_shell"),
        material=molded_black,
        name="shell",
    )
    lower.visual(
        Box((CASE_LENGTH - 2.0 * WALL - 0.018, CASE_DEPTH - 2.0 * WALL - 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=plush,
        name="velvet_liner",
    )
    for x in (-0.19, 0.19):
        lower.visual(
            Box((0.12, 0.046, 0.018)),
            origin=Origin(xyz=(x, -0.010, 0.027)),
            material=plush,
            name=f"clarinet_cradle_{0 if x < 0 else 1}",
        )

    # Alternating full-length piano hinge knuckles.  The hinge axis is exposed
    # just behind the rear edge, while each leaf is vertically offset to avoid
    # the moving lid hardware colliding with the fixed shell leaf.
    for i, (x, length) in enumerate(((-0.285, 0.110), (0.000, 0.200), (0.285, 0.110))):
        lower.visual(
            Box((length, 0.006, 0.012)),
            origin=Origin(xyz=(x, HINGE_Y - 0.007, HINGE_Z - 0.011)),
            material=brass,
            name=f"hinge_leaf_{i}",
        )
        lower.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"hinge_barrel_{i}",
        )

    latch_xs = (-0.215, 0.215)
    for i, x in enumerate(latch_xs):
        lower.visual(
            Box((0.088, 0.004, 0.036)),
            origin=Origin(xyz=(x, FRONT_Y - 0.002, 0.038)),
            material=brass,
            name=f"latch_mount_{i}",
        )
        for side, sx in enumerate((-0.034, 0.034)):
            lower.visual(
                Box((0.008, 0.018, 0.018)),
                origin=Origin(xyz=(x + sx, FRONT_Y - 0.006, 0.058)),
                material=brass,
                name=f"latch_ear_{i}_{side}",
            )
        lower.visual(
            Box((0.050, 0.006, 0.012)),
            origin=Origin(xyz=(x, FRONT_Y - 0.003, 0.010)),
            material=brass,
            name=f"latch_strike_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel_shape(), "lid_panel"),
        material=molded_black,
        name="lid_panel",
    )
    lid.visual(
        mesh_from_cadquery(_lid_lip_shape(), "lid_inner_lip"),
        material=black_rubber,
        name="inner_lip",
    )
    for i, (x, length) in enumerate(((-0.165, 0.110), (0.165, 0.110))):
        lid.visual(
            Box((length, 0.006, 0.010)),
            origin=Origin(xyz=(x, -0.007, 0.001)),
            material=brass,
            name=f"hinge_leaf_{i}",
        )
        lid.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"hinge_barrel_{i}",
        )

    hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.30),
    )
    hinge.meta["description"] = "Full-length rear piano hinge; positive rotation opens the lid upward."

    for i, x in enumerate(latch_xs):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="pivot_knuckle",
        )
        latch.visual(
            Box((0.054, 0.006, 0.052)),
            origin=Origin(xyz=(0.0, -0.006, -0.032)),
            material=brass,
            name="latch_plate",
        )
        latch.visual(
            Box((0.043, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, -0.009, -0.060)),
            material=brass,
            name="hook_lip",
        )
        model.articulation(
            f"latch_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=latch,
            origin=Origin(xyz=(x, FRONT_Y - 0.006, 0.058)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=0.0, upper=1.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")
    latch_pivot_0 = object_model.get_articulation("latch_pivot_0")
    latch_pivot_1 = object_model.get_articulation("latch_pivot_1")

    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="shell",
        max_gap=0.003,
        max_penetration=0.001,
        name="flat lid sits on lower shell rim",
    )
    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="lid_panel",
        elem_b="shell",
        min_overlap=0.18,
        name="lid spans the case opening",
    )
    ctx.expect_within(
        lid,
        lower,
        axes="xy",
        inner_elem="inner_lip",
        outer_elem="shell",
        margin=0.004,
        name="lid lip nests inside lower shell",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "rear hinge opens lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for i, (latch, pivot) in enumerate(((latch_0, latch_pivot_0), (latch_1, latch_pivot_1))):
        closed_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
        with ctx.pose({pivot: 1.0}):
            swung_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
        ctx.check(
            f"latch {i} swings outward on front pivot",
            closed_aabb is not None
            and swung_aabb is not None
            and swung_aabb[0][1] < closed_aabb[0][1] - 0.018,
            details=f"closed={closed_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

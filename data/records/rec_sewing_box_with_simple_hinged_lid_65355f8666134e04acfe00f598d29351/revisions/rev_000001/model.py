from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.300
BODY_WIDTH = 0.200
BODY_HEIGHT = 0.095
WALL = 0.006
LID_THICKNESS = 0.012
HINGE_X = -BODY_LENGTH / 2.0 - 0.008
HINGE_Z = BODY_HEIGHT + 0.006


def _make_body_shell() -> cq.Workplane:
    """A compact hollow sewing-box tray with low, built-in divider ribs."""

    outer = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .shell(-WALL)
    )

    divider_height = 0.035
    divider_overlap = 0.001
    long_divider = cq.Workplane("XY").box(
        0.205,
        0.004,
        divider_height + divider_overlap,
    ).translate((0.025, 0.000, WALL + divider_height / 2.0 - divider_overlap))
    short_divider = cq.Workplane("XY").box(
        0.004,
        0.090,
        divider_height + divider_overlap,
    ).translate((-0.055, -0.045, WALL + divider_height / 2.0 - divider_overlap))

    return outer.union(long_divider).union(short_divider)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sewing_box")

    model.material("matte_sage_plastic", rgba=(0.42, 0.55, 0.49, 1.0))
    model.material("warm_ivory", rgba=(0.86, 0.81, 0.70, 1.0))
    model.material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    model.material("felt_red", rgba=(0.72, 0.12, 0.14, 1.0))
    model.material("shadow_recess", rgba=(0.08, 0.07, 0.06, 1.0))

    box = model.part("box")
    box.visual(
        mesh_from_cadquery(_make_body_shell(), "sewing_box_body"),
        material="matte_sage_plastic",
        name="body_shell",
    )

    # Built-in sewing features are fixed to the tray floor so they stow under
    # the lid without creating loose, floating contents.
    box.visual(
        Box((0.065, 0.045, 0.016)),
        origin=Origin(xyz=(-0.095, 0.055, WALL + 0.008)),
        material="felt_red",
        name="pin_cushion",
    )
    for idx, x in enumerate((0.025, 0.060, 0.095)):
        box.visual(
            Cylinder(radius=0.003, length=0.035),
            origin=Origin(xyz=(x, 0.052, WALL + 0.0170)),
            material="warm_ivory",
            name=f"spool_peg_{idx}",
        )

    # Rear hinge leaf and two fixed outer knuckles.  The hinge axis is the same
    # physical line used by the lid joint below.
    box.visual(
        Box((0.004, 0.194, 0.024)),
        origin=Origin(xyz=(-BODY_LENGTH / 2.0 - 0.001, 0.0, BODY_HEIGHT - 0.010)),
        material="brushed_steel",
        name="rear_hinge_leaf",
    )
    box.visual(
        Cylinder(radius=0.005, length=0.052),
        origin=Origin(xyz=(HINGE_X, -0.064, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="hinge_knuckle_0",
    )
    box.visual(
        Box((0.010, 0.050, 0.004)),
        origin=Origin(xyz=(HINGE_X + 0.002, -0.064, HINGE_Z - 0.004)),
        material="brushed_steel",
        name="hinge_web_0",
    )
    box.visual(
        Cylinder(radius=0.005, length=0.052),
        origin=Origin(xyz=(HINGE_X, 0.064, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="hinge_knuckle_1",
    )
    box.visual(
        Box((0.010, 0.050, 0.004)),
        origin=Origin(xyz=(HINGE_X + 0.002, 0.064, HINGE_Z - 0.004)),
        material="brushed_steel",
        name="hinge_web_1",
    )

    lid = model.part("lid")
    # Child coordinates are expressed from the hinge pin axis; when closed the
    # broad lid panel extends forward along local +X.
    lid.visual(
        Box((0.302, 0.202, LID_THICKNESS)),
        origin=Origin(xyz=(0.159, 0.0, 0.001)),
        material="warm_ivory",
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.068),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.016, 0.064, 0.004)),
        origin=Origin(xyz=(0.007, 0.0, 0.002)),
        material="brushed_steel",
        name="hinge_web",
    )
    lip_z = -0.010
    lid.visual(
        Box((0.238, 0.004, 0.010)),
        origin=Origin(xyz=(0.155, -0.081, lip_z)),
        material="warm_ivory",
        name="locator_side_lip_0",
    )
    lid.visual(
        Box((0.238, 0.004, 0.010)),
        origin=Origin(xyz=(0.155, 0.081, lip_z)),
        material="warm_ivory",
        name="locator_side_lip_1",
    )
    for idx, x in enumerate((0.036, 0.274)):
        lid.visual(
            Box((0.004, 0.162, 0.010)),
            origin=Origin(xyz=(x, 0.0, lip_z)),
            material="warm_ivory",
            name=f"locator_end_lip_{idx}",
        )
    lid.visual(
        Box((0.050, 0.014, 0.0015)),
        origin=Origin(xyz=(0.288, 0.0, 0.0072)),
        material="shadow_recess",
        name="finger_recess",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=box,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.85),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    box = object_model.get_part("box")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            box,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            min_gap=0.0005,
            max_gap=0.0030,
            name="closed lid has a safe rim clearance",
        )
        ctx.expect_gap(
            lid,
            box,
            axis="y",
            positive_elem="hinge_knuckle",
            negative_elem="hinge_knuckle_0",
            min_gap=0.002,
            max_gap=0.006,
            name="left hinge knuckles have running clearance",
        )
        ctx.expect_gap(
            box,
            lid,
            axis="y",
            positive_elem="hinge_knuckle_1",
            negative_elem="hinge_knuckle",
            min_gap=0.002,
            max_gap=0.006,
            name="right hinge knuckles have running clearance",
        )
        ctx.expect_within(
            lid,
            box,
            axes="xy",
            inner_elem="locator_side_lip_0",
            outer_elem="body_shell",
            margin=0.0,
            name="closed locator lip stows inside box footprint",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.85}):
        ctx.expect_gap(
            lid,
            box,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            min_gap=0.008,
            name="opened lid panel clears the storage tray",
        )
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "hinged lid lifts to a compact upright stop",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.16,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

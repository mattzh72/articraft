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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    """CadQuery sleeve wall, authored with its bottom at z_min."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height).translate((0.0, 0.0, z_min))
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.04)
        .translate((0.0, 0.0, z_min - 0.02))
    )
    return outer.cut(cutter)


def _bollard_base_mesh() -> cq.Workplane:
    """Fixed curb socket with a recessed sleeve, locking pocket, and hinge pads."""
    slab = cq.Workplane("XY").box(0.92, 0.58, 0.12).translate((0.0, 0.0, -0.06))

    through_hole = (
        cq.Workplane("XY")
        .circle(0.118)
        .extrude(0.22)
        .translate((0.0, 0.0, -0.16))
    )
    counterbore = (
        cq.Workplane("XY")
        .circle(0.190)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.040))
    )
    locking_pocket = cq.Workplane("XY").box(0.25, 0.16, 0.055).translate((0.245, 0.0, -0.015))
    slab = slab.cut(through_hole).cut(counterbore).cut(locking_pocket)

    sleeve = _annular_cylinder(outer_radius=0.150, inner_radius=0.106, height=0.66, z_min=-0.66)
    rim = _annular_cylinder(outer_radius=0.182, inner_radius=0.112, height=0.028, z_min=-0.034)

    # Two hinge cheek pads flank the plate barrel without occupying the same space.
    lug_pos = cq.Workplane("XY").box(0.060, 0.034, 0.070).translate((0.370, 0.099, 0.017))
    lug_neg = cq.Workplane("XY").box(0.060, 0.034, 0.070).translate((0.370, -0.099, 0.017))
    root = slab.union(sleeve).union(rim).union(lug_pos).union(lug_neg)

    # Small curb-anchor bosses help the scale read as a flush access-control fitting.
    for x in (-0.31, 0.31):
        for y in (-0.20, 0.20):
            boss = cq.Workplane("XY").circle(0.026).extrude(0.010).translate((x, y, 0.0))
            root = root.union(boss)
    return root


def _post_mesh() -> cq.Workplane:
    """Stout removable post with hidden insertion length below the sleeve lip."""
    shaft = cq.Workplane("XY").circle(0.085).extrude(1.27).translate((0.0, 0.0, -0.45))
    top_cap = cq.Workplane("XY").circle(0.087).extrude(0.035).translate((0.0, 0.0, 0.82))
    stop_ring = cq.Workplane("XY").circle(0.100).extrude(0.035).translate((0.0, 0.0, 0.060))
    return shaft.union(top_cap).union(stop_ring)


def _locking_plate_mesh() -> cq.Workplane:
    """Flat slotted hasp plate with its hinge barrel centered on the part origin."""
    plate = cq.Workplane("XY").box(0.240, 0.150, 0.016).translate((-0.120, 0.0, -0.006))
    slot_rect = cq.Workplane("XY").box(0.110, 0.034, 0.040).translate((-0.125, 0.0, -0.006))
    slot_a = cq.Workplane("XY").circle(0.017).extrude(0.040).translate((-0.180, 0.0, -0.026))
    slot_b = cq.Workplane("XY").circle(0.017).extrude(0.040).translate((-0.070, 0.0, -0.026))
    slot = slot_rect.union(slot_a).union(slot_b)
    plate = plate.cut(slot)

    # A short barrel along local Y gives the plate a believable horizontal hinge.
    barrel = cq.Workplane("XZ").circle(0.018).extrude(0.160).translate((0.0, 0.080, 0.0))
    return plate.union(barrel)


def _release_plug_mesh() -> cq.Workplane:
    """Round rotating release plug with a coin/key slot recessed into the top."""
    plug = cq.Workplane("XY").circle(0.036).extrude(0.022)
    slot = cq.Workplane("XY").box(0.054, 0.009, 0.010).translate((0.0, 0.0, 0.020))
    return plug.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="removable_access_bollard")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.57, 0.55, 1.0))
    dark_steel = model.material("dark_recessed_steel", rgba=(0.08, 0.085, 0.075, 1.0))
    safety_yellow = model.material("safety_yellow_powdercoat", rgba=(0.95, 0.72, 0.08, 1.0))
    worn_black = model.material("black_rubber_shadow", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_bollard_base_mesh(), "recessed_sleeve_base", tolerance=0.002),
        material=galvanized,
        name="sleeve_base",
    )
    base.visual(
        Box((0.205, 0.115, 0.004)),
        origin=Origin(xyz=(0.245, 0.0, -0.041)),
        material=dark_steel,
        name="locking_recess_shadow",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_cylinder(outer_radius=0.106, inner_radius=0.091, height=0.006, z_min=-0.041),
            "sleeve_mouth_shadow",
            tolerance=0.001,
        ),
        origin=Origin(),
        material=worn_black,
        name="sleeve_mouth_shadow",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.228),
        origin=Origin(xyz=(0.370, 0.0, 0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    base.visual(
        Box((0.030, 0.070, 0.101)),
        origin=Origin(xyz=(0.115, 0.0, 0.0445)),
        material=dark_steel,
        name="guide_pad",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_mesh(), "sliding_bollard_post", tolerance=0.0015),
        material=safety_yellow,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=0.087, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=dark_steel,
        name="lower_wear_band",
    )
    post.visual(
        Cylinder(radius=0.087, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.645)),
        material=dark_steel,
        name="upper_wear_band",
    )

    plate = model.part("locking_plate")
    plate.visual(
        mesh_from_cadquery(_locking_plate_mesh(), "slotted_locking_plate", tolerance=0.001),
        material=dark_steel,
        name="slotted_plate",
    )

    plug = model.part("release_plug")
    plug.visual(
        mesh_from_cadquery(_release_plug_mesh(), "top_release_plug", tolerance=0.0008),
        material=dark_steel,
        name="plug_body",
    )
    plug.visual(
        Box((0.050, 0.007, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=worn_black,
        name="plug_slot_shadow",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.35, lower=0.0, upper=0.40),
    )
    model.articulation(
        "base_to_locking_plate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=plate,
        origin=Origin(xyz=(0.370, 0.0, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.30),
    )
    model.articulation(
        "post_to_release_plug",
        ArticulationType.REVOLUTE,
        parent=post,
        child=plug,
        origin=Origin(xyz=(0.0, 0.0, 0.855)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    plate = object_model.get_part("locking_plate")
    plug = object_model.get_part("release_plug")
    slide = object_model.get_articulation("sleeve_to_post")
    plate_hinge = object_model.get_articulation("base_to_locking_plate")
    plug_turn = object_model.get_articulation("post_to_release_plug")

    ctx.allow_overlap(
        base,
        plate,
        elem_a="hinge_pin",
        elem_b="slotted_plate",
        reason="The fixed hinge pin intentionally passes through the plate barrel as a captured shaft.",
    )
    ctx.expect_overlap(
        base,
        plate,
        axes="y",
        elem_a="hinge_pin",
        elem_b="slotted_plate",
        min_overlap=0.12,
        name="hinge pin spans the locking plate barrel",
    )

    ctx.expect_within(
        post,
        base,
        axes="xy",
        inner_elem="post_shaft",
        outer_elem="sleeve_base",
        margin=0.0,
        name="post centered inside recessed sleeve footprint",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="post_shaft",
        elem_b="sleeve_base",
        min_overlap=0.40,
        name="post has hidden retained insertion in sleeve",
    )
    ctx.expect_contact(
        post,
        base,
        elem_a="post_shaft",
        elem_b="guide_pad",
        contact_tol=0.002,
        name="post bears against sleeve guide pad",
    )
    ctx.expect_gap(
        plug,
        post,
        axis="z",
        positive_elem="plug_body",
        negative_elem="post_shaft",
        min_gap=-0.001,
        max_gap=0.0015,
        name="release plug is seated on post top",
    )
    ctx.expect_overlap(
        plate,
        base,
        axes="xy",
        elem_a="slotted_plate",
        elem_b="locking_recess_shadow",
        min_overlap=0.10,
        name="closed slotted plate covers locking recess",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: 0.40}):
        extended_pos = ctx.part_world_position(post)
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="post_shaft",
            elem_b="sleeve_base",
            min_overlap=0.07,
            name="raised post remains captured in sleeve",
        )
    ctx.check(
        "post slides upward on vertical prismatic joint",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({plate_hinge: 1.10}):
        ctx.expect_gap(
            plate,
            base,
            axis="z",
            positive_elem="slotted_plate",
            negative_elem="locking_recess_shadow",
            min_gap=0.035,
            name="rotated locking plate uncovers recess",
        )

    with ctx.pose({plug_turn: math.pi / 2.0}):
        ctx.expect_contact(
            plug,
            post,
            elem_a="plug_body",
            elem_b="post_shaft",
            contact_tol=0.002,
            name="release plug stays seated while rotating",
        )

    return ctx.report()


object_model = build_object_model()

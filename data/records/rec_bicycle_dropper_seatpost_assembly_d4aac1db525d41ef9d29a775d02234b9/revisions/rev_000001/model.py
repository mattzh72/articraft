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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A vertical tube with an actual open bore, authored in metres."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _seat_tube_adapter() -> cq.Workplane:
    sleeve = _annular_cylinder(0.0208, 0.01575, 0.200)
    lower_bead = _annular_cylinder(0.0228, 0.01575, 0.018).translate((0.0, 0.0, 0.006))
    top_lip = _annular_cylinder(0.0224, 0.01575, 0.018).translate((0.0, 0.0, 0.188))
    witness_slot = (
        cq.Workplane("XY")
        .box(0.007, 0.0030, 0.205)
        .translate((0.0200, 0.0, 0.101))
    )
    return sleeve.union(lower_bead).union(top_lip).cut(witness_slot)


def _clamp_ring() -> cq.Workplane:
    height = 0.036
    ring = _annular_cylinder(0.0228, 0.01545, height)
    split = cq.Workplane("XY").box(0.017, 0.0058, height + 0.004).translate((0.0187, 0.0, height / 2.0))
    upper_ear = cq.Workplane("XY").box(0.025, 0.0062, 0.030).translate((0.0285, 0.0090, height / 2.0))
    lower_ear = cq.Workplane("XY").box(0.025, 0.0062, 0.030).translate((0.0285, -0.0090, height / 2.0))
    nose_pad = cq.Workplane("XY").box(0.006, 0.026, 0.014).translate((0.040, 0.0, height / 2.0))
    return ring.union(upper_ear).union(lower_ear).union(nose_pad).cut(split)


def _outer_dropper_tube() -> cq.Workplane:
    body = _annular_cylinder(0.01545, 0.01120, 0.420)
    lower_polished_band = _annular_cylinder(0.01575, 0.01120, 0.018).translate((0.0, 0.0, 0.020))
    seal_head = _annular_cylinder(0.0184, 0.01120, 0.026).translate((0.0, 0.0, 0.407))
    dust_lip = _annular_cylinder(0.0191, 0.01120, 0.006).translate((0.0, 0.0, 0.430))
    return body.union(lower_polished_band).union(seal_head).union(dust_lip)


def _lever_cam_plate() -> cq.Workplane:
    thickness = 0.0050
    cam_round = cq.Workplane("XZ").circle(0.0115).extrude(thickness)
    cam_eccentric = cq.Workplane("XZ").center(0.006, 0.002).circle(0.0080).extrude(thickness)
    handle = cq.Workplane("XZ").center(0.014, -0.043).rect(0.012, 0.080).extrude(thickness)
    curled_tip = cq.Workplane("XZ").center(0.018, -0.084).circle(0.008).extrude(thickness)
    bore = cq.Workplane("XZ").circle(0.0038).extrude(thickness + 0.002).translate((0.0, -0.001, 0.0))
    return (
        cam_round.union(cam_eccentric).union(handle).union(curled_tip)
        .cut(bore)
        .translate((0.0, -thickness / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost_assembly")

    anodized_black = model.material("anodized_black", rgba=(0.015, 0.017, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.66, 0.60, 1.0))
    polished_stanchion = model.material("polished_stanchion", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_marking = model.material("etched_marking", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("stainless_steel", rgba=(0.72, 0.72, 0.70, 1.0))

    frame_sleeve = model.part("frame_sleeve")
    frame_sleeve.visual(
        mesh_from_cadquery(_seat_tube_adapter(), "seat_tube_adapter", tolerance=0.0008),
        material=brushed_aluminum,
        name="adapter_sleeve",
    )

    clamp_ring = model.part("clamp_ring")
    clamp_ring.visual(
        mesh_from_cadquery(_clamp_ring(), "split_clamp_ring", tolerance=0.0008),
        material=anodized_black,
        name="ring_body",
    )
    clamp_ring.visual(
        Cylinder(radius=0.0032, length=0.034),
        origin=Origin(xyz=(0.040, 0.0, 0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        mesh_from_cadquery(_outer_dropper_tube(), "outer_dropper_tube", tolerance=0.0008),
        material=satin_black,
        name="outer_body",
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        Cylinder(radius=0.0112, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=polished_stanchion,
        name="stanchion",
    )
    for idx, z in enumerate((0.080, 0.125, 0.170)):
        inner_tube.visual(
            Cylinder(radius=0.01135, length=0.0020),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_marking,
            name=f"sag_mark_{idx}",
        )

    saddle_perch = model.part("saddle_perch")
    saddle_perch.visual(
        Cylinder(radius=0.0135, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_black,
        name="lower_boss",
    )
    saddle_perch.visual(
        Box((0.070, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_black,
        name="lower_cradle",
    )
    saddle_perch.visual(
        Box((0.060, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=satin_black,
        name="center_bridge",
    )
    saddle_perch.visual(
        Box((0.076, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=satin_black,
        name="upper_clamp",
    )
    for idx, y in enumerate((-0.011, 0.011)):
        saddle_perch.visual(
            Cylinder(radius=0.0030, length=0.084),
            origin=Origin(xyz=(0.0, y, 0.041), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"rail_groove_{idx}",
        )

    lever = model.part("binder_lever")
    lever.visual(
        mesh_from_cadquery(_lever_cam_plate(), "cam_binder_lever", tolerance=0.0006),
        material=anodized_black,
        name="cam_plate",
    )

    for x, name in ((-0.024, "front_bolt"), (0.024, "rear_bolt")):
        bolt = model.part(name)
        bolt.visual(
            Cylinder(radius=0.0054, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=steel,
            name="bolt_head",
        )
        bolt.visual(
            Box((0.0070, 0.0015, 0.0010)),
            origin=Origin(xyz=(0.0, 0.0, 0.0062)),
            material=dark_marking,
            name="hex_slot",
        )
        model.articulation(
            f"saddle_perch_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=saddle_perch,
            child=bolt,
            origin=Origin(xyz=(x, 0.0, 0.047)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )

    model.articulation(
        "frame_sleeve_to_clamp_ring",
        ArticulationType.FIXED,
        parent=frame_sleeve,
        child=clamp_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.206)),
    )
    model.articulation(
        "clamp_ring_to_outer_tube",
        ArticulationType.FIXED,
        parent=clamp_ring,
        child=outer_tube,
        origin=Origin(xyz=(0.0, 0.0, -0.166)),
    )
    model.articulation(
        "outer_tube_to_inner_tube",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.436)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    model.articulation(
        "inner_tube_to_saddle_perch",
        ArticulationType.FIXED,
        parent=inner_tube,
        child=saddle_perch,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
    )
    model.articulation(
        "clamp_ring_to_binder_lever",
        ArticulationType.REVOLUTE,
        parent=clamp_ring,
        child=lever,
        origin=Origin(xyz=(0.040, 0.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=4.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.
    frame_sleeve = object_model.get_part("frame_sleeve")
    clamp_ring = object_model.get_part("clamp_ring")
    outer_tube = object_model.get_part("outer_tube")
    inner_tube = object_model.get_part("inner_tube")
    saddle_perch = object_model.get_part("saddle_perch")
    binder_lever = object_model.get_part("binder_lever")
    dropper = object_model.get_articulation("outer_tube_to_inner_tube")
    lever_hinge = object_model.get_articulation("clamp_ring_to_binder_lever")

    ctx.allow_overlap(
        frame_sleeve,
        outer_tube,
        elem_a="adapter_sleeve",
        elem_b="outer_body",
        reason="The outer post is intentionally represented as passing through the hollow seat-tube sleeve adapter.",
    )
    ctx.allow_overlap(
        clamp_ring,
        outer_tube,
        elem_a="ring_body",
        elem_b="outer_body",
        reason="The split clamp ring intentionally surrounds and captures the outer post at the binder clamp.",
    )
    ctx.allow_overlap(
        outer_tube,
        inner_tube,
        elem_a="outer_body",
        elem_b="stanchion",
        reason="The inner stanchion is intentionally modeled as a retained sliding member inside the outer dropper tube.",
    )

    ctx.expect_gap(
        clamp_ring,
        frame_sleeve,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="ring_body",
        negative_elem="adapter_sleeve",
        name="clamp ring sits on adapter sleeve",
    )
    ctx.expect_within(
        outer_tube,
        frame_sleeve,
        axes="xy",
        margin=0.001,
        inner_elem="outer_body",
        outer_elem="adapter_sleeve",
        name="outer post passes through sleeve adapter bore",
    )
    ctx.expect_overlap(
        outer_tube,
        frame_sleeve,
        axes="z",
        min_overlap=0.140,
        elem_a="outer_body",
        elem_b="adapter_sleeve",
        name="seat-tube sleeve retains outer post",
    )
    ctx.expect_within(
        outer_tube,
        clamp_ring,
        axes="xy",
        margin=0.001,
        inner_elem="outer_body",
        outer_elem="ring_body",
        name="outer post is centered in clamp bore",
    )
    ctx.expect_overlap(
        outer_tube,
        clamp_ring,
        axes="z",
        min_overlap=0.030,
        elem_a="outer_body",
        elem_b="ring_body",
        name="clamp ring surrounds outer post",
    )
    ctx.expect_within(
        inner_tube,
        outer_tube,
        axes="xy",
        margin=0.001,
        inner_elem="stanchion",
        outer_elem="outer_body",
        name="inner stanchion is centered in outer tube",
    )
    ctx.expect_overlap(
        inner_tube,
        outer_tube,
        axes="z",
        min_overlap=0.150,
        elem_a="stanchion",
        elem_b="outer_body",
        name="collapsed dropper retains insertion",
    )

    rest_inner_aabb = ctx.part_world_aabb(inner_tube)
    rest_lever_aabb = ctx.part_world_aabb(binder_lever)
    with ctx.pose({dropper: 0.120, lever_hinge: 1.25}):
        ctx.expect_within(
            inner_tube,
            outer_tube,
            axes="xy",
            margin=0.001,
            inner_elem="stanchion",
            outer_elem="outer_body",
            name="extended stanchion remains centered",
        )
        ctx.expect_overlap(
            inner_tube,
            outer_tube,
            axes="z",
            min_overlap=0.070,
            elem_a="stanchion",
            elem_b="outer_body",
            name="extended dropper still engaged",
        )
        extended_inner_aabb = ctx.part_world_aabb(inner_tube)
        open_lever_aabb = ctx.part_world_aabb(binder_lever)

    ctx.check(
        "dropper extends upward",
        rest_inner_aabb is not None
        and extended_inner_aabb is not None
        and extended_inner_aabb[1][2] > rest_inner_aabb[1][2] + 0.10,
        details=f"rest={rest_inner_aabb}, extended={extended_inner_aabb}",
    )
    ctx.check(
        "quick release lever swings outward",
        rest_lever_aabb is not None
        and open_lever_aabb is not None
        and open_lever_aabb[1][0] > rest_lever_aabb[1][0] + 0.020,
        details=f"closed={rest_lever_aabb}, open={open_lever_aabb}",
    )
    ctx.expect_contact(
        saddle_perch,
        inner_tube,
        elem_a="lower_boss",
        elem_b="stanchion",
        contact_tol=0.001,
        name="saddle perch is seated on stanchion head",
    )

    return ctx.report()


object_model = build_object_model()

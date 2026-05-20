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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_min: float):
    """CadQuery annular tube aligned to world/local Z."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height).translate((0.0, 0.0, z_min))
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.04)
        .translate((0.0, 0.0, z_min - 0.02))
    )
    return outer.cut(cutter)


def _holed_slab(width: float, depth: float, thickness: float, hole_radius: float):
    slab = cq.Workplane("XY").box(width, depth, thickness).translate((0.0, 0.0, -thickness / 2.0))
    cutter = (
        cq.Workplane("XY")
        .circle(hole_radius)
        .extrude(thickness + 0.08)
        .translate((0.0, 0.0, -thickness - 0.02))
    )
    return slab.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="removable_access_bollard")

    concrete = Material("weathered_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_steel = Material("dark_burnished_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    safety_yellow = Material("safety_yellow_paint", rgba=(0.96, 0.72, 0.05, 1.0))
    reflector = Material("white_reflective_tape", rgba=(0.90, 0.92, 0.86, 1.0))
    black = Material("black_keyway_shadow", rgba=(0.01, 0.01, 0.012, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_holed_slab(1.20, 0.85, 0.08, 0.18), "sidewalk_slab"),
        material=concrete,
        name="sidewalk_slab",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_cylinder(0.165, 0.105, 0.54, -0.52), "sleeve_barrel"),
        material=dark_steel,
        name="sleeve_barrel",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_cylinder(0.245, 0.105, 0.045, -0.020), "sleeve_rim"),
        material=galvanized,
        name="sleeve_rim",
    )
    sleeve.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.365, 0.0, 0.026)),
        material=black,
        name="lock_socket",
    )
    sleeve.visual(
        Box((0.240, 0.105, 0.010)),
        origin=Origin(xyz=(0.365, 0.0, 0.025)),
        material=galvanized,
        name="lock_pocket_boss",
    )
    # Tangent bronze/nylon guide pads give the removable post an actual sliding
    # bearing path instead of leaving it visually floating in the clearance.
    sleeve.visual(
        Box((0.020, 0.032, 0.090)),
        origin=Origin(xyz=(0.095, 0.0, -0.165)),
        material=galvanized,
        name="guide_pad_0",
    )
    sleeve.visual(
        Box((0.020, 0.032, 0.090)),
        origin=Origin(xyz=(-0.095, 0.0, -0.165)),
        material=galvanized,
        name="guide_pad_1",
    )
    # Fixed hinge cheeks and short outer knuckles are welded to the sleeve rim.
    for i, y in enumerate((-0.069, 0.069)):
        sleeve.visual(
            Box((0.045, 0.035, 0.044)),
            origin=Origin(xyz=(0.247, y, 0.042)),
            material=galvanized,
            name=f"hinge_cheek_{i}",
        )
        sleeve.visual(
            Cylinder(radius=0.013, length=0.033),
            origin=Origin(xyz=(0.247, y, 0.047), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"hinge_knuckle_{i}",
        )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_annular_cylinder(0.085, 0.058, 1.10, -0.45), "post_tube"),
        material=safety_yellow,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.087, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=reflector,
        name="reflector_band_0",
    )
    post.visual(
        Cylinder(radius=0.087, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=reflector,
        name="reflector_band_1",
    )

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        Cylinder(radius=0.012, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cover_barrel",
    )
    lock_cover.visual(
        Box((0.060, 0.088, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, -0.005)),
        material=galvanized,
        name="cover_leaf",
    )
    lock_cover.visual(
        Box((0.225, 0.120, 0.016)),
        origin=Origin(xyz=(0.140, 0.0, -0.006)),
        material=galvanized,
        name="cover_plate",
    )
    lock_cover.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.205, 0.0, 0.004)),
        material=black,
        name="cover_finger_recess",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        Cylinder(radius=0.096, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="cap_body",
    )
    top_cap.visual(
        Cylinder(radius=0.068, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=galvanized,
        name="cap_top_insert",
    )
    top_cap.visual(
        Box((0.014, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0645)),
        material=black,
        name="key_slot",
    )
    top_cap.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, -0.030, 0.065)),
        material=black,
        name="key_round",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        # The child frame is at the sleeve mouth; the post mesh extends below it
        # so the bollard remains captured in the buried sleeve at full travel.
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.20, lower=0.0, upper=0.25),
    )
    model.articulation(
        "sleeve_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=lock_cover,
        origin=Origin(xyz=(0.247, 0.0, 0.047)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.65),
    )
    model.articulation(
        "post_to_top_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    lock_cover = object_model.get_part("lock_cover")
    top_cap = object_model.get_part("top_cap")
    slide = object_model.get_articulation("sleeve_to_post")
    hinge = object_model.get_articulation("sleeve_to_lock_cover")
    cap_turn = object_model.get_articulation("post_to_top_cap")

    ctx.check("post slides vertically", slide.articulation_type == ArticulationType.PRISMATIC and slide.axis == (0.0, 0.0, 1.0))
    ctx.check("lock cover is hinged", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("top cap turns continuously", cap_turn.articulation_type == ArticulationType.CONTINUOUS)

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="sleeve_barrel",
        margin=0.0,
        name="post tube is centered in the sleeve opening",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_barrel",
        min_overlap=0.40,
        name="post remains deeply inserted when lowered",
    )
    ctx.expect_gap(
        top_cap,
        post,
        axis="z",
        positive_elem="cap_body",
        negative_elem="post_tube",
        max_gap=0.001,
        max_penetration=0.0,
        name="separate top cap seats on the post tube",
    )
    ctx.expect_overlap(
        lock_cover,
        sleeve,
        axes="xy",
        elem_a="cover_plate",
        elem_b="lock_socket",
        min_overlap=0.05,
        name="closed lock cover sits over the lock recess",
    )
    ctx.expect_gap(
        lock_cover,
        sleeve,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="lock_socket",
        min_gap=0.002,
        max_gap=0.020,
        name="lock cover is a supported flap above the rim recess",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: 0.25}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_barrel",
            min_overlap=0.15,
            name="raised post still retains sleeve insertion",
        )
        raised_pos = ctx.part_world_position(post)
    ctx.check(
        "post raises along sleeve axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    closed_aabb = ctx.part_element_world_aabb(lock_cover, elem="cover_plate")
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(lock_cover, elem="cover_plate")
    ctx.check(
        "lock cover rotates upward from rim",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

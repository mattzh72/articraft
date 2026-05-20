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


def _origin_for_x(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder local +Z laid along world +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_for_y(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder local +Z laid along world +Y."""
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _hollow_sleeve_mesh(name: str, *, length: float, outer_radius: float, inner_radius: float):
    sleeve = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(sleeve, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farm_pedestrian_gate")

    painted_steel = model.material("weathered_green_steel", rgba=(0.08, 0.23, 0.13, 1.0))
    galvanized = model.material("galvanized_wire", rgba=(0.62, 0.66, 0.64, 1.0))
    dark_steel = model.material("black_latch_steel", rgba=(0.02, 0.025, 0.025, 1.0))
    concrete = model.material("pale_concrete", rgba=(0.52, 0.50, 0.45, 1.0))

    posts = model.part("posts")
    # A low concrete footing ties the hinge and latch posts together so the fixed
    # side reads as one installed assembly rather than two disconnected posts.
    posts.visual(
        Box((3.22, 0.22, 0.06)),
        origin=Origin(xyz=(1.43, 0.0, 0.03)),
        material=concrete,
        name="footing",
    )
    posts.visual(
        Cylinder(radius=0.060, length=1.75),
        origin=Origin(xyz=(-0.10, 0.0, 0.875)),
        material=painted_steel,
        name="hinge_post",
    )
    posts.visual(
        Cylinder(radius=0.060, length=1.75),
        origin=Origin(xyz=(2.95, 0.0, 0.875)),
        material=painted_steel,
        name="latch_post",
    )
    posts.visual(
        Cylinder(radius=0.064, length=0.018),
        origin=Origin(xyz=(-0.10, 0.0, 1.759)),
        material=painted_steel,
        name="hinge_post_cap",
    )
    posts.visual(
        Cylinder(radius=0.064, length=0.018),
        origin=Origin(xyz=(2.95, 0.0, 1.759)),
        material=painted_steel,
        name="latch_post_cap",
    )

    # Fixed gudgeon pins and their small welded support straps at the hinge post.
    for zc, name in ((0.55, "lower"), (1.31, "upper")):
        posts.visual(
            Cylinder(radius=0.012, length=0.31),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=galvanized,
            name=f"{name}_hinge_pin",
        )
        posts.visual(
            Box((0.105, 0.055, 0.040)),
            origin=Origin(xyz=(-0.050, 0.0, zc - 0.155)),
            material=painted_steel,
            name=f"{name}_pin_support",
        )
        posts.visual(
            Box((0.070, 0.036, 0.020)),
            origin=Origin(xyz=(-0.030, 0.0, zc + 0.155)),
            material=painted_steel,
            name=f"{name}_pin_capstrap",
        )

    # Two-slot receiver on the latch post.  The rotating lever rests between
    # these lips at the closed pose and can be lifted out of the slot.
    posts.visual(
        Box((0.130, 0.060, 0.016)),
        origin=Origin(xyz=(2.895, 0.035, 1.067)),
        material=dark_steel,
        name="lower_catch",
    )
    posts.visual(
        Box((0.130, 0.060, 0.016)),
        origin=Origin(xyz=(2.895, 0.035, 1.133)),
        material=dark_steel,
        name="upper_catch",
    )
    posts.visual(
        Box((0.022, 0.060, 0.120)),
        origin=Origin(xyz=(2.956, 0.035, 1.100)),
        material=dark_steel,
        name="catch_back",
    )

    leaf = model.part("leaf")
    tube_r = 0.030
    x_hinge = 0.18
    x_latch = 2.75
    bottom_z = 0.32
    top_z = 1.48
    mid_z = 0.90
    span_x = x_latch - x_hinge
    center_x = (x_hinge + x_latch) / 2.0
    frame_h = top_z - bottom_z

    leaf.visual(
        Cylinder(radius=tube_r, length=span_x),
        origin=_origin_for_x((center_x, 0.0, top_z)),
        material=painted_steel,
        name="top_rail",
    )
    leaf.visual(
        Cylinder(radius=tube_r, length=span_x),
        origin=_origin_for_x((center_x, 0.0, bottom_z)),
        material=painted_steel,
        name="bottom_rail",
    )
    leaf.visual(
        Cylinder(radius=tube_r, length=span_x),
        origin=_origin_for_x((center_x, 0.0, mid_z)),
        material=painted_steel,
        name="middle_rail",
    )
    leaf.visual(
        Cylinder(radius=tube_r, length=frame_h),
        origin=Origin(xyz=(x_hinge, 0.0, (bottom_z + top_z) / 2.0)),
        material=painted_steel,
        name="hinge_stile",
    )
    leaf.visual(
        Cylinder(radius=tube_r, length=frame_h),
        origin=Origin(xyz=(x_latch, 0.0, (bottom_z + top_z) / 2.0)),
        material=painted_steel,
        name="latch_stile",
    )
    # Diagonal welded tube brace from the low hinge corner to the latch-side top.
    diag_dx = x_latch - x_hinge
    diag_dz = top_z - bottom_z
    diag_len = math.hypot(diag_dx, diag_dz)
    diag_angle = math.atan2(diag_dx, diag_dz)
    leaf.visual(
        Cylinder(radius=0.020, length=diag_len),
        origin=Origin(
            xyz=((x_hinge + x_latch) / 2.0, 0.0, (bottom_z + top_z) / 2.0),
            rpy=(0.0, diag_angle, 0.0),
        ),
        material=painted_steel,
        name="diagonal_brace",
    )

    # Welded wire mesh infill: thin galvanized rods welded to each other and to
    # the tube frame.  They sit inside the tube outline and intersect the frame,
    # as real welded farm-gate mesh would.
    wire_r = 0.0045
    for i, x in enumerate([0.38, 0.58, 0.78, 0.98, 1.18, 1.38, 1.58, 1.78, 1.98, 2.18, 2.38, 2.58]):
        leaf.visual(
            Cylinder(radius=wire_r, length=1.04),
            origin=Origin(xyz=(x, 0.0, 0.90)),
            material=galvanized,
            name=f"vertical_wire_{i}",
        )
    for i, z in enumerate([0.44, 0.56, 0.68, 0.80, 1.02, 1.14, 1.26, 1.38]):
        leaf.visual(
            Cylinder(radius=wire_r, length=2.40),
            origin=_origin_for_x((1.48, 0.0, z)),
            material=galvanized,
            name=f"horizontal_wire_{i}",
        )

    # Gate-side hinge sleeves are real hollow tubes around the fixed pins.
    for zc, name in ((0.55, "lower"), (1.31, "upper")):
        leaf.visual(
            _hollow_sleeve_mesh(f"{name}_sleeve_mesh", length=0.200, outer_radius=0.028, inner_radius=0.0118),
            origin=Origin(xyz=(0.0, 0.0, zc - 0.100)),
            material=painted_steel,
            name=f"{name}_sleeve",
        )
        leaf.visual(
            Box((0.158, 0.050, 0.050)),
            origin=Origin(xyz=(0.101, 0.0, zc)),
            material=painted_steel,
            name=f"{name}_sleeve_strap",
        )

    leaf.visual(
        Box((0.090, 0.030, 0.120)),
        origin=Origin(xyz=(x_latch - 0.020, 0.038, 1.100)),
        material=dark_steel,
        name="pivot_plate",
    )

    model.articulation(
        "post_to_leaf",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    latch = model.part("lever_latch")
    # Child frame is the latch pivot; the lever extends toward the latch post.
    latch.visual(
        Box((0.150, 0.026, 0.024)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=dark_steel,
        name="lever_bar",
    )
    latch.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=_origin_for_y((0.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_boss",
    )
    latch.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=_origin_for_y((0.135, 0.0, 0.0)),
        material=galvanized,
        name="grip_knob",
    )

    model.articulation(
        "leaf_to_latch",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=latch,
        origin=Origin(xyz=(x_latch - 0.020, 0.066, 1.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    leaf = object_model.get_part("leaf")
    latch = object_model.get_part("lever_latch")
    gate_hinge = object_model.get_articulation("post_to_leaf")
    latch_pivot = object_model.get_articulation("leaf_to_latch")

    for name in ("lower", "upper"):
        ctx.allow_overlap(
            leaf,
            posts,
            elem_a=f"{name}_sleeve",
            elem_b=f"{name}_hinge_pin",
            reason="The gate sleeve is intentionally captured around the fixed hinge pin.",
        )
        ctx.expect_within(
            posts,
            leaf,
            axes="xy",
            inner_elem=f"{name}_hinge_pin",
            outer_elem=f"{name}_sleeve",
            margin=0.002,
            name=f"{name} hinge pin is centered in sleeve",
        )
        ctx.expect_overlap(
            posts,
            leaf,
            axes="z",
            elem_a=f"{name}_hinge_pin",
            elem_b=f"{name}_sleeve",
            min_overlap=0.18,
            name=f"{name} hinge sleeve retains pin engagement",
        )

    ctx.expect_gap(
        posts,
        leaf,
        axis="x",
        positive_elem="latch_post",
        negative_elem="latch_stile",
        min_gap=0.08,
        max_gap=0.14,
        name="closed leaf meets latch post with farm gate clearance",
    )
    ctx.expect_gap(
        posts,
        latch,
        axis="z",
        positive_elem="upper_catch",
        negative_elem="lever_bar",
        min_gap=0.004,
        max_gap=0.030,
        name="lever rests below upper catch lip",
    )
    ctx.expect_gap(
        latch,
        posts,
        axis="z",
        positive_elem="lever_bar",
        negative_elem="lower_catch",
        min_gap=0.004,
        max_gap=0.030,
        name="lever rests above lower catch lip",
    )

    closed_leaf_aabb = ctx.part_world_aabb(leaf)
    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="lever_bar")
    with ctx.pose({gate_hinge: 1.20}):
        open_leaf_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({latch_pivot: 0.70}):
        raised_latch_aabb = ctx.part_element_world_aabb(latch, elem="lever_bar")

    ctx.check(
        "leaf swings outward about vertical post hinge",
        closed_leaf_aabb is not None
        and open_leaf_aabb is not None
        and open_leaf_aabb[1][1] > closed_leaf_aabb[1][1] + 1.0,
        details=f"closed_aabb={closed_leaf_aabb}, open_aabb={open_leaf_aabb}",
    )
    ctx.check(
        "lever latch lifts out of catch",
        closed_latch_aabb is not None
        and raised_latch_aabb is not None
        and raised_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.08,
        details=f"closed_aabb={closed_latch_aabb}, raised_aabb={raised_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

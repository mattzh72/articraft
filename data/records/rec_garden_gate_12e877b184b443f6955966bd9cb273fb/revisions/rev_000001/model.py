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
import cadquery as cq


def make_leaf(width, height, thickness, rail_count, direction=1):
    # direction=1 for left leaf, -1 for right leaf
    # Local origin is the hinge axis.
    
    frame_x_start = 0.03 * direction
    frame_x_end = (0.03 + width) * direction
    frame_y_center = -0.08
    frame_thickness = thickness
    
    x_min = min(frame_x_start, frame_x_end)
    x_max = max(frame_x_start, frame_x_end)
    cx = (x_min + x_max) / 2
    cy = frame_y_center
    
    z_min = 0.1
    
    # Outer frame
    frame = cq.Workplane("XY").center(cx, cy).box(width, frame_thickness, height)
    frame = frame.translate((0, 0, z_min + height/2))
    
    # Cutout
    cutout_width = width - 2 * frame_thickness
    cutout_height = height - 2 * frame_thickness
    cutout = cq.Workplane("XY").center(cx, cy).box(cutout_width, frame_thickness + 0.02, cutout_height)
    cutout = cutout.translate((0, 0, z_min + height/2))
    frame = frame.cut(cutout)
    
    # Rails
    rail_spacing = cutout_height / (rail_count + 1)
    for i in range(rail_count):
        z_offset = z_min + frame_thickness + (i + 1) * rail_spacing
        rail = cq.Workplane("XY").center(cx, cy).box(cutout_width, frame_thickness, frame_thickness)
        rail = rail.translate((0, 0, z_offset))
        frame = frame.union(rail)
        
    # Hinge eyes and connectors
    conn_cx = 0.015 * direction
    conn_cy = -0.04
    
    lower_eye = cq.Workplane("XY").workplane(offset=0.30).circle(0.015).circle(0.011).extrude(0.04)
    lower_conn = cq.Workplane("XY").workplane(offset=0.30).center(conn_cx, conn_cy).rect(0.03, 0.08).extrude(0.04)
    
    upper_eye = cq.Workplane("XY").workplane(offset=0.90).circle(0.015).circle(0.011).extrude(0.04)
    upper_conn = cq.Workplane("XY").workplane(offset=0.90).center(conn_cx, conn_cy).rect(0.03, 0.08).extrude(0.04)
    
    res = frame.union(lower_eye).union(lower_conn).union(upper_eye).union(upper_conn)
    return res


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_garden_gate")

    posts = model.part("posts")
    # Ground beam
    posts.visual(Box((3.35, 0.15, 0.05)), origin=Origin((0.0, 0.0, 0.025)), name="ground_beam")
    
    # Left post and pintles
    posts.visual(Box((0.15, 0.15, 1.5)), origin=Origin((-1.6, 0.0, 0.75)), name="left_post")
    posts.visual(Box((0.05, 0.06, 0.04)), origin=Origin((-1.510, 0.07, 0.28)), name="left_lower_pintle_block")
    posts.visual(Cylinder(0.01, 0.04), origin=Origin((-1.5, 0.08, 0.32)), name="left_lower_pintle_pin")
    posts.visual(Box((0.05, 0.06, 0.04)), origin=Origin((-1.510, 0.07, 0.88)), name="left_upper_pintle_block")
    posts.visual(Cylinder(0.01, 0.04), origin=Origin((-1.5, 0.08, 0.92)), name="left_upper_pintle_pin")
    
    # Right post and pintles
    posts.visual(Box((0.15, 0.15, 1.5)), origin=Origin((1.6, 0.0, 0.75)), name="right_post")
    posts.visual(Box((0.05, 0.06, 0.04)), origin=Origin((1.510, 0.07, 0.28)), name="right_lower_pintle_block")
    posts.visual(Cylinder(0.01, 0.04), origin=Origin((1.5, 0.08, 0.32)), name="right_lower_pintle_pin")
    posts.visual(Box((0.05, 0.06, 0.04)), origin=Origin((1.510, 0.07, 0.88)), name="right_upper_pintle_block")
    posts.visual(Cylinder(0.01, 0.04), origin=Origin((1.5, 0.08, 0.92)), name="right_upper_pintle_pin")

    # Left leaf
    left_leaf = model.part("left_leaf")
    left_geom = make_leaf(1.45, 1.1, 0.05, 3, direction=1)
    left_leaf.visual(mesh_from_cadquery(left_geom, "left_leaf_frame"), name="frame")

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=left_leaf,
        origin=Origin((-1.5, 0.08, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=2.1),
    )

    # Right leaf
    right_leaf = model.part("right_leaf")
    right_geom = make_leaf(1.45, 1.1, 0.05, 3, direction=-1)
    right_leaf.visual(mesh_from_cadquery(right_geom, "right_leaf_frame"), name="frame")

    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=right_leaf,
        origin=Origin((1.5, 0.08, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    posts = object_model.get_part("posts")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    
    # Allow hinge overlaps
    ctx.allow_overlap(left_leaf, posts, elem_a="frame", elem_b="left_lower_pintle_pin", reason="Hinge pin inside eye")
    ctx.allow_overlap(left_leaf, posts, elem_a="frame", elem_b="left_upper_pintle_pin", reason="Hinge pin inside eye")
    ctx.allow_overlap(right_leaf, posts, elem_a="frame", elem_b="right_lower_pintle_pin", reason="Hinge pin inside eye")
    ctx.allow_overlap(right_leaf, posts, elem_a="frame", elem_b="right_upper_pintle_pin", reason="Hinge pin inside eye")
    
    ctx.allow_overlap(left_leaf, posts, elem_a="frame", elem_b="left_lower_pintle_block", reason="Hinge eye rests on block")
    ctx.allow_overlap(left_leaf, posts, elem_a="frame", elem_b="left_upper_pintle_block", reason="Hinge eye rests on block")
    ctx.allow_overlap(right_leaf, posts, elem_a="frame", elem_b="right_lower_pintle_block", reason="Hinge eye rests on block")
    ctx.allow_overlap(right_leaf, posts, elem_a="frame", elem_b="right_upper_pintle_block", reason="Hinge eye rests on block")
    
    # Check gap between leaves at rest
    ctx.expect_gap(right_leaf, left_leaf, axis="x", min_gap=0.03, max_gap=0.05, positive_elem="frame", negative_elem="frame")
    
    # Check that leaves remain supported by the mounts when opened
    with ctx.pose(left_hinge=2.1, right_hinge=2.1):
        ctx.expect_overlap(left_leaf, posts, axes="z", elem_a="frame", elem_b="left_lower_pintle_pin")
        ctx.expect_overlap(right_leaf, posts, axes="z", elem_a="frame", elem_b="right_lower_pintle_pin")
        
    return ctx.report()


object_model = build_object_model()
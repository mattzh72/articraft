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


def _cut_rectangular_ring(
    *,
    outer_width: float,
    outer_height: float,
    depth: float,
    opening_width: float,
    opening_height: float,
    y_center: float = 0.0,
) -> cq.Workplane:
    """A rectangular ring in XZ, with thickness along Y."""
    solid = cq.Workplane("XY").box(outer_width, depth, outer_height).translate((0, y_center, 0))
    cutter = cq.Workplane("XY").box(opening_width, depth + 0.020, opening_height).translate(
        (0, y_center, 0)
    )
    return solid.cut(cutter)


def _steel_frame_shape() -> cq.Workplane:
    """Face flange plus a recessed throat that reads as the steel trim frame."""
    flange = _cut_rectangular_ring(
        outer_width=0.660,
        outer_height=0.900,
        depth=0.018,
        opening_width=0.580,
        opening_height=0.800,
        y_center=0.006,
    )

    # Return walls that continue the frame into the recessed wall box.
    throat_depth = 0.060
    throat_y = -0.032
    side_thick = 0.012
    top = cq.Workplane("XY").box(0.580, throat_depth, side_thick).translate((0, throat_y, 0.406))
    bottom = cq.Workplane("XY").box(0.580, throat_depth, side_thick).translate((0, throat_y, -0.406))
    left = cq.Workplane("XY").box(side_thick, throat_depth, 0.812).translate((-0.296, throat_y, 0))
    right = cq.Workplane("XY").box(side_thick, throat_depth, 0.812).translate((0.296, throat_y, 0))
    return flange.union(top).union(bottom).union(left).union(right)


def _inner_cover_shape() -> cq.Workplane:
    """Thin hinged dead-front cover with breaker handle apertures."""
    cover_width = 0.500
    cover_height = 0.540
    panel = (
        cq.Workplane("XY")
        .box(cover_width, 0.006, cover_height)
        # The top edge is below the hinge barrel, leaving room for knuckles.
        .translate((0, 0, -0.014 - cover_height / 2.0))
    )

    slot_w = 0.029
    slot_h = 0.078
    for i in range(8):
        x = -0.175 + i * 0.050
        slot = cq.Workplane("XY").box(slot_w, 0.020, slot_h).translate((x, 0, -0.200))
        panel = panel.cut(slot)

    # A shallow pressed stiffening lip around the cover face.
    lip_top = cq.Workplane("XY").box(0.460, 0.003, 0.010).translate((0, 0.0045, -0.060))
    lip_bottom = cq.Workplane("XY").box(0.460, 0.003, 0.010).translate((0, 0.0045, -0.500))
    lip_left = cq.Workplane("XY").box(0.010, 0.003, 0.440).translate((-0.235, 0.0045, -0.280))
    lip_right = cq.Workplane("XY").box(0.010, 0.003, 0.440).translate((0.235, 0.0045, -0.280))
    return panel.union(lip_top).union(lip_bottom).union(lip_left).union(lip_right)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_panel_board")

    plaster = model.material("painted_wall", color=(0.78, 0.77, 0.72, 1.0))
    frame_paint = model.material("powder_coated_steel", color=(0.18, 0.20, 0.21, 1.0))
    galvanized = model.material("galvanized_box", color=(0.48, 0.50, 0.50, 1.0))
    door_paint = model.material("slightly_gloss_door", color=(0.72, 0.74, 0.73, 1.0))
    dark = model.material("black_plastic", color=(0.015, 0.015, 0.014, 1.0))
    label = model.material("paper_labels", color=(0.93, 0.91, 0.80, 1.0))
    copper = model.material("brass_latch", color=(0.82, 0.64, 0.30, 1.0))

    wall_box = model.part("wall_box")

    wall = _cut_rectangular_ring(
        outer_width=0.920,
        outer_height=1.120,
        depth=0.060,
        opening_width=0.640,
        opening_height=0.880,
        y_center=-0.030,
    )
    wall_box.visual(
        mesh_from_cadquery(wall, "wall_cutout", tolerance=0.0008),
        material=plaster,
        name="wall_cutout",
    )
    wall_box.visual(
        mesh_from_cadquery(_steel_frame_shape(), "steel_frame", tolerance=0.0008),
        material=frame_paint,
        name="steel_frame",
    )

    # Recessed steel wall box: open at the front, with a visible back pan.
    wall_box.visual(
        Box((0.560, 0.012, 0.780)),
        origin=Origin(xyz=(0.0, -0.116, 0.0)),
        material=galvanized,
        name="back_pan",
    )
    wall_box.visual(
        Box((0.016, 0.112, 0.780)),
        origin=Origin(xyz=(-0.288, -0.062, 0.0)),
        material=galvanized,
        name="side_wall_0",
    )
    wall_box.visual(
        Box((0.016, 0.112, 0.780)),
        origin=Origin(xyz=(0.288, -0.062, 0.0)),
        material=galvanized,
        name="side_wall_1",
    )
    wall_box.visual(
        Box((0.560, 0.112, 0.016)),
        origin=Origin(xyz=(0.0, -0.062, 0.398)),
        material=galvanized,
        name="top_wall",
    )
    wall_box.visual(
        Box((0.560, 0.112, 0.016)),
        origin=Origin(xyz=(0.0, -0.062, -0.398)),
        material=galvanized,
        name="bottom_wall",
    )

    # DIN rail, breaker modules, toggles, and the paper directory that make the
    # interior read as an electrical panel once the covers are opened.
    wall_box.visual(
        Box((0.460, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.104, 0.110)),
        material=galvanized,
        name="din_rail",
    )
    for i in range(8):
        x = -0.175 + i * 0.050
        wall_box.visual(
            Box((0.043, 0.040, 0.122)),
            origin=Origin(xyz=(x, -0.090, 0.110)),
            material=dark,
            name=f"breaker_{i}",
        )
        wall_box.visual(
            Box((0.016, 0.022, 0.044)),
            origin=Origin(xyz=(x, -0.060, 0.110)),
            material=frame_paint,
            name=f"toggle_{i}",
        )
    wall_box.visual(
        Box((0.410, 0.003, 0.055)),
        origin=Origin(xyz=(0.0, -0.109, -0.210)),
        material=label,
        name="circuit_directory",
    )

    # Two exposed barrel hinges fixed to the left side of the steel frame.
    hinge_x = -0.295
    hinge_y = 0.026
    for idx, zc in enumerate((0.260, -0.260)):
        for seg, dz in enumerate((-0.043, 0.043)):
            wall_box.visual(
                Box((0.034, 0.006, 0.038)),
                origin=Origin(xyz=(hinge_x - 0.023, hinge_y - 0.008, zc + dz)),
                material=frame_paint,
                name=f"frame_hinge_leaf_{idx}_{seg}",
            )
            wall_box.visual(
                Cylinder(radius=0.010, length=0.032),
                origin=Origin(xyz=(hinge_x, hinge_y, zc + dz)),
                material=frame_paint,
                name=f"frame_hinge_barrel_{idx}_{seg}",
            )

    # Fixed knuckles for the inner top hinge.
    top_hinge_y = -0.025
    top_hinge_z = 0.310
    for idx, x in enumerate((-0.180, 0.180)):
        wall_box.visual(
            Box((0.140, 0.006, 0.088)),
            origin=Origin(xyz=(x, top_hinge_y - 0.006, top_hinge_z + 0.044)),
            material=galvanized,
            name=f"inner_hinge_leaf_{idx}",
        )
    for idx, x in enumerate((-0.180, 0.180)):
        wall_box.visual(
            Cylinder(radius=0.007, length=0.120),
            origin=Origin(xyz=(x, top_hinge_y, top_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"inner_hinge_barrel_{idx}",
        )
    wall_box.visual(
        Cylinder(radius=0.0035, length=0.500),
        origin=Origin(xyz=(0.0, top_hinge_y, top_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="inner_hinge_pin",
    )

    door = model.part("door")
    door.visual(
        Box((0.540, 0.014, 0.760)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((0.430, 0.0025, 0.600)),
        origin=Origin(xyz=(0.305, 0.0082, 0.0)),
        material=model.material("door_recess_shadow", color=(0.60, 0.63, 0.62, 1.0)),
        name="door_recess",
    )
    for name, xyz, size in (
        ("door_top_rib", (0.305, 0.010, 0.328), (0.430, 0.004, 0.018)),
        ("door_bottom_rib", (0.305, 0.010, -0.328), (0.430, 0.004, 0.018)),
        ("door_edge_rib_0", (0.082, 0.010, 0.0), (0.018, 0.004, 0.638)),
        ("door_edge_rib_1", (0.528, 0.010, 0.0), (0.018, 0.004, 0.638)),
    ):
        door.visual(Box(size), origin=Origin(xyz=xyz), material=door_paint, name=name)
    for idx, zc in enumerate((0.260, -0.260)):
        door.visual(
            Box((0.045, 0.004, 0.090)),
            origin=Origin(xyz=(0.025, 0.000, zc)),
            material=door_paint,
            name=f"door_hinge_leaf_{idx}",
        )
        door.visual(
            Cylinder(radius=0.010, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=door_paint,
            name=f"door_hinge_barrel_{idx}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_box,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=15.0, velocity=1.0),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=copper,
        name="latch_knob",
    )
    latch.visual(
        Box((0.044, 0.006, 0.009)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=copper,
        name="latch_grip",
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.505, 0.009, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi / 2.0, effort=1.0, velocity=2.0),
    )

    inner_cover = model.part("inner_cover")
    inner_cover.visual(
        mesh_from_cadquery(_inner_cover_shape(), "inner_cover_panel", tolerance=0.0007),
        material=galvanized,
        name="inner_cover_panel",
    )
    inner_cover.visual(
        Box((0.200, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=galvanized,
        name="cover_hinge_leaf",
    )
    inner_cover.visual(
        Cylinder(radius=0.007, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="cover_hinge_barrel",
    )
    model.articulation(
        "inner_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_box,
        child=inner_cover,
        origin=Origin(xyz=(0.0, top_hinge_y, top_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=4.0, velocity=1.2),
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

    wall_box = object_model.get_part("wall_box")
    door = object_model.get_part("door")
    inner_cover = object_model.get_part("inner_cover")
    door_hinge = object_model.get_articulation("door_hinge")
    inner_hinge = object_model.get_articulation("inner_cover_hinge")

    ctx.allow_overlap(
        wall_box,
        inner_cover,
        elem_a="inner_hinge_pin",
        elem_b="cover_hinge_barrel",
        reason="The top cover's center barrel is intentionally captured around the fixed hinge pin.",
    )
    ctx.expect_overlap(
        inner_cover,
        wall_box,
        axes="x",
        elem_a="cover_hinge_barrel",
        elem_b="inner_hinge_pin",
        min_overlap=0.17,
        name="inner cover barrel is retained on the hinge pin length",
    )
    ctx.expect_overlap(
        inner_cover,
        wall_box,
        axes="yz",
        elem_a="cover_hinge_barrel",
        elem_b="inner_hinge_pin",
        min_overlap=0.005,
        name="inner cover barrel surrounds the hinge pin radially",
    )

    ctx.expect_gap(
        door,
        wall_box,
        axis="y",
        positive_elem="door_panel",
        negative_elem="steel_frame",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door sits just proud of recessed steel frame",
    )
    ctx.expect_within(
        door,
        wall_box,
        axes="xz",
        inner_elem="door_panel",
        outer_elem="steel_frame",
        margin=0.0,
        name="main door is contained by the rectangular frame outline",
    )
    ctx.expect_gap(
        door,
        inner_cover,
        axis="y",
        positive_elem="door_panel",
        negative_elem="inner_cover_panel",
        min_gap=0.030,
        name="main door closes in front of the inner hinged cover",
    )
    ctx.expect_overlap(
        inner_cover,
        wall_box,
        axes="x",
        elem_a="inner_cover_panel",
        elem_b="din_rail",
        min_overlap=0.40,
        name="inner cover spans the breaker row",
    )

    rest_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "main door opens outward from left barrel hinges",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > rest_door_aabb[1][1] + 0.35,
        details=f"closed={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_cover_aabb = ctx.part_element_world_aabb(inner_cover, elem="inner_cover_panel")
    with ctx.pose({inner_hinge: 1.00}):
        open_cover_aabb = ctx.part_element_world_aabb(inner_cover, elem="inner_cover_panel")
    ctx.check(
        "inner cover flips outward about its top hinge",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > rest_cover_aabb[1][1] + 0.25,
        details=f"closed={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

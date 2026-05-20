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


BODY_WIDTH = 1.10
BODY_HEIGHT = 0.72
BODY_DEPTH = 0.080
OPENING_WIDTH = 0.56
OPENING_HEIGHT = 0.36
FRAME_OUTER_WIDTH = 0.74
FRAME_OUTER_HEIGHT = 0.50
FRAME_PROUD = 0.026

DOOR_WIDTH = 0.52
DOOR_HEIGHT = 0.30
DOOR_DEPTH = 0.075
HINGE_OFFSET = 0.030
HINGE_RADIUS = 0.016


def _ring_box(width: float, height: float, depth: float, center_y: float) -> cq.Workplane:
    """CadQuery rectangular frame in XZ with depth along Y."""
    outer = cq.Workplane("XY").box(width, depth, height).translate((0.0, center_y, 0.0))
    cutter = cq.Workplane("XY").box(OPENING_WIDTH, depth + 0.020, OPENING_HEIGHT).translate(
        (0.0, center_y, 0.0)
    )
    return outer.cut(cutter)


def _equipment_face_mesh() -> object:
    """Deep equipment face with a real through opening and raised service frame."""
    plate = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate(
        (0.0, BODY_DEPTH / 2.0, 0.0)
    )
    opening_cut = cq.Workplane("XY").box(
        OPENING_WIDTH, BODY_DEPTH + 0.060, OPENING_HEIGHT
    ).translate((0.0, BODY_DEPTH / 2.0, 0.0))
    plate = plate.cut(opening_cut)

    raised_frame = _ring_box(
        FRAME_OUTER_WIDTH,
        FRAME_OUTER_HEIGHT,
        FRAME_PROUD,
        -FRAME_PROUD / 2.0,
    )

    # A shallow darkened return tunnel makes the aperture read as an actual
    # service opening rather than a painted-on rectangle.
    wall = 0.030
    tunnel_depth = 0.135
    tunnel_center_y = BODY_DEPTH + tunnel_depth / 2.0
    side_wall = cq.Workplane("XY").box(wall, tunnel_depth, OPENING_HEIGHT + 2.0 * wall)
    left_wall = side_wall.translate((-OPENING_WIDTH / 2.0 - wall / 2.0, tunnel_center_y, 0.0))
    right_wall = side_wall.translate((OPENING_WIDTH / 2.0 + wall / 2.0, tunnel_center_y, 0.0))
    top_wall = cq.Workplane("XY").box(OPENING_WIDTH, tunnel_depth, wall).translate(
        (0.0, tunnel_center_y, OPENING_HEIGHT / 2.0 + wall / 2.0)
    )
    bottom_wall = cq.Workplane("XY").box(OPENING_WIDTH, tunnel_depth, wall).translate(
        (0.0, tunnel_center_y, -OPENING_HEIGHT / 2.0 - wall / 2.0)
    )
    back_plate = cq.Workplane("XY").box(
        OPENING_WIDTH + 2.0 * wall, 0.018, OPENING_HEIGHT + 2.0 * wall
    ).translate((0.0, BODY_DEPTH + tunnel_depth + 0.009, 0.0))

    face = (
        plate.union(raised_frame)
        .union(left_wall)
        .union(right_wall)
        .union(top_wall)
        .union(bottom_wall)
        .union(back_plate)
    )
    return face.edges("|Y").fillet(0.006)


def _door_panel_mesh() -> object:
    """One-piece short, deep access door with a raised pressed perimeter."""
    base = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_DEPTH, DOOR_HEIGHT)
        .translate((HINGE_OFFSET + DOOR_WIDTH / 2.0, 0.0, 0.0))
        .edges()
        .chamfer(0.004)
    )

    front_y = -DOOR_DEPTH / 2.0
    rib_depth = 0.014
    rib_overlap = 0.0015
    rib_y = front_y - rib_depth / 2.0 + rib_overlap
    rib_w = 0.020
    inset_x0 = HINGE_OFFSET + 0.028
    inset_x1 = HINGE_OFFSET + DOOR_WIDTH - 0.028
    rib_span = inset_x1 - inset_x0
    center_x = (inset_x0 + inset_x1) / 2.0
    top_rib = cq.Workplane("XY").box(rib_span, rib_depth, rib_w).translate(
        (center_x, rib_y, DOOR_HEIGHT / 2.0 - 0.032)
    )
    bottom_rib = cq.Workplane("XY").box(rib_span, rib_depth, rib_w).translate(
        (center_x, rib_y, -DOOR_HEIGHT / 2.0 + 0.032)
    )
    hinge_rib = cq.Workplane("XY").box(rib_w, rib_depth, DOOR_HEIGHT - 0.064).translate(
        (inset_x0, rib_y, 0.0)
    )
    latch_rib = cq.Workplane("XY").box(rib_w, rib_depth, DOOR_HEIGHT - 0.064).translate(
        (inset_x1, rib_y, 0.0)
    )

    return base.union(top_rib).union(bottom_rib).union(hinge_rib).union(latch_rib).edges().fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    painted_steel = model.material("painted_steel", color=(0.18, 0.21, 0.23, 1.0))
    frame_paint = model.material("slightly_lighter_frame", color=(0.24, 0.28, 0.30, 1.0))
    door_paint = model.material("blue_gray_door", color=(0.20, 0.31, 0.38, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.016, 0.014, 1.0))
    zinc = model.material("zinc_plated_hardware", color=(0.72, 0.72, 0.66, 1.0))
    amber = model.material("amber_latch_marker", color=(0.95, 0.55, 0.11, 1.0))

    equipment = model.part("equipment_face")
    equipment.visual(
        mesh_from_cadquery(_equipment_face_mesh(), "equipment_face_shell", tolerance=0.0015),
        material=painted_steel,
        name="face_frame",
    )

    # Rubber gasket seated on the frame just behind the closed door.
    gasket_y = -FRAME_PROUD - 0.003
    gasket_depth = 0.006
    gasket_wall = 0.018
    equipment.visual(
        Box((OPENING_WIDTH + 0.030, gasket_depth, gasket_wall)),
        origin=Origin(xyz=(0.0, gasket_y, OPENING_HEIGHT / 2.0 + gasket_wall / 2.0)),
        material=black_rubber,
        name="top_gasket",
    )
    equipment.visual(
        Box((OPENING_WIDTH + 0.030, gasket_depth, gasket_wall)),
        origin=Origin(xyz=(0.0, gasket_y, -OPENING_HEIGHT / 2.0 - gasket_wall / 2.0)),
        material=black_rubber,
        name="bottom_gasket",
    )
    equipment.visual(
        Box((gasket_wall, gasket_depth, OPENING_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - gasket_wall / 2.0, gasket_y, 0.0)),
        material=black_rubber,
        name="hinge_gasket",
    )
    equipment.visual(
        Box((gasket_wall, gasket_depth, OPENING_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + gasket_wall / 2.0, gasket_y, 0.0)),
        material=black_rubber,
        name="latch_gasket",
    )

    hinge_x = -OPENING_WIDTH / 2.0 - HINGE_OFFSET
    hinge_y = -FRAME_PROUD - 0.040
    # Fixed hinge leaf sits on the frame side of the pin.
    equipment.visual(
        Box((0.034, 0.046, 0.355)),
        origin=Origin(xyz=(hinge_x - 0.036, -FRAME_PROUD - 0.020, 0.0)),
        material=frame_paint,
        name="hinge_mount_spine",
    )
    equipment.visual(
        Box((0.032, 0.008, 0.345)),
        origin=Origin(xyz=(hinge_x - 0.036, hinge_y, 0.0)),
        material=zinc,
        name="fixed_hinge_leaf",
    )
    for zc, length, name in (
        (-0.128, 0.082, "fixed_lower_knuckle"),
        (0.128, 0.082, "fixed_upper_knuckle"),
    ):
        equipment.visual(
            Cylinder(radius=HINGE_RADIUS, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=zinc,
            name=name,
        )
        equipment.visual(
            Box((0.018, 0.010, length)),
            origin=Origin(xyz=(hinge_x - 0.020, hinge_y, zc)),
            material=zinc,
            name=f"{name}_bridge",
        )
    equipment.visual(
        Cylinder(radius=0.006, length=0.350),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=zinc,
        name="hinge_pin",
    )

    # Latch strike on the frame edge opposite the hinge.
    equipment.visual(
        Box((0.030, 0.010, 0.130)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + 0.040, -FRAME_PROUD - 0.004, 0.0)),
        material=zinc,
        name="latch_strike",
    )
    equipment.visual(
        Box((0.018, 0.012, 0.040)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + 0.020, -FRAME_PROUD - 0.014, 0.0)),
        material=black_rubber,
        name="latch_bumper",
    )

    screw_positions = (
        (-FRAME_OUTER_WIDTH / 2.0 + 0.055, FRAME_OUTER_HEIGHT / 2.0 - 0.055),
        (FRAME_OUTER_WIDTH / 2.0 - 0.055, FRAME_OUTER_HEIGHT / 2.0 - 0.055),
        (-FRAME_OUTER_WIDTH / 2.0 + 0.055, -FRAME_OUTER_HEIGHT / 2.0 + 0.055),
        (FRAME_OUTER_WIDTH / 2.0 - 0.055, -FRAME_OUTER_HEIGHT / 2.0 + 0.055),
    )
    for idx, (x, z) in enumerate(screw_positions):
        equipment.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, -FRAME_PROUD - 0.003, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"frame_screw_{idx}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel_mesh(), "short_deep_door_panel", tolerance=0.001),
        material=door_paint,
        name="door_panel",
    )
    # Moving hinge leaf and the single center knuckle belong to the door part.
    door.visual(
        Box((0.026, 0.008, 0.325)),
        origin=Origin(xyz=(HINGE_RADIUS + 0.018, 0.0, 0.0)),
        material=zinc,
        name="moving_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="moving_center_knuckle",
    )
    door.visual(
        Box((0.017, 0.010, 0.155)),
        origin=Origin(xyz=(HINGE_RADIUS + 0.006, 0.0, 0.0)),
        material=zinc,
        name="moving_knuckle_bridge",
    )
    # Latch edge features are fixed to the door because the prompt requests one
    # revolute door joint, not a separately articulated latch.
    door.visual(
        Box((0.044, 0.026, 0.112)),
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_WIDTH - 0.050, -DOOR_DEPTH / 2.0 - 0.013, 0.0)),
        material=zinc,
        name="latch_handle",
    )
    door.visual(
        Box((0.010, 0.030, 0.070)),
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_WIDTH + 0.004, -0.002, 0.0)),
        material=zinc,
        name="latch_edge_tongue",
    )
    door.visual(
        Box((0.030, 0.006, 0.050)),
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_WIDTH - 0.050, -DOOR_DEPTH / 2.0 - 0.029, 0.038)),
        material=amber,
        name="latch_marker",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=equipment,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        # The door panel extends along child +X.  Negative Z makes positive
        # motion swing the latch edge outward from the equipment face (-Y).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    equipment = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        equipment,
        door,
        elem_a="hinge_pin",
        elem_b="moving_center_knuckle",
        reason="The fixed hinge pin is intentionally captured inside the moving barrel knuckle.",
    )
    ctx.expect_within(
        equipment,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="moving_center_knuckle",
        margin=0.002,
        name="hinge pin is centered inside moving knuckle",
    )
    ctx.expect_overlap(
        equipment,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="moving_center_knuckle",
        min_overlap=0.14,
        name="hinge pin passes through moving knuckle",
    )
    ctx.expect_gap(
        equipment,
        door,
        axis="y",
        positive_elem="face_frame",
        negative_elem="door_panel",
        min_gap=0.001,
        max_gap=0.010,
        name="closed door stands just proud of frame",
    )
    ctx.expect_overlap(
        door,
        equipment,
        axes="xz",
        elem_a="door_panel",
        elem_b="face_frame",
        min_overlap=0.24,
        name="door covers the framed service opening",
    )
    ctx.expect_overlap(
        door,
        equipment,
        axes="z",
        elem_a="moving_center_knuckle",
        elem_b="fixed_hinge_leaf",
        min_overlap=0.12,
        name="door knuckle shares the vertical hinge span",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="latch_edge_tongue")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(door, elem="latch_edge_tongue")
    ctx.check(
        "positive hinge angle swings latch edge outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

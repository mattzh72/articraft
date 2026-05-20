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


DASH_W = 0.52
DASH_H = 0.28
DASH_T = 0.028
OPEN_W = 0.36
OPEN_H = 0.215
BIN_DEPTH = 0.235
WALL_T = 0.018

HINGE_X = -OPEN_W / 2.0 - 0.006
HINGE_Y = -0.033
DOOR_W = 0.354
DOOR_H = 0.205
DOOR_T = 0.016
LOCK_X = 0.300
LOCK_Z = 0.024


def _box_at(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    solid = shapes[0]
    for shape in shapes[1:]:
        solid = solid.union(shape)
    return solid


def _tube_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center_z: float,
) -> cq.Workplane:
    # Local Z-axis hollow hinge knuckle, centered at (0, 0, center_z).
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, center_z - length / 2.0))
    )


def _dashboard_shell() -> cq.Workplane:
    fascia = _box_at((DASH_W, DASH_T, DASH_H), (0.0, 0.0, 0.0))
    opening_cutter = _box_at((OPEN_W, DASH_T + 0.020, OPEN_H), (0.0, 0.0, 0.0))
    fascia = fascia.cut(opening_cutter)

    bin_center_y = DASH_T / 2.0 + BIN_DEPTH / 2.0
    side_z = 0.0
    side_h = OPEN_H + WALL_T
    shell_members = [
        fascia,
        _box_at((WALL_T, BIN_DEPTH, side_h), (-OPEN_W / 2.0 - WALL_T / 2.0, bin_center_y, side_z)),
        _box_at((WALL_T, BIN_DEPTH, side_h), (OPEN_W / 2.0 + WALL_T / 2.0, bin_center_y, side_z)),
        _box_at((OPEN_W + 2.0 * WALL_T, BIN_DEPTH, WALL_T), (0.0, bin_center_y, OPEN_H / 2.0 + WALL_T / 2.0)),
        _box_at((OPEN_W + 2.0 * WALL_T, BIN_DEPTH, WALL_T), (0.0, bin_center_y, -OPEN_H / 2.0 - WALL_T / 2.0)),
        _box_at((OPEN_W + 2.0 * WALL_T, WALL_T, OPEN_H + 2.0 * WALL_T), (0.0, DASH_T / 2.0 + BIN_DEPTH, 0.0)),
    ]
    return _union_all(shell_members)


def _gasket_ring() -> cq.Workplane:
    ring = _box_at((OPEN_W + 0.030, 0.004, OPEN_H + 0.030), (0.0, -DASH_T / 2.0 - 0.002, 0.0))
    hole = _box_at((OPEN_W + 0.004, 0.010, OPEN_H + 0.004), (0.0, -DASH_T / 2.0 - 0.002, 0.0))
    return ring.cut(hole)


def _door_panel() -> cq.Workplane:
    panel = _box_at((DOOR_W, DOOR_T, DOOR_H), (DOOR_W / 2.0 + 0.006, 0.0, 0.0))
    # Through-bore for the rotating lock cylinder along the local Y axis.
    lock_cutter = (
        cq.Workplane("XY")
        .circle(0.0155)
        .extrude(0.090)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((LOCK_X, 0.045, LOCK_Z))
    )
    return panel.cut(lock_cutter)


def _frame_hinge(hinge_z: float) -> cq.Workplane:
    outer_r = 0.0070
    inner_r = 0.0034
    # Mounting leaf is offset to the fixed side and reaches back to the
    # dashboard face, so the exposed hinge is visibly bolted to the side wall.
    root_leaf = _box_at((0.051, 0.018, 0.060), (-0.0327, 0.0155, hinge_z))
    lower = _tube_z(outer_r, inner_r, 0.018, hinge_z - 0.026)
    upper = _tube_z(outer_r, inner_r, 0.018, hinge_z + 0.026)
    lower_bridge = _box_at((0.008, 0.008, 0.014), (-0.008, 0.0075, hinge_z - 0.026))
    upper_bridge = _box_at((0.008, 0.008, 0.014), (-0.008, 0.0075, hinge_z + 0.026))
    return _union_all([root_leaf, lower, upper, lower_bridge, upper_bridge])


def _door_hinge(hinge_z: float) -> cq.Workplane:
    outer_r = 0.0068
    inner_r = 0.0036
    door_leaf = _box_at((0.054, 0.004, 0.043), (0.035, -0.0085, hinge_z))
    middle = _tube_z(outer_r, inner_r, 0.018, hinge_z)
    return _union_all([door_leaf, middle])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lockable_glove_compartment")

    dashboard_mat = model.material("charcoal_dashboard", rgba=(0.05, 0.055, 0.060, 1.0))
    inner_mat = model.material("matte_black_liner", rgba=(0.012, 0.012, 0.014, 1.0))
    gasket_mat = model.material("soft_rubber_gasket", rgba=(0.003, 0.003, 0.003, 1.0))
    door_mat = model.material("slightly_lighter_door", rgba=(0.075, 0.078, 0.083, 1.0))
    recess_mat = model.material("door_recess_shadow", rgba=(0.025, 0.027, 0.030, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    zinc_mat = model.material("zinc_cam", rgba=(0.46, 0.47, 0.44, 1.0))
    slot_mat = model.material("key_slot_black", rgba=(0.0, 0.0, 0.0, 1.0))

    dashboard = model.part("dashboard_bin")
    dashboard.visual(
        mesh_from_cadquery(_dashboard_shell(), "dashboard_shell", tolerance=0.0008),
        material=dashboard_mat,
        name="fascia_shell",
    )
    dashboard.visual(
        mesh_from_cadquery(_gasket_ring(), "gasket_ring", tolerance=0.0008),
        material=gasket_mat,
        name="gasket_ring",
    )
    # Dark back panel makes the fixed bin read as a real cavity behind the opening.
    dashboard.visual(
        Box((OPEN_W - 0.030, 0.004, OPEN_H - 0.030)),
        origin=Origin(xyz=(0.0, DASH_T / 2.0 + BIN_DEPTH - 0.010, 0.0)),
        material=inner_mat,
        name="dark_bin_back",
    )
    dashboard.visual(
        mesh_from_cadquery(_frame_hinge(0.070), "upper_frame_hinge", tolerance=0.0006),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        material=steel_mat,
        name="upper_frame_hinge",
    )
    dashboard.visual(
        mesh_from_cadquery(_frame_hinge(-0.070), "lower_frame_hinge", tolerance=0.0006),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        material=steel_mat,
        name="lower_frame_hinge",
    )
    dashboard.visual(
        Box((0.010, 0.020, 0.055)),
        origin=Origin(xyz=(OPEN_W / 2.0 + WALL_T * 0.88, DASH_T / 2.0 + 0.026, LOCK_Z)),
        material=steel_mat,
        name="latch_keeper",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel(), "door_panel", tolerance=0.0007),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Box((0.235, 0.0012, 0.105)),
        origin=Origin(xyz=(0.178, -DOOR_T / 2.0 - 0.0006, -0.010)),
        material=recess_mat,
        name="front_recess",
    )
    door.visual(
        mesh_from_cadquery(_door_hinge(0.070), "upper_door_hinge", tolerance=0.0006),
        material=steel_mat,
        name="upper_door_hinge",
    )
    door.visual(
        mesh_from_cadquery(_door_hinge(-0.070), "lower_door_hinge", tolerance=0.0006),
        material=steel_mat,
        name="lower_door_hinge",
    )

    lock = model.part("lock_cylinder")
    lock.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="cylinder_body",
    )
    lock.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, -0.0125, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="front_bezel",
    )
    lock.visual(
        Box((0.017, 0.001, 0.0032)),
        origin=Origin(xyz=(0.0, -0.0147, 0.0)),
        material=slot_mat,
        name="key_slot",
    )

    cam = model.part("cam")
    cam.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=zinc_mat,
        name="cam_shaft",
    )
    cam.visual(
        Box((0.076, 0.004, 0.014)),
        origin=Origin(xyz=(0.038, 0.033, 0.0)),
        material=zinc_mat,
        name="cam_arm",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "lock_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock,
        origin=Origin(xyz=(LOCK_X, 0.0, LOCK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "lock_to_cam",
        ArticulationType.FIXED,
        parent=lock,
        child=cam,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard = object_model.get_part("dashboard_bin")
    door = object_model.get_part("door")
    cam = object_model.get_part("cam")
    hinge = object_model.get_articulation("door_hinge")
    lock_turn = object_model.get_articulation("lock_turn")

    ctx.expect_gap(
        dashboard,
        door,
        axis="y",
        positive_elem="fascia_shell",
        negative_elem="door_panel",
        min_gap=0.006,
        name="closed door sits proud of the dashboard opening",
    )
    ctx.expect_overlap(
        door,
        dashboard,
        axes="xz",
        elem_a="door_panel",
        elem_b="fascia_shell",
        min_overlap=0.18,
        name="door covers the rectangular glovebox opening",
    )
    ctx.expect_overlap(
        door,
        dashboard,
        axes="xy",
        elem_a="upper_door_hinge",
        elem_b="upper_frame_hinge",
        min_overlap=0.010,
        name="upper door hinge remains captured on the side pin line",
    )
    ctx.expect_overlap(
        door,
        dashboard,
        axes="xy",
        elem_a="lower_door_hinge",
        elem_b="lower_frame_hinge",
        min_overlap=0.010,
        name="lower door hinge remains captured on the side pin line",
    )

    closed_hinge_pos = ctx.part_world_position(door)
    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.25}):
        open_hinge_pos = ctx.part_world_position(door)
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_overlap(
            door,
            dashboard,
            axes="xy",
            elem_a="upper_door_hinge",
            elem_b="upper_frame_hinge",
            min_overlap=0.010,
            name="opened door is still clipped into upper hinge",
        )
        ctx.expect_overlap(
            door,
            dashboard,
            axes="xy",
            elem_a="lower_door_hinge",
            elem_b="lower_frame_hinge",
            min_overlap=0.010,
            name="opened door is still clipped into lower hinge",
        )

    if closed_aabb is not None and open_aabb is not None:
        closed_min_y = closed_aabb[0][1]
        open_min_y = open_aabb[0][1]
    else:
        closed_min_y = open_min_y = None
    ctx.check(
        "side door swings outward about a fixed vertical hinge",
        closed_hinge_pos is not None
        and open_hinge_pos is not None
        and closed_min_y is not None
        and open_min_y is not None
        and abs(closed_hinge_pos[0] - open_hinge_pos[0]) < 0.001
        and abs(closed_hinge_pos[1] - open_hinge_pos[1]) < 0.001
        and abs(closed_hinge_pos[2] - open_hinge_pos[2]) < 0.001
        and open_min_y < closed_min_y - 0.12,
        details=f"closed_hinge={closed_hinge_pos}, open_hinge={open_hinge_pos}, closed_min_y={closed_min_y}, open_min_y={open_min_y}",
    )

    cam_rest = ctx.part_element_world_aabb(cam, elem="cam_arm")
    with ctx.pose({lock_turn: math.pi / 2.0}):
        cam_turned = ctx.part_element_world_aabb(cam, elem="cam_arm")
    if cam_rest is not None and cam_turned is not None:
        rest_dx = cam_rest[1][0] - cam_rest[0][0]
        rest_dz = cam_rest[1][2] - cam_rest[0][2]
        turned_dx = cam_turned[1][0] - cam_turned[0][0]
        turned_dz = cam_turned[1][2] - cam_turned[0][2]
    else:
        rest_dx = rest_dz = turned_dx = turned_dz = None
    ctx.check(
        "key cylinder rotates the internal cam",
        rest_dx is not None
        and rest_dz is not None
        and turned_dx is not None
        and turned_dz is not None
        and turned_dx < rest_dx * 0.55
        and turned_dz > rest_dz + 0.045,
        details=f"rest_dx={rest_dx}, rest_dz={rest_dz}, turned_dx={turned_dx}, turned_dz={turned_dz}",
    )

    return ctx.report()


object_model = build_object_model()

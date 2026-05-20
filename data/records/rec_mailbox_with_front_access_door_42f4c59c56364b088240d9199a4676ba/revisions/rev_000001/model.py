from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_LENGTH = 0.56
BODY_WIDTH = 0.22
BODY_SIDE_HEIGHT = 0.14
WALL_THICKNESS = 0.012
BODY_BASE_Z = 0.575
FRONT_X = BODY_LENGTH / 2.0
DOOR_OFFSET_X = 0.006
HINGE_DROP = 0.012
DOOR_THICKNESS = 0.018
HINGE_RADIUS = 0.009


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _arch_loop(width: float, side_height: float, *, bottom_z: float = 0.0, segments: int = 28):
    """Return a convex rural-mailbox cross section in the YZ plane."""
    half_width = width * 0.5
    loop = [(-half_width, bottom_z), (half_width, bottom_z)]
    for index in range(segments + 1):
        theta = (math.pi * index) / segments
        loop.append((half_width * math.cos(theta), side_height + half_width * math.sin(theta)))
    return loop


def _mailbox_shell_mesh() -> MeshGeometry:
    """Thin-walled arched tube with an open front and a closed rear panel."""
    geom = MeshGeometry()
    inner_width = BODY_WIDTH - 2.0 * WALL_THICKNESS
    outer_loop = _arch_loop(BODY_WIDTH, BODY_SIDE_HEIGHT, segments=30)
    inner_loop = _arch_loop(
        inner_width,
        BODY_SIDE_HEIGHT,
        bottom_z=WALL_THICKNESS,
        segments=30,
    )

    x_rear = -BODY_LENGTH / 2.0
    x_front = BODY_LENGTH / 2.0

    def add_loop(x: float, loop):
        return [geom.add_vertex(x, y, z) for y, z in loop]

    outer_rear = add_loop(x_rear, outer_loop)
    outer_front = add_loop(x_front, outer_loop)
    inner_rear = add_loop(x_rear, inner_loop)
    inner_front = add_loop(x_front, inner_loop)
    count = len(outer_loop)

    for i in range(count):
        j = (i + 1) % count
        _add_quad(geom, outer_rear[i], outer_rear[j], outer_front[j], outer_front[i])
        _add_quad(geom, inner_rear[j], inner_rear[i], inner_front[i], inner_front[j])
        # Rolled front lip around the mail opening.
        _add_quad(geom, outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        # Rear wall annulus tying the outer shell to the inner liner.
        _add_quad(geom, outer_rear[j], outer_rear[i], inner_rear[i], inner_rear[j])

    rear_center = geom.add_vertex(x_rear, 0.0, BODY_SIDE_HEIGHT * 0.52)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(rear_center, inner_rear[j], inner_rear[i])

    return geom


def _arched_door_mesh() -> MeshGeometry:
    """Solid arched door slab, built in the child frame whose Y axis is the hinge."""
    geom = MeshGeometry()
    # Slightly oversize the door so it visibly covers the rolled front rim.
    profile = _arch_loop(BODY_WIDTH + 0.018, BODY_SIDE_HEIGHT + 0.006, bottom_z=HINGE_DROP, segments=30)

    rear = [geom.add_vertex(0.0, y, z) for y, z in profile]
    front = [geom.add_vertex(DOOR_THICKNESS, y, z) for y, z in profile]
    count = len(profile)

    for i in range(count):
        j = (i + 1) % count
        _add_quad(geom, rear[i], rear[j], front[j], front[i])

    rear_center = geom.add_vertex(0.0, 0.0, BODY_SIDE_HEIGHT * 0.56)
    front_center = geom.add_vertex(DOOR_THICKNESS, 0.0, BODY_SIDE_HEIGHT * 0.56)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(rear_center, rear[i], rear[j])
        geom.add_face(front_center, front[j], front[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_curbside_mailbox")

    galvanized = model.material("galvanized_steel", rgba=(0.64, 0.67, 0.69, 1.0))
    dark_metal = model.material("dark_hinge_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    post_finish = model.material("painted_post", rgba=(0.30, 0.24, 0.18, 1.0))
    plate_finish = model.material("mounting_plate", rgba=(0.42, 0.43, 0.42, 1.0))
    black_handle = model.material("black_handle", rgba=(0.04, 0.04, 0.04, 1.0))

    body = model.part("mailbox_body")
    body.visual(
        Box((0.28, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=post_finish,
        name="ground_foot",
    )
    body.visual(
        Box((0.08, 0.08, 0.51)),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=post_finish,
        name="short_post",
    )
    body.visual(
        Box((0.43, 0.18, 0.038)),
        origin=Origin(xyz=(-0.035, 0.0, 0.559)),
        material=plate_finish,
        name="mounting_plate",
    )
    body.visual(
        mesh_from_geometry(_mailbox_shell_mesh(), "mailbox_rounded_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z)),
        material=galvanized,
        name="body_shell",
    )
    # Small screw heads on the mounting plate; they are seated into the plate so
    # they read as fasteners rather than floating dots.
    for idx, x in enumerate((-0.17, 0.08)):
        body.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, -0.055, BODY_BASE_Z + 0.002)),
            material=dark_metal,
            name=f"screw_head_{idx}_0",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, 0.055, BODY_BASE_Z + 0.002)),
            material=dark_metal,
            name=f"screw_head_{idx}_1",
        )

    hinge_x = FRONT_X + DOOR_OFFSET_X
    hinge_z = BODY_BASE_Z - HINGE_DROP
    for y in (-0.086, 0.086):
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.052),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"body_hinge_barrel_{'neg' if y < 0 else 'pos'}",
        )
        body.visual(
            Box((0.020, 0.050, 0.014)),
            origin=Origin(xyz=(hinge_x, y, BODY_BASE_Z - 0.007)),
            material=dark_metal,
            name=f"body_hinge_leaf_{'neg' if y < 0 else 'pos'}",
        )

    door = model.part("front_door")
    door.visual(
        mesh_from_geometry(_arched_door_mesh(), "mailbox_front_door"),
        material=galvanized,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.018, 0.082, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.007)),
        material=dark_metal,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(DOOR_THICKNESS + 0.011, 0.0, BODY_SIDE_HEIGHT * 0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_handle,
        name="handle_stem",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(DOOR_THICKNESS + 0.028, 0.0, BODY_SIDE_HEIGHT * 0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_handle,
        name="handle_knob",
    )

    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("mailbox_body")
    door = object_model.get_part("front_door")
    hinge = object_model.get_articulation("body_to_front_door")

    ctx.check(
        "front door hinge has ninety degree travel",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and abs(hinge.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={hinge.motion_limits}",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        min_gap=0.003,
        max_gap=0.010,
        positive_elem="door_panel",
        negative_elem="body_shell",
        name="closed door sits just outside front rim",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.20,
        elem_a="door_panel",
        elem_b="body_shell",
        name="arched door covers the mail opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: math.pi / 2.0}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "front door swings outward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.20
        and open_aabb[1][2] < closed_aabb[1][2] - 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

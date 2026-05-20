from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _sloped_rain_cap(width: float, depth: float, thickness: float, fall: float) -> MeshGeometry:
    """Simple sloped rectangular rain cap mesh, centered on X and starting at Y=0."""

    half_w = width / 2.0
    y0 = 0.0
    y1 = depth
    z_back = fall
    z_front = 0.0

    vertices = [
        (-half_w, y0, z_back),
        (half_w, y0, z_back),
        (half_w, y1, z_front),
        (-half_w, y1, z_front),
        (-half_w, y0, z_back + thickness),
        (half_w, y0, z_back + thickness),
        (half_w, y1, z_front + thickness),
        (-half_w, y1, z_front + thickness),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]

    geom = MeshGeometry()
    for x, y, z in vertices:
        geom.add_vertex(x, y, z)
    for face in faces:
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mailbox")

    painted_metal = model.material("deep_green_enamel", rgba=(0.05, 0.16, 0.11, 1.0))
    door_metal = model.material("slightly_lighter_green", rgba=(0.07, 0.22, 0.15, 1.0))
    dark_interior = model.material("shadowed_interior", rgba=(0.015, 0.018, 0.016, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.68, 0.62, 1.0))
    latch_brass = model.material("aged_brass", rgba=(0.72, 0.50, 0.18, 1.0))
    wall_finish = model.material("painted_wall", rgba=(0.72, 0.69, 0.62, 1.0))

    width = 0.38
    depth = 0.16
    height = 0.28
    wall_t = 0.012
    door_t = 0.012
    hinge_radius = 0.005
    hinge_axis_x = -width / 2.0 - 0.013
    hinge_axis_y = depth + 0.010
    hinge_axis_z = height / 2.0
    door_width = width + 0.026
    door_height = height - 0.030

    body = model.part("body")

    # A small wall field is included on the fixed body so the mailbox clearly reads
    # as wall-mounted while keeping the body as the single fixed root assembly.
    body.visual(
        Box((0.54, 0.018, 0.46)),
        origin=Origin(xyz=(0.0, -0.009, height / 2.0)),
        material=wall_finish,
        name="wall_panel",
    )

    shell_outer = cq.Workplane("XY").box(width, depth, height).translate((0.0, depth / 2.0, height / 2.0))
    shell_void = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall_t, depth + 0.020, height - 2.0 * wall_t)
        .translate((0.0, wall_t + (depth + 0.020) / 2.0, height / 2.0))
    )
    shell = shell_outer.cut(shell_void)
    body.visual(
        mesh_from_cadquery(shell, "box_shell", tolerance=0.0008),
        origin=Origin(),
        material=painted_metal,
        name="box_shell",
    )

    # Dark visible back plane inside the hollow cavity.
    body.visual(
        Box((width - 0.035, 0.003, height - 0.055)),
        origin=Origin(xyz=(0.0, wall_t + 0.0015, height / 2.0)),
        material=dark_interior,
        name="inner_back",
    )

    cap_mesh = _sloped_rain_cap(width + 0.070, depth + 0.058, 0.010, 0.026)
    body.visual(
        mesh_from_geometry(cap_mesh, "rain_cap"),
        origin=Origin(xyz=(0.0, -0.018, height + 0.004)),
        material=painted_metal,
        name="rain_cap",
    )
    body.visual(
        Box((width - 0.020, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.030, height + 0.007)),
        material=painted_metal,
        name="cap_seam",
    )

    # Fixed strike/keeper plate opposite the hinge for the rotary latch.
    body.visual(
        Box((0.008, 0.026, 0.080)),
        origin=Origin(xyz=(width / 2.0 + 0.007, depth + 0.004, height / 2.0)),
        material=brushed_steel,
        name="latch_keeper",
    )
    body.visual(
        Box((0.018, 0.012, 0.046)),
        origin=Origin(xyz=(width / 2.0 + 0.001, depth - 0.006, height / 2.0)),
        material=brushed_steel,
        name="keeper_mount",
    )

    # Fixed hinge pin and small bridges that visibly tie the pin to the body side.
    body.visual(
        Cylinder(radius=hinge_radius, length=door_height * 0.86),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_axis_z)),
        material=brushed_steel,
        name="hinge_pin",
    )
    body.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(hinge_axis_x + 0.005, hinge_axis_y - 0.011, hinge_axis_z)),
        material=brushed_steel,
        name="hinge_bridge",
    )
    body.visual(
        Box((0.018, 0.052, 0.018)),
        origin=Origin(xyz=(-width / 2.0 - 0.003, depth - 0.040, hinge_axis_z)),
        material=painted_metal,
        name="hinge_reinforcement",
    )

    door = model.part("door")
    door.visual(
        Box((door_width - 0.026, door_t, door_height)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.0)),
        material=door_metal,
        name="door_panel",
    )
    # Raised perimeter trim and a shallow mail-door emboss make the access panel
    # read as a thin sheet-metal door rather than a blank slab.
    trim_y = door_t / 2.0 + 0.003
    door.visual(
        Box((door_width - 0.050, 0.006, 0.012)),
        origin=Origin(xyz=(door_width / 2.0 + 0.004, trim_y, door_height / 2.0 - 0.018)),
        material=painted_metal,
        name="top_rail",
    )
    door.visual(
        Box((door_width - 0.050, 0.006, 0.012)),
        origin=Origin(xyz=(door_width / 2.0 + 0.004, trim_y, -door_height / 2.0 + 0.018)),
        material=painted_metal,
        name="bottom_rail",
    )
    door.visual(
        Box((0.012, 0.006, door_height - 0.050)),
        origin=Origin(xyz=(0.045, trim_y, 0.0)),
        material=painted_metal,
        name="hinge_stile",
    )
    door.visual(
        Box((0.012, 0.006, door_height - 0.050)),
        origin=Origin(xyz=(door_width - 0.020, trim_y, 0.0)),
        material=painted_metal,
        name="latch_stile",
    )
    door.visual(
        Box((door_width - 0.135, 0.004, 0.018)),
        origin=Origin(xyz=(door_width / 2.0 + 0.020, door_t / 2.0 + 0.001, 0.043)),
        material=painted_metal,
        name="mail_emboss",
    )
    door.visual(
        Box((0.046, 0.007, 0.020)),
        origin=Origin(xyz=(0.028, -0.001, 0.088)),
        material=brushed_steel,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.046, 0.007, 0.020)),
        origin=Origin(xyz=(0.028, -0.001, -0.088)),
        material=brushed_steel,
        name="lower_hinge_leaf",
    )

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_brass,
        name="latch_knob",
    )
    latch.visual(
        Box((0.020, 0.007, 0.078)),
        origin=Origin(xyz=(0.0, 0.021, 0.0)),
        material=latch_brass,
        name="turn_bar",
    )
    latch.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="latch_spindle",
    )

    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(door_width - 0.043, door_t / 2.0 + 0.001, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-1.57, upper=1.57),
    )

    # Keep names live for static analyzers and emphasize the primary mechanism.
    assert door_hinge.name == "body_to_door"
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

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("body_to_door")
    latch_pivot = object_model.get_articulation("door_to_latch")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="door_panel",
        negative_elem="box_shell",
        name="closed door sits just proud of the box front",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.20,
        elem_a="door_panel",
        elem_b="box_shell",
        name="closed access door covers the mailbox opening",
    )
    ctx.expect_gap(
        latch,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="latch_knob",
        negative_elem="door_panel",
        name="rotary latch is seated on the door face",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        min_overlap=0.018,
        elem_a="latch_knob",
        elem_b="door_panel",
        name="rotary latch pivot lies on the opposite door edge",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "front door swings outward on a vertical side hinge",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][1] > closed_panel[1][1] + 0.20,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    closed_bar = ctx.part_element_world_aabb(latch, elem="turn_bar")
    with ctx.pose({latch_pivot: math.pi / 2.0}):
        turned_bar = ctx.part_element_world_aabb(latch, elem="turn_bar")

    def _extent(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "latch turn bar rotates about its own short pivot",
        closed_bar is not None
        and turned_bar is not None
        and _extent(closed_bar, 2) > _extent(closed_bar, 0) * 2.0
        and _extent(turned_bar, 0) > _extent(turned_bar, 2) * 2.0,
        details=f"closed={closed_bar}, turned={turned_bar}",
    )

    return ctx.report()


object_model = build_object_model()

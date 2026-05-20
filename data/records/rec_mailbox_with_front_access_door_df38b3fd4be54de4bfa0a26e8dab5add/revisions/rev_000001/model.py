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


def _arched_profile(width: float, side_height: float, *, bottom: float = 0.0, segments: int = 20):
    """Return a flat-bottomed arched loop in the local YZ plane."""
    r = width / 2.0
    pts = [(-r, bottom), (r, bottom), (r, side_height)]
    for i in range(1, segments + 1):
        a = math.pi * i / segments
        pts.append((r * math.cos(a), side_height + r * math.sin(a)))
    return pts


def _add_loop(geom: MeshGeometry, points):
    return [geom.add_vertex(*p) for p in points]


def _connect_loops(geom: MeshGeometry, loop_a, loop_b, *, flip: bool = False) -> None:
    n = len(loop_a)
    for i in range(n):
        a0 = loop_a[i]
        a1 = loop_a[(i + 1) % n]
        b0 = loop_b[i]
        b1 = loop_b[(i + 1) % n]
        if flip:
            geom.add_face(a0, b0, b1)
            geom.add_face(a0, b1, a1)
        else:
            geom.add_face(a0, a1, b1)
            geom.add_face(a0, b1, b0)


def _cap_loop(geom: MeshGeometry, loop, *, flip: bool = False) -> None:
    for i in range(1, len(loop) - 1):
        if flip:
            geom.add_face(loop[0], loop[i + 1], loop[i])
        else:
            geom.add_face(loop[0], loop[i], loop[i + 1])


def _solid_arched_prism(width: float, side_height: float, depth: float, *, x_center: float = 0.0) -> MeshGeometry:
    profile = _arched_profile(width, side_height)
    x0 = x_center - depth / 2.0
    x1 = x_center + depth / 2.0
    geom = MeshGeometry()
    back = _add_loop(geom, [(x0, y, z) for y, z in profile])
    front = _add_loop(geom, [(x1, y, z) for y, z in profile])
    _connect_loops(geom, back, front)
    _cap_loop(geom, back, flip=True)
    _cap_loop(geom, front)
    return geom


def _arched_ring_prism(
    outer_width: float,
    outer_side_height: float,
    inner_width: float,
    inner_side_height: float,
    depth: float,
    *,
    x_center: float = 0.0,
) -> MeshGeometry:
    outer = _arched_profile(outer_width, outer_side_height)
    inner = _arched_profile(inner_width, inner_side_height, bottom=0.014)
    x0 = x_center - depth / 2.0
    x1 = x_center + depth / 2.0
    geom = MeshGeometry()
    outer_back = _add_loop(geom, [(x0, y, z) for y, z in outer])
    outer_front = _add_loop(geom, [(x1, y, z) for y, z in outer])
    inner_back = _add_loop(geom, [(x0, y, z) for y, z in inner])
    inner_front = _add_loop(geom, [(x1, y, z) for y, z in inner])
    _connect_loops(geom, outer_back, outer_front)
    _connect_loops(geom, inner_front, inner_back, flip=True)
    _connect_loops(geom, outer_front, inner_front)
    _connect_loops(geom, inner_back, outer_back)
    return geom


def _mailbox_shell_mesh() -> MeshGeometry:
    """Thin arched tunnel with a broader flat arched front rim."""
    wall = 0.015
    sections = [
        (-0.300, 0.240, 0.250, 0.240 - 2.0 * wall, 0.250),
        (0.205, 0.240, 0.250, 0.240 - 2.0 * wall, 0.250),
        (0.260, 0.280, 0.270, 0.240 - 2.0 * wall, 0.250),
    ]
    geom = MeshGeometry()
    outer_loops = []
    inner_loops = []
    for x, outer_w, outer_h, inner_w, inner_h in sections:
        outer = _arched_profile(outer_w, outer_h)
        inner = _arched_profile(inner_w, inner_h, bottom=wall)
        outer_loops.append(_add_loop(geom, [(x, y, z) for y, z in outer]))
        inner_loops.append(_add_loop(geom, [(x, y, z) for y, z in inner]))

    for i in range(len(sections) - 1):
        _connect_loops(geom, outer_loops[i], outer_loops[i + 1])
        _connect_loops(geom, inner_loops[i + 1], inner_loops[i], flip=True)

    # Flat front arched face/rim, leaving a real opening into the hollow box.
    _connect_loops(geom, outer_loops[-1], inner_loops[-1])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decorative_arched_mailbox")

    painted_metal = model.material("deep_green_painted_metal", color=(0.05, 0.22, 0.16, 1.0))
    dark_metal = model.material("dark_oiled_hinge_metal", color=(0.02, 0.018, 0.015, 1.0))
    brass = model.material("aged_brass_trim", color=(0.78, 0.56, 0.22, 1.0))
    door_green = model.material("slightly_lighter_door_enamel", color=(0.07, 0.32, 0.22, 1.0))
    flag_red = model.material("red_signal_flag", color=(0.86, 0.04, 0.025, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_mailbox_shell_mesh(), "arched_hollow_body"),
        material=painted_metal,
        name="arched_hollow_body",
    )
    body.visual(
        mesh_from_geometry(_solid_arched_prism(0.242, 0.250, 0.014, x_center=-0.307), "rear_cap"),
        material=painted_metal,
        name="rear_cap",
    )
    body.visual(
        mesh_from_geometry(
            _arched_ring_prism(0.290, 0.272, 0.216, 0.248, 0.006, x_center=0.263),
            "front_brass_rim",
        ),
        material=brass,
        name="front_brass_rim",
    )
    # Side flag bracket and washer are part of the fixed mailbox body and touch the side shell.
    body.visual(
        Box((0.088, 0.016, 0.080)),
        origin=Origin(xyz=(0.050, 0.127, 0.265)),
        material=brass,
        name="flag_bracket_plate",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.050, 0.138, 0.265), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="flag_pivot_washer",
    )

    front_door = model.part("front_door")
    front_door.visual(
        mesh_from_geometry(_solid_arched_prism(0.245, 0.258, 0.018, x_center=0.006), "door_panel"),
        material=door_green,
        name="door_panel",
    )
    front_door.visual(
        mesh_from_geometry(
            _arched_ring_prism(0.236, 0.252, 0.176, 0.212, 0.006, x_center=0.015),
            "raised_door_border",
        ),
        material=brass,
        name="raised_door_border",
    )
    for y in (-0.088, 0.088):
        for z in (0.075, 0.285):
            front_door.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(0.020, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brass,
                name=f"door_rivet_{y}_{z}",
            )
    front_door.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.024, 0.000, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="pull_knob",
    )
    for y in (-0.068, 0.068):
        front_door.visual(
            Cylinder(radius=0.011, length=0.074),
            origin=Origin(xyz=(0.000, y, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"door_hinge_knuckle_{y}",
        )

    flag = model.part("flag")
    flag.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flag_red,
        name="flag_hub",
    )
    flag.visual(
        Box((0.024, 0.010, 0.145)),
        origin=Origin(xyz=(0.0, 0.006, 0.075)),
        material=flag_red,
        name="flag_stem",
    )
    flag.visual(
        Box((0.110, 0.011, 0.060)),
        origin=Origin(xyz=(0.043, 0.006, 0.160)),
        material=flag_red,
        name="flag_plate",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(0.277, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.65),
    )
    model.articulation(
        "flag_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(0.050, 0.150, 0.265)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.57),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("front_door")
    flag = object_model.get_part("flag")
    door_hinge = object_model.get_articulation("door_hinge")
    flag_pivot = object_model.get_articulation("flag_pivot")

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_panel",
        min_gap=0.001,
        max_gap=0.020,
        name="closed door is proud of the flat front rim",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.18,
        name="arched door covers the mailbox opening",
    )
    ctx.expect_contact(
        flag,
        body,
        elem_a="flag_hub",
        elem_b="flag_pivot_washer",
        contact_tol=0.0015,
        name="flag hub seats on side pivot washer",
    )

    door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    hinge_pos = ctx.part_world_position(door)
    ctx.check(
        "door panel is clipped at the lower hinge line",
        door_panel_aabb is not None
        and hinge_pos is not None
        and door_panel_aabb[0][2] >= hinge_pos[2] - 0.001
        and door_panel_aabb[0][2] <= hinge_pos[2] + 0.002,
        details=f"door_panel_aabb={door_panel_aabb}, hinge_pos={hinge_pos}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.40}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "front flap drops outward around its lower hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.18
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    raised_flag_aabb = ctx.part_element_world_aabb(flag, elem="flag_plate")
    with ctx.pose({flag_pivot: 1.30}):
        lowered_flag_aabb = ctx.part_element_world_aabb(flag, elem="flag_plate")
    ctx.check(
        "side flag rotates forward and downward on its small pivot",
        raised_flag_aabb is not None
        and lowered_flag_aabb is not None
        and lowered_flag_aabb[1][0] > raised_flag_aabb[1][0] + 0.08
        and lowered_flag_aabb[1][2] < raised_flag_aabb[1][2] - 0.04,
        details=f"raised={raised_flag_aabb}, lowered={lowered_flag_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

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


BODY_LENGTH = 0.78
BODY_WIDTH = 0.32
BODY_SIDE_HEIGHT = 0.20
BODY_WALL = 0.014
BODY_BASE_Z = 0.70
FRONT_X = BODY_LENGTH / 2.0
SIDE_Y = BODY_WIDTH / 2.0


def _d_profile(
    width: float,
    roof_center_z: float,
    *,
    z_bottom: float = 0.0,
    arc_segments: int = 24,
) -> list[tuple[float, float]]:
    """D-shaped rural-mailbox section, returned as (y, z) points."""

    radius = width / 2.0
    pts: list[tuple[float, float]] = [
        (-radius, z_bottom),
        (radius, z_bottom),
        (radius, roof_center_z),
    ]
    for i in range(1, arc_segments + 1):
        theta = math.pi * i / arc_segments
        pts.append((radius * math.cos(theta), roof_center_z + radius * math.sin(theta)))
    return pts


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _add_loop(mesh: MeshGeometry, x: float, profile: list[tuple[float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for y, z in profile]


def _cap_profile(mesh: MeshGeometry, loop: list[int], x: float, center_yz: tuple[float, float]) -> None:
    center_idx = mesh.add_vertex(x, center_yz[0], center_yz[1])
    n = len(loop)
    for i in range(n):
        mesh.add_face(center_idx, loop[i], loop[(i + 1) % n])


def _make_mailbox_shell() -> MeshGeometry:
    """Open-front, hollow, arched metal body with a closed rear wall."""

    mesh = MeshGeometry()
    outer = _d_profile(BODY_WIDTH, BODY_SIDE_HEIGHT)
    inner = _d_profile(
        BODY_WIDTH - 2.0 * BODY_WALL,
        BODY_SIDE_HEIGHT,
        z_bottom=BODY_WALL,
    )

    x_back = -BODY_LENGTH / 2.0
    x_inner_back = x_back + 0.030
    x_front = BODY_LENGTH / 2.0

    outer_back = _add_loop(mesh, x_back, outer)
    outer_front = _add_loop(mesh, x_front, outer)
    inner_back = _add_loop(mesh, x_inner_back, inner)
    inner_front = _add_loop(mesh, x_front, inner)
    n = len(outer)

    for i in range(n):
        j = (i + 1) % n
        _add_quad(mesh, outer_back[i], outer_back[j], outer_front[j], outer_front[i])
        _add_quad(mesh, inner_front[i], inner_front[j], inner_back[j], inner_back[i])
        _add_quad(mesh, outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        _add_quad(mesh, outer_back[i], inner_back[i], inner_back[j], outer_back[j])

    _cap_profile(mesh, outer_back, x_back, (0.0, BODY_SIDE_HEIGHT * 0.52))
    _cap_profile(mesh, inner_back, x_inner_back, (0.0, BODY_SIDE_HEIGHT * 0.56))
    return mesh


def _make_d_solid(
    width: float,
    roof_center_z: float,
    *,
    x0: float,
    x1: float,
    z_bottom: float = 0.0,
) -> MeshGeometry:
    mesh = MeshGeometry()
    profile = _d_profile(width, roof_center_z, z_bottom=z_bottom)
    back = _add_loop(mesh, x0, profile)
    front = _add_loop(mesh, x1, profile)
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        _add_quad(mesh, back[i], back[j], front[j], front[i])
    _cap_profile(mesh, back, x0, (0.0, (z_bottom + roof_center_z) * 0.50))
    _cap_profile(mesh, front, x1, (0.0, (z_bottom + roof_center_z) * 0.50))
    return mesh


def _make_d_ring(
    outer_width: float,
    roof_center_z: float,
    rim: float,
    *,
    x0: float,
    x1: float,
) -> MeshGeometry:
    mesh = MeshGeometry()
    outer = _d_profile(outer_width, roof_center_z)
    inner = _d_profile(outer_width - 2.0 * rim, roof_center_z, z_bottom=rim)
    outer_back = _add_loop(mesh, x0, outer)
    outer_front = _add_loop(mesh, x1, outer)
    inner_back = _add_loop(mesh, x0, inner)
    inner_front = _add_loop(mesh, x1, inner)
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        _add_quad(mesh, outer_back[i], outer_back[j], outer_front[j], outer_front[i])
        _add_quad(mesh, inner_front[i], inner_front[j], inner_back[j], inner_back[i])
        _add_quad(mesh, outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        _add_quad(mesh, outer_back[i], inner_back[i], inner_back[j], outer_back[j])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_rural_mailbox")

    galvanized = model.material("galvanized", rgba=(0.62, 0.66, 0.67, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    flag_red = model.material("flag_red", rgba=(0.86, 0.05, 0.03, 1.0))
    wood = model.material("weathered_wood", rgba=(0.46, 0.31, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_mailbox_shell(), "mailbox_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z)),
        material=galvanized,
        name="body_shell",
    )
    body.visual(
        Box((0.45, 0.23, 0.045)),
        origin=Origin(xyz=(-0.045, 0.0, BODY_BASE_Z - 0.020)),
        material=wood,
        name="saddle_board",
    )
    body.visual(
        Box((0.10, 0.10, BODY_BASE_Z)),
        origin=Origin(xyz=(-0.055, 0.0, BODY_BASE_Z / 2.0 - 0.010)),
        material=wood,
        name="post",
    )
    body.visual(
        Box((0.20, 0.18, 0.035)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0175)),
        material=wood,
        name="post_foot",
    )
    body.visual(
        Box((0.035, 0.34, 0.009)),
        origin=Origin(xyz=(FRONT_X + 0.006, 0.0, BODY_BASE_Z - 0.0115)),
        material=dark_metal,
        name="hinge_sill",
    )
    body.visual(
        Box((0.210, 0.055, 0.034)),
        origin=Origin(xyz=(0.282, 0.0, BODY_BASE_Z - 0.024)),
        material=dark_metal,
        name="sill_web",
    )
    body.visual(
        Box((0.090, 0.008, 0.105)),
        origin=Origin(xyz=(0.18, SIDE_Y + 0.004, BODY_BASE_Z + 0.200)),
        material=dark_metal,
        name="flag_bracket",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(
            xyz=(0.18, SIDE_Y + 0.012, BODY_BASE_Z + 0.200),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="flag_boss",
    )

    door = model.part("door")
    door_width = BODY_WIDTH + 0.012
    door_roof_z = BODY_SIDE_HEIGHT + 0.002
    door.visual(
        mesh_from_geometry(
            _make_d_solid(door_width, door_roof_z, x0=0.004, x1=0.020),
            "door_panel",
        ),
        material=galvanized,
        name="door_panel",
    )
    door.visual(
        mesh_from_geometry(
            _make_d_ring(door_width, door_roof_z, 0.026, x0=0.017, x1=0.028),
            "door_rolled_rim",
        ),
        material=galvanized,
        name="door_rim",
    )
    door.visual(
        Cylinder(radius=0.0065, length=0.245),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.028, 0.018, 0.050)),
        origin=Origin(xyz=(0.034, -0.052, 0.160)),
        material=dark_metal,
        name="handle_stem_0",
    )
    door.visual(
        Box((0.028, 0.018, 0.050)),
        origin=Origin(xyz=(0.034, 0.052, 0.160)),
        material=dark_metal,
        name="handle_stem_1",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.122),
        origin=Origin(xyz=(0.047, 0.0, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handle_grip",
    )

    flag = model.part("signal_flag")
    flag.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flag_red,
        name="flag_hub",
    )
    flag.visual(
        Box((0.285, 0.008, 0.030)),
        origin=Origin(xyz=(-0.142, 0.006, 0.0)),
        material=flag_red,
        name="flag_arm",
    )
    flag.visual(
        Box((0.080, 0.009, 0.105)),
        origin=Origin(xyz=(-0.275, 0.006, 0.040)),
        material=flag_red,
        name="flag_plate",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(FRONT_X + 0.006, 0.0, BODY_BASE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "flag_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(0.18, SIDE_Y + 0.028, BODY_BASE_Z + 0.200)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    flag = object_model.get_part("signal_flag")
    door_hinge = object_model.get_articulation("door_hinge")
    flag_pivot = object_model.get_articulation("flag_pivot")

    with ctx.pose({door_hinge: 0.0, flag_pivot: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_panel",
            negative_elem="body_shell",
            min_gap=0.004,
            max_gap=0.025,
            name="closed door sits just proud of front opening",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="door_panel",
            elem_b="body_shell",
            min_overlap=0.20,
            name="rounded door covers arched mailbox opening",
        )
        door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.check(
            "door panel is clipped above lower hinge line",
            door_aabb is not None and door_aabb[0][2] >= BODY_BASE_Z - 0.001,
            details=f"door panel aabb={door_aabb}",
        )
        flag_down = ctx.part_world_aabb(flag)

    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.check(
            "front door swings outward and downward",
            open_aabb is not None
            and open_aabb[1][0] > FRONT_X + 0.18
            and open_aabb[1][2] < BODY_BASE_Z + 0.24,
            details=f"open door panel aabb={open_aabb}",
        )

    with ctx.pose({flag_pivot: 1.35}):
        flag_up = ctx.part_world_aabb(flag)
        ctx.check(
            "side signal flag raises on its pivot",
            flag_down is not None and flag_up is not None and flag_up[1][2] > flag_down[1][2] + 0.10,
            details=f"down={flag_down}, up={flag_up}",
        )

    return ctx.report()


object_model = build_object_model()

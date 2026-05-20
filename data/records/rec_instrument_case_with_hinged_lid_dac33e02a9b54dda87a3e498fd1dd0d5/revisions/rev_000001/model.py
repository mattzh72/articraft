from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_LENGTH = 0.78
CASE_WIDTH = 0.24
LOWER_HEIGHT = 0.068
CAVITY_BOTTOM_Z = 0.034
HINGE_Y = CASE_WIDTH / 2.0 + 0.006
HINGE_Z = LOWER_HEIGHT + 0.011


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _add_fan_cap(geom: MeshGeometry, loop: list[int], center_xyz: tuple[float, float, float], *, reverse: bool = False) -> None:
    center_id = geom.add_vertex(*center_xyz)
    count = len(loop)
    for index in range(count):
        a = loop[index]
        b = loop[(index + 1) % count]
        if reverse:
            geom.add_face(center_id, b, a)
        else:
            geom.add_face(center_id, a, b)


def _loop_vertices(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _profile_3d(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    y_center: float = 0.0,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    safe_radius = min(radius, width * 0.48, depth * 0.48)
    return [
        (x, y + y_center, z)
        for x, y in rounded_rect_profile(width, depth, safe_radius, corner_segments=corner_segments)
    ]


def _connect_loops(geom: MeshGeometry, lower: list[int], upper: list[int], *, reverse: bool = False) -> None:
    count = len(lower)
    for index in range(count):
        j = (index + 1) % count
        if reverse:
            _add_quad(geom, lower[index], upper[index], upper[j], lower[j])
        else:
            _add_quad(geom, lower[index], lower[j], upper[j], upper[index])


def _build_lower_shell_mesh() -> MeshGeometry:
    """Rounded flat-bottom tray with a molded top recess."""
    geom = MeshGeometry()
    outer_bottom = _loop_vertices(geom, _profile_3d(CASE_LENGTH, CASE_WIDTH, 0.038, 0.0))
    outer_top = _loop_vertices(geom, _profile_3d(CASE_LENGTH, CASE_WIDTH, 0.038, LOWER_HEIGHT))
    inner_top = _loop_vertices(
        geom,
        _profile_3d(CASE_LENGTH - 0.085, CASE_WIDTH - 0.070, 0.030, LOWER_HEIGHT),
    )
    inner_bottom = _loop_vertices(
        geom,
        _profile_3d(CASE_LENGTH - 0.105, CASE_WIDTH - 0.090, 0.025, CAVITY_BOTTOM_Z),
    )

    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, inner_top, outer_top, reverse=True)  # flat molded rim
    _connect_loops(geom, inner_bottom, inner_top, reverse=True)  # sloped pocket wall
    _add_fan_cap(geom, outer_bottom, (0.0, 0.0, 0.0), reverse=True)
    _add_fan_cap(geom, inner_bottom, (0.0, 0.0, CAVITY_BOTTOM_Z))
    return geom


def _build_lid_shell_mesh() -> MeshGeometry:
    """Thin, open-bottom domed lid clipped to the rear hinge by knuckles."""
    geom = MeshGeometry()
    y_center = -0.130
    outer_sections = [
        _profile_3d(CASE_LENGTH - 0.010, 0.232, 0.040, -0.006, y_center=y_center),
        _profile_3d(CASE_LENGTH - 0.022, 0.218, 0.038, 0.017, y_center=y_center),
        _profile_3d(CASE_LENGTH - 0.060, 0.178, 0.032, 0.044, y_center=y_center),
        _profile_3d(CASE_LENGTH - 0.155, 0.092, 0.025, 0.062, y_center=y_center),
    ]
    inner_sections = [
        _profile_3d(CASE_LENGTH - 0.095, 0.154, 0.027, 0.004, y_center=y_center),
        _profile_3d(CASE_LENGTH - 0.118, 0.134, 0.024, 0.025, y_center=y_center),
        _profile_3d(CASE_LENGTH - 0.185, 0.080, 0.020, 0.046, y_center=y_center),
    ]

    outer_loops = [_loop_vertices(geom, section) for section in outer_sections]
    inner_loops = [_loop_vertices(geom, section) for section in inner_sections]

    for lower, upper in zip(outer_loops, outer_loops[1:]):
        _connect_loops(geom, lower, upper)
    top_center = (
        0.0,
        y_center,
        0.062,
    )
    _add_fan_cap(geom, outer_loops[-1], top_center)

    for lower, upper in zip(inner_loops, inner_loops[1:]):
        _connect_loops(geom, lower, upper, reverse=True)
    _add_fan_cap(geom, inner_loops[-1], (0.0, y_center, 0.046), reverse=True)

    # Thick rim around the underside; the central underside is intentionally open.
    _connect_loops(geom, outer_loops[0], inner_loops[0], reverse=True)
    return geom


def _slot_profile(length: float, width: float, *, segments: int = 10) -> list[tuple[float, float]]:
    radius = width / 2.0
    straight = max(0.0, length - width)
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        angle = -math.pi / 2.0 + math.pi * i / segments
        pts.append((straight / 2.0 + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = math.pi / 2.0 + math.pi * i / segments
        pts.append((-straight / 2.0 + radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_clarinet_case")

    shell_mat = model.material("textured_black_shell", rgba=(0.025, 0.027, 0.030, 1.0))
    edge_mat = model.material("slightly_worn_edges", rgba=(0.055, 0.057, 0.062, 1.0))
    velvet_mat = model.material("wine_velvet_liner", rgba=(0.30, 0.025, 0.055, 1.0))
    cavity_shadow = model.material("deep_cavity_shadow", rgba=(0.015, 0.010, 0.014, 1.0))
    metal_mat = model.material("brushed_nickel", rgba=(0.64, 0.62, 0.56, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.010, 0.010, 0.011, 1.0))

    lower = model.part("lower_body")
    lower.visual(
        mesh_from_geometry(_build_lower_shell_mesh(), "lower_molded_shell"),
        material=shell_mat,
        name="lower_shell",
    )
    lower.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(CASE_LENGTH - 0.125, CASE_WIDTH - 0.108, 0.024, corner_segments=10),
                0.004,
            ),
            "velvet_recess_insert",
        ),
        origin=Origin(xyz=(0.0, 0.0, CAVITY_BOTTOM_Z - 0.001)),
        material=velvet_mat,
        name="velvet_bed",
    )
    lower.visual(
        mesh_from_geometry(ExtrudeGeometry.from_z0(_slot_profile(0.515, 0.036), 0.002), "long_body_cavity"),
        origin=Origin(xyz=(-0.030, -0.012, CAVITY_BOTTOM_Z + 0.003)),
        material=cavity_shadow,
        name="body_cavity",
    )
    lower.visual(
        Cylinder(radius=0.038, length=0.002),
        origin=Origin(xyz=(0.290, -0.012, CAVITY_BOTTOM_Z + 0.004)),
        material=cavity_shadow,
        name="bell_cup",
    )
    lower.visual(
        mesh_from_geometry(ExtrudeGeometry.from_z0(_slot_profile(0.155, 0.026), 0.002), "mouthpiece_cavity"),
        origin=Origin(xyz=(-0.280, 0.042, CAVITY_BOTTOM_Z + 0.004), rpy=(0.0, 0.0, 0.18)),
        material=cavity_shadow,
        name="mouthpiece_cavity",
    )
    lower.visual(
        mesh_from_geometry(ExtrudeGeometry.from_z0(_slot_profile(0.145, 0.030), 0.002), "barrel_cavity"),
        origin=Origin(xyz=(-0.160, 0.043, CAVITY_BOTTOM_Z + 0.004), rpy=(0.0, 0.0, -0.08)),
        material=cavity_shadow,
        name="barrel_cavity",
    )
    lower.visual(
        Box((CASE_LENGTH - 0.055, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -CASE_WIDTH / 2.0 - 0.002, LOWER_HEIGHT + 0.002)),
        material=edge_mat,
        name="front_seal",
    )

    # Alternating rear hinge barrels fixed to the lower shell.
    lower.visual(
        Box((0.110, 0.012, 0.014)),
        origin=Origin(xyz=(-0.310, HINGE_Y - 0.004, HINGE_Z - 0.006)),
        material=shell_mat,
        name="body_hinge_0_leaf",
    )
    lower.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(-0.310, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="body_hinge_0",
    )
    lower.visual(
        Box((0.110, 0.012, 0.014)),
        origin=Origin(xyz=(0.000, HINGE_Y - 0.004, HINGE_Z - 0.006)),
        material=shell_mat,
        name="body_hinge_1_leaf",
    )
    lower.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.000, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="body_hinge_1",
    )
    lower.visual(
        Box((0.110, 0.012, 0.014)),
        origin=Origin(xyz=(0.310, HINGE_Y - 0.004, HINGE_Z - 0.006)),
        material=shell_mat,
        name="body_hinge_2_leaf",
    )
    lower.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.310, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="body_hinge_2",
    )

    # Fixed hinge ears for the two snap latch pivots.
    latch_xs = (-0.225, 0.225)
    latch_pivot_y = -CASE_WIDTH / 2.0 - 0.018
    latch_pivot_z = LOWER_HEIGHT + 0.005
    for latch_index, x in enumerate(latch_xs):
        lower.visual(
            Box((0.086, 0.018, 0.016)),
            origin=Origin(xyz=(x, -CASE_WIDTH / 2.0 - 0.007, latch_pivot_z)),
            material=shell_mat,
            name=f"latch_mount_{latch_index}",
        )
        for ear_index, x_offset in enumerate((-0.028, 0.028)):
            lower.visual(
                Cylinder(radius=0.006, length=0.014),
                origin=Origin(
                    xyz=(x + x_offset, latch_pivot_y, latch_pivot_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=metal_mat,
                name=f"latch_ear_{latch_index}_{ear_index}",
            )

    # Small molded rubber feet keep the flat-bottom shell visibly grounded.
    for foot_index, (x, y) in enumerate(((-0.300, -0.075), (0.300, -0.075), (-0.300, 0.075), (0.300, 0.075))):
        lower.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber_mat,
            name=f"foot_{foot_index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell_mesh(), "domed_hollow_lid"),
        material=shell_mat,
        name="lid_shell",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.120, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.239, 0.020)),
        material=edge_mat,
        name="front_catch_rail",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.140),
        origin=Origin(xyz=(-0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="lid_hinge_0",
    )
    lid.visual(
        Box((0.130, 0.026, 0.010)),
        origin=Origin(xyz=(-0.155, -0.008, 0.000)),
        material=shell_mat,
        name="hinge_tab_0",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.140),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="lid_hinge_1",
    )
    lid.visual(
        Box((0.130, 0.026, 0.010)),
        origin=Origin(xyz=(0.155, -0.008, 0.000)),
        material=shell_mat,
        name="hinge_tab_1",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.35),
    )

    for latch_index, x in enumerate(latch_xs):
        latch = model.part(f"latch_{latch_index}")
        latch.visual(
            Cylinder(radius=0.006, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.048, 0.011, 0.014)),
            origin=Origin(xyz=(0.0, -0.007, 0.004)),
            material=metal_mat,
            name="barrel_bridge",
        )
        latch.visual(
            Box((0.064, 0.010, 0.060)),
            origin=Origin(xyz=(0.0, -0.014, 0.026)),
            material=metal_mat,
            name="snap_flap",
        )
        latch.visual(
            Box((0.054, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, -0.001, 0.055)),
            material=metal_mat,
            name="snap_hook",
        )
        model.articulation(
            f"latch_pivot_{latch_index}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=latch,
            origin=Origin(xyz=(x, latch_pivot_y, latch_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_body")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")

    with ctx.pose({rear_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            lower,
            axes="xy",
            min_overlap=0.18,
            elem_a="lid_shell",
            elem_b="lower_shell",
            name="closed lid covers the lower shell footprint",
        )
        ctx.expect_gap(
            lid,
            lower,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="lid_shell",
            negative_elem="lower_shell",
            name="closed domed lid sits just above the lower rim",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({"latch_pivot_0": 1.05, "latch_pivot_1": 1.05}):
        open_latch_0_aabb = ctx.part_element_world_aabb(latch_0, elem="snap_hook")
        open_latch_1_aabb = ctx.part_element_world_aabb(latch_1, elem="snap_hook")

    with ctx.pose({rear_hinge: 1.20, "latch_pivot_0": 1.05, "latch_pivot_1": 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.expect_overlap(
            lid,
            lower,
            axes="yz",
            min_overlap=0.015,
            elem_a="lid_hinge_0",
            elem_b="body_hinge_0",
            name="lid hinge knuckle stays coaxial with rear hinge while open",
        )

    ctx.check(
        "rear hinge lifts the domed lid",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.11,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_latch_0 = ctx.part_element_world_aabb(latch_0, elem="snap_hook")
    closed_latch_1 = ctx.part_element_world_aabb(latch_1, elem="snap_hook")
    ctx.check(
        "snap latches rotate outward and downward",
        closed_latch_0 is not None
        and closed_latch_1 is not None
        and open_latch_0_aabb is not None
        and open_latch_1_aabb is not None
        and open_latch_0_aabb[0][1] < closed_latch_0[0][1] - 0.015
        and open_latch_1_aabb[0][1] < closed_latch_1[0][1] - 0.015
        and open_latch_0_aabb[1][2] < closed_latch_0[1][2] - 0.010
        and open_latch_1_aabb[1][2] < closed_latch_1[1][2] - 0.010,
        details=f"closed=({closed_latch_0}, {closed_latch_1}), open=({open_latch_0_aabb}, {open_latch_1_aabb})",
    )

    return ctx.report()


object_model = build_object_model()

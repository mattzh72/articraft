from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _ring(outer_radius: float, inner_radius: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z1 - z0)
        .translate((0, 0, z0))
    )

def _make_body_geometry() -> LatheGeometry:
    """Connected thin-wall lathe mesh for the hollow bottle body."""

    profile = [
        (0.082, 0.000),
        (0.082, 0.030),
        (0.070, 0.030),
        (0.077, 0.070),
        (0.077, 0.185),
        (0.045, 0.235),
        (0.025, 0.235),
        (0.025, 0.291),
        (0.031, 0.291),
        (0.031, 0.299),
        (0.018, 0.299),
        (0.018, 0.238),
        (0.056, 0.215),
        (0.056, 0.035),
        (0.065, 0.025),
        (0.065, 0.006),
        (0.075, 0.000),
    ]
    return LatheGeometry(profile, segments=96, closed=True)


def _thread_mesh_geometry(
    *,
    radius: float,
    pitch: float,
    height: float,
    radial_depth: float,
    axial_half_width: float,
    z0: float = 0.0,
    segments_per_turn: int = 56,
) -> MeshGeometry:
    """Closed helical ridge mesh without tiny detached OCC end slivers."""

    geom = MeshGeometry()
    turns = height / pitch
    steps = max(32, int(turns * segments_per_turn))
    station_indices: list[list[int]] = []

    for i in range(steps + 1):
        t = i / steps
        theta = 2.0 * pi * turns * t
        z = z0 + height * t
        section = [
            (radius, z - axial_half_width),
            (radius, z + axial_half_width),
            (radius + radial_depth, z + axial_half_width * 0.16),
            (radius + radial_depth, z - axial_half_width * 0.16),
        ]
        indices = []
        for r, zz in section:
            indices.append(geom.add_vertex(r * cos(theta), r * sin(theta), zz))
        station_indices.append(indices)

    for i in range(steps):
        a = station_indices[i]
        b = station_indices[i + 1]
        for k in range(4):
            k_next = (k + 1) % 4
            geom.add_face(a[k], b[k], b[k_next])
            geom.add_face(a[k], b[k_next], a[k_next])

    start = station_indices[0]
    end = station_indices[-1]
    geom.add_face(start[0], start[1], start[2])
    geom.add_face(start[0], start[2], start[3])
    geom.add_face(end[0], end[2], end[1])
    geom.add_face(end[0], end[3], end[2])
    return geom


def _make_cap_shell() -> cq.Workplane:
    """Open-bottom, service-windowed utility cap with knurled vertical ribs."""

    cap = cq.Workplane("XY").circle(0.047).extrude(0.066)
    cap = cap.cut(cq.Workplane("XY").circle(0.034).extrude(0.060))
    cap = cap.union(_ring(0.041, 0.018, 0.066, 0.073))

    for index in range(28):
        angle = 360.0 * index / 28.0
        rib = (
            cq.Workplane("XY")
            .box(0.006, 0.006, 0.052)
            .translate((0.049, 0.0, 0.034))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        cap = cap.union(rib)

    # Two opposed service windows reveal the internal thread and the engaged neck.
    for angle in (0.0, 180.0):
        cutter = (
            cq.Workplane("XY")
            .box(0.032, 0.034, 0.028)
            .translate((0.050, 0.0, 0.030))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        cap = cap.cut(cutter)

    try:
        cap = cap.edges().fillet(0.0015)
    except Exception:
        pass
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_screwcap_bottle")

    molded_olive = Material("molded_olive_hdpe", rgba=(0.20, 0.27, 0.16, 1.0))
    rubber_black = Material("matte_black_rubber", rgba=(0.015, 0.017, 0.016, 1.0))
    cap_black = Material("painted_black_cap", rgba=(0.025, 0.030, 0.033, 1.0))
    dark_recess = Material("dark_recesses", rgba=(0.002, 0.002, 0.002, 1.0))
    thread_wear = Material("worn_thread_highlights", rgba=(0.45, 0.50, 0.42, 1.0))
    stainless = Material("stainless_fasteners", rgba=(0.78, 0.75, 0.68, 1.0))
    white_paint = Material("worn_white_index_paint", rgba=(0.88, 0.86, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_body_geometry(), "hollow_molded_body"),
        material=molded_olive,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_ring(0.085, 0.060, 0.006, 0.030), "rubber_base_bumper"),
        material=rubber_black,
        name="base_bumper",
    )
    body.visual(
        mesh_from_cadquery(_ring(0.055, 0.030, 0.226, 0.242), "reinforced_neck_collar"),
        material=rubber_black,
        name="neck_collar",
    )
    body.visual(
        mesh_from_geometry(
            _thread_mesh_geometry(
                radius=0.0246,
                pitch=0.014,
                height=0.038,
                radial_depth=0.0043,
                axial_half_width=0.0035,
                z0=0.247,
            ),
            "external_neck_thread",
        ),
        material=thread_wear,
        name="neck_thread",
    )
    body.visual(
        Box((0.092, 0.008, 0.108)),
        origin=Origin(xyz=(0.0, -0.077, 0.122)),
        material=rubber_black,
        name="front_grip_pad",
    )
    body.visual(
        Box((0.092, 0.008, 0.108)),
        origin=Origin(xyz=(0.0, 0.077, 0.122)),
        material=rubber_black,
        name="rear_grip_pad",
    )
    body.visual(
        Box((0.012, 0.038, 0.150)),
        origin=Origin(xyz=(0.077, 0.0, 0.112)),
        material=molded_olive,
        name="side_rail_0",
    )
    body.visual(
        Box((0.012, 0.038, 0.150)),
        origin=Origin(xyz=(-0.077, 0.0, 0.112)),
        material=molded_olive,
        name="side_rail_1",
    )

    for i, (x, y, z) in enumerate(
        (
            (-0.038, -0.082, 0.158),
            (0.038, -0.082, 0.158),
            (-0.038, 0.082, 0.158),
            (0.038, 0.082, 0.158),
        )
    ):
        body.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"fastener_{i}",
        )
        body.visual(
            Box((0.008, 0.0012, 0.0015)),
            origin=Origin(xyz=(x, y + (-0.0006 if y < 0 else 0.0006), z)),
            material=dark_recess,
            name=f"fastener_slot_{i}",
        )

    cap_axis = model.part("cap_axis")

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_make_cap_shell(), "knurled_screw_cap", tolerance=0.0008),
        material=cap_black,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_geometry(
            _thread_mesh_geometry(
                radius=0.0346,
                pitch=0.014,
                height=0.038,
                radial_depth=-0.0041,
                axial_half_width=0.0035,
                z0=0.010,
            ),
            "internal_cap_thread",
        ),
        material=thread_wear,
        name="cap_thread",
    )
    cap.visual(
        Box((0.030, 0.006, 0.002)),
        origin=Origin(xyz=(0.021, 0.0, 0.074)),
        material=white_paint,
        name="cap_index_mark",
    )
    cap.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=dark_recess,
        name="seal_shadow",
    )

    model.articulation(
        "body_to_cap_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap_axis,
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=0.0, upper=2.0 * pi),
    )
    model.articulation(
        "cap_thread_lift",
        ArticulationType.PRISMATIC,
        parent=cap_axis,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.04, lower=0.0, upper=0.014),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    cap_turn = object_model.get_articulation("body_to_cap_turn")
    cap_lift = object_model.get_articulation("cap_thread_lift")

    ctx.expect_within(
        body,
        cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_shell",
        margin=0.002,
        name="neck thread is centered inside cap skirt",
    )
    ctx.expect_overlap(
        body,
        cap,
        axes="z",
        elem_a="neck_thread",
        elem_b="cap_thread",
        min_overlap=0.030,
        name="cap and neck threads share practical engagement depth",
    )

    mark0 = ctx.part_element_world_aabb(cap, elem="cap_index_mark")
    cap_rest = ctx.part_world_position(cap)
    with ctx.pose({cap_turn: pi / 2.0}):
        mark90 = ctx.part_element_world_aabb(cap, elem="cap_index_mark")

    if mark0 is None or mark90 is None:
        ctx.fail("cap index mark can be measured", "index mark AABB was not available")
    else:
        cx0 = (mark0[0][0] + mark0[1][0]) / 2.0
        cy0 = (mark0[0][1] + mark0[1][1]) / 2.0
        cx90 = (mark90[0][0] + mark90[1][0]) / 2.0
        cy90 = (mark90[0][1] + mark90[1][1]) / 2.0
        ctx.check(
            "cap rotates about the bottle neck axis",
            cx0 > 0.018 and abs(cy0) < 0.006 and cy90 > 0.018 and abs(cx90) < 0.006,
            details=f"q0 mark center=({cx0:.4f}, {cy0:.4f}), q90 center=({cx90:.4f}, {cy90:.4f})",
        )

    with ctx.pose({cap_turn: 2.0 * pi, cap_lift: 0.014}):
        cap_lifted = ctx.part_world_position(cap)
        ctx.expect_overlap(
            body,
            cap,
            axes="z",
            elem_a="neck_thread",
            elem_b="cap_thread",
            min_overlap=0.020,
            name="raised cap still retains thread engagement",
        )

    ctx.check(
        "loosened cap pose rises along the threaded axis",
        cap_rest is not None and cap_lifted is not None and cap_lifted[2] > cap_rest[2] + 0.012,
        details=f"rest={cap_rest}, lifted={cap_lifted}",
    )

    return ctx.report()


object_model = build_object_model()

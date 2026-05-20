from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


ROOT_X = 0.08
ROOT_Z = 0.42
LINK_0_LENGTH = 0.34
LINK_1_LENGTH = 0.24


def _circle_profile(cx: float, cz: float, radius: float, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cz + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _tapered_capsule_profile(
    x0: float,
    x1: float,
    r0: float,
    r1: float,
    *,
    arc_segments: int = 24,
) -> list[tuple[float, float]]:
    """A link side profile in local X/Z before extrusion through Y."""
    points: list[tuple[float, float]] = []
    points.append((x0, r0))
    points.append((x1, r1))
    for i in range(arc_segments + 1):
        angle = math.pi / 2.0 - math.pi * i / arc_segments
        points.append((x1 + r1 * math.cos(angle), r1 * math.sin(angle)))
    points.append((x0, -r0))
    for i in range(arc_segments + 1):
        angle = -math.pi / 2.0 - math.pi * i / arc_segments
        points.append((x0 + r0 * math.cos(angle), r0 * math.sin(angle)))
    return points


def _rounded_plate_profile(width: float, height: float, radius: float, segments: int = 8) -> list[tuple[float, float]]:
    """Rounded rectangle profile centered on the pin bore."""
    hw = width / 2.0
    hh = height / 2.0
    pts: list[tuple[float, float]] = []
    corners = [
        (hw - radius, hh - radius, 0.0, math.pi / 2.0),
        (-hw + radius, hh - radius, math.pi / 2.0, math.pi),
        (-hw + radius, -hh + radius, math.pi, 3.0 * math.pi / 2.0),
        (hw - radius, -hh + radius, 3.0 * math.pi / 2.0, 2.0 * math.pi),
    ]
    for cx, cz, a0, a1 in corners:
        for i in range(segments + 1):
            a = a0 + (a1 - a0) * i / segments
            pts.append((cx + radius * math.cos(a), cz + radius * math.sin(a)))
    return pts


def _extruded_plate_mesh(
    outer_profile: list[tuple[float, float]],
    holes: list[list[tuple[float, float]]],
    thickness: float,
    name: str,
):
    # The SDK extrudes the profile through local Z.  Rotate so the link thickness
    # is through local Y while the profile remains in the visible X/Z plane.
    geom = ExtrudeWithHolesGeometry(outer_profile, holes, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_revolute_chain")

    blue = model.material("powder_blue", rgba=(0.10, 0.22, 0.34, 1.0))
    link_finish = model.material("anodized_graphite", rgba=(0.23, 0.27, 0.30, 1.0))
    tab_finish = model.material("light_alloy", rgba=(0.62, 0.66, 0.68, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    bronze = model.material("oil_bronze", rgba=(0.74, 0.48, 0.22, 1.0))
    dark = model.material("bore_shadow", rgba=(0.03, 0.035, 0.04, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.28, 0.52, 0.030)),
        origin=Origin(xyz=(-0.045, 0.0, 0.015)),
        material=blue,
        name="base_plate",
    )
    for y, name in ((-0.22, "post_0"), (0.22, "post_1")):
        bridge.visual(
            Box((0.065, 0.055, 0.405)),
            origin=Origin(xyz=(-0.070, y, 0.217)),
            material=blue,
            name=name,
        )
    bridge.visual(
        Box((0.120, 0.52, 0.060)),
        origin=Origin(xyz=(-0.045, 0.0, 0.400)),
        material=blue,
        name="top_bridge",
    )
    bridge.visual(
        Box((0.030, 0.118, 0.075)),
        origin=Origin(xyz=(0.010, 0.0, ROOT_Z)),
        material=blue,
        name="clevis_neck",
    )

    root_cheek_mesh = _extruded_plate_mesh(
        _rounded_plate_profile(0.125, 0.120, 0.016),
        [_circle_profile(0.0, 0.0, 0.018)],
        0.018,
        "root_clevis_cheek",
    )
    bridge.visual(
        root_cheek_mesh,
        origin=Origin(xyz=(ROOT_X, -0.041, ROOT_Z)),
        material=blue,
        name="clevis_cheek_0",
    )
    bridge.visual(
        root_cheek_mesh,
        origin=Origin(xyz=(ROOT_X, 0.041, ROOT_Z)),
        material=blue,
        name="clevis_cheek_1",
    )
    bridge.visual(
        Cylinder(radius=0.014, length=0.130),
        origin=Origin(xyz=(ROOT_X, 0.0, ROOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="root_pin",
    )
    for y, name in ((-0.067, "root_pin_head_0"), (0.067, "root_pin_head_1")):
        bridge.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(ROOT_X, y, ROOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    proximal_link = model.part("proximal_link")
    body_mesh = _extruded_plate_mesh(
        _tapered_capsule_profile(0.0, LINK_0_LENGTH - 0.070, 0.050, 0.032),
        [_circle_profile(0.0, 0.0, 0.018)],
        0.038,
        "proximal_tapered_body",
    )
    proximal_link.visual(
        body_mesh,
        material=link_finish,
        name="tapered_body",
    )
    fork_mesh = _extruded_plate_mesh(
        _tapered_capsule_profile(LINK_0_LENGTH - 0.090, LINK_0_LENGTH, 0.028, 0.036),
        [_circle_profile(LINK_0_LENGTH, 0.0, 0.013)],
        0.016,
        "distal_fork_cheek",
    )
    proximal_link.visual(
        fork_mesh,
        origin=Origin(xyz=(0.0, -0.0265, 0.0)),
        material=link_finish,
        name="fork_cheek_0",
    )
    proximal_link.visual(
        fork_mesh,
        origin=Origin(xyz=(0.0, 0.0265, 0.0)),
        material=link_finish,
        name="fork_cheek_1",
    )
    proximal_link.visual(
        Cylinder(radius=0.010, length=0.091),
        origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_pin",
    )
    for y, name in ((-0.047, "elbow_pin_head_0"), (0.047, "elbow_pin_head_1")):
        proximal_link.visual(
            Cylinder(radius=0.016, length=0.005),
            origin=Origin(xyz=(LINK_0_LENGTH, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )
    root_bushing = TorusGeometry(radius=0.0195, tube=0.0045)
    root_bushing.rotate_x(math.pi / 2.0)
    proximal_link.visual(
        mesh_from_geometry(root_bushing, "root_bushing_ring"),
        material=bronze,
        name="root_bushing_face",
    )

    distal_tab = model.part("distal_tab")
    tab_mesh = _extruded_plate_mesh(
        _tapered_capsule_profile(0.0, LINK_1_LENGTH, 0.032, 0.024),
        [
            _circle_profile(0.0, 0.0, 0.013),
            _circle_profile(LINK_1_LENGTH, 0.0, 0.008),
        ],
        0.026,
        "distal_compact_tab",
    )
    distal_tab.visual(
        tab_mesh,
        material=tab_finish,
        name="compact_tab",
    )
    distal_tab.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="end_tab_hole_liner",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=proximal_link,
        origin=Origin(xyz=(ROOT_X, 0.0, ROOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_tab,
        origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    proximal = object_model.get_part("proximal_link")
    distal = object_model.get_part("distal_tab")
    root = object_model.get_articulation("root_pivot")
    elbow = object_model.get_articulation("elbow_pivot")

    ctx.allow_overlap(
        bridge,
        proximal,
        elem_a="root_pin",
        elem_b="tapered_body",
        reason="The root pin is intentionally represented as passing through the proximal link bore.",
    )
    ctx.allow_overlap(
        distal,
        proximal,
        elem_a="compact_tab",
        elem_b="elbow_pin",
        reason="The elbow pin is intentionally represented as captured through the distal tab bore.",
    )

    revolute_count = sum(
        1 for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check("two revolute joints in series", revolute_count == 2)
    ctx.check("root pivot parent is bridge", root.parent == "bridge" and root.child == "proximal_link")
    ctx.check(
        "elbow pivot follows proximal link",
        elbow.parent == "proximal_link" and elbow.child == "distal_tab",
    )

    ctx.expect_within(
        proximal,
        bridge,
        axes="y",
        inner_elem="tapered_body",
        outer_elem="clevis_cheek_0",
        margin=0.075,
        name="root link sits between clevis cheeks",
    )
    ctx.expect_overlap(
        bridge,
        proximal,
        axes="xz",
        elem_a="root_pin",
        elem_b="tapered_body",
        min_overlap=0.020,
        name="root pin crosses the proximal bore",
    )
    ctx.expect_gap(
        proximal,
        bridge,
        axis="x",
        positive_elem="tapered_body",
        negative_elem="clevis_neck",
        min_gap=0.003,
        name="root link clears bridge neck",
    )
    ctx.expect_within(
        distal,
        proximal,
        axes="y",
        inner_elem="compact_tab",
        outer_elem="fork_cheek_0",
        margin=0.040,
        name="distal tab is captured by fork span",
    )
    ctx.expect_overlap(
        proximal,
        distal,
        axes="xz",
        elem_a="elbow_pin",
        elem_b="compact_tab",
        min_overlap=0.015,
        name="elbow pin crosses the compact tab bore",
    )

    rest_tip = ctx.part_world_position(distal)
    with ctx.pose({root: 0.55, elbow: -0.45}):
        moved_tip = ctx.part_world_position(distal)
        ctx.expect_gap(
            proximal,
            bridge,
            axis="x",
            positive_elem="tapered_body",
            negative_elem="clevis_neck",
            min_gap=0.003,
            name="raised root link still clears bridge neck",
        )
    ctx.check(
        "serial chain changes pose",
        rest_tip is not None
        and moved_tip is not None
        and (abs(moved_tip[0] - rest_tip[0]) > 0.010 or abs(moved_tip[2] - rest_tip[2]) > 0.010),
        details=f"rest={rest_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()

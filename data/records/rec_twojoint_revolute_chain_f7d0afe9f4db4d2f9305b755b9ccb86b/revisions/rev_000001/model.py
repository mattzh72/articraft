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


SHOULDER_Z = 0.100
PRIMARY_LENGTH = 0.220
SECONDARY_LENGTH = 0.360
SECONDARY_Y_OFFSET = 0.032


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 32,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    if clockwise:
        return [
            (
                cx + radius * math.cos(2.0 * math.pi * i / segments),
                cy - radius * math.sin(2.0 * math.pi * i / segments),
            )
            for i in range(segments)
        ]
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(length: float, radius: float, *, segments: int = 18) -> list[tuple[float, float]]:
    """Capsule outline in local X/Y, with pivot centers at x=0 and x=length."""
    pts: list[tuple[float, float]] = [(0.0, -radius), (length, -radius)]
    for i in range(1, segments + 1):
        a = -math.pi / 2.0 + math.pi * i / segments
        pts.append((length + radius * math.cos(a), radius * math.sin(a)))
    pts.append((0.0, radius))
    for i in range(1, segments + 1):
        a = math.pi / 2.0 + math.pi * i / segments
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _cheek_profile(width: float, height: float, top_radius: float, *, segments: int = 12) -> list[tuple[float, float]]:
    """Flat-bottom upright cheek profile with softly rounded top corners."""
    x0 = -width / 2.0
    x1 = width / 2.0
    y0 = -height / 2.0
    y1 = height / 2.0
    r = min(top_radius, width / 2.0, height / 2.0)
    pts: list[tuple[float, float]] = [(x0, y0), (x1, y0), (x1, y1 - r)]
    for i in range(1, segments + 1):
        a = 0.0 + (math.pi / 2.0) * i / segments
        pts.append((x1 - r + r * math.cos(a), y1 - r + r * math.sin(a)))
    pts.append((x0 + r, y1))
    for i in range(1, segments + 1):
        a = math.pi / 2.0 + (math.pi / 2.0) * i / segments
        pts.append((x0 + r + r * math.cos(a), y1 - r + r * math.sin(a)))
    return pts


def _secondary_profile(
    length: float,
    *,
    elbow_radius: float,
    bar_height: float,
    tab_length: float,
    tab_height: float,
) -> list[tuple[float, float]]:
    """Long link outline with a round elbow eye and a square rectangular end tab."""
    tab_start = length - tab_length
    pts: list[tuple[float, float]] = [
        (0.0, -elbow_radius),
        (tab_start, -bar_height / 2.0),
        (tab_start, -tab_height / 2.0),
        (length, -tab_height / 2.0),
        (length, tab_height / 2.0),
        (tab_start, tab_height / 2.0),
        (tab_start, bar_height / 2.0),
        (0.0, elbow_radius),
    ]
    for i in range(1, 25):
        a = math.pi / 2.0 + math.pi * i / 24.0
        pts.append((elbow_radius * math.cos(a), elbow_radius * math.sin(a)))
    return pts


def _extruded_plate_mesh(
    outer: list[tuple[float, float]],
    holes: list[list[tuple[float, float]]],
    thickness: float,
    name: str,
):
    # Author the plate profile in CadQuery so the pin and bolt bores are true
    # through-cuts rather than decorative face marks.  The source plate is
    # extruded along local Z, then rotated so stock thickness lies along Y.
    solid = cq.Workplane("XY").polyline(outer).close().extrude(thickness)
    solid = solid.translate((0.0, 0.0, -thickness / 2.0))
    for hole in holes:
        cx = sum(p[0] for p in hole) / len(hole)
        cy = sum(p[1] for p in hole) / len(hole)
        radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in hole)
        cutter = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(radius)
            .extrude(thickness * 3.0)
            .translate((0.0, 0.0, -1.5 * thickness))
        )
        solid = solid.cut(cutter)
    solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    return mesh_from_cadquery(solid, name, tolerance=0.0005, angular_tolerance=0.05)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_link_folding_linkage")

    painted_steel = Material("dark_powder_coated_steel", color=(0.10, 0.12, 0.13, 1.0))
    cut_steel = Material("brushed_cut_steel", color=(0.70, 0.72, 0.70, 1.0))
    pin_steel = Material("polished_pin_steel", color=(0.82, 0.80, 0.74, 1.0))
    black_bolt = Material("black_oxide_bolts", color=(0.015, 0.014, 0.013, 1.0))

    base_top = 0.018
    clevis_gap = 0.030
    cheek_thickness = 0.012
    cheek_height = 0.118
    cheek_width = 0.084
    cheek_center_z = base_top + cheek_height / 2.0 - 0.002
    cheek_hole_y = SHOULDER_Z - cheek_center_z

    clevis = model.part("clevis")
    clevis.visual(
        Box((0.240, 0.130, base_top)),
        origin=Origin(xyz=(0.020, 0.0, base_top / 2.0)),
        material=painted_steel,
        name="base_plate",
    )
    clevis.visual(
        Box((0.070, 0.086, 0.016)),
        origin=Origin(xyz=(0.004, 0.0, base_top + 0.006)),
        material=painted_steel,
        name="clevis_plinth",
    )

    cheek_mesh = _extruded_plate_mesh(
        _cheek_profile(cheek_width, cheek_height, 0.012),
        [_circle_profile(0.0, cheek_hole_y, 0.0105, segments=40)],
        cheek_thickness,
        "clevis_cheek",
    )
    cheek_y = clevis_gap / 2.0 + cheek_thickness / 2.0
    clevis.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, cheek_y, cheek_center_z)),
        material=painted_steel,
        name="cheek_0",
    )
    clevis.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -cheek_y, cheek_center_z)),
        material=painted_steel,
        name="cheek_1",
    )
    clevis.visual(
        Box((0.018, 0.082, 0.055)),
        origin=Origin(xyz=(-0.044, 0.0, base_top + 0.029)),
        material=painted_steel,
        name="rear_bridge",
    )
    clevis.visual(
        Cylinder(radius=0.0116, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="shoulder_pin",
    )
    for y, cap_name in ((0.0375, "shoulder_cap_0"), (-0.0375, "shoulder_cap_1")):
        clevis.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(0.0, y, SHOULDER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black_bolt,
            name=cap_name,
        )
    for x, y, bolt_name in (
        (-0.065, 0.041, "bolt_0"),
        (0.105, 0.041, "bolt_1"),
        (-0.065, -0.041, "bolt_2"),
        (0.105, -0.041, "bolt_3"),
    ):
        clevis.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, y, base_top + 0.002)),
            material=black_bolt,
            name=bolt_name,
        )

    primary = model.part("primary_link")
    primary.visual(
        _extruded_plate_mesh(
            _capsule_profile(PRIMARY_LENGTH, 0.025),
            [
                _circle_profile(0.0, 0.0, 0.0115, segments=44),
                _circle_profile(PRIMARY_LENGTH, 0.0, 0.0115, segments=44),
            ],
            0.020,
            "primary_link_plate",
        ),
        material=cut_steel,
        name="primary_body",
    )
    primary.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(PRIMARY_LENGTH, -0.0115, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_bolt,
        name="elbow_washer_0",
    )

    secondary = model.part("secondary_link")
    secondary.visual(
        _extruded_plate_mesh(
            _secondary_profile(
                SECONDARY_LENGTH,
                elbow_radius=0.025,
                bar_height=0.041,
                tab_length=0.090,
                tab_height=0.072,
            ),
            [
                _circle_profile(0.0, 0.0, 0.0115, segments=44),
                _circle_profile(SECONDARY_LENGTH - 0.060, 0.0, 0.0065, segments=28),
                _circle_profile(SECONDARY_LENGTH - 0.028, 0.0, 0.0065, segments=28),
            ],
            0.020,
            "secondary_link_plate",
        ),
        origin=Origin(xyz=(0.0, SECONDARY_Y_OFFSET, 0.0)),
        material=cut_steel,
        name="secondary_body",
    )
    secondary.visual(
        Cylinder(radius=0.0116, length=0.062),
        origin=Origin(
            xyz=(0.0, SECONDARY_Y_OFFSET / 2.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=pin_steel,
        name="elbow_pin",
    )
    secondary.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.0175, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_bolt,
        name="elbow_spacer",
    )
    secondary.visual(
        Box((0.055, 0.0035, 0.076)),
        origin=Origin(xyz=(SECONDARY_LENGTH - 0.045, SECONDARY_Y_OFFSET + 0.0110, 0.0)),
        material=Material("tab_edge_highlight", color=(0.52, 0.54, 0.53, 1.0)),
        name="tab_edge",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=clevis,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-1.25, upper=0.55),
    )

    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0, lower=-0.12, upper=2.72),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clevis = object_model.get_part("clevis")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        clevis,
        primary,
        elem_a="shoulder_pin",
        elem_b="primary_body",
        reason="The shoulder pin is intentionally captured through the primary link's pivot eye.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="primary_body",
        elem_b="elbow_pin",
        reason="The elbow pin is intentionally captured through the primary link's distal pivot eye.",
    )
    ctx.expect_within(
        clevis,
        primary,
        axes="xz",
        margin=0.001,
        inner_elem="shoulder_pin",
        outer_elem="primary_body",
        name="shoulder pin is centered inside primary pivot eye",
    )
    ctx.expect_overlap(
        clevis,
        primary,
        axes="y",
        min_overlap=0.018,
        elem_a="shoulder_pin",
        elem_b="primary_body",
        name="shoulder pin spans the primary link thickness",
    )
    ctx.expect_within(
        secondary,
        primary,
        axes="xz",
        margin=0.001,
        inner_elem="elbow_pin",
        outer_elem="primary_body",
        name="elbow pin is centered inside primary distal eye",
    )
    ctx.expect_overlap(
        secondary,
        primary,
        axes="y",
        min_overlap=0.018,
        elem_a="elbow_pin",
        elem_b="primary_body",
        name="elbow pin spans the primary distal eye thickness",
    )

    ctx.expect_gap(
        clevis,
        primary,
        axis="y",
        min_gap=0.003,
        max_gap=0.009,
        positive_elem="cheek_0",
        negative_elem="primary_body",
        name="primary link clears positive clevis cheek",
    )
    ctx.expect_gap(
        primary,
        clevis,
        axis="y",
        min_gap=0.003,
        max_gap=0.009,
        positive_elem="primary_body",
        negative_elem="cheek_1",
        name="primary link clears negative clevis cheek",
    )
    ctx.expect_gap(
        secondary,
        primary,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="secondary_body",
        negative_elem="primary_body",
        name="elbow plates are side-by-side with spacer clearance",
    )

    shoulder_axis = tuple(round(v, 6) for v in shoulder.axis)
    elbow_axis = tuple(round(v, 6) for v in elbow.axis)
    ctx.check(
        "shoulder and elbow axes are parallel",
        shoulder_axis == elbow_axis == (0.0, 1.0, 0.0),
        details=f"shoulder={shoulder_axis}, elbow={elbow_axis}",
    )
    ctx.check(
        "revolute limits cover folded and near-straight poses",
        shoulder.motion_limits.lower <= -1.0
        and shoulder.motion_limits.upper >= 0.5
        and elbow.motion_limits.lower <= 0.0
        and elbow.motion_limits.upper >= 2.6,
        details=f"shoulder={shoulder.motion_limits}, elbow={elbow.motion_limits}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        sec_aabb = ctx.part_element_world_aabb(secondary, elem="secondary_body")
        ctx.check(
            "straight pose reaches forward",
            sec_aabb is not None and sec_aabb[1][0] > PRIMARY_LENGTH + SECONDARY_LENGTH - 0.035,
            details=f"secondary_aabb={sec_aabb}",
        )

    with ctx.pose({shoulder: 0.0, elbow: 2.65}):
        sec_aabb = ctx.part_element_world_aabb(secondary, elem="secondary_body")
        ctx.check(
            "folded pose brings tab back near clevis base",
            sec_aabb is not None and sec_aabb[0][0] < -0.055 and sec_aabb[0][2] < 0.030,
            details=f"secondary_aabb={sec_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def _extruded_yz_polygon(points: list[tuple[float, float]], x0: float, x1: float) -> MeshGeometry:
    """Extrude a simple CCW polygon in the YZ plane between two X planes."""
    geom = MeshGeometry()
    n = len(points)
    left = [geom.add_vertex(x0, y, z) for y, z in points]
    right = [geom.add_vertex(x1, y, z) for y, z in points]

    for i in range(1, n - 1):
        geom.add_face(left[0], left[i], left[i + 1])
        geom.add_face(right[0], right[i + 1], right[i])

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(left[i], right[i], right[j])
        geom.add_face(left[i], right[j], left[j])
    return geom


def _annular_plate_x(
    *,
    x_center: float,
    thickness: float,
    inner_radius: float,
    root_radius: float,
    tip_radius: float,
    teeth: int,
) -> MeshGeometry:
    """A thin chainring-like annular plate in the YZ plane, with alternating tooth tips."""
    geom = MeshGeometry()
    count = teeth * 2
    x_back = x_center - thickness / 2.0
    x_front = x_center + thickness / 2.0
    outer_back: list[int] = []
    inner_back: list[int] = []
    outer_front: list[int] = []
    inner_front: list[int] = []

    for i in range(count):
        a = 2.0 * math.pi * i / count
        # Tooth tips are a little swept forward so the ring reads like a real chainring.
        radius = tip_radius if i % 2 == 0 else root_radius
        y = radius * math.cos(a)
        z = radius * math.sin(a)
        yi = inner_radius * math.cos(a)
        zi = inner_radius * math.sin(a)
        outer_back.append(geom.add_vertex(x_back, y, z))
        inner_back.append(geom.add_vertex(x_back, yi, zi))
        outer_front.append(geom.add_vertex(x_front, y, z))
        inner_front.append(geom.add_vertex(x_front, yi, zi))

    for i in range(count):
        j = (i + 1) % count
        # Back and front annular faces.
        geom.add_face(outer_back[i], outer_back[j], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], inner_back[i])
        geom.add_face(outer_front[i], inner_front[j], outer_front[j])
        geom.add_face(outer_front[i], inner_front[i], inner_front[j])
        # Tooth outer wall and inner bore wall.
        geom.add_face(outer_back[i], outer_front[i], outer_front[j])
        geom.add_face(outer_back[i], outer_front[j], outer_back[j])
        geom.add_face(inner_back[i], inner_back[j], inner_front[j])
        geom.add_face(inner_back[i], inner_front[j], inner_front[i])
    return geom


def _stepped_tube_x(
    sections: list[tuple[float, float, float]],
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Connected stepped tube mesh along X. Each section is (x, inner_r, outer_r)."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, inner_r, outer_r in sections:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ca = math.cos(a)
            sa = math.sin(a)
            outer_ring.append(geom.add_vertex(x, outer_r * ca, outer_r * sa))
            inner_ring.append(geom.add_vertex(x, inner_r * ca, inner_r * sa))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for s in range(len(sections) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(outer[s][i], outer[s + 1][i], outer[s + 1][j])
            geom.add_face(outer[s][i], outer[s + 1][j], outer[s][j])
            geom.add_face(inner[s][i], inner[s][j], inner[s + 1][j])
            geom.add_face(inner[s][i], inner[s + 1][j], inner[s + 1][i])

    # End caps connect outer and inner walls while leaving the bore open.
    for ring_index in (0, len(sections) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            if ring_index == 0:
                geom.add_face(outer[ring_index][i], inner[ring_index][j], inner[ring_index][i])
                geom.add_face(outer[ring_index][i], outer[ring_index][j], inner[ring_index][j])
            else:
                geom.add_face(outer[ring_index][i], inner[ring_index][i], inner[ring_index][j])
                geom.add_face(outer[ring_index][i], inner[ring_index][j], outer[ring_index][j])
    return geom


def _spider_plate_x(
    *,
    x_center: float,
    thickness: float,
    arms: int = 4,
    valley_radius: float = 0.035,
    peak_radius: float = 0.081,
    samples: int = 160,
    phase: float = math.radians(45.0),
) -> MeshGeometry:
    points: list[tuple[float, float]] = []
    for i in range(samples):
        a = 2.0 * math.pi * i / samples
        lobe = max(0.0, math.cos(arms * (a - phase)))
        radius = valley_radius + (peak_radius - valley_radius) * (lobe**3.0)
        points.append((radius * math.cos(a), radius * math.sin(a)))
    return _extruded_yz_polygon(points, x_center - thickness / 2.0, x_center + thickness / 2.0)


def _crank_arm_plate_x(
    *,
    x_center: float,
    thickness: float,
    length: float,
    direction: float,
    root_width: float = 0.038,
    tip_width: float = 0.027,
) -> MeshGeometry:
    """Tapered rounded-ended arm extending along +/-Z in the YZ plane."""
    root_z = direction * 0.026
    tip_z = direction * length
    pts: list[tuple[float, float]] = []

    # One side of the tapered blade.
    pts.append((-root_width / 2.0, root_z))
    pts.append((-tip_width / 2.0, tip_z - direction * tip_width / 2.0))

    # Rounded pedal end.
    center_z = tip_z
    if direction < 0.0:
        angles = [math.pi + t * math.pi / 10.0 for t in range(11)]
    else:
        angles = [0.0 + t * math.pi / 10.0 for t in range(11)]
    for a in angles:
        pts.append((math.cos(a) * tip_width / 2.0, center_z + direction * math.sin(a) * tip_width / 2.0))

    pts.append((tip_width / 2.0, tip_z - direction * tip_width / 2.0))
    pts.append((root_width / 2.0, root_z))
    # Broad scalloped root so the blade visibly grows into the spindle boss.
    pts.append((root_width * 0.45, direction * 0.005))
    pts.append((0.0, direction * 0.000))
    pts.append((-root_width * 0.45, direction * 0.005))
    return _extruded_yz_polygon(pts, x_center - thickness / 2.0, x_center + thickness / 2.0)


def _bolt_positions(radius: float, count: int = 4, phase: float = math.radians(45.0)) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(phase + 2.0 * math.pi * i / count), radius * math.sin(phase + 2.0 * math.pi * i / count))
        for i in range(count)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_crankset")

    alloy = model.material("brushed_alloy", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_alloy = model.material("black_anodized_alloy", rgba=(0.015, 0.017, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.025, 1.0))
    steel = model.material("polished_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    bearing = model.material("dark_bearing_rubber", rgba=(0.005, 0.005, 0.006, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_geometry(
            _stepped_tube_x(
                [
                    (-0.052, 0.0140, 0.035),
                    (-0.044, 0.0140, 0.030),
                    (0.044, 0.0140, 0.030),
                    (0.052, 0.0140, 0.035),
                ]
            ),
            "hollow_bottom_bracket_shell",
        ),
        material=dark_alloy,
        name="hollow_shell",
    )
    bottom_bracket.visual(
        mesh_from_geometry(
            _stepped_tube_x(
                [
                    (-0.053, 0.0118, 0.026),
                    (-0.049, 0.0118, 0.026),
                    (0.049, 0.0118, 0.026),
                    (0.053, 0.0118, 0.026),
                ],
                segments=64,
            ),
            "bearing_seal_lips",
        ),
        material=bearing,
        name="bearing_seals",
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.0110, length=0.182),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.033, length=0.056),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="drive_hub",
    )
    crankset.visual(
        Cylinder(radius=0.032, length=0.056),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="offside_hub",
    )
    crankset.visual(
        mesh_from_geometry(_spider_plate_x(x_center=0.070, thickness=0.018), "four_arm_spider"),
        material=alloy,
        name="spider",
    )
    crankset.visual(
        mesh_from_geometry(
            _annular_plate_x(
                x_center=0.080,
                thickness=0.004,
                inner_radius=0.071,
                root_radius=0.091,
                tip_radius=0.097,
                teeth=36,
            ),
            "outer_chainring_36t",
        ),
        material=dark_alloy,
        name="outer_chainring",
    )
    crankset.visual(
        mesh_from_geometry(
            _annular_plate_x(
                x_center=0.060,
                thickness=0.004,
                inner_radius=0.050,
                root_radius=0.068,
                tip_radius=0.074,
                teeth=24,
            ),
            "inner_chainring_24t",
        ),
        material=dark_alloy,
        name="inner_chainring",
    )
    crankset.visual(
        mesh_from_geometry(
            _crank_arm_plate_x(x_center=0.099, thickness=0.020, length=0.175, direction=-1.0),
            "wide_drive_crank_arm",
        ),
        material=alloy,
        name="drive_arm",
    )
    crankset.visual(
        mesh_from_geometry(
            _crank_arm_plate_x(x_center=-0.099, thickness=0.020, length=0.175, direction=1.0),
            "wide_offside_crank_arm",
        ),
        material=alloy,
        name="offside_arm",
    )
    crankset.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.094, 0.0, -0.175), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="drive_pedal_boss",
    )
    crankset.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(-0.094, 0.0, 0.175), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="offside_pedal_boss",
    )
    for i, (y, z) in enumerate(_bolt_positions(0.073)):
        crankset.visual(
            Cylinder(radius=0.0048, length=0.026),
            origin=Origin(xyz=(0.070, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"chainring_bolt_{i}",
        )

    def add_pedal(part_name: str, direction: float, material: Material) -> None:
        pedal = model.part(part_name)
        pedal.visual(
            Cylinder(radius=0.0048, length=0.066),
            origin=Origin(xyz=(direction * 0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="stub_axle",
        )
        pedal.visual(
            Box((0.058, 0.090, 0.024)),
            origin=Origin(xyz=(direction * 0.092, 0.0, 0.0)),
            material=material,
            name="clipless_body",
        )
        pedal.visual(
            Box((0.044, 0.010, 0.010)),
            origin=Origin(xyz=(direction * 0.092, -0.024, 0.017)),
            material=steel,
            name="front_cleat_jaw",
        )
        pedal.visual(
            Box((0.044, 0.010, 0.010)),
            origin=Origin(xyz=(direction * 0.092, 0.024, 0.017)),
            material=steel,
            name="rear_cleat_jaw",
        )
        pedal.visual(
            Box((0.040, 0.050, 0.006)),
            origin=Origin(xyz=(direction * 0.092, 0.0, -0.015)),
            material=steel,
            name="lower_retention_plate",
        )
        return None

    add_pedal("drive_pedal", 1.0, satin_black)
    add_pedal("offside_pedal", -1.0, satin_black)

    model.articulation(
        "bottom_bracket_to_crankset",
        ArticulationType.REVOLUTE,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=14.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "drive_pedal_axle",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child="drive_pedal",
        origin=Origin(xyz=(0.114, 0.0, -0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "offside_pedal_axle",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child="offside_pedal",
        origin=Origin(xyz=(-0.114, 0.0, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crankset = object_model.get_part("crankset")
    bottom_bracket = object_model.get_part("bottom_bracket")
    drive_pedal = object_model.get_part("drive_pedal")
    offside_pedal = object_model.get_part("offside_pedal")
    crank_joint = object_model.get_articulation("bottom_bracket_to_crankset")
    drive_pedal_joint = object_model.get_articulation("drive_pedal_axle")

    visual_names = {visual.name for visual in crankset.visuals}
    ctx.check(
        "double chainring spider is explicit",
        {"outer_chainring", "inner_chainring", "spider"}.issubset(visual_names),
        details=f"crankset visuals={sorted(visual_names)}",
    )
    ctx.expect_overlap(
        crankset,
        bottom_bracket,
        axes="x",
        elem_a="spindle",
        elem_b="hollow_shell",
        min_overlap=0.085,
        name="spindle passes through bottom bracket shell",
    )
    ctx.expect_within(
        crankset,
        bottom_bracket,
        axes="yz",
        inner_elem="spindle",
        outer_elem="hollow_shell",
        margin=0.0,
        name="spindle is centered inside bottom bracket bore",
    )
    ctx.expect_contact(
        drive_pedal,
        crankset,
        elem_a="stub_axle",
        elem_b="drive_pedal_boss",
        contact_tol=0.0015,
        name="drive pedal stub axle seats on crank boss",
    )
    ctx.expect_contact(
        offside_pedal,
        crankset,
        elem_a="stub_axle",
        elem_b="offside_pedal_boss",
        contact_tol=0.0015,
        name="offside pedal stub axle seats on crank boss",
    )

    rest_drive = ctx.part_world_position(drive_pedal)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        quarter_drive = ctx.part_world_position(drive_pedal)
    ctx.check(
        "bottom bracket revolute joint swings the crank arm",
        rest_drive is not None
        and quarter_drive is not None
        and abs(quarter_drive[1] - rest_drive[1]) > 0.12
        and abs(quarter_drive[2] - rest_drive[2]) > 0.12,
        details=f"rest={rest_drive}, quarter_turn={quarter_drive}",
    )

    with ctx.pose({drive_pedal_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(drive_pedal)
    with ctx.pose({drive_pedal_joint: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(drive_pedal)
    rest_span_y = None if rest_aabb is None else rest_aabb[1][1] - rest_aabb[0][1]
    rotated_span_y = None if rotated_aabb is None else rotated_aabb[1][1] - rotated_aabb[0][1]
    ctx.check(
        "clipless pedal body rotates around stub axle",
        rest_span_y is not None and rotated_span_y is not None and rest_span_y - rotated_span_y > 0.04,
        details=f"rest_span_y={rest_span_y}, rotated_span_y={rotated_span_y}",
    )

    return ctx.report()


object_model = build_object_model()

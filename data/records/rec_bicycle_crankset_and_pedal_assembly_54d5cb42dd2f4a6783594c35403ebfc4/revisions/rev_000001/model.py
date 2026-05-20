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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube_x(
    *,
    length: float,
    inner_radius: float,
    outer_radius: float,
    center_x: float = 0.0,
    segments: int = 72,
) -> MeshGeometry:
    """Watertight hollow cylinder aligned on the local X axis."""
    geom = MeshGeometry()
    x0 = center_x - length / 2.0
    x1 = center_x + length / 2.0
    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        y = math.cos(theta)
        z = math.sin(theta)
        outer0.append(geom.add_vertex(x0, outer_radius * y, outer_radius * z))
        outer1.append(geom.add_vertex(x1, outer_radius * y, outer_radius * z))
        inner0.append(geom.add_vertex(x0, inner_radius * y, inner_radius * z))
        inner1.append(geom.add_vertex(x1, inner_radius * y, inner_radius * z))

    for i in range(segments):
        j = (i + 1) % segments
        # outer wall
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # inner wall
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
        # end caps
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
    return geom


def _stepped_pressfit_shell() -> MeshGeometry:
    """Press-fit bottom bracket shell with larger bearing lips at each end."""
    stations = [
        (-0.060, 0.032),
        (-0.047, 0.032),
        (-0.047, 0.026),
        (0.047, 0.026),
        (0.047, 0.032),
        (0.060, 0.032),
    ]
    inner_radius = 0.0145
    segments = 72
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for x, outer_radius in stations:
        outer_loop: list[int] = []
        inner_loop: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            y = math.cos(theta)
            z = math.sin(theta)
            outer_loop.append(geom.add_vertex(x, outer_radius * y, outer_radius * z))
            inner_loop.append(geom.add_vertex(x, inner_radius * y, inner_radius * z))
        outer.append(outer_loop)
        inner.append(inner_loop)

    for s in range(len(stations) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(outer[s][i], outer[s][j], outer[s + 1][j])
            geom.add_face(outer[s][i], outer[s + 1][j], outer[s + 1][i])
            geom.add_face(inner[s][j], inner[s][i], inner[s + 1][i])
            geom.add_face(inner[s][j], inner[s + 1][i], inner[s + 1][j])

    # Open tube end faces.
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer[0][j], outer[0][i], inner[0][i])
        geom.add_face(outer[0][j], inner[0][i], inner[0][j])
        geom.add_face(outer[-1][i], outer[-1][j], inner[-1][j])
        geom.add_face(outer[-1][i], inner[-1][j], inner[-1][i])
    return geom


def _chainring_mesh(
    *,
    center_x: float,
    thickness: float,
    inner_radius: float = 0.124,
    root_radius: float = 0.156,
    tooth_tip_radius: float = 0.168,
    teeth: int = 48,
) -> MeshGeometry:
    """Thin track chainring: hollow annulus with repeated tooth tips."""
    segments = teeth * 4
    geom = MeshGeometry()
    x0 = center_x - thickness / 2.0
    x1 = center_x + thickness / 2.0
    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        phase = (i % 4) / 4.0
        tooth_shape = 1.0 if phase in (0.25, 0.50) else 0.20
        radius = root_radius + (tooth_tip_radius - root_radius) * tooth_shape
        y = math.cos(theta)
        z = math.sin(theta)
        outer0.append(geom.add_vertex(x0, radius * y, radius * z))
        outer1.append(geom.add_vertex(x1, radius * y, radius * z))
        inner0.append(geom.add_vertex(x0, inner_radius * y, inner_radius * z))
        inner1.append(geom.add_vertex(x1, inner_radius * y, inner_radius * z))

    for i in range(segments):
        j = (i + 1) % segments
        # faces and side walls
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
    return geom


def _extrude_polygon_x(
    profile_yz: list[tuple[float, float]],
    *,
    center_x: float,
    thickness: float,
) -> MeshGeometry:
    """Extrude a convex YZ profile into a thin solid along X."""
    geom = MeshGeometry()
    x0 = center_x - thickness / 2.0
    x1 = center_x + thickness / 2.0
    back: list[int] = []
    front: list[int] = []
    for y, z in profile_yz:
        back.append(geom.add_vertex(x0, y, z))
        front.append(geom.add_vertex(x1, y, z))

    cy = sum(p[0] for p in profile_yz) / len(profile_yz)
    cz = sum(p[1] for p in profile_yz) / len(profile_yz)
    c0 = geom.add_vertex(x0, cy, cz)
    c1 = geom.add_vertex(x1, cy, cz)
    n = len(profile_yz)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back[i], back[j], front[j])
        geom.add_face(back[i], front[j], front[i])
        geom.add_face(c1, front[i], front[j])
        geom.add_face(c0, back[j], back[i])
    return geom


def _crank_arm_profile(*, direction: int, length: float = 0.170) -> list[tuple[float, float]]:
    """Rounded, tapered forged arm outline in local YZ."""
    root_radius = 0.022
    end_radius = 0.0155
    root_half = 0.019
    end_half = 0.013
    side_samples = 12
    cap_samples = 10
    pts: list[tuple[float, float]] = []

    for i in range(side_samples + 1):
        t = i / side_samples
        v = length * t
        half = root_half + (end_half - root_half) * t
        pts.append((half, direction * v))
    for j in range(1, cap_samples + 1):
        theta = math.pi * j / cap_samples
        pts.append((end_radius * math.cos(theta), direction * (length + end_radius * math.sin(theta))))
    for i in range(side_samples - 1, -1, -1):
        t = i / side_samples
        v = length * t
        half = root_half + (end_half - root_half) * t
        pts.append((-half, direction * v))
    for j in range(1, cap_samples):
        theta = math.pi + math.pi * j / cap_samples
        pts.append((root_radius * math.cos(theta), direction * (root_radius * math.sin(theta))))
    return pts


def _spoke_profile(
    *, angle: float, inner_radius: float, outer_radius: float, width: float
) -> list[tuple[float, float]]:
    """Rounded radial spider spoke profile in YZ."""
    samples = 10
    cap_samples = 8
    radial = (math.cos(angle), math.sin(angle))
    tangent = (-math.sin(angle), math.cos(angle))
    half = width / 2.0
    pts: list[tuple[float, float]] = []

    # One long rounded slot-like bar from inner_radius to outer_radius.
    for i in range(samples + 1):
        t = i / samples
        r = inner_radius + (outer_radius - inner_radius) * t
        pts.append((r * radial[0] + half * tangent[0], r * radial[1] + half * tangent[1]))
    for j in range(1, cap_samples + 1):
        theta = angle + math.pi / 2.0 - math.pi * j / cap_samples
        pts.append((outer_radius * radial[0] + half * math.cos(theta), outer_radius * radial[1] + half * math.sin(theta)))
    for i in range(samples - 1, -1, -1):
        t = i / samples
        r = inner_radius + (outer_radius - inner_radius) * t
        pts.append((r * radial[0] - half * tangent[0], r * radial[1] - half * tangent[1]))
    for j in range(1, cap_samples):
        theta = angle - math.pi / 2.0 - math.pi * j / cap_samples
        pts.append((inner_radius * radial[0] + half * math.cos(theta), inner_radius * radial[1] + half * math.sin(theta)))
    return pts


def _add_pedal_visuals(part, *, side: int, metal: Material, rubber: Material) -> None:
    """Add a flat cage pedal that extends from the crank along +/-X."""
    sign = 1 if side >= 0 else -1
    # Threaded spindle and bearing barrel along the revolute axis.
    part.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(sign * 0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(sign * 0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="center_barrel",
    )

    center_x = sign * 0.073
    inner_x = sign * 0.034
    outer_x = sign * 0.112
    # Open rectangular cage: rails overlap at their mitred corners and the
    # central bridge ties the bearing barrel to the perimeter.
    part.visual(
        Box((0.090, 0.010, 0.012)),
        origin=Origin(xyz=(center_x, 0.050, 0.0)),
        material=metal,
        name="front_rail",
    )
    part.visual(
        Box((0.090, 0.010, 0.012)),
        origin=Origin(xyz=(center_x, -0.050, 0.0)),
        material=metal,
        name="rear_rail",
    )
    part.visual(
        Box((0.010, 0.108, 0.012)),
        origin=Origin(xyz=(outer_x, 0.0, 0.0)),
        material=metal,
        name="outer_side_rail",
    )
    part.visual(
        Box((0.010, 0.108, 0.012)),
        origin=Origin(xyz=(inner_x, 0.0, 0.0)),
        material=metal,
        name="inner_side_rail",
    )
    part.visual(
        Box((0.082, 0.008, 0.010)),
        origin=Origin(xyz=(center_x, 0.0, 0.0)),
        material=metal,
        name="center_bridge",
    )
    part.visual(
        Box((0.082, 0.006, 0.006)),
        origin=Origin(xyz=(center_x, 0.025, 0.0)),
        material=metal,
        name="front_bridge",
    )
    part.visual(
        Box((0.082, 0.006, 0.006)),
        origin=Origin(xyz=(center_x, -0.025, 0.0)),
        material=metal,
        name="rear_bridge",
    )

    # Low rubber grip pads and small cage teeth sit on the upper and lower edges.
    for yi, y in enumerate((-0.050, 0.050)):
        part.visual(
            Box((0.072, 0.006, 0.004)),
            origin=Origin(xyz=(center_x, y, 0.008)),
            material=rubber,
            name=f"grip_pad_{yi}",
        )
        for tx, x_offset in enumerate((-0.030, -0.010, 0.010, 0.030)):
            part.visual(
                Box((0.006, 0.006, 0.007)),
                origin=Origin(xyz=(center_x + sign * x_offset, y, 0.013)),
                material=metal,
                name=f"tooth_{yi}_{tx}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_speed_track_crankset")

    polished = model.material("polished_aluminum", rgba=(0.78, 0.76, 0.70, 1.0))
    dark = model.material("anodized_black", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    bearing = model.material("bearing_steel", rgba=(0.55, 0.58, 0.58, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_geometry(_stepped_pressfit_shell(), "pressfit_shell"),
        material=dark,
        name="pressfit_shell",
    )
    # Thin exposed bearing races inside the press-fit cups.
    bottom_bracket.visual(
        mesh_from_geometry(
            _annular_tube_x(
                length=0.010,
                inner_radius=0.0115,
                outer_radius=0.023,
                center_x=0.054,
                segments=64,
            ),
            "drive_bearing_race",
        ),
        material=bearing,
        name="drive_bearing_race",
    )
    bottom_bracket.visual(
        mesh_from_geometry(
            _annular_tube_x(
                length=0.010,
                inner_radius=0.0115,
                outer_radius=0.023,
                center_x=-0.054,
                segments=64,
            ),
            "opposite_bearing_race",
        ),
        material=bearing,
        name="opposite_bearing_race",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0115, length=0.168),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="drive_boss",
    )
    crank.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(-0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="opposite_boss",
    )
    crank.visual(
        mesh_from_geometry(
            _chainring_mesh(center_x=0.061, thickness=0.006),
            "large_toothed_chainring",
        ),
        material=dark,
        name="large_chainring",
    )
    for index in range(5):
        angle = -math.pi / 2.0 + 2.0 * math.pi * index / 5.0
        crank.visual(
            mesh_from_geometry(
                _extrude_polygon_x(
                    _spoke_profile(
                        angle=angle,
                        inner_radius=0.026,
                        outer_radius=0.132,
                        width=0.018,
                    ),
                    center_x=0.066,
                    thickness=0.006,
                ),
                f"spider_arm_{index}",
            ),
            material=polished,
            name=f"spider_arm_{index}",
        )
        bolt_y = 0.122 * math.cos(angle)
        bolt_z = 0.122 * math.sin(angle)
        crank.visual(
            Cylinder(radius=0.0055, length=0.005),
            origin=Origin(xyz=(0.068, bolt_y, bolt_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing,
            name=f"chainring_bolt_{index}",
        )

    crank.visual(
        mesh_from_geometry(
            _extrude_polygon_x(_crank_arm_profile(direction=-1), center_x=0.080, thickness=0.014),
            "drive_forged_arm",
        ),
        material=polished,
        name="drive_arm",
    )
    crank.visual(
        mesh_from_geometry(
            _extrude_polygon_x(_crank_arm_profile(direction=1), center_x=-0.080, thickness=0.014),
            "opposite_forged_arm",
        ),
        material=polished,
        name="opposite_arm",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.080, 0.0, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="drive_pedal_eye",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.080, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="opposite_pedal_eye",
    )

    drive_pedal = model.part("drive_pedal")
    _add_pedal_visuals(drive_pedal, side=1, metal=polished, rubber=rubber)
    opposite_pedal = model.part("opposite_pedal")
    _add_pedal_visuals(opposite_pedal, side=-1, metal=polished, rubber=rubber)

    model.articulation(
        "bottom_bracket_to_crank",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crank,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "crank_to_drive_pedal",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=drive_pedal,
        origin=Origin(xyz=(0.089, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.003),
    )
    model.articulation(
        "crank_to_opposite_pedal",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=opposite_pedal,
        origin=Origin(xyz=(-0.089, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.003),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crank = object_model.get_part("crank")
    drive_pedal = object_model.get_part("drive_pedal")
    opposite_pedal = object_model.get_part("opposite_pedal")
    crank_joint = object_model.get_articulation("bottom_bracket_to_crank")
    drive_pedal_joint = object_model.get_articulation("crank_to_drive_pedal")

    ctx.allow_overlap(
        bottom_bracket,
        crank,
        elem_a="drive_bearing_race",
        elem_b="spindle",
        reason=(
            "The crank spindle is intentionally captured inside the press-fit "
            "bearing race; the local mesh proxy overlap represents bearing contact."
        ),
    )
    ctx.allow_overlap(
        bottom_bracket,
        crank,
        elem_a="opposite_bearing_race",
        elem_b="spindle",
        reason=(
            "The crank spindle is intentionally captured inside the opposite "
            "bearing race; the local mesh proxy overlap represents bearing contact."
        ),
    )

    chainring_aabb = ctx.part_element_world_aabb(crank, elem="large_chainring")
    if chainring_aabb is not None:
        chainring_min, chainring_max = chainring_aabb
        chainring_diameter_y = chainring_max[1] - chainring_min[1]
        chainring_diameter_z = chainring_max[2] - chainring_min[2]
        chainring_thickness = chainring_max[0] - chainring_min[0]
        ctx.check(
            "large thin chainring proportions",
            chainring_diameter_y > 0.32
            and chainring_diameter_z > 0.32
            and chainring_thickness < 0.012,
            details=(
                f"dy={chainring_diameter_y:.3f}, "
                f"dz={chainring_diameter_z:.3f}, "
                f"dx={chainring_thickness:.3f}"
            ),
        )
    else:
        ctx.fail("large thin chainring proportions", "large_chainring AABB unavailable")

    ctx.expect_within(
        crank,
        bottom_bracket,
        axes="yz",
        inner_elem="spindle",
        outer_elem="pressfit_shell",
        margin=0.0,
        name="spindle centered through bottom bracket bore",
    )
    ctx.expect_overlap(
        crank,
        bottom_bracket,
        axes="x",
        elem_a="spindle",
        elem_b="pressfit_shell",
        min_overlap=0.11,
        name="spindle retained through pressfit shell",
    )
    ctx.expect_overlap(
        crank,
        bottom_bracket,
        axes="x",
        elem_a="spindle",
        elem_b="drive_bearing_race",
        min_overlap=0.008,
        name="drive bearing captures spindle length",
    )
    ctx.expect_overlap(
        crank,
        bottom_bracket,
        axes="x",
        elem_a="spindle",
        elem_b="opposite_bearing_race",
        min_overlap=0.008,
        name="opposite bearing captures spindle length",
    )
    ctx.expect_gap(
        crank,
        bottom_bracket,
        axis="x",
        positive_elem="drive_boss",
        negative_elem="pressfit_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="drive crank boss clears bearing cup",
    )
    ctx.expect_gap(
        bottom_bracket,
        crank,
        axis="x",
        positive_elem="pressfit_shell",
        negative_elem="opposite_boss",
        min_gap=0.001,
        max_gap=0.006,
        name="opposite crank boss clears bearing cup",
    )
    ctx.expect_gap(
        drive_pedal,
        crank,
        axis="x",
        positive_elem="axle",
        negative_elem="drive_pedal_eye",
        max_gap=0.001,
        max_penetration=0.0,
        name="drive pedal axle seats on crank eye",
    )
    ctx.expect_gap(
        crank,
        opposite_pedal,
        axis="x",
        positive_elem="opposite_pedal_eye",
        negative_elem="axle",
        max_gap=0.001,
        max_penetration=0.0,
        name="opposite pedal axle seats on crank eye",
    )

    rest_drive_pos = ctx.part_world_position(drive_pedal)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        turned_drive_pos = ctx.part_world_position(drive_pedal)
    ctx.check(
        "bottom bracket joint rotates crank around spindle",
        rest_drive_pos is not None
        and turned_drive_pos is not None
        and turned_drive_pos[1] > rest_drive_pos[1] + 0.12
        and abs(turned_drive_pos[2]) < 0.04,
        details=f"rest={rest_drive_pos}, turned={turned_drive_pos}",
    )

    rest_pedal_aabb = ctx.part_world_aabb(drive_pedal)
    with ctx.pose({drive_pedal_joint: math.pi / 2.0}):
        turned_pedal_aabb = ctx.part_world_aabb(drive_pedal)
    if rest_pedal_aabb is not None and turned_pedal_aabb is not None:
        rest_height = rest_pedal_aabb[1][2] - rest_pedal_aabb[0][2]
        turned_height = turned_pedal_aabb[1][2] - turned_pedal_aabb[0][2]
        ctx.check(
            "flat cage pedal spins on its axle",
            rest_height < 0.04 and turned_height > 0.09,
            details=f"rest_height={rest_height:.3f}, turned_height={turned_height:.3f}",
        )
    else:
        ctx.fail("flat cage pedal spins on its axle", "pedal AABB unavailable")

    return ctx.report()


object_model = build_object_model()

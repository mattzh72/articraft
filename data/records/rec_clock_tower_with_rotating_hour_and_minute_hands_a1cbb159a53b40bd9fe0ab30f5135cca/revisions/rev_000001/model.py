from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

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


SHAFT_WIDTH = 0.90
SHAFT_DEPTH = 0.70
SHAFT_HEIGHT = 3.00
CLOCK_Z = 2.10
CLOCK_FACE_Y = -0.395
CLOCK_RADIUS = 0.315
FACE_FRONT_Y = CLOCK_FACE_Y - 0.0175
HOUR_Y = FACE_FRONT_Y - 0.0135
MINUTE_Y = HOUR_Y - 0.0140


def _pyramid_mesh(width: float, depth: float, base_z: float, height: float) -> MeshGeometry:
    """Closed square-pyramid roof, authored in the tower/root frame."""
    geom = MeshGeometry()
    corners = [
        (-width / 2.0, -depth / 2.0, base_z),
        (width / 2.0, -depth / 2.0, base_z),
        (width / 2.0, depth / 2.0, base_z),
        (-width / 2.0, depth / 2.0, base_z),
    ]
    ids = [geom.add_vertex(*p) for p in corners]
    apex = geom.add_vertex(0.0, 0.0, base_z + height)
    geom.add_face(ids[0], ids[1], apex)
    geom.add_face(ids[1], ids[2], apex)
    geom.add_face(ids[2], ids[3], apex)
    geom.add_face(ids[3], ids[0], apex)
    geom.add_face(ids[0], ids[2], ids[1])
    geom.add_face(ids[0], ids[3], ids[2])
    return geom


def _add_sloped_box(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    direction: tuple[float, float, float],
    length: float,
    thickness: float,
    depth: float,
    material,
) -> None:
    """Add a timber whose long local Z axis follows a vector in the XZ plane."""
    dx, _dy, dz = direction
    angle = atan2(dx, dz)
    part.visual(
        Box((thickness, depth, length)),
        origin=Origin(xyz=center, rpy=(0.0, angle, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_clock_tower")

    pale_wood = Material("pale weathered wood", rgba=(0.64, 0.45, 0.25, 1.0))
    dark_timber = Material("dark exposed timber", rgba=(0.23, 0.12, 0.055, 1.0))
    end_grain = Material("cut end grain", rgba=(0.34, 0.20, 0.10, 1.0))
    stone = Material("rough stone plinth", rgba=(0.46, 0.43, 0.38, 1.0))
    roof_mat = Material("dark cedar shingles", rgba=(0.18, 0.16, 0.13, 1.0))
    shingle_edge = Material("raised shingle edges", rgba=(0.10, 0.085, 0.07, 1.0))
    dial_mat = Material("aged enamel clock dial", rgba=(0.91, 0.87, 0.73, 1.0))
    black = Material("painted black metal", rgba=(0.015, 0.013, 0.010, 1.0))
    brass = Material("aged brass spindle", rgba=(0.72, 0.52, 0.20, 1.0))

    tower = model.part("tower")

    # Village-scale stone foot and simple rectangular timber shaft.
    tower.visual(
        Box((1.08, 0.88, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=stone,
        name="stone_plinth",
    )
    tower.visual(
        Box((SHAFT_WIDTH, SHAFT_DEPTH, SHAFT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.18 + SHAFT_HEIGHT / 2.0)),
        material=pale_wood,
        name="wooden_shaft",
    )

    # Exposed timber-frame posts and horizontal girts, slightly proud of the shaft.
    post_z = 0.18 + SHAFT_HEIGHT / 2.0
    for ix, x in enumerate((-SHAFT_WIDTH / 2.0, SHAFT_WIDTH / 2.0)):
        for iy, y in enumerate((-SHAFT_DEPTH / 2.0, SHAFT_DEPTH / 2.0)):
            tower.visual(
                Box((0.085, 0.085, SHAFT_HEIGHT + 0.02)),
                origin=Origin(xyz=(x, y, post_z)),
                material=dark_timber,
                name=f"corner_post_{ix}_{iy}",
            )

    for iz, z in enumerate((0.42, 1.22, 2.02, 2.86)):
        tower.visual(
            Box((SHAFT_WIDTH + 0.08, 0.075, 0.090)),
            origin=Origin(xyz=(0.0, -SHAFT_DEPTH / 2.0 - 0.026, z)),
            material=dark_timber,
            name=f"front_girt_{iz}",
        )
        tower.visual(
            Box((SHAFT_WIDTH + 0.08, 0.075, 0.090)),
            origin=Origin(xyz=(0.0, SHAFT_DEPTH / 2.0 + 0.026, z)),
            material=dark_timber,
            name=f"rear_girt_{iz}",
        )
        tower.visual(
            Box((0.075, SHAFT_DEPTH + 0.08, 0.090)),
            origin=Origin(xyz=(-SHAFT_WIDTH / 2.0 - 0.026, 0.0, z)),
            material=dark_timber,
            name=f"side_girt_{iz}_0",
        )
        tower.visual(
            Box((0.075, SHAFT_DEPTH + 0.08, 0.090)),
            origin=Origin(xyz=(SHAFT_WIDTH / 2.0 + 0.026, 0.0, z)),
            material=dark_timber,
            name=f"side_girt_{iz}_1",
        )

    brace_length = sqrt(0.58 * 0.58 + 0.62 * 0.62)
    _add_sloped_box(
        tower,
        name="front_brace_0",
        center=(-0.12, -SHAFT_DEPTH / 2.0 - 0.034, 1.56),
        direction=(0.58, 0.0, 0.62),
        length=brace_length,
        thickness=0.070,
        depth=0.070,
        material=dark_timber,
    )
    _add_sloped_box(
        tower,
        name="front_brace_1",
        center=(0.12, -SHAFT_DEPTH / 2.0 - 0.034, 1.56),
        direction=(-0.58, 0.0, 0.62),
        length=brace_length,
        thickness=0.070,
        depth=0.070,
        material=dark_timber,
    )

    # Top plate and a pyramidal roof with proud shingle courses on all four faces.
    tower.visual(
        Box((1.18, 0.98, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 3.21)),
        material=dark_timber,
        name="roof_plate",
    )
    roof_mesh = _pyramid_mesh(1.30, 1.10, 3.24, 0.70)
    tower.visual(
        mesh_from_geometry(roof_mesh, "pyramidal_roof"),
        material=roof_mat,
        name="pyramidal_roof",
    )

    for row, t in enumerate((0.13, 0.26, 0.39, 0.52, 0.65)):
        z = 3.24 + 0.70 * t
        x_len = 1.30 * (1.0 - t) + 0.04
        y_len = 1.10 * (1.0 - t) + 0.04
        front_y = -0.55 * (1.0 - t) - 0.010
        rear_y = 0.55 * (1.0 - t) + 0.010
        left_x = -0.65 * (1.0 - t) - 0.010
        right_x = 0.65 * (1.0 - t) + 0.010
        tower.visual(
            Box((x_len, 0.026, 0.028)),
            origin=Origin(xyz=(0.0, front_y, z)),
            material=shingle_edge,
            name=f"front_shingle_{row}",
        )
        tower.visual(
            Box((x_len, 0.026, 0.028)),
            origin=Origin(xyz=(0.0, rear_y, z)),
            material=shingle_edge,
            name=f"rear_shingle_{row}",
        )
        tower.visual(
            Box((0.026, y_len, 0.028)),
            origin=Origin(xyz=(left_x, 0.0, z)),
            material=shingle_edge,
            name=f"side_shingle_{row}_0",
        )
        tower.visual(
            Box((0.026, y_len, 0.028)),
            origin=Origin(xyz=(right_x, 0.0, z)),
            material=shingle_edge,
            name=f"side_shingle_{row}_1",
        )

    # A single front clock face, with fixed tick marks and a visible spindle boss.
    tower.visual(
        Cylinder(radius=CLOCK_RADIUS, length=0.035),
        origin=Origin(xyz=(0.0, CLOCK_FACE_Y, CLOCK_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_mat,
        name="clock_face",
    )
    for i in range(12):
        theta = i * pi / 6.0
        major = i % 3 == 0
        radius = CLOCK_RADIUS - (0.035 if major else 0.028)
        tick_length = 0.078 if major else 0.046
        tick_width = 0.026 if major else 0.014
        tower.visual(
            Box((tick_width, 0.008, tick_length)),
            origin=Origin(
                xyz=(radius * sin(theta), FACE_FRONT_Y - 0.001, CLOCK_Z + radius * cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=black,
            name=f"tick_{i}",
        )
    tower.visual(
        Cylinder(radius=0.045, length=0.0085),
        origin=Origin(
            xyz=(0.0, (FACE_FRONT_Y + HOUR_Y + 0.005) / 2.0, CLOCK_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="bearing_post",
    )

    hour_hand = model.part("hour_hand")
    hour_angle = -0.82
    hour_len = 0.185
    hour_hand.visual(
        Cylinder(radius=0.062, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hub",
    )
    hour_hand.visual(
        Box((0.044, 0.010, hour_len)),
        origin=Origin(
            xyz=(0.5 * hour_len * sin(hour_angle), 0.0, 0.5 * hour_len * cos(hour_angle)),
            rpy=(0.0, hour_angle, 0.0),
        ),
        material=black,
        name="pointer",
    )
    hour_hand.visual(
        Box((0.026, 0.010, 0.070)),
        origin=Origin(
            xyz=(-0.035 * sin(hour_angle), 0.0, -0.035 * cos(hour_angle)),
            rpy=(0.0, hour_angle, 0.0),
        ),
        material=end_grain,
        name="tail",
    )

    minute_hand = model.part("minute_hand")
    minute_len = 0.265
    minute_hand.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub",
    )
    minute_hand.visual(
        Box((0.026, 0.008, minute_len)),
        origin=Origin(xyz=(0.0, 0.0, minute_len / 2.0)),
        material=black,
        name="pointer",
    )
    minute_hand.visual(
        Box((0.018, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=black,
        name="tail",
    )

    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, HOUR_Y, CLOCK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, MINUTE_Y, CLOCK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour = object_model.get_part("hour_hand")
    minute = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    ctx.check(
        "minute hand uses a continuous joint",
        minute_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={minute_joint.articulation_type}",
    )
    ctx.check(
        "hour hand uses a separate continuous joint",
        hour_joint.articulation_type == ArticulationType.CONTINUOUS
        and hour_joint.name != minute_joint.name,
        details=f"hour={hour_joint.name}:{hour_joint.articulation_type}, "
        f"minute={minute_joint.name}:{minute_joint.articulation_type}",
    )
    ctx.expect_origin_distance(
        hour,
        minute,
        axes="xz",
        max_dist=0.001,
        name="hand origins are coaxial through the clock center",
    )
    ctx.expect_overlap(
        minute,
        tower,
        axes="xz",
        elem_a="pointer",
        elem_b="clock_face",
        min_overlap=0.020,
        name="minute pointer lies over the single clock face",
    )
    ctx.expect_overlap(
        hour,
        tower,
        axes="xz",
        elem_a="pointer",
        elem_b="clock_face",
        min_overlap=0.040,
        name="hour pointer lies over the single clock face",
    )
    ctx.expect_gap(
        tower,
        hour,
        axis="y",
        positive_elem="bearing_post",
        negative_elem="hub",
        max_gap=0.001,
        max_penetration=0.0001,
        name="hour hand hub is seated on the clock spindle",
    )
    ctx.expect_gap(
        hour,
        minute,
        axis="y",
        positive_elem="hub",
        negative_elem="hub",
        max_gap=0.001,
        max_penetration=0.0001,
        name="minute hand hub is stacked in front of hour hand hub",
    )

    rest_minute = ctx.part_element_world_aabb(minute, elem="pointer")
    with ctx.pose({minute_joint: pi / 2.0}):
        turned_minute = ctx.part_element_world_aabb(minute, elem="pointer")
    ctx.check(
        "minute hand rotates about the face center",
        rest_minute is not None
        and turned_minute is not None
        and turned_minute[0][0] < rest_minute[0][0] - 0.15
        and abs(turned_minute[0][2] - CLOCK_Z) < 0.04,
        details=f"rest={rest_minute}, turned={turned_minute}",
    )

    rest_hour = ctx.part_element_world_aabb(hour, elem="pointer")
    with ctx.pose({hour_joint: pi / 2.0}):
        turned_hour = ctx.part_element_world_aabb(hour, elem="pointer")
    ctx.check(
        "hour hand rotates independently on its coaxial joint",
        rest_hour is not None
        and turned_hour is not None
        and turned_hour[0][2] < rest_hour[0][2] - 0.10,
        details=f"rest={rest_hour}, turned={turned_hour}",
    )

    return ctx.report()


object_model = build_object_model()

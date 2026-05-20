from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


OUTER_RADIUS = 1.22
WALL_THICKNESS = 0.06
INNER_RADIUS = OUTER_RADIUS - WALL_THICKNESS
FLOOR_THICKNESS = 0.04
CLEAR_HEIGHT = 2.30
CANOPY_THICKNESS = 0.10
CANOPY_CENTER_Z = CLEAR_HEIGHT + CANOPY_THICKNESS / 2.0

COLUMN_RADIUS = 0.12
COLUMN_HEIGHT = 2.22

WING_BOTTOM = 0.05
WING_HEIGHT = 2.12
WING_THICKNESS = 0.045
STILE_WIDTH = 0.06
RAIL_HEIGHT = 0.06
INNER_STILE_START = 0.06
WING_TIP_GAP = 0.015
LEADING_EDGE_X = INNER_RADIUS - WING_TIP_GAP
TOP_RAIL_LENGTH = LEADING_EDGE_X - INNER_STILE_START - 2.0 * STILE_WIDTH
WING_CENTER_X = (INNER_STILE_START + LEADING_EDGE_X) / 2.0


def add_quad(
    geom: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def cylindrical_sector_shell(
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    start_angle: float,
    end_angle: float,
    segments: int = 40,
) -> MeshGeometry:
    geom = MeshGeometry()
    angles = [
        start_angle + (end_angle - start_angle) * i / segments
        for i in range(segments + 1)
    ]

    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for angle in angles:
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z0))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z1))
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z0))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z1))

    for i in range(segments):
        add_quad(
            geom,
            outer_bottom[i],
            outer_bottom[i + 1],
            outer_top[i + 1],
            outer_top[i],
        )
        add_quad(
            geom,
            inner_bottom[i + 1],
            inner_bottom[i],
            inner_top[i],
            inner_top[i + 1],
        )
        add_quad(
            geom,
            outer_top[i],
            outer_top[i + 1],
            inner_top[i + 1],
            inner_top[i],
        )
        add_quad(
            geom,
            outer_bottom[i + 1],
            outer_bottom[i],
            inner_bottom[i],
            inner_bottom[i + 1],
        )

    add_quad(
        geom,
        inner_bottom[0],
        outer_bottom[0],
        outer_top[0],
        inner_top[0],
    )
    add_quad(
        geom,
        outer_bottom[-1],
        inner_bottom[-1],
        inner_top[-1],
        outer_top[-1],
    )
    return geom


def rotated_xy(radius_along_x: float, angle: float) -> tuple[float, float]:
    return (
        radius_along_x * math.cos(angle),
        radius_along_x * math.sin(angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_three_wing_revolving_door")

    metal = model.material("frame_metal", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_metal = model.material("column_metal", rgba=(0.36, 0.38, 0.40, 1.0))
    floor_finish = model.material("floor_finish", rgba=(0.18, 0.18, 0.20, 1.0))
    glass = model.material("door_glass", rgba=(0.68, 0.82, 0.92, 0.35))
    drum_glass = model.material("drum_glass", rgba=(0.76, 0.88, 0.96, 0.24))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=OUTER_RADIUS, length=FLOOR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS / 2.0)),
        material=floor_finish,
        name="floor_disc",
    )
    frame.visual(
        Cylinder(radius=OUTER_RADIUS, length=CANOPY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_CENTER_Z)),
        material=metal,
        name="canopy_disc",
    )

    left_wall = cylindrical_sector_shell(
        inner_radius=INNER_RADIUS,
        outer_radius=OUTER_RADIUS,
        z0=FLOOR_THICKNESS,
        z1=CLEAR_HEIGHT,
        start_angle=math.radians(45.0),
        end_angle=math.radians(135.0),
        segments=36,
    )
    right_wall = cylindrical_sector_shell(
        inner_radius=INNER_RADIUS,
        outer_radius=OUTER_RADIUS,
        z0=FLOOR_THICKNESS,
        z1=CLEAR_HEIGHT,
        start_angle=math.radians(225.0),
        end_angle=math.radians(315.0),
        segments=36,
    )
    frame.visual(
        mesh_from_geometry(left_wall, "left_drum_wall"),
        material=drum_glass,
        name="left_drum_wall",
    )
    frame.visual(
        mesh_from_geometry(right_wall, "right_drum_wall"),
        material=drum_glass,
        name="right_drum_wall",
    )

    post_height = CLEAR_HEIGHT - FLOOR_THICKNESS
    post_z = FLOOR_THICKNESS + post_height / 2.0
    for label, angle_deg in (
        ("front_left_jamb", 45.0),
        ("rear_left_jamb", 135.0),
        ("rear_right_jamb", 225.0),
        ("front_right_jamb", 315.0),
    ):
        angle = math.radians(angle_deg)
        post_radius = (INNER_RADIUS + OUTER_RADIUS) / 2.0
        frame.visual(
            Box((WALL_THICKNESS + 0.01, 0.10, post_height)),
            origin=Origin(
                xyz=(post_radius * math.cos(angle), post_radius * math.sin(angle), post_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=metal,
            name=label,
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT / 2.0)),
        material=dark_metal,
        name="center_column",
    )
    rotor.visual(
        Cylinder(radius=0.17, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="lower_hub_disc",
    )
    rotor.visual(
        Cylinder(radius=0.17, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT - 0.14)),
        material=dark_metal,
        name="upper_hub_disc",
    )

    glass_height = WING_HEIGHT - 2.0 * RAIL_HEIGHT + 0.004
    glass_length = TOP_RAIL_LENGTH + 0.004
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        inner_stile_xy = rotated_xy(INNER_STILE_START + STILE_WIDTH / 2.0, angle)
        leading_stile_xy = rotated_xy(LEADING_EDGE_X - STILE_WIDTH / 2.0, angle)
        wing_center_xy = rotated_xy(WING_CENTER_X, angle)
        rotor.visual(
            Box((STILE_WIDTH, WING_THICKNESS, WING_HEIGHT)),
            origin=Origin(
                xyz=(inner_stile_xy[0], inner_stile_xy[1], WING_BOTTOM + WING_HEIGHT / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"wing_{index}_inner_stile",
        )
        rotor.visual(
            Box((STILE_WIDTH, WING_THICKNESS, WING_HEIGHT)),
            origin=Origin(
                xyz=(leading_stile_xy[0], leading_stile_xy[1], WING_BOTTOM + WING_HEIGHT / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"wing_{index}_leading_stile",
        )
        rotor.visual(
            Box((TOP_RAIL_LENGTH, WING_THICKNESS, RAIL_HEIGHT)),
            origin=Origin(
                xyz=(wing_center_xy[0], wing_center_xy[1], WING_BOTTOM + WING_HEIGHT - RAIL_HEIGHT / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"wing_{index}_top_rail",
        )
        rotor.visual(
            Box((TOP_RAIL_LENGTH, WING_THICKNESS, RAIL_HEIGHT)),
            origin=Origin(
                xyz=(wing_center_xy[0], wing_center_xy[1], WING_BOTTOM + RAIL_HEIGHT / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"wing_{index}_bottom_rail",
        )
        rotor.visual(
            Box((glass_length, WING_THICKNESS * 0.36, glass_height)),
            origin=Origin(
                xyz=(wing_center_xy[0], wing_center_xy[1], WING_BOTTOM + WING_HEIGHT / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=glass,
            name=f"wing_{index}_glass",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )
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

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "revolving assembly uses continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="center_column",
        elem_b="floor_disc",
        contact_tol=1e-6,
        name="center column is seated on the floor disc",
    )

    leading_angles: list[float] = []
    for index in range(3):
        aabb = ctx.part_element_world_aabb(rotor, elem=f"wing_{index}_leading_stile")
        if aabb is None:
            ctx.fail("all three wing leading stiles exist", f"missing wing {index} leading stile")
            continue
        center_x = (aabb[0][0] + aabb[1][0]) / 2.0
        center_y = (aabb[0][1] + aabb[1][1]) / 2.0
        leading_angles.append((math.atan2(center_y, center_x) + 2.0 * math.pi) % (2.0 * math.pi))

    if len(leading_angles) == 3:
        leading_angles.sort()
        separations = [
            leading_angles[1] - leading_angles[0],
            leading_angles[2] - leading_angles[1],
            (leading_angles[0] + 2.0 * math.pi) - leading_angles[2],
        ]
        ctx.check(
            "three wings are equally spaced at 120 degrees",
            all(abs(separation - 2.0 * math.pi / 3.0) < 0.14 for separation in separations),
            details=f"angles={leading_angles}, separations={separations}",
        )

    wing0_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_leading_stile")
    if wing0_aabb is not None:
        ctx.check(
            "front wing reaches close to the drum wall",
            INNER_RADIUS - 0.04 <= wing0_aabb[1][0] <= INNER_RADIUS + 0.001,
            details=f"leading_stile_max_x={wing0_aabb[1][0]}, target_inner_radius={INNER_RADIUS}",
        )

    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_leading_stile")
    if wing0_aabb is not None and turned_aabb is not None:
        rest_center = (
            (wing0_aabb[0][0] + wing0_aabb[1][0]) / 2.0,
            (wing0_aabb[0][1] + wing0_aabb[1][1]) / 2.0,
        )
        turned_center = (
            (turned_aabb[0][0] + turned_aabb[1][0]) / 2.0,
            (turned_aabb[0][1] + turned_aabb[1][1]) / 2.0,
        )
        ctx.check(
            "wing assembly rotates around the exposed center column",
            rest_center[0] > 0.95
            and abs(rest_center[1]) < 0.08
            and turned_center[1] > 0.95
            and abs(turned_center[0]) < 0.08,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

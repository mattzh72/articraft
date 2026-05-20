from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


DISH_RADIUS = 0.55
DISH_FOCAL_LENGTH = 0.36
DISH_VERTEX_Y = 0.16
DISH_THICKNESS = 0.018
DISH_RIM_Y = DISH_VERTEX_Y + (DISH_RADIUS * DISH_RADIUS) / (4.0 * DISH_FOCAL_LENGTH)
DISH_FOCUS_Y = DISH_VERTEX_Y + DISH_FOCAL_LENGTH
ELEVATION_AXIS_Z = 0.50


def _parabolic_reflector_geometry(
    *,
    radius: float = DISH_RADIUS,
    focal_length: float = DISH_FOCAL_LENGTH,
    vertex_y: float = DISH_VERTEX_Y,
    thickness: float = DISH_THICKNESS,
    radial_segments: int = 96,
    rings: int = 18,
) -> MeshGeometry:
    """Thin-walled parabolic reflector opening toward local +Y."""

    geom = MeshGeometry()
    front: list[list[int]] = []
    back: list[list[int]] = []

    for i in range(rings + 1):
        r = radius * i / rings
        y = vertex_y + (r * r) / (4.0 * focal_length)
        front_ring: list[int] = []
        back_ring: list[int] = []
        for j in range(radial_segments):
            theta = 2.0 * math.pi * j / radial_segments
            x = r * math.cos(theta)
            z = r * math.sin(theta)
            front_ring.append(geom.add_vertex(x, y, z))
            # The back skin is offset rearward; the small thickness reads as a
            # real spun/composite dish instead of a single mathematical surface.
            back_ring.append(geom.add_vertex(x, y - thickness, z))
        front.append(front_ring)
        back.append(back_ring)

    # Front reflective surface.
    for i in range(rings):
        for j in range(radial_segments):
            j2 = (j + 1) % radial_segments
            if i == 0:
                geom.add_face(front[i][j], front[i + 1][j], front[i + 1][j2])
            else:
                geom.add_face(front[i][j], front[i + 1][j], front[i + 1][j2])
                geom.add_face(front[i][j], front[i + 1][j2], front[i][j2])

    # Rear skin.
    for i in range(rings):
        for j in range(radial_segments):
            j2 = (j + 1) % radial_segments
            if i == 0:
                geom.add_face(back[i][j], back[i + 1][j2], back[i + 1][j])
            else:
                geom.add_face(back[i][j], back[i + 1][j2], back[i + 1][j])
                geom.add_face(back[i][j], back[i][j2], back[i + 1][j2])

    # Rolled rim closing the thin shell.
    i = rings
    for j in range(radial_segments):
        j2 = (j + 1) % radial_segments
        geom.add_face(front[i][j], front[i][j2], back[i][j2])
        geom.add_face(front[i][j], back[i][j2], back[i][j])

    return geom


def _lathed_feed_horn_geometry() -> MeshGeometry:
    """Small corrugated feed horn with its axis along local +Y."""

    profile = [
        (0.018, -0.055),
        (0.055, -0.040),
        (0.066, -0.030),
        (0.052, -0.018),
        (0.044, 0.010),
        (0.033, 0.045),
        (0.023, 0.070),
    ]
    horn = LatheGeometry(profile, segments=48, closed=True)
    # Lathe axis is local Z.  Rotate so the horn points along +Y, with the
    # flared mouth looking back toward the reflector.
    horn.rotate_x(-math.pi / 2.0)
    return horn


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_satellite_dish")

    concrete = model.material("weathered_concrete", rgba=(0.42, 0.42, 0.38, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.58, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    white = model.material("matte_white_reflector", rgba=(0.86, 0.88, 0.84, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    amber = model.material("status_amber_lens", rgba=(1.0, 0.55, 0.08, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.44, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="concrete_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.1425)),
        material=galvanized,
        name="base_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.085, length=0.96),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=galvanized,
        name="mast_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        material=dark_metal,
        name="lower_bearing_race",
    )

    # Four welded gussets between the mast and base flange.
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal.visual(
            Box((0.035, 0.20, 0.22)),
            origin=Origin(
                xyz=(0.075 * math.cos(yaw), 0.075 * math.sin(yaw), 0.27),
                rpy=(0.0, 0.0, yaw),
            ),
            material=galvanized,
            name=f"mast_gusset_{idx}",
        )

    # Anchor bolts on the concrete plinth.
    for idx, yaw in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        pedestal.visual(
            Cylinder(radius=0.018, length=0.035),
            origin=Origin(xyz=(0.245 * math.cos(yaw), 0.245 * math.sin(yaw), 0.137)),
            material=dark_metal,
            name=f"anchor_bolt_{idx}",
        )

    # Stationary outdoor electronics cabinet strapped to the mast.
    pedestal.visual(
        Box((0.28, 0.12, 0.36)),
        origin=Origin(xyz=(0.0, -0.145, 0.62)),
        material=dark_metal,
        name="control_cabinet",
    )
    pedestal.visual(
        Box((0.24, 0.015, 0.30)),
        origin=Origin(xyz=(0.0, -0.212, 0.62)),
        material=galvanized,
        name="cabinet_door_seam",
    )
    pedestal.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.07, -0.222, 0.73), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="status_lamp",
    )
    pedestal.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(0.151, -0.145, 0.62)),
        material=black,
        name="cabinet_conduit",
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.18, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark_metal,
        name="azimuth_turntable",
    )
    azimuth_head.visual(
        Cylinder(radius=0.10, length=0.33),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=galvanized,
        name="rotating_neck",
    )
    azimuth_head.visual(
        Box((0.34, 0.18, 0.17)),
        origin=Origin(xyz=(0.0, -0.185, 0.16)),
        material=dark_metal,
        name="azimuth_motor_box",
    )
    azimuth_head.visual(
        Cylinder(radius=0.060, length=0.37),
        origin=Origin(xyz=(0.0, -0.095, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="worm_gear_cover",
    )

    yoke = TrunnionYokeGeometry(
        (1.36, 0.22, 0.66),
        span_width=1.12,
        trunnion_diameter=0.13,
        trunnion_center_z=ELEVATION_AXIS_Z,
        base_thickness=0.055,
        corner_radius=0.018,
        center=False,
    )
    azimuth_head.visual(
        mesh_from_geometry(yoke, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="trunnion_yoke",
    )
    for idx, x in enumerate((-0.555, 0.555)):
        azimuth_head.visual(
            Box((0.11, 0.11, 0.055)),
            origin=Origin(xyz=(x, 0.0, ELEVATION_AXIS_Z - 0.052 - 0.0275)),
            material=dark_metal,
            name=f"bearing_saddle_{idx}",
        )

    # Service bolts around the rotating bearing.
    for idx in range(8):
        yaw = 2.0 * math.pi * idx / 8.0
        azimuth_head.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(0.135 * math.cos(yaw), 0.135 * math.sin(yaw), 0.084)),
            material=galvanized,
            name=f"turntable_bolt_{idx}",
        )

    dish = model.part("dish")
    dish.visual(
        mesh_from_geometry(_parabolic_reflector_geometry(), "parabolic_reflector"),
        origin=Origin(),
        material=white,
        name="reflector_shell",
    )
    # Central bearing barrel and back boss sit on the elevation axis.
    dish.visual(
        Cylinder(radius=0.052, length=1.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="trunnion_barrel",
    )
    dish.visual(
        Cylinder(radius=0.10, length=0.13),
        origin=Origin(xyz=(0.0, 0.080, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="rear_hub",
    )

    # Eight shallow rear ribs visibly tie the reflector skin into the hub.
    for idx in range(8):
        theta = 2.0 * math.pi * idx / 8.0
        x_mid = 0.26 * math.cos(theta)
        z_mid = 0.26 * math.sin(theta)
        x_end = 0.50 * math.cos(theta)
        z_end = 0.50 * math.sin(theta)
        rib = tube_from_spline_points(
            [
                (0.07 * math.cos(theta), 0.080, 0.07 * math.sin(theta)),
                (x_mid, DISH_VERTEX_Y + (0.26 * 0.26) / (4.0 * DISH_FOCAL_LENGTH) - 0.026, z_mid),
                (x_end, DISH_VERTEX_Y + (0.50 * 0.50) / (4.0 * DISH_FOCAL_LENGTH) - 0.020, z_end),
            ],
            radius=0.010,
            samples_per_segment=10,
            radial_segments=12,
        )
        dish.visual(
            mesh_from_geometry(rib, f"rear_rib_{idx}"),
            origin=Origin(),
            material=galvanized,
            name=f"rear_rib_{idx}",
        )

    feed_point = (0.0, DISH_FOCUS_Y + 0.010, 0.0)
    horn = _lathed_feed_horn_geometry().translate(*feed_point)
    dish.visual(
        mesh_from_geometry(horn, "feed_horn"),
        origin=Origin(),
        material=dark_metal,
        name="feed_horn",
    )
    dish.visual(
        Cylinder(radius=0.022, length=0.13),
        origin=Origin(xyz=(0.0, DISH_FOCUS_Y + 0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="low_noise_block",
    )

    # Three feed support struts converge at the horn, with a small black cable
    # riding along the lower strut.
    rim_angles = (math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)
    for idx, theta in enumerate(rim_angles):
        rim_point = (DISH_RADIUS * math.cos(theta), DISH_RIM_Y + 0.005, DISH_RADIUS * math.sin(theta))
        mid = (
            0.45 * rim_point[0],
            (rim_point[1] + feed_point[1]) * 0.5 + 0.035,
            0.45 * rim_point[2],
        )
        arm = tube_from_spline_points(
            [rim_point, mid, feed_point],
            radius=0.009,
            samples_per_segment=12,
            radial_segments=12,
        )
        dish.visual(
            mesh_from_geometry(arm, f"feed_strut_{idx}"),
            origin=Origin(),
            material=dark_metal,
            name=f"feed_strut_{idx}",
        )

    cable = tube_from_spline_points(
        [
            (0.0, DISH_FOCUS_Y + 0.080, -0.018),
            (-0.10, 0.13, -0.20),
            (-0.16, 0.01, -0.38),
            (-0.08, 0.050, -0.16),
        ],
        radius=0.006,
        samples_per_segment=12,
        radial_segments=10,
    )
    dish.visual(
        mesh_from_geometry(cable, "feed_cable"),
        origin=Origin(),
        material=black,
        name="feed_cable",
    )

    # Articulated aiming: azimuth slews on the pedestal, elevation tips the
    # entire dish/feed assembly in the trunnion yoke.
    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 1.1525)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=dish,
        # A real installed dish rests already aimed above the horizon; the joint
        # frame carries that nominal tilt, while q adjusts elevation.
        origin=Origin(xyz=(0.0, 0.0, ELEVATION_AXIS_Z), rpy=(0.42, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.22, lower=-0.32, upper=0.82),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    azimuth_head = object_model.get_part("azimuth_head")
    dish = object_model.get_part("dish")
    yaw = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.expect_gap(
        azimuth_head,
        pedestal,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="azimuth_turntable",
        negative_elem="lower_bearing_race",
        name="azimuth bearing is seated on pedestal",
    )
    ctx.expect_within(
        dish,
        azimuth_head,
        axes="x",
        margin=0.02,
        inner_elem="trunnion_barrel",
        outer_elem="trunnion_yoke",
        name="dish trunnion is captured between yoke cheeks",
    )
    ctx.expect_overlap(
        dish,
        azimuth_head,
        axes="x",
        min_overlap=0.95,
        elem_a="trunnion_barrel",
        elem_b="trunnion_yoke",
        name="trunnion spans the yoke opening",
    )

    rest_feed_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({elevation: 0.65}):
        raised_feed_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")

    rest_feed_z = None if rest_feed_aabb is None else (rest_feed_aabb[0][2] + rest_feed_aabb[1][2]) * 0.5
    raised_feed_z = None if raised_feed_aabb is None else (raised_feed_aabb[0][2] + raised_feed_aabb[1][2]) * 0.5
    ctx.check(
        "positive elevation raises feed assembly",
        rest_feed_z is not None and raised_feed_z is not None and raised_feed_z > rest_feed_z + 0.08,
        details=f"rest_feed_z={rest_feed_z}, raised_feed_z={raised_feed_z}",
    )

    rest_reflector_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({yaw: 1.05}):
        slewed_reflector_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
    rest_feed_x = None if rest_reflector_aabb is None else (rest_reflector_aabb[0][0] + rest_reflector_aabb[1][0]) * 0.5
    slewed_feed_x = None if slewed_reflector_aabb is None else (slewed_reflector_aabb[0][0] + slewed_reflector_aabb[1][0]) * 0.5
    ctx.check(
        "azimuth slew moves feed around mast",
        rest_feed_x is not None and slewed_feed_x is not None and abs(slewed_feed_x - rest_feed_x) > 0.06,
        details=f"rest_feed_x={rest_feed_x}, slewed_feed_x={slewed_feed_x}",
    )

    return ctx.report()


object_model = build_object_model()

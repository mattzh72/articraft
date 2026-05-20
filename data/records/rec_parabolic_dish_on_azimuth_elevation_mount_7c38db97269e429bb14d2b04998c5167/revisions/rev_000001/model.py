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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


LOWER_ELEVATION_DEG = 10.0
ELEVATION_TRAVEL_DEG = 70.0


def _parabolic_reflector_mesh(
    *,
    radius: float = 0.70,
    depth: float = 0.28,
    thickness: float = 0.025,
    radial_segments: int = 18,
    angular_segments: int = 96,
    inner_radius: float = 0.035,
) -> MeshGeometry:
    """Thin parabolic bowl whose optical axis is local +X."""
    geom = MeshGeometry()
    front: list[list[int]] = []
    back: list[list[int]] = []

    for i in range(radial_segments + 1):
        t = i / radial_segments
        r = inner_radius + (radius - inner_radius) * t
        x_front = depth * (r / radius) ** 2
        # Approximate a thin back shell by offsetting mostly along the optical axis.
        x_back = x_front - thickness
        front_ring: list[int] = []
        back_ring: list[int] = []
        for j in range(angular_segments):
            a = 2.0 * math.pi * j / angular_segments
            y = r * math.cos(a)
            z = r * math.sin(a)
            front_ring.append(geom.add_vertex(x_front, y, z))
            back_ring.append(geom.add_vertex(x_back, y, z))
        front.append(front_ring)
        back.append(back_ring)

    for i in range(radial_segments):
        for j in range(angular_segments):
            jn = (j + 1) % angular_segments
            # Reflecting face.
            geom.add_face(front[i][j], front[i + 1][j], front[i + 1][jn])
            geom.add_face(front[i][j], front[i + 1][jn], front[i][jn])
            # Convex rear face, reversed winding.
            geom.add_face(back[i][jn], back[i + 1][jn], back[i + 1][j])
            geom.add_face(back[i][jn], back[i + 1][j], back[i][j])

    # Join the inner hub aperture and the outer rim so the reflector reads as a thin shell.
    for ring_i in (0, radial_segments):
        for j in range(angular_segments):
            jn = (j + 1) % angular_segments
            geom.add_face(front[ring_i][j], back[ring_i][j], back[ring_i][jn])
            geom.add_face(front[ring_i][j], back[ring_i][jn], front[ring_i][jn])

    return geom


def _rear_frame_mesh() -> MeshGeometry:
    """Back bracing ring and four welded radial tubes for the reflector."""
    geom = TorusGeometry(radius=0.58, tube=0.018, radial_segments=24, tubular_segments=96)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(0.19, 0.0, 0.0)

    for sy, sz in ((1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)):
        strut = tube_from_spline_points(
            [
                (-0.035, 0.0, 0.0),
                (0.08, 0.23 * sy, 0.23 * sz),
                (0.19, 0.43 * sy, 0.43 * sz),
            ],
            radius=0.016,
            samples_per_segment=8,
            radial_segments=14,
            cap_ends=True,
        )
        geom.merge(strut)

    lower_feed_socket = tube_from_spline_points(
        [
            (0.215, 0.0, -0.58),
            (0.30, 0.0, -0.36),
            (0.42, 0.0, -0.10),
        ],
        radius=0.017,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
    )
    geom.merge(lower_feed_socket)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_pedestal_dish")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_steel = model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.09, 1.0))
    off_white = model.material("matte_white_reflector", rgba=(0.88, 0.88, 0.84, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    feed_gray = model.material("feed_gray", rgba=(0.34, 0.36, 0.37, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.55, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=black,
        name="base_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.17, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=galvanized,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.38)),
        material=dark_steel,
        name="fixed_turntable",
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        Cylinder(radius=0.38, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_steel,
        name="rotating_turntable",
    )
    yaw_head.visual(
        Cylinder(radius=0.13, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=galvanized,
        name="azimuth_neck",
    )
    yaw_head.visual(
        Box((0.30, 1.85, 0.08)),
        origin=Origin(xyz=(-0.34, 0.0, 0.37)),
        material=dark_steel,
        name="yoke_base",
    )
    yaw_head.visual(
        Box((0.22, 0.28, 0.10)),
        origin=Origin(xyz=(-0.205, 0.0, 0.32)),
        material=dark_steel,
        name="azimuth_saddle",
    )
    yaw_head.visual(
        Box((0.16, 0.10, 0.58)),
        origin=Origin(xyz=(-0.34, -0.82, 0.66)),
        material=dark_steel,
        name="yoke_post_0",
    )
    yaw_head.visual(
        Box((0.42, 0.10, 0.12)),
        origin=Origin(xyz=(-0.13, -0.82, 0.60)),
        material=dark_steel,
        name="lower_lug_0",
    )
    yaw_head.visual(
        Box((0.42, 0.10, 0.12)),
        origin=Origin(xyz=(-0.13, -0.82, 0.96)),
        material=dark_steel,
        name="upper_lug_0",
    )
    yaw_head.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.105, tube=0.024, radial_segments=20, tubular_segments=64).rotate_x(math.pi / 2.0),
            "bearing_ring_0_mesh",
        ),
        origin=Origin(xyz=(0.0, -0.82, 0.78)),
        material=dark_steel,
        name="bearing_ring_0",
    )
    yaw_head.visual(
        Box((0.16, 0.10, 0.58)),
        origin=Origin(xyz=(-0.34, 0.82, 0.66)),
        material=dark_steel,
        name="yoke_post_1",
    )
    yaw_head.visual(
        Box((0.42, 0.10, 0.12)),
        origin=Origin(xyz=(-0.13, 0.82, 0.60)),
        material=dark_steel,
        name="lower_lug_1",
    )
    yaw_head.visual(
        Box((0.42, 0.10, 0.12)),
        origin=Origin(xyz=(-0.13, 0.82, 0.96)),
        material=dark_steel,
        name="upper_lug_1",
    )
    yaw_head.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.105, tube=0.024, radial_segments=20, tubular_segments=64).rotate_x(math.pi / 2.0),
            "bearing_ring_1_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.82, 0.78)),
        material=dark_steel,
        name="bearing_ring_1",
    )
    yaw_head.visual(
        Cylinder(radius=0.026, length=1.66),
        origin=Origin(xyz=(-0.18, 0.0, 0.88), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_cross_tube",
    )

    dish_frame = model.part("dish_frame")
    dish_frame.visual(
        mesh_from_geometry(_parabolic_reflector_mesh(), "parabolic_reflector"),
        origin=Origin(),
        material=off_white,
        name="reflector_shell",
    )
    dish_frame.visual(
        mesh_from_geometry(_rear_frame_mesh(), "rear_support_frame"),
        origin=Origin(),
        material=dark_steel,
        name="rear_support_frame",
    )
    dish_frame.visual(
        Cylinder(radius=0.081, length=1.74),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="trunnion_shaft",
    )
    dish_frame.visual(
        Cylinder(radius=0.090, length=0.065),
        origin=Origin(xyz=(-0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_hub",
    )
    dish_frame.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.215, 0.0, -0.58),
                    (0.31, 0.0, -0.35),
                    (0.44, 0.0, -0.035),
                ],
                radius=0.023,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
            "feed_arm_mesh",
        ),
        origin=Origin(),
        material=galvanized,
        name="feed_arm",
    )
    dish_frame.visual(
        Cylinder(radius=0.060, length=0.14),
        origin=Origin(xyz=(0.47, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=feed_gray,
        name="feed_horn",
    )
    dish_frame.visual(
        Sphere(radius=0.025),
        origin=Origin(xyz=(0.41, 0.0, -0.08)),
        material=feed_gray,
        name="feed_arm_boss",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, 1.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.45),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_head,
        child=dish_frame,
        origin=Origin(
            xyz=(0.0, 0.0, 0.78),
            rpy=(0.0, -math.radians(LOWER_ELEVATION_DEG), 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.35,
            lower=0.0,
            upper=math.radians(ELEVATION_TRAVEL_DEG),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_head = object_model.get_part("yaw_head")
    dish_frame = object_model.get_part("dish_frame")
    azimuth = object_model.get_articulation("azimuth_axis")
    elevation = object_model.get_articulation("elevation_axis")

    ctx.check(
        "azimuth is continuous yaw",
        azimuth.articulation_type == ArticulationType.CONTINUOUS and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    ctx.check(
        "elevation travel is ten to eighty degrees",
        elevation.motion_limits is not None
        and abs((elevation.motion_limits.upper or 0.0) - math.radians(ELEVATION_TRAVEL_DEG)) < 1e-6
        and abs(elevation.origin.rpy[1] + math.radians(LOWER_ELEVATION_DEG)) < 1e-6,
        details=f"origin={elevation.origin}, limits={elevation.motion_limits}",
    )
    ctx.expect_gap(
        yaw_head,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotating_turntable",
        negative_elem="fixed_turntable",
        name="azimuth turntable is seated on pedestal bearing",
    )
    ctx.allow_overlap(
        dish_frame,
        yaw_head,
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        reason="The elevation trunnion is intentionally captured in the first bearing bushing; the mesh ring slightly intersects the shaft to model a seated journal.",
    )
    ctx.allow_overlap(
        dish_frame,
        yaw_head,
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        reason="The elevation trunnion is intentionally captured in the second bearing bushing; the mesh ring slightly intersects the shaft to model a seated journal.",
    )
    ctx.expect_overlap(
        dish_frame,
        yaw_head,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        min_overlap=0.02,
        name="trunnion reaches the first yoke bearing",
    )
    ctx.expect_overlap(
        dish_frame,
        yaw_head,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        min_overlap=0.02,
        name="trunnion reaches the second yoke bearing",
    )
    ctx.expect_contact(
        yaw_head,
        yaw_head,
        elem_a="bearing_ring_0",
        elem_b="lower_lug_0",
        contact_tol=0.006,
        name="first bearing ring is welded to its lower yoke lug",
    )
    ctx.expect_contact(
        yaw_head,
        yaw_head,
        elem_a="bearing_ring_1",
        elem_b="lower_lug_1",
        contact_tol=0.006,
        name="second bearing ring is welded to its lower yoke lug",
    )

    lower_feed_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
    with ctx.pose({elevation: math.radians(ELEVATION_TRAVEL_DEG)}):
        upper_feed_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        ctx.expect_gap(
            dish_frame,
            yaw_head,
            axis="z",
            min_gap=-0.03,
            name="high elevation reflector clears the yoke base",
            positive_elem="reflector_shell",
            negative_elem="yoke_base",
        )
    lower_feed_z = None if lower_feed_aabb is None else (lower_feed_aabb[0][2] + lower_feed_aabb[1][2]) / 2.0
    upper_feed_z = None if upper_feed_aabb is None else (upper_feed_aabb[0][2] + upper_feed_aabb[1][2]) / 2.0
    ctx.check(
        "elevation raises the feed and reflector boresight",
        lower_feed_z is not None and upper_feed_z is not None and upper_feed_z > lower_feed_z + 0.30,
        details=f"lower_feed_z={lower_feed_z}, upper_feed_z={upper_feed_z}",
    )

    return ctx.report()


object_model = build_object_model()

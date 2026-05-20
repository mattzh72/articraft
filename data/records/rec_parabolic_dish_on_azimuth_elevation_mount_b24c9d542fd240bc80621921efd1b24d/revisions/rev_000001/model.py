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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)
import cadquery as cq


def _parabolic_reflector_shell(
    *,
    radius: float = 0.30,
    depth: float = 0.10,
    thickness: float = 0.008,
    center_radius: float = 0.014,
    rings: int = 18,
    segments: int = 96,
) -> MeshGeometry:
    """Thin parabolic dish shell with its boresight along local +X."""

    geom = MeshGeometry()
    front: list[list[int]] = []
    back: list[list[int]] = []

    for i in range(rings + 1):
        t = i / rings
        r = center_radius + (radius - center_radius) * t
        x_front = 0.020 + depth * (r / radius) ** 2
        x_back = x_front - thickness
        front_ring: list[int] = []
        back_ring: list[int] = []
        for j in range(segments):
            a = 2.0 * math.pi * j / segments
            y = r * math.cos(a)
            z = r * math.sin(a)
            front_ring.append(geom.add_vertex(x_front, y, z))
            back_ring.append(geom.add_vertex(x_back, y, z))
        front.append(front_ring)
        back.append(back_ring)

    for i in range(rings):
        for j in range(segments):
            j2 = (j + 1) % segments
            geom.add_face(front[i][j], front[i + 1][j], front[i + 1][j2])
            geom.add_face(front[i][j], front[i + 1][j2], front[i][j2])

            geom.add_face(back[i][j2], back[i + 1][j2], back[i + 1][j])
            geom.add_face(back[i][j2], back[i + 1][j], back[i][j])

    # Close the small center ring and the rolled outer lip so the shell is one
    # continuous thin-walled body rather than two disconnected paraboloids.
    for j in range(segments):
        j2 = (j + 1) % segments
        geom.add_face(front[0][j], front[0][j2], back[0][j2])
        geom.add_face(front[0][j], back[0][j2], back[0][j])
        geom.add_face(front[-1][j2], front[-1][j], back[-1][j])
        geom.add_face(front[-1][j2], back[-1][j], back[-1][j2])

    return geom


def _collar_yoke_shape() -> cq.Workplane:
    """Rotating collar and one-piece elevation yoke, authored in meters."""

    collar = cq.Workplane("XY").circle(0.098).circle(0.052).extrude(0.090, both=True)
    turntable = (
        cq.Workplane("XY")
        .circle(0.145)
        .circle(0.052)
        .extrude(0.030)
        .translate((0.0, 0.0, 0.045))
    )
    pedestal = cq.Workplane("XY").box(0.160, 0.150, 0.090).translate((0.0, 0.0, 0.120))
    lower_bridge = cq.Workplane("XY").box(0.190, 0.730, 0.080).translate((-0.015, 0.0, 0.135))
    cheek_a = cq.Workplane("XY").box(0.180, 0.048, 0.690).translate((-0.010, 0.345, 0.455))
    cheek_b = cq.Workplane("XY").box(0.180, 0.048, 0.690).translate((-0.010, -0.345, 0.455))

    # Bearing bosses on the outside faces; the elevation bore is cut through
    # the bosses and both cheeks after the unions below.
    boss_a = (
        cq.Workplane("XY")
        .circle(0.058)
        .extrude(0.028, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, 0.373, 0.500))
    )
    boss_b = (
        cq.Workplane("XY")
        .circle(0.058)
        .extrude(0.028, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, -0.373, 0.500))
    )

    body = (
        collar.union(turntable)
        .union(pedestal)
        .union(lower_bridge)
        .union(cheek_a)
        .union(cheek_b)
        .union(boss_a)
        .union(boss_b)
    )
    elevation_bore = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(0.900, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, 0.0, 0.500))
    )
    return body.cut(elevation_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="azimuth_elevation_satellite_dish")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    reflector_white = model.material("warm_white_reflector", rgba=(0.86, 0.86, 0.80, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.015, 0.018, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.045, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=galvanized,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.054, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=dark_steel,
        name="azimuth_bearing",
    )
    mast.visual(
        Cylinder(radius=0.170, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="base_plate",
    )

    yoke = model.part("collar_yoke")
    yoke.visual(
        mesh_from_cadquery(_collar_yoke_shape(), "collar_yoke_shell", tolerance=0.0012),
        material=galvanized,
        name="yoke_shell",
    )

    reflector = model.part("reflector")
    reflector.visual(
        mesh_from_geometry(_parabolic_reflector_shell(), "parabolic_reflector_shell"),
        material=reflector_white,
        name="dish_shell",
    )
    reflector.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.300, tube=0.010, radial_segments=16, tubular_segments=96).rotate_y(
                math.pi / 2.0
            ).translate(0.120, 0.0, 0.0),
            "rolled_rim",
        ),
        material=reflector_white,
        name="rolled_rim",
    )
    reflector.visual(
        Cylinder(radius=0.060, length=0.070),
        origin=Origin(xyz=(-0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_hub",
    )
    reflector.visual(
        Cylinder(radius=0.027, length=0.830),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="trunnion_axle",
    )
    reflector.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.425, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_cap_0",
    )
    reflector.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, -0.425, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_cap_1",
    )
    reflector.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.088, 0.0, -0.258), (0.230, 0.0, -0.130), (0.390, 0.0, -0.006)],
                radius=0.011,
                samples_per_segment=10,
                radial_segments=14,
            ),
            "feed_arm_tube",
        ),
        material=galvanized,
        name="feed_arm",
    )
    reflector.visual(
        Box((0.055, 0.075, 0.040)),
        origin=Origin(xyz=(0.392, 0.0, -0.004)),
        material=black,
        name="feed_block",
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.6, lower=-0.15, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    yoke = object_model.get_part("collar_yoke")
    reflector = object_model.get_part("reflector")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.allow_overlap(
        yoke,
        mast,
        elem_a="yoke_shell",
        elem_b="azimuth_bearing",
        reason="The mast-top bearing is intentionally seated inside the rotating collar bore to show the azimuth collar captured on the mast.",
    )
    ctx.allow_overlap(
        yoke,
        reflector,
        elem_a="yoke_shell",
        elem_b="trunnion_axle",
        reason="The elevation trunnion shaft is intentionally captured in the yoke bearing bores with a tiny proxy interference fit.",
    )
    ctx.expect_overlap(
        yoke,
        mast,
        axes="z",
        elem_a="yoke_shell",
        elem_b="azimuth_bearing",
        min_overlap=0.040,
        name="azimuth bearing remains inserted in the collar",
    )
    ctx.expect_within(
        mast,
        yoke,
        axes="xy",
        inner_elem="azimuth_bearing",
        outer_elem="yoke_shell",
        margin=0.002,
        name="mast bearing is concentric with the collar",
    )
    ctx.expect_overlap(
        yoke,
        mast,
        axes="xy",
        elem_a="yoke_shell",
        elem_b="mast_tube",
        min_overlap=0.075,
        name="collar is centered over the mast",
    )
    ctx.expect_within(
        reflector,
        yoke,
        axes="y",
        inner_elem="dish_shell",
        outer_elem="yoke_shell",
        margin=0.020,
        name="reflector fits between the yoke cheeks",
    )
    ctx.expect_overlap(
        reflector,
        yoke,
        axes="y",
        elem_a="trunnion_axle",
        elem_b="yoke_shell",
        min_overlap=0.600,
        name="trunnion spans both elevation bearings",
    )

    with ctx.pose({elevation: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    with ctx.pose({elevation: 0.75}):
        raised_aabb = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 if rest_aabb else None
    raised_center_z = (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5 if raised_aabb else None
    ctx.check(
        "positive elevation raises the reflector face",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.025,
        details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
    )

    with ctx.pose({azimuth: 0.0}):
        rest_az_aabb = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    with ctx.pose({azimuth: 0.85}):
        swept_az_aabb = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    rest_center_y = (rest_az_aabb[0][1] + rest_az_aabb[1][1]) * 0.5 if rest_az_aabb else None
    swept_center_y = (swept_az_aabb[0][1] + swept_az_aabb[1][1]) * 0.5 if swept_az_aabb else None
    ctx.check(
        "azimuth joint sweeps the dish around the mast",
        rest_center_y is not None
        and swept_center_y is not None
        and abs(swept_center_y - rest_center_y) > 0.030,
        details=f"rest_y={rest_center_y}, swept_y={swept_center_y}",
    )

    return ctx.report()


object_model = build_object_model()

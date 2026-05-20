from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _dish_shell(radius: float, depth: float, thickness: float) -> LatheGeometry:
    """Thin parabolic reflector shell, authored around local +Z."""

    outer_profile = []
    inner_profile = []
    for i in range(18):
        t = i / 17.0
        r = radius * t
        z_inner = -depth + depth * (t * t)
        outer_profile.append((r, z_inner - thickness))
        inner_profile.append((r, z_inner))
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _feed_struts(radius: float, feed_y: float) -> MeshGeometry:
    """Four slender stays from the rim to the feed head."""

    geom = MeshGeometry()
    for angle in (math.radians(38), math.radians(142), math.radians(218), math.radians(322)):
        rim = (radius * math.cos(angle), 0.0, radius * math.sin(angle))
        strut = wire_from_points(
            [rim, (0.0, feed_y, 0.0)],
            radius=0.007,
            radial_segments=12,
            cap_ends=True,
        )
        geom.merge(strut)
    return geom


def _back_spider(radius: float, depth: float) -> MeshGeometry:
    """Rear reinforcing spider ribs on the back of the reflector."""

    geom = MeshGeometry()
    hub = (0.0, -depth - 0.010, 0.0)
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        rim = (0.72 * radius * math.cos(angle), -0.020, 0.72 * radius * math.sin(angle))
        rib = wire_from_points(
            [hub, rim],
            radius=0.010,
            radial_segments=12,
            cap_ends=True,
        )
        geom.merge(rib)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shipboard_gimbaled_communications_dish")

    white = Material("weathered_marine_white", rgba=(0.82, 0.85, 0.82, 1.0))
    light_grey = Material("satin_light_grey", rgba=(0.58, 0.62, 0.62, 1.0))
    dark_grey = Material("dark_sealed_bearing", rgba=(0.10, 0.12, 0.13, 1.0))
    black = Material("black_rubber_gasket", rgba=(0.02, 0.025, 0.03, 1.0))
    bronze = Material("bronze_bolt_heads", rgba=(0.58, 0.43, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.38, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_grey,
        name="deck_flange",
    )
    base.visual(
        Cylinder(radius=0.20, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=light_grey,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.29, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=black,
        name="fixed_bearing_race",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        base.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(
                xyz=(0.315 * math.cos(angle), 0.315 * math.sin(angle), 0.063)
            ),
            material=bronze,
            name=f"bolt_{i}",
        )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.305, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=white,
        name="turntable_disc",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.250, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black,
        name="rotating_bearing_cap",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.135, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=light_grey,
        name="central_neck",
    )
    azimuth_stage.visual(
        Box((0.78, 0.30, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=white,
        name="yoke_saddle",
    )
    azimuth_stage.visual(
        Box((1.16, 0.140, 0.070)),
        origin=Origin(xyz=(0.0, -0.175, 0.105)),
        material=white,
        name="rear_bridge",
    )
    azimuth_stage.visual(
        Box((0.150, 0.060, 0.150)),
        origin=Origin(xyz=(0.0, -0.215, 0.075)),
        material=white,
        name="rear_support",
    )
    azimuth_stage.visual(
        Box((0.100, 0.220, 0.470)),
        origin=Origin(xyz=(-0.515, 0.0, 0.375)),
        material=white,
        name="yoke_arm_0",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.095, length=0.075),
        origin=Origin(xyz=(-0.515, 0.0, 0.575), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_grey,
        name="bearing_0",
    )
    azimuth_stage.visual(
        Box((0.100, 0.220, 0.470)),
        origin=Origin(xyz=(0.515, 0.0, 0.375)),
        material=white,
        name="yoke_arm_1",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.095, length=0.075),
        origin=Origin(xyz=(0.515, 0.0, 0.575), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_grey,
        name="bearing_1",
    )

    reflector = model.part("reflector")
    dish_radius = 0.420
    dish_depth = 0.165
    reflector.visual(
        mesh_from_geometry(_dish_shell(dish_radius, dish_depth, 0.016), "reflector_shell"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="reflector_shell",
    )
    reflector.visual(
        mesh_from_geometry(TorusGeometry(dish_radius, 0.017, radial_segments=20, tubular_segments=96), "rim_ring"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=light_grey,
        name="rim_ring",
    )
    reflector.visual(
        Cylinder(radius=0.055, length=1.070),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="trunnion_axle",
    )
    for side, x in enumerate((-0.445, 0.445)):
        reflector.visual(
            Cylinder(radius=0.076, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=light_grey,
            name=f"trunnion_collar_{side}",
        )
    reflector.visual(
        Cylinder(radius=0.094, length=0.060),
        origin=Origin(xyz=(0.0, -dish_depth - 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=light_grey,
        name="rear_hub",
    )
    reflector.visual(
        mesh_from_geometry(_back_spider(dish_radius, dish_depth), "back_spider"),
        material=light_grey,
        name="back_spider",
    )
    reflector.visual(
        Cylinder(radius=0.014, length=0.620),
        origin=Origin(xyz=(0.0, 0.185, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="feed_boom",
    )
    reflector.visual(
        mesh_from_geometry(_feed_struts(dish_radius * 0.92, 0.435), "feed_struts"),
        material=dark_grey,
        name="feed_struts",
    )
    reflector.visual(
        mesh_from_geometry(ConeGeometry(0.065, 0.120, radial_segments=32, closed=True), "feed_horn"),
        origin=Origin(xyz=(0.0, 0.465, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=light_grey,
        name="feed_horn",
    )
    reflector.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.425, 0.0)),
        material=black,
        name="feed_collar",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=-0.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    azimuth_stage = object_model.get_part("azimuth_stage")
    reflector = object_model.get_part("reflector")
    elevation = object_model.get_articulation("elevation")

    for support_elem in ("yoke_arm_0", "yoke_arm_1", "bearing_0", "bearing_1"):
        ctx.allow_overlap(
            azimuth_stage,
            reflector,
            elem_a=support_elem,
            elem_b="trunnion_axle",
            reason="The reflector trunnion is intentionally captured through the cradle bearing bore.",
        )

    ctx.expect_overlap(
        reflector,
        azimuth_stage,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="yoke_arm_0",
        min_overlap=0.060,
        name="trunnion passes through one yoke arm",
    )
    ctx.expect_overlap(
        reflector,
        azimuth_stage,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="yoke_arm_1",
        min_overlap=0.060,
        name="trunnion passes through opposite yoke arm",
    )
    ctx.expect_overlap(
        reflector,
        azimuth_stage,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="bearing_0",
        min_overlap=0.040,
        name="trunnion engages one cradle bearing",
    )
    ctx.expect_overlap(
        reflector,
        azimuth_stage,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="bearing_1",
        min_overlap=0.040,
        name="trunnion engages opposite cradle bearing",
    )
    ctx.expect_within(
        reflector,
        azimuth_stage,
        axes="x",
        inner_elem="reflector_shell",
        margin=0.000,
        name="reflector bowl fits between yoke cheeks",
    )

    rest_aabb = ctx.part_element_world_aabb(reflector, elem="feed_horn")
    with ctx.pose({elevation: 0.80}):
        raised_aabb = ctx.part_element_world_aabb(reflector, elem="feed_horn")
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    raised_center_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
    ctx.check(
        "positive elevation raises the feed horn",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.20,
        details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()

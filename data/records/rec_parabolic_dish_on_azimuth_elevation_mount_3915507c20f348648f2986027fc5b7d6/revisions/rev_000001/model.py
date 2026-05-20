from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _reflector_shell():
    """Thin, shallow parabolic reflector with its optical axis along local +X."""

    radius = 0.33
    depth = 0.105
    thickness = 0.014
    samples = 12

    inner_profile = []
    outer_profile = []
    for i in range(samples + 1):
        t = i / samples
        r = 0.026 + (radius - 0.026) * t
        x = -depth + depth * (r / radius) ** 2
        inner_profile.append((r, x))
        # The rear surface is slightly fatter at the rim, giving a visible lip.
        outer_profile.append((r + 0.010 * t, x - thickness))

    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    # Lathe revolves around local Z; rotate that axis into local +X, then make
    # the reflector a little taller-than-flat but not perfectly circular.
    geom.rotate_y(math.pi / 2.0)
    geom.scale(1.0, 1.0, 0.88)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_mount_satellite_dish")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    off_white = model.material("matte_off_white_reflector", rgba=(0.86, 0.86, 0.80, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bolt_mat = model.material("zinc_bolts", rgba=(0.72, 0.72, 0.68, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.90, 0.75, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=galvanized,
        name="flashing_plate",
    )
    roof_mount.visual(
        Cylinder(radius=0.185, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=rubber,
        name="azimuth_bearing",
    )
    roof_mount.visual(
        Cylinder(radius=0.225, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark_steel,
        name="bearing_shadow_ring",
    )
    for i, (x, y) in enumerate(((-0.33, -0.25), (-0.33, 0.25), (0.33, -0.25), (0.33, 0.25))):
        roof_mount.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(xyz=(x, y, 0.054)),
            material=bolt_mat,
            name=f"lag_bolt_{i}",
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.160, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="turntable_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.068, length=0.385),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=dark_steel,
        name="short_column",
    )
    pedestal.visual(
        Box((0.25, 0.88, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=dark_steel,
        name="yoke_crossbeam",
    )
    for i, y in enumerate((-0.455, 0.455)):
        pedestal.visual(
            Box((0.17, 0.060, 0.480)),
            origin=Origin(xyz=(0.0, y, 0.660)),
            material=dark_steel,
            name=f"yoke_cheek_{i}",
        )
    pedestal.visual(
        Cylinder(radius=0.074, length=0.076),
        origin=Origin(xyz=(0.0, -0.392, 0.780), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pivot_collar_0",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, -0.392, 0.780), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="dark_bore_0",
    )
    pedestal.visual(
        Cylinder(radius=0.074, length=0.076),
        origin=Origin(xyz=(0.0, 0.392, 0.780), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pivot_collar_1",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, 0.392, 0.780), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="dark_bore_1",
    )

    dish_frame = model.part("dish_frame")
    dish_frame.visual(
        mesh_from_geometry(_reflector_shell(), "shallow_reflector_shell"),
        origin=Origin(),
        material=off_white,
        name="reflector_shell",
    )
    dish_frame.visual(
        Cylinder(radius=0.080, length=0.070),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="center_hub",
    )
    dish_frame.visual(
        Cylinder(radius=0.034, length=0.760),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="trunnion_axle",
    )
    for i, y in enumerate((-0.250, 0.250)):
        dish_frame.visual(
            Box((0.036, 0.040, 0.450)),
            origin=Origin(xyz=(-0.100, y, 0.0)),
            material=dark_steel,
            name=f"rear_side_rail_{i}",
        )
    for i, z in enumerate((-0.205, 0.205)):
        dish_frame.visual(
            Box((0.036, 0.560, 0.036)),
            origin=Origin(xyz=(-0.100, 0.0, z)),
            material=dark_steel,
            name=f"rear_cross_rail_{i}",
        )
    for i, y in enumerate((-0.315, 0.315)):
        dish_frame.visual(
            Box((0.120, 0.090, 0.080)),
            origin=Origin(xyz=(-0.052, y * 0.94, 0.0)),
            material=dark_steel,
            name=f"trunnion_lug_{i}",
        )

    feed_angle = -0.425
    dish_frame.visual(
        Box((0.505, 0.025, 0.025)),
        origin=Origin(xyz=(0.250, 0.0, -0.176), rpy=(0.0, feed_angle, 0.0)),
        material=dark_steel,
        name="feed_arm",
    )
    dish_frame.visual(
        Box((0.080, 0.065, 0.052)),
        origin=Origin(xyz=(0.020, 0.0, -0.278)),
        material=dark_steel,
        name="feed_arm_foot",
    )
    dish_frame.visual(
        Box((0.090, 0.058, 0.052)),
        origin=Origin(xyz=(0.510, 0.0, -0.070)),
        material=rubber,
        name="lnb_body",
    )
    dish_frame.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.455, 0.0, -0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="feed_horn",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=roof_mount,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.60),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=dish_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.45, lower=-0.32, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    pedestal = object_model.get_part("pedestal")
    dish_frame = object_model.get_part("dish_frame")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.allow_overlap(
        pedestal,
        dish_frame,
        elem_a="pivot_collar_0",
        elem_b="trunnion_axle",
        reason="The side trunnion axle is intentionally captured inside the yoke pivot collar.",
    )
    ctx.allow_overlap(
        pedestal,
        dish_frame,
        elem_a="pivot_collar_1",
        elem_b="trunnion_axle",
        reason="The side trunnion axle is intentionally captured inside the yoke pivot collar.",
    )
    ctx.allow_overlap(
        pedestal,
        dish_frame,
        elem_a="dark_bore_0",
        elem_b="trunnion_axle",
        reason="The black bushing represents the trunnion bore that surrounds the captured axle.",
    )
    ctx.allow_overlap(
        pedestal,
        dish_frame,
        elem_a="dark_bore_1",
        elem_b="trunnion_axle",
        reason="The black bushing represents the trunnion bore that surrounds the captured axle.",
    )

    ctx.check(
        "azimuth is vertical continuous rotation",
        azimuth.articulation_type == ArticulationType.CONTINUOUS and azimuth.axis == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    ctx.check(
        "elevation is a limited horizontal trunnion",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and elevation.axis == (0.0, -1.0, 0.0)
        and elevation.motion_limits is not None
        and elevation.motion_limits.lower < 0.0
        and elevation.motion_limits.upper > 0.8,
        details=f"type={elevation.articulation_type}, axis={elevation.axis}, limits={elevation.motion_limits}",
    )
    ctx.expect_gap(
        pedestal,
        roof_mount,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_disk",
        negative_elem="azimuth_bearing",
        name="rotating pedestal sits on roof bearing",
    )
    ctx.expect_overlap(
        pedestal,
        roof_mount,
        axes="xy",
        min_overlap=0.25,
        elem_a="turntable_disk",
        elem_b="azimuth_bearing",
        name="azimuth bearing footprints overlap",
    )
    for collar in ("pivot_collar_0", "pivot_collar_1"):
        ctx.expect_overlap(
            dish_frame,
            pedestal,
            axes="xz",
            min_overlap=0.055,
            elem_a="trunnion_axle",
            elem_b=collar,
            name=f"{collar} stays coaxial with trunnion axle",
        )
    for bore in ("dark_bore_0", "dark_bore_1"):
        ctx.expect_overlap(
            dish_frame,
            pedestal,
            axes="xz",
            min_overlap=0.030,
            elem_a="trunnion_axle",
            elem_b=bore,
            name=f"{bore} surrounds trunnion axle",
        )
    ctx.expect_within(
        dish_frame,
        pedestal,
        axes="y",
        margin=0.002,
        inner_elem="trunnion_axle",
        outer_elem="yoke_crossbeam",
        name="rear frame axle remains clipped between side pivots",
    )

    rest_feed = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
    with ctx.pose({elevation: 0.60}):
        raised_feed = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        for collar in ("pivot_collar_0", "pivot_collar_1"):
            ctx.expect_overlap(
                dish_frame,
                pedestal,
                axes="xz",
                min_overlap=0.055,
                elem_a="trunnion_axle",
                elem_b=collar,
                name=f"{collar} supports elevated dish",
            )

    if rest_feed is None or raised_feed is None:
        ctx.fail("feed horn pose measurement available", "feed_horn AABB was unavailable")
    else:
        rest_z = (rest_feed[0][2] + rest_feed[1][2]) / 2.0
        raised_z = (raised_feed[0][2] + raised_feed[1][2]) / 2.0
        ctx.check(
            "positive elevation raises the feed side",
            raised_z > rest_z + 0.18,
            details=f"rest_z={rest_z:.3f}, raised_z={raised_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()

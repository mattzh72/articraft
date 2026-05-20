from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_bracket_communication_dish")

    model.material("galvanized_steel", rgba=(0.62, 0.65, 0.64, 1.0))
    model.material("dark_clamp", rgba=(0.16, 0.17, 0.17, 1.0))
    model.material("warm_wall", rgba=(0.76, 0.72, 0.64, 1.0))
    model.material("dish_white", rgba=(0.92, 0.94, 0.91, 1.0))
    model.material("rubber_black", rgba=(0.02, 0.025, 0.025, 1.0))
    model.material("signal_gray", rgba=(0.52, 0.55, 0.55, 1.0))

    # Root: a wall plate with two projecting brackets carrying the vertical pipe mast.
    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.040, 0.340, 0.560)),
        origin=Origin(xyz=(-0.180, 0.0, 0.0)),
        material="warm_wall",
        name="wall_plate",
    )
    wall_mount.visual(
        Cylinder(radius=0.026, length=1.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="pipe_mast",
    )
    for z in (-0.245, 0.245):
        wall_mount.visual(
            Box((0.162, 0.058, 0.046)),
            origin=Origin(xyz=(-0.080, 0.0, z)),
            material="galvanized_steel",
            name=f"standoff_arm_{'lower' if z < 0 else 'upper'}",
        )
        wall_mount.visual(
            Box((0.070, 0.080, 0.030)),
            origin=Origin(xyz=(-0.026, 0.0, z)),
            material="galvanized_steel",
            name=f"pipe_saddle_{'lower' if z < 0 else 'upper'}",
        )
    for y in (-0.110, 0.110):
        for z in (-0.180, 0.180):
            wall_mount.visual(
                Cylinder(radius=0.016, length=0.012),
                origin=Origin(xyz=(-0.154, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material="dark_clamp",
                name=f"wall_bolt_{y:+.2f}_{z:+.2f}",
            )

    # Azimuth/yaw head: a clearanced annular clamp around the mast with a captured
    # elevation bracket at its forward side.
    clamp_head = model.part("clamp_head")
    clamp_shell = LatheGeometry.from_shell_profiles(
        [(0.056, -0.075), (0.056, 0.075)],
        [(0.032, -0.075), (0.032, 0.075)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    clamp_head.visual(
        mesh_from_geometry(clamp_shell, "azimuth_clamp_collar"),
        material="dark_clamp",
        name="azimuth_collar",
    )
    for x, suffix in ((0.033, "front"), (-0.033, "rear")):
        clamp_head.visual(
            Box((0.014, 0.034, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="rubber_black",
            name=f"mast_liner_{suffix}",
        )
    # Split-clamp ears and bolts so the head reads as a real pipe clamp.
    for y in (-0.047, 0.047):
        clamp_head.visual(
            Box((0.205, 0.018, 0.026)),
            origin=Origin(xyz=(0.080, y, -0.055)),
            material="dark_clamp",
            name=f"lower_side_arm_{0 if y < 0 else 1}",
        )
    clamp_head.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(xyz=(0.035, 0.0, -0.056), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="clamp_bolt",
    )
    for y in (-0.055, 0.055):
        clamp_head.visual(
            Box((0.026, 0.014, 0.174)),
            origin=Origin(xyz=(0.180, y, 0.0)),
            material="dark_clamp",
            name=f"elevation_cheek_{0 if y < 0 else 1}",
        )
    clamp_head.visual(
        Box((0.030, 0.124, 0.018)),
        origin=Origin(xyz=(0.180, 0.0, -0.038)),
        material="dark_clamp",
        name="keeper_bridge_0",
    )
    clamp_head.visual(
        Box((0.030, 0.124, 0.018)),
        origin=Origin(xyz=(0.180, 0.0, 0.038)),
        material="dark_clamp",
        name="keeper_bridge_1",
    )
    clamp_head.visual(
        Box((0.004, 0.018, 0.145)),
        origin=Origin(xyz=(0.196, -0.068, 0.0)),
        material="galvanized_steel",
        name="elevation_scale_plate",
    )

    model.articulation(
        "mast_to_clamp",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-2.4, upper=2.4),
    )

    dish_frame = model.part("dish_frame")
    # The child frame is the elevation axis.  The trunnion barrel fits inside
    # the keeper bridges without touching the cheeks, so the frame stays captured.
    dish_frame.visual(
        Cylinder(radius=0.023, length=0.096),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="galvanized_steel",
        name="trunnion_barrel",
    )
    dish_frame.visual(
        Cylinder(radius=0.055, length=0.036),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="galvanized_steel",
        name="rear_hub",
    )
    dish_frame.visual(
        Cylinder(radius=0.015, length=0.168),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="galvanized_steel",
        name="rear_support_tube",
    )

    reflector_shell = LatheGeometry.from_shell_profiles(
        [(0.050, 0.000), (0.180, 0.044), (0.305, 0.112), (0.366, 0.165)],
        [(0.030, 0.012), (0.158, 0.056), (0.285, 0.120), (0.344, 0.158)],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    reflector_shell.rotate_y(math.pi / 2.0).translate(0.158, 0.0, 0.0)
    dish_frame.visual(
        mesh_from_geometry(reflector_shell, "shallow_parabolic_reflector"),
        material="dish_white",
        name="reflector_shell",
    )

    dish_frame.visual(
        Box((0.070, 0.042, 0.038)),
        origin=Origin(xyz=(0.255, 0.0, -0.305)),
        material="galvanized_steel",
        name="feed_arm_foot",
    )
    feed_arm_geom = tube_from_spline_points(
        [
            (0.248, 0.0, -0.312),
            (0.350, 0.0, -0.260),
            (0.500, 0.0, -0.105),
            (0.600, 0.0, -0.010),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    dish_frame.visual(
        mesh_from_geometry(feed_arm_geom, "curved_feed_arm"),
        material="galvanized_steel",
        name="feed_arm",
    )
    dish_frame.visual(
        Cylinder(radius=0.032, length=0.082),
        origin=Origin(xyz=(0.638, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="feed_horn",
    )
    dish_frame.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(xyz=(0.590, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="signal_gray",
        name="feed_collar",
    )

    model.articulation(
        "clamp_to_dish",
        ArticulationType.REVOLUTE,
        parent=clamp_head,
        child=dish_frame,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.8, lower=-0.65, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("wall_mount")
    clamp = object_model.get_part("clamp_head")
    dish = object_model.get_part("dish_frame")
    yaw = object_model.get_articulation("mast_to_clamp")
    elev = object_model.get_articulation("clamp_to_dish")

    ctx.expect_within(
        mast,
        clamp,
        axes="xy",
        inner_elem="pipe_mast",
        outer_elem="azimuth_collar",
        margin=0.0,
        name="mast stays centered through the azimuth collar",
    )
    ctx.expect_within(
        dish,
        clamp,
        axes="y",
        inner_elem="trunnion_barrel",
        outer_elem="keeper_bridge_0",
        margin=0.0,
        name="trunnion barrel is captured between elevation cheeks",
    )
    ctx.expect_gap(
        dish,
        clamp,
        axis="z",
        positive_elem="trunnion_barrel",
        negative_elem="keeper_bridge_0",
        min_gap=0.0,
        max_gap=0.008,
        name="lower keeper sits close under trunnion barrel",
    )
    with ctx.pose({elev: 0.75}):
        ctx.expect_within(
            dish,
            clamp,
            axes="y",
            inner_elem="trunnion_barrel",
            outer_elem="keeper_bridge_1",
            margin=0.0,
            name="tilted trunnion remains between side cheeks",
        )
        ctx.expect_gap(
            clamp,
            dish,
            axis="z",
            positive_elem="keeper_bridge_1",
            negative_elem="trunnion_barrel",
            min_gap=0.0,
            max_gap=0.008,
            name="upper keeper stays close over tilted trunnion",
        )

    rest_feed = ctx.part_element_world_aabb(dish, elem="feed_horn")
    rest_clamp = ctx.part_world_position(clamp)
    with ctx.pose({yaw: 0.75, elev: 0.55}):
        raised_feed = ctx.part_element_world_aabb(dish, elem="feed_horn")
        yawed_clamp = ctx.part_world_position(clamp)
    ctx.check(
        "positive elevation raises the feed end",
        rest_feed is not None
        and raised_feed is not None
        and raised_feed[0][2] > rest_feed[0][2] + 0.10,
        details=f"rest_feed={rest_feed}, raised_feed={raised_feed}",
    )
    ctx.check(
        "azimuth joint remains centered while yawing",
        rest_clamp is not None
        and yawed_clamp is not None
        and abs(rest_clamp[2] - yawed_clamp[2]) < 1e-6,
        details=f"rest_clamp={rest_clamp}, yawed_clamp={yawed_clamp}",
    )

    return ctx.report()


object_model = build_object_model()

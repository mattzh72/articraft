from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sealed_manipulator_joystick_core")

    tray_mat = model.material("anodized_black", rgba=(0.015, 0.017, 0.020, 1.0))
    rim_mat = model.material("satin_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    seal_mat = model.material("matte_rubber_seal", rgba=(0.005, 0.005, 0.004, 1.0))
    bearing_mat = model.material("dark_bearing_bronze", rgba=(0.38, 0.28, 0.16, 1.0))
    yoke_mat = model.material("brushed_steel_yoke", rgba=(0.55, 0.58, 0.60, 1.0))
    cradle_mat = model.material("machined_inner_cradle", rgba=(0.43, 0.45, 0.47, 1.0))
    stick_mat = model.material("polished_stick", rgba=(0.72, 0.74, 0.76, 1.0))
    fastener_mat = model.material("black_oxide_fasteners", rgba=(0.02, 0.02, 0.018, 1.0))

    tray = model.part("tray")
    tray.visual(
        Box((0.42, 0.42, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=tray_mat,
        name="mounting_plate",
    )
    tray.visual(
        Box((0.42, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, 0.1925, 0.043)),
        material=rim_mat,
        name="front_rim",
    )
    tray.visual(
        Box((0.42, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, -0.1925, 0.043)),
        material=rim_mat,
        name="rear_rim",
    )
    tray.visual(
        Box((0.035, 0.35, 0.026)),
        origin=Origin(xyz=(0.1925, 0.0, 0.043)),
        material=rim_mat,
        name="side_rim_0",
    )
    tray.visual(
        Box((0.035, 0.35, 0.026)),
        origin=Origin(xyz=(-0.1925, 0.0, 0.043)),
        material=rim_mat,
        name="side_rim_1",
    )
    tray.visual(
        Cylinder(radius=0.118, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=seal_mat,
        name="sealed_center_plate",
    )
    tray.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=rim_mat,
        name="raised_seal_lip",
    )

    # Fixed support towers hold the outer gimbal trunnions above the sealed tray.
    tray.visual(
        Box((0.056, 0.122, 0.112)),
        origin=Origin(xyz=(0.200, 0.0, 0.086)),
        material=rim_mat,
        name="outer_bearing_block_0",
    )
    tray.visual(
        Box((0.056, 0.122, 0.112)),
        origin=Origin(xyz=(-0.200, 0.0, 0.086)),
        material=rim_mat,
        name="outer_bearing_block_1",
    )
    tray.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.159, 0.0, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_mat,
        name="outer_bushing_0",
    )
    tray.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(-0.159, 0.0, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_mat,
        name="outer_bushing_1",
    )

    for i, (x, y) in enumerate(
        ((0.155, 0.155), (-0.155, 0.155), (-0.155, -0.155), (0.155, -0.155))
    ):
        tray.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(x, y, 0.033)),
            material=fastener_mat,
            name=f"corner_screw_{i}",
        )

    outer_yoke = model.part("outer_yoke")
    # Broad rectangular yoke frame, leaving the center open for the inner cradle.
    outer_yoke.visual(
        Box((0.260, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, 0.114, -0.005)),
        material=yoke_mat,
        name="front_crossbar",
    )
    outer_yoke.visual(
        Box((0.260, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, -0.114, -0.005)),
        material=yoke_mat,
        name="rear_crossbar",
    )
    outer_yoke.visual(
        Box((0.030, 0.230, 0.040)),
        origin=Origin(xyz=(0.110, 0.0, -0.002)),
        material=yoke_mat,
        name="side_rail_0",
    )
    outer_yoke.visual(
        Box((0.030, 0.230, 0.040)),
        origin=Origin(xyz=(-0.110, 0.0, -0.002)),
        material=yoke_mat,
        name="side_rail_1",
    )
    outer_yoke.visual(
        Cylinder(radius=0.019, length=0.046),
        origin=Origin(xyz=(0.147, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yoke_mat,
        name="outer_trunnion_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.019, length=0.046),
        origin=Origin(xyz=(-0.147, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yoke_mat,
        name="outer_trunnion_1",
    )
    # Bearings for the inner cradle protrude inward from the broad yoke.
    outer_yoke.visual(
        Cylinder(radius=0.029, length=0.026),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="inner_bushing_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.029, length=0.026),
        origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="inner_bushing_1",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.015, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cradle_mat,
        name="inner_pivot_pin",
    )
    inner_cradle.visual(
        Box((0.132, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=cradle_mat,
        name="saddle_bridge",
    )
    inner_cradle.visual(
        Box((0.024, 0.074, 0.070)),
        origin=Origin(xyz=(0.055, 0.0, -0.004)),
        material=cradle_mat,
        name="cradle_cheek_0",
    )
    inner_cradle.visual(
        Box((0.024, 0.074, 0.070)),
        origin=Origin(xyz=(-0.055, 0.0, -0.004)),
        material=cradle_mat,
        name="cradle_cheek_1",
    )
    inner_cradle.visual(
        Cylinder(radius=0.046, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=cradle_mat,
        name="center_hub",
    )
    inner_cradle.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=bearing_mat,
        name="stick_collar",
    )
    inner_cradle.visual(
        Cylinder(radius=0.023, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=stick_mat,
        name="stick_shaft",
    )
    inner_cradle.visual(
        Sphere(radius=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        material=stick_mat,
        name="stick_tip",
    )

    model.articulation(
        "tray_to_outer_yoke",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "outer_yoke_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("tray")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_cradle = object_model.get_part("inner_cradle")
    outer_joint = object_model.get_articulation("tray_to_outer_yoke")
    inner_joint = object_model.get_articulation("outer_yoke_to_inner_cradle")

    # The four scoped overlaps are intentional captured trunnion fits: the
    # shafts are locally seated inside simplified solid bushing proxies.
    ctx.allow_overlap(
        tray,
        outer_yoke,
        elem_a="outer_bushing_0",
        elem_b="outer_trunnion_0",
        reason="Outer yoke trunnion is intentionally captured inside the tray bushing.",
    )
    ctx.allow_overlap(
        tray,
        outer_yoke,
        elem_a="outer_bushing_1",
        elem_b="outer_trunnion_1",
        reason="Outer yoke trunnion is intentionally captured inside the tray bushing.",
    )
    ctx.allow_overlap(
        outer_yoke,
        inner_cradle,
        elem_a="inner_bushing_0",
        elem_b="inner_pivot_pin",
        reason="Inner cradle pivot pin is intentionally captured inside the yoke bushing.",
    )
    ctx.allow_overlap(
        outer_yoke,
        inner_cradle,
        elem_a="inner_bushing_1",
        elem_b="inner_pivot_pin",
        reason="Inner cradle pivot pin is intentionally captured inside the yoke bushing.",
    )

    ctx.expect_within(
        outer_yoke,
        tray,
        axes="yz",
        inner_elem="outer_trunnion_0",
        outer_elem="outer_bushing_0",
        margin=0.001,
        name="outer trunnion 0 centered in tray bushing",
    )
    ctx.expect_within(
        outer_yoke,
        tray,
        axes="yz",
        inner_elem="outer_trunnion_1",
        outer_elem="outer_bushing_1",
        margin=0.001,
        name="outer trunnion 1 centered in tray bushing",
    )
    ctx.expect_overlap(
        outer_yoke,
        tray,
        axes="x",
        elem_a="outer_trunnion_0",
        elem_b="outer_bushing_0",
        min_overlap=0.010,
        name="outer trunnion 0 retained in bushing",
    )
    ctx.expect_overlap(
        outer_yoke,
        tray,
        axes="x",
        elem_a="outer_trunnion_1",
        elem_b="outer_bushing_1",
        min_overlap=0.010,
        name="outer trunnion 1 retained in bushing",
    )
    ctx.expect_within(
        inner_cradle,
        outer_yoke,
        axes="xz",
        inner_elem="inner_pivot_pin",
        outer_elem="inner_bushing_0",
        margin=0.001,
        name="inner pin centered in front yoke bushing",
    )
    ctx.expect_within(
        inner_cradle,
        outer_yoke,
        axes="xz",
        inner_elem="inner_pivot_pin",
        outer_elem="inner_bushing_1",
        margin=0.001,
        name="inner pin centered in rear yoke bushing",
    )
    ctx.expect_overlap(
        inner_cradle,
        outer_yoke,
        axes="y",
        elem_a="inner_pivot_pin",
        elem_b="inner_bushing_0",
        min_overlap=0.010,
        name="inner pin retained in front bushing",
    )
    ctx.expect_overlap(
        inner_cradle,
        outer_yoke,
        axes="y",
        elem_a="inner_pivot_pin",
        elem_b="inner_bushing_1",
        min_overlap=0.010,
        name="inner pin retained in rear bushing",
    )

    outer_axis = tuple(outer_joint.axis or ())
    inner_axis = tuple(inner_joint.axis or ())
    dot = sum(a * b for a, b in zip(outer_axis, inner_axis))
    ctx.check(
        "gimbal axes are orthogonal",
        len(outer_axis) == 3 and len(inner_axis) == 3 and abs(dot) < 1.0e-6,
        details=f"outer_axis={outer_axis}, inner_axis={inner_axis}, dot={dot}",
    )

    def aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_tip = aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="stick_tip"))
    with ctx.pose({outer_joint: 0.45}):
        tilted_tip = aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="stick_tip"))
    ctx.check(
        "outer yoke tilts stick about x axis",
        tilted_tip[1] < rest_tip[1] - 0.09 and tilted_tip[2] < rest_tip[2] - 0.010,
        details=f"rest_tip={rest_tip}, tilted_tip={tilted_tip}",
    )

    with ctx.pose({inner_joint: 0.45}):
        side_tip = aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="stick_tip"))
    ctx.check(
        "inner cradle tilts stick about y axis",
        side_tip[0] > rest_tip[0] + 0.09 and side_tip[2] < rest_tip[2] - 0.010,
        details=f"rest_tip={rest_tip}, side_tip={side_tip}",
    )

    return ctx.report()


object_model = build_object_model()

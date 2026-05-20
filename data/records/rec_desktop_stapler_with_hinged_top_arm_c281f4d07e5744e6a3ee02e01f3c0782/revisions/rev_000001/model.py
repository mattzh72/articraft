from __future__ import annotations

import cadquery as cq
from math import pi
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.036
ARM_WIDTH = 0.034
HINGE_Z = 0.033
ARM_OPEN_ANGLE = 0.95
TRAY_TRAVEL = 0.072
FOLLOWER_TRAVEL = 0.082
ANVIL_ROTATION = pi / 2.0


def _base_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.016, 0.000),
                (0.150, 0.000),
                (0.150, 0.004),
                (0.143, 0.006),
                (0.000, 0.006),
                (-0.016, 0.007),
            ]
        )
        .close()
        .extrude(BASE_WIDTH / 2.0, both=True)
    )
    side_cheek = cq.Workplane("XY").box(0.022, 0.005, 0.024).translate((-0.003, 0.0, 0.018))
    right_cheek = side_cheek.translate((0.0, 0.0205, 0.0))
    left_cheek = side_cheek.translate((0.0, -0.0205, 0.0))
    hinge_pin = cq.Workplane("XZ").circle(0.0017).extrude(0.023, both=True).translate((-0.003, 0.0, HINGE_Z))
    right_pin_web = cq.Workplane("XY").box(0.004, 0.005, 0.0035).translate((-0.003, 0.0205, 0.03175))
    left_pin_web = cq.Workplane("XY").box(0.004, 0.005, 0.0035).translate((-0.003, -0.0205, 0.03175))
    anvil_pocket = cq.Workplane("XY").box(0.026, 0.022, 0.0035).translate((0.124, 0.0, 0.00175))
    anvil_pivot = cq.Workplane("XY").circle(0.0014).extrude(0.0035).translate((0.124, 0.0, 0.0))
    return (
        base_plate.union(right_cheek)
        .union(left_cheek)
        .union(right_pin_web)
        .union(left_pin_web)
        .union(hinge_pin)
        .cut(anvil_pocket)
        .union(anvil_pivot)
    )


def _arm_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.010, 0.0025),
                (-0.010, 0.0075),
                (0.020, 0.0105),
                (0.080, 0.0160),
                (0.132, 0.0185),
                (0.149, 0.0105),
                (0.147, -0.0135),
                (0.036, -0.0150),
                (0.022, -0.0105),
                (0.008, 0.0015),
            ]
        )
        .close()
        .extrude(ARM_WIDTH / 2.0, both=True)
    )
    right_lug = cq.Workplane("XZ").circle(0.003).extrude(0.004, both=True).translate((-0.003, 0.0205, 0.0))
    left_lug = cq.Workplane("XZ").circle(0.003).extrude(0.004, both=True).translate((-0.003, -0.0205, 0.0))
    cavity = cq.Workplane("XY").box(0.164, 0.024, 0.018).translate((0.068, 0.0, -0.005))
    paper_throat = cq.Workplane("XY").box(0.020, 0.010, 0.008).translate((0.136, 0.0, -0.011))
    return outer.union(right_lug).union(left_lug).cut(cavity).cut(paper_throat)


def _tray_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.140, 0.0218, 0.010).translate((0.070, 0.0, 0.005))
    rear_cap = cq.Workplane("XY").box(0.016, 0.0220, 0.011).translate((-0.004, 0.0, 0.0055))
    left_rail = cq.Workplane("XY").box(0.102, 0.0014, 0.003).translate((0.072, -0.0113, 0.0072))
    right_rail = cq.Workplane("XY").box(0.102, 0.0014, 0.003).translate((0.072, 0.0113, 0.0072))
    channel_cut = cq.Workplane("XY").box(0.134, 0.0182, 0.0082).translate((0.070, 0.0, 0.0059))
    return body.union(rear_cap).union(left_rail).union(right_rail).cut(channel_cut)


def _follower_shape() -> cq.Workplane:
    pusher = cq.Workplane("XY").box(0.007, 0.0176, 0.0065).translate((0.0, 0.0, 0.00325))
    left_shoe = cq.Workplane("XY").box(0.007, 0.0006, 0.0045).translate((0.0, -0.0088, 0.00325))
    right_shoe = cq.Workplane("XY").box(0.007, 0.0006, 0.0045).translate((0.0, 0.0088, 0.00325))
    spring_rod = cq.Workplane("YZ").circle(0.0013).extrude(0.052).translate((-0.052, 0.0, 0.00325))
    rear_tab = cq.Workplane("XY").box(0.005, 0.012, 0.010).translate((-0.056, 0.0, 0.005))
    rear_collar = cq.Workplane("XY").box(0.009, 0.005, 0.009).translate((-0.049, 0.0, 0.0045))
    return pusher.union(left_shoe).union(right_shoe).union(spring_rod).union(rear_tab).union(rear_collar)


def _anvil_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.018, 0.012, 0.0022)
    pivot_hub = cq.Workplane("XY").circle(0.0018).extrude(0.0016, both=True)
    selector_tab = cq.Workplane("XY").box(0.006, 0.004, 0.0022).translate((-0.004, 0.005, 0.0))
    return plate.union(pivot_hub).union(selector_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("black_enamel", rgba=(0.15, 0.16, 0.18, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base"), material="steel", name="base_shell")

    arm = model.part("arm")
    arm.visual(mesh_from_cadquery(_arm_shape(), "arm"), material="black_enamel", name="arm_shell")

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_shape(), "tray"), material="steel", name="tray_shell")

    follower = model.part("follower")
    follower.visual(mesh_from_cadquery(_follower_shape(), "follower"), material="steel", name="follower_shell")

    anvil = model.part("anvil")
    anvil.visual(mesh_from_cadquery(_anvil_shape(), "anvil"), material="steel", name="anvil_shell")

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=ARM_OPEN_ANGLE, effort=20.0, velocity=3.0),
    )
    model.articulation(
        "arm_to_tray",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tray,
        origin=Origin(xyz=(0.004, 0.0, -0.0145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=15.0, velocity=0.25),
    )
    model.articulation(
        "tray_to_follower",
        ArticulationType.PRISMATIC,
        parent=tray,
        child=follower,
        origin=Origin(xyz=(0.114, 0.0, 0.0018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FOLLOWER_TRAVEL, effort=6.0, velocity=0.20),
    )
    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(0.124, 0.0, 0.00175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=ANVIL_ROTATION, effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    arm = object_model.get_part("arm")
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    follower = object_model.get_part("follower")
    anvil = object_model.get_part("anvil")
    arm_hinge = object_model.get_articulation("base_to_arm")
    tray_slide = object_model.get_articulation("arm_to_tray")
    follower_slide = object_model.get_articulation("tray_to_follower")
    anvil_pivot = object_model.get_articulation("base_to_anvil")

    ctx.allow_overlap(
        base,
        arm,
        elem_a="base_shell",
        elem_b="arm_shell",
        reason="The rear hinge pin intentionally passes through the arm barrel region.",
    )
    ctx.allow_overlap(
        arm,
        tray,
        elem_a="arm_shell",
        elem_b="tray_shell",
        reason="The staple tray is intentionally represented as sliding inside the upper-body sleeve proxy.",
    )
    ctx.allow_overlap(
        tray,
        follower,
        elem_a="tray_shell",
        elem_b="follower_shell",
        reason="The spring follower is intentionally represented as sliding inside the tray channel proxy.",
    )
    ctx.allow_overlap(
        base,
        anvil,
        elem_a="base_shell",
        elem_b="anvil_shell",
        reason="The clincher anvil rotates around the nose pivot boss captured in the base pocket.",
    )

    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_within(
            tray,
            arm,
            axes="yz",
            margin=0.0015,
            name="tray stays centered inside the upper arm",
        )

    tray_rest = None
    tray_open = None
    with ctx.pose({tray_slide: 0.0}):
        tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        tray_open = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            min_overlap=0.055,
            name="opened tray remains captured by the arm",
        )
    ctx.check(
        "tray slides rearward for loading",
        tray_rest is not None and tray_open is not None and tray_open[0] < tray_rest[0] - 0.05,
        details=f"rest={tray_rest}, open={tray_open}",
    )

    with ctx.pose({follower_slide: 0.0}):
        ctx.expect_within(
            follower,
            tray,
            axes="y",
            margin=0.001,
            name="follower stays guided inside the tray",
        )

    follower_rest = None
    follower_retracted = None
    with ctx.pose({follower_slide: 0.0}):
        follower_rest = ctx.part_world_position(follower)
    with ctx.pose({follower_slide: FOLLOWER_TRAVEL}):
        follower_retracted = ctx.part_world_position(follower)
        ctx.expect_overlap(
            follower,
            tray,
            axes="x",
            min_overlap=0.010,
            name="follower remains retained by the tray",
        )
    ctx.check(
        "follower pulls back from the staple line",
        follower_rest is not None
        and follower_retracted is not None
        and follower_retracted[0] < follower_rest[0] - 0.05,
        details=f"rest={follower_rest}, pulled={follower_retracted}",
    )

    with ctx.pose({anvil_pivot: 0.0}):
        ctx.expect_within(
            anvil,
            base,
            axes="xy",
            margin=0.006,
            name="anvil stays within the nose pocket",
        )
    with ctx.pose({anvil_pivot: ANVIL_ROTATION}):
        ctx.expect_within(
            anvil,
            base,
            axes="xy",
            margin=0.006,
            name="rotated anvil remains within the nose pocket",
        )

    anvil_closed = None
    anvil_rotated = None
    with ctx.pose({anvil_pivot: 0.0}):
        anvil_closed = ctx.part_world_aabb(anvil)
    with ctx.pose({anvil_pivot: ANVIL_ROTATION}):
        anvil_rotated = ctx.part_world_aabb(anvil)
    ctx.check(
        "anvil selector rotates in plane",
        anvil_closed is not None
        and anvil_rotated is not None
        and (anvil_closed[1][0] - anvil_closed[0][0]) > (anvil_rotated[1][0] - anvil_rotated[0][0])
        and (anvil_rotated[1][1] - anvil_rotated[0][1]) > (anvil_closed[1][1] - anvil_closed[0][1]),
        details=f"closed={anvil_closed}, rotated={anvil_rotated}",
    )

    closed_aabb = None
    open_aabb = None
    with ctx.pose({arm_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(arm)
    with ctx.pose({arm_hinge: ARM_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(arm)
    ctx.check(
        "arm opens upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

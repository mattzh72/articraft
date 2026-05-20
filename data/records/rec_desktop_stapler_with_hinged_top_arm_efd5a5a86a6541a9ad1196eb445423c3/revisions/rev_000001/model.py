from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.172
BASE_W = 0.046
BASE_BOTTOM_Z = -0.044
HINGE_Z = 0.0

ARM_LEN = 0.170
ARM_W = 0.031
ARM_OPEN = math.radians(58.0)
TRAY_TRAVEL = 0.050
FOLLOWER_TRAVEL = 0.090
GUIDE_TRAVEL = 0.024


def _soft_fillet(shape, selector: str, radius: float):
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _base_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.006, BASE_BOTTOM_Z),
                (0.150, BASE_BOTTOM_Z),
                (0.168, -0.042),
                (0.174, -0.036),
                (0.174, -0.026),
                (0.164, -0.022),
                (0.120, -0.022),
                (0.040, -0.026),
                (-0.004, -0.030),
            ]
        )
        .close()
        .extrude(BASE_W / 2.0, both=True)
    )
    body = _soft_fillet(body, "|Y", 0.0035)

    front_pad = cq.Workplane("XY").box(0.022, 0.024, 0.007).translate((0.149, 0.0, -0.026))
    body = body.union(front_pad)

    guide_groove = cq.Workplane("XY").box(0.040, 0.014, 0.005).translate((0.146, 0.0, -0.0415))
    body = body.cut(guide_groove)

    top_relief = (
        cq.Workplane("XY")
        .box(0.100, 0.020, 0.008)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 6.0)
        .translate((0.094, 0.0, -0.019))
    )
    body = body.cut(top_relief)

    return _soft_fillet(body, "|X", 0.0015)


def _hinge_yoke_shape() -> cq.Workplane:
    yoke = cq.Workplane("XY").box(0.022, 0.028, 0.020).translate((0.002, 0.0, -0.021))
    for y_pos in (-0.019, 0.019):
        ear = cq.Workplane("XY").box(0.018, 0.006, 0.038).translate((0.003, y_pos, -0.019))
        ear = _soft_fillet(ear, "|Z", 0.0018)
        yoke = yoke.union(ear)
    return yoke


def _arm_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.006, -0.016),
                (0.154, -0.012),
                (0.170, -0.009),
                (0.170, -0.001),
                (0.148, 0.0004),
                (0.032, -0.0015),
                (0.010, -0.0052),
            ]
        )
        .close()
        .extrude(ARM_W / 2.0, both=True)
    )
    outer = _soft_fillet(outer, "|Y", 0.0045)

    rear_boss = cq.Workplane("XY").box(0.020, 0.024, 0.008).translate((0.010, 0.0, -0.007))
    outer = outer.union(rear_boss)

    inner = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, -0.0134),
                (0.148, -0.0108),
                (0.154, -0.0088),
                (0.154, -0.0050),
                (0.142, -0.0042),
                (0.028, -0.0042),
                (0.018, -0.0063),
            ]
        )
        .close()
        .extrude(0.023 / 2.0, both=True)
    )
    shell = outer.cut(inner)

    bottom_opening = cq.Workplane("XY").box(0.138, 0.0225, 0.014).translate((0.086, 0.0, -0.015))
    shell = shell.cut(bottom_opening)

    nose_beak = cq.Workplane("XY").box(0.022, 0.020, 0.003).translate((0.159, 0.0, -0.0115))
    shell = shell.union(nose_beak)
    return _soft_fillet(shell, "|X", 0.0012)

def _arm_barrel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(0.0072)
        .extrude(0.028 / 2.0, both=True)
        .translate((0.0, 0.0, 0.0))
    )


def _tray_shape() -> cq.Workplane:
    floor = cq.Workplane("XY").box(0.146, 0.019, 0.0016).translate((0.067, 0.0, 0.0008))
    left_wall = cq.Workplane("XY").box(0.138, 0.0022, 0.0046).translate((0.069, 0.0084, 0.0023))
    right_wall = cq.Workplane("XY").box(0.138, 0.0022, 0.0046).translate((0.069, -0.0084, 0.0023))
    front_rail = cq.Workplane("XY").box(0.010, 0.015, 0.0034).translate((0.140, 0.0, 0.0017))
    rear_bridge = cq.Workplane("XY").box(0.006, 0.015, 0.0040).translate((0.001, 0.0, 0.0020))
    pull_tab = cq.Workplane("XY").box(0.018, 0.020, 0.0018).translate((-0.010, 0.0, 0.0022))
    tray = floor.union(left_wall).union(right_wall).union(front_rail).union(rear_bridge).union(pull_tab)
    return _soft_fillet(tray, "|X", 0.0008)


def _follower_shape() -> cq.Workplane:
    block = cq.Workplane("XY").box(0.018, 0.012, 0.0032).translate((0.009, 0.0, 0.0016))
    thumb = cq.Workplane("XY").box(0.006, 0.016, 0.0044).translate((0.003, 0.0, 0.0022))
    shoe = cq.Workplane("XY").box(0.010, 0.008, 0.0014).translate((0.013, 0.0, 0.0007))
    return _soft_fillet(block.union(thumb).union(shoe), "|Z", 0.0008)


def _depth_guide_shape() -> cq.Workplane:
    tongue = cq.Workplane("XY").box(0.028, 0.010, 0.0032).translate((0.014, 0.0, 0.0016))
    fence = cq.Workplane("XY").box(0.004, 0.018, 0.010).translate((0.030, 0.0, -0.005))
    lower_lip = cq.Workplane("XY").box(0.010, 0.014, 0.002).translate((0.026, 0.0, -0.011))
    guide = tongue.union(fence).union(lower_lip)
    return _soft_fillet(guide, "|Y", 0.0008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_stapler")

    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "stapler_base"),
        material=graphite,
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_hinge_yoke_shape(), "stapler_yoke"),
        material=graphite,
        name="hinge_yoke",
    )
    base.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.152, 0.034, 0.002).translate((0.085, 0.0, -0.045)),
            "stapler_pad",
        ),
        material=rubber,
        name="base_pad",
    )
    base.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.028, 0.018, 0.003).translate((0.151, 0.0, -0.021)),
            "stapler_anvil",
        ),
        material=satin_steel,
        name="anvil",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shell_shape(), "stapler_arm"),
        material=graphite,
        name="arm_shell",
    )
    arm.visual(
        mesh_from_cadquery(_arm_barrel_shape(), "stapler_barrel"),
        material=satin_steel,
        name="hinge_barrel",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "stapler_tray"),
        material=satin_steel,
        name="tray_channel",
    )

    follower = model.part("follower")
    follower.visual(
        mesh_from_cadquery(_follower_shape(), "stapler_follower"),
        material=graphite,
        name="follower_block",
    )

    depth_guide = model.part("depth_guide")
    depth_guide.visual(
        mesh_from_cadquery(_depth_guide_shape(), "stapler_depth_guide"),
        material=satin_steel,
        name="guide_fence",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=ARM_OPEN,
            effort=18.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "arm_to_tray",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tray,
        origin=Origin(xyz=(0.020, 0.0, -0.0145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TRAY_TRAVEL,
            effort=6.0,
            velocity=0.14,
        ),
    )
    model.articulation(
        "tray_to_follower",
        ArticulationType.PRISMATIC,
        parent=tray,
        child=follower,
        origin=Origin(xyz=(0.006, 0.0, 0.0016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FOLLOWER_TRAVEL,
            effort=3.0,
            velocity=0.10,
        ),
    )
    model.articulation(
        "base_to_depth_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=depth_guide,
        origin=Origin(xyz=(0.128, 0.0, -0.0420)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=GUIDE_TRAVEL,
            effort=2.0,
            velocity=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    follower = object_model.get_part("follower")
    depth_guide = object_model.get_part("depth_guide")
    hinge = object_model.get_articulation("base_to_arm")
    tray_slide = object_model.get_articulation("arm_to_tray")
    follower_slide = object_model.get_articulation("tray_to_follower")
    guide_slide = object_model.get_articulation("base_to_depth_guide")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            arm,
            base,
            axes="xy",
            elem_a="arm_shell",
            elem_b="base_body",
            min_overlap=0.020,
            name="arm covers base footprint when closed",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="anvil",
            min_gap=0.002,
            max_gap=0.024,
            name="closed arm sits just above anvil",
        )
        ctx.expect_within(
            tray,
            arm,
            axes="yz",
            inner_elem="tray_channel",
            outer_elem="arm_shell",
            margin=0.0015,
            name="tray stays under the arm shell",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            elem_a="tray_channel",
            elem_b="arm_shell",
            min_overlap=0.105,
            name="closed tray remains deeply inserted",
        )
        ctx.expect_within(
            depth_guide,
            base,
            axes="y",
            inner_elem="guide_fence",
            outer_elem="base_body",
            margin=0.002,
            name="depth guide stays centered under the base",
        )

    closed_shell = ctx.part_element_world_aabb(arm, elem="arm_shell")
    tray_rest = ctx.part_world_position(tray)
    guide_rest = ctx.part_world_position(depth_guide)
    follower_rest = ctx.part_world_position(follower)

    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_within(
            tray,
            arm,
            axes="yz",
            inner_elem="tray_channel",
            outer_elem="arm_shell",
            margin=0.0015,
            name="extended tray stays guided under the arm shell",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            elem_a="tray_channel",
            elem_b="arm_shell",
            min_overlap=0.055,
            name="extended tray retains insertion",
        )
        tray_extended = ctx.part_world_position(tray)

    with ctx.pose({follower_slide: FOLLOWER_TRAVEL}):
        ctx.expect_within(
            follower,
            tray,
            axes="yz",
            inner_elem="follower_block",
            outer_elem="tray_channel",
            margin=0.0015,
            name="follower stays guided inside tray channel",
        )
        ctx.expect_overlap(
            follower,
            tray,
            axes="x",
            elem_a="follower_block",
            elem_b="tray_channel",
            min_overlap=0.010,
            name="follower remains engaged in tray channel",
        )
        follower_extended = ctx.part_world_position(follower)

    with ctx.pose({guide_slide: GUIDE_TRAVEL}):
        ctx.expect_within(
            depth_guide,
            base,
            axes="y",
            inner_elem="guide_fence",
            outer_elem="base_body",
            margin=0.002,
            name="extended depth guide stays centered",
        )
        ctx.expect_overlap(
            depth_guide,
            base,
            axes="x",
            elem_a="guide_fence",
            elem_b="base_body",
            min_overlap=0.008,
            name="depth guide remains captured by the base",
        )
        guide_extended = ctx.part_world_position(depth_guide)

    with ctx.pose({hinge: ARM_OPEN}):
        open_shell = ctx.part_element_world_aabb(arm, elem="arm_shell")

    ctx.check(
        "arm opens upward",
        closed_shell is not None
        and open_shell is not None
        and float(open_shell[1][2]) > float(closed_shell[1][2]) + 0.040,
        details=f"closed={closed_shell}, open={open_shell}",
    )
    ctx.check(
        "tray pulls rearward",
        tray_rest is not None
        and tray_extended is not None
        and float(tray_extended[0]) < float(tray_rest[0]) - 0.030,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )
    ctx.check(
        "follower advances forward",
        follower_rest is not None
        and follower_extended is not None
        and float(follower_extended[0]) > float(follower_rest[0]) + 0.050,
        details=f"rest={follower_rest}, extended={follower_extended}",
    )
    ctx.check(
        "depth guide extends forward",
        guide_rest is not None
        and guide_extended is not None
        and float(guide_extended[0]) > float(guide_rest[0]) + 0.015,
        details=f"rest={guide_rest}, extended={guide_extended}",
    )

    return ctx.report()


object_model = build_object_model()

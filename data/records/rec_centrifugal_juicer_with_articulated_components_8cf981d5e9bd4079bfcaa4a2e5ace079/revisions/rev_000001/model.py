from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL = Material("brushed_stainless_steel", rgba=(0.72, 0.72, 0.68, 1.0))
DARK_STEEL = Material("dark_perforated_steel", rgba=(0.24, 0.25, 0.25, 1.0))
BLACK = Material("black_plastic", rgba=(0.02, 0.022, 0.025, 1.0))
RUBBER = Material("matte_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
CLEAR = Material("clear_polycarbonate", rgba=(0.72, 0.90, 1.00, 0.34))
SMOKE = Material("smoked_clear_lid", rgba=(0.58, 0.72, 0.82, 0.46))


def _rounded_box(sx: float, sy: float, sz: float, radius: float) -> cq.Workplane:
    """Centered rounded rectangular solid used for appliance housings."""
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .edges("|Z")
        .fillet(radius)
    )


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Open-ended cylindrical tube, authored with its bottom on z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _tray_shape() -> cq.Workplane:
    """A shallow drip tray with a recessed basin and raised rim."""
    length, width, height = 0.145, 0.120, 0.026
    rim, floor = 0.012, 0.006
    outer = (
        cq.Workplane("XY")
        .box(length, width, height)
        .translate((length / 2.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )
    cutter = (
        cq.Workplane("XY")
        .box(length - 2.0 * rim, width - 2.0 * rim, height)
        .translate((length / 2.0, 0.0, floor + height / 2.0))
    )
    return outer.cut(cutter)


def _basket_shape() -> cq.Workplane:
    """Conical strainer basket with rim, hub, and radial spokes."""
    outer = (
        cq.Workplane("XY")
        .circle(0.045)
        .workplane(offset=0.096)
        .circle(0.088)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.024))
    )
    inner = (
        cq.Workplane("XY")
        .circle(0.038)
        .workplane(offset=0.100)
        .circle(0.081)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.023))
    )
    shell = outer.cut(inner)
    top_rim = cq.Workplane("XY").circle(0.092).circle(0.078).extrude(0.008).translate((0.0, 0.0, 0.120))
    bottom_rim = cq.Workplane("XY").circle(0.049).circle(0.024).extrude(0.008).translate((0.0, 0.0, 0.023))
    hub = cq.Workplane("XY").circle(0.022).extrude(0.050)
    spoke_x = cq.Workplane("XY").box(0.096, 0.008, 0.007).translate((0.0, 0.0, 0.048))
    spoke_y = cq.Workplane("XY").box(0.008, 0.096, 0.007).translate((0.0, 0.0, 0.048))
    return shell.union(top_rim).union(bottom_rim).union(hub).union(spoke_x).union(spoke_y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_steel_juicer")

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_box(0.420, 0.300, 0.200, 0.026), "deep_stainless_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=STEEL,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.135, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.207)),
        material=BLACK,
        name="top_plinth",
    )
    base.visual(
        Box((0.044, 0.070, 0.048)),
        origin=Origin(xyz=(0.171, 0.0, 0.224)),
        material=STEEL,
        name="spout_neck",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.106),
        origin=Origin(xyz=(0.224, 0.0, 0.238), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="front_spout",
    )
    base.visual(
        Box((0.036, 0.030, 0.010)),
        origin=Origin(xyz=(0.283, 0.0, 0.232)),
        material=STEEL,
        name="spout_lip",
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(_annular_cylinder(0.125, 0.106, 0.155), "clear_upper_chamber"),
        material=CLEAR,
        name="chamber_wall",
    )
    chamber.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, 0.145, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pivot_boss_0",
    )
    chamber.visual(
        Box((0.054, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.128, 0.095)),
        material=STEEL,
        name="pivot_support_0",
    )
    chamber.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, -0.145, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pivot_boss_1",
    )
    chamber.visual(
        Box((0.054, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.128, 0.095)),
        material=STEEL,
        name="pivot_support_1",
    )
    chamber.visual(
        Box((0.030, 0.020, 0.055)),
        origin=Origin(xyz=(-0.152, 0.055, 0.176)),
        material=STEEL,
        name="hinge_post_0",
    )
    chamber.visual(
        Box((0.030, 0.020, 0.055)),
        origin=Origin(xyz=(-0.152, -0.055, 0.176)),
        material=STEEL,
        name="hinge_post_1",
    )
    chamber.visual(
        Box((0.040, 0.018, 0.020)),
        origin=Origin(xyz=(-0.135, 0.055, 0.140)),
        material=STEEL,
        name="hinge_foot_0",
    )
    chamber.visual(
        Box((0.040, 0.018, 0.020)),
        origin=Origin(xyz=(-0.135, -0.055, 0.140)),
        material=STEEL,
        name="hinge_foot_1",
    )
    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base,
        child=chamber,
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "rotating_strainer_basket"),
        material=DARK_STEEL,
        name="basket_shell",
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_annular_cylinder(0.130, 0.030, 0.022), "clear_lid_with_chute_hole"),
        origin=Origin(xyz=(0.130, 0.0, -0.005)),
        material=SMOKE,
        name="lid_panel",
    )
    lid.visual(
        mesh_from_cadquery(_annular_cylinder(0.137, 0.126, 0.014), "stainless_lid_rim"),
        origin=Origin(xyz=(0.130, 0.0, -0.001)),
        material=STEEL,
        name="lid_rim",
    )
    lid.visual(
        mesh_from_cadquery(_annular_cylinder(0.037, 0.027, 0.145), "central_feed_chute"),
        origin=Origin(xyz=(0.130, 0.0, 0.017)),
        material=CLEAR,
        name="feed_chute",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="hinge_barrel",
    )
    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(-0.130, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    pusher = model.part("feed_pusher")
    pusher.visual(
        Cylinder(radius=0.022, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=BLACK,
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.045, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=STEEL,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=RUBBER,
        name="thumb_pad",
    )
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.130, 0.0, 0.152)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.055),
    )

    tray = model.part("drip_tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "sliding_drip_tray"),
        material=BLACK,
        name="tray_pan",
    )
    tray.visual(
        Box((0.118, 0.010, 0.006)),
        origin=Origin(xyz=(0.073, 0.033, 0.009)),
        material=RUBBER,
        name="drip_groove_0",
    )
    tray.visual(
        Box((0.118, 0.010, 0.006)),
        origin=Origin(xyz=(0.073, -0.033, 0.009)),
        material=RUBBER,
        name="drip_groove_1",
    )
    model.articulation(
        "base_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.210, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.085),
    )

    clamp_0 = model.part("side_clamp_0")
    clamp_0.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pivot_pin",
    )
    clamp_0.visual(
        Box((0.018, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, 0.016, 0.045)),
        material=STEEL,
        name="clamp_arm",
    )
    clamp_0.visual(
        Box((0.052, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.096)),
        material=STEEL,
        name="lid_hook",
    )
    model.articulation(
        "chamber_to_side_clamp_0",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=clamp_0,
        origin=Origin(xyz=(0.0, 0.145, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.20, upper=0.0),
    )

    clamp_1 = model.part("side_clamp_1")
    clamp_1.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pivot_pin",
    )
    clamp_1.visual(
        Box((0.018, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, -0.016, 0.045)),
        material=STEEL,
        name="clamp_arm",
    )
    clamp_1.visual(
        Box((0.052, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.096)),
        material=STEEL,
        name="lid_hook",
    )
    model.articulation(
        "chamber_to_side_clamp_1",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=clamp_1,
        origin=Origin(xyz=(0.0, -0.145, 0.095)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.20, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    chamber = object_model.get_part("chamber")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    tray = object_model.get_part("drip_tray")
    clamp_0 = object_model.get_part("side_clamp_0")
    clamp_1 = object_model.get_part("side_clamp_1")

    lid_hinge = object_model.get_articulation("chamber_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_feed_pusher")
    tray_slide = object_model.get_articulation("base_to_drip_tray")
    clamp_joint_0 = object_model.get_articulation("chamber_to_side_clamp_0")
    clamp_joint_1 = object_model.get_articulation("chamber_to_side_clamp_1")

    ctx.allow_overlap(
        chamber,
        clamp_0,
        elem_a="pivot_boss_0",
        elem_b="pivot_pin",
        reason="The clamp pivot pin is intentionally captured inside the side boss.",
    )
    ctx.allow_overlap(
        chamber,
        clamp_1,
        elem_a="pivot_boss_1",
        elem_b="pivot_pin",
        reason="The clamp pivot pin is intentionally captured inside the side boss.",
    )
    ctx.expect_overlap(
        chamber,
        clamp_0,
        axes="xyz",
        elem_a="pivot_boss_0",
        elem_b="pivot_pin",
        min_overlap=0.006,
        name="clamp 0 pivot is captured by its boss",
    )
    ctx.expect_overlap(
        chamber,
        clamp_1,
        axes="xyz",
        elem_a="pivot_boss_1",
        elem_b="pivot_pin",
        min_overlap=0.006,
        name="clamp 1 pivot is captured by its boss",
    )

    ctx.expect_gap(
        lid,
        chamber,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="chamber_wall",
        max_gap=0.0015,
        max_penetration=0.001,
        name="lid sits on the chamber rim",
    )
    ctx.expect_within(
        basket,
        chamber,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="chamber_wall",
        margin=0.002,
        name="rotating basket is centered inside clear chamber",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="feed_chute",
        margin=0.001,
        name="feed pusher runs through the central chute",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="x",
        positive_elem="tray_pan",
        negative_elem="base_shell",
        min_gap=0.0,
        max_gap=0.010,
        name="drip tray nests just in front of the base",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.085}):
        tray_forward = ctx.part_world_position(tray)
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_forward is not None and tray_forward[0] > tray_rest[0] + 0.075,
        details=f"rest={tray_rest}, forward={tray_forward}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.055}):
        pusher_down = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="feed_chute",
            margin=0.001,
            name="lowered pusher remains guided by chute",
        )
    ctx.check(
        "feed pusher slides down through chute",
        pusher_rest is not None and pusher_down is not None and pusher_down[2] < pusher_rest[2] - 0.045,
        details=f"rest={pusher_rest}, down={pusher_down}",
    )

    lid_closed = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 0.85}):
        lid_raised = ctx.part_world_aabb(lid)
    ctx.check(
        "rear-hinged lid rotates upward",
        lid_closed is not None and lid_raised is not None and lid_raised[1][2] > lid_closed[1][2] + 0.045,
        details=f"closed={lid_closed}, raised={lid_raised}",
    )

    clamp_0_up = ctx.part_world_aabb(clamp_0)
    clamp_1_up = ctx.part_world_aabb(clamp_1)
    with ctx.pose({clamp_joint_0: -1.10, clamp_joint_1: -1.10}):
        clamp_0_down = ctx.part_world_aabb(clamp_0)
        clamp_1_down = ctx.part_world_aabb(clamp_1)
    ctx.check(
        "side clamps rotate down from secured pose",
        clamp_0_up is not None
        and clamp_1_up is not None
        and clamp_0_down is not None
        and clamp_1_down is not None
        and clamp_0_down[1][2] < clamp_0_up[1][2] - 0.030
        and clamp_1_down[1][2] < clamp_1_up[1][2] - 0.030,
        details=f"up0={clamp_0_up}, down0={clamp_0_down}, up1={clamp_1_up}, down1={clamp_1_down}",
    )

    return ctx.report()


object_model = build_object_model()

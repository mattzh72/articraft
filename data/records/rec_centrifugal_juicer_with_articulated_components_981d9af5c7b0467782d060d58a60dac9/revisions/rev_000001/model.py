from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _base_body() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.300, 0.240, 0.118, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
        .edges(">Z")
        .fillet(0.010)
    )
    motor_pedestal = (
        cq.Workplane("XY")
        .circle(0.086)
        .extrude(0.010)
        .translate((0.0, -0.008, 0.118))
    )
    front_face = (
        cq.Workplane("XZ")
        .workplane(offset=0.102)
        .center(0.0, 0.050)
        .rect(0.150, 0.030)
        .extrude(0.012)
    )
    return body.union(motor_pedestal).union(front_face)


def _chamber_body() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.096).extrude(0.068)
    inner = cq.Workplane("XY").circle(0.088).extrude(0.060).translate((0.0, 0.0, 0.008))
    bowl = outer.cut(inner)
    axle_hole = cq.Workplane("XY").circle(0.016).extrude(0.014)
    return bowl.cut(axle_hole)


def _lid_body() -> cq.Workplane:
    lid_plate = (
        cq.Workplane("XY")
        .box(0.220, 0.185, 0.024, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.004)
    )
    lid_plate = (
        lid_plate.faces(">Z")
        .workplane()
        .center(0.0, 0.096)
        .rect(0.086, 0.074)
        .cutThruAll()
    )

    chute_outer = (
        cq.Workplane("XY")
        .box(0.104, 0.092, 0.112, centered=(True, True, False))
        .translate((0.0, 0.096, 0.024))
    )
    chute_inner = (
        cq.Workplane("XY")
        .box(0.086, 0.074, 0.118, centered=(True, True, False))
        .translate((0.0, 0.096, 0.022))
    )
    return lid_plate.union(chute_outer.cut(chute_inner))


def _basket_body() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.078).extrude(0.048)
    inner = cq.Workplane("XY").circle(0.071).extrude(0.044).translate((0.0, 0.0, 0.004))
    basket = outer.cut(inner)
    flare = cq.Workplane("XY").circle(0.084).circle(0.072).extrude(0.006).translate((0.0, 0.0, 0.048))
    hub = cq.Workplane("XY").circle(0.018).extrude(0.020)
    return basket.union(flare).union(hub)


def _add_clamp_geometry(part, material, side_sign: float) -> None:
    part.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="pivot",
    )
    part.visual(
        Box((0.016, 0.012, 0.044)),
        origin=Origin(xyz=(0.0, 0.006, 0.022)),
        material=material,
        name="arm",
    )
    part.visual(
        Box((0.018, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, 0.035)),
        material=material,
        name="hook",
    )
    part.visual(
        Box((0.022, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, 0.010)),
        material=material,
        name="grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.92, 0.92, 0.90, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.82, 0.86, 0.90, 0.34))
    steel = model.material("steel", rgba=(0.75, 0.76, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "juicer_base"),
        material=body_white,
        name="housing",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(-0.105, -0.082, 0.004)),
        material=rubber,
        name="foot_0",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.105, -0.082, 0.004)),
        material=rubber,
        name="foot_1",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(-0.105, 0.072, 0.004)),
        material=rubber,
        name="foot_2",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.105, 0.072, 0.004)),
        material=rubber,
        name="foot_3",
    )
    base.visual(
        Box((0.018, 0.160, 0.014)),
        origin=Origin(xyz=(-0.058, 0.110, 0.036)),
        material=graphite,
        name="left_rail",
    )
    base.visual(
        Box((0.018, 0.160, 0.014)),
        origin=Origin(xyz=(0.058, 0.110, 0.036)),
        material=graphite,
        name="right_rail",
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(_chamber_body(), "juicer_chamber"),
        material=smoke_clear,
        name="bowl",
    )
    chamber.visual(
        Box((0.050, 0.060, 0.024)),
        origin=Origin(xyz=(0.0, 0.116, 0.030)),
        material=graphite,
        name="spout",
    )
    chamber.visual(
        Box((0.018, 0.028, 0.034)),
        origin=Origin(xyz=(-0.104, -0.018, 0.017)),
        material=graphite,
        name="left_pivot_pad",
    )
    chamber.visual(
        Box((0.018, 0.028, 0.034)),
        origin=Origin(xyz=(0.104, -0.018, 0.017)),
        material=graphite,
        name="right_pivot_pad",
    )
    chamber.visual(
        Box((0.070, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.102, 0.068)),
        material=graphite,
        name="hinge_bridge",
    )

    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base,
        child=chamber,
        origin=Origin(xyz=(0.0, -0.008, 0.128)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_body(), "juicer_lid"),
        material=smoke_clear,
        name="lid_shell",
    )
    lid.visual(
        Box((0.018, 0.030, 0.012)),
        origin=Origin(xyz=(-0.103, 0.088, -0.006)),
        material=smoke_clear,
        name="left_lug",
    )
    lid.visual(
        Box((0.018, 0.030, 0.012)),
        origin=Origin(xyz=(0.103, 0.088, -0.006)),
        material=smoke_clear,
        name="right_lug",
    )

    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(0.0, -0.092, 0.069)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.082, 0.070, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=graphite,
        name="plunger",
    )
    pusher.visual(
        Box((0.002, 0.050, 0.060)),
        origin=Origin(xyz=(-0.042, 0.0, -0.020)),
        material=graphite,
        name="left_guide",
    )
    pusher.visual(
        Box((0.002, 0.050, 0.060)),
        origin=Origin(xyz=(0.042, 0.0, -0.020)),
        material=graphite,
        name="right_guide",
    )
    pusher.visual(
        Box((0.102, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=graphite,
        name="cap",
    )
    pusher.visual(
        Box((0.040, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=graphite,
        name="grip",
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, 0.096, 0.136)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.18,
            lower=0.0,
            upper=0.060,
        ),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_body(), "juicer_basket"),
        material=steel,
        name="basket_body",
    )
    basket.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=graphite,
        name="shaft",
    )

    model.articulation(
        "chamber_to_basket",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=25.0,
        ),
    )

    left_clamp = model.part("left_clamp")
    _add_clamp_geometry(left_clamp, graphite, side_sign=-1.0)
    model.articulation(
        "chamber_to_left_clamp",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=left_clamp,
        origin=Origin(xyz=(-0.104, 0.0, 0.014)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )

    right_clamp = model.part("right_clamp")
    _add_clamp_geometry(right_clamp, graphite, side_sign=1.0)
    model.articulation(
        "chamber_to_right_clamp",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=right_clamp,
        origin=Origin(xyz=(0.104, 0.0, 0.014)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
        mimic=Mimic("chamber_to_left_clamp"),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.168, 0.142, 0.012)),
        origin=Origin(xyz=(0.0, 0.071, 0.006)),
        material=graphite,
        name="pan",
    )
    drip_tray.visual(
        Box((0.008, 0.142, 0.014)),
        origin=Origin(xyz=(-0.080, 0.071, 0.013)),
        material=graphite,
        name="left_wall",
    )
    drip_tray.visual(
        Box((0.008, 0.142, 0.014)),
        origin=Origin(xyz=(0.080, 0.071, 0.013)),
        material=graphite,
        name="right_wall",
    )
    drip_tray.visual(
        Box((0.168, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.138, 0.010)),
        material=graphite,
        name="front_lip",
    )
    drip_tray.visual(
        Box((0.090, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, -0.008, 0.014)),
        material=graphite,
        name="tongue",
    )

    model.articulation(
        "base_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.121, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.15,
            lower=0.0,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    left_clamp = object_model.get_part("left_clamp")
    tray = object_model.get_part("drip_tray")

    lid_hinge = object_model.get_articulation("chamber_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    clamp_joint = object_model.get_articulation("chamber_to_left_clamp")
    tray_slide = object_model.get_articulation("base_to_drip_tray")

    ctx.allow_overlap(
        base,
        tray,
        elem_a="housing",
        elem_b="tongue",
        reason="The tray tongue is intentionally represented as sliding into the base's simplified front receiver volume.",
    )
    ctx.allow_overlap(
        chamber,
        left_clamp,
        elem_a="left_pivot_pad",
        elem_b="pivot",
        reason="The left clamp pivot barrel is intentionally seated through the chamber's pivot boss.",
    )
    ctx.allow_overlap(
        chamber,
        object_model.get_part("right_clamp"),
        elem_a="right_pivot_pad",
        elem_b="pivot",
        reason="The right clamp pivot barrel is intentionally seated through the chamber's pivot boss.",
    )

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0, clamp_joint: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="bowl",
            min_gap=0.0005,
            max_gap=0.004,
            name="lid seats just above chamber rim",
        )
        ctx.expect_overlap(
            lid,
            chamber,
            axes="xy",
            elem_a="lid_shell",
            elem_b="bowl",
            min_overlap=0.160,
            name="lid covers the chamber opening",
        )
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="plunger",
            outer_elem="lid_shell",
            margin=0.012,
            name="pusher stays centered over the feed opening",
        )
        ctx.expect_overlap(
            lid,
            left_clamp,
            axes="xy",
            elem_a="left_lug",
            elem_b="hook",
            min_overlap=0.008,
            name="left clamp hook aligns with the lid lug",
        )
        ctx.expect_gap(
            lid,
            left_clamp,
            axis="z",
            positive_elem="left_lug",
            negative_elem="hook",
            min_gap=0.0005,
            max_gap=0.008,
            name="left clamp hook nests under the lid lug",
        )
        ctx.expect_overlap(
            tray,
            chamber,
            axes="x",
            elem_a="pan",
            elem_b="spout",
            min_overlap=0.040,
            name="drip tray stays under the outlet in width",
        )
        ctx.expect_gap(
            chamber,
            tray,
            axis="z",
            positive_elem="spout",
            negative_elem="pan",
            min_gap=0.090,
            max_gap=0.160,
            name="outlet sits above the drip tray",
        )
        ctx.expect_within(
            basket,
            chamber,
            axes="xy",
            inner_elem="basket_body",
            outer_elem="bowl",
            margin=0.012,
            name="basket stays centered inside the juicing chamber",
        )

    closed_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.080,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.055}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="plunger",
            outer_elem="lid_shell",
            margin=0.012,
            name="pusher remains guided while descending",
        )
        pushed_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "pusher travels downward through the chute",
        rest_pusher is not None
        and pushed_pusher is not None
        and pushed_pusher[2] < rest_pusher[2] - 0.045,
        details=f"rest={rest_pusher}, pushed={pushed_pusher}",
    )

    closed_hook = ctx.part_element_world_aabb(left_clamp, elem="hook")
    with ctx.pose({clamp_joint: 0.95}):
        opened_hook = ctx.part_element_world_aabb(left_clamp, elem="hook")
        ctx.expect_gap(
            lid,
            left_clamp,
            axis="z",
            positive_elem="left_lug",
            negative_elem="hook",
            min_gap=0.014,
            name="open clamp clears the lid lug",
        )
    ctx.check(
        "clamp swings forward when opened",
        closed_hook is not None
        and opened_hook is not None
        and opened_hook[0][1] > closed_hook[0][1] + 0.020,
        details=f"closed={closed_hook}, open={opened_hook}",
    )

    rest_tray = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.065}):
        extended_tray = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            base,
            axes="x",
            elem_a="tongue",
            elem_b="housing",
            min_overlap=0.060,
            name="drip tray keeps lateral engagement while extended",
        )
    ctx.check(
        "drip tray slides forward",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[1] > rest_tray[1] + 0.050,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()

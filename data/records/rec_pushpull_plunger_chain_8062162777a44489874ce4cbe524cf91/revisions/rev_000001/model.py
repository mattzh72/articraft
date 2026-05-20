from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _guide_body_shape() -> cq.Workplane:
    """One-piece bearing block with a through bore, viewing slot, wall foot, and hinge yoke."""

    sleeve = cq.Workplane("XY").box(0.300, 0.070, 0.080)

    # Wall mounting land: its back face touches the side plate and its front
    # face slightly keys into the sleeve so the guide reads as one casting.
    mounting_land = cq.Workplane("XY").box(0.320, 0.013, 0.100).translate(
        (0.0, -0.0405, 0.0)
    )

    # A two-ear yoke at the output end carries the flap hinge pin.  The central
    # gap leaves clearance for the flap barrel and blade.
    yoke_outer = cq.Workplane("XY").box(0.065, 0.018, 0.066).translate(
        (0.180, 0.037, 0.0)
    )
    yoke_inner = cq.Workplane("XY").box(0.065, 0.018, 0.066).translate(
        (0.180, -0.037, 0.0)
    )

    guide = sleeve.union(mounting_land).union(yoke_outer).union(yoke_inner)

    plunger_bore = cq.Workplane("YZ").circle(0.0215).extrude(0.360, both=True)
    viewing_slot = cq.Workplane("XY").box(0.185, 0.045, 0.038).translate(
        (0.000, 0.043, 0.0)
    )
    hinge_bore = (
        cq.Workplane("XZ")
        .center(0.198, 0.0)
        .circle(0.0115)
        .extrude(0.120, both=True)
    )

    return guide.cut(plunger_bore).cut(viewing_slot).cut(hinge_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_plunger_linkage")

    painted_steel = model.material("dark_painted_steel", rgba=(0.13, 0.15, 0.17, 1.0))
    cast_aluminum = model.material("cast_aluminum", rgba=(0.58, 0.60, 0.58, 1.0))
    bright_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    brass = model.material("oiled_bronze", rgba=(0.70, 0.48, 0.20, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.520, 0.018, 0.300)),
        origin=Origin(xyz=(0.040, 0.0, 0.160)),
        material=painted_steel,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.560, 0.030, 0.026)),
        origin=Origin(xyz=(0.040, -0.006, 0.013)),
        material=painted_steel,
        name="grounding_foot",
    )
    for index, (x_pos, z_pos) in enumerate(
        ((-0.170, 0.060), (0.250, 0.060), (-0.170, 0.260), (0.250, 0.260))
    ):
        side_plate.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x_pos, 0.012, z_pos), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"bolt_head_{index}",
        )

    guide_body = model.part("guide_body")
    guide_body.visual(
        Box((0.320, 0.013, 0.100)),
        origin=Origin(xyz=(0.0, -0.0405, 0.0)),
        material=cast_aluminum,
        name="mounting_land",
    )
    guide_body.visual(
        Box((0.300, 0.070, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_aluminum,
        name="top_rail",
    )
    guide_body.visual(
        Box((0.300, 0.070, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=cast_aluminum,
        name="bottom_rail",
    )
    guide_body.visual(
        Box((0.300, 0.020, 0.076)),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=cast_aluminum,
        name="back_web",
    )
    guide_body.visual(
        Box((0.064, 0.018, 0.022)),
        origin=Origin(xyz=(0.180, -0.037, 0.022)),
        material=cast_aluminum,
        name="yoke_top_0",
    )
    guide_body.visual(
        Box((0.064, 0.018, 0.022)),
        origin=Origin(xyz=(0.180, -0.037, -0.022)),
        material=cast_aluminum,
        name="yoke_bottom_0",
    )
    guide_body.visual(
        Box((0.012, 0.018, 0.066)),
        origin=Origin(xyz=(0.153, -0.037, 0.0)),
        material=cast_aluminum,
        name="yoke_bridge_0",
    )
    guide_body.visual(
        Box((0.064, 0.018, 0.022)),
        origin=Origin(xyz=(0.180, 0.037, 0.022)),
        material=cast_aluminum,
        name="yoke_top_1",
    )
    guide_body.visual(
        Box((0.064, 0.018, 0.022)),
        origin=Origin(xyz=(0.180, 0.037, -0.022)),
        material=cast_aluminum,
        name="yoke_bottom_1",
    )
    guide_body.visual(
        Box((0.012, 0.018, 0.066)),
        origin=Origin(xyz=(0.153, 0.037, 0.0)),
        material=cast_aluminum,
        name="yoke_bridge_1",
    )
    guide_body.visual(
        Box((0.070, 0.026, 0.018)),
        origin=Origin(xyz=(0.170, 0.0, 0.044)),
        material=cast_aluminum,
        name="yoke_crown",
    )
    model.articulation(
        "plate_to_guide",
        ArticulationType.FIXED,
        parent=side_plate,
        child=guide_body,
        origin=Origin(xyz=(0.020, 0.056, 0.160)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.014, length=0.360),
        origin=Origin(xyz=(-0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="thumb_cap",
    )
    plunger.visual(
        Box((0.016, 0.020, 0.070)),
        origin=Origin(xyz=(0.1595, 0.0, -0.035)),
        material=bright_steel,
        name="nose_web",
    )
    plunger.visual(
        Box((0.018, 0.030, 0.020)),
        origin=Origin(xyz=(0.1595, 0.0, -0.058)),
        material=bright_steel,
        name="drive_pad",
    )
    model.articulation(
        "guide_to_plunger",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.016, effort=90.0, velocity=0.18),
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.005, length=0.096),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hinge_pin",
    )
    flap.visual(
        Cylinder(radius=0.0095, length=0.050),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.018, 0.040, 0.105)),
        origin=Origin(xyz=(0.006, 0.0, -0.058)),
        material=safety_yellow,
        name="flap_plate",
    )
    flap.visual(
        Box((0.010, 0.032, 0.026)),
        origin=Origin(xyz=(-0.008, 0.0, -0.058)),
        material=brass,
        name="strike_pad",
    )
    flap.visual(
        Cylinder(radius=0.0155, length=0.004),
        origin=Origin(xyz=(0.0, -0.048, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hinge_washer_0",
    )
    flap.visual(
        Cylinder(radius=0.0155, length=0.004),
        origin=Origin(xyz=(0.0, 0.048, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hinge_washer_1",
    )
    flap.visual(
        Box((0.044, 0.032, 0.012)),
        origin=Origin(xyz=(0.022, 0.0, -0.112)),
        material=safety_yellow,
        name="output_lip",
    )
    model.articulation(
        "guide_to_flap",
        ArticulationType.REVOLUTE,
        parent=guide_body,
        child=flap,
        origin=Origin(xyz=(0.198, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=12.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    guide_body = object_model.get_part("guide_body")
    plunger = object_model.get_part("plunger")
    flap = object_model.get_part("flap")
    plunger_slide = object_model.get_articulation("guide_to_plunger")
    flap_hinge = object_model.get_articulation("guide_to_flap")

    ctx.expect_contact(
        guide_body,
        side_plate,
        elem_a="mounting_land",
        elem_b="wall_plate",
        name="guide mounting land is seated on the grounded side plate",
    )
    ctx.expect_contact(
        plunger,
        guide_body,
        elem_a="shaft",
        elem_b="top_rail",
        name="plunger shaft bears against the upper guide rail",
    )
    ctx.expect_contact(
        plunger,
        guide_body,
        elem_a="shaft",
        elem_b="bottom_rail",
        name="plunger shaft bears against the lower guide rail",
    )
    ctx.expect_overlap(
        plunger,
        guide_body,
        axes="x",
        elem_a="shaft",
        elem_b="top_rail",
        min_overlap=0.22,
        name="plunger remains inserted through the guide at rest",
    )
    ctx.expect_contact(
        flap,
        guide_body,
        elem_a="hinge_washer_0",
        elem_b="yoke_top_0",
        name="flap hinge washer seats against the yoke cheek",
    )

    ctx.expect_gap(
        flap,
        plunger,
        axis="x",
        positive_elem="strike_pad",
        negative_elem="drive_pad",
        min_gap=0.0,
        max_gap=0.020,
        name="drive pad starts just short of the flap strike pad",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(flap, elem="output_lip")
    with ctx.pose({plunger_slide: 0.016}):
        ctx.expect_overlap(
            plunger,
            guide_body,
            axes="x",
            elem_a="shaft",
            elem_b="top_rail",
            min_overlap=0.22,
            name="extended plunger stays retained in the guide",
        )
        ctx.expect_gap(
            flap,
            plunger,
            axis="x",
            positive_elem="strike_pad",
            negative_elem="drive_pad",
            min_gap=0.0,
            max_gap=0.002,
            name="extended drive pad reaches the flap strike pad without penetration",
        )

    with ctx.pose({flap_hinge: 0.85}):
        swung_lip_aabb = ctx.part_element_world_aabb(flap, elem="output_lip")

    ctx.check(
        "revolute flap swings outward from the side plate",
        rest_lip_aabb is not None
        and swung_lip_aabb is not None
        and swung_lip_aabb[1][0] > rest_lip_aabb[1][0] + 0.035,
        details=f"rest={rest_lip_aabb}, swung={swung_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PEDESTAL_HEIGHT = 0.7625
BODY_HEIGHT = 0.170
JAR_HEIGHT = 0.420
TOP_HEIGHT = 0.108


def _make_glass_jar() -> cq.Workplane:
    outer_radius = 0.130
    inner_radius = 0.121
    height = JAR_HEIGHT
    bottom_thickness = 0.016

    shell = cq.Workplane("XY").circle(outer_radius).extrude(height)
    void = (
        cq.Workplane("XY")
        .workplane(offset=bottom_thickness)
        .circle(inner_radius)
        .extrude(height - bottom_thickness + 0.002)
    )
    jar = shell.cut(void)
    jar = jar.edges(">Z").fillet(0.004)
    return jar


def _make_peanut_pile() -> cq.Workplane:
    mound = cq.Workplane("XY").sphere(0.112).translate((0.0, 0.0, 0.080))
    crop = cq.Workplane("XY").circle(0.106).extrude(0.200)
    return mound.intersect(crop)


def _make_top_cap() -> cq.Workplane:
    cap_radius = 0.146
    dome = cq.Workplane("XY").sphere(cap_radius).translate((0.0, 0.0, -0.046))
    cap_crop = cq.Workplane("XY").box(0.400, 0.400, 0.160).translate((0.0, 0.0, 0.080))
    base_ring = cq.Workplane("XY").circle(cap_radius).extrude(0.018)
    cap = dome.intersect(cap_crop).union(base_ring)

    refill_opening = cq.Workplane("XY").rect(0.082, 0.054).extrude(0.140).translate((0.0, 0.010, 0.006))
    hinge_relief = cq.Workplane("XY").rect(0.040, 0.020).extrude(0.050).translate((0.0, -0.036, 0.072))
    return cap.cut(refill_opening.union(hinge_relief))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_peanut_vendor")

    pedestal_black = model.material("pedestal_black", rgba=(0.12, 0.12, 0.13, 1.0))
    body_red = model.material("body_red", rgba=(0.60, 0.08, 0.08, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.73, 0.74, 0.76, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.26, 1.0))
    glass = model.material("glass", rgba=(0.78, 0.88, 0.92, 0.28))
    peanut_color = model.material("peanut_color", rgba=(0.78, 0.62, 0.36, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    recess_black = model.material("recess_black", rgba=(0.07, 0.07, 0.08, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.125, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=pedestal_black,
        name="base_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.030, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.378)),
        material=pedestal_black,
        name="stem",
    )
    pedestal.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.745)),
        material=pedestal_black,
        name="capital",
    )

    body = model.part("mechanism_body")
    body.visual(
        Box((0.190, 0.150, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=body_red,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.074, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=trim_metal,
        name="jar_seat",
    )
    body.visual(
        Box((0.132, 0.018, 0.104)),
        origin=Origin(xyz=(0.0, 0.084, 0.078)),
        material=trim_metal,
        name="front_panel",
    )
    body.visual(
        Box((0.082, 0.046, 0.032)),
        origin=Origin(xyz=(0.0, 0.046, 0.124)),
        material=trim_metal,
        name="coin_head",
    )
    body.visual(
        Box((0.054, 0.024, 0.038)),
        origin=Origin(xyz=(0.0, 0.086, 0.024)),
        material=trim_metal,
        name="chute_body",
    )
    body.visual(
        Box((0.040, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.098, 0.024)),
        material=recess_black,
        name="chute_mouth",
    )
    body.visual(
        Box((0.050, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.060, 0.132)),
        material=recess_black,
        name="coin_slot",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.085, 0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="knob_shaft",
    )
    body.visual(
        Box((0.024, 0.008, 0.010)),
        origin=Origin(xyz=(0.044, 0.090, 0.098)),
        material=trim_metal,
        name="coin_hinge_tab_0",
    )
    body.visual(
        Box((0.024, 0.008, 0.010)),
        origin=Origin(xyz=(0.066, 0.090, 0.098)),
        material=trim_metal,
        name="coin_hinge_tab_1",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.007),
        origin=Origin(xyz=(0.044, 0.094, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="coin_hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.007),
        origin=Origin(xyz=(0.066, 0.094, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="coin_hinge_barrel_1",
    )

    jar = model.part("jar")
    jar.visual(
        mesh_from_cadquery(_make_glass_jar(), "glass_jar"),
        material=glass,
        name="jar_shell",
    )
    jar.visual(
        mesh_from_cadquery(_make_peanut_pile(), "peanut_pile"),
        material=peanut_color,
        name="peanuts",
    )

    top = model.part("top_cap")
    top.visual(
        mesh_from_cadquery(_make_top_cap(), "top_cap"),
        material=trim_metal,
        name="top_shell",
    )
    top.visual(
        Box((0.020, 0.012, 0.032)),
        origin=Origin(xyz=(-0.028, -0.048, 0.094)),
        material=trim_metal,
        name="lid_hinge_tab_0",
    )
    top.visual(
        Box((0.020, 0.012, 0.032)),
        origin=Origin(xyz=(0.028, -0.048, 0.094)),
        material=trim_metal,
        name="lid_hinge_tab_1",
    )
    top.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(-0.028, -0.040, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lid_hinge_barrel_0",
    )
    top.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.028, -0.040, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lid_hinge_barrel_1",
    )

    lid = model.part("refill_lid")
    lid.visual(
        Box((0.094, 0.072, 0.008)),
        origin=Origin(xyz=(0.0, 0.036, -0.001)),
        material=trim_metal,
        name="lid_panel",
    )
    lid.visual(
        Box((0.020, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.066, 0.004)),
        material=trim_metal,
        name="lid_pull",
    )
    lid.visual(
        Box((0.040, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.006, -0.001)),
        material=trim_metal,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lid_hinge_barrel",
    )

    knob = model.part("dispense_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.030,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.052, 0.007, flare=0.10),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "dispense_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="knob_hub",
    )

    coin_cover = model.part("coin_cover")
    coin_cover.visual(
        Box((0.028, 0.006, 0.050)),
        origin=Origin(xyz=(0.0, 0.003, -0.025)),
        material=trim_metal,
        name="cover_panel",
    )
    coin_cover.visual(
        Box((0.014, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, -0.003)),
        material=trim_metal,
        name="cover_hinge_bridge",
    )
    coin_cover.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="cover_hinge_barrel",
    )

    model.articulation(
        "pedestal_to_body",
        ArticulationType.FIXED,
        parent=pedestal,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT)),
    )
    model.articulation(
        "body_to_jar",
        ArticulationType.FIXED,
        parent=body,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
    )
    model.articulation(
        "jar_to_top",
        ArticulationType.FIXED,
        parent=jar,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, JAR_HEIGHT)),
    )
    model.articulation(
        "top_to_lid",
        ArticulationType.REVOLUTE,
        parent=top,
        child=lid,
        origin=Origin(xyz=(0.0, -0.040, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.094, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "body_to_coin_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=coin_cover,
        origin=Origin(xyz=(0.055, 0.094, 0.098)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    body = object_model.get_part("mechanism_body")
    jar = object_model.get_part("jar")
    top = object_model.get_part("top_cap")
    lid = object_model.get_part("refill_lid")
    knob = object_model.get_part("dispense_knob")
    coin_cover = object_model.get_part("coin_cover")

    lid_hinge = object_model.get_articulation("top_to_lid")
    coin_hinge = object_model.get_articulation("body_to_coin_cover")

    ctx.expect_gap(
        body,
        pedestal,
        axis="z",
        positive_elem="body_shell",
        negative_elem="capital",
        max_gap=0.002,
        max_penetration=0.0,
        name="mechanism body sits on pedestal capital",
    )
    ctx.expect_gap(
        jar,
        body,
        axis="z",
        positive_elem="jar_shell",
        negative_elem="jar_seat",
        max_gap=0.002,
        max_penetration=0.0,
        name="jar seats on the mechanism collar",
    )
    ctx.expect_gap(
        top,
        jar,
        axis="z",
        positive_elem="top_shell",
        negative_elem="jar_shell",
        max_gap=0.002,
        max_penetration=0.0,
        name="domed top sits on the jar rim",
    )
    ctx.expect_origin_gap(
        knob,
        body,
        axis="y",
        min_gap=0.085,
        max_gap=0.102,
        name="dispense knob is mounted on the front shaft",
    )
    ctx.expect_origin_distance(
        coin_cover,
        knob,
        axes="x",
        min_dist=0.045,
        max_dist=0.065,
        name="coin return cover sits beside the knob",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    open_lid_aabb = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "refill lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.040,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_coin_aabb = ctx.part_world_aabb(coin_cover)
    open_coin_aabb = None
    if coin_hinge.motion_limits is not None and coin_hinge.motion_limits.upper is not None:
        with ctx.pose({coin_hinge: coin_hinge.motion_limits.upper}):
            open_coin_aabb = ctx.part_world_aabb(coin_cover)

    ctx.check(
        "coin return cover swings outward",
        closed_coin_aabb is not None
        and open_coin_aabb is not None
        and open_coin_aabb[1][1] > closed_coin_aabb[1][1] + 0.018,
        details=f"closed={closed_coin_aabb}, open={open_coin_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

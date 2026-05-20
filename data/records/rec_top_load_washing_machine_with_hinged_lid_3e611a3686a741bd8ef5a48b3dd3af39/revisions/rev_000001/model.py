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


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """Small utility for soft appliance-control caps."""
    shape = cq.Workplane("XY").box(width, depth, height)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _washer_cabinet_shell() -> cq.Workplane:
    width = 0.72
    depth = 0.72
    height = 0.88
    opening_radius = 0.275
    opening_y = -0.04

    body = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.025)
    )
    chamber_cut = (
        cq.Workplane("XY")
        .circle(opening_radius)
        .extrude(height + 0.16)
        .translate((0.0, opening_y, -0.06))
    )
    body = body.cut(chamber_cut)

    console = (
        cq.Workplane("XY")
        .box(width, 0.18, 0.23)
        .translate((0.0, 0.365, 0.995))
        .edges("|Z")
        .fillet(0.018)
    )
    return body.union(console)


def _rotating_basket() -> cq.Workplane:
    outer_r = 0.238
    inner_r = 0.205
    height = 0.56
    bottom_thickness = 0.045

    basket = cq.Workplane("XY").circle(outer_r).extrude(height)
    hollow_cut = (
        cq.Workplane("XY")
        .circle(inner_r)
        .extrude(height + 0.08)
        .translate((0.0, 0.0, bottom_thickness))
    )
    basket = basket.cut(hollow_cut)

    # Rolled top lip and low impeller plate make the rotating chamber read as a
    # real washer basket instead of a plain pipe.
    lip_outer = cq.Workplane("XY").circle(outer_r + 0.018).extrude(0.030).translate((0.0, 0.0, height - 0.006))
    lip_inner_cut = cq.Workplane("XY").circle(inner_r - 0.002).extrude(0.052).translate((0.0, 0.0, height - 0.017))
    basket = basket.union(lip_outer.cut(lip_inner_cut))

    impeller = cq.Workplane("XY").circle(0.075).extrude(0.026).translate((0.0, 0.0, bottom_thickness))
    basket = basket.union(impeller)

    for angle in (0.0, 120.0, 240.0):
        fin = (
            cq.Workplane("XY")
            .box(0.060, 0.064, 0.330)
            .translate((inner_r - 0.014, 0.0, 0.205))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        basket = basket.union(fin)

    return basket


def _annular_cylinder(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_r).extrude(height)
    inner = cq.Workplane("XY").circle(inner_r).extrude(height + 0.020).translate((0.0, 0.0, -0.010))
    return outer.cut(inner)


def _bearing_support() -> cq.Workplane:
    pedestal = cq.Workplane("XY").circle(0.075).extrude(0.270)
    cross_x = cq.Workplane("XY").box(0.640, 0.045, 0.055).translate((0.0, 0.0, 0.035))
    cross_y = cq.Workplane("XY").box(0.045, 0.550, 0.055).translate((0.0, 0.0, 0.035))
    return pedestal.union(cross_x).union(cross_y)


def _lid_frame() -> cq.Workplane:
    lid_w = 0.66
    lid_d = 0.56
    thickness = 0.035
    window_w = 0.455
    window_d = 0.330

    frame = (
        cq.Workplane("XY")
        .box(lid_w, lid_d, thickness)
        .translate((0.0, -lid_d / 2.0, thickness / 2.0))
        .edges("|Z")
        .fillet(0.045)
    )
    window_cut = (
        cq.Workplane("XY")
        .box(window_w, window_d, thickness + 0.030)
        .translate((0.0, -lid_d / 2.0, thickness / 2.0))
    )
    return frame.cut(window_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_top_load_washer")

    porcelain = model.material("porcelain_white", rgba=(0.92, 0.94, 0.93, 1.0))
    soft_black = model.material("soft_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    glass = model.material("smoked_glass", rgba=(0.25, 0.43, 0.55, 0.38))
    blue_led = model.material("blue_led", rgba=(0.18, 0.55, 1.00, 1.0))
    white_print = model.material("white_print", rgba=(0.94, 0.96, 0.95, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_washer_cabinet_shell(), "cabinet_shell", tolerance=0.002),
        material=porcelain,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.66, 0.026, 0.160)),
        origin=Origin(xyz=(0.0, 0.263, 1.015)),
        material=soft_black,
        name="control_fascia",
    )
    cabinet.visual(
        mesh_from_cadquery(_annular_cylinder(0.302, 0.252, 0.018), "deck_opening_lip", tolerance=0.0015),
        origin=Origin(xyz=(0.0, -0.040, 0.860)),
        material=graphite,
        name="deck_opening_lip",
    )
    cabinet.visual(
        mesh_from_cadquery(_annular_cylinder(0.276, 0.262, 0.455), "deep_chamber_wall", tolerance=0.002),
        origin=Origin(xyz=(0.0, -0.040, 0.425)),
        material=soft_black,
        name="deep_chamber_wall",
    )
    cabinet.visual(
        mesh_from_cadquery(_bearing_support(), "bearing_support", tolerance=0.002),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=graphite,
        name="bearing_support",
    )
    cabinet.visual(
        Box((0.160, 0.050, 0.008)),
        origin=Origin(xyz=(0.0, -0.342, 0.884)),
        material=soft_black,
        name="release_button_well",
    )
    for idx, x in enumerate((-0.255, 0.255)):
        cabinet.visual(
            Box((0.080, 0.018, 0.040)),
            origin=Origin(xyz=(x, 0.268, 0.902)),
            material=stainless,
            name=f"hinge_mount_{idx}",
        )

    # Small printed legends and status lamps on the broad console face.
    button_xs = (-0.265, -0.185, -0.105, 0.105, 0.185, 0.265)
    for idx, x in enumerate(button_xs):
        cabinet.visual(
            Box((0.042, 0.004, 0.006)),
            origin=Origin(xyz=(x, 0.248, 1.056)),
            material=white_print,
            name=f"program_label_{idx}",
        )
        cabinet.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, 0.249, 0.976), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue_led,
            name=f"status_light_{idx}",
        )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_rotating_basket(), "stainless_basket", tolerance=0.0015),
        material=stainless,
        name="stainless_basket",
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, -0.040, 0.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=12.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame(), "lid_frame", tolerance=0.0015),
        material=porcelain,
        name="lid_frame",
    )
    lid.visual(
        Box((0.475, 0.350, 0.008)),
        origin=Origin(xyz=(0.0, -0.280, 0.018)),
        material=glass,
        name="glass_window",
    )
    lid.visual(
        Box((0.310, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, -0.515, 0.041)),
        material=graphite,
        name="front_handle",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.017), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.245, 0.885)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.80, effort=35.0, velocity=1.2),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.118,
                0.036,
                body_style="skirted",
                top_diameter=0.092,
                skirt=KnobSkirt(0.132, 0.008, flare=0.04, chamfer=0.0015),
                grip=KnobGrip(style="fluted", count=28, depth=0.0016),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "selector_dial",
        ),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="selector_dial",
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(0.0, 0.250, 1.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=5.0),
    )

    program_button_mesh = mesh_from_cadquery(_rounded_box(0.056, 0.018, 0.030, 0.006), "program_button_cap")
    for idx, x in enumerate(button_xs):
        program_button = model.part(f"program_button_{idx}")
        program_button.visual(
            program_button_mesh,
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=graphite,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_program_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=program_button,
            origin=Origin(xyz=(x, 0.250, 1.015)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.014, effort=4.0, velocity=0.08),
        )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(_rounded_box(0.145, 0.038, 0.014, 0.008), "release_button_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=graphite,
        name="release_cap",
    )
    model.articulation(
        "cabinet_to_release_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=release_button,
        origin=Origin(xyz=(0.0, -0.342, 0.888)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.014, effort=5.0, velocity=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("tub")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    release_button = object_model.get_part("release_button")

    tub_joint = object_model.get_articulation("cabinet_to_tub")
    lid_joint = object_model.get_articulation("cabinet_to_lid")
    dial_joint = object_model.get_articulation("cabinet_to_dial")
    release_joint = object_model.get_articulation("cabinet_to_release_button")

    ctx.check(
        "washer has full size cabinet scale",
        len(object_model.parts) == 11,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "tub and selector are continuous rotary controls",
        tub_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"tub={tub_joint.articulation_type}, dial={dial_joint.articulation_type}",
    )
    ctx.check(
        "lid uses a rear revolute hinge",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_joint.axis) == (-1.0, 0.0, 0.0)
        and lid_joint.motion_limits.lower == 0.0
        and lid_joint.motion_limits.upper >= 1.5,
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}, limits={lid_joint.motion_limits}",
    )
    ctx.check(
        "front release button is vertical prismatic",
        release_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(release_joint.axis) == (0.0, 0.0, -1.0)
        and release_joint.motion_limits.upper >= 0.010,
        details=f"type={release_joint.articulation_type}, axis={release_joint.axis}, limits={release_joint.motion_limits}",
    )

    program_button_joints = [
        object_model.get_articulation(f"cabinet_to_program_button_{idx}") for idx in range(6)
    ]
    ctx.check(
        "six independent program button sliders",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in program_button_joints)
        and all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in program_button_joints),
        details=[(j.name, j.articulation_type, j.axis) for j in program_button_joints],
    )

    ctx.expect_contact(
        dial,
        cabinet,
        elem_a="selector_dial",
        elem_b="control_fascia",
        contact_tol=0.003,
        name="selector dial seats on console fascia",
    )
    ctx.expect_contact(
        release_button,
        cabinet,
        elem_a="release_cap",
        elem_b="release_button_well",
        contact_tol=0.001,
        name="release button is seated in the top deck well",
    )
    ctx.expect_contact(
        tub,
        cabinet,
        elem_a="stainless_basket",
        elem_b="bearing_support",
        contact_tol=0.002,
        name="rotating tub is carried by central bearing support",
    )
    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        inner_elem="stainless_basket",
        outer_elem="deep_chamber_wall",
        margin=0.010,
        name="basket sits inside the deep chamber wall",
    )
    ctx.expect_overlap(
        tub,
        cabinet,
        axes="z",
        elem_a="stainless_basket",
        elem_b="deep_chamber_wall",
        min_overlap=0.40,
        name="basket and chamber wall share visible depth",
    )

    tub_aabb = ctx.part_element_world_aabb(tub, elem="stainless_basket")
    ctx.check(
        "tub is a deep laundry chamber",
        tub_aabb is not None and (tub_aabb[1][2] - tub_aabb[0][2]) > 0.50,
        details=f"tub_aabb={tub_aabb}",
    )

    with ctx.pose({lid_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.25}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    release_rest = ctx.part_world_position(release_button)
    with ctx.pose({release_joint: release_joint.motion_limits.upper}):
        release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button slides down into top deck",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[2] < release_rest[2] - 0.010,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    first_button = object_model.get_part("program_button_0")
    second_button = object_model.get_part("program_button_1")
    first_joint = program_button_joints[0]
    first_rest = ctx.part_world_position(first_button)
    second_rest = ctx.part_world_position(second_button)
    with ctx.pose({first_joint: first_joint.motion_limits.upper}):
        first_pressed = ctx.part_world_position(first_button)
        second_unmoved = ctx.part_world_position(second_button)
    ctx.check(
        "program buttons press independently into console",
        first_rest is not None
        and first_pressed is not None
        and second_rest is not None
        and second_unmoved is not None
        and first_pressed[1] > first_rest[1] + 0.010
        and abs(second_unmoved[1] - second_rest[1]) < 0.001,
        details=f"first_rest={first_rest}, first_pressed={first_pressed}, second_rest={second_rest}, second_unmoved={second_unmoved}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

from math import pi

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


def _blade_mesh():
    hub = cq.Workplane("XY").circle(0.012).extrude(0.008).translate((0.0, 0.0, -0.004))
    strip = cq.Workplane("XY").box(0.060, 0.010, 0.0028).translate((0.0, 0.0, 0.002))
    blade = (
        hub.union(strip.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 14.0).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 35.0))
        .union(strip.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -14.0).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 125.0))
    )
    return mesh_from_cadquery(blade, "blade_assembly")


def _dial_mesh():
    dial = KnobGeometry(
        0.065,
        0.028,
        body_style="skirted",
        top_diameter=0.050,
        skirt=KnobSkirt(0.075, 0.007, flare=0.06),
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(dial, "blender_front_dial")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_smoothie_blender")

    body_dark = model.material("body_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.07, 0.08, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.20, 0.24, 0.26, 0.28))
    jar_clear = model.material("jar_clear", rgba=(0.76, 0.83, 0.88, 0.22))

    body = model.part("body")
    body.visual(
        Box((0.240, 0.250, 0.155)),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=body_dark,
        name="housing",
    )
    body.visual(
        Box((0.190, 0.190, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=trim_black,
        name="shoulder",
    )
    body.visual(
        Box((0.160, 0.160, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.189)),
        material=steel,
        name="jar_seat",
    )
    body.visual(
        Box((0.140, 0.010, 0.102)),
        origin=Origin(xyz=(0.0, -0.130, 0.086)),
        material=trim_black,
        name="front_bezel",
    )
    body.visual(
        Box((0.030, 0.030, 0.280)),
        origin=Origin(xyz=(-0.091, 0.095, 0.325)),
        material=body_dark,
        name="rear_upright_0",
    )
    body.visual(
        Box((0.030, 0.030, 0.280)),
        origin=Origin(xyz=(0.091, 0.095, 0.325)),
        material=body_dark,
        name="rear_upright_1",
    )
    body.visual(
        Box((0.212, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.095, 0.480)),
        material=trim_black,
        name="rear_crossbar",
    )

    jar = model.part("jar")
    jar.visual(
        Box((0.148, 0.148, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim_black,
        name="jar_collar",
    )
    jar.visual(
        Box((0.126, 0.126, 0.0045)),
        origin=Origin(xyz=(0.0, 0.0, 0.02225)),
        material=jar_clear,
        name="jar_floor",
    )
    jar.visual(
        Box((0.135, 0.0045, 0.220)),
        origin=Origin(xyz=(0.0, -0.06525, 0.130)),
        material=jar_clear,
        name="jar_front",
    )
    jar.visual(
        Box((0.135, 0.0045, 0.220)),
        origin=Origin(xyz=(0.0, 0.06525, 0.130)),
        material=jar_clear,
        name="jar_back",
    )
    jar.visual(
        Box((0.0045, 0.126, 0.220)),
        origin=Origin(xyz=(-0.06525, 0.0, 0.130)),
        material=jar_clear,
        name="jar_left",
    )
    jar.visual(
        Box((0.0045, 0.126, 0.220)),
        origin=Origin(xyz=(0.06525, 0.0, 0.130)),
        material=jar_clear,
        name="jar_right",
    )
    jar.visual(
        Box((0.136, 0.136, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=rubber_black,
        name="lid_flange",
    )
    jar.visual(
        Box((0.118, 0.118, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=trim_black,
        name="jar_lid",
    )
    jar.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        material=rubber_black,
        name="lid_cap",
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.212, 0.198, 0.004)),
        origin=Origin(xyz=(0.0, -0.099, -0.002)),
        material=smoked_clear,
        name="hood_top",
    )
    hood.visual(
        Box((0.212, 0.004, 0.301)),
        origin=Origin(xyz=(0.0, -0.196, -0.1505)),
        material=smoked_clear,
        name="hood_front",
    )
    hood.visual(
        Box((0.004, 0.198, 0.301)),
        origin=Origin(xyz=(-0.104, -0.099, -0.1505)),
        material=smoked_clear,
        name="hood_left",
    )
    hood.visual(
        Box((0.004, 0.198, 0.301)),
        origin=Origin(xyz=(0.104, -0.099, -0.1505)),
        material=smoked_clear,
        name="hood_right",
    )
    hood.visual(
        Box((0.212, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, -0.010, -0.0275)),
        material=trim_black,
        name="hood_hinge_band",
    )
    hood.visual(
        Box((0.110, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, -0.105, 0.007)),
        material=trim_black,
        name="hood_handle",
    )

    dial = model.part("dial")
    dial.visual(
        _dial_mesh(),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="dial_cap",
    )

    pulse = model.part("pulse")
    pulse.visual(
        Box((0.040, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=rubber_black,
        name="pulse_cap",
    )

    blade = model.part("blade")
    blade.visual(
        _blade_mesh(),
        material=steel,
        name="blade_assembly",
    )

    model.articulation(
        "body_to_jar",
        ArticulationType.FIXED,
        parent=body,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
    )
    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.0, 0.077, 0.495)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.3, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, -0.135, 0.098)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )
    model.articulation(
        "body_to_pulse",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pulse,
        origin=Origin(xyz=(0.0, -0.135, 0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0025),
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jar = object_model.get_part("jar")
    hood = object_model.get_part("hood")
    dial = object_model.get_part("dial")
    pulse = object_model.get_part("pulse")
    blade = object_model.get_part("blade")

    hood_hinge = object_model.get_articulation("body_to_hood")
    pulse_slide = object_model.get_articulation("body_to_pulse")

    ctx.expect_within(
        jar,
        hood,
        axes="xy",
        margin=0.0,
        name="jar footprint stays inside the hood",
    )
    ctx.expect_gap(
        hood,
        body,
        axis="z",
        positive_elem="hood_front",
        negative_elem="shoulder",
        min_gap=0.003,
        max_gap=0.012,
        name="closed hood settles just above the deck shoulder",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="y",
        positive_elem="front_bezel",
        negative_elem="dial_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="dial sits flush on the front bezel",
    )
    ctx.expect_gap(
        body,
        pulse,
        axis="y",
        positive_elem="front_bezel",
        negative_elem="pulse_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="pulse button sits against the bezel at rest",
    )
    ctx.expect_origin_gap(
        dial,
        pulse,
        axis="z",
        min_gap=0.045,
        name="pulse button is mounted below the main dial",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        inner_elem="blade_assembly",
        outer_elem="jar_collar",
        margin=0.0,
        name="blade stays centered inside the jar footprint",
    )
    ctx.expect_gap(
        blade,
        jar,
        axis="z",
        positive_elem="blade_assembly",
        negative_elem="jar_collar",
        min_gap=0.003,
        name="blade sits above the lower jar collar",
    )

    closed_hood_aabb = ctx.part_world_aabb(hood)
    closed_front_aabb = ctx.part_element_world_aabb(hood, elem="hood_front")
    rest_pos = ctx.part_world_position(pulse)
    with ctx.pose({pulse_slide: 0.0025}):
        pressed_pos = ctx.part_world_position(pulse)

    with ctx.pose({hood_hinge: 1.25}):
        open_hood_aabb = ctx.part_world_aabb(hood)
        open_front_aabb = ctx.part_element_world_aabb(hood, elem="hood_front")

    ctx.check(
        "pulse button moves inward when pressed",
        rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.0015,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )
    ctx.check(
        "hood rotates upward and rearward",
        closed_hood_aabb is not None
        and open_hood_aabb is not None
        and open_hood_aabb[0][1] > closed_hood_aabb[0][1] + 0.040,
        details=f"closed={closed_hood_aabb}, open={open_hood_aabb}",
    )
    ctx.check(
        "hood front edge lifts when opened",
        closed_front_aabb is not None
        and open_front_aabb is not None
        and open_front_aabb[0][2] > closed_front_aabb[0][2] + 0.015,
        details=f"closed_front={closed_front_aabb}, open_front={open_front_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

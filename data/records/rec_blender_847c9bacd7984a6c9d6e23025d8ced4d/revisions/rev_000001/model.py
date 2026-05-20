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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """CadQuery annular sleeve/ring, authored in meters around local +Z."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _base_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.48, 0.32, 0.20).translate((0.0, 0.0, 0.10))
    # Rounded vertical corners keep the motor housing from reading as a raw block.
    return body.edges("|Z").fillet(0.030)


def _jar_cup() -> cq.Workplane:
    # A real wide-mouth jar is hollow: transparent wall, open mouth, and thick floor.
    outer = cq.Workplane("XY").circle(0.130).extrude(0.420)
    inner_cutter = cq.Workplane("XY").circle(0.116).extrude(0.430).translate((0.0, 0.0, 0.024))
    return outer.cut(inner_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_blender")

    dark_shell = model.material("dark_graphite", rgba=(0.035, 0.039, 0.045, 1.0))
    satin_black = model.material("satin_black", rgba=(0.002, 0.002, 0.002, 1.0))
    smoked_panel = model.material("smoked_touch_panel", rgba=(0.0, 0.015, 0.025, 1.0))
    blue_display = model.material("blue_display_glow", rgba=(0.05, 0.55, 1.0, 0.85))
    stainless = model.material("brushed_stainless", rgba=(0.78, 0.80, 0.78, 1.0))
    clear_copolyester = model.material("clear_copolyester", rgba=(0.70, 0.92, 1.0, 0.32))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    white_ink = model.material("white_ink", rgba=(1.0, 1.0, 0.92, 0.75))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "rounded_motor_base"),
        material=dark_shell,
        name="motor_housing",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.168, 0.136, 0.045), "bayonet_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=satin_black,
        name="bayonet_collar",
    )
    # Three raised bayonet stop ramps show the jar's twist-lock path without
    # colliding with the jar lugs at the locked rest angle.
    for i, angle in enumerate((math.radians(60.0), math.radians(180.0), math.radians(300.0))):
        base.visual(
            Box((0.026, 0.020, 0.010)),
            origin=Origin(
                xyz=(0.153 * math.cos(angle), 0.153 * math.sin(angle), 0.250),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_black,
            name=f"bayonet_stop_{i}",
        )
    base.visual(
        Box((0.240, 0.006, 0.095)),
        origin=Origin(xyz=(0.0, -0.163, 0.105), rpy=(0.0, 0.0, 0.0)),
        material=smoked_panel,
        name="touch_panel",
    )
    base.visual(
        Box((0.115, 0.007, 0.026)),
        origin=Origin(xyz=(-0.050, -0.167, 0.126)),
        material=blue_display,
        name="vacuum_display",
    )
    for i, x in enumerate((-0.090, -0.030, 0.030, 0.090)):
        base.visual(
            Box((0.018, 0.007, 0.018)),
            origin=Origin(xyz=(x, -0.167, 0.078)),
            material=satin_black,
            name=f"touch_icon_{i}",
        )

    jar = model.part("jar")
    jar.visual(
        mesh_from_cadquery(_jar_cup(), "hollow_wide_mouth_jar"),
        material=clear_copolyester,
        name="jar_glass",
    )
    jar.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=rubber,
        name="bearing_socket",
    )
    lug_names = ("bayonet_lug_0", "bayonet_lug_1", "bayonet_lug_2")
    for lug_name, angle in zip(lug_names, (0.0, math.radians(120.0), math.radians(240.0))):
        jar.visual(
            Box((0.044, 0.020, 0.014)),
            origin=Origin(
                xyz=(0.146 * math.cos(angle), 0.146 * math.sin(angle), 0.007),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=lug_name,
        )
    # Tactile molded measuring marks on the transparent wall.
    jar.visual(
        Box((0.006, 0.003, 0.270)),
        origin=Origin(xyz=(0.0, -0.129, 0.250)),
        material=white_ink,
        name="measure_scale",
    )
    for i, z in enumerate((0.125, 0.180, 0.235, 0.290, 0.345)):
        jar.visual(
            Box((0.032, 0.003, 0.004)),
            origin=Origin(xyz=(0.013, -0.128, z)),
            material=white_ink,
            name=f"measure_tick_{i}",
        )

    blade = model.part("blade_assembly")
    blade.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=stainless,
        name="blade_hub",
    )
    for i in range(6):
        angle = i * math.tau / 6.0
        pitch = 0.18 if i % 2 == 0 else -0.18
        blade.visual(
            Box((0.092, 0.020, 0.006)),
            origin=Origin(
                xyz=(0.050 * math.cos(angle), 0.050 * math.sin(angle), 0.017),
                rpy=(0.0, pitch, angle),
            ),
            material=stainless,
            name=f"blade_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.142, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=rubber,
        name="lid_disk",
    )
    lid.visual(
        Cylinder(radius=0.110, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=rubber,
        name="seal_plug",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.036, 0.0, 0.027)),
        material=smoked_panel,
        name="vacuum_port",
    )
    for i, y in enumerate((-0.034, 0.034)):
        lid.visual(
            Box((0.014, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.032)),
            material=rubber,
            name=f"valve_hinge_ear_{i}",
        )

    valve_flap = model.part("valve_flap")
    valve_flap.visual(
        Cylinder(radius=0.005, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="hinge_barrel",
    )
    valve_flap.visual(
        Box((0.060, 0.045, 0.004)),
        origin=Origin(xyz=(0.036, 0.0, -0.007)),
        material=rubber,
        name="flap_pad",
    )
    valve_flap.visual(
        Box((0.010, 0.039, 0.004)),
        origin=Origin(xyz=(0.006, 0.0, -0.004)),
        material=rubber,
        name="flap_web",
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=220.0),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
    )
    model.articulation(
        "lid_to_valve_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=valve_flap,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade = object_model.get_part("blade_assembly")
    lid = object_model.get_part("lid")
    valve_flap = object_model.get_part("valve_flap")
    blade_joint = object_model.get_articulation("jar_to_blade")
    flap_joint = object_model.get_articulation("lid_to_valve_flap")

    ctx.expect_within(
        jar,
        base,
        axes="xy",
        inner_elem="jar_glass",
        outer_elem="bayonet_collar",
        margin=0.0,
        name="wide jar sits inside bayonet collar footprint",
    )
    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem="bayonet_lug_0",
        negative_elem="bayonet_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="bayonet lug seats on collar rim",
    )
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        positive_elem="lid_disk",
        negative_elem="jar_glass",
        max_gap=0.001,
        max_penetration=0.0,
        name="vacuum lid seals on jar mouth",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        outer_elem="jar_glass",
        margin=0.0,
        name="six-blade assembly stays inside cylindrical jar",
    )
    ctx.expect_gap(
        blade,
        jar,
        axis="z",
        positive_elem="blade_hub",
        negative_elem="bearing_socket",
        max_gap=0.001,
        max_penetration=0.0,
        name="blade hub bears on jar bottom socket",
    )
    ctx.expect_gap(
        valve_flap,
        lid,
        axis="z",
        positive_elem="flap_pad",
        negative_elem="lid_disk",
        max_gap=0.001,
        max_penetration=0.0,
        name="valve flap rests on lid seal",
    )

    blade_names = {visual.name for visual in blade.visuals}
    ctx.check(
        "blade assembly has six blades",
        all(f"blade_{i}" in blade_names for i in range(6)),
        details=f"blade visuals={sorted(blade_names)}",
    )
    ctx.check(
        "blade joint is continuous and vertical",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS and blade_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={blade_joint.articulation_type}, axis={blade_joint.axis}",
    )

    rest_aabb = ctx.part_world_aabb(valve_flap)
    with ctx.pose({flap_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(valve_flap)
    ctx.check(
        "valve flap opens upward on center hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.020,
        details=f"closed_aabb={rest_aabb}, opened_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

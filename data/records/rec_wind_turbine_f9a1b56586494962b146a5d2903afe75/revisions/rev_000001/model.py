from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOWER_TOP_Z = 7.20
ROTOR_AXIS_Z = 0.55
ROTOR_AXIS_X = 1.29
BLADE_ROOT_RADIUS = 0.665


def _tapered_tower_shell() -> cq.Workplane:
    height = 6.70
    return (
        cq.Workplane("XY")
        .circle(0.38)
        .workplane(offset=height)
        .circle(0.22)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.36))
    )


def _nacelle_body_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(1.52, 0.72, 0.58)
        .edges()
        .fillet(0.09)
        .translate((0.18, 0.0, 0.58))
    )


def _annular_ring_along_x(*, outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length + 0.004)
    return outer.cut(inner.translate((-0.002, 0.0, 0.0)))


def _nose_cone() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(0.27)
        .workplane(offset=0.43)
        .circle(0.018)
        .loft(combine=True)
        .translate((0.24, 0.0, 0.0))
    )


def _blade_shell() -> cq.Workplane:
    # Local blade frame: +Z is span/radial direction, X is aerodynamic thickness,
    # and Y is chord.  The root end intentionally overlaps the metal root insert
    # within the same blade part, making a continuous molded blade root.
    return (
        cq.Workplane("XY")
        .workplane(offset=0.24)
        .ellipse(0.060, 0.205)
        .workplane(offset=0.52)
        .ellipse(0.052, 0.185)
        .workplane(offset=0.62)
        .ellipse(0.040, 0.140)
        .workplane(offset=0.72)
        .ellipse(0.026, 0.075)
        .loft(combine=True)
    )


def _radial_origin(radius: float, angle: float) -> Origin:
    y = radius * math.sin(angle)
    z = radius * math.cos(angle)
    return Origin(xyz=(0.0, y, z), rpy=(-angle, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_wind_turbine")

    galvanized = model.material("galvanized_steel", rgba=(0.64, 0.67, 0.70, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.74, 0.76, 0.78, 1.0))
    concrete = model.material("sealed_concrete", rgba=(0.54, 0.54, 0.50, 1.0))
    nacelle_paint = model.material("weatherproof_white", rgba=(0.88, 0.90, 0.88, 1.0))
    blade_paint = model.material("gelcoat_blade_white", rgba=(0.93, 0.94, 0.90, 1.0))
    dark_seal = model.material("black_epdm_seal", rgba=(0.02, 0.025, 0.025, 1.0))
    warning_red = model.material("tip_marking_red", rgba=(0.78, 0.05, 0.03, 1.0))

    tower = model.part("tower")
    tower.visual(Cylinder(radius=1.35, length=0.28), origin=Origin(xyz=(0.0, 0.0, 0.14)), material=concrete, name="foundation_pad")
    tower.visual(Cylinder(radius=0.72, length=0.18), origin=Origin(xyz=(0.0, 0.0, 0.37)), material=galvanized, name="base_flange")
    tower.visual(mesh_from_cadquery(_tapered_tower_shell(), "tapered_tower_shell", tolerance=0.004), material=galvanized, name="tapered_tower_shell")
    tower.visual(Cylinder(radius=0.50, length=0.14), origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z - 0.07)), material=galvanized, name="top_yaw_flange")
    tower.visual(Cylinder(radius=0.43, length=0.018), origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z - 0.009)), material=dark_seal, name="top_gasket")
    for index in range(10):
        angle = index * math.tau / 10.0
        x = 0.56 * math.cos(angle)
        y = 0.56 * math.sin(angle)
        tower.visual(Cylinder(radius=0.030, length=0.20), origin=Origin(xyz=(x, y, 0.49)), material=stainless, name=f"anchor_stud_{index}")
        tower.visual(Cylinder(radius=0.055, length=0.045), origin=Origin(xyz=(x, y, 0.612)), material=stainless, name=f"anchor_nut_{index}")

    nacelle = model.part("nacelle")
    nacelle.visual(Cylinder(radius=0.42, length=0.11), origin=Origin(xyz=(0.0, 0.0, 0.055)), material=galvanized, name="yaw_bearing_base")
    nacelle.visual(Cylinder(radius=0.35, length=0.31), origin=Origin(xyz=(0.0, 0.0, 0.205)), material=nacelle_paint, name="sealed_yaw_skirt")
    nacelle.visual(mesh_from_cadquery(_nacelle_body_shell(), "nacelle_body_shell", tolerance=0.003), material=nacelle_paint, name="rounded_nacelle_shell")
    nacelle.visual(Box((1.68, 0.88, 0.07)), origin=Origin(xyz=(0.20, 0.0, 0.905)), material=nacelle_paint, name="drip_roof_overhang")
    nacelle.visual(Box((1.62, 0.040, 0.055)), origin=Origin(xyz=(0.20, 0.455, 0.8425)), material=nacelle_paint, name="side_drip_rail_0")
    nacelle.visual(Box((1.62, 0.040, 0.055)), origin=Origin(xyz=(0.20, -0.455, 0.8425)), material=nacelle_paint, name="side_drip_rail_1")
    nacelle.visual(
        mesh_from_cadquery(_annular_ring_along_x(outer_radius=0.39, inner_radius=0.235, length=0.12), "front_bearing_ring", tolerance=0.002),
        origin=Origin(xyz=(0.900, 0.0, ROTOR_AXIS_Z)),
        material=galvanized,
        name="front_bearing_ring",
    )
    nacelle.visual(
        mesh_from_cadquery(_annular_ring_along_x(outer_radius=0.345, inner_radius=0.250, length=0.020), "front_epdm_seal", tolerance=0.002),
        origin=Origin(xyz=(1.000, 0.0, ROTOR_AXIS_Z)),
        material=dark_seal,
        name="front_epdm_seal",
    )

    rotor = model.part("hub")
    rotor.visual(Cylinder(radius=0.30, length=0.48), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=galvanized, name="hub_barrel")
    rotor.visual(Cylinder(radius=0.315, length=0.030), origin=Origin(xyz=(-0.255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_seal, name="rear_labyrinth_seal")
    rotor.visual(mesh_from_cadquery(_nose_cone(), "spinner_nose_cone", tolerance=0.002), material=nacelle_paint, name="spinner_nose_cone")
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        socket_origin = _radial_origin(0.43, angle)
        flange_origin = _radial_origin(0.63, angle)
        rotor.visual(Cylinder(radius=0.150, length=0.34), origin=socket_origin, material=galvanized, name=f"root_socket_{index}")
        rotor.visual(Cylinder(radius=0.210, length=0.070), origin=flange_origin, material=galvanized, name=f"root_flange_{index}")

    blade_shell_mesh = mesh_from_cadquery(_blade_shell(), "tapered_blade_shell", tolerance=0.004)
    blades = []
    for index in range(3):
        blade = model.part(f"blade_{index}")
        blade.visual(Cylinder(radius=0.155, length=0.235), origin=Origin(xyz=(0.0, 0.0, 0.1825)), material=stainless, name="root_insert")
        blade.visual(Cylinder(radius=0.205, length=0.065), origin=Origin(xyz=(0.0, 0.0, 0.0325)), material=stainless, name="sealed_root_flange")
        for bolt_index in range(6):
            bolt_angle = bolt_index * math.tau / 6.0
            blade.visual(
                Cylinder(radius=0.018, length=0.018),
                origin=Origin(
                    xyz=(0.165 * math.cos(bolt_angle), 0.165 * math.sin(bolt_angle), 0.070)
                ),
                material=stainless,
                name=f"root_bolt_{bolt_index}",
            )
        blade.visual(blade_shell_mesh, material=blade_paint, name="blade_shell")
        blade.visual(Box((0.040, 0.135, 0.18)), origin=Origin(xyz=(0.0, 0.0, 2.02)), material=warning_red, name="tip_mark")
        blades.append(blade)

    yaw = model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=0.20),
        motion_properties=MotionProperties(damping=0.35, friction=0.04),
    )
    spin = model.articulation(
        "nacelle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(ROTOR_AXIS_X, 0.0, ROTOR_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.01),
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        model.articulation(
            f"hub_to_blade_{index}",
            ArticulationType.FIXED,
            parent=rotor,
            child=blades[index],
            origin=_radial_origin(BLADE_ROOT_RADIUS, angle),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("hub")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_hub")

    ctx.expect_contact(
        nacelle,
        tower,
        elem_a="yaw_bearing_base",
        elem_b="top_gasket",
        name="sealed yaw bearing sits on tower gasket",
    )
    ctx.allow_overlap(
        hub,
        nacelle,
        elem_a="rear_labyrinth_seal",
        elem_b="front_epdm_seal",
        reason="The rubber labyrinth seal is intentionally represented as lightly compressed at the protected front bearing face.",
    )
    ctx.expect_contact(
        hub,
        nacelle,
        elem_a="rear_labyrinth_seal",
        elem_b="front_epdm_seal",
        name="hub seal bears against protected nacelle interface",
    )
    ctx.expect_origin_gap(
        hub,
        nacelle,
        axis="x",
        min_gap=1.20,
        max_gap=1.38,
        name="rotor hub is carried in front of nacelle",
    )

    for index in range(3):
        blade = object_model.get_part(f"blade_{index}")
        ctx.expect_contact(
            blade,
            hub,
            elem_a="sealed_root_flange",
            elem_b=f"root_flange_{index}",
            name=f"blade {index} root flange seats on hub",
        )

    rest_hub = ctx.part_world_position(hub)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_hub = ctx.part_world_position(hub)
    ctx.check(
        "weather yaw turns nacelle about tower",
        rest_hub is not None
        and yawed_hub is not None
        and abs(yawed_hub[1] - rest_hub[1]) > 1.0
        and abs(yawed_hub[2] - rest_hub[2]) < 0.01,
        details=f"rest={rest_hub}, yawed={yawed_hub}",
    )

    blade_0 = object_model.get_part("blade_0")
    rest_blade = ctx.part_world_position(blade_0)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_blade = ctx.part_world_position(blade_0)
    ctx.check(
        "rotor spin carries blade roots around hub axis",
        rest_blade is not None
        and spun_blade is not None
        and abs(spun_blade[0] - rest_blade[0]) < 0.01
        and abs(spun_blade[1] - rest_blade[1]) > 0.50
        and abs(spun_blade[2] - rest_blade[2]) > 0.50,
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    return ctx.report()


object_model = build_object_model()

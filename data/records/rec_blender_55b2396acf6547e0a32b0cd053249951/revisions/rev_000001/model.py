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


def _rounded_base_body() -> cq.Workplane:
    """Wide appliance base with softly radiused vertical corners."""
    return (
        cq.Workplane("XY")
        .box(0.380, 0.280, 0.135, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.024)
    )


def _jar_shell() -> cq.Workplane:
    """Transparent hollow rectangular jar shell with an open wide mouth."""
    outer_x, outer_y, height = 0.230, 0.190, 0.385
    wall, bottom = 0.012, 0.035
    shell = (
        cq.Workplane("XY")
        .box(outer_x, outer_y, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(outer_x - 2.0 * wall, outer_y - 2.0 * wall)
        .cutBlind(-(height - bottom))
    )
    shaft_clearance = cq.Workplane("XY").circle(0.036).extrude(bottom + 0.012)
    return shell.cut(shaft_clearance)


def _annular_ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _fill_cap() -> cq.Workplane:
    """Screw cap with a top disk, down-skirt, external thread bands, and a grip boss."""
    cap = cq.Workplane("XY").circle(0.113).extrude(0.026)
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-0.028)
        .circle(0.113)
        .circle(0.106)
        .extrude(0.028)
    )
    cap = cap.union(skirt)
    for z in (-0.022, -0.014, -0.006):
        band = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(0.117)
            .circle(0.106)
            .extrude(0.003)
        )
        cap = cap.union(band)
    grip = cq.Workplane("XY").workplane(offset=0.026).circle(0.046).extrude(0.010)
    return cap.union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cold_press_smoothie_blender")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.055, 0.060, 0.067, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.69, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    polycarbonate = model.material("clear_polycarbonate", rgba=(0.72, 0.92, 1.00, 0.34))
    smoky_poly = model.material("smoky_polycarbonate", rgba=(0.40, 0.55, 0.62, 0.42))
    white_marking = model.material("etched_white", rgba=(0.92, 0.96, 1.00, 0.78))
    blade_steel = model.material("sharpened_steel", rgba=(0.86, 0.88, 0.86, 1.0))
    red_mark = model.material("red_index", rgba=(0.85, 0.06, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base_body(), "base_body", tolerance=0.0015),
        material=matte_black,
        name="base_body",
    )
    base.visual(
        Box((0.260, 0.010, 0.072)),
        origin=Origin(xyz=(0.0, -0.143, 0.074)),
        material=charcoal,
        name="front_panel",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=brushed_steel,
        name="bayonet_plate",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=brushed_steel,
        name="drive_socket",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        base.visual(
            Box((0.042, 0.014, 0.010)),
            origin=Origin(
                xyz=(0.079 * math.cos(angle), 0.079 * math.sin(angle), 0.162),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=brushed_steel,
            name=f"bayonet_lug_{i}",
        )
    for i, (x, y) in enumerate(((-0.145, -0.095), (0.145, -0.095), (-0.145, 0.095), (0.145, 0.095))):
        base.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"foot_{i}",
        )

    jar = model.part("jar")
    jar.visual(
        Cylinder(radius=0.080, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brushed_steel,
        name="bayonet_skirt",
    )
    jar.visual(
        mesh_from_cadquery(_annular_ring(0.069, 0.018, 0.020), "lower_collar", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=charcoal,
        name="lower_collar",
    )
    jar.visual(
        mesh_from_cadquery(_jar_shell(), "jar_shell", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=polycarbonate,
        name="jar_shell",
    )
    jar.visual(
        mesh_from_cadquery(_annular_ring(0.045, 0.014, 0.024), "blade_seal", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=charcoal,
        name="blade_seal",
    )
    jar.visual(
        mesh_from_cadquery(_annular_ring(0.103, 0.078, 0.032), "threaded_neck", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=polycarbonate,
        name="threaded_neck",
    )
    for i, z in enumerate((0.170, 0.220, 0.270, 0.320, 0.370)):
        tick_width = 0.052 if i % 2 == 0 else 0.034
        jar.visual(
            Box((tick_width, 0.0025, 0.004)),
            origin=Origin(xyz=(-0.063, -0.09625, z)),
            material=white_marking,
            name=f"measure_tick_{i}",
        )
    jar.visual(
        Box((0.066, 0.058, 0.022)),
        origin=Origin(xyz=(0.0, 0.116, 0.320)),
        material=smoky_poly,
        name="handle_upper_mount",
    )
    jar.visual(
        Box((0.066, 0.058, 0.022)),
        origin=Origin(xyz=(0.0, 0.116, 0.145)),
        material=smoky_poly,
        name="handle_lower_mount",
    )
    jar.visual(
        Box((0.046, 0.034, 0.202)),
        origin=Origin(xyz=(0.0, 0.149, 0.232)),
        material=smoky_poly,
        name="rear_handle",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blade_steel,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=blade_steel,
        name="drive_shaft",
    )
    for i in range(6):
        angle = i * math.pi / 3.0
        blade.visual(
            Box((0.066, 0.017, 0.0036)),
            origin=Origin(
                xyz=(0.041 * math.cos(angle), 0.041 * math.sin(angle), 0.002 * (-1) ** i),
                rpy=(0.30 * (-1) ** i, 0.0, angle),
            ),
            material=blade_steel,
            name=f"blade_{i}",
        )

    fill_cap = model.part("fill_cap")
    fill_cap.visual(
        mesh_from_cadquery(_fill_cap(), "fill_cap_body", tolerance=0.001),
        material=matte_black,
        name="cap_body",
    )
    fill_cap.visual(
        Box((0.040, 0.006, 0.003)),
        origin=Origin(xyz=(0.042, 0.0, 0.0365)),
        material=red_mark,
        name="cap_index_mark",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="dial_cap",
    )
    speed_dial.visual(
        Box((0.005, 0.0025, 0.021)),
        origin=Origin(xyz=(0.0, -0.019, 0.012)),
        material=red_mark,
        name="dial_pointer",
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.183)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=0.42),
        motion_properties=MotionProperties(damping=0.25, friction=0.08),
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=80.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "jar_to_fill_cap",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=fill_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.462)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=2.0 * math.pi),
        motion_properties=MotionProperties(damping=0.12, friction=0.25),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(-0.070, -0.148, 0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-2.25, upper=2.25),
        motion_properties=MotionProperties(damping=0.05, friction=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade = object_model.get_part("blade")
    fill_cap = object_model.get_part("fill_cap")
    speed_dial = object_model.get_part("speed_dial")

    bayonet = object_model.get_articulation("base_to_jar")
    blade_spin = object_model.get_articulation("jar_to_blade")
    cap_seal = object_model.get_articulation("jar_to_fill_cap")
    dial_joint = object_model.get_articulation("base_to_speed_dial")

    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem="bayonet_skirt",
        negative_elem="drive_socket",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="bayonet skirt seats on drive socket",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a="bayonet_skirt",
        elem_b="drive_socket",
        min_overlap=0.090,
        name="jar bayonet surrounds base coupling",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        inner_elem="blade_0",
        outer_elem="jar_shell",
        margin=0.0,
        name="blade sweep remains inside jar footprint",
    )
    ctx.expect_gap(
        blade,
        jar,
        axis="z",
        positive_elem="hub",
        negative_elem="blade_seal",
        min_gap=0.0015,
        max_gap=0.008,
        name="blade hub clears the bottom seal",
    )
    ctx.expect_contact(
        fill_cap,
        jar,
        elem_a="cap_body",
        elem_b="threaded_neck",
        contact_tol=0.0015,
        name="screw cap bears on threaded neck",
    )
    ctx.expect_overlap(
        fill_cap,
        jar,
        axes="xy",
        elem_a="cap_body",
        elem_b="threaded_neck",
        min_overlap=0.150,
        name="fill cap covers the wide mouth neck",
    )
    ctx.expect_gap(
        base,
        speed_dial,
        axis="y",
        positive_elem="front_panel",
        negative_elem="dial_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="speed dial is mounted proud of front panel",
    )

    ctx.check(
        "bayonet lock has limited twist travel",
        bayonet.motion_limits is not None
        and bayonet.motion_limits.lower == 0.0
        and bayonet.motion_limits.upper is not None
        and 0.30 <= bayonet.motion_limits.upper <= 0.55,
        details=f"limits={bayonet.motion_limits}",
    )
    ctx.check(
        "blade uses continuous rotary spin joint",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_spin.articulation_type}",
    )
    ctx.check(
        "fill cap unscrews through a full turn",
        cap_seal.motion_limits is not None
        and cap_seal.motion_limits.upper is not None
        and cap_seal.motion_limits.upper >= 2.0 * math.pi - 0.001,
        details=f"limits={cap_seal.motion_limits}",
    )
    ctx.check(
        "front speed control is articulated",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is not None
        and dial_joint.motion_limits.upper is not None
        and dial_joint.motion_limits.upper - dial_joint.motion_limits.lower > 4.0,
        details=f"limits={dial_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()

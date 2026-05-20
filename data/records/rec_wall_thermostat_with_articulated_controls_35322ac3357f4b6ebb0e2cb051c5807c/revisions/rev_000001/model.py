from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    KnobTopFeature,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """Weatherproof enclosure shell: rectangular, but molded with rounded corners."""
    depth, width, height = size
    body = cq.Workplane("XY").box(depth, width, height)
    if radius > 0.0:
        body = body.edges("|X").fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.0007, angular_tolerance=0.08)


def _annular_ring_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Annular seal/bushing, built along local +Z so visuals can rotate it onto +X."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return mesh_from_cadquery(ring, name, tolerance=0.0005, angular_tolerance=0.06)


def _front_plate_mesh(name: str):
    """Thin sealed front skin with real openings for the rotary shaft and buttons."""
    plate = cq.Workplane("XY").box(0.010, 0.168, 0.222)
    for y, z, radius in ((0.0, 0.0, 0.033), (-0.038, -0.083, 0.010), (0.038, -0.083, 0.010)):
        cutter = cq.Workplane("YZ").center(y, z).circle(radius).extrude(0.030, both=True)
        plate = plate.cut(cutter)
    return mesh_from_cadquery(plate, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_wall_thermostat")

    body_plastic = model.material("uv_stabilized_polycarbonate", color=(0.16, 0.18, 0.18, 1.0))
    dark_plastic = model.material("dark_display_bezel", color=(0.02, 0.025, 0.03, 1.0))
    rubber = model.material("black_epdm_seal", color=(0.005, 0.006, 0.005, 1.0))
    dial_plastic = model.material("matte_light_dial", color=(0.78, 0.80, 0.78, 1.0))
    marker_red = model.material("red_temperature_marker", color=(0.9, 0.06, 0.02, 1.0))
    stainless = model.material("passivated_stainless", color=(0.72, 0.74, 0.72, 1.0))
    glass = model.material("smoked_polycarbonate_window", color=(0.08, 0.14, 0.18, 0.55))
    label = model.material("screen_printed_scale", color=(0.88, 0.92, 0.88, 1.0))

    # Root assembly: the wall-mounted weatherproof enclosure, molded overhangs,
    # static seals, bushing, display window, and corrosion-resistant fasteners.
    body = model.part("body")
    body.visual(
        Box((0.012, 0.205, 0.265)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=body_plastic,
        name="backplate",
    )
    body.visual(
        _rounded_box_mesh((0.060, 0.180, 0.240), 0.018, "rounded_housing"),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=body_plastic,
        name="housing_shell",
    )
    body.visual(
        _front_plate_mesh("front_plate"),
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
        material=body_plastic,
        name="front_plate",
    )
    # Drip-aware roof/side geometry: overhanging rain lip and molded side gutters.
    body.visual(
        Box((0.082, 0.210, 0.014)),
        origin=Origin(xyz=(0.025, 0.0, 0.127)),
        material=body_plastic,
        name="rain_hood",
    )
    body.visual(
        Box((0.006, 0.210, 0.006)),
        origin=Origin(xyz=(0.067, 0.0, 0.117)),
        material=body_plastic,
        name="drip_break",
    )
    for y, name in ((-0.092, "side_gutter_0"), (0.092, "side_gutter_1")):
        body.visual(
            Box((0.064, 0.010, 0.216)),
            origin=Origin(xyz=(0.024, y, -0.006)),
            material=body_plastic,
            name=name,
        )

    # Recessed front gasket frame makes the closed shell read as weatherproof.
    body.visual(Box((0.004, 0.150, 0.006)), origin=Origin(xyz=(0.052, 0.0, 0.104)), material=rubber, name="top_gasket")
    body.visual(Box((0.004, 0.150, 0.006)), origin=Origin(xyz=(0.052, 0.0, -0.112)), material=rubber, name="bottom_gasket")
    body.visual(Box((0.004, 0.006, 0.210)), origin=Origin(xyz=(0.052, -0.075, -0.004)), material=rubber, name="gasket_rail_0")
    body.visual(Box((0.004, 0.006, 0.210)), origin=Origin(xyz=(0.052, 0.075, -0.004)), material=rubber, name="gasket_rail_1")

    # Smoked display window with a raised sealed bezel above the dial.
    body.visual(
        Box((0.004, 0.096, 0.038)),
        origin=Origin(xyz=(0.053, 0.0, 0.065)),
        material=dark_plastic,
        name="display_bezel",
    )
    body.visual(
        Box((0.003, 0.082, 0.026)),
        origin=Origin(xyz=(0.056, 0.0, 0.065)),
        material=glass,
        name="display_window",
    )
    for y, name in ((-0.052, "display_side_seal_0"), (0.052, "display_side_seal_1")):
        body.visual(
            Box((0.004, 0.004, 0.044)),
            origin=Origin(xyz=(0.057, y, 0.065)),
            material=rubber,
            name=name,
        )

    # Dial shaft protection: an EPDM boot and hard acetal retainer/bushing ring.
    body.visual(
        _annular_ring_mesh(0.060, 0.036, 0.006, "dial_boot_ring"),
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="dial_boot",
    )
    body.visual(
        _annular_ring_mesh(0.055, 0.038, 0.007, "dial_retainer_ring"),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="dial_seal",
    )

    # Temperature scale ticks live on the sealed front panel outside the dial sweep.
    for i in range(13):
        angle = -120.0 + i * 20.0
        r = 0.071
        y = r * math.sin(math.radians(angle))
        z = r * math.cos(math.radians(angle))
        tick_len = 0.014 if i % 3 == 0 else 0.009
        tick_w = 0.0026 if i % 3 == 0 else 0.0018
        body.visual(
            Box((0.0018, tick_w, tick_len)),
            origin=Origin(
                xyz=(0.0584, y, z),
                rpy=(math.radians(angle), 0.0, 0.0),
            ),
            material=label,
            name=f"scale_tick_{i}",
        )

    # Protected, gasketed lower pushbutton seats.
    body.visual(
        _annular_ring_mesh(0.020, 0.012, 0.005, "mode_button_seal_mesh"),
        origin=Origin(xyz=(0.054, -0.038, -0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="mode_button_seal",
    )
    body.visual(
        _annular_ring_mesh(0.020, 0.012, 0.005, "set_button_seal_mesh"),
        origin=Origin(xyz=(0.054, 0.038, -0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="set_button_seal",
    )

    # Four exposed stainless fasteners clamp the gasketed housing to the wall plate.
    for y in (-0.075, 0.075):
        for z in (-0.100, 0.100):
            screw_name = f"screw_{0 if y < 0 else 1}_{0 if z < 0 else 1}"
            body.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(0.053, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=stainless,
                name=screw_name,
            )
            body.visual(
                Box((0.002, 0.012, 0.0015)),
                origin=Origin(xyz=(0.056, y, z), rpy=(0.0, 0.0, 0.0)),
                material=dark_plastic,
                name=f"{screw_name}_slot",
            )

    # Main center-axis rotating thermostat dial. The part frame is the shaft axis
    # and sits on the retainer front face; all geometry extends outward.
    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.106,
            0.024,
            body_style="skirted",
            base_diameter=0.108,
            top_diameter=0.088,
            edge_radius=0.0015,
            side_draft_deg=4.0,
            skirt=KnobSkirt(0.112, 0.005, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=36, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            top_feature=KnobTopFeature(style="top_recess", diameter=0.026, height=0.0012),
            bore=KnobBore(style="round", diameter=0.012),
            center=False,
        ),
        "sealed_rotary_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_plastic,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.006, length=0.031),
        origin=Origin(xyz=(0.0155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.015, length=0.003),
        origin=Origin(xyz=(0.0255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="retainer",
    )
    dial.visual(
        Box((0.0020, 0.006, 0.034)),
        origin=Origin(xyz=(0.027, 0.0, 0.031)),
        material=marker_red,
        name="pointer_marker",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-2.35, upper=2.35),
    )

    # Two sealed pushbuttons use very short inward travel under EPDM caps.
    for button_name, y, mark in (("mode_button", -0.038, "M"), ("set_button", 0.038, "S")):
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="rubber_cap",
        )
        button.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.008, 0.0, 0.0)),
            material=rubber,
            name="domed_face",
        )
        button.visual(
            Box((0.0015, 0.010, 0.0015)),
            origin=Origin(xyz=(0.015, 0.0, 0.0)),
            material=label,
            name=f"{mark.lower()}_mark",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.059, y, -0.083)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    mode_button = object_model.get_part("mode_button")
    set_button = object_model.get_part("set_button")
    dial_joint = object_model.get_articulation("body_to_dial")
    mode_joint = object_model.get_articulation("body_to_mode_button")
    set_joint = object_model.get_articulation("body_to_set_button")

    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="dial_seal",
        contact_tol=0.001,
        name="dial skirt bears on sealed retainer",
    )
    ctx.expect_within(
        dial,
        body,
        axes="yz",
        inner_elem="shaft",
        outer_elem="dial_seal",
        margin=0.0,
        name="shaft centered through retainer bore",
    )
    ctx.expect_contact(
        mode_button,
        body,
        elem_a="rubber_cap",
        elem_b="mode_button_seal",
        contact_tol=0.001,
        name="mode cap seated on seal",
    )
    ctx.expect_contact(
        set_button,
        body,
        elem_a="rubber_cap",
        elem_b="set_button_seal",
        contact_tol=0.001,
        name="set cap seated on seal",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="pointer_marker")
    with ctx.pose({dial_joint: 1.0}):
        pointer_turned = ctx.part_element_world_aabb(dial, elem="pointer_marker")
    ctx.check(
        "dial rotates pointer around shaft",
        pointer_rest is not None
        and pointer_turned is not None
        and abs(((pointer_rest[0][1] + pointer_rest[1][1]) * 0.5) - ((pointer_turned[0][1] + pointer_turned[1][1]) * 0.5)) > 0.010,
        details=f"rest={pointer_rest}, turned={pointer_turned}",
    )

    mode_rest = ctx.part_world_position(mode_button)
    set_rest = ctx.part_world_position(set_button)
    with ctx.pose({mode_joint: 0.004, set_joint: 0.004}):
        mode_pressed = ctx.part_world_position(mode_button)
        set_pressed = ctx.part_world_position(set_button)
    ctx.check(
        "buttons travel inward toward body",
        mode_rest is not None
        and mode_pressed is not None
        and set_rest is not None
        and set_pressed is not None
        and mode_pressed[0] < mode_rest[0] - 0.003
        and set_pressed[0] < set_rest[0] - 0.003,
        details=f"mode={mode_rest}->{mode_pressed}, set={set_rest}->{set_pressed}",
    )

    return ctx.report()


object_model = build_object_model()

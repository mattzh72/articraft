from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CARD_LENGTH = 0.290
CARD_WIDTH = 0.108
PCB_THICKNESS = 0.004

SHROUD_BOTTOM_Z = 0.025
SHROUD_THICKNESS = 0.012
SHROUD_TOP_Z = SHROUD_BOTTOM_Z + SHROUD_THICKNESS

LARGE_FAN_CENTER = (-0.026, 0.006, 0.035)
SIDE_FAN_CENTER = (0.096, -0.014, 0.035)
LARGE_ROTOR_RADIUS = 0.043
SIDE_ROTOR_RADIUS = 0.028
LARGE_OPENING_RADIUS = 0.050
SIDE_OPENING_RADIUS = 0.034


def _asymmetric_shroud_mesh():
    """Single fixed cover plate with two true circular fan openings."""
    outline = [
        (-0.128, -0.048),
        (-0.108, -0.061),
        (0.050, -0.061),
        (0.142, -0.047),
        (0.153, 0.025),
        (0.112, 0.058),
        (-0.066, 0.064),
        (-0.126, 0.043),
    ]
    shroud = cq.Workplane("XY").polyline(outline).close().extrude(SHROUD_THICKNESS)

    for x, y, radius in (
        (LARGE_FAN_CENTER[0], LARGE_FAN_CENTER[1], LARGE_OPENING_RADIUS),
        (SIDE_FAN_CENTER[0], SIDE_FAN_CENTER[1], SIDE_OPENING_RADIUS),
    ):
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(radius)
            .extrude(SHROUD_THICKNESS * 3.0)
            .translate((0.0, 0.0, -SHROUD_THICKNESS))
        )
        shroud = shroud.cut(cutter)

    return shroud.edges("|Z").fillet(0.0025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="asymmetric_dual_slot_graphics_card")

    pcb_green = model.material("pcb_green", rgba=(0.02, 0.22, 0.12, 1.0))
    copper = model.material("copper_edge", rgba=(0.95, 0.67, 0.25, 1.0))
    heatsink = model.material("brushed_heatsink", rgba=(0.58, 0.60, 0.58, 1.0))
    fin_dark = model.material("shadowed_fin", rgba=(0.22, 0.23, 0.24, 1.0))
    shroud_black = model.material("satin_black_shroud", rgba=(0.015, 0.016, 0.018, 1.0))
    trim = model.material("graphite_trim", rgba=(0.055, 0.060, 0.068, 1.0))
    blade_mat = model.material("matte_black_rotor", rgba=(0.01, 0.011, 0.012, 1.0))
    metal = model.material("io_bracket_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    port_black = model.material("dark_io_ports", rgba=(0.02, 0.022, 0.025, 1.0))
    accent = model.material("subtle_red_accent", rgba=(0.55, 0.035, 0.025, 1.0))

    card = model.part("card")

    card.visual(
        Box((CARD_LENGTH, CARD_WIDTH, PCB_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PCB_THICKNESS / 2.0)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.012, CARD_WIDTH + 0.004, 0.0014)),
        origin=Origin(xyz=(0.138, 0.0, PCB_THICKNESS + 0.0007)),
        material=copper,
        name="gold_fingers",
    )

    card.visual(
        Box((0.244, 0.096, 0.021)),
        origin=Origin(xyz=(0.016, 0.000, 0.0145)),
        material=heatsink,
        name="heatsink_slab",
    )
    for index, y in enumerate((-0.039, -0.028, -0.017, -0.006, 0.005, 0.016, 0.027, 0.038)):
        card.visual(
            Box((0.228, 0.0028, 0.0045)),
            origin=Origin(xyz=(0.020, y, 0.02725)),
            material=fin_dark if index % 2 else heatsink,
            name=f"fin_{index}",
        )

    card.visual(
        mesh_from_cadquery(_asymmetric_shroud_mesh(), "asymmetric_shroud"),
        origin=Origin(xyz=(0.0, 0.0, SHROUD_BOTTOM_Z)),
        material=shroud_black,
        name="shroud",
    )

    large_ring = BezelGeometry(
        (LARGE_OPENING_RADIUS * 2.0, LARGE_OPENING_RADIUS * 2.0),
        ((LARGE_OPENING_RADIUS + 0.006) * 2.0, (LARGE_OPENING_RADIUS + 0.006) * 2.0),
        0.004,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    side_ring = BezelGeometry(
        (SIDE_OPENING_RADIUS * 2.0, SIDE_OPENING_RADIUS * 2.0),
        ((SIDE_OPENING_RADIUS + 0.005) * 2.0, (SIDE_OPENING_RADIUS + 0.005) * 2.0),
        0.004,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    card.visual(
        mesh_from_geometry(large_ring, "large_fan_ring"),
        origin=Origin(xyz=(LARGE_FAN_CENTER[0], LARGE_FAN_CENTER[1], SHROUD_TOP_Z + 0.002)),
        material=trim,
        name="large_ring",
    )
    card.visual(
        mesh_from_geometry(side_ring, "side_fan_ring"),
        origin=Origin(xyz=(SIDE_FAN_CENTER[0], SIDE_FAN_CENTER[1], SHROUD_TOP_Z + 0.002)),
        material=trim,
        name="side_ring",
    )

    # Fixed hub pins rise from the heatsink through the rotor hub bores, giving
    # each continuous joint a visible captured axle without touching the blades.
    card.visual(
        Cylinder(radius=0.0036, length=0.019),
        origin=Origin(xyz=LARGE_FAN_CENTER),
        material=metal,
        name="large_axle",
    )
    card.visual(
        Cylinder(radius=0.0030, length=0.017),
        origin=Origin(xyz=SIDE_FAN_CENTER),
        material=metal,
        name="side_axle",
    )

    card.visual(
        Box((0.048, 0.005, 0.0015)),
        origin=Origin(xyz=(0.118, 0.039, SHROUD_TOP_Z + 0.00075), rpy=(0.0, 0.0, -0.24)),
        material=accent,
        name="red_accent",
    )

    card.visual(
        Box((0.006, 0.123, 0.086)),
        origin=Origin(xyz=(-0.148, 0.0, 0.043)),
        material=metal,
        name="io_bracket",
    )
    card.visual(
        Box((0.0014, 0.022, 0.010)),
        origin=Origin(xyz=(-0.1517, -0.033, 0.040)),
        material=port_black,
        name="display_port_0",
    )
    card.visual(
        Box((0.0014, 0.022, 0.010)),
        origin=Origin(xyz=(-0.1517, -0.006, 0.040)),
        material=port_black,
        name="display_port_1",
    )
    card.visual(
        Box((0.0014, 0.016, 0.020)),
        origin=Origin(xyz=(-0.1517, 0.026, 0.046)),
        material=port_black,
        name="hdmi_port",
    )
    card.visual(
        Box((0.0014, 0.096, 0.006)),
        origin=Origin(xyz=(-0.1517, 0.0, 0.074)),
        material=port_black,
        name="bracket_vent",
    )

    center_fan = model.part("center_fan")
    center_fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                LARGE_ROTOR_RADIUS,
                0.0145,
                11,
                thickness=0.007,
                blade_pitch_deg=32.0,
                blade_sweep_deg=31.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18, tip_clearance=0.0015),
                hub=FanRotorHub(style="capped", rear_collar_height=0.004, rear_collar_radius=0.011, bore_diameter=0.0072),
            ),
            "center_fan_rotor",
        ),
        material=blade_mat,
        name="rotor",
    )

    side_fan = model.part("side_fan")
    side_fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                SIDE_ROTOR_RADIUS,
                0.010,
                9,
                thickness=0.006,
                blade_pitch_deg=34.0,
                blade_sweep_deg=25.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=16.0, camber=0.14, tip_clearance=0.0012),
                hub=FanRotorHub(style="domed", rear_collar_height=0.003, rear_collar_radius=0.008, bore_diameter=0.0060),
            ),
            "side_fan_rotor",
        ),
        material=blade_mat,
        name="rotor",
    )

    model.articulation(
        "card_to_center_fan",
        ArticulationType.CONTINUOUS,
        parent=card,
        child=center_fan,
        origin=Origin(xyz=LARGE_FAN_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=160.0),
    )
    model.articulation(
        "card_to_side_fan",
        ArticulationType.CONTINUOUS,
        parent=card,
        child=side_fan,
        origin=Origin(xyz=SIDE_FAN_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.06, velocity=180.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card")
    center_fan = object_model.get_part("center_fan")
    side_fan = object_model.get_part("side_fan")
    center_joint = object_model.get_articulation("card_to_center_fan")
    side_joint = object_model.get_articulation("card_to_side_fan")

    ctx.check(
        "both fan rotors are continuous",
        center_joint.articulation_type == ArticulationType.CONTINUOUS
        and side_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"center={center_joint.articulation_type}, side={side_joint.articulation_type}",
    )
    ctx.check(
        "fan axes are normal to card face",
        tuple(center_joint.axis) == (0.0, 0.0, 1.0) and tuple(side_joint.axis) == (0.0, 0.0, 1.0),
        details=f"center_axis={center_joint.axis}, side_axis={side_joint.axis}",
    )
    ctx.check(
        "fan layout is offset and asymmetric",
        abs(center_joint.origin.xyz[0] - side_joint.origin.xyz[0]) > 0.10
        and abs(center_joint.origin.xyz[1] - side_joint.origin.xyz[1]) > 0.015,
        details=f"center={center_joint.origin.xyz}, side={side_joint.origin.xyz}",
    )

    ctx.expect_within(
        center_fan,
        card,
        axes="xy",
        inner_elem="rotor",
        outer_elem="large_ring",
        margin=0.0,
        name="large rotor is inside its shroud ring",
    )
    ctx.expect_within(
        side_fan,
        card,
        axes="xy",
        inner_elem="rotor",
        outer_elem="side_ring",
        margin=0.0,
        name="small rotor is inside its shroud ring",
    )
    ctx.expect_overlap(
        center_fan,
        card,
        axes="xy",
        elem_a="rotor",
        elem_b="large_axle",
        min_overlap=0.004,
        name="large hub stays on captured center axle",
    )
    ctx.expect_overlap(
        side_fan,
        card,
        axes="xy",
        elem_a="rotor",
        elem_b="side_axle",
        min_overlap=0.003,
        name="small hub stays on captured center axle",
    )

    center_rest = ctx.part_world_position(center_fan)
    side_rest = ctx.part_world_position(side_fan)
    with ctx.pose({center_joint: 1.7, side_joint: -2.1}):
        center_spun = ctx.part_world_position(center_fan)
        side_spun = ctx.part_world_position(side_fan)

    ctx.check(
        "continuous spin keeps rotors captured",
        center_rest is not None
        and side_rest is not None
        and center_spun is not None
        and side_spun is not None
        and math.dist(center_rest, center_spun) < 1e-6
        and math.dist(side_rest, side_spun) < 1e-6,
        details=f"center_rest={center_rest}, center_spun={center_spun}, side_rest={side_rest}, side_spun={side_spun}",
    )

    return ctx.report()


object_model = build_object_model()

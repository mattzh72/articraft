from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


FAN_CENTERS_X = (-0.102, 0.0, 0.102)


def _shroud_shape() -> cq.Workplane:
    """Low-profile triple-fan shroud with a chamfered/angled outline and holes."""
    length = 0.326
    width = 0.136
    thickness = 0.010
    half_l = length / 2.0
    half_w = width / 2.0
    outline = [
        (-half_l + 0.016, -half_w),
        (half_l - 0.030, -half_w),
        (half_l, -half_w + 0.020),
        (half_l - 0.012, half_w),
        (-half_l + 0.030, half_w),
        (-half_l, half_w - 0.018),
        (-half_l, -half_w + 0.020),
    ]
    body = cq.Workplane("XY").polyline(outline).close().extrude(thickness)
    cutters = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .pushPoints([(x, 0.0) for x in FAN_CENTERS_X])
        .circle(0.046)
        .extrude(thickness + 0.004)
    )
    return body.cut(cutters).edges("|Z").fillet(0.002)


def _backplate_shape() -> cq.Workplane:
    """Rigid rear backplate with clipped corners."""
    length = 0.318
    width = 0.112
    thickness = 0.004
    half_l = length / 2.0
    half_w = width / 2.0
    outline = [
        (-half_l + 0.012, -half_w),
        (half_l - 0.020, -half_w),
        (half_l, -half_w + 0.014),
        (half_l - 0.008, half_w),
        (-half_l + 0.020, half_w),
        (-half_l, half_w - 0.014),
        (-half_l, -half_w + 0.010),
    ]
    return cq.Workplane("XY").polyline(outline).close().extrude(thickness).edges("|Z").fillet(0.0015)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    satin_black = model.material("satin_black", rgba=(0.03, 0.032, 0.037, 1.0))
    graphite = model.material("graphite_backplate", rgba=(0.10, 0.105, 0.115, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    pcb_black = model.material("black_pcb", rgba=(0.005, 0.035, 0.025, 1.0))
    gold = model.material("gold_contacts", rgba=(0.95, 0.70, 0.20, 1.0))
    port_black = model.material("port_black", rgba=(0.0, 0.0, 0.0, 1.0))
    accent = model.material("silver_edge_trim", rgba=(0.55, 0.58, 0.60, 1.0))

    card = model.part("card")

    # Flat PCB and the rigid backplate on the rear side.
    card.visual(
        Box((0.340, 0.118, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pcb_black,
        name="pcb",
    )
    card.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate"),
        origin=Origin(xyz=(0.004, 0.0, -0.009)),
        material=graphite,
        name="backplate",
    )

    # Gold PCIe edge connector along the lower long edge of the card.
    card.visual(
        Box((0.205, 0.008, 0.0025)),
        origin=Origin(xyz=(0.028, -0.061, -0.0006)),
        material=gold,
        name="pcie_edge",
    )
    for i, x in enumerate((-0.066, -0.044, -0.022, 0.000, 0.022, 0.044, 0.066, 0.088)):
        card.visual(
            Box((0.010, 0.009, 0.0030)),
            origin=Origin(xyz=(x, -0.062, -0.0002)),
            material=gold,
            name=f"contact_{i}",
        )

    # Heat spreader and fin stack, visible through the fan openings.
    card.visual(
        Box((0.302, 0.096, 0.007)),
        origin=Origin(xyz=(0.008, 0.0, 0.0060)),
        material=dark_metal,
        name="cold_plate",
    )
    for i in range(19):
        y = -0.045 + i * 0.005
        card.visual(
            Box((0.298, 0.0015, 0.018)),
            origin=Origin(xyz=(0.009, y, 0.0175)),
            material=aluminum,
            name=f"fin_{i}",
        )

    # Side skirts lift the angled shroud off the heat sink and make it a rigid shell.
    card.visual(
        Box((0.304, 0.009, 0.024)),
        origin=Origin(xyz=(0.005, -0.065, 0.0170)),
        material=satin_black,
        name="lower_skirt",
    )
    card.visual(
        Box((0.304, 0.009, 0.024)),
        origin=Origin(xyz=(0.005, 0.065, 0.0170)),
        material=satin_black,
        name="upper_skirt",
    )
    card.visual(
        Box((0.010, 0.118, 0.022)),
        origin=Origin(xyz=(0.165, 0.0, 0.0170)),
        material=satin_black,
        name="end_skirt",
    )
    card.visual(
        mesh_from_cadquery(_shroud_shape(), "angled_shroud"),
        origin=Origin(xyz=(0.004, 0.0, 0.0250)),
        material=matte_black,
        name="angled_shroud",
    )

    # Angular raised trim strips echo the faceted outer shroud.
    card.visual(
        Box((0.092, 0.004, 0.003)),
        origin=Origin(xyz=(-0.112, 0.054, 0.0365), rpy=(0.0, 0.0, 0.24)),
        material=accent,
        name="trim_0",
    )
    card.visual(
        Box((0.106, 0.004, 0.003)),
        origin=Origin(xyz=(0.112, -0.054, 0.0365), rpy=(0.0, 0.0, 0.24)),
        material=accent,
        name="trim_1",
    )

    # Stationary bearing bosses and cross-spokes in each opening support the rotors.
    for i, x in enumerate(FAN_CENTERS_X):
        card.visual(
            Cylinder(radius=0.010, length=0.009),
            origin=Origin(xyz=(x + 0.004, 0.0, 0.0285)),
            material=dark_metal,
            name=f"bearing_{i}",
        )
        card.visual(
            Cylinder(radius=0.0018, length=0.015),
            origin=Origin(xyz=(x + 0.004, 0.0, 0.0395)),
            material=dark_metal,
            name=f"axle_{i}",
        )
        card.visual(
            Box((0.076, 0.004, 0.0035)),
            origin=Origin(xyz=(x + 0.004, 0.0, 0.0310)),
            material=dark_metal,
            name=f"spoke_x_{i}",
        )
        card.visual(
            Box((0.004, 0.076, 0.0035)),
            origin=Origin(xyz=(x + 0.004, 0.0, 0.0310)),
            material=dark_metal,
            name=f"spoke_y_{i}",
        )

    # Six backplate standoffs and visible screw heads tie the rear plate to the PCB.
    screw_points = [(-0.116, -0.040), (-0.116, 0.040), (0.000, -0.042), (0.000, 0.042), (0.116, -0.040), (0.116, 0.040)]
    for i, (x, y) in enumerate(screw_points):
        card.visual(
            Cylinder(radius=0.0042, length=0.012),
            origin=Origin(xyz=(x, y, -0.0030)),
            material=dark_metal,
            name=f"standoff_{i}",
        )
        card.visual(
            Cylinder(radius=0.0062, length=0.0020),
            origin=Origin(xyz=(x, y, -0.0098)),
            material=aluminum,
            name=f"screw_{i}",
        )

    # Metal I/O bracket at the short end, with dark port insets and a small flange.
    card.visual(
        Box((0.006, 0.160, 0.062)),
        origin=Origin(xyz=(-0.171, 0.0, 0.0170)),
        material=aluminum,
        name="bracket_plate",
    )
    card.visual(
        Box((0.035, 0.018, 0.006)),
        origin=Origin(xyz=(-0.153, -0.052, 0.0020)),
        material=aluminum,
        name="bracket_flange",
    )
    for i, (y, z, w, h) in enumerate(((-0.042, 0.027, 0.022, 0.009), (-0.012, 0.027, 0.022, 0.009), (0.020, 0.025, 0.026, 0.011), (0.052, 0.025, 0.026, 0.011))):
        card.visual(
            Box((0.0015, w, h)),
            origin=Origin(xyz=(-0.1742, y, z)),
            material=port_black,
            name=f"port_{i}",
        )

    rotor_mesh = FanRotorGeometry(
        outer_radius=0.039,
        hub_radius=0.0125,
        blade_count=9,
        thickness=0.008,
        blade_pitch_deg=31.0,
        blade_sweep_deg=23.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.13, tip_clearance=0.0015),
        hub=FanRotorHub(style="domed", rear_collar_height=0.003, rear_collar_radius=0.011, bore_diameter=0.003),
    )
    fan_material = Material("fan_black", rgba=(0.01, 0.01, 0.012, 1.0))
    for i, x in enumerate(FAN_CENTERS_X):
        rotor = model.part(f"fan_{i}")
        rotor.visual(
            mesh_from_geometry(rotor_mesh, f"fan_rotor_{i}"),
            origin=Origin(),
            material=fan_material,
            name="rotor",
        )
        model.articulation(
            f"fan_joint_{i}",
            ArticulationType.CONTINUOUS,
            parent=card,
            child=rotor,
            origin=Origin(xyz=(x + 0.004, 0.0, 0.0425)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=80.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card")

    fan_positions = []
    for i in range(3):
        fan = object_model.get_part(f"fan_{i}")
        joint = object_model.get_articulation(f"fan_joint_{i}")
        fan_positions.append(ctx.part_world_position(fan))
        ctx.allow_overlap(
            card,
            fan,
            elem_a=f"axle_{i}",
            elem_b="rotor",
            reason="A small stationary axle is intentionally captured inside the rotor hub bearing.",
        )
        ctx.check(
            f"fan {i} has axial spin joint",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_within(
            card,
            fan,
            axes="xy",
            margin=0.0005,
            inner_elem=f"axle_{i}",
            outer_elem="rotor",
            name=f"fan {i} axle is centered in hub",
        )
        ctx.expect_overlap(
            card,
            fan,
            axes="z",
            min_overlap=0.006,
            elem_a=f"axle_{i}",
            elem_b="rotor",
            name=f"fan {i} axle passes through hub",
        )
        ctx.expect_gap(
            fan,
            card,
            axis="z",
            min_gap=0.0005,
            max_gap=0.020,
            positive_elem="rotor",
            negative_elem="angled_shroud",
            name=f"fan {i} clears shroud axially",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            margin=0.000,
            inner_elem="rotor",
            outer_elem="angled_shroud",
            name=f"fan {i} sits inside shroud footprint",
        )

    if all(position is not None for position in fan_positions):
        x_positions = [position[0] for position in fan_positions if position is not None]
        spacing_0 = x_positions[1] - x_positions[0]
        spacing_1 = x_positions[2] - x_positions[1]
        ctx.check(
            "fan openings are evenly spaced",
            abs(spacing_0 - spacing_1) < 1e-6 and 0.095 < spacing_0 < 0.110,
            details=f"spacings=({spacing_0:.4f}, {spacing_1:.4f})",
        )
    else:
        ctx.fail("fan positions available", details=f"positions={fan_positions}")

    return ctx.report()


object_model = build_object_model()

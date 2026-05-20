from __future__ import annotations

import math

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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


FAN_CENTERS = ((-0.106, 0.006), (0.0, 0.006), (0.106, 0.006))
HINGE_XYZ = (-0.012, -0.078, 0.020)


def _fan_shroud_cq() -> cq.Workplane:
    """Single front shroud plate with three real circular fan cutouts."""
    length = 0.335
    height = 0.132
    plate_thickness = 0.010
    cutout_radius = 0.043
    ring_outer = 0.050
    ring_depth = 0.018

    body = (
        cq.Workplane("XY")
        .box(length, height, plate_thickness)
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(FAN_CENTERS)
        .circle(cutout_radius)
        .cutThruAll()
    )

    for x, y in FAN_CENTERS:
        ring = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(ring_outer)
            .circle(cutout_radius)
            .extrude(ring_depth)
            .translate((0.0, 0.0, plate_thickness / 2.0))
        )
        body = body.union(ring)

    return body


def _add_root_body(card, materials) -> None:
    """Fixed PCB, heat sink, shroud, edge connector, and hinge hardware."""
    black = materials["black"]
    dark = materials["dark"]
    pcb_green = materials["pcb_green"]
    aluminum = materials["aluminum"]
    copper = materials["copper"]
    gold = materials["gold"]
    steel = materials["steel"]

    card.visual(
        Box((0.323, 0.115, 0.004)),
        origin=Origin(xyz=(0.006, 0.000, -0.022)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.330, 0.124, 0.0035)),
        origin=Origin(xyz=(0.002, 0.000, -0.024)),
        material=dark,
        name="backplate",
    )
    card.visual(
        Box((0.295, 0.102, 0.024)),
        origin=Origin(xyz=(0.012, 0.004, -0.005)),
        material=aluminum,
        name="heatsink_base",
    )

    # Dense visible fin pack behind the fan openings.
    for i, x in enumerate([-0.122, -0.100, -0.078, -0.056, -0.034, -0.012, 0.010, 0.032, 0.054, 0.076, 0.098, 0.120]):
        card.visual(
            Box((0.0045, 0.092, 0.030)),
            origin=Origin(xyz=(x, 0.004, -0.001)),
            material=aluminum,
            name=f"fin_{i}",
        )

    for i, y in enumerate((-0.030, 0.030)):
        card.visual(
            Cylinder(radius=0.0038, length=0.270),
            origin=Origin(xyz=(0.010, y, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=copper,
            name=f"heatpipe_{i}",
        )

    card.visual(
        mesh_from_cadquery(_fan_shroud_cq(), "triple_fan_shroud", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=black,
        name="shroud_shell",
    )

    # Angular accent ribs make the shroud read as a molded GPU cooler, not a plain slab.
    for i, (x, yaw) in enumerate(((-0.145, -0.18), (-0.052, 0.14), (0.052, -0.14), (0.145, 0.18))):
        card.visual(
            Box((0.052, 0.007, 0.003)),
            origin=Origin(xyz=(x, 0.057, 0.0185), rpy=(0.0, 0.0, yaw)),
            material=dark,
            name=f"top_rib_{i}",
        )

    card.visual(
        Box((0.012, 0.145, 0.030)),
        origin=Origin(xyz=(-0.171, 0.000, -0.004)),
        material=steel,
        name="io_bracket",
    )
    card.visual(
        Box((0.044, 0.008, 0.020)),
        origin=Origin(xyz=(-0.150, -0.050, -0.013)),
        material=steel,
        name="bracket_tongue",
    )
    card.visual(
        Box((0.180, 0.012, 0.0025)),
        origin=Origin(xyz=(0.036, -0.061, -0.020)),
        material=gold,
        name="pcie_contacts",
    )
    for i, x in enumerate((-0.154, -0.106, -0.053, 0.0, 0.053, 0.106, 0.154)):
        card.visual(
            Cylinder(radius=0.0032, length=0.004),
            origin=Origin(xyz=(x, -0.052, 0.019)),
            material=steel,
            name=f"screw_boss_{i}",
        )

    for i, (x, y) in enumerate(FAN_CENTERS):
        card.visual(
            Cylinder(radius=0.0050, length=0.006),
            origin=Origin(xyz=(x, y, 0.026)),
            material=steel,
            name=f"fan_motor_{i}",
        )
        # Four thin fixed spokes connect each motor boss to its circular shroud frame.
        for j, (dx, dy, sx, sy) in enumerate(
            (
                (0.0248, 0.0000, 0.0400, 0.0040),
                (-0.0248, 0.0000, 0.0400, 0.0040),
                (0.0000, 0.0248, 0.0040, 0.0400),
                (0.0000, -0.0248, 0.0040, 0.0400),
            )
        ):
            card.visual(
                Box((sx, sy, 0.003)),
                origin=Origin(xyz=(x + dx, y + dy, 0.022)),
                material=black,
                name=f"fan_spoke_{i}_{j}",
            )

    hx, hy, hz = HINGE_XYZ
    card.visual(
        Box((0.004, 0.012, 0.036)),
        origin=Origin(xyz=(hx - 0.012, hy + 0.006, hz)),
        material=black,
        name="hinge_side_bridge",
    )
    for i, z_offset in enumerate((-0.014, 0.014)):
        card.visual(
            Box((0.028, 0.018, 0.008)),
            origin=Origin(xyz=(hx + 0.001, hy + 0.006, hz + z_offset)),
            material=black,
            name=f"hinge_lug_{i}",
        )
        card.visual(
            Cylinder(radius=0.0065, length=0.009),
            origin=Origin(xyz=(hx, hy, hz + z_offset)),
            material=black,
            name=f"hinge_barrel_{i}",
        )


def _add_fan_rotor(rotor_part, mesh_name: str) -> None:
    rotor = FanRotorGeometry(
        0.037,
        0.011,
        11,
        thickness=0.010,
        blade_pitch_deg=32.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.18, tip_clearance=0.0015),
        hub=FanRotorHub(style="capped", bore_diameter=0.004),
    )
    rotor_part.visual(
        mesh_from_geometry(rotor, mesh_name),
        origin=Origin(),
        material=Material(name="rotor_black", rgba=(0.005, 0.005, 0.006, 1.0)),
        name="rotor",
    )
    rotor_part.visual(
        Cylinder(radius=0.0030, length=0.020),
        origin=Origin(),
        material=Material(name="rotor_axle_steel", rgba=(0.46, 0.47, 0.48, 1.0)),
        name="rotor_shaft",
    )


def _add_prop_leg(prop_leg, materials) -> None:
    graphite = materials["graphite"]
    steel = materials["steel"]
    rubber = materials["rubber"]

    prop_leg.visual(
        Cylinder(radius=0.0058, length=0.015),
        origin=Origin(),
        material=graphite,
        name="hinge_knuckle",
    )
    prop_leg.visual(
        Cylinder(radius=0.0022, length=0.044),
        origin=Origin(),
        material=steel,
        name="hinge_pin",
    )
    prop_leg.visual(
        Box((0.122, 0.010, 0.008)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=graphite,
        name="leg_beam",
    )
    prop_leg.visual(
        Box((0.020, 0.020, 0.007)),
        origin=Origin(xyz=(0.135, 0.0, -0.001)),
        material=rubber,
        name="foot_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_gpu_with_prop_leg")

    materials = {
        "black": model.material("satin_black", rgba=(0.010, 0.011, 0.013, 1.0)),
        "dark": model.material("dark_gunmetal", rgba=(0.055, 0.058, 0.064, 1.0)),
        "graphite": model.material("graphite_plastic", rgba=(0.025, 0.027, 0.030, 1.0)),
        "rubber": model.material("matte_rubber", rgba=(0.002, 0.002, 0.002, 1.0)),
        "pcb_green": model.material("dark_pcb_green", rgba=(0.015, 0.105, 0.065, 1.0)),
        "aluminum": model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0)),
        "copper": model.material("copper_heatpipe", rgba=(0.86, 0.38, 0.13, 1.0)),
        "gold": model.material("gold_contacts", rgba=(1.0, 0.72, 0.18, 1.0)),
        "steel": model.material("stamped_steel", rgba=(0.62, 0.64, 0.66, 1.0)),
    }

    card = model.part("card_body")
    _add_root_body(card, materials)

    for i, (x, y) in enumerate(FAN_CENTERS):
        fan = model.part(f"fan_{i}")
        _add_fan_rotor(fan, f"gpu_fan_rotor_{i}")
        model.articulation(
            f"fan_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=card,
            child=fan,
            origin=Origin(xyz=(x, y, 0.030)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=120.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.0),
        )

    prop_leg = model.part("prop_leg")
    _add_prop_leg(prop_leg, materials)
    model.articulation(
        "prop_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=card,
        child=prop_leg,
        origin=Origin(xyz=HINGE_XYZ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card_body")
    prop_leg = object_model.get_part("prop_leg")
    prop_hinge = object_model.get_articulation("prop_leg_hinge")

    for i in range(3):
        fan = object_model.get_part(f"fan_{i}")
        spin = object_model.get_articulation(f"fan_{i}_spin")
        ctx.check(
            f"fan {i} is continuous",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"joint type is {spin.articulation_type}",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="rotor",
            outer_elem="shroud_shell",
            margin=0.001,
            name=f"fan {i} sits inside shroud frame",
        )
        ctx.allow_overlap(
            card,
            fan,
            elem_a=f"fan_motor_{i}",
            elem_b="rotor_shaft",
            reason="The rotor shaft is intentionally captured inside the fixed fan motor boss.",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="rotor_shaft",
            outer_elem=f"fan_motor_{i}",
            margin=0.001,
            name=f"fan {i} shaft centered in motor boss",
        )
        ctx.expect_overlap(
            fan,
            card,
            axes="z",
            elem_a="rotor_shaft",
            elem_b=f"fan_motor_{i}",
            min_overlap=0.005,
            name=f"fan {i} shaft retained in motor boss",
        )

    # The prop-leg hinge pin is intentionally captured inside the two fixed hinge-barrel proxies.
    for i in range(2):
        ctx.allow_overlap(
            card,
            prop_leg,
            elem_a=f"hinge_barrel_{i}",
            elem_b="hinge_pin",
            reason="The steel hinge pin is intentionally represented as captured inside the fixed barrel proxy.",
        )
        ctx.expect_within(
            prop_leg,
            card,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=f"hinge_barrel_{i}",
            margin=0.001,
            name=f"hinge pin centered in barrel {i}",
        )
        ctx.expect_overlap(
            prop_leg,
            card,
            axes="z",
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{i}",
            min_overlap=0.007,
            name=f"hinge pin retained through barrel {i}",
        )

    with ctx.pose({prop_hinge: 0.0}):
        folded_aabb = ctx.part_element_world_aabb(prop_leg, elem="foot_pad")
    with ctx.pose({prop_hinge: prop_hinge.motion_limits.upper}):
        deployed_aabb = ctx.part_element_world_aabb(prop_leg, elem="foot_pad")

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    folded_y = _center_y(folded_aabb)
    deployed_y = _center_y(deployed_aabb)
    ctx.check(
        "prop leg folds downward from lower edge",
        folded_y is not None and deployed_y is not None and deployed_y < folded_y - 0.09,
        details=f"folded foot y={folded_y}, deployed foot y={deployed_y}",
    )

    return ctx.report()


object_model = build_object_model()

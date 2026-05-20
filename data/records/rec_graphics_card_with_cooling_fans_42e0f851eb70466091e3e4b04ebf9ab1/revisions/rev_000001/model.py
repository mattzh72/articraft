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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PCB_LENGTH = 0.285
PCB_HEIGHT = 0.110
PCB_THICKNESS = 0.0016

SHROUD_LENGTH = 0.235
SHROUD_HEIGHT = 0.094
SHROUD_THICKNESS = 0.012
SHROUD_CENTER_X = 0.010
SHROUD_CENTER_Y = 0.002
SHROUD_CENTER_Z = 0.028

FAN_CENTERS = ((-0.050, 0.003), (0.060, 0.003))
FAN_JOINT_Z = 0.041
FAN_ROTOR_RADIUS = 0.034
FAN_HOLE_RADIUS = 0.0395
FAN_FRAME_OUTER_RADIUS = 0.047


def _shroud_shell() -> cq.Workplane:
    """A single-piece cooler shroud with two circular fan apertures and raised rims."""
    local_fan_centers = [
        (x - SHROUD_CENTER_X, y - SHROUD_CENTER_Y) for x, y in FAN_CENTERS
    ]
    shell = cq.Workplane("XY").box(SHROUD_LENGTH, SHROUD_HEIGHT, SHROUD_THICKNESS)
    shell = (
        shell.faces(">Z")
        .workplane()
        .pushPoints(local_fan_centers)
        .circle(FAN_HOLE_RADIUS)
        .cutThruAll()
    )

    for fan_x, fan_y in local_fan_centers:
        raised_lip = (
            cq.Workplane("XY")
            .center(fan_x, fan_y)
            .circle(FAN_FRAME_OUTER_RADIUS)
            .circle(FAN_HOLE_RADIUS)
            .extrude(0.006)
            .translate((0.0, 0.0, SHROUD_THICKNESS / 2.0))
        )
        shell = shell.union(raised_lip)

    return shell


def _fan_rotor_mesh(name: str):
    rotor = FanRotorGeometry(
        FAN_ROTOR_RADIUS,
        0.012,
        11,
        thickness=0.008,
        blade_pitch_deg=33.0,
        blade_sweep_deg=25.0,
        blade=FanRotorBlade(
            shape="scimitar",
            tip_pitch_deg=14.0,
            camber=0.14,
            tip_clearance=0.0015,
        ),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.0025, rear_collar_radius=0.010),
    )
    return mesh_from_geometry(rotor, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_slot_graphics_card")

    pcb_mat = model.material("pcb_green", rgba=(0.03, 0.28, 0.12, 1.0))
    solder_mat = model.material("dark_solder_mask", rgba=(0.015, 0.08, 0.045, 1.0))
    shroud_mat = model.material("satin_black_plastic", rgba=(0.015, 0.015, 0.017, 1.0))
    fan_mat = model.material("black_fan_plastic", rgba=(0.005, 0.005, 0.006, 1.0))
    fin_mat = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.68, 0.68, 0.64, 1.0))
    gold_mat = model.material("gold_contacts", rgba=(1.0, 0.70, 0.16, 1.0))
    port_mat = model.material("port_void_black", rgba=(0.0, 0.0, 0.0, 1.0))

    card = model.part("card")

    # Long printed circuit board carrying all fixed cooler and connector hardware.
    card.visual(
        Box((PCB_LENGTH, PCB_HEIGHT, PCB_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PCB_THICKNESS / 2.0)),
        material=pcb_mat,
        name="pcb",
    )
    card.visual(
        Box((0.058, 0.052, 0.003)),
        origin=Origin(xyz=(-0.010, 0.000, 0.0031)),
        material=solder_mat,
        name="gpu_package",
    )

    # Aluminum heatsink: a base and many thin fins visible below the black shroud.
    card.visual(
        Box((0.218, 0.078, 0.008)),
        origin=Origin(xyz=(0.020, 0.003, 0.008)),
        material=fin_mat,
        name="heatsink_base",
    )
    for idx in range(21):
        x = -0.082 + idx * 0.010
        card.visual(
            Box((0.0024, 0.080, 0.024)),
            origin=Origin(xyz=(x, 0.003, 0.020)),
            material=fin_mat,
            name=f"fin_{idx}",
        )

    # The shroud mesh is one continuous cover with two actual circular openings.
    card.visual(
        mesh_from_cadquery(_shroud_shell(), "dual_fan_shroud", tolerance=0.0008),
        origin=Origin(xyz=(SHROUD_CENTER_X, SHROUD_CENTER_Y, SHROUD_CENTER_Z)),
        material=shroud_mat,
        name="shroud",
    )

    # Fan support frames: low cross-spokes and central bearing bosses sit behind
    # the rotating blades, connecting each rotor to its circular aperture frame.
    for fan_idx, (fan_x, fan_y) in enumerate(FAN_CENTERS):
        card.visual(
            Cylinder(radius=0.0115, length=0.006),
            origin=Origin(xyz=(fan_x, fan_y, 0.031), rpy=(0.0, 0.0, 0.0)),
            material=fan_mat,
            name=f"bearing_{fan_idx}",
        )
        card.visual(
            Cylinder(radius=0.0032, length=0.005),
            origin=Origin(xyz=(fan_x, fan_y, 0.03575), rpy=(0.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"shaft_{fan_idx}",
        )
        card.visual(
            Box((0.080, 0.0045, 0.0032)),
            origin=Origin(xyz=(fan_x, fan_y, 0.031)),
            material=fan_mat,
            name=f"fan_spoke_{fan_idx}_x",
        )
        card.visual(
            Box((0.0045, 0.080, 0.0032)),
            origin=Origin(xyz=(fan_x, fan_y, 0.031)),
            material=fan_mat,
            name=f"fan_spoke_{fan_idx}_y",
        )

    # Edge power connector and small shroud screws.
    card.visual(
        Box((0.032, 0.014, 0.012)),
        origin=Origin(xyz=(0.106, 0.054, 0.0076)),
        material=port_mat,
        name="power_socket",
    )
    for fan_idx, (fan_x, fan_y) in enumerate(FAN_CENTERS):
        for screw_idx, (dx, dy) in enumerate(((-0.035, -0.035), (0.035, -0.035), (-0.035, 0.035), (0.035, 0.035))):
            card.visual(
                Cylinder(radius=0.0032, length=0.002),
                origin=Origin(xyz=(fan_x + dx, fan_y + dy, 0.0405)),
                material=steel_mat,
                name=f"screw_{fan_idx}_{screw_idx}",
            )

    # PCIe edge contacts along the lower board edge.
    for idx in range(12):
        x = -0.075 + idx * 0.006
        card.visual(
            Box((0.0042, 0.014, 0.0007)),
            origin=Origin(xyz=(x, -0.051, 0.00185)),
            material=gold_mat,
            name=f"pcie_finger_{idx}",
        )

    # Dual-slot I/O bracket at the short end, with dark inset ports and vents.
    card.visual(
        Box((0.006, 0.126, 0.044)),
        origin=Origin(xyz=(-0.145, 0.000, 0.023)),
        material=steel_mat,
        name="io_bracket",
    )
    card.visual(
        Box((0.014, 0.017, 0.005)),
        origin=Origin(xyz=(-0.138, -0.038, 0.005)),
        material=steel_mat,
        name="bracket_foot",
    )
    for port_idx, y in enumerate((-0.032, 0.000, 0.032)):
        card.visual(
            Box((0.0012, 0.023, 0.008)),
            origin=Origin(xyz=(-0.1479, y, 0.024)),
            material=port_mat,
            name=f"display_port_{port_idx}",
        )
    for vent_idx in range(5):
        card.visual(
            Box((0.0012, 0.004, 0.021)),
            origin=Origin(xyz=(-0.1479, -0.052 + vent_idx * 0.006, 0.025)),
            material=port_mat,
            name=f"vent_slot_{vent_idx}",
        )

    # Two independent spinning fan rotors, each articulated about the card-face normal.
    for fan_idx, (fan_x, fan_y) in enumerate(FAN_CENTERS):
        fan = model.part(f"fan_{fan_idx}")
        fan.visual(
            _fan_rotor_mesh(f"fan_rotor_{fan_idx}"),
            origin=Origin(),
            material=fan_mat,
            name="rotor",
        )
        model.articulation(
            f"fan_{fan_idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=card,
            child=fan,
            origin=Origin(xyz=(fan_x, fan_y, FAN_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.15, velocity=90.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card")

    for fan_idx in range(2):
        fan = object_model.get_part(f"fan_{fan_idx}")
        spin = object_model.get_articulation(f"fan_{fan_idx}_spin")
        ctx.allow_overlap(
            card,
            fan,
            elem_a=f"shaft_{fan_idx}",
            elem_b="rotor",
            reason="The stationary fan shaft is intentionally captured inside the rotor hub bearing.",
        )
        ctx.check(
            f"fan {fan_idx} uses continuous axial spin",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="rotor",
            outer_elem="shroud",
            margin=0.004,
            name=f"fan {fan_idx} is contained by its shroud footprint",
        )
        ctx.expect_gap(
            fan,
            card,
            axis="z",
            positive_elem="rotor",
            negative_elem=f"shaft_{fan_idx}",
            max_penetration=0.0015,
            max_gap=0.004,
            name=f"fan {fan_idx} rotor sits just proud of bearing shaft",
        )

        rest_position = ctx.part_world_position(fan)
        with ctx.pose({spin: math.pi / 2.0}):
            rotated_position = ctx.part_world_position(fan)
        ctx.check(
            f"fan {fan_idx} spins without translating",
            rest_position is not None
            and rotated_position is not None
            and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, rotated_position)),
            details=f"rest={rest_position}, rotated={rotated_position}",
        )

    pcb_aabb = ctx.part_element_world_aabb(card, elem="pcb")
    bracket_aabb = ctx.part_element_world_aabb(card, elem="io_bracket")
    ctx.check(
        "i/o bracket is mounted at short card end",
        pcb_aabb is not None
        and bracket_aabb is not None
        and bracket_aabb[0][0] < pcb_aabb[0][0] + 0.002
        and bracket_aabb[1][2] > 0.038,
        details=f"pcb={pcb_aabb}, bracket={bracket_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


CARD_LENGTH = 0.330
CARD_WIDTH = 0.125
PCB_THICKNESS = 0.003
FAN_CENTERS = (-0.096, 0.0, 0.096)
FAN_Z = 0.036
FAN_RADIUS = 0.0405
FAN_OPENING_RADIUS = 0.0475
SHROUD_Z = 0.0285
SHROUD_THICKNESS = 0.009


def _make_shroud() -> cq.Workplane:
    """Single angled top shroud with three real circular fan cutouts."""
    outline = [
        (-0.159, -0.044),
        (-0.145, -0.061),
        (0.135, -0.061),
        (0.158, -0.040),
        (0.158, 0.039),
        (0.135, 0.061),
        (-0.145, 0.061),
        (-0.159, 0.044),
    ]
    shroud = cq.Workplane("XY").polyline(outline).close().extrude(SHROUD_THICKNESS)

    for x in FAN_CENTERS:
        cutter = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(FAN_OPENING_RADIUS)
            .extrude(SHROUD_THICKNESS + 0.006)
            .translate((0.0, 0.0, -0.003))
        )
        shroud = shroud.cut(cutter)

    return shroud.translate((0.0, 0.0, SHROUD_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    pcb_green = model.material("pcb_green", rgba=(0.03, 0.22, 0.09, 1.0))
    solder_dark = model.material("solder_dark", rgba=(0.02, 0.025, 0.025, 1.0))
    shroud_black = model.material("angled_black_shroud", rgba=(0.015, 0.016, 0.018, 1.0))
    shroud_graphite = model.material("graphite_bevels", rgba=(0.10, 0.11, 0.12, 1.0))
    fin_aluminum = model.material("brushed_aluminum", rgba=(0.65, 0.68, 0.66, 1.0))
    copper = model.material("copper_heatpipe", rgba=(0.72, 0.34, 0.12, 1.0))
    steel = model.material("stamped_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    fan_black = model.material("fan_matte_black", rgba=(0.005, 0.005, 0.006, 1.0))

    card = model.part("card")

    # Flat PCB, rigid rear backplate, and the broad heatsink mass.
    card.visual(
        Box((CARD_LENGTH, CARD_WIDTH, PCB_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PCB_THICKNESS / 2.0)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.307, 0.119, 0.004)),
        origin=Origin(xyz=(0.004, 0.0, -0.0015)),
        material=solder_dark,
        name="backplate",
    )
    card.visual(
        Box((0.304, 0.107, 0.014)),
        origin=Origin(xyz=(0.006, 0.0, 0.010)),
        material=solder_dark,
        name="heatsink_core",
    )

    # Fin stack visible through the fan openings and around the shroud edge.
    fin_count = 25
    fin_span = 0.286
    for i in range(fin_count):
        x = -fin_span / 2.0 + i * (fin_span / (fin_count - 1))
        card.visual(
            Box((0.0042, 0.104, 0.0135)),
            origin=Origin(xyz=(x, 0.0, 0.0234)),
            material=fin_aluminum,
            name=f"fin_{i}",
        )

    # Copper heat pipes crossing the fin pack, just below the fan plane.
    for i, y in enumerate((-0.035, -0.012, 0.012, 0.035)):
        card.visual(
            Cylinder(radius=0.0032, length=0.285),
            origin=Origin(xyz=(0.004, y, 0.0282), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=copper,
            name=f"heatpipe_{i}",
        )

    # Single mesh shroud: faceted outline, sloped-looking corners, and true round openings.
    card.visual(
        mesh_from_cadquery(_make_shroud(), "angled_outer_shroud"),
        material=shroud_black,
        name="shroud",
    )

    # Raised angular ribs/accents on the shroud make the outer cover read faceted.
    for i, (x, y, yaw, sx) in enumerate(
        [
            (-0.138, -0.047, -0.55, 0.056),
            (-0.138, 0.047, 0.55, 0.056),
            (0.142, -0.047, 0.55, 0.054),
            (0.142, 0.047, -0.55, 0.054),
            (-0.048, -0.053, 0.10, 0.060),
            (0.048, 0.053, 0.10, 0.060),
        ]
    ):
        card.visual(
            Box((sx, 0.006, 0.003)),
            origin=Origin(xyz=(x, y, SHROUD_Z + SHROUD_THICKNESS + 0.0012), rpy=(0.0, 0.0, yaw)),
            material=shroud_graphite,
            name=f"shroud_rib_{i}",
        )

    # Stationary bearing posts below each rotor give the fans a visible support path.
    for i, x in enumerate(FAN_CENTERS):
        card.visual(
            Cylinder(radius=0.012, length=0.005),
            origin=Origin(xyz=(x, 0.0, FAN_Z - 0.0075)),
            material=solder_dark,
            name=f"fan_{i}_motor_post",
        )
        for j, yaw in enumerate((0.0, math.pi / 2.0)):
            card.visual(
                Box((0.073, 0.004, 0.0025)),
                origin=Origin(xyz=(x, 0.0, FAN_Z - 0.0097), rpy=(0.0, 0.0, yaw)),
                material=solder_dark,
                name=f"fan_{i}_stator_{j}",
            )

    # Stamped I/O bracket at the short end, with dark display-port cutouts.
    card.visual(
        Box((0.010, 0.132, 0.108)),
        origin=Origin(xyz=(-0.166, 0.0, 0.041)),
        material=steel,
        name="io_bracket",
    )
    port_specs = [
        (-0.041, 0.025, 0.020, 0.009),
        (-0.014, 0.025, 0.020, 0.009),
        (0.014, 0.025, 0.020, 0.009),
        (0.042, 0.025, 0.018, 0.011),
        (-0.028, 0.051, 0.025, 0.011),
        (0.028, 0.051, 0.025, 0.011),
    ]
    for i, (y, z, sy, sz) in enumerate(port_specs):
        card.visual(
            Box((0.0012, sy, sz)),
            origin=Origin(xyz=(-0.1716, y, z)),
            material=fan_black,
            name=f"port_{i}",
        )

    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=FAN_RADIUS,
            hub_radius=0.014,
            blade_count=11,
            thickness=0.010,
            blade_pitch_deg=32.0,
            blade_sweep_deg=31.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.0015),
            hub=FanRotorHub(style="capped"),
        ),
        "triple_gpu_fan_rotor",
    )

    for i, x in enumerate(FAN_CENTERS):
        fan = model.part(f"fan_{i}")
        fan.visual(
            rotor_mesh,
            material=fan_black,
            name="rotor",
        )
        model.articulation(
            f"fan_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=card,
            child=fan,
            origin=Origin(xyz=(x, 0.0, FAN_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.20, velocity=180.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    card = object_model.get_part("card")
    fan_parts = [object_model.get_part(f"fan_{i}") for i in range(3)]
    fan_joints = [object_model.get_articulation(f"fan_{i}_spin") for i in range(3)]

    positions = [ctx.part_world_position(fan) for fan in fan_parts]
    ctx.check(
        "three fan rotors are evenly spaced along card length",
        all(pos is not None for pos in positions)
        and abs((positions[1][0] - positions[0][0]) - (positions[2][0] - positions[1][0])) < 1e-6
        and abs(positions[1][1]) < 1e-6
        and abs(positions[1][2] - FAN_Z) < 1e-6,
        details=f"fan origin positions={positions}",
    )

    for i, (fan, joint) in enumerate(zip(fan_parts, fan_joints)):
        ctx.check(
            f"fan_{i} has a continuous axial spin joint",
            getattr(joint, "articulation_type", None) == ArticulationType.CONTINUOUS
            and tuple(getattr(joint, "axis", ())) == (0.0, 0.0, 1.0),
            details=f"type={getattr(joint, 'articulation_type', None)}, axis={getattr(joint, 'axis', None)}",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="rotor",
            outer_elem="shroud",
            margin=0.002,
            name=f"fan_{i} rotor sits inside the shroud footprint",
        )
        ctx.expect_overlap(
            fan,
            card,
            axes="xy",
            elem_a="rotor",
            elem_b=f"fan_{i}_motor_post",
            min_overlap=0.010,
            name=f"fan_{i} hub overlaps its motor post in plan",
        )
        ctx.expect_gap(
            fan,
            card,
            axis="z",
            positive_elem="rotor",
            negative_elem=f"fan_{i}_motor_post",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"fan_{i} rotor is seated on its bearing post",
        )
        before = ctx.part_world_position(fan)
        with ctx.pose({joint: math.pi / 2.0}):
            after = ctx.part_world_position(fan)
            ctx.check(
                f"fan_{i} spins about its own fixed center",
                before is not None
                and after is not None
                and all(abs(before[k] - after[k]) < 1e-7 for k in range(3)),
                details=f"before={before}, after={after}",
            )

    return ctx.report()


object_model = build_object_model()

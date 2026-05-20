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
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CARD_LENGTH = 0.340
CARD_WIDTH = 0.132
SHROUD_Z0 = 0.000
SHROUD_THICKNESS = 0.008
FAN_Z = 0.020

FANS = (
    {
        "part": "fan_0",
        "joint": "fan_0_spin",
        "frame": "fan_frame_0",
        "center": (-0.104, 0.0),
        "rotor_radius": 0.049,
        "hub_radius": 0.017,
        "opening_radius": 0.055,
        "frame_outer": 0.064,
        "bore": 0.009,
        "axle_radius": 0.0050,
        "blade_count": 11,
    },
    {
        "part": "fan_1",
        "joint": "fan_1_spin",
        "frame": "fan_frame_1",
        "center": (0.020, 0.0),
        "rotor_radius": 0.049,
        "hub_radius": 0.017,
        "opening_radius": 0.055,
        "frame_outer": 0.064,
        "bore": 0.009,
        "axle_radius": 0.0050,
        "blade_count": 11,
    },
    {
        "part": "tail_fan",
        "joint": "tail_fan_spin",
        "frame": "tail_frame",
        "center": (0.126, 0.0),
        "rotor_radius": 0.034,
        "hub_radius": 0.012,
        "opening_radius": 0.039,
        "frame_outer": 0.047,
        "bore": 0.0075,
        "axle_radius": 0.0040,
        "blade_count": 9,
    },
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cut_cylinder(
    body: cq.Workplane,
    center: tuple[float, float],
    radius: float,
    z0: float = -0.006,
    height: float = 0.040,
) -> cq.Workplane:
    cutter = cq.Workplane("XY").center(center[0], center[1]).circle(radius).extrude(height).translate((0.0, 0.0, z0))
    return body.cut(cutter)


def _build_shroud_face() -> cq.Workplane:
    """Angular front shroud plate with real through openings for all three fans."""

    outline = [
        (-0.170, -0.044),
        (-0.153, -0.064),
        (0.111, -0.064),
        (0.165, -0.045),
        (0.169, 0.035),
        (0.146, 0.060),
        (-0.123, 0.066),
        (-0.170, 0.047),
    ]
    shroud = cq.Workplane("XY").polyline(outline).close().extrude(SHROUD_THICKNESS)

    # The two large fan apertures are round through-holes in the fixed shroud.
    for spec in FANS[:2]:
        shroud = _cut_cylinder(shroud, spec["center"], spec["opening_radius"])

    # A broad flow-through tail opening removes the back end of the plate; the
    # smaller tail fan frame bridges this real cut-through.
    tail_x, tail_y = FANS[2]["center"]
    tail_cut = (
        cq.Workplane("XY")
        .center(tail_x, tail_y)
        .rect(0.088, 0.108)
        .extrude(0.040)
        .translate((0.0, 0.0, -0.006))
    )
    shroud = shroud.cut(tail_cut)

    # Raised angular styling ribs make the cover read like a molded GPU shroud.
    accents = [
        _cq_box((0.115, 0.008, 0.004), (-0.101, 0.055, SHROUD_THICKNESS + 0.002)),
        _cq_box((0.092, 0.008, 0.004), (-0.006, -0.057, SHROUD_THICKNESS + 0.002)),
        _cq_box((0.068, 0.007, 0.004), (0.090, 0.052, SHROUD_THICKNESS + 0.002)),
        _cq_box((0.007, 0.030, 0.004), (0.070, -0.047, SHROUD_THICKNESS + 0.002)),
    ]
    for accent in accents:
        shroud = shroud.union(accent)
    return shroud


def _build_fan_frame(spec: dict[str, object]) -> cq.Workplane:
    """Stationary annular frame, spider supports, and bearing axle for one rotor."""

    outer_r = float(spec["frame_outer"])
    inner_r = float(spec["opening_radius"])
    axle_r = float(spec["axle_radius"])
    arm_w = 0.0045 if outer_r > 0.055 else 0.0037
    frame_depth = 0.023

    frame = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(frame_depth)

    # Low rear spider arms connect the central bearing to the fixed ring without
    # sweeping through the rotating blade plane.
    arm_z = 0.0035
    arm_t = 0.0040
    arm_len = inner_r * 2.0 + 0.010
    arms = [
        _cq_box((arm_len, arm_w, arm_t), (0.0, 0.0, arm_z + arm_t / 2.0)),
        _cq_box((arm_w, arm_len, arm_t), (0.0, 0.0, arm_z + arm_t / 2.0)),
    ]
    diagonal = _cq_box((arm_len * 0.72, arm_w * 0.8, arm_t), (0.0, 0.0, arm_z + arm_t / 2.0)).rotate(
        (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 38.0
    )

    # The axle passes through the rotor's modeled bore; it visually clips the
    # rotor into the circular frame while leaving clearance for continuous spin.
    axle = cq.Workplane("XY").circle(axle_r).extrude(0.030).translate((0.0, 0.0, 0.000))

    for arm in arms:
        frame = frame.union(arm)
    frame = frame.union(diagonal).union(axle)
    return frame


def _build_pcb() -> cq.Workplane:
    board = cq.Workplane("XY").box(0.326, 0.116, 0.003)
    tail_x, tail_y = FANS[2]["center"]
    tail_cut = cq.Workplane("XY").center(tail_x, tail_y).rect(0.086, 0.096).extrude(0.020).translate((0.0, 0.0, -0.010))
    return board.cut(tail_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = model.material("dark_graphite", rgba=(0.050, 0.055, 0.062, 1.0))
    fin_metal = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.030, 0.27, 0.13, 1.0))
    copper = model.material("edge_gold", rgba=(0.95, 0.67, 0.18, 1.0))
    bracket_metal = model.material("io_bracket_steel", rgba=(0.62, 0.64, 0.66, 1.0))

    card = model.part("card")

    # Base PCB and metal stack sit behind the fan shroud.  The PCB shares the
    # tail flow-through cutout so the small fan really spans an open back end.
    card.visual(
        mesh_from_cadquery(_build_pcb(), "pcb_with_tail_cutout", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.300, 0.106, 0.013)),
        origin=Origin(xyz=(-0.010, 0.0, -0.018)),
        material=fin_metal,
        name="heatsink_base",
    )

    # A row of continuous aluminum fins is visible through the side gaps and
    # the tail flow-through opening.
    for idx, x in enumerate([0.118 - i * 0.018 for i in range(15)]):
        card.visual(
            Box((0.0045, 0.105, 0.027)),
            origin=Origin(xyz=(x, 0.0, -0.0065)),
            material=fin_metal,
            name=f"cooling_fin_{idx}",
        )

    # Molded angular shroud face and its side wraps around the PCB/heatsink.
    card.visual(
        mesh_from_cadquery(_build_shroud_face(), "angular_shroud", tolerance=0.0008),
        origin=Origin(),
        material=black,
        name="shroud",
    )
    card.visual(
        Box((0.312, 0.008, 0.034)),
        origin=Origin(xyz=(-0.006, 0.067, -0.006)),
        material=black,
        name="upper_side_wrap",
    )
    card.visual(
        Box((0.292, 0.008, 0.034)),
        origin=Origin(xyz=(-0.020, -0.067, -0.006)),
        material=black,
        name="lower_side_wrap",
    )
    card.visual(
        Box((0.010, 0.086, 0.032)),
        origin=Origin(xyz=(-0.170, 0.003, -0.006)),
        material=black,
        name="front_end_wrap",
    )
    card.visual(
        Box((0.006, 0.138, 0.056)),
        origin=Origin(xyz=(-0.178, 0.0, -0.012)),
        material=bracket_metal,
        name="io_bracket",
    )

    # PCIe edge fingers and auxiliary power socket make the card read as a GPU.
    card.visual(
        Box((0.130, 0.010, 0.0025)),
        origin=Origin(xyz=(-0.060, -0.063, -0.027)),
        material=copper,
        name="pcie_fingers",
    )
    for idx, x in enumerate([0.060 + i * 0.006 for i in range(6)]):
        card.visual(
            Box((0.0032, 0.016, 0.004)),
            origin=Origin(xyz=(x, 0.069, -0.008)),
            material=dark,
            name=f"power_pin_{idx}",
        )
    card.visual(
        Box((0.052, 0.020, 0.018)),
        origin=Origin(xyz=(0.078, 0.068, -0.004)),
        material=dark,
        name="power_socket",
    )

    # Fixed circular frames and bearing axles.  The rotors sit inside these
    # frames, with their bores around the fixed axles.
    fan_0_x, fan_0_y = FANS[0]["center"]
    card.visual(
        mesh_from_cadquery(_build_fan_frame(FANS[0]), "fan_frame_0", tolerance=0.0006, angular_tolerance=0.06),
        origin=Origin(xyz=(float(fan_0_x), float(fan_0_y), SHROUD_Z0 + 0.001)),
        material=black,
        name="fan_frame_0",
    )
    fan_1_x, fan_1_y = FANS[1]["center"]
    card.visual(
        mesh_from_cadquery(_build_fan_frame(FANS[1]), "fan_frame_1", tolerance=0.0006, angular_tolerance=0.06),
        origin=Origin(xyz=(float(fan_1_x), float(fan_1_y), SHROUD_Z0 + 0.001)),
        material=black,
        name="fan_frame_1",
    )
    tail_x, tail_y = FANS[2]["center"]
    card.visual(
        mesh_from_cadquery(_build_fan_frame(FANS[2]), "tail_frame", tolerance=0.0006, angular_tolerance=0.06),
        origin=Origin(xyz=(float(tail_x), float(tail_y), SHROUD_Z0 + 0.001)),
        material=black,
        name="tail_frame",
    )

    # Three continuously spinning rotors: two large front fans and one smaller
    # tail fan over the flow-through opening.
    for spec in FANS:
        rotor = model.part(str(spec["part"]))
        rotor_geom = FanRotorGeometry(
            float(spec["rotor_radius"]),
            float(spec["hub_radius"]),
            int(spec["blade_count"]),
            thickness=0.010 if spec["part"] != "tail_fan" else 0.008,
            blade_pitch_deg=32.0,
            blade_sweep_deg=26.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.18, tip_clearance=0.0015),
            hub=FanRotorHub(style="spinner", bore_diameter=float(spec["bore"])),
        )
        rotor.visual(
            mesh_from_geometry(rotor_geom, f"{spec['part']}_rotor"),
            origin=Origin(),
            material=dark,
            name="rotor",
        )
        cx, cy = spec["center"]
        model.articulation(
            str(spec["joint"]),
            ArticulationType.CONTINUOUS,
            parent=card,
            child=rotor,
            origin=Origin(xyz=(float(cx), float(cy), FAN_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=90.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card")

    for spec in FANS:
        fan = object_model.get_part(str(spec["part"]))
        joint = object_model.get_articulation(str(spec["joint"]))
        frame_name = str(spec["frame"])

        ctx.allow_overlap(
            card,
            fan,
            elem_a=frame_name,
            elem_b="rotor",
            reason="The fixed bearing axle is intentionally captured through the rotor hub so the fan stays clipped into the shroud frame while spinning.",
        )
        ctx.check(
            f"{spec['part']} has continuous spin joint",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} type is {joint.articulation_type}",
        )
        ctx.expect_overlap(
            fan,
            card,
            axes="xy",
            elem_a="rotor",
            elem_b=frame_name,
            min_overlap=0.006,
            name=f"{spec['part']} hub overlaps retained axle in plan",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="rotor",
            outer_elem=frame_name,
            margin=0.0005,
            name=f"{spec['part']} rotor is clipped inside its circular frame",
        )
        ctx.expect_overlap(
            fan,
            card,
            axes="z",
            elem_a="rotor",
            elem_b=frame_name,
            min_overlap=0.004,
            name=f"{spec['part']} rotor stays axially seated in frame depth",
        )

        rest_position = ctx.part_world_position(fan)
        with ctx.pose({joint: math.pi / 2.0}):
            spun_position = ctx.part_world_position(fan)
            ctx.expect_within(
                fan,
                card,
                axes="xy",
                inner_elem="rotor",
                outer_elem=frame_name,
                margin=0.0005,
                name=f"{spec['part']} remains in frame while spinning",
            )
        ctx.check(
            f"{spec['part']} spin does not translate rotor",
            rest_position is not None
            and spun_position is not None
            and all(abs(rest_position[i] - spun_position[i]) < 1e-6 for i in range(3)),
            details=f"rest={rest_position}, spun={spun_position}",
        )

    ctx.expect_overlap(
        "tail_fan",
        "card",
        axes="xy",
        elem_a="rotor",
        elem_b="tail_frame",
        min_overlap=0.060,
        name="tail fan spans the flow-through back opening",
    )

    return ctx.report()


object_model = build_object_model()

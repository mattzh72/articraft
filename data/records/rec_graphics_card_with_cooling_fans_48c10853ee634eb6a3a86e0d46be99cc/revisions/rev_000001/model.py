from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 72,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_slot_graphics_card")

    pcb_green = model.material("solder_mask_green", rgba=(0.02, 0.30, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    fan_black = model.material("fan_black", rgba=(0.008, 0.009, 0.011, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.64, 0.66, 0.65, 1.0))
    bracket_metal = model.material("nickel_bracket", rgba=(0.72, 0.73, 0.70, 1.0))
    gold = model.material("edge_contact_gold", rgba=(1.0, 0.68, 0.18, 1.0))
    port_dark = model.material("port_shadow_black", rgba=(0.0, 0.0, 0.0, 1.0))

    card = model.part("card")

    # Long, thin desktop GPU circuit board.
    card.visual(
        Box((0.270, 0.112, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pcb_green,
        name="pcb",
    )
    card.visual(
        Box((0.152, 0.010, 0.0012)),
        origin=Origin(xyz=(0.024, -0.059, 0.0016)),
        material=gold,
        name="pcie_fingers",
    )
    card.visual(
        Box((0.030, 0.014, 0.010)),
        origin=Origin(xyz=(0.113, 0.044, 0.006)),
        material=dark_plastic,
        name="power_socket",
    )

    # Dual-slot rear bracket at the I/O end, including dark port openings.
    card.visual(
        Box((0.004, 0.132, 0.052)),
        origin=Origin(xyz=(-0.137, 0.0, 0.024)),
        material=bracket_metal,
        name="bracket_plate",
    )
    card.visual(
        Box((0.012, 0.006, 0.052)),
        origin=Origin(xyz=(-0.132, 0.0, 0.024)),
        material=bracket_metal,
        name="bracket_flange",
    )
    card.visual(
        Box((0.004, 0.020, 0.010)),
        origin=Origin(xyz=(-0.137, 0.074, 0.044)),
        material=bracket_metal,
        name="screw_ear",
    )
    for idx, (y, z, sy, sz) in enumerate(
        (
            (-0.039, 0.018, 0.018, 0.008),
            (-0.014, 0.018, 0.018, 0.008),
            (0.014, 0.018, 0.018, 0.008),
            (0.041, 0.026, 0.020, 0.018),
        )
    ):
        card.visual(
            Box((0.0008, sy, sz)),
            origin=Origin(xyz=(-0.1394, y, z)),
            material=port_dark,
            name=f"display_port_{idx}",
        )

    # Heatsink stack under a single shroud.
    card.visual(
        Box((0.222, 0.094, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, 0.005)),
        material=aluminum,
        name="heatsink_base",
    )
    for idx, y in enumerate((-0.039, -0.030, -0.021, -0.012, -0.003, 0.006, 0.015, 0.024, 0.033, 0.042)):
        card.visual(
            Box((0.212, 0.0023, 0.020)),
            origin=Origin(xyz=(0.020, y, 0.019)),
            material=aluminum,
            name=f"cooling_fin_{idx}",
        )

    fan_centers = (-0.042, 0.082)
    shroud_outline = rounded_rect_profile(0.242, 0.104, 0.012, corner_segments=8)
    fan_holes = [_circle_profile(0.039, center=(x - 0.020, 0.0), segments=80) for x in fan_centers]
    shroud_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(shroud_outline, fan_holes, 0.008, center=True),
        "gpu_shroud_face",
    )
    card.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.020, 0.0, 0.032)),
        material=matte_black,
        name="shroud_face",
    )
    for name, y in (("shroud_rail_0", -0.051), ("shroud_rail_1", 0.051)):
        card.visual(
            Box((0.232, 0.004, 0.024)),
            origin=Origin(xyz=(0.020, y, 0.021)),
            material=matte_black,
            name=name,
        )
    for name, x in (("shroud_end_0", -0.101), ("shroud_end_1", 0.141)):
        card.visual(
            Box((0.004, 0.100, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.021)),
            material=matte_black,
            name=name,
        )

    fan_frame_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.079, 0.079),
            (0.094, 0.094),
            0.008,
            opening_shape="circle",
            outer_shape="circle",
            center=True,
        ),
        "gpu_fan_frame",
    )
    for idx, x in enumerate(fan_centers):
        card.visual(
            fan_frame_mesh,
            origin=Origin(xyz=(x, 0.0, 0.038)),
            material=matte_black,
            name=f"fan_frame_{idx}",
        )
        # Rear stator struts and motor boss physically tie each circular frame
        # to the shroud while staying behind the rotating blades.
        card.visual(
            Box((0.081, 0.004, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=dark_plastic,
            name=f"fan_spoke_x_{idx}",
        )
        card.visual(
            Box((0.004, 0.081, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=dark_plastic,
            name=f"fan_spoke_y_{idx}",
        )
        card.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=dark_plastic,
            name=f"fan_bearing_{idx}",
        )
        card.visual(
            Cylinder(radius=0.004, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material=dark_plastic,
            name=f"fan_axle_{idx}",
        )

    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.036,
            0.013,
            9,
            thickness=0.009,
            blade_pitch_deg=31.0,
            blade_sweep_deg=28.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
            hub=FanRotorHub(style="capped", rear_collar_height=0.002, rear_collar_radius=0.010),
        ),
        "gpu_axial_rotor",
    )
    for idx, x in enumerate(fan_centers):
        fan = model.part(f"fan_{idx}")
        fan.visual(
            rotor_mesh,
            origin=Origin(),
            material=fan_black,
            name="rotor",
        )
        model.articulation(
            f"fan_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=card,
            child=fan,
            origin=Origin(xyz=(x, 0.0, 0.044)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.05, velocity=150.0),
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
    fan_0 = object_model.get_part("fan_0")
    fan_1 = object_model.get_part("fan_1")
    spin_0 = object_model.get_articulation("fan_spin_0")
    spin_1 = object_model.get_articulation("fan_spin_1")

    ctx.check(
        "two continuous fan spin joints",
        spin_0.articulation_type == ArticulationType.CONTINUOUS
        and spin_1.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin_0.axis) == (0.0, 0.0, 1.0)
        and tuple(spin_1.axis) == (0.0, 0.0, 1.0),
        details=f"fan_spin_0={spin_0.articulation_type} axis={spin_0.axis}; "
        f"fan_spin_1={spin_1.articulation_type} axis={spin_1.axis}",
    )

    for idx, (fan, spin) in enumerate(((fan_0, spin_0), (fan_1, spin_1))):
        ctx.allow_overlap(
            card,
            fan,
            elem_a=f"fan_axle_{idx}",
            elem_b="rotor",
            reason="The fixed motor shaft is intentionally captured inside the rotating fan hub.",
        )
        ctx.expect_overlap(
            card,
            fan,
            axes="z",
            elem_a=f"fan_axle_{idx}",
            elem_b="rotor",
            min_overlap=0.004,
            name=f"fan {idx} axle remains captured in hub",
        )
        ctx.expect_within(
            card,
            fan,
            axes="xy",
            inner_elem=f"fan_axle_{idx}",
            outer_elem="rotor",
            margin=0.0,
            name=f"fan {idx} axle is centered inside rotor",
        )
        ctx.expect_within(
            fan,
            card,
            axes="xy",
            inner_elem="rotor",
            outer_elem=f"fan_frame_{idx}",
            margin=0.003,
            name=f"fan {idx} rotor sits inside its circular frame",
        )
        ctx.expect_gap(
            fan,
            card,
            axis="z",
            min_gap=0.002,
            max_gap=0.006,
            positive_elem="rotor",
            negative_elem=f"fan_bearing_{idx}",
            name=f"fan {idx} rotor clears rear bearing",
        )
        start = ctx.part_world_position(fan)
        with ctx.pose({spin: math.pi / 2.0}):
            spun = ctx.part_world_position(fan)
        ctx.check(
            f"fan {idx} spins about a fixed center",
            start is not None
            and spun is not None
            and all(abs(a - b) < 1.0e-6 for a, b in zip(start, spun)),
            details=f"start={start}, spun={spun}",
        )

    return ctx.report()


object_model = build_object_model()

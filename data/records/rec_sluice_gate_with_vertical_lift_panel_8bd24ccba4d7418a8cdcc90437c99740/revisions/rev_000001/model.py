from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_sluice_gate")

    steel = model.material("galvanized_steel", rgba=(0.42, 0.46, 0.48, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.12, 0.17, 0.20, 1.0))
    painted_gate = model.material("painted_gate_steel", rgba=(0.05, 0.18, 0.28, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.55, 0.54, 0.50, 1.0))
    water = model.material("channel_water", rgba=(0.05, 0.27, 0.40, 0.55))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bronze = model.material("bronze_wear_metal", rgba=(0.62, 0.43, 0.20, 1.0))

    frame = model.part("frame")
    # Concrete channel and sill below the steel guide frame.
    frame.visual(
        Box((2.80, 0.95, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="base_slab",
    )
    frame.visual(
        Box((2.28, 0.36, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=steel,
        name="bottom_sill",
    )
    frame.visual(
        Box((1.45, 0.36, 0.018)),
        origin=Origin(xyz=(0.0, 0.30, 0.225)),
        material=water,
        name="water_surface",
    )

    # Two heavy side guide columns tied by a split top beam.  The split leaves a
    # central vertical slot for actuator hardware while still reading as one
    # rigid yoke because the beams interlock with the side columns.
    frame.visual(
        Box((0.18, 0.36, 2.55)),
        origin=Origin(xyz=(-1.05, 0.0, 1.55)),
        material=steel,
        name="side_column_0",
    )
    frame.visual(
        Box((0.10, 0.055, 2.34)),
        origin=Origin(xyz=(-0.91, -0.095, 1.52)),
        material=dark_steel,
        name="front_guide_lip_0",
    )
    frame.visual(
        Box((0.10, 0.055, 2.34)),
        origin=Origin(xyz=(-0.91, 0.095, 1.52)),
        material=dark_steel,
        name="rear_guide_lip_0",
    )
    frame.visual(
        Box((0.18, 0.36, 2.55)),
        origin=Origin(xyz=(1.05, 0.0, 1.55)),
        material=steel,
        name="side_column_1",
    )
    frame.visual(
        Box((0.10, 0.055, 2.34)),
        origin=Origin(xyz=(0.91, -0.095, 1.52)),
        material=dark_steel,
        name="front_guide_lip_1",
    )
    frame.visual(
        Box((0.10, 0.055, 2.34)),
        origin=Origin(xyz=(0.91, 0.095, 1.52)),
        material=dark_steel,
        name="rear_guide_lip_1",
    )

    frame.visual(
        Box((2.50, 0.12, 0.24)),
        origin=Origin(xyz=(0.0, -0.18, 2.85)),
        material=steel,
        name="top_beam_front",
    )
    frame.visual(
        Box((2.50, 0.12, 0.24)),
        origin=Origin(xyz=(0.0, 0.18, 2.85)),
        material=steel,
        name="top_beam_rear",
    )
    frame.visual(
        Box((0.42, 0.42, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 3.05)),
        material=dark_steel,
        name="actuator_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.056, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 3.22)),
        material=steel,
        name="handwheel_axle",
    )

    gate = model.part("gate_panel")
    gate.visual(
        Box((1.68, 0.080, 1.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=painted_gate,
        name="gate_plate",
    )
    gate.visual(
        Box((1.56, 0.055, 0.085)),
        origin=Origin(xyz=(0.0, -0.067, 0.46)),
        material=dark_steel,
        name="lower_stiffener",
    )
    gate.visual(
        Box((1.56, 0.055, 0.085)),
        origin=Origin(xyz=(0.0, -0.067, 1.12)),
        material=dark_steel,
        name="upper_stiffener",
    )
    for x, suffix in [(-0.42, "0"), (0.42, "1")]:
        gate.visual(
            Box((0.080, 0.060, 1.36)),
            origin=Origin(xyz=(x, -0.070, 0.82)),
            material=dark_steel,
            name=f"vertical_stiffener_{suffix}",
        )
    gate.visual(
        Box((0.020, 0.160, 1.38)),
        origin=Origin(xyz=(-0.85, 0.0, 0.83)),
        material=bronze,
        name="guide_shoe_0",
    )
    gate.visual(
        Box((0.020, 0.160, 1.38)),
        origin=Origin(xyz=(0.85, 0.0, 0.83)),
        material=bronze,
        name="guide_shoe_1",
    )
    gate.visual(
        Box((1.58, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=rubber,
        name="bottom_seal",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.32, tube=0.024, radial_segments=18, tubular_segments=64), "handwheel_outer_ring"),
        material=steel,
        name="outer_ring",
    )
    handwheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.078, tube=0.022, radial_segments=14, tubular_segments=40), "handwheel_center_collar"),
        material=steel,
        name="center_collar",
    )
    spoke_extent = 0.250
    spoke_center = 0.195
    for sign, suffix in [(-1.0, "neg"), (1.0, "pos")]:
        handwheel.visual(
            Box((spoke_extent, 0.038, 0.032)),
            origin=Origin(xyz=(sign * spoke_center, 0.0, 0.0)),
            material=steel,
            name=f"x_spoke_{suffix}",
        )
        handwheel.visual(
            Box((0.038, spoke_extent, 0.032)),
            origin=Origin(xyz=(0.0, sign * spoke_center, 0.0)),
            material=steel,
            name=f"y_spoke_{suffix}",
        )
    handwheel.visual(
        Cylinder(radius=0.028, length=0.11),
        origin=Origin(xyz=(0.225, 0.225, 0.055)),
        material=rubber,
        name="spinner_grip",
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.18, lower=0.0, upper=0.75),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.0, 3.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate_panel")
    wheel = object_model.get_part("handwheel")
    gate_slide = object_model.get_articulation("frame_to_gate")
    wheel_spin = object_model.get_articulation("frame_to_handwheel")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="handwheel_axle",
        elem_b="center_collar",
        reason=(
            "The handwheel hub is intentionally captured on the fixed axle; "
            "the torus mesh is used as a compact collar proxy around the shaft."
        ),
    )

    ctx.check(
        "handwheel uses continuous rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_spin.articulation_type}",
    )

    with ctx.pose({gate_slide: 0.0}):
        ctx.expect_gap(
            gate,
            frame,
            axis="z",
            positive_elem="gate_plate",
            negative_elem="bottom_sill",
            min_gap=0.010,
            max_gap=0.035,
            name="closed gate seats just above sill",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="x",
            positive_elem="front_guide_lip_1",
            negative_elem="gate_plate",
            min_gap=0.010,
            max_gap=0.040,
            name="right guide captures gate with side clearance",
        )
        ctx.expect_gap(
            gate,
            frame,
            axis="x",
            positive_elem="gate_plate",
            negative_elem="front_guide_lip_0",
            min_gap=0.010,
            max_gap=0.040,
            name="left guide captures gate with side clearance",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="y",
            positive_elem="rear_guide_lip_0",
            negative_elem="gate_plate",
            min_gap=0.015,
            max_gap=0.040,
            name="rear lip clears sliding gate face",
        )
        ctx.expect_gap(
            gate,
            frame,
            axis="y",
            positive_elem="gate_plate",
            negative_elem="front_guide_lip_0",
            min_gap=0.015,
            max_gap=0.040,
            name="front lip clears sliding gate face",
        )
        ctx.expect_contact(
            gate,
            frame,
            elem_a="guide_shoe_0",
            elem_b="front_guide_lip_0",
            name="left guide shoe bears on guide lip",
        )
        ctx.expect_contact(
            gate,
            frame,
            elem_a="guide_shoe_1",
            elem_b="front_guide_lip_1",
            name="right guide shoe bears on guide lip",
        )
        rest_gate_pos = ctx.part_world_position(gate)

    with ctx.pose({gate_slide: 0.75}):
        ctx.expect_gap(
            frame,
            gate,
            axis="z",
            positive_elem="top_beam_front",
            negative_elem="gate_plate",
            min_gap=0.020,
            max_gap=0.080,
            name="raised gate clears underside of top beam",
        )
        ctx.expect_overlap(
            gate,
            frame,
            axes="z",
            elem_a="gate_plate",
            elem_b="front_guide_lip_0",
            min_overlap=1.45,
            name="raised gate remains captured in guide height",
        )
        raised_gate_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate travels upward along guide direction",
        rest_gate_pos is not None
        and raised_gate_pos is not None
        and raised_gate_pos[2] > rest_gate_pos[2] + 0.70,
        details=f"rest={rest_gate_pos}, raised={raised_gate_pos}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    ctx.expect_within(
        frame,
        wheel,
        axes="xy",
        inner_elem="handwheel_axle",
        outer_elem="center_collar",
        margin=0.015,
        name="axle stays centered in handwheel collar",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="z",
        elem_a="handwheel_axle",
        elem_b="center_collar",
        min_overlap=0.030,
        name="handwheel collar remains captured on axle",
    )
    with ctx.pose({wheel_spin: math.pi * 1.5}):
        turned_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "handwheel spins about fixed axle center",
        rest_wheel_pos is not None
        and turned_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, turned_wheel_pos)) < 1e-6,
        details=f"rest={rest_wheel_pos}, turned={turned_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()

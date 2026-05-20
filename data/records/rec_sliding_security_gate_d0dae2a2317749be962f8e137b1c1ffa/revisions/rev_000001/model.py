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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_steel = model.material("dark_powdercoat", rgba=(0.08, 0.09, 0.09, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.10, 1.0))

    def box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    base = model.part("base_frame")
    box(base, "concrete_sill", (4.55, 0.42, 0.12), (0.35, -0.02, 0.06), concrete)
    box(base, "rear_plinth", (4.20, 0.11, 0.09), (0.40, -0.24, 0.165), concrete)
    box(base, "latch_post", (0.16, 0.16, 2.02), (-1.72, -0.24, 1.13), galvanized)
    box(base, "receiver_post", (0.16, 0.16, 2.02), (2.52, -0.24, 1.13), galvanized)
    box(base, "fixed_header", (4.24, 0.12, 0.13), (0.40, -0.24, 2.08), galvanized)
    box(base, "lower_frame_rail", (4.24, 0.09, 0.08), (0.40, -0.24, 0.28), galvanized)
    for x in (-1.72, 2.52):
        box(base, f"post_cap_{x:+.1f}", (0.22, 0.22, 0.055), (x, -0.24, 2.17), dark_steel)
        box(base, f"warning_plate_{x:+.1f}", (0.04, 0.012, 0.42), (x, -0.156, 1.16), yellow)

    guide = model.part("guide_module")
    box(guide, "bottom_channel_base", (4.06, 0.25, 0.014), (0.40, 0.00, 0.127), dark_steel)
    box(guide, "running_rail", (3.96, 0.036, 0.020), (0.40, 0.00, 0.144), galvanized)
    box(guide, "front_guide_wall", (4.06, 0.026, 0.090), (0.40, 0.102, 0.179), dark_steel)
    box(guide, "rear_guide_wall", (4.06, 0.026, 0.090), (0.40, -0.102, 0.179), dark_steel)
    box(guide, "top_channel_cap", (4.06, 0.23, 0.040), (0.40, 0.00, 2.020), dark_steel)
    box(guide, "top_front_wall", (4.06, 0.026, 0.130), (0.40, 0.102, 1.935), dark_steel)
    box(guide, "top_rear_wall", (4.06, 0.026, 0.130), (0.40, -0.102, 1.935), dark_steel)
    for i, x in enumerate((-1.55, 0.40, 2.35)):
        box(guide, f"guide_upright_{i}", (0.075, 0.070, 1.790), (x, 0.102, 1.075), galvanized)
        box(guide, f"rear_tie_{i}", (0.075, 0.230, 0.070), (x, 0.000, 1.982), galvanized)
    for i, x in enumerate((-1.20, 0.40, 1.95)):
        box(guide, f"anchor_plate_{i}", (0.18, 0.30, 0.010), (x, 0.00, 0.134), galvanized)
        cyl(guide, f"anchor_bolt_{i}", 0.018, 0.030, (x - 0.055, 0.080, 0.151), dark_steel)
        cyl(guide, f"anchor_bolt_{i}_b", 0.018, 0.030, (x + 0.055, -0.080, 0.151), dark_steel)

    moving = model.part("moving_carriage")
    gate_width = 2.76
    box(moving, "lower_gate_rail", (gate_width, 0.065, 0.105), (0.00, 0.00, 0.375), galvanized)
    box(moving, "upper_gate_rail", (gate_width, 0.065, 0.105), (0.00, 0.00, 1.705), galvanized)
    box(moving, "nose_stile", (0.105, 0.065, 1.435), (-1.38, 0.00, 1.040), galvanized)
    box(moving, "tail_stile", (0.105, 0.065, 1.435), (1.38, 0.00, 1.040), galvanized)
    box(moving, "center_stile", (0.070, 0.060, 1.330), (0.00, 0.00, 1.040), galvanized)

    for i, x in enumerate((-1.08, -0.81, -0.54, -0.27, 0.27, 0.54, 0.81, 1.08)):
        box(moving, f"security_bar_{i}", (0.035, 0.042, 1.290), (x, 0.00, 1.040), dark_steel)

    brace_length = math.hypot(2.28, 1.08)
    brace_angle = -math.atan2(1.08, 2.28)
    box(
        moving,
        "diagonal_brace",
        (brace_length, 0.050, 0.055),
        (0.00, 0.00, 1.040),
        galvanized,
        rpy=(0.0, brace_angle, 0.0),
    )

    wheel_rpy = (math.pi / 2.0, 0.0, 0.0)
    for i, x in enumerate((-0.88, 0.88)):
        cyl(moving, f"bottom_wheel_{i}", 0.080, 0.050, (x, 0.00, 0.234), rubber, rpy=wheel_rpy)
        cyl(moving, f"bottom_axle_{i}", 0.014, 0.125, (x, 0.00, 0.234), dark_steel, rpy=wheel_rpy)
        box(moving, f"wheel_fork_front_{i}", (0.055, 0.014, 0.170), (x, 0.039, 0.292), dark_steel)
        box(moving, f"wheel_fork_rear_{i}", (0.055, 0.014, 0.170), (x, -0.039, 0.292), dark_steel)
        box(moving, f"wheel_mount_{i}", (0.130, 0.070, 0.040), (x, 0.00, 0.340), dark_steel)

    for i, x in enumerate((-0.72, 0.72)):
        cyl(moving, f"top_roller_{i}", 0.054, 0.050, (x, 0.00, 1.905), rubber, rpy=wheel_rpy)
        cyl(moving, f"top_axle_{i}", 0.012, 0.118, (x, 0.00, 1.905), dark_steel, rpy=wheel_rpy)
        box(moving, f"top_roller_stem_{i}", (0.045, 0.045, 0.190), (x, 0.00, 1.805), dark_steel)
        box(moving, f"top_roller_mount_{i}", (0.125, 0.065, 0.040), (x, 0.00, 1.760), dark_steel)

    box(moving, "sliding_handle", (0.045, 0.120, 0.220), (-1.23, -0.080, 1.08), dark_steel)
    box(moving, "handle_upper_bracket", (0.150, 0.100, 0.035), (-1.292, -0.066, 1.165), dark_steel)
    box(moving, "handle_lower_bracket", (0.150, 0.100, 0.035), (-1.292, -0.066, 0.995), dark_steel)

    model.articulation(
        "base_to_guide",
        ArticulationType.FIXED,
        parent=base,
        child=guide,
        origin=Origin(),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=moving,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.45, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    guide = object_model.get_part("guide_module")
    moving = object_model.get_part("moving_carriage")
    slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "gate slide is prismatic along the track",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.expect_contact(
        guide,
        base,
        elem_a="bottom_channel_base",
        elem_b="concrete_sill",
        name="guide module is bolted down to the base sill",
    )
    ctx.expect_overlap(
        moving,
        guide,
        axes="x",
        elem_a="lower_gate_rail",
        elem_b="running_rail",
        min_overlap=2.4,
        name="closed gate leaf rides on the guide rail",
    )
    for roller in ("top_roller_0", "top_roller_1"):
        ctx.expect_gap(
            guide,
            moving,
            axis="y",
            positive_elem="top_front_wall",
            negative_elem=roller,
            min_gap=0.020,
            name=f"{roller} clears the front top channel wall",
        )
        ctx.expect_gap(
            moving,
            guide,
            axis="y",
            positive_elem=roller,
            negative_elem="top_rear_wall",
            min_gap=0.020,
            name=f"{roller} clears the rear top channel wall",
        )
        ctx.expect_within(
            moving,
            guide,
            axes="x",
            inner_elem=roller,
            outer_elem="top_channel_cap",
            margin=0.0,
            name=f"{roller} stays under the top track at rest",
        )

    rest_pos = ctx.part_world_position(moving)
    with ctx.pose({slide: 1.15}):
        extended_pos = ctx.part_world_position(moving)
        ctx.expect_overlap(
            moving,
            guide,
            axes="x",
            elem_a="lower_gate_rail",
            elem_b="running_rail",
            min_overlap=2.0,
            name="extended gate remains retained on the rail",
        )
        ctx.expect_within(
            moving,
            guide,
            axes="x",
            inner_elem="top_roller_1",
            outer_elem="top_channel_cap",
            margin=0.0,
            name="extended top carriage remains under the track",
        )
    ctx.check(
        "positive slide opens the gate along +X",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()

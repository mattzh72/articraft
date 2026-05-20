from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_sliding_security_gate")

    painted = model.material("hammered_dark_green_paint", rgba=(0.06, 0.12, 0.10, 1.0))
    rail_galv = model.material("galvanized_worn_steel", rgba=(0.48, 0.50, 0.48, 1.0))
    rubber = model.material("black_molded_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    concrete = model.material("rough_concrete", rgba=(0.46, 0.44, 0.40, 1.0))
    bolt_metal = model.material("bright_worn_bolt_heads", rgba=(0.70, 0.67, 0.58, 1.0))
    warning = model.material("safety_yellow_handle", rgba=(0.95, 0.68, 0.08, 1.0))

    track_frame = model.part("track_frame")
    _box(track_frame, "concrete_footing", (6.05, 0.62, 0.18), (2.58, 0.0, 0.09), concrete)
    _box(track_frame, "lower_rail_bed", (5.85, 0.052, 0.055), (2.55, 0.0, 0.2075), rail_galv)
    _box(track_frame, "lower_channel_lip_0", (5.85, 0.032, 0.145), (2.55, -0.095, 0.2525), rail_galv)
    _box(track_frame, "lower_channel_lip_1", (5.85, 0.032, 0.145), (2.55, 0.095, 0.2525), rail_galv)
    _box(track_frame, "latch_post", (0.19, 0.24, 2.34), (-0.34, 0.0, 1.35), painted)
    _box(track_frame, "guide_post", (0.19, 0.24, 2.34), (3.47, 0.0, 1.35), painted)
    _box(track_frame, "service_post", (0.18, 0.22, 2.34), (5.45, 0.0, 1.35), painted)
    _box(track_frame, "overhead_track_spine", (5.95, 0.29, 0.13), (2.58, 0.0, 2.35), rail_galv)
    _box(track_frame, "overhead_track_lip_0", (5.95, 0.035, 0.18), (2.58, -0.145, 2.235), rail_galv)
    _box(track_frame, "overhead_track_lip_1", (5.95, 0.035, 0.18), (2.58, 0.145, 2.235), rail_galv)
    _box(track_frame, "post_base_plate_0", (0.38, 0.34, 0.035), (-0.34, 0.0, 0.1975), rail_galv)
    _box(track_frame, "post_base_plate_1", (0.38, 0.34, 0.035), (3.47, 0.0, 0.1975), rail_galv)
    _box(track_frame, "post_base_plate_2", (0.36, 0.32, 0.035), (5.45, 0.0, 0.1975), rail_galv)
    _box(track_frame, "receiver_mount_top", (0.24, 0.075, 0.09), (-0.33, -0.145, 1.315), painted)
    _box(track_frame, "receiver_mount_bottom", (0.24, 0.075, 0.09), (-0.33, -0.145, 1.085), painted)
    _box(track_frame, "receiver_upper_jaw", (0.23, 0.065, 0.055), (-0.12, -0.145, 1.300), painted)
    _box(track_frame, "receiver_lower_jaw", (0.23, 0.065, 0.055), (-0.12, -0.145, 1.100), painted)
    _box(track_frame, "receiver_back_strap", (0.065, 0.075, 0.30), (-0.23, -0.145, 1.20), painted)

    for i, x in enumerate((-0.42, -0.26, 3.39, 3.55, 5.37, 5.53)):
        y = -0.12 if i % 2 == 0 else 0.12
        _cyl(track_frame, f"base_anchor_{i}", 0.025, 0.018, (x, y, 0.226), bolt_metal)
    for i, (x, z) in enumerate(((-0.32, 1.315), (-0.32, 1.085), (-0.14, 1.275), (-0.14, 1.125))):
        _cyl(track_frame, f"receiver_fastener_{i}", 0.018, 0.018, (x, -0.185, z), bolt_metal, rpy=(math.pi / 2.0, 0.0, 0.0))

    gate_leaf = model.part("gate_leaf")
    _box(gate_leaf, "bottom_tube", (3.28, 0.115, 0.12), (1.65, 0.0, 0.58), painted)
    _box(gate_leaf, "top_tube", (3.28, 0.18, 0.12), (1.65, 0.0, 1.94), painted)
    _box(gate_leaf, "leading_stile", (0.13, 0.12, 1.54), (0.07, 0.0, 1.26), painted)
    _box(gate_leaf, "trailing_stile", (0.13, 0.12, 1.54), (3.23, 0.0, 1.26), painted)
    for i, x in enumerate((0.52, 0.92, 1.32, 1.72, 2.12, 2.52, 2.92)):
        _box(gate_leaf, f"vertical_bar_{i}", (0.055, 0.07, 1.28), (x, 0.0, 1.25), painted)
    brace_angle = -math.atan2(1.08, 2.80)
    brace_len = math.hypot(2.80, 1.08)
    _box(gate_leaf, "diagonal_brace_0", (brace_len, 0.080, 0.075), (1.65, 0.006, 1.26), painted, rpy=(0.0, brace_angle, 0.0))
    _box(gate_leaf, "diagonal_brace_1", (brace_len, 0.080, 0.075), (1.65, -0.006, 1.26), painted, rpy=(0.0, -brace_angle, 0.0))
    _box(gate_leaf, "kick_plate", (3.03, 0.035, 0.32), (1.66, 0.067, 0.82), painted)
    for i, (x, z) in enumerate(((0.17, 0.70), (0.17, 1.82), (3.12, 0.70), (3.12, 1.82))):
        _box(gate_leaf, f"corner_gusset_{i}", (0.25, 0.036, 0.18), (x, -0.066, z), painted)

    for i, x in enumerate((0.55, 2.75)):
        _box(gate_leaf, f"roller_yoke_plate_{i}_0", (0.26, 0.014, 0.245), (x, -0.055, 0.425), painted)
        _box(gate_leaf, f"roller_yoke_plate_{i}_1", (0.26, 0.014, 0.245), (x, 0.055, 0.425), painted)
        _box(gate_leaf, f"roller_yoke_cap_{i}", (0.28, 0.13, 0.045), (x, 0.0, 0.525), painted)

    for i, x in enumerate((0.82, 2.42)):
        _box(gate_leaf, f"top_guide_stem_{i}_0", (0.055, 0.04, 0.13), (x, -0.090, 2.035), painted)
        _box(gate_leaf, f"top_guide_stem_{i}_1", (0.055, 0.04, 0.13), (x, 0.090, 2.035), painted)
        _cyl(gate_leaf, f"top_guide_roller_{i}_0", 0.035, 0.105, (x, -0.090, 2.100), rubber)
        _cyl(gate_leaf, f"top_guide_roller_{i}_1", 0.035, 0.105, (x, 0.090, 2.100), rubber)

    _box(gate_leaf, "latch_backplate", (0.24, 0.032, 0.27), (0.13, -0.070, 1.20), painted)
    _box(gate_leaf, "latch_upper_guide", (0.23, 0.070, 0.035), (0.12, -0.115, 1.255), painted)
    _box(gate_leaf, "latch_lower_guide", (0.23, 0.070, 0.035), (0.12, -0.115, 1.145), painted)
    _box(gate_leaf, "latch_stop_lug", (0.045, 0.055, 0.16), (0.25, -0.120, 1.20), painted)

    fastener_points = []
    for x in (0.32, 0.92, 1.52, 2.12, 2.72, 3.05):
        fastener_points.append((x, 0.59))
        fastener_points.append((x, 1.93))
    for z in (0.80, 1.23, 1.66):
        fastener_points.append((0.07, z))
        fastener_points.append((3.23, z))
    for i, (x, z) in enumerate(fastener_points):
        _cyl(gate_leaf, f"exposed_bolt_{i}", 0.018, 0.016, (x, -0.065, z), bolt_metal, rpy=(math.pi / 2.0, 0.0, 0.0))

    model.articulation(
        "track_to_gate",
        ArticulationType.PRISMATIC,
        parent=track_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=2.20),
        motion_properties=MotionProperties(damping=12.0, friction=5.0),
    )

    for i, x in enumerate((0.55, 2.75)):
        roller = model.part(f"bottom_roller_{i}")
        _cyl(roller, "tire", 0.120, 0.055, (0.0, 0.0, 0.0), rubber, rpy=(math.pi / 2.0, 0.0, 0.0))
        _cyl(roller, "flange_0", 0.132, 0.012, (0.0, -0.034, 0.0), rail_galv, rpy=(math.pi / 2.0, 0.0, 0.0))
        _cyl(roller, "flange_1", 0.132, 0.012, (0.0, 0.034, 0.0), rail_galv, rpy=(math.pi / 2.0, 0.0, 0.0))
        _cyl(roller, "hub", 0.045, 0.096, (0.0, 0.0, 0.0), bolt_metal, rpy=(math.pi / 2.0, 0.0, 0.0))
        model.articulation(
            f"gate_to_bottom_roller_{i}",
            ArticulationType.CONTINUOUS,
            parent=gate_leaf,
            child=roller,
            origin=Origin(xyz=(x, 0.0, 0.352)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.2, friction=0.05),
        )

    latch_bolt = model.part("latch_bolt")
    _box(latch_bolt, "bolt_bar", (0.30, 0.035, 0.075), (-0.10, 0.0, 0.0), bolt_metal)
    _box(latch_bolt, "handle_stem", (0.035, 0.105, 0.035), (0.02, -0.045, 0.0), bolt_metal)
    _cyl(latch_bolt, "pull_grip", 0.026, 0.17, (0.02, -0.105, 0.085), warning)
    model.articulation(
        "gate_to_latch_bolt",
        ArticulationType.PRISMATIC,
        parent=gate_leaf,
        child=latch_bolt,
        origin=Origin(xyz=(0.15, -0.150, 1.20)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.16),
        motion_properties=MotionProperties(damping=2.0, friction=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    track = object_model.get_part("track_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("track_to_gate")
    roller_0 = object_model.get_part("bottom_roller_0")
    roller_1 = object_model.get_part("bottom_roller_1")
    latch = object_model.get_part("latch_bolt")
    latch_slide = object_model.get_articulation("gate_to_latch_bolt")

    ctx.allow_overlap(
        roller_0,
        track,
        elem_a="tire",
        elem_b="lower_rail_bed",
        reason="The molded rubber bottom wheel is intentionally seated with slight compression on the steel rail.",
    )
    ctx.allow_overlap(
        roller_1,
        track,
        elem_a="tire",
        elem_b="lower_rail_bed",
        reason="The molded rubber bottom wheel is intentionally seated with slight compression on the steel rail.",
    )

    ctx.expect_gap(
        roller_0,
        track,
        axis="z",
        positive_elem="tire",
        negative_elem="lower_rail_bed",
        max_gap=0.002,
        max_penetration=0.004,
        name="front roller sits on lower rail bed",
    )
    ctx.expect_gap(
        roller_1,
        track,
        axis="z",
        positive_elem="tire",
        negative_elem="lower_rail_bed",
        max_gap=0.002,
        max_penetration=0.004,
        name="rear roller sits on lower rail bed",
    )
    ctx.expect_within(
        roller_0,
        track,
        axes="y",
        inner_elem="tire",
        outer_elem="lower_rail_bed",
        margin=0.002,
        name="roller is captured between channel lips",
    )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: 2.20}):
        open_pos = ctx.part_world_position(gate)
        ctx.expect_gap(
            roller_1,
            track,
            axis="z",
            positive_elem="tire",
            negative_elem="lower_rail_bed",
            max_gap=0.002,
            max_penetration=0.004,
            name="open gate remains supported by rail",
        )
    ctx.check(
        "gate leaf slides along visible track",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 2.0,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    with ctx.pose({latch_slide: 0.10}):
        ctx.expect_overlap(
            latch,
            track,
            axes="xy",
            elem_a="bolt_bar",
            elem_b="receiver_lower_jaw",
            min_overlap=0.02,
            name="extended latch bolt enters receiver footprint",
        )
        ctx.expect_gap(
            latch,
            track,
            axis="z",
            positive_elem="bolt_bar",
            negative_elem="receiver_lower_jaw",
            min_gap=0.015,
            max_gap=0.050,
            name="latch bolt clears lower receiver jaw",
        )
        ctx.expect_gap(
            track,
            latch,
            axis="z",
            positive_elem="receiver_upper_jaw",
            negative_elem="bolt_bar",
            min_gap=0.015,
            max_gap=0.050,
            name="latch bolt clears upper receiver jaw",
        )

    return ctx.report()


object_model = build_object_model()

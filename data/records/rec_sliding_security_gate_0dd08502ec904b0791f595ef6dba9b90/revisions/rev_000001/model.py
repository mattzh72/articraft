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


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_sliding_security_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.60, 0.58, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.10, 0.12, 0.12, 1.0))
    concrete = model.material("cast_concrete", rgba=(0.42, 0.42, 0.38, 1.0))
    hazard_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.85, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    support = model.part("support")

    # Continuous fixed support structure: plinth, captured lower track, posts,
    # overhead guide channel, keeper, guards, and hard over-travel stops.
    _box(support, "plinth", (5.25, 0.46, 0.10), (0.20, 0.0, 0.05), concrete)
    _box(support, "track_plate", (5.00, 0.11, 0.04), (0.20, 0.0, 0.12), dark_steel)
    _box(support, "front_track_guard", (5.00, 0.04, 0.20), (0.20, -0.16, 0.20), hazard_yellow)
    _box(support, "rear_track_guard", (5.00, 0.04, 0.20), (0.20, 0.16, 0.20), hazard_yellow)
    _box(support, "guard_tie_0", (0.10, 0.36, 0.06), (-2.15, 0.0, 0.24), hazard_yellow)
    _box(support, "guard_tie_1", (0.10, 0.36, 0.06), (2.55, 0.0, 0.24), hazard_yellow)

    for x, name in [(-1.98, "receiver_post"), (0.78, "guide_post"), (2.28, "stop_post")]:
        _box(support, name, (0.18, 0.18, 2.10), (x, -0.22, 1.15), galvanized)
        _box(support, f"{name}_foot", (0.42, 0.34, 0.035), (x, -0.22, 0.1175), dark_steel)
        for sx in (-0.13, 0.13):
            _box(support, f"{name}_gusset_{sx:+.2f}", (0.035, 0.30, 0.30), (x + sx, -0.22, 0.28), dark_steel)

    _box(support, "top_beam", (4.35, 0.12, 0.12), (0.16, -0.22, 2.16), galvanized)
    _box(support, "front_top_channel", (4.55, 0.035, 0.08), (0.18, -0.095, 1.97), hazard_yellow)
    _box(support, "rear_top_channel", (4.55, 0.035, 0.08), (0.18, 0.095, 1.97), hazard_yellow)
    for x in (-1.70, -0.40, 0.90, 2.05):
        _box(support, f"top_channel_bridge_{x:+.1f}", (0.10, 0.34, 0.08), (x, 0.0, 1.97), dark_steel)

    for x in (0.58, 1.72):
        _cyl(support, f"front_guide_roller_{x:.1f}", 0.026, 0.24, (x, -0.125, 1.70), dark_steel)
        _cyl(support, f"rear_guide_roller_{x:.1f}", 0.026, 0.24, (x, 0.125, 1.70), dark_steel)
        _box(support, f"front_roller_stem_{x:.1f}", (0.06, 0.05, 0.26), (x, -0.125, 1.90), galvanized)
        _box(support, f"rear_roller_stem_{x:.1f}", (0.06, 0.05, 0.26), (x, 0.125, 1.90), galvanized)

    _box(support, "closed_stop_block", (0.14, 0.08, 0.34), (-1.92, -0.23, 0.36), hazard_yellow)
    _box(support, "open_stop_block", (0.14, 0.08, 0.34), (2.22, -0.23, 0.36), hazard_yellow)
    _box(support, "keeper_backplate", (0.16, 0.04, 0.34), (-1.86, -0.13, 1.10), dark_steel)
    _box(support, "keeper_pocket", (0.14, 0.05, 0.24), (-1.84, -0.095, 1.10), galvanized)
    _box(support, "lockout_hinge_plate", (0.14, 0.03, 0.24), (-1.92, -0.115, 1.38), lockout_red)
    _cyl(support, "fixed_hinge_pin", 0.022, 0.05, (-1.92, -0.125, 1.38), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))

    gate = model.part("gate_leaf")

    # Moving welded gate leaf.  All load paths touch the rectangular tube frame:
    # corners are plated, the lower rollers are forked back into the bottom rail,
    # and the latch striker is backed by a reinforcement pad.
    _box(gate, "bottom_tube", (2.50, 0.08, 0.10), (0.0, 0.0, 0.42), galvanized)
    _box(gate, "top_tube", (2.50, 0.08, 0.10), (0.0, 0.0, 1.82), galvanized)
    _box(gate, "left_stile", (0.10, 0.08, 1.50), (-1.20, 0.0, 1.12), galvanized)
    _box(gate, "right_stile", (0.10, 0.08, 1.50), (1.20, 0.0, 1.12), galvanized)
    _box(gate, "mid_rail", (2.30, 0.06, 0.07), (0.0, 0.0, 1.12), galvanized)
    _box(gate, "kick_plate", (2.25, 0.026, 0.26), (0.0, -0.053, 0.59), hazard_yellow)

    for i, x in enumerate((-0.90, -0.60, -0.30, 0.0, 0.30, 0.60, 0.90)):
        _box(gate, f"picket_{i}", (0.035, 0.035, 1.34), (x, 0.0, 1.12), galvanized)

    diag_len = math.hypot(1.16, 1.28)
    diag_angle = math.atan2(1.28, 1.16)
    _box(gate, "left_diagonal_brace", (diag_len, 0.055, 0.055), (-0.58, 0.0, 1.12), dark_steel, rpy=(0.0, -diag_angle, 0.0))
    _box(gate, "right_diagonal_brace", (diag_len, 0.055, 0.055), (0.58, 0.0, 1.12), dark_steel, rpy=(0.0, diag_angle, 0.0))

    for x, side in [(-1.20, "left"), (1.20, "right")]:
        _box(gate, f"{side}_corner_plate_low", (0.24, 0.024, 0.24), (x, -0.040, 0.48), dark_steel)
        _box(gate, f"{side}_corner_plate_high", (0.24, 0.024, 0.24), (x, -0.040, 1.76), dark_steel)
        _box(gate, f"{side}_rear_corner_plate_low", (0.24, 0.024, 0.24), (x, 0.040, 0.48), dark_steel)
        _box(gate, f"{side}_rear_corner_plate_high", (0.24, 0.024, 0.24), (x, 0.040, 1.76), dark_steel)

    for x, idx in [(-0.82, 0), (0.82, 1)]:
        _cyl(gate, f"wheel_{idx}", 0.11, 0.07, (x, 0.0, 0.25), rubber, rpy=(math.pi / 2.0, 0.0, 0.0))
        _cyl(gate, f"wheel_axle_{idx}", 0.024, 0.18, (x, 0.0, 0.25), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
        _box(gate, f"front_wheel_fork_{idx}", (0.18, 0.018, 0.28), (x, -0.058, 0.34), dark_steel)
        _box(gate, f"rear_wheel_fork_{idx}", (0.18, 0.018, 0.28), (x, 0.058, 0.34), dark_steel)
        _box(gate, f"wheel_guard_{idx}", (0.36, 0.030, 0.18), (x, -0.080, 0.30), hazard_yellow)

    _box(gate, "striker_backer", (0.24, 0.024, 0.36), (-1.23, -0.055, 1.10), dark_steel)
    _box(gate, "striker_plate", (0.16, 0.030, 0.24), (-1.28, -0.055, 1.10), galvanized)
    _box(gate, "inspection_tag", (0.32, 0.018, 0.16), (0.42, -0.025, 1.42), lockout_red)

    lockout_bar = model.part("lockout_bar")
    _cyl(lockout_bar, "hinge_barrel", 0.040, 0.05, (0.0, 0.0, 0.0), lockout_red, rpy=(math.pi / 2.0, 0.0, 0.0))
    _box(lockout_bar, "hasp_arm", (0.62, 0.040, 0.060), (0.31, 0.0, 0.0), lockout_red)
    _box(lockout_bar, "padlock_tab", (0.11, 0.046, 0.16), (0.60, 0.0, -0.02), lockout_red)
    _cyl(lockout_bar, "pull_knob", 0.035, 0.050, (0.42, -0.045, 0.0), hazard_yellow, rpy=(math.pi / 2.0, 0.0, 0.0))

    model.articulation(
        "support_to_gate",
        ArticulationType.PRISMATIC,
        parent=support,
        child=gate,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.35, lower=0.0, upper=1.40),
    )

    model.articulation(
        "lockout_hinge",
        ArticulationType.REVOLUTE,
        parent=support,
        child=lockout_bar,
        origin=Origin(xyz=(-1.92, -0.075, 1.38)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    gate = object_model.get_part("gate_leaf")
    lockout = object_model.get_part("lockout_bar")
    gate_slide = object_model.get_articulation("support_to_gate")
    lockout_hinge = object_model.get_articulation("lockout_hinge")

    for wheel in ("wheel_0", "wheel_1"):
        ctx.expect_gap(
            gate,
            support,
            axis="z",
            positive_elem=wheel,
            negative_elem="track_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{wheel} rides on the lower track",
        )
        ctx.expect_within(
            gate,
            support,
            axes="y",
            inner_elem=wheel,
            outer_elem="track_plate",
            margin=0.001,
            name=f"{wheel} is laterally captured by the track",
        )

    ctx.expect_gap(
        gate,
        support,
        axis="y",
        positive_elem="striker_plate",
        negative_elem="keeper_pocket",
        max_gap=0.002,
        max_penetration=0.0,
        name="striker seats at keeper without penetrating it",
    )

    rest_gate_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 1.40}):
        extended_gate_pos = ctx.part_world_position(gate)
        for wheel in ("wheel_0", "wheel_1"):
            ctx.expect_gap(
                gate,
                support,
                axis="z",
                positive_elem=wheel,
                negative_elem="track_plate",
                max_gap=0.001,
                max_penetration=0.0,
                name=f"{wheel} remains on track at full travel",
            )
            ctx.expect_within(
                gate,
                support,
                axes="x",
                inner_elem=wheel,
                outer_elem="track_plate",
                margin=0.0,
                name=f"{wheel} remains on the end-to-end rail",
            )

    ctx.check(
        "gate translates in the opening direction",
        rest_gate_pos is not None
        and extended_gate_pos is not None
        and extended_gate_pos[0] > rest_gate_pos[0] + 1.30,
        details=f"rest={rest_gate_pos}, extended={extended_gate_pos}",
    )

    rest_lockout_aabb = ctx.part_world_aabb(lockout)
    with ctx.pose({lockout_hinge: 1.20}):
        raised_lockout_aabb = ctx.part_world_aabb(lockout)
    ctx.check(
        "lockout hasp pivots upward for service access",
        rest_lockout_aabb is not None
        and raised_lockout_aabb is not None
        and raised_lockout_aabb[1][2] > rest_lockout_aabb[1][2] + 0.25,
        details=f"rest={rest_lockout_aabb}, raised={raised_lockout_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

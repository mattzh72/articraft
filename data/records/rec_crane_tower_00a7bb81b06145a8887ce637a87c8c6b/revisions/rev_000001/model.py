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


def _beam_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return an Origin that aligns a box's local +X axis between two points."""

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must be distinct")

    yaw = math.atan2(dy, dx)
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(-dz, horizontal)
    origin = Origin(
        xyz=(
            (start[0] + end[0]) * 0.5,
            (start[1] + end[1]) * 0.5,
            (start[2] + end[2]) * 0.5,
        ),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_beam(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: float,
    material,
    *,
    overshoot: float = 0.02,
) -> None:
    """Add a rectangular structural member that slightly seats into its nodes."""

    origin, length = _beam_origin_between(start, end)
    part.visual(
        Box((length + 2.0 * overshoot, thickness, thickness)),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane")

    crane_yellow = model.material("painted_yellow", color=(1.0, 0.74, 0.05, 1.0))
    dark_steel = model.material("dark_steel", color=(0.05, 0.055, 0.06, 1.0))
    concrete = model.material("weathered_concrete", color=(0.55, 0.55, 0.50, 1.0))
    counterweight_gray = model.material("counterweight_gray", color=(0.38, 0.38, 0.36, 1.0))
    glass_blue = model.material("cab_glass", color=(0.08, 0.20, 0.32, 1.0))
    safety_red = model.material("safety_red", color=(0.85, 0.05, 0.03, 1.0))

    # Fixed base and tower mast: a square lattice mast on a concrete footing.
    mast = model.part("mast")
    mast.visual(
        Box((2.2, 2.2, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=concrete,
        name="foundation",
    )
    mast.visual(
        Box((1.10, 1.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 9.95)),
        material=crane_yellow,
        name="top_cap",
    )

    leg_span = 0.82
    leg_z0 = 0.18
    leg_z1 = 9.92
    for ix, x in enumerate((-leg_span * 0.5, leg_span * 0.5)):
        for iy, y in enumerate((-leg_span * 0.5, leg_span * 0.5)):
            _add_beam(
                mast,
                f"mast_leg_{ix}_{iy}",
                (x, y, leg_z0),
                (x, y, leg_z1),
                0.115,
                crane_yellow,
                overshoot=0.015,
            )

    bay_count = 8
    bay_step = (leg_z1 - leg_z0) / bay_count
    for bay in range(bay_count):
        z0 = leg_z0 + bay * bay_step
        z1 = z0 + bay_step
        # Lacing diagonals on all four faces give the mast its tower-crane look.
        for y in (-leg_span * 0.5, leg_span * 0.5):
            if bay % 2 == 0:
                a = (-leg_span * 0.5, y, z0)
                b = (leg_span * 0.5, y, z1)
            else:
                a = (leg_span * 0.5, y, z0)
                b = (-leg_span * 0.5, y, z1)
            _add_beam(mast, f"mast_x_lace_{bay}_{'p' if y > 0 else 'n'}", a, b, 0.055, crane_yellow)

        for x in (-leg_span * 0.5, leg_span * 0.5):
            if bay % 2 == 0:
                a = (x, -leg_span * 0.5, z0)
                b = (x, leg_span * 0.5, z1)
            else:
                a = (x, leg_span * 0.5, z0)
                b = (x, -leg_span * 0.5, z1)
            _add_beam(mast, f"mast_y_lace_{bay}_{'p' if x > 0 else 'n'}", a, b, 0.055, crane_yellow)

    # Rotating superstructure: slewing bearing, cab, jib, counter-jib, tower head.
    slewing_unit = model.part("slewing_unit")
    slewing_unit.visual(
        Cylinder(radius=0.66, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_steel,
        name="slewing_ring",
    )
    slewing_unit.visual(
        Cylinder(radius=0.78, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=crane_yellow,
        name="turntable",
    )
    slewing_unit.visual(
        Box((0.95, 0.22, 0.12)),
        origin=Origin(xyz=(0.24, -0.53, 0.38)),
        material=crane_yellow,
        name="cab_support",
    )
    slewing_unit.visual(
        Box((0.78, 0.54, 0.62)),
        origin=Origin(xyz=(0.34, -0.86, 0.69)),
        material=crane_yellow,
        name="operator_cab",
    )
    slewing_unit.visual(
        Box((0.48, 0.018, 0.28)),
        origin=Origin(xyz=(0.34, -1.135, 0.76)),
        material=glass_blue,
        name="cab_front_window",
    )
    slewing_unit.visual(
        Box((0.28, 0.020, 0.24)),
        origin=Origin(xyz=(0.735, -0.86, 0.76), rpy=(0.0, 0.0, math.pi * 0.5)),
        material=glass_blue,
        name="cab_side_window",
    )

    _add_beam(slewing_unit, "tower_head_front", (-0.25, -0.20, 0.30), (0.0, -0.08, 2.55), 0.115, crane_yellow)
    _add_beam(slewing_unit, "tower_head_rear", (-0.25, 0.20, 0.30), (0.0, 0.08, 2.55), 0.115, crane_yellow)
    _add_beam(slewing_unit, "tower_head_cross", (-0.12, -0.18, 1.45), (-0.12, 0.18, 1.45), 0.080, crane_yellow)

    # Jib truss extending along +X.
    jib_start = 0.55
    jib_end = 12.2
    lower_z = 0.68
    upper_z = 1.28
    half_width = 0.24
    _add_beam(slewing_unit, "jib_lower_chord_0", (jib_start, -half_width, lower_z), (jib_end, -half_width, lower_z), 0.095, crane_yellow)
    _add_beam(slewing_unit, "jib_lower_chord_1", (jib_start, half_width, lower_z), (jib_end, half_width, lower_z), 0.095, crane_yellow)
    _add_beam(slewing_unit, "jib_upper_chord", (jib_start, 0.0, upper_z), (jib_end, 0.0, upper_z), 0.080, crane_yellow)
    _add_beam(slewing_unit, "jib_track", (0.75, 0.0, 0.45), (11.95, 0.0, 0.45), 0.085, dark_steel)

    jib_bays = 10
    for i in range(jib_bays + 1):
        x = jib_start + (jib_end - jib_start) * i / jib_bays
        _add_beam(slewing_unit, f"jib_web_left_{i}", (x, -half_width, lower_z), (x, 0.0, upper_z), 0.045, crane_yellow)
        _add_beam(slewing_unit, f"jib_web_right_{i}", (x, half_width, lower_z), (x, 0.0, upper_z), 0.045, crane_yellow)
        if i < jib_bays:
            x2 = jib_start + (jib_end - jib_start) * (i + 1) / jib_bays
            _add_beam(slewing_unit, f"jib_diag_left_{i}", (x, -half_width, lower_z), (x2, 0.0, upper_z), 0.040, crane_yellow)
            _add_beam(slewing_unit, f"jib_diag_right_{i}", (x, half_width, lower_z), (x2, 0.0, upper_z), 0.040, crane_yellow)
            _add_beam(slewing_unit, f"track_hanger_{i}", (x, 0.0, 0.45), (x, 0.0, lower_z), 0.040, crane_yellow)
            _add_beam(slewing_unit, f"track_tie_{i}", (x, -half_width, lower_z), (x, half_width, lower_z), 0.040, crane_yellow)

    # Counter-jib and counterweight package along -X.
    counter_end = -4.35
    _add_beam(slewing_unit, "counter_lower_chord_0", (-0.45, -half_width, 0.76), (counter_end, -half_width, 0.76), 0.100, crane_yellow)
    _add_beam(slewing_unit, "counter_lower_chord_1", (-0.45, half_width, 0.76), (counter_end, half_width, 0.76), 0.100, crane_yellow)
    _add_beam(slewing_unit, "counter_upper_chord", (-0.45, 0.0, 1.16), (counter_end, 0.0, 1.16), 0.080, crane_yellow)
    for i in range(4):
        x = -0.75 - i * 0.90
        _add_beam(slewing_unit, f"counter_web_{i}", (x, -half_width, 0.76), (x, 0.0, 1.16), 0.045, crane_yellow)
        _add_beam(slewing_unit, f"counter_web_pair_{i}", (x, half_width, 0.76), (x, 0.0, 1.16), 0.045, crane_yellow)
    slewing_unit.visual(
        Box((1.20, 0.62, 0.50)),
        origin=Origin(xyz=(-3.65, 0.0, 0.49)),
        material=counterweight_gray,
        name="counterweight_block",
    )
    _add_beam(slewing_unit, "jib_pendant", (0.0, 0.0, 2.48), (10.9, 0.0, 1.30), 0.035, dark_steel)
    _add_beam(slewing_unit, "counter_pendant", (0.0, 0.0, 2.42), (-3.90, 0.0, 1.16), 0.040, dark_steel)

    model.articulation(
        "mast_to_slewing",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=slewing_unit,
        origin=Origin(xyz=(0.0, 0.0, 10.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )

    # Trolley on the jib track; the child frame follows the carriage center.
    trolley = model.part("trolley")
    trolley.visual(
        Box((0.66, 0.56, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=crane_yellow,
        name="trolley_body",
    )
    trolley.visual(
        Box((0.60, 0.08, 0.30)),
        origin=Origin(xyz=(0.0, -0.30, 0.00)),
        material=crane_yellow,
        name="side_plate_0",
    )
    trolley.visual(
        Box((0.60, 0.08, 0.30)),
        origin=Origin(xyz=(0.0, 0.30, 0.00)),
        material=crane_yellow,
        name="side_plate_1",
    )
    trolley.visual(
        Cylinder(radius=0.07, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.08), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="track_roller",
    )
    _add_beam(trolley, "hoist_cable", (0.0, 0.0, -0.18), (0.0, 0.0, -2.00), 0.025, dark_steel, overshoot=0.0)
    trolley.visual(
        Box((0.26, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -2.08)),
        material=crane_yellow,
        name="hook_block",
    )
    _add_beam(trolley, "hook_shank", (0.0, 0.0, -2.16), (0.0, 0.0, -2.45), 0.045, dark_steel, overshoot=0.0)
    _add_beam(trolley, "hook_throat", (0.0, 0.0, -2.45), (0.18, 0.0, -2.45), 0.045, safety_red, overshoot=0.0)
    _add_beam(trolley, "hook_tip", (0.18, 0.0, -2.45), (0.18, 0.0, -2.30), 0.045, safety_red, overshoot=0.0)

    model.articulation(
        "jib_to_trolley",
        ArticulationType.PRISMATIC,
        parent=slewing_unit,
        child=trolley,
        origin=Origin(xyz=(1.20, 0.0, 0.2575)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.80, lower=0.0, upper=9.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    slewing_unit = object_model.get_part("slewing_unit")
    trolley = object_model.get_part("trolley")
    slew_joint = object_model.get_articulation("mast_to_slewing")
    trolley_joint = object_model.get_articulation("jib_to_trolley")

    ctx.check(
        "slewing joint is vertical revolute",
        slew_joint.articulation_type == ArticulationType.REVOLUTE and tuple(slew_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={slew_joint.articulation_type}, axis={slew_joint.axis}",
    )
    ctx.check(
        "trolley joint slides along jib",
        trolley_joint.articulation_type == ArticulationType.PRISMATIC and tuple(trolley_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={trolley_joint.articulation_type}, axis={trolley_joint.axis}",
    )
    ctx.expect_contact(
        mast,
        slewing_unit,
        elem_a="top_cap",
        elem_b="slewing_ring",
        contact_tol=0.002,
        name="slewing ring sits on mast top",
    )
    ctx.expect_gap(
        slewing_unit,
        trolley,
        axis="z",
        positive_elem="jib_track",
        negative_elem="track_roller",
        min_gap=-0.001,
        max_gap=0.003,
        name="trolley roller is seated under the jib track",
    )
    ctx.expect_within(
        trolley,
        slewing_unit,
        axes="x",
        inner_elem="trolley_body",
        outer_elem="jib_track",
        margin=0.0,
        name="trolley starts within the jib track length",
    )

    rest_pos = ctx.part_world_position(trolley)
    with ctx.pose({trolley_joint: 9.80}):
        ctx.expect_within(
            trolley,
            slewing_unit,
            axes="x",
            inner_elem="trolley_body",
            outer_elem="jib_track",
            margin=0.0,
            name="trolley remains within the jib track at max travel",
        )
        extended_pos = ctx.part_world_position(trolley)

    ctx.check(
        "trolley moves outward along the jib",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 9.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()

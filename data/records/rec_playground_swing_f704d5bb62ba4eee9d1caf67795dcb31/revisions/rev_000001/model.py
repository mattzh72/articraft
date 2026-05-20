from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_seat_a_frame_swing")

    green = Material("powder_coated_green", rgba=(0.08, 0.42, 0.18, 1.0))
    steel = Material("galvanized_steel", rgba=(0.68, 0.70, 0.69, 1.0))
    dark_steel = Material("dark_pivot_bushing", rgba=(0.08, 0.08, 0.075, 1.0))
    rubber = Material("red_rubber_seat", rgba=(0.72, 0.05, 0.04, 1.0))

    model.material("powder_coated_green", rgba=green.rgba)
    model.material("galvanized_steel", rgba=steel.rgba)
    model.material("dark_pivot_bushing", rgba=dark_steel.rgba)
    model.material("red_rubber_seat", rgba=rubber.rgba)

    side_y = 0.86
    foot_x = 0.76
    foot_z = 0.06
    apex_z = 2.12
    leg_length = math.hypot(foot_x, apex_z - foot_z)
    leg_angle = math.atan2(foot_x, apex_z - foot_z)

    frame = model.part("frame")
    # Two A-shaped side frames: each side has a front and rear splayed tubular leg,
    # a ground skid, and a mid-height tie bar.  The small intersections overlap
    # within this single welded/static frame part.
    for side_index, y in enumerate((-side_y, side_y)):
        for leg_index, x_foot in enumerate((-foot_x, foot_x)):
            theta = leg_angle if x_foot < 0.0 else -leg_angle
            frame.visual(
                Cylinder(radius=0.045, length=leg_length),
                origin=Origin(
                    xyz=(x_foot / 2.0, y, (foot_z + apex_z) / 2.0),
                    rpy=(0.0, theta, 0.0),
                ),
                material=green,
                name=f"side_{side_index}_leg_{leg_index}",
            )
        frame.visual(
            Cylinder(radius=0.036, length=1.65),
            origin=Origin(xyz=(0.0, y, foot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=green,
            name=f"side_{side_index}_ground_bar",
        )
        frame.visual(
            Cylinder(radius=0.032, length=1.02),
            origin=Origin(xyz=(0.0, y, 0.88), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=green,
            name=f"side_{side_index}_tie_bar",
        )

    frame.visual(
        Cylinder(radius=0.055, length=2.05),
        origin=Origin(xyz=(0.0, 0.0, 2.15), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=green,
        name="top_crossbeam",
    )

    pivot_y = (-0.34, 0.34)
    pivot_z = 1.98
    for pivot_index, y in enumerate(pivot_y):
        # Clevis plates hang below the crossbeam with a clear central gap for the
        # swinging barrel.  They are welded into the static frame.
        for plate_index, offset in enumerate((-0.0575, 0.0575)):
            frame.visual(
                Box((0.080, 0.025, 0.160)),
                origin=Origin(xyz=(0.0, y + offset, 2.045)),
                material=dark_steel,
                name=f"pivot_{pivot_index}_plate_{plate_index}",
            )

    hanger_len = 1.43
    lower_eye_radius = 0.027
    rod_len = hanger_len - lower_eye_radius + 0.001
    top_limits = MotionLimits(
        effort=60.0,
        velocity=2.5,
        lower=-math.pi / 4.0,
        upper=math.pi / 4.0,
    )

    hangers = []
    for hanger_index, y in enumerate(pivot_y):
        hanger = model.part(f"hanger_{hanger_index}")
        hanger.visual(
            Cylinder(radius=0.038, length=0.090),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="top_barrel",
        )
        hanger.visual(
            Cylinder(radius=0.016, length=rod_len),
            origin=Origin(xyz=(0.0, 0.0, -rod_len / 2.0)),
            material=steel,
            name="rod",
        )
        hanger.visual(
            Cylinder(radius=lower_eye_radius, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -hanger_len), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="lower_eye",
        )
        hangers.append(hanger)

        model.articulation(
            f"pivot_{hanger_index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=hanger,
            origin=Origin(xyz=(0.0, y, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=top_limits,
            mimic=Mimic("pivot_0") if hanger_index == 1 else None,
        )

    seat = model.part("seat")
    # The seat frame is located at the lower eye of hanger_0.  The board center
    # is offset across the width so the two raised side ears line up under both
    # hanger rods.
    seat.visual(
        Box((0.56, 0.82, 0.065)),
        origin=Origin(xyz=(0.0, 0.34, -0.065)),
        material=rubber,
        name="seat_board",
    )
    seat.visual(
        Box((0.140, 0.026, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=dark_steel,
        name="side_eye_0",
    )
    seat.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_pin_0",
    )
    seat.visual(
        Box((0.140, 0.026, 0.060)),
        origin=Origin(xyz=(0.0, 0.68, -0.0025)),
        material=dark_steel,
        name="side_eye_1",
    )
    seat.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.68, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_pin_1",
    )

    model.articulation(
        "hanger_to_seat",
        ArticulationType.FIXED,
        parent=hangers[0],
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, -hanger_len)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hanger_0 = object_model.get_part("hanger_0")
    hanger_1 = object_model.get_part("hanger_1")
    seat = object_model.get_part("seat")
    pivot_0 = object_model.get_articulation("pivot_0")
    pivot_1 = object_model.get_articulation("pivot_1")

    for hanger_name, side_eye_name, side_pin_name in (
        ("hanger_0", "side_eye_0", "side_pin_0"),
        ("hanger_1", "side_eye_1", "side_pin_1"),
    ):
        ctx.allow_overlap(
            hanger_name,
            "seat",
            elem_a="lower_eye",
            elem_b=side_eye_name,
            reason="The lower eye is intentionally captured through the raised seat ear to read as a pinned hanger connection.",
        )
        ctx.allow_overlap(
            hanger_name,
            "seat",
            elem_a="lower_eye",
            elem_b=side_pin_name,
            reason="The simplified solid lower eye includes the pivot bore volume that is occupied by the seat pin.",
        )
        ctx.expect_overlap(
            hanger_name,
            "seat",
            axes="xyz",
            min_overlap=0.010,
            elem_a="lower_eye",
            elem_b=side_eye_name,
            name=f"{hanger_name} lower eye is captured by {side_eye_name}",
        )
        ctx.expect_overlap(
            hanger_name,
            "seat",
            axes="xyz",
            min_overlap=0.010,
            elem_a="lower_eye",
            elem_b=side_pin_name,
            name=f"{hanger_name} lower eye surrounds {side_pin_name}",
        )

    ctx.check(
        "top pivots have about plus-minus forty five degree travel",
        pivot_0.motion_limits is not None
        and pivot_1.motion_limits is not None
        and abs(pivot_0.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(pivot_0.motion_limits.upper - math.pi / 4.0) < 1e-6
        and abs(pivot_1.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(pivot_1.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )
    ctx.check(
        "second hanger follows first pivot",
        pivot_1.mimic is not None and pivot_1.mimic.joint == "pivot_0",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({pivot_0: math.pi / 4.0}):
        plus_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            hanger_0,
            seat,
            axes="xyz",
            min_overlap=0.008,
            elem_a="lower_eye",
            elem_b="side_eye_0",
            name="seat stays pinned at forward swing limit",
        )
    with ctx.pose({pivot_0: -math.pi / 4.0}):
        minus_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            hanger_1,
            seat,
            axes="xyz",
            min_overlap=0.008,
            elem_a="lower_eye",
            elem_b="side_eye_1",
            name="seat stays pinned at rear swing limit",
        )

    ctx.check(
        "seat swings through a vertical plane",
        rest_pos is not None
        and plus_pos is not None
        and minus_pos is not None
        and plus_pos[0] < rest_pos[0] - 0.9
        and minus_pos[0] > rest_pos[0] + 0.9
        and abs(plus_pos[1] - rest_pos[1]) < 0.01
        and abs(minus_pos[1] - rest_pos[1]) < 0.01,
        details=f"rest={rest_pos}, plus={plus_pos}, minus={minus_pos}",
    )

    return ctx.report()


object_model = build_object_model()

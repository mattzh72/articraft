from __future__ import annotations

from math import pi

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


SLIDE_TRAVEL = 0.78
LOCK_TRAVEL = 0.06


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_sliding_window")

    galvanized = model.material("dark_galvanized_steel", rgba=(0.23, 0.25, 0.25, 1.0))
    worn_steel = model.material("worn_bare_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    red = model.material("lockout_red", rgba=(0.85, 0.05, 0.02, 1.0))
    polycarbonate = model.material("smoke_polycarbonate", rgba=(0.55, 0.72, 0.86, 0.38))

    frame = model.part("frame")

    # Heavy welded outer frame around a real opening.
    frame.visual(Box((0.12, 0.20, 1.35)), origin=Origin(xyz=(-1.05, 0.0, 0.675)), material=galvanized, name="side_jamb_0")
    frame.visual(Box((0.12, 0.20, 1.35)), origin=Origin(xyz=(1.05, 0.0, 0.675)), material=galvanized, name="side_jamb_1")
    frame.visual(Box((2.10, 0.20, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=galvanized, name="bottom_sill")
    frame.visual(Box((2.10, 0.20, 0.12)), origin=Origin(xyz=(0.0, 0.0, 1.29)), material=galvanized, name="top_header")

    # Opposed lips form open upper/lower channel guides; the sash rides between
    # them with clearance rather than being represented as a buried block.
    frame.visual(Box((1.86, 0.035, 0.080)), origin=Origin(xyz=(0.0, -0.075, 0.160)), material=worn_steel, name="bottom_channel_front_lip")
    frame.visual(Box((1.86, 0.035, 0.080)), origin=Origin(xyz=(0.0, 0.075, 0.160)), material=worn_steel, name="bottom_channel_back_lip")
    frame.visual(Box((1.86, 0.040, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.130)), material=worn_steel, name="bottom_channel_web")
    frame.visual(Box((1.86, 0.035, 0.080)), origin=Origin(xyz=(0.0, -0.075, 1.190)), material=worn_steel, name="top_channel_front_lip")
    frame.visual(Box((1.86, 0.035, 0.080)), origin=Origin(xyz=(0.0, 0.075, 1.190)), material=worn_steel, name="top_channel_back_lip")
    frame.visual(Box((1.86, 0.040, 0.020)), origin=Origin(xyz=(0.0, 0.0, 1.220)), material=worn_steel, name="top_channel_web")

    # Over-travel stops bolted into the channels, with rubber faces toward the
    # sliding sash.  They are part of the fixed frame load path.
    frame.visual(Box((0.050, 0.090, 0.100)), origin=Origin(xyz=(-0.925, 0.0, 0.170)), material=safety_yellow, name="lower_left_stop")
    frame.visual(Box((0.010, 0.070, 0.080)), origin=Origin(xyz=(-0.895, 0.0, 0.170)), material=black_rubber, name="lower_left_stop_pad")
    frame.visual(Box((0.050, 0.090, 0.100)), origin=Origin(xyz=(0.925, 0.0, 0.170)), material=safety_yellow, name="lower_right_stop")
    frame.visual(Box((0.010, 0.070, 0.080)), origin=Origin(xyz=(0.895, 0.0, 0.170)), material=black_rubber, name="lower_right_stop_pad")
    frame.visual(Box((0.050, 0.090, 0.100)), origin=Origin(xyz=(-0.925, 0.0, 1.180)), material=safety_yellow, name="upper_left_stop")
    frame.visual(Box((0.010, 0.070, 0.080)), origin=Origin(xyz=(-0.895, 0.0, 1.180)), material=black_rubber, name="upper_left_stop_pad")
    frame.visual(Box((0.050, 0.090, 0.100)), origin=Origin(xyz=(0.925, 0.0, 1.180)), material=safety_yellow, name="upper_right_stop")
    frame.visual(Box((0.010, 0.070, 0.080)), origin=Origin(xyz=(0.895, 0.0, 1.180)), material=black_rubber, name="upper_right_stop_pad")

    # Front protective guard and spacer blocks.  Every guard bar terminates in
    # a guard rail, and the rail is stood off from the main frame by welded pads.
    frame.visual(Box((1.90, 0.035, 0.040)), origin=Origin(xyz=(0.0, -0.135, 0.160)), material=safety_yellow, name="guard_bottom_rail")
    frame.visual(Box((1.90, 0.035, 0.040)), origin=Origin(xyz=(0.0, -0.135, 1.190)), material=safety_yellow, name="guard_top_rail")
    frame.visual(Box((0.040, 0.035, 1.070)), origin=Origin(xyz=(-0.960, -0.135, 0.675)), material=safety_yellow, name="guard_side_0")
    frame.visual(Box((0.040, 0.035, 1.070)), origin=Origin(xyz=(0.960, -0.135, 0.675)), material=safety_yellow, name="guard_side_1")
    for i, x in enumerate((-0.72, -0.48, -0.24, 0.0, 0.24, 0.48, 0.72)):
        frame.visual(Box((0.020, 0.025, 1.030)), origin=Origin(xyz=(x, -0.135, 0.675)), material=safety_yellow, name=f"guard_bar_{i}")
    for i, (x, z) in enumerate(((-0.96, 0.16), (0.96, 0.16), (-0.96, 1.19), (0.96, 1.19))):
        frame.visual(Box((0.120, 0.070, 0.100)), origin=Origin(xyz=(x, -0.107, z)), material=galvanized, name=f"guard_spacer_{i}")

    # Diagonal gusset straps and corner plates show the load path at high-stress
    # frame/channel joints.
    gussets = (
        (-0.92, 0.245, 0.62, "lower_gusset_0"),
        (0.92, 0.245, -0.62, "lower_gusset_1"),
        (-0.92, 1.105, -0.62, "upper_gusset_0"),
        (0.92, 1.105, 0.62, "upper_gusset_1"),
    )
    for x, z, angle, name in gussets:
        frame.visual(
            Box((0.36, 0.014, 0.050)),
            origin=Origin(xyz=(x, -0.106, z), rpy=(0.0, angle, 0.0)),
            material=safety_yellow,
            name=name,
        )
    for i, (x, z) in enumerate(((-1.05, 0.18), (1.05, 0.18), (-1.05, 1.17), (1.05, 1.17))):
        frame.visual(Box((0.190, 0.014, 0.170)), origin=Origin(xyz=(x, -0.108, z)), material=worn_steel, name=f"corner_plate_{i}")

    # Visible bolt logic on plates, spacers, and stop blocks.
    bolt_points = [
        (-1.08, 0.18), (-1.02, 0.18), (1.02, 0.18), (1.08, 0.18),
        (-1.08, 1.17), (-1.02, 1.17), (1.02, 1.17), (1.08, 1.17),
        (-0.96, 0.16), (0.96, 0.16), (-0.96, 1.19), (0.96, 1.19),
        (-0.925, 0.17), (0.925, 0.17), (-0.925, 1.18), (0.925, 1.18),
    ]
    for i, (x, z) in enumerate(bolt_points):
        frame.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(x, -0.108, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bolt_head_{i}",
        )

    # Captive lockout pin bracket mounted from the front guard.  The moving pin
    # is a separate prismatic part, while the cheeks and stand-offs stay welded
    # to the fixed frame assembly.
    frame.visual(Box((0.160, 0.090, 0.030)), origin=Origin(xyz=(0.0, -0.175, 0.755)), material=safety_yellow, name="lock_standoff_lower")
    frame.visual(Box((0.160, 0.090, 0.030)), origin=Origin(xyz=(0.0, -0.175, 0.945)), material=safety_yellow, name="lock_standoff_upper")
    frame.visual(Box((0.018, 0.055, 0.240)), origin=Origin(xyz=(-0.045, -0.215, 0.850)), material=safety_yellow, name="lock_bracket_cheek_0")
    frame.visual(Box((0.018, 0.055, 0.240)), origin=Origin(xyz=(0.045, -0.215, 0.850)), material=safety_yellow, name="lock_bracket_cheek_1")
    frame.visual(Box((0.145, 0.018, 0.070)), origin=Origin(xyz=(0.0, -0.118, 0.850)), material=red, name="lockout_notice_plate")

    sash = model.part("sash")
    # The child frame origin is the sash center at the left-hand parked position.
    sash.visual(Box((0.950, 0.065, 0.080)), origin=Origin(xyz=(0.0, 0.0, 0.480)), material=galvanized, name="top_rail")
    sash.visual(Box((0.950, 0.065, 0.080)), origin=Origin(xyz=(0.0, 0.0, -0.480)), material=galvanized, name="bottom_rail")
    sash.visual(Box((0.070, 0.065, 1.000)), origin=Origin(xyz=(-0.455, 0.0, 0.0)), material=galvanized, name="side_stile_0")
    sash.visual(Box((0.070, 0.065, 1.000)), origin=Origin(xyz=(0.455, 0.0, 0.0)), material=galvanized, name="side_stile_1")
    sash.visual(Box((0.860, 0.018, 0.880)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=polycarbonate, name="polycarbonate_pane")
    sash.visual(Box((1.230, 0.022, 0.045)), origin=Origin(xyz=(0.0, -0.042, 0.0), rpy=(0.0, 0.69, 0.0)), material=safety_yellow, name="diagonal_brace")
    sash.visual(Box((0.160, 0.055, 0.030)), origin=Origin(xyz=(-0.320, 0.0, -0.520)), material=black_rubber, name="guide_shoe_0")
    sash.visual(Box((0.160, 0.055, 0.030)), origin=Origin(xyz=(0.320, 0.0, -0.520)), material=black_rubber, name="guide_shoe_1")
    sash.visual(Box((0.085, 0.015, 0.190)), origin=Origin(xyz=(0.425, -0.039, 0.0)), material=safety_yellow, name="pull_handle_base")
    sash.visual(Cylinder(radius=0.018, length=0.230), origin=Origin(xyz=(0.425, -0.075, 0.0), rpy=(0.0, 0.0, 0.0)), material=worn_steel, name="vertical_pull_handle")
    sash.visual(Box((0.040, 0.045, 0.026)), origin=Origin(xyz=(0.425, -0.057, 0.085)), material=worn_steel, name="handle_mount_0")
    sash.visual(Box((0.040, 0.045, 0.026)), origin=Origin(xyz=(0.425, -0.057, -0.085)), material=worn_steel, name="handle_mount_1")
    sash.visual(Box((0.105, 0.034, 0.105)), origin=Origin(xyz=(0.420, -0.047, 0.160)), material=red, name="lock_keeper_plate")
    for i, (x, z) in enumerate(((-0.455, 0.480), (0.455, 0.480), (-0.455, -0.480), (0.455, -0.480), (0.425, 0.070), (0.425, -0.070))):
        sash.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(x, -0.039, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"sash_bolt_{i}",
        )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.390, 0.0, 0.675)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )

    lock_pin = model.part("lock_pin")
    lock_pin.visual(Cylinder(radius=0.012, length=0.240), origin=Origin(), material=worn_steel, name="pin_shaft")
    lock_pin.visual(Cylinder(radius=0.014, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.145)), material=worn_steel, name="handle_neck")
    lock_pin.visual(Cylinder(radius=0.018, length=0.160), origin=Origin(xyz=(0.0, 0.0, 0.160), rpy=(0.0, pi / 2.0, 0.0)), material=red, name="tee_handle")
    lock_pin.visual(Box((0.030, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.016, -0.085)), material=red, name="lockout_flag")

    model.articulation(
        "frame_to_lock_pin",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lock_pin,
        origin=Origin(xyz=(0.0, -0.215, 0.850)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=LOCK_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    lock_pin = object_model.get_part("lock_pin")
    slide = object_model.get_articulation("frame_to_sash")
    lock = object_model.get_articulation("frame_to_lock_pin")

    ctx.allow_overlap(
        frame,
        lock_pin,
        elem_a="lock_standoff_lower",
        elem_b="pin_shaft",
        reason="The lockout pin shaft is intentionally captured through the lower bracket bushing proxy.",
    )
    ctx.allow_overlap(
        frame,
        lock_pin,
        elem_a="lock_standoff_upper",
        elem_b="pin_shaft",
        reason="The lockout pin shaft is intentionally captured through the upper bracket bushing proxy.",
    )

    # Rest pose: the sash is captured between front/back channel lips with
    # clearance, while still overlapping the guide length so it cannot read as
    # unsupported.
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        positive_elem="bottom_rail",
        negative_elem="bottom_channel_front_lip",
        min_gap=0.006,
        max_gap=0.040,
        name="sash clears lower front channel lip",
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        positive_elem="bottom_channel_back_lip",
        negative_elem="bottom_rail",
        min_gap=0.006,
        max_gap=0.040,
        name="sash clears lower back channel lip",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        elem_a="bottom_rail",
        elem_b="bottom_channel_front_lip",
        min_overlap=0.80,
        name="sash retained in bottom guide at rest",
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="x",
        positive_elem="side_stile_0",
        negative_elem="lower_left_stop_pad",
        min_gap=0.0,
        max_gap=0.035,
        name="left over-travel pad stops parked sash",
    )
    ctx.expect_overlap(
        lock_pin,
        frame,
        axes="z",
        elem_a="pin_shaft",
        elem_b="lock_standoff_lower",
        min_overlap=0.015,
        name="pin is retained by lower bushing",
    )
    ctx.expect_overlap(
        lock_pin,
        frame,
        axes="z",
        elem_a="pin_shaft",
        elem_b="lock_standoff_upper",
        min_overlap=0.015,
        name="pin is retained by upper bushing",
    )

    rest_pos = ctx.part_world_position(sash)
    rest_pin = ctx.part_world_position(lock_pin)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="top_rail",
            negative_elem="top_channel_front_lip",
            min_gap=0.006,
            max_gap=0.040,
            name="extended sash clears upper front channel lip",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="top_channel_back_lip",
            negative_elem="top_rail",
            min_gap=0.006,
            max_gap=0.040,
            name="extended sash clears upper back channel lip",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="top_rail",
            elem_b="top_channel_front_lip",
            min_overlap=0.80,
            name="sash retained in top guide at full travel",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="lower_right_stop_pad",
            negative_elem="side_stile_1",
            min_gap=0.0,
            max_gap=0.035,
            name="right over-travel pad stops extended sash",
        )
        extended_pos = ctx.part_world_position(sash)

    with ctx.pose({lock: LOCK_TRAVEL}):
        raised_pin = ctx.part_world_position(lock_pin)

    ctx.check(
        "sash translates horizontally in channels",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.70,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "lockout pin lifts from bracket",
        rest_pin is not None and raised_pin is not None and raised_pin[2] > rest_pin[2] + 0.045,
        details=f"rest={rest_pin}, raised={raised_pin}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    vinyl = model.material("warm_white_vinyl", rgba=(0.86, 0.84, 0.78, 1.0))
    track_shadow = model.material("dark_channel_shadow", rgba=(0.04, 0.045, 0.05, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.78, 0.95, 0.34))
    seal = model.material("black_rubber_gasket", rgba=(0.01, 0.012, 0.012, 1.0))

    frame = model.part("frame")

    # Rectangular fixed outer frame, sized like a domestic vertical sliding window.
    frame.visual(Box((1.20, 0.12, 0.080)), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=vinyl, name="bottom_rail")
    frame.visual(Box((1.20, 0.12, 0.080)), origin=Origin(xyz=(0.0, 0.0, 1.460)), material=vinyl, name="top_rail")
    frame.visual(Box((0.080, 0.12, 1.50)), origin=Origin(xyz=(-0.560, 0.0, 0.750)), material=vinyl, name="side_jamb_0")
    frame.visual(Box((0.080, 0.12, 1.50)), origin=Origin(xyz=(0.560, 0.0, 0.750)), material=vinyl, name="side_jamb_1")

    # The top and bottom horizontal guide channels are U-shaped tracks in the
    # front depth lane.  The sliding sash is clearanced between these lips.
    for z, prefix in ((0.108, "bottom"), (1.392, "top")):
        frame.visual(
            Box((1.08, 0.012, 0.055)),
            origin=Origin(xyz=(0.0, -0.049, z)),
            material=vinyl,
            name=f"{prefix}_front_lip",
        )
        frame.visual(
            Box((1.08, 0.012, 0.055)),
            origin=Origin(xyz=(0.0, -0.003, z)),
            material=vinyl,
            name=f"{prefix}_rear_lip",
        )
    frame.visual(
        Box((1.06, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, -0.026, 0.083)),
        material=track_shadow,
        name="bottom_channel_shadow",
    )
    frame.visual(
        Box((1.06, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, -0.026, 1.417)),
        material=track_shadow,
        name="top_channel_shadow",
    )

    # Fixed rear segment: a narrow, stationary framed pane on the left half.
    frame.visual(Box((0.040, 0.036, 1.34)), origin=Origin(xyz=(-0.500, 0.025, 0.750)), material=vinyl, name="fixed_outer_stile")
    frame.visual(Box((0.046, 0.036, 1.34)), origin=Origin(xyz=(-0.020, 0.025, 0.750)), material=vinyl, name="fixed_meeting_stile")
    frame.visual(Box((0.520, 0.036, 0.042)), origin=Origin(xyz=(-0.260, 0.025, 1.390)), material=vinyl, name="fixed_top_rail")
    frame.visual(Box((0.520, 0.036, 0.042)), origin=Origin(xyz=(-0.260, 0.025, 0.110)), material=vinyl, name="fixed_bottom_rail")
    frame.visual(Box((0.470, 0.006, 1.250)), origin=Origin(xyz=(-0.260, 0.025, 0.750)), material=glass, name="fixed_glass")
    frame.visual(Box((0.018, 0.010, 1.235)), origin=Origin(xyz=(-0.488, 0.018, 0.750)), material=seal, name="fixed_gasket_0")
    frame.visual(Box((0.018, 0.010, 1.235)), origin=Origin(xyz=(-0.032, 0.018, 0.750)), material=seal, name="fixed_gasket_1")
    frame.visual(Box((0.470, 0.010, 0.018)), origin=Origin(xyz=(-0.260, 0.018, 1.365)), material=seal, name="fixed_gasket_2")
    frame.visual(Box((0.470, 0.010, 0.018)), origin=Origin(xyz=(-0.260, 0.018, 0.135)), material=seal, name="fixed_gasket_3")

    sash = model.part("sash")
    # The sash part frame is its closed center point in the front track.  Its
    # geometry is local so prismatic motion translates the whole sash sideways.
    sash.visual(Box((0.045, 0.026, 1.260)), origin=Origin(xyz=(-0.2075, 0.0, 0.0)), material=vinyl, name="sash_stile_0")
    sash.visual(Box((0.045, 0.026, 1.260)), origin=Origin(xyz=(0.2075, 0.0, 0.0)), material=vinyl, name="sash_stile_1")
    sash.visual(Box((0.460, 0.026, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.6075)), material=vinyl, name="sash_top_rail")
    sash.visual(Box((0.460, 0.026, 0.045)), origin=Origin(xyz=(0.0, 0.0, -0.6075)), material=vinyl, name="sash_bottom_rail")
    sash.visual(Box((0.395, 0.006, 1.180)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=glass, name="sash_glass")
    sash.visual(Box((0.018, 0.010, 1.180)), origin=Origin(xyz=(-0.195, -0.006, 0.0)), material=seal, name="sash_gasket_0")
    sash.visual(Box((0.018, 0.010, 1.180)), origin=Origin(xyz=(0.195, -0.006, 0.0)), material=seal, name="sash_gasket_1")
    sash.visual(Box((0.405, 0.010, 0.018)), origin=Origin(xyz=(0.0, -0.006, 0.585)), material=seal, name="sash_gasket_2")
    sash.visual(Box((0.405, 0.010, 0.018)), origin=Origin(xyz=(0.0, -0.006, -0.585)), material=seal, name="sash_gasket_3")
    sash.visual(Box((0.028, 0.018, 0.520)), origin=Origin(xyz=(-0.185, -0.022, 0.0)), material=vinyl, name="finger_pull")

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.270, -0.022, 0.750)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.380),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    joint = object_model.get_articulation("frame_to_sash")

    ctx.check(
        "sash uses horizontal prismatic guide motion",
        joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (-1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_within(
        sash,
        frame,
        axes="xz",
        margin=0.0,
        name="closed sash sits inside the outer frame opening",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        elem_a="sash_bottom_rail",
        elem_b="bottom_channel_shadow",
        min_overlap=0.40,
        name="closed sash is seated in the lower horizontal channel",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        elem_a="sash_top_rail",
        elem_b="top_channel_shadow",
        min_overlap=0.40,
        name="closed sash is seated in the upper horizontal channel",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({joint: 0.38}):
        ctx.expect_within(
            sash,
            frame,
            axes="xz",
            margin=0.0,
            name="opened sash remains within the guide frame",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="sash_bottom_rail",
            elem_b="bottom_channel_shadow",
            min_overlap=0.40,
            name="opened sash remains captured by the lower channel",
        )
        opened_pos = ctx.part_world_position(sash)

    ctx.check(
        "upper joint limit slides sash left along the guide",
        rest_pos is not None
        and opened_pos is not None
        and opened_pos[0] < rest_pos[0] - 0.35
        and abs(opened_pos[1] - rest_pos[1]) < 1e-6
        and abs(opened_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, opened={opened_pos}",
    )

    return ctx.report()


object_model = build_object_model()

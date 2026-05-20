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
    model = ArticulatedObject(name="sunroof_cassette")

    frame = model.part("frame")
    # Front rail
    frame.visual(
        Box((0.04, 0.90, 0.04)),
        origin=Origin(xyz=(0.90, 0.0, 0.02)),
        name="front_rail",
    )
    # Rear rail
    frame.visual(
        Box((0.04, 0.90, 0.04)),
        origin=Origin(xyz=(-0.90, 0.0, 0.02)),
        name="rear_rail",
    )
    # Left rail
    frame.visual(
        Box((1.76, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.43, 0.02)),
        name="left_rail",
    )
    # Right rail
    frame.visual(
        Box((1.76, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.43, 0.02)),
        name="right_rail",
    )
    # Left track
    frame.visual(
        Box((1.76, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.38, 0.01)),
        name="left_track",
    )
    # Right track
    frame.visual(
        Box((1.76, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, -0.38, 0.01)),
        name="right_track",
    )

    carriage = model.part("carriage")
    # Front bar under the hinge
    carriage.visual(
        Box((0.02, 0.74, 0.01)),
        origin=Origin(xyz=(0.87, 0.0, 0.025)),
        name="front_bar",
    )
    # Left slider
    carriage.visual(
        Box((0.12, 0.02, 0.01)),
        origin=Origin(xyz=(0.82, 0.38, 0.025)),
        name="left_slider",
    )
    # Right slider
    carriage.visual(
        Box((0.12, 0.02, 0.01)),
        origin=Origin(xyz=(0.82, -0.38, 0.025)),
        name="right_slider",
    )

    model.articulation(
        "slide_joint",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        axis=(1.0, 0.0, 0.0),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.85, upper=0.0),
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        Box((0.85, 0.80, 0.01)),
        origin=Origin(xyz=(-0.425, 0.0, 0.005)),
        name="glass",
    )

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=glass_panel,
        axis=(0.0, 1.0, 0.0),
        origin=Origin(xyz=(0.87, 0.0, 0.03)),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.15),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    glass = object_model.get_part("glass_panel")
    slide_joint = object_model.get_articulation("slide_joint")
    tilt_hinge = object_model.get_articulation("tilt_hinge")

    # At rest, glass is flush with the frame
    ctx.expect_within(
        glass,
        frame,
        axes="xy",
        margin=0.0,
        name="glass fits inside the frame opening",
    )

    # Sliders should sit on tracks
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="left_slider",
        elem_b="left_track",
        min_overlap=0.0,
        name="left slider sits on left track",
    )

    # Test open slide pose
    with ctx.pose({slide_joint: -0.85}):
        ctx.expect_within(
            glass,
            frame,
            axes="xy",
            margin=0.0,
            name="glass remains inside the frame when fully slid back",
        )

    # Test tilt pose
    with ctx.pose({tilt_hinge: 0.15}):
        # Check that the rear of the glass has moved up
        glass_pos = ctx.part_world_aabb(glass)
        frame_pos = ctx.part_world_aabb(frame)
        if glass_pos and frame_pos:
            ctx.check(
                "glass tilts up",
                glass_pos[1][2] > frame_pos[1][2] + 0.02,
                details="Glass top should be significantly above the frame when tilted.",
            )

    return ctx.report()

object_model = build_object_model()
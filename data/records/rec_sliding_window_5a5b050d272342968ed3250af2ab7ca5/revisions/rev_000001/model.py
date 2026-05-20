from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")
    
    frame_mat = Material("frame_mat", rgba=(0.9, 0.9, 0.9, 1.0))
    glass_mat = Material("glass_mat", rgba=(0.6, 0.8, 0.9, 0.4))
    metal_mat = Material("metal_mat", rgba=(0.3, 0.3, 0.3, 1.0))

    # Outer Frame
    frame = model.part("frame")
    
    # Sill and Head
    frame.visual(Box((1.5, 0.12, 0.05)), origin=Origin(xyz=(0, 0, -0.575)), material=frame_mat, name="sill")
    frame.visual(Box((1.5, 0.12, 0.05)), origin=Origin(xyz=(0, 0, 0.575)), material=frame_mat, name="head")
    
    # Jambs
    frame.visual(Box((0.05, 0.12, 1.1)), origin=Origin(xyz=(-0.725, 0, 0)), material=frame_mat, name="left_jamb")
    frame.visual(Box((0.05, 0.12, 1.1)), origin=Origin(xyz=(0.725, 0, 0)), material=frame_mat, name="right_jamb")
    
    # Tracks (bottom)
    frame.visual(Box((1.4, 0.01, 0.01)), origin=Origin(xyz=(0, -0.02, -0.545)), material=frame_mat, name="track1")
    frame.visual(Box((1.4, 0.01, 0.01)), origin=Origin(xyz=(0, 0.02, -0.545)), material=frame_mat, name="track2")
    
    # Guides (top)
    frame.visual(Box((1.4, 0.01, 0.015)), origin=Origin(xyz=(0, -0.04, 0.5425)), material=frame_mat, name="guide1")
    frame.visual(Box((1.4, 0.01, 0.015)), origin=Origin(xyz=(0, 0.0, 0.5425)), material=frame_mat, name="guide2")
    frame.visual(Box((1.4, 0.01, 0.015)), origin=Origin(xyz=(0, 0.04, 0.5425)), material=frame_mat, name="guide3")

    # Sash dimensions
    s_width = 0.74
    s_height = 1.085
    s_thick = 0.028
    f_width = 0.06
    
    g_width = s_width - 2 * f_width
    g_height = s_height - 2 * f_width
    g_thick = 0.005

    # Fixed sash (left side, outer track at Y=-0.02)
    # X = -0.33, Y = -0.02, Z = 0.003
    fixed_sash = model.part("fixed_sash")
    
    # Sash frame
    fixed_sash.visual(Box((f_width, s_thick, s_height)), origin=Origin(xyz=(-s_width/2 + f_width/2, 0, 0)), material=frame_mat, name="left_stile")
    fixed_sash.visual(Box((f_width, s_thick, s_height)), origin=Origin(xyz=(s_width/2 - f_width/2, 0, 0)), material=frame_mat, name="right_stile")
    fixed_sash.visual(Box((g_width, s_thick, f_width)), origin=Origin(xyz=(0, 0, s_height/2 - f_width/2)), material=frame_mat, name="top_rail")
    fixed_sash.visual(Box((g_width, s_thick, f_width)), origin=Origin(xyz=(0, 0, -s_height/2 + f_width/2)), material=frame_mat, name="bottom_rail")
    
    # Glass
    fixed_sash.visual(
        Box((g_width, g_thick, g_height)),
        material=glass_mat,
        name="glass",
    )
    
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_sash,
        origin=Origin(xyz=(-0.33, -0.02, 0.003)),
    )

    # Sliding sash (right side, inner track at Y=0.02)
    # X = 0.33, Y = 0.02, Z = 0.003
    sliding_sash = model.part("sliding_sash")
    
    # Sash frame
    sliding_sash.visual(Box((f_width, s_thick, s_height)), origin=Origin(xyz=(-s_width/2 + f_width/2, 0, 0)), material=frame_mat, name="left_stile")
    sliding_sash.visual(Box((f_width, s_thick, s_height)), origin=Origin(xyz=(s_width/2 - f_width/2, 0, 0)), material=frame_mat, name="right_stile")
    sliding_sash.visual(Box((g_width, s_thick, f_width)), origin=Origin(xyz=(0, 0, s_height/2 - f_width/2)), material=frame_mat, name="top_rail")
    sliding_sash.visual(Box((g_width, s_thick, f_width)), origin=Origin(xyz=(0, 0, -s_height/2 + f_width/2)), material=frame_mat, name="bottom_rail")

    # Glass
    sliding_sash.visual(
        Box((g_width, g_thick, g_height)),
        material=glass_mat,
        name="glass",
    )
    
    # Handle on the inner face (+Y)
    # Left stile center is at local X = -0.37 + 0.03 = -0.34
    # Inner face is at Y = 0.014
    sliding_sash.visual(
        Box((0.02, 0.02, 0.15)),
        origin=Origin(xyz=(-0.34, 0.024, 0.0)),
        material=metal_mat,
        name="handle",
    )
    
    # Lock on the left stile, inner face
    sliding_sash.visual(
        Box((0.02, 0.015, 0.06)),
        origin=Origin(xyz=(-0.34, 0.0215, -0.1)),
        material=metal_mat,
        name="lock",
    )

    # Prismatic joint for sliding sash
    # Slides to the left (-X) to open. Max opening = 0.66m
    model.articulation(
        "slide_joint",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.33, 0.02, 0.003)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.66),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    slide_joint = object_model.get_articulation("slide_joint")

    # The sashes overlap slightly in Z with the frame's channels, but they are nested inside.
    # The `expect_within` checks will verify they are contained within the outer frame bounds.
    ctx.expect_within(fixed_sash, frame, axes="z", margin=0.0)
    ctx.expect_within(sliding_sash, frame, axes="z", margin=0.0)
    
    # Check that sashes overlap correctly when closed
    ctx.expect_overlap(sliding_sash, fixed_sash, axes="x", min_overlap=0.05)
    
    # Test open pose
    with ctx.pose({slide_joint: 0.66}):
        # In fully open position, it should still be within the frame in X
        ctx.expect_within(sliding_sash, frame, axes="x", margin=0.002)
        # And it should significantly overlap the fixed sash
        ctx.expect_overlap(sliding_sash, fixed_sash, axes="x", min_overlap=0.6)

    return ctx.report()


object_model = build_object_model()
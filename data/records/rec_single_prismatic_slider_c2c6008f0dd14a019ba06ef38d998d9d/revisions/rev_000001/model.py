from __future__ import annotations

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
    model = ArticulatedObject(name="vertical_lift_slider")

    # Materials
    model.material("frame_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("carriage_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("guide_rod_chrome", rgba=(0.85, 0.88, 0.92, 1.0))
    model.material("stop_yellow", rgba=(0.95, 0.80, 0.20, 1.0))
    model.material("travel_red", rgba=(0.90, 0.15, 0.15, 1.0))
    model.material("travel_green", rgba=(0.15, 0.80, 0.25, 1.0))

    # --- Fixed Frame (base) ---
    frame = model.part("frame")

    # Base platform - wide and stable
    frame.visual(
        Box((0.30, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="frame_steel",
        name="base_platform",
    )

    # Upright columns (left and right) - positioned to create channel for carriage
    # Carriage is 0.20 wide (x), so uprights at x=-0.12 and x=+0.12 with width 0.04
    # means they span x=[-0.14, -0.10] and x=[+0.10, +0.14]
    # Left upright
    frame.visual(
        Box((0.04, 0.20, 0.50)),
        origin=Origin(xyz=(-0.12, 0.0, 0.25 + 0.0125)),
        material="frame_steel",
        name="left_upright",
    )

    # Right upright
    frame.visual(
        Box((0.04, 0.20, 0.50)),
        origin=Origin(xyz=(0.12, 0.0, 0.25 + 0.0125)),
        material="frame_steel",
        name="right_upright",
    )

    # Top crossbar connecting the uprights
    frame.visual(
        Box((0.28, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.50 + 0.0125 + 0.02)),
        material="frame_steel",
        name="top_crossbar",
    )

    # Two guide rods (vertical cylindrical rods) - positioned inside the uprights
    # Guide rods at x = -0.06 and +0.06 (within the 0.20 carriage width)
    # Left guide rod
    frame.visual(
        Cylinder(radius=0.012, length=0.45),
        origin=Origin(xyz=(-0.06, 0.0, 0.0125 + 0.225)),
        material="guide_rod_chrome",
        name="left_guide_rod",
    )

    # Right guide rod
    frame.visual(
        Cylinder(radius=0.012, length=0.45),
        origin=Origin(xyz=(0.06, 0.0, 0.0125 + 0.225)),
        material="guide_rod_chrome",
        name="right_guide_rod",
    )

    # Bottom stop - carriage rests on it at bottom position
    # At q=0, carriage block extends from z=0.06 to z=0.12
    # Bottom stop at z=0.045, extends from z=0.0375 to z=0.0525
    # Small gap of 0.06 - 0.0525 = 0.0075 between them
    frame.visual(
        Box((0.26, 0.08, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="stop_yellow",
        name="bottom_stop",
    )

    # Top stop - carriage hits it at top position
    # At q=0.30, carriage_block extends to z=0.51 (center 0.48 + 0.03)
    # Top stop at z=0.525, extends from z=0.5175 to z=0.5325
    # Gap of 0.5175 - 0.51 = 0.0075 between them
    frame.visual(
        Box((0.26, 0.08, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material="stop_yellow",
        name="top_stop",
    )

    # Travel range markers on the front of the left upright (facing +y)
    # Lower limit mark (green - bottom position)
    # At q=0, carriage_block and indicator center at z=0.18 (0.09 part frame + 0.09 relative)
    # Put mark at z=0.18 to align with carriage indicator at bottom
    frame.visual(
        Box((0.02, 0.005, 0.04)),
        origin=Origin(xyz=(-0.14, 0.102, 0.18)),
        material="travel_green",
        name="lower_travel_mark",
    )

    # Upper limit mark (red - top position)
    # At q=0.30, carriage_block and indicator center at z=0.48 (0.39 part frame + 0.09 relative)
    # Put mark at z=0.48 to align with carriage indicator at top
    frame.visual(
        Box((0.02, 0.005, 0.04)),
        origin=Origin(xyz=(-0.14, 0.102, 0.48)),
        material="travel_red",
        name="upper_travel_mark",
    )

    # --- Sliding Carriage (moving part) ---
    carriage = model.part("carriage")

    # Main carriage block - slides between the uprights
    # Width: 0.20 (fits between left_upright at x=-0.10 and right_upright at x=+0.10)
    # Depth: 0.16 (less than upright depth of 0.20)
    # Height: 0.06
    # Carriage part frame origin at z=0.09 (center of block is at z=0.09 relative)
    # At q=0: block center at z=0.09, extends from z=0.06 to z=0.12
    # At q=0.30: block center at z=0.39, extends from z=0.36 to z=0.42
    carriage.visual(
        Box((0.20, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material="carriage_aluminum",
        name="carriage_block",
    )

    # Colored travel indicator on the front face of carriage (facing +y)
    # This aligns with the travel marks on the left upright
    # Positioned at same z as carriage center (0.09 relative to part frame)
    carriage.visual(
        Box((0.04, 0.005, 0.04)),
        origin=Origin(xyz=(-0.06, 0.0825, 0.09)),
        material="travel_red",
        name="carriage_travel_indicator",
    )

    # --- Prismatic Joint ---
    # Joint origin at the carriage part frame position at bottom (z=0.09)
    # At q=0: carriage part frame at z=0.09 (aligned with joint)
    # At q=0.30: carriage part frame at z=0.39
    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,      # At bottom
            upper=0.30,     # Travel distance
            effort=100.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift_joint = object_model.get_articulation("frame_to_carriage")

    # Allow overlaps between carriage and guide rods (intentional sliding contact)
    ctx.allow_overlap(
        "carriage",
        "frame",
        elem_a="carriage_block",
        elem_b="left_guide_rod",
        reason="Carriage slides on left guide rod - intentional contact for sliding mechanism",
    )
    ctx.allow_overlap(
        "carriage",
        "frame",
        elem_a="carriage_block",
        elem_b="right_guide_rod",
        reason="Carriage slides on right guide rod - intentional contact for sliding mechanism",
    )

    # --- Rest Position (Bottom, q=0) ---
    with ctx.pose({lift_joint: 0.0}):
        # Carriage part frame should be at z=0.09
        carriage_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage_at_bottom",
            carriage_pos is not None and abs(carriage_pos[2] - 0.09) < 0.001,
            details=f"carriage_z={carriage_pos[2] if carriage_pos else None}, expected=0.09",
        )

        # Carriage should be centered on the frame in xy
        ctx.expect_within(
            carriage,
            frame,
            axes="xy",
            margin=0.02,
            name="carriage_centered_on_frame",
        )

        # Check clearance from bottom stop
        # bottom_stop extends to z=0.0525, carriage_block starts at z=0.06
        # gap = 0.06 - 0.0525 = 0.0075
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="carriage_block",
            negative_elem="bottom_stop",
            min_gap=0.005,
            name="carriage_above_bottom_stop",
        )

    # --- Extended Position (Top, q=0.30) ---
    with ctx.pose({lift_joint: 0.30}):
        # Carriage part frame should be at z=0.39
        carriage_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage_at_top",
            carriage_pos is not None and abs(carriage_pos[2] - 0.39) < 0.001,
            details=f"carriage_z={carriage_pos[2] if carriage_pos else None}, expected=0.39",
        )

        # Check clearance from top stop
        # carriage_block extends to z=0.42, top_stop starts at z=0.5125
        # gap = 0.5125 - 0.42 = 0.0925
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_stop",
            negative_elem="carriage_block",
            min_gap=0.005,
            name="carriage_below_top_stop",
        )

    # --- Verify vertical travel ---
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift_joint: 0.30}):
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage_moves_upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.25,
        details=f"rest_z={rest_pos[2] if rest_pos else None}, extended_z={extended_pos[2] if extended_pos else None}",
    )

    # --- Check carriage is supported/guided by guide rods ---
    ctx.expect_contact(
        carriage,
        frame,
        elem_a="carriage_block",
        elem_b="left_guide_rod",
        contact_tol=0.025,
        name="carriage_near_left_guide_rod",
    )

    ctx.expect_contact(
        carriage,
        frame,
        elem_a="carriage_block",
        elem_b="right_guide_rod",
        contact_tol=0.025,
        name="carriage_near_right_guide_rod",
    )

    # --- Verify travel marks are visible ---
    lower_mark = frame.get_visual("lower_travel_mark")
    upper_mark = frame.get_visual("upper_travel_mark")
    ctx.check(
        "travel_marks_exist",
        lower_mark is not None and upper_mark is not None,
        details="Lower and upper travel marks should exist",
    )

    # --- Check that carriage travel indicator exists ---
    indicator = carriage.get_visual("carriage_travel_indicator")
    ctx.check(
        "carriage_has_travel_indicator",
        indicator is not None,
        details="Carriage should have a colored travel indicator",
    )

    return ctx.report()


object_model = build_object_model()

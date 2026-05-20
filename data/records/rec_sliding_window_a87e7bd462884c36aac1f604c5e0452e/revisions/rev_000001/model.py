from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WINDOW_WIDTH = 1.405
WINDOW_HEIGHT = 1.08
FRAME_DEPTH = 0.14
JAMB_THICKNESS = 0.07
HEAD_HEIGHT = 0.075
SILL_HEIGHT = 0.105

OPENING_WIDTH = WINDOW_WIDTH - 2.0 * JAMB_THICKNESS
OPENING_HEIGHT = WINDOW_HEIGHT - HEAD_HEIGHT - SILL_HEIGHT

FRONT_TRACK_Y = -0.023
REAR_TRACK_Y = 0.023

SASH_WIDTH = 0.69
SASH_HEIGHT = 0.88
SASH_DEPTH = 0.032
SASH_MEETING_STILE = 0.085
SASH_LOCK_STILE = 0.055
SASH_RAIL_HEIGHT = 0.055
GUIDE_SHOE_PROTRUSION = 0.01
GUIDE_CHANNEL_DEPTH = 0.042

FIXED_WIDTH = 0.69
FIXED_HEIGHT = 0.88
FIXED_DEPTH = 0.032
FIXED_LEFT_STILE = 0.055
FIXED_MEETING_STILE = 0.075
FIXED_RAIL_HEIGHT = 0.055

SASH_CENTER_Z = SILL_HEIGHT + GUIDE_SHOE_PROTRUSION + SASH_HEIGHT / 2.0
SASH_CLOSED_CENTER_X = OPENING_WIDTH / 2.0 - SASH_WIDTH / 2.0
SASH_OPEN_TRAVEL = OPENING_WIDTH - SASH_WIDTH
FIXED_CENTER_X = -SASH_CLOSED_CENTER_X


def _add_box(part, name, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder_y(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_slider_window")

    frame_metal = model.material("frame_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.43, 0.45, 0.48, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    wear_strip = model.material("wear_strip", rgba=(0.88, 0.88, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.66, 0.78, 0.84, 0.35))

    frame = model.part("frame")

    # Outer welded perimeter frame.
    _add_box(
        frame,
        "left_jamb",
        (JAMB_THICKNESS, FRAME_DEPTH, WINDOW_HEIGHT),
        (-WINDOW_WIDTH / 2.0 + JAMB_THICKNESS / 2.0, 0.0, WINDOW_HEIGHT / 2.0),
        frame_metal,
    )
    _add_box(
        frame,
        "right_jamb",
        (JAMB_THICKNESS, FRAME_DEPTH, WINDOW_HEIGHT),
        (WINDOW_WIDTH / 2.0 - JAMB_THICKNESS / 2.0, 0.0, WINDOW_HEIGHT / 2.0),
        frame_metal,
    )
    _add_box(
        frame,
        "head",
        (WINDOW_WIDTH, FRAME_DEPTH, HEAD_HEIGHT),
        (0.0, 0.0, WINDOW_HEIGHT - HEAD_HEIGHT / 2.0),
        frame_metal,
    )
    _add_box(
        frame,
        "sill",
        (WINDOW_WIDTH, FRAME_DEPTH, SILL_HEIGHT),
        (0.0, 0.0, SILL_HEIGHT / 2.0),
        frame_metal,
    )

    # Front sliding track with replaceable wear strip and tall guide ribs.
    _add_box(
        frame,
        "front_sill_wear_strip",
        (OPENING_WIDTH, 0.022, 0.012),
        (0.0, FRONT_TRACK_Y, SILL_HEIGHT - 0.006),
        wear_strip,
    )
    _add_box(
        frame,
        "front_outer_sill_guide",
        (OPENING_WIDTH, 0.010, 0.030),
        (0.0, -0.049, SILL_HEIGHT + 0.015),
        trim_metal,
    )
    _add_box(
        frame,
        "front_inner_sill_guide",
        (OPENING_WIDTH, 0.010, 0.030),
        (0.0, 0.003, SILL_HEIGHT + 0.015),
        trim_metal,
    )
    _add_box(
        frame,
        "front_outer_head_guide",
        (OPENING_WIDTH, 0.010, 0.030),
        (0.0, -0.049, WINDOW_HEIGHT - HEAD_HEIGHT - 0.015),
        trim_metal,
    )
    _add_box(
        frame,
        "front_inner_head_guide",
        (OPENING_WIDTH, 0.010, 0.030),
        (0.0, 0.003, WINDOW_HEIGHT - HEAD_HEIGHT - 0.015),
        trim_metal,
    )
    _add_box(
        frame,
        "left_stop_liner",
        (0.020, GUIDE_CHANNEL_DEPTH, OPENING_HEIGHT),
        (-OPENING_WIDTH / 2.0 - 0.010, FRONT_TRACK_Y, SILL_HEIGHT + OPENING_HEIGHT / 2.0),
        trim_metal,
    )
    _add_box(
        frame,
        "right_stop_liner",
        (0.020, GUIDE_CHANNEL_DEPTH, OPENING_HEIGHT),
        (OPENING_WIDTH / 2.0 + 0.010, FRONT_TRACK_Y, SILL_HEIGHT + OPENING_HEIGHT / 2.0),
        trim_metal,
    )

    # Fixed rear-lite assembly captured in the rear pocket of the same frame.
    fixed_left_x = FIXED_CENTER_X - FIXED_WIDTH / 2.0 + FIXED_LEFT_STILE / 2.0
    fixed_right_x = FIXED_CENTER_X + FIXED_WIDTH / 2.0 - FIXED_MEETING_STILE / 2.0
    fixed_inner_left = FIXED_CENTER_X - FIXED_WIDTH / 2.0 + FIXED_LEFT_STILE
    fixed_inner_right = FIXED_CENTER_X + FIXED_WIDTH / 2.0 - FIXED_MEETING_STILE
    fixed_rail_width = fixed_inner_right - fixed_inner_left
    fixed_rail_center_x = (fixed_inner_left + fixed_inner_right) / 2.0

    _add_box(
        frame,
        "fixed_left_stile",
        (FIXED_LEFT_STILE, FIXED_DEPTH, FIXED_HEIGHT),
        (fixed_left_x, REAR_TRACK_Y, SASH_CENTER_Z),
        trim_metal,
    )
    _add_box(
        frame,
        "fixed_right_meeting_stile",
        (FIXED_MEETING_STILE, FIXED_DEPTH, FIXED_HEIGHT),
        (fixed_right_x, REAR_TRACK_Y, SASH_CENTER_Z),
        trim_metal,
    )
    _add_box(
        frame,
        "fixed_top_rail",
        (fixed_rail_width, FIXED_DEPTH, FIXED_RAIL_HEIGHT),
        (
            fixed_rail_center_x,
            REAR_TRACK_Y,
            SASH_CENTER_Z + FIXED_HEIGHT / 2.0 - FIXED_RAIL_HEIGHT / 2.0,
        ),
        trim_metal,
    )
    _add_box(
        frame,
        "fixed_bottom_rail",
        (fixed_rail_width, FIXED_DEPTH, FIXED_RAIL_HEIGHT),
        (
            fixed_rail_center_x,
            REAR_TRACK_Y,
            SASH_CENTER_Z - FIXED_HEIGHT / 2.0 + FIXED_RAIL_HEIGHT / 2.0,
        ),
        trim_metal,
    )
    _add_box(
        frame,
        "fixed_glass",
        (fixed_rail_width + 0.010, 0.006, FIXED_HEIGHT - 2.0 * FIXED_RAIL_HEIGHT + 0.010),
        (fixed_rail_center_x, REAR_TRACK_Y, SASH_CENTER_Z),
        glass,
    )

    # Field-service access hardware on the sill nose.
    _add_box(
        frame,
        "service_cover",
        (0.54, 0.006, 0.055),
        (0.34, -FRAME_DEPTH / 2.0 + 0.003, 0.055),
        trim_metal,
    )
    _add_cylinder_y(
        frame,
        "service_cover_fastener_left",
        0.009,
        0.008,
        (0.15, -FRAME_DEPTH / 2.0 + 0.010, 0.055),
        hardware_dark,
    )
    _add_cylinder_y(
        frame,
        "service_cover_fastener_right",
        0.009,
        0.008,
        (0.53, -FRAME_DEPTH / 2.0 + 0.010, 0.055),
        hardware_dark,
    )

    frame.inertial = Inertial.from_geometry(
        Box((WINDOW_WIDTH, FRAME_DEPTH, WINDOW_HEIGHT)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, WINDOW_HEIGHT / 2.0)),
    )

    sash = model.part("sliding_sash")

    sash_left_x = -SASH_WIDTH / 2.0 + SASH_MEETING_STILE / 2.0
    sash_right_x = SASH_WIDTH / 2.0 - SASH_LOCK_STILE / 2.0
    sash_inner_left = -SASH_WIDTH / 2.0 + SASH_MEETING_STILE
    sash_inner_right = SASH_WIDTH / 2.0 - SASH_LOCK_STILE
    sash_rail_width = sash_inner_right - sash_inner_left
    sash_rail_center_x = (sash_inner_left + sash_inner_right) / 2.0

    _add_box(
        sash,
        "sash_meeting_stile",
        (SASH_MEETING_STILE, SASH_DEPTH, SASH_HEIGHT),
        (sash_left_x, 0.0, 0.0),
        frame_metal,
    )
    _add_box(
        sash,
        "sash_lock_stile",
        (SASH_LOCK_STILE, SASH_DEPTH, SASH_HEIGHT),
        (sash_right_x, 0.0, 0.0),
        frame_metal,
    )
    _add_box(
        sash,
        "sash_top_rail",
        (sash_rail_width, SASH_DEPTH, SASH_RAIL_HEIGHT),
        (sash_rail_center_x, 0.0, SASH_HEIGHT / 2.0 - SASH_RAIL_HEIGHT / 2.0),
        frame_metal,
    )
    _add_box(
        sash,
        "sash_bottom_rail",
        (sash_rail_width, SASH_DEPTH, SASH_RAIL_HEIGHT),
        (sash_rail_center_x, 0.0, -SASH_HEIGHT / 2.0 + SASH_RAIL_HEIGHT / 2.0),
        frame_metal,
    )
    _add_box(
        sash,
        "sash_glass",
        (sash_rail_width + 0.010, 0.006, SASH_HEIGHT - 2.0 * SASH_RAIL_HEIGHT + 0.010),
        (sash_rail_center_x, 0.0, 0.0),
        glass,
    )

    # Replaceable guide shoes: proud enough to stay captured, but still serviceable.
    _add_box(
        sash,
        "bottom_left_shoe",
        (0.080, GUIDE_CHANNEL_DEPTH, GUIDE_SHOE_PROTRUSION),
        (-0.175, 0.0, -SASH_HEIGHT / 2.0 - GUIDE_SHOE_PROTRUSION / 2.0),
        wear_strip,
    )
    _add_box(
        sash,
        "bottom_right_shoe",
        (0.080, GUIDE_CHANNEL_DEPTH, GUIDE_SHOE_PROTRUSION),
        (0.185, 0.0, -SASH_HEIGHT / 2.0 - GUIDE_SHOE_PROTRUSION / 2.0),
        wear_strip,
    )
    _add_box(
        sash,
        "top_left_guide",
        (0.080, GUIDE_CHANNEL_DEPTH, GUIDE_SHOE_PROTRUSION),
        (-0.175, 0.0, SASH_HEIGHT / 2.0 + GUIDE_SHOE_PROTRUSION / 2.0),
        wear_strip,
    )
    _add_box(
        sash,
        "top_right_guide",
        (0.080, GUIDE_CHANNEL_DEPTH, GUIDE_SHOE_PROTRUSION),
        (0.185, 0.0, SASH_HEIGHT / 2.0 + GUIDE_SHOE_PROTRUSION / 2.0),
        wear_strip,
    )

    # Chunky pull handle mounted on the meeting stile with visible standoffs.
    _add_box(
        sash,
        "handle_upper_standoff",
        (0.016, 0.020, 0.100),
        (-0.292, -0.026, 0.120),
        hardware_dark,
    )
    _add_box(
        sash,
        "handle_lower_standoff",
        (0.016, 0.020, 0.100),
        (-0.292, -0.026, -0.120),
        hardware_dark,
    )
    _add_box(
        sash,
        "handle_grip",
        (0.024, 0.018, 0.340),
        (-0.292, -0.045, 0.0),
        hardware_dark,
    )
    _add_box(
        sash,
        "latch_housing",
        (0.028, 0.020, 0.090),
        (-0.292, -0.025, 0.0),
        hardware_dark,
    )

    sash.inertial = Inertial.from_geometry(
        Box((SASH_WIDTH, GUIDE_CHANNEL_DEPTH, OPENING_HEIGHT)),
        mass=18.0,
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(SASH_CLOSED_CENTER_X, FRONT_TRACK_Y, SASH_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.60,
            lower=0.0,
            upper=SASH_OPEN_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sliding_sash")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    axis = tuple(round(v, 6) for v in slide.axis)
    ctx.check(
        "slide axis points left along the sill track",
        axis == (-1.0, 0.0, 0.0),
        details=f"expected (-1, 0, 0), got {slide.axis}",
    )
    limits = slide.motion_limits
    ctx.check(
        "slide travel matches the framed opening",
        limits is not None and limits.lower == 0.0 and abs(limits.upper - SASH_OPEN_TRAVEL) < 1e-9,
        details=f"expected travel {SASH_OPEN_TRAVEL:.6f}, got {None if limits is None else limits.upper}",
    )

    ctx.expect_contact(
        sash,
        frame,
        elem_a="bottom_left_shoe",
        elem_b="front_sill_wear_strip",
        name="bottom left shoe sits on the service wear strip",
    )
    ctx.expect_contact(
        sash,
        frame,
        elem_a="bottom_right_shoe",
        elem_b="front_sill_wear_strip",
        name="bottom right shoe sits on the service wear strip",
    )
    ctx.expect_contact(
        frame,
        sash,
        elem_a="front_inner_sill_guide",
        elem_b="bottom_left_shoe",
        name="inner guide face retains the left shoe",
    )
    ctx.expect_contact(
        sash,
        frame,
        elem_a="bottom_left_shoe",
        elem_b="front_outer_sill_guide",
        name="outer guide face retains the left shoe",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="right_stop_liner",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed sash lands on the right stop",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            elem_a="sash_meeting_stile",
            elem_b="fixed_right_meeting_stile",
            min_overlap=0.03,
            name="closed meeting stiles keep a believable interlock overlap",
        )

    with ctx.pose({slide: SASH_OPEN_TRAVEL}):
        ctx.expect_gap(
            sash,
            frame,
            axis="x",
            negative_elem="left_stop_liner",
            max_gap=0.001,
            max_penetration=0.0,
            name="open sash lands on the left stop",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="right_stop_liner",
            min_gap=0.55,
            max_gap=0.60,
            name="open pose leaves a serviceable clear opening on the right",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="bottom_left_shoe",
            elem_b="front_sill_wear_strip",
            name="open sash still rides on the wear strip",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

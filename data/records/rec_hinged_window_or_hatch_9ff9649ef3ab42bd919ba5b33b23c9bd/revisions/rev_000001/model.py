from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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


FRAME_WIDTH = 1.00
FRAME_HEIGHT = 0.75
FRAME_DEPTH = 0.10
FRAME_MEMBER = 0.065

OPENING_WIDTH = FRAME_WIDTH - 2.0 * FRAME_MEMBER
OPENING_HEIGHT = FRAME_HEIGHT - 2.0 * FRAME_MEMBER

SASH_WIDTH = 0.862
SASH_HEIGHT = 0.602
SASH_MEMBER = 0.055
SASH_DEPTH = 0.045
HINGE_RADIUS = 0.008

HINGE_AXIS_Y = 0.060
HINGE_AXIS_Z = FRAME_HEIGHT * 0.5 - FRAME_MEMBER + HINGE_RADIUS
SASH_CENTER_Y = -0.040
TOP_CLEARANCE = 0.020

SASH_TOP_EDGE = -TOP_CLEARANCE
SASH_BOTTOM_EDGE = SASH_TOP_EDGE - SASH_HEIGHT
SASH_TOP_RAIL_Z = SASH_TOP_EDGE - SASH_MEMBER * 0.5
SASH_BOTTOM_RAIL_Z = SASH_BOTTOM_EDGE + SASH_MEMBER * 0.5
SASH_SIDE_CENTER_Z = (SASH_TOP_EDGE + SASH_BOTTOM_EDGE) * 0.5
SASH_SIDE_CENTER_X = SASH_WIDTH * 0.5 - SASH_MEMBER * 0.5

GLASS_WIDTH = 0.780
GLASS_HEIGHT = 0.490
GLASS_THICKNESS = 0.018
ESCUTCHEON_WIDTH = 0.032
ESCUTCHEON_HEIGHT = 0.040
ESCUTCHEON_DEPTH = 0.006

FRAME_HINGE_LENGTH = 0.265
SASH_HINGE_LENGTH = 0.240
FRAME_HINGE_X = SASH_WIDTH * 0.5 - FRAME_HINGE_LENGTH * 0.5
SASH_HINGE_X = SASH_WIDTH * 0.5 - SASH_HINGE_LENGTH * 0.5

HANDLE_PIVOT_X = SASH_WIDTH * 0.5 - 0.065
HANDLE_PIVOT_Y = SASH_CENTER_Y + SASH_DEPTH * 0.5 + ESCUTCHEON_DEPTH
HANDLE_PIVOT_Z = SASH_BOTTOM_RAIL_Z + 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="awning_window")

    frame_white = model.material("frame_white", rgba=(0.93, 0.94, 0.95, 1.0))
    sash_white = model.material("sash_white", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.24, 0.26, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.67, 0.80, 0.88, 0.35))

    frame = model.part("outer_frame")
    frame.visual(
        Box((FRAME_MEMBER, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-FRAME_WIDTH * 0.5 + FRAME_MEMBER * 0.5, 0.0, 0.0)),
        material=frame_white,
        name="left_jamb",
    )
    frame.visual(
        Box((FRAME_MEMBER, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(FRAME_WIDTH * 0.5 - FRAME_MEMBER * 0.5, 0.0, 0.0)),
        material=frame_white,
        name="right_jamb",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_MEMBER)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT * 0.5 - FRAME_MEMBER * 0.5)),
        material=frame_white,
        name="head",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_MEMBER)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HEIGHT * 0.5 + FRAME_MEMBER * 0.5)),
        material=frame_white,
        name="sill",
    )
    frame.visual(
        Cylinder(radius=HINGE_RADIUS, length=FRAME_HINGE_LENGTH),
        origin=Origin(
            xyz=(-FRAME_HINGE_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=dark_hardware,
        name="frame_left_hinge",
    )
    frame.visual(
        Cylinder(radius=HINGE_RADIUS, length=FRAME_HINGE_LENGTH),
        origin=Origin(
            xyz=(FRAME_HINGE_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=dark_hardware,
        name="frame_right_hinge",
    )
    frame.visual(
        Box((FRAME_HINGE_LENGTH, 0.020, 0.024)),
        origin=Origin(xyz=(-FRAME_HINGE_X, 0.050, HINGE_AXIS_Z)),
        material=dark_hardware,
        name="frame_left_leaf",
    )
    frame.visual(
        Box((FRAME_HINGE_LENGTH, 0.020, 0.024)),
        origin=Origin(xyz=(FRAME_HINGE_X, 0.050, HINGE_AXIS_Z)),
        material=dark_hardware,
        name="frame_right_leaf",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=14.0,
    )

    sash = model.part("sash")
    sash.visual(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_MEMBER)),
        origin=Origin(xyz=(0.0, SASH_CENTER_Y, SASH_TOP_RAIL_Z)),
        material=sash_white,
        name="top_rail",
    )
    sash.visual(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_MEMBER)),
        origin=Origin(xyz=(0.0, SASH_CENTER_Y, SASH_BOTTOM_RAIL_Z)),
        material=sash_white,
        name="bottom_rail",
    )
    sash.visual(
        Box((SASH_MEMBER, SASH_DEPTH, SASH_HEIGHT)),
        origin=Origin(xyz=(-SASH_SIDE_CENTER_X, SASH_CENTER_Y, SASH_SIDE_CENTER_Z)),
        material=sash_white,
        name="left_stile",
    )
    sash.visual(
        Box((SASH_MEMBER, SASH_DEPTH, SASH_HEIGHT)),
        origin=Origin(xyz=(SASH_SIDE_CENTER_X, SASH_CENTER_Y, SASH_SIDE_CENTER_Z)),
        material=sash_white,
        name="right_stile",
    )
    sash.visual(
        Box((GLASS_WIDTH, GLASS_THICKNESS, GLASS_HEIGHT)),
        origin=Origin(xyz=(0.0, SASH_CENTER_Y + 0.004, SASH_SIDE_CENTER_Z)),
        material=glass_tint,
        name="glass",
    )
    sash.visual(
        Box((SASH_HINGE_LENGTH, 0.065, 0.012)),
        origin=Origin(xyz=(-SASH_HINGE_X, 0.0075, -0.016)),
        material=dark_hardware,
        name="sash_left_bracket",
    )
    sash.visual(
        Box((SASH_HINGE_LENGTH, 0.065, 0.012)),
        origin=Origin(xyz=(SASH_HINGE_X, 0.0075, -0.016)),
        material=dark_hardware,
        name="sash_right_bracket",
    )
    sash.visual(
        Box((ESCUTCHEON_WIDTH, ESCUTCHEON_DEPTH, ESCUTCHEON_HEIGHT)),
        origin=Origin(
            xyz=(
                HANDLE_PIVOT_X,
                SASH_CENTER_Y + SASH_DEPTH * 0.5 + ESCUTCHEON_DEPTH * 0.5,
                HANDLE_PIVOT_Z,
            )
        ),
        material=dark_hardware,
        name="escutcheon",
    )
    sash.inertial = Inertial.from_geometry(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_HEIGHT)),
        mass=9.0,
        origin=Origin(xyz=(0.0, SASH_CENTER_Y, SASH_SIDE_CENTER_Z)),
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_hardware,
        name="pivot",
    )
    handle.visual(
        Box((0.012, 0.010, 0.024)),
        origin=Origin(xyz=(-0.002, 0.010, 0.0)),
        material=dark_hardware,
        name="body",
    )
    handle.visual(
        Box((0.048, 0.008, 0.010)),
        origin=Origin(xyz=(-0.030, 0.015, -0.002)),
        material=dark_hardware,
        name="lever",
    )
    handle.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(-0.054, 0.015, -0.002)),
        material=dark_hardware,
        name="grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.020, 0.026)),
        mass=0.15,
        origin=Origin(xyz=(-0.022, 0.011, 0.0)),
    )

    model.articulation(
        "sash_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.8,
            lower=0.0,
            upper=radians(55.0),
        ),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=radians(-15.0),
            upper=radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("outer_frame")
    sash = object_model.get_part("sash")
    handle = object_model.get_part("latch_handle")

    sash_hinge = object_model.get_articulation("sash_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

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

    ctx.check(
        "sash_hinge_axis_is_horizontal",
        tuple(sash_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected sash hinge axis (1, 0, 0), got {sash_hinge.axis}.",
    )
    ctx.check(
        "handle_pivot_axis_faces_out_of_sash",
        tuple(handle_pivot.axis) == (0.0, 1.0, 0.0),
        f"Expected handle pivot axis (0, 1, 0), got {handle_pivot.axis}.",
    )

    with ctx.pose({sash_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_contact(frame, sash, elem_a="frame_left_leaf", elem_b="sash_left_bracket", name="left_hinge_contact_closed")
        ctx.expect_contact(frame, sash, elem_a="frame_right_leaf", elem_b="sash_right_bracket", name="right_hinge_contact_closed")
        ctx.expect_contact(
            handle,
            sash,
            elem_a="pivot",
            elem_b="escutcheon",
            name="handle_pivot_contact_closed",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="sill",
            min_gap=0.006,
            max_gap=0.0085,
            name="bottom_rail_clears_sill_closed",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            min_overlap=0.80,
            name="sash_spans_frame_width",
        )

    sash_limits = sash_hinge.motion_limits
    if sash_limits is not None and sash_limits.lower is not None and sash_limits.upper is not None:
        with ctx.pose({sash_hinge: sash_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="sash_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="sash_lower_no_floating")
        with ctx.pose({sash_hinge: sash_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="sash_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="sash_upper_no_floating")
            ctx.expect_overlap(
                frame,
                sash,
                elem_a="frame_left_hinge",
                elem_b="sash_left_bracket",
                axes="x",
                min_overlap=0.20,
                name="left_hinge_alignment_open",
            )
            ctx.expect_overlap(
                frame,
                sash,
                elem_a="frame_right_hinge",
                elem_b="sash_right_bracket",
                axes="x",
                min_overlap=0.20,
                name="right_hinge_alignment_open",
            )
        with ctx.pose({sash_hinge: 0.0}):
            closed_bottom = ctx.part_element_world_aabb(sash, elem="bottom_rail")
        with ctx.pose({sash_hinge: sash_limits.upper}):
            open_bottom = ctx.part_element_world_aabb(sash, elem="bottom_rail")
        bottom_aabbs_ready = closed_bottom is not None and open_bottom is not None
        ctx.check(
            "bottom_rail_aabbs_available",
            bottom_aabbs_ready,
            "Could not resolve bottom rail AABB in closed or open sash pose.",
        )
        if bottom_aabbs_ready:
            assert closed_bottom is not None
            assert open_bottom is not None
            ctx.check(
                "bottom_rail_swings_outward",
                open_bottom[1][1] > closed_bottom[1][1] + 0.30,
                (
                    f"Expected open bottom rail to project outward by >0.30 m; "
                    f"closed max y={closed_bottom[1][1]:.4f}, open max y={open_bottom[1][1]:.4f}."
                ),
            )
            ctx.check(
                "bottom_rail_lifts_when_open",
                open_bottom[0][2] > closed_bottom[0][2] + 0.18,
                (
                    f"Expected open bottom rail to rise by >0.18 m; "
                    f"closed min z={closed_bottom[0][2]:.4f}, open min z={open_bottom[0][2]:.4f}."
                ),
            )

    handle_limits = handle_pivot.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_pivot: handle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_lower_no_floating")
            ctx.expect_contact(
                handle,
                sash,
                elem_a="pivot",
                elem_b="escutcheon",
                name="handle_contact_lower",
            )
        with ctx.pose({handle_pivot: 0.0}):
            rest_handle = ctx.part_element_world_aabb(handle, elem="lever")
        with ctx.pose({handle_pivot: handle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_upper_no_floating")
            ctx.expect_contact(
                handle,
                sash,
                elem_a="pivot",
                elem_b="escutcheon",
                name="handle_contact_upper",
            )
            open_handle = ctx.part_element_world_aabb(handle, elem="lever")
        handle_aabbs_ready = rest_handle is not None and open_handle is not None
        ctx.check(
            "handle_lever_aabbs_available",
            handle_aabbs_ready,
            "Could not resolve lever AABB in rest or actuated pose.",
        )
        if handle_aabbs_ready:
            assert rest_handle is not None
            assert open_handle is not None
            ctx.check(
                "handle_lever_rotates_upward",
                open_handle[1][2] > rest_handle[1][2] + 0.03,
                (
                    f"Expected handle lever to rotate upward by >0.03 m; "
                    f"rest max z={rest_handle[1][2]:.4f}, open max z={open_handle[1][2]:.4f}."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

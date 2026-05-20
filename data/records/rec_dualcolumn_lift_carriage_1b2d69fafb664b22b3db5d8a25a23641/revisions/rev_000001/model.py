from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.42
BASE_DEPTH = 0.22
BASE_HEIGHT = 0.06
SPINE_WIDTH = 0.34
SPINE_THICKNESS = 0.02
SPINE_HEIGHT = 0.78
HEAD_WIDTH = 0.32
HEAD_DEPTH = 0.17
HEAD_HEIGHT = 0.08
COLUMN_RADIUS = 0.016
COLUMN_SPACING = 0.22
COLUMN_CENTER_Y = 0.03
COLUMN_CENTER_Z = 0.48
COLUMN_LENGTH = 0.64

FACEPLATE_WIDTH = 0.34
FACEPLATE_HEIGHT = 0.22
FACEPLATE_THICKNESS = 0.018
FRONT_PLATE_OFFSET_Y = 0.045
ARM_WIDTH = 0.07
ARM_DEPTH = 0.02
ARM_HEIGHT = 0.13
GUIDE_PAD_WIDTH = 0.028
GUIDE_PAD_DEPTH = 0.018
GUIDE_PAD_HEIGHT = 0.12
GUIDE_PAD_CENTER_X = (COLUMN_SPACING / 2.0) - COLUMN_RADIUS - (GUIDE_PAD_WIDTH / 2.0)
GUIDE_PAD_CENTER_Y = 0.008
LOWER_FACEPLATE_Z = 0.29
FACEPLATE_Y = 0.03
SLIDE_TRAVEL = 0.38


def make_frame_body() -> cq.Workplane:
    base = cq.Workplane("XY").box(FRAME_WIDTH, BASE_DEPTH, BASE_HEIGHT).translate(
        (0.0, 0.0, BASE_HEIGHT / 2.0)
    )
    spine = cq.Workplane("XY").box(
        SPINE_WIDTH, SPINE_THICKNESS, SPINE_HEIGHT
    ).translate((0.0, -0.055, BASE_HEIGHT + SPINE_HEIGHT / 2.0))
    lower_head = cq.Workplane("XY").box(HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT).translate(
        (0.0, -0.005, 0.13)
    )
    upper_head = cq.Workplane("XY").box(HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT).translate(
        (0.0, -0.005, 0.83)
    )
    center_rib = cq.Workplane("XY").box(0.07, 0.032, 0.48).translate(
        (0.0, -0.03, 0.48)
    )
    frame = base.union(spine).union(lower_head).union(upper_head).union(center_rib)
    return frame.edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_frame_dual_column_carriage")

    frame_dark = model.material("frame_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    column_steel = model.material("column_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    carriage_light = model.material("carriage_light", rgba=(0.77, 0.79, 0.82, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_body(), "frame_body"),
        material=frame_dark,
        name="frame_body",
    )
    frame.visual(
        Box((HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.005, 0.13)),
        material=frame_dark,
        name="lower_head",
    )
    frame.visual(
        Box((HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.005, 0.83)),
        material=frame_dark,
        name="upper_head",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_LENGTH),
        origin=Origin(
            xyz=(-(COLUMN_SPACING / 2.0), COLUMN_CENTER_Y, COLUMN_CENTER_Z)
        ),
        material=column_steel,
        name="left_column",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_LENGTH),
        origin=Origin(
            xyz=((COLUMN_SPACING / 2.0), COLUMN_CENTER_Y, COLUMN_CENTER_Z)
        ),
        material=column_steel,
        name="right_column",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Box((FACEPLATE_WIDTH, FACEPLATE_THICKNESS, FACEPLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, FRONT_PLATE_OFFSET_Y, 0.0)),
        material=carriage_light,
        name="front_plate",
    )
    faceplate.visual(
        Box((0.12, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, FRONT_PLATE_OFFSET_Y + 0.002, 0.075)),
        material=carriage_light,
        name="top_stiffener",
    )
    faceplate.visual(
        Box((0.12, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, FRONT_PLATE_OFFSET_Y + 0.002, -0.075)),
        material=carriage_light,
        name="bottom_stiffener",
    )
    faceplate.visual(
        Box((ARM_WIDTH, ARM_DEPTH, ARM_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_PAD_CENTER_X, 0.026, 0.0)),
        material=carriage_light,
        name="left_arm",
    )
    faceplate.visual(
        Box((ARM_WIDTH, ARM_DEPTH, ARM_HEIGHT)),
        origin=Origin(xyz=(GUIDE_PAD_CENTER_X, 0.026, 0.0)),
        material=carriage_light,
        name="right_arm",
    )
    faceplate.visual(
        Box((GUIDE_PAD_WIDTH, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_PAD_CENTER_X, GUIDE_PAD_CENTER_Y, 0.0)),
        material=carriage_light,
        name="left_guide_pad",
    )
    faceplate.visual(
        Box((GUIDE_PAD_WIDTH, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(GUIDE_PAD_CENTER_X, GUIDE_PAD_CENTER_Y, 0.0)),
        material=carriage_light,
        name="right_guide_pad",
    )

    model.articulation(
        "frame_to_faceplate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=faceplate,
        origin=Origin(xyz=(0.0, FACEPLATE_Y, LOWER_FACEPLATE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.40,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    faceplate = object_model.get_part("faceplate")
    slide = object_model.get_articulation("frame_to_faceplate")

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

    ctx.check("frame_exists", frame is not None, "frame part missing")
    ctx.check("faceplate_exists", faceplate is not None, "faceplate part missing")
    ctx.check(
        "slide_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected prismatic articulation, got {slide.articulation_type}",
    )
    ctx.check(
        "slide_axis_vertical",
        tuple(round(v, 6) for v in slide.axis) == (0.0, 0.0, 1.0),
        f"expected vertical +Z slide axis, got {slide.axis}",
    )
    ctx.check(
        "slide_limits_reasonable",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == SLIDE_TRAVEL,
        f"unexpected slide limits: {slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            faceplate,
            frame,
            elem_b="left_column",
            contact_tol=0.0015,
            name="lower_pose_left_column_support",
        )
        ctx.expect_contact(
            faceplate,
            frame,
            elem_b="right_column",
            contact_tol=0.0015,
            name="lower_pose_right_column_support",
        )
        ctx.expect_gap(
            faceplate,
            frame,
            axis="z",
            positive_elem="front_plate",
            negative_elem="lower_head",
            min_gap=0.008,
            max_gap=0.02,
            name="lower_pose_clear_of_lower_head",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            faceplate,
            frame,
            elem_b="left_column",
            contact_tol=0.0015,
            name="upper_pose_left_column_support",
        )
        ctx.expect_contact(
            faceplate,
            frame,
            elem_b="right_column",
            contact_tol=0.0015,
            name="upper_pose_right_column_support",
        )
        ctx.expect_gap(
            frame,
            faceplate,
            axis="z",
            positive_elem="upper_head",
            negative_elem="front_plate",
            min_gap=0.008,
            max_gap=0.02,
            name="upper_pose_clear_of_upper_head",
        )

    with ctx.pose({slide: 0.0}):
        lower_pos = ctx.part_world_position(faceplate)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        upper_pos = ctx.part_world_position(faceplate)

    moves_correctly = (
        lower_pos is not None
        and upper_pos is not None
        and abs(upper_pos[0] - lower_pos[0]) <= 1e-6
        and abs(upper_pos[1] - lower_pos[1]) <= 1e-6
        and abs((upper_pos[2] - lower_pos[2]) - SLIDE_TRAVEL) <= 1e-6
    )
    ctx.check(
        "faceplate_moves_only_along_vertical_guide_axis",
        moves_correctly,
        f"lower={lower_pos}, upper={upper_pos}, expected dz={SLIDE_TRAVEL}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_PLATE_X = 0.28
BASE_PLATE_Y = 0.22
BASE_PLATE_Z = 0.022

PEDESTAL_X = 0.12
PEDESTAL_Y = 0.10
PEDESTAL_Z = 0.055

MAST_X = 0.095
MAST_Y = 0.085
MAST_Z = 0.46

VERTICAL_RAIL_X = 0.016
VERTICAL_RAIL_Y = 0.010
VERTICAL_RAIL_Z = 0.34
VERTICAL_RAIL_X_OFFSET = 0.026
VERTICAL_RAIL_BOTTOM_Z = 0.15

SCREW_COVER_X = 0.018
SCREW_COVER_Y = 0.012
SCREW_COVER_Z = 0.36
SCREW_COVER_BOTTOM_Z = 0.135

HEAD_BODY_X = 0.090
HEAD_BODY_Y = 0.036
HEAD_BODY_Z = 0.074
HEAD_PAD_X = 0.024
HEAD_PAD_Y = 0.018
HEAD_PAD_Z = 0.068
HEAD_PAD_X_OFFSET = 0.026
HEAD_PAD_CENTER_Y = -0.027

HEAD_FRONT_X = 0.070
HEAD_FRONT_Y = 0.024
HEAD_FRONT_Z = 0.030
HEAD_FRONT_CENTER_Y = 0.030
HEAD_FRONT_CENTER_Z = 0.010

HEAD_TOP_X = 0.082
HEAD_TOP_Y = 0.022
HEAD_TOP_Z = 0.020
HEAD_TOP_CENTER_Y = 0.029
HEAD_TOP_CENTER_Z = 0.036

CROSS_RAIL_X = 0.164
CROSS_RAIL_Y = 0.014
CROSS_RAIL_Z = 0.016
CROSS_RAIL_CENTER_Y = 0.038
CROSS_RAIL_CENTER_Z = 0.032

SLIDE_CARRIAGE_X = 0.052
SLIDE_CARRIAGE_Y = 0.032
SLIDE_CARRIAGE_Z = 0.038

SLIDE_TABLE_X = 0.080
SLIDE_TABLE_Y = 0.060
SLIDE_TABLE_Z = 0.012
SLIDE_TABLE_CENTER_Y = 0.020
SLIDE_TABLE_CENTER_Z = 0.025

SLIDE_FRONT_LIP_X = 0.080
SLIDE_FRONT_LIP_Y = 0.010
SLIDE_FRONT_LIP_Z = 0.016
SLIDE_FRONT_LIP_CENTER_Y = 0.045
SLIDE_FRONT_LIP_CENTER_Z = 0.016

LIFT_HOME_Z = 0.24
LIFT_TRAVEL = 0.18
HEAD_FRAME_Y = MAST_Y / 2.0 + VERTICAL_RAIL_Y + abs(HEAD_PAD_CENTER_Y) + HEAD_PAD_Y / 2.0

SLIDE_HOME_Y = CROSS_RAIL_CENTER_Y + CROSS_RAIL_Y / 2.0 + SLIDE_CARRIAGE_Y / 2.0
SLIDE_HOME_Z = CROSS_RAIL_CENTER_Z
SLIDE_TRAVEL = 0.045


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material: str,
    name: str,
):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sideways_stage")

    model.material("base_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machined_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("bearing_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("stage_blue", rgba=(0.30, 0.42, 0.64, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        (BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z),
        (0.0, 0.0, BASE_PLATE_Z / 2.0),
        material="base_black",
        name="base_plate",
    )
    _add_box(
        frame,
        (PEDESTAL_X, PEDESTAL_Y, PEDESTAL_Z),
        (0.0, 0.0, BASE_PLATE_Z + PEDESTAL_Z / 2.0),
        material="base_black",
        name="pedestal",
    )
    _add_box(
        frame,
        (MAST_X, MAST_Y, MAST_Z),
        (0.0, 0.0, BASE_PLATE_Z + PEDESTAL_Z + MAST_Z / 2.0),
        material="machined_gray",
        name="mast",
    )
    _add_box(
        frame,
        (VERTICAL_RAIL_X, VERTICAL_RAIL_Y, VERTICAL_RAIL_Z),
        (
            -VERTICAL_RAIL_X_OFFSET,
            MAST_Y / 2.0 + VERTICAL_RAIL_Y / 2.0,
            VERTICAL_RAIL_BOTTOM_Z + VERTICAL_RAIL_Z / 2.0,
        ),
        material="rail_steel",
        name="left_vertical_rail",
    )
    _add_box(
        frame,
        (VERTICAL_RAIL_X, VERTICAL_RAIL_Y, VERTICAL_RAIL_Z),
        (
            VERTICAL_RAIL_X_OFFSET,
            MAST_Y / 2.0 + VERTICAL_RAIL_Y / 2.0,
            VERTICAL_RAIL_BOTTOM_Z + VERTICAL_RAIL_Z / 2.0,
        ),
        material="rail_steel",
        name="right_vertical_rail",
    )
    _add_box(
        frame,
        (SCREW_COVER_X, SCREW_COVER_Y, SCREW_COVER_Z),
        (
            0.0,
            MAST_Y / 2.0 + SCREW_COVER_Y / 2.0,
            SCREW_COVER_BOTTOM_Z + SCREW_COVER_Z / 2.0,
        ),
        material="base_black",
        name="screw_cover",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z + PEDESTAL_Z + MAST_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_PLATE_Z + PEDESTAL_Z + MAST_Z) / 2.0)),
    )

    head = model.part("head")
    _add_box(
        head,
        (HEAD_PAD_X, HEAD_PAD_Y, HEAD_PAD_Z),
        (-HEAD_PAD_X_OFFSET, HEAD_PAD_CENTER_Y, 0.0),
        material="bearing_dark",
        name="left_bearing_pad",
    )
    _add_box(
        head,
        (HEAD_PAD_X, HEAD_PAD_Y, HEAD_PAD_Z),
        (HEAD_PAD_X_OFFSET, HEAD_PAD_CENTER_Y, 0.0),
        material="bearing_dark",
        name="right_bearing_pad",
    )
    _add_box(
        head,
        (HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_Z),
        (0.0, 0.0, 0.0),
        material="machined_gray",
        name="head_body",
    )
    _add_box(
        head,
        (HEAD_FRONT_X, HEAD_FRONT_Y, HEAD_FRONT_Z),
        (0.0, HEAD_FRONT_CENTER_Y, HEAD_FRONT_CENTER_Z),
        material="machined_gray",
        name="head_front_block",
    )
    _add_box(
        head,
        (HEAD_TOP_X, HEAD_TOP_Y, HEAD_TOP_Z),
        (0.0, HEAD_TOP_CENTER_Y, HEAD_TOP_CENTER_Z),
        material="machined_gray",
        name="head_top_bridge",
    )
    _add_box(
        head,
        (CROSS_RAIL_X, CROSS_RAIL_Y, CROSS_RAIL_Z),
        (0.0, CROSS_RAIL_CENTER_Y, CROSS_RAIL_CENTER_Z),
        material="rail_steel",
        name="cross_rail",
    )
    head.inertial = Inertial.from_geometry(
        Box((CROSS_RAIL_X, 0.081, 0.082)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.004, 0.004)),
    )

    slide = model.part("cross_slide")
    _add_box(
        slide,
        (SLIDE_CARRIAGE_X, SLIDE_CARRIAGE_Y, SLIDE_CARRIAGE_Z),
        (0.0, 0.0, 0.0),
        material="bearing_dark",
        name="slide_carriage",
    )
    _add_box(
        slide,
        (SLIDE_TABLE_X, SLIDE_TABLE_Y, SLIDE_TABLE_Z),
        (0.0, SLIDE_TABLE_CENTER_Y, SLIDE_TABLE_CENTER_Z),
        material="stage_blue",
        name="slide_table",
    )
    _add_box(
        slide,
        (SLIDE_FRONT_LIP_X, SLIDE_FRONT_LIP_Y, SLIDE_FRONT_LIP_Z),
        (0.0, SLIDE_FRONT_LIP_CENTER_Y, SLIDE_FRONT_LIP_CENTER_Z),
        material="machined_gray",
        name="front_lip",
    )
    slide.inertial = Inertial.from_geometry(
        Box((SLIDE_TABLE_X, 0.066, 0.050)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.017, 0.006)),
    )

    model.articulation(
        "frame_to_head_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, HEAD_FRAME_Y, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=180.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "head_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=head,
        child=slide,
        origin=Origin(xyz=(0.0, SLIDE_HOME_Y, SLIDE_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=90.0,
            velocity=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    slide = object_model.get_part("cross_slide")
    lift = object_model.get_articulation("frame_to_head_lift")
    sideways = object_model.get_articulation("head_to_cross_slide")

    ctx.expect_contact(
        head,
        frame,
        name="head carriage is supported by the grounded mast at rest",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="z",
        min_overlap=0.060,
        name="head keeps substantial overlap with the vertical guide span at rest",
    )
    ctx.expect_contact(
        slide,
        head,
        name="cross slide is supported by the moving head at rest",
    )
    ctx.expect_overlap(
        slide,
        head,
        axes="x",
        min_overlap=0.050,
        name="cross slide keeps lateral rail engagement at rest",
    )

    head_rest = ctx.part_world_position(head)
    slide_rest = ctx.part_world_position(slide)

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_contact(
            head,
            frame,
            name="head carriage remains supported at full lift",
        )
        ctx.expect_overlap(
            head,
            frame,
            axes="z",
            min_overlap=0.060,
            name="head retains guide overlap at full lift",
        )
        head_top = ctx.part_world_position(head)

    with ctx.pose({sideways: SLIDE_TRAVEL}):
        ctx.expect_contact(
            slide,
            head,
            name="cross slide remains supported at right travel",
        )
        ctx.expect_overlap(
            slide,
            head,
            axes="x",
            min_overlap=0.030,
            name="cross slide retains right-side rail overlap",
        )
        slide_right = ctx.part_world_position(slide)

    with ctx.pose({sideways: -SLIDE_TRAVEL}):
        ctx.expect_contact(
            slide,
            head,
            name="cross slide remains supported at left travel",
        )
        ctx.expect_overlap(
            slide,
            head,
            axes="x",
            min_overlap=0.030,
            name="cross slide retains left-side rail overlap",
        )
        slide_left = ctx.part_world_position(slide)

    ctx.check(
        "vertical stage moves upward along +Z",
        head_rest is not None and head_top is not None and head_top[2] > head_rest[2] + 0.10,
        details=f"rest={head_rest}, top={head_top}",
    )
    ctx.check(
        "sideways stage moves toward +X",
        slide_rest is not None and slide_right is not None and slide_right[0] > slide_rest[0] + 0.03,
        details=f"rest={slide_rest}, right={slide_right}",
    )
    ctx.check(
        "sideways stage moves toward -X",
        slide_rest is not None and slide_left is not None and slide_left[0] < slide_rest[0] - 0.03,
        details=f"rest={slide_rest}, left={slide_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_W = 0.62
FRAME_H = 1.18
FRAME_D = 0.10
BACK_T = 0.016
SIDE_RIM = 0.06
END_CAP_H = 0.10
SPINE_W = 0.11
SPINE_D = 0.06
RAIL_PAD_W = 0.044
RAIL_PAD_D = 0.030
RAIL_H = 0.94
RAIL_W = 0.028
RAIL_D = 0.022
RAIL_X = 0.185
RAIL_CENTER_Y = BACK_T + RAIL_PAD_D + (RAIL_D / 2.0)

CROSSHEAD_W = 0.48
CROSSHEAD_H = 0.156
CROSSHEAD_D = 0.060
SHOE_W = 0.074
BEARING_BLOCK_D = 0.050
FRONT_PLATE_T = 0.012
TOP_BOTTOM_WALL = 0.020
POCKET_W = 0.270
POCKET_H = 0.086
POCKET_D = 0.012
GUIDE_BAR_D = 0.012
GUIDE_BAR_H = 0.010
GUIDE_BAR_Z = 0.031
TONGUE_D = 0.008
TONGUE_H = 0.052
GUIDE_SPAN_W = 0.296
CROSSHEAD_CENTER_Y = RAIL_CENTER_Y + (RAIL_D / 2.0) + (BEARING_BLOCK_D / 2.0)
CROSSHEAD_LOW_Z = -0.32
CROSSHEAD_TRAVEL = 0.62

CARRIAGE_W = 0.165
CARRIAGE_H = 0.072
CARRIAGE_D = 0.012
CARRIAGE_FRONT_PAD_W = 0.090
CARRIAGE_FRONT_PAD_H = 0.050
CARRIAGE_FRONT_PAD_D = 0.010
CARRIAGE_CENTER_Y = GUIDE_BAR_D / 2.0
CARRIAGE_TRAVEL = 0.048


def _rear_frame_shape() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(BACK_T, FRAME_W, FRAME_H).rotate(
        (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0
    ).translate((0.0, -(FRAME_D / 2.0) + (BACK_T / 2.0), 0.0))

    upright = cq.Workplane("XY").box(SIDE_RIM, FRAME_D - 0.004, FRAME_H)
    left_upright = upright.translate((-(FRAME_W / 2.0) + (SIDE_RIM / 2.0), -0.002, 0.0))
    right_upright = upright.translate(((FRAME_W / 2.0) - (SIDE_RIM / 2.0), -0.002, 0.0))

    crossmember = cq.Workplane("XY").box(FRAME_W, 0.060, END_CAP_H)
    top_cap = crossmember.translate((0.0, -0.020, (FRAME_H / 2.0) - (END_CAP_H / 2.0)))
    bottom_cap = crossmember.translate((0.0, -0.020, -((FRAME_H / 2.0) - (END_CAP_H / 2.0))))

    spine = cq.Workplane("XY").box(SPINE_W, 0.060, FRAME_H - 0.24).translate((0.0, -0.020, 0.0))

    rail_mount = cq.Workplane("XY").box(RAIL_PAD_W, 0.090, RAIL_H + 0.06)
    left_mount = rail_mount.translate((-RAIL_X, 0.001, 0.0))
    right_mount = rail_mount.translate((RAIL_X, 0.001, 0.0))

    return (
        back_plate.union(left_upright)
        .union(right_upright)
        .union(top_cap)
        .union(bottom_cap)
        .union(spine)
        .union(left_mount)
        .union(right_mount)
    )


def _rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(RAIL_W, RAIL_D, RAIL_H)
    return rail.edges("|Z").fillet(0.002)


def _crosshead_shape() -> cq.Workplane:
    left_block = cq.Workplane("XY").box(SHOE_W, BEARING_BLOCK_D, CROSSHEAD_H).translate(
        (-RAIL_X, 0.0, 0.0)
    )
    right_block = cq.Workplane("XY").box(SHOE_W, BEARING_BLOCK_D, CROSSHEAD_H).translate(
        (RAIL_X, 0.0, 0.0)
    )
    top_bridge = cq.Workplane("XY").box(CROSSHEAD_W, 0.018, TOP_BOTTOM_WALL).translate(
        (0.0, 0.002, (CROSSHEAD_H / 2.0) - (TOP_BOTTOM_WALL / 2.0))
    )
    bottom_bridge = cq.Workplane("XY").box(CROSSHEAD_W, 0.018, TOP_BOTTOM_WALL).translate(
        (0.0, 0.002, -((CROSSHEAD_H / 2.0) - (TOP_BOTTOM_WALL / 2.0)))
    )
    upper_guide = cq.Workplane("XY").box(GUIDE_SPAN_W, GUIDE_BAR_D, GUIDE_BAR_H).translate(
        (0.0, 0.0, GUIDE_BAR_Z)
    )
    lower_guide = cq.Workplane("XY").box(GUIDE_SPAN_W, GUIDE_BAR_D, GUIDE_BAR_H).translate(
        (0.0, 0.0, -GUIDE_BAR_Z)
    )
    left_upright = cq.Workplane("XY").box(0.028, FRONT_PLATE_T, POCKET_H + 0.028).translate(
        (-0.132, 0.014, 0.0)
    )
    right_upright = cq.Workplane("XY").box(0.028, FRONT_PLATE_T, POCKET_H + 0.028).translate(
        (0.132, 0.014, 0.0)
    )
    top_lip = cq.Workplane("XY").box(POCKET_W + 0.050, FRONT_PLATE_T, 0.016).translate(
        (0.0, 0.014, 0.051)
    )
    bottom_lip = cq.Workplane("XY").box(POCKET_W + 0.050, FRONT_PLATE_T, 0.016).translate(
        (0.0, 0.014, -0.051)
    )
    return (
        left_block.union(right_block)
        .union(top_bridge)
        .union(bottom_bridge)
        .union(upper_guide)
        .union(lower_guide)
        .union(left_upright)
        .union(right_upright)
        .union(top_lip)
        .union(bottom_lip)
    )


def _carriage_shape() -> cq.Workplane:
    runner = cq.Workplane("XY").box(
        CARRIAGE_W - 0.022,
        TONGUE_D,
        TONGUE_H,
        centered=(True, False, True),
    )
    face_plate = cq.Workplane("XY").box(
        CARRIAGE_W,
        0.008,
        CARRIAGE_H,
        centered=(True, False, True),
    ).translate((0.0, TONGUE_D, 0.0))
    front_pad = cq.Workplane("XY").box(
        CARRIAGE_FRONT_PAD_W,
        CARRIAGE_FRONT_PAD_D,
        CARRIAGE_FRONT_PAD_H,
        centered=(True, False, True),
    ).translate((0.0, TONGUE_D + 0.008, 0.0))
    tool_block = cq.Workplane("XY").box(
        0.050,
        0.010,
        0.036,
        centered=(True, False, True),
    ).translate((0.0, TONGUE_D + 0.018, 0.0))
    return runner.union(face_plate).union(front_pad).union(tool_block)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_gantry_transfer_module")

    frame_color = model.material("frame_gray", color=(0.22, 0.24, 0.26, 1.0))
    rail_color = model.material("rail_steel", color=(0.72, 0.74, 0.77, 1.0))
    crosshead_color = model.material("crosshead_aluminum", color=(0.66, 0.69, 0.72, 1.0))
    carriage_color = model.material("carriage_black", color=(0.18, 0.18, 0.20, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        origin=Origin(),
        material=frame_color,
        name="frame_shell",
    )

    left_rail = model.part("left_base_rail")
    left_rail.visual(
        mesh_from_cadquery(_rail_shape(), "left_base_rail"),
        origin=Origin(),
        material=rail_color,
        name="rail_body",
    )

    right_rail = model.part("right_base_rail")
    right_rail.visual(
        mesh_from_cadquery(_rail_shape(), "right_base_rail"),
        origin=Origin(),
        material=rail_color,
        name="rail_body",
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_crosshead_shape(), "crosshead"),
        origin=Origin(),
        material=crosshead_color,
        name="crosshead_body",
    )

    carriage = model.part("nested_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "nested_carriage"),
        origin=Origin(),
        material=carriage_color,
        name="carriage_body",
    )

    model.articulation(
        "frame_to_left_rail",
        ArticulationType.FIXED,
        parent=rear_frame,
        child=left_rail,
        origin=Origin(xyz=(-RAIL_X, RAIL_CENTER_Y, 0.0)),
    )
    model.articulation(
        "frame_to_right_rail",
        ArticulationType.FIXED,
        parent=rear_frame,
        child=right_rail,
        origin=Origin(xyz=(RAIL_X, RAIL_CENTER_Y, 0.0)),
    )
    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, CROSSHEAD_CENTER_Y, CROSSHEAD_LOW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.80,
            lower=0.0,
            upper=CROSSHEAD_TRAVEL,
        ),
    )
    model.articulation(
        "crosshead_to_carriage",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_CENTER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.45,
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    left_rail = object_model.get_part("left_base_rail")
    right_rail = object_model.get_part("right_base_rail")
    crosshead = object_model.get_part("crosshead")
    carriage = object_model.get_part("nested_carriage")
    crosshead_slide = object_model.get_articulation("frame_to_crosshead")
    carriage_slide = object_model.get_articulation("crosshead_to_carriage")

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

    ctx.expect_contact(left_rail, rear_frame, name="left_rail_contacts_rear_frame")
    ctx.expect_contact(right_rail, rear_frame, name="right_rail_contacts_rear_frame")
    ctx.expect_gap(
        crosshead,
        left_rail,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="crosshead_runs_flush_to_left_rail_face",
    )
    ctx.expect_gap(
        crosshead,
        right_rail,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="crosshead_runs_flush_to_right_rail_face",
    )
    ctx.expect_overlap(
        crosshead,
        left_rail,
        axes="xz",
        min_overlap=0.02,
        name="crosshead_left_rail_vertical_registration",
    )
    ctx.expect_overlap(
        crosshead,
        right_rail,
        axes="xz",
        min_overlap=0.02,
        name="crosshead_right_rail_vertical_registration",
    )
    ctx.expect_within(
        carriage,
        crosshead,
        axes="xz",
        margin=0.01,
        name="carriage_nested_in_crosshead_at_center",
    )

    ctx.check(
        "crosshead_joint_axis_is_vertical",
        tuple(crosshead_slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {crosshead_slide.axis}",
    )
    ctx.check(
        "carriage_joint_axis_is_horizontal",
        tuple(carriage_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {carriage_slide.axis}",
    )

    with ctx.pose({crosshead_slide: 0.0}):
        crosshead_low = ctx.part_world_position(crosshead)
    with ctx.pose({crosshead_slide: CROSSHEAD_TRAVEL}):
        crosshead_high = ctx.part_world_position(crosshead)
        ctx.expect_gap(
            crosshead,
            left_rail,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="crosshead_flush_to_left_rail_at_upper_limit",
        )
        ctx.expect_gap(
            crosshead,
            right_rail,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="crosshead_flush_to_right_rail_at_upper_limit",
        )
        ctx.expect_overlap(
            crosshead,
            left_rail,
            axes="xz",
            min_overlap=0.02,
            name="crosshead_left_rail_registration_at_upper_limit",
        )
        ctx.expect_overlap(
            crosshead,
            right_rail,
            axes="xz",
            min_overlap=0.02,
            name="crosshead_right_rail_registration_at_upper_limit",
        )
    ctx.check(
        "positive_crosshead_travel_moves_upward",
        (
            crosshead_low is not None
            and crosshead_high is not None
            and (crosshead_high[2] - crosshead_low[2]) > 0.50
        ),
        details=f"low={crosshead_low}, high={crosshead_high}",
    )

    with ctx.pose({carriage_slide: -CARRIAGE_TRAVEL}):
        carriage_left = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            crosshead,
            axes="xz",
            margin=0.01,
            name="carriage_nested_at_left_limit",
        )
    with ctx.pose({carriage_slide: CARRIAGE_TRAVEL}):
        carriage_right = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            crosshead,
            axes="xz",
            margin=0.01,
            name="carriage_nested_at_right_limit",
        )
    ctx.check(
        "positive_carriage_travel_moves_rightward",
        (
            carriage_left is not None
            and carriage_right is not None
            and (carriage_right[0] - carriage_left[0]) > 0.08
        ),
        details=f"left={carriage_left}, right={carriage_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

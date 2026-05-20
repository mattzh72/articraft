from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drafting_table")

    powder_gray = model.material("powder_gray", rgba=(0.25, 0.27, 0.29, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    board_cream = model.material("board_cream", rgba=(0.88, 0.85, 0.76, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.58, 0.56, 0.52, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal = model.part("pedestal_base")
    pedestal.visual(
        Box((0.68, 0.48, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=charcoal,
        name="base_plinth",
    )
    pedestal.visual(
        Box((0.40, 0.30, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=powder_gray,
        name="base_step",
    )
    pedestal.visual(
        Box((0.03, 0.18, 0.42)),
        origin=Origin(xyz=(-0.075, 0.0, 0.35)),
        material=powder_gray,
        name="sleeve_left_wall",
    )
    pedestal.visual(
        Box((0.03, 0.18, 0.42)),
        origin=Origin(xyz=(0.075, 0.0, 0.35)),
        material=powder_gray,
        name="sleeve_right_wall",
    )
    pedestal.visual(
        Box((0.12, 0.03, 0.42)),
        origin=Origin(xyz=(0.0, -0.075, 0.35)),
        material=powder_gray,
        name="sleeve_front_wall",
    )
    pedestal.visual(
        Box((0.12, 0.03, 0.42)),
        origin=Origin(xyz=(0.0, 0.075, 0.35)),
        material=powder_gray,
        name="sleeve_back_wall",
    )
    pedestal.visual(
        Box((0.24, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=warm_gray,
        name="sleeve_lower_collar",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.68, 0.48, 0.56)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    column = model.part("telescoping_column")
    column.visual(
        Box((0.108, 0.108, 0.68)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=powder_gray,
        name="column_mast",
    )
    column.visual(
        Box((0.18, 0.18, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=warm_gray,
        name="lift_collar",
    )
    column.visual(
        Box((0.18, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=powder_gray,
        name="trunnion_crosshead",
    )
    column.visual(
        Box((0.022, 0.10, 0.11)),
        origin=Origin(xyz=(-0.079, 0.0, 0.385)),
        material=powder_gray,
        name="left_cheek",
    )
    column.visual(
        Box((0.022, 0.10, 0.11)),
        origin=Origin(xyz=(0.079, 0.0, 0.385)),
        material=powder_gray,
        name="right_cheek",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.80)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    model.articulation(
        "pedestal_to_column",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.08,
            lower=0.0,
            upper=0.22,
        ),
    )

    board = model.part("drafting_board")
    board.visual(
        Cylinder(radius=0.028, length=0.136),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=charcoal,
        name="trunnion_barrel",
    )
    board.visual(
        Box((0.12, 0.16, 0.064)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=powder_gray,
        name="center_mount_block",
    )
    board.visual(
        Box((1.00, 0.70, 0.028)),
        origin=Origin(xyz=(0.0, -0.05, 0.078)),
        material=board_cream,
        name="board_panel",
    )
    board.visual(
        Box((0.94, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.392, 0.054)),
        material=warm_gray,
        name="front_trim",
    )
    board.visual(
        Box((0.86, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.275, 0.039)),
        material=warm_gray,
        name="rear_stiffener",
    )
    board.visual(
        Box((0.018, 0.24, 0.05)),
        origin=Origin(xyz=(-0.349, -0.23, 0.039)),
        material=powder_gray,
        name="left_runner",
    )
    board.visual(
        Box((0.018, 0.24, 0.05)),
        origin=Origin(xyz=(0.349, -0.23, 0.039)),
        material=powder_gray,
        name="right_runner",
    )
    board.visual(
        Box((0.72, 0.016, 0.05)),
        origin=Origin(xyz=(0.0, -0.112, 0.039)),
        material=powder_gray,
        name="runner_backstop",
    )
    board.inertial = Inertial.from_geometry(
        Box((1.00, 0.70, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.05, 0.05)),
    )

    model.articulation(
        "column_to_board",
        ArticulationType.REVOLUTE,
        parent=column,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.8,
            lower=0.0,
            upper=1.12,
        ),
    )

    tray = model.part("pencil_tray")
    tray.visual(
        Box((0.68, 0.18, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=warm_gray,
        name="tray_body_bottom",
    )
    tray.visual(
        Box((0.01, 0.18, 0.036)),
        origin=Origin(xyz=(-0.335, 0.0, -0.002)),
        material=warm_gray,
        name="tray_left_wall",
    )
    tray.visual(
        Box((0.01, 0.18, 0.036)),
        origin=Origin(xyz=(0.335, 0.0, -0.002)),
        material=warm_gray,
        name="tray_right_wall",
    )
    tray.visual(
        Box((0.66, 0.01, 0.036)),
        origin=Origin(xyz=(0.0, 0.085, -0.002)),
        material=warm_gray,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((0.72, 0.018, 0.05)),
        origin=Origin(xyz=(0.0, -0.099, 0.002)),
        material=powder_gray,
        name="tray_front",
    )
    tray.visual(
        Box((0.12, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.117, 0.008)),
        material=handle_black,
        name="tray_pull",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.72, 0.22, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.01, 0.0)),
    )

    model.articulation(
        "board_to_tray",
        ArticulationType.PRISMATIC,
        parent=board,
        child=tray,
        origin=Origin(xyz=(0.0, -0.289, -0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.15,
            lower=0.0,
            upper=0.10,
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

    pedestal = object_model.get_part("pedestal_base")
    column = object_model.get_part("telescoping_column")
    board = object_model.get_part("drafting_board")
    tray = object_model.get_part("pencil_tray")

    column_slide = object_model.get_articulation("pedestal_to_column")
    board_tilt = object_model.get_articulation("column_to_board")
    tray_slide = object_model.get_articulation("board_to_tray")

    column_mast = column.get_visual("column_mast")
    board_panel = board.get_visual("board_panel")
    front_trim = board.get_visual("front_trim")
    rear_stiffener = board.get_visual("rear_stiffener")
    left_runner = board.get_visual("left_runner")
    tray_bottom = tray.get_visual("tray_body_bottom")
    tray_front = tray.get_visual("tray_front")

    ctx.expect_origin_distance(
        column,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="column stays centered on pedestal",
    )
    ctx.expect_origin_distance(
        tray,
        board,
        axes="x",
        max_dist=0.001,
        name="pencil tray stays centered under board",
    )

    with ctx.pose({column_slide: 0.0}):
        ctx.expect_overlap(
            column,
            pedestal,
            axes="z",
            elem_a=column_mast,
            min_overlap=0.32,
            name="column remains deeply inserted at rest",
        )

    slide_upper = column_slide.motion_limits.upper if column_slide.motion_limits is not None else None
    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({column_slide: slide_upper if slide_upper is not None else 0.0}):
        ctx.expect_overlap(
            column,
            pedestal,
            axes="z",
            elem_a=column_mast,
            min_overlap=0.10,
            name="column keeps retained insertion at full height",
        )
        extended_column_pos = ctx.part_world_position(column)
    ctx.check(
        "column raises upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.18,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    ctx.expect_gap(
        board,
        tray,
        axis="z",
        positive_elem=board_panel,
        negative_elem=tray_front,
        min_gap=0.02,
        name="tray sits below drafting surface",
    )

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_overlap(
            tray,
            board,
            axes="y",
            elem_a=tray_bottom,
            elem_b=left_runner,
            min_overlap=0.14,
            name="closed tray sits fully on runners",
        )

    tray_upper = tray_slide.motion_limits.upper if tray_slide.motion_limits is not None else None
    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_upper if tray_upper is not None else 0.0}):
        ctx.expect_overlap(
            tray,
            board,
            axes="y",
            elem_a=tray_bottom,
            elem_b=left_runner,
            min_overlap=0.04,
            name="extended tray keeps runner engagement",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "tray pulls forward",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < rest_tray_pos[1] - 0.08,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    tilt_probe = 0.9
    with ctx.pose({board_tilt: tilt_probe}):
        rear_aabb = ctx.part_element_world_aabb(board, elem=rear_stiffener)
        front_aabb = ctx.part_element_world_aabb(board, elem=front_trim)
    ctx.check(
        "board tilts upward at the rear",
        rear_aabb is not None
        and front_aabb is not None
        and rear_aabb[1][2] > front_aabb[1][2] + 0.25,
        details=f"rear={rear_aabb}, front={front_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

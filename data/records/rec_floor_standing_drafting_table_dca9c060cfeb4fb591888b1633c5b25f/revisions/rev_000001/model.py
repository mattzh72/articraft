from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="teaching_room_drafting_table")

    blue_gray = model.material("blue_gray_painted_steel", rgba=(0.17, 0.23, 0.28, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    satin_steel = model.material("satin_zinc_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.014, 0.012, 1.0))
    board_green = model.material("matte_green_drafting_surface", rgba=(0.62, 0.70, 0.61, 1.0))
    tray_dark = model.material("dark_anodized_tray", rgba=(0.08, 0.085, 0.09, 1.0))

    floor_base = model.part("floor_base")
    floor_base.visual(
        Box((1.35, 0.54, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=blue_gray,
        name="wide_floor_foot",
    )
    floor_base.visual(
        Box((0.30, 0.86, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue_gray,
        name="fore_aft_outrigger",
    )
    floor_base.visual(
        Box((0.32, 0.24, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_steel,
        name="raised_pedestal",
    )

    column_sleeve = model.part("column_sleeve")
    sleeve_height = 0.62
    column_sleeve.visual(
        Box((0.185, 0.014, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.066, sleeve_height / 2.0)),
        material=dark_steel,
        name="front_sleeve_wall",
    )
    column_sleeve.visual(
        Box((0.185, 0.014, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.066, sleeve_height / 2.0)),
        material=dark_steel,
        name="rear_sleeve_wall",
    )
    column_sleeve.visual(
        Box((0.014, 0.132, sleeve_height)),
        origin=Origin(xyz=(0.0855, 0.0, sleeve_height / 2.0)),
        material=dark_steel,
        name="side_sleeve_wall_0",
    )
    column_sleeve.visual(
        Box((0.014, 0.132, sleeve_height)),
        origin=Origin(xyz=(-0.0855, 0.0, sleeve_height / 2.0)),
        material=dark_steel,
        name="side_sleeve_wall_1",
    )
    # A slightly larger collar makes the sleeve read as a hollow height-adjustment tube.
    column_sleeve.visual(
        Box((0.215, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, 0.079, sleeve_height - 0.0275)),
        material=satin_steel,
        name="front_top_collar",
    )
    column_sleeve.visual(
        Box((0.215, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, -0.079, sleeve_height - 0.0275)),
        material=satin_steel,
        name="rear_top_collar",
    )
    column_sleeve.visual(
        Box((0.026, 0.158, 0.055)),
        origin=Origin(xyz=(0.098, 0.0, sleeve_height - 0.0275)),
        material=satin_steel,
        name="side_top_collar_0",
    )
    column_sleeve.visual(
        Box((0.026, 0.158, 0.055)),
        origin=Origin(xyz=(-0.098, 0.0, sleeve_height - 0.0275)),
        material=satin_steel,
        name="side_top_collar_1",
    )

    height_column = model.part("height_column")
    height_column.visual(
        Box((0.090, 0.054, 0.980)),
        # The joint frame is the top lip of the outer sleeve; the member extends
        # downward into the sleeve and upward to the board-support head.
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=satin_steel,
        name="inner_column",
    )
    height_column.visual(
        Box((0.125, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=dark_steel,
        name="head_mount_plate",
    )
    for index, x_pos in enumerate((0.06175, -0.06175)):
        height_column.visual(
            Box((0.0335, 0.030, 0.060)),
            origin=Origin(xyz=(x_pos, 0.0, -0.320)),
            material=satin_steel,
            name=f"side_guide_pad_{index}",
        )
    for index, y_pos in enumerate((0.043, -0.043)):
        height_column.visual(
            Box((0.030, 0.032, 0.060)),
            origin=Origin(xyz=(0.0, y_pos, -0.320)),
            material=satin_steel,
            name=f"fore_aft_guide_pad_{index}",
        )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.22, 0.14, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark_steel,
        name="separate_head_block",
    )
    support_head.visual(
        Box((0.10, 0.080, 0.130)),
        origin=Origin(xyz=(0.0, -0.015, 0.115)),
        material=dark_steel,
        name="short_neck",
    )
    support_head.visual(
        Box((1.46, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, -0.040, 0.145)),
        material=dark_steel,
        name="lower_yoke_crossbar",
    )
    support_head.visual(
        Box((0.050, 0.105, 0.225)),
        origin=Origin(xyz=(0.690, -0.040, 0.235)),
        material=dark_steel,
        name="yoke_cheek_0",
    )
    support_head.visual(
        Box((0.050, 0.105, 0.225)),
        origin=Origin(xyz=(-0.690, -0.040, 0.235)),
        material=dark_steel,
        name="yoke_cheek_1",
    )
    support_head.visual(
        Cylinder(radius=0.018, length=1.48),
        origin=Origin(xyz=(0.0, -0.040, 0.240), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_pin",
    )
    support_head.visual(
        Box((0.050, 0.095, 0.125)),
        origin=Origin(xyz=(0.720, -0.040, 0.240)),
        material=dark_steel,
        name="clamp_side_tab",
    )
    support_head.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.754, -0.040, 0.240), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="knob_boss",
    )

    board = model.part("board")
    board.visual(
        Box((1.20, 0.82, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=board_green,
        name="drawing_board_panel",
    )
    board.visual(
        Box((1.24, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.425, 0.065)),
        material=satin_steel,
        name="top_edge_frame",
    )
    board.visual(
        Box((1.24, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.425, 0.065)),
        material=satin_steel,
        name="bottom_edge_frame",
    )
    board.visual(
        Box((0.030, 0.86, 0.040)),
        origin=Origin(xyz=(0.615, 0.0, 0.065)),
        material=satin_steel,
        name="side_edge_frame_0",
    )
    board.visual(
        Box((0.030, 0.86, 0.040)),
        origin=Origin(xyz=(-0.615, 0.0, 0.065)),
        material=satin_steel,
        name="side_edge_frame_1",
    )
    board.visual(
        Box((0.52, 0.090, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_steel,
        name="hinge_leaf",
    )
    board.visual(
        Cylinder(radius=0.026, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    board.visual(
        Box((1.15, 0.085, 0.024)),
        origin=Origin(xyz=(0.0, -0.475, 0.058)),
        material=tray_dark,
        name="tool_tray_shelf",
    )
    board.visual(
        Box((1.15, 0.024, 0.082)),
        origin=Origin(xyz=(0.0, -0.425, 0.078)),
        material=tray_dark,
        name="tool_tray_back",
    )
    board.visual(
        Box((1.15, 0.026, 0.072)),
        origin=Origin(xyz=(0.0, -0.520, 0.078)),
        material=tray_dark,
        name="tool_tray_lip",
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="threaded_shaft",
    )
    for index, x_pos in enumerate((0.007, 0.017, 0.027, 0.037, 0.047)):
        clamp_knob.visual(
            Cylinder(radius=0.014, length=0.0035),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"thread_ridge_{index}",
        )
    clamp_knob.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="knob_back_washer",
    )
    clamp_knob.visual(
        Cylinder(radius=0.040, length=0.045),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="knob_center_hub",
    )
    for index, (y_pos, z_pos) in enumerate(
        (
            (0.000, 0.044),
            (0.038, 0.022),
            (0.038, -0.022),
            (0.000, -0.044),
            (-0.038, -0.022),
            (-0.038, 0.022),
        )
    ):
        clamp_knob.visual(
            Cylinder(radius=0.025, length=0.045),
            origin=Origin(xyz=(0.085, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"knob_lobe_{index}",
        )
    clamp_knob.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.1105, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="knob_end_cap",
    )

    model.articulation(
        "base_to_sleeve",
        ArticulationType.FIXED,
        parent=floor_base,
        child=column_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )
    model.articulation(
        "sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=column_sleeve,
        child=height_column,
        origin=Origin(xyz=(0.0, 0.0, sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.260),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=height_column,
        child=support_head,
        origin=Origin(xyz=(0.0, 0.0, 0.5775)),
    )
    model.articulation(
        "head_to_board",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=board,
        origin=Origin(xyz=(0.0, -0.040, 0.240), rpy=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-0.20, upper=1.00),
    )
    model.articulation(
        "head_to_knob",
        ArticulationType.CONTINUOUS,
        parent=support_head,
        child=clamp_knob,
        origin=Origin(xyz=(0.763, -0.040, 0.240)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("column_sleeve")
    column = object_model.get_part("height_column")
    head = object_model.get_part("support_head")
    board = object_model.get_part("board")
    knob = object_model.get_part("clamp_knob")
    height_slide = object_model.get_articulation("sleeve_to_column")
    tilt_hinge = object_model.get_articulation("head_to_board")
    knob_spin = object_model.get_articulation("head_to_knob")

    ctx.allow_overlap(
        head,
        board,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        reason="The board hinge knuckle is intentionally modeled around the support-head hinge pin.",
    )

    ctx.expect_within(
        column,
        sleeve,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="front_sleeve_wall",
        margin=0.10,
        name="sliding column stays centered in the outer sleeve footprint",
    )
    ctx.expect_overlap(
        column,
        sleeve,
        axes="z",
        elem_a="inner_column",
        elem_b="front_sleeve_wall",
        min_overlap=0.15,
        name="column remains inserted in sleeve at classroom rest height",
    )
    ctx.expect_gap(
        head,
        column,
        axis="z",
        positive_elem="separate_head_block",
        negative_elem="head_mount_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="separate support head is seated on the height column",
    )
    ctx.expect_overlap(
        head,
        board,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        min_overlap=0.45,
        name="horizontal hinge pin runs through the board knuckle",
    )
    ctx.expect_gap(
        knob,
        head,
        axis="x",
        positive_elem="threaded_shaft",
        negative_elem="knob_boss",
        min_gap=-0.001,
        max_gap=0.004,
        max_penetration=0.001,
        name="clamp knob shaft seats against the head boss without fusing to it",
    )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({height_slide: 0.260}):
        raised_column_pos = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            sleeve,
            axes="z",
            elem_a="inner_column",
            elem_b="front_sleeve_wall",
            min_overlap=0.12,
            name="raised column still retains insertion in sleeve",
        )
    ctx.check(
        "height adjustment raises the column",
        rest_column_pos is not None
        and raised_column_pos is not None
        and raised_column_pos[2] > rest_column_pos[2] + 0.20,
        details=f"rest={rest_column_pos}, raised={raised_column_pos}",
    )

    rest_board_aabb = ctx.part_world_aabb(board)
    with ctx.pose({tilt_hinge: 1.00}):
        tilted_board_aabb = ctx.part_world_aabb(board)
    ctx.check(
        "board tilts upward about a horizontal head hinge",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and tilted_board_aabb[1][2] > rest_board_aabb[1][2] + 0.18,
        details=f"rest_aabb={rest_board_aabb}, tilted_aabb={tilted_board_aabb}",
    )

    ctx.check(
        "clamp knob is a separate continuous rotary joint on the threaded axis",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (1.0, 0.0, 0.0)
        and knob.name == "clamp_knob",
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()

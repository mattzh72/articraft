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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drafting_table")

    powder_coat = model.material("powder_coat", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.47, 0.48, 0.50, 1.0))
    board_white = model.material("board_white", rgba=(0.93, 0.93, 0.90, 1.0))
    maple = model.material("maple", rgba=(0.73, 0.62, 0.47, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    pedestal_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.110, 0.000),
                (0.110, 0.014),
                (0.096, 0.090),
                (0.078, 0.186),
                (0.072, 0.200),
            ],
            [
                (0.072, 0.000),
                (0.060, 0.090),
                (0.050, 0.200),
            ],
            segments=72,
        ),
        "drafting_table_pedestal_shell",
    )
    base_plate_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.220, 0.000),
                (0.220, 0.024),
            ],
            [
                (0.040, 0.000),
                (0.040, 0.024),
            ],
            segments=72,
        ),
        "drafting_table_base_plate",
    )
    guide_collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.055, 0.000),
                (0.055, 0.028),
            ],
            [
                (0.033, 0.000),
                (0.033, 0.028),
            ],
            segments=48,
        ),
        "drafting_table_guide_collar",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        base_plate_mesh,
        origin=Origin(),
        material=powder_coat,
        name="base_plate",
    )
    pedestal.visual(
        pedestal_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=powder_coat,
        name="pedestal_shell",
    )
    pedestal.visual(
        guide_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=warm_gray,
        name="guide_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(0.070, 0.0, 0.226),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_steel,
        name="height_lock_stem",
    )
    pedestal.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(
            xyz=(0.097, 0.0, 0.226),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="height_lock_grip",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.440, 0.440, 0.266)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.028, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=satin_steel,
        name="inner_column",
    )
    column.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=warm_gray,
        name="base_stop_collar",
    )
    column.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=warm_gray,
        name="lift_collar",
    )
    column.visual(
        Box((0.082, 0.090, 0.060)),
        origin=Origin(xyz=(0.008, 0.0, 0.675)),
        material=powder_coat,
        name="yoke_neck",
    )
    column.visual(
        Box((0.070, 0.540, 0.050)),
        origin=Origin(xyz=(-0.060, 0.0, 0.690)),
        material=powder_coat,
        name="yoke_crosshead",
    )
    column.visual(
        Box((0.084, 0.030, 0.160)),
        origin=Origin(xyz=(0.006, -0.255, 0.755)),
        material=powder_coat,
        name="left_yoke_arm",
    )
    column.visual(
        Box((0.084, 0.030, 0.160)),
        origin=Origin(xyz=(0.006, 0.255, 0.755)),
        material=powder_coat,
        name="right_yoke_arm",
    )
    column.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(
            xyz=(0.0, -0.245, 0.750),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=warm_gray,
        name="left_bearing_cap",
    )
    column.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.245, 0.750),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=warm_gray,
        name="right_bearing_cap",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.110, 0.560, 0.950)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
    )

    model.articulation(
        "pedestal_to_column",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.10,
            lower=0.0,
            upper=0.180,
        ),
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.015, length=0.460),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_steel,
        name="trunnion_shaft",
    )
    board.visual(
        Box((0.050, 0.040, 0.050)),
        origin=Origin(xyz=(0.026, -0.210, 0.0)),
        material=powder_coat,
        name="left_trunnion_block",
    )
    board.visual(
        Box((0.050, 0.040, 0.050)),
        origin=Origin(xyz=(0.026, 0.210, 0.0)),
        material=powder_coat,
        name="right_trunnion_block",
    )
    board.visual(
        Box((0.620, 0.460, 0.018)),
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        material=board_white,
        name="board_panel",
    )
    board.visual(
        Box((0.120, 0.040, 0.014)),
        origin=Origin(xyz=(0.140, 0.0, -0.016)),
        material=powder_coat,
        name="center_stiffener",
    )
    board.visual(
        Box((0.620, 0.024, 0.026)),
        origin=Origin(xyz=(0.310, -0.218, -0.018)),
        material=powder_coat,
        name="left_edge_rail",
    )
    board.visual(
        Box((0.620, 0.024, 0.026)),
        origin=Origin(xyz=(0.310, 0.218, -0.018)),
        material=powder_coat,
        name="right_edge_rail",
    )
    board.visual(
        Box((0.028, 0.460, 0.030)),
        origin=Origin(xyz=(0.606, 0.0, -0.018)),
        material=powder_coat,
        name="upper_end_rail",
    )
    board.visual(
        Box((0.056, 0.460, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, -0.016)),
        material=maple,
        name="paper_shelf",
    )
    board.visual(
        Box((0.012, 0.460, 0.022)),
        origin=Origin(xyz=(0.056, 0.0, 0.011)),
        material=maple,
        name="paper_fence",
    )
    board.visual(
        Box((0.224, 0.224, 0.010)),
        origin=Origin(xyz=(0.336, 0.0, -0.014)),
        material=powder_coat,
        name="drawer_housing_roof",
    )
    board.visual(
        Box((0.224, 0.012, 0.026)),
        origin=Origin(xyz=(0.336, -0.106, -0.022)),
        material=powder_coat,
        name="left_runner",
    )
    board.visual(
        Box((0.224, 0.012, 0.026)),
        origin=Origin(xyz=(0.336, 0.106, -0.022)),
        material=powder_coat,
        name="right_runner",
    )
    board.visual(
        Box((0.012, 0.224, 0.026)),
        origin=Origin(xyz=(0.218, 0.0, -0.022)),
        material=powder_coat,
        name="drawer_backstop",
    )
    board.inertial = Inertial.from_geometry(
        Box((0.630, 0.470, 0.120)),
        mass=6.2,
        origin=Origin(xyz=(0.315, 0.0, -0.010)),
    )

    board_rest_tilt = 0.44
    model.articulation(
        "column_to_board",
        ArticulationType.REVOLUTE,
        parent=column,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.750), rpy=(0.0, board_rest_tilt, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.80,
            lower=-0.38,
            upper=0.72,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.220, 0.188, 0.006)),
        origin=Origin(xyz=(0.110, 0.0, -0.012)),
        material=maple,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.200, 0.006, 0.032)),
        origin=Origin(xyz=(0.100, -0.097, 0.001)),
        material=maple,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.200, 0.006, 0.032)),
        origin=Origin(xyz=(0.100, 0.097, 0.001)),
        material=maple,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.006, 0.188, 0.032)),
        origin=Origin(xyz=(0.003, 0.0, 0.001)),
        material=maple,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.014, 0.188, 0.036)),
        origin=Origin(xyz=(0.227, 0.0, 0.003)),
        material=powder_coat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(0.239, 0.0, -0.005)),
        material=satin_steel,
        name="handle_stem",
    )
    drawer.visual(
        Box((0.016, 0.074, 0.012)),
        origin=Origin(xyz=(0.253, 0.0, -0.005)),
        material=satin_steel,
        name="handle_bar",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.240, 0.200, 0.060)),
        mass=1.0,
        origin=Origin(xyz=(0.105, 0.0, 0.005)),
    )

    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.230, 0.0, -0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.18,
            lower=0.0,
            upper=0.120,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    column = object_model.get_part("column")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    column_slide = object_model.get_articulation("pedestal_to_column")
    board_tilt = object_model.get_articulation("column_to_board")
    drawer_slide = object_model.get_articulation("board_to_drawer")

    column_upper = column_slide.motion_limits.upper or 0.0
    board_upper = board_tilt.motion_limits.upper or 0.0
    drawer_upper = drawer_slide.motion_limits.upper or 0.0

    with ctx.pose({column_slide: 0.0, drawer_slide: 0.0}):
        ctx.expect_contact(
            column,
            pedestal,
            elem_a="base_stop_collar",
            elem_b="guide_collar",
            name="column rests on the pedestal guide collar at the low position",
        )
        ctx.expect_overlap(
            column,
            pedestal,
            axes="xy",
            elem_a="inner_column",
            elem_b="guide_collar",
            min_overlap=0.050,
            name="column stays centered in the pedestal collar at rest",
        )
        ctx.expect_overlap(
            drawer,
            board,
            axes="x",
            elem_a="drawer_bottom",
            elem_b="drawer_housing_roof",
            min_overlap=0.180,
            name="drawer is deeply nested in its housing at rest",
        )
        ctx.expect_contact(
            drawer,
            board,
            elem_a="drawer_left_side",
            elem_b="left_runner",
            contact_tol=0.0005,
            name="drawer side bears against the left runner at rest",
        )

    with ctx.pose({column_slide: column_upper}):
        ctx.expect_overlap(
            column,
            pedestal,
            axes="xy",
            elem_a="inner_column",
            elem_b="guide_collar",
            min_overlap=0.050,
            name="extended column stays centered in the pedestal collar",
        )
        ctx.expect_overlap(
            column,
            pedestal,
            axes="z",
            elem_a="inner_column",
            elem_b="pedestal_shell",
            min_overlap=0.035,
            name="extended column retains insertion inside the pedestal shell",
        )

    low_column_pos = ctx.part_world_position(column)
    with ctx.pose({column_slide: column_upper}):
        high_column_pos = ctx.part_world_position(column)
    ctx.check(
        "column raises the table when extended",
        low_column_pos is not None
        and high_column_pos is not None
        and high_column_pos[2] > low_column_pos[2] + 0.12,
        details=f"low={low_column_pos}, high={high_column_pos}",
    )

    flat_panel = ctx.part_element_world_aabb(board, elem="board_panel")
    with ctx.pose({board_tilt: board_upper}):
        steep_panel = ctx.part_element_world_aabb(board, elem="board_panel")
    ctx.check(
        "board tilts upward about the top yoke trunnion",
        flat_panel is not None
        and steep_panel is not None
        and steep_panel[1][2] > flat_panel[1][2] + 0.15,
        details=f"flat={flat_panel}, steep={steep_panel}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_upper}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            board,
            axes="x",
            elem_a="drawer_bottom",
            elem_b="drawer_housing_roof",
            min_overlap=0.040,
            name="drawer stays retained on the runners when extended",
        )
        ctx.expect_within(
            drawer,
            board,
            axes="y",
            inner_elem="drawer_front",
            outer_elem="drawer_housing_roof",
            margin=0.010,
            name="drawer remains laterally aligned with the housing",
        )
        ctx.expect_contact(
            drawer,
            board,
            elem_a="drawer_left_side",
            elem_b="left_runner",
            contact_tol=0.0005,
            name="drawer remains guided by the left runner when extended",
        )
    ctx.check(
        "drawer translates outward from beneath the board",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.08
        and extended_drawer_pos[2] < rest_drawer_pos[2] - 0.04,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

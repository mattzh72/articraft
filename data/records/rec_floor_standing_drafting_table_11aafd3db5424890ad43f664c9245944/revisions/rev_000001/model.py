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
    model = ArticulatedObject(name="floor_standing_drafting_table")

    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    board_mat = model.material("matte_green_drafting_board", rgba=(0.36, 0.48, 0.38, 1.0))
    edge_mat = model.material("pale_hardwood_edges", rgba=(0.74, 0.58, 0.36, 1.0))
    drawer_mat = model.material("shallow_gray_tool_drawer", rgba=(0.42, 0.45, 0.47, 1.0))

    frame = model.part("frame")
    # Two floor-standing outer posts with broad feet and cross ties.
    for x in (-0.55, 0.55):
        frame.visual(
            Box((0.16, 0.80, 0.06)),
            origin=Origin(xyz=(x, 0.22, 0.03)),
            material=dark_steel,
            name=f"floor_foot_{'neg' if x < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.06, 0.06, 1.42)),
            origin=Origin(xyz=(x, 0.00, 0.75)),
            material=dark_steel,
            name=f"outer_post_{'neg' if x < 0 else 'pos'}",
        )

    frame.visual(
        Box((1.18, 0.055, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        material=dark_steel,
        name="top_cross_tie",
    )
    frame.visual(
        Box((1.18, 0.055, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_steel,
        name="lower_cross_tie",
    )
    frame.visual(
        Box((0.085, 0.060, 1.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=brushed,
        name="center_guide",
    )

    carriage = model.part("carriage")
    # A rectangular sliding sleeve wraps the center guide with clearance.
    carriage.visual(
        Box((0.034, 0.145, 0.200)),
        origin=Origin(xyz=(-0.0595, 0.0, 0.0)),
        material=brushed,
        name="left_sleeve_plate",
    )
    carriage.visual(
        Box((0.034, 0.145, 0.200)),
        origin=Origin(xyz=(0.0595, 0.0, 0.0)),
        material=brushed,
        name="right_sleeve_plate",
    )
    carriage.visual(
        Box((0.162, 0.024, 0.200)),
        origin=Origin(xyz=(0.0, 0.065, 0.0)),
        material=brushed,
        name="front_sleeve_bridge",
    )
    carriage.visual(
        Box((0.162, 0.024, 0.200)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=brushed,
        name="rear_sleeve_bridge",
    )
    # Twin arms carry the hinge yoke forward from the central slider.
    for x in (-0.065, 0.065):
        carriage.visual(
            Box((0.036, 0.170, 0.055)),
            origin=Origin(xyz=(x, 0.135, 0.025)),
            material=brushed,
            name=f"forward_arm_{'neg' if x < 0 else 'pos'}",
        )
    carriage.visual(
        Box((1.02, 0.050, 0.065)),
        origin=Origin(xyz=(0.0, 0.205, 0.030)),
        material=brushed,
        name="hinge_crosshead",
    )
    carriage.visual(
        Box((0.085, 0.095, 0.070)),
        origin=Origin(xyz=(-0.47, 0.205, 0.060)),
        material=brushed,
        name="hinge_lug_0",
    )
    carriage.visual(
        Box((0.085, 0.095, 0.070)),
        origin=Origin(xyz=(0.47, 0.205, 0.060)),
        material=brushed,
        name="hinge_lug_1",
    )

    board = model.part("board")
    board.visual(
        Box((1.18, 0.80, 0.040)),
        origin=Origin(xyz=(0.0, 0.40, 0.055)),
        material=board_mat,
        name="work_surface",
    )
    # Hardwood perimeter strips make the board read as a real drafting surface.
    board.visual(
        Box((1.24, 0.035, 0.052)),
        origin=Origin(xyz=(0.0, 0.815, 0.054)),
        material=edge_mat,
        name="working_edge_strip",
    )
    board.visual(
        Box((1.24, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, -0.018, 0.052)),
        material=edge_mat,
        name="rear_edge_strip",
    )
    for x in (-0.615, 0.615):
        board.visual(
            Box((0.030, 0.80, 0.050)),
            origin=Origin(xyz=(x, 0.40, 0.052)),
            material=edge_mat,
            name=f"side_edge_{0 if x < 0 else 1}",
        )
    # Pivot shaft and hinge straps attached to the underside of the board.
    board.visual(
        Cylinder(radius=0.026, length=1.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    for x in (-0.34, 0.34):
        board.visual(
            Box((0.115, 0.155, 0.022)),
            origin=Origin(xyz=(x, 0.065, 0.032)),
            material=dark_steel,
            name=f"hinge_strap_{0 if x < 0 else 1}",
        )
    # Drawer slide rails mounted under the working/front edge.
    board.visual(
        Box((0.040, 0.340, 0.040)),
        origin=Origin(xyz=(-0.385, 0.550, -0.020)),
        material=dark_steel,
        name="drawer_rail_0",
    )
    board.visual(
        Box((0.040, 0.340, 0.040)),
        origin=Origin(xyz=(0.385, 0.550, -0.020)),
        material=dark_steel,
        name="drawer_rail_1",
    )
    for x, side in ((-0.385, 0), (0.385, 1)):
        for y in (0.455, 0.700):
            board.visual(
                Box((0.040, 0.035, 0.075)),
                origin=Origin(xyz=(x, y, 0.0175)),
                material=dark_steel,
                name=f"rail_hanger_{side}_{0 if y < 0.6 else 1}",
            )
    board.visual(
        Box((0.78, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.390, -0.018)),
        material=dark_steel,
        name="drawer_rear_stop",
    )

    drawer = model.part("tool_drawer")
    drawer.visual(
        Box((0.70, 0.320, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=drawer_mat,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.045, 0.320, 0.058)),
        origin=Origin(xyz=(-0.365, 0.0, -0.009)),
        material=drawer_mat,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.045, 0.320, 0.058)),
        origin=Origin(xyz=(0.365, 0.0, -0.009)),
        material=drawer_mat,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.76, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.175, -0.003)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.70, 0.024, 0.052)),
        origin=Origin(xyz=(0.0, -0.166, -0.008)),
        material=drawer_mat,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.42, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.212, -0.004)),
        material=dark_steel,
        name="drawer_pull",
    )
    for x in (-0.17, 0.17):
        drawer.visual(
            Box((0.026, 0.050, 0.018)),
            origin=Origin(xyz=(x, 0.194, -0.004)),
            material=dark_steel,
            name=f"pull_stem_{0 if x < 0 else 1}",
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.42),
    )
    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.205, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=1.15),
    )
    model.articulation(
        "board_to_tool_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.590, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    drawer = object_model.get_part("tool_drawer")

    lift = object_model.get_articulation("frame_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    drawer_slide = object_model.get_articulation("board_to_tool_drawer")

    # The visible hinge barrel is intentionally captured inside the two yoke lugs.
    for lug_name in ("hinge_lug_0", "hinge_lug_1"):
        ctx.allow_overlap(
            carriage,
            board,
            elem_a=lug_name,
            elem_b="hinge_barrel",
            reason="The drafting board pivot shaft is intentionally captured in the carriage hinge lug.",
        )
        ctx.expect_overlap(
            board,
            carriage,
            axes="x",
            elem_a="hinge_barrel",
            elem_b=lug_name,
            min_overlap=0.045,
            name=f"{lug_name} captures hinge barrel along the tilt axis",
        )
        ctx.expect_gap(
            board,
            carriage,
            axis="z",
            positive_elem="hinge_barrel",
            negative_elem=lug_name,
            max_penetration=0.030,
            name=f"{lug_name} has only local shaft penetration",
        )

    # The carriage sleeve bears directly on the central guide rail and remains
    # engaged through the height adjustment travel.
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem="center_guide",
        negative_elem="left_sleeve_plate",
        max_gap=0.001,
        max_penetration=1e-6,
        name="left carriage shoe touches the guide rail",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem="right_sleeve_plate",
        negative_elem="center_guide",
        max_gap=0.001,
        max_penetration=1e-6,
        name="right carriage shoe touches the guide rail",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="left_sleeve_plate",
        elem_b="center_guide",
        min_overlap=0.19,
        name="lower carriage remains sleeved around the guide",
    )

    lower_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.42}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="left_sleeve_plate",
            elem_b="center_guide",
            min_overlap=0.19,
            name="raised carriage remains sleeved around the guide",
        )
        raised_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage translates upward along the center guide",
        lower_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > lower_carriage_pos[2] + 0.35,
        details=f"lower={lower_carriage_pos}, raised={raised_carriage_pos}",
    )

    # Drawer slide rails support the shallow tool drawer under the working edge.
    ctx.expect_gap(
        board,
        drawer,
        axis="z",
        positive_elem="drawer_rail_0",
        negative_elem="drawer_side_0",
        max_gap=0.001,
        max_penetration=1e-6,
        name="drawer rides directly under the left rail",
    )
    ctx.expect_gap(
        board,
        drawer,
        axis="z",
        positive_elem="drawer_rail_1",
        negative_elem="drawer_side_1",
        max_gap=0.001,
        max_penetration=1e-6,
        name="drawer rides directly under the right rail",
    )
    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.28}):
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "tool drawer translates outward from the working edge",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > closed_drawer_pos[1] + 0.20,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    flat_edge_aabb = ctx.part_element_world_aabb(board, elem="working_edge_strip")
    with ctx.pose({tilt: 1.15}):
        tilted_edge_aabb = ctx.part_element_world_aabb(board, elem="working_edge_strip")
    flat_edge_z = flat_edge_aabb[1][2] if flat_edge_aabb is not None else None
    tilted_edge_z = tilted_edge_aabb[1][2] if tilted_edge_aabb is not None else None
    ctx.check(
        "board tilts upward about the carriage axis",
        flat_edge_z is not None and tilted_edge_z is not None and tilted_edge_z > flat_edge_z + 0.45,
        details=f"flat_zmax={flat_edge_z}, tilted_zmax={tilted_edge_z}",
    )

    return ctx.report()


object_model = build_object_model()

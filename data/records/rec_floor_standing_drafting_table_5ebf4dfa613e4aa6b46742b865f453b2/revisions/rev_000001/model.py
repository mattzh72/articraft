from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_drafting_table")

    steel = model.material("satin_black_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_steel = model.material("dark_runner_steel", rgba=(0.015, 0.018, 0.02, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    wood = model.material("sealed_beech_edge", rgba=(0.72, 0.54, 0.34, 1.0))
    drawing_vinyl = model.material("warm_off_white_drawing_surface", rgba=(0.92, 0.89, 0.80, 1.0))
    drawer_mat = model.material("shallow_grey_drawer", rgba=(0.36, 0.39, 0.40, 1.0))
    fastener = model.material("brushed_steel_pin", rgba=(0.62, 0.62, 0.58, 1.0))

    stand = model.part("stand")
    # Flat H base: two long low feet joined by a crossbar.
    for y, name in ((-0.42, "foot_0"), (0.42, "foot_1")):
        stand.visual(
            Box((0.98, 0.085, 0.045)),
            origin=Origin(xyz=(0.05, y, 0.0225)),
            material=steel,
            name=name,
        )
        for x, pad_name in ((-0.37, f"{name}_pad_0"), (0.47, f"{name}_pad_1")):
            stand.visual(
                Box((0.13, 0.105, 0.018)),
                origin=Origin(xyz=(x, y, 0.009)),
                material=rubber,
                name=pad_name,
            )
    stand.visual(
        Box((0.14, 0.92, 0.045)),
        origin=Origin(xyz=(-0.20, 0.0, 0.0225)),
        material=steel,
        name="h_crossbar",
    )
    # The visible fixed rail rising from the crossbar.  The sliding carriage rides
    # on the rail's front face and side flanges.
    stand.visual(
        Box((0.080, 0.060, 1.30)),
        origin=Origin(xyz=(-0.25, 0.0, 0.695)),
        material=steel,
        name="vertical_rail",
    )
    stand.visual(
        Box((0.16, 0.16, 0.055)),
        origin=Origin(xyz=(-0.25, 0.0, 0.0675)),
        material=steel,
        name="rail_foot_block",
    )
    stand.visual(
        Box((0.13, 0.11, 0.035)),
        origin=Origin(xyz=(-0.25, 0.0, 1.3625)),
        material=steel,
        name="rail_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.025, 0.24, 0.270)),
        origin=Origin(xyz=(-0.0975, 0.0, -0.025)),
        material=dark_steel,
        name="slide_plate",
    )
    for y, name in ((-0.055, "side_shoe_0"), (0.055, "side_shoe_1")):
        carriage.visual(
            Box((0.100, 0.025, 0.250)),
            origin=Origin(xyz=(-0.145, y, -0.025)),
            material=dark_steel,
            name=name,
        )
    for y, name in ((-0.170, "bracket_bridge_0"), (0.170, "bracket_bridge_1")):
        carriage.visual(
            Box((0.150, 0.100, 0.040)),
            origin=Origin(xyz=(-0.050, y, -0.062)),
            material=dark_steel,
            name=name,
        )
    for y, name in ((-0.190, "tilt_cheek_0"), (0.190, "tilt_cheek_1")):
        carriage.visual(
            Box((0.105, 0.035, 0.125)),
            origin=Origin(xyz=(-0.020, y, 0.0)),
            material=dark_steel,
            name=name,
        )
    carriage.visual(
        Cylinder(radius=0.018, length=0.50),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="tilt_pin",
    )
    for y, name in ((-0.245, "pin_washer_0"), (0.245, "pin_washer_1")):
        carriage.visual(
            Cylinder(radius=0.033, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=fastener,
            name=name,
        )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.032, length=0.300),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="hinge_sleeve",
    )
    board.visual(
        Box((0.080, 0.300, 0.025)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=wood,
        name="hinge_leaf",
    )
    board.visual(
        Box((0.730, 1.200, 0.035)),
        origin=Origin(xyz=(0.465, 0.0, 0.0)),
        material=wood,
        name="board_panel",
    )
    board.visual(
        Box((0.670, 1.080, 0.007)),
        origin=Origin(xyz=(0.500, 0.0, 0.019)),
        material=drawing_vinyl,
        name="drawing_surface",
    )
    board.visual(
        Box((0.055, 1.100, 0.070)),
        origin=Origin(xyz=(0.820, 0.0, 0.045)),
        material=wood,
        name="lower_ledge",
    )
    board.visual(
        Box((0.040, 1.220, 0.045)),
        origin=Origin(xyz=(0.100, 0.0, 0.006)),
        material=wood,
        name="top_frame",
    )
    for y, name in ((-0.605, "side_frame_0"), (0.605, "side_frame_1")):
        board.visual(
            Box((0.725, 0.030, 0.045)),
            origin=Origin(xyz=(0.470, y, 0.006)),
            material=wood,
            name=name,
        )
    for y, name in ((-0.340, "runner_0"), (0.340, "runner_1")):
        board.visual(
            Box((0.380, 0.035, 0.025)),
            origin=Origin(xyz=(0.490, y, -0.0275)),
            material=dark_steel,
            name=name,
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.360, 0.620, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=drawer_mat,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.025, 0.660, 0.070)),
        origin=Origin(xyz=(0.1925, 0.0, 0.010)),
        material=drawer_mat,
        name="front_wall",
    )
    drawer.visual(
        Box((0.025, 0.620, 0.055)),
        origin=Origin(xyz=(-0.1925, 0.0, 0.0025)),
        material=drawer_mat,
        name="back_wall",
    )
    for y, name in ((-0.310, "side_wall_0"), (0.310, "side_wall_1")):
        drawer.visual(
            Box((0.360, 0.025, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.010)),
            material=drawer_mat,
            name=name,
        )
    drawer.visual(
        Cylinder(radius=0.014, length=0.230),
        origin=Origin(xyz=(0.202, 0.0, 0.022), rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="drawer_pull",
    )

    model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(-0.10, 0.0, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.20, lower=-0.20, upper=0.25),
    )
    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(rpy=(0.0, 0.35, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.6, lower=-0.35, upper=0.65),
    )
    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.500, 0.0, -0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")
    lift = object_model.get_articulation("stand_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    slide = object_model.get_articulation("board_to_drawer")

    ctx.allow_overlap(
        carriage,
        board,
        elem_a="tilt_pin",
        elem_b="hinge_sleeve",
        reason="The board hinge sleeve intentionally rotates around the captured steel tilt pin.",
    )
    ctx.expect_within(
        carriage,
        board,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="tilt pin is captured inside the board hinge sleeve",
    )
    ctx.expect_overlap(
        carriage,
        board,
        axes="y",
        elem_a="tilt_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.28,
        name="hinge sleeve has broad engagement along the horizontal pin",
    )

    ctx.expect_contact(
        stand,
        carriage,
        elem_a="vertical_rail",
        elem_b="slide_plate",
        contact_tol=0.002,
        name="carriage slide plate bears on vertical rail",
    )
    ctx.expect_overlap(
        stand,
        carriage,
        axes="z",
        elem_a="vertical_rail",
        elem_b="slide_plate",
        min_overlap=0.22,
        name="carriage remains engaged with vertical rail at rest",
    )
    with ctx.pose({lift: 0.25}):
        ctx.expect_overlap(
            stand,
            carriage,
            axes="z",
            elem_a="vertical_rail",
            elem_b="slide_plate",
            min_overlap=0.10,
            name="raised carriage still retains rail engagement",
        )

    ctx.expect_contact(
        board,
        drawer,
        elem_a="runner_1",
        elem_b="side_wall_1",
        contact_tol=0.002,
        name="drawer side rides on guide runner",
    )
    ctx.expect_overlap(
        board,
        drawer,
        axes="x",
        elem_a="runner_1",
        elem_b="side_wall_1",
        min_overlap=0.30,
        name="closed drawer is carried by short runners",
    )
    with ctx.pose({slide: 0.24}):
        ctx.expect_overlap(
            board,
            drawer,
            axes="x",
            elem_a="runner_1",
            elem_b="side_wall_1",
            min_overlap=0.10,
            name="open drawer remains engaged with guide runners",
        )

    rest_carriage = ctx.part_world_position(carriage)
    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({lift: 0.25, slide: 0.24, tilt: -0.35}):
        raised_carriage = ctx.part_world_position(carriage)
        open_drawer = ctx.part_world_position(drawer)
    ctx.check(
        "carriage prismatic joint raises the board support",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.20,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )
    ctx.check(
        "drawer prismatic joint pulls the tray outward",
        rest_drawer is not None
        and open_drawer is not None
        and open_drawer[0] > rest_drawer[0] + 0.10,
        details=f"rest={rest_drawer}, open={open_drawer}",
    )

    return ctx.report()


object_model = build_object_model()

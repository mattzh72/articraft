from __future__ import annotations

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
    model = ArticulatedObject(name="tall_control_cabinet")

    cabinet_paint = model.material("warm_gray_powder_coat", color=(0.72, 0.73, 0.70, 1.0))
    door_paint = model.material("slightly_lighter_door_paint", color=(0.82, 0.83, 0.79, 1.0))
    hinge_steel = model.material("brushed_hinge_steel", color=(0.40, 0.42, 0.43, 1.0))
    dark_gasket = model.material("black_rubber_gasket", color=(0.02, 0.022, 0.024, 1.0))
    handle_black = model.material("black_polymer_handle", color=(0.015, 0.015, 0.016, 1.0))
    backplate_mat = model.material("galvanized_backplate", color=(0.62, 0.65, 0.66, 1.0))
    component_blue = model.material("blue_control_module", color=(0.05, 0.16, 0.42, 1.0))
    component_orange = model.material("orange_terminal_blocks", color=(0.95, 0.45, 0.08, 1.0))
    label_white = model.material("white_label_cards", color=(0.95, 0.95, 0.90, 1.0))

    width = 0.48
    depth = 0.30
    height = 1.75
    wall = 0.035
    front_y = depth / 2.0

    carcass = model.part("carcass")

    # A tall, narrow sheet-metal enclosure: separate walls make the front visibly hollow.
    carcass.visual(
        Box((width, 0.025, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.0125, height / 2.0)),
        material=cabinet_paint,
        name="back_panel",
    )
    carcass.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=cabinet_paint,
        name="side_wall_0",
    )
    carcass.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=cabinet_paint,
        name="side_wall_1",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=cabinet_paint,
        name="top_wall",
    )
    carcass.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=cabinet_paint,
        name="bottom_wall",
    )

    frame_y = front_y + 0.010
    frame_depth = 0.024
    carcass.visual(
        Box((wall, frame_depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, frame_y, height / 2.0)),
        material=cabinet_paint,
        name="front_jamb_0",
    )
    carcass.visual(
        Box((wall, frame_depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, frame_y, height / 2.0)),
        material=cabinet_paint,
        name="front_jamb_1",
    )
    carcass.visual(
        Box((width, frame_depth, wall)),
        origin=Origin(xyz=(0.0, frame_y, height - wall / 2.0)),
        material=cabinet_paint,
        name="front_header",
    )
    carcass.visual(
        Box((width, frame_depth, wall)),
        origin=Origin(xyz=(0.0, frame_y, wall / 2.0)),
        material=cabinet_paint,
        name="front_sill",
    )
    carcass.visual(
        Box((0.39, 0.012, 1.38)),
        origin=Origin(xyz=(0.025, -0.126, 0.88)),
        material=backplate_mat,
        name="equipment_backplate",
    )
    for z in (0.72, 1.06):
        carcass.visual(
            Box((0.34, 0.012, 0.030)),
            origin=Origin(xyz=(0.035, -0.113, z)),
            material=hinge_steel,
            name=f"din_rail_{int(z * 100)}",
        )
    carcass.visual(
        Box((0.16, 0.030, 0.17)),
        origin=Origin(xyz=(-0.040, -0.095, 1.12)),
        material=component_blue,
        name="control_module",
    )
    carcass.visual(
        Box((0.22, 0.028, 0.065)),
        origin=Origin(xyz=(0.060, -0.094, 0.74)),
        material=component_orange,
        name="terminal_blocks",
    )

    outer_axis_x = -width / 2.0 - 0.015
    outer_axis_y = front_y + 0.037
    outer_radius = 0.017
    # Fixed hinge knuckles and leaves on the cabinet side of the outer door.
    for index, zc, length in ((0, 0.28, 0.24), (1, 0.82, 0.24), (2, 1.35, 0.22)):
        carcass.visual(
            Cylinder(radius=outer_radius, length=length),
            origin=Origin(xyz=(outer_axis_x, outer_axis_y, zc)),
            material=hinge_steel,
            name=f"outer_fixed_knuckle_{index}",
        )
        carcass.visual(
            Box((0.060, 0.008, length * 0.88)),
            origin=Origin(xyz=(outer_axis_x + 0.033, front_y + 0.019, zc)),
            material=hinge_steel,
            name=f"outer_fixed_leaf_{index}",
        )

    # A stout internal hinge rail carries the inner equipment sub-door.
    inner_axis_x = -0.185
    inner_axis_y = 0.103
    inner_axis_z = 0.20
    carcass.visual(
        Box((0.023, 0.040, 1.38)),
        origin=Origin(xyz=(-0.214, 0.088, 0.88)),
        material=cabinet_paint,
        name="inner_hinge_rail",
    )
    for index, zc, length in ((0, 0.39, 0.18), (1, 0.89, 0.26), (2, 1.37, 0.18)):
        carcass.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(xyz=(inner_axis_x, inner_axis_y, zc)),
            material=hinge_steel,
            name=f"inner_fixed_knuckle_{index}",
        )
        carcass.visual(
            Box((0.028, 0.012, length * 0.86)),
            origin=Origin(xyz=(-0.197, 0.094, zc)),
            material=hinge_steel,
            name=f"inner_fixed_leaf_{index}",
        )

    outer_door = model.part("outer_door")
    outer_door.visual(
        Box((0.480, 0.026, 1.70)),
        origin=Origin(xyz=(0.266, 0.000, 0.875)),
        material=door_paint,
        name="door_panel",
    )
    # Proud folded edges and a dark gasket make the closed sheet read as an enclosure door.
    outer_door.visual(
        Box((0.450, 0.010, 0.034)),
        origin=Origin(xyz=(0.286, 0.018, 1.690)),
        material=door_paint,
        name="top_fold",
    )
    outer_door.visual(
        Box((0.450, 0.010, 0.034)),
        origin=Origin(xyz=(0.286, 0.018, 0.060)),
        material=door_paint,
        name="bottom_fold",
    )
    outer_door.visual(
        Box((0.030, 0.010, 1.58)),
        origin=Origin(xyz=(0.050, 0.018, 0.875)),
        material=door_paint,
        name="hinge_side_fold",
    )
    outer_door.visual(
        Box((0.030, 0.010, 1.58)),
        origin=Origin(xyz=(0.500, 0.018, 0.875)),
        material=door_paint,
        name="latch_side_fold",
    )
    outer_door.visual(
        Box((0.405, 0.006, 1.47)),
        origin=Origin(xyz=(0.285, -0.010, 0.875)),
        material=dark_gasket,
        name="rear_gasket",
    )
    for index, zc, length in ((0, 0.55, 0.30), (1, 1.09, 0.30)):
        outer_door.visual(
            Cylinder(radius=0.015, length=length),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_steel,
            name=f"outer_moving_knuckle_{index}",
        )
        outer_door.visual(
            Box((0.070, 0.010, length * 0.82)),
            origin=Origin(xyz=(0.040, 0.010, zc)),
            material=hinge_steel,
            name=f"outer_moving_leaf_{index}",
        )
    outer_door.visual(
        Cylinder(radius=0.012, length=0.55),
        origin=Origin(xyz=(0.455, 0.065, 0.910)),
        material=handle_black,
        name="pull_handle",
    )
    for zc in (0.670, 1.150):
        outer_door.visual(
            Box((0.038, 0.055, 0.045)),
            origin=Origin(xyz=(0.455, 0.039, zc)),
            material=handle_black,
            name=f"handle_standoff_{int(zc * 100)}",
        )

    inner_door = model.part("inner_door")
    inner_door.visual(
        Box((0.320, 0.018, 1.25)),
        origin=Origin(xyz=(0.182, 0.000, 0.675)),
        material=backplate_mat,
        name="subdoor_panel",
    )
    inner_door.visual(
        Box((0.280, 0.008, 0.030)),
        origin=Origin(xyz=(0.190, 0.010, 1.245)),
        material=dark_gasket,
        name="subdoor_top_lip",
    )
    inner_door.visual(
        Box((0.280, 0.008, 0.030)),
        origin=Origin(xyz=(0.190, 0.010, 0.105)),
        material=dark_gasket,
        name="subdoor_bottom_lip",
    )
    inner_door.visual(
        Box((0.085, 0.006, 0.055)),
        origin=Origin(xyz=(0.255, 0.010, 0.970)),
        material=label_white,
        name="equipment_label",
    )
    inner_door.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.292, 0.017, 0.615), rpy=(1.5708, 0.0, 0.0)),
        material=handle_black,
        name="quarter_turn_latch",
    )
    for index, zc, length in ((0, 0.42, 0.28), (1, 0.95, 0.26)):
        inner_door.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_steel,
            name=f"inner_moving_knuckle_{index}",
        )
        inner_door.visual(
            Box((0.050, 0.008, length * 0.82)),
            origin=Origin(xyz=(0.027, 0.010, zc)),
            material=hinge_steel,
            name=f"inner_moving_leaf_{index}",
        )

    model.articulation(
        "outer_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=outer_door,
        origin=Origin(xyz=(outer_axis_x, outer_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "inner_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=inner_door,
        origin=Origin(xyz=(inner_axis_x, inner_axis_y, inner_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    outer_door = object_model.get_part("outer_door")
    inner_door = object_model.get_part("inner_door")
    outer_hinge = object_model.get_articulation("outer_hinge")
    inner_hinge = object_model.get_articulation("inner_hinge")

    ctx.expect_gap(
        outer_door,
        carcass,
        axis="y",
        min_gap=0.0015,
        max_gap=0.010,
        positive_elem="door_panel",
        negative_elem="front_header",
        name="outer door skin sits just proud of the front frame",
    )
    ctx.expect_gap(
        outer_door,
        inner_door,
        axis="y",
        min_gap=0.055,
        positive_elem="door_panel",
        negative_elem="subdoor_panel",
        name="outer door clears the recessed equipment sub-door",
    )
    ctx.expect_within(
        inner_door,
        carcass,
        axes="xz",
        margin=0.010,
        inner_elem="subdoor_panel",
        outer_elem="equipment_backplate",
        name="inner sub-door stays within the cabinet equipment bay",
    )

    closed_outer_aabb = ctx.part_element_world_aabb(outer_door, elem="door_panel")
    with ctx.pose({outer_hinge: 1.20}):
        open_outer_aabb = ctx.part_element_world_aabb(outer_door, elem="door_panel")
    ctx.check(
        "outer door swings outward about its left knuckle line",
        closed_outer_aabb is not None
        and open_outer_aabb is not None
        and open_outer_aabb[1][1] > closed_outer_aabb[1][1] + 0.20,
        details=f"closed={closed_outer_aabb}, open={open_outer_aabb}",
    )

    closed_inner_aabb = ctx.part_element_world_aabb(inner_door, elem="subdoor_panel")
    with ctx.pose({outer_hinge: 1.35, inner_hinge: 1.00}):
        open_inner_aabb = ctx.part_element_world_aabb(inner_door, elem="subdoor_panel")
    ctx.check(
        "inner equipment sub-door swings on its own recessed hinge",
        closed_inner_aabb is not None
        and open_inner_aabb is not None
        and open_inner_aabb[1][1] > closed_inner_aabb[1][1] + 0.12,
        details=f"closed={closed_inner_aabb}, open={open_inner_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

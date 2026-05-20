from __future__ import annotations

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
    model = ArticulatedObject(name="wall_mounted_cabinet")

    cabinet_white = Material("satin_white_lacquer", color=(0.88, 0.86, 0.80, 1.0))
    inner_shadow = Material("warm_shadow_interior", color=(0.62, 0.60, 0.55, 1.0))
    wall_paint = Material("painted_wall", color=(0.78, 0.79, 0.77, 1.0))
    brushed_metal = Material("brushed_aluminum", color=(0.70, 0.70, 0.68, 1.0))
    dark_seam = Material("dark_hinge_shadow", color=(0.06, 0.06, 0.055, 1.0))

    width = 0.50
    height = 0.65
    depth = 0.20
    board = 0.016
    back = 0.012

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.62, 0.025, 0.78)),
        origin=Origin(xyz=(0.0, -0.0115, height / 2.0)),
        material=wall_paint,
        name="wall_panel",
    )
    cabinet.visual(
        Box((width, back, height)),
        origin=Origin(xyz=(0.0, back / 2.0, height / 2.0)),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((board, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + board / 2.0, depth / 2.0, height / 2.0)),
        material=cabinet_white,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((board, depth, height)),
        origin=Origin(xyz=(width / 2.0 - board / 2.0, depth / 2.0, height / 2.0)),
        material=cabinet_white,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, depth, board)),
        origin=Origin(xyz=(0.0, depth / 2.0, board / 2.0)),
        material=cabinet_white,
        name="bottom_board",
    )
    cabinet.visual(
        Box((width, depth, board)),
        origin=Origin(xyz=(0.0, depth / 2.0, height - board / 2.0)),
        material=cabinet_white,
        name="top_board",
    )
    cabinet.visual(
        Box((width - 2.0 * board, depth - back - 0.012, 0.012)),
        origin=Origin(xyz=(0.0, back + (depth - back - 0.012) / 2.0, height * 0.52)),
        material=inner_shadow,
        name="middle_shelf",
    )

    hinge_x = -width / 2.0 - 0.008
    hinge_y = depth + 0.014
    cabinet.visual(
        Box((0.008, 0.024, height * 0.90)),
        origin=Origin(xyz=(hinge_x + 0.004, depth + 0.004, height / 2.0)),
        material=brushed_metal,
        name="hinge_leaf",
    )
    cabinet.visual(
        Cylinder(radius=0.007, length=height * 0.90),
        origin=Origin(xyz=(hinge_x, hinge_y, height / 2.0)),
        material=brushed_metal,
        name="hinge_barrel",
    )

    door = model.part("door")
    door_width = 0.492
    door_height = 0.646
    door_thickness = 0.020
    door_center_x = 0.012 + door_width / 2.0
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_center_x, 0.0, height / 2.0)),
        material=cabinet_white,
        name="door_panel",
    )
    # Subtle raised stiles and rails keep the closed face from reading as a plain slab.
    trim_y = door_thickness / 2.0 + 0.0018
    trim_t = 0.004
    rail_h = 0.032
    stile_w = 0.028
    door.visual(
        Box((door_width - 0.030, trim_t, rail_h)),
        origin=Origin(xyz=(door_center_x, trim_y, 0.040)),
        material=cabinet_white,
        name="lower_rail",
    )
    door.visual(
        Box((door_width - 0.030, trim_t, rail_h)),
        origin=Origin(xyz=(door_center_x, trim_y, height - 0.040)),
        material=cabinet_white,
        name="upper_rail",
    )
    door.visual(
        Box((stile_w, trim_t, door_height - 0.104)),
        origin=Origin(xyz=(0.042, trim_y, height / 2.0)),
        material=cabinet_white,
        name="hinge_stile",
    )
    door.visual(
        Box((stile_w, trim_t, door_height - 0.104)),
        origin=Origin(xyz=(door_center_x + door_width / 2.0 - 0.042, trim_y, height / 2.0)),
        material=cabinet_white,
        name="pull_stile",
    )
    # A slim vertical pull is fixed to the door by two short stand-offs.
    handle_x = door_center_x + door_width / 2.0 - 0.060
    door.visual(
        Cylinder(radius=0.006, length=0.280),
        origin=Origin(xyz=(handle_x, 0.038, height / 2.0)),
        material=brushed_metal,
        name="pull_handle",
    )
    door.visual(
        Box((0.018, 0.026, 0.018)),
        origin=Origin(xyz=(handle_x, 0.022, height / 2.0 + 0.105)),
        material=brushed_metal,
        name="upper_standoff",
    )
    door.visual(
        Box((0.018, 0.026, 0.018)),
        origin=Origin(xyz=(handle_x, 0.022, height / 2.0 - 0.105)),
        material=brushed_metal,
        name="lower_standoff",
    )
    door.visual(
        Box((0.006, 0.004, door_height)),
        origin=Origin(xyz=(0.009, -door_thickness / 2.0 - 0.001, height / 2.0)),
        material=dark_seam,
        name="hinge_shadow",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")

    ctx.check(
        "single clean revolute hinge",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        positive_elem="door_panel",
        negative_elem="top_board",
        min_gap=0.002,
        max_gap=0.006,
        name="closed door sits just proud of cabinet front",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="back_panel",
        min_overlap=0.45,
        name="closed door covers cabinet opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward about hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

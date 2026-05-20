from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="wall_distribution_board")

    housing_paint = model.material("housing_paint", rgba=(0.82, 0.84, 0.82, 1.0))
    door_paint = model.material("door_paint", rgba=(0.90, 0.91, 0.89, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.64, 0.66, 0.68, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    outer_width = 0.32
    outer_height = 0.58
    depth = 0.13
    wall_thickness = 0.018
    back_thickness = 0.003
    front_frame_depth = 0.008

    door_width = 0.304
    door_height = 0.564
    door_skin_thickness = 0.0016
    door_return_depth = 0.012
    stile_width = 0.018
    rail_height = 0.018

    hinge_radius = 0.0055
    housing_knuckle_length = 0.055
    door_knuckle_length = 0.040
    top_housing_knuckle_z = 0.215
    top_door_knuckle_z = 0.160
    bottom_door_knuckle_z = -0.160
    bottom_housing_knuckle_z = -0.215

    hinge_axis_x = depth + 0.007
    hinge_axis_y = -outer_width / 2.0 - 0.001

    latch_y = door_width - 0.040
    latch_plate_x = 0.0085
    latch_plate_thickness = 0.004
    latch_pivot_x = latch_plate_x + latch_plate_thickness / 2.0

    housing = model.part("housing")
    housing.visual(
        Box((back_thickness, outer_width, outer_height)),
        origin=Origin(xyz=(back_thickness / 2.0, 0.0, 0.0)),
        material=housing_paint,
        name="back_panel",
    )

    side_wall_depth = depth - back_thickness
    side_wall_x = back_thickness + side_wall_depth / 2.0
    housing.visual(
        Box((side_wall_depth, wall_thickness, outer_height)),
        origin=Origin(xyz=(side_wall_x, -outer_width / 2.0 + wall_thickness / 2.0, 0.0)),
        material=housing_paint,
        name="left_wall",
    )
    housing.visual(
        Box((side_wall_depth, wall_thickness, outer_height)),
        origin=Origin(xyz=(side_wall_x, outer_width / 2.0 - wall_thickness / 2.0, 0.0)),
        material=housing_paint,
        name="right_wall",
    )

    roof_floor_width = outer_width - 2.0 * wall_thickness
    housing.visual(
        Box((side_wall_depth, roof_floor_width, wall_thickness)),
        origin=Origin(xyz=(side_wall_x, 0.0, outer_height / 2.0 - wall_thickness / 2.0)),
        material=housing_paint,
        name="top_wall",
    )
    housing.visual(
        Box((side_wall_depth, roof_floor_width, wall_thickness)),
        origin=Origin(xyz=(side_wall_x, 0.0, -outer_height / 2.0 + wall_thickness / 2.0)),
        material=housing_paint,
        name="bottom_wall",
    )

    front_frame_x = depth - front_frame_depth / 2.0
    housing.visual(
        Box((front_frame_depth, wall_thickness, outer_height)),
        origin=Origin(xyz=(front_frame_x, -outer_width / 2.0 + wall_thickness / 2.0, 0.0)),
        material=housing_paint,
        name="front_frame_left",
    )
    housing.visual(
        Box((front_frame_depth, wall_thickness, outer_height)),
        origin=Origin(xyz=(front_frame_x, outer_width / 2.0 - wall_thickness / 2.0, 0.0)),
        material=housing_paint,
        name="front_frame_right",
    )
    housing.visual(
        Box((front_frame_depth, roof_floor_width, wall_thickness)),
        origin=Origin(xyz=(front_frame_x, 0.0, outer_height / 2.0 - wall_thickness / 2.0)),
        material=housing_paint,
        name="front_frame_top",
    )
    housing.visual(
        Box((front_frame_depth, roof_floor_width, wall_thickness)),
        origin=Origin(xyz=(front_frame_x, 0.0, -outer_height / 2.0 + wall_thickness / 2.0)),
        material=housing_paint,
        name="front_frame_bottom",
    )

    hinge_leaf_width = 0.012
    hinge_leaf_thickness = 0.015
    hinge_leaf_x = 0.1245
    hinge_leaf_y = hinge_axis_y + hinge_leaf_width / 2.0
    housing.visual(
        Box((hinge_leaf_thickness, hinge_leaf_width, 0.082)),
        origin=Origin(xyz=(hinge_leaf_x, hinge_leaf_y, top_housing_knuckle_z)),
        material=hinge_finish,
        name="hinge_leaf_top",
    )
    housing.visual(
        Box((hinge_leaf_thickness, hinge_leaf_width, 0.082)),
        origin=Origin(xyz=(hinge_leaf_x, hinge_leaf_y, bottom_housing_knuckle_z)),
        material=hinge_finish,
        name="hinge_leaf_bottom",
    )
    housing.visual(
        Cylinder(radius=hinge_radius, length=housing_knuckle_length),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, top_housing_knuckle_z)),
        material=hinge_finish,
        name="hinge_barrel_top",
    )
    housing.visual(
        Cylinder(radius=hinge_radius, length=housing_knuckle_length),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, bottom_housing_knuckle_z)),
        material=hinge_finish,
        name="hinge_barrel_bottom",
    )

    door = model.part("door")
    door.visual(
        Box((door_skin_thickness, door_width - 0.004, door_height)),
        origin=Origin(xyz=(0.0065, (door_width - 0.004) / 2.0 + 0.002, 0.0)),
        material=door_paint,
        name="door_outer_skin",
    )
    door.visual(
        Box((door_return_depth, stile_width, 0.115)),
        origin=Origin(xyz=(door_return_depth / 2.0 - 0.0055, stile_width / 2.0, 0.125)),
        material=door_paint,
        name="door_hinge_return_upper",
    )
    door.visual(
        Box((door_return_depth, stile_width, 0.135)),
        origin=Origin(xyz=(door_return_depth / 2.0 - 0.0055, stile_width / 2.0, 0.0)),
        material=door_paint,
        name="door_hinge_return_middle",
    )
    door.visual(
        Box((door_return_depth, stile_width, 0.115)),
        origin=Origin(xyz=(door_return_depth / 2.0 - 0.0055, stile_width / 2.0, -0.125)),
        material=door_paint,
        name="door_hinge_return_lower",
    )
    door.visual(
        Box((door_return_depth, stile_width, door_height)),
        origin=Origin(
            xyz=(door_return_depth / 2.0 - 0.0055, door_width - stile_width / 2.0, 0.0)
        ),
        material=door_paint,
        name="door_lock_return",
    )
    door.visual(
        Box((door_return_depth, door_width - 2.0 * stile_width, rail_height)),
        origin=Origin(
            xyz=(
                door_return_depth / 2.0 - 0.0055,
                door_width / 2.0,
                door_height / 2.0 - rail_height / 2.0,
            )
        ),
        material=door_paint,
        name="door_top_return",
    )
    door.visual(
        Box((door_return_depth, door_width - 2.0 * stile_width, rail_height)),
        origin=Origin(
            xyz=(
                door_return_depth / 2.0 - 0.0055,
                door_width / 2.0,
                -door_height / 2.0 + rail_height / 2.0,
            )
        ),
        material=door_paint,
        name="door_bottom_return",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=door_knuckle_length),
        origin=Origin(xyz=(0.0, 0.0, top_door_knuckle_z)),
        material=hinge_finish,
        name="door_knuckle_top",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=door_knuckle_length),
        origin=Origin(xyz=(0.0, 0.0, bottom_door_knuckle_z)),
        material=hinge_finish,
        name="door_knuckle_bottom",
    )
    door.visual(
        Box((latch_plate_thickness, 0.034, 0.105)),
        origin=Origin(xyz=(latch_plate_x, latch_y, 0.0)),
        material=hinge_finish,
        name="door_latch_escutcheon",
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=2.2),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_finish,
        name="latch_hub",
    )
    latch_handle.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_finish,
        name="latch_spindle",
    )
    latch_handle.visual(
        Box((0.020, 0.010, 0.088)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=latch_finish,
        name="latch_paddle",
    )

    model.articulation(
        "door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_handle,
        origin=Origin(xyz=(latch_pivot_x, latch_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=pi / 2.0),
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

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    latch_handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("housing_to_door")
    latch_joint = object_model.get_articulation("door_to_latch_handle")

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            door,
            housing,
            axis="x",
            positive_elem="door_outer_skin",
            negative_elem="front_frame_right",
            min_gap=0.010,
            max_gap=0.020,
            name="closed door sits slightly proud of the front frame",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="yz",
            elem_a="door_outer_skin",
            elem_b="back_panel",
            min_overlap=0.28,
            name="door leaf covers the rectangular housing face",
        )
        ctx.expect_overlap(
            latch_handle,
            door,
            axes="yz",
            elem_a="latch_hub",
            elem_b="door_latch_escutcheon",
            min_overlap=0.02,
            name="latch handle stays mounted on the door stile",
        )

        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_outer_skin")
        closed_paddle_aabb = ctx.part_element_world_aabb(latch_handle, elem="latch_paddle")

    with ctx.pose({door_hinge: 1.2, latch_joint: 0.0}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_outer_skin")

    ctx.check(
        "door swings outward from the left hinge line",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.16,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_hinge: 0.0, latch_joint: pi / 2.0}):
        turned_paddle_aabb = ctx.part_element_world_aabb(latch_handle, elem="latch_paddle")

    def _extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    ctx.check(
        "quarter-turn latch rotates the paddle from vertical toward horizontal",
        closed_paddle_aabb is not None
        and turned_paddle_aabb is not None
        and _extent(closed_paddle_aabb, 2) > _extent(turned_paddle_aabb, 2) + 0.05
        and _extent(turned_paddle_aabb, 1) > _extent(closed_paddle_aabb, 1) + 0.05,
        details=f"closed={closed_paddle_aabb}, turned={turned_paddle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

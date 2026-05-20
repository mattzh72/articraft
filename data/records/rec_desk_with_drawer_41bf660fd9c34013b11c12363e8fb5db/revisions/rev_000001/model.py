from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dorm_loft_desk")

    wood = model.material("laminate_wood", rgba=(0.72, 0.60, 0.46, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.84, 0.84, 0.82, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.45, 0.47, 0.50, 1.0))
    pull_dark = model.material("pull_dark", rgba=(0.18, 0.18, 0.20, 1.0))

    desk_width = 1.22
    desk_depth = 0.65
    desk_height = 0.76
    top_thickness = 0.03
    side_thickness = 0.03
    side_depth = 0.62
    support_height = desk_height - top_thickness

    hutch_width = 0.78
    hutch_depth = 0.325
    hutch_panel = 0.018
    hutch_bottom_thickness = 0.018
    hutch_bottom_z = 0.39
    hutch_top_z = desk_height - top_thickness
    hutch_height = hutch_top_z - hutch_bottom_z
    hutch_y_center = -desk_depth / 2.0 + hutch_depth / 2.0

    left_hutch_inner_x = -hutch_width / 2.0 + hutch_panel
    right_hutch_inner_x = hutch_width / 2.0 - hutch_panel
    divider_half = hutch_panel / 2.0
    left_bay_width = 0.363
    right_bay_width = 0.363

    desk = model.part("desk_frame")
    desk.visual(
        Box((desk_width, desk_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=wood,
        name="desktop",
    )
    desk.visual(
        Box((side_thickness, side_depth, support_height)),
        origin=Origin(
            xyz=(
                -desk_width / 2.0 + side_thickness / 2.0,
                0.0,
                support_height / 2.0,
            )
        ),
        material=panel_gray,
        name="left_end_panel",
    )
    desk.visual(
        Box((side_thickness, side_depth, support_height)),
        origin=Origin(
            xyz=(
                desk_width / 2.0 - side_thickness / 2.0,
                0.0,
                support_height / 2.0,
            )
        ),
        material=panel_gray,
        name="right_end_panel",
    )
    desk.visual(
        Box((desk_width - 2.0 * side_thickness, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, desk_depth / 2.0 - 0.03, desk_height - 0.09)),
        material=panel_gray,
        name="front_stretcher",
    )
    desk.visual(
        Box((desk_width - 2.0 * side_thickness, 0.02, 0.32)),
        origin=Origin(xyz=(0.0, -desk_depth / 2.0 + 0.01, 0.26)),
        material=panel_gray,
        name="rear_modesty_panel",
    )
    desk.visual(
        Box((desk_width - 2.0 * side_thickness, 0.035, 0.09)),
        origin=Origin(xyz=(0.0, -desk_depth / 2.0 + 0.035, 0.08)),
        material=panel_gray,
        name="rear_lower_stretcher",
    )

    desk.visual(
        Box((hutch_panel, hutch_depth, hutch_height)),
        origin=Origin(
            xyz=(
                -hutch_width / 2.0 + hutch_panel / 2.0,
                hutch_y_center,
                hutch_bottom_z + hutch_height / 2.0,
            )
        ),
        material=panel_gray,
        name="hutch_left_side",
    )
    desk.visual(
        Box((hutch_panel, hutch_depth, hutch_height)),
        origin=Origin(
            xyz=(
                hutch_width / 2.0 - hutch_panel / 2.0,
                hutch_y_center,
                hutch_bottom_z + hutch_height / 2.0,
            )
        ),
        material=panel_gray,
        name="hutch_right_side",
    )
    desk.visual(
        Box((hutch_panel, hutch_depth, hutch_height)),
        origin=Origin(
            xyz=(0.0, hutch_y_center, hutch_bottom_z + hutch_height / 2.0)
        ),
        material=panel_gray,
        name="hutch_divider",
    )
    desk.visual(
        Box((hutch_width, hutch_depth, hutch_bottom_thickness)),
        origin=Origin(
            xyz=(0.0, hutch_y_center, hutch_bottom_z + hutch_bottom_thickness / 2.0)
        ),
        material=panel_gray,
        name="hutch_bottom_panel",
    )
    desk.visual(
        Box((hutch_width, hutch_panel, hutch_height)),
        origin=Origin(
            xyz=(
                0.0,
                -desk_depth / 2.0 + hutch_panel / 2.0,
                hutch_bottom_z + hutch_height / 2.0,
            )
        ),
        material=panel_gray,
        name="hutch_back_panel",
    )

    drawer_shelf_z = 0.548
    desk.visual(
        Box((right_bay_width, hutch_depth, hutch_panel)),
        origin=Origin(
            xyz=(
                (divider_half + right_hutch_inner_x) / 2.0,
                hutch_y_center,
                drawer_shelf_z,
            )
        ),
        material=panel_gray,
        name="drawer_shelf",
    )
    desk.visual(
        Box((0.012, 0.012, 0.318)),
        origin=Origin(xyz=(-0.015, -0.006, 0.567)),
        material=panel_gray,
        name="door_stop_stile",
    )

    rail_length = 0.25
    rail_thickness = 0.014
    rail_height = 0.012
    rail_y_center = -0.175
    rail_z_center = 0.549
    desk.visual(
        Box((rail_thickness, rail_length, rail_height)),
        origin=Origin(
            xyz=(divider_half + rail_thickness / 2.0, rail_y_center, rail_z_center)
        ),
        material=rail_metal,
        name="drawer_left_rail",
    )
    desk.visual(
        Box((rail_thickness, rail_length, rail_height)),
        origin=Origin(
            xyz=(
                right_hutch_inner_x - rail_thickness / 2.0,
                rail_y_center,
                rail_z_center,
            )
        ),
        material=rail_metal,
        name="drawer_right_rail",
    )

    door = model.part("hutch_door")
    door_width = 0.355
    door_height = 0.314
    door_thickness = 0.016
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, door_height / 2.0)),
        material=wood,
        name="door_panel",
    )
    door.visual(
        Box((0.016, 0.022, 0.11)),
        origin=Origin(xyz=(door_width - 0.05, 0.014, door_height / 2.0)),
        material=pull_dark,
        name="door_pull",
    )

    drawer = model.part("drawer")
    drawer_face_width = 0.359
    drawer_face_height = 0.14
    drawer_face_thickness = 0.018
    drawer_body_width = 0.351
    drawer_body_height = 0.10
    drawer_body_length = 0.27
    drawer_wall = 0.012
    drawer_bottom = 0.010
    drawer_body_z = -0.02
    drawer_y_center = -drawer_body_length / 2.0 + drawer_face_thickness / 2.0

    drawer.visual(
        Box((drawer_face_width, drawer_face_thickness, drawer_face_height)),
        origin=Origin(xyz=(0.0, drawer_face_thickness / 2.0, 0.0)),
        material=wood,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_wall, drawer_body_length, drawer_body_height)),
        origin=Origin(
            xyz=(
                -drawer_body_width / 2.0 + drawer_wall / 2.0,
                drawer_y_center,
                drawer_body_z,
            )
        ),
        material=panel_gray,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_wall, drawer_body_length, drawer_body_height)),
        origin=Origin(
            xyz=(
                drawer_body_width / 2.0 - drawer_wall / 2.0,
                drawer_y_center,
                drawer_body_z,
            )
        ),
        material=panel_gray,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((drawer_body_width, drawer_wall, drawer_body_height)),
        origin=Origin(
            xyz=(
                0.0,
                -drawer_body_length + drawer_face_thickness / 2.0 + drawer_wall / 2.0,
                drawer_body_z,
            )
        ),
        material=panel_gray,
        name="drawer_back",
    )
    drawer.visual(
        Box((drawer_body_width - 2.0 * drawer_wall, drawer_body_length, drawer_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                drawer_y_center,
                drawer_body_z - drawer_body_height / 2.0 + drawer_bottom / 2.0,
            )
        ),
        material=panel_gray,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.12, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=pull_dark,
        name="drawer_pull",
    )

    model.articulation(
        "hutch_door_hinge",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=door,
        origin=Origin(xyz=(-0.377, 0.008, 0.412)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(0.195, 0.0, 0.628)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.35,
            lower=0.0,
            upper=0.17,
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
    desk = object_model.get_part("desk_frame")
    door = object_model.get_part("hutch_door")
    drawer = object_model.get_part("drawer")
    door_hinge = object_model.get_articulation("hutch_door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.check(
        "required parts exist",
        desk is not None and door is not None and drawer is not None,
        details=f"desk={desk}, door={door}, drawer={drawer}",
    )

    ctx.expect_gap(
        door,
        desk,
        axis="y",
        positive_elem="door_panel",
        negative_elem="door_stop_stile",
        min_gap=0.0,
        max_gap=0.01,
        name="closed door seats against the cabinet stop",
    )
    ctx.expect_gap(
        drawer,
        desk,
        axis="z",
        positive_elem="drawer_bottom",
        negative_elem="drawer_shelf",
        min_gap=0.0005,
        max_gap=0.01,
        name="drawer clears the support shelf",
    )
    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a="drawer_left_side",
        elem_b="drawer_left_rail",
        min_overlap=0.18,
        name="closed drawer sits fully on its left guide rail",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.4}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door opens outward on its vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.17}):
        open_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="drawer_left_side",
            elem_b="drawer_left_rail",
            min_overlap=0.03,
            name="drawer retains insertion at full extension",
        )
    ctx.check(
        "drawer slides forward along the guide rails",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] > closed_drawer_pos[1] + 0.12,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_by_side_refrigerator")

    width = 0.92
    depth = 0.72
    height = 1.78

    side_wall = 0.03
    top_wall = 0.03
    bottom_wall = 0.05
    back_wall = 0.02
    mullion = 0.02

    hinge_offset = 0.008
    hinge_radius = 0.008
    hinge_len = 0.02
    door_y_offset = hinge_radius

    center_gap = 0.004
    door_thickness = 0.085
    door_bottom_gap = 0.01
    door_height = height - 2.0 * door_bottom_gap
    door_width = width / 2.0 + hinge_offset - center_gap / 2.0

    recess_width = 0.16
    recess_height = 0.34
    freezer_back_depth = 0.022
    freezer_front_skin_depth = door_thickness - freezer_back_depth
    recess_x_center = door_width * 0.61
    recess_x0 = recess_x_center - recess_width / 2.0
    recess_x1 = recess_x_center + recess_width / 2.0
    recess_z0 = 0.88
    recess_z1 = recess_z0 + recess_height

    paddle_width = 0.085
    paddle_height = 0.082
    paddle_thickness = 0.012
    paddle_barrel_len = 0.03
    paddle_barrel_radius = 0.004
    paddle_mount_y = door_y_offset + freezer_back_depth + 0.024
    paddle_pivot_x = recess_x_center
    paddle_pivot_y = paddle_mount_y
    paddle_pivot_z = recess_z0 + recess_height * 0.62
    paddle_plate_y = 0.016
    paddle_plate_top_drop = 0.003
    paddle_connector_x = 0.018

    stainless = model.material("stainless", color=(0.74, 0.76, 0.79))
    dark_trim = model.material("dark_trim", color=(0.14, 0.15, 0.17))
    black = model.material("black", color=(0.07, 0.07, 0.08))
    cabinet_back = model.material("cabinet_back", color=(0.55, 0.57, 0.60))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_wall / 2.0, depth / 2.0, height / 2.0)),
        material=stainless,
        name="left_wall",
    )
    cabinet.visual(
        Box((side_wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_wall / 2.0, depth / 2.0, height / 2.0)),
        material=stainless,
        name="right_wall",
    )
    cabinet.visual(
        Box((width - 2.0 * side_wall, depth, top_wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, height - top_wall / 2.0)),
        material=stainless,
        name="top_shell",
    )
    cabinet.visual(
        Box((width - 2.0 * side_wall, depth, bottom_wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, bottom_wall / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((width - 2.0 * side_wall, back_wall, height - top_wall - bottom_wall)),
        origin=Origin(
            xyz=(
                0.0,
                back_wall / 2.0,
                bottom_wall + (height - top_wall - bottom_wall) / 2.0,
            )
        ),
        material=cabinet_back,
        name="back_panel",
    )
    cabinet.visual(
        Box((mullion, depth, height - top_wall - bottom_wall)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0,
                bottom_wall + (height - top_wall - bottom_wall) / 2.0,
            )
        ),
        material=stainless,
        name="center_mullion",
    )

    left_hinge_x = -width / 2.0 - hinge_offset
    right_hinge_x = width / 2.0 + hinge_offset
    lower_door_barrel_center = 0.03
    lower_cabinet_barrel_center = 0.05
    upper_door_barrel_center = height - 0.03
    upper_cabinet_barrel_center = height - 0.01

    for side_name, hinge_x in (("left", left_hinge_x), ("right", right_hinge_x)):
        cabinet.visual(
            Cylinder(radius=hinge_radius, length=hinge_len),
            origin=Origin(xyz=(hinge_x, depth - hinge_radius, lower_cabinet_barrel_center)),
            material=dark_trim,
            name=f"{side_name}_lower_hinge_barrel",
        )
        cabinet.visual(
            Cylinder(radius=hinge_radius, length=hinge_len),
            origin=Origin(xyz=(hinge_x, depth - hinge_radius, upper_cabinet_barrel_center)),
            material=dark_trim,
            name=f"{side_name}_upper_hinge_barrel",
        )

    freezer_door = model.part("freezer_door")
    freezer_door.visual(
        Box((door_width, freezer_back_depth, door_height)),
        origin=Origin(
            xyz=(
                door_width / 2.0,
                door_y_offset + freezer_back_depth / 2.0,
                door_bottom_gap + door_height / 2.0,
            )
        ),
        material=stainless,
        name="freezer_core",
    )

    lower_skin_height = recess_z0 - door_bottom_gap
    upper_skin_height = door_bottom_gap + door_height - recess_z1
    left_skin_width = recess_x0
    right_skin_width = door_width - recess_x1
    skin_center_y = freezer_back_depth + freezer_front_skin_depth / 2.0

    freezer_door.visual(
        Box((door_width, freezer_front_skin_depth, lower_skin_height)),
        origin=Origin(
            xyz=(
                door_width / 2.0,
                door_y_offset + skin_center_y,
                door_bottom_gap + lower_skin_height / 2.0,
            )
        ),
        material=stainless,
        name="freezer_lower_skin",
    )
    freezer_door.visual(
        Box((door_width, freezer_front_skin_depth, upper_skin_height)),
        origin=Origin(
            xyz=(door_width / 2.0, door_y_offset + skin_center_y, recess_z1 + upper_skin_height / 2.0)
        ),
        material=stainless,
        name="freezer_upper_skin",
    )
    freezer_door.visual(
        Box((left_skin_width, freezer_front_skin_depth, recess_height)),
        origin=Origin(
            xyz=(left_skin_width / 2.0, door_y_offset + skin_center_y, recess_z0 + recess_height / 2.0)
        ),
        material=stainless,
        name="freezer_left_skin",
    )
    freezer_door.visual(
        Box((right_skin_width, freezer_front_skin_depth, recess_height)),
        origin=Origin(
            xyz=(
                recess_x1 + right_skin_width / 2.0,
                door_y_offset + skin_center_y,
                recess_z0 + recess_height / 2.0,
            )
        ),
        material=stainless,
        name="freezer_right_skin",
    )
    freezer_door.visual(
        Box((recess_width - 0.016, 0.003, recess_height - 0.018)),
        origin=Origin(
            xyz=(
                recess_x_center,
                door_y_offset + freezer_back_depth - 0.0015,
                recess_z0 + recess_height / 2.0,
            )
        ),
        material=dark_trim,
        name="dispenser_back_panel",
    )
    freezer_door.visual(
        Cylinder(radius=hinge_radius, length=hinge_len),
        origin=Origin(xyz=(0.0, 0.0, lower_door_barrel_center)),
        material=dark_trim,
        name="freezer_lower_hinge_barrel",
    )
    freezer_door.visual(
        Cylinder(radius=hinge_radius, length=hinge_len),
        origin=Origin(xyz=(0.0, 0.0, upper_door_barrel_center)),
        material=dark_trim,
        name="freezer_upper_hinge_barrel",
    )
    freezer_door.visual(
        Cylinder(radius=paddle_barrel_radius, length=paddle_barrel_len),
        origin=Origin(
            xyz=(
                paddle_pivot_x - paddle_barrel_len / 2.0,
                paddle_pivot_y,
                paddle_pivot_z,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="paddle_support_barrel",
    )
    freezer_door.visual(
        Box(
            (
                0.026,
                paddle_pivot_y - paddle_barrel_radius - (door_y_offset + freezer_back_depth),
                0.016,
            )
        ),
        origin=Origin(
            xyz=(
                paddle_pivot_x - paddle_barrel_len / 2.0,
                (
                    paddle_pivot_y
                    - paddle_barrel_radius
                    + door_y_offset
                    + freezer_back_depth
                )
                / 2.0,
                paddle_pivot_z + 0.008,
            )
        ),
        material=dark_trim,
        name="paddle_support_web",
    )

    fridge_door = model.part("fridge_door")
    fridge_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                -door_width / 2.0,
                door_y_offset + door_thickness / 2.0,
                door_bottom_gap + door_height / 2.0,
            )
        ),
        material=stainless,
        name="fresh_food_panel",
    )
    fridge_door.visual(
        Cylinder(radius=hinge_radius, length=hinge_len),
        origin=Origin(xyz=(0.0, 0.0, lower_door_barrel_center)),
        material=dark_trim,
        name="fridge_lower_hinge_barrel",
    )
    fridge_door.visual(
        Cylinder(radius=hinge_radius, length=hinge_len),
        origin=Origin(xyz=(0.0, 0.0, upper_door_barrel_center)),
        material=dark_trim,
        name="fridge_upper_hinge_barrel",
    )

    dispenser_paddle = model.part("dispenser_paddle")
    dispenser_paddle.visual(
        Cylinder(radius=paddle_barrel_radius, length=paddle_barrel_len),
        origin=Origin(
            xyz=(paddle_barrel_len / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="paddle_hinge_barrel",
    )
    dispenser_paddle.visual(
        Box((paddle_width, paddle_thickness, paddle_height)),
        origin=Origin(
            xyz=(0.0, paddle_plate_y, -(paddle_plate_top_drop + paddle_height / 2.0))
        ),
        material=black,
        name="paddle_plate",
    )
    dispenser_paddle.visual(
        Box((0.018, 0.016, 0.014)),
        origin=Origin(xyz=(paddle_connector_x, 0.008, -0.007)),
        material=black,
        name="paddle_connector",
    )
    dispenser_paddle.visual(
        Box((paddle_width * 0.78, paddle_thickness * 0.8, 0.016)),
        origin=Origin(
            xyz=(
                0.0,
                paddle_plate_y,
                -(paddle_plate_top_drop + paddle_height - 0.008),
            )
        ),
        material=dark_trim,
        name="paddle_pad",
    )

    left_door_hinge = model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(left_hinge_x, depth - hinge_radius, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=1.92,
        ),
    )
    right_door_hinge = model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=fridge_door,
        origin=Origin(xyz=(right_hinge_x, depth - hinge_radius, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-1.92,
            upper=0.0,
        ),
    )
    model.articulation(
        "paddle_hinge",
        ArticulationType.REVOLUTE,
        parent=freezer_door,
        child=dispenser_paddle,
        origin=Origin(xyz=(paddle_pivot_x, paddle_pivot_y, paddle_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.0,
        ),
    )

    cabinet.meta["dimensions_m"] = (width, depth, height)
    freezer_door.meta["door_panel_size_m"] = (door_width, door_thickness, door_height)
    fridge_door.meta["door_panel_size_m"] = (door_width, door_thickness, door_height)
    left_door_hinge.meta["description"] = "Freezer door swings from the left exterior edge."
    right_door_hinge.meta["description"] = "Fresh-food door swings from the right exterior edge."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    freezer_door = object_model.get_part("freezer_door")
    fridge_door = object_model.get_part("fridge_door")
    dispenser_paddle = object_model.get_part("dispenser_paddle")

    left_door_hinge = object_model.get_articulation("left_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")
    paddle_hinge = object_model.get_articulation("paddle_hinge")

    freezer_core = freezer_door.get_visual("freezer_core")
    fresh_food_panel = fridge_door.get_visual("fresh_food_panel")
    paddle_support_barrel = freezer_door.get_visual("paddle_support_barrel")
    paddle_hinge_barrel = dispenser_paddle.get_visual("paddle_hinge_barrel")

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

    ctx.check(
        "left_door_hinge_axis_is_vertical",
        tuple(left_door_hinge.axis) == (0.0, 0.0, 1.0),
        f"expected left door hinge axis (0, 0, 1), got {left_door_hinge.axis}",
    )
    ctx.check(
        "right_door_hinge_axis_is_vertical",
        tuple(right_door_hinge.axis) == (0.0, 0.0, 1.0),
        f"expected right door hinge axis (0, 0, 1), got {right_door_hinge.axis}",
    )
    ctx.check(
        "paddle_hinge_axis_is_horizontal",
        tuple(paddle_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected paddle hinge axis (1, 0, 0), got {paddle_hinge.axis}",
    )

    with ctx.pose({left_door_hinge: 0.0, right_door_hinge: 0.0, paddle_hinge: 0.0}):
        ctx.expect_gap(
            freezer_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=freezer_core,
            name="freezer_door_closed_against_cabinet",
        )
        ctx.expect_gap(
            fridge_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=fresh_food_panel,
            name="fridge_door_closed_against_cabinet",
        )
        ctx.expect_overlap(
            freezer_door,
            cabinet,
            axes="xz",
            min_overlap=0.25,
            elem_a=freezer_core,
            name="freezer_door_covers_left_opening",
        )
        ctx.expect_overlap(
            fridge_door,
            cabinet,
            axes="xz",
            min_overlap=0.25,
            elem_a=fresh_food_panel,
            name="fridge_door_covers_right_opening",
        )
        ctx.expect_gap(
            fridge_door,
            freezer_door,
            axis="x",
            min_gap=0.003,
            max_gap=0.005,
            positive_elem=fresh_food_panel,
            negative_elem=freezer_core,
            name="center_seam_gap",
        )
        ctx.expect_contact(
            dispenser_paddle,
            freezer_door,
            elem_a=paddle_hinge_barrel,
            elem_b=paddle_support_barrel,
            name="paddle_hinge_barrels_touch",
        )

    left_limits = left_door_hinge.motion_limits
    if left_limits is not None and left_limits.lower is not None and left_limits.upper is not None:
        with ctx.pose({left_door_hinge: left_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_lower_no_floating")
        with ctx.pose({left_door_hinge: left_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_upper_no_floating")

    right_limits = right_door_hinge.motion_limits
    if right_limits is not None and right_limits.lower is not None and right_limits.upper is not None:
        with ctx.pose({right_door_hinge: right_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_lower_no_floating")
        with ctx.pose({right_door_hinge: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_upper_no_floating")

    paddle_limits = paddle_hinge.motion_limits
    if paddle_limits is not None and paddle_limits.lower is not None and paddle_limits.upper is not None:
        with ctx.pose({paddle_hinge: paddle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="paddle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="paddle_lower_no_floating")
            paddle_aabb = ctx.part_world_aabb(dispenser_paddle)
            door_aabb = ctx.part_world_aabb(freezer_door)
            ctx.check(
                "paddle_pressed_stays_within_door_depth",
                paddle_aabb is not None
                and door_aabb is not None
                and paddle_aabb[1][1] <= door_aabb[1][1] + 0.001,
                f"pressed paddle extends past door front: paddle={paddle_aabb}, door={door_aabb}",
            )
        with ctx.pose({paddle_hinge: paddle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="paddle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="paddle_upper_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

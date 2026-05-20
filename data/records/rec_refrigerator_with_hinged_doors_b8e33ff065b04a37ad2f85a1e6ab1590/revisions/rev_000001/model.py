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
    model = ArticulatedObject(name="bottom_freezer_french_door_refrigerator")

    # Overall proportions for a full-size bottom-freezer French-door refrigerator.
    width = 0.91
    depth = 0.72
    height = 1.78

    wall = 0.025
    back_wall = 0.02
    top_thickness = 0.03
    divider_thickness = 0.03
    plinth_height = 0.10
    door_thickness = 0.055
    drawer_front_thickness = 0.055

    freezer_opening_height = 0.36
    fresh_food_base_z = plinth_height + freezer_opening_height + divider_thickness
    fresh_food_height = height - fresh_food_base_z - top_thickness

    door_gap_total = 0.018
    side_clearance_each = 0.004
    door_width = (width - door_gap_total) * 0.5
    handle_bar_width = 0.022
    handle_bar_depth = 0.026
    handle_standoff_depth = 0.014
    handle_mount_height = 0.08
    handle_center_inset = 0.050
    main_handle_height = 0.92

    hatch_width = 0.25
    hatch_height = 0.36
    hatch_thickness = 0.024
    hatch_from_free_edge = 0.07
    hatch_from_top = 0.24

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    side_gray = model.material("side_gray", rgba=(0.64, 0.66, 0.70, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    liner = model.material("liner", rgba=(0.92, 0.94, 0.96, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.18, 1.0))

    cabinet = model.part("cabinet")

    # Side walls
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width * 0.5 + wall * 0.5, 0.0, height * 0.5)),
        material=side_gray,
        name="left_side",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width * 0.5 - wall * 0.5, 0.0, height * 0.5)),
        material=side_gray,
        name="right_side",
    )
    # Back shell
    cabinet.visual(
        Box((width - 2.0 * wall, back_wall, height - plinth_height * 0.15)),
        origin=Origin(
            xyz=(
                0.0,
                -depth * 0.5 + back_wall * 0.5,
                (height - plinth_height * 0.15) * 0.5 + plinth_height * 0.15,
            )
        ),
        material=liner,
        name="back_shell",
    )
    # Top panel
    cabinet.visual(
        Box((width - 2.0 * wall, depth - back_wall, top_thickness)),
        origin=Origin(
            xyz=(0.0, back_wall * 0.5, height - top_thickness * 0.5)
        ),
        material=liner,
        name="top_panel",
    )
    # Fresh food / freezer divider deck
    cabinet.visual(
        Box((width - 2.0 * wall, depth - back_wall, divider_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                back_wall * 0.5,
                plinth_height + freezer_opening_height + divider_thickness * 0.5,
            )
        ),
        material=liner,
        name="divider_deck",
    )
    # Freezer floor
    cabinet.visual(
        Box((width - 2.0 * wall, depth - back_wall, wall)),
        origin=Origin(
            xyz=(0.0, back_wall * 0.5, plinth_height - wall * 0.5)
        ),
        material=liner,
        name="freezer_floor",
    )
    # Recessed toe-kick / plinth front
    cabinet.visual(
        Box((width - 2.0 * wall, 0.055, plinth_height)),
        origin=Origin(
            xyz=(0.0, depth * 0.5 - 0.055 * 0.5 - 0.020, plinth_height * 0.5)
        ),
        material=dark_trim,
        name="toe_kick",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, fresh_food_height)),
        origin=Origin(
            xyz=(door_width * 0.5, door_thickness * 0.5, fresh_food_height * 0.5)
        ),
        material=stainless,
        name="left_door_shell",
    )
    # Slightly inset inner liner so the doors read as refrigerator doors when open.
    left_door.visual(
        Box((door_width - 0.05, 0.018, fresh_food_height - 0.08)),
        origin=Origin(
            xyz=(door_width * 0.5, 0.009, fresh_food_height * 0.5)
        ),
        material=liner,
        name="left_inner_liner",
    )
    left_door.visual(
        Box((handle_bar_width, handle_bar_depth, main_handle_height)),
        origin=Origin(
            xyz=(
                door_width - handle_center_inset,
                door_thickness + handle_bar_depth * 0.5 - 0.004,
                fresh_food_height * 0.5,
            )
        ),
        material=handle_dark,
        name="left_handle_bar",
    )
    left_door.visual(
        Box((handle_bar_width, handle_standoff_depth, handle_mount_height)),
        origin=Origin(
            xyz=(
                door_width - handle_center_inset,
                door_thickness + handle_standoff_depth * 0.5 - 0.002,
                fresh_food_height * 0.5 + main_handle_height * 0.5 - 0.10,
            )
        ),
        material=handle_dark,
        name="left_handle_upper_mount",
    )
    left_door.visual(
        Box((handle_bar_width, handle_standoff_depth, handle_mount_height)),
        origin=Origin(
            xyz=(
                door_width - handle_center_inset,
                door_thickness + handle_standoff_depth * 0.5 - 0.002,
                fresh_food_height * 0.5 - main_handle_height * 0.5 + 0.10,
            )
        ),
        material=handle_dark,
        name="left_handle_lower_mount",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, fresh_food_height)),
        origin=Origin(
            xyz=(-door_width * 0.5, door_thickness * 0.5, fresh_food_height * 0.5)
        ),
        material=stainless,
        name="right_door_shell",
    )
    right_door.visual(
        Box((door_width - 0.05, 0.018, fresh_food_height - 0.08)),
        origin=Origin(
            xyz=(-door_width * 0.5, 0.009, fresh_food_height * 0.5)
        ),
        material=liner,
        name="right_inner_liner",
    )
    right_door.visual(
        Box((handle_bar_width, handle_bar_depth, main_handle_height)),
        origin=Origin(
            xyz=(
                -door_width + handle_center_inset,
                door_thickness + handle_bar_depth * 0.5 - 0.004,
                fresh_food_height * 0.5,
            )
        ),
        material=handle_dark,
        name="right_handle_bar",
    )
    right_door.visual(
        Box((handle_bar_width, handle_standoff_depth, handle_mount_height)),
        origin=Origin(
            xyz=(
                -door_width + handle_center_inset,
                door_thickness + handle_standoff_depth * 0.5 - 0.002,
                fresh_food_height * 0.5 + main_handle_height * 0.5 - 0.10,
            )
        ),
        material=handle_dark,
        name="right_handle_upper_mount",
    )
    right_door.visual(
        Box((handle_bar_width, handle_standoff_depth, handle_mount_height)),
        origin=Origin(
            xyz=(
                -door_width + handle_center_inset,
                door_thickness + handle_standoff_depth * 0.5 - 0.002,
                fresh_food_height * 0.5 - main_handle_height * 0.5 + 0.10,
            )
        ),
        material=handle_dark,
        name="right_handle_lower_mount",
    )

    # Small side-hinged access hatch mounted on the outer face of the right door.
    hatch = model.part("right_door_hatch")
    hatch.visual(
        Box((hatch_width, hatch_thickness, hatch_height)),
        origin=Origin(
            xyz=(hatch_width * 0.5, hatch_thickness * 0.5, hatch_height * 0.5)
        ),
        material=stainless,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.11, 0.010, 0.020)),
        origin=Origin(
            xyz=(hatch_width * 0.62, hatch_thickness + 0.003, hatch_height * 0.25)
        ),
        material=handle_dark,
        name="hatch_pull",
    )

    freezer_drawer = model.part("freezer_drawer")
    freezer_drawer.visual(
        Box((width - 0.010, drawer_front_thickness, freezer_opening_height)),
        origin=Origin(
            xyz=(0.0, drawer_front_thickness * 0.5, freezer_opening_height * 0.5)
        ),
        material=stainless,
        name="drawer_front",
    )
    freezer_drawer.visual(
        Box((0.56, 0.024, 0.024)),
        origin=Origin(
            xyz=(0.0, drawer_front_thickness + 0.012 - 0.004, freezer_opening_height - 0.070)
        ),
        material=handle_dark,
        name="drawer_handle",
    )
    freezer_drawer.visual(
        Box((width - 2.0 * wall - 0.06, 0.54, freezer_opening_height - 0.05)),
        origin=Origin(
            xyz=(0.0, -0.27, (freezer_opening_height - 0.05) * 0.5 + 0.01)
        ),
        material=liner,
        name="drawer_bin",
    )

    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(
            xyz=(-width * 0.5 + side_clearance_each, depth * 0.5, fresh_food_base_z)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=2.2,
        ),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(
            xyz=(width * 0.5 - side_clearance_each, depth * 0.5, fresh_food_base_z)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=2.2,
        ),
    )
    model.articulation(
        "right_door_to_hatch",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=hatch,
        origin=Origin(
            xyz=(
                -door_width + hatch_from_free_edge,
                door_thickness,
                fresh_food_height - hatch_from_top - hatch_height,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.7,
        ),
    )
    model.articulation(
        "cabinet_to_freezer_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=freezer_drawer,
        origin=Origin(xyz=(0.0, depth * 0.5, plinth_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.34,
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

    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    hatch = object_model.get_part("right_door_hatch")
    freezer_drawer = object_model.get_part("freezer_drawer")

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    hatch_hinge = object_model.get_articulation("right_door_to_hatch")
    drawer_slide = object_model.get_articulation("cabinet_to_freezer_drawer")

    for part_name in (
        "cabinet",
        "left_door",
        "right_door",
        "right_door_hatch",
        "freezer_drawer",
    ):
        ctx.check(
            f"part present: {part_name}",
            object_model.get_part(part_name) is not None,
            details=f"missing {part_name}",
        )

    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            hatch_hinge: 0.0,
            drawer_slide: 0.0,
        }
    ):
        ctx.expect_gap(
            left_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="left_door_shell",
            negative_elem="left_side",
            name="left door closes against cabinet front plane",
        )
        ctx.expect_gap(
            right_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="right_door_shell",
            negative_elem="right_side",
            name="right door closes against cabinet front plane",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.010,
            max_gap=0.030,
            positive_elem="right_door_shell",
            negative_elem="left_door_shell",
            name="French doors keep a narrow center seam",
        )
        ctx.expect_gap(
            hatch,
            right_door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0001,
            positive_elem="hatch_panel",
            negative_elem="right_door_shell",
            name="hatch sits on the outer face of the right door",
        )
        ctx.expect_within(
            hatch,
            right_door,
            axes="xz",
            margin=0.0,
            inner_elem="hatch_panel",
            outer_elem="right_door_shell",
            name="hatch stays inside the right door face footprint",
        )
        ctx.expect_gap(
            freezer_drawer,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="drawer_front",
            name="freezer drawer front closes flush with cabinet front",
        )
        ctx.expect_overlap(
            freezer_drawer,
            cabinet,
            axes="x",
            min_overlap=0.80,
            elem_a="drawer_front",
            name="freezer drawer matches cabinet width",
        )

        left_closed = ctx.part_element_world_aabb(left_door, elem="left_door_shell")
        right_closed = ctx.part_element_world_aabb(right_door, elem="right_door_shell")
        hatch_closed = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
        drawer_closed = ctx.part_element_world_aabb(freezer_drawer, elem="drawer_front")

    with ctx.pose({left_hinge: 1.3}):
        left_open = ctx.part_element_world_aabb(left_door, elem="left_door_shell")
    with ctx.pose({right_hinge: 1.3}):
        right_open = ctx.part_element_world_aabb(right_door, elem="right_door_shell")
    with ctx.pose({hatch_hinge: 1.0}):
        hatch_open = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({drawer_slide: 0.30}):
        drawer_open = ctx.part_element_world_aabb(freezer_drawer, elem="drawer_front")

    ctx.check(
        "left main door swings outward",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.16,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right main door swings outward",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.16,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "access hatch swings outward from the door face",
        hatch_closed is not None
        and hatch_open is not None
        and hatch_open[1][1] > hatch_closed[1][1] + 0.08,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )
    ctx.check(
        "freezer drawer extends forward",
        drawer_closed is not None
        and drawer_open is not None
        and drawer_open[0][1] > drawer_closed[0][1] + 0.20,
        details=f"closed={drawer_closed}, open={drawer_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

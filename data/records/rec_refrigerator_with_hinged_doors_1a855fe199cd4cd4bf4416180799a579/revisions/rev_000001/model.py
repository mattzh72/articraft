from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_refrigerator_door(
    part,
    *,
    door_width: float,
    door_height: float,
    door_thickness: float,
    panel_center_y: float,
    handle_length: float,
    door_material,
    gasket_material,
    handle_material,
    panel_name: str,
    handle_name: str,
) -> None:
    panel_inset_x = 0.020
    panel_width = door_width - panel_inset_x
    hinge_side_wrap_width = 0.024
    hinge_side_wrap_depth = 0.012
    part.visual(
        Box((hinge_side_wrap_width, hinge_side_wrap_depth, door_height)),
        origin=Origin(
            xyz=(hinge_side_wrap_width / 2.0 - 0.002, 0.004, 0.0)
        ),
        material=door_material,
        name=f"{panel_name}_hinge_side_wrap",
    )
    part.visual(
        Box((panel_width, door_thickness, door_height)),
        origin=Origin(xyz=(panel_inset_x + panel_width / 2.0, panel_center_y, 0.0)),
        material=door_material,
        name=panel_name,
    )

    gasket_depth = 0.008
    gasket_border = 0.020
    gasket_thickness = 0.018
    gasket_center_y = panel_center_y - (door_thickness / 2.0) + (gasket_depth / 2.0)
    gasket_span_x = panel_width - 2.0 * gasket_border
    gasket_span_z = door_height - 2.0 * gasket_border

    part.visual(
        Box((gasket_thickness, gasket_depth, gasket_span_z)),
        origin=Origin(
            xyz=(
                panel_inset_x + gasket_border + gasket_thickness / 2.0,
                gasket_center_y,
                0.0,
            )
        ),
        material=gasket_material,
        name=f"{panel_name}_gasket_hinge_side",
    )
    part.visual(
        Box((gasket_thickness, gasket_depth, gasket_span_z)),
        origin=Origin(
            xyz=(
                panel_inset_x + panel_width - gasket_border - gasket_thickness / 2.0,
                gasket_center_y,
                0.0,
            )
        ),
        material=gasket_material,
        name=f"{panel_name}_gasket_latch_side",
    )
    part.visual(
        Box((gasket_span_x, gasket_depth, gasket_thickness)),
        origin=Origin(
            xyz=(
                panel_inset_x + panel_width / 2.0,
                gasket_center_y,
                door_height / 2.0 - gasket_border - gasket_thickness / 2.0,
            )
        ),
        material=gasket_material,
        name=f"{panel_name}_gasket_top",
    )
    part.visual(
        Box((gasket_span_x, gasket_depth, gasket_thickness)),
        origin=Origin(
            xyz=(
                panel_inset_x + panel_width / 2.0,
                gasket_center_y,
                -door_height / 2.0 + gasket_border + gasket_thickness / 2.0,
            )
        ),
        material=gasket_material,
        name=f"{panel_name}_gasket_bottom",
    )

    handle_width = 0.028
    handle_depth = 0.038
    handle_inset = 0.060
    handle_center_y = panel_center_y + (door_thickness / 2.0) + (handle_depth / 2.0)
    part.visual(
        Box((handle_width, handle_depth, handle_length)),
        origin=Origin(
            xyz=(panel_inset_x + panel_width - handle_inset, handle_center_y, 0.0)
        ),
        material=handle_material,
        name=handle_name,
    )

    hinge_leaf_width = 0.012
    hinge_leaf_depth = 0.012
    hinge_leaf_height = 0.060
    hinge_leaf_center_x = 0.002
    hinge_leaf_center_y = 0.004
    hinge_leaf_z = (door_height / 2.0) - 0.055
    for suffix, z_sign in (("top", 1.0), ("bottom", -1.0)):
        part.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(
                xyz=(
                    hinge_leaf_center_x,
                    hinge_leaf_center_y,
                    z_sign * hinge_leaf_z,
                )
            ),
            material=handle_material,
            name=f"{panel_name}_hinge_leaf_{suffix}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_freezer_refrigerator")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    door_white = model.material("door_white", rgba=(0.97, 0.98, 0.98, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.82, 0.83, 0.84, 1.0))
    handle_silver = model.material("handle_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    toe_kick_dark = model.material("toe_kick_dark", rgba=(0.19, 0.20, 0.21, 1.0))

    cabinet_width = 0.78
    cabinet_depth = 0.74
    cabinet_height = 1.79
    wall_thickness = 0.032
    back_thickness = 0.018
    toe_kick_height = 0.10
    top_reveal = 0.015
    bottom_reveal = 0.014
    mid_gap = 0.008
    upper_door_height = 0.41
    lower_door_height = (
        cabinet_height
        - toe_kick_height
        - top_reveal
        - bottom_reveal
        - mid_gap
        - upper_door_height
    )
    door_thickness = 0.055
    door_panel_center_y = 0.015
    hinge_axis_x = -cabinet_width / 2.0 - 0.003
    hinge_axis_y = (cabinet_depth / 2.0) + 0.014
    door_width = cabinet_width - 0.010
    partition_z = toe_kick_height + bottom_reveal + lower_door_height + (mid_gap / 2.0)
    lower_door_center_z = toe_kick_height + bottom_reveal + (lower_door_height / 2.0)
    upper_door_center_z = partition_z + (mid_gap / 2.0) + (upper_door_height / 2.0)

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=cabinet_white,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((wall_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=cabinet_white,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall_thickness, cabinet_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, wall_thickness / 2.0)),
        material=cabinet_white,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall_thickness, cabinet_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - wall_thickness / 2.0)),
        material=cabinet_white,
        name="top_cap",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall_thickness, back_thickness, cabinet_height - 2.0 * wall_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + back_thickness / 2.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall_thickness, cabinet_depth - back_thickness, wall_thickness)),
        origin=Origin(xyz=(0.0, back_thickness / 2.0, partition_z)),
        material=cabinet_white,
        name="freezer_floor_partition",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall_thickness, 0.018, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - 0.009,
                toe_kick_height / 2.0,
            )
        ),
        material=toe_kick_dark,
        name="toe_kick_grille",
    )
    cabinet.visual(
        Box((cabinet_width - 0.12, 0.014, 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - 0.007,
                toe_kick_height + 0.010,
            )
        ),
        material=liner_gray,
        name="fresh_food_threshold",
    )
    hinge_bracket_x = -cabinet_width / 2.0 + 0.007
    hinge_bracket_y = cabinet_depth / 2.0 + 0.006
    hinge_bracket_size = (0.014, 0.012, 0.060)
    cabinet.visual(
        Box(hinge_bracket_size),
        origin=Origin(xyz=(hinge_bracket_x, hinge_bracket_y, upper_door_center_z + upper_door_height / 2.0 - 0.055)),
        material=handle_silver,
        name="freezer_hinge_bracket_top",
    )
    cabinet.visual(
        Box(hinge_bracket_size),
        origin=Origin(xyz=(hinge_bracket_x, hinge_bracket_y, upper_door_center_z - upper_door_height / 2.0 + 0.055)),
        material=handle_silver,
        name="freezer_hinge_bracket_bottom",
    )
    cabinet.visual(
        Box(hinge_bracket_size),
        origin=Origin(xyz=(hinge_bracket_x, hinge_bracket_y, lower_door_center_z + lower_door_height / 2.0 - 0.055)),
        material=handle_silver,
        name="fresh_food_hinge_bracket_top",
    )
    cabinet.visual(
        Box(hinge_bracket_size),
        origin=Origin(xyz=(hinge_bracket_x, hinge_bracket_y, lower_door_center_z - lower_door_height / 2.0 + 0.055)),
        material=handle_silver,
        name="fresh_food_hinge_bracket_bottom",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    freezer_door = model.part("freezer_door")
    _add_refrigerator_door(
        freezer_door,
        door_width=door_width,
        door_height=upper_door_height,
        door_thickness=door_thickness,
        panel_center_y=door_panel_center_y,
        handle_length=0.27,
        door_material=door_white,
        gasket_material=liner_gray,
        handle_material=handle_silver,
        panel_name="freezer_outer_panel",
        handle_name="freezer_handle",
    )
    freezer_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, upper_door_height)),
        mass=15.0,
        origin=Origin(xyz=(door_width / 2.0, door_panel_center_y, 0.0)),
    )

    fresh_food_door = model.part("fresh_food_door")
    _add_refrigerator_door(
        fresh_food_door,
        door_width=door_width,
        door_height=lower_door_height,
        door_thickness=door_thickness,
        panel_center_y=door_panel_center_y,
        handle_length=0.78,
        door_material=door_white,
        gasket_material=liner_gray,
        handle_material=handle_silver,
        panel_name="fresh_food_outer_panel",
        handle_name="fresh_food_handle",
    )
    fresh_food_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, lower_door_height)),
        mass=24.0,
        origin=Origin(xyz=(door_width / 2.0, door_panel_center_y, 0.0)),
    )

    model.articulation(
        "cabinet_to_freezer_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, upper_door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "cabinet_to_fresh_food_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=fresh_food_door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, lower_door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(120.0),
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
    freezer_door = object_model.get_part("freezer_door")
    fresh_food_door = object_model.get_part("fresh_food_door")
    freezer_hinge = object_model.get_articulation("cabinet_to_freezer_door")
    fresh_food_hinge = object_model.get_articulation("cabinet_to_fresh_food_door")
    cabinet_left_side = cabinet.get_visual("left_side_panel")
    freezer_panel = freezer_door.get_visual("freezer_outer_panel")
    fresh_panel = fresh_food_door.get_visual("fresh_food_outer_panel")
    freezer_leaf_top = freezer_door.get_visual("freezer_outer_panel_hinge_leaf_top")
    fresh_leaf_top = fresh_food_door.get_visual("fresh_food_outer_panel_hinge_leaf_top")
    freezer_bracket_top = cabinet.get_visual("freezer_hinge_bracket_top")
    fresh_bracket_top = cabinet.get_visual("fresh_food_hinge_bracket_top")

    ctx.check(
        "both doors use left-side vertical hinges",
        freezer_hinge.axis == (0.0, 0.0, 1.0)
        and fresh_food_hinge.axis == (0.0, 0.0, 1.0)
        and freezer_hinge.origin.xyz[0] < 0.0
        and fresh_food_hinge.origin.xyz[0] < 0.0,
        details=(
            f"freezer axis={freezer_hinge.axis}, freezer origin={freezer_hinge.origin.xyz}; "
            f"fresh axis={fresh_food_hinge.axis}, fresh origin={fresh_food_hinge.origin.xyz}"
        ),
    )

    with ctx.pose({freezer_hinge: 0.0, fresh_food_hinge: 0.0}):
        ctx.expect_gap(
            freezer_door,
            cabinet,
            axis="y",
            positive_elem=freezer_panel,
            negative_elem=cabinet_left_side,
            min_gap=0.001,
            max_gap=0.003,
            name="freezer outer panel sits just proud of cabinet front plane",
        )
        ctx.expect_gap(
            fresh_food_door,
            cabinet,
            axis="y",
            positive_elem=fresh_panel,
            negative_elem=cabinet_left_side,
            min_gap=0.001,
            max_gap=0.003,
            name="fresh-food outer panel sits just proud of cabinet front plane",
        )
        ctx.expect_contact(
            freezer_door,
            cabinet,
            elem_a=freezer_leaf_top,
            elem_b=freezer_bracket_top,
            name="freezer door hinge hardware meets cabinet bracket",
        )
        ctx.expect_contact(
            fresh_food_door,
            cabinet,
            elem_a=fresh_leaf_top,
            elem_b=fresh_bracket_top,
            name="fresh-food door hinge hardware meets cabinet bracket",
        )
        ctx.expect_overlap(
            freezer_door,
            cabinet,
            axes="xz",
            min_overlap=0.34,
            name="freezer door covers upper cabinet opening",
        )
        ctx.expect_overlap(
            fresh_food_door,
            cabinet,
            axes="xz",
            min_overlap=0.34,
            name="fresh-food door covers lower cabinet opening",
        )
        ctx.expect_gap(
            freezer_door,
            fresh_food_door,
            axis="z",
            min_gap=0.007,
            max_gap=0.010,
            name="stacked doors keep a realistic center seam gap",
        )

        freezer_rest_aabb = ctx.part_world_aabb(freezer_door)
        fresh_rest_aabb = ctx.part_world_aabb(fresh_food_door)

    with ctx.pose({freezer_hinge: math.radians(75.0)}):
        freezer_open_aabb = ctx.part_world_aabb(freezer_door)
        ctx.check(
            "freezer door swings forward when opened",
            freezer_rest_aabb is not None
            and freezer_open_aabb is not None
            and freezer_open_aabb[1][1] > freezer_rest_aabb[1][1] + 0.18,
            details=f"rest={freezer_rest_aabb}, open={freezer_open_aabb}",
        )

    with ctx.pose({fresh_food_hinge: math.radians(75.0)}):
        fresh_open_aabb = ctx.part_world_aabb(fresh_food_door)
        ctx.check(
            "fresh-food door swings forward when opened",
            fresh_rest_aabb is not None
            and fresh_open_aabb is not None
            and fresh_open_aabb[1][1] > fresh_rest_aabb[1][1] + 0.18,
            details=f"rest={fresh_rest_aabb}, open={fresh_open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

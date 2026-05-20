from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _add_vertical_handle(
    part,
    *,
    x: float,
    z_center: float,
    length: float,
    door_gap: float,
    door_thickness: float,
    bar_radius: float,
    material,
    prefix: str,
) -> None:
    bar_center_y = door_gap + door_thickness + 0.026 + bar_radius
    part.visual(
        Cylinder(radius=bar_radius, length=length),
        origin=Origin(xyz=(x, bar_center_y, z_center)),
        material=material,
        name=f"{prefix}_handle_bar",
    )

    bracket_min_y = door_gap + door_thickness - 0.002
    bracket_max_y = bar_center_y
    bracket_depth = bracket_max_y - bracket_min_y
    bracket_center_y = (bracket_min_y + bracket_max_y) / 2.0
    bracket_z_offset = length * 0.32
    bracket_size = (bar_radius * 2.4, bracket_depth, bar_radius * 2.2)

    for suffix, z in (("upper", z_center + bracket_z_offset), ("lower", z_center - bracket_z_offset)):
        part.visual(
            Box(bracket_size),
            origin=Origin(xyz=(x, bracket_center_y, z)),
            material=material,
            name=f"{prefix}_{suffix}_handle_bracket",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator")

    body_gray = model.material("body_gray", rgba=(0.55, 0.58, 0.62, 1.0))
    steel = model.material("stainless_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    liner_white = model.material("liner_white", rgba=(0.94, 0.95, 0.97, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.14, 0.16, 1.0))

    cabinet_width = 0.92
    cabinet_depth = 0.70
    cabinet_height = 1.80
    wall = 0.03
    divider = 0.03
    center_mullion = 0.025
    liner = 0.004

    lower_opening_height = 0.66
    upper_opening_height = cabinet_height - (2.0 * wall + divider + lower_opening_height)
    upper_section_z0 = wall + lower_opening_height + divider

    door_gap = 0.0025
    vertical_reveal = 0.005
    upper_center_gap = 0.008
    upper_door_thickness = 0.065
    lower_door_thickness = 0.068
    upper_door_width = cabinet_width / 2.0 - upper_center_gap / 2.0
    lower_door_width = cabinet_width - 0.010
    upper_door_height = upper_opening_height - 2.0 * vertical_reveal
    lower_door_height = lower_opening_height - 2.0 * vertical_reveal

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall / 2.0, 0.0, cabinet_height / 2.0)),
        material=body_gray,
        name="left_side_shell",
    )
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall / 2.0, 0.0, cabinet_height / 2.0)),
        material=body_gray,
        name="right_side_shell",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, wall, cabinet_height)),
        origin=Origin(xyz=(0.0, -cabinet_depth / 2.0 + wall / 2.0, cabinet_height / 2.0)),
        material=body_gray,
        name="back_shell",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, wall)),
        origin=Origin(xyz=(0.0, wall / 2.0, wall / 2.0)),
        material=body_gray,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, wall)),
        origin=Origin(xyz=(0.0, wall / 2.0, cabinet_height - wall / 2.0)),
        material=body_gray,
        name="top_shell",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, divider)),
        origin=Origin(
            xyz=(0.0, wall / 2.0, wall + lower_opening_height + divider / 2.0),
        ),
        material=liner_white,
        name="mid_divider_shelf",
    )
    cabinet.visual(
        Box((center_mullion, cabinet_depth - wall, upper_opening_height)),
        origin=Origin(
            xyz=(0.0, wall / 2.0, upper_section_z0 + upper_opening_height / 2.0),
        ),
        material=liner_white,
        name="upper_center_mullion",
    )
    cabinet.visual(
        Box((cabinet_width * 0.58, 0.12, 0.12)),
        origin=Origin(
            xyz=(0.0, -cabinet_depth / 2.0 + wall + 0.06, wall + 0.06),
        ),
        material=liner_white,
        name="compressor_cover",
    )

    cabinet.visual(
        Box((liner, cabinet_depth - wall, cabinet_height - 2.0 * wall)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + wall + liner / 2.0, wall / 2.0, cabinet_height / 2.0)),
        material=liner_white,
        name="left_inner_liner",
    )
    cabinet.visual(
        Box((liner, cabinet_depth - wall, cabinet_height - 2.0 * wall)),
        origin=Origin(xyz=(cabinet_width / 2.0 - wall - liner / 2.0, wall / 2.0, cabinet_height / 2.0)),
        material=liner_white,
        name="right_inner_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, liner, cabinet_height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -cabinet_depth / 2.0 + wall + liner / 2.0, cabinet_height / 2.0)),
        material=liner_white,
        name="back_inner_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, liner)),
        origin=Origin(xyz=(0.0, wall / 2.0, wall + liner / 2.0)),
        material=liner_white,
        name="bottom_inner_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, liner)),
        origin=Origin(xyz=(0.0, wall / 2.0, cabinet_height - wall - liner / 2.0)),
        material=liner_white,
        name="top_inner_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, liner)),
        origin=Origin(xyz=(0.0, wall / 2.0, wall + lower_opening_height + liner / 2.0)),
        material=liner_white,
        name="lower_ceiling_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, cabinet_depth - wall, liner)),
        origin=Origin(xyz=(0.0, wall / 2.0, upper_section_z0 + liner / 2.0)),
        material=liner_white,
        name="upper_floor_liner",
    )

    left_upper_door = model.part("left_upper_door")
    left_upper_door.visual(
        Box((upper_door_width, upper_door_thickness, upper_door_height)),
        origin=Origin(
            xyz=(upper_door_width / 2.0, door_gap + upper_door_thickness / 2.0, upper_door_height / 2.0),
        ),
        material=steel,
        name="left_upper_panel",
    )
    left_upper_door.visual(
        Box((upper_door_width - 0.08, 0.018, upper_door_height - 0.08)),
        origin=Origin(
            xyz=(upper_door_width / 2.0, door_gap + 0.009, upper_door_height / 2.0),
        ),
        material=liner_white,
        name="left_upper_inner_liner",
    )
    left_upper_door.visual(
        Box((0.018, 0.012, upper_door_height - 0.06)),
        origin=Origin(xyz=(0.009, 0.006, upper_door_height / 2.0)),
        material=trim_dark,
        name="left_upper_hinge_leaf",
    )
    _add_vertical_handle(
        left_upper_door,
        x=upper_door_width - 0.055,
        z_center=upper_door_height * 0.54,
        length=0.56,
        door_gap=door_gap,
        door_thickness=upper_door_thickness,
        bar_radius=0.010,
        material=trim_dark,
        prefix="left_upper",
    )

    right_upper_door = model.part("right_upper_door")
    right_upper_door.visual(
        Box((upper_door_width, upper_door_thickness, upper_door_height)),
        origin=Origin(
            xyz=(-upper_door_width / 2.0, door_gap + upper_door_thickness / 2.0, upper_door_height / 2.0),
        ),
        material=steel,
        name="right_upper_panel",
    )
    right_upper_door.visual(
        Box((upper_door_width - 0.08, 0.018, upper_door_height - 0.08)),
        origin=Origin(
            xyz=(-upper_door_width / 2.0, door_gap + 0.009, upper_door_height / 2.0),
        ),
        material=liner_white,
        name="right_upper_inner_liner",
    )
    right_upper_door.visual(
        Box((0.018, 0.012, upper_door_height - 0.06)),
        origin=Origin(xyz=(-0.009, 0.006, upper_door_height / 2.0)),
        material=trim_dark,
        name="right_upper_hinge_leaf",
    )
    _add_vertical_handle(
        right_upper_door,
        x=-(upper_door_width - 0.055),
        z_center=upper_door_height * 0.54,
        length=0.56,
        door_gap=door_gap,
        door_thickness=upper_door_thickness,
        bar_radius=0.010,
        material=trim_dark,
        prefix="right_upper",
    )

    freezer_door = model.part("freezer_door")
    freezer_door.visual(
        Box((lower_door_width, lower_door_thickness, lower_door_height)),
        origin=Origin(
            xyz=(lower_door_width / 2.0, door_gap + lower_door_thickness / 2.0, lower_door_height / 2.0),
        ),
        material=steel,
        name="freezer_panel",
    )
    freezer_door.visual(
        Box((lower_door_width - 0.09, 0.020, lower_door_height - 0.09)),
        origin=Origin(
            xyz=(lower_door_width / 2.0, door_gap + 0.010, lower_door_height / 2.0),
        ),
        material=liner_white,
        name="freezer_inner_liner",
    )
    freezer_door.visual(
        Box((0.018, 0.012, lower_door_height - 0.06)),
        origin=Origin(xyz=(0.009, 0.006, lower_door_height / 2.0)),
        material=trim_dark,
        name="freezer_hinge_leaf",
    )
    _add_vertical_handle(
        freezer_door,
        x=lower_door_width - 0.060,
        z_center=lower_door_height * 0.54,
        length=0.44,
        door_gap=door_gap,
        door_thickness=lower_door_thickness,
        bar_radius=0.010,
        material=trim_dark,
        prefix="freezer",
    )

    model.articulation(
        "left_upper_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_upper_door,
        origin=Origin(xyz=(-cabinet_width / 2.0, cabinet_depth / 2.0, upper_section_z0 + vertical_reveal)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=2.1),
    )
    model.articulation(
        "right_upper_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_upper_door,
        origin=Origin(xyz=(cabinet_width / 2.0, cabinet_depth / 2.0, upper_section_z0 + vertical_reveal)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=2.1),
    )
    model.articulation(
        "freezer_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(-cabinet_width / 2.0, cabinet_depth / 2.0, wall + vertical_reveal)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.3, lower=0.0, upper=2.1),
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
    left_upper_door = object_model.get_part("left_upper_door")
    right_upper_door = object_model.get_part("right_upper_door")
    freezer_door = object_model.get_part("freezer_door")

    left_upper_hinge = object_model.get_articulation("left_upper_hinge")
    right_upper_hinge = object_model.get_articulation("right_upper_hinge")
    freezer_hinge = object_model.get_articulation("freezer_hinge")

    required_cabinet_visuals = (
        "mid_divider_shelf",
        "upper_center_mullion",
        "compressor_cover",
        "left_inner_liner",
        "right_inner_liner",
    )
    for visual_name in required_cabinet_visuals:
        try:
            cabinet.get_visual(visual_name)
            ok = True
            details = ""
        except Exception as exc:  # pragma: no cover - defensive authoring check
            ok = False
            details = str(exc)
        ctx.check(f"cabinet includes {visual_name}", ok, details=details)

    for door, panel_name in (
        (left_upper_door, "left_upper_panel"),
        (right_upper_door, "right_upper_panel"),
        (freezer_door, "freezer_panel"),
    ):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.0015,
            max_gap=0.0045,
            positive_elem=panel_name,
            name=f"{door.name} sits just proud of cabinet face",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="z",
            min_overlap=0.55 if door.name != "freezer_door" else 0.40,
            elem_a=panel_name,
            name=f"{door.name} spans its intended compartment height",
        )

    closed_left = ctx.part_element_world_aabb(left_upper_door, elem="left_upper_panel")
    with ctx.pose({left_upper_hinge: 1.2}):
        open_left = ctx.part_element_world_aabb(left_upper_door, elem="left_upper_panel")
    ctx.check(
        "left upper door opens outward",
        closed_left is not None
        and open_left is not None
        and open_left[1][1] > closed_left[1][1] + 0.12,
        details=f"closed={closed_left}, open={open_left}",
    )

    closed_right = ctx.part_element_world_aabb(right_upper_door, elem="right_upper_panel")
    with ctx.pose({right_upper_hinge: 1.2}):
        open_right = ctx.part_element_world_aabb(right_upper_door, elem="right_upper_panel")
    ctx.check(
        "right upper door opens outward",
        closed_right is not None
        and open_right is not None
        and open_right[1][1] > closed_right[1][1] + 0.12,
        details=f"closed={closed_right}, open={open_right}",
    )

    closed_freezer = ctx.part_element_world_aabb(freezer_door, elem="freezer_panel")
    with ctx.pose({freezer_hinge: 1.1}):
        open_freezer = ctx.part_element_world_aabb(freezer_door, elem="freezer_panel")
    ctx.check(
        "lower freezer door opens outward",
        closed_freezer is not None
        and open_freezer is not None
        and open_freezer[1][1] > closed_freezer[1][1] + 0.12,
        details=f"closed={closed_freezer}, open={open_freezer}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

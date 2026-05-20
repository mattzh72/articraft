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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


COLUMNS = 3
ROWS = 4


def _drawer_part_name(row: int, col: int) -> str:
    return f"drawer_r{row}_c{col}"


def _drawer_joint_name(row: int, col: int) -> str:
    return f"cabinet_to_drawer_r{row}_c{col}"


def _left_rail_name(row: int, col: int) -> str:
    return f"left_guide_rail_r{row}_c{col}"


def _right_rail_name(row: int, col: int) -> str:
    return f"right_guide_rail_r{row}_c{col}"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_card_catalog_desk")

    dark_oak = model.material("dark_oak", rgba=(0.35, 0.23, 0.12, 1.0))
    medium_oak = model.material("medium_oak", rgba=(0.46, 0.31, 0.17, 1.0))
    light_oak = model.material("light_oak", rgba=(0.63, 0.47, 0.28, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.70, 0.58, 0.30, 1.0))
    shadow = model.material("shadow", rgba=(0.15, 0.11, 0.08, 1.0))

    top_width_y = 1.36
    top_depth_x = 0.82
    top_thickness = 0.055

    cabinet_width_y = 0.72
    cabinet_depth_x = 0.58
    cabinet_height_z = 0.845

    plinth_height = 0.085
    plinth_width_y = 0.70
    plinth_depth_x = 0.56

    side_thickness = 0.025
    divider_thickness = 0.018
    back_thickness = 0.018
    shelf_thickness = 0.018
    bottom_thickness = 0.018
    top_deck_thickness = 0.025

    inner_width_y = cabinet_width_y - 2.0 * side_thickness
    opening_width_y = (inner_width_y - (COLUMNS - 1) * divider_thickness) / COLUMNS

    bottom_top_z = plinth_height + bottom_thickness
    top_deck_bottom_z = cabinet_height_z - top_deck_thickness
    vertical_clear_z = top_deck_bottom_z - bottom_top_z
    opening_height_z = (vertical_clear_z - (ROWS - 1) * shelf_thickness) / ROWS

    inner_depth_x = cabinet_depth_x - back_thickness
    inner_x_center = back_thickness * 0.5
    front_plane_x = cabinet_depth_x * 0.5

    drawer_front_thickness = 0.018
    drawer_front_width = opening_width_y - 0.008
    drawer_front_height = opening_height_z - 0.006

    drawer_body_width = opening_width_y - 0.014
    drawer_body_height = opening_height_z - 0.012
    drawer_body_depth = 0.52
    side_board = 0.009
    bottom_board = 0.008
    back_board = 0.009

    rail_width_y = 0.014
    rail_height_z = 0.012
    rail_length_x = 0.49
    slide_travel = 0.26

    cabinet = model.part("catalog_desk")
    cabinet.visual(
        Box((top_depth_x, top_width_y, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height_z + top_thickness / 2.0)),
        material=dark_oak,
        name="work_surface",
    )
    cabinet.visual(
        Box((plinth_depth_x, plinth_width_y, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=shadow,
        name="plinth",
    )
    cabinet.visual(
        Box((cabinet_depth_x, side_thickness, cabinet_height_z - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_width_y / 2.0 + side_thickness / 2.0,
                plinth_height + (cabinet_height_z - plinth_height) / 2.0,
            )
        ),
        material=medium_oak,
        name="left_case_side",
    )
    cabinet.visual(
        Box((cabinet_depth_x, side_thickness, cabinet_height_z - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_width_y / 2.0 - side_thickness / 2.0,
                plinth_height + (cabinet_height_z - plinth_height) / 2.0,
            )
        ),
        material=medium_oak,
        name="right_case_side",
    )
    cabinet.visual(
        Box((back_thickness, inner_width_y, cabinet_height_z - plinth_height)),
        origin=Origin(
            xyz=(
                -cabinet_depth_x / 2.0 + back_thickness / 2.0,
                0.0,
                plinth_height + (cabinet_height_z - plinth_height) / 2.0,
            )
        ),
        material=medium_oak,
        name="back_panel",
    )
    cabinet.visual(
        Box((inner_depth_x, inner_width_y, bottom_thickness)),
        origin=Origin(
            xyz=(inner_x_center, 0.0, plinth_height + bottom_thickness / 2.0)
        ),
        material=medium_oak,
        name="bottom_deck",
    )
    cabinet.visual(
        Box((inner_depth_x, inner_width_y, top_deck_thickness)),
        origin=Origin(
            xyz=(
                inner_x_center,
                0.0,
                cabinet_height_z - top_deck_thickness / 2.0,
            )
        ),
        material=medium_oak,
        name="top_deck",
    )

    divider_height = top_deck_bottom_z - bottom_top_z
    divider_center_z = bottom_top_z + divider_height / 2.0
    inner_left_y = -cabinet_width_y / 2.0 + side_thickness
    inner_right_y = cabinet_width_y / 2.0 - side_thickness

    for divider_index in range(1, COLUMNS):
        divider_center_y = (
            inner_left_y
            + divider_index * opening_width_y
            + (divider_index - 0.5) * divider_thickness
        )
        cabinet.visual(
            Box((inner_depth_x, divider_thickness, divider_height)),
            origin=Origin(xyz=(inner_x_center, divider_center_y, divider_center_z)),
            material=medium_oak,
            name=f"vertical_divider_{divider_index}",
        )

    row_bottoms: list[float] = []
    for row in range(ROWS):
        row_bottom = bottom_top_z + row * (opening_height_z + shelf_thickness)
        row_bottoms.append(row_bottom)
        if row < ROWS - 1:
            separator_center_z = row_bottom + opening_height_z + shelf_thickness / 2.0
            cabinet.visual(
                Box((inner_depth_x, inner_width_y, shelf_thickness)),
                origin=Origin(xyz=(inner_x_center, 0.0, separator_center_z)),
                material=medium_oak,
                name=f"row_separator_{row}",
            )

    column_centers_y: list[float] = []
    opening_lefts_y: list[float] = []
    opening_rights_y: list[float] = []
    cursor_y = inner_left_y
    for col in range(COLUMNS):
        opening_left = cursor_y
        opening_right = opening_left + opening_width_y
        opening_lefts_y.append(opening_left)
        opening_rights_y.append(opening_right)
        column_centers_y.append((opening_left + opening_right) / 2.0)
        cursor_y = opening_right + divider_thickness

    for row, row_bottom in enumerate(row_bottoms):
        for col, center_y in enumerate(column_centers_y):
            left_rail_center_y = opening_lefts_y[col] + rail_width_y / 2.0
            right_rail_center_y = opening_rights_y[col] - rail_width_y / 2.0
            rail_center_x = front_plane_x - rail_length_x / 2.0
            rail_center_z = row_bottom - rail_height_z / 2.0

            cabinet.visual(
                Box((rail_length_x, rail_width_y, rail_height_z)),
                origin=Origin(
                    xyz=(rail_center_x, left_rail_center_y, rail_center_z)
                ),
                material=light_oak,
                name=_left_rail_name(row, col),
            )
            cabinet.visual(
                Box((rail_length_x, rail_width_y, rail_height_z)),
                origin=Origin(
                    xyz=(rail_center_x, right_rail_center_y, rail_center_z)
                ),
                material=light_oak,
                name=_right_rail_name(row, col),
            )

    cabinet.inertial = Inertial.from_geometry(
        Box((top_depth_x, top_width_y, cabinet_height_z + top_thickness)),
        mass=72.0,
        origin=Origin(
            xyz=(0.0, 0.0, (cabinet_height_z + top_thickness) / 2.0)
        ),
    )

    def add_drawer(row: int, col: int) -> None:
        drawer = model.part(_drawer_part_name(row, col))

        drawer.visual(
            Box((drawer_front_thickness, drawer_front_width, drawer_front_height)),
            origin=Origin(
                xyz=(
                    drawer_front_thickness / 2.0,
                    0.0,
                    0.002 + drawer_front_height / 2.0,
                )
            ),
            material=dark_oak,
            name="drawer_front",
        )
        drawer.visual(
            Box((drawer_body_depth - back_board, side_board, drawer_body_height)),
            origin=Origin(
                xyz=(
                    -(drawer_body_depth - back_board) / 2.0,
                    -drawer_body_width / 2.0 + side_board / 2.0,
                    drawer_body_height / 2.0,
                )
            ),
            material=light_oak,
            name="left_side",
        )
        drawer.visual(
            Box((drawer_body_depth - back_board, side_board, drawer_body_height)),
            origin=Origin(
                xyz=(
                    -(drawer_body_depth - back_board) / 2.0,
                    drawer_body_width / 2.0 - side_board / 2.0,
                    drawer_body_height / 2.0,
                )
            ),
            material=light_oak,
            name="right_side",
        )
        drawer.visual(
            Box((drawer_body_depth - back_board, drawer_body_width, bottom_board)),
            origin=Origin(
                xyz=(
                    -(drawer_body_depth - back_board) / 2.0,
                    0.0,
                    bottom_board / 2.0,
                )
            ),
            material=light_oak,
            name="bottom_panel",
        )
        drawer.visual(
            Box((back_board, drawer_body_width, drawer_body_height)),
            origin=Origin(
                xyz=(
                    -drawer_body_depth + back_board / 2.0,
                    0.0,
                    drawer_body_height / 2.0,
                )
            ),
            material=light_oak,
            name="back_panel",
        )
        drawer.visual(
            Box((0.004, 0.092, 0.038)),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + 0.002,
                    0.0,
                    drawer_front_height * 0.56,
                )
            ),
            material=aged_brass,
            name="label_frame",
        )
        drawer.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + 0.006,
                    -0.036,
                    drawer_front_height * 0.36,
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=aged_brass,
            name="left_pull_post",
        )
        drawer.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + 0.006,
                    0.036,
                    drawer_front_height * 0.36,
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=aged_brass,
            name="right_pull_post",
        )
        drawer.visual(
            Cylinder(radius=0.005, length=0.082),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + 0.012,
                    0.0,
                    drawer_front_height * 0.36,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aged_brass,
            name="pull_bar",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((drawer_body_depth + drawer_front_thickness, drawer_front_width, drawer_front_height)),
            mass=1.8,
            origin=Origin(
                xyz=(
                    -(drawer_body_depth - drawer_front_thickness) / 2.0,
                    0.0,
                    drawer_front_height / 2.0,
                )
            ),
        )

        model.articulation(
            _drawer_joint_name(row, col),
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(front_plane_x, column_centers_y[col], row_bottoms[row])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=55.0,
                velocity=0.22,
                lower=0.0,
                upper=slide_travel,
            ),
        )

    for row in range(ROWS):
        for col in range(COLUMNS):
            add_drawer(row, col)

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
    cabinet = object_model.get_part("catalog_desk")
    drawers = []
    joints = []
    for row in range(ROWS):
        for col in range(COLUMNS):
            drawers.append(object_model.get_part(_drawer_part_name(row, col)))
            joints.append(object_model.get_articulation(_drawer_joint_name(row, col)))

    for row in range(ROWS):
        for col in range(COLUMNS):
            drawer = object_model.get_part(_drawer_part_name(row, col))
            joint = object_model.get_articulation(_drawer_joint_name(row, col))

            ctx.expect_contact(
                cabinet,
                drawer,
                elem_a=_left_rail_name(row, col),
                elem_b="bottom_panel",
                name=f"left guide rail supports drawer r{row} c{col}",
            )
            ctx.expect_contact(
                cabinet,
                drawer,
                elem_a=_right_rail_name(row, col),
                elem_b="bottom_panel",
                name=f"right guide rail supports drawer r{row} c{col}",
            )

            closed_pos = ctx.part_world_position(drawer)
            upper = joint.motion_limits.upper if joint.motion_limits is not None else None
            extended_pos = None
            if upper is not None:
                with ctx.pose({joint: upper}):
                    extended_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"drawer r{row} c{col} slides outward",
                closed_pos is not None
                and extended_pos is not None
                and upper is not None
                and extended_pos[0] > closed_pos[0] + 0.15,
                details=f"closed={closed_pos}, extended={extended_pos}, upper={upper}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

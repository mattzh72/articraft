from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

HOOD_WIDTH = 0.90
HOOD_DEPTH = 0.50
HOOD_HEIGHT = 0.18
BODY_SHELL_THICKNESS = 0.018

CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.28
CHIMNEY_HEIGHT = 0.62
CHIMNEY_SHELL_THICKNESS = 0.012

FILTER_THICKNESS = 0.006
FILTER_BOTTOM_OFFSET = BODY_SHELL_THICKNESS

BUTTON_WIDTH = 0.026
BUTTON_HEIGHT = 0.016
BUTTON_FACE_DEPTH = 0.006
BUTTON_STEM_WIDTH = 0.020
BUTTON_STEM_HEIGHT = 0.010
BUTTON_STEM_DEPTH = 0.030
BUTTON_GUIDE_DEPTH = 0.026
BUTTON_TRAVEL = 0.004
BUTTON_CENTER_Y = HOOD_DEPTH + BUTTON_FACE_DEPTH * 0.5
BUTTON_OUTWARD_OFFSET = 0.003
BUTTON_COLUMN_X = (-0.050, 0.000, 0.050)
BUTTON_ROW_Z = (0.105, 0.075)
def _button_part_name(row_index: int, column_index: int) -> str:
    return f"button_r{row_index}_c{column_index}"


def _button_joint_name(row_index: int, column_index: int) -> str:
    return f"housing_to_button_r{row_index}_c{column_index}"


def _button_center_xyz(row_index: int, column_index: int) -> tuple[float, float, float]:
    outward = BUTTON_OUTWARD_OFFSET if (row_index, column_index) == (1, 2) else 0.0
    return (
        BUTTON_COLUMN_X[column_index],
        BUTTON_CENTER_Y + outward,
        BUTTON_ROW_Z[row_index],
    )


def _add_front_face(housing, material) -> None:
    front_y = HOOD_DEPTH - BODY_SHELL_THICKNESS * 0.5
    x_edges = (
        -HOOD_WIDTH * 0.5,
        BUTTON_COLUMN_X[0] - BUTTON_WIDTH * 0.5,
        BUTTON_COLUMN_X[0] + BUTTON_WIDTH * 0.5,
        BUTTON_COLUMN_X[1] - BUTTON_WIDTH * 0.5,
        BUTTON_COLUMN_X[1] + BUTTON_WIDTH * 0.5,
        BUTTON_COLUMN_X[2] - BUTTON_WIDTH * 0.5,
        BUTTON_COLUMN_X[2] + BUTTON_WIDTH * 0.5,
        HOOD_WIDTH * 0.5,
    )
    z_edges = (
        0.0,
        BUTTON_ROW_Z[1] - BUTTON_HEIGHT * 0.5,
        BUTTON_ROW_Z[1] + BUTTON_HEIGHT * 0.5,
        BUTTON_ROW_Z[0] - BUTTON_HEIGHT * 0.5,
        BUTTON_ROW_Z[0] + BUTTON_HEIGHT * 0.5,
        HOOD_HEIGHT,
    )
    hole_x_indices = {1, 3, 5}
    hole_z_indices = {1, 3}

    for x_index in range(len(x_edges) - 1):
        x_min = x_edges[x_index]
        x_max = x_edges[x_index + 1]
        for z_index in range(len(z_edges) - 1):
            if x_index in hole_x_indices and z_index in hole_z_indices:
                continue
            z_min = z_edges[z_index]
            z_max = z_edges[z_index + 1]
            housing.visual(
                Box((x_max - x_min, BODY_SHELL_THICKNESS, z_max - z_min)),
                origin=Origin(
                    xyz=((x_min + x_max) * 0.5, front_y, (z_min + z_max) * 0.5),
                ),
                material=material,
                name=f"front_patch_{x_index}_{z_index}",
            )


def _add_button_guides(housing, material) -> None:
    rail_x = (BUTTON_WIDTH - BUTTON_STEM_WIDTH) * 0.5
    rail_z = (BUTTON_HEIGHT - BUTTON_STEM_HEIGHT) * 0.5
    guide_center_y = HOOD_DEPTH - BODY_SHELL_THICKNESS - BUTTON_GUIDE_DEPTH * 0.5

    for row_index, z_pos in enumerate(BUTTON_ROW_Z):
        for column_index, x_pos in enumerate(BUTTON_COLUMN_X):
            suffix = f"r{row_index}_c{column_index}"
            housing.visual(
                Box((rail_x, BUTTON_GUIDE_DEPTH, BUTTON_HEIGHT)),
                origin=Origin(
                    xyz=(
                        x_pos - (BUTTON_STEM_WIDTH * 0.5 + rail_x * 0.5),
                        guide_center_y,
                        z_pos,
                    ),
                ),
                material=material,
                name=f"guide_left_{suffix}",
            )
            housing.visual(
                Box((rail_x, BUTTON_GUIDE_DEPTH, BUTTON_HEIGHT)),
                origin=Origin(
                    xyz=(
                        x_pos + (BUTTON_STEM_WIDTH * 0.5 + rail_x * 0.5),
                        guide_center_y,
                        z_pos,
                    ),
                ),
                material=material,
                name=f"guide_right_{suffix}",
            )
            housing.visual(
                Box((BUTTON_STEM_WIDTH, BUTTON_GUIDE_DEPTH, rail_z)),
                origin=Origin(
                    xyz=(
                        x_pos,
                        guide_center_y,
                        z_pos + (BUTTON_STEM_HEIGHT * 0.5 + rail_z * 0.5),
                    ),
                ),
                material=material,
                name=f"guide_top_{suffix}",
            )
            housing.visual(
                Box((BUTTON_STEM_WIDTH, BUTTON_GUIDE_DEPTH, rail_z)),
                origin=Origin(
                    xyz=(
                        x_pos,
                        guide_center_y,
                        z_pos - (BUTTON_STEM_HEIGHT * 0.5 + rail_z * 0.5),
                    ),
                ),
                material=material,
                name=f"guide_bottom_{suffix}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.28, 0.30, 0.32, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    button_plastic = model.material("button_plastic", rgba=(0.18, 0.18, 0.19, 1.0))

    housing = model.part("housing")
    _add_front_face(housing, stainless)
    _add_button_guides(housing, stainless)
    housing.visual(
        Box((HOOD_WIDTH, HOOD_DEPTH, BODY_SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, HOOD_DEPTH * 0.5, HOOD_HEIGHT - BODY_SHELL_THICKNESS * 0.5)),
        material=stainless,
        name="top_panel",
    )
    housing.visual(
        Box((HOOD_WIDTH, BODY_SHELL_THICKNESS, HOOD_HEIGHT - BODY_SHELL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, BODY_SHELL_THICKNESS * 0.5, (HOOD_HEIGHT - BODY_SHELL_THICKNESS) * 0.5),
        ),
        material=stainless,
        name="back_panel",
    )
    housing.visual(
        Box((BODY_SHELL_THICKNESS, HOOD_DEPTH, HOOD_HEIGHT - BODY_SHELL_THICKNESS)),
        origin=Origin(
            xyz=(
                -HOOD_WIDTH * 0.5 + BODY_SHELL_THICKNESS * 0.5,
                HOOD_DEPTH * 0.5,
                (HOOD_HEIGHT - BODY_SHELL_THICKNESS) * 0.5,
            ),
        ),
        material=stainless,
        name="left_side_panel",
    )
    housing.visual(
        Box((BODY_SHELL_THICKNESS, HOOD_DEPTH, HOOD_HEIGHT - BODY_SHELL_THICKNESS)),
        origin=Origin(
            xyz=(
                HOOD_WIDTH * 0.5 - BODY_SHELL_THICKNESS * 0.5,
                HOOD_DEPTH * 0.5,
                (HOOD_HEIGHT - BODY_SHELL_THICKNESS) * 0.5,
            ),
        ),
        material=stainless,
        name="right_side_panel",
    )
    housing.visual(
        Box(
            (
                HOOD_WIDTH - 2.0 * BODY_SHELL_THICKNESS,
                HOOD_DEPTH - 2.0 * BODY_SHELL_THICKNESS,
                FILTER_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                HOOD_DEPTH * 0.5,
                FILTER_BOTTOM_OFFSET + FILTER_THICKNESS * 0.5,
            ),
        ),
        material=brushed_dark,
        name="filter_plane",
    )

    chimney_center_z = HOOD_HEIGHT + CHIMNEY_HEIGHT * 0.5
    housing.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_SHELL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_DEPTH - CHIMNEY_SHELL_THICKNESS * 0.5,
                chimney_center_z,
            ),
        ),
        material=stainless,
        name="chimney_front",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_SHELL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CHIMNEY_SHELL_THICKNESS * 0.5, chimney_center_z),
        ),
        material=stainless,
        name="chimney_back",
    )
    housing.visual(
        Box((CHIMNEY_SHELL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                -CHIMNEY_WIDTH * 0.5 + CHIMNEY_SHELL_THICKNESS * 0.5,
                CHIMNEY_DEPTH * 0.5,
                chimney_center_z,
            ),
        ),
        material=stainless,
        name="chimney_left",
    )
    housing.visual(
        Box((CHIMNEY_SHELL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                CHIMNEY_WIDTH * 0.5 - CHIMNEY_SHELL_THICKNESS * 0.5,
                CHIMNEY_DEPTH * 0.5,
                chimney_center_z,
            ),
        ),
        material=stainless,
        name="chimney_right",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_SHELL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_DEPTH * 0.5,
                HOOD_HEIGHT + CHIMNEY_HEIGHT - CHIMNEY_SHELL_THICKNESS * 0.5,
            ),
        ),
        material=charcoal,
        name="chimney_top",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOOD_WIDTH, HOOD_DEPTH, HOOD_HEIGHT + CHIMNEY_HEIGHT)),
        mass=23.0,
        origin=Origin(xyz=(0.0, HOOD_DEPTH * 0.5, (HOOD_HEIGHT + CHIMNEY_HEIGHT) * 0.5)),
    )

    for row_index, _ in enumerate(BUTTON_ROW_Z):
        for column_index, _ in enumerate(BUTTON_COLUMN_X):
            button = model.part(_button_part_name(row_index, column_index))
            button.visual(
                Box((BUTTON_WIDTH, BUTTON_FACE_DEPTH, BUTTON_HEIGHT)),
                material=button_plastic,
                name="button_face",
            )
            button.visual(
                Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
                origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH * 0.5, 0.0)),
                material=button_plastic,
                name="button_stem",
            )
            button.inertial = Inertial.from_geometry(
                Box((BUTTON_WIDTH, BUTTON_FACE_DEPTH + BUTTON_STEM_DEPTH, BUTTON_HEIGHT)),
                mass=0.03,
                origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH * 0.5, 0.0)),
            )
            model.articulation(
                _button_joint_name(row_index, column_index),
                ArticulationType.PRISMATIC,
                parent=housing,
                child=button,
                origin=Origin(xyz=_button_center_xyz(row_index, column_index)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=0.06,
                    lower=0.0,
                    upper=BUTTON_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    buttons: dict[tuple[int, int], object] = {}
    button_faces: dict[tuple[int, int], object] = {}
    joints: dict[tuple[int, int], object] = {}
    for row_index, _ in enumerate(BUTTON_ROW_Z):
        for column_index, _ in enumerate(BUTTON_COLUMN_X):
            key = (row_index, column_index)
            button = object_model.get_part(_button_part_name(row_index, column_index))
            buttons[key] = button
            button_faces[key] = button.get_visual("button_face")
            joints[key] = object_model.get_articulation(_button_joint_name(row_index, column_index))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is None:
        ctx.fail("housing_bounds_exist", "housing AABB is unavailable")
    else:
        min_corner, max_corner = housing_aabb
        width = max_corner[0] - min_corner[0]
        depth = max_corner[1] - min_corner[1]
        height = max_corner[2] - min_corner[2]
        ctx.check(
            "housing_width_realistic",
            0.88 <= width <= 0.92,
            f"expected about 0.90 m width, got {width:.4f} m",
        )
        ctx.check(
            "housing_depth_realistic",
            0.49 <= depth <= 0.51,
            f"expected about 0.50 m depth, got {depth:.4f} m",
        )
        ctx.check(
            "housing_height_realistic",
            0.79 <= height <= 0.81,
            f"expected about 0.80 m height, got {height:.4f} m",
        )

    with ctx.pose():
        button_positions = {key: ctx.part_world_position(button) for key, button in buttons.items()}
        if any(position is None for position in button_positions.values()):
            ctx.fail("button_positions_available", "one or more button positions are unavailable")
        else:
            mean_x = sum(position[0] for position in button_positions.values()) / len(button_positions)
            mean_z = sum(position[2] for position in button_positions.values()) / len(button_positions)
            ctx.check(
                "button_grid_centered_x",
                abs(mean_x) <= 1e-6,
                f"expected centered button grid in x, got mean x {mean_x:.6f}",
            )
            ctx.check(
                "button_grid_centered_z",
                abs(mean_z - HOOD_HEIGHT * 0.5) <= 1e-6,
                f"expected centered button grid in z, got mean z {mean_z:.6f}",
            )

            top_row = [button_positions[(0, col)] for col, _ in enumerate(BUTTON_COLUMN_X)]
            bottom_row = [button_positions[(1, col)] for col, _ in enumerate(BUTTON_COLUMN_X)]
            ctx.check(
                "button_column_pitch_top",
                abs((top_row[1][0] - top_row[0][0]) - 0.050) <= 1e-6
                and abs((top_row[2][0] - top_row[1][0]) - 0.050) <= 1e-6,
                f"top-row pitch is incorrect: {top_row}",
            )
            ctx.check(
                "button_column_pitch_bottom",
                abs((bottom_row[1][0] - bottom_row[0][0]) - 0.050) <= 1e-6
                and abs((bottom_row[2][0] - bottom_row[1][0]) - 0.050) <= 1e-6,
                f"bottom-row pitch is incorrect: {bottom_row}",
            )
            ctx.check(
                "button_row_pitch",
                abs((top_row[0][2] - bottom_row[0][2]) - 0.030) <= 1e-6,
                f"expected 30 mm row spacing, got {top_row[0][2] - bottom_row[0][2]:.6f}",
            )
            lower_right_y = button_positions[(1, 2)][1]
            other_y = [position[1] for key, position in button_positions.items() if key != (1, 2)]
            ctx.check(
                "lower_right_button_offset_outward",
                lower_right_y - max(other_y) >= 0.0025,
                f"expected the lower-right button to sit outward, got delta {lower_right_y - max(other_y):.6f}",
            )

    for row_index, _ in enumerate(BUTTON_ROW_Z):
        for column_index, _ in enumerate(BUTTON_COLUMN_X):
            key = (row_index, column_index)
            button = buttons[key]
            button_face = button_faces[key]
            joint = joints[key]
            limits = joint.motion_limits

            ctx.check(
                f"{button.name}_axis_direction",
                tuple(joint.axis) == (0.0, -1.0, 0.0),
                f"expected inward button axis (0, -1, 0), got {joint.axis}",
            )

            with ctx.pose({joint: 0.0}):
                if key == (1, 2):
                    ctx.expect_gap(
                        button,
                        housing,
                        axis="y",
                        positive_elem=button_face,
                        min_gap=0.0025,
                        max_gap=0.0035,
                        name=f"{button.name}_rest_face_offset",
                    )
                else:
                    ctx.expect_contact(
                        button,
                        housing,
                        elem_a=button_face,
                        name=f"{button.name}_rest_contact_housing",
                    )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button.name}_rest_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button.name}_rest_no_floating")
                rest_position = ctx.part_world_position(button)

            if limits is not None and limits.upper is not None:
                with ctx.pose({joint: limits.upper}):
                    ctx.expect_contact(
                        button,
                        housing,
                        name=f"{button.name}_pressed_contact_housing",
                    )
                    ctx.fail_if_parts_overlap_in_current_pose(name=f"{button.name}_pressed_no_overlap")
                    ctx.fail_if_isolated_parts(name=f"{button.name}_pressed_no_floating")
                    pressed_position = ctx.part_world_position(button)

                if rest_position is None or pressed_position is None:
                    ctx.fail(f"{button.name}_motion_position_available", "button world position is unavailable")
                else:
                    ctx.check(
                        f"{button.name}_moves_only_inward",
                        abs(pressed_position[0] - rest_position[0]) <= 1e-6
                        and abs(pressed_position[2] - rest_position[2]) <= 1e-6
                        and abs((rest_position[1] - pressed_position[1]) - limits.upper) <= 1e-6,
                        (
                            "expected pure inward travel; "
                            f"rest={rest_position}, pressed={pressed_position}, upper={limits.upper}"
                        ),
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    Sphere,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 1.26
CABINET_DEPTH = 0.58
PLINTH_HEIGHT = 0.10
CARCASS_HEIGHT = 1.22
TOP_CAP_THICKNESS = 0.018
SIDE_THICKNESS = 0.028
TOP_BOTTOM_THICKNESS = 0.030
BACK_THICKNESS = 0.012
CENTER_DIVIDER_THICKNESS = 0.024
SHELF_THICKNESS = 0.022
DRAWER_ROWS = 5
DRAWER_COLUMNS = 2

OPENING_HEIGHT = (
    CARCASS_HEIGHT
    - (2.0 * TOP_BOTTOM_THICKNESS)
    - ((DRAWER_ROWS - 1) * SHELF_THICKNESS)
) / DRAWER_ROWS
COMPARTMENT_WIDTH = (
    CABINET_WIDTH
    - (2.0 * SIDE_THICKNESS)
    - CENTER_DIVIDER_THICKNESS
) / DRAWER_COLUMNS

DRAWER_FACE_WIDTH = COMPARTMENT_WIDTH - 0.008
DRAWER_BOX_WIDTH = DRAWER_FACE_WIDTH - 0.004
DRAWER_FACE_HEIGHT = OPENING_HEIGHT - 0.022
DRAWER_BOX_HEIGHT = 0.182
DRAWER_FACE_THICKNESS = 0.018
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BACK_THICKNESS = 0.010
DRAWER_BOTTOM_THICKNESS = 0.009
DRAWER_BODY_DEPTH = 0.500
DRAWER_TRAVEL = 0.250

RUNNER_WIDTH = 0.024
RUNNER_HEIGHT = 0.016
RUNNER_FELT_THICKNESS = 0.003
RUNNER_DEPTH = 0.460
RUNNER_FRONT_SETBACK = 0.024

DRAWER_FACE_BOTTOM_BELOW_SUPPORT = 0.008


def _drawer_name(row: int, column: int) -> str:
    return f"drawer_r{row}_c{column}"


def _drawer_joint_name(row: int, column: int) -> str:
    return f"carcass_to_drawer_r{row}_c{column}"


def _opening_bottom_z(row: int) -> float:
    return (
        PLINTH_HEIGHT
        + TOP_BOTTOM_THICKNESS
        + row * (OPENING_HEIGHT + SHELF_THICKNESS)
    )


def _drawer_support_z(row: int) -> float:
    return _opening_bottom_z(row) + RUNNER_HEIGHT + RUNNER_FELT_THICKNESS


def _column_center_x(column: int) -> float:
    left_opening_min = (-CABINET_WIDTH * 0.5) + SIDE_THICKNESS
    step = COMPARTMENT_WIDTH + CENTER_DIVIDER_THICKNESS
    return left_opening_min + (COMPARTMENT_WIDTH * 0.5) + column * step


def _opening_side_x(column: int) -> tuple[float, float]:
    center_x = _column_center_x(column)
    return (
        center_x - (COMPARTMENT_WIDTH * 0.5),
        center_x + (COMPARTMENT_WIDTH * 0.5),
    )


def _runner_center_y() -> float:
    runner_front_y = (CABINET_DEPTH * 0.5) - RUNNER_FRONT_SETBACK
    return runner_front_y - (RUNNER_DEPTH * 0.5)


def _add_drawer_visuals(drawer_part, *, material, brass_material, face_z0: float) -> None:
    knob_center_z = face_z0 + (DRAWER_FACE_HEIGHT * 0.5)

    drawer_part.visual(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(DRAWER_FACE_THICKNESS * 0.5),
                face_z0 + (DRAWER_FACE_HEIGHT * 0.5),
            )
        ),
        material=material,
        name="front",
    )
    drawer_part.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DRAWER_BOX_WIDTH * 0.5) + (DRAWER_SIDE_THICKNESS * 0.5),
                -DRAWER_FACE_THICKNESS - (DRAWER_BODY_DEPTH * 0.5),
                DRAWER_BOX_HEIGHT * 0.5,
            )
        ),
        material=material,
        name="left_side",
    )
    drawer_part.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                (DRAWER_BOX_WIDTH * 0.5) - (DRAWER_SIDE_THICKNESS * 0.5),
                -DRAWER_FACE_THICKNESS - (DRAWER_BODY_DEPTH * 0.5),
                DRAWER_BOX_HEIGHT * 0.5,
            )
        ),
        material=material,
        name="right_side",
    )
    drawer_part.visual(
        Box(
            (
                DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
                DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -DRAWER_FACE_THICKNESS
                - ((DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS) * 0.5),
                DRAWER_BOTTOM_THICKNESS * 0.5,
            )
        ),
        material=material,
        name="bottom",
    )
    drawer_part.visual(
        Box((DRAWER_BOX_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -DRAWER_FACE_THICKNESS
                - DRAWER_BODY_DEPTH
                + (DRAWER_BACK_THICKNESS * 0.5),
                DRAWER_BOX_HEIGHT * 0.5,
            )
        ),
        material=material,
        name="back",
    )

    drawer_part.visual(
        Cylinder(radius=0.017, length=0.003),
        origin=Origin(
            xyz=(0.0, 0.0015, knob_center_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass_material,
        name="knob_backplate",
    )
    drawer_part.visual(
        Cylinder(radius=0.0042, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.011, knob_center_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass_material,
        name="knob_stem",
    )
    drawer_part.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.0, 0.032, knob_center_z)),
        material=brass_material,
        name="knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="museum_specimen_cabinet")

    case_wood = model.material("case_wood", rgba=(0.43, 0.29, 0.18, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.48, 0.33, 0.20, 1.0))
    plinth_wood = model.material("plinth_wood", rgba=(0.33, 0.22, 0.14, 1.0))
    felt = model.material("felt_green", rgba=(0.22, 0.30, 0.24, 1.0))
    brass = model.material("aged_brass", rgba=(0.77, 0.65, 0.31, 1.0))

    carcass = model.part("carcass")

    panel_depth = CABINET_DEPTH - BACK_THICKNESS
    panel_center_y = BACK_THICKNESS * 0.5
    carcass_center_z = PLINTH_HEIGHT + (CARCASS_HEIGHT * 0.5)

    carcass.visual(
        Box((SIDE_THICKNESS, panel_depth, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH * 0.5) + (SIDE_THICKNESS * 0.5),
                panel_center_y,
                carcass_center_z,
            )
        ),
        material=case_wood,
        name="left_side_panel",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, panel_depth, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH * 0.5) - (SIDE_THICKNESS * 0.5),
                panel_center_y,
                carcass_center_z,
            )
        ),
        material=case_wood,
        name="right_side_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), panel_depth, TOP_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                panel_center_y,
                PLINTH_HEIGHT + (TOP_BOTTOM_THICKNESS * 0.5),
            )
        ),
        material=case_wood,
        name="bottom_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), panel_depth, TOP_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                panel_center_y,
                PLINTH_HEIGHT + CARCASS_HEIGHT - (TOP_BOTTOM_THICKNESS * 0.5),
            )
        ),
        material=case_wood,
        name="top_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), BACK_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH * 0.5) + (BACK_THICKNESS * 0.5),
                carcass_center_z,
            )
        ),
        material=case_wood,
        name="back_panel",
    )
    carcass.visual(
        Box((CENTER_DIVIDER_THICKNESS, panel_depth, CARCASS_HEIGHT)),
        origin=Origin(xyz=(0.0, panel_center_y, carcass_center_z)),
        material=case_wood,
        name="center_divider",
    )

    for shelf_index in range(DRAWER_ROWS - 1):
        shelf_bottom_z = (
            PLINTH_HEIGHT
            + TOP_BOTTOM_THICKNESS
            + (shelf_index + 1) * OPENING_HEIGHT
            + shelf_index * SHELF_THICKNESS
        )
        carcass.visual(
            Box(
                (
                    CABINET_WIDTH - (2.0 * SIDE_THICKNESS),
                    panel_depth,
                    SHELF_THICKNESS,
                )
            ),
            origin=Origin(
                xyz=(0.0, panel_center_y, shelf_bottom_z + (SHELF_THICKNESS * 0.5))
            ),
            material=case_wood,
            name=f"shelf_{shelf_index}",
        )

    plinth_width = CABINET_WIDTH - 0.110
    plinth_depth = CABINET_DEPTH - 0.080
    carcass.visual(
        Box((plinth_width, plinth_depth, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.015,
                PLINTH_HEIGHT * 0.5,
            )
        ),
        material=plinth_wood,
        name="plinth",
    )
    carcass.visual(
        Box(
            (
                CABINET_WIDTH + 0.012,
                CABINET_DEPTH + 0.008,
                TOP_CAP_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.002,
                PLINTH_HEIGHT + CARCASS_HEIGHT + (TOP_CAP_THICKNESS * 0.5),
            )
        ),
        material=case_wood,
        name="top_cap",
    )

    runner_center_y = _runner_center_y()
    for row in range(DRAWER_ROWS):
        runner_bottom_z = _opening_bottom_z(row)
        felt_center_z = runner_bottom_z + RUNNER_HEIGHT + (RUNNER_FELT_THICKNESS * 0.5)
        wood_center_z = runner_bottom_z + (RUNNER_HEIGHT * 0.5)
        for column in range(DRAWER_COLUMNS):
            opening_left_x, opening_right_x = _opening_side_x(column)
            left_runner_center_x = opening_left_x + (RUNNER_WIDTH * 0.5)
            right_runner_center_x = opening_right_x - (RUNNER_WIDTH * 0.5)
            for side_name, center_x in (
                ("left", left_runner_center_x),
                ("right", right_runner_center_x),
            ):
                carcass.visual(
                    Box((RUNNER_WIDTH, RUNNER_DEPTH, RUNNER_HEIGHT)),
                    origin=Origin(xyz=(center_x, runner_center_y, wood_center_z)),
                    material=case_wood,
                    name=f"runner_wood_r{row}_c{column}_{side_name}",
                )
                carcass.visual(
                    Box((RUNNER_WIDTH, RUNNER_DEPTH, RUNNER_FELT_THICKNESS)),
                    origin=Origin(xyz=(center_x, runner_center_y, felt_center_z)),
                    material=felt,
                    name=f"runner_felt_r{row}_c{column}_{side_name}",
                )

    carcass.inertial = Inertial.from_geometry(
        Box(
            (
                CABINET_WIDTH + 0.012,
                CABINET_DEPTH + 0.008,
                PLINTH_HEIGHT + CARCASS_HEIGHT + TOP_CAP_THICKNESS,
            )
        ),
        mass=78.0,
        origin=Origin(
            xyz=(
                0.0,
                0.002,
                (PLINTH_HEIGHT + CARCASS_HEIGHT + TOP_CAP_THICKNESS) * 0.5,
            )
        ),
    )

    for row in range(DRAWER_ROWS):
        face_z0 = -DRAWER_FACE_BOTTOM_BELOW_SUPPORT
        drawer_part = model.part(_drawer_name(row, 0))  # temporary to satisfy typing
        model.parts.pop()
        for column in range(DRAWER_COLUMNS):
            drawer_part = model.part(_drawer_name(row, column))
            _add_drawer_visuals(
                drawer_part,
                material=drawer_wood,
                brass_material=brass,
                face_z0=face_z0,
            )
            drawer_part.inertial = Inertial.from_geometry(
                Box((DRAWER_FACE_WIDTH, DRAWER_BODY_DEPTH + 0.040, DRAWER_FACE_HEIGHT)),
                mass=4.2,
                origin=Origin(
                    xyz=(
                        0.0,
                        -(DRAWER_FACE_THICKNESS + (DRAWER_BODY_DEPTH * 0.5)),
                        face_z0 + (DRAWER_FACE_HEIGHT * 0.5),
                    )
                ),
            )
            model.articulation(
                _drawer_joint_name(row, column),
                ArticulationType.PRISMATIC,
                parent=carcass,
                child=drawer_part,
                origin=Origin(
                    xyz=(
                        _column_center_x(column),
                        CABINET_DEPTH * 0.5,
                        _drawer_support_z(row),
                    )
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=40.0,
                    velocity=0.35,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    carcass = object_model.get_part("carcass")
    sample_bottom_left = object_model.get_part(_drawer_name(0, 0))
    sample_top_right = object_model.get_part(_drawer_name(DRAWER_ROWS - 1, DRAWER_COLUMNS - 1))
    sample_bottom_left_joint = object_model.get_articulation(_drawer_joint_name(0, 0))
    sample_top_right_joint = object_model.get_articulation(
        _drawer_joint_name(DRAWER_ROWS - 1, DRAWER_COLUMNS - 1)
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for row in range(DRAWER_ROWS):
        for column in range(DRAWER_COLUMNS):
            drawer = object_model.get_part(_drawer_name(row, column))
            ctx.expect_contact(
                drawer,
                carcass,
                name=f"{drawer.name}_rests_on_runners",
            )
            ctx.expect_within(
                drawer,
                carcass,
                axes="xz",
                margin=0.0,
                name=f"{drawer.name}_stays_within_stack_width_and_height",
            )

    bottom_left_rest = ctx.part_world_position(sample_bottom_left)
    top_right_rest = ctx.part_world_position(sample_top_right)
    assert bottom_left_rest is not None
    assert top_right_rest is not None

    with ctx.pose({sample_bottom_left_joint: DRAWER_TRAVEL}):
        moved = ctx.part_world_position(sample_bottom_left)
        assert moved is not None
        ctx.check(
            "bottom_left_drawer_slides_forward",
            moved[1] > bottom_left_rest[1] + 0.20,
            details=f"expected >0.20 m forward travel, got {moved[1] - bottom_left_rest[1]:.4f} m",
        )
        ctx.expect_contact(
            sample_bottom_left,
            carcass,
            name="bottom_left_drawer_keeps_runner_contact_open",
        )
        ctx.expect_within(
            sample_bottom_left,
            carcass,
            axes="xz",
            margin=0.0,
            name="bottom_left_drawer_remains_aligned_open",
        )

    with ctx.pose({sample_top_right_joint: DRAWER_TRAVEL * 0.8}):
        moved = ctx.part_world_position(sample_top_right)
        assert moved is not None
        ctx.check(
            "top_right_drawer_slides_forward",
            moved[1] > top_right_rest[1] + 0.18,
            details=f"expected >0.18 m forward travel, got {moved[1] - top_right_rest[1]:.4f} m",
        )
        ctx.expect_contact(
            sample_top_right,
            carcass,
            name="top_right_drawer_keeps_runner_contact_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

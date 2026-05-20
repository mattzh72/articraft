from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ValidationError,
)


CABINET_WIDTH = 0.98
CABINET_DEPTH = 0.48
CABINET_HEIGHT = 1.82
SIDE_THICKNESS = 0.018
TOP_THICKNESS = 0.022
BACK_THICKNESS = 0.012
DIVIDER_THICKNESS = 0.012
SHELF_THICKNESS = 0.010
BOTTOM_DECK_THICKNESS = 0.022
PLINTH_HEIGHT = 0.10

ROWS = 12
COLS = 4

INTERIOR_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
CELL_WIDTH = (INTERIOR_WIDTH - DIVIDER_THICKNESS * (COLS - 1)) / COLS
CELL_HEIGHT = 0.121
BAY_Z0 = 0.146
DRAWER_TRAVEL = 0.18

DRAWER_FRONT_WIDTH = 0.219
DRAWER_FRONT_HEIGHT = 0.117
DRAWER_FRONT_THICKNESS = 0.018
DRAWER_BODY_DEPTH = 0.396
DRAWER_SIDE_THICKNESS = 0.006
DRAWER_BODY_WIDTH = 0.204
DRAWER_SIDE_HEIGHT = 0.095
DRAWER_BOTTOM_THICKNESS = 0.006
DRAWER_BACK_THICKNESS = 0.006

RAIL_WIDTH = 0.010
RAIL_HEIGHT = 0.008
RAIL_LENGTH = 0.35
RAIL_CENTER_Y = -0.215
DRAWER_FRAME_TO_RAIL_CENTER_Z = -0.0515


def _drawer_name(row: int, col: int) -> str:
    return f"drawer_r{row + 1:02d}_c{col + 1:02d}"


def _joint_name(row: int, col: int) -> str:
    return f"cabinet_to_drawer_r{row + 1:02d}_c{col + 1:02d}"


def _rail_name(row: int, col: int, side: str) -> str:
    return f"rail_r{row + 1:02d}_c{col + 1:02d}_{side}"


def _cell_left_x(col: int) -> float:
    return -CABINET_WIDTH * 0.5 + SIDE_THICKNESS + col * (CELL_WIDTH + DIVIDER_THICKNESS)


def _cell_center_x(col: int) -> float:
    return _cell_left_x(col) + CELL_WIDTH * 0.5


def _cell_bottom_z(row: int) -> float:
    return BAY_Z0 + row * (CELL_HEIGHT + SHELF_THICKNESS)


def _cell_center_z(row: int) -> float:
    return _cell_bottom_z(row) + CELL_HEIGHT * 0.5


def _add_drawer(model: ArticulatedObject, *, row: int, col: int, body, drawer_material, trim_material) -> None:
    drawer = model.part(_drawer_name(row, col))

    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=drawer_material,
        name="front_panel",
    )
    drawer.visual(
        Box((DRAWER_BODY_WIDTH, DRAWER_BODY_DEPTH, DRAWER_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.196, -0.048)),
        material=drawer_material,
        name="bottom_plate",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_SIDE_HEIGHT)),
        origin=Origin(xyz=(-0.105, -0.196, 0.0)),
        material=drawer_material,
        name="left_wall",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_SIDE_HEIGHT)),
        origin=Origin(xyz=(0.105, -0.196, 0.0)),
        material=drawer_material,
        name="right_wall",
    )
    drawer.visual(
        Box((DRAWER_BODY_WIDTH, DRAWER_BACK_THICKNESS, 0.093)),
        origin=Origin(xyz=(0.0, -0.391, -0.001)),
        material=drawer_material,
        name="back_wall",
    )
    drawer.visual(
        Box((0.102, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.022, 0.028)),
        material=trim_material,
        name="label_holder",
    )
    drawer.visual(
        Box((0.084, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.025, -0.015)),
        material=trim_material,
        name="pull_bar",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_WIDTH, DRAWER_BODY_DEPTH + 0.03, DRAWER_FRONT_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.16, 0.0)),
    )

    model.articulation(
        _joint_name(row, col),
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(_cell_center_x(col), 0.0, _cell_center_z(row))),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_dispensary_multi_bin_cabinet")

    body_material = model.material("body_offwhite", rgba=(0.88, 0.89, 0.85, 1.0))
    drawer_material = model.material("drawer_beige", rgba=(0.80, 0.76, 0.68, 1.0))
    trim_material = model.material("trim_grey", rgba=(0.43, 0.45, 0.46, 1.0))
    rail_material = model.material("rail_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    plinth_material = model.material("plinth_shadow", rgba=(0.24, 0.24, 0.23, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH * 0.5 + SIDE_THICKNESS * 0.5,
                -CABINET_DEPTH * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=body_material,
        name="left_side",
    )
    body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH * 0.5 - SIDE_THICKNESS * 0.5,
                -CABINET_DEPTH * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=body_material,
        name="right_side",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, -CABINET_DEPTH * 0.5, CABINET_HEIGHT - TOP_THICKNESS * 0.5)),
        material=body_material,
        name="top_panel",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH + BACK_THICKNESS * 0.5,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=body_material,
        name="back_panel",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH - BACK_THICKNESS, BOTTOM_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH - BACK_THICKNESS) * 0.5,
                BAY_Z0 - BOTTOM_DECK_THICKNESS * 0.5,
            )
        ),
        material=body_material,
        name="bottom_deck",
    )
    body.visual(
        Box((CABINET_WIDTH - 0.08, 0.020, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.010, PLINTH_HEIGHT * 0.5)),
        material=plinth_material,
        name="front_plinth",
    )
    body.visual(
        Box((0.042, 0.120, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(-0.451, -0.060, PLINTH_HEIGHT * 0.5)
        ),
        material=plinth_material,
        name="left_plinth_return",
    )
    body.visual(
        Box((0.042, 0.120, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(0.451, -0.060, PLINTH_HEIGHT * 0.5)
        ),
        material=plinth_material,
        name="right_plinth_return",
    )

    stack_top_z = BAY_Z0 + ROWS * CELL_HEIGHT + (ROWS - 1) * SHELF_THICKNESS
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH - BACK_THICKNESS, SHELF_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH - BACK_THICKNESS) * 0.5,
                stack_top_z + SHELF_THICKNESS * 0.5,
            )
        ),
        material=body_material,
        name="header_shelf",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.009, stack_top_z + 0.050)),
        material=body_material,
        name="header_face",
    )

    for divider_index in range(COLS - 1):
        divider_x = -CABINET_WIDTH * 0.5 + SIDE_THICKNESS + CELL_WIDTH * (divider_index + 1) + DIVIDER_THICKNESS * divider_index + DIVIDER_THICKNESS * 0.5
        body.visual(
            Box(
                (
                    DIVIDER_THICKNESS,
                    CABINET_DEPTH - BACK_THICKNESS,
                    stack_top_z + SHELF_THICKNESS - BAY_Z0,
                )
            ),
            origin=Origin(
                xyz=(
                    divider_x,
                    -(CABINET_DEPTH - BACK_THICKNESS) * 0.5,
                    BAY_Z0 + (stack_top_z + SHELF_THICKNESS - BAY_Z0) * 0.5,
                )
            ),
            material=body_material,
            name=f"divider_{divider_index + 1}",
        )

    for shelf_index in range(ROWS - 1):
        shelf_z = BAY_Z0 + (shelf_index + 1) * CELL_HEIGHT + shelf_index * SHELF_THICKNESS + SHELF_THICKNESS * 0.5
        body.visual(
            Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH - BACK_THICKNESS, SHELF_THICKNESS)),
            origin=Origin(xyz=(0.0, -(CABINET_DEPTH - BACK_THICKNESS) * 0.5, shelf_z)),
            material=body_material,
            name=f"shelf_{shelf_index + 1:02d}",
        )

    for row in range(ROWS):
        cell_bottom = _cell_bottom_z(row)
        rail_z = _cell_center_z(row) + DRAWER_FRAME_TO_RAIL_CENTER_Z
        for col in range(COLS):
            left_x = _cell_left_x(col)
            right_x = left_x + CELL_WIDTH
            body.visual(
                Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
                origin=Origin(xyz=(left_x + RAIL_WIDTH * 0.5, RAIL_CENTER_Y, rail_z)),
                material=rail_material,
                name=_rail_name(row, col, "l"),
            )
            body.visual(
                Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
                origin=Origin(xyz=(right_x - RAIL_WIDTH * 0.5, RAIL_CENTER_Y, rail_z)),
                material=rail_material,
                name=_rail_name(row, col, "r"),
            )

    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, -CABINET_DEPTH * 0.5, CABINET_HEIGHT * 0.5)),
    )

    for row in range(ROWS):
        for col in range(COLS):
            _add_drawer(
                model,
                row=row,
                col=col,
                body=body,
                drawer_material=drawer_material,
                trim_material=trim_material,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")

    missing_parts: list[str] = []
    missing_joints: list[str] = []
    bad_joints: list[str] = []
    all_drawer_parts = []
    all_drawer_joints = []

    for row in range(ROWS):
        for col in range(COLS):
            drawer_name = _drawer_name(row, col)
            joint_name = _joint_name(row, col)
            try:
                drawer = object_model.get_part(drawer_name)
                all_drawer_parts.append(drawer)
            except ValidationError:
                missing_parts.append(drawer_name)
                drawer = None

            try:
                joint = object_model.get_articulation(joint_name)
                all_drawer_joints.append(joint)
            except ValidationError:
                missing_joints.append(joint_name)
                joint = None

            if drawer is not None and joint is not None:
                limits = joint.motion_limits
                if (
                    joint.articulation_type != ArticulationType.PRISMATIC
                    or joint.parent != "cabinet_body"
                    or joint.child != drawer_name
                    or joint.axis != (0.0, 1.0, 0.0)
                    or limits is None
                    or limits.lower != 0.0
                    or limits.upper != DRAWER_TRAVEL
                ):
                    bad_joints.append(joint_name)

    ctx.check(
        "all drawers and slide joints are present",
        not missing_parts and not missing_joints,
        details=f"missing_parts={missing_parts}, missing_joints={missing_joints}",
    )
    ctx.check(
        "all drawer joints use forward prismatic travel",
        not bad_joints,
        details=f"bad_joints={bad_joints}",
    )

    representative_slots = ((0, 0), (5, 1), (11, 3))
    representative_pose = {}
    representative_position_errors: list[str] = []

    for row, col in representative_slots:
        drawer = object_model.get_part(_drawer_name(row, col))
        joint = object_model.get_articulation(_joint_name(row, col))
        representative_pose[joint] = DRAWER_TRAVEL

        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            margin=0.0,
            name=f"{drawer.name} stays inside cabinet width and height when closed",
        )
        ctx.expect_gap(
            drawer,
            body,
            axis="y",
            positive_elem="front_panel",
            min_gap=0.0015,
            max_gap=0.0025,
            name=f"{drawer.name} front sits slightly proud of the cabinet face",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="left_wall",
            elem_b=_rail_name(row, col, "l"),
            name=f"{drawer.name} left wall bears on its left guide rail",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="right_wall",
            elem_b=_rail_name(row, col, "r"),
            name=f"{drawer.name} right wall bears on its right guide rail",
        )

    closed_positions = {drawer.name: ctx.part_world_position(drawer) for drawer in all_drawer_parts}
    with ctx.pose({joint: DRAWER_TRAVEL for joint in all_drawer_joints}):
        for row, col in representative_slots:
            drawer = object_model.get_part(_drawer_name(row, col))
            ctx.expect_within(
                drawer,
                body,
                axes="xz",
                margin=0.0,
                name=f"{drawer.name} stays aligned in cabinet tracks when open",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                min_overlap=0.21,
                name=f"{drawer.name} retains insertion at full extension",
            )

        for drawer in all_drawer_parts:
            closed_pos = closed_positions.get(drawer.name)
            open_pos = ctx.part_world_position(drawer)
            if (
                closed_pos is None
                or open_pos is None
                or open_pos[1] <= closed_pos[1] + DRAWER_TRAVEL - 0.002
            ):
                representative_position_errors.append(
                    f"{drawer.name}: closed={closed_pos}, open={open_pos}"
                )

    ctx.check(
        "all drawers extend outward along the cabinet depth axis",
        not representative_position_errors,
        details="; ".join(representative_position_errors[:12]),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

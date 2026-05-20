from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.72
CABINET_DEPTH = 0.42
CABINET_HEIGHT = 0.84
SIDE_WALL = 0.018
BACK_PANEL = 0.010
CENTER_DIVIDER = 0.020
FACE_FRAME_DEPTH = 0.020
LID_THICKNESS = 0.018

TRAY_FLOOR_BOTTOM = 0.70
TRAY_FLOOR_THICKNESS = 0.012
TRAY_FRONT_WALL = 0.050
TRAY_DIVIDER = 0.010

BASE_RAIL_HEIGHT = 0.055
MID_RAIL_HEIGHT = 0.030
TOP_RAIL_HEIGHT = 0.040
SHELF_THICKNESS = 0.018

DRAWER_FRONT_THICKNESS = 0.020
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.010
DRAWER_BACK_THICKNESS = 0.012
DRAWER_BOX_DEPTH = 0.320
DRAWER_TRAVEL = 0.20

FIXED_RUNNER_WIDTH = 0.016
FIXED_RUNNER_HEIGHT = 0.012
FIXED_RUNNER_LENGTH = 0.310

OPENING_WIDTH = (CABINET_WIDTH - (2.0 * SIDE_WALL) - CENTER_DIVIDER) / 2.0
DRAWER_FRONT_WIDTH = OPENING_WIDTH - 0.010
DRAWER_BOX_WIDTH = OPENING_WIDTH - 0.028
DRAWER_FACE_HEIGHT = (
    TRAY_FLOOR_BOTTOM - BASE_RAIL_HEIGHT - MID_RAIL_HEIGHT - TOP_RAIL_HEIGHT
) / 2.0 - 0.010
DRAWER_BOX_HEIGHT = 0.240

LOWER_OPENING_BOTTOM = BASE_RAIL_HEIGHT
LOWER_OPENING_TOP = LOWER_OPENING_BOTTOM + DRAWER_FACE_HEIGHT + 0.010
UPPER_OPENING_BOTTOM = LOWER_OPENING_TOP + MID_RAIL_HEIGHT
UPPER_OPENING_TOP = UPPER_OPENING_BOTTOM + DRAWER_FACE_HEIGHT + 0.010

LEFT_DRAWER_X = -((CENTER_DIVIDER / 2.0) + (OPENING_WIDTH / 2.0))
RIGHT_DRAWER_X = -LEFT_DRAWER_X
LOWER_DRAWER_Z = (LOWER_OPENING_BOTTOM + LOWER_OPENING_TOP) / 2.0
UPPER_DRAWER_Z = (UPPER_OPENING_BOTTOM + UPPER_OPENING_TOP) / 2.0

INNER_REAR_Y = (-CABINET_DEPTH / 2.0) + BACK_PANEL
INNER_FRONT_Y = (CABINET_DEPTH / 2.0) - FACE_FRAME_DEPTH
INTERIOR_CENTER_Y = (INNER_FRONT_Y + INNER_REAR_Y) / 2.0
INTERIOR_DEPTH = INNER_FRONT_Y - INNER_REAR_Y
RUNNER_Y = INTERIOR_CENTER_Y - 0.005

DRAWER_LAYOUT = (
    ("lower_left_drawer", "lower_left_slide", LEFT_DRAWER_X, LOWER_DRAWER_Z),
    ("lower_right_drawer", "lower_right_slide", RIGHT_DRAWER_X, LOWER_DRAWER_Z),
    ("upper_left_drawer", "upper_left_slide", LEFT_DRAWER_X, UPPER_DRAWER_Z),
    ("upper_right_drawer", "upper_right_slide", RIGHT_DRAWER_X, UPPER_DRAWER_Z),
)


def _add_box(part, size, xyz, *, material, name):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    joint_name: str,
    x_center: float,
    z_center: float,
) -> None:
    drawer = model.part(name)

    _add_box(
        drawer,
        (DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FACE_HEIGHT),
        (0.0, -DRAWER_FRONT_THICKNESS / 2.0, 0.0),
        material="drawer_face",
        name="drawer_front",
    )
    _add_box(
        drawer,
        (DRAWER_BOX_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_BOX_HEIGHT),
        (
            0.0,
            -DRAWER_FRONT_THICKNESS - DRAWER_BOX_DEPTH + (DRAWER_BACK_THICKNESS / 2.0),
            0.0,
        ),
        material="drawer_box",
        name="drawer_back",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT),
        (
            -(DRAWER_BOX_WIDTH / 2.0) + (DRAWER_SIDE_THICKNESS / 2.0),
            -DRAWER_FRONT_THICKNESS - (DRAWER_BOX_DEPTH / 2.0),
            0.0,
        ),
        material="drawer_box",
        name="drawer_side_left",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT),
        (
            (DRAWER_BOX_WIDTH / 2.0) - (DRAWER_SIDE_THICKNESS / 2.0),
            -DRAWER_FRONT_THICKNESS - (DRAWER_BOX_DEPTH / 2.0),
            0.0,
        ),
        material="drawer_box",
        name="drawer_side_right",
    )
    _add_box(
        drawer,
        (
            DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
            DRAWER_BOX_DEPTH - DRAWER_BACK_THICKNESS,
            DRAWER_BOTTOM_THICKNESS,
        ),
        (
            0.0,
            -DRAWER_FRONT_THICKNESS
            - ((DRAWER_BOX_DEPTH - DRAWER_BACK_THICKNESS) / 2.0),
            -(DRAWER_BOX_HEIGHT / 2.0) + (DRAWER_BOTTOM_THICKNESS / 2.0),
        ),
        material="drawer_box",
        name="drawer_bottom",
    )
    _add_box(
        drawer,
        (0.140, 0.014, 0.018),
        (0.0, 0.007, 0.0),
        material="pull",
        name="drawer_pull",
    )
    _add_box(
        drawer,
        (0.004, DRAWER_BOX_DEPTH - 0.020, 0.028),
        (
            -(DRAWER_BOX_WIDTH / 2.0) + 0.004,
            -DRAWER_FRONT_THICKNESS - ((DRAWER_BOX_DEPTH - 0.020) / 2.0),
            0.0,
        ),
        material="runner",
        name="runner_strip_left",
    )
    _add_box(
        drawer,
        (0.004, DRAWER_BOX_DEPTH - 0.020, 0.028),
        (
            (DRAWER_BOX_WIDTH / 2.0) - 0.004,
            -DRAWER_FRONT_THICKNESS - ((DRAWER_BOX_DEPTH - 0.020) / 2.0),
            0.0,
        ),
        material="runner",
        name="runner_strip_right",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child=drawer,
        origin=Origin(xyz=(x_center, CABINET_DEPTH / 2.0, z_center)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=80.0,
            velocity=0.25,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="craft_storage_cabinet")

    model.material("carcass", rgba=(0.90, 0.88, 0.82, 1.0))
    model.material("drawer_face", rgba=(0.95, 0.93, 0.88, 1.0))
    model.material("drawer_box", rgba=(0.79, 0.71, 0.58, 1.0))
    model.material("runner", rgba=(0.68, 0.71, 0.75, 1.0))
    model.material("pull", rgba=(0.18, 0.19, 0.21, 1.0))

    cabinet = model.part("cabinet")

    _add_box(
        cabinet,
        (SIDE_WALL, CABINET_DEPTH, CABINET_HEIGHT),
        (-(CABINET_WIDTH / 2.0) + (SIDE_WALL / 2.0), 0.0, CABINET_HEIGHT / 2.0),
        material="carcass",
        name="side_left",
    )
    _add_box(
        cabinet,
        (SIDE_WALL, CABINET_DEPTH, CABINET_HEIGHT),
        ((CABINET_WIDTH / 2.0) - (SIDE_WALL / 2.0), 0.0, CABINET_HEIGHT / 2.0),
        material="carcass",
        name="side_right",
    )
    _add_box(
        cabinet,
        (CABINET_WIDTH - (2.0 * SIDE_WALL), CABINET_DEPTH - BACK_PANEL, SIDE_WALL),
        (0.0, BACK_PANEL / 2.0, SIDE_WALL / 2.0),
        material="carcass",
        name="bottom_panel",
    )
    _add_box(
        cabinet,
        (CABINET_WIDTH - (2.0 * SIDE_WALL), BACK_PANEL, CABINET_HEIGHT - SIDE_WALL),
        (0.0, -(CABINET_DEPTH / 2.0) + (BACK_PANEL / 2.0), (CABINET_HEIGHT - SIDE_WALL) / 2.0),
        material="carcass",
        name="back_panel",
    )
    _add_box(
        cabinet,
        (CENTER_DIVIDER, CABINET_DEPTH - BACK_PANEL - FACE_FRAME_DEPTH, TRAY_FLOOR_BOTTOM),
        (0.0, (BACK_PANEL - FACE_FRAME_DEPTH) / 2.0, TRAY_FLOOR_BOTTOM / 2.0),
        material="carcass",
        name="center_divider",
    )
    _add_box(
        cabinet,
        (OPENING_WIDTH, INTERIOR_DEPTH, SHELF_THICKNESS),
        (LEFT_DRAWER_X, INTERIOR_CENTER_Y, LOWER_OPENING_TOP + (MID_RAIL_HEIGHT / 2.0)),
        material="carcass",
        name="shelf_left",
    )
    _add_box(
        cabinet,
        (OPENING_WIDTH, INTERIOR_DEPTH, SHELF_THICKNESS),
        (RIGHT_DRAWER_X, INTERIOR_CENTER_Y, LOWER_OPENING_TOP + (MID_RAIL_HEIGHT / 2.0)),
        material="carcass",
        name="shelf_right",
    )
    _add_box(
        cabinet,
        (CABINET_WIDTH - (2.0 * SIDE_WALL), CABINET_DEPTH - BACK_PANEL - FACE_FRAME_DEPTH, TRAY_FLOOR_THICKNESS),
        (0.0, (BACK_PANEL - FACE_FRAME_DEPTH) / 2.0, TRAY_FLOOR_BOTTOM + (TRAY_FLOOR_THICKNESS / 2.0)),
        material="carcass",
        name="tray_floor",
    )
    _add_box(
        cabinet,
        (CABINET_WIDTH - (2.0 * SIDE_WALL), FACE_FRAME_DEPTH, TRAY_FRONT_WALL),
        (
            0.0,
            (CABINET_DEPTH / 2.0) - (FACE_FRAME_DEPTH / 2.0),
            TRAY_FLOOR_BOTTOM + TRAY_FLOOR_THICKNESS + (TRAY_FRONT_WALL / 2.0),
        ),
        material="carcass",
        name="tray_front_rail",
    )
    _add_box(
        cabinet,
        (TRAY_DIVIDER, CABINET_DEPTH - BACK_PANEL - FACE_FRAME_DEPTH - 0.050, 0.045),
        (-0.115, (BACK_PANEL - FACE_FRAME_DEPTH - 0.050) / 2.0, TRAY_FLOOR_BOTTOM + TRAY_FLOOR_THICKNESS + 0.0225),
        material="carcass",
        name="tray_divider_left",
    )
    _add_box(
        cabinet,
        (TRAY_DIVIDER, CABINET_DEPTH - BACK_PANEL - FACE_FRAME_DEPTH - 0.050, 0.045),
        (0.115, (BACK_PANEL - FACE_FRAME_DEPTH - 0.050) / 2.0, TRAY_FLOOR_BOTTOM + TRAY_FLOOR_THICKNESS + 0.0225),
        material="carcass",
        name="tray_divider_right",
    )

    for name, x_center in (
        ("base_rail_left", LEFT_DRAWER_X),
        ("base_rail_right", RIGHT_DRAWER_X),
    ):
        _add_box(
            cabinet,
            (OPENING_WIDTH, FACE_FRAME_DEPTH, BASE_RAIL_HEIGHT),
            (x_center, (CABINET_DEPTH / 2.0) - (FACE_FRAME_DEPTH / 2.0), BASE_RAIL_HEIGHT / 2.0),
            material="carcass",
            name=name,
        )

    for name, x_center in (
        ("mid_rail_left", LEFT_DRAWER_X),
        ("mid_rail_right", RIGHT_DRAWER_X),
    ):
        _add_box(
            cabinet,
            (OPENING_WIDTH, FACE_FRAME_DEPTH, MID_RAIL_HEIGHT),
            (
                x_center,
                (CABINET_DEPTH / 2.0) - (FACE_FRAME_DEPTH / 2.0),
                LOWER_OPENING_TOP + (MID_RAIL_HEIGHT / 2.0),
            ),
            material="carcass",
            name=name,
        )

    for name, x_center in (
        ("top_rail_left", LEFT_DRAWER_X),
        ("top_rail_right", RIGHT_DRAWER_X),
    ):
        _add_box(
            cabinet,
            (OPENING_WIDTH, FACE_FRAME_DEPTH, TOP_RAIL_HEIGHT),
            (
                x_center,
                (CABINET_DEPTH / 2.0) - (FACE_FRAME_DEPTH / 2.0),
                TRAY_FLOOR_BOTTOM - (TOP_RAIL_HEIGHT / 2.0),
            ),
            material="carcass",
            name=name,
        )

    left_runner_x = (
        -(CABINET_WIDTH / 2.0) + SIDE_WALL + (FIXED_RUNNER_WIDTH / 2.0),
        -(CENTER_DIVIDER / 2.0) - (FIXED_RUNNER_WIDTH / 2.0),
    )
    right_runner_x = (
        (CENTER_DIVIDER / 2.0) + (FIXED_RUNNER_WIDTH / 2.0),
        (CABINET_WIDTH / 2.0) - SIDE_WALL - (FIXED_RUNNER_WIDTH / 2.0),
    )
    for prefix, x_positions, z_center in (
        ("lower_left", left_runner_x, LOWER_DRAWER_Z),
        ("lower_right", right_runner_x, LOWER_DRAWER_Z),
        ("upper_left", left_runner_x, UPPER_DRAWER_Z),
        ("upper_right", right_runner_x, UPPER_DRAWER_Z),
    ):
        _add_box(
            cabinet,
            (FIXED_RUNNER_WIDTH, FIXED_RUNNER_LENGTH, FIXED_RUNNER_HEIGHT),
            (x_positions[0], RUNNER_Y, z_center),
            material="runner",
            name=f"{prefix}_runner_outer",
        )
        _add_box(
            cabinet,
            (FIXED_RUNNER_WIDTH, FIXED_RUNNER_LENGTH, FIXED_RUNNER_HEIGHT),
            (x_positions[1], RUNNER_Y, z_center),
            material="runner",
            name=f"{prefix}_runner_inner",
        )

    lid = model.part("lid")
    _add_box(
        lid,
        (CABINET_WIDTH, CABINET_DEPTH, LID_THICKNESS),
        (0.0, CABINET_DEPTH / 2.0, LID_THICKNESS / 2.0),
        material="carcass",
        name="lid_panel",
    )
    _add_box(
        lid,
        (0.180, 0.026, 0.018),
        (0.0, CABINET_DEPTH - 0.013, LID_THICKNESS + 0.009),
        material="pull",
        name="lid_pull",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, -(CABINET_DEPTH / 2.0), CABINET_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.18,
            effort=10.0,
            velocity=1.4,
        ),
    )

    for drawer_name, joint_name, x_center, z_center in DRAWER_LAYOUT:
        _add_drawer(
            model,
            name=drawer_name,
            joint_name=joint_name,
            x_center=x_center,
            z_center=z_center,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            min_overlap=0.30,
            name="closed lid covers the organizer tray",
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="tray_front_rail",
            min_gap=0.06,
            max_gap=0.10,
            name="closed lid sits just above the organizer tray rail",
        )

    closed_pull_aabb = ctx.part_element_world_aabb(lid, elem="lid_pull")
    with ctx.pose({lid_hinge: 1.18}):
        open_pull_aabb = ctx.part_element_world_aabb(lid, elem="lid_pull")
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_pull",
            negative_elem="tray_front_rail",
            min_gap=0.30,
            name="opened lid clears the tray access zone",
        )
    ctx.check(
        "lid front lifts upward",
        closed_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[1][2] > closed_pull_aabb[1][2] + 0.25,
        details=f"closed={closed_pull_aabb}, open={open_pull_aabb}",
    )
    ctx.check(
        "lid swings rearward",
        closed_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[0][1] < closed_pull_aabb[0][1] - 0.20,
        details=f"closed={closed_pull_aabb}, open={open_pull_aabb}",
    )

    for drawer_name, joint_name, _, _ in DRAWER_LAYOUT:
        drawer = object_model.get_part(drawer_name)
        slide = object_model.get_articulation(joint_name)
        limits = slide.motion_limits
        upper = DRAWER_TRAVEL if limits is None or limits.upper is None else limits.upper
        runner_prefix = drawer_name.replace("_drawer", "")

        rest_pos = ctx.part_world_position(drawer)
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a="runner_strip_left",
            elem_b=f"{runner_prefix}_runner_outer",
            name=f"{drawer_name} left runner stays mounted",
        )
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a="runner_strip_right",
            elem_b=f"{runner_prefix}_runner_inner",
            name=f"{drawer_name} right runner stays mounted",
        )
        with ctx.pose({slide: upper}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_within(
                drawer,
                cabinet,
                axes="xz",
                margin=0.0,
                name=f"{drawer_name} stays centered in its bay",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                elem_a="runner_strip_left",
                elem_b=f"{runner_prefix}_runner_outer",
                min_overlap=0.05,
                name=f"{drawer_name} left runner keeps engagement",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                elem_a="runner_strip_right",
                elem_b=f"{runner_prefix}_runner_inner",
                min_overlap=0.05,
                name=f"{drawer_name} right runner keeps engagement",
            )

        ctx.check(
            f"{drawer_name} extends forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] > rest_pos[1] + 0.18,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()

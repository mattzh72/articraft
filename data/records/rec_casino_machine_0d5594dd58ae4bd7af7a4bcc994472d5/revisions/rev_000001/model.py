from __future__ import annotations

from math import pi

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


CABINET_WIDTH = 0.68
CABINET_DEPTH = 0.72
LOWER_HEIGHT = 0.92
NECK_WIDTH = 0.46
NECK_DEPTH = 0.50
NECK_HEIGHT = 0.18

SCREEN_WIDTH = 0.60
SCREEN_DEPTH = 0.22
SCREEN_HEIGHT = 0.70
SCREEN_TILT = 0.30

SHELF_WIDTH = 0.58
SHELF_DEPTH = 0.24
SHELF_HEIGHT = 0.10

DOOR_WIDTH = 0.32
DOOR_HEIGHT = 0.36
DOOR_THICKNESS = 0.028
DOOR_BOTTOM = 0.14


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="video_poker_machine")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.16, 0.15, 0.17, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.42, 0.10, 0.12, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.63, 0.56, 0.30, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.05, 0.09, 0.13, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.69, 0.71, 0.74, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    button_red = model.material("button_red", rgba=(0.75, 0.10, 0.10, 1.0))
    button_amber = model.material("button_amber", rgba=(0.89, 0.56, 0.10, 1.0))
    button_green = model.material("button_green", rgba=(0.12, 0.58, 0.20, 1.0))
    button_blue = model.material("button_blue", rgba=(0.10, 0.30, 0.72, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT / 2.0)),
        material=cabinet_finish,
        name="lower_shell",
    )
    cabinet.visual(
        Box((0.60, 0.60, 0.08)),
        origin=Origin(xyz=(0.0, 0.00, 0.04)),
        material=accent_finish,
        name="toe_kick",
    )
    cabinet.visual(
        Box((NECK_WIDTH, NECK_DEPTH, NECK_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.03, LOWER_HEIGHT + NECK_HEIGHT / 2.0)),
        material=cabinet_finish,
        name="neck",
    )
    cabinet.visual(
        Box((0.54, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0 - 0.03, 0.74)),
        material=trim_finish,
        name="front_trim",
    )

    screen_housing = model.part("screen_housing")
    screen_housing.visual(
        Box((SCREEN_WIDTH, SCREEN_DEPTH, SCREEN_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.02, 0.367), rpy=(SCREEN_TILT, 0.0, 0.0)),
        material=accent_finish,
        name="housing_shell",
    )
    screen_housing.visual(
        Box((0.32, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, -0.05, 0.025)),
        material=accent_finish,
        name="mount_block",
    )
    screen_housing.visual(
        Box((0.54, 0.028, 0.58)),
        origin=Origin(xyz=(0.0, 0.090, 0.368), rpy=(SCREEN_TILT, 0.0, 0.0)),
        material=trim_finish,
        name="screen_bezel",
    )
    screen_housing.visual(
        Box((0.48, 0.010, 0.50)),
        origin=Origin(xyz=(0.0, 0.100, 0.370), rpy=(SCREEN_TILT, 0.0, 0.0)),
        material=screen_finish,
        name="screen_glass",
    )
    screen_housing.visual(
        Box((0.50, 0.020, 0.11)),
        origin=Origin(xyz=(0.0, 0.088, 0.090), rpy=(SCREEN_TILT, 0.0, 0.0)),
        material=cabinet_finish,
        name="speaker_panel",
    )

    control_shelf = model.part("control_shelf")
    control_shelf.visual(
        Box((SHELF_WIDTH, SHELF_DEPTH, SHELF_HEIGHT)),
        origin=Origin(xyz=(0.0, SHELF_DEPTH / 2.0, SHELF_HEIGHT / 2.0)),
        material=accent_finish,
        name="shelf_body",
    )
    control_shelf.visual(
        Box((0.56, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.11, 0.10)),
        material=cabinet_finish,
        name="deck_top",
    )
    control_shelf.visual(
        Box((0.56, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, SHELF_DEPTH - 0.015, 0.05)),
        material=trim_finish,
        name="deck_fascia",
    )
    for button_x, button_material, button_name in (
        (-0.20, button_red, "button_0"),
        (-0.10, button_amber, "button_1"),
        (0.00, button_green, "button_2"),
        (0.10, button_blue, "button_3"),
        (0.20, button_red, "button_4"),
    ):
        control_shelf.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(button_x, 0.13, 0.117)),
            material=button_material,
            name=button_name,
        )

    cash_box_panel = model.part("cash_box_panel")
    cash_box_panel.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=trim_finish,
        name="door_skin",
    )
    cash_box_panel.visual(
        Box((0.25, 0.010, 0.23)),
        origin=Origin(xyz=(0.17, 0.010, 0.19)),
        material=accent_finish,
        name="door_inset",
    )
    cash_box_panel.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.275, 0.018, 0.20), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="lock",
    )

    handle_bracket = model.part("handle_bracket")
    handle_bracket.visual(
        Box((0.018, 0.16, 0.22)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=steel_finish,
        name="back_plate",
    )
    handle_bracket.visual(
        Box((0.052, 0.028, 0.080)),
        origin=Origin(xyz=(0.044, 0.042, 0.060)),
        material=steel_finish,
        name="front_lug",
    )
    handle_bracket.visual(
        Box((0.052, 0.028, 0.080)),
        origin=Origin(xyz=(0.044, -0.042, 0.060)),
        material=steel_finish,
        name="rear_lug",
    )
    for bolt_y, bolt_z, bolt_name in (
        (0.050, 0.065, "bolt_0"),
        (-0.050, 0.065, "bolt_1"),
        (0.050, -0.065, "bolt_2"),
        (-0.050, -0.065, "bolt_3"),
    ):
        handle_bracket.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.021, bolt_y, bolt_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=trim_finish,
            name=bolt_name,
        )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Cylinder(radius=0.011, length=0.056),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="hub",
    )
    pull_handle.visual(
        Box((0.024, 0.045, 0.26)),
        origin=Origin(xyz=(0.012, 0.0, -0.13)),
        material=steel_finish,
        name="inner_stile",
    )
    pull_handle.visual(
        Box((0.042, 0.045, 0.024)),
        origin=Origin(xyz=(0.033, 0.0, -0.018)),
        material=steel_finish,
        name="top_bar",
    )
    pull_handle.visual(
        Box((0.042, 0.045, 0.024)),
        origin=Origin(xyz=(0.033, 0.0, -0.242)),
        material=steel_finish,
        name="bottom_bar",
    )
    pull_handle.visual(
        Cylinder(radius=0.013, length=0.20),
        origin=Origin(xyz=(0.054, 0.0, -0.13)),
        material=rubber_finish,
        name="grip",
    )

    model.articulation(
        "screen_mount",
        ArticulationType.FIXED,
        parent=cabinet,
        child=screen_housing,
        origin=Origin(xyz=(0.0, -0.05, LOWER_HEIGHT + NECK_HEIGHT)),
    )
    model.articulation(
        "shelf_mount",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_shelf,
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0, 0.82)),
    )
    model.articulation(
        "cash_box_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cash_box_panel,
        origin=Origin(
            xyz=(
                -DOOR_WIDTH / 2.0,
                CABINET_DEPTH / 2.0 + DOOR_THICKNESS / 2.0,
                DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=15.0, velocity=1.5),
    )
    model.articulation(
        "handle_bracket_mount",
        ArticulationType.FIXED,
        parent=cabinet,
        child=handle_bracket,
        origin=Origin(xyz=(CABINET_WIDTH / 2.0, -0.06, 0.78)),
    )
    model.articulation(
        "pull_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=handle_bracket,
        child=pull_handle,
        origin=Origin(xyz=(0.044, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=8.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    screen_housing = object_model.get_part("screen_housing")
    control_shelf = object_model.get_part("control_shelf")
    cash_box_panel = object_model.get_part("cash_box_panel")
    handle_bracket = object_model.get_part("handle_bracket")
    pull_handle = object_model.get_part("pull_handle")

    cash_box_hinge = object_model.get_articulation("cash_box_hinge")
    pull_handle_pivot = object_model.get_articulation("pull_handle_pivot")

    with ctx.pose({cash_box_hinge: 0.0, pull_handle_pivot: 0.0}):
        ctx.expect_gap(
            screen_housing,
            control_shelf,
            axis="z",
            positive_elem="screen_bezel",
            negative_elem="deck_top",
            min_gap=0.18,
            name="control shelf stays clearly below the screen housing",
        )
        ctx.expect_gap(
            cash_box_panel,
            cabinet,
            axis="y",
            positive_elem="door_skin",
            negative_elem="lower_shell",
            min_gap=0.0,
            max_gap=0.003,
            name="cash box panel sits nearly flush on the cabinet front",
        )
        ctx.expect_overlap(
            cash_box_panel,
            cabinet,
            axes="xz",
            elem_a="door_skin",
            elem_b="lower_shell",
            min_overlap=0.25,
            name="cash box panel covers the lower cabinet opening area",
        )
        ctx.expect_contact(
            handle_bracket,
            cabinet,
            elem_a="back_plate",
            elem_b="lower_shell",
            name="handle bracket stays bolted against the cabinet side",
        )
        ctx.expect_gap(
            pull_handle,
            handle_bracket,
            axis="x",
            positive_elem="grip",
            negative_elem="back_plate",
            min_gap=0.045,
            max_gap=0.090,
            name="pull handle rests close to the side bracket",
        )

    with ctx.pose({cash_box_hinge: 1.25}):
        ctx.expect_gap(
            cash_box_panel,
            cabinet,
            axis="y",
            positive_elem="lock",
            negative_elem="lower_shell",
            min_gap=0.18,
            name="cash box lock edge swings out from the cabinet when opened",
        )

    closed_grip_aabb = None
    with ctx.pose({pull_handle_pivot: 0.0}):
        closed_grip_aabb = ctx.part_element_world_aabb(pull_handle, elem="grip")

    with ctx.pose({pull_handle_pivot: 1.20}):
        open_grip_aabb = ctx.part_element_world_aabb(pull_handle, elem="grip")
        closed_max_x = closed_grip_aabb[1][0] if closed_grip_aabb is not None else None
        open_max_x = open_grip_aabb[1][0] if open_grip_aabb is not None else None
        ctx.check(
            "pull handle swings outward from the side bracket",
            closed_max_x is not None and open_max_x is not None and open_max_x > closed_max_x + 0.12,
            details=f"closed_max_x={closed_max_x}, open_max_x={open_max_x}",
        )

    return ctx.report()


object_model = build_object_model()

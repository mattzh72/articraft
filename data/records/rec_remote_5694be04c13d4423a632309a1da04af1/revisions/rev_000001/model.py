from __future__ import annotations

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

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_door_remote")

    body_mat = model.material("body_plastic", rgba=(0.12, 0.12, 0.14, 1.0))
    trim_mat = model.material("trim_plastic", rgba=(0.18, 0.18, 0.21, 1.0))
    cover_mat = model.material("cover_satin", rgba=(0.76, 0.77, 0.80, 1.0))
    button_mat = model.material("button_rubber", rgba=(0.15, 0.15, 0.17, 1.0))

    body_length = 0.068
    body_width = 0.034
    body_thickness = 0.012
    end_radius = body_width / 2.0
    center_length = body_length - body_width

    deck_center_x = -0.006
    deck_size = (0.031, 0.022, 0.0005)
    deck_top_z = body_thickness + deck_size[2]

    rail_length = 0.050
    rail_width = 0.0022
    rail_height = 0.0025
    rail_center_x = 0.004
    rail_center_y = 0.0103
    rail_top_z = body_thickness + rail_height

    body = model.part("body")
    body.visual(
        Box((center_length, body_width, body_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_thickness / 2.0)),
        material=body_mat,
        name="shell_center",
    )
    body.visual(
        Cylinder(radius=end_radius, length=body_thickness),
        origin=Origin(xyz=(-center_length / 2.0, 0.0, body_thickness / 2.0)),
        material=body_mat,
        name="shell_end_0",
    )
    body.visual(
        Cylinder(radius=end_radius, length=body_thickness),
        origin=Origin(xyz=(center_length / 2.0, 0.0, body_thickness / 2.0)),
        material=body_mat,
        name="shell_end_1",
    )
    body.visual(
        Box(deck_size),
        origin=Origin(xyz=(deck_center_x, 0.0, body_thickness + deck_size[2] / 2.0)),
        material=trim_mat,
        name="deck",
    )
    body.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(
            xyz=(rail_center_x, rail_center_y, body_thickness + rail_height / 2.0)
        ),
        material=trim_mat,
        name="rail_0",
    )
    body.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(
            xyz=(rail_center_x, -rail_center_y, body_thickness + rail_height / 2.0)
        ),
        material=trim_mat,
        name="rail_1",
    )

    cover = model.part("cover")
    cover_length = 0.036
    cover_plate_width = 0.023
    cover_plate_thickness = 0.0014
    cover_center_x = -0.005
    cover_plate_center_z = rail_top_z + 0.0003 + cover_plate_thickness / 2.0
    runner_width = 0.0018
    runner_height = 0.0008
    flange_width = 0.0018
    flange_height = 0.0032
    flange_center_y = 0.0124
    thumb_rib_size = (0.008, 0.017, 0.0007)

    cover.visual(
        Box((cover_length, cover_plate_width, cover_plate_thickness)),
        origin=Origin(xyz=(cover_center_x, 0.0, cover_plate_center_z)),
        material=cover_mat,
        name="plate",
    )
    cover.visual(
        Box((cover_length, runner_width, runner_height)),
        origin=Origin(
            xyz=(
                cover_center_x,
                rail_center_y,
                rail_top_z + runner_height / 2.0,
            )
        ),
        material=cover_mat,
        name="runner_0",
    )
    cover.visual(
        Box((cover_length, runner_width, runner_height)),
        origin=Origin(
            xyz=(
                cover_center_x,
                -rail_center_y,
                rail_top_z + runner_height / 2.0,
            )
        ),
        material=cover_mat,
        name="runner_1",
    )
    for index, flange_y in enumerate((flange_center_y, -flange_center_y)):
        cover.visual(
            Box((cover_length, flange_width, flange_height)),
            origin=Origin(
                xyz=(
                    cover_center_x,
                    flange_y,
                    cover_plate_center_z + cover_plate_thickness / 2.0 - flange_height / 2.0,
                )
            ),
            material=cover_mat,
            name=f"flange_{index}",
        )
    cover.visual(
        Box(thumb_rib_size),
        origin=Origin(
            xyz=(
                cover_center_x - 0.012,
                0.0,
                cover_plate_center_z + cover_plate_thickness / 2.0 + thumb_rib_size[2] / 2.0,
            )
        ),
        material=trim_mat,
        name="thumb_rib",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cover,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.031,
        ),
    )

    button_positions = (
        (-0.013, 0.006),
        (-0.013, -0.006),
        (0.001, 0.006),
        (0.001, -0.006),
    )
    button_size = (0.010, 0.0085, 0.0012)
    button_center_z = button_size[2] / 2.0

    for index, (button_x, button_y) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box(button_size),
            origin=Origin(xyz=(0.0, 0.0, button_center_z)),
            material=button_mat,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, button_y, deck_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.02,
                lower=0.0,
                upper=0.0007,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    cover_slide = object_model.get_articulation("body_to_cover")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_3 = object_model.get_part("button_3")
    button_0_joint = object_model.get_articulation("body_to_button_0")

    ctx.expect_contact(
        button_0,
        body,
        elem_a="cap",
        elem_b="deck",
        name="button cap seats on the button deck",
    )
    ctx.expect_overlap(
        cover,
        button_0,
        axes="xy",
        elem_a="plate",
        elem_b="cap",
        min_overlap=0.007,
        name="closed cover shields the lower button",
    )
    ctx.expect_overlap(
        cover,
        button_3,
        axes="xy",
        elem_a="plate",
        elem_b="cap",
        min_overlap=0.007,
        name="closed cover shields the upper button",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="runner_0",
        negative_elem="rail_0",
        min_gap=0.0,
        max_gap=0.0002,
        name="cover runner sits on the side guide",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    pressed_button_0 = None
    pressed_button_1 = None

    with ctx.pose({button_0_joint: 0.0007}):
        pressed_button_0 = ctx.part_world_position(button_0)
        pressed_button_1 = ctx.part_world_position(button_1)

    ctx.check(
        "button depresses independently",
        rest_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_0 is not None
        and pressed_button_1 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.0005
        and abs(pressed_button_1[2] - rest_button_1[2]) < 1e-6,
        details=(
            f"rest_button_0={rest_button_0}, pressed_button_0={pressed_button_0}, "
            f"rest_button_1={rest_button_1}, pressed_button_1={pressed_button_1}"
        ),
    )

    with ctx.pose({cover_slide: 0.031}):
        ctx.expect_gap(
            cover,
            button_3,
            axis="x",
            positive_elem="plate",
            negative_elem="cap",
            min_gap=0.0015,
            name="open cover clears the button face",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="x",
            elem_a="runner_0",
            elem_b="rail_0",
            min_overlap=0.018,
            name="open cover stays captured on the guide rail",
        )

    return ctx.report()


object_model = build_object_model()

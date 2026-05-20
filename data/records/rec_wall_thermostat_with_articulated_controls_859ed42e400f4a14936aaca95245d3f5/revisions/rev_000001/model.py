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

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_thermostat")

    body_width = 0.132
    body_depth = 0.026
    body_height = 0.092
    half_width = body_width / 2.0
    front_y = body_depth / 2.0
    bottom_z = -body_height / 2.0

    button_thickness = 0.0032
    button_depth = 0.014
    button_height = 0.022
    button_travel = 0.0018

    cover_width = 0.116
    cover_thickness = 0.003
    cover_height = 0.032

    body_mat = model.material("body", rgba=(0.94, 0.95, 0.93, 1.0))
    cover_mat = model.material("cover", rgba=(0.88, 0.89, 0.87, 1.0))
    button_mat = model.material("button", rgba=(0.70, 0.73, 0.76, 1.0))
    screen_mat = model.material("screen", rgba=(0.16, 0.19, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, body_height)),
        material=body_mat,
        name="housing_shell",
    )
    body.visual(
        Box((0.082, 0.0018, 0.026)),
        origin=Origin(xyz=(0.0, front_y + 0.0008, 0.014)),
        material=screen_mat,
        name="display_window",
    )

    left_button = model.part("left_button")
    left_button.visual(
        Box((button_thickness, button_depth, button_height)),
        material=button_mat,
        name="button_cap",
    )

    right_button = model.part("right_button")
    right_button.visual(
        Box((button_thickness, button_depth, button_height)),
        material=button_mat,
        name="button_cap",
    )

    battery_cover = model.part("battery_cover")
    battery_cover.visual(
        Box((cover_width, cover_thickness, cover_height)),
        origin=Origin(xyz=(0.0, 0.0, cover_height / 2.0)),
        material=cover_mat,
        name="cover_panel",
    )
    battery_cover.visual(
        Box((0.054, 0.0045, 0.004)),
        origin=Origin(xyz=(0.0, 0.0007, 0.002)),
        material=cover_mat,
        name="pull_lip",
    )

    model.articulation(
        "left_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_button,
        origin=Origin(xyz=(-half_width - button_thickness / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.03,
            lower=0.0,
            upper=button_travel,
        ),
    )
    model.articulation(
        "right_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_button,
        origin=Origin(xyz=(half_width + button_thickness / 2.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.03,
            lower=0.0,
            upper=button_travel,
        ),
    )
    model.articulation(
        "battery_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_cover,
        origin=Origin(xyz=(0.0, front_y + cover_thickness / 2.0, bottom_z + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    battery_cover = object_model.get_part("battery_cover")

    left_button_slide = object_model.get_articulation("left_button_slide")
    right_button_slide = object_model.get_articulation("right_button_slide")
    battery_cover_hinge = object_model.get_articulation("battery_cover_hinge")

    left_upper = left_button_slide.motion_limits.upper or 0.0
    right_upper = right_button_slide.motion_limits.upper or 0.0
    cover_upper = battery_cover_hinge.motion_limits.upper or 0.0

    ctx.expect_gap(
        body,
        left_button,
        axis="x",
        positive_elem="housing_shell",
        negative_elem="button_cap",
        max_gap=0.0003,
        max_penetration=0.0,
        name="left button sits flush on the housing side",
    )
    ctx.expect_gap(
        right_button,
        body,
        axis="x",
        positive_elem="button_cap",
        negative_elem="housing_shell",
        max_gap=0.0003,
        max_penetration=0.0,
        name="right button sits flush on the housing side",
    )
    ctx.expect_overlap(
        left_button,
        body,
        axes="yz",
        elem_a="button_cap",
        elem_b="housing_shell",
        min_overlap=0.014,
        name="left button aligns with the housing sidewall",
    )
    ctx.expect_overlap(
        right_button,
        body,
        axes="yz",
        elem_a="button_cap",
        elem_b="housing_shell",
        min_overlap=0.014,
        name="right button aligns with the housing sidewall",
    )
    ctx.expect_gap(
        battery_cover,
        body,
        axis="y",
        positive_elem="cover_panel",
        negative_elem="housing_shell",
        max_gap=0.0003,
        max_penetration=0.0,
        name="battery cover closes flush to the front face",
    )
    ctx.expect_overlap(
        battery_cover,
        body,
        axes="x",
        elem_a="cover_panel",
        elem_b="housing_shell",
        min_overlap=0.110,
        name="battery cover spans most of the thermostat width",
    )

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    with ctx.pose({left_button_slide: left_upper}):
        left_pressed = ctx.part_world_position(left_button)
        right_during_left_press = ctx.part_world_position(right_button)
    ctx.check(
        "left button depresses inward",
        left_rest is not None
        and left_pressed is not None
        and left_pressed[0] > left_rest[0] + 0.001,
        details=f"rest={left_rest}, pressed={left_pressed}",
    )
    ctx.check(
        "left button press leaves right button at rest",
        right_rest is not None
        and right_during_left_press is not None
        and abs(right_during_left_press[0] - right_rest[0]) < 1e-6,
        details=f"rest={right_rest}, during_left_press={right_during_left_press}",
    )

    with ctx.pose({right_button_slide: right_upper}):
        right_pressed = ctx.part_world_position(right_button)
        left_during_right_press = ctx.part_world_position(left_button)
    ctx.check(
        "right button depresses inward",
        right_rest is not None
        and right_pressed is not None
        and right_pressed[0] < right_rest[0] - 0.001,
        details=f"rest={right_rest}, pressed={right_pressed}",
    )
    ctx.check(
        "right button press leaves left button at rest",
        left_rest is not None
        and left_during_right_press is not None
        and abs(left_during_right_press[0] - left_rest[0]) < 1e-6,
        details=f"rest={left_rest}, during_right_press={left_during_right_press}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(battery_cover, elem="cover_panel")
    with ctx.pose({battery_cover_hinge: cover_upper}):
        open_cover_aabb = ctx.part_element_world_aabb(battery_cover, elem="cover_panel")
    ctx.check(
        "battery cover swings downward and outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.020
        and open_cover_aabb[1][2] < closed_cover_aabb[1][2] - 0.010,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.095
BODY_D = 0.046
BODY_H = 0.186

HOUSING_W = 0.084
HOUSING_D = 0.041
HOUSING_H = 0.176

FRONT_Y = HOUSING_D * 0.5
BACK_Y = -HOUSING_D * 0.5

SCREEN_Z = 0.052
BUTTON_Z = 0.020
KNOB_Z = -0.026
JACK_Z = -0.072

BUTTON_W = 0.014
BUTTON_H = 0.010
BUTTON_TRAVEL = 0.0008

SUPPORT_W = 0.074
SUPPORT_T = 0.004
SUPPORT_H = 0.124


def _housing_shape():
    return (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.010)
        .val()
    )


def _button_shape():
    return (
        cq.Workplane("XY")
        .box(BUTTON_W, 0.0010, BUTTON_H, centered=(True, True, True))
        .edges("|Y")
        .fillet(0.0007)
        .val()
    )


def _rear_support_shape():
    outer = (
        cq.Workplane("XY")
        .box(SUPPORT_W, SUPPORT_T, SUPPORT_H, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0025))
    )
    inner = (
        cq.Workplane("XY")
        .box(SUPPORT_W - 0.020, SUPPORT_T + 0.002, SUPPORT_H - 0.038, centered=(True, True, False))
        .translate((0.0, 0.0, 0.018))
    )
    top_pad = (
        cq.Workplane("XY")
        .box(0.030, SUPPORT_T + 0.002, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, SUPPORT_H - 0.002))
    )
    return outer.cut(inner).union(top_pad).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_multimeter")

    housing_dark = model.material("housing_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    overmold_yellow = model.material("overmold_yellow", rgba=(0.92, 0.72, 0.16, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_green = model.material("glass_green", rgba=(0.38, 0.59, 0.52, 0.58))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    button_gray = model.material("button_gray", rgba=(0.31, 0.33, 0.35, 1.0))
    support_dark = model.material("support_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    jack_dark = model.material("jack_dark", rgba=(0.06, 0.06, 0.07, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_housing_shape(), "multimeter_housing"), material=housing_dark, name="housing")

    body.visual(
        Box((0.012, BODY_D, 0.146)),
        origin=Origin(xyz=(-0.037, 0.0, 0.000)),
        material=overmold_yellow,
        name="left_overmold",
    )
    body.visual(
        Box((0.012, BODY_D, 0.146)),
        origin=Origin(xyz=(0.037, 0.0, 0.000)),
        material=overmold_yellow,
        name="right_overmold",
    )
    body.visual(
        Box((0.071, BODY_D, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=overmold_yellow,
        name="top_overmold",
    )
    body.visual(
        Box((0.071, BODY_D, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=overmold_yellow,
        name="bottom_overmold",
    )
    for x_pos in (-0.036, 0.036):
        for z_pos in (-0.078, 0.078):
            body.visual(
                Box((0.018, BODY_D, 0.028)),
                origin=Origin(xyz=(x_pos, 0.0, z_pos)),
                material=overmold_yellow,
                name=f"corner_overmold_{'p' if x_pos > 0.0 else 'n'}x_{'p' if z_pos > 0.0 else 'n'}z",
            )

    body.visual(
        Box((0.066, 0.0020, 0.040)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0005, SCREEN_Z)),
        material=panel_black,
        name="screen_bezel",
    )
    body.visual(
        Box((0.056, 0.0012, 0.030)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0011, SCREEN_Z)),
        material=glass_green,
        name="screen_glass",
    )

    for index, x_pos in enumerate((-0.022, 0.0, 0.022)):
        body.visual(
            Cylinder(radius=0.0060, length=0.0030),
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.0015, JACK_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=jack_dark,
            name=f"jack_{index}",
        )

    for index, x_pos in enumerate((-0.028, 0.028)):
        body.visual(
            Box((0.010, 0.0030, 0.010)),
            origin=Origin(xyz=(x_pos, BACK_Y - 0.0002, -0.078)),
            material=housing_dark,
            name=f"pivot_pad_{index}",
        )

    body.inertial = None

    selector_knob = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.017,
            body_style="skirted",
            top_diameter=0.028,
            base_diameter=0.036,
            edge_radius=0.0010,
            grip=KnobGrip(style="knurled", count=40, depth=0.0010, helix_angle_deg=22.0),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "selector_knob",
    )
    selector_knob.visual(
        knob_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_cap",
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support_frame"),
        material=support_dark,
        name="support_frame",
    )
    for index, x_pos in enumerate((-0.028, 0.028)):
        rear_support.visual(
            Cylinder(radius=0.0030, length=0.010),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=support_dark,
            name=f"pivot_barrel_{index}",
        )

    button_mesh = mesh_from_cadquery(_button_shape(), "multimeter_button")
    button_specs = [
        ("hold_button", -0.027),
        ("range_button", -0.009),
        ("minmax_button", 0.009),
        ("backlight_button", 0.027),
    ]

    for part_name, x_pos in button_specs:
        button = model.part(part_name)
        button.visual(button_mesh, material=button_gray, name="button_cap")

        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.0005, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, FRONT_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=8.0,
        ),
    )

    model.articulation(
        "body_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_support,
        origin=Origin(xyz=(0.0, -0.0260, -0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rear_support = object_model.get_part("rear_support")
    selector_knob = object_model.get_part("selector_knob")
    hold_button = object_model.get_part("hold_button")
    range_button = object_model.get_part("range_button")
    minmax_button = object_model.get_part("minmax_button")
    backlight_button = object_model.get_part("backlight_button")

    support_joint = object_model.get_articulation("body_to_rear_support")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    hold_joint = object_model.get_articulation("body_to_hold_button")

    ctx.expect_gap(
        range_button,
        hold_button,
        axis="x",
        min_gap=0.003,
        positive_elem="button_cap",
        negative_elem="button_cap",
        name="hold and range buttons stay visually separate",
    )
    ctx.expect_gap(
        minmax_button,
        range_button,
        axis="x",
        min_gap=0.003,
        positive_elem="button_cap",
        negative_elem="button_cap",
        name="range and minmax buttons stay visually separate",
    )
    ctx.expect_gap(
        backlight_button,
        minmax_button,
        axis="x",
        min_gap=0.003,
        positive_elem="button_cap",
        negative_elem="button_cap",
        name="minmax and backlight buttons stay visually separate",
    )

    ctx.expect_overlap(
        rear_support,
        body,
        axes="x",
        min_overlap=0.070,
        elem_a="support_frame",
        elem_b="housing",
        name="rear support spans most of the back width",
    )
    ctx.expect_gap(
        body,
        rear_support,
        axis="y",
        min_gap=0.002,
        max_gap=0.006,
        positive_elem="housing",
        negative_elem="support_frame",
        name="rear support folds close to the back",
    )

    hold_rest = ctx.part_world_position(hold_button)
    range_rest = ctx.part_world_position(range_button)
    with ctx.pose({hold_joint: BUTTON_TRAVEL}):
        hold_pressed = ctx.part_world_position(hold_button)
        range_after = ctx.part_world_position(range_button)

    ctx.check(
        "hold button presses inward independently",
        hold_rest is not None
        and hold_pressed is not None
        and range_rest is not None
        and range_after is not None
        and hold_pressed[1] < hold_rest[1] - 0.0006
        and abs(range_after[1] - range_rest[1]) < 1.0e-6,
        details=(
            f"hold_rest={hold_rest}, hold_pressed={hold_pressed}, "
            f"range_rest={range_rest}, range_after={range_after}"
        ),
    )

    knob_rest = ctx.part_world_position(selector_knob)
    with ctx.pose({knob_joint: 1.2}):
        knob_turned = ctx.part_world_position(selector_knob)

    ctx.check(
        "selector knob spins in place",
        knob_rest is not None
        and knob_turned is not None
        and max(abs(knob_turned[i] - knob_rest[i]) for i in range(3)) < 1.0e-6,
        details=f"knob_rest={knob_rest}, knob_turned={knob_turned}",
    )

    support_closed = ctx.part_world_aabb(rear_support)
    with ctx.pose({support_joint: 0.95}):
        support_open = ctx.part_world_aabb(rear_support)

    ctx.check(
        "rear support swings backward when opened",
        support_closed is not None
        and support_open is not None
        and support_open[0][1] < support_closed[0][1] - 0.040,
        details=f"closed={support_closed}, open={support_open}",
    )

    return ctx.report()


object_model = build_object_model()

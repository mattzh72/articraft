from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.086
BODY_D = 0.036
BODY_H = 0.170
FRONT_Y = BODY_D / 2.0
BACK_Y = -BODY_D / 2.0
DIAL_Z = 0.061
KICKSTAND_UPPER = 0.46
BUTTON_TRAVEL = 0.0032


def _rounded_case(width: float, depth: float, height: float) -> cq.Workplane:
    """Boxy handheld case with softened vertical edges."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, height / 2.0))
    )


def _rounded_button(width: float, height: float, thickness: float) -> cq.Workplane:
    """Low rubber pushbutton, local +Z is the press-face normal."""
    return (
        cq.Workplane("XY")
        .box(width, height, thickness)
        .edges("|Z")
        .fillet(0.0028)
        .translate((0.0, 0.0, thickness / 2.0))
    )


def _kickstand_frame() -> cq.Workplane:
    """A single U-shaped rear prop, local X is hinge width, local Y is back, local Z is length."""
    width = 0.057
    thickness = 0.004
    length = 0.104
    z0 = 0.002
    outer = (
        cq.Workplane("XY")
        .box(width, thickness, length)
        .translate((0.0, -thickness / 2.0, z0 + length / 2.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(0.037, thickness * 3.0, 0.071)
        .translate((0.0, -thickness / 2.0, z0 + 0.057))
    )
    return outer.cut(opening).edges("|Y").fillet(0.0018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_autoranging_multimeter")

    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.017, 0.016, 1.0))
    dark = model.material("dark_gray_plastic", rgba=(0.075, 0.080, 0.083, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.018, 0.020, 0.022, 1.0))
    lcd = model.material("green_gray_lcd", rgba=(0.58, 0.66, 0.54, 1.0))
    white = model.material("white_print", rgba=(0.92, 0.92, 0.86, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.70, 0.06, 1.0))
    blue = model.material("backlight_blue", rgba=(0.08, 0.26, 0.52, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_case(BODY_W, BODY_D, BODY_H), "main_case"),
        material=dark,
        name="main_case",
    )

    # Rugged overmold pads are slightly proud but embedded into the case so the
    # static body remains one supported assembly.
    for sx in (-1.0, 1.0):
        for sz, name_z in ((0.020, "lower"), (0.150, "upper")):
            body.visual(
                Box((0.018, 0.040, 0.030)),
                origin=Origin(xyz=(sx * 0.036, 0.0, sz)),
                material=rubber,
                name=f"{name_z}_corner_{'pos' if sx > 0 else 'neg'}",
            )
    body.visual(
        Box((0.080, 0.0030, 0.134)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0009, 0.087)),
        material=charcoal,
        name="front_panel",
    )
    body.visual(
        Box((0.064, 0.0020, 0.026)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0010, 0.095)),
        material=rubber,
        name="button_well",
    )

    display_bezel = BezelGeometry(
        (0.052, 0.022),
        (0.071, 0.041),
        0.005,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.003,
        outer_corner_radius=0.006,
        center=False,
    )
    body.visual(
        mesh_from_geometry(display_bezel, "display_bezel"),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0012, 0.132), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="display_bezel",
    )
    body.visual(
        Box((0.050, 0.0010, 0.020)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0015, 0.132)),
        material=lcd,
        name="lcd_glass",
    )

    # Simple seven-segment-like LCD bars behind the broad bezel.
    seg_w = 0.010
    for i, x in enumerate((-0.017, 0.0, 0.017)):
        body.visual(
            Box((seg_w, 0.0006, 0.0014)),
            origin=Origin(xyz=(x, FRONT_Y + 0.0021, 0.138)),
            material=charcoal,
            name=f"lcd_top_{i}",
        )
        body.visual(
            Box((seg_w, 0.0006, 0.0014)),
            origin=Origin(xyz=(x, FRONT_Y + 0.0021, 0.132)),
            material=charcoal,
            name=f"lcd_mid_{i}",
        )
        body.visual(
            Box((seg_w, 0.0006, 0.0014)),
            origin=Origin(xyz=(x, FRONT_Y + 0.0021, 0.126)),
            material=charcoal,
            name=f"lcd_bot_{i}",
        )
        body.visual(
            Box((0.0015, 0.0006, 0.0068)),
            origin=Origin(xyz=(x - seg_w / 2.0, FRONT_Y + 0.0021, 0.135)),
            material=charcoal,
            name=f"lcd_left_{i}",
        )
        body.visual(
            Box((0.0015, 0.0006, 0.0068)),
            origin=Origin(xyz=(x + seg_w / 2.0, FRONT_Y + 0.0021, 0.129)),
            material=charcoal,
            name=f"lcd_right_{i}",
        )

    # Printed autoranging tick marks around the dial, with one yellow voltage mark.
    for i, angle in enumerate((-140, -105, -70, -35, 0, 35, 70, 105, 140)):
        theta = math.radians(angle)
        radius = 0.033
        x = radius * math.sin(theta)
        z = DIAL_Z + radius * math.cos(theta)
        body.visual(
            Box((0.0020, 0.0009, 0.0100)),
            origin=Origin(xyz=(x, FRONT_Y + 0.0017, z), rpy=(0.0, theta, 0.0)),
            material=yellow if i == 7 else white,
            name=f"range_tick_{i}",
        )
    body.visual(
        Box((0.036, 0.0010, 0.006)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0016, DIAL_Z - 0.036)),
        material=white,
        name="input_label",
    )

    # Rear hinge sockets, hinge pin, and a shallow angle-stop ledge on the back.
    hinge_y = BACK_Y - 0.004
    hinge_z = 0.034
    for x, name in ((-0.026, "hinge_socket_0"), (0.026, "hinge_socket_1")):
        body.visual(
            Cylinder(radius=0.0040, length=0.014),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=name,
        )
    body.visual(
        Cylinder(radius=0.0016, length=0.070),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="hinge_pin",
    )
    for x, name in ((-0.024, "angle_stop_0"), (0.024, "angle_stop_1")):
        body.visual(
            Box((0.020, 0.004, 0.016)),
            origin=Origin(xyz=(x, BACK_Y - 0.004, 0.052), rpy=(0.42, 0.0, 0.0)),
            material=rubber,
            name=name,
        )

    dial = model.part("range_dial")
    range_knob = KnobGeometry(
        0.044,
        0.017,
        body_style="skirted",
        top_diameter=0.033,
        skirt=KnobSkirt(0.049, 0.004, flare=0.06, chamfer=0.0010),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(range_knob, "range_dial_cap"),
        material=rubber,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0040, 0.025, 0.0012)),
        origin=Origin(xyz=(0.0, 0.010, 0.0182)),
        material=white,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_range_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0022, DIAL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=6.0),
    )

    button_specs = (
        ("hold_button", "body_to_hold_button", -0.024, white),
        ("minmax_button", "body_to_minmax_button", 0.0, yellow),
        ("backlight_button", "body_to_backlight_button", 0.024, blue),
    )
    for part_name, joint_name, x, mat in button_specs:
        button = model.part(part_name)
        button.visual(
            mesh_from_cadquery(_rounded_button(0.019, 0.009, 0.0048), f"{part_name}_cap"),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.0015, 0.0008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0052)),
            material=charcoal,
            name="button_mark",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y + 0.0022, 0.098), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        mesh_from_cadquery(_kickstand_frame(), "kickstand_frame"),
        material=rubber,
        name="kickstand_frame",
    )
    kickstand.visual(
        Cylinder(radius=0.0034, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="kickstand_barrel",
    )
    kickstand.visual(
        Box((0.050, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.104)),
        material=rubber,
        name="stand_foot",
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=KICKSTAND_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("range_dial")
    kickstand = object_model.get_part("kickstand")
    dial_joint = object_model.get_articulation("body_to_range_dial")
    stand_joint = object_model.get_articulation("body_to_kickstand")

    ctx.allow_overlap(
        body,
        kickstand,
        elem_a="hinge_pin",
        elem_b="kickstand_barrel",
        reason="The rear kickstand barrel intentionally rotates around the captured hinge pin.",
    )
    ctx.expect_overlap(
        body,
        kickstand,
        axes="x",
        elem_a="hinge_pin",
        elem_b="kickstand_barrel",
        min_overlap=0.025,
        name="kickstand barrel is captured on pin",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        min_gap=-0.0005,
        max_gap=0.004,
        positive_elem="dial_cap",
        negative_elem="front_panel",
        name="range dial sits proud of front panel",
    )

    aabb = ctx.part_world_aabb(body)
    if aabb is not None:
        lo, hi = aabb
        ctx.check(
            "compact one-hand body size",
            (hi[0] - lo[0]) < 0.10 and (hi[2] - lo[2]) < 0.19 and (hi[1] - lo[1]) < 0.055,
            details=f"body_aabb={aabb}",
        )

    ctx.check(
        "range dial is continuous",
        getattr(dial_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(dial_joint, 'articulation_type', None)}",
    )

    for button_name, joint_name in (
        ("hold_button", "body_to_hold_button"),
        ("minmax_button", "body_to_minmax_button"),
        ("backlight_button", "body_to_backlight_button"),
    ):
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(joint_name)
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"{button_name} pushes inward independently",
            rest is not None and pressed is not None and pressed[1] < rest[1] - 0.002,
            details=f"rest={rest}, pressed={pressed}",
        )

    closed = ctx.part_world_aabb(kickstand)
    with ctx.pose({stand_joint: KICKSTAND_UPPER}):
        opened = ctx.part_world_aabb(kickstand)
    ctx.check(
        "kickstand opens rearward to shallow stop",
        closed is not None and opened is not None and opened[0][1] < closed[0][1] - 0.035,
        details=f"closed={closed}, opened={opened}",
    )

    return ctx.report()


object_model = build_object_model()

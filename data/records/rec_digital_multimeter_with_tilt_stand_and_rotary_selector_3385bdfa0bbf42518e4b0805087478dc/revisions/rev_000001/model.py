from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.094
BODY_D = 0.050
BODY_H = 0.188
FRONT_Y = BODY_D * 0.5
BACK_Y = -BODY_D * 0.5

SCREEN_Z = 0.056
BUTTON_Z = 0.024
DIAL_Z = -0.012
JACK_Z = -0.070

BUTTON_ORDER = (
    ("hold_button", -0.027),
    ("range_button", -0.009),
    ("min_max_button", 0.009),
    ("backlight_button", 0.027),
)
JACK_XS = (-0.020, 0.000, 0.020)


def _make_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    shell = shell.edges("|Z").fillet(0.010)

    shell = shell.faces(">Y").workplane().rect(0.078, 0.166).cutBlind(0.0016)
    shell = shell.faces(">Y").workplane().center(0.0, SCREEN_Z).rect(0.064, 0.038).cutBlind(0.0045)
    shell = shell.faces(">Y").workplane().center(0.0, BUTTON_Z).rect(0.071, 0.018).cutBlind(0.0018)

    for _, x_center in BUTTON_ORDER:
        shell = (
            shell.faces(">Y")
            .workplane()
            .center(x_center, BUTTON_Z)
            .rect(0.0165, 0.0108)
            .cutBlind(0.0145)
        )

    shell = shell.faces(">Y").workplane().center(0.0, DIAL_Z).circle(0.030).cutBlind(0.0135)
    shell = shell.faces(">Y").workplane().center(0.0, DIAL_Z).circle(0.0135).cutBlind(0.0165)

    shell = shell.faces(">Y").workplane().center(0.0, JACK_Z).rect(0.072, 0.028).cutBlind(0.014)
    for jack_x in JACK_XS:
        shell = shell.faces(">Y").workplane().center(jack_x, JACK_Z).circle(0.0058).cutBlind(0.018)

    shell = shell.faces("<Y").workplane().center(0.0, -0.028).rect(0.056, 0.118).cutBlind(0.0026)
    return shell


def _make_dial_shape() -> cq.Workplane:
    rear_hub = cq.Workplane("XZ", origin=(0.0, -0.008, 0.0)).circle(0.020).extrude(0.008)
    front_knob = cq.Workplane("XZ", origin=(0.0, -0.0005, 0.0)).circle(0.027).extrude(0.010)
    front_knob = front_knob.faces(">Y").workplane().circle(0.0085).cutBlind(0.0015)
    return rear_hub.union(front_knob)


def _add_front_button(model: ArticulatedObject, button_name: str, x_center: float, material) -> None:
    button = model.part(button_name)
    button.visual(
        Box((0.0145, 0.0032, 0.0085)),
        origin=Origin(),
        material=material,
        name="cap",
    )
    button.visual(
        Box((0.0095, 0.0090, 0.0055)),
        origin=Origin(xyz=(0.0, -0.0059, 0.0)),
        material=material,
        name="plunger",
    )
    model.articulation(
        f"body_to_{button_name}",
        ArticulationType.PRISMATIC,
        parent="body",
        child=button,
        origin=Origin(xyz=(x_center, 0.0236, BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.10,
            lower=0.0,
            upper=0.0025,
        ),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_digital_multimeter")

    shell_dark = model.material("shell_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    boot_yellow = model.material("boot_yellow", rgba=(0.88, 0.74, 0.12, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    button_gray = model.material("button_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.18, 0.30, 0.34, 0.55))
    jack_black = model.material("jack_black", rgba=(0.07, 0.07, 0.08, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    pointer_orange = model.material("pointer_orange", rgba=(0.95, 0.54, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "multimeter_body_shell"),
        material=shell_dark,
        name="shell",
    )
    body.visual(
        Box((0.012, 0.048, 0.160)),
        origin=Origin(xyz=(-0.0415, 0.0, 0.0)),
        material=boot_yellow,
        name="side_bumper_0",
    )
    body.visual(
        Box((0.012, 0.048, 0.160)),
        origin=Origin(xyz=(0.0415, 0.0, 0.0)),
        material=boot_yellow,
        name="side_bumper_1",
    )
    body.visual(
        Box((0.084, 0.046, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=boot_yellow,
        name="top_bumper",
    )
    body.visual(
        Box((0.086, 0.048, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=boot_yellow,
        name="bottom_bumper",
    )

    display = model.part("display")
    display.visual(
        Box((0.064, 0.0040, 0.038)),
        origin=Origin(),
        material=lens_tint,
        name="window",
    )
    display.visual(
        Box((0.068, 0.0010, 0.042)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=shell_dark,
        name="bezel",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, 0.0220, SCREEN_Z)),
    )

    jacks = model.part("jacks")
    jacks.visual(
        Box((0.060, 0.008, 0.015)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=jack_black,
        name="manifold",
    )
    for index, jack_x in enumerate(JACK_XS):
        jacks.visual(
            Cylinder(radius=0.0034, length=0.0060),
            origin=Origin(
                xyz=(jack_x, -0.0010, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=jack_black,
            name=f"socket_{index}",
        )
        jacks.visual(
            Cylinder(radius=0.0060, length=0.0040),
            origin=Origin(
                xyz=(jack_x, 0.0040, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=jack_black,
            name=f"collar_{index}",
        )
    model.articulation(
        "body_to_jacks",
        ArticulationType.FIXED,
        parent=body,
        child=jacks,
        origin=Origin(xyz=(0.0, 0.0190, JACK_Z)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_make_dial_shape(), "multimeter_dial"),
        material=dial_dark,
        name="knob",
    )
    dial.visual(
        Cylinder(radius=0.0340, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dial_dark,
        name="rim",
    )
    dial.visual(
        Cylinder(radius=0.0060, length=0.0120),
        origin=Origin(xyz=(0.0, -0.0060, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dial_dark,
        name="shaft",
    )
    dial.visual(
        Box((0.006, 0.0015, 0.012)),
        origin=Origin(xyz=(0.0, -0.0010, 0.018)),
        material=pointer_orange,
        name="pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0238, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=5.0,
        ),
    )

    for button_name, x_center in BUTTON_ORDER:
        _add_front_button(model, button_name, x_center, button_gray)

    kickstand = model.part("kickstand")
    kickstand.visual(
        Box((0.052, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.0030, 0.005)),
        material=stand_dark,
        name="hinge_bar",
    )
    kickstand.visual(
        Box((0.048, 0.005, 0.112)),
        origin=Origin(xyz=(0.0, -0.0035, 0.056)),
        material=stand_dark,
        name="leg",
    )
    kickstand.visual(
        Box((0.030, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.0035, 0.104)),
        material=stand_dark,
        name="pad",
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, -0.0250, -0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.allow_overlap(
        "body",
        "display",
        elem_a="shell",
        elem_b="bezel",
        reason="The display module is intentionally seated into the recessed front bezel.",
    )
    ctx.allow_overlap(
        "body",
        "dial",
        elem_a="shell",
        elem_b="rim",
        reason="The selector knob sits tightly into the front-face dial opening.",
    )

    display = object_model.get_part("display")
    jacks = object_model.get_part("jacks")
    dial = object_model.get_part("dial")
    kickstand = object_model.get_part("kickstand")

    hold_button = object_model.get_part("hold_button")
    range_button = object_model.get_part("range_button")
    min_max_button = object_model.get_part("min_max_button")
    backlight_button = object_model.get_part("backlight_button")

    button_parts = {
        "hold_button": hold_button,
        "range_button": range_button,
        "min_max_button": min_max_button,
        "backlight_button": backlight_button,
    }
    button_joints = {
        name: object_model.get_articulation(f"body_to_{name}")
        for name in button_parts
    }

    ctx.expect_origin_gap(display, hold_button, axis="z", min_gap=0.024, name="screen sits above the button row")
    ctx.expect_origin_gap(hold_button, jacks, axis="z", min_gap=0.080, name="button row is clearly separated from the jack field")
    ctx.expect_origin_gap(dial, jacks, axis="z", min_gap=0.045, name="dial stays above the jack field")

    ctx.expect_origin_gap(range_button, hold_button, axis="x", min_gap=0.016, max_gap=0.020, name="hold and range buttons are separate controls")
    ctx.expect_origin_gap(min_max_button, range_button, axis="x", min_gap=0.016, max_gap=0.020, name="range and min max buttons are separate controls")
    ctx.expect_origin_gap(backlight_button, min_max_button, axis="x", min_gap=0.016, max_gap=0.020, name="min max and backlight buttons are separate controls")

    rest_positions = {
        name: ctx.part_world_position(part)
        for name, part in button_parts.items()
    }
    for name, part in button_parts.items():
        joint = button_joints[name]
        limits = joint.motion_limits
        upper = limits.upper if limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            pressed_positions = {
                other_name: ctx.part_world_position(other_part)
                for other_name, other_part in button_parts.items()
            }

        moved = (
            rest_positions[name] is not None
            and pressed_positions[name] is not None
            and pressed_positions[name][1] < rest_positions[name][1] - 0.0018
        )
        neighbors_still = True
        for other_name in button_parts:
            if other_name == name:
                continue
            rest_pos = rest_positions[other_name]
            pressed_pos = pressed_positions[other_name]
            if rest_pos is None or pressed_pos is None:
                neighbors_still = False
                break
            if abs(pressed_pos[1] - rest_pos[1]) > 1e-6:
                neighbors_still = False
                break

        ctx.check(
            f"{name} presses independently",
            moved and neighbors_still,
            details=f"rest={rest_positions}, pressed={pressed_positions}",
        )

    dial_joint = object_model.get_articulation("body_to_dial")
    rest_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer"))
    with ctx.pose({dial_joint: 1.2}):
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer"))
    ctx.check(
        "dial rotates on the front face",
        rest_pointer is not None
        and turned_pointer is not None
        and abs(turned_pointer[0] - rest_pointer[0]) > 0.010
        and abs(turned_pointer[2] - rest_pointer[2]) > 0.004,
        details=f"rest_pointer={rest_pointer}, turned_pointer={turned_pointer}",
    )

    kickstand_joint = object_model.get_articulation("body_to_kickstand")
    kickstand_limits = kickstand_joint.motion_limits
    closed_leg = ctx.part_element_world_aabb(kickstand, elem="leg")
    with ctx.pose({kickstand_joint: kickstand_limits.upper if kickstand_limits is not None else 0.0}):
        open_leg = ctx.part_element_world_aabb(kickstand, elem="leg")
    ctx.check(
        "kickstand swings out from the back",
        closed_leg is not None
        and open_leg is not None
        and open_leg[0][1] < closed_leg[0][1] - 0.035,
        details=f"closed_leg={closed_leg}, open_leg={open_leg}",
    )

    return ctx.report()


object_model = build_object_model()

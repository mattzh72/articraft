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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.096
BODY_D = 0.048
BODY_H = 0.186
CASE_W = 0.082
CASE_D = 0.036
CASE_H = 0.170
FRONT_FACE_Y = 0.024
PANEL_FACE_Y = 0.0215
REAR_FACE_Y = -0.024


def _build_boot_shape() -> object:
    boot = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    boot = boot.edges("|Z").fillet(0.010)
    boot = boot.faces(">Y").workplane().rect(0.078, 0.154).cutBlind(0.010)
    boot = boot.faces("<Y").workplane().rect(0.066, 0.132).cutBlind(0.008)

    side_guard = cq.Workplane("XY").box(0.010, 0.044, 0.112).translate((0.043, 0.0, -0.004))
    boot = boot.union(side_guard)
    boot = boot.union(side_guard.translate((-0.086, 0.0, 0.0)))
    boot = boot.union(cq.Workplane("XY").box(0.070, 0.048, 0.016).translate((0.0, 0.0, 0.076)))
    boot = boot.union(cq.Workplane("XY").box(0.070, 0.048, 0.016).translate((0.0, 0.0, -0.076)))
    return boot


def _build_kickstand_shape() -> object:
    panel = cq.Workplane("XY").box(0.050, 0.004, 0.104).translate((0.0, -0.002, 0.052))
    foot = cq.Workplane("XY").box(0.060, 0.006, 0.012).translate((0.0, -0.003, 0.098))
    lower_rib = cq.Workplane("XY").box(0.028, 0.006, 0.010).translate((0.0, -0.003, 0.012))
    return panel.union(foot).union(lower_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_multimeter")

    boot_yellow = model.material("boot_yellow", rgba=(0.91, 0.75, 0.15, 1.0))
    case_black = model.material("case_black", rgba=(0.14, 0.15, 0.16, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.04, 0.05, 0.06, 1.0))
    glass_green = model.material("glass_green", rgba=(0.58, 0.72, 0.56, 0.85))
    button_grey = model.material("button_grey", rgba=(0.42, 0.45, 0.48, 1.0))
    jack_dark = model.material("jack_dark", rgba=(0.05, 0.05, 0.05, 1.0))
    dial_black = model.material("dial_black", rgba=(0.11, 0.12, 0.13, 1.0))
    dial_grey = model.material("dial_grey", rgba=(0.30, 0.31, 0.33, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_boot_shape(), "multimeter_boot"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material=boot_yellow,
        name="boot",
    )
    body.visual(
        Box((CASE_W, CASE_D, CASE_H)),
        origin=Origin(xyz=(0.0, 0.0, CASE_H / 2.0)),
        material=case_black,
        name="case",
    )
    body.visual(
        Box((0.074, 0.003, 0.152)),
        origin=Origin(xyz=(0.0, PANEL_FACE_Y - 0.0015, 0.090)),
        material=panel_black,
        name="front_panel",
    )
    body.visual(
        Box((0.064, 0.004, 0.046)),
        origin=Origin(xyz=(0.0, PANEL_FACE_Y, 0.129)),
        material=bezel_black,
        name="screen_bezel",
    )
    body.visual(
        Box((0.052, 0.0018, 0.030)),
        origin=Origin(xyz=(0.0, PANEL_FACE_Y + 0.0019, 0.129)),
        material=glass_green,
        name="screen_glass",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.003),
        origin=Origin(xyz=(0.0, PANEL_FACE_Y - 0.0005, 0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_grey,
        name="dial_seat",
    )
    body.visual(
        Box((0.060, 0.003, 0.016)),
        origin=Origin(xyz=(0.0, PANEL_FACE_Y - 0.0005, 0.094)),
        material=bezel_black,
        name="button_shelf",
    )
    body.visual(
        Box((0.068, 0.003, 0.034)),
        origin=Origin(xyz=(0.0, PANEL_FACE_Y - 0.0005, 0.031)),
        material=bezel_black,
        name="jack_panel",
    )
    for index, x_pos in enumerate((-0.022, 0.0, 0.022)):
        body.visual(
            Cylinder(radius=0.0065, length=0.0035),
            origin=Origin(
                xyz=(x_pos, PANEL_FACE_Y + 0.00025, 0.029),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=jack_dark,
            name=f"jack_{index}",
        )
    body.visual(
        Box((0.060, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.020, 0.026)),
        material=case_black,
        name="kickstand_mount",
    )
    body.visual(
        Box((0.028, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.019, 0.047)),
        material=case_black,
        name="kickstand_stop",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.018,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.058, 0.0045, flare=0.06),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "multimeter_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_grey,
        name="dial_hub",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, PANEL_FACE_Y + 0.0005, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    button_specs = (
        ("hold_button", -0.020),
        ("minmax_button", 0.0),
        ("backlight_button", 0.020),
    )
    for part_name, x_pos in button_specs:
        button = model.part(part_name)
        button.visual(
            Box((0.014, 0.006, 0.009)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, PANEL_FACE_Y + 0.0022, 0.094)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0012,
            ),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        mesh_from_cadquery(_build_kickstand_shape(), "multimeter_kickstand"),
        material=stand_dark,
        name="stand_panel",
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, REAR_FACE_Y, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=0.62,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    kickstand = object_model.get_part("kickstand")
    hold_button = object_model.get_part("hold_button")
    minmax_button = object_model.get_part("minmax_button")
    backlight_button = object_model.get_part("backlight_button")

    dial_joint = object_model.get_articulation("body_to_dial")
    kickstand_joint = object_model.get_articulation("body_to_kickstand")
    hold_joint = object_model.get_articulation("body_to_hold_button")
    minmax_joint = object_model.get_articulation("body_to_minmax_button")
    backlight_joint = object_model.get_articulation("body_to_backlight_button")

    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "kickstand uses revolute hinge",
        kickstand_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={kickstand_joint.articulation_type!r}",
    )
    for name, joint in (
        ("hold", hold_joint),
        ("minmax", minmax_joint),
        ("backlight", backlight_joint),
    ):
        ctx.check(
            f"{name} button uses prismatic travel",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type!r}",
        )

    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="dial_cap",
        elem_b="dial_seat",
        min_overlap=0.045,
        name="dial sits over the rotary seat",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="dial_cap",
        negative_elem="front_panel",
        min_gap=0.0,
        max_gap=0.004,
        name="dial stands proud of the front panel",
    )

    for name, button in (
        ("hold", hold_button),
        ("minmax", minmax_button),
        ("backlight", backlight_button),
    ):
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="button_shelf",
            min_gap=0.001,
            max_gap=0.004,
            name=f"{name} button sits just proud of the button shelf",
        )

    screen_aabb = ctx.part_element_world_aabb(body, elem="screen_bezel")
    ctx.check("screen bezel aabb present", screen_aabb is not None, details=f"aabb={screen_aabb!r}")
    if screen_aabb is not None:
        screen_bottom = float(screen_aabb[0][2])
        for name, button in (
            ("hold", hold_button),
            ("minmax", minmax_button),
            ("backlight", backlight_button),
        ):
            button_aabb = ctx.part_world_aabb(button)
            ctx.check(
                f"{name} button stays below the screen",
                button_aabb is not None and float(button_aabb[1][2]) < screen_bottom - 0.004,
                details=f"button={button_aabb!r}, screen_bottom={screen_bottom!r}",
            )

    for name, button, joint in (
        ("hold", hold_button, hold_joint),
        ("minmax", minmax_button, minmax_joint),
        ("backlight", backlight_button, backlight_joint),
    ):
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{name} button moves inward when pressed",
            rest_pos is not None
            and pressed_pos is not None
            and float(pressed_pos[1]) < float(rest_pos[1]) - 0.001,
            details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    stand_limits = kickstand_joint.motion_limits
    ctx.check(
        "kickstand open stop stays shallow",
        stand_limits is not None
        and stand_limits.upper is not None
        and 0.45 <= float(stand_limits.upper) <= 0.75,
        details=f"limits={stand_limits!r}",
    )

    with ctx.pose({kickstand_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(kickstand)
    with ctx.pose({kickstand_joint: stand_limits.upper if stand_limits is not None else 0.62}):
        open_aabb = ctx.part_world_aabb(kickstand)
    ctx.check(
        "kickstand swings rearward when opened",
        closed_aabb is not None
        and open_aabb is not None
        and float(open_aabb[0][1]) < float(closed_aabb[0][1]) - 0.030,
        details=f"closed={closed_aabb!r}, open={open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()

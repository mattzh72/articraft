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


BODY_DEPTH = 0.036
JAW_DEPTH = 0.014
JAW_PIVOT_X = -0.018
JAW_PIVOT_Z = 0.212


def _extruded_profile(
    points: list[tuple[float, float]],
    *,
    plane: str = "XZ",
    depth: float,
    both: bool = True,
    fillet: float | None = None,
):
    solid = cq.Workplane(plane).polyline(points).close().extrude(depth * 0.5, both=both)
    if fillet is not None and fillet > 0.0:
        try:
            solid = solid.edges("|Y").fillet(fillet)
        except Exception:
            pass
    return solid


def _build_body_shell():
    housing_profile = [
        (-0.027, 0.000),
        (-0.032, 0.028),
        (-0.035, 0.072),
        (-0.036, 0.120),
        (-0.034, 0.155),
        (-0.029, 0.178),
        (-0.021, 0.197),
        (-0.019, 0.214),
        (-0.010, 0.224),
        (0.008, 0.224),
        (0.018, 0.218),
        (0.022, 0.205),
        (0.018, 0.188),
        (0.012, 0.176),
        (0.010, 0.150),
        (0.011, 0.112),
        (0.011, 0.070),
        (0.010, 0.035),
        (0.008, 0.000),
    ]
    body = _extruded_profile(housing_profile, depth=BODY_DEPTH, fillet=0.0032)
    jaw_relief = cq.Workplane("XY").box(0.064, BODY_DEPTH + 0.010, 0.074).translate((0.008, 0.0, 0.196))
    body = body.cut(jaw_relief)
    trigger_recess = cq.Workplane("XY").box(0.032, 0.028, 0.048).translate((-0.010, 0.004, 0.112))
    button_left_recess = cq.Workplane("XY").box(0.014, 0.020, 0.011).translate((-0.012, 0.008, 0.025))
    button_right_recess = cq.Workplane("XY").box(0.014, 0.020, 0.011).translate((0.012, 0.008, 0.025))
    body = body.cut(trigger_recess).cut(button_left_recess).cut(button_right_recess)

    return body


def _build_jaw_arm():
    jaw_profile = [
        (-0.003, 0.006),
        (0.013, 0.007),
        (0.028, 0.001),
        (0.037, -0.009),
        (0.038, -0.017),
        (0.033, -0.025),
        (0.025, -0.031),
        (0.017, -0.032),
        (0.021, -0.024),
        (0.026, -0.014),
        (0.023, -0.006),
        (0.012, -0.001),
        (-0.003, -0.002),
    ]
    arm = _extruded_profile(jaw_profile, depth=JAW_DEPTH, fillet=0.0012)
    barrel = cq.Workplane("XZ").center(0.0, 0.0).circle(0.0052).extrude(JAW_DEPTH * 0.5, both=True)
    return arm.union(barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hvac_clamp_meter")

    housing_dark = model.material("housing_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    jaw_black = model.material("jaw_black", rgba=(0.09, 0.09, 0.10, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.94, 0.48, 0.12, 1.0))
    dial_gray = model.material("dial_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    button_blue = model.material("button_blue", rgba=(0.20, 0.39, 0.62, 1.0))
    display_glass = model.material("display_glass", rgba=(0.52, 0.70, 0.76, 0.40))
    light_mark = model.material("light_mark", rgba=(0.88, 0.89, 0.86, 1.0))

    body_shell_mesh = _build_body_shell()
    jaw_mesh = _build_jaw_arm()

    body = model.part("body")
    body.visual(mesh_from_cadquery(body_shell_mesh, "clamp_meter_body_shell"), material=housing_dark, name="housing")
    body.visual(
        Box((0.020, 0.010, 0.022)),
        origin=Origin(xyz=(-0.026, 0.013, 0.208)),
        material=jaw_black,
        name="upper_cheek",
    )
    body.visual(
        Box((0.020, 0.010, 0.022)),
        origin=Origin(xyz=(-0.026, -0.013, 0.208)),
        material=jaw_black,
        name="lower_cheek",
    )
    body.visual(
        Box((0.010, 0.010, 0.036)),
        origin=Origin(xyz=(-0.022, 0.013, 0.189)),
        material=jaw_black,
        name="upper_rib",
    )
    body.visual(
        Box((0.010, 0.010, 0.036)),
        origin=Origin(xyz=(-0.022, -0.013, 0.189)),
        material=jaw_black,
        name="lower_rib",
    )
    body.visual(
        Box((0.052, JAW_DEPTH, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.168)),
        material=jaw_black,
        name="fixed_bridge",
    )
    body.visual(
        Box((0.008, JAW_DEPTH, 0.020)),
        origin=Origin(xyz=(0.026, 0.000, 0.178)),
        material=jaw_black,
        name="fixed_hook",
    )
    body.visual(
        Box((0.008, 0.010, 0.012)),
        origin=Origin(xyz=(0.026, 0.000, 0.191)),
        material=jaw_black,
        name="fixed_neck",
    )
    body.visual(
        Box((0.008, 0.010, 0.006)),
        origin=Origin(xyz=(0.026, 0.000, 0.197)),
        material=jaw_black,
        name="fixed_tip",
    )
    body.visual(
        Box((0.012, BODY_DEPTH + 0.002, 0.118)),
        origin=Origin(xyz=(-0.028, 0.000, 0.083)),
        material=accent_orange,
        name="left_bumper",
    )
    body.visual(
        Box((0.012, BODY_DEPTH + 0.002, 0.118)),
        origin=Origin(xyz=(0.006, 0.000, 0.083)),
        material=accent_orange,
        name="right_bumper",
    )
    body.visual(
        Box((0.040, 0.003, 0.032)),
        origin=Origin(xyz=(-0.002, 0.0165, 0.138)),
        material=jaw_black,
        name="display_bezel",
    )
    body.visual(
        Box((0.034, 0.0015, 0.025)),
        origin=Origin(xyz=(-0.002, 0.0180, 0.138)),
        material=display_glass,
        name="display",
    )
    body.visual(
        Box((0.038, 0.003, 0.014)),
        origin=Origin(xyz=(0.000, 0.0150, 0.025)),
        material=housing_dark,
        name="button_panel",
    )
    body.visual(
        Box((0.004, 0.0015, 0.012)),
        origin=Origin(xyz=(0.000, 0.0180, 0.103)),
        material=light_mark,
        name="dial_tick",
    )

    jaw = model.part("jaw")
    jaw.visual(mesh_from_cadquery(jaw_mesh, "clamp_meter_moving_jaw"), material=jaw_black, name="jaw")
    jaw.visual(
        Box((0.008, 0.010, 0.005)),
        origin=Origin(xyz=(0.036, 0.000, -0.015)),
        material=jaw_black,
        name="jaw_tip",
    )

    model.articulation(
        "jaw_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(JAW_PIVOT_X, 0.0, JAW_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.028, 0.008, 0.036)),
        origin=Origin(xyz=(0.000, 0.004, 0.000)),
        material=accent_orange,
        name="trigger_face",
    )
    trigger.visual(
        Box((0.018, 0.014, 0.030)),
        origin=Origin(xyz=(0.000, -0.007, 0.000)),
        material=accent_orange,
        name="trigger_stem",
    )
    model.articulation(
        "trigger_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(-0.010, BODY_DEPTH * 0.5, 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.010),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.000, 0.002, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=jaw_black,
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.000, 0.007, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_gray,
        name="dial_body",
    )
    dial.visual(
        Box((0.003, 0.002, 0.012)),
        origin=Origin(xyz=(0.000, 0.013, 0.010)),
        material=light_mark,
        name="dial_pointer",
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.000, BODY_DEPTH * 0.5, 0.073)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=5.0),
    )

    for button_index, button_x in enumerate((-0.012, 0.012)):
        button = model.part(f"mode_button_{button_index}")
        button.visual(
            Box((0.010, 0.004, 0.008)),
            origin=Origin(xyz=(0.000, 0.002, 0.000)),
            material=button_blue,
            name="button_cap",
        )
        button.visual(
            Box((0.006, 0.010, 0.006)),
            origin=Origin(xyz=(0.000, -0.005, 0.000)),
            material=button_blue,
            name="button_stem",
        )
        model.articulation(
            f"mode_button_{button_index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, 0.0165, 0.025)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    jaw_hinge = object_model.get_articulation("jaw_hinge")
    jaw_limits = jaw_hinge.motion_limits
    trigger = object_model.get_part("trigger")
    trigger_slide = object_model.get_articulation("trigger_slide")
    dial = object_model.get_part("dial")
    dial_turn = object_model.get_articulation("dial_turn")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")
    button_0_press = object_model.get_articulation("mode_button_0_press")
    button_1_press = object_model.get_articulation("mode_button_1_press")

    with ctx.pose({jaw_hinge: 0.0}):
        ctx.expect_contact(
            jaw,
            body,
            elem_a="jaw_tip",
            elem_b="fixed_tip",
            contact_tol=0.0035,
            name="jaw closes to the fixed tip",
        )

    if jaw_limits is not None and jaw_limits.upper is not None:
        with ctx.pose({jaw_hinge: jaw_limits.upper}):
            ctx.expect_gap(
                jaw,
                body,
                axis="z",
                positive_elem="jaw_tip",
                negative_elem="fixed_tip",
                min_gap=0.010,
                name="jaw tip lifts above the fixed tip when opened",
            )

    ctx.expect_within(
        trigger,
        body,
        axes="xz",
        inner_elem="trigger_stem",
        outer_elem="housing",
        margin=0.004,
        name="trigger stem stays captured in the handle slot",
    )

    trigger_rest = ctx.part_world_position(trigger)
    trigger_upper = trigger_slide.motion_limits.upper if trigger_slide.motion_limits is not None else None
    if trigger_rest is not None and trigger_upper is not None:
        with ctx.pose({trigger_slide: trigger_upper}):
            ctx.expect_within(
                trigger,
                body,
                axes="xz",
                inner_elem="trigger_stem",
                outer_elem="housing",
                margin=0.004,
                name="pressed trigger stem stays captured in the handle slot",
            )
            trigger_pressed = ctx.part_world_position(trigger)
        ctx.check(
            "trigger presses into the handle",
            trigger_pressed is not None and trigger_pressed[1] < trigger_rest[1] - 0.008,
            details=f"rest={trigger_rest}, pressed={trigger_pressed}",
        )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_turn: math.radians(120.0)}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place on the front face",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_rest[0] - dial_turned[0]) < 1e-6
        and abs(dial_rest[1] - dial_turned[1]) < 1e-6
        and abs(dial_rest[2] - dial_turned[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_0_upper = button_0_press.motion_limits.upper if button_0_press.motion_limits is not None else None
    button_1_upper = button_1_press.motion_limits.upper if button_1_press.motion_limits is not None else None

    if button_0_upper is not None and button_0_rest is not None and button_1_rest is not None:
        with ctx.pose({button_0_press: button_0_upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_idle = ctx.part_world_position(button_1)
        ctx.check(
            "mode button 0 presses independently",
            button_0_pressed is not None
            and button_0_pressed[1] < button_0_rest[1] - 0.0015
            and button_1_idle is not None
            and abs(button_1_idle[1] - button_1_rest[1]) < 1e-6,
            details=f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, button_1_rest={button_1_rest}, button_1_idle={button_1_idle}",
        )

    if button_1_upper is not None and button_0_rest is not None and button_1_rest is not None:
        with ctx.pose({button_1_press: button_1_upper}):
            button_0_idle = ctx.part_world_position(button_0)
            button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "mode button 1 presses independently",
            button_1_pressed is not None
            and button_1_pressed[1] < button_1_rest[1] - 0.0015
            and button_0_idle is not None
            and abs(button_0_idle[1] - button_0_rest[1]) < 1e-6,
            details=f"button_0_rest={button_0_rest}, button_0_idle={button_0_idle}, button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()

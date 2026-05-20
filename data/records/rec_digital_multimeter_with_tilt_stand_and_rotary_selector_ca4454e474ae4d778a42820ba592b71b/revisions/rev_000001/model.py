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


BODY_WIDTH = 0.092
BODY_DEPTH = 0.041
BODY_HEIGHT = 0.184

BUMPER_WIDTH = 0.104
BUMPER_DEPTH = 0.053
BUMPER_HEIGHT = 0.196

FRONT_Y = -BODY_DEPTH * 0.5
BACK_Y = BODY_DEPTH * 0.5

DISPLAY_Z = 0.060
SELECTOR_Z = -0.005
KEY_ROW_Z = -0.065
FLASHLIGHT_X = 0.029
FLASHLIGHT_Z = 0.078


def _body_case_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
    body = body.edges("|Y").fillet(0.014)

    front = body.faces("<Y").workplane()
    body = (
        front.center(0.0, DISPLAY_Z)
        .rect(0.074, 0.048)
        .cutBlind(0.0025)
        .faces("<Y")
        .workplane()
        .center(0.0, DISPLAY_Z)
        .rect(0.058, 0.032)
        .cutBlind(0.0055)
    )

    body = (
        body.faces("<Y")
        .workplane()
        .center(0.0, SELECTOR_Z)
        .circle(0.032)
        .cutBlind(0.003)
        .faces("<Y")
        .workplane()
        .center(0.0, SELECTOR_Z)
        .circle(0.0065)
        .cutBlind(0.015)
    )

    key_x_positions = (-0.024, 0.0, 0.024)
    body = (
        body.faces("<Y")
        .workplane()
        .pushPoints([(x, KEY_ROW_Z) for x in key_x_positions])
        .rect(0.018, 0.010)
        .cutBlind(0.004)
    )
    body = (
        body.faces("<Y")
        .workplane()
        .pushPoints([(x, KEY_ROW_Z) for x in key_x_positions])
        .rect(0.012, 0.006)
        .cutBlind(0.014)
    )

    body = (
        body.faces("<Y")
        .workplane()
        .center(FLASHLIGHT_X, FLASHLIGHT_Z)
        .circle(0.007)
        .cutBlind(0.003)
    )
    body = (
        body.faces("<Y")
        .workplane()
        .center(FLASHLIGHT_X, FLASHLIGHT_Z)
        .circle(0.0035)
        .cutBlind(0.013)
    )

    return body


def _bumper_shape() -> cq.Workplane:
    bumper = cq.Workplane("XY").box(BUMPER_WIDTH, BUMPER_DEPTH, BUMPER_HEIGHT)
    bumper = bumper.edges("|Y").fillet(0.018)
    bumper = bumper.cut(cq.Workplane("XY").box(0.0935, 0.0430, 0.1855))
    bumper = bumper.cut(
        cq.Workplane("XY")
        .box(0.086, 0.024, 0.174)
        .translate((0.0, -0.0145, 0.0))
    )
    bumper = bumper.cut(
        cq.Workplane("XY")
        .box(0.074, 0.024, 0.132)
        .translate((0.0, 0.0145, -0.010))
    )
    return bumper


def _bezel_shape() -> cq.Workplane:
    bezel = cq.Workplane("XY").box(0.074, 0.004, 0.048)
    bezel = bezel.edges("|Y").fillet(0.008)
    bezel = bezel.cut(cq.Workplane("XY").box(0.056, 0.006, 0.030))
    return bezel


def _selector_cap_mesh():
    selector = KnobGeometry(
        0.050,
        0.014,
        body_style="skirted",
        top_diameter=0.042,
        skirt=KnobSkirt(0.058, 0.003, flare=0.12),
        grip=KnobGrip(style="fluted", count=16, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(selector, "multimeter_selector")


def _stand_shape() -> cq.Workplane:
    stand = cq.Workplane("XY").box(0.062, 0.003, 0.112)
    stand = stand.translate((0.0, 0.0015, 0.056))
    stand = stand.cut(
        cq.Workplane("XY")
        .box(0.040, 0.006, 0.078)
        .translate((0.0, 0.0015, 0.054))
    )
    stand = stand.cut(
        cq.Workplane("XY")
        .box(0.030, 0.006, 0.020)
        .translate((0.0, 0.0015, 0.096))
    )
    hinge_ridge = cq.Workplane("XY").box(0.050, 0.004, 0.010)
    hinge_ridge = hinge_ridge.translate((0.0, 0.0, 0.005))
    return stand.union(hinge_ridge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_multimeter")

    shell_mat = model.material("shell_grey", rgba=(0.23, 0.24, 0.26, 1.0))
    bumper_mat = model.material("bumper_orange", rgba=(0.91, 0.52, 0.14, 1.0))
    bezel_mat = model.material("bezel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    display_mat = model.material("display_glass", rgba=(0.24, 0.48, 0.44, 0.55))
    selector_mat = model.material("selector_black", rgba=(0.14, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_case_shape(), "multimeter_body"),
        material=shell_mat,
        name="case_shell",
    )
    for index, x_pos in enumerate((-0.025, 0.025)):
        body.visual(
            Box((0.012, 0.004, 0.014)),
            origin=Origin(xyz=(x_pos, BACK_Y - 0.0020, -0.074)),
            material=shell_mat,
            name=f"stand_mount_{index}",
        )
    for index, x_pos in enumerate((-0.0460, 0.0460)):
        body.visual(
            Box((0.0015, 0.020, 0.120)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=shell_mat,
            name=f"bumper_pad_{index}",
        )

    bumper = model.part("bumper")
    bumper.visual(
        mesh_from_cadquery(_bumper_shape(), "multimeter_bumper"),
        material=bumper_mat,
        name="bumper_shell",
    )
    model.articulation(
        "body_to_bumper",
        ArticulationType.FIXED,
        parent=body,
        child=bumper,
    )

    bezel = model.part("bezel")
    bezel.visual(
        mesh_from_cadquery(_bezel_shape(), "multimeter_bezel"),
        material=bezel_mat,
        name="lcd_bezel",
    )
    model.articulation(
        "body_to_bezel",
        ArticulationType.FIXED,
        parent=body,
        child=bezel,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0020, DISPLAY_Z)),
    )

    display = model.part("display")
    display.visual(
        Box((0.062, 0.002, 0.034)),
        material=display_mat,
        name="lcd",
    )
    model.articulation(
        "bezel_to_display",
        ArticulationType.FIXED,
        parent=bezel,
        child=display,
        origin=Origin(xyz=(0.0, 0.0030, 0.0)),
    )

    selector = model.part("selector")
    selector.visual(
        _selector_cap_mesh(),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=selector_mat,
        name="selector_cap",
    )
    selector.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.006, 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=selector_mat,
        name="selector_shaft",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, SELECTOR_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    key_mat = model.material("key_blue", rgba=(0.17, 0.28, 0.58, 1.0))
    light_button_mat = model.material("light_button_grey", rgba=(0.82, 0.83, 0.84, 1.0))
    stand_mat = model.material("stand_grey", rgba=(0.18, 0.19, 0.21, 1.0))

    for index, x_pos in enumerate((-0.024, 0.0, 0.024)):
        key = model.part(f"key_{index}")
        key.visual(
            Box((0.0180, 0.0032, 0.0100)),
            origin=Origin(xyz=(0.0, -0.0016, 0.0)),
            material=key_mat,
            name="key_cap",
        )
        key.visual(
            Box((0.0120, 0.0070, 0.0060)),
            origin=Origin(xyz=(0.0, 0.0035, 0.0)),
            material=key_mat,
            name="key_stem",
        )
        model.articulation(
            f"body_to_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.0040, KEY_ROW_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0018,
            ),
        )

    flashlight_button = model.part("flashlight_button")
    flashlight_button.visual(
        Cylinder(radius=0.0068, length=0.0028),
        origin=Origin(xyz=(0.0, -0.0014, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=light_button_mat,
        name="button_cap",
    )
    flashlight_button.visual(
        Cylinder(radius=0.0035, length=0.0070),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=light_button_mat,
        name="button_stem",
    )
    model.articulation(
        "body_to_flashlight_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flashlight_button,
        origin=Origin(xyz=(FLASHLIGHT_X, FRONT_Y + 0.0030, FLASHLIGHT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0012,
        ),
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "multimeter_stand"),
        material=stand_mat,
        name="stand_panel",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BACK_Y + 0.0020, -0.074)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bezel = object_model.get_part("bezel")
    display = object_model.get_part("display")
    selector = object_model.get_part("selector")
    key_0 = object_model.get_part("key_0")
    key_1 = object_model.get_part("key_1")
    key_2 = object_model.get_part("key_2")
    flashlight_button = object_model.get_part("flashlight_button")
    stand = object_model.get_part("stand")
    selector_joint = object_model.get_articulation("body_to_selector")
    key_joint = object_model.get_articulation("body_to_key_0")
    stand_joint = object_model.get_articulation("body_to_stand")

    for key in (key_0, key_1, key_2):
        ctx.allow_isolated_part(
            key,
            reason="Each function key is represented as a separate sliding cap with its hidden switch plunger guide omitted inside the sealed meter housing.",
        )

    ctx.expect_contact(
        bezel,
        body,
        elem_a="lcd_bezel",
        elem_b="case_shell",
        name="lcd bezel seats on the front case",
    )
    ctx.expect_within(
        display,
        bezel,
        axes="xz",
        margin=0.001,
        elem_a="lcd",
        elem_b="lcd_bezel",
        name="lcd stays within the bezel opening footprint",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="xz",
        min_overlap=0.040,
        elem_a="selector_cap",
        elem_b="case_shell",
        name="selector stays centered on the front dial area",
    )
    with ctx.pose({selector_joint: math.pi * 0.5}):
        ctx.expect_overlap(
            selector,
            body,
            axes="xz",
            min_overlap=0.040,
            elem_a="selector_cap",
            elem_b="case_shell",
            name="selector remains centered while rotated",
        )

    rest_key_0 = ctx.part_world_position(key_0)
    rest_key_1 = ctx.part_world_position(key_1)
    key_upper = key_joint.motion_limits.upper if key_joint.motion_limits is not None else None
    if key_upper is not None:
        with ctx.pose({key_joint: key_upper}):
            ctx.expect_overlap(
                body,
                key_0,
                axes="xz",
                min_overlap=0.009,
                elem_a="case_shell",
                elem_b="key_cap",
                name="pressed function key stays guided in its pocket",
            )
            pressed_key_0 = ctx.part_world_position(key_0)
            pressed_key_1 = ctx.part_world_position(key_1)
        ctx.check(
            "function key moves inward independently",
            rest_key_0 is not None
            and pressed_key_0 is not None
            and pressed_key_0[1] > rest_key_0[1] + 0.001
            and rest_key_1 is not None
            and pressed_key_1 is not None
            and abs(pressed_key_1[1] - rest_key_1[1]) < 1e-6,
            details=(
                f"key_0_rest={rest_key_0}, key_0_pressed={pressed_key_0}, "
                f"key_1_rest={rest_key_1}, key_1_pressed={pressed_key_1}"
            ),
        )

    ctx.expect_overlap(
        body,
        flashlight_button,
        axes="xz",
        min_overlap=0.010,
        elem_a="case_shell",
        elem_b="button_cap",
        name="flashlight button sits in the top corner pocket",
    )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        body_aabb = ctx.part_world_aabb(body)
        closed_aabb = ctx.part_world_aabb(stand)
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_aabb = ctx.part_world_aabb(stand)
        ctx.check(
            "stand opens rearward",
            body_aabb is not None
            and closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.030
            and open_aabb[1][1] > body_aabb[1][1] + 0.030,
            details=f"body={body_aabb}, closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

BODY_DEPTH = 0.390
BODY_WIDTH = 0.430
BODY_REAR_X = -BODY_DEPTH / 2.0
BODY_FRONT_X = BODY_DEPTH / 2.0
BODY_BACK_TOP_Z = 0.362
BODY_FRONT_TOP_Z = 0.302
BODY_TOP_BREAK_X = -0.040
PANEL_PITCH = math.atan2(
    BODY_BACK_TOP_Z - BODY_FRONT_TOP_Z,
    BODY_FRONT_X - BODY_TOP_BREAK_X,
)

DRAWER_TRAVEL = 0.165
DRAWER_SEAT_X = 0.159
DRAWER_SEAT_Z = 0.070

KNOB_X = 0.030
BUTTON_OFFSET_X = 0.058
BUTTON_OFFSET_Y = 0.062


def _panel_normal() -> tuple[float, float, float]:
    return (math.sin(PANEL_PITCH), 0.0, math.cos(PANEL_PITCH))


def _top_surface_z(x_pos: float) -> float:
    return BODY_BACK_TOP_Z - (x_pos - BODY_TOP_BREAK_X) * math.tan(PANEL_PITCH)


def _panel_point(x_pos: float, y_pos: float, normal_offset: float = 0.0) -> tuple[float, float, float]:
    nx, ny, nz = _panel_normal()
    return (
        x_pos + nx * normal_offset,
        y_pos + ny * normal_offset,
        _top_surface_z(x_pos) + nz * normal_offset,
    )


def _build_drawer_shape() -> cq.Workplane:
    outer_tub = cq.Workplane("XY").box(0.292, 0.346, 0.154).translate((-0.126, 0.0, 0.077))
    tub_cavity = cq.Workplane("XY").box(0.270, 0.324, 0.160).translate((-0.128, 0.0, 0.087))
    front_panel = cq.Workplane("XY").box(0.022, 0.364, 0.168).translate((0.025, 0.0, 0.082))
    drawer = outer_tub.cut(tub_cavity).union(front_panel)
    return drawer


def _build_basket_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(0.210, 0.270, 0.100).translate((-0.105, 0.0, 0.050))
    cavity = cq.Workplane("XY").box(0.192, 0.252, 0.108).translate((-0.107, 0.0, 0.058))
    return shell.cut(cavity)


def _build_handle_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.050, 0.236, 0.046).translate((0.025, 0.0, 0.0))
    grip_cut = cq.Workplane("XY").box(0.026, 0.166, 0.016).translate((0.010, 0.0, -0.013))
    return outer.cut(grip_cut)


def _preset_button_positions() -> list[tuple[float, float]]:
    return [
        (KNOB_X - BUTTON_OFFSET_X, -BUTTON_OFFSET_Y),
        (KNOB_X - BUTTON_OFFSET_X, BUTTON_OFFSET_Y),
        (KNOB_X + BUTTON_OFFSET_X, -BUTTON_OFFSET_Y),
        (KNOB_X + BUTTON_OFFSET_X, BUTTON_OFFSET_Y),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.21, 0.22, 0.23, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.32, 0.33, 0.35, 1.0))
    control_finish = model.material("control_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    button_finish = model.material("button_finish", rgba=(0.17, 0.18, 0.19, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.350, BODY_WIDTH, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=body_finish,
        name="base",
    )
    body.visual(
        Box((0.030, BODY_WIDTH, 0.334)),
        origin=Origin(xyz=(-0.180, 0.0, 0.167)),
        material=body_finish,
        name="rear_wall",
    )
    body.visual(
        Box((0.350, 0.032, 0.318)),
        origin=Origin(xyz=(0.0, 0.199, 0.159)),
        material=body_finish,
        name="side_wall_0",
    )
    body.visual(
        Box((0.350, 0.032, 0.318)),
        origin=Origin(xyz=(0.0, -0.199, 0.159)),
        material=body_finish,
        name="side_wall_1",
    )
    body.visual(
        Box((0.120, BODY_WIDTH, 0.028)),
        origin=Origin(xyz=(-0.105, 0.0, 0.348)),
        material=body_finish,
        name="rear_cap",
    )
    body.visual(
        Box((0.240, BODY_WIDTH, 0.028)),
        origin=Origin(
            xyz=_panel_point(0.078, 0.0, normal_offset=-0.014),
            rpy=(0.0, PANEL_PITCH, 0.0),
        ),
        material=body_finish,
        name="roof",
    )
    body.visual(
        Box((0.020, 0.366, 0.032)),
        origin=Origin(xyz=(0.185, 0.0, 0.259)),
        material=body_finish,
        name="front_lip",
    )
    body.visual(
        Box((0.020, 0.033, 0.202)),
        origin=Origin(xyz=(0.185, 0.1985, 0.159)),
        material=body_finish,
        name="front_side_0",
    )
    body.visual(
        Box((0.020, 0.033, 0.202)),
        origin=Origin(xyz=(0.185, -0.1985, 0.159)),
        material=body_finish,
        name="front_side_1",
    )
    body.visual(
        Box((0.160, 0.020, 0.012)),
        origin=Origin(xyz=(-0.095, 0.193, 0.124)),
        material=trim_finish,
        name="guide_0",
    )
    body.visual(
        Box((0.160, 0.020, 0.012)),
        origin=Origin(xyz=(-0.095, -0.193, 0.124)),
        material=trim_finish,
        name="guide_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_shape(), "air_fryer_drawer"),
        material=trim_finish,
        name="outer_shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket_shape(), "air_fryer_basket"),
        material=basket_finish,
        name="basket_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "air_fryer_handle"),
        material=trim_finish,
        name="handle_shell",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.010, 0.044, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.006)),
        material=button_finish,
        name="button_cap",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.028,
                body_style="skirted",
                top_diameter=0.050,
                skirt=KnobSkirt(0.074, 0.006, flare=0.08),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "air_fryer_selector_knob",
        ),
        material=control_finish,
        name="knob_cap",
    )

    preset_buttons = []
    for index, _ in enumerate(_preset_button_positions()):
        preset = model.part(f"preset_button_{index}")
        preset.visual(
            Box((0.038, 0.038, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_finish,
            name="button_cap",
        )
        preset_buttons.append(preset)

    drawer_joint = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_SEAT_X, 0.0, DRAWER_SEAT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=DRAWER_TRAVEL),
    )

    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(0.006, 0.0, 0.007)),
    )

    model.articulation(
        "drawer_to_handle",
        ArticulationType.FIXED,
        parent=drawer,
        child=handle,
        origin=Origin(xyz=(0.036, 0.0, 0.084)),
    )

    model.articulation(
        "handle_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=latch_button,
        origin=Origin(xyz=(0.050, 0.0, 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.06, lower=0.0, upper=0.004),
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=_panel_point(KNOB_X, 0.0), rpy=(0.0, PANEL_PITCH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.20, velocity=6.0),
    )

    for index, ((button_x, button_y), preset) in enumerate(zip(_preset_button_positions(), preset_buttons)):
        model.articulation(
            f"body_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=preset,
            origin=Origin(xyz=_panel_point(button_x, button_y), rpy=(0.0, PANEL_PITCH, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    latch_button = object_model.get_part("latch_button")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    selector_joint = object_model.get_articulation("body_to_selector_knob")
    latch_joint = object_model.get_articulation("handle_to_latch_button")

    ctx.allow_overlap(
        drawer,
        basket,
        elem_a="outer_shell",
        elem_b="basket_shell",
        reason="The basket nests inside the drawer shell, and the current shell proxy representation reports overlap for the nested tub geometry.",
    )
    ctx.allow_overlap(
        body,
        object_model.get_part("selector_knob"),
        elem_a="roof",
        elem_b="knob_cap",
        reason="The selector knob is seated against a simplified solid roof panel rather than a detailed shaft opening.",
    )
    for index in range(4):
        ctx.allow_overlap(
            body,
            object_model.get_part(f"preset_button_{index}"),
            elem_a="roof",
            elem_b="button_cap",
            reason="Each preset button is modeled flush against the simplified control-panel roof instead of with a detailed switch well.",
        )

    with ctx.pose({drawer_joint: 0.0}):
        drawer_rest = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="yz",
            min_overlap=0.140,
            name="drawer stays registered with the body opening when closed",
        )
        ctx.expect_within(
            basket,
            drawer,
            axes="yz",
            margin=0.010,
            name="basket shell sits inside the drawer shell",
        )

    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="yz",
            min_overlap=0.120,
            name="drawer stays aligned on the side guides when extended",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.080,
            name="drawer retains insertion at full extension",
        )

    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.140,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    panel_normal = _panel_normal()
    for index in range(4):
        button_part = object_model.get_part(f"preset_button_{index}")
        button_joint = object_model.get_articulation(f"body_to_preset_button_{index}")
        button_rest = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: 0.004}):
            button_pressed = ctx.part_world_position(button_part)

        moved = None
        if button_rest is not None and button_pressed is not None:
            moved = sum(
                (button_rest[axis] - button_pressed[axis]) * panel_normal[axis]
                for axis in range(3)
            )

        ctx.check(
            f"preset_button_{index}_presses_inward",
            moved is not None and moved > 0.0025,
            details=f"rest={button_rest}, pressed={button_pressed}, inward_motion={moved}",
        )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_joint: 0.004}):
        latch_pressed = ctx.part_world_position(latch_button)

    ctx.check(
        "handle latch button presses inward",
        latch_rest is not None
        and latch_pressed is not None
        and latch_pressed[0] < latch_rest[0] - 0.0025,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    ctx.check(
        "selector knob is continuous",
        getattr(selector_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"joint_type={getattr(selector_joint, 'articulation_type', None)}",
    )

    return ctx.report()


object_model = build_object_model()

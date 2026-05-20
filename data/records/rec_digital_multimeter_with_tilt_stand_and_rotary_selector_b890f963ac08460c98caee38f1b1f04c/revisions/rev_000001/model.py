from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

CASE_W = 0.088
CASE_H = 0.158
CASE_D = 0.036
CASE_FRONT_Z = CASE_D * 0.5
CASE_REAR_Z = -CASE_D * 0.5

FRONT_RECESS_W = 0.072
FRONT_RECESS_H = 0.138
FRONT_RECESS_D = 0.0020
FRONT_PANEL_T = 0.0016
FRONT_PANEL_CENTER_Z = CASE_FRONT_Z - FRONT_RECESS_D + (FRONT_PANEL_T * 0.5)
FRONT_PANEL_SURFACE_Z = FRONT_PANEL_CENTER_Z + (FRONT_PANEL_T * 0.5)

DISPLAY_Y = 0.045
DIAL_Y = -0.002
ROUND_BUTTON_Y = -0.041
SQUARE_BUTTON_Y = -0.058
BUTTON_X = 0.016

BUTTON_PROUD = 0.0008
BUTTON_TRAVEL = 0.0007
ROUND_BUTTON_H = 0.0022
SQUARE_BUTTON_H = 0.0020

STAND_HINGE_Y = -0.067


def _build_case_body() -> object:
    case = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_H, CASE_D)
        .edges("|Z")
        .fillet(0.011)
    )

    front_recess = cq.Workplane("XY").box(
        FRONT_RECESS_W,
        FRONT_RECESS_H,
        FRONT_RECESS_D + 0.0002,
    ).translate((0.0, 0.0, CASE_FRONT_Z - (FRONT_RECESS_D * 0.5)))
    display_recess = cq.Workplane("XY").box(
        0.052,
        0.030,
        0.0030,
    ).translate((0.0, DISPLAY_Y, CASE_FRONT_Z - 0.0017))
    dial_moat = (
        cq.Workplane("XY")
        .circle(0.024)
        .extrude(0.0012)
        .translate((0.0, DIAL_Y, CASE_FRONT_Z - 0.0012))
    )
    back_pocket = cq.Workplane("XY").box(
        0.048,
        0.102,
        0.0036,
    ).translate((0.0, -0.010, CASE_REAR_Z + 0.0018))

    jack_cutters = []
    for jack_x in (-0.021, 0.0, 0.021):
        jack_cutters.append(
            cq.Workplane("XY")
            .circle(0.0052)
            .extrude(0.0014)
            .translate((jack_x, -0.059, CASE_FRONT_Z - 0.0014))
        )

    case = case.cut(front_recess)
    case = case.cut(display_recess)
    case = case.cut(dial_moat)
    case = case.cut(back_pocket)
    for cutter in jack_cutters:
        case = case.cut(cutter)
    return case


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_multimeter")

    shell_yellow = model.material("shell_yellow", rgba=(0.92, 0.78, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.17, 0.18, 0.20, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    button_gray = model.material("button_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    button_dark = model.material("button_dark", rgba=(0.24, 0.25, 0.28, 1.0))
    display_glass = model.material("display_glass", rgba=(0.42, 0.69, 0.79, 0.55))
    dial_black = model.material("dial_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pointer_orange = model.material("pointer_orange", rgba=(0.95, 0.49, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_case_body(), "multimeter_body"),
        material=shell_yellow,
        name="shell",
    )
    body.visual(
        Box((FRONT_RECESS_W, FRONT_RECESS_H, FRONT_PANEL_T)),
        origin=Origin(xyz=(0.0, 0.0, FRONT_PANEL_CENTER_Z)),
        material=panel_black,
        name="front_panel",
    )
    body.visual(
        Box((0.050, 0.029, 0.0012)),
        origin=Origin(xyz=(0.0, DISPLAY_Y, CASE_FRONT_Z - 0.0020)),
        material=panel_dark,
        name="display_bezel",
    )
    for jack_index, jack_x in enumerate((-0.021, 0.0, 0.021)):
        body.visual(
            Cylinder(radius=0.0056, length=0.0012),
            origin=Origin(xyz=(jack_x, -0.059, CASE_FRONT_Z - 0.0016)),
            material=panel_dark,
            name=f"jack_ring_{jack_index}",
        )
    body.visual(
        Cylinder(radius=0.0215, length=0.0010),
        origin=Origin(xyz=(0.0, DIAL_Y, FRONT_PANEL_SURFACE_Z + 0.0002)),
        material=panel_dark,
        name="dial_ring",
    )
    body.visual(
        Cylinder(radius=0.0023, length=0.0400),
        origin=Origin(
            xyz=(0.0, STAND_HINGE_Y, CASE_REAR_Z + 0.0023),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=stand_gray,
        name="stand_barrel",
    )

    display = model.part("display")
    display.visual(
        Box((0.046, 0.026, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=display_glass,
        name="screen",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, DISPLAY_Y, CASE_FRONT_Z - 0.0022)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.014,
                body_style="skirted",
                top_diameter=0.027,
                base_diameter=0.036,
                edge_radius=0.001,
                side_draft_deg=8.0,
                center=False,
            ),
            "multimeter_dial",
        ),
        material=dial_black,
        name="knob",
    )
    dial.visual(
        Box((0.0024, 0.0100, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0060, 0.0136)),
        material=pointer_orange,
        name="pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_Y, FRONT_PANEL_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=4.0),
    )

    stand = model.part("stand")
    stand.visual(
        Box((0.044, 0.104, 0.0030)),
        origin=Origin(xyz=(0.0, 0.052, -0.0015)),
        material=stand_gray,
        name="panel",
    )
    stand.visual(
        Box((0.013, 0.070, 0.0040)),
        origin=Origin(xyz=(0.0, 0.043, -0.0035)),
        material=stand_gray,
        name="rib",
    )
    stand.visual(
        Box((0.036, 0.012, 0.0050)),
        origin=Origin(xyz=(0.0, 0.099, -0.0030)),
        material=stand_gray,
        name="foot",
    )
    stand.visual(
        Cylinder(radius=0.0022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.0015), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=stand_gray,
        name="barrel",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, STAND_HINGE_Y, CASE_REAR_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.08),
    )

    for index, button_x in enumerate((-BUTTON_X, BUTTON_X)):
        round_button = model.part(f"round_button_{index}")
        round_button.visual(
            Cylinder(radius=0.0055, length=ROUND_BUTTON_H),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD + (ROUND_BUTTON_H * 0.5))),
            material=button_gray,
            name="cap",
        )
        round_button.visual(
            Cylinder(radius=0.0024, length=BUTTON_PROUD),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD * 0.5)),
            material=button_dark,
            name="stem",
        )
        model.articulation(
            f"body_to_round_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=round_button,
            origin=Origin(xyz=(button_x, ROUND_BUTTON_Y, FRONT_PANEL_SURFACE_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    for index, button_x in enumerate((-BUTTON_X, BUTTON_X)):
        square_button = model.part(f"square_button_{index}")
        square_button.visual(
            Box((0.0115, 0.0115, SQUARE_BUTTON_H)),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD + (SQUARE_BUTTON_H * 0.5))),
            material=button_dark,
            name="cap",
        )
        square_button.visual(
            Box((0.0040, 0.0040, BUTTON_PROUD)),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD * 0.5)),
            material=button_dark,
            name="stem",
        )
        model.articulation(
            f"body_to_square_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=square_button,
            origin=Origin(xyz=(button_x, SQUARE_BUTTON_Y, FRONT_PANEL_SURFACE_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")

    dial_joint = object_model.get_articulation("body_to_dial")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.check(
        "dial articulation is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "stand articulation is revolute",
        stand_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={stand_joint.articulation_type!r}",
    )

    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="knob",
        negative_elem="front_panel",
        max_gap=0.0002,
        max_penetration=0.0,
        name="dial sits on front panel",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="knob",
        elem_b="front_panel",
        min_overlap=0.030,
        name="dial is centered within front panel footprint",
    )

    ctx.expect_gap(
        body,
        stand,
        axis="z",
        negative_elem="panel",
        max_gap=0.0002,
        max_penetration=0.0,
        name="stand nests against the rear shell",
    )

    for button_name in (
        "round_button_0",
        "round_button_1",
        "square_button_0",
        "square_button_1",
    ):
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"body_to_{button_name}")
        limits = joint.motion_limits

        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="cap",
            negative_elem="front_panel",
            min_gap=0.0006,
            max_gap=0.0010,
            name=f"{button_name} stands proud at rest",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xy",
            elem_a="cap",
            elem_b="front_panel",
            min_overlap=0.010,
            name=f"{button_name} stays on the control plate",
        )

        rest_pos = ctx.part_world_position(button)
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.expect_gap(
                    button,
                    body,
                    axis="z",
                    positive_elem="cap",
                    negative_elem="front_panel",
                    min_gap=0.0,
                    max_gap=0.0004,
                    name=f"{button_name} can press nearly flush",
                )
                pressed_pos = ctx.part_world_position(button)

            ctx.check(
                f"{button_name} presses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[2] < rest_pos[2] - 0.0005,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    stand_limits = stand_joint.motion_limits
    closed_panel_aabb = ctx.part_element_world_aabb(stand, elem="panel")
    if stand_limits is not None and stand_limits.upper is not None:
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_panel_aabb = ctx.part_element_world_aabb(stand, elem="panel")
        ctx.check(
            "stand swings outward",
            closed_panel_aabb is not None
            and open_panel_aabb is not None
            and open_panel_aabb[0][2] < closed_panel_aabb[0][2] - 0.040,
            details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

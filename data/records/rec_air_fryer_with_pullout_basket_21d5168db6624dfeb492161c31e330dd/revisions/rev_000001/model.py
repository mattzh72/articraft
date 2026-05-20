from __future__ import annotations

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


BODY_DEPTH = 0.310
BODY_WIDTH = 0.260
SHELL_BOTTOM = 0.012
BODY_HEIGHT = 0.298
TOTAL_HEIGHT = SHELL_BOTTOM + BODY_HEIGHT

CAVITY_DEPTH = 0.228
CAVITY_WIDTH = 0.194
CAVITY_HEIGHT = 0.086
CAVITY_CENTER_Z = 0.096

BASKET_DEPTH = 0.214
BASKET_WIDTH = 0.186
BASKET_HEIGHT = 0.076
BASKET_TRAVEL = 0.115

FRONT_PANEL_THICKNESS = 0.022
FRONT_PANEL_WIDTH = 0.202
FRONT_PANEL_HEIGHT = 0.088

HANDLE_DEPTH = 0.072
HANDLE_WIDTH = 0.054
HANDLE_HEIGHT = 0.036
HANDLE_CENTER_X = FRONT_PANEL_THICKNESS + HANDLE_DEPTH * 0.5 - 0.004
HANDLE_CENTER_Z = -0.006

RELEASE_BUTTON_WIDTH = 0.014
RELEASE_BUTTON_DEPTH = 0.022
RELEASE_BUTTON_HEIGHT = 0.006
RELEASE_TRAVEL = 0.004

PROGRAM_BUTTON_WIDTH = 0.020
PROGRAM_BUTTON_HEIGHT = 0.012
PROGRAM_BUTTON_DEPTH = 0.010
PROGRAM_BUTTON_TRAVEL = 0.004

ROCKER_WIDTH = 0.032
ROCKER_HEIGHT = 0.034
ROCKER_THICKNESS = 0.006


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, SHELL_BOTTOM + BODY_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.032)
    )

    cavity = (
        cq.Workplane("XY")
        .box(CAVITY_DEPTH, CAVITY_WIDTH, CAVITY_HEIGHT)
        .translate(
            (
                BODY_DEPTH * 0.5 - CAVITY_DEPTH * 0.5,
                0.0,
                CAVITY_CENTER_Z,
            )
        )
    )

    shell = shell.cut(cavity)

    knob_pedestal = (
        cq.Workplane("XY")
        .cylinder(0.006, 0.024)
        .translate((0.008, 0.0, TOTAL_HEIGHT + 0.003))
    )
    knob_recess = (
        cq.Workplane("XY")
        .cylinder(0.002, 0.010)
        .translate((0.008, 0.0, TOTAL_HEIGHT + 0.005))
    )
    shell = shell.union(knob_pedestal).cut(knob_recess)

    button_pocket = (
        cq.Workplane("XY")
        .box(0.012, PROGRAM_BUTTON_WIDTH, PROGRAM_BUTTON_HEIGHT)
        .translate((BODY_DEPTH * 0.5 - 0.006, 0.0, 0.208))
    )
    shell = shell.cut(button_pocket)

    rocker_opening = (
        cq.Workplane("XY")
        .box(0.032, 0.016, 0.038)
        .translate((0.040, BODY_WIDTH * 0.5 - 0.008, 0.178))
    )
    shell = shell.cut(rocker_opening)

    foot_centers = (
        (-0.100, -0.086),
        (-0.100, 0.086),
        (0.080, -0.086),
        (0.080, 0.086),
    )
    for foot_x, foot_y in foot_centers:
        shell = shell.union(
            cq.Workplane("XY")
            .box(0.030, 0.018, SHELL_BOTTOM)
            .translate((foot_x, foot_y, SHELL_BOTTOM * 0.5))
        )

    return shell


def _basket_shape() -> cq.Workplane:
    tray = (
        cq.Workplane("XY")
        .box(BASKET_DEPTH, BASKET_WIDTH, BASKET_HEIGHT)
        .translate((-BASKET_DEPTH * 0.5, 0.0, 0.0))
        .faces(">Z")
        .shell(-0.003)
    )

    front_panel = (
        cq.Workplane("XY")
        .box(FRONT_PANEL_THICKNESS, FRONT_PANEL_WIDTH, FRONT_PANEL_HEIGHT)
        .translate((FRONT_PANEL_THICKNESS * 0.5, 0.0, 0.0))
    )

    handle_block = (
        cq.Workplane("XY")
        .box(HANDLE_DEPTH, HANDLE_WIDTH, HANDLE_HEIGHT)
        .translate((HANDLE_CENTER_X, 0.0, HANDLE_CENTER_Z))
    )
    grip_recess = (
        cq.Workplane("XY")
        .box(HANDLE_DEPTH * 0.72, HANDLE_WIDTH * 0.52, HANDLE_HEIGHT * 0.54)
        .translate((HANDLE_CENTER_X + 0.005, 0.0, HANDLE_CENTER_Z - 0.008))
    )
    release_pocket = (
        cq.Workplane("XY")
        .box(0.026, 0.018, 0.007)
        .translate(
            (
                HANDLE_CENTER_X - 0.005,
                0.0,
                HANDLE_CENTER_Z + HANDLE_HEIGHT * 0.5 - 0.0035,
            )
        )
    )

    handle = handle_block.cut(grip_recess).cut(release_pocket)
    return tray.union(front_panel).union(handle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_air_fryer")

    shell_finish = model.material("shell_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    release_finish = model.material("release_finish", rgba=(0.82, 0.83, 0.84, 1.0))
    control_finish = model.material("control_finish", rgba=(0.13, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "air_fryer_body"),
        material=shell_finish,
        name="shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "air_fryer_basket"),
        material=basket_finish,
        name="basket_shell",
    )
    basket.visual(
        Box((HANDLE_DEPTH, HANDLE_WIDTH, HANDLE_HEIGHT)),
        origin=Origin(xyz=(HANDLE_CENTER_X + 0.003, 0.0, HANDLE_CENTER_Z)),
        material=handle_finish,
        name="handle",
    )
    for guide_index, guide_y in enumerate(
        (
            BASKET_WIDTH * 0.5 + 0.002,
            -(BASKET_WIDTH * 0.5 + 0.002),
        )
    ):
        basket.visual(
            Box((0.120, 0.004, 0.014)),
            origin=Origin(xyz=(-0.086, guide_y, -0.020)),
            material=basket_finish,
            name=f"guide_{guide_index}",
        )

    model.articulation(
        "body_to_basket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=basket,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.001, 0.0, CAVITY_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.28,
            lower=0.0,
            upper=BASKET_TRAVEL,
        ),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((RELEASE_BUTTON_DEPTH, RELEASE_BUTTON_WIDTH, RELEASE_BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RELEASE_BUTTON_HEIGHT * 0.5)),
        material=release_finish,
        name="release_cap",
    )

    model.articulation(
        "basket_to_release_button",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=release_button,
        origin=Origin(
            xyz=(
                HANDLE_CENTER_X - 0.005,
                0.0,
                HANDLE_CENTER_Z + HANDLE_HEIGHT * 0.5,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=RELEASE_TRAVEL,
        ),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.026,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.056, 0.006, flare=0.05),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "timer_knob",
        ),
        material=control_finish,
        name="knob_shell",
    )

    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(0.008, 0.0, TOTAL_HEIGHT + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.30, velocity=6.0),
    )

    program_button = model.part("program_button")
    program_button.visual(
        Box((PROGRAM_BUTTON_DEPTH, PROGRAM_BUTTON_WIDTH, PROGRAM_BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.001, 0.0, 0.0)),
        material=control_finish,
        name="button_face",
    )

    model.articulation(
        "body_to_program_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=program_button,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, 0.208)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=0.0,
            upper=PROGRAM_BUTTON_TRAVEL,
        ),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((ROCKER_WIDTH, ROCKER_THICKNESS, ROCKER_HEIGHT)),
        origin=Origin(xyz=(0.0, ROCKER_THICKNESS * 0.5, -ROCKER_HEIGHT * 0.5)),
        material=control_finish,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((ROCKER_WIDTH, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, -0.004)),
        material=control_finish,
        name="rocker_barrel",
    )

    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(0.040, BODY_WIDTH * 0.5, 0.195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-0.28,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    power_rocker = object_model.get_part("power_rocker")
    program_button = object_model.get_part("program_button")
    release_button = object_model.get_part("release_button")
    timer_knob = object_model.get_part("timer_knob")

    basket_slide = object_model.get_articulation("body_to_basket")
    rocker_hinge = object_model.get_articulation("body_to_power_rocker")
    program_slide = object_model.get_articulation("body_to_program_button")
    release_slide = object_model.get_articulation("basket_to_release_button")

    ctx.expect_gap(
        basket,
        body,
        axis="x",
        positive_elem="handle",
        negative_elem="shell",
        min_gap=0.020,
        max_gap=0.080,
        name="basket handle projects ahead of the shell",
    )
    ctx.expect_within(
        basket,
        body,
        axes="yz",
        inner_elem="basket_shell",
        outer_elem="shell",
        margin=0.010,
        name="basket stays centered in the fryer body footprint",
    )
    ctx.expect_gap(
        timer_knob,
        body,
        axis="z",
        positive_elem="knob_shell",
        negative_elem="shell",
        max_gap=0.008,
        max_penetration=0.0,
        name="timer knob sits on the top control pedestal",
    )

    rest_basket_pos = ctx.part_world_position(basket)
    extended_basket_pos = None
    basket_limits = basket_slide.motion_limits
    if basket_limits is not None and basket_limits.upper is not None:
        with ctx.pose({basket_slide: basket_limits.upper}):
            ctx.expect_overlap(
                basket,
                body,
                axes="x",
                elem_a="basket_shell",
                elem_b="shell",
                min_overlap=0.080,
                name="extended basket remains engaged in the body",
            )
            ctx.expect_within(
                basket,
                body,
                axes="yz",
                inner_elem="basket_shell",
                outer_elem="shell",
                margin=0.010,
                name="basket stays aligned while extended",
            )
            extended_basket_pos = ctx.part_world_position(basket)

    ctx.check(
        "basket extends outward from the fryer body",
        rest_basket_pos is not None
        and extended_basket_pos is not None
        and extended_basket_pos[0] > rest_basket_pos[0] + 0.08,
        details=f"rest={rest_basket_pos}, extended={extended_basket_pos}",
    )

    rest_release_pos = ctx.part_world_position(release_button)
    pressed_release_pos = None
    release_limits = release_slide.motion_limits
    if release_limits is not None and release_limits.upper is not None:
        with ctx.pose({release_slide: release_limits.upper}):
            pressed_release_pos = ctx.part_world_position(release_button)

    ctx.check(
        "release button presses down into the basket handle",
        rest_release_pos is not None
        and pressed_release_pos is not None
        and pressed_release_pos[2] < rest_release_pos[2] - 0.002,
        details=f"rest={rest_release_pos}, pressed={pressed_release_pos}",
    )

    rest_program_pos = ctx.part_world_position(program_button)
    pressed_program_pos = None
    program_limits = program_slide.motion_limits
    if program_limits is not None and program_limits.upper is not None:
        with ctx.pose({program_slide: program_limits.upper}):
            pressed_program_pos = ctx.part_world_position(program_button)

    ctx.check(
        "program button pushes into the front panel",
        rest_program_pos is not None
        and pressed_program_pos is not None
        and pressed_program_pos[0] < rest_program_pos[0] - 0.002,
        details=f"rest={rest_program_pos}, pressed={pressed_program_pos}",
    )

    rocker_low = None
    rocker_high = None
    rocker_limits = rocker_hinge.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_hinge: rocker_limits.lower}):
            rocker_low = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        with ctx.pose({rocker_hinge: rocker_limits.upper}):
            rocker_high = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")

    ctx.check(
        "power rocker pivots outward on its side hinge",
        rocker_low is not None
        and rocker_high is not None
        and rocker_high[1][1] > rocker_low[1][1] + 0.004,
        details=f"lower={rocker_low}, upper={rocker_high}",
    )

    return ctx.report()


object_model = build_object_model()

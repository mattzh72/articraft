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

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.60
CANOPY_HEIGHT = 0.085
CANOPY_WALL = 0.015
CANOPY_TOP = 0.006

CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.22
CHIMNEY_HEIGHT = 0.88
CHIMNEY_WALL = 0.008
CEILING_PLATE_WIDTH = 0.38
CEILING_PLATE_DEPTH = 0.28
CEILING_PLATE_THICKNESS = 0.018

STRIP_WIDTH = 0.31
STRIP_HEIGHT = 0.046
STRIP_THICKNESS = 0.006
STRIP_CENTER_Z = -0.028
CANOPY_FRONT_OPENING_WIDTH = 0.28
CANOPY_FRONT_OPENING_HEIGHT = 0.026

DIAL_X = -0.092
BUTTON_XS = (0.020, 0.072, 0.124)
BUTTON_TRAVEL = 0.004


def _make_canopy_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT).translate(
        (0.0, 0.0, -CANOPY_HEIGHT / 2.0)
    )
    inner = cq.Workplane("XY").box(
        CANOPY_WIDTH - 2.0 * CANOPY_WALL,
        CANOPY_DEPTH - 2.0 * CANOPY_WALL,
        CANOPY_HEIGHT - CANOPY_TOP + 0.004,
    ).translate((0.0, 0.0, -(CANOPY_HEIGHT + CANOPY_TOP + 0.004) / 2.0))
    front_opening = cq.Workplane("XY").box(
        CANOPY_FRONT_OPENING_WIDTH,
        0.080,
        CANOPY_FRONT_OPENING_HEIGHT,
    ).translate((0.0, -(CANOPY_DEPTH / 2.0 - 0.040), STRIP_CENTER_Z))
    return outer.cut(inner).cut(front_opening)


def _make_chimney_shape() -> cq.Workplane:
    sleeve = cq.Workplane("XY").box(CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT).translate(
        (0.0, 0.0, CHIMNEY_HEIGHT / 2.0)
    )
    sleeve_void = cq.Workplane("XY").box(
        CHIMNEY_WIDTH - 2.0 * CHIMNEY_WALL,
        CHIMNEY_DEPTH - 2.0 * CHIMNEY_WALL,
        CHIMNEY_HEIGHT + 0.020,
    ).translate((0.0, 0.0, CHIMNEY_HEIGHT / 2.0))
    ceiling_plate = cq.Workplane("XY").box(
        CEILING_PLATE_WIDTH,
        CEILING_PLATE_DEPTH,
        CEILING_PLATE_THICKNESS,
    ).translate((0.0, 0.0, CHIMNEY_HEIGHT + CEILING_PLATE_THICKNESS / 2.0))
    return sleeve.cut(sleeve_void).union(ceiling_plate)


def _make_control_strip_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(STRIP_WIDTH, STRIP_THICKNESS, STRIP_HEIGHT)
    dial_cut = (
        cq.Workplane("XZ")
        .center(DIAL_X, 0.0)
        .circle(0.0068)
        .extrude(0.020, both=True)
    )
    button_cuts = (
        cq.Workplane("XZ")
        .pushPoints([(x_pos, 0.0) for x_pos in BUTTON_XS])
        .slot2D(0.014, 0.010, 0.0)
        .extrude(0.020, both=True)
    )
    return plate.cut(dial_cut).cut(button_cuts)


def _make_button_cap_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.018, 0.004, 0.012).translate((0.0, -0.002, 0.0))
    return cap.edges("|Y").fillet(0.0014)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_range_hood")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    soft_black = model.material("soft_black", rgba=(0.18, 0.18, 0.19, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_cadquery(_make_canopy_shape(), "range_hood_canopy"),
        material=stainless,
        name="shell",
    )

    chimney = model.part("chimney")
    chimney.visual(
        mesh_from_cadquery(_make_chimney_shape(), "range_hood_chimney"),
        material=stainless,
        name="shell",
    )
    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        mesh_from_cadquery(_make_control_strip_shape(), "range_hood_control_strip"),
        material=brushed_dark,
        name="plate",
    )
    model.articulation(
        "canopy_to_control_strip",
        ArticulationType.FIXED,
        parent=canopy,
        child=control_strip,
        origin=Origin(
            xyz=(
                0.0,
                -(CANOPY_DEPTH / 2.0 + STRIP_THICKNESS / 2.0),
                STRIP_CENTER_Z,
            )
        ),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.020,
                body_style="skirted",
                top_diameter=0.030,
                skirt=KnobSkirt(0.040, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=16, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "range_hood_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="knob",
    )
    dial.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="shaft",
    )
    model.articulation(
        "control_strip_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_strip,
        child=dial,
        origin=Origin(xyz=(DIAL_X, -STRIP_THICKNESS / 2.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.5),
    )

    button_cap_mesh = mesh_from_cadquery(_make_button_cap_shape(), "range_hood_button_cap")
    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(button_cap_mesh, material=soft_black, name="cap")
        button.visual(
            Box((0.008, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
            material=soft_black,
            name="stem",
        )
        model.articulation(
            f"control_strip_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(x_pos, -STRIP_THICKNESS / 2.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
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

    canopy = object_model.get_part("canopy")
    chimney = object_model.get_part("chimney")
    control_strip = object_model.get_part("control_strip")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("control_strip_to_dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [
        object_model.get_articulation(f"control_strip_to_button_{index}") for index in range(3)
    ]

    ctx.expect_gap(
        chimney,
        canopy,
        axis="z",
        elem_a="shell",
        elem_b="shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="chimney seats on canopy top",
    )
    ctx.expect_within(
        chimney,
        canopy,
        axes="xy",
        inner_elem="shell",
        outer_elem="shell",
        margin=0.02,
        name="chimney stays within canopy footprint",
    )
    ctx.expect_contact(
        control_strip,
        canopy,
        elem_a="plate",
        elem_b="shell",
        name="control strip mounts to canopy front",
    )

    dial_axis = tuple(float(value) for value in dial_joint.axis)
    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "dial axis faces forward",
        abs(dial_axis[0]) < 1e-9 and abs(abs(dial_axis[1]) - 1.0) < 1e-9 and abs(dial_axis[2]) < 1e-9,
        details=f"axis={dial_axis!r}",
    )
    ctx.expect_overlap(
        dial,
        control_strip,
        axes="xz",
        elem_a="knob",
        elem_b="plate",
        min_overlap=0.020,
        name="dial stays on control strip footprint",
    )
    ctx.expect_gap(
        control_strip,
        dial,
        axis="y",
        positive_elem="plate",
        negative_elem="knob",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial sits against strip front face",
    )

    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        button_axis = tuple(float(value) for value in joint.axis)
        ctx.check(
            f"button_{index} uses inward prismatic travel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and abs(button_axis[0]) < 1e-9
            and abs(button_axis[1] - 1.0) < 1e-9
            and abs(button_axis[2]) < 1e-9,
            details=f"type={joint.articulation_type!r}, axis={button_axis!r}",
        )
        ctx.expect_within(
            button,
            control_strip,
            axes="xz",
            inner_elem="cap",
            outer_elem="plate",
            margin=0.001,
            name=f"button_{index} stays on strip footprint",
        )
        ctx.expect_gap(
            control_strip,
            button,
            axis="y",
            positive_elem="plate",
            negative_elem="cap",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"button_{index} sits proud of strip at rest",
        )

        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} depresses inward",
                rest_pos is not None
                and pressed_pos is not None
                and upper is not None
                and pressed_pos[1] > rest_pos[1] + upper * 0.75,
                details=f"rest={rest_pos!r}, pressed={pressed_pos!r}, upper={upper!r}",
            )
            ctx.expect_within(
                button,
                control_strip,
                axes="xz",
                inner_elem="cap",
                outer_elem="plate",
                margin=0.001,
                name=f"button_{index} stays aligned when pressed",
            )

    middle_rest = ctx.part_world_position(buttons[1])
    button_0_upper = (
        button_joints[0].motion_limits.upper if button_joints[0].motion_limits is not None else 0.0
    )
    with ctx.pose({button_joints[0]: button_0_upper}):
        middle_during_other_press = ctx.part_world_position(buttons[1])
    ctx.check(
        "buttons depress independently",
        middle_rest is not None
        and middle_during_other_press is not None
        and abs(middle_rest[0] - middle_during_other_press[0]) < 1e-9
        and abs(middle_rest[1] - middle_during_other_press[1]) < 1e-9
        and abs(middle_rest[2] - middle_during_other_press[2]) < 1e-9,
        details=f"rest={middle_rest!r}, during_other_press={middle_during_other_press!r}",
    )

    return ctx.report()


object_model = build_object_model()

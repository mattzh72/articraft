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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.76
BODY_DEPTH = 0.50
BODY_HEIGHT = 0.14
SHELL_WALL = 0.015

CONTROL_Z = -0.004
KNOB_X = -0.285
BUTTON_0_X = 0.248
BUTTON_1_X = 0.292

FILTER_WIDTH = 0.70
FILTER_DEPTH = 0.34
FILTER_CENTER_Y = 0.030
FILTER_CENTER_Z = -BODY_HEIGHT / 2.0 + 0.0025


def _build_hood_shell() -> object:
    hood = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .edges()
        .fillet(0.003)
        .faces("<Z")
        .shell(-SHELL_WALL)
    )

    front_cut_depth = SHELL_WALL + 0.012
    front_start_y = -BODY_DEPTH / 2.0 - 0.002

    knob_hole = cq.Solid.makeBox(0.0115, front_cut_depth, 0.0115).translate(
        (KNOB_X - 0.00575, front_start_y, CONTROL_Z - 0.00575)
    )
    button_0_hole = cq.Solid.makeBox(0.0175, front_cut_depth, 0.009).translate(
        (BUTTON_0_X - 0.00875, front_start_y, CONTROL_Z - 0.0045)
    )
    button_1_hole = cq.Solid.makeBox(0.0175, front_cut_depth, 0.009).translate(
        (BUTTON_1_X - 0.00875, front_start_y, CONTROL_Z - 0.0045)
    )

    return hood.cut(knob_hole).cut(button_0_hole).cut(button_1_hole)


def _build_button_shape(*, width: float, height: float, cap_depth: float) -> object:
    return (
        cq.Workplane("XY")
        .box(width, cap_depth, height)
        .translate((0.0, -cap_depth / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.0012)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood")

    body_finish = model.material("body_finish", rgba=(0.80, 0.81, 0.82, 1.0))
    rail_finish = model.material("rail_finish", rgba=(0.46, 0.47, 0.49, 1.0))
    filter_finish = model.material("filter_finish", rgba=(0.34, 0.35, 0.37, 1.0))
    control_finish = model.material("control_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    button_finish = model.material("button_finish", rgba=(0.90, 0.91, 0.92, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_hood_shell(), "range_hood_shell"),
        material=body_finish,
        name="hood_shell",
    )

    rail_z = -BODY_HEIGHT / 2.0 + 0.004
    rail_height = 0.008
    rail_thickness = 0.015
    side_rail_y = FILTER_CENTER_Y
    front_rail_y = FILTER_CENTER_Y - FILTER_DEPTH / 2.0 - rail_thickness / 2.0
    rear_rail_y = FILTER_CENTER_Y + FILTER_DEPTH / 2.0 + rail_thickness / 2.0
    side_rail_x = FILTER_WIDTH / 2.0 + rail_thickness / 2.0

    body.visual(
        Box((rail_thickness, FILTER_DEPTH + 2.0 * rail_thickness, rail_height)),
        origin=Origin(xyz=(-side_rail_x, side_rail_y, rail_z)),
        material=rail_finish,
        name="filter_rail_0",
    )
    body.visual(
        Box((rail_thickness, FILTER_DEPTH + 2.0 * rail_thickness, rail_height)),
        origin=Origin(xyz=(side_rail_x, side_rail_y, rail_z)),
        material=rail_finish,
        name="filter_rail_1",
    )
    body.visual(
        Box((FILTER_WIDTH, rail_thickness, rail_height)),
        origin=Origin(xyz=(0.0, front_rail_y, rail_z)),
        material=rail_finish,
        name="front_filter_rail",
    )
    body.visual(
        Box((FILTER_WIDTH, rail_thickness, rail_height)),
        origin=Origin(xyz=(0.0, rear_rail_y, rail_z)),
        material=rail_finish,
        name="rear_filter_rail",
    )

    filter_part = model.part("filter")
    filter_part.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (FILTER_WIDTH, FILTER_DEPTH),
                0.005,
                slot_size=(0.034, 0.0048),
                pitch=(0.043, 0.015),
                frame=0.012,
                corner_radius=0.004,
                slot_angle_deg=9.0,
                stagger=True,
            ),
            "range_hood_filter",
        ),
        material=filter_finish,
        name="grease_filter",
    )
    model.articulation(
        "body_to_filter",
        ArticulationType.FIXED,
        parent=body,
        child=filter_part,
        origin=Origin(xyz=(0.0, FILTER_CENTER_Y, FILTER_CENTER_Z)),
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.0048, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="collar",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.023,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.054, 0.0055, flare=0.10),
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "range_hood_selector_knob",
        ),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="knob_cap",
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(KNOB_X, -BODY_DEPTH / 2.0, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    button_shape = mesh_from_cadquery(
        _build_button_shape(width=0.022, height=0.0105, cap_depth=0.003),
        "range_hood_button",
    )

    for index, button_x in enumerate((BUTTON_0_X, BUTTON_1_X)):
        button = model.part(f"button_{index}")
        button.visual(button_shape, material=button_finish, name="button")
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, -BODY_DEPTH / 2.0, CONTROL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    filter_part = object_model.get_part("filter")
    knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    knob_joint = object_model.get_articulation("body_to_selector_knob")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Body should produce world bounds.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "range_hood_width",
            0.72 <= size[0] <= 0.80,
            f"expected cabinet-width hood silhouette, got size={size!r}",
        )
        ctx.check(
            "range_hood_depth",
            0.45 <= size[1] <= 0.54,
            f"expected shallow under-cabinet depth, got size={size!r}",
        )
        ctx.check(
            "range_hood_height",
            0.12 <= size[2] <= 0.16,
            f"expected shallow appliance height, got size={size!r}",
        )

    ctx.expect_overlap(
        filter_part,
        body,
        axes="xy",
        elem_a="grease_filter",
        min_overlap=0.28,
        name="filter spans the underside opening footprint",
    )

    ctx.check(
        "selector_joint_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"expected continuous selector knob, got {knob_joint.articulation_type!r}",
    )
    ctx.check(
        "button_joints_are_prismatic",
        button_0_joint.articulation_type == ArticulationType.PRISMATIC
        and button_1_joint.articulation_type == ArticulationType.PRISMATIC,
        f"button joints are {button_0_joint.articulation_type!r} and {button_1_joint.articulation_type!r}",
    )

    knob_rest = ctx.part_world_position(knob)
    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    ctx.check(
        "controls_present_on_front_strip",
        knob_rest is not None
        and button_0_rest is not None
        and button_1_rest is not None
        and knob_rest[0] < -0.18
        and button_0_rest[0] > 0.16
        and button_1_rest[0] > button_0_rest[0] + 0.02,
        f"knob={knob_rest}, button_0={button_0_rest}, button_1={button_1_rest}",
    )

    with ctx.pose({button_0_joint: button_0_joint.motion_limits.upper}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_unchanged = ctx.part_world_position(button_1)
    ctx.check(
        "button_0_depresses_inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[1] > button_0_rest[1] + 0.003,
        f"rest={button_0_rest}, pressed={button_0_pressed}",
    )
    ctx.check(
        "button_0_motion_is_independent",
        button_1_rest is not None
        and button_1_unchanged is not None
        and abs(button_1_rest[1] - button_1_unchanged[1]) < 1e-6,
        f"button_1 rest={button_1_rest}, while button_0 pressed={button_1_unchanged}",
    )

    with ctx.pose({button_1_joint: button_1_joint.motion_limits.upper}):
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button_1_depresses_inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[1] > button_1_rest[1] + 0.003,
        f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    with ctx.pose({knob_joint: math.pi}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "selector_rotates_without_translating",
        knob_rest is not None
        and knob_rotated is not None
        and max(abs(knob_rotated[i] - knob_rest[i]) for i in range(3)) < 1e-6,
        f"rest={knob_rest}, rotated={knob_rotated}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

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
    mesh_from_geometry,
    place_on_face,
)


CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.12
PANEL_WIDTH = 0.52
PANEL_DEPTH = 0.022
PANEL_HEIGHT = 0.09


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    filter_black = model.material("filter_black", rgba=(0.21, 0.22, 0.23, 1.0))
    button_satin = model.material("button_satin", rgba=(0.66, 0.68, 0.70, 1.0))
    light_glass = model.material("light_glass", rgba=(0.92, 0.94, 0.98, 0.45))

    hood = model.part("hood")
    hood.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT * 0.5)),
        material=stainless,
        name="canopy",
    )
    hood.visual(
        Box((0.46, 0.30, 0.10)),
        origin=Origin(xyz=(0.0, -0.08, 0.17)),
        material=stainless,
        name="motor_cowl",
    )
    hood.visual(
        Box((0.32, 0.24, 0.46)),
        origin=Origin(xyz=(0.0, -0.11, 0.35)),
        material=stainless,
        name="stack_lower",
    )
    hood.visual(
        Box((0.28, 0.20, 0.36)),
        origin=Origin(xyz=(0.0, -0.11, 0.72)),
        material=stainless,
        name="stack_upper",
    )
    hood.visual(
        Box((0.32, 0.22, 0.006)),
        origin=Origin(xyz=(-0.18, -0.03, 0.003)),
        material=filter_black,
        name="filter_0",
    )
    hood.visual(
        Box((0.32, 0.22, 0.006)),
        origin=Origin(xyz=(0.18, -0.03, 0.003)),
        material=filter_black,
        name="filter_1",
    )
    hood.visual(
        Box((0.12, 0.07, 0.004)),
        origin=Origin(xyz=(0.0, 0.08, 0.002)),
        material=light_glass,
        name="lamp_lens",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_DEPTH * 0.5, 0.0)),
        material=panel_dark,
        name="panel_shell",
    )
    panel.visual(
        Box((PANEL_WIDTH, 0.003, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_DEPTH - 0.0015, 0.0)),
        material=stainless,
        name="panel_trim",
    )
    model.articulation(
        "hood_to_panel",
        ArticulationType.FIXED,
        parent=hood,
        child=panel,
        origin=Origin(xyz=(0.0, CANOPY_DEPTH * 0.5, 0.072)),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.028,
            body_style="skirted",
            top_diameter=0.044,
            skirt=KnobSkirt(0.060, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "hood_knob",
    )

    for index, x_pos in enumerate((-0.115, 0.115)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            material=control_black,
            name="knob_shell",
        )
        model.articulation(
            f"panel_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=panel,
            child=knob,
            origin=place_on_face(panel, "+y", face_pos=(x_pos, 0.018)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=6.0),
        )

    for index, x_pos in enumerate((-0.085, 0.0, 0.085)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.056, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=button_satin,
            name="button_cap",
        )
        model.articulation(
            f"panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=place_on_face(panel, "+y", face_pos=(x_pos, -0.018)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood = object_model.get_part("hood")
    panel = object_model.get_part("panel")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(2)]
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    knob_joints = [object_model.get_articulation(f"panel_to_knob_{index}") for index in range(2)]
    button_joints = [object_model.get_articulation(f"panel_to_button_{index}") for index in range(3)]

    ctx.expect_contact(panel, hood, name="panel_is_mounted_to_hood")
    for index, knob in enumerate(knobs):
        ctx.expect_contact(knob, panel, name=f"knob_{index}_is_mounted_to_panel")
    for index, button in enumerate(buttons):
        ctx.expect_contact(button, panel, name=f"button_{index}_is_mounted_to_panel")

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    button_positions = [ctx.part_world_position(button) for button in buttons]
    controls_stacked = (
        all(position is not None for position in knob_positions + button_positions)
        and min(position[2] for position in knob_positions if position is not None)
        > max(position[2] for position in button_positions if position is not None) + 0.02
    )
    ctx.check(
        "knob_row_above_button_row",
        controls_stacked,
        details=f"knobs={knob_positions!r}, buttons={button_positions!r}",
    )

    for joint in knob_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type!r}",
        )
        ctx.check(
            f"{joint.name}_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits!r}",
        )

    for joint in button_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type!r}",
        )
        ctx.check(
            f"{joint.name}_travel_is_small",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and 0.003 <= limits.upper <= 0.005,
            details=f"limits={limits!r}",
        )

    middle_button = buttons[1]
    middle_button_joint = button_joints[1]
    rest_position = ctx.part_world_position(middle_button)
    upper = middle_button_joint.motion_limits.upper if middle_button_joint.motion_limits is not None else None
    with ctx.pose({middle_button_joint: upper if upper is not None else 0.0}):
        extended_position = ctx.part_world_position(middle_button)
    ctx.check(
        "middle_button_moves_forward",
        rest_position is not None
        and extended_position is not None
        and upper is not None
        and extended_position[1] > rest_position[1] + 0.003,
        details=f"rest={rest_position!r}, extended={extended_position!r}, upper={upper!r}",
    )

    return ctx.report()


object_model = build_object_model()

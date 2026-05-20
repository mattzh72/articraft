from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CANOPY_DEPTH = 0.50
CANOPY_WIDTH = 0.90
CANOPY_HEIGHT = 0.08
SHELL_THICKNESS = 0.018

PANEL_WIDTH = 0.86
PANEL_THICKNESS = 0.014
PANEL_FACE_HEIGHT = 0.110
PANEL_GRIP_HEIGHT = 0.024
PANEL_OPEN_ANGLE = 1.25

BUTTON_SIZE = 0.018
BUTTON_TRAVEL = 0.006
BUTTON_Y_OFFSETS = (-0.0525, -0.0175, 0.0175, 0.0525)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_hood")

    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        Box((CANOPY_DEPTH, CANOPY_WIDTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - SHELL_THICKNESS / 2.0)),
        material=steel,
        name="top_plate",
    )
    for index, side_y in enumerate(
        (
            -(CANOPY_WIDTH / 2.0 - SHELL_THICKNESS / 2.0),
            CANOPY_WIDTH / 2.0 - SHELL_THICKNESS / 2.0,
        )
    ):
        body.visual(
            Box((CANOPY_DEPTH, SHELL_THICKNESS, CANOPY_HEIGHT - SHELL_THICKNESS)),
            origin=Origin(xyz=(0.0, side_y, (CANOPY_HEIGHT - SHELL_THICKNESS) / 2.0)),
            material=steel,
            name=f"side_wall_{index}",
        )
    body.visual(
        Box((SHELL_THICKNESS, CANOPY_WIDTH - 2.0 * SHELL_THICKNESS, CANOPY_HEIGHT - SHELL_THICKNESS)),
        origin=Origin(
            xyz=(
                -(CANOPY_DEPTH / 2.0 - SHELL_THICKNESS / 2.0),
                0.0,
                (CANOPY_HEIGHT - SHELL_THICKNESS) / 2.0,
            )
        ),
        material=steel,
        name="rear_wall",
    )
    body.visual(
        Box((0.036, CANOPY_WIDTH, 0.050)),
        origin=Origin(xyz=(CANOPY_DEPTH / 2.0 - 0.018, 0.0, 0.055)),
        material=steel,
        name="front_lip",
    )
    body.visual(
        Box((0.018, 0.180, 0.018)),
        origin=Origin(xyz=(0.205, 0.0, 0.021)),
        material=dark_steel,
        name="button_rail",
    )
    body.visual(
        Box((0.260, 0.300, 0.420)),
        origin=Origin(xyz=(-0.050, 0.0, CANOPY_HEIGHT + 0.210)),
        material=steel,
        name="lower_flue",
    )
    body.visual(
        Box((0.220, 0.260, 0.340)),
        origin=Origin(xyz=(-0.050, 0.0, CANOPY_HEIGHT + 0.420 + 0.170)),
        material=steel,
        name="upper_flue",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_THICKNESS, PANEL_WIDTH, PANEL_FACE_HEIGHT)),
        origin=Origin(xyz=(PANEL_THICKNESS / 2.0, 0.0, -PANEL_FACE_HEIGHT / 2.0)),
        material=steel,
        name="panel_face",
    )
    panel.visual(
        Box((0.020, PANEL_WIDTH, PANEL_GRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.010,
                0.0,
                -(PANEL_FACE_HEIGHT + PANEL_GRIP_HEIGHT / 2.0 - 0.001),
            )
        ),
        material=dark_steel,
        name="panel_grip",
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(CANOPY_DEPTH / 2.0, 0.0, CANOPY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=0.0,
            upper=PANEL_OPEN_ANGLE,
        ),
    )

    for index, button_y in enumerate(BUTTON_Y_OFFSETS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_SIZE, BUTTON_SIZE, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=charcoal,
            name="cap",
        )

        model.articulation(
            f"button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.223, button_y, 0.024)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    panel = object_model.get_part("panel")
    panel_hinge = object_model.get_articulation("panel_hinge")

    with ctx.pose({panel_hinge: 0.0}):
        ctx.expect_gap(
            panel,
            body,
            axis="x",
            positive_elem="panel_face",
            negative_elem="front_lip",
            max_gap=0.001,
            max_penetration=0.0,
            name="panel face closes flush to the front lip",
        )
        ctx.expect_overlap(
            panel,
            body,
            axes="y",
            elem_a="panel_face",
            elem_b="front_lip",
            min_overlap=0.82,
            name="panel stays centered across the hood width",
        )

    closed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: PANEL_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "panel swings outward when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.08,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    buttons = [object_model.get_part(f"button_{index}") for index in range(4)]
    rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, button in enumerate(buttons):
        button_joint = object_model.get_articulation(f"button_{index}_slide")
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_positions = [ctx.part_world_position(candidate) for candidate in buttons]
        moved_button = (
            rest_positions[index] is not None
            and pressed_positions[index] is not None
            and pressed_positions[index][2] > rest_positions[index][2] + BUTTON_TRAVEL * 0.9
        )
        other_buttons_static = all(
            rest_positions[other_index] is not None
            and pressed_positions[other_index] is not None
            and abs(pressed_positions[other_index][2] - rest_positions[other_index][2]) < 1e-6
            for other_index in range(4)
            if other_index != index
        )
        ctx.check(
            f"button_{index} presses independently",
            moved_button and other_buttons_static,
            details=(
                f"rest_positions={rest_positions}, "
                f"pressed_positions={pressed_positions}"
            ),
        )

    return ctx.report()


object_model = build_object_model()

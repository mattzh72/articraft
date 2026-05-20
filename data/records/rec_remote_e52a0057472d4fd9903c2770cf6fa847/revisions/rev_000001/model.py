from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    place_on_face,
)

BODY_WIDTH = 0.052
BODY_LENGTH = 0.165
BODY_TOP_THICKNESS = 0.010
BODY_BOTTOM_THICKNESS = 0.005
BODY_TOTAL_THICKNESS = 0.014
BODY_BACK_Z = -BODY_TOTAL_THICKNESS / 2.0
BODY_TOP_Z = BODY_TOTAL_THICKNESS / 2.0

FACEPLATE_WIDTH = 0.043
FACEPLATE_LENGTH = 0.134
FACEPLATE_THICKNESS = 0.0018

DISPLAY_SIZE = (0.027, 0.018, 0.0014)
DISPLAY_Y = 0.039

BUTTON_TRAVEL = 0.0014
BUTTON_BASE_SIZE = (0.0128, 0.0090, 0.0010)
BUTTON_CAP_SIZE = (0.0114, 0.0076, 0.0013)
BUTTON_STEM_SIZE = (0.0092, 0.0060, FACEPLATE_THICKNESS + BUTTON_TRAVEL + BUTTON_BASE_SIZE[2])
BUTTON_RETAINER_SIZE = (0.0104, 0.0070, 0.0008)
BUTTON_POSITIONS = (
    (-0.00775, 0.0105),
    (0.00775, 0.0105),
    (-0.00775, -0.0095),
    (0.00775, -0.0095),
    (-0.00775, -0.0305),
    (0.00775, -0.0305),
)

STAND_WIDTH = 0.030
STAND_LENGTH = 0.092
STAND_THICKNESS = 0.0030
STAND_BARREL_RADIUS = 0.0025
STAND_BARREL_LENGTH = 0.028
STAND_HINGE_Y = -0.074
STAND_OPEN_ANGLE = math.radians(68.0)


def _add_button(
    model: ArticulatedObject,
    faceplate,
    button_material,
    cap_material,
    *,
    index: int,
    face_pos: tuple[float, float],
):
    button = model.part(f"button_{index}")
    button.visual(
        Box(BUTTON_BASE_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BUTTON_TRAVEL + BUTTON_BASE_SIZE[2] / 2.0,
            )
        ),
        material=button_material,
        name="base",
    )
    button.visual(
        Box(BUTTON_STEM_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -FACEPLATE_THICKNESS + BUTTON_STEM_SIZE[2] / 2.0,
            )
        ),
        material=button_material,
        name="stem",
    )
    button.visual(
        Box(BUTTON_RETAINER_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -FACEPLATE_THICKNESS - BUTTON_RETAINER_SIZE[2] / 2.0,
            )
        ),
        material=button_material,
        name="retainer",
    )
    button.visual(
        Box(BUTTON_CAP_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BUTTON_TRAVEL + BUTTON_BASE_SIZE[2] + BUTTON_CAP_SIZE[2] / 2.0,
            )
        ),
        material=cap_material,
        name="cap",
    )

    model.articulation(
        f"faceplate_to_button_{index}",
        ArticulationType.PRISMATIC,
        parent=faceplate,
        child=button,
        origin=Origin(xyz=(face_pos[0], face_pos[1], FACEPLATE_THICKNESS)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_automation_remote")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    faceplate_graphite = model.material("faceplate_graphite", rgba=(0.21, 0.23, 0.26, 1.0))
    button_grey = model.material("button_grey", rgba=(0.73, 0.75, 0.78, 1.0))
    button_cap = model.material("button_cap", rgba=(0.84, 0.85, 0.87, 1.0))
    display_glass = model.material("display_glass", rgba=(0.14, 0.20, 0.24, 0.65))
    stand_grey = model.material("stand_grey", rgba=(0.76, 0.78, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_LENGTH, BODY_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=body_white,
        name="shell",
    )
    body.visual(
        Box((0.048, 0.159, BODY_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=body_white,
        name="base",
    )
    body.visual(
        Box((0.036, 0.010, 0.003)),
        origin=Origin(
            xyz=(
                0.0,
                STAND_HINGE_Y,
                BODY_BACK_Z - 0.0015,
            )
        ),
        material=body_white,
        name="stand_mount",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Box((0.008, FACEPLATE_LENGTH, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(-0.0175, 0.0, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="left_rail",
    )
    faceplate.visual(
        Box((0.008, FACEPLATE_LENGTH, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0175, 0.0, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="right_rail",
    )
    faceplate.visual(
        Box((0.027, 0.012, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.061, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="top_rail",
    )
    faceplate.visual(
        Box((0.027, 0.007, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.022, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="display_rail",
    )
    faceplate.visual(
        Box((0.004, 0.077, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.0165, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="center_rib",
    )
    faceplate.visual(
        Box((0.027, 0.006, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.000, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="separator_0",
    )
    faceplate.visual(
        Box((0.027, 0.006, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.019, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="separator_1",
    )
    faceplate.visual(
        Box((0.027, 0.016, FACEPLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FACEPLATE_THICKNESS / 2.0)),
        material=faceplate_graphite,
        name="bottom_rail",
    )
    model.articulation(
        "body_to_faceplate",
        ArticulationType.FIXED,
        parent=body,
        child=faceplate,
        origin=place_on_face(body, "+z"),
    )

    display = model.part("display")
    display.visual(
        Box(DISPLAY_SIZE),
        origin=Origin(xyz=(0.0, 0.0, DISPLAY_SIZE[2] / 2.0)),
        material=display_glass,
        name="window",
    )
    model.articulation(
        "faceplate_to_display",
        ArticulationType.FIXED,
        parent=faceplate,
        child=display,
        origin=Origin(xyz=(0.0, DISPLAY_Y, FACEPLATE_THICKNESS)),
    )

    for index, face_pos in enumerate(BUTTON_POSITIONS):
        _add_button(
            model,
            faceplate,
            button_grey,
            button_cap,
            index=index,
            face_pos=face_pos,
        )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=STAND_BARREL_RADIUS, length=STAND_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_grey,
        name="barrel",
    )
    stand.visual(
        Box((STAND_WIDTH, STAND_LENGTH, STAND_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                STAND_LENGTH / 2.0,
                STAND_BARREL_RADIUS - STAND_THICKNESS / 2.0,
            )
        ),
        material=stand_grey,
        name="panel",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(
            xyz=(
                0.0,
                STAND_HINGE_Y,
                BODY_BACK_Z - STAND_BARREL_RADIUS,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=STAND_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    faceplate = object_model.get_part("faceplate")
    display = object_model.get_part("display")
    stand = object_model.get_part("stand")
    stand_hinge = object_model.get_articulation("body_to_stand")

    buttons = [object_model.get_part(f"button_{index}") for index in range(6)]
    button_joints = [
        object_model.get_articulation(f"faceplate_to_button_{index}")
        for index in range(6)
    ]

    ctx.expect_origin_gap(
        display,
        buttons[0],
        axis="y",
        min_gap=0.020,
        name="display sits above the button bank",
    )

    for index, button in enumerate(buttons):
        ctx.expect_overlap(
            button,
            faceplate,
            axes="xy",
            min_overlap=0.007,
            elem_a="cap",
            name=f"button_{index} stays within the faceplate footprint",
        )

    ctx.expect_gap(
        body,
        stand,
        axis="z",
        positive_elem="base",
        negative_elem="panel",
        max_gap=0.0002,
        max_penetration=0.0006,
        name="stand nests against the rear body",
    )

    button_rest = ctx.part_element_world_aabb(buttons[0], elem="cap")
    neighbor_rest = ctx.part_element_world_aabb(buttons[1], elem="cap")
    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        button_pressed = ctx.part_element_world_aabb(buttons[0], elem="cap")
        neighbor_pressed = ctx.part_element_world_aabb(buttons[1], elem="cap")

    button_rest_top = button_rest[1][2] if button_rest is not None else None
    button_pressed_top = button_pressed[1][2] if button_pressed is not None else None
    neighbor_rest_top = neighbor_rest[1][2] if neighbor_rest is not None else None
    neighbor_pressed_top = neighbor_pressed[1][2] if neighbor_pressed is not None else None
    ctx.check(
        "buttons depress independently",
        (
            button_rest_top is not None
            and button_pressed_top is not None
            and neighbor_rest_top is not None
            and neighbor_pressed_top is not None
            and button_pressed_top < button_rest_top - 0.0010
            and abs(neighbor_pressed_top - neighbor_rest_top) <= 1e-6
        ),
        details=(
            f"button_0 rest_top={button_rest_top}, "
            f"button_0 pressed_top={button_pressed_top}, "
            f"button_1 rest_top={neighbor_rest_top}, "
            f"button_1 pressed_top={neighbor_pressed_top}"
        ),
    )

    stand_rest = ctx.part_element_world_aabb(stand, elem="panel")
    with ctx.pose({stand_hinge: STAND_OPEN_ANGLE}):
        stand_open = ctx.part_element_world_aabb(stand, elem="panel")

    stand_rest_min_z = stand_rest[0][2] if stand_rest is not None else None
    stand_open_min_z = stand_open[0][2] if stand_open is not None else None
    ctx.check(
        "stand rotates outward below the body",
        (
            stand_rest_min_z is not None
            and stand_open_min_z is not None
            and stand_open_min_z < stand_rest_min_z - 0.020
        ),
        details=f"rest_min_z={stand_rest_min_z}, open_min_z={stand_open_min_z}",
    )

    return ctx.report()


object_model = build_object_model()

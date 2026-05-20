from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)

COOKTOP_WIDTH = 0.91
COOKTOP_DEPTH = 0.52
GLASS_THICKNESS = 0.006

PAN_WIDTH = 0.86
PAN_DEPTH = 0.39
PAN_HEIGHT = 0.040
PAN_CENTER_Y = 0.045

FRONT_RAIL_DEPTH = 0.024
FRONT_RAIL_HEIGHT = 0.024
FRONT_RAIL_CENTER_Y = (-COOKTOP_DEPTH * 0.5) + (FRONT_RAIL_DEPTH * 0.5)

BUTTON_COUNT = 6
BUTTON_PITCH = 0.034
BUTTON_CENTER_Y = -0.223

BUTTON_CAP_WIDTH = 0.020
BUTTON_CAP_DEPTH = 0.010
BUTTON_CAP_RADIUS = 0.0028
BUTTON_CAP_THICKNESS = 0.0020
BUTTON_CAP_TOP_REST = 0.0030

BUTTON_STEM_WIDTH = 0.0118
BUTTON_STEM_DEPTH = 0.0052
BUTTON_STEM_LENGTH = 0.015

BUTTON_GUIDE_WIDTH = 0.0168
BUTTON_GUIDE_DEPTH = 0.0082
BUTTON_GUIDE_THICKNESS = 0.0012

BUTTON_OPENING_WIDTH = 0.0134
BUTTON_OPENING_DEPTH = 0.0068
BUTTON_OPENING_RADIUS = 0.0018
BUTTON_TRAVEL = 0.0008


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _button_centers_x() -> list[float]:
    half_span = (BUTTON_COUNT - 1) * 0.5
    return [(index - half_span) * BUTTON_PITCH for index in range(BUTTON_COUNT)]


def _build_glass_mesh():
    glass = (
        cq.Workplane("XY")
        .box(
            COOKTOP_WIDTH,
            COOKTOP_DEPTH,
            GLASS_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -GLASS_THICKNESS))
        .edges("|Z")
        .fillet(0.018)
    )

    for center_x in _button_centers_x():
        cutter = (
            cq.Workplane("XY")
            .box(
                BUTTON_OPENING_WIDTH,
                BUTTON_OPENING_DEPTH,
                GLASS_THICKNESS * 3.0,
                centered=(True, True, True),
            )
            .translate((center_x, BUTTON_CENTER_Y, -GLASS_THICKNESS * 0.5))
        )
        glass = glass.cut(cutter)

    return glass


def _build_button_cap_mesh():
    cap = ExtrudeGeometry.from_z0(
        rounded_rect_profile(BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_RADIUS),
        BUTTON_CAP_THICKNESS,
        cap=True,
        closed=True,
    )
    cap.translate(0.0, 0.0, -BUTTON_CAP_THICKNESS)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_stovetop")

    glass_black = model.material("glass_black", rgba=(0.07, 0.08, 0.09, 1.0))
    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.23, 0.23, 0.24, 1.0))
    button_finish = model.material("button_finish", rgba=(0.30, 0.31, 0.33, 1.0))

    glass_mesh = mesh_from_cadquery(_build_glass_mesh(), "induction_glass_top")
    button_cap_mesh = mesh_from_geometry(_build_button_cap_mesh(), "induction_button_cap")

    cooktop = model.part("cooktop")
    cooktop.visual(glass_mesh, material=glass_black, name="glass_top")
    cooktop.visual(
        Box((PAN_WIDTH, PAN_DEPTH, PAN_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                PAN_CENTER_Y,
                -GLASS_THICKNESS - (PAN_HEIGHT * 0.5),
            )
        ),
        material=body_black,
        name="underside_pan",
    )
    cooktop.visual(
        Box((COOKTOP_WIDTH * 0.98, FRONT_RAIL_DEPTH, FRONT_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_RAIL_CENTER_Y,
                -GLASS_THICKNESS - (FRONT_RAIL_HEIGHT * 0.5),
            )
        ),
        material=trim_dark,
        name="front_rail",
    )

    for index, center_x in enumerate(_button_centers_x()):
        button = model.part(f"button_{index}")
        button.visual(button_cap_mesh, material=button_finish, name="cap")
        button.visual(
            Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_LENGTH)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    -BUTTON_CAP_THICKNESS - (BUTTON_STEM_LENGTH * 0.5),
                )
            ),
            material=button_finish,
            name="stem",
        )
        button.visual(
            Box((BUTTON_GUIDE_WIDTH, BUTTON_GUIDE_DEPTH, BUTTON_GUIDE_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    -(
                        BUTTON_CAP_TOP_REST
                        + GLASS_THICKNESS
                        + (BUTTON_GUIDE_THICKNESS * 0.5)
                    ),
                )
            ),
            material=button_finish,
            name="guide",
        )

        model.articulation(
            f"cooktop_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(center_x, BUTTON_CENTER_Y, BUTTON_CAP_TOP_REST)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cooktop = object_model.get_part("cooktop")
    buttons = [object_model.get_part(f"button_{index}") for index in range(BUTTON_COUNT)]
    button_joints = [
        object_model.get_articulation(f"cooktop_to_button_{index}")
        for index in range(BUTTON_COUNT)
    ]

    aabb = ctx.part_world_aabb(cooktop)
    ctx.check("cooktop_aabb_present", aabb is not None, "Cooktop AABB should resolve.")
    if aabb is not None:
        mins, maxs = aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("cooktop_width_scale", 0.89 <= size[0] <= 0.93, f"size={size!r}")
        ctx.check("cooktop_depth_scale", 0.50 <= size[1] <= 0.54, f"size={size!r}")
        ctx.check("cooktop_thickness_scale", 0.045 <= size[2] <= 0.055, f"size={size!r}")

    button_positions = [ctx.part_world_position(button) for button in buttons]
    ctx.check(
        "button_positions_present",
        all(position is not None for position in button_positions),
        f"positions={button_positions!r}",
    )
    if all(position is not None for position in button_positions):
        x_positions = [float(position[0]) for position in button_positions if position is not None]
        y_positions = [float(position[1]) for position in button_positions if position is not None]
        spacing = [x_positions[index + 1] - x_positions[index] for index in range(len(x_positions) - 1)]

        ctx.check(
            "button_row_centered",
            abs(sum(x_positions) / len(x_positions)) <= 0.002,
            f"x_positions={x_positions!r}",
        )
        ctx.check(
            "button_row_uniform_spacing",
            max(spacing) - min(spacing) <= 0.001,
            f"spacing={spacing!r}",
        )
        ctx.check(
            "button_row_front_edge",
            all(y <= -0.20 for y in y_positions),
            f"y_positions={y_positions!r}",
        )

    for index, button in enumerate(buttons):
        ctx.expect_contact(
            button,
            cooktop,
            elem_a="guide",
            elem_b="glass_top",
            name=f"button_{index}_guide_contacts_glass_underside",
        )
        ctx.expect_gap(
            button,
            cooktop,
            axis="z",
            positive_elem="cap",
            negative_elem="glass_top",
            min_gap=0.0009,
            max_gap=0.0011,
            name=f"button_{index}_rests_proud_of_glass",
        )

    rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, joint in enumerate(button_joints):
        with ctx.pose({joint: BUTTON_TRAVEL}):
            posed_positions = [ctx.part_world_position(button) for button in buttons]
            ctx.expect_gap(
                buttons[index],
                cooktop,
                axis="z",
                positive_elem="cap",
                negative_elem="glass_top",
                min_gap=0.0001,
                max_gap=0.00035,
                name=f"button_{index}_pressed_clearance",
            )

        moved_down = (
            rest_positions[index] is not None
            and posed_positions[index] is not None
            and posed_positions[index][2] < rest_positions[index][2] - 0.0006
        )
        others_static = all(
            rest_positions[other_index] is not None
            and posed_positions[other_index] is not None
            and abs(posed_positions[other_index][2] - rest_positions[other_index][2]) <= 1e-7
            for other_index in range(BUTTON_COUNT)
            if other_index != index
        )
        ctx.check(
            f"button_{index}_moves_independently",
            moved_down and others_static,
            details=f"rest={rest_positions!r}, posed={posed_positions!r}",
        )

    return ctx.report()


object_model = build_object_model()

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
    TrunnionYokeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


DISPLAY_WIDTH = 0.84
DISPLAY_HEIGHT = 0.41
DISPLAY_EDGE_DEPTH = 0.052
DISPLAY_CENTER_DEPTH = 0.078
DISPLAY_BASE_Y = 0.048
DISPLAY_CURVE_SAG = 0.018

SCREEN_WIDTH = 0.78
SCREEN_HEIGHT = 0.335
SCREEN_DEPTH = 0.0042


def _blend(edge_value: float, center_value: float, normalized: float, *, power: float = 1.5) -> float:
    t = 1.0 - normalized**power
    return edge_value + (center_value - edge_value) * t


def _shell_depth(x: float) -> float:
    half_width = DISPLAY_WIDTH * 0.5
    normalized = min(abs(x) / half_width, 1.0)
    return _blend(DISPLAY_EDGE_DEPTH, DISPLAY_CENTER_DEPTH, normalized, power=1.25)


def _shell_center_y(x: float) -> float:
    half_width = DISPLAY_WIDTH * 0.5
    normalized = min(abs(x) / half_width, 1.0)
    return DISPLAY_BASE_Y + DISPLAY_CURVE_SAG * (1.0 - normalized * normalized)


def _yz_rounded_section(
    x: float,
    *,
    center_y: float,
    depth: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        depth,
        height,
        min(radius, depth * 0.45, height * 0.2),
        corner_segments=8,
    )
    return [(x, center_y + y, z) for y, z in profile]


def _curved_shell_mesh() -> object:
    half_width = DISPLAY_WIDTH * 0.5
    x_sections = (
        -half_width,
        -0.30,
        -0.18,
        -0.06,
        0.06,
        0.18,
        0.30,
        half_width,
    )
    sections = [
        _yz_rounded_section(
            x,
            center_y=_shell_center_y(x),
            depth=_shell_depth(x),
            height=DISPLAY_HEIGHT,
            radius=0.028,
        )
        for x in x_sections
    ]
    return mesh_from_geometry(section_loft(sections), "screen_shell")


def _screen_panel_mesh() -> object:
    half_width = SCREEN_WIDTH * 0.5
    shell_half_width = DISPLAY_WIDTH * 0.5
    x_sections = (
        -half_width,
        -0.27,
        -0.15,
        -0.05,
        0.05,
        0.15,
        0.27,
        half_width,
    )
    sections = []
    for x in x_sections:
        shell_x = x * (shell_half_width / half_width)
        screen_center_y = _shell_center_y(shell_x) + (_shell_depth(shell_x) * 0.5) - 0.0026
        sections.append(
            _yz_rounded_section(
                x,
                center_y=screen_center_y,
                depth=SCREEN_DEPTH,
                height=SCREEN_HEIGHT,
                radius=0.018,
            )
        )
    return mesh_from_geometry(section_loft(sections), "screen_panel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_monitor")

    shell_black = model.material("shell_black", rgba=(0.07, 0.07, 0.08, 1.0))
    screen_black = model.material("screen_black", rgba=(0.02, 0.025, 0.03, 1.0))
    stand_black = model.material("stand_black", rgba=(0.12, 0.12, 0.13, 1.0))
    accent_red = model.material("accent_red", rgba=(0.55, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=stand_black,
        name="hub",
    )
    base.visual(
        Box((0.032, 0.240, 0.012)),
        origin=Origin(xyz=(0.0, 0.110, 0.006)),
        material=stand_black,
        name="front_leg",
    )
    base.visual(
        Box((0.030, 0.230, 0.012)),
        origin=Origin(xyz=(-0.095, -0.055, 0.006), rpy=(0.0, 0.0, 2.0 * math.pi / 3.0)),
        material=stand_black,
        name="rear_leg_0",
    )
    base.visual(
        Box((0.030, 0.230, 0.012)),
        origin=Origin(xyz=(0.095, -0.055, 0.006), rpy=(0.0, 0.0, -2.0 * math.pi / 3.0)),
        material=stand_black,
        name="rear_leg_1",
    )
    base.visual(
        Box((0.030, 0.050, 0.003)),
        origin=Origin(xyz=(0.0, 0.215, 0.0115)),
        material=accent_red,
        name="front_tip",
    )
    base.visual(
        Box((0.028, 0.044, 0.003)),
        origin=Origin(xyz=(-0.185, -0.107, 0.0115), rpy=(0.0, 0.0, 2.0 * math.pi / 3.0)),
        material=accent_red,
        name="rear_tip_0",
    )
    base.visual(
        Box((0.028, 0.044, 0.003)),
        origin=Origin(xyz=(0.185, -0.107, 0.0115), rpy=(0.0, 0.0, -2.0 * math.pi / 3.0)),
        material=accent_red,
        name="rear_tip_1",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.041, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stand_black,
        name="swivel_collar",
    )
    stand.visual(
        Box((0.056, 0.030, 0.300)),
        origin=Origin(xyz=(0.0, -0.008, 0.150)),
        material=stand_black,
        name="column",
    )
    stand.visual(
        Box((0.086, 0.024, 0.100)),
        origin=Origin(xyz=(0.0, -0.006, 0.350)),
        material=stand_black,
        name="column_head",
    )
    stand.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.160, 0.050, 0.120),
                span_width=0.094,
                trunnion_diameter=0.016,
                trunnion_center_z=0.097,
                base_thickness=0.016,
                corner_radius=0.010,
                center=False,
            ),
            "yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=stand_black,
        name="yoke",
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=4.0),
    )

    display = model.part("display")
    display.visual(_curved_shell_mesh(), material=shell_black, name="screen_shell")
    display.visual(_screen_panel_mesh(), material=screen_black, name="screen_panel")
    display.visual(
        Box((0.086, 0.040, 0.120)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=stand_black,
        name="mount_block",
    )
    display.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="trunnion_0",
    )
    display.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="trunnion_1",
    )
    display.visual(
        Box((0.140, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.050, -0.213)),
        material=stand_black,
        name="control_rail_front",
    )
    display.visual(
        Box((0.140, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.034, -0.213)),
        material=stand_black,
        name="control_rail_rear",
    )
    display.visual(
        Box((0.006, 0.020, 0.022)),
        origin=Origin(xyz=(-0.070, 0.042, -0.205)),
        material=stand_black,
        name="control_side_0",
    )
    display.visual(
        Box((0.006, 0.020, 0.022)),
        origin=Origin(xyz=(0.070, 0.042, -0.205)),
        material=stand_black,
        name="control_side_1",
    )
    display.visual(
        Box((0.010, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.028, -0.209)),
        material=stand_black,
        name="control_bracket",
    )

    model.articulation(
        "stand_to_display",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.487)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(20.0),
        ),
    )

    button_x = (-0.048, -0.024, 0.0, 0.024, 0.048)
    for index, x in enumerate(button_x):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.015, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=stand_black,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=stand_black,
            name="button_stem",
        )
        model.articulation(
            f"display_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, 0.042, -0.218)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=0.05,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    display = object_model.get_part("display")
    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("stand_to_display")
    center_button = object_model.get_part("button_2")
    center_button_joint = object_model.get_articulation("display_to_button_2")

    ctx.allow_overlap(
        display,
        stand,
        elem_a="trunnion_0",
        elem_b="yoke",
        reason="The display trunnion is intentionally retained inside the simplified yoke cheek bore mesh.",
    )
    ctx.allow_overlap(
        display,
        stand,
        elem_a="trunnion_1",
        elem_b="yoke",
        reason="The mirrored display trunnion is intentionally retained inside the simplified yoke cheek bore mesh.",
    )

    ctx.expect_gap(
        display,
        base,
        axis="z",
        min_gap=0.22,
        name="display clears the desk base",
    )
    ctx.expect_gap(
        display,
        center_button,
        axis="z",
        positive_elem="screen_shell",
        negative_elem="button_cap",
        min_gap=0.009,
        max_gap=0.018,
        name="control buttons hang under the lower edge",
    )
    ctx.expect_contact(
        center_button,
        display,
        elem_a="button_stem",
        elem_b="control_rail_front",
        name="center button is captured by the control guide",
    )
    ctx.expect_contact(
        display,
        stand,
        elem_a="trunnion_0",
        elem_b="yoke",
        name="display trunnion seats in the yoke",
    )

    for index in range(4):
        ctx.expect_origin_distance(
            f"button_{index}",
            f"button_{index + 1}",
            axes="x",
            min_dist=0.020,
            max_dist=0.028,
            name=f"button spacing {index}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_screen = ctx.part_element_world_aabb(display, elem="screen_panel")
        with ctx.pose({tilt: tilt_limits.upper}):
            tilted_screen = ctx.part_element_world_aabb(display, elem="screen_panel")
        rest_top = rest_screen[1][2] if rest_screen is not None else None
        tilted_top = tilted_screen[1][2] if tilted_screen is not None else None
        ctx.check(
            "positive tilt raises the top edge",
            rest_top is not None and tilted_top is not None and tilted_top > rest_top + 0.015,
            details=f"rest_top={rest_top}, tilted_top={tilted_top}",
        )

    rest_center = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_panel"))
    with ctx.pose({swivel: 1.0}):
        swiveled_center = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_panel"))
    ctx.check(
        "swivel rotates the display footprint around the stand axis",
        rest_center is not None
        and swiveled_center is not None
        and abs(swiveled_center[0] - rest_center[0]) > 0.03,
        details=f"rest_center={rest_center}, swiveled_center={swiveled_center}",
    )

    rest_button_pos = ctx.part_world_position(center_button)
    with ctx.pose({center_button_joint: 0.0025}):
        pressed_button_pos = ctx.part_world_position(center_button)
        ctx.expect_gap(
            display,
            center_button,
            axis="z",
            positive_elem="screen_shell",
            negative_elem="button_cap",
            min_gap=0.006,
            name="pressed button still clears the display shell",
        )
    ctx.check(
        "center button presses upward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] > rest_button_pos[2] + 0.0015,
        details=f"rest_button_pos={rest_button_pos}, pressed_button_pos={pressed_button_pos}",
    )

    ctx.check(
        "five independent underside buttons are present",
        all(object_model.get_part(f"button_{index}") is not None for index in range(5)),
        details="Expected button_0 through button_4.",
    )

    return ctx.report()


object_model = build_object_model()

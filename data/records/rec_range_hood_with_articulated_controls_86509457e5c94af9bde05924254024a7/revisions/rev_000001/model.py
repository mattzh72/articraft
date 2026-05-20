from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOOD_WIDTH = 1.52
HOOD_HALF_DEPTH = 0.48
TOP_FRONT_Y = 0.30
BODY_BOTTOM_Z = 0.18
ROOF_Z = 0.60
SHELL_THICKNESS = 0.025
FRONT_RAIL_THICKNESS = 0.03
FRONT_RAIL_HEIGHT = 0.10
FRONT_OUTER_Y = HOOD_HALF_DEPTH
BUTTON_WIDTH = 0.07
BUTTON_HEIGHT = 0.028
BUTTON_CAP_DEPTH = 0.012
BUTTON_STEM_WIDTH = 0.038
BUTTON_STEM_HEIGHT = 0.015
BUTTON_STEM_DEPTH = 0.02
BUTTON_TRAVEL = 0.007
BUTTON_CENTER_Z = BODY_BOTTOM_Z + 0.052
BUTTON_XS = (-0.24, -0.12, 0.0, 0.12, 0.24)
FILTER_WIDTH = 0.70
FILTER_DEPTH = 0.58
FILTER_THICKNESS = 0.028
FILTER_HINGE_Y = -0.20
FILTER_HINGE_Z = 0.168
FILTER_CENTER_GAP = 0.03
FILTER_OPEN_LIMIT = 1.20


def make_front_rail_mesh() -> object:
    front_rail = (
        cq.Workplane("XY")
        .box(HOOD_WIDTH, FRONT_RAIL_THICKNESS, FRONT_RAIL_HEIGHT)
        .translate(
            (
                0.0,
                FRONT_OUTER_Y - FRONT_RAIL_THICKNESS / 2.0,
                BODY_BOTTOM_Z + FRONT_RAIL_HEIGHT / 2.0,
            )
        )
    )

    button_cutter = None
    for button_x in BUTTON_XS:
        cut = (
            cq.Workplane("XY")
            .box(
                BUTTON_STEM_WIDTH + 0.006,
                FRONT_RAIL_THICKNESS + 0.015,
                BUTTON_STEM_HEIGHT + 0.008,
            )
            .translate((button_x, FRONT_OUTER_Y - FRONT_RAIL_THICKNESS / 2.0, BUTTON_CENTER_Z))
        )
        button_cutter = cut if button_cutter is None else button_cutter.union(cut)
    return front_rail.cut(button_cutter)


def make_side_panel_mesh(x_pos: float) -> object:
    side_profile = [
        (-HOOD_HALF_DEPTH, BODY_BOTTOM_Z),
        (-HOOD_HALF_DEPTH, ROOF_Z),
        (TOP_FRONT_Y, ROOF_Z),
        (FRONT_OUTER_Y, BODY_BOTTOM_Z + FRONT_RAIL_HEIGHT),
        (FRONT_OUTER_Y, BODY_BOTTOM_Z),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(SHELL_THICKNESS)
        .translate((x_pos, 0.0, 0.0))
    )


def make_filter_mesh() -> object:
    frame_band = 0.03
    rib_width = 0.022
    rib_height = 0.016

    panel = (
        cq.Workplane("XY")
        .box(FILTER_WIDTH, FILTER_DEPTH, FILTER_THICKNESS)
        .translate((0.0, FILTER_DEPTH / 2.0, -FILTER_THICKNESS / 2.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(
            FILTER_WIDTH - 2.0 * frame_band,
            FILTER_DEPTH - 2.0 * frame_band,
            FILTER_THICKNESS + 0.01,
        )
        .translate((0.0, FILTER_DEPTH / 2.0, -FILTER_THICKNESS / 2.0))
    )
    panel = panel.cut(opening)

    inner_width = FILTER_WIDTH - 2.0 * frame_band
    rib_count = 6
    pitch = inner_width / rib_count
    for index in range(rib_count - 1):
        rib_x = -inner_width / 2.0 + pitch * (index + 1)
        rib = (
            cq.Workplane("XY")
            .box(rib_width, FILTER_DEPTH - 2.0 * frame_band, rib_height)
            .translate((rib_x, FILTER_DEPTH / 2.0, -rib_height / 2.0))
        )
        panel = panel.union(rib)

    pull_tab = (
        cq.Workplane("XY")
        .box(0.10, 0.016, 0.024)
        .translate((0.0, FILTER_DEPTH - 0.008, -0.012))
    )
    return panel.union(pull_tab)


def add_button(
    model: ArticulatedObject,
    canopy,
    *,
    index: int,
    x_pos: float,
    material: str,
) -> None:
    button = model.part(f"button_{index}")
    button.visual(
        Box((BUTTON_WIDTH, BUTTON_CAP_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, BUTTON_CAP_DEPTH / 2.0, 0.0)),
        material=material,
        name="cap",
    )
    button.visual(
        Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH / 2.0, 0.0)),
        material=material,
        name="stem",
    )

    model.articulation(
        f"button_{index}_slide",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=button,
        origin=Origin(xyz=(x_pos, FRONT_OUTER_Y, BUTTON_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BUTTON_TRAVEL,
            effort=15.0,
            velocity=0.12,
        ),
    )


def add_filter(
    model: ArticulatedObject,
    canopy,
    *,
    index: int,
    x_pos: float,
    material: str,
) -> None:
    filter_panel = model.part(f"filter_{index}")
    filter_panel.visual(
        mesh_from_cadquery(make_filter_mesh(), f"filter_{index}_panel"),
        material=material,
        name="panel",
    )

    model.articulation(
        f"filter_{index}_hinge",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=filter_panel,
        origin=Origin(xyz=(x_pos, FILTER_HINGE_Y, FILTER_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FILTER_OPEN_LIMIT,
            effort=25.0,
            velocity=1.0,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canopy_range_hood")

    stainless = model.material("stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.66, 0.69, 0.71, 1.0))
    button_black = model.material("button_black", rgba=(0.11, 0.11, 0.12, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Box((HOOD_WIDTH, TOP_FRONT_Y + HOOD_HALF_DEPTH, SHELL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, (TOP_FRONT_Y - HOOD_HALF_DEPTH) / 2.0, ROOF_Z - SHELL_THICKNESS / 2.0)
        ),
        material=stainless,
        name="roof",
    )
    canopy.visual(
        Box((HOOD_WIDTH, SHELL_THICKNESS, ROOF_Z - BODY_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                0.0,
                -HOOD_HALF_DEPTH + SHELL_THICKNESS / 2.0,
                BODY_BOTTOM_Z + (ROOF_Z - BODY_BOTTOM_Z) / 2.0,
            )
        ),
        material=stainless,
        name="back",
    )
    fascia_length = math.hypot(
        FRONT_OUTER_Y - TOP_FRONT_Y,
        (ROOF_Z - SHELL_THICKNESS) - (BODY_BOTTOM_Z + FRONT_RAIL_HEIGHT),
    )
    fascia_angle = math.atan2(
        TOP_FRONT_Y - FRONT_OUTER_Y,
        (ROOF_Z - SHELL_THICKNESS) - (BODY_BOTTOM_Z + FRONT_RAIL_HEIGHT),
    )
    canopy.visual(
        Box((HOOD_WIDTH, SHELL_THICKNESS, fascia_length)),
        origin=Origin(
            xyz=(
                0.0,
                (TOP_FRONT_Y + FRONT_OUTER_Y) / 2.0,
                ((ROOF_Z - SHELL_THICKNESS) + (BODY_BOTTOM_Z + FRONT_RAIL_HEIGHT)) / 2.0,
            ),
            rpy=(fascia_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="fascia",
    )
    canopy.visual(
        mesh_from_cadquery(make_front_rail_mesh(), "front_rail"),
        material=stainless,
        name="front_rail",
    )
    canopy.visual(
        mesh_from_cadquery(make_side_panel_mesh(-HOOD_WIDTH / 2.0), "side_panel_0"),
        material=stainless,
        name="side_0",
    )
    canopy.visual(
        mesh_from_cadquery(make_side_panel_mesh(HOOD_WIDTH / 2.0 - SHELL_THICKNESS), "side_panel_1"),
        material=stainless,
        name="side_1",
    )
    canopy.visual(
        Box((HOOD_WIDTH - 2.0 * SHELL_THICKNESS, HOOD_HALF_DEPTH + FILTER_HINGE_Y, 0.02)),
        origin=Origin(xyz=(0.0, (-HOOD_HALF_DEPTH + FILTER_HINGE_Y) / 2.0, 0.19)),
        material=stainless,
        name="rear_hinge_rail",
    )
    canopy.visual(
        Box((HOOD_WIDTH - 2.0 * SHELL_THICKNESS, 0.07, 0.02)),
        origin=Origin(xyz=(0.0, 0.415, 0.178)),
        material=stainless,
        name="front_latch",
    )
    canopy.visual(
        Box((0.34, 0.26, 0.16)),
        origin=Origin(xyz=(0.0, -0.04, ROOF_Z + 0.08)),
        material=stainless,
        name="collar",
    )

    for index, x_pos in enumerate(BUTTON_XS):
        add_button(model, canopy, index=index, x_pos=x_pos, material=button_black)

    filter_x_offset = FILTER_WIDTH / 2.0 + FILTER_CENTER_GAP / 2.0
    add_filter(model, canopy, index=0, x_pos=-filter_x_offset, material=filter_metal)
    add_filter(model, canopy, index=1, x_pos=filter_x_offset, material=filter_metal)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    buttons = [object_model.get_part(f"button_{index}") for index in range(5)]
    button_joints = [object_model.get_articulation(f"button_{index}_slide") for index in range(5)]
    filters = [object_model.get_part(f"filter_{index}") for index in range(2)]
    filter_joints = [object_model.get_articulation(f"filter_{index}_hinge") for index in range(2)]

    for button in buttons:
        ctx.expect_contact(
            button,
            canopy,
            elem_a="cap",
            name=f"{button.name} cap stays mounted to the front control rail",
        )

    rest_button_0 = ctx.part_world_position(buttons[0])
    rest_button_4 = ctx.part_world_position(buttons[4])
    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        pressed_button_0 = ctx.part_world_position(buttons[0])
        untouched_button_4 = ctx.part_world_position(buttons[4])

    ctx.check(
        "buttons depress independently",
        rest_button_0 is not None
        and rest_button_4 is not None
        and pressed_button_0 is not None
        and untouched_button_4 is not None
        and pressed_button_0[1] < rest_button_0[1] - 0.004
        and abs(untouched_button_4[1] - rest_button_4[1]) < 1e-6,
        details=(
            f"rest_button_0={rest_button_0}, pressed_button_0={pressed_button_0}, "
            f"rest_button_4={rest_button_4}, untouched_button_4={untouched_button_4}"
        ),
    )

    for filter_panel in filters:
        ctx.expect_contact(
            filter_panel,
            canopy,
            name=f"{filter_panel.name} stays supported by the rear gutter mount",
        )

    closed_filter_aabb = ctx.part_world_aabb(filters[0])
    with ctx.pose({filter_joints[0]: FILTER_OPEN_LIMIT}):
        open_filter_aabb = ctx.part_world_aabb(filters[0])

    ctx.check(
        "filter swings downward on its rear hinge",
        closed_filter_aabb is not None
        and open_filter_aabb is not None
        and open_filter_aabb[0][2] < closed_filter_aabb[0][2] - 0.18,
        details=f"closed={closed_filter_aabb}, open={open_filter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

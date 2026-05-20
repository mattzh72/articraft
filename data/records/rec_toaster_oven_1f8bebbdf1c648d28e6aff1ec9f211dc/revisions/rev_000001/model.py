from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_W = 0.52
BODY_D = 0.39
BODY_H = 0.33
WALL = 0.015
FRONT_FASCIA = 0.012
CONTROL_W = 0.135

LEFT_REGION_W = BODY_W - CONTROL_W
LEFT_REGION_X = -CONTROL_W / 2.0
OPENING_W = 0.345
OPENING_H = 0.220
OPENING_BOTTOM = 0.055
OPENING_TOP = OPENING_BOTTOM + OPENING_H
OPENING_X = LEFT_REGION_X
OPENING_LEFT = OPENING_X - OPENING_W / 2.0
OPENING_RIGHT = OPENING_X + OPENING_W / 2.0

DOOR_W = OPENING_W - 0.010
DOOR_H = OPENING_H - 0.008
DOOR_T = 0.022

BUTTON_W = 0.046
BUTTON_H = 0.018
BUTTON_T = 0.010
BUTTON_TRAVEL = 0.0035


def _add_button(
    model: ArticulatedObject,
    housing,
    button_index: int,
    *,
    x: float,
    y: float,
    z: float,
    body_finish,
    legend_finish,
) -> None:
    button = model.part(f"button_{button_index}")
    button.visual(
        Box((BUTTON_W * 0.72, BUTTON_T * 0.55, BUTTON_H * 0.56)),
        origin=Origin(xyz=(0.0, -BUTTON_T * 0.25, 0.0)),
        material=body_finish,
        name="button_stem",
    )
    button.visual(
        Box((BUTTON_W, BUTTON_T, BUTTON_H)),
        material=body_finish,
        name="button_cap",
    )
    button.visual(
        Box((BUTTON_W * 0.78, BUTTON_T * 0.20, BUTTON_H * 0.16)),
        origin=Origin(xyz=(0.0, BUTTON_T * 0.32, BUTTON_H * 0.18)),
        material=legend_finish,
        name="button_legend",
    )

    model.articulation(
        f"housing_to_button_{button_index}",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_toaster_oven")

    stainless = model.material("stainless", rgba=(0.73, 0.74, 0.76, 1.0))
    fascia = model.material("fascia", rgba=(0.11, 0.11, 0.12, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.16, 0.20, 0.24, 0.42))
    display_glass = model.material("display_glass", rgba=(0.17, 0.34, 0.40, 0.55))
    selector_finish = model.material("selector_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    trim = model.material("trim", rgba=(0.84, 0.85, 0.87, 1.0))
    button_finish = model.material("button_finish", rgba=(0.18, 0.18, 0.19, 1.0))
    button_legend = model.material("button_legend", rgba=(0.82, 0.83, 0.85, 1.0))
    rubber = model.material("rubber", rgba=(0.13, 0.13, 0.13, 1.0))

    housing = model.part("housing")

    housing.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    housing.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - WALL / 2.0)),
        material=stainless,
        name="top_shell",
    )
    housing.visual(
        Box((WALL, BODY_D, BODY_H - 2.0 * WALL)),
        origin=Origin(
            xyz=(-BODY_W / 2.0 + WALL / 2.0, 0.0, BODY_H / 2.0),
        ),
        material=stainless,
        name="left_shell",
    )
    housing.visual(
        Box((WALL, BODY_D, BODY_H - 2.0 * WALL)),
        origin=Origin(
            xyz=(BODY_W / 2.0 - WALL / 2.0, 0.0, BODY_H / 2.0),
        ),
        material=stainless,
        name="right_shell",
    )
    housing.visual(
        Box((BODY_W - 2.0 * WALL, WALL, BODY_H - 2.0 * WALL)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + WALL / 2.0, BODY_H / 2.0),
        ),
        material=stainless,
        name="rear_shell",
    )

    housing.visual(
        Box((LEFT_REGION_W, FRONT_FASCIA, OPENING_BOTTOM)),
        origin=Origin(
            xyz=(LEFT_REGION_X, BODY_D / 2.0 - FRONT_FASCIA / 2.0, OPENING_BOTTOM / 2.0),
        ),
        material=fascia,
        name="lower_fascia",
    )
    housing.visual(
        Box((LEFT_REGION_W, FRONT_FASCIA, BODY_H - OPENING_TOP)),
        origin=Origin(
            xyz=(
                LEFT_REGION_X,
                BODY_D / 2.0 - FRONT_FASCIA / 2.0,
                OPENING_TOP + (BODY_H - OPENING_TOP) / 2.0,
            ),
        ),
        material=fascia,
        name="upper_fascia",
    )
    housing.visual(
        Box((OPENING_LEFT - (-BODY_W / 2.0), FRONT_FASCIA, OPENING_H)),
        origin=Origin(
            xyz=(
                -BODY_W / 2.0 + (OPENING_LEFT - (-BODY_W / 2.0)) / 2.0,
                BODY_D / 2.0 - FRONT_FASCIA / 2.0,
                OPENING_BOTTOM + OPENING_H / 2.0,
            ),
        ),
        material=fascia,
        name="left_jamb",
    )
    housing.visual(
        Box((BODY_W / 2.0 - CONTROL_W - OPENING_RIGHT, FRONT_FASCIA, OPENING_H)),
        origin=Origin(
            xyz=(
                OPENING_RIGHT + (BODY_W / 2.0 - CONTROL_W - OPENING_RIGHT) / 2.0,
                BODY_D / 2.0 - FRONT_FASCIA / 2.0,
                OPENING_BOTTOM + OPENING_H / 2.0,
            ),
        ),
        material=fascia,
        name="center_stile",
    )
    housing.visual(
        Box((CONTROL_W, FRONT_FASCIA, BODY_H)),
        origin=Origin(
            xyz=(BODY_W / 2.0 - CONTROL_W / 2.0, BODY_D / 2.0 - FRONT_FASCIA / 2.0, BODY_H / 2.0),
        ),
        material=fascia,
        name="control_column",
    )
    housing.visual(
        Box((LEFT_REGION_W, 0.006, 0.014)),
        origin=Origin(
            xyz=(LEFT_REGION_X, BODY_D / 2.0 + 0.003, OPENING_TOP + 0.010),
        ),
        material=trim,
        name="door_header_trim",
    )

    foot_positions = (
        (-0.18, -0.13),
        (0.18, -0.13),
        (-0.18, 0.13),
        (0.18, 0.13),
    )
    for index, (x, y) in enumerate(foot_positions):
        housing.visual(
            Box((0.055, 0.032, 0.012)),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"foot_{index}",
        )

    display = model.part("display")
    display.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.082, 0.038),
                (0.108, 0.062),
                0.010,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.006,
                outer_corner_radius=0.009,
                center=False,
            ),
            "display_bezel",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="display_bezel",
    )
    display.visual(
        Box((0.086, 0.004, 0.042)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=display_glass,
        name="display_screen",
    )
    model.articulation(
        "housing_to_display",
        ArticulationType.FIXED,
        parent=housing,
        child=display,
        origin=Origin(xyz=(BODY_W / 2.0 - CONTROL_W / 2.0, BODY_D / 2.0, 0.262)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, 0.018)),
        origin=Origin(xyz=(0.0, DOOR_T / 2.0, 0.009)),
        material=fascia,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, 0.018)),
        origin=Origin(xyz=(0.0, DOOR_T / 2.0, DOOR_H - 0.009)),
        material=fascia,
        name="top_rail",
    )
    door.visual(
        Box((0.022, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.011, DOOR_T / 2.0, DOOR_H / 2.0)),
        material=fascia,
        name="left_rail",
    )
    door.visual(
        Box((0.022, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0 - 0.011, DOOR_T / 2.0, DOOR_H / 2.0)),
        material=fascia,
        name="right_rail",
    )
    door.visual(
        Box((DOOR_W - 0.036, 0.008, DOOR_H - 0.036)),
        origin=Origin(xyz=(0.0, DOOR_T / 2.0 - 0.001, DOOR_H / 2.0)),
        material=dark_glass,
        name="glass_panel",
    )
    handle_z = DOOR_H * 0.72
    for suffix, x in (("0", -0.100), ("1", 0.100)):
        door.visual(
            Box((0.018, 0.018, 0.024)),
            origin=Origin(xyz=(x, 0.018, handle_z)),
            material=trim,
            name=f"handle_post_{suffix}",
        )
    door.visual(
        Box((0.250, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, handle_z)),
        material=trim,
        name="handle_bar",
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(
            xyz=(OPENING_X, BODY_D / 2.0, OPENING_BOTTOM),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.65,
        ),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="selector_shaft",
    )
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.026,
                body_style="skirted",
                top_diameter=0.036,
                base_diameter=0.046,
                edge_radius=0.0015,
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=selector_finish,
        name="selector_cap",
    )
    selector.visual(
        Box((0.004, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, 0.026, 0.011)),
        material=trim,
        name="selector_marker",
    )
    model.articulation(
        "housing_to_selector",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector,
        origin=Origin(
            xyz=(BODY_W / 2.0 - CONTROL_W / 2.0, BODY_D / 2.0, 0.073),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    button_x_offsets = (-0.030, 0.030)
    button_z_rows = (0.210, 0.168, 0.126)
    button_center_x = BODY_W / 2.0 - CONTROL_W / 2.0
    button_center_y = BODY_D / 2.0 + 0.00525
    button_index = 0
    for z in button_z_rows:
        for x_offset in button_x_offsets:
            _add_button(
                model,
                housing,
                button_index,
                x=button_center_x + x_offset,
                y=button_center_y,
                z=z,
                body_finish=button_finish,
                legend_finish=button_legend,
            )
            button_index += 1

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    display = object_model.get_part("display")
    door = object_model.get_part("door")
    selector = object_model.get_part("selector")
    door_hinge = object_model.get_articulation("housing_to_door")
    selector_joint = object_model.get_articulation("housing_to_selector")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_4 = object_model.get_part("button_4")
    button_5 = object_model.get_part("button_5")
    button_0_joint = object_model.get_articulation("housing_to_button_0")
    button_5_joint = object_model.get_articulation("housing_to_button_5")

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="glass_panel",
        negative_elem="left_jamb",
        min_gap=0.004,
        max_gap=0.012,
        name="door glass sits ahead of the front jamb",
    )
    ctx.expect_contact(
        door,
        housing,
        elem_a="bottom_rail",
        elem_b="lower_fascia",
        name="door is carried by the lower front rail",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        min_overlap=0.20,
        name="door covers the oven aperture",
    )

    for index in range(6):
        button = object_model.get_part(f"button_{index}")
        ctx.expect_origin_gap(
            display,
            button,
            axis="z",
            min_gap=0.030,
            name=f"display stays above button_{index}",
        )

    ctx.check(
        "selector articulation is continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.motion_limits is not None
        and selector_joint.motion_limits.lower is None
        and selector_joint.motion_limits.upper is None,
        details=f"type={selector_joint.articulation_type!r}, limits={selector_joint.motion_limits!r}",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.50}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.12
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
        details=f"closed={closed_door_aabb!r}, open={open_door_aabb!r}",
    )

    button_0_rest = ctx.part_world_aabb(button_0)
    button_1_rest = ctx.part_world_aabb(button_1)
    with ctx.pose({button_0_joint: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_aabb(button_0)
        button_1_steady = ctx.part_world_aabb(button_1)
    ctx.check(
        "button_0 depresses independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_steady is not None
        and button_0_pressed[1][1] < button_0_rest[1][1] - 0.002
        and abs(button_1_steady[1][1] - button_1_rest[1][1]) < 0.0005,
        details=(
            f"button_0_rest={button_0_rest!r}, button_0_pressed={button_0_pressed!r}, "
            f"button_1_rest={button_1_rest!r}, button_1_steady={button_1_steady!r}"
        ),
    )

    button_4_rest = ctx.part_world_aabb(button_4)
    button_5_rest = ctx.part_world_aabb(button_5)
    with ctx.pose({button_5_joint: BUTTON_TRAVEL}):
        button_4_steady = ctx.part_world_aabb(button_4)
        button_5_pressed = ctx.part_world_aabb(button_5)
    ctx.check(
        "button_5 depresses independently",
        button_4_rest is not None
        and button_4_steady is not None
        and button_5_rest is not None
        and button_5_pressed is not None
        and button_5_pressed[1][1] < button_5_rest[1][1] - 0.002
        and abs(button_4_steady[1][1] - button_4_rest[1][1]) < 0.0005,
        details=(
            f"button_4_rest={button_4_rest!r}, button_4_steady={button_4_steady!r}, "
            f"button_5_rest={button_5_rest!r}, button_5_pressed={button_5_pressed!r}"
        ),
    )

    ctx.expect_origin_gap(
        selector,
        door,
        axis="x",
        min_gap=0.18,
        name="selector stays to the right of the oven door",
    )

    return ctx.report()


object_model = build_object_model()

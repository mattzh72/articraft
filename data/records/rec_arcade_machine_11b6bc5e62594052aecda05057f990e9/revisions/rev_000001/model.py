from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

import cadquery as cq


TABLE_LENGTH = 0.92
TABLE_WIDTH = 0.66
TOP_HEIGHT = 0.73

BASE_LENGTH = 0.84
BASE_WIDTH = 0.58
BASE_HEIGHT = 0.12

PEDESTAL_LENGTH = 0.78
PEDESTAL_WIDTH = 0.52
PEDESTAL_HEIGHT = 0.42
PEDESTAL_Z0 = BASE_HEIGHT
PEDESTAL_Z1 = PEDESTAL_Z0 + PEDESTAL_HEIGHT

UPPER_HEIGHT = TOP_HEIGHT - PEDESTAL_Z1
UPPER_WALL_THICK = 0.022

SCREEN_FRAME_LENGTH = 0.66
SCREEN_FRAME_WIDTH = 0.42
GLASS_LENGTH = 0.63
GLASS_WIDTH = 0.39
GLASS_THICK = 0.006
WELL_HEIGHT = 0.11
WELL_OUTER_LENGTH = 0.63
WELL_OUTER_WIDTH = 0.39
WELL_INNER_LENGTH = 0.56
WELL_INNER_WIDTH = 0.30
SCREEN_BEZEL_LENGTH = 0.60
SCREEN_BEZEL_WIDTH = 0.34
SCREEN_LENGTH = 0.50
SCREEN_WIDTH = 0.27

SERVICE_OPENING_WIDTH = 0.336
SERVICE_OPENING_HEIGHT = 0.31
SERVICE_OPENING_Z0 = PEDESTAL_Z0 + 0.055
SERVICE_OPENING_Z1 = SERVICE_OPENING_Z0 + SERVICE_OPENING_HEIGHT
SERVICE_FRAME_X = -(PEDESTAL_LENGTH / 2.0) + (UPPER_WALL_THICK / 2.0)
SERVICE_JAMB_WIDTH = (PEDESTAL_WIDTH - 2.0 * UPPER_WALL_THICK - SERVICE_OPENING_WIDTH) / 2.0
SERVICE_RAIL_HEIGHT = (PEDESTAL_HEIGHT - SERVICE_OPENING_HEIGHT) / 2.0

DOOR_THICK = 0.018
DOOR_WIDTH = 0.332
DOOR_HEIGHT = 0.304
DOOR_BOTTOM = SERVICE_OPENING_Z0 + 0.003
DOOR_HINGE_X = -(PEDESTAL_LENGTH / 2.0) + (DOOR_THICK / 2.0)
DOOR_HINGE_Y = -(DOOR_WIDTH / 2.0)
DOOR_MAX_SWING = 1.55

CONTROL_PANEL_LENGTH = 0.38
CONTROL_PANEL_DEPTH = 0.13
CONTROL_PANEL_THICK = 0.022
CONTROL_RISER_DEPTH = 0.05
CONTROL_RISER_HEIGHT = 0.04
CONTROL_RISER_Y = TABLE_WIDTH / 2.0 + 0.015
CONTROL_PANEL_Y = TABLE_WIDTH / 2.0 + 0.04
CONTROL_PANEL_TOP = TOP_HEIGHT + CONTROL_RISER_HEIGHT + CONTROL_PANEL_THICK

BUTTON_RADIUS = 0.017
BUTTON_CAP_LENGTH = 0.008
BUTTON_REST_OFFSET = 0.008
BUTTON_TRAVEL = 0.0035
BUTTON_XS = (-0.09, 0.0, 0.09)
BUTTON_HOLE_RADIUS = 0.0105
BUTTON_STEM_RADIUS = 0.0085
BUTTON_STEM_LENGTH = 0.026
BUTTON_STEM_CENTER_Z = -0.009
BUTTON_FLANGE_RADIUS = 0.0125
BUTTON_FLANGE_LENGTH = 0.004
BUTTON_FLANGE_CENTER_Z = -0.024


def add_visual(part, geometry, xyz, *, material, name):
    part.visual(geometry, origin=Origin(xyz=xyz), material=material, name=name)


def make_control_shelf():
    shelf = cq.Workplane("XY").box(CONTROL_PANEL_LENGTH, CONTROL_PANEL_DEPTH, CONTROL_PANEL_THICK)
    shelf = (
        shelf.faces(">Z")
        .workplane()
        .pushPoints([(button_x, 0.0) for button_x in BUTTON_XS])
        .circle(BUTTON_HOLE_RADIUS)
        .cutThruAll()
    )
    return shelf


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cocktail_table_arcade")

    cabinet_black = model.material("cabinet_black", rgba=(0.14, 0.13, 0.12, 1.0))
    cabinet_trim = model.material("cabinet_trim", rgba=(0.22, 0.20, 0.18, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.45, 0.52, 0.56, 0.35))
    screen_black = model.material("screen_black", rgba=(0.03, 0.03, 0.04, 1.0))
    screen_glow = model.material("screen_glow", rgba=(0.09, 0.20, 0.32, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.62, 0.60, 1.0))
    button_red = model.material("button_red", rgba=(0.76, 0.12, 0.14, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.90, 0.73, 0.14, 1.0))
    button_blue = model.material("button_blue", rgba=(0.13, 0.34, 0.72, 1.0))

    body = model.part("body")

    add_visual(
        body,
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        (0.0, 0.0, BASE_HEIGHT / 2.0),
        material=cabinet_black,
        name="base_plinth",
    )
    add_visual(
        body,
        Box((PEDESTAL_LENGTH, PEDESTAL_WIDTH, 0.02)),
        (0.0, 0.0, PEDESTAL_Z0 + 0.01),
        material=cabinet_trim,
        name="pedestal_floor",
    )
    add_visual(
        body,
        Box((PEDESTAL_LENGTH, UPPER_WALL_THICK, PEDESTAL_HEIGHT)),
        (0.0, PEDESTAL_WIDTH / 2.0 - UPPER_WALL_THICK / 2.0, PEDESTAL_Z0 + PEDESTAL_HEIGHT / 2.0),
        material=cabinet_black,
        name="pedestal_side_0",
    )
    add_visual(
        body,
        Box((PEDESTAL_LENGTH, UPPER_WALL_THICK, PEDESTAL_HEIGHT)),
        (0.0, -(PEDESTAL_WIDTH / 2.0 - UPPER_WALL_THICK / 2.0), PEDESTAL_Z0 + PEDESTAL_HEIGHT / 2.0),
        material=cabinet_black,
        name="pedestal_side_1",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, PEDESTAL_WIDTH - 2.0 * UPPER_WALL_THICK, PEDESTAL_HEIGHT)),
        (PEDESTAL_LENGTH / 2.0 - UPPER_WALL_THICK / 2.0, 0.0, PEDESTAL_Z0 + PEDESTAL_HEIGHT / 2.0),
        material=cabinet_black,
        name="pedestal_end",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, PEDESTAL_WIDTH - 2.0 * UPPER_WALL_THICK, SERVICE_RAIL_HEIGHT)),
        (SERVICE_FRAME_X, 0.0, PEDESTAL_Z1 - SERVICE_RAIL_HEIGHT / 2.0),
        material=cabinet_black,
        name="service_top_rail",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, PEDESTAL_WIDTH - 2.0 * UPPER_WALL_THICK, SERVICE_RAIL_HEIGHT)),
        (SERVICE_FRAME_X, 0.0, PEDESTAL_Z0 + SERVICE_RAIL_HEIGHT / 2.0),
        material=cabinet_black,
        name="service_bottom_rail",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, SERVICE_JAMB_WIDTH, SERVICE_OPENING_HEIGHT)),
        (
            SERVICE_FRAME_X,
            -(SERVICE_OPENING_WIDTH / 2.0 + SERVICE_JAMB_WIDTH / 2.0),
            SERVICE_OPENING_Z0 + SERVICE_OPENING_HEIGHT / 2.0,
        ),
        material=cabinet_black,
        name="service_jamb_0",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, SERVICE_JAMB_WIDTH, SERVICE_OPENING_HEIGHT)),
        (
            SERVICE_FRAME_X,
            SERVICE_OPENING_WIDTH / 2.0 + SERVICE_JAMB_WIDTH / 2.0,
            SERVICE_OPENING_Z0 + SERVICE_OPENING_HEIGHT / 2.0,
        ),
        material=cabinet_black,
        name="service_jamb_1",
    )
    add_visual(
        body,
        Box((TABLE_LENGTH, TABLE_WIDTH, 0.02)),
        (0.0, 0.0, PEDESTAL_Z1 + 0.01),
        material=cabinet_trim,
        name="upper_floor",
    )
    add_visual(
        body,
        Box((TABLE_LENGTH, UPPER_WALL_THICK, UPPER_HEIGHT)),
        (0.0, TABLE_WIDTH / 2.0 - UPPER_WALL_THICK / 2.0, PEDESTAL_Z1 + UPPER_HEIGHT / 2.0),
        material=cabinet_black,
        name="upper_side_0",
    )
    add_visual(
        body,
        Box((TABLE_LENGTH, UPPER_WALL_THICK, UPPER_HEIGHT)),
        (0.0, -(TABLE_WIDTH / 2.0 - UPPER_WALL_THICK / 2.0), PEDESTAL_Z1 + UPPER_HEIGHT / 2.0),
        material=cabinet_black,
        name="upper_side_1",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, TABLE_WIDTH - 2.0 * UPPER_WALL_THICK, UPPER_HEIGHT)),
        (TABLE_LENGTH / 2.0 - UPPER_WALL_THICK / 2.0, 0.0, PEDESTAL_Z1 + UPPER_HEIGHT / 2.0),
        material=cabinet_black,
        name="upper_end_0",
    )
    add_visual(
        body,
        Box((UPPER_WALL_THICK, TABLE_WIDTH - 2.0 * UPPER_WALL_THICK, UPPER_HEIGHT)),
        (-(TABLE_LENGTH / 2.0 - UPPER_WALL_THICK / 2.0), 0.0, PEDESTAL_Z1 + UPPER_HEIGHT / 2.0),
        material=cabinet_black,
        name="upper_end_1",
    )
    add_visual(
        body,
        Box((TABLE_LENGTH - SCREEN_FRAME_LENGTH, TABLE_WIDTH, 0.022)),
        (SCREEN_FRAME_LENGTH / 2.0 + (TABLE_LENGTH - SCREEN_FRAME_LENGTH) / 4.0, 0.0, TOP_HEIGHT - 0.011),
        material=cabinet_trim,
        name="top_rim_0",
    )
    add_visual(
        body,
        Box((TABLE_LENGTH - SCREEN_FRAME_LENGTH, TABLE_WIDTH, 0.022)),
        (-(SCREEN_FRAME_LENGTH / 2.0 + (TABLE_LENGTH - SCREEN_FRAME_LENGTH) / 4.0), 0.0, TOP_HEIGHT - 0.011),
        material=cabinet_trim,
        name="top_rim_1",
    )
    add_visual(
        body,
        Box((SCREEN_FRAME_LENGTH, TABLE_WIDTH - SCREEN_FRAME_WIDTH, 0.022)),
        (0.0, SCREEN_FRAME_WIDTH / 2.0 + (TABLE_WIDTH - SCREEN_FRAME_WIDTH) / 4.0, TOP_HEIGHT - 0.011),
        material=cabinet_trim,
        name="top_rim_2",
    )
    add_visual(
        body,
        Box((SCREEN_FRAME_LENGTH, TABLE_WIDTH - SCREEN_FRAME_WIDTH, 0.022)),
        (0.0, -(SCREEN_FRAME_WIDTH / 2.0 + (TABLE_WIDTH - SCREEN_FRAME_WIDTH) / 4.0), TOP_HEIGHT - 0.011),
        material=cabinet_trim,
        name="top_rim_3",
    )
    add_visual(
        body,
        Box((GLASS_LENGTH, GLASS_WIDTH, GLASS_THICK)),
        (0.0, 0.0, TOP_HEIGHT - GLASS_THICK / 2.0),
        material=smoked_glass,
        name="glass_top",
    )
    add_visual(
        body,
        Box((WELL_OUTER_LENGTH, WELL_OUTER_WIDTH - WELL_INNER_WIDTH, WELL_HEIGHT)),
        (0.0, WELL_INNER_WIDTH / 2.0 + (WELL_OUTER_WIDTH - WELL_INNER_WIDTH) / 4.0, TOP_HEIGHT - GLASS_THICK - WELL_HEIGHT / 2.0),
        material=cabinet_trim,
        name="well_wall_0",
    )
    add_visual(
        body,
        Box((WELL_OUTER_LENGTH, WELL_OUTER_WIDTH - WELL_INNER_WIDTH, WELL_HEIGHT)),
        (0.0, -(WELL_INNER_WIDTH / 2.0 + (WELL_OUTER_WIDTH - WELL_INNER_WIDTH) / 4.0), TOP_HEIGHT - GLASS_THICK - WELL_HEIGHT / 2.0),
        material=cabinet_trim,
        name="well_wall_1",
    )
    add_visual(
        body,
        Box((WELL_OUTER_LENGTH - WELL_INNER_LENGTH, WELL_INNER_WIDTH, WELL_HEIGHT)),
        (WELL_INNER_LENGTH / 2.0 + (WELL_OUTER_LENGTH - WELL_INNER_LENGTH) / 4.0, 0.0, TOP_HEIGHT - GLASS_THICK - WELL_HEIGHT / 2.0),
        material=cabinet_trim,
        name="well_wall_2",
    )
    add_visual(
        body,
        Box((WELL_OUTER_LENGTH - WELL_INNER_LENGTH, WELL_INNER_WIDTH, WELL_HEIGHT)),
        (-(WELL_INNER_LENGTH / 2.0 + (WELL_OUTER_LENGTH - WELL_INNER_LENGTH) / 4.0), 0.0, TOP_HEIGHT - GLASS_THICK - WELL_HEIGHT / 2.0),
        material=cabinet_trim,
        name="well_wall_3",
    )
    add_visual(
        body,
        Box((SCREEN_BEZEL_LENGTH, SCREEN_BEZEL_WIDTH, 0.012)),
        (0.0, 0.0, TOP_HEIGHT - GLASS_THICK - WELL_HEIGHT - 0.006),
        material=screen_black,
        name="screen_bezel",
    )
    add_visual(
        body,
        Box((SCREEN_LENGTH, SCREEN_WIDTH, 0.004)),
        (0.0, 0.0, TOP_HEIGHT - GLASS_THICK - WELL_HEIGHT - 0.001),
        material=screen_glow,
        name="screen_surface",
    )
    add_visual(
        body,
        Box((0.012, 0.008, 0.07)),
        (DOOR_HINGE_X - 0.003, DOOR_HINGE_Y - 0.004, DOOR_BOTTOM + 0.06),
        material=metal,
        name="hinge_leaf_0",
    )
    add_visual(
        body,
        Box((0.012, 0.008, 0.07)),
        (DOOR_HINGE_X - 0.003, DOOR_HINGE_Y - 0.004, DOOR_BOTTOM + DOOR_HEIGHT - 0.06),
        material=metal,
        name="hinge_leaf_1",
    )
    add_visual(
        body,
        Box((CONTROL_PANEL_LENGTH, CONTROL_RISER_DEPTH, CONTROL_RISER_HEIGHT)),
        (0.0, CONTROL_RISER_Y, TOP_HEIGHT + CONTROL_RISER_HEIGHT / 2.0),
        material=cabinet_black,
        name="control_riser",
    )
    add_visual(
        body,
        mesh_from_cadquery(make_control_shelf(), "control_shelf"),
        (0.0, CONTROL_PANEL_Y, TOP_HEIGHT + CONTROL_RISER_HEIGHT + CONTROL_PANEL_THICK / 2.0),
        material=cabinet_trim,
        name="control_shelf",
    )
    add_visual(
        body,
        Box((CONTROL_PANEL_LENGTH, 0.018, 0.07)),
        (0.0, CONTROL_PANEL_Y + CONTROL_PANEL_DEPTH / 2.0 - 0.009, TOP_HEIGHT + 0.035),
        material=cabinet_black,
        name="control_fascia",
    )

    door = model.part("service_door")
    add_visual(
        door,
        Box((DOOR_THICK, DOOR_WIDTH, DOOR_HEIGHT)),
        (0.0, DOOR_WIDTH / 2.0, DOOR_HEIGHT / 2.0),
        material=cabinet_trim,
        name="door_panel",
    )
    add_visual(
        door,
        Box((0.028, 0.014, 0.07)),
        (-0.018, DOOR_WIDTH - 0.05, DOOR_HEIGHT * 0.55),
        material=metal,
        name="door_pull",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=DOOR_MAX_SWING,
        ),
    )

    button_materials = (button_red, button_yellow, button_blue)
    for index, (button_x, button_material) in enumerate(zip(BUTTON_XS, button_materials)):
        button = model.part(f"button_{index}")
        add_visual(
            button,
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_CAP_LENGTH),
            (0.0, 0.0, BUTTON_REST_OFFSET),
            material=button_material,
            name="button_cap",
        )
        add_visual(
            button,
            Cylinder(radius=BUTTON_STEM_RADIUS, length=BUTTON_STEM_LENGTH),
            (0.0, 0.0, BUTTON_STEM_CENTER_Z),
            material=metal,
            name="button_stem",
        )
        add_visual(
            button,
            Cylinder(radius=BUTTON_FLANGE_RADIUS, length=BUTTON_FLANGE_LENGTH),
            (0.0, 0.0, BUTTON_FLANGE_CENTER_Z),
            material=metal,
            name="button_flange",
        )
        model.articulation(
            f"button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_PANEL_Y, CONTROL_PANEL_TOP)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("service_door")
    door_hinge = object_model.get_articulation("door_hinge")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_1_slide = object_model.get_articulation("button_1_slide")

    with ctx.pose({door_hinge: 0.0}):
        closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "service door sits within the short-end aperture",
        closed_panel is not None
        and closed_panel[0][1] >= -(SERVICE_OPENING_WIDTH / 2.0)
        and closed_panel[1][1] <= SERVICE_OPENING_WIDTH / 2.0
        and closed_panel[0][2] >= SERVICE_OPENING_Z0
        and closed_panel[1][2] <= SERVICE_OPENING_Z1
        and abs(closed_panel[0][0] - (-(PEDESTAL_LENGTH / 2.0))) <= 0.0025,
        details=f"closed_panel={closed_panel}",
    )

    limits = door_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.upper}):
            open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.check(
            "service door swings outward on a vertical side hinge",
            closed_panel is not None
            and open_panel is not None
            and open_panel[0][0] < closed_panel[0][0] - 0.08
            and open_panel[1][1] < closed_panel[1][1] - 0.12,
            details=f"closed_panel={closed_panel}, open_panel={open_panel}",
        )

    ctx.expect_gap(
        button_1,
        body,
        axis="z",
        positive_elem="button_cap",
        negative_elem="control_shelf",
        min_gap=0.003,
        max_gap=0.01,
        name="button caps sit proud above the raised control shelf",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    limits = button_1_slide.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({button_1_slide: limits.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "play buttons articulate independently",
            button_0_rest is not None
            and button_1_rest is not None
            and button_0_pressed is not None
            and button_1_pressed is not None
            and abs(button_0_pressed[2] - button_0_rest[2]) <= 1e-6
            and button_1_pressed[2] < button_1_rest[2] - 0.003,
            details=(
                f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
                f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}"
            ),
        )

    return ctx.report()


object_model = build_object_model()

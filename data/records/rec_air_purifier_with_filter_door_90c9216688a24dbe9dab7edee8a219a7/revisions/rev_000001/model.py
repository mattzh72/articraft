from __future__ import annotations

import cadquery as cq

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


BODY_W = 0.255
BODY_D = 0.225
BODY_H = 0.740
WALL = 0.0035

OPENING_W = 0.182
OPENING_H = 0.548
OPENING_CENTER_Z = 0.382
REAR_FACE_Y = -BODY_D / 2.0 + WALL * 0.5

DOOR_W = 0.178
DOOR_H = 0.544
DOOR_T = 0.014
DOOR_HINGE_R = 0.006
DOOR_HINGE_LEN = 0.118
DOOR_SWING = 2.05

FILTER_W = 0.166
FILTER_D = 0.108
FILTER_H = 0.518
FILTER_TRAVEL = 0.078
FILTER_ORIGIN_Y = REAR_FACE_Y + 0.01525

BUTTON_BANK_Y = 0.042
BUTTON_BANK_W = 0.102
BUTTON_BANK_D = 0.038
BUTTON_TRAY_D = 0.003
BUTTON_STEM_SLOT_W = 0.084
BUTTON_STEM_SLOT_D = 0.018
BUTTON_TRAVEL = 0.0015
BUTTON_CAP_R = 0.009
BUTTON_CAP_H = 0.0032
BUTTON_STEM_R = 0.0037
BUTTON_STEM_H = 0.006
BUTTON_XS = (-0.033, -0.011, 0.011, 0.033)

FRONT_PANEL_W = 0.160
FRONT_PANEL_H = 0.470
FRONT_PANEL_Z = 0.390
FRONT_PANEL_RECESS = 0.004

TOP_VENT_Y = -0.018
TOP_VENT_XS = (-0.050, -0.032, -0.014, 0.004, 0.022, 0.040, 0.058)
TOP_VENT_W = 0.010
TOP_VENT_D = 0.052


def _housing_shell() -> object:
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(0.032)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .faces("<Z")
        .shell(-WALL)
    )

    rear_opening = (
        cq.Workplane("XY")
        .box(OPENING_W, WALL * 6.0, OPENING_H)
        .translate((0.0, -BODY_D / 2.0, OPENING_CENTER_Z))
    )
    body = body.cut(rear_opening)

    front_recess = (
        cq.Workplane("XY")
        .box(FRONT_PANEL_W, FRONT_PANEL_RECESS, FRONT_PANEL_H)
        .translate((0.0, BODY_D / 2.0 - FRONT_PANEL_RECESS / 2.0, FRONT_PANEL_Z))
    )
    body = body.cut(front_recess)

    button_tray = (
        cq.Workplane("XY")
        .box(BUTTON_BANK_W, BUTTON_BANK_D, BUTTON_TRAY_D)
        .translate((0.0, BUTTON_BANK_Y, BODY_H - BUTTON_TRAY_D / 2.0))
    )
    body = body.cut(button_tray)

    stem_slot = (
        cq.Workplane("XY")
        .box(BUTTON_STEM_SLOT_W, BUTTON_STEM_SLOT_D, 0.010)
        .translate((0.0, BUTTON_BANK_Y, BODY_H - BUTTON_TRAY_D - 0.005))
    )
    body = body.cut(stem_slot)

    vent_cutters = (
        cq.Workplane("XY")
        .pushPoints([(x, TOP_VENT_Y) for x in TOP_VENT_XS])
        .rect(TOP_VENT_W, TOP_VENT_D)
        .extrude(0.010)
        .translate((0.0, 0.0, BODY_H - 0.006))
    )
    body = body.cut(vent_cutters)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_purifier")

    shell_white = model.material("shell_white", rgba=(0.92, 0.93, 0.95, 1.0))
    trim_charcoal = model.material("trim_charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    filter_frame = model.material("filter_frame", rgba=(0.18, 0.19, 0.21, 1.0))
    filter_media = model.material("filter_media", rgba=(0.36, 0.46, 0.52, 1.0))
    tab_orange = model.material("tab_orange", rgba=(0.91, 0.47, 0.16, 1.0))
    button_black = model.material("button_black", rgba=(0.08, 0.09, 0.10, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "purifier_housing"),
        material=shell_white,
        name="shell",
    )
    housing.visual(
        Box((BODY_W + 0.010, BODY_D + 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=trim_charcoal,
        name="base_ring",
    )

    rear_door = model.part("rear_door")
    rear_door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_W / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="door_leaf",
    )
    rear_door.visual(
        Box((DOOR_W - 0.022, 0.004, DOOR_H - 0.070)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.004, -0.004, 0.0)),
        material=trim_charcoal,
        name="door_inset",
    )
    for index, z in enumerate((-0.195, 0.0, 0.195)):
        rear_door.visual(
            Cylinder(radius=DOOR_HINGE_R, length=DOOR_HINGE_LEN),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=trim_charcoal,
            name=f"hinge_knuckle_{index}",
        )

    filter_part = model.part("filter")
    filter_part.visual(
        Box((FILTER_W, FILTER_D, FILTER_H)),
        origin=Origin(xyz=(0.0, FILTER_D / 2.0, 0.0)),
        material=filter_frame,
        name="cartridge",
    )
    filter_part.visual(
        Box((FILTER_W - 0.020, FILTER_D - 0.018, FILTER_H - 0.040)),
        origin=Origin(xyz=(0.0, FILTER_D / 2.0 + 0.003, 0.0)),
        material=filter_media,
        name="media",
    )
    filter_part.visual(
        Box((0.022, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -0.004, FILTER_H / 2.0 - 0.030)),
        material=tab_orange,
        name="tab_stem",
    )
    filter_part.visual(
        Box((0.050, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.013, FILTER_H / 2.0 - 0.024)),
        material=tab_orange,
        name="tab_pull",
    )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_CAP_R, length=BUTTON_CAP_H),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_CAP_H / 2.0)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=BUTTON_STEM_R, length=BUTTON_STEM_H),
            origin=Origin(xyz=(0.0, 0.0, -BUTTON_STEM_H / 2.0)),
            material=button_black,
            name="button_stem",
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x, BUTTON_BANK_Y, BODY_H - BUTTON_TRAY_D)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "housing_to_rear_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=rear_door,
        origin=Origin(
            xyz=(OPENING_W / 2.0, -BODY_D / 2.0 - DOOR_T / 2.0, OPENING_CENTER_Z)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=DOOR_SWING,
        ),
    )
    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_part,
        origin=Origin(xyz=(0.0, FILTER_ORIGIN_Y, OPENING_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.15,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rear_door = object_model.get_part("rear_door")
    filter_part = object_model.get_part("filter")
    door_joint = object_model.get_articulation("housing_to_rear_door")
    filter_joint = object_model.get_articulation("housing_to_filter")
    button_joints = [
        object_model.get_articulation(f"housing_to_button_{index}")
        for index in range(len(BUTTON_XS))
    ]
    buttons = [object_model.get_part(f"button_{index}") for index in range(len(BUTTON_XS))]

    ctx.expect_contact(
        rear_door,
        housing,
        name="rear door seats on the housing at rest",
    )
    ctx.expect_within(
        filter_part,
        housing,
        axes="xz",
        margin=0.020,
        name="filter remains centered inside the housing",
    )
    ctx.expect_overlap(
        filter_part,
        housing,
        axes="y",
        min_overlap=0.090,
        name="filter stays substantially inserted at rest",
    )

    closed_door = ctx.part_element_world_aabb(rear_door, elem="door_leaf")
    closed_filter = ctx.part_world_aabb(filter_part)

    with ctx.pose({door_joint: DOOR_SWING, filter_joint: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_part,
            housing,
            axes="xz",
            margin=0.020,
            name="extended filter stays aligned with the rear opening",
        )
        ctx.expect_overlap(
            filter_part,
            housing,
            axes="y",
            min_overlap=0.028,
            name="extended filter remains retained in the housing",
        )
        open_door = ctx.part_element_world_aabb(rear_door, elem="door_leaf")
        open_filter = ctx.part_world_aabb(filter_part)

    door_swings_clear = (
        closed_door is not None
        and open_door is not None
        and open_door[0][1] < closed_door[0][1] - 0.080
    )
    ctx.check(
        "rear door swings outward from the rear opening",
        door_swings_clear,
        details=f"closed={closed_door}, open={open_door}",
    )

    filter_extends = (
        closed_filter is not None
        and open_filter is not None
        and open_filter[0][1] < closed_filter[0][1] - 0.060
    )
    ctx.check(
        "filter cartridge slides out toward the rear",
        filter_extends,
        details=f"closed={closed_filter}, open={open_filter}",
    )

    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        rest_aabb = ctx.part_world_aabb(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_aabb = ctx.part_world_aabb(button)
        button_moves_down = (
            rest_aabb is not None
            and pressed_aabb is not None
            and pressed_aabb[1][2] < rest_aabb[1][2] - 0.001
        )
        ctx.check(
            f"button_{index}_depresses",
            button_moves_down,
            details=f"rest={rest_aabb}, pressed={pressed_aabb}",
        )

    button_1_rest = ctx.part_world_aabb(buttons[1])
    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_aabb(buttons[0])
        button_1_unchanged = ctx.part_world_aabb(buttons[1])
    independent_motion = (
        button_0_pressed is not None
        and button_1_rest is not None
        and button_1_unchanged is not None
        and abs(button_1_unchanged[1][2] - button_1_rest[1][2]) <= 1e-6
        and button_0_pressed[1][2] < button_1_rest[1][2] - 0.001
    )
    ctx.check(
        "top buttons depress independently",
        independent_motion,
        details=(
            f"button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, "
            f"button_1_unchanged={button_1_unchanged}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

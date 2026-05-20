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
)


DRAWER_TRAVEL = 0.10
DISPLAY_TILT = math.radians(28.0)
DECK_PITCH = math.radians(18.0)
DECK_CENTER = (-0.005, 0.0, 0.114)
DECK_PANEL_LOCAL_Z = 0.015
DECK_PANEL_TOP_LOCAL_Z = 0.017


def _pitch_transform(
    local_x: float,
    local_y: float,
    local_z: float,
    *,
    pitch: float,
    center: tuple[float, float, float],
) -> tuple[float, float, float]:
    c = math.cos(pitch)
    s = math.sin(pitch)
    cx, cy, cz = center
    return (
        cx + local_x * c + local_z * s,
        cy + local_y,
        cz - local_x * s + local_z * c,
    )
def _add_key(
    model: ArticulatedObject,
    body,
    *,
    part_name: str,
    joint_name: str,
    local_x: float,
    local_y: float,
    size: tuple[float, float, float],
    material,
    visual_name: str,
    effort: float,
    travel: float,
) -> None:
    key = model.part(part_name)
    stem_width = min(size[0] * 0.52, size[1] * 0.78)
    stem_depth = min(size[1] * 0.52, size[0] * 0.78)
    key.visual(
        Box((stem_width, stem_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=material,
        name="key_stem",
    )
    key.visual(
        Box(size),
        origin=Origin(xyz=(0.0, 0.0, size[2] * 0.5)),
        material=material,
        name=visual_name,
    )
    key_origin = _pitch_transform(
        local_x,
        local_y,
        DECK_PANEL_TOP_LOCAL_Z,
        pitch=DECK_PITCH,
        center=DECK_CENTER,
    )
    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=body,
        child=key,
        origin=Origin(xyz=key_origin, rpy=(0.0, DECK_PITCH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=effort,
            velocity=0.05,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_register")

    body_cream = model.material("body_cream", rgba=(0.86, 0.84, 0.79, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    drawer_grey = model.material("drawer_grey", rgba=(0.69, 0.70, 0.72, 1.0))
    display_grey = model.material("display_grey", rgba=(0.34, 0.36, 0.39, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.15, 0.32, 0.33, 0.55))
    dept_key_cream = model.material("dept_key_cream", rgba=(0.93, 0.90, 0.83, 1.0))
    number_key_dark = model.material("number_key_dark", rgba=(0.27, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.300, 0.280, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_cream,
        name="body_floor",
    )
    body.visual(
        Box((0.300, 0.019, 0.090)),
        origin=Origin(xyz=(0.0, -0.1305, 0.055)),
        material=body_cream,
        name="body_side_0",
    )
    body.visual(
        Box((0.300, 0.019, 0.090)),
        origin=Origin(xyz=(0.0, 0.1305, 0.055)),
        material=body_cream,
        name="body_side_1",
    )
    body.visual(
        Box((0.018, 0.242, 0.090)),
        origin=Origin(xyz=(-0.141, 0.0, 0.055)),
        material=body_cream,
        name="body_rear",
    )
    body.visual(
        Box((0.060, 0.242, 0.028)),
        origin=Origin(xyz=(0.120, 0.0, 0.086)),
        material=body_cream,
        name="body_front_top",
    )
    body.visual(
        Box((0.010, 0.019, 0.060)),
        origin=Origin(xyz=(0.145, -0.1305, 0.040)),
        material=body_cream,
        name="body_front_side_0",
    )
    body.visual(
        Box((0.010, 0.019, 0.060)),
        origin=Origin(xyz=(0.145, 0.1305, 0.040)),
        material=body_cream,
        name="body_front_side_1",
    )
    body.visual(
        Box((0.120, 0.242, 0.018)),
        origin=Origin(xyz=(-0.080, 0.0, 0.101)),
        material=body_cream,
        name="rear_plinth",
    )
    body.visual(
        Box((0.145, 0.205, 0.012)),
        origin=Origin(
            xyz=_pitch_transform(
                0.0,
                0.0,
                0.005,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=body_cream,
        name="deck_support",
    )
    body.visual(
        Box((0.114, 0.058, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                0.000,
                -0.056,
                0.011,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=body_cream,
        name="dept_guide",
    )
    body.visual(
        Box((0.094, 0.078, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                0.002,
                0.050,
                0.011,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=body_cream,
        name="number_guide",
    )
    body.visual(
        Box((0.152, 0.022, 0.032)),
        origin=Origin(
            xyz=_pitch_transform(
                0.0,
                -0.105,
                -0.003,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=body_cream,
        name="deck_cheek_0",
    )
    body.visual(
        Box((0.152, 0.022, 0.032)),
        origin=Origin(
            xyz=_pitch_transform(
                0.0,
                0.105,
                -0.003,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=body_cream,
        name="deck_cheek_1",
    )
    body.visual(
        Box((0.160, 0.020, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                0.0,
                -0.104,
                DECK_PANEL_LOCAL_Z,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=trim_graphite,
        name="deck_rear_rail",
    )
    body.visual(
        Box((0.160, 0.020, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                0.0,
                0.104,
                DECK_PANEL_LOCAL_Z,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=trim_graphite,
        name="deck_front_rail",
    )
    body.visual(
        Box((0.014, 0.188, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                -0.073,
                0.0,
                DECK_PANEL_LOCAL_Z,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=trim_graphite,
        name="deck_side_0",
    )
    body.visual(
        Box((0.014, 0.188, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                0.073,
                0.0,
                DECK_PANEL_LOCAL_Z,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=trim_graphite,
        name="deck_side_1",
    )
    body.visual(
        Box((0.132, 0.018, 0.004)),
        origin=Origin(
            xyz=_pitch_transform(
                -0.004,
                -0.006,
                DECK_PANEL_LOCAL_Z,
                pitch=DECK_PITCH,
                center=DECK_CENTER,
            ),
            rpy=(0.0, DECK_PITCH, 0.0),
        ),
        material=trim_graphite,
        name="deck_divider",
    )
    body.visual(
        Box((0.070, 0.140, 0.006)),
        origin=Origin(xyz=(-0.082, 0.0, 0.109)),
        material=trim_graphite,
        name="rear_cap",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.010, 0.250, 0.060)),
        origin=Origin(xyz=(0.005, 0.0, 0.031)),
        material=drawer_grey,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.180, 0.228, 0.048)),
        origin=Origin(xyz=(-0.090, 0.0, 0.028)),
        material=drawer_grey,
        name="drawer_tray",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(xyz=(0.013, 0.0, 0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_graphite,
        name="drawer_handle",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.140, 0.0, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    stand = model.part("display_stand")
    stand.visual(
        Box((0.050, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=display_grey,
        name="stand_foot",
    )
    stand.visual(
        Box((0.020, 0.026, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=display_grey,
        name="stand_stalk",
    )
    stand.visual(
        Box((0.010, 0.046, 0.008)),
        origin=Origin(xyz=(-0.006, 0.0, 0.063)),
        material=display_grey,
        name="stand_crossbar",
    )
    stand.visual(
        Box((0.010, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.024, 0.059)),
        material=display_grey,
        name="stand_ear_0",
    )
    stand.visual(
        Box((0.010, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.024, 0.059)),
        material=display_grey,
        name="stand_ear_1",
    )
    model.articulation(
        "body_to_display_stand",
        ArticulationType.FIXED,
        parent=body,
        child=stand,
        origin=Origin(xyz=(-0.095, 0.0, 0.110)),
    )

    display = model.part("display")
    display.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=display_grey,
        name="hinge_barrel",
    )
    display.visual(
        Box((0.018, 0.110, 0.072)),
        origin=Origin(xyz=(0.009, 0.0, 0.041)),
        material=display_grey,
        name="display_shell",
    )
    display.visual(
        Box((0.0015, 0.094, 0.054)),
        origin=Origin(xyz=(0.018, 0.0, 0.043)),
        material=screen_glass,
        name="screen_panel",
    )
    model.articulation(
        "display_stand_to_display",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=math.radians(-12.0),
            upper=DISPLAY_TILT,
        ),
    )

    for row_index, local_x in enumerate((-0.028, 0.001, 0.030)):
        for col_index, local_y in enumerate((-0.074, -0.038)):
            _add_key(
                model,
                body,
                part_name=f"dept_key_{row_index}_{col_index}",
                joint_name=f"body_to_dept_key_{row_index}_{col_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.028, 0.022, 0.010),
                material=dept_key_cream,
                visual_name="dept_key",
                effort=2.4,
                travel=0.0025,
            )

    for row_index, local_x in enumerate((-0.034, -0.010, 0.014, 0.038)):
        for col_index, local_y in enumerate((0.026, 0.050, 0.074)):
            _add_key(
                model,
                body,
                part_name=f"number_key_{row_index}_{col_index}",
                joint_name=f"body_to_number_key_{row_index}_{col_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.016, 0.016, 0.009),
                material=number_key_dark,
                visual_name="number_key",
                effort=1.6,
                travel=0.0020,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    stand = object_model.get_part("display_stand")
    display = object_model.get_part("display")
    display_joint = object_model.get_articulation("display_stand_to_display")
    dept_key = object_model.get_part("dept_key_1_0")
    dept_joint = object_model.get_articulation("body_to_dept_key_1_0")
    number_key = object_model.get_part("number_key_2_1")
    number_joint = object_model.get_articulation("body_to_number_key_2_1")

    for row_index in range(3):
        for col_index in range(2):
            ctx.allow_overlap(
                body,
                object_model.get_part(f"dept_key_{row_index}_{col_index}"),
                elem_a="dept_guide",
                elem_b="key_stem",
                reason="The department key stem is intentionally simplified as plunging into the internal guide block proxy.",
            )
    for row_index in range(4):
        for col_index in range(3):
            ctx.allow_overlap(
                body,
                object_model.get_part(f"number_key_{row_index}_{col_index}"),
                elem_a="number_guide",
                elem_b="key_stem",
                reason="The number key stem is intentionally simplified as plunging into the internal guide block proxy.",
            )

    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        elem_a="drawer_face",
        min_overlap=0.05,
        name="drawer face stays centered in the base front",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_tray",
            min_overlap=0.08,
            name="drawer tray remains retained at full extension",
        )
        open_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[0] > closed_drawer_pos[0] + 0.08,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    ctx.expect_contact(
        stand,
        body,
        elem_a="stand_foot",
        name="display stand is seated on the register body",
    )

    closed_display_aabb = ctx.part_element_world_aabb(display, elem="screen_panel")
    with ctx.pose({display_joint: DISPLAY_TILT}):
        tilted_display_aabb = ctx.part_element_world_aabb(display, elem="screen_panel")

    ctx.check(
        "display tilts upward",
        closed_display_aabb is not None
        and tilted_display_aabb is not None
        and ((tilted_display_aabb[0][2] + tilted_display_aabb[1][2]) * 0.5)
        > ((closed_display_aabb[0][2] + closed_display_aabb[1][2]) * 0.5) + 0.002,
        details=f"closed={closed_display_aabb}, tilted={tilted_display_aabb}",
    )

    dept_rest = ctx.part_world_position(dept_key)
    with ctx.pose({dept_joint: 0.0015}):
        dept_pressed = ctx.part_world_position(dept_key)
    ctx.check(
        "department key depresses",
        dept_rest is not None
        and dept_pressed is not None
        and dept_pressed[2] < dept_rest[2] - 0.001,
        details=f"rest={dept_rest}, pressed={dept_pressed}",
    )

    number_rest = ctx.part_world_position(number_key)
    with ctx.pose({number_joint: 0.0012}):
        number_pressed = ctx.part_world_position(number_key)
    ctx.check(
        "number key depresses",
        number_rest is not None
        and number_pressed is not None
        and number_pressed[2] < number_rest[2] - 0.0008,
        details=f"rest={number_rest}, pressed={number_pressed}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_RADIUS = 0.155
BASE_TOP_Z = 0.052

BODY_HEIGHT = 0.900
BODY_WIDTH_BOTTOM = 0.128
BODY_DEPTH_BOTTOM = 0.166
BODY_WIDTH_MID = 0.136
BODY_DEPTH_MID = 0.174
BODY_WIDTH_TOP = 0.148
BODY_DEPTH_TOP = 0.186
BODY_WALL = 0.0035
BODY_BOTTOM_FLOOR = 0.032
BODY_TOP_WALL = 0.020

FRONT_GRILLE_SIZE = (0.086, 0.730)
FRONT_OPENING_SIZE = (0.072, 0.702)
FRONT_OPENING_Z = 0.468

REAR_INTAKE_SIZE = (0.094, 0.410)
REAR_OPENING_SIZE = (0.078, 0.382)
REAR_OPENING_Z = 0.528

TOP_PANEL_SIZE = (0.118, 0.082, 0.003)
TOP_PANEL_CENTER = (0.010, 0.000, BODY_HEIGHT - 0.002)

DIAL_ORIGIN_Z = BODY_HEIGHT
BUTTON_ORIGIN_Z = BODY_HEIGHT
BUTTON_TRAVEL = 0.0012

SPEED_DIAL_POS = (-0.026, 0.013, DIAL_ORIGIN_Z)
TIMER_DIAL_POS = (0.027, 0.013, DIAL_ORIGIN_Z)
BUTTON_POS = (0.046, -0.017, BUTTON_ORIGIN_Z)

BLOWER_CENTER_Z = 0.468
BLOWER_RADIUS = 0.046
BLOWER_WIDTH = 0.650


def _tower_loft(
    bottom_width: float,
    bottom_depth: float,
    mid_width: float,
    mid_depth: float,
    top_width: float,
    top_depth: float,
    height: float,
):
    return (
        cq.Workplane("XY")
        .rect(bottom_width, bottom_depth)
        .workplane(offset=height * 0.46)
        .rect(mid_width, mid_depth)
        .workplane(offset=height * 0.54)
        .rect(top_width, top_depth)
        .loft(combine=True)
    )


def _build_body_shell():
    outer = _tower_loft(
        BODY_WIDTH_BOTTOM,
        BODY_DEPTH_BOTTOM,
        BODY_WIDTH_MID,
        BODY_DEPTH_MID,
        BODY_WIDTH_TOP,
        BODY_DEPTH_TOP,
        BODY_HEIGHT,
    )
    try:
        outer = outer.edges("|Z").fillet(0.021)
    except Exception:
        pass

    inner = _tower_loft(
        BODY_WIDTH_BOTTOM - 2.0 * BODY_WALL,
        BODY_DEPTH_BOTTOM - 2.0 * BODY_WALL,
        BODY_WIDTH_MID - 2.0 * BODY_WALL,
        BODY_DEPTH_MID - 2.0 * BODY_WALL,
        BODY_WIDTH_TOP - 2.0 * BODY_WALL,
        BODY_DEPTH_TOP - 2.0 * BODY_WALL,
        BODY_HEIGHT - BODY_BOTTOM_FLOOR - BODY_TOP_WALL,
    ).translate((0.0, 0.0, BODY_BOTTOM_FLOOR))

    shell = outer.cut(inner)

    front_cut = cq.Workplane("XY").box(
        FRONT_OPENING_SIZE[0],
        0.088,
        FRONT_OPENING_SIZE[1],
    ).translate((0.0, BODY_DEPTH_TOP * 0.33, FRONT_OPENING_Z))
    rear_cut = cq.Workplane("XY").box(
        REAR_OPENING_SIZE[0],
        0.072,
        REAR_OPENING_SIZE[1],
    ).translate((0.0, -BODY_DEPTH_TOP * 0.31, REAR_OPENING_Z))
    return shell.cut(front_cut).cut(rear_cut)


def _dial_mesh(name: str, *, diameter: float, height: float, skirt_diameter: float):
    return mesh_from_geometry(
        KnobGeometry(
            diameter,
            height,
            body_style="skirted",
            top_diameter=diameter * 0.82,
            skirt=KnobSkirt(skirt_diameter, height * 0.24, flare=0.05),
            grip=KnobGrip(style="fluted", count=14, depth=0.0008),
            center=False,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    housing_white = model.material("housing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    blower_dark = model.material("blower_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=housing_white,
        name="foot_plate",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=housing_white,
        name="base_mass",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=trim_silver,
        name="turntable_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, BASE_TOP_Z)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z * 0.5)),
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "tower_fan_body_shell"),
        material=housing_white,
        name="shell",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                FRONT_GRILLE_SIZE,
                0.003,
                slot_size=(0.058, 0.0046),
                pitch=(0.0094, 0.066),
                frame=0.006,
                corner_radius=0.018,
                slot_angle_deg=89.0,
                center=True,
            ),
            "tower_fan_front_grille",
        ),
        origin=Origin(
            xyz=(0.0, BODY_DEPTH_TOP * 0.47, FRONT_OPENING_Z),
            rpy=(3.141592653589793 / 2.0, 0.0, 0.0),
        ),
        material=grille_dark,
        name="front_grille",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                REAR_INTAKE_SIZE,
                0.0028,
                slot_size=(0.042, 0.006),
                pitch=(0.012, 0.050),
                frame=0.007,
                corner_radius=0.012,
                slot_angle_deg=89.0,
                center=True,
            ),
            "tower_fan_rear_intake",
        ),
        origin=Origin(
            xyz=(0.0, -BODY_DEPTH_TOP * 0.45, REAR_OPENING_Z),
            rpy=(-3.141592653589793 / 2.0, 0.0, 0.0),
        ),
        material=grille_dark,
        name="rear_intake",
    )
    body.visual(
        Box(TOP_PANEL_SIZE),
        origin=Origin(xyz=TOP_PANEL_CENTER),
        material=panel_dark,
        name="top_panel",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.001),
        origin=Origin(xyz=(SPEED_DIAL_POS[0], SPEED_DIAL_POS[1], BODY_HEIGHT - 0.0005)),
        material=trim_silver,
        name="speed_bezel",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.001),
        origin=Origin(xyz=(TIMER_DIAL_POS[0], TIMER_DIAL_POS[1], BODY_HEIGHT - 0.0005)),
        material=trim_silver,
        name="timer_bezel",
    )
    body.visual(
        Box((0.022, 0.016, 0.001)),
        origin=Origin(xyz=(BUTTON_POS[0], BUTTON_POS[1], BODY_HEIGHT - 0.0005)),
        material=trim_silver,
        name="button_bezel",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim_silver,
        name="turntable_collar",
    )
    body.visual(
        Box((0.018, 0.022, BLOWER_WIDTH + 0.080)),
        origin=Origin(xyz=(0.0, -0.079, BLOWER_CENTER_Z)),
        material=panel_dark,
        name="blower_spine",
    )
    body.visual(
        Box((0.010, 0.068, 0.002)),
        origin=Origin(xyz=(0.0, -0.034, BLOWER_CENTER_Z - (BLOWER_WIDTH * 0.5) - 0.001)),
        material=panel_dark,
        name="lower_support_arm",
    )
    body.visual(
        Box((0.010, 0.068, 0.002)),
        origin=Origin(xyz=(0.0, -0.034, BLOWER_CENTER_Z + (BLOWER_WIDTH * 0.5) + 0.001)),
        material=panel_dark,
        name="upper_support_arm",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.190, 0.200, BODY_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                BLOWER_RADIUS,
                0.019,
                BLOWER_WIDTH,
                28,
                blade_thickness=0.0022,
                blade_sweep_deg=29.0,
                backplate=True,
                shroud=True,
            ),
            "tower_fan_blower",
        ),
        material=blower_dark,
        name="wheel",
    )
    blower.visual(
        Cylinder(radius=0.006, length=BLOWER_WIDTH + 0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_silver,
        name="shaft",
    )
    blower.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -(BLOWER_WIDTH * 0.5) + 0.007)),
        material=blower_dark,
        name="lower_hub",
    )
    blower.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, (BLOWER_WIDTH * 0.5) - 0.007)),
        material=blower_dark,
        name="upper_hub",
    )
    blower.visual(
        Box((0.006, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, BLOWER_RADIUS - 0.006, -(BLOWER_WIDTH * 0.5) + 0.020)),
        material=trim_silver,
        name="index_tab",
    )
    blower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=BLOWER_WIDTH + 0.020),
        mass=0.65,
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        _dial_mesh(
            "tower_fan_speed_dial",
            diameter=0.034,
            height=0.015,
            skirt_diameter=0.040,
        ),
        material=knob_dark,
        name="dial",
    )
    speed_dial.visual(
        Box((0.0022, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, 0.010, 0.0156)),
        material=trim_silver,
        name="speed_pointer",
    )
    speed_dial.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.017)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        _dial_mesh(
            "tower_fan_timer_dial",
            diameter=0.038,
            height=0.017,
            skirt_diameter=0.044,
        ),
        material=knob_dark,
        name="dial",
    )
    timer_dial.visual(
        Box((0.0022, 0.011, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0115, 0.0175)),
        material=trim_silver,
        name="timer_pointer",
    )
    timer_dial.inertial = Inertial.from_geometry(
        Box((0.044, 0.044, 0.019)),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
    )

    oscillation_button = model.part("oscillation_button")
    oscillation_button.visual(
        Box((0.018, 0.012, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=knob_dark,
        name="button_cap",
    )
    oscillation_button.visual(
        Box((0.010, 0.007, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0053)),
        material=trim_silver,
        name="button_ridge",
    )
    oscillation_button.inertial = Inertial.from_geometry(
        Box((0.018, 0.012, 0.006)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.7,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, BLOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=28.0,
        ),
    )
    model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=SPEED_DIAL_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=6.0,
        ),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=TIMER_DIAL_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=6.0,
        ),
    )
    model.articulation(
        "body_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=oscillation_button,
        origin=Origin(xyz=BUTTON_POS),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.03,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    speed_dial = object_model.get_part("speed_dial")
    timer_dial = object_model.get_part("timer_dial")
    oscillation_button = object_model.get_part("oscillation_button")

    body_joint = object_model.get_articulation("base_to_body")
    blower_joint = object_model.get_articulation("body_to_blower")
    speed_joint = object_model.get_articulation("body_to_speed_dial")
    timer_joint = object_model.get_articulation("body_to_timer_dial")
    button_joint = object_model.get_articulation("body_to_oscillation_button")

    ctx.expect_overlap(
        body,
        base,
        axes="xy",
        min_overlap=0.10,
        name="body remains centered over the weighted base",
    )
    ctx.expect_gap(
        body,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="oscillating body seats on the base turntable",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        margin=0.006,
        name="blower wheel stays inside the tower body footprint",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        min_overlap=0.58,
        name="blower wheel spans the tall outlet region",
    )
    ctx.expect_gap(
        speed_dial,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="speed dial sits just above the fan cap",
    )
    ctx.expect_gap(
        timer_dial,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="timer dial sits just above the fan cap",
    )
    ctx.expect_gap(
        oscillation_button,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="oscillation button is supported above the top deck",
    )

    body_limits = body_joint.motion_limits
    if body_limits is not None and body_limits.upper is not None:
        rest_front = _aabb_center(ctx.part_element_world_aabb(body, elem="front_grille"))
        with ctx.pose({body_joint: body_limits.upper}):
            swung_front = _aabb_center(ctx.part_element_world_aabb(body, elem="front_grille"))
        ctx.check(
            "body oscillation swings the outlet sideways",
            rest_front is not None
            and swung_front is not None
            and swung_front[0] < rest_front[0] - 0.04
            and swung_front[1] < rest_front[1] - 0.02,
            details=f"rest={rest_front}, swung={swung_front}",
        )

    rest_blower_tab = _aabb_center(ctx.part_element_world_aabb(blower, elem="index_tab"))
    with ctx.pose({blower_joint: 1.15}):
        spun_blower_tab = _aabb_center(ctx.part_element_world_aabb(blower, elem="index_tab"))
    ctx.check(
        "blower wheel visibly rotates on the centerline",
        rest_blower_tab is not None
        and spun_blower_tab is not None
        and (
            abs(spun_blower_tab[0] - rest_blower_tab[0]) > 0.012
            or abs(spun_blower_tab[1] - rest_blower_tab[1]) > 0.012
        ),
        details=f"rest={rest_blower_tab}, spun={spun_blower_tab}",
    )

    rest_speed_pointer = _aabb_center(ctx.part_element_world_aabb(speed_dial, elem="speed_pointer"))
    with ctx.pose({speed_joint: 1.05}):
        turned_speed_pointer = _aabb_center(
            ctx.part_element_world_aabb(speed_dial, elem="speed_pointer")
        )
    ctx.check(
        "speed dial rotates as a discrete top control",
        rest_speed_pointer is not None
        and turned_speed_pointer is not None
        and (
            abs(turned_speed_pointer[0] - rest_speed_pointer[0]) > 0.008
            or abs(turned_speed_pointer[1] - rest_speed_pointer[1]) > 0.008
        ),
        details=f"rest={rest_speed_pointer}, turned={turned_speed_pointer}",
    )

    rest_timer_pointer = _aabb_center(ctx.part_element_world_aabb(timer_dial, elem="timer_pointer"))
    with ctx.pose({timer_joint: -0.95}):
        turned_timer_pointer = _aabb_center(
            ctx.part_element_world_aabb(timer_dial, elem="timer_pointer")
        )
    ctx.check(
        "timer dial rotates independently of the speed dial",
        rest_timer_pointer is not None
        and turned_timer_pointer is not None
        and (
            abs(turned_timer_pointer[0] - rest_timer_pointer[0]) > 0.008
            or abs(turned_timer_pointer[1] - rest_timer_pointer[1]) > 0.008
        ),
        details=f"rest={rest_timer_pointer}, turned={turned_timer_pointer}",
    )

    rest_button_pos = ctx.part_world_position(oscillation_button)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed_button_pos = ctx.part_world_position(oscillation_button)
    ctx.check(
        "oscillation button presses downward independently",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0010,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()

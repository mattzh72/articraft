from __future__ import annotations

import math

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


CABINET_WIDTH = 0.56
CABINET_DEPTH = 0.57
BODY_HEIGHT = 0.87
TOTAL_HEIGHT = 0.93
WALL_THICKNESS = 0.012
TOP_THICKNESS = 0.020

OPENING_WIDTH = 0.37
OPENING_DEPTH = 0.31
OPENING_CENTER_Y = -0.05
OPENING_CORNER_RADIUS = 0.045

LID_WIDTH = 0.40
LID_DEPTH = 0.34
LID_THICKNESS = 0.018
LID_HINGE_Y = OPENING_CENTER_Y + (OPENING_DEPTH * 0.5) + 0.005
LID_HINGE_Z = BODY_HEIGHT + (LID_THICKNESS * 0.5) + 0.0015

CONSOLE_WIDTH = CABINET_WIDTH - 0.030
CONSOLE_FRONT_Y = 0.135
CONSOLE_TOP_FRONT_Y = 0.210
CONSOLE_ROLL = math.atan2(TOTAL_HEIGHT - BODY_HEIGHT, CONSOLE_TOP_FRONT_Y - CONSOLE_FRONT_Y)
CONSOLE_ROLL_DEG = math.degrees(CONSOLE_ROLL)

TUB_RIM_Z = 0.79
TUB_OUTER_RADIUS = 0.170
TUB_INNER_RADIUS = 0.156
TUB_DEPTH = 0.42
TUB_BOTTOM_THICKNESS = 0.012
AGITATOR_BASE_RADIUS = 0.052
AGITATOR_TOP_RADIUS = 0.036
AGITATOR_HEIGHT = 0.32

DIAL_X = -0.120
DIAL_S = 0.034
MODE_BUTTON_0_X = 0.040
MODE_BUTTON_1_X = 0.088
MODE_BUTTON_S = 0.030
START_BUTTON_X = 0.170
START_BUTTON_S = 0.033
BUTTON_TRAVEL = 0.0032
CONTROL_OUTWARD = 0.0008
BUTTON_POCKET_DEPTH = 0.005
DIAL_HOLE_DEPTH = 0.015


def _console_surface_xyz(x: float, s: float) -> tuple[float, float, float]:
    return (
        x,
        CONSOLE_FRONT_Y + (s * math.cos(CONSOLE_ROLL)),
        BODY_HEIGHT + (s * math.sin(CONSOLE_ROLL)),
    )


def _console_origin(x: float, s: float, outward: float = 0.0) -> Origin:
    base_x, base_y, base_z = _console_surface_xyz(x, s)
    return Origin(
        xyz=(
            base_x,
            base_y - (outward * math.sin(CONSOLE_ROLL)),
            base_z + (outward * math.cos(CONSOLE_ROLL)),
        ),
        rpy=(CONSOLE_ROLL, 0.0, 0.0),
    )


def _console_plane() -> cq.Workplane:
    return cq.Workplane("XY").transformed(
        offset=(0.0, CONSOLE_FRONT_Y, BODY_HEIGHT),
        rotate=(CONSOLE_ROLL_DEG, 0.0, 0.0),
    )


def _build_cabinet_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(CABINET_WIDTH, CABINET_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .cut(
            cq.Workplane("XY").box(
                CABINET_WIDTH - (2.0 * WALL_THICKNESS),
                CABINET_DEPTH - (2.0 * WALL_THICKNESS),
                BODY_HEIGHT - TOP_THICKNESS,
                centered=(True, True, False),
            )
        )
    )

    opening_cutter = (
        cq.Workplane("XY")
        .box(OPENING_WIDTH, OPENING_DEPTH, TOP_THICKNESS + 0.060)
        .edges("|Z")
        .fillet(OPENING_CORNER_RADIUS)
        .translate((0.0, OPENING_CENTER_Y, BODY_HEIGHT - (TOP_THICKNESS * 0.5)))
    )
    shell = shell.cut(opening_cutter)

    console = (
        cq.Workplane("YZ")
        .polyline(
            [
                (CONSOLE_FRONT_Y, BODY_HEIGHT),
                (CONSOLE_TOP_FRONT_Y, TOTAL_HEIGHT),
                (CABINET_DEPTH * 0.5, TOTAL_HEIGHT),
                (CABINET_DEPTH * 0.5, BODY_HEIGHT),
            ]
        )
        .close()
        .extrude(CONSOLE_WIDTH)
        .translate((-(CONSOLE_WIDTH * 0.5), 0.0, 0.0))
    )

    dial_shaft_hole = _console_plane().center(DIAL_X, DIAL_S).circle(0.0045).extrude(-DIAL_HOLE_DEPTH)
    mode_pocket_0 = _console_plane().center(MODE_BUTTON_0_X, MODE_BUTTON_S).rect(0.022, 0.010).extrude(-BUTTON_POCKET_DEPTH)
    mode_pocket_1 = _console_plane().center(MODE_BUTTON_1_X, MODE_BUTTON_S).rect(0.022, 0.010).extrude(-BUTTON_POCKET_DEPTH)
    start_pocket = _console_plane().center(START_BUTTON_X, START_BUTTON_S).circle(0.0115).extrude(-BUTTON_POCKET_DEPTH)
    console = console.cut(dial_shaft_hole).cut(mode_pocket_0).cut(mode_pocket_1).cut(start_pocket)

    return shell.union(console)


def _build_lid_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(0.0, -(LID_DEPTH * 0.5))
        .box(LID_WIDTH, LID_DEPTH, LID_THICKNESS)
        .edges("|Z")
        .fillet(0.006)
    )


def _build_tub_shape() -> cq.Workplane:
    basket = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_RADIUS)
        .extrude(-TUB_DEPTH)
        .cut(cq.Workplane("XY").circle(TUB_INNER_RADIUS).extrude(-(TUB_DEPTH - TUB_BOTTOM_THICKNESS)))
    )

    rim = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_RADIUS + 0.008)
        .circle(TUB_INNER_RADIUS - 0.002)
        .extrude(-0.014)
    )
    basket = basket.union(rim)

    agitator_base_z = -TUB_DEPTH + TUB_BOTTOM_THICKNESS
    agitator = (
        cq.Workplane("XY", origin=(0.0, 0.0, agitator_base_z))
        .circle(AGITATOR_BASE_RADIUS)
        .extrude(AGITATOR_HEIGHT - 0.070)
        .faces(">Z")
        .circle(AGITATOR_TOP_RADIUS)
        .extrude(0.070)
    )

    for angle in (0.0, 90.0, 180.0, 270.0):
        fin = (
            cq.Workplane("XY")
            .box(0.014, 0.090, 0.160)
            .translate((0.0, 0.040, agitator_base_z + 0.120))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        agitator = agitator.union(fin)

    return basket.union(agitator)


def _build_timer_dial_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.004)
        .faces(">Z")
        .circle(0.024)
        .extrude(0.018)
        .faces(">Z")
        .workplane()
        .center(0.0, 0.012)
        .rect(0.004, 0.014)
        .extrude(0.002)
    )


def _build_softener_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.031)
        .extrude(0.014)
        .faces(">Z")
        .circle(0.025)
        .extrude(0.010)
        .faces(">Z")
        .workplane()
        .center(0.012, 0.0)
        .rect(0.010, 0.004)
        .extrude(0.002)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    enamel_white = model.material("enamel_white", rgba=(0.97, 0.97, 0.98, 1.0))
    console_white = model.material("console_white", rgba=(0.93, 0.94, 0.95, 1.0))
    tub_grey = model.material("tub_grey", rgba=(0.73, 0.76, 0.80, 1.0))
    agitator_white = model.material("agitator_white", rgba=(0.95, 0.96, 0.97, 1.0))
    control_grey = model.material("control_grey", rgba=(0.80, 0.82, 0.84, 1.0))
    control_dark = model.material("control_dark", rgba=(0.34, 0.37, 0.41, 1.0))
    cap_blue = model.material("cap_blue", rgba=(0.61, 0.76, 0.90, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shape(), "washer_cabinet"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - (2.0 * WALL_THICKNESS), CABINET_DEPTH - (2.0 * WALL_THICKNESS), 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=console_white,
        name="base_pan",
    )
    cabinet.visual(
        Cylinder(radius=0.055, length=TUB_RIM_Z - TUB_DEPTH - 0.018),
        origin=Origin(
            xyz=(
                0.0,
                OPENING_CENTER_Y,
                0.018 + ((TUB_RIM_Z - TUB_DEPTH - 0.018) * 0.5),
            )
        ),
        material=control_grey,
        name="drive_pedestal",
    )
    cabinet.visual(
        Box((0.34, 0.020, 0.0015)),
        origin=Origin(xyz=(0.0, LID_HINGE_Y - 0.010, BODY_HEIGHT + 0.00075)),
        material=console_white,
        name="hinge_seat",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "washer_lid"),
        material=enamel_white,
        name="lid_panel",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_build_tub_shape(), "washer_tub"),
        material=tub_grey,
        name="basket",
    )
    tub.visual(
        Cylinder(radius=0.056, length=AGITATOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, -TUB_DEPTH + TUB_BOTTOM_THICKNESS + (AGITATOR_HEIGHT * 0.5))),
        material=agitator_white,
        name="agitator_core",
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, TUB_RIM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=8.0),
    )

    softener_cap = model.part("softener_cap")
    softener_cap.visual(
        mesh_from_cadquery(_build_softener_cap_shape(), "softener_cap"),
        material=cap_blue,
        name="softener_cap",
    )
    model.articulation(
        "tub_to_softener_cap",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=softener_cap,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -TUB_DEPTH + TUB_BOTTOM_THICKNESS + AGITATOR_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_cadquery(_build_timer_dial_shape(), "timer_dial"),
        material=control_dark,
        name="dial",
    )
    timer_dial.visual(
        Cylinder(radius=0.004, length=DIAL_HOLE_DEPTH + CONTROL_OUTWARD),
        origin=Origin(xyz=(0.0, 0.0, -((DIAL_HOLE_DEPTH + CONTROL_OUTWARD) * 0.5))),
        material=control_dark,
        name="dial_shaft",
    )
    model.articulation(
        "cabinet_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=timer_dial,
        origin=_console_origin(DIAL_X, DIAL_S, outward=CONTROL_OUTWARD),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.010, length=BUTTON_POCKET_DEPTH + CONTROL_OUTWARD),
        origin=Origin(xyz=(0.0, 0.0, -((BUTTON_POCKET_DEPTH + CONTROL_OUTWARD) * 0.5))),
        material=control_dark,
        name="button_stem",
    )
    start_button.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=control_grey,
        name="button_cap",
    )
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=_console_origin(START_BUTTON_X, START_BUTTON_S, outward=CONTROL_OUTWARD),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )

    for name, x_pos in (("mode_button_0", MODE_BUTTON_0_X), ("mode_button_1", MODE_BUTTON_1_X)):
        button = model.part(name)
        button.visual(
            Box((0.018, 0.008, BUTTON_POCKET_DEPTH + CONTROL_OUTWARD)),
            origin=Origin(xyz=(0.0, 0.0, -((BUTTON_POCKET_DEPTH + CONTROL_OUTWARD) * 0.5))),
            material=control_dark,
            name="button_stem",
        )
        button.visual(
            Box((0.028, 0.016, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=console_white,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=_console_origin(x_pos, MODE_BUTTON_S, outward=CONTROL_OUTWARD),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def _distance(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.dist(a, b)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    softener_cap = object_model.get_part("softener_cap")
    timer_dial = object_model.get_part("timer_dial")
    start_button = object_model.get_part("start_button")
    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    cap_spin = object_model.get_articulation("tub_to_softener_cap")
    dial_spin = object_model.get_articulation("cabinet_to_timer_dial")
    start_button_slide = object_model.get_articulation("cabinet_to_start_button")
    mode_button_0_slide = object_model.get_articulation("cabinet_to_mode_button_0")
    mode_button_1_slide = object_model.get_articulation("cabinet_to_mode_button_1")

    ctx.allow_overlap(
        cabinet,
        timer_dial,
        elem_a="cabinet_shell",
        elem_b="dial_shaft",
        reason="The timer dial uses a short mounting shaft that intentionally sits inside the console's dial bore.",
    )
    ctx.allow_overlap(
        cabinet,
        start_button,
        elem_a="cabinet_shell",
        elem_b="button_stem",
        reason="The start button stem intentionally slides into the console pocket behind the exposed cap.",
    )
    ctx.allow_overlap(
        cabinet,
        mode_button_0,
        elem_a="cabinet_shell",
        elem_b="button_stem",
        reason="The mode button stem intentionally slides into the console pocket behind the exposed cap.",
    )
    ctx.allow_overlap(
        cabinet,
        mode_button_1,
        elem_a="cabinet_shell",
        elem_b="button_stem",
        reason="The mode button stem intentionally slides into the console pocket behind the exposed cap.",
    )

    ctx.expect_overlap(
        tub,
        lid,
        axes="xy",
        min_overlap=0.28,
        name="tub stays centered under the lid footprint",
    )
    ctx.expect_gap(
        lid,
        tub,
        axis="z",
        min_gap=0.05,
        name="closed lid sits clearly above the deep basket",
    )
    ctx.expect_contact(
        timer_dial,
        cabinet,
        contact_tol=0.003,
        name="timer dial seats on the sloped console",
    )
    ctx.expect_contact(
        start_button,
        cabinet,
        contact_tol=0.0015,
        name="start button seats on the sloped console",
    )
    ctx.expect_contact(
        mode_button_0,
        cabinet,
        contact_tol=0.0015,
        name="first mode button seats on the sloped console",
    )
    ctx.expect_contact(
        mode_button_1,
        cabinet,
        contact_tol=0.0015,
        name="second mode button seats on the sloped console",
    )
    ctx.expect_within(
        softener_cap,
        tub,
        axes="xy",
        margin=0.02,
        name="softener cap stays centered inside the basket footprint",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    start_rest = ctx.part_world_position(start_button)
    with ctx.pose({start_button_slide: BUTTON_TRAVEL}):
        start_pressed = ctx.part_world_position(start_button)
    start_travel = _distance(start_rest, start_pressed)
    ctx.check(
        "start button depresses",
        start_travel is not None and start_travel > 0.0025,
        details=f"rest={start_rest}, pressed={start_pressed}, travel={start_travel}",
    )

    mode_0_rest = ctx.part_world_position(mode_button_0)
    with ctx.pose({mode_button_0_slide: BUTTON_TRAVEL}):
        mode_0_pressed = ctx.part_world_position(mode_button_0)
    mode_0_travel = _distance(mode_0_rest, mode_0_pressed)
    ctx.check(
        "first mode button depresses",
        mode_0_travel is not None and mode_0_travel > 0.0025,
        details=f"rest={mode_0_rest}, pressed={mode_0_pressed}, travel={mode_0_travel}",
    )

    mode_1_rest = ctx.part_world_position(mode_button_1)
    with ctx.pose({mode_button_1_slide: BUTTON_TRAVEL}):
        mode_1_pressed = ctx.part_world_position(mode_button_1)
    mode_1_travel = _distance(mode_1_rest, mode_1_pressed)
    ctx.check(
        "second mode button depresses",
        mode_1_travel is not None and mode_1_travel > 0.0025,
        details=f"rest={mode_1_rest}, pressed={mode_1_pressed}, travel={mode_1_travel}",
    )

    with ctx.pose({dial_spin: 1.4}):
        ctx.expect_contact(
            timer_dial,
            cabinet,
            contact_tol=0.003,
            name="timer dial stays mounted while rotated",
        )

    with ctx.pose({tub_spin: 1.1, cap_spin: 0.9}):
        ctx.expect_within(
            softener_cap,
            tub,
            axes="xy",
            margin=0.02,
            name="softener cap stays centered while the tub and cap rotate",
        )
        ctx.expect_gap(
            lid,
            tub,
            axis="z",
            min_gap=0.05,
            name="rotating basket remains visibly below the lid plane",
        )

    return ctx.report()


object_model = build_object_model()

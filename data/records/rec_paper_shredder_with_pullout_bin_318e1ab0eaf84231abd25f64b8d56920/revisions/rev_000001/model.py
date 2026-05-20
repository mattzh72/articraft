from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.42
BODY_D = 0.34
BODY_H = 0.82
BODY_WALL = 0.016
TOP_WALL = 0.028

BIN_W = 0.355
BIN_D = 0.275
BIN_H = 0.365
BIN_WALL = 0.012
BIN_TRAVEL = 0.21
BIN_BOTTOM_Z = 0.055

FLAP_W = 0.305
FLAP_H = 0.120
FLAP_T = 0.014
FLAP_BOTTOM_Z = 0.555

PANEL_ANGLE = math.radians(22.0)
PANEL_CENTER = (0.0, 0.075, 0.764)
PANEL_SIZE = (0.350, 0.145, 0.004)


def _build_body_shell() -> object:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    shell = shell.cut(
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * BODY_WALL,
            BODY_D - 2.0 * BODY_WALL,
            BODY_H - BODY_WALL - TOP_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_WALL))
    )

    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.374, 0.090, 0.410, centered=(True, True, False))
        .translate((0.0, BODY_D / 2.0 - 0.045, BIN_BOTTOM_Z))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.318, 0.070, 0.132, centered=(True, True, False))
        .translate((0.0, BODY_D / 2.0 - 0.035, FLAP_BOTTOM_Z))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.270, 0.026, 0.050, centered=(True, True, False))
        .translate((0.0, -0.008, BODY_H - 0.050))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.320, 0.090, 0.090, centered=(True, True, False))
        .translate((0.0, 0.010, BODY_H - 0.090))
    )
    return shell


def _build_bin_shell() -> object:
    tray = (
        cq.Workplane("XY")
        .box(BIN_W, BIN_D, BIN_H, centered=(True, True, False))
        .translate((0.0, -BIN_D / 2.0, 0.0))
    )
    tray = tray.cut(
        cq.Workplane("XY")
        .box(BIN_W - 2.0 * BIN_WALL, BIN_D - 2.0 * BIN_WALL, BIN_H - BIN_WALL, centered=(True, True, False))
        .translate((0.0, -BIN_D / 2.0, BIN_WALL))
    )
    tray = tray.cut(
        cq.Workplane("XY")
        .box(0.122, 0.008, 0.048, centered=(True, True, False))
        .translate((0.0, -0.004, 0.145))
    )
    tray = tray.union(
        cq.Workplane("XY")
        .box(BIN_W, 0.018, 0.050, centered=(True, True, False))
        .translate((0.0, -0.009, BIN_H - 0.050))
    )
    return tray


def _panel_mount(local_x: float, local_y: float, *, surface_z: float = PANEL_SIZE[2] / 2.0) -> Origin:
    sin_a = math.sin(PANEL_ANGLE)
    cos_a = math.cos(PANEL_ANGLE)
    return Origin(
        xyz=(
            PANEL_CENTER[0] + local_x,
            PANEL_CENTER[1] + local_y * cos_a + surface_z * sin_a,
            PANEL_CENTER[2] - local_y * sin_a + surface_z * cos_a,
        ),
        rpy=(-PANEL_ANGLE, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_shredder")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    body_mid = model.material("body_mid", rgba=(0.24, 0.25, 0.28, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    button_blue = model.material("button_blue", rgba=(0.19, 0.35, 0.58, 1.0))
    button_green = model.material("button_green", rgba=(0.23, 0.49, 0.30, 1.0))
    button_amber = model.material("button_amber", rgba=(0.72, 0.49, 0.16, 1.0))
    button_red = model.material("button_red", rgba=(0.62, 0.20, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + BODY_WALL / 2.0, 0.0, BODY_H / 2.0)),
        material=body_dark,
        name="left_side",
    )
    body.visual(
        Box((BODY_WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - BODY_WALL / 2.0, 0.0, BODY_H / 2.0)),
        material=body_dark,
        name="right_side",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + BODY_WALL / 2.0, BODY_H / 2.0)),
        material=body_dark,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_D - 0.080, BODY_WALL)),
        origin=Origin(xyz=(0.0, -0.040, BODY_WALL / 2.0)),
        material=body_dark,
        name="base_floor",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_D, TOP_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_WALL / 2.0)),
        material=body_dark,
        name="top_roof",
    )
    body.visual(
        Box((0.384, BODY_WALL, 0.120)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - BODY_WALL / 2.0, 0.705)),
        material=body_dark,
        name="upper_front_band",
    )
    body.visual(
        Box(PANEL_SIZE),
        origin=Origin(xyz=PANEL_CENTER, rpy=(-PANEL_ANGLE, 0.0, 0.0)),
        material=body_mid,
        name="front_fascia",
    )
    body.visual(
        Box((0.320, 0.112, 0.008)),
        origin=Origin(xyz=(0.0, 0.075, BODY_H - 0.004)),
        material=body_mid,
        name="control_panel",
    )
    body.visual(
        Box((0.388, BODY_WALL, 0.012)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - BODY_WALL / 2.0, BIN_BOTTOM_Z + BIN_H + 0.010)),
        material=trim_dark,
        name="bin_header",
    )
    body.visual(
        Box((0.040, BODY_WALL, 0.132)),
        origin=Origin(xyz=(-0.179, BODY_D / 2.0 - BODY_WALL / 2.0, FLAP_BOTTOM_Z + 0.066)),
        material=body_dark,
        name="flap_jamb_0",
    )
    body.visual(
        Box((0.040, BODY_WALL, 0.132)),
        origin=Origin(xyz=(0.179, BODY_D / 2.0 - BODY_WALL / 2.0, FLAP_BOTTOM_Z + 0.066)),
        material=body_dark,
        name="flap_jamb_1",
    )
    body.visual(
        Box((0.286, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.010, BODY_H - 0.028)),
        material=trim_dark,
        name="feed_slot_trim",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_build_bin_shell(), "shredder_bin"),
        material=body_mid,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.110, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.005, 0.145)),
        material=trim_dark,
        name="bin_handle_trim",
    )
    bin_part.visual(
        Box((0.0165, 0.210, 0.020)),
        origin=Origin(xyz=(-0.18575, -0.135, 0.185)),
        material=trim_dark,
        name="runner_0",
    )
    bin_part.visual(
        Box((0.0165, 0.210, 0.020)),
        origin=Origin(xyz=(0.18575, -0.135, 0.185)),
        material=trim_dark,
        name="runner_1",
    )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, BIN_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=BIN_TRAVEL,
        ),
    )

    flap = model.part("access_flap")
    flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, FLAP_T / 2.0, FLAP_H / 2.0)),
        material=body_mid,
        name="door_panel",
    )
    flap.visual(
        Cylinder(radius=0.004, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.150, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.018, 0.088)),
        material=trim_dark,
        name="door_pull",
    )

    model.articulation(
        "body_to_access_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, FLAP_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    for drum_index, drum_y in enumerate((-0.024, 0.024)):
        drum = model.part(f"cutter_drum_{drum_index}")
        drum.visual(
            Cylinder(radius=0.014, length=0.332),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="drum_core",
        )
        drum.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_dark,
            name="journal_0",
        )
        drum.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_dark,
            name="journal_1",
        )
        for cutter_index, cutter_x in enumerate((-0.120, -0.072, -0.024, 0.024, 0.072, 0.120)):
            drum.visual(
                Cylinder(radius=0.022, length=0.018),
                origin=Origin(xyz=(cutter_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"cutter_{cutter_index}",
            )

        model.articulation(
            f"body_to_cutter_drum_{drum_index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum,
            origin=Origin(xyz=(0.0, drum_y, 0.732)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=24.0,
            ),
        )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=trim_dark,
        name="dial_skirt",
    )
    timer_dial.visual(
        Cylinder(radius=0.029, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dial_dark,
        name="dial_body",
    )
    timer_dial.visual(
        Box((0.004, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.010, 0.027)),
        material=steel,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(-0.098, 0.075, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
        ),
    )

    button_specs = [
        ("program_button_0", -0.020, button_blue),
        ("program_button_1", 0.030, button_green),
        ("program_button_2", 0.080, button_amber),
        ("program_button_3", 0.130, button_red),
    ]
    for button_name, local_x, button_material in button_specs:
        button = model.part(button_name)
        button.visual(
            Box((0.038, 0.022, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=button_material,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(local_x, 0.075, BODY_H)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.06,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    flap = object_model.get_part("access_flap")
    timer_dial = object_model.get_part("timer_dial")
    drum_0 = object_model.get_part("cutter_drum_0")
    drum_1 = object_model.get_part("cutter_drum_1")
    bin_joint = object_model.get_articulation("body_to_bin")
    flap_joint = object_model.get_articulation("body_to_access_flap")
    button_joints = [object_model.get_articulation(f"body_to_program_button_{index}") for index in range(4)]
    button_parts = [object_model.get_part(f"program_button_{index}") for index in range(4)]

    ctx.expect_overlap(
        bin_part,
        body,
        axes="xz",
        min_overlap=0.20,
        name="bin stays aligned within the cabinet opening",
    )

    with ctx.pose({bin_joint: 0.0}):
        ctx.expect_overlap(
            bin_part,
            body,
            axes="y",
            min_overlap=0.16,
            name="closed bin remains deeply inserted",
        )

    with ctx.pose({bin_joint: BIN_TRAVEL}):
        ctx.expect_overlap(
            bin_part,
            body,
            axes="y",
            min_overlap=0.05,
            name="extended bin retains insertion",
        )
        rest_pos = ctx.part_world_position(bin_part)

    with ctx.pose({bin_joint: 0.0}):
        closed_pos = ctx.part_world_position(bin_part)

    ctx.check(
        "bin extends forward",
        closed_pos is not None and rest_pos is not None and rest_pos[1] > closed_pos[1] + 0.16,
        details=f"closed={closed_pos}, extended={rest_pos}",
    )

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_gap(
            flap,
            body,
            axis="y",
            positive_elem="door_panel",
            max_gap=0.020,
            max_penetration=0.0,
            name="access flap closes near flush to the front face",
        )
        flap_closed = ctx.part_world_aabb(flap)

    with ctx.pose({flap_joint: 1.20}):
        flap_open = ctx.part_world_aabb(flap)

    flap_opens = False
    if flap_closed is not None and flap_open is not None:
        flap_opens = flap_open[1][1] > flap_closed[1][1] + 0.05 and flap_open[1][2] < flap_closed[1][2] - 0.05
    ctx.check(
        "access flap swings outward and downward",
        flap_opens,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    ctx.expect_gap(
        timer_dial,
        body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="timer dial seats on the top control panel",
    )

    for index, (button_part, button_joint) in enumerate(zip(button_parts, button_joints)):
        with ctx.pose({button_joint: 0.0}):
            rest_pos = ctx.part_world_position(button_part)
            ctx.expect_gap(
                button_part,
                body,
                axis="z",
                max_gap=0.002,
                max_penetration=0.0,
                name=f"program button {index} sits proud on the control panel",
            )
        with ctx.pose({button_joint: 0.002}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"program button {index} presses downward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.expect_origin_distance(
        drum_0,
        drum_1,
        axes="xz",
        max_dist=0.001,
        name="cutter drums share matched horizontal axis height and span",
    )
    ctx.expect_gap(
        drum_1,
        drum_0,
        axis="y",
        min_gap=0.003,
        max_gap=0.008,
        name="cutter drums keep a narrow working clearance",
    )

    return ctx.report()


object_model = build_object_model()

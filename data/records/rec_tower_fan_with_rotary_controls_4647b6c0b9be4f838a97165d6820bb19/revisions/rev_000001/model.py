from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_RADIUS = 0.165
BASE_HEIGHT = 0.044
COLLAR_RADIUS = 0.060
COLLAR_HEIGHT = 0.010

STEM_RADIUS = 0.048
STEM_HEIGHT = 0.026
COLUMN_Z0 = 0.018
COLUMN_WIDTH = 0.146
COLUMN_DEPTH = 0.108
COLUMN_HEIGHT = 0.900
BACK_SHELL_DEPTH = 0.030
SIDE_WALL_THICKNESS = 0.011
SIDE_WALL_DEPTH = 0.078
PLENUM_WIDTH = 0.124
PLENUM_DEPTH = 0.082
BOTTOM_PLENUM_HEIGHT = 0.095
TOP_PLENUM_Z0 = 0.865
TOP_PLENUM_HEIGHT = 0.080
OPENING_WIDTH = 0.108
OPENING_HEIGHT = 0.790
OPENING_Z0 = 0.108
FRAME_DEPTH = 0.012
FRAME_STRIP_WIDTH = 0.011
FRAME_RAIL_HEIGHT = 0.015
FRAME_Y = 0.043
TOP_POD_Z0 = 0.900
TOP_POD_HEIGHT = 0.046
TOP_POD_WIDTH = 0.152
TOP_POD_DEPTH = 0.114
TOP_POD_CORNER = 0.025
TOWER_HEIGHT = TOP_POD_Z0 + TOP_POD_HEIGHT

DIAL_X = 0.028
SELECTOR_X = -0.030
CONTROL_Y = 0.012
DIAL_BOSS_RADIUS = 0.027
SELECTOR_BOSS_RADIUS = 0.016
BOSS_HEIGHT = 0.003
CONTROL_TOP_Z = TOWER_HEIGHT + BOSS_HEIGHT

WHEEL_RADIUS = 0.031
WHEEL_INNER_RADIUS = 0.017
WHEEL_HEIGHT = 0.730
WHEEL_CENTER_Z = 0.500
WHEEL_CENTER_Y = 0.007

GRILLE_WIDTH = 0.100
GRILLE_HEIGHT = 0.782
GRILLE_DEPTH = 0.004
GRILLE_FRAME = 0.005
GRILLE_BAR_WIDTH = 0.0032
GRILLE_Y = 0.051
GRILLE_Z0 = 0.112


def _mesh(shape, name: str):
    return mesh_from_cadquery(shape, name)


def _rounded_box(width: float, depth: float, height: float, corner: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(corner)
    )


def _build_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(BASE_RADIUS).extrude(0.022)
    deck = cq.Workplane("XY").circle(0.138).extrude(0.012).translate((0.0, 0.0, 0.022))
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT - COLLAR_HEIGHT))
    )
    return foot.union(deck).union(collar)


def _build_tower_shape() -> cq.Workplane:
    stem = cq.Workplane("XY").circle(STEM_RADIUS).extrude(STEM_HEIGHT)
    back_shell = _rounded_box(COLUMN_WIDTH, BACK_SHELL_DEPTH, COLUMN_HEIGHT, 0.013).translate(
        (0.0, -0.039, COLUMN_Z0)
    )
    left_wall = (
        cq.Workplane("XY")
        .box(SIDE_WALL_THICKNESS, SIDE_WALL_DEPTH, COLUMN_HEIGHT, centered=(True, True, False))
        .translate((-(COLUMN_WIDTH - SIDE_WALL_THICKNESS) * 0.5, 0.004, COLUMN_Z0))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(SIDE_WALL_THICKNESS, SIDE_WALL_DEPTH, COLUMN_HEIGHT, centered=(True, True, False))
        .translate(((COLUMN_WIDTH - SIDE_WALL_THICKNESS) * 0.5, 0.004, COLUMN_Z0))
    )
    bottom_plenum = (
        cq.Workplane("XY")
        .box(PLENUM_WIDTH, PLENUM_DEPTH, BOTTOM_PLENUM_HEIGHT, centered=(True, True, False))
        .translate((0.0, -0.004, COLUMN_Z0))
    )
    top_plenum = (
        cq.Workplane("XY")
        .box(PLENUM_WIDTH, PLENUM_DEPTH, TOP_PLENUM_HEIGHT, centered=(True, True, False))
        .translate((0.0, -0.004, TOP_PLENUM_Z0))
    )
    pod = _rounded_box(TOP_POD_WIDTH, TOP_POD_DEPTH, TOP_POD_HEIGHT, TOP_POD_CORNER).translate(
        (0.0, 0.0, TOP_POD_Z0)
    )
    dial_boss = (
        cq.Workplane("XY")
        .circle(DIAL_BOSS_RADIUS)
        .extrude(BOSS_HEIGHT)
        .translate((DIAL_X, CONTROL_Y, TOWER_HEIGHT))
    )
    selector_boss = (
        cq.Workplane("XY")
        .circle(SELECTOR_BOSS_RADIUS)
        .extrude(BOSS_HEIGHT)
        .translate((SELECTOR_X, CONTROL_Y, TOWER_HEIGHT))
    )
    left_frame = (
        cq.Workplane("XY")
        .box(FRAME_STRIP_WIDTH, FRAME_DEPTH, OPENING_HEIGHT, centered=(True, True, False))
        .translate((-(OPENING_WIDTH - FRAME_STRIP_WIDTH) * 0.5, FRAME_Y, OPENING_Z0))
    )
    right_frame = (
        cq.Workplane("XY")
        .box(FRAME_STRIP_WIDTH, FRAME_DEPTH, OPENING_HEIGHT, centered=(True, True, False))
        .translate(((OPENING_WIDTH - FRAME_STRIP_WIDTH) * 0.5, FRAME_Y, OPENING_Z0))
    )
    lower_rail = (
        cq.Workplane("XY")
        .box(PLENUM_WIDTH, FRAME_DEPTH, FRAME_RAIL_HEIGHT, centered=(True, True, False))
        .translate((0.0, FRAME_Y, OPENING_Z0))
    )
    upper_rail = (
        cq.Workplane("XY")
        .box(PLENUM_WIDTH, FRAME_DEPTH, FRAME_RAIL_HEIGHT, centered=(True, True, False))
        .translate((0.0, FRAME_Y, OPENING_Z0 + OPENING_HEIGHT - FRAME_RAIL_HEIGHT))
    )
    return (
        stem.union(back_shell)
        .union(left_wall)
        .union(right_wall)
        .union(bottom_plenum)
        .union(top_plenum)
        .union(pod)
        .union(dial_boss)
        .union(selector_boss)
        .union(left_frame)
        .union(right_frame)
        .union(lower_rail)
        .union(upper_rail)
    )


def _build_grille_shape() -> cq.Workplane:
    outer = _rounded_box(GRILLE_WIDTH, GRILLE_DEPTH, GRILLE_HEIGHT, 0.0015).translate(
        (0.0, GRILLE_Y, GRILLE_Z0)
    )
    inner = _rounded_box(
        GRILLE_WIDTH - 2.0 * GRILLE_FRAME,
        GRILLE_DEPTH + 0.002,
        GRILLE_HEIGHT - 2.0 * GRILLE_FRAME,
        0.0012,
    ).translate((0.0, GRILLE_Y, GRILLE_Z0 + GRILLE_FRAME))
    grille = outer.cut(inner)
    bar_span = GRILLE_WIDTH - 2.0 * (GRILLE_FRAME + 0.004)
    bar_count = 11
    for index in range(bar_count):
        t = 0.0 if bar_count == 1 else index / (bar_count - 1)
        x = -bar_span * 0.5 + bar_span * t
        bar = (
            cq.Workplane("XY")
            .box(
                GRILLE_BAR_WIDTH,
                GRILLE_DEPTH,
                GRILLE_HEIGHT - 2.0 * GRILLE_FRAME + 0.002,
                centered=(True, True, False),
            )
            .translate((x, GRILLE_Y, GRILLE_Z0 + GRILLE_FRAME - 0.001))
        )
        grille = grille.union(bar)
    return grille


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    shell_white = model.material("shell_white", rgba=(0.90, 0.91, 0.92, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_grille = model.material("dark_grille", rgba=(0.11, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(_mesh(_build_base_shape(), "tower_fan_base"), material=graphite, name="base_shell")

    tower = model.part("tower")
    tower.visual(_mesh(_build_tower_shape(), "tower_fan_tower"), material=shell_white, name="housing")

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=WHEEL_RADIUS,
                inner_radius=WHEEL_INNER_RADIUS,
                width=WHEEL_HEIGHT,
                blade_count=34,
                blade_thickness=0.0018,
                blade_sweep_deg=30.0,
                backplate=True,
                shroud=True,
                center=True,
            ),
            "tower_fan_blower_wheel",
        ),
        material=dark_grille,
        name="wheel",
    )

    grille = model.part("grille")
    grille.visual(_mesh(_build_grille_shape(), "tower_fan_grille"), material=dark_grille, name="front_grille")

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="dial",
    )
    speed_dial.visual(
        Cylinder(radius=0.028, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=graphite,
        name="dial_skirt",
    )
    speed_dial.visual(
        Box((0.011, 0.003, 0.0022)),
        origin=Origin(xyz=(0.0105, 0.0, 0.0162)),
        material=shell_white,
        name="dial_marker",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.0125, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=graphite,
        name="selector",
    )
    selector_knob.visual(
        Cylinder(radius=0.0155, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=graphite,
        name="selector_skirt",
    )
    selector_knob.visual(
        Box((0.0065, 0.0025, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.013)),
        material=shell_white,
        name="selector_marker",
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "tower_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "tower_to_grille",
        ArticulationType.FIXED,
        parent=tower,
        child=grille,
        origin=Origin(),
    )
    model.articulation(
        "tower_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_dial,
        origin=Origin(xyz=(DIAL_X, CONTROL_Y, CONTROL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )
    model.articulation(
        "tower_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=selector_knob,
        origin=Origin(xyz=(SELECTOR_X, CONTROL_Y, CONTROL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=2.0, lower=-0.9, upper=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    blower_wheel = object_model.get_part("blower_wheel")
    grille = object_model.get_part("grille")
    speed_dial = object_model.get_part("speed_dial")
    selector_knob = object_model.get_part("selector_knob")

    swivel = object_model.get_articulation("base_to_tower")
    wheel_spin = object_model.get_articulation("tower_to_blower_wheel")
    dial_spin = object_model.get_articulation("tower_to_speed_dial")
    selector_turn = object_model.get_articulation("tower_to_selector_knob")

    ctx.allow_overlap(
        blower_wheel,
        tower,
        elem_a="wheel",
        elem_b="housing",
        reason="The squirrel-cage blower wheel is intentionally housed inside the tower shell, which is represented as an enclosure proxy around the spinning drum.",
    )

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="tower sits directly above the round base",
    )
    ctx.expect_within(
        blower_wheel,
        tower,
        axes="xy",
        inner_elem="wheel",
        outer_elem="housing",
        margin=0.010,
        name="blower wheel stays within the tower body footprint",
    )
    ctx.expect_within(
        blower_wheel,
        tower,
        axes="z",
        inner_elem="wheel",
        outer_elem="housing",
        margin=0.030,
        name="blower wheel stays within the tower height",
    )
    ctx.expect_overlap(
        grille,
        tower,
        axes="xz",
        elem_a="front_grille",
        elem_b="housing",
        min_overlap=0.090,
        name="front grille spans the tower opening",
    )
    ctx.expect_gap(
        speed_dial,
        tower,
        axis="z",
        positive_elem="dial",
        negative_elem="housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="speed dial sits on the tower cap",
    )
    ctx.expect_gap(
        selector_knob,
        tower,
        axis="z",
        positive_elem="selector",
        negative_elem="housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector knob sits on the adjacent shaft boss",
    )
    ctx.expect_origin_distance(
        speed_dial,
        selector_knob,
        axes="xy",
        min_dist=0.045,
        max_dist=0.070,
        name="selector knob sits beside the main speed dial",
    )

    limits = swivel.motion_limits
    ctx.check(
        "tower swivel allows visible oscillation",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < -0.5 and limits.upper > 0.5,
        details=f"swivel_limits={limits}",
    )
    ctx.check(
        "blower wheel uses continuous rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"wheel_joint_type={wheel_spin.articulation_type}",
    )
    ctx.check(
        "speed dial uses continuous rotation",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"dial_joint_type={dial_spin.articulation_type}",
    )
    selector_limits = selector_turn.motion_limits
    ctx.check(
        "selector knob has a finite rotary range",
        selector_limits is not None and selector_limits.lower is not None and selector_limits.upper is not None and selector_limits.lower < 0.0 < selector_limits.upper,
        details=f"selector_limits={selector_limits}",
    )

    rest_pos = ctx.part_world_position(speed_dial)
    with ctx.pose({swivel: 0.6}):
        turned_pos = ctx.part_world_position(speed_dial)
    ctx.check(
        "tower oscillation yaws the control head around the base",
        rest_pos is not None and turned_pos is not None and turned_pos[1] > rest_pos[1] + 0.01,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()

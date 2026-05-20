from __future__ import annotations

from math import cos, pi, sin

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

BASE_RADIUS = 0.165
BASE_THICKNESS = 0.024
BASE_CROWN_THICKNESS = 0.006
TURNTABLE_RADIUS = 0.053
TURNTABLE_HEIGHT = 0.046
OSCILLATION_Z = BASE_THICKNESS + BASE_CROWN_THICKNESS + TURNTABLE_HEIGHT

LOWER_COLLAR_WIDTH = 0.096
LOWER_COLLAR_DEPTH = 0.072
LOWER_COLLAR_HEIGHT = 0.040
TOWER_WIDTH = 0.136
TOWER_DEPTH = 0.114
COLUMN_HEIGHT = 0.828
TOP_CAP_WIDTH = 0.146
TOP_CAP_DEPTH = 0.120
TOP_CAP_HEIGHT = 0.032
TOWER_HEIGHT = LOWER_COLLAR_HEIGHT + COLUMN_HEIGHT + TOP_CAP_HEIGHT
WALL_THICKNESS = 0.012

OUTLET_WIDTH = 0.102
OUTLET_HEIGHT = 0.668
OUTLET_BOTTOM = 0.126
SLAT_COUNT = 18
SLAT_HEIGHT = 0.004

BLOWER_BOTTOM = 0.106
BLOWER_RADIUS = 0.045
BLOWER_INNER_RADIUS = 0.026
BLOWER_HEIGHT = 0.720

DISPLAY_WIDTH = 0.046
DISPLAY_DEPTH = 0.004
DISPLAY_HEIGHT = 0.028
DISPLAY_Z = 0.804

KNOB_DECK_Z = TOWER_HEIGHT - 0.001
SPEED_KNOB_X = -0.030
TIMER_KNOB_X = 0.030
KNOB_Y = -0.004


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _make_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    base = base.faces(">Z").workplane().circle(BASE_RADIUS * 0.68).extrude(BASE_CROWN_THICKNESS)
    base = base.faces(">Z").workplane().circle(TURNTABLE_RADIUS).extrude(TURNTABLE_HEIGHT)
    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_tower_fan")

    tower_body = model.material("tower_body", rgba=(0.90, 0.91, 0.92, 1.0))
    tower_trim = model.material("tower_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    control_finish = model.material("control_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    display_finish = model.material("display_finish", rgba=(0.18, 0.23, 0.27, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "pedestal"),
        material=tower_trim,
        name="pedestal",
    )

    tower = model.part("tower")
    tower.visual(
        Box((LOWER_COLLAR_WIDTH, LOWER_COLLAR_DEPTH, LOWER_COLLAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_HEIGHT * 0.5)),
        material=tower_trim,
        name="lower_collar",
    )
    tower.visual(
        Box((0.104, 0.038, 0.080)),
        origin=Origin(xyz=(0.0, -0.032, 0.040)),
        material=tower_body,
        name="rear_neck",
    )
    tower.visual(
        Box((0.016, 0.062, 0.080)),
        origin=Origin(xyz=(-0.052, 0.0, 0.040)),
        material=tower_body,
        name="side_neck_0",
    )
    tower.visual(
        Box((0.016, 0.062, 0.080)),
        origin=Origin(xyz=(0.052, 0.0, 0.040)),
        material=tower_body,
        name="side_neck_1",
    )
    tower.visual(
        Box((0.124, 0.012, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.051, 0.039 + COLUMN_HEIGHT * 0.5)),
        material=tower_body,
        name="tower_shell",
    )
    tower.visual(
        Box((0.012, TOWER_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-0.062, 0.0, 0.039 + COLUMN_HEIGHT * 0.5)),
        material=tower_body,
        name="side_shell_0",
    )
    tower.visual(
        Box((0.012, TOWER_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.062, 0.0, 0.039 + COLUMN_HEIGHT * 0.5)),
        material=tower_body,
        name="side_shell_1",
    )
    tower.visual(
        Box((0.112, 0.010, 0.086)),
        origin=Origin(xyz=(0.0, 0.052, 0.0825)),
        material=tower_body,
        name="lower_bridge",
    )
    tower.visual(
        Box((0.112, 0.010, 0.080)),
        origin=Origin(xyz=(0.0, 0.052, 0.834)),
        material=tower_body,
        name="upper_bridge",
    )
    tower.visual(
        Box((0.034, TOP_CAP_DEPTH, TOP_CAP_HEIGHT)),
        origin=Origin(xyz=(-0.056, 0.0, LOWER_COLLAR_HEIGHT + COLUMN_HEIGHT + TOP_CAP_HEIGHT * 0.5 - 0.001)),
        material=tower_body,
        name="top_cap_0",
    )
    tower.visual(
        Box((0.034, TOP_CAP_DEPTH, TOP_CAP_HEIGHT)),
        origin=Origin(xyz=(0.056, 0.0, LOWER_COLLAR_HEIGHT + COLUMN_HEIGHT + TOP_CAP_HEIGHT * 0.5 - 0.001)),
        material=tower_body,
        name="top_cap_1",
    )
    tower.visual(
        Box((0.078, 0.054, TOP_CAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.024, LOWER_COLLAR_HEIGHT + COLUMN_HEIGHT + TOP_CAP_HEIGHT * 0.5 - 0.001)),
        material=tower_body,
        name="top_cap_2",
    )
    tower.visual(
        Box((0.018, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.026, BLOWER_BOTTOM - 0.012)),
        material=tower_trim,
        name="blower_support_0",
    )
    tower.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_BOTTOM - 0.012)),
        material=tower_trim,
        name="blower_bearing_0",
    )
    tower.visual(
        Box((0.018, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.026, BLOWER_BOTTOM + BLOWER_HEIGHT + 0.012)),
        material=tower_trim,
        name="blower_support_1",
    )
    tower.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_BOTTOM + BLOWER_HEIGHT + 0.012)),
        material=tower_trim,
        name="blower_bearing_1",
    )

    slat_pitch = OUTLET_HEIGHT / (SLAT_COUNT + 1)
    for index in range(SLAT_COUNT):
        tower.visual(
            Box((0.112, 0.008, SLAT_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.052,
                    OUTLET_BOTTOM + slat_pitch * (index + 1),
                )
            ),
            material=tower_trim,
            name=f"slat_{index}",
        )
    tower.visual(
        Box((DISPLAY_WIDTH, DISPLAY_DEPTH, DISPLAY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.054,
                DISPLAY_Z,
            )
        ),
        material=display_finish,
        name="display_panel",
    )

    blower = model.part("blower")
    blower.visual(
        Cylinder(radius=0.010, length=BLOWER_HEIGHT + 0.016),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_HEIGHT * 0.5)),
        material=rotor_finish,
        name="blower_wheel",
    )
    blower.visual(
        Cylinder(radius=BLOWER_RADIUS, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rotor_finish,
        name="endplate_0",
    )
    blower.visual(
        Cylinder(radius=BLOWER_RADIUS, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_HEIGHT - 0.004)),
        material=rotor_finish,
        name="endplate_1",
    )
    blade_height = BLOWER_HEIGHT - 0.016
    blade_radius = 0.034
    blade_count = 24
    for index in range(blade_count):
        angle = index * (2.0 * pi / blade_count)
        blower.visual(
            Box((0.004, 0.020, blade_height)),
            origin=Origin(
                xyz=(blade_radius * cos(angle), blade_radius * sin(angle), BLOWER_HEIGHT * 0.5),
                rpy=(0.0, 0.0, angle + 0.35),
            ),
            material=rotor_finish,
            name=f"blade_{index}",
        )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.018,
                body_style="skirted",
                top_diameter=0.026,
                skirt=KnobSkirt(0.038, 0.004, flare=0.12),
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "speed_knob",
        ),
        material=control_finish,
        name="speed_cap",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.016,
                body_style="skirted",
                top_diameter=0.023,
                skirt=KnobSkirt(0.033, 0.0035, flare=0.10),
                grip=KnobGrip(style="fluted", count=12, depth=0.0008),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob",
        ),
        material=control_finish,
        name="timer_cap",
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, OSCILLATION_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "tower_to_blower",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, BLOWER_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=35.0),
    )
    model.articulation(
        "tower_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_knob,
        origin=Origin(xyz=(SPEED_KNOB_X, KNOB_Y, KNOB_DECK_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "tower_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=timer_knob,
        origin=Origin(xyz=(TIMER_KNOB_X, KNOB_Y, KNOB_DECK_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    blower = object_model.get_part("blower")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")

    oscillation = object_model.get_articulation("base_to_tower")

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="tower seats on the oscillation turntable",
    )
    ctx.expect_overlap(
        tower,
        base,
        axes="xy",
        min_overlap=0.070,
        name="tower stays centered above the pedestal",
    )

    ctx.expect_within(
        blower,
        tower,
        axes="xy",
        margin=0.002,
        name="blower stays inside the tower shell",
    )
    ctx.expect_overlap(
        blower,
        tower,
        axes="z",
        min_overlap=0.680,
        name="blower spans the active tower height",
    )

    ctx.expect_gap(
        speed_knob,
        tower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="speed knob seats on the control deck",
    )
    ctx.expect_overlap(
        speed_knob,
        tower,
        axes="xy",
        min_overlap=0.020,
        name="speed knob remains on the top deck",
    )
    ctx.expect_gap(
        timer_knob,
        tower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="timer knob seats on the control deck",
    )
    ctx.expect_overlap(
        timer_knob,
        tower,
        axes="xy",
        min_overlap=0.018,
        name="timer knob remains on the top deck",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_display = aabb_center(ctx.part_element_world_aabb(tower, elem="display_panel"))
    upper = oscillation.motion_limits.upper if oscillation.motion_limits is not None else None
    with ctx.pose({oscillation: upper if upper is not None else 0.0}):
        swung_display = aabb_center(ctx.part_element_world_aabb(tower, elem="display_panel"))
        ctx.expect_within(
            blower,
            tower,
            axes="xy",
            margin=0.002,
            name="blower remains centered while the tower oscillates",
        )

    ctx.check(
        "tower oscillates about the pedestal axis",
        rest_display is not None
        and swung_display is not None
        and swung_display[0] < rest_display[0] - 0.030,
        details=f"rest={rest_display}, swung={swung_display}",
    )

    return ctx.report()


object_model = build_object_model()

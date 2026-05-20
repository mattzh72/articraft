from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """Small CadQuery helper for appliance panels with softly radiused corners."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(radius)
    )


def _tower_housing() -> cq.Workplane:
    """Rounded rectangular fan body with a real hollow cavity and front opening."""
    width = 0.145
    depth = 0.105
    height = 0.520
    wall = 0.008

    outer = _rounded_box(width, depth, height, 0.016)
    inner_cavity = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - 0.040)
        .translate((0.0, 0.0, 0.0))
    )
    front_opening = (
        cq.Workplane("XY")
        .box(0.114, 0.045, 0.420)
        .translate((0.0, -depth / 2.0 + 0.006, -0.010))
    )
    return outer.cut(inner_cavity).cut(front_opening)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dorm_tower_fan")

    white = model.material("warm_white_plastic", color=(0.88, 0.88, 0.82, 1.0))
    dark = model.material("charcoal_grille", color=(0.025, 0.028, 0.030, 1.0))
    grey = model.material("dark_grey_control", color=(0.16, 0.17, 0.18, 1.0))
    pale = model.material("pale_indicator", color=(0.92, 0.92, 0.84, 1.0))
    steel = model.material("brushed_steel", color=(0.55, 0.57, 0.58, 1.0))
    blue = model.material("muted_blue_impeller", color=(0.26, 0.36, 0.43, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=white,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=white,
        name="base_turntable",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=steel,
        name="pivot_button",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=white,
        name="lower_collar",
    )
    column.visual(
        Cylinder(radius=0.043, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=white,
        name="neck_post",
    )
    column.visual(
        mesh_from_cadquery(_tower_housing(), "housing_shell", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=white,
        name="housing_shell",
    )
    column.visual(
        mesh_from_cadquery(_rounded_box(0.145, 0.105, 0.040, 0.016), "top_cap", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        material=white,
        name="top_cap",
    )
    column.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.120, 0.425),
                frame=0.009,
                face_thickness=0.004,
                duct_depth=0.010,
                duct_wall=0.002,
                slat_pitch=0.018,
                slat_width=0.006,
                slat_angle_deg=8.0,
                corner_radius=0.006,
                slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=2, divider_width=0.003),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.001),
                sleeve=VentGrilleSleeve(style="none"),
            ),
            "front_grille",
        ),
        origin=Origin(xyz=(0.0, -0.054, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_grille",
    )
    column.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, -0.002, 0.080)),
        material=dark,
        name="lower_bearing_cup",
    )
    column.visual(
        Box((0.014, 0.062, 0.010)),
        origin=Origin(xyz=(0.0, 0.022, 0.080)),
        material=dark,
        name="lower_bearing_bridge",
    )
    column.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, -0.002, 0.520)),
        material=dark,
        name="upper_bearing_cup",
    )
    column.visual(
        Box((0.014, 0.062, 0.010)),
        origin=Origin(xyz=(0.0, 0.022, 0.520)),
        material=dark,
        name="upper_bearing_bridge",
    )

    # Printed tick marks on the fixed top cap, placed outside the rotating knobs.
    for i, ang in enumerate((-55.0, 0.0, 55.0)):
        a = math.radians(ang)
        column.visual(
            Box((0.003, 0.010, 0.0015)),
            origin=Origin(
                xyz=(-0.035 + 0.036 * math.sin(a), -0.002 + 0.036 * math.cos(a), 0.60075),
                rpy=(0.0, 0.0, -a),
            ),
            material=dark,
            name=f"mode_tick_{i}",
        )
    for i, ang in enumerate((-45.0, 45.0)):
        a = math.radians(ang)
        column.visual(
            Box((0.0025, 0.0075, 0.0015)),
            origin=Origin(
                xyz=(0.038 + 0.025 * math.sin(a), -0.001 + 0.025 * math.cos(a), 0.60075),
                rpy=(0.0, 0.0, -a),
            ),
            material=dark,
            name=f"timer_tick_{i}",
        )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.75, upper=0.75),
    )

    impeller = model.part("impeller")
    impeller.visual(
        Cylinder(radius=0.006, length=0.430),
        origin=Origin(),
        material=steel,
        name="rotor_shaft",
    )
    impeller.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(),
        material=steel,
        name="center_hub",
    )
    impeller.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.034,
                0.016,
                0.420,
                30,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel",
        ),
        origin=Origin(),
        material=blue,
        name="blower_wheel",
    )
    model.articulation(
        "column_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=impeller,
        origin=Origin(xyz=(0.0, -0.002, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=60.0),
    )

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.055,
                0.025,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.064, 0.005, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "mode_knob_cap",
        ),
        origin=Origin(),
        material=grey,
        name="mode_knob_cap",
    )
    mode_knob.visual(
        Cylinder(radius=0.0035, length=0.0012),
        origin=Origin(xyz=(0.0, 0.018, 0.0256)),
        material=pale,
        name="mode_pointer",
    )
    model.articulation(
        "column_to_mode_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=mode_knob,
        origin=Origin(xyz=(-0.035, -0.002, 0.6000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=0.0, upper=2.0 * math.pi),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.035,
                0.021,
                body_style="tapered",
                base_diameter=0.039,
                top_diameter=0.029,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob_cap",
        ),
        origin=Origin(),
        material=grey,
        name="timer_knob_cap",
    )
    timer_knob.visual(
        Cylinder(radius=0.0026, length=0.0010),
        origin=Origin(xyz=(0.0, 0.012, 0.0215)),
        material=pale,
        name="timer_pointer",
    )
    model.articulation(
        "column_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=timer_knob,
        origin=Origin(xyz=(0.038, -0.001, 0.6000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.5, lower=0.0, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    impeller = object_model.get_part("impeller")
    mode_knob = object_model.get_part("mode_knob")
    timer_knob = object_model.get_part("timer_knob")

    oscillation = object_model.get_articulation("base_to_column")
    impeller_spin = object_model.get_articulation("column_to_impeller")
    mode_turn = object_model.get_articulation("column_to_mode_knob")
    timer_turn = object_model.get_articulation("column_to_timer_knob")

    ctx.check(
        "impeller uses continuous vertical spin",
        impeller_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(impeller_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={impeller_spin.articulation_type}, axis={impeller_spin.axis}",
    )
    ctx.check(
        "column oscillates about the base",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower < 0.0
        and oscillation.motion_limits.upper > 0.0,
        details=f"type={oscillation.articulation_type}, limits={oscillation.motion_limits}",
    )
    ctx.check(
        "top knobs have independent rotary joints",
        mode_turn.articulation_type == ArticulationType.REVOLUTE
        and timer_turn.articulation_type == ArticulationType.REVOLUTE
        and mode_turn.child == "mode_knob"
        and timer_turn.child == "timer_knob",
        details=f"mode={mode_turn}, timer={timer_turn}",
    )

    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="base_turntable",
        max_gap=0.002,
        max_penetration=0.0,
        name="oscillating collar sits on base turntable",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="xy",
        elem_a="lower_collar",
        elem_b="base_turntable",
        min_overlap=0.080,
        name="column collar is centered over circular base",
    )
    ctx.expect_within(
        impeller,
        column,
        axes="xy",
        inner_elem="blower_wheel",
        outer_elem="housing_shell",
        margin=0.0,
        name="impeller fits inside hollow column footprint",
    )
    ctx.expect_overlap(
        impeller,
        column,
        axes="z",
        elem_a="blower_wheel",
        elem_b="front_grille",
        min_overlap=0.350,
        name="impeller spans the tall front grille",
    )
    ctx.expect_gap(
        impeller,
        column,
        axis="z",
        positive_elem="rotor_shaft",
        negative_elem="lower_bearing_cup",
        max_gap=0.001,
        max_penetration=0.00001,
        name="lower bearing supports impeller shaft",
    )
    ctx.expect_gap(
        column,
        impeller,
        axis="z",
        positive_elem="upper_bearing_cup",
        negative_elem="rotor_shaft",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper bearing captures impeller shaft",
    )
    ctx.expect_gap(
        mode_knob,
        column,
        axis="z",
        positive_elem="mode_knob_cap",
        negative_elem="top_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="large mode knob sits on top cap",
    )
    ctx.expect_gap(
        timer_knob,
        column,
        axis="z",
        positive_elem="timer_knob_cap",
        negative_elem="top_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="small timer knob sits on top cap",
    )

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    front_rest = _center(ctx.part_element_world_aabb(column, elem="front_grille"))
    with ctx.pose({oscillation: 0.70}):
        front_swept = _center(ctx.part_element_world_aabb(column, elem="front_grille"))
    ctx.check(
        "oscillation sweeps the fan column sideways",
        front_rest is not None and front_swept is not None and abs(front_swept[0] - front_rest[0]) > 0.025,
        details=f"rest={front_rest}, swept={front_swept}",
    )

    mode_rest = _center(ctx.part_element_world_aabb(mode_knob, elem="mode_pointer"))
    timer_rest = _center(ctx.part_element_world_aabb(timer_knob, elem="timer_pointer"))
    with ctx.pose({mode_turn: math.pi / 2.0}):
        mode_quarter = _center(ctx.part_element_world_aabb(mode_knob, elem="mode_pointer"))
        timer_after_mode = _center(ctx.part_element_world_aabb(timer_knob, elem="timer_pointer"))
    ctx.check(
        "mode knob pointer rotates without moving timer knob",
        mode_rest is not None
        and mode_quarter is not None
        and timer_rest is not None
        and timer_after_mode is not None
        and abs(mode_quarter[0] - mode_rest[0]) > 0.010
        and abs(timer_after_mode[0] - timer_rest[0]) < 0.001,
        details=f"mode_rest={mode_rest}, mode_quarter={mode_quarter}, timer_rest={timer_rest}, timer_after={timer_after_mode}",
    )

    timer_rest = _center(ctx.part_element_world_aabb(timer_knob, elem="timer_pointer"))
    mode_rest = _center(ctx.part_element_world_aabb(mode_knob, elem="mode_pointer"))
    with ctx.pose({timer_turn: math.pi / 2.0}):
        timer_quarter = _center(ctx.part_element_world_aabb(timer_knob, elem="timer_pointer"))
        mode_after_timer = _center(ctx.part_element_world_aabb(mode_knob, elem="mode_pointer"))
    ctx.check(
        "timer knob pointer rotates without moving mode knob",
        timer_rest is not None
        and timer_quarter is not None
        and mode_rest is not None
        and mode_after_timer is not None
        and abs(timer_quarter[0] - timer_rest[0]) > 0.006
        and abs(mode_after_timer[0] - mode_rest[0]) < 0.001,
        details=f"timer_rest={timer_rest}, timer_quarter={timer_quarter}, mode_rest={mode_rest}, mode_after={mode_after_timer}",
    )

    return ctx.report()


object_model = build_object_model()

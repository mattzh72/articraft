from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

HOUSING_SIZE = 0.52
HOUSING_DEPTH = 0.17
HOUSING_WALL = 0.026
HOUSING_HALF = HOUSING_SIZE * 0.5
HOUSING_OPEN = HOUSING_SIZE - (2.0 * HOUSING_WALL)
HOUSING_FRONT_Y = HOUSING_DEPTH * 0.5
HOUSING_REAR_Y = -HOUSING_DEPTH * 0.5

PIVOT_HEIGHT = 0.320
SIDE_THICKNESS = 0.038
SIDE_INNER_X = 0.288
SIDE_OUTER_X = SIDE_INNER_X + SIDE_THICKNESS
TRUNNION_RADIUS = 0.017
TRUNNION_HOLE_RADIUS = 0.022
TRUNNION_OUTER_X = SIDE_OUTER_X
TRUNNION_LENGTH = TRUNNION_OUTER_X - (HOUSING_HALF - 0.001)
TRUNNION_CENTER_X = (TRUNNION_OUTER_X + (HOUSING_HALF - 0.001)) * 0.5

GRILLE_THICKNESS = 0.006
GRILLE_PLANE_Y = HOUSING_FRONT_Y + (GRILLE_THICKNESS * 0.5)
REAR_GRILLE_PLANE_Y = HOUSING_REAR_Y - (GRILLE_THICKNESS * 0.5)
GRILLE_FRAME_BAR = 0.014
GRILLE_BAR = 0.012
GRILLE_HALF = HOUSING_OPEN * 0.5

SPEED_KNOB_X = 0.190
SPEED_KNOB_Z = -0.190
SPEED_KNOB_Y = GRILLE_PLANE_Y + (GRILLE_THICKNESS * 0.5)


def _build_side_arm() -> cq.Workplane:
    outer_profile = [
        (-0.145, 0.000),
        (-0.145, 0.040),
        (-0.105, 0.040),
        (-0.105, 0.275),
        (-0.050, 0.350),
        (0.000, 0.392),
        (0.055, 0.350),
        (0.110, 0.300),
        (0.145, 0.220),
        (0.145, 0.000),
    ]
    inner_cutout = [
        (-0.105, 0.065),
        (-0.105, 0.245),
        (-0.042, 0.325),
        (0.000, 0.355),
        (0.048, 0.326),
        (0.100, 0.272),
        (0.100, 0.065),
    ]

    arm = cq.Workplane("YZ").polyline(outer_profile).close().extrude(SIDE_THICKNESS)
    cutout = (
        cq.Workplane("YZ")
        .polyline(inner_cutout)
        .close()
        .extrude(SIDE_THICKNESS + 0.002)
        .translate((-0.001, 0.0, 0.0))
    )
    pivot_hole = (
        cq.Workplane("YZ")
        .center(0.0, PIVOT_HEIGHT)
        .circle(TRUNNION_HOLE_RADIUS)
        .extrude(SIDE_THICKNESS + 0.002)
        .translate((-0.001, 0.0, 0.0))
    )
    return arm.cut(cutout).cut(pivot_hole)


def _build_cradle() -> cq.Workplane:
    right_arm = _build_side_arm().translate((SIDE_INNER_X, 0.0, 0.0))
    left_arm = _build_side_arm().translate((-(SIDE_INNER_X + SIDE_THICKNESS), 0.0, 0.0))

    front_bar = cq.Workplane("XY").box(0.600, 0.040, 0.050).translate((0.0, 0.105, 0.025))
    rear_bar = cq.Workplane("XY").box(0.600, 0.042, 0.050).translate((0.0, -0.110, 0.025))
    return right_arm.union(left_arm).union(front_bar).union(rear_bar)


def _add_square_grille(part, *, plane_y: float, prefix: str, material) -> None:
    frame_center = GRILLE_HALF - (GRILLE_FRAME_BAR * 0.5)
    inner_length = HOUSING_OPEN - (2.0 * GRILLE_FRAME_BAR)

    part.visual(
        Box((HOUSING_OPEN, GRILLE_THICKNESS, GRILLE_FRAME_BAR)),
        origin=Origin(xyz=(0.0, plane_y, frame_center)),
        material=material,
        name=f"{prefix}_top",
    )
    part.visual(
        Box((HOUSING_OPEN, GRILLE_THICKNESS, GRILLE_FRAME_BAR)),
        origin=Origin(xyz=(0.0, plane_y, -frame_center)),
        material=material,
        name=f"{prefix}_bottom",
    )
    part.visual(
        Box((GRILLE_FRAME_BAR, GRILLE_THICKNESS, HOUSING_OPEN)),
        origin=Origin(xyz=(frame_center, plane_y, 0.0)),
        material=material,
        name=f"{prefix}_right",
    )
    part.visual(
        Box((GRILLE_FRAME_BAR, GRILLE_THICKNESS, HOUSING_OPEN)),
        origin=Origin(xyz=(-frame_center, plane_y, 0.0)),
        material=material,
        name=f"{prefix}_left",
    )

    bar_positions = (-0.154, -0.077, 0.0, 0.077, 0.154)
    for index, x_pos in enumerate(bar_positions):
        part.visual(
            Box((GRILLE_BAR, GRILLE_THICKNESS, inner_length)),
            origin=Origin(xyz=(x_pos, plane_y, 0.0)),
            material=material,
            name=f"{prefix}_vertical_{index}",
        )
    for index, z_pos in enumerate(bar_positions):
        part.visual(
            Box((inner_length, GRILLE_THICKNESS, GRILLE_BAR)),
            origin=Origin(xyz=(0.0, plane_y, z_pos)),
            material=material,
            name=f"{prefix}_horizontal_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan")

    frame_black = model.material("frame_black", rgba=(0.15, 0.16, 0.17, 1.0))
    cradle_black = model.material("cradle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grille_black = model.material("grille_black", rgba=(0.08, 0.09, 0.10, 1.0))
    blade_black = model.material("blade_black", rgba=(0.18, 0.18, 0.19, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    motor_black = model.material("motor_black", rgba=(0.13, 0.13, 0.14, 1.0))

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_build_cradle(), "cradle"),
        material=cradle_black,
        name="cradle_frame",
    )
    cradle.visual(
        Box((0.006, 0.060, 0.012)),
        origin=Origin(xyz=(SIDE_INNER_X + 0.003, 0.0, PIVOT_HEIGHT + 0.027)),
        material=cradle_black,
        name="pivot_pad_0_top",
    )
    cradle.visual(
        Box((0.006, 0.060, 0.012)),
        origin=Origin(xyz=(SIDE_INNER_X + 0.003, 0.0, PIVOT_HEIGHT - 0.027)),
        material=cradle_black,
        name="pivot_pad_0_bottom",
    )
    cradle.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(SIDE_INNER_X + 0.003, 0.027, PIVOT_HEIGHT)),
        material=cradle_black,
        name="pivot_pad_0_front",
    )
    cradle.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(SIDE_INNER_X + 0.003, -0.027, PIVOT_HEIGHT)),
        material=cradle_black,
        name="pivot_pad_0_rear",
    )
    cradle.visual(
        Box((0.006, 0.060, 0.012)),
        origin=Origin(xyz=(-(SIDE_INNER_X + 0.003), 0.0, PIVOT_HEIGHT + 0.027)),
        material=cradle_black,
        name="pivot_pad_1_top",
    )
    cradle.visual(
        Box((0.006, 0.060, 0.012)),
        origin=Origin(xyz=(-(SIDE_INNER_X + 0.003), 0.0, PIVOT_HEIGHT - 0.027)),
        material=cradle_black,
        name="pivot_pad_1_bottom",
    )
    cradle.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(-(SIDE_INNER_X + 0.003), 0.027, PIVOT_HEIGHT)),
        material=cradle_black,
        name="pivot_pad_1_front",
    )
    cradle.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(-(SIDE_INNER_X + 0.003), -0.027, PIVOT_HEIGHT)),
        material=cradle_black,
        name="pivot_pad_1_rear",
    )

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_SIZE, HOUSING_DEPTH, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HALF - (HOUSING_WALL * 0.5))),
        material=frame_black,
        name="top_shell",
    )
    housing.visual(
        Box((HOUSING_SIZE, HOUSING_DEPTH, HOUSING_WALL)),
        origin=Origin(xyz=(0.0, 0.0, -(HOUSING_HALF - (HOUSING_WALL * 0.5)))),
        material=frame_black,
        name="bottom_shell",
    )
    housing.visual(
        Box((HOUSING_WALL, HOUSING_DEPTH, HOUSING_OPEN)),
        origin=Origin(xyz=(HOUSING_HALF - (HOUSING_WALL * 0.5), 0.0, 0.0)),
        material=frame_black,
        name="side_shell_0",
    )
    housing.visual(
        Box((HOUSING_WALL, HOUSING_DEPTH, HOUSING_OPEN)),
        origin=Origin(xyz=(-(HOUSING_HALF - (HOUSING_WALL * 0.5)), 0.0, 0.0)),
        material=frame_black,
        name="side_shell_1",
    )
    housing.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(TRUNNION_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=frame_black,
        name="trunnion_0",
    )
    housing.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(-TRUNNION_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=frame_black,
        name="trunnion_1",
    )
    housing.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(SIDE_INNER_X - 0.005, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=frame_black,
        name="pivot_collar_0",
    )
    housing.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(-(SIDE_INNER_X - 0.005), 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=frame_black,
        name="pivot_collar_1",
    )

    front_grille = model.part("front_grille")
    _add_square_grille(front_grille, plane_y=GRILLE_PLANE_Y, prefix="front", material=grille_black)
    front_grille.visual(
        Box((0.074, GRILLE_THICKNESS, 0.074)),
        origin=Origin(xyz=(SPEED_KNOB_X, GRILLE_PLANE_Y, SPEED_KNOB_Z)),
        material=grille_black,
        name="control_pod",
    )

    rear_grille = model.part("rear_grille")
    _add_square_grille(rear_grille, plane_y=REAR_GRILLE_PLANE_Y, prefix="rear", material=grille_black)
    rear_grille.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=motor_black,
        name="motor_pod",
    )
    rear_grille.visual(
        Box((0.034, 0.060, 0.176)),
        origin=Origin(xyz=(0.0, -0.055, 0.132)),
        material=motor_black,
        name="motor_strut_top",
    )
    rear_grille.visual(
        Box((0.034, 0.060, 0.176)),
        origin=Origin(xyz=(0.0, -0.055, -0.132)),
        material=motor_black,
        name="motor_strut_bottom",
    )
    rear_grille.visual(
        Box((0.176, 0.060, 0.034)),
        origin=Origin(xyz=(0.132, -0.055, 0.0)),
        material=motor_black,
        name="motor_strut_right",
    )
    rear_grille.visual(
        Box((0.176, 0.060, 0.034)),
        origin=Origin(xyz=(-0.132, -0.055, 0.0)),
        material=motor_black,
        name="motor_strut_left",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.205,
                0.052,
                5,
                thickness=0.028,
                blade_pitch_deg=31.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", camber=0.12, tip_pitch_deg=15.0),
                hub=FanRotorHub(style="domed", bore_diameter=0.010),
            ),
            "blade",
        ),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=blade_black,
        name="impeller",
    )
    blade.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=motor_black,
        name="axle",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.024,
                body_style="skirted",
                top_diameter=0.034,
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="cap",
    )
    speed_knob.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="seat",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.032,
                body_style="mushroom",
                top_diameter=0.056,
                base_diameter=0.060,
                center=False,
            ),
            "tilt_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=knob_black,
        name="cap",
    )
    tilt_knob.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=knob_black,
        name="seat",
    )

    tilt = model.articulation(
        "cradle_to_housing",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "housing_to_front_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_rear_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_grille,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(SPEED_KNOB_X, SPEED_KNOB_Y, SPEED_KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=4.0),
    )
    model.articulation(
        "housing_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=tilt_knob,
        origin=Origin(xyz=(TRUNNION_OUTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )

    tilt.meta["qc_samples"] = [-0.45, 0.0, 0.55]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cradle = object_model.get_part("cradle")
    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rear_grille = object_model.get_part("rear_grille")
    blade = object_model.get_part("blade")
    speed_knob = object_model.get_part("speed_knob")
    tilt_knob = object_model.get_part("tilt_knob")

    tilt = object_model.get_articulation("cradle_to_housing")
    blade_joint = object_model.get_articulation("housing_to_blade")
    speed_joint = object_model.get_articulation("housing_to_speed_knob")
    tilt_knob_joint = object_model.get_articulation("housing_to_tilt_knob")

    ctx.expect_gap(
        speed_knob,
        front_grille,
        axis="y",
        positive_elem="seat",
        negative_elem="control_pod",
        max_gap=0.001,
        max_penetration=0.0,
        name="speed knob seats against the front grille",
    )
    ctx.expect_gap(
        tilt_knob,
        housing,
        axis="x",
        positive_elem="seat",
        negative_elem="trunnion_0",
        max_gap=0.001,
        max_penetration=1e-6,
        name="tilt knob seats on the housing trunnion",
    )
    ctx.expect_gap(
        blade,
        rear_grille,
        axis="y",
        positive_elem="axle",
        negative_elem="motor_pod",
        max_gap=0.001,
        max_penetration=0.0,
        name="blade axle meets the rear motor pod",
    )

    rest_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({tilt: 0.55}):
        tilted_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "housing tilts upward in the cradle",
        rest_pos is not None and tilted_pos is not None and tilted_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, tilted={tilted_pos}",
    )

    ctx.check(
        "fan controls and blade use continuous rotation",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.CONTINUOUS
        and tilt_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"blade={blade_joint.articulation_type}, "
            f"speed={speed_joint.articulation_type}, "
            f"tilt_knob={tilt_knob_joint.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

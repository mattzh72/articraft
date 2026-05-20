from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OUTER_SIZE = 0.56
HOUSING_DEPTH = 0.17
SHELL_WALL = 0.018
FRONT_BORDER = 0.055
FRONT_FRAME_DEPTH = 0.020
REAR_FRAME_DEPTH = 0.014
PAN_DISK_RADIUS = 0.068
PAN_DISK_THICKNESS = 0.012
REAR_PAD_RADIUS = 0.058
REAR_PAD_THICKNESS = 0.014
MOTOR_POD_RADIUS = 0.064
MOTOR_POD_LENGTH = 0.070
MOTOR_STRUT_LENGTH = 0.530
ROTOR_RADIUS = 0.192
ROTOR_THICKNESS = 0.020
GUARD_BAR_RADIUS = 0.0045
GUARD_SPAN = OUTER_SIZE - 2.0 * FRONT_BORDER + 0.016
KNOB_DIAMETER = 0.044
KNOB_HEIGHT = 0.022
CONTROL_BOSS_RADIUS = 0.028
CONTROL_BOSS_DEPTH = 0.012


def _cylinder_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_cylinder(part, *, axis: str, radius: float, length: float, xyz, material: str, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_cylinder_rpy(axis)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_box_fan")

    model.material("bracket_grey", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("bracket_dark", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("housing_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("guard_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("rotor_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("accent_silver", rgba=(0.78, 0.79, 0.80, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.16, 0.012, 0.24)),
        origin=Origin(xyz=(0.0, -0.148, 0.0)),
        material="bracket_grey",
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.08, 0.13, 0.06)),
        origin=Origin(xyz=(0.0, -0.077, 0.0)),
        material="bracket_grey",
        name="arm",
    )
    wall_bracket.visual(
        Box((0.024, 0.124, 0.19)),
        origin=Origin(xyz=(0.0, -0.078, 0.0)),
        material="bracket_dark",
        name="spine",
    )
    wall_bracket.visual(
        Box((0.05, 0.10, 0.022)),
        origin=Origin(xyz=(0.0, -0.095, 0.055)),
        material="bracket_grey",
        name="upper_brace",
    )
    wall_bracket.visual(
        Box((0.05, 0.10, 0.022)),
        origin=Origin(xyz=(0.0, -0.095, -0.055)),
        material="bracket_grey",
        name="lower_brace",
    )
    _add_cylinder(
        wall_bracket,
        axis="y",
        radius=PAN_DISK_RADIUS,
        length=PAN_DISK_THICKNESS,
        xyz=(0.0, -PAN_DISK_THICKNESS / 2.0, 0.0),
        material="bracket_dark",
        name="pan_disk",
    )

    housing = model.part("housing")
    _add_cylinder(
        housing,
        axis="y",
        radius=REAR_PAD_RADIUS,
        length=REAR_PAD_THICKNESS,
        xyz=(0.0, REAR_PAD_THICKNESS / 2.0, 0.0),
        material="housing_dark",
        name="rear_pad",
    )
    _add_cylinder(
        housing,
        axis="y",
        radius=MOTOR_POD_RADIUS,
        length=MOTOR_POD_LENGTH,
        xyz=(0.0, REAR_FRAME_DEPTH + MOTOR_POD_LENGTH / 2.0 - 0.001, 0.0),
        material="housing_dark",
        name="motor_pod",
    )

    for side in (-1.0, 1.0):
        housing.visual(
            Box((SHELL_WALL, HOUSING_DEPTH, OUTER_SIZE - 2.0 * SHELL_WALL)),
            origin=Origin(xyz=(side * (OUTER_SIZE / 2.0 - SHELL_WALL / 2.0), HOUSING_DEPTH / 2.0, 0.0)),
            material="housing_dark",
            name=f"side_wall_{int((side + 1.0) / 2.0)}",
        )
        housing.visual(
            Box((OUTER_SIZE - 2.0 * FRONT_BORDER, FRONT_FRAME_DEPTH, FRONT_BORDER)),
            origin=Origin(
                xyz=(0.0, HOUSING_DEPTH - FRONT_FRAME_DEPTH / 2.0, side * (OUTER_SIZE / 2.0 - FRONT_BORDER / 2.0))
            ),
            material="housing_dark",
            name=f"front_band_{int((side + 1.0) / 2.0)}",
        )
        housing.visual(
            Box((OUTER_SIZE - 2.0 * FRONT_BORDER, REAR_FRAME_DEPTH, FRONT_BORDER)),
            origin=Origin(xyz=(0.0, REAR_FRAME_DEPTH / 2.0, side * (OUTER_SIZE / 2.0 - FRONT_BORDER / 2.0))),
            material="housing_dark",
            name=f"rear_band_{int((side + 1.0) / 2.0)}",
        )

    for side in (-1.0, 1.0):
        housing.visual(
            Box((FRONT_BORDER, FRONT_FRAME_DEPTH, OUTER_SIZE - 2.0 * FRONT_BORDER)),
            origin=Origin(
                xyz=(side * (OUTER_SIZE / 2.0 - FRONT_BORDER / 2.0), HOUSING_DEPTH - FRONT_FRAME_DEPTH / 2.0, 0.0)
            ),
            material="housing_dark",
            name=f"front_upright_{int((side + 1.0) / 2.0)}",
        )
        housing.visual(
            Box((FRONT_BORDER, REAR_FRAME_DEPTH, OUTER_SIZE - 2.0 * FRONT_BORDER)),
            origin=Origin(xyz=(side * (OUTER_SIZE / 2.0 - FRONT_BORDER / 2.0), REAR_FRAME_DEPTH / 2.0, 0.0)),
            material="housing_dark",
            name=f"rear_upright_{int((side + 1.0) / 2.0)}",
        )
        housing.visual(
            Box((OUTER_SIZE - 2.0 * SHELL_WALL, HOUSING_DEPTH, SHELL_WALL)),
            origin=Origin(xyz=(0.0, HOUSING_DEPTH / 2.0, side * (OUTER_SIZE / 2.0 - SHELL_WALL / 2.0))),
            material="housing_dark",
            name=f"top_bottom_wall_{int((side + 1.0) / 2.0)}",
        )

    _add_cylinder(
        housing,
        axis="x",
        radius=0.007,
        length=MOTOR_STRUT_LENGTH,
        xyz=(0.0, 0.062, 0.0),
        material="housing_dark",
        name="motor_strut_x",
    )
    _add_cylinder(
        housing,
        axis="z",
        radius=0.007,
        length=MOTOR_STRUT_LENGTH,
        xyz=(0.0, 0.062, 0.0),
        material="housing_dark",
        name="motor_strut_z",
    )
    _add_cylinder(
        housing,
        axis="y",
        radius=CONTROL_BOSS_RADIUS,
        length=CONTROL_BOSS_DEPTH,
        xyz=(0.202, HOUSING_DEPTH - CONTROL_BOSS_DEPTH / 2.0, 0.202),
        material="housing_dark",
        name="control_boss",
    )

    guard_y = HOUSING_DEPTH - FRONT_FRAME_DEPTH / 2.0
    for index, z_pos in enumerate((-0.145, -0.075, 0.0, 0.075, 0.145)):
        _add_cylinder(
            housing,
            axis="x",
            radius=GUARD_BAR_RADIUS,
            length=GUARD_SPAN,
            xyz=(0.0, guard_y, z_pos),
            material="guard_dark",
            name=f"guard_bar_x_{index}",
        )
    for index, x_pos in enumerate((-0.145, -0.075, 0.0, 0.075, 0.145)):
        _add_cylinder(
            housing,
            axis="z",
            radius=GUARD_BAR_RADIUS,
            length=GUARD_SPAN,
            xyz=(x_pos, guard_y, 0.0),
            material="guard_dark",
            name=f"guard_bar_z_{index}",
        )

    model.articulation(
        "wall_bracket_to_housing",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=housing,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=-1.0, upper=1.0),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.056,
                5,
                thickness=ROTOR_THICKNESS,
                blade_pitch_deg=27.0,
                blade_sweep_deg=26.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=12.0, camber=0.14, tip_clearance=0.006),
                hub=FanRotorHub(style="domed", rear_collar_height=0.010, rear_collar_radius=0.028, bore_diameter=0.008),
            ),
            "rotor",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="rotor_black",
        name="blade_set",
    )
    _add_cylinder(
        rotor,
        axis="y",
        radius=0.009,
        length=0.026,
        xyz=(0.0, -0.010, 0.0),
        material="accent_silver",
        name="shaft",
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.101, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_HEIGHT,
                body_style="skirted",
                top_diameter=0.034,
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=True,
            ),
            "speed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="knob_black",
        name="knob_cap",
    )
    _add_cylinder(
        knob,
        axis="y",
        radius=0.006,
        length=0.016,
        xyz=(0.0, -0.002, 0.0),
        material="accent_silver",
        name="knob_shaft",
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.202, HOUSING_DEPTH, 0.202)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("knob")
    pan = object_model.get_articulation("wall_bracket_to_housing")
    rotor_joint = object_model.get_articulation("housing_to_rotor")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.allow_overlap(
        rotor,
        housing,
        elem_a="shaft",
        elem_b="motor_pod",
        reason="The rotor shaft is intentionally represented as seated into the motor hub proxy.",
    )
    ctx.allow_overlap(
        knob,
        housing,
        elem_a="knob_shaft",
        elem_b="control_boss",
        reason="The speed knob shaft is intentionally represented as inserted into the front control boss proxy.",
    )
    ctx.expect_gap(
        housing,
        wall_bracket,
        axis="y",
        positive_elem="rear_pad",
        negative_elem="pan_disk",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear pan pad seats on the wall bracket disk",
    )
    ctx.check(
        "pan joint uses a vertical axis",
        tuple(float(v) for v in pan.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan.axis!r}",
    )
    ctx.expect_within(
        rotor,
        housing,
        axes="xz",
        inner_elem="blade_set",
        margin=0.0,
        name="rotor stays within the square housing opening footprint",
    )
    ctx.expect_overlap(
        knob,
        housing,
        elem_a="knob_shaft",
        elem_b="control_boss",
        axes="xz",
        min_overlap=0.010,
        name="speed knob shaft stays seated in the control boss",
    )
    ctx.check(
        "rotor joint uses the fan axle direction",
        tuple(float(v) for v in rotor_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={rotor_joint.axis!r}",
    )
    ctx.check(
        "knob joint uses the front shaft direction",
        tuple(float(v) for v in knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={knob_joint.axis!r}",
    )

    rest_knob = ctx.part_world_position(knob)
    with ctx.pose({pan: pan.motion_limits.upper}):
        panned_knob = ctx.part_world_position(knob)
    ctx.check(
        "housing pan moves the front corner sideways",
        rest_knob is not None
        and panned_knob is not None
        and abs(panned_knob[0] - rest_knob[0]) > 0.10,
        details=f"rest={rest_knob}, panned={panned_knob}",
    )

    return ctx.report()


object_model = build_object_model()

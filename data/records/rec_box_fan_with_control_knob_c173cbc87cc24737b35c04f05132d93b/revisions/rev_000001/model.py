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

HOUSING_SIZE = 0.54
HOUSING_DEPTH = 0.18
HOUSING_WALL = 0.018
PIVOT_HEIGHT = 0.34
PIVOT_PIN_RADIUS = 0.015
PIVOT_PIN_LENGTH = 0.06

CRADLE_TUBE_RADIUS = 0.015
CRADLE_RUNNER_Y = 0.30
CRADLE_RUNNER_LENGTH = 0.44
CRADLE_CROSS_X = 0.15
CRADLE_SLEEVE_RADIUS = 0.024
CRADLE_SLEEVE_BORE = PIVOT_PIN_RADIUS + 0.002
CRADLE_SLEEVE_LENGTH = 0.06

GRILLE_SIZE = HOUSING_SIZE - 2.0 * HOUSING_WALL - 0.010
GRILLE_THICKNESS = 0.004
GRILLE_FRAME = 0.014
GRILLE_BAR = 0.004
SELECTOR_X = 0.075
SELECTOR_Z = -0.145


def _centered_cylinder(
    axis: str,
    radius: float,
    length: float,
    center: tuple[float, float, float],
):
    solid = cq.Workplane("XY").circle(radius).extrude(length * 0.5, both=True).val()
    if axis == "x":
        solid = solid.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    elif axis == "y":
        solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    elif axis != "z":
        raise ValueError(f"Unsupported axis: {axis}")
    return solid.translate(center)


def _centered_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).val().translate(center)


def _fuse(base, other):
    return base.fuse(other)


def _build_housing_shape():
    outer = (
        cq.Workplane("XY")
        .box(HOUSING_DEPTH, HOUSING_SIZE, HOUSING_SIZE)
        .edges("|X")
        .fillet(0.028)
        .val()
    )
    inner = cq.Workplane("XY").box(
        HOUSING_DEPTH + 0.01,
        HOUSING_SIZE - 2.0 * HOUSING_WALL,
        HOUSING_SIZE - 2.0 * HOUSING_WALL,
    ).val()
    shell = outer.cut(inner)
    return shell


def _build_cradle_shape():
    runner_z = CRADLE_TUBE_RADIUS
    upright_length = PIVOT_HEIGHT - CRADLE_SLEEVE_RADIUS + 0.004 - runner_z
    upright_center_z = runner_z + upright_length * 0.5

    cradle = _centered_cylinder(
        "x",
        CRADLE_TUBE_RADIUS,
        CRADLE_RUNNER_LENGTH,
        (0.0, CRADLE_RUNNER_Y, runner_z),
    )
    cradle = _fuse(
        cradle,
        _centered_cylinder(
            "x",
            CRADLE_TUBE_RADIUS,
            CRADLE_RUNNER_LENGTH,
            (0.0, -CRADLE_RUNNER_Y, runner_z),
        )
    )
    cradle = _fuse(
        cradle,
        _centered_cylinder(
            "y",
            CRADLE_TUBE_RADIUS,
            CRADLE_RUNNER_Y * 2.0,
            (CRADLE_CROSS_X, 0.0, runner_z),
        )
    )
    cradle = _fuse(
        cradle,
        _centered_cylinder(
            "y",
            CRADLE_TUBE_RADIUS,
            CRADLE_RUNNER_Y * 2.0,
            (-CRADLE_CROSS_X, 0.0, runner_z),
        )
    )

    for sign in (-1.0, 1.0):
        cradle = _fuse(
            cradle,
            _centered_cylinder(
                "z",
                CRADLE_TUBE_RADIUS,
                upright_length,
                (0.0, sign * CRADLE_RUNNER_Y, upright_center_z),
            )
        )

    return cradle


def _build_pivot_sleeve_shape():
    sleeve = _centered_cylinder("y", CRADLE_SLEEVE_RADIUS, CRADLE_SLEEVE_LENGTH, (0.0, 0.0, 0.0))
    return sleeve.cut(
        _centered_cylinder("y", CRADLE_SLEEVE_BORE, CRADLE_SLEEVE_LENGTH + 0.01, (0.0, 0.0, 0.0))
    )


def _build_grille_shape(*, with_motor_mount: bool):
    span = GRILLE_SIZE - 2.0 * GRILLE_FRAME
    grille = _centered_box((GRILLE_THICKNESS, GRILLE_SIZE, GRILLE_SIZE), (0.0, 0.0, 0.0))
    grille = grille.cut(
        _centered_box(
            (GRILLE_THICKNESS + 0.01, span, span),
            (0.0, 0.0, 0.0),
        )
    )

    bar_positions = (-0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18)
    for y in bar_positions:
        grille = _fuse(
            grille,
            _centered_box((GRILLE_THICKNESS, GRILLE_BAR, span), (0.0, y, 0.0))
        )
    for z in bar_positions:
        grille = _fuse(
            grille,
            _centered_box((GRILLE_THICKNESS, span, GRILLE_BAR), (0.0, 0.0, z))
        )

    outer_half = GRILLE_SIZE * 0.5
    tab_offset = outer_half + 0.001
    grille = _fuse(
        grille,
        _centered_box((GRILLE_THICKNESS, 0.008, 0.050), (0.0, tab_offset, 0.0)),
    )
    grille = _fuse(
        grille,
        _centered_box((GRILLE_THICKNESS, 0.008, 0.050), (0.0, -tab_offset, 0.0)),
    )
    grille = _fuse(
        grille,
        _centered_box((GRILLE_THICKNESS, 0.050, 0.008), (0.0, 0.0, tab_offset)),
    )
    grille = _fuse(
        grille,
        _centered_box((GRILLE_THICKNESS, 0.050, 0.008), (0.0, 0.0, -tab_offset)),
    )

    if with_motor_mount:
        grille = _fuse(
            grille,
            _centered_box((0.006, 0.016, span * 0.70), (0.008, 0.0, 0.0))
        )
        grille = _fuse(
            grille,
            _centered_box((0.006, span * 0.70, 0.016), (0.008, 0.0, 0.0))
        )
        grille = _fuse(
            grille,
            _centered_cylinder("x", 0.044, 0.048, (0.024, 0.0, 0.0))
        )

    return grille


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_box_fan")

    cradle_finish = model.material("cradle_finish", rgba=(0.74, 0.76, 0.78, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.70, 0.72, 0.74, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    indicator_finish = model.material("indicator_finish", rgba=(0.83, 0.64, 0.20, 1.0))

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_build_cradle_shape(), "cradle"),
        material=cradle_finish,
        name="cradle_tube",
    )
    pivot_sleeve_mesh = mesh_from_cadquery(_build_pivot_sleeve_shape(), "pivot_sleeve")
    cradle.visual(
        pivot_sleeve_mesh,
        origin=Origin(xyz=(0.0, -CRADLE_RUNNER_Y, PIVOT_HEIGHT)),
        material=cradle_finish,
        name="pivot_sleeve_0",
    )
    cradle.visual(
        pivot_sleeve_mesh,
        origin=Origin(xyz=(0.0, CRADLE_RUNNER_Y, PIVOT_HEIGHT)),
        material=cradle_finish,
        name="pivot_sleeve_1",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "housing"),
        material=housing_finish,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=PIVOT_PIN_RADIUS, length=PIVOT_PIN_LENGTH),
        origin=Origin(
            xyz=(0.0, -(HOUSING_SIZE * 0.5 + PIVOT_PIN_LENGTH * 0.5), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_finish,
        name="pivot_pin_0",
    )
    housing.visual(
        Cylinder(radius=PIVOT_PIN_RADIUS, length=PIVOT_PIN_LENGTH),
        origin=Origin(
            xyz=(0.0, HOUSING_SIZE * 0.5 + PIVOT_PIN_LENGTH * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_finish,
        name="pivot_pin_1",
    )
    housing.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(
            xyz=(SELECTOR_X, HOUSING_SIZE * 0.5 + 0.005, SELECTOR_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_finish,
        name="selector_boss",
    )

    front_grille = model.part("front_grille")
    front_grille.visual(
        mesh_from_cadquery(_build_grille_shape(with_motor_mount=False), "front_grille"),
        material=grille_finish,
        name="front_guard",
    )

    rear_grille = model.part("rear_grille")
    rear_grille.visual(
        mesh_from_cadquery(_build_grille_shape(with_motor_mount=True), "rear_grille"),
        material=grille_finish,
        name="rear_guard",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.205,
                0.050,
                5,
                thickness=0.022,
                blade_pitch_deg=30.0,
                blade_sweep_deg=20.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=14.0, camber=0.14),
                hub=FanRotorHub(
                    style="capped",
                    rear_collar_height=0.010,
                    rear_collar_radius=0.032,
                    bore_diameter=0.010,
                ),
            ),
            "rotor",
        ),
        material=rotor_finish,
        name="rotor_blades",
    )
    rotor.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=rotor_finish,
        name="shaft",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=knob_finish,
        name="shaft",
    )
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.076,
                0.038,
                body_style="skirted",
                top_diameter=0.058,
                edge_radius=0.004,
                crown_radius=0.006,
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=knob_finish,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.003, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, 0.024, 0.061)),
        material=indicator_finish,
        name="indicator",
    )

    model.articulation(
        "cradle_to_housing",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.75,
        ),
    )

    model.articulation(
        "housing_to_front_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(HOUSING_DEPTH * 0.5 - 0.014, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_rear_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_grille,
        origin=Origin(xyz=(-(HOUSING_DEPTH * 0.5 - 0.010), 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector_knob,
        origin=Origin(
            xyz=(SELECTOR_X, HOUSING_SIZE * 0.5, SELECTOR_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rear_grille = object_model.get_part("rear_grille")
    rotor = object_model.get_part("rotor")
    selector_knob = object_model.get_part("selector_knob")

    tilt_joint = object_model.get_articulation("cradle_to_housing")
    rotor_joint = object_model.get_articulation("housing_to_rotor")
    knob_joint = object_model.get_articulation("housing_to_selector_knob")

    ctx.allow_overlap(
        housing,
        front_grille,
        elem_a="housing_shell",
        elem_b="front_guard",
        reason="The front grille uses simplified hidden mounting tabs that tuck into the housing lip.",
    )
    ctx.allow_overlap(
        housing,
        rear_grille,
        elem_a="housing_shell",
        elem_b="rear_guard",
        reason="The rear grille uses simplified hidden mounting tabs that tuck into the housing lip.",
    )
    ctx.allow_overlap(
        rear_grille,
        rotor,
        elem_a="rear_guard",
        elem_b="shaft",
        reason="The rotor shaft is intentionally represented as seated inside the rear motor-hub proxy.",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="yz",
        margin=0.0,
        name="rotor stays within the square housing opening",
    )
    ctx.expect_gap(
        front_grille,
        rotor,
        axis="x",
        min_gap=0.050,
        name="front guard sits forward of the rotor",
    )
    ctx.expect_gap(
        rotor,
        rear_grille,
        axis="x",
        positive_elem="rotor_blades",
        negative_elem="rear_guard",
        min_gap=0.012,
        name="rear guard clears the spinning blades",
    )
    ctx.expect_contact(
        selector_knob,
        housing,
        elem_a="shaft",
        elem_b="selector_boss",
        name="selector knob shaft seats against the side boss",
    )
    ctx.expect_contact(
        rotor,
        rear_grille,
        elem_a="shaft",
        elem_b="rear_guard",
        name="rotor shaft seats in the rear motor hub",
    )

    ctx.check(
        "rotor uses continuous spin articulation",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and rotor_joint.motion_limits is not None
        and rotor_joint.motion_limits.lower is None
        and rotor_joint.motion_limits.upper is None,
        details=f"type={rotor_joint.articulation_type}, limits={rotor_joint.motion_limits}",
    )
    ctx.check(
        "selector knob uses continuous spin articulation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )

    rest_front_position = ctx.part_world_position(front_grille)
    upper_tilt = tilt_joint.motion_limits.upper if tilt_joint.motion_limits is not None else 0.75
    with ctx.pose({tilt_joint: upper_tilt}):
        tilted_front_position = ctx.part_world_position(front_grille)

    ctx.check(
        "positive housing tilt raises the front opening",
        rest_front_position is not None
        and tilted_front_position is not None
        and tilted_front_position[2] > rest_front_position[2] + 0.04,
        details=f"rest={rest_front_position}, tilted={tilted_front_position}",
    )

    return ctx.report()


object_model = build_object_model()

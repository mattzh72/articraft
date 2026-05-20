from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.078
BODY_DEPTH = 0.034
BODY_HEIGHT = 0.172

BUMPER_WIDTH = 0.092
BUMPER_DEPTH = 0.046
BUMPER_HEIGHT = 0.192

FRONT_Y = BODY_DEPTH * 0.5
BACK_Y = -BODY_DEPTH * 0.5

DIAL_Z = 0.086
DISPLAY_Z = 0.143
BUTTON_Z = 0.115

BUTTON_XS = (-0.024, -0.008, 0.008, 0.024)
BUTTON_TRAVEL = 0.0018

STAND_HINGE_Z = 0.030
STAND_BARREL_RADIUS = 0.0036


def _box(size: tuple[float, float, float], *, z0: float = 0.0):
    sx, sy, sz = size
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((0.0, 0.0, z0))
    )


def _cylinder_y(radius: float, length: float):
    return (
        cq.Workplane("XY")
        .cylinder(length, radius, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _build_housing():
    housing = _box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)).edges().fillet(0.005)
    housing = housing.cut(_box((0.062, 0.007, 0.036), z0=0.124).translate((0.0, 0.0135, 0.0)))
    housing = housing.cut(_cylinder_y(0.0295, 0.007).translate((0.0, 0.0135, DIAL_Z)))

    for x in BUTTON_XS:
        housing = housing.cut(
            _box((0.016, 0.010, 0.010), z0=BUTTON_Z - 0.005).translate((x, 0.0125, 0.0))
        )

    return housing


def _build_bumper():
    bumper = _box((BUMPER_WIDTH, BUMPER_DEPTH, BUMPER_HEIGHT)).edges().fillet(0.009)
    inner_cavity = _box((0.077, 0.032, 0.178), z0=0.006)
    front_window = _box((0.074, 0.030, 0.164), z0=0.014).translate((0.0, 0.011, 0.0))
    back_window = _box((0.068, 0.031, 0.134), z0=0.024).translate((0.0, -0.011, 0.0))
    return bumper.cut(inner_cavity).cut(front_window).cut(back_window)


def _build_stand_frame():
    outer = _box((0.052, 0.004, 0.104), z0=0.006)
    inner = _box((0.030, 0.010, 0.064), z0=0.024)
    foot = _box((0.040, 0.004, 0.014), z0=0.006)
    bridge = _box((0.034, 0.004, 0.012), z0=0.0)
    return outer.cut(inner).union(foot).union(bridge).edges("|Y").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_multimeter")

    housing_plastic = model.material("housing_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    bumper_rubber = model.material("bumper_rubber", rgba=(0.93, 0.71, 0.12, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("display_glass", rgba=(0.41, 0.56, 0.42, 0.55))
    button_blue = model.material("button_blue", rgba=(0.25, 0.38, 0.55, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    jack_black = model.material("jack_black", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_housing(), "multimeter_housing"),
        material=housing_plastic,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(_build_bumper(), "multimeter_bumper"),
        material=bumper_rubber,
        name="bumper",
    )
    body.visual(
        Box((0.060, 0.0045, 0.034)),
        origin=Origin(xyz=(0.0, 0.0148, DISPLAY_Z)),
        material=panel_dark,
        name="display_bezel",
    )
    body.visual(
        Box((0.052, 0.0024, 0.024)),
        origin=Origin(xyz=(0.0, 0.0158, DISPLAY_Z)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0154, DIAL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel_dark,
        name="dial_bezel",
    )
    body.visual(
        Box((0.064, 0.0035, 0.020)),
        origin=Origin(xyz=(0.0, 0.0152, 0.028)),
        material=panel_dark,
        name="jack_panel",
    )
    for index, x in enumerate((-0.022, 0.0, 0.022)):
        body.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(x, 0.0160, 0.028), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=jack_black,
            name=f"jack_{index}",
        )

    hinge_axis_y = BACK_Y - STAND_BARREL_RADIUS
    for index, x in enumerate((-0.023, 0.023)):
        body.visual(
            Box((0.010, 0.0045, 0.013)),
            origin=Origin(xyz=(x, BACK_Y - 0.0017, STAND_HINGE_Z + 0.0035)),
            material=housing_plastic,
            name=f"stand_bracket_{index}",
        )
        body.visual(
            Cylinder(radius=0.0032, length=0.011),
            origin=Origin(xyz=(x, hinge_axis_y, STAND_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=housing_plastic,
            name=f"stand_lug_{index}",
        )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.014,
                body_style="tapered",
                base_diameter=0.052,
                top_diameter=0.046,
                edge_radius=0.0015,
                center=False,
            ),
            "multimeter_selector",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="selector_knob",
    )
    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0035, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_build_stand_frame(), "multimeter_stand_frame"),
        origin=Origin(xyz=(0.0, STAND_BARREL_RADIUS - 0.002, 0.0)),
        material=panel_dark,
        name="stand_frame",
    )
    stand.visual(
        Cylinder(radius=STAND_BARREL_RADIUS, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=panel_dark,
        name="stand_barrel",
    )
    model.articulation(
        "stand_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, hinge_axis_y, STAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.95),
    )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.014, 0.0034, 0.0080)),
            origin=Origin(xyz=(0.0, 0.0017, 0.0)),
            material=button_blue,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.0055, 0.0055)),
            origin=Origin(xyz=(0.0, -0.00275, 0.0)),
            material=button_blue,
            name="button_stem",
        )
        model.articulation(
            f"button_press_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.0003, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    selector_joint = object_model.get_articulation("selector_spin")
    stand_joint = object_model.get_articulation("stand_hinge")

    ctx.check(
        "selector uses continuous rotation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(selector_joint.articulation_type),
    )
    ctx.check(
        "rear stand uses revolute hinge",
        stand_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(stand_joint.articulation_type),
    )

    ctx.expect_overlap(
        selector,
        body,
        axes="xz",
        elem_a="selector_knob",
        elem_b="dial_bezel",
        min_overlap=0.040,
        name="selector sits inside the front dial bezel",
    )

    with ctx.pose({stand_joint: 0.0}):
        ctx.expect_gap(
            body,
            stand,
            axis="y",
            positive_elem="housing",
            negative_elem="stand_frame",
            max_gap=0.001,
            max_penetration=0.0,
            name="stand folds flat against the rear housing",
        )

    closed_aabb = None
    with ctx.pose({stand_joint: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(stand, elem="stand_frame")

    with ctx.pose({stand_joint: 0.85}):
        open_aabb = ctx.part_element_world_aabb(stand, elem="stand_frame")
        ctx.check(
            "stand swings rearward into a tilt position",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.050,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    with ctx.pose({selector_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            selector,
            body,
            axes="xz",
            elem_a="selector_knob",
            elem_b="dial_bezel",
            min_overlap=0.040,
            name="selector stays centered while rotated",
        )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"button_press_{index}")

        ctx.check(
            f"button_{index} uses prismatic travel",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=str(button_joint.articulation_type),
        )
        ctx.expect_gap(
            body,
            button,
            axis="z",
            positive_elem="display_window",
            negative_elem="button_cap",
            min_gap=0.010,
            max_gap=0.032,
            name=f"display sits above button_{index}",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} presses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] < rest_pos[1] - 0.001,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    return ctx.report()


object_model = build_object_model()

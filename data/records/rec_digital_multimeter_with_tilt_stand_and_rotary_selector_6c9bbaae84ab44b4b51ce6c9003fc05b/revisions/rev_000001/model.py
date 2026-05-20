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


BODY_WIDTH = 0.078
BODY_DEPTH = 0.034
BODY_HEIGHT = 0.154
FRONT_Y = BODY_DEPTH * 0.5
BACK_Y = -BODY_DEPTH * 0.5
HINGE_RADIUS = 0.0032
HINGE_Z = -0.070


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length * 0.5, center[1], center[2]))
    )


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length * 0.5, center[2]))
    )


def _make_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
    return shell.edges("|Y").fillet(0.0105)


def _make_bumper_frame() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_WIDTH + 0.010, BODY_DEPTH + 0.006, BODY_HEIGHT + 0.010)
    outer = outer.edges("|Y").fillet(0.0125)
    inner = cq.Workplane("XY").box(BODY_WIDTH - 0.004, BODY_DEPTH + 0.020, BODY_HEIGHT - 0.018)
    return outer.cut(inner)


def _make_stand() -> cq.Workplane:
    stand_plate = _box_solid((0.046, 0.0032, 0.070), (0.000, -0.0016, 0.036))
    stand_plate = stand_plate.cut(_box_solid((0.024, 0.008, 0.038), (0.000, -0.0016, 0.041)))
    top_foot = _box_solid((0.036, 0.0046, 0.008), (0.000, -0.0023, 0.068))
    hinge_barrel = _x_cylinder(HINGE_RADIUS, 0.022, (0.000, 0.000, 0.000))
    return stand_plate.union(top_foot).union(hinge_barrel)


def _make_selector_mesh():
    selector = KnobGeometry(
        0.040,
        0.014,
        body_style="skirted",
        top_diameter=0.034,
        skirt=KnobSkirt(0.046, 0.0028, flare=0.05),
        grip=KnobGrip(style="fluted", count=16, depth=0.0009),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
        center=False,
    )
    return mesh_from_geometry(selector, "multimeter_selector")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_digital_multimeter")

    shell_mat = model.material("shell_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    bumper_mat = model.material("bumper_amber", rgba=(0.86, 0.70, 0.20, 1.0))
    bezel_mat = model.material("bezel_grey", rgba=(0.30, 0.32, 0.35, 1.0))
    glass_mat = model.material("display_glass", rgba=(0.48, 0.70, 0.42, 0.42))
    selector_mat = model.material("selector_black", rgba=(0.09, 0.09, 0.10, 1.0))
    button_mat = model.material("button_grey", rgba=(0.36, 0.39, 0.42, 1.0))
    jack_black_mat = model.material("jack_black", rgba=(0.08, 0.08, 0.09, 1.0))
    jack_red_mat = model.material("jack_red", rgba=(0.73, 0.10, 0.09, 1.0))
    stand_mat = model.material("stand_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "multimeter_body_shell"),
        material=shell_mat,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_make_bumper_frame(), "multimeter_bumper"),
        material=bumper_mat,
        name="bumper",
    )
    body.visual(
        Box((0.062, 0.0016, 0.126)),
        origin=Origin(xyz=(0.000, FRONT_Y - 0.0008, -0.004)),
        material=bezel_mat,
        name="faceplate",
    )
    body.visual(
        Box((0.044, 0.0018, 0.024)),
        origin=Origin(xyz=(0.000, FRONT_Y - 0.0002, 0.044)),
        material=glass_mat,
        name="display",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.0014),
        origin=Origin(xyz=(0.000, FRONT_Y - 0.0007, -0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bezel_mat,
        name="selector_bezel",
    )

    jack_positions = (-0.0255, -0.0085, 0.0085, 0.0255)
    jack_materials = (jack_black_mat, jack_black_mat, jack_red_mat, jack_red_mat)
    for index, (x_pos, jack_mat) in enumerate(zip(jack_positions, jack_materials)):
        body.visual(
            Cylinder(radius=0.0057, length=0.0014),
            origin=Origin(
                xyz=(x_pos, FRONT_Y - 0.0005, -0.065),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=jack_mat,
            name=f"jack_collar_{index}",
        )
        body.visual(
            Cylinder(radius=0.0038, length=0.0046),
            origin=Origin(
                xyz=(x_pos, FRONT_Y - 0.0023, -0.065),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=jack_black_mat,
            name=f"jack_socket_{index}",
        )

    selector = model.part("selector")
    selector.visual(
        _make_selector_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=selector_mat,
        name="selector_cap",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.000, FRONT_Y, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
        ),
    )

    button_x_positions = (-0.0185, 0.000, 0.0185)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.014, 0.0016, 0.010)),
            origin=Origin(xyz=(0.000, 0.0008, 0.000)),
            material=button_mat,
            name="flange",
        )
        button.visual(
            Box((0.0105, 0.0036, 0.006)),
            origin=Origin(xyz=(0.000, 0.0018, 0.000)),
            material=button_mat,
            name="cap",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, FRONT_Y, -0.041)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0016,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand(), "multimeter_stand"),
        material=stand_mat,
        name="kickstand",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.000, BACK_Y - 0.006199, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.check(
        "selector uses continuous rotation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_joint.articulation_type}",
    )
    ctx.check(
        "selector rotates about front-back axis",
        tuple(selector_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={selector_joint.axis}",
    )
    ctx.expect_contact(
        selector,
        body,
        name="selector mounts against the front housing",
    )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        with ctx.pose({stand_joint: 0.0}):
            ctx.expect_gap(
                body,
                stand,
                axis="y",
                min_gap=0.0,
                max_gap=0.004,
                name="stand folds close to the back",
            )
            rest_stand_aabb = ctx.part_world_aabb(stand)
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_stand_aabb = ctx.part_world_aabb(stand)
            ctx.check(
                "stand swings backward when opened",
                rest_stand_aabb is not None
                and open_stand_aabb is not None
                and open_stand_aabb[0][1] < rest_stand_aabb[0][1] - 0.04
                and open_stand_aabb[1][2] < rest_stand_aabb[1][2] - 0.02,
                details=f"rest={rest_stand_aabb}, open={open_stand_aabb}",
            )

    rest_button_positions = {
        index: ctx.part_world_position(object_model.get_part(f"mode_button_{index}"))
        for index in range(3)
    }
    for index in range(3):
        button = object_model.get_part(f"mode_button_{index}")
        joint = object_model.get_articulation(f"body_to_mode_button_{index}")
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            other_positions = {
                other_index: ctx.part_world_position(object_model.get_part(f"mode_button_{other_index}"))
                for other_index in range(3)
            }
        rest_position = rest_button_positions[index]
        ctx.check(
            f"mode_button_{index} presses inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] < rest_position[1] - 0.001,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )
        for other_index in range(3):
            if other_index == index:
                continue
            other_rest = rest_button_positions[other_index]
            other_pressed = other_positions[other_index]
            ctx.check(
                f"mode_button_{index} does not drive mode_button_{other_index}",
                other_rest is not None
                and other_pressed is not None
                and abs(other_pressed[1] - other_rest[1]) < 1e-6,
                details=f"rest={other_rest}, posed={other_pressed}",
            )

    return ctx.report()


object_model = build_object_model()

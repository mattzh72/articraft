from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    WheelGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.315
BODY_D = 0.246
BODY_H = 0.540
TOP_W = 0.275
TOP_D = 0.210
SHELL_WALL = 0.014

SLEEVE_X = 0.084
SLEEVE_Y = -0.132
SLEEVE_BOTTOM = 0.240
SLEEVE_TOP = 0.470
SLEEVE_LEN = SLEEVE_TOP - SLEEVE_BOTTOM
SLEEVE_OUTER_R = 0.0090
SLEEVE_INNER_R = 0.0062

HANDLE_TRAVEL = 0.120
HANDLE_RAIL_R = 0.0055
HANDLE_RAIL_LEN = 0.360
HANDLE_RAIL_CENTER_Z = 0.020

WHEEL_RADIUS = 0.043
WHEEL_WIDTH = 0.030


def _body_shell_mesh():
    outer = (
        cq.Workplane("XY")
        .rect(BODY_W, BODY_D)
        .workplane(offset=BODY_H)
        .rect(TOP_W, TOP_D)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.018)
        .rect(BODY_W - 2.0 * SHELL_WALL, BODY_D - 2.0 * SHELL_WALL)
        .workplane(offset=BODY_H - 0.036)
        .rect(TOP_W - 2.0 * SHELL_WALL, TOP_D - 2.0 * SHELL_WALL)
        .loft(combine=True)
    )

    front_opening = (
        cq.Workplane("XY")
        .box(0.236, 0.085, 0.332)
        .translate((0.0, BODY_D * 0.5 - 0.020, 0.220))
    )

    return outer.cut(inner).cut(front_opening)


def _tube_mesh(outer_r: float, inner_r: float, length: float):
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(length)


def _rounded_box_mesh(size_x: float, size_y: float, size_z: float, fillet: float):
    return cq.Workplane("XY").box(size_x, size_y, size_z).edges("|Z").fillet(fillet)


def _build_wheel_part(model: ArticulatedObject, name: str, tire_mesh, rim_mesh, rubber, wheel_core):
    wheel = model.part(name)
    wheel.visual(tire_mesh, material=rubber, name="tire")
    wheel.visual(rim_mesh, material=wheel_core, name="rim")
    wheel.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_core,
        name="hub_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.28,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="karaoke_speaker")

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    grille_black = model.material("grille_black", rgba=(0.10, 0.10, 0.11, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    button_grey = model.material("button_grey", rgba=(0.56, 0.58, 0.61, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.56, 0.59, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell_mesh(), "speaker_body_shell"), material=body_plastic, name="shell")
    body.visual(
        mesh_from_cadquery(_rounded_box_mesh(0.170, 0.110, 0.010, 0.010), "speaker_top_deck"),
        origin=Origin(xyz=(0.000, -0.006, BODY_H - 0.005)),
        material=trim_black,
        name="top_deck",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box_mesh(0.160, 0.014, 0.070, 0.004), "speaker_control_bezel"),
        origin=Origin(xyz=(0.000, 0.114, 0.433)),
        material=trim_black,
        name="control_bezel",
    )
    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.225, 0.315),
                0.0035,
                hole_diameter=0.005,
                pitch=(0.010, 0.010),
                frame=0.010,
                corner_radius=0.006,
                stagger=True,
                center=False,
            ),
            "speaker_grille",
        ),
        origin=Origin(xyz=(0.000, 0.114, 0.220), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grille_black,
        name="speaker_grille",
    )
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").rect(0.243, 0.333).rect(0.213, 0.303).extrude(0.004),
            "speaker_grille_frame",
        ),
        origin=Origin(xyz=(0.000, 0.112, 0.220), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="grille_frame",
    )
    body.visual(
        Box((0.162, 0.024, 0.064)),
        origin=Origin(xyz=(0.000, -0.121, 0.414)),
        material=trim_black,
        name="rail_bridge",
    )
    body.visual(
        Box((0.224, 0.020, 0.080)),
        origin=Origin(xyz=(0.000, -0.113, 0.062)),
        material=trim_black,
        name="rear_bumper",
    )
    body.visual(
        Box((0.022, 0.054, 0.050)),
        origin=Origin(xyz=(-0.147, -0.141, 0.046)),
        material=trim_black,
        name="left_axle_bracket",
    )
    body.visual(
        Box((0.022, 0.054, 0.050)),
        origin=Origin(xyz=(0.147, -0.141, 0.046)),
        material=trim_black,
        name="right_axle_bracket",
    )
    body.visual(
        Box((0.052, 0.030, 0.014)),
        origin=Origin(xyz=(-0.090, 0.075, 0.007)),
        material=trim_black,
        name="foot_0",
    )
    body.visual(
        Box((0.052, 0.030, 0.014)),
        origin=Origin(xyz=(0.090, 0.075, 0.007)),
        material=trim_black,
        name="foot_1",
    )

    sleeve_mesh = mesh_from_cadquery(_tube_mesh(SLEEVE_OUTER_R, SLEEVE_INNER_R, SLEEVE_LEN), "handle_sleeve")
    body.visual(
        sleeve_mesh,
        origin=Origin(xyz=(-SLEEVE_X, SLEEVE_Y, SLEEVE_BOTTOM)),
        material=handle_metal,
        name="left_sleeve",
    )
    body.visual(
        sleeve_mesh,
        origin=Origin(xyz=(SLEEVE_X, SLEEVE_Y, SLEEVE_BOTTOM)),
        material=handle_metal,
        name="right_sleeve",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=8.6,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    handle = model.part("trolley_handle")
    handle.visual(
        Cylinder(radius=HANDLE_RAIL_R, length=HANDLE_RAIL_LEN),
        origin=Origin(xyz=(-SLEEVE_X, 0.0, HANDLE_RAIL_CENTER_Z)),
        material=handle_metal,
        name="inner_rail_left",
    )
    handle.visual(
        Cylinder(radius=HANDLE_RAIL_R, length=HANDLE_RAIL_LEN),
        origin=Origin(xyz=(SLEEVE_X, 0.0, HANDLE_RAIL_CENTER_Z)),
        material=handle_metal,
        name="inner_rail_right",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.194), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="crossbar",
    )
    handle.visual(
        mesh_from_cadquery(_rounded_box_mesh(0.132, 0.030, 0.024, 0.007), "speaker_handle_grip"),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        material=trim_black,
        name="grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.182, 0.030, 0.384)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    wheel_tire_mesh = mesh_from_geometry(
        TireGeometry(WHEEL_RADIUS, WHEEL_WIDTH, inner_radius=0.030),
        "speaker_wheel_tire",
    )
    wheel_rim_mesh = mesh_from_geometry(
        WheelGeometry(0.033, 0.024),
        "speaker_wheel_rim",
    )
    left_wheel = _build_wheel_part(model, "left_wheel", wheel_tire_mesh, wheel_rim_mesh, rubber, wheel_core)
    right_wheel = _build_wheel_part(model, "right_wheel", wheel_tire_mesh, wheel_rim_mesh, rubber, wheel_core)

    knob = model.part("main_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.022,
                body_style="skirted",
                top_diameter=0.036,
                base_diameter=0.048,
                edge_radius=0.0015,
                center=False,
            ),
            "main_knob",
        ),
        material=knob_black,
        name="knob",
    )
    knob.visual(
        Box((0.003, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, 0.016, 0.019)),
        material=button_grey,
        name="indicator",
    )

    for index, x_pos in enumerate((-0.050, 0.050)):
        mode_button = model.part(f"mode_button_{index}")
        mode_button.visual(
            Cylinder(radius=0.0115, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=button_grey,
            name="cap",
        )

    for index, x_pos in enumerate((-0.045, 0.000, 0.045)):
        front_button = model.part(f"front_button_{index}")
        front_button.visual(
            Cylinder(radius=0.0105, length=0.009),
            origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_grey,
            name="cap",
        )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, SLEEVE_Y, SLEEVE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.20, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-0.173, -0.166, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(0.173, -0.166, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "body_to_main_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, -0.004, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )

    for index, x_pos in enumerate((-0.050, 0.050)):
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"mode_button_{index}",
            origin=Origin(xyz=(x_pos, 0.026, BODY_H)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.10, lower=0.0, upper=0.0025),
        )

    for index, x_pos in enumerate((-0.045, 0.000, 0.045)):
        model.articulation(
            f"body_to_front_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"front_button_{index}",
            origin=Origin(xyz=(x_pos, 0.121, 0.433)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.10, lower=0.0, upper=0.0030),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("trolley_handle")
    button_0 = object_model.get_part("front_button_0")
    button_1 = object_model.get_part("front_button_1")
    button_2 = object_model.get_part("front_button_2")
    mode_button_0 = object_model.get_part("mode_button_0")

    handle_joint = object_model.get_articulation("body_to_handle")
    left_wheel_joint = object_model.get_articulation("body_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("body_to_right_wheel")
    knob_joint = object_model.get_articulation("body_to_main_knob")
    button_0_joint = object_model.get_articulation("body_to_front_button_0")
    button_1_joint = object_model.get_articulation("body_to_front_button_1")
    button_2_joint = object_model.get_articulation("body_to_front_button_2")
    mode_button_0_joint = object_model.get_articulation("body_to_mode_button_0")
    mode_button_1_joint = object_model.get_articulation("body_to_mode_button_1")

    ctx.check(
        "speaker control articulation types",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and handle_joint.articulation_type == ArticulationType.PRISMATIC
        and button_0_joint.articulation_type == ArticulationType.PRISMATIC
        and button_1_joint.articulation_type == ArticulationType.PRISMATIC
        and button_2_joint.articulation_type == ArticulationType.PRISMATIC
        and mode_button_0_joint.articulation_type == ArticulationType.PRISMATIC
        and mode_button_1_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"knob={knob_joint.articulation_type}, "
            f"left_wheel={left_wheel_joint.articulation_type}, "
            f"right_wheel={right_wheel_joint.articulation_type}, "
            f"handle={handle_joint.articulation_type}"
        ),
    )

    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None
    if handle_upper is not None:
        with ctx.pose({handle_joint: 0.0}):
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="inner_rail_left",
                outer_elem="left_sleeve",
                margin=0.002,
                name="left handle rail stays centered in sleeve at rest",
            )
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="inner_rail_right",
                outer_elem="right_sleeve",
                margin=0.002,
                name="right handle rail stays centered in sleeve at rest",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="inner_rail_left",
                elem_b="left_sleeve",
                min_overlap=0.150,
                name="left handle rail remains inserted when collapsed",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="inner_rail_right",
                elem_b="right_sleeve",
                min_overlap=0.150,
                name="right handle rail remains inserted when collapsed",
            )
            handle_rest = ctx.part_world_position(handle)

        with ctx.pose({handle_joint: handle_upper}):
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="inner_rail_left",
                outer_elem="left_sleeve",
                margin=0.002,
                name="left handle rail stays centered when extended",
            )
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="inner_rail_right",
                outer_elem="right_sleeve",
                margin=0.002,
                name="right handle rail stays centered when extended",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="inner_rail_left",
                elem_b="left_sleeve",
                min_overlap=0.035,
                name="left handle rail keeps retained insertion at full extension",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="inner_rail_right",
                elem_b="right_sleeve",
                min_overlap=0.035,
                name="right handle rail keeps retained insertion at full extension",
            )
            handle_extended = ctx.part_world_position(handle)

        ctx.check(
            "trolley handle rises upward",
            handle_rest is not None
            and handle_extended is not None
            and handle_extended[2] > handle_rest[2] + 0.08,
            details=f"rest={handle_rest}, extended={handle_extended}",
        )

    ctx.expect_within(
        button_0,
        body,
        axes="xz",
        inner_elem="cap",
        outer_elem="control_bezel",
        margin=0.006,
        name="left front button sits within control bezel footprint",
    )
    ctx.expect_within(
        button_1,
        body,
        axes="xz",
        inner_elem="cap",
        outer_elem="control_bezel",
        margin=0.006,
        name="center front button sits within control bezel footprint",
    )
    ctx.expect_within(
        button_2,
        body,
        axes="xz",
        inner_elem="cap",
        outer_elem="control_bezel",
        margin=0.006,
        name="right front button sits within control bezel footprint",
    )

    button_upper = button_0_joint.motion_limits.upper if button_0_joint.motion_limits is not None else None
    if button_upper is not None:
        rest_0 = ctx.part_world_position(button_0)
        rest_1 = ctx.part_world_position(button_1)
        rest_2 = ctx.part_world_position(button_2)
        with ctx.pose({button_0_joint: button_upper}):
            pressed_0 = ctx.part_world_position(button_0)
            unchanged_1 = ctx.part_world_position(button_1)
            unchanged_2 = ctx.part_world_position(button_2)
        ctx.check(
            "front buttons depress independently",
            rest_0 is not None
            and rest_1 is not None
            and rest_2 is not None
            and pressed_0 is not None
            and unchanged_1 is not None
            and unchanged_2 is not None
            and pressed_0[1] < rest_0[1] - 0.002
            and abs(unchanged_1[1] - rest_1[1]) < 1e-6
            and abs(unchanged_2[1] - rest_2[1]) < 1e-6,
            details=(
                f"rest0={rest_0}, pressed0={pressed_0}, "
                f"rest1={rest_1}, unchanged1={unchanged_1}, "
                f"rest2={rest_2}, unchanged2={unchanged_2}"
            ),
        )

    mode_upper = mode_button_0_joint.motion_limits.upper if mode_button_0_joint.motion_limits is not None else None
    if mode_upper is not None:
        mode_rest = ctx.part_world_position(mode_button_0)
        with ctx.pose({mode_button_0_joint: mode_upper}):
            mode_pressed = ctx.part_world_position(mode_button_0)
        ctx.check(
            "top mode button depresses downward",
            mode_rest is not None and mode_pressed is not None and mode_pressed[2] < mode_rest[2] - 0.0015,
            details=f"rest={mode_rest}, pressed={mode_pressed}",
        )

    return ctx.report()


object_model = build_object_model()

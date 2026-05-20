from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
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


HOUSING_SIZE = 0.52
HOUSING_DEPTH = 0.138
FRAME_WALL = 0.038
INNER_SPAN = HOUSING_SIZE - 2.0 * FRAME_WALL
FRAME_HALF = HOUSING_SIZE / 2.0
DEPTH_HALF = HOUSING_DEPTH / 2.0

GUARD_THICKNESS = 0.006
GUARD_BAR = 0.006
GUARD_OUTER_SPAN = INNER_SPAN + 0.010
GUARD_INNER_SPAN = INNER_SPAN - 0.050
FRONT_GUARD_Y = DEPTH_HALF - GUARD_THICKNESS / 2.0
REAR_GUARD_Y = -FRONT_GUARD_Y

MOTOR_HUB_RADIUS = 0.060
MOTOR_HUB_LENGTH = 0.040
MOTOR_HUB_CENTER_Y = -0.040
MOTOR_CROSS_Y = -0.048

BLADE_RADIUS = 0.188
BLADE_HUB_RADIUS = 0.046
BLADE_THICKNESS = 0.022

PANEL_WIDTH = 0.118
PANEL_HEIGHT = 0.436
PANEL_THICKNESS = 0.014
HINGE_PIN_RADIUS = 0.004
HINGE_SLEEVE_INNER_RADIUS = 0.0048
HINGE_SLEEVE_OUTER_RADIUS = 0.0072
HINGE_PIN_X = FRAME_HALF + HINGE_SLEEVE_OUTER_RADIUS + 0.001

KNOB_X = 0.156
KNOB_Z = -0.230
KNOB_COLLAR_RADIUS = 0.019
KNOB_COLLAR_LENGTH = 0.004
KNOB_COLLAR_CENTER_Y = DEPTH_HALF + 0.002
KNOB_JOINT_Y = KNOB_COLLAR_CENTER_Y + KNOB_COLLAR_LENGTH / 2.0

AXIS_Y_RPY = (-math.pi / 2.0, 0.0, 0.0)


def _rect_frame(width: float, depth: float, height: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height)
    inner = cq.Workplane("XY").box(width - 2.0 * wall, depth + 0.004, height - 2.0 * wall)
    return outer.cut(inner)


def _grille(y_center: float) -> cq.Workplane:
    grille = _rect_frame(
        width=GUARD_OUTER_SPAN,
        depth=GUARD_THICKNESS,
        height=GUARD_OUTER_SPAN,
        wall=0.020,
    ).translate((0.0, y_center, 0.0))

    vertical_x = (-0.156, -0.078, 0.0, 0.078, 0.156)
    for x_pos in vertical_x:
        grille = grille.union(
            cq.Workplane("XY")
            .box(GUARD_BAR, GUARD_THICKNESS, GUARD_OUTER_SPAN - 0.010)
            .translate((x_pos, y_center, 0.0))
        )

    horizontal_z = (-0.156, -0.078, 0.0, 0.078, 0.156)
    for z_pos in horizontal_z:
        grille = grille.union(
            cq.Workplane("XY")
            .box(GUARD_OUTER_SPAN - 0.010, GUARD_THICKNESS, GUARD_BAR)
            .translate((0.0, y_center, z_pos))
        )

    center_ring = (
        cq.Workplane("XZ")
        .circle(0.078)
        .circle(0.067)
        .extrude(GUARD_THICKNESS / 2.0, both=True)
        .translate((0.0, y_center, 0.0))
    )
    return grille.union(center_ring)


def _motor_cross() -> cq.Workplane:
    vertical = cq.Workplane("XY").box(0.020, 0.016, INNER_SPAN + 0.016).translate((0.0, MOTOR_CROSS_Y, 0.0))
    horizontal = cq.Workplane("XY").box(INNER_SPAN + 0.016, 0.016, 0.020).translate((0.0, MOTOR_CROSS_Y, 0.0))
    return vertical.union(horizontal)


def _side_panel_mesh(side: str) -> cq.Workplane:
    sign = 1.0 if side == "right" else -1.0
    panel_center_x = sign * (HINGE_SLEEVE_OUTER_RADIUS + PANEL_WIDTH / 2.0 - 0.001)

    outer = cq.Workplane("XY").box(PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT).translate((panel_center_x, 0.0, 0.0))
    recess_offset = PANEL_THICKNESS * 0.24
    front_recess = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH - 0.022, PANEL_THICKNESS * 0.52, PANEL_HEIGHT - 0.022)
        .translate((panel_center_x, recess_offset, 0.0))
    )
    rear_recess = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH - 0.022, PANEL_THICKNESS * 0.52, PANEL_HEIGHT - 0.022)
        .translate((panel_center_x, -recess_offset, 0.0))
    )
    panel = outer.cut(front_recess).cut(rear_recess)

    hinge_sleeve = (
        cq.Workplane("XY")
        .circle(HINGE_SLEEVE_OUTER_RADIUS)
        .circle(HINGE_SLEEVE_INNER_RADIUS)
        .extrude(PANEL_HEIGHT / 2.0, both=True)
    )
    panel = panel.union(hinge_sleeve)

    tab_center_x = sign * 0.012
    for z_pos in (-0.148, 0.0, 0.148):
        panel = panel.union(
            cq.Workplane("XY")
            .box(0.024, PANEL_THICKNESS, 0.060)
            .translate((tab_center_x, 0.0, z_pos))
        )

    return panel


def _hinge_pin_mesh(side: str) -> cq.Workplane:
    sign = 1.0 if side == "right" else -1.0
    pin = (
        cq.Workplane("XY")
        .circle(HINGE_PIN_RADIUS)
        .extrude(PANEL_HEIGHT / 2.0, both=True)
        .translate((sign * HINGE_PIN_X, 0.0, 0.0))
    )

    bracket_width = HINGE_PIN_X - FRAME_HALF + HINGE_PIN_RADIUS + 0.002
    bracket_center_x = sign * (FRAME_HALF + bracket_width / 2.0 - 0.001)
    hinge = pin
    for z_pos in (-0.148, 0.0, 0.148):
        hinge = hinge.union(
            cq.Workplane("XY")
            .box(bracket_width, 0.020, 0.060)
            .translate((bracket_center_x, 0.0, z_pos))
        )

    for z_pos in (-(PANEL_HEIGHT / 2.0 + 0.003), PANEL_HEIGHT / 2.0 + 0.003):
        hinge = hinge.union(
            cq.Workplane("XY")
            .circle(0.010)
            .circle(0.006)
            .extrude(0.003, both=True)
            .translate((sign * HINGE_PIN_X, 0.0, z_pos))
        )

    return hinge


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_box_fan")

    housing_finish = model.material("housing_finish", rgba=(0.92, 0.93, 0.91, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.87, 0.88, 0.86, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.56, 0.74, 0.90, 0.95))
    knob_finish = model.material("knob_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.62, 0.64, 0.67, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(
            _rect_frame(HOUSING_SIZE, HOUSING_DEPTH, HOUSING_SIZE, FRAME_WALL),
            "fan_housing_shell",
        ),
        material=housing_finish,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_grille(FRONT_GUARD_Y), "fan_front_guard"),
        material=housing_finish,
        name="front_guard",
    )
    housing.visual(
        mesh_from_cadquery(_grille(REAR_GUARD_Y), "fan_rear_guard"),
        material=housing_finish,
        name="rear_guard",
    )
    housing.visual(
        mesh_from_cadquery(_motor_cross(), "fan_motor_cross"),
        material=metal_finish,
        name="motor_cross",
    )
    housing.visual(
        Cylinder(radius=MOTOR_HUB_RADIUS, length=MOTOR_HUB_LENGTH),
        origin=Origin(xyz=(0.0, MOTOR_HUB_CENTER_Y, 0.0), rpy=AXIS_Y_RPY),
        material=metal_finish,
        name="motor_hub",
    )
    housing.visual(
        Cylinder(radius=KNOB_COLLAR_RADIUS, length=KNOB_COLLAR_LENGTH),
        origin=Origin(xyz=(KNOB_X, KNOB_COLLAR_CENTER_Y, KNOB_Z), rpy=AXIS_Y_RPY),
        material=housing_finish,
        name="knob_collar",
    )
    housing.visual(
        mesh_from_cadquery(_hinge_pin_mesh("left"), "fan_left_hinge_pin"),
        material=metal_finish,
        name="left_hinge_pin",
    )
    housing.visual(
        mesh_from_cadquery(_hinge_pin_mesh("right"), "fan_right_hinge_pin"),
        material=metal_finish,
        name="right_hinge_pin",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                BLADE_RADIUS,
                BLADE_HUB_RADIUS,
                5,
                thickness=BLADE_THICKNESS,
                blade_pitch_deg=22.0,
                blade_sweep_deg=16.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=10.0, camber=0.10),
            ),
            "box_fan_blade_rotor",
        ),
        origin=Origin(rpy=AXIS_Y_RPY),
        material=blade_finish,
        name="blade_rotor",
    )
    blade.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=AXIS_Y_RPY),
        material=metal_finish,
        name="blade_axle",
    )

    right_panel = model.part("right_panel")
    right_panel.visual(
        mesh_from_cadquery(_side_panel_mesh("right"), "fan_right_panel"),
        material=panel_finish,
        name="panel_shell",
    )

    left_panel = model.part("left_panel")
    left_panel.visual(
        mesh_from_cadquery(_side_panel_mesh("left"), "fan_left_panel"),
        material=panel_finish,
        name="panel_shell",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=AXIS_Y_RPY),
        material=metal_finish,
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.020,
                body_style="skirted",
                top_diameter=0.024,
                skirt=KnobSkirt(0.036, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "box_fan_control_knob",
        ),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=AXIS_Y_RPY),
        material=knob_finish,
        name="knob_cap",
    )

    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=18.0),
    )
    model.articulation(
        "housing_to_right_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_panel,
        origin=Origin(xyz=(HINGE_PIN_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=-0.15, upper=1.10),
    )
    model.articulation(
        "housing_to_left_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_panel,
        origin=Origin(xyz=(-HINGE_PIN_X, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=-0.15, upper=1.10),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(KNOB_X, KNOB_JOINT_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")
    knob = object_model.get_part("knob")

    blade_joint = object_model.get_articulation("housing_to_blade")
    left_joint = object_model.get_articulation("housing_to_left_panel")
    right_joint = object_model.get_articulation("housing_to_right_panel")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.allow_overlap(
        housing,
        right_panel,
        elem_a="right_hinge_pin",
        elem_b="panel_shell",
        reason="The right expansion panel's hinge barrel is represented as a simplified close-fitting sleeve around the housing-side hinge pin.",
    )
    ctx.allow_overlap(
        housing,
        left_panel,
        elem_a="left_hinge_pin",
        elem_b="panel_shell",
        reason="The left expansion panel's hinge barrel is represented as a simplified close-fitting sleeve around the housing-side hinge pin.",
    )

    ctx.check(
        "blade_joint_continuous",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type!r}",
    )
    ctx.check(
        "knob_joint_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )

    ctx.expect_overlap(
        blade,
        housing,
        axes="xz",
        elem_a="blade_rotor",
        elem_b="front_guard",
        min_overlap=0.34,
        name="blade fills the square opening",
    )
    ctx.expect_gap(
        housing,
        blade,
        axis="y",
        positive_elem="front_guard",
        negative_elem="blade_rotor",
        min_gap=0.040,
        max_gap=0.070,
        name="front guard sits in front of the blade",
    )
    ctx.expect_gap(
        blade,
        housing,
        axis="y",
        positive_elem="blade_rotor",
        negative_elem="rear_guard",
        min_gap=0.040,
        max_gap=0.070,
        name="rear guard sits behind the blade",
    )
    ctx.expect_gap(
        blade,
        housing,
        axis="y",
        positive_elem="blade_axle",
        negative_elem="motor_hub",
        min_gap=0.0,
        max_gap=0.002,
        name="blade axle seats on the motor hub",
    )

    ctx.expect_gap(
        right_panel,
        housing,
        axis="x",
        positive_elem="panel_shell",
        negative_elem="housing_shell",
        min_gap=0.0,
        max_gap=0.020,
        name="right panel parks outside the housing wall",
    )
    ctx.expect_gap(
        housing,
        left_panel,
        axis="x",
        positive_elem="housing_shell",
        negative_elem="panel_shell",
        min_gap=0.0,
        max_gap=0.020,
        name="left panel parks outside the housing wall",
    )
    ctx.expect_overlap(
        right_panel,
        housing,
        axes="z",
        elem_a="panel_shell",
        elem_b="right_hinge_pin",
        min_overlap=0.40,
        name="right hinge sleeve spans the full hinge pin height",
    )
    ctx.expect_overlap(
        left_panel,
        housing,
        axes="z",
        elem_a="panel_shell",
        elem_b="left_hinge_pin",
        min_overlap=0.40,
        name="left hinge sleeve spans the full hinge pin height",
    )

    ctx.expect_gap(
        knob,
        housing,
        axis="y",
        positive_elem="knob_shaft",
        negative_elem="knob_collar",
        min_gap=0.0,
        max_gap=0.002,
        name="knob shaft seats on the front collar",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="housing_shell",
        min_gap=0.010,
        max_gap=0.040,
        name="control knob projects proud of the housing front",
    )
    ctx.expect_overlap(
        knob,
        housing,
        axes="xz",
        elem_a="knob_cap",
        elem_b="knob_collar",
        min_overlap=0.020,
        name="control knob stays centered on its collar",
    )

    right_rest = ctx.part_element_world_aabb(right_panel, elem="panel_shell")
    left_rest = ctx.part_element_world_aabb(left_panel, elem="panel_shell")
    with ctx.pose({right_joint: 0.85, left_joint: 0.85}):
        right_swung = ctx.part_element_world_aabb(right_panel, elem="panel_shell")
        left_swung = ctx.part_element_world_aabb(left_panel, elem="panel_shell")

    right_forward = (
        right_rest is not None
        and right_swung is not None
        and float(right_swung[1][1]) > float(right_rest[1][1]) + 0.070
    )
    left_forward = (
        left_rest is not None
        and left_swung is not None
        and float(left_swung[1][1]) > float(left_rest[1][1]) + 0.070
    )

    ctx.check(
        "right panel swings outward toward the room",
        right_forward,
        details=f"rest={right_rest}, swung={right_swung}",
    )
    ctx.check(
        "left panel swings outward toward the room",
        left_forward,
        details=f"rest={left_rest}, swung={left_swung}",
    )

    return ctx.report()


object_model = build_object_model()

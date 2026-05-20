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
HOUSING_DEPTH = 0.16
FAN_RADIUS = 0.182
PIVOT_X = 0.175
PIVOT_Z = 0.325
HOUSING_CENTER_X = -0.285
HANDLE_AXIS_X = HOUSING_CENTER_X
HANDLE_AXIS_Y = -(HOUSING_DEPTH * 0.5 + 0.053)
HANDLE_AXIS_Z = 0.215


def _centered_extrude_on_xz(workplane: cq.Workplane, depth: float) -> cq.Workplane:
    return workplane.extrude(depth * 0.5, both=True)


def _make_guard_shape() -> cq.Workplane:
    outer = 0.476
    frame = 0.018
    thickness = 0.006
    grid = _centered_extrude_on_xz(
        cq.Workplane("XZ").rect(outer, outer).rect(outer - 2.0 * frame, outer - 2.0 * frame),
        thickness,
    )
    span = outer - 2.0 * frame + 0.002
    for offset in (-0.150, -0.075, 0.0, 0.075, 0.150):
        grid = grid.union(
            _centered_extrude_on_xz(
                cq.Workplane("XZ").center(offset, 0.0).rect(0.008, span),
                thickness,
            )
        )
        grid = grid.union(
            _centered_extrude_on_xz(
                cq.Workplane("XZ").center(0.0, offset).rect(span, 0.008),
                thickness,
            )
        )
    return grid.combine()


def _make_shroud_ring() -> cq.Workplane:
    outer = _centered_extrude_on_xz(cq.Workplane("XZ").circle(FAN_RADIUS + 0.028), 0.100)
    inner = _centered_extrude_on_xz(cq.Workplane("XZ").circle(FAN_RADIUS + 0.010), 0.110)
    return outer.cut(inner)


def _make_carrying_grip_shape() -> cq.Workplane:
    side_profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.012, -0.010),
                (0.000, -0.010),
                (-0.010, 0.016),
                (-0.033, 0.045),
                (-0.060, 0.074),
                (-0.074, 0.074),
                (-0.044, 0.034),
                (-0.016, 0.000),
            ]
        )
        .close()
        .extrude(0.010, both=True)
    )
    left_plate = side_profile.translate((-0.155, 0.0, 0.0))
    right_plate = side_profile.translate((0.155, 0.0, 0.0))
    crossbar = (
        cq.Workplane("YZ")
        .circle(0.0105)
        .extrude(0.155, both=True)
        .translate((0.0, -0.052, 0.062))
    )
    left_sleeve = (
        cq.Workplane("YZ").circle(0.008).extrude(0.017, both=True).translate((-0.155, 0.0, 0.0))
    )
    right_sleeve = (
        cq.Workplane("YZ").circle(0.008).extrude(0.017, both=True).translate((0.155, 0.0, 0.0))
    )
    return left_plate.union(right_plate).union(crossbar).union(left_sleeve).union(right_sleeve).combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_box_fan")

    stand_finish = model.material("stand_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    guard_finish = model.material("guard_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.72, 0.75, 0.78, 1.0))
    motor_finish = model.material("motor_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    control_finish = model.material("control_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.50, 0.32, 0.028)),
        origin=Origin(xyz=(-0.02, 0.0, 0.014)),
        material=stand_finish,
        name="base_plate",
    )
    stand.visual(
        Box((0.30, 0.05, 0.018)),
        origin=Origin(xyz=(-0.02, 0.125, 0.037)),
        material=stand_finish,
        name="front_runner",
    )
    stand.visual(
        Box((0.32, 0.05, 0.018)),
        origin=Origin(xyz=(-0.01, -0.125, 0.037)),
        material=stand_finish,
        name="rear_runner",
    )
    stand.visual(
        Box((0.040, 0.100, 0.350)),
        origin=Origin(xyz=(0.205, 0.0, 0.190)),
        material=stand_finish,
        name="upright",
    )
    stand.visual(
        Box((0.060, 0.090, 0.120)),
        origin=Origin(xyz=(0.187, 0.0, 0.305)),
        material=stand_finish,
        name="upper_arm",
    )
    stand.visual(
        Box((0.080, 0.080, 0.090)),
        origin=Origin(xyz=(0.225, 0.0, 0.085), rpy=(0.0, math.radians(-28.0), 0.0)),
        material=stand_finish,
        name="gusset",
    )
    stand.visual(
        Box((0.020, 0.070, 0.070)),
        origin=Origin(xyz=(0.165, 0.0, PIVOT_Z)),
        material=stand_finish,
        name="pivot_bridge",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.240, 0.0, PIVOT_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stand_finish,
        name="bracket_collar",
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.54, HOUSING_DEPTH, 0.055)),
        origin=Origin(xyz=(HOUSING_CENTER_X, 0.0, 0.2425)),
        material=housing_finish,
        name="top_band",
    )
    housing.visual(
        Box((0.54, HOUSING_DEPTH, 0.055)),
        origin=Origin(xyz=(HOUSING_CENTER_X, 0.0, -0.2425)),
        material=housing_finish,
        name="bottom_band",
    )
    housing.visual(
        Box((0.055, HOUSING_DEPTH, 0.430)),
        origin=Origin(xyz=(-0.5275, 0.0, 0.0)),
        material=housing_finish,
        name="left_band",
    )
    housing.visual(
        Box((0.040, HOUSING_DEPTH, 0.430)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=housing_finish,
        name="right_band",
    )
    shroud = model.part("shroud")
    shroud.visual(
        mesh_from_cadquery(_make_shroud_ring(), "shroud"),
        origin=Origin(xyz=(HOUSING_CENTER_X, 0.0, 0.0)),
        material=housing_finish,
        name="shroud_ring",
    )
    shroud.visual(
        Box((0.080, 0.100, 0.026)),
        origin=Origin(xyz=(HOUSING_CENTER_X, 0.0, 0.202)),
        material=housing_finish,
        name="top_bridge",
    )
    shroud.visual(
        Box((0.080, 0.100, 0.026)),
        origin=Origin(xyz=(HOUSING_CENTER_X, 0.0, -0.202)),
        material=housing_finish,
        name="bottom_bridge",
    )
    shroud.visual(
        Box((0.040, 0.100, 0.085)),
        origin=Origin(xyz=(-0.480, 0.0, 0.0)),
        material=housing_finish,
        name="left_bridge",
    )
    shroud.visual(
        Box((0.040, 0.100, 0.085)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=housing_finish,
        name="right_bridge",
    )

    rear_pivots = model.part("rear_pivots")
    rear_pivots.visual(
        Box((0.034, 0.050, 0.042)),
        origin=Origin(xyz=(HANDLE_AXIS_X - 0.155, -0.108, HANDLE_AXIS_Z)),
        material=housing_finish,
        name="pivot_0",
    )
    rear_pivots.visual(
        Box((0.034, 0.050, 0.042)),
        origin=Origin(xyz=(HANDLE_AXIS_X + 0.155, -0.108, HANDLE_AXIS_Z)),
        material=housing_finish,
        name="pivot_1",
    )
    rear_pivots.visual(
        Box((0.340, 0.050, 0.018)),
        origin=Origin(xyz=(HOUSING_CENTER_X, -0.108, 0.236)),
        material=housing_finish,
        name="mount_bar",
    )

    front_guard = model.part("front_guard")
    front_guard.visual(
        mesh_from_cadquery(_make_guard_shape(), "front_guard"),
        origin=Origin(xyz=(HOUSING_CENTER_X, HOUSING_DEPTH * 0.5 + 0.003, 0.0)),
        material=guard_finish,
        name="front_guard",
    )

    rear_guard = model.part("rear_guard")
    rear_guard.visual(
        mesh_from_cadquery(_make_guard_shape(), "rear_guard"),
        origin=Origin(xyz=(HOUSING_CENTER_X, -0.083, 0.0)),
        material=guard_finish,
        name="rear_guard",
    )
    rear_guard.visual(
        Box((0.120, 0.042, 0.120)),
        origin=Origin(xyz=(HOUSING_CENTER_X, -0.104, 0.0)),
        material=motor_finish,
        name="motor_pod",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                FAN_RADIUS,
                0.045,
                5,
                thickness=0.018,
                blade_pitch_deg=29.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=13.0, camber=0.10),
                hub=FanRotorHub(
                    style="domed",
                    rear_collar_height=0.020,
                    rear_collar_radius=0.034,
                    bore_diameter=0.010,
                ),
            ),
            "blade",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=blade_finish,
        name="blade_rotor",
    )
    blade.visual(
        Box((0.024, 0.083, 0.024)),
        origin=Origin(xyz=(0.0, -0.0415, 0.0)),
        material=motor_finish,
        name="blade_axle",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.026,
                body_style="domed",
                top_diameter=0.034,
                base_diameter=0.040,
                edge_radius=0.0015,
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_finish,
        name="selector_knob",
    )

    carrying_grip = model.part("carrying_grip")
    carrying_grip.visual(
        mesh_from_cadquery(_make_carrying_grip_shape(), "carrying_grip"),
        material=grip_finish,
        name="grip_loop",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "housing_to_shroud",
        ArticulationType.FIXED,
        parent=housing,
        child=shroud,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_rear_pivots",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_pivots,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_front_guard",
        ArticulationType.FIXED,
        parent=housing,
        child=front_guard,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_rear_guard",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_guard,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(HOUSING_CENTER_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=26.0),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector_knob,
        origin=Origin(xyz=(-0.015, -0.045, -0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "housing_to_carrying_grip",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=carrying_grip,
        origin=Origin(xyz=(HANDLE_AXIS_X, HANDLE_AXIS_Y, HANDLE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    front_guard = object_model.get_part("front_guard")
    blade = object_model.get_part("blade")
    selector_knob = object_model.get_part("selector_knob")
    carrying_grip = object_model.get_part("carrying_grip")
    rear_pivots = object_model.get_part("rear_pivots")

    pan_joint = object_model.get_articulation("stand_to_housing")
    blade_joint = object_model.get_articulation("housing_to_blade")
    knob_joint = object_model.get_articulation("housing_to_selector_knob")
    grip_joint = object_model.get_articulation("housing_to_carrying_grip")

    ctx.check(
        "pan joint is revolute",
        pan_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pan_joint.articulation_type!r}",
    )
    ctx.check(
        "blade joint is continuous",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type!r}",
    )
    ctx.check(
        "selector knob joint is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )
    ctx.check(
        "grip joint is revolute",
        grip_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={grip_joint.articulation_type!r}",
    )
    ctx.allow_overlap(
        carrying_grip,
        rear_pivots,
        elem_a="grip_loop",
        elem_b="pivot_0",
        reason="The carrying handle is represented with a captured pivot sleeve nested inside the first rear support bracket.",
    )
    ctx.allow_overlap(
        carrying_grip,
        rear_pivots,
        elem_a="grip_loop",
        elem_b="pivot_1",
        reason="The carrying handle is represented with a captured pivot sleeve nested inside the second rear support bracket.",
    )

    ctx.expect_overlap(
        blade,
        front_guard,
        axes="xz",
        min_overlap=0.30,
        name="blade remains centered behind the front guard",
    )
    ctx.expect_gap(
        selector_knob,
        housing,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        elem_a="selector_knob",
        elem_b="right_band",
        name="selector knob seats against the housing side",
    )

    stand_aabb = ctx.part_world_aabb(stand)
    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "housing clears the base runners",
        stand_aabb is not None and housing_aabb is not None and housing_aabb[0][2] > stand_aabb[0][2] + 0.05,
        details=f"stand={stand_aabb}, housing={housing_aabb}",
    )

    rest_housing_aabb = housing_aabb
    with ctx.pose({"stand_to_housing": 0.55}):
        panned_housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "housing visibly pans at the side bracket",
        rest_housing_aabb is not None
        and panned_housing_aabb is not None
        and abs(((panned_housing_aabb[0][1] + panned_housing_aabb[1][1]) * 0.5) - ((rest_housing_aabb[0][1] + rest_housing_aabb[1][1]) * 0.5))
        > 0.08,
        details=f"rest={rest_housing_aabb}, panned={panned_housing_aabb}",
    )

    rest_grip_aabb = ctx.part_world_aabb(carrying_grip)
    ctx.check(
        "grip rests just behind the rear pivots",
        rest_grip_aabb is not None
        and housing_aabb is not None
        and rest_grip_aabb[1][1] <= housing_aabb[0][1] - 0.005,
        details=f"rest={rest_grip_aabb}, housing={housing_aabb}",
    )

    with ctx.pose({"housing_to_carrying_grip": 1.15}):
        raised_grip_aabb = ctx.part_world_aabb(carrying_grip)
    ctx.check(
        "carrying grip raises above the housing",
        rest_grip_aabb is not None
        and raised_grip_aabb is not None
        and housing_aabb is not None
        and raised_grip_aabb[1][2] > housing_aabb[1][2] + 0.02
        and raised_grip_aabb[1][2] > rest_grip_aabb[1][2] + 0.01,
        details=f"rest={rest_grip_aabb}, raised={raised_grip_aabb}, housing={housing_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

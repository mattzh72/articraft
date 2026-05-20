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

FAN_WIDTH = 0.66
FAN_HEIGHT = 0.66
FAN_DEPTH = 0.22
FRAME_WALL = 0.022
RIM_DEPTH = 0.020
FRONT_OPENING = 0.584
PIVOT_HEIGHT = 0.42
PIVOT_BOSS_RADIUS = 0.018
PIVOT_BOSS_LENGTH = 0.028

STAND_SIDE_CENTER = 0.405
STAND_SIDE_THICKNESS = 0.034
STAND_BASE_DEPTH = 0.34
STAND_RAIL_WIDTH = 0.046
STAND_RAIL_HEIGHT = 0.032
STAND_UPRIGHT_HEIGHT = 0.43

MOTOR_POD_RADIUS = 0.094
MOTOR_POD_LENGTH = 0.120
MOTOR_POD_CENTER_X = -0.102
FLAP_HINGE_X = -0.168
FLAP_HINGE_Z = 0.160


def _yz_ring(radius: float, wire: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).circle(radius - wire).extrude(wire, both=True)


def _guard_face(x_pos: float) -> cq.Workplane:
    wire = 0.0048
    outer_radius = 0.278
    opening_half = FRONT_OPENING / 2.0
    tab_gap = opening_half - outer_radius

    guard = _yz_ring(outer_radius, wire)
    for radius in (0.224, 0.168, 0.112, 0.062):
        guard = guard.union(_yz_ring(radius, wire))

    crossbar = cq.Workplane("YZ").rect(0.016, 0.520).extrude(wire, both=True)
    guard = guard.union(crossbar.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 45.0))
    guard = guard.union(crossbar.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -45.0))
    guard = guard.union(cq.Workplane("YZ").rect(wire, 2.0 * outer_radius).extrude(wire, both=True))
    guard = guard.union(cq.Workplane("YZ").rect(2.0 * outer_radius, wire).extrude(wire, both=True))

    tab_span = tab_gap + wire
    guard = guard.union(
        cq.Workplane("YZ")
        .center(0.0, (opening_half + outer_radius) / 2.0)
        .rect(0.050, tab_span)
        .extrude(wire, both=True)
    )
    guard = guard.union(
        cq.Workplane("YZ")
        .center(0.0, -(opening_half + outer_radius) / 2.0)
        .rect(0.050, tab_span)
        .extrude(wire, both=True)
    )
    guard = guard.union(
        cq.Workplane("YZ")
        .center((opening_half + outer_radius) / 2.0, 0.0)
        .rect(tab_span, 0.050)
        .extrude(wire, both=True)
    )
    guard = guard.union(
        cq.Workplane("YZ")
        .center(-(opening_half + outer_radius) / 2.0, 0.0)
        .rect(tab_span, 0.050)
        .extrude(wire, both=True)
    )
    return guard.translate((x_pos, 0.0, 0.0))


def _housing_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(FAN_DEPTH, FAN_WIDTH, FAN_HEIGHT)
    shell = shell.cut(
        cq.Workplane("XY").box(
            FAN_DEPTH - 2.0 * RIM_DEPTH,
            FAN_WIDTH - 2.0 * FRAME_WALL,
            FAN_HEIGHT - 2.0 * FRAME_WALL,
        )
    )
    shell = shell.cut(
        cq.Workplane("XY").box(RIM_DEPTH + 0.004, FRONT_OPENING, FRONT_OPENING).translate(
            (FAN_DEPTH / 2.0 - (RIM_DEPTH + 0.004) / 2.0 + 0.002, 0.0, 0.0)
        )
    )
    shell = shell.cut(
        cq.Workplane("XY").box(RIM_DEPTH + 0.004, FRONT_OPENING, FRONT_OPENING).translate(
            (-FAN_DEPTH / 2.0 + (RIM_DEPTH + 0.004) / 2.0 - 0.002, 0.0, 0.0)
        )
    )

    motor_pod = (
        cq.Workplane("YZ")
        .circle(MOTOR_POD_RADIUS)
        .extrude(MOTOR_POD_LENGTH, both=True)
        .translate((MOTOR_POD_CENTER_X, 0.0, 0.0))
    )
    pod_top = cq.Workplane("XY").box(0.178, 0.176, 0.060).translate((-0.088, 0.0, 0.090))
    pod_neck = cq.Workplane("XY").box(0.120, 0.114, 0.068).translate((-0.090, 0.0, 0.040))
    shell = shell.union(motor_pod).union(pod_top).union(pod_neck)
    shell = shell.cut(cq.Workplane("XY").box(0.122, 0.148, 0.060).translate((-0.108, 0.0, 0.164)))

    spider = cq.Workplane("YZ").rect(0.015, 0.500).extrude(0.012, both=True).translate((-0.035, 0.0, 0.0))
    shell = shell.union(spider.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 45.0))
    shell = shell.union(spider.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -45.0))

    shell = shell.union(_guard_face(FAN_DEPTH / 2.0 - 0.010))
    shell = shell.union(_guard_face(-FAN_DEPTH / 2.0 + 0.010))
    return shell


def _stand_shape() -> cq.Workplane:
    stand = cq.Workplane("XY").box(STAND_BASE_DEPTH, STAND_RAIL_WIDTH, STAND_RAIL_HEIGHT).translate(
        (0.0, STAND_SIDE_CENTER, STAND_RAIL_HEIGHT / 2.0)
    )
    stand = stand.union(
        cq.Workplane("XY").box(STAND_BASE_DEPTH, STAND_RAIL_WIDTH, STAND_RAIL_HEIGHT).translate(
            (0.0, -STAND_SIDE_CENTER, STAND_RAIL_HEIGHT / 2.0)
        )
    )
    stand = stand.union(
        cq.Workplane("XY").box(0.050, 2.0 * STAND_SIDE_CENTER + STAND_RAIL_WIDTH, STAND_RAIL_HEIGHT).translate(
            (0.120, 0.0, STAND_RAIL_HEIGHT / 2.0)
        )
    )
    stand = stand.union(
        cq.Workplane("XY").box(0.050, 2.0 * STAND_SIDE_CENTER + STAND_RAIL_WIDTH, STAND_RAIL_HEIGHT).translate(
            (-0.120, 0.0, STAND_RAIL_HEIGHT / 2.0)
        )
    )

    upright = cq.Workplane("XY").box(0.050, STAND_SIDE_THICKNESS, STAND_UPRIGHT_HEIGHT).translate(
        (0.0, STAND_SIDE_CENTER, STAND_UPRIGHT_HEIGHT / 2.0)
    )
    brace = (
        cq.Workplane("XY")
        .box(0.028, STAND_SIDE_THICKNESS, 0.320)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
        .translate((-0.055, STAND_SIDE_CENTER, 0.180))
    )
    pivot_plate = cq.Workplane("XZ").circle(0.054).extrude(STAND_SIDE_THICKNESS, both=True).translate(
        (0.0, STAND_SIDE_CENTER, PIVOT_HEIGHT)
    )

    stand = stand.union(upright).union(brace).union(pivot_plate)
    stand = stand.union(upright.mirror(mirrorPlane="XZ"))
    stand = stand.union(brace.mirror(mirrorPlane="XZ"))
    stand = stand.union(pivot_plate.mirror(mirrorPlane="XZ"))

    return stand


def _service_flap_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.104, 0.132, 0.006).translate((0.052, 0.0, 0.011))
    hinge_leaf = cq.Workplane("XY").box(0.010, 0.116, 0.016).translate((0.005, 0.0, 0.008))
    return panel.union(hinge_leaf)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan_with_stand")

    stand_finish = model.material("stand_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.83, 0.85, 0.80, 1.0))
    rotor_finish = model.material("rotor_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.10, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "box_fan_stand"),
        material=stand_finish,
        name="stand_frame",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "box_fan_housing"),
        material=housing_finish,
        name="housing_shell",
    )
    housing.visual(
        Box((0.008, 0.096, 0.034)),
        origin=Origin(xyz=(FAN_DEPTH / 2.0 - 0.004, 0.0, 0.0)),
        material=stand_finish,
        name="front_badge",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.058),
        origin=Origin(xyz=(0.0, 0.359, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="pivot_stub_0",
    )
    housing.visual(
        Cylinder(radius=0.013, length=0.058),
        origin=Origin(xyz=(0.0, -0.359, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="pivot_stub_1",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.225,
                0.042,
                5,
                thickness=0.030,
                blade_pitch_deg=25.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=11.0, camber=0.10),
                hub=FanRotorHub(style="domed", rear_collar_height=0.014, rear_collar_radius=0.034, bore_diameter=0.012),
            ),
            "box_fan_rotor",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_finish,
        name="impeller",
    )
    rotor.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="shaft",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.086,
                0.042,
                body_style="mushroom",
                top_diameter=0.060,
                base_diameter=0.086,
                crown_radius=0.006,
                edge_radius=0.003,
                center=False,
            ),
            "tilt_adjust_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_body",
    )
    tilt_knob.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stand_finish,
        name="knob_spindle",
    )

    service_flap = model.part("service_flap")
    service_flap.visual(
        mesh_from_cadquery(_service_flap_shape(), "box_fan_service_flap"),
        material=housing_finish,
        name="flap_panel",
    )

    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.0,
            lower=math.radians(-18.0),
            upper=math.radians(55.0),
        ),
    )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )

    model.articulation(
        "stand_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=tilt_knob,
        origin=Origin(xyz=(0.0, STAND_SIDE_CENTER + STAND_SIDE_THICKNESS / 2.0 + 0.020, PIVOT_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    model.articulation(
        "housing_to_service_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=service_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    tilt_knob = object_model.get_part("tilt_knob")
    service_flap = object_model.get_part("service_flap")

    tilt_joint = object_model.get_articulation("stand_to_housing")
    rotor_joint = object_model.get_articulation("housing_to_rotor")
    knob_joint = object_model.get_articulation("stand_to_tilt_knob")
    flap_joint = object_model.get_articulation("housing_to_service_flap")

    ctx.check(
        "articulation_types_match_prompt",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and flap_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"tilt={tilt_joint.articulation_type}, rotor={rotor_joint.articulation_type}, "
            f"knob={knob_joint.articulation_type}, flap={flap_joint.articulation_type}"
        ),
    )

    ctx.allow_overlap(
        housing,
        rotor,
        elem_a="housing_shell",
        elem_b="shaft",
        reason="The rotor shaft is intentionally represented as entering the motor housing proxy at the rear bearing.",
    )
    ctx.allow_overlap(
        housing,
        stand,
        elem_a="pivot_stub_0",
        elem_b="stand_frame",
        reason="The right-side pivot stub is intentionally represented as seated inside the stand pivot plate.",
    )
    ctx.allow_overlap(
        housing,
        stand,
        elem_a="pivot_stub_1",
        elem_b="stand_frame",
        reason="The left-side pivot stub is intentionally represented as seated inside the stand pivot plate.",
    )
    ctx.allow_overlap(
        stand,
        tilt_knob,
        elem_a="stand_frame",
        elem_b="knob_spindle",
        reason="The tilt-adjustment knob spindle is intentionally represented as entering the stand bracket.",
    )
    ctx.allow_overlap(
        housing,
        service_flap,
        elem_a="housing_shell",
        elem_b="flap_panel",
        reason="The service flap is represented as a flush-fitting cover seated into the motor-housing rebate rather than as a thin separate shell.",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="yz",
        margin=0.035,
        name="rotor stays inside the housing opening",
    )
    ctx.expect_origin_distance(
        rotor,
        housing,
        axes="yz",
        max_dist=0.015,
        name="rotor remains centered on the housing axis",
    )
    ctx.expect_origin_gap(
        housing,
        stand,
        axis="z",
        min_gap=0.38,
        max_gap=0.46,
        name="housing pivot sits at a realistic low-stand height",
    )
    ctx.expect_origin_gap(
        tilt_knob,
        housing,
        axis="y",
        min_gap=0.36,
        name="tilt knob sits outboard on the stand side",
    )

    rest_badge_aabb = ctx.part_element_world_aabb(housing, elem="front_badge")
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        tilted_badge_aabb = ctx.part_element_world_aabb(housing, elem="front_badge")
    ctx.check(
        "housing tilts upward at positive limit",
        rest_badge_aabb is not None
        and tilted_badge_aabb is not None
        and tilted_badge_aabb[1][2] > rest_badge_aabb[1][2] + 0.040,
        details=f"rest={rest_badge_aabb}, tilted={tilted_badge_aabb}",
    )

    rest_flap_aabb = ctx.part_world_aabb(service_flap)
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        open_flap_aabb = ctx.part_world_aabb(service_flap)
    ctx.check(
        "service flap opens upward",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > rest_flap_aabb[1][2] + 0.050,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

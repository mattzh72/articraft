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
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_THICKNESS = 0.044
JAW_THICKNESS = 0.024
JAW_CENTER_Z = 0.169
JAW_OUTER_RADIUS = 0.032
JAW_INNER_RADIUS = 0.021
HINGE_ANGLE_DEG = 150.0
UPPER_JAW_END_DEG = 16.0
LOWER_JAW_START_DEG = 24.0


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            center_x + radius * math.cos(math.radians(start_deg + (end_deg - start_deg) * t)),
            center_z + radius * math.sin(math.radians(start_deg + (end_deg - start_deg) * t)),
        )
        for t in [index / segments for index in range(segments + 1)]
    ]


def _extrude_xz_profile(points: list[tuple[float, float]], thickness: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness).translate((0.0, thickness / 2.0, 0.0))


def _ring_segment(
    center_x: float,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
    *,
    segments: int = 48,
) -> cq.Workplane:
    outer = _arc_points(center_x, center_z, outer_radius, start_deg, end_deg, segments=segments)
    inner = list(
        reversed(
            _arc_points(center_x, center_z, inner_radius, start_deg, end_deg, segments=segments)
        )
    )
    return _extrude_xz_profile(outer + inner, thickness)


def _hinge_point() -> tuple[float, float]:
    radius = JAW_OUTER_RADIUS + 0.004
    angle = math.radians(HINGE_ANGLE_DEG)
    return (radius * math.cos(angle), JAW_CENTER_Z + radius * math.sin(angle))


def _jaw_ring() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(0.0, JAW_CENTER_Z)
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(JAW_THICKNESS)
        .translate((0.0, JAW_THICKNESS / 2.0, 0.0))
    )


def _jaw_sector(start_deg: float, end_deg: float, *, radius: float = 0.050) -> cq.Workplane:
    return _extrude_xz_profile(
        [(0.0, JAW_CENTER_Z)] + _arc_points(0.0, JAW_CENTER_Z, radius, start_deg, end_deg, segments=40),
        JAW_THICKNESS + 0.020,
    )


def _build_fixed_jaw() -> cq.Workplane:
    return _jaw_ring().cut(_jaw_sector(16.0, 148.0))


def _build_body_housing() -> cq.Workplane:
    profile = [
        (-0.030, 0.000),
        (0.030, 0.000),
        (0.034, 0.052),
        (0.041, 0.098),
        (0.044, 0.113),
        (0.040, 0.126),
        (0.031, 0.140),
        (0.026, 0.150),
        (-0.026, 0.150),
        (-0.031, 0.140),
        (-0.040, 0.126),
        (-0.044, 0.113),
        (-0.041, 0.098),
        (-0.034, 0.052),
    ]
    housing = _extrude_xz_profile(profile, BODY_THICKNESS)
    selector_pocket = (
        cq.Workplane("XZ")
        .circle(0.0205)
        .extrude(0.008)
        .translate((0.0, BODY_THICKNESS * 0.5 - 0.008, 0.090))
    )
    display_pocket = (
        cq.Workplane("XY")
        .box(0.046, 0.003, 0.026)
        .translate((0.0, BODY_THICKNESS * 0.5 - 0.0015, 0.132))
    )
    button_pocket = (
        cq.Workplane("XY")
        .box(0.042, 0.0035, 0.014)
        .translate((0.0, BODY_THICKNESS * 0.5 - 0.00175, 0.062))
    )
    trigger_pocket = (
        cq.Workplane("XY")
        .box(0.024, 0.010, 0.062)
        .translate((0.0, BODY_THICKNESS * 0.5 - 0.005, 0.039))
    )
    return housing.cut(selector_pocket).cut(display_pocket).cut(button_pocket).cut(trigger_pocket)


def _build_upper_jaw_arc() -> cq.Workplane:
    hinge_x, hinge_z = _hinge_point()
    jaw_arc = _jaw_ring().intersect(_jaw_sector(20.0, 144.0))
    return jaw_arc.translate((-hinge_x, 0.0, -hinge_z))


def _build_head_shroud() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.074, 0.052, 0.030)
        .edges("|Y")
        .fillet(0.004)
        .edges(">Z")
        .fillet(0.0025)
    )


def _build_trigger() -> cq.Workplane:
    profile = [
        (-0.011, -0.029),
        (0.011, -0.029),
        (0.010, -0.006),
        (0.007, 0.020),
        (0.004, 0.031),
        (-0.004, 0.031),
        (-0.007, 0.020),
        (-0.010, -0.006),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.014).translate((0.0, 0.014, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hvac_clamp_meter")

    housing_yellow = model.material("housing_yellow", rgba=(0.92, 0.71, 0.14, 1.0))
    overmold = model.material("overmold", rgba=(0.16, 0.17, 0.18, 1.0))
    jaw_dark = model.material("jaw_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.22, 0.34, 0.38, 0.45))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_housing(), "clamp_meter_body_housing"),
        material=housing_yellow,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(_build_head_shroud(), "clamp_meter_head_shroud"),
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        material=housing_yellow,
        name="head_shroud",
    )
    body.visual(
        mesh_from_cadquery(
            _build_fixed_jaw(),
            "clamp_meter_fixed_jaw",
        ),
        material=jaw_dark,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.034, JAW_THICKNESS, 0.026)),
        origin=Origin(xyz=(-0.018, 0.0, 0.148)),
        material=jaw_dark,
        name="jaw_bridge",
    )
    hinge_x, hinge_z = _hinge_point()
    lug_offset = JAW_THICKNESS * 0.5 - 0.004
    for index, lug_y in enumerate((-lug_offset, lug_offset)):
        body.visual(
            Cylinder(radius=0.0042, length=0.008),
            origin=Origin(xyz=(hinge_x, lug_y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=jaw_dark,
            name=f"hinge_lug_{index}",
        )

    body.visual(
        Box((0.056, 0.003, 0.039)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.0015, 0.123)),
        material=overmold,
        name="control_face",
    )
    body.visual(
        Box((0.040, 0.0025, 0.023)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.0043, 0.132)),
        material=glass,
        name="display",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.003),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.0015, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=overmold,
        name="selector_seat",
    )
    body.visual(
        Box((0.006, 0.003, 0.060)),
        origin=Origin(xyz=(-0.017, BODY_THICKNESS * 0.5 + 0.0015, 0.039)),
        material=overmold,
        name="trigger_rail_0",
    )
    body.visual(
        Box((0.006, 0.003, 0.060)),
        origin=Origin(xyz=(0.017, BODY_THICKNESS * 0.5 + 0.0015, 0.039)),
        material=overmold,
        name="trigger_rail_1",
    )
    body.visual(
        Box((0.038, 0.003, 0.008)),
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.0015, 0.072)),
        material=overmold,
        name="trigger_bridge",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.092, 0.056, 0.202)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
    )

    upper_jaw = model.part("upper_jaw")
    upper_jaw.visual(
        mesh_from_cadquery(_build_upper_jaw_arc(), "clamp_meter_upper_jaw_arc"),
        material=jaw_dark,
        name="jaw_arc",
    )
    upper_jaw.visual(
        Cylinder(radius=0.0040, length=0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=jaw_dark,
        name="barrel",
    )
    upper_jaw.visual(
        Box((0.008, 0.007, 0.011)),
        origin=Origin(xyz=(0.004, 0.0, 0.006)),
        material=jaw_dark,
        name="cheek",
    )
    upper_jaw.inertial = Inertial.from_geometry(
        Box((0.074, 0.024, 0.054)),
        mass=0.06,
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
    )
    model.articulation(
        "body_to_upper_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_jaw,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.014,
                body_style="skirted",
                top_diameter=0.031,
                grip=KnobGrip(style="fluted", count=12, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "clamp_meter_selector",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=jaw_dark,
        name="knob",
    )
    selector.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.014),
        mass=0.025,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, 0.0325, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_build_trigger(), "clamp_meter_trigger"),
        material=jaw_dark,
        name="trigger",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.062)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.007, 0.001)),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, BODY_THICKNESS * 0.5 + 0.003, 0.039)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.007),
    )

    button_positions = (-0.014, 0.014)
    for index, button_x in enumerate(button_positions):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.016, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=jaw_dark,
            name="button",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.016, 0.006, 0.010)),
            mass=0.006,
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BODY_THICKNESS * 0.5 + 0.003, 0.062)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0018),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_jaw = object_model.get_part("upper_jaw")
    selector = object_model.get_part("selector")
    trigger = object_model.get_part("trigger")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")

    jaw_joint = object_model.get_articulation("body_to_upper_jaw")
    selector_joint = object_model.get_articulation("body_to_selector")
    trigger_joint = object_model.get_articulation("body_to_trigger")
    button_0_joint = object_model.get_articulation("body_to_mode_button_0")
    button_1_joint = object_model.get_articulation("body_to_mode_button_1")

    ctx.expect_gap(
        trigger,
        body,
        axis="y",
        positive_elem="trigger",
        negative_elem="housing",
        min_gap=0.0025,
        max_gap=0.0040,
        name="trigger starts proud of the handle slot",
    )
    ctx.expect_gap(
        button_0,
        body,
        axis="y",
        positive_elem="button",
        negative_elem="housing",
        min_gap=0.0025,
        max_gap=0.0040,
        name="mode button 0 sits proud of the lower face",
    )
    ctx.expect_gap(
        button_1,
        body,
        axis="y",
        positive_elem="button",
        negative_elem="housing",
        min_gap=0.0025,
        max_gap=0.0040,
        name="mode button 1 sits proud of the lower face",
    )

    ctx.check(
        "selector articulation is continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_joint.articulation_type}",
    )

    selector_rest = ctx.part_world_position(selector)
    with ctx.pose(body_to_selector=1.4):
        selector_rotated = ctx.part_world_position(selector)
    ctx.check(
        "selector rotates about its center axis",
        selector_rest is not None
        and selector_rotated is not None
        and abs(selector_rotated[0] - selector_rest[0]) < 1e-6
        and abs(selector_rotated[1] - selector_rest[1]) < 1e-6
        and abs(selector_rotated[2] - selector_rest[2]) < 1e-6,
        details=f"rest={selector_rest}, rotated={selector_rotated}",
    )

    closed_jaw_aabb = ctx.part_world_aabb(upper_jaw)
    jaw_upper = jaw_joint.motion_limits.upper if jaw_joint.motion_limits is not None else None
    if jaw_upper is not None:
        with ctx.pose(body_to_upper_jaw=jaw_upper):
            open_jaw_aabb = ctx.part_world_aabb(upper_jaw)
        ctx.check(
            "upper jaw opens upward from the split jaw",
            closed_jaw_aabb is not None
            and open_jaw_aabb is not None
            and open_jaw_aabb[1][2] > closed_jaw_aabb[1][2] + 0.035
            and open_jaw_aabb[1][0] < closed_jaw_aabb[1][0] - 0.015,
            details=f"closed={closed_jaw_aabb}, open={open_jaw_aabb}",
        )

    trigger_rest = ctx.part_world_position(trigger)
    trigger_upper = trigger_joint.motion_limits.upper if trigger_joint.motion_limits is not None else None
    if trigger_upper is not None:
        with ctx.pose(body_to_trigger=trigger_upper):
            trigger_pressed = ctx.part_world_position(trigger)
        ctx.check(
            "trigger retracts into the handle",
            trigger_rest is not None
            and trigger_pressed is not None
            and trigger_pressed[1] < trigger_rest[1] - 0.005,
            details=f"rest={trigger_rest}, pressed={trigger_pressed}",
        )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_0_upper = button_0_joint.motion_limits.upper if button_0_joint.motion_limits is not None else None
    button_1_upper = button_1_joint.motion_limits.upper if button_1_joint.motion_limits is not None else None
    if button_0_upper is not None:
        with ctx.pose(body_to_mode_button_0=button_0_upper):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_steady = ctx.part_world_position(button_1)
        ctx.check(
            "mode button 0 presses independently",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_1_rest is not None
            and button_1_steady is not None
            and button_0_pressed[1] < button_0_rest[1] - 0.0015
            and abs(button_1_steady[1] - button_1_rest[1]) < 1e-6,
            details=f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, button_1_rest={button_1_rest}, button_1_steady={button_1_steady}",
        )
    if button_1_upper is not None:
        with ctx.pose(body_to_mode_button_1=button_1_upper):
            button_1_pressed = ctx.part_world_position(button_1)
            button_0_steady = ctx.part_world_position(button_0)
        ctx.check(
            "mode button 1 presses independently",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_0_rest is not None
            and button_0_steady is not None
            and button_1_pressed[1] < button_1_rest[1] - 0.0015
            and abs(button_0_steady[1] - button_0_rest[1]) < 1e-6,
            details=f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}, button_0_rest={button_0_rest}, button_0_steady={button_0_steady}",
        )

    return ctx.report()


object_model = build_object_model()

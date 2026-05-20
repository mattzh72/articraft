from __future__ import annotations

from math import cos, pi, radians, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_THICKNESS = 0.044
BODY_FRONT_Y = BODY_THICKNESS / 2.0

JAW_CENTER_Z = 0.198
JAW_OUTER_RADIUS = 0.028
JAW_INNER_RADIUS = 0.019
JAW_DEPTH = 0.028
JAW_MID_RADIUS = (JAW_OUTER_RADIUS + JAW_INNER_RADIUS) * 0.5

HINGE_ANGLE_DEG = 152.0
UPPER_TIP_ANGLE_DEG = 44.0
LOWER_TIP_ANGLE_DEG = 22.0

DIAL_Z = 0.104
DISPLAY_Z = 0.141
TRIGGER_Z = 0.058
RANGE_Z = 0.104
NCV_Z = 0.175

DIAL_X = 0.0
TRIGGER_X = 0.0
RANGE_X = 0.027
NCV_X = -0.019


def polar_xz(radius: float, angle_deg: float, *, z_offset: float = JAW_CENTER_Z) -> tuple[float, float]:
    angle = radians(angle_deg)
    return (radius * cos(angle), z_offset + radius * sin(angle))


HINGE_X, HINGE_Z = polar_xz(JAW_MID_RADIUS, HINGE_ANGLE_DEG)
UPPER_TIP_X, UPPER_TIP_Z = polar_xz(JAW_MID_RADIUS, UPPER_TIP_ANGLE_DEG)
LOWER_TIP_X, LOWER_TIP_Z = polar_xz(JAW_MID_RADIUS, LOWER_TIP_ANGLE_DEG)


def wedge_cutter(
    start_deg: float,
    end_deg: float,
    *,
    reach: float = 0.090,
    depth: float = 0.080,
    center_x: float = 0.0,
    center_z: float = JAW_CENTER_Z,
) -> cq.Workplane:
    p1 = (
        center_x + reach * cos(radians(start_deg)),
        center_z + reach * sin(radians(start_deg)),
    )
    p2 = (
        center_x + reach * cos(radians(end_deg)),
        center_z + reach * sin(radians(end_deg)),
    )
    return (
        cq.Workplane("XZ")
        .polyline([(center_x, center_z), p1, p2])
        .close()
        .extrude(depth)
        .translate((0.0, depth * 0.5, 0.0))
    )


def make_body_shape() -> cq.Workplane:
    outline = [
        (-0.028, 0.000),
        (0.028, 0.000),
        (0.032, 0.078),
        (0.038, 0.126),
        (0.043, 0.146),
        (0.043, 0.153),
        (0.032, 0.167),
        (0.024, 0.167),
        (0.024, 0.179),
        (0.034, 0.179),
        (0.039, 0.188),
        (0.039, 0.207),
        (0.015, 0.207),
        (0.011, 0.191),
        (-0.011, 0.191),
        (-0.015, 0.207),
        (-0.039, 0.207),
        (-0.039, 0.188),
        (-0.034, 0.179),
        (-0.024, 0.179),
        (-0.024, 0.167),
        (-0.032, 0.167),
        (-0.043, 0.153),
        (-0.043, 0.146),
        (-0.038, 0.126),
        (-0.032, 0.078),
    ]

    shell = (
        cq.Workplane("XZ")
        .polyline(outline)
        .close()
        .extrude(BODY_THICKNESS)
        .translate((0.0, BODY_FRONT_Y, 0.0))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.054, BODY_THICKNESS + 0.006, 0.020)
        .translate((0.0, 0.0, 0.205))
    )

    ring = (
        cq.Workplane("XZ")
        .center(0.0, JAW_CENTER_Z)
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(JAW_DEPTH)
        .translate((0.0, JAW_DEPTH * 0.5, 0.0))
    )
    lower_jaw = ring.cut(wedge_cutter(24.0, 170.0))

    front_lug = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(0.0055)
        .extrude(0.006)
        .translate((0.0, 0.018, 0.0))
    )
    rear_lug = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(0.0055)
        .extrude(0.006)
        .translate((0.0, -0.012, 0.0))
    )
    body = shell.union(lower_jaw).union(front_lug).union(rear_lug)

    return body


def make_upper_jaw_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .center(0.0, JAW_CENTER_Z)
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(0.024)
        .translate((0.0, 0.012, 0.0))
    )

    jaw = ring.cut(wedge_cutter(136.0, 270.0)).cut(wedge_cutter(270.0, 402.0))
    barrel = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(0.0042)
        .extrude(0.018)
        .translate((0.0, 0.009, 0.0))
    )
    return jaw.union(barrel).translate((-HINGE_X, 0.0, -HINGE_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hvac_clamp_meter")

    shell_mat = model.material("shell_orange", rgba=(0.93, 0.43, 0.12, 1.0))
    charcoal_mat = model.material("charcoal", rgba=(0.16, 0.16, 0.17, 1.0))
    bezel_mat = model.material("bezel_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    screen_mat = model.material("screen_tint", rgba=(0.06, 0.10, 0.08, 1.0))
    cap_mat = model.material("cap_gray", rgba=(0.68, 0.70, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shape(), "body_shell"),
        material=shell_mat,
        name="shell",
    )
    body.visual(
        Box((0.004, 0.008, 0.004)),
        origin=Origin(xyz=(LOWER_TIP_X, 0.0, LOWER_TIP_Z)),
        material=shell_mat,
        name="jaw_tip",
    )

    display = model.part("display")
    display.visual(
        Box((0.046, 0.003, 0.032)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=bezel_mat,
        name="bezel",
    )
    display.visual(
        Box((0.036, 0.001, 0.024)),
        origin=Origin(xyz=(0.0, 0.0035, 0.0)),
        material=screen_mat,
        name="lens",
    )
    model.articulation(
        "display_mount",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DISPLAY_Z)),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=charcoal_mat,
        name="dial",
    )
    selector.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=cap_mat,
        name="cap",
    )
    selector.visual(
        Box((0.004, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, 0.010)),
        material=cap_mat,
        name="pointer",
    )
    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(DIAL_X, BODY_FRONT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.024, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=charcoal_mat,
        name="paddle",
    )
    model.articulation(
        "trigger_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(TRIGGER_X, BODY_FRONT_Y, TRIGGER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.06, lower=0.0, upper=0.004),
    )

    ncv_button = model.part("ncv_button")
    ncv_button.visual(
        Cylinder(radius=0.005, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0016, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=charcoal_mat,
        name="cap",
    )
    model.articulation(
        "ncv_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=ncv_button,
        origin=Origin(xyz=(NCV_X, BODY_FRONT_Y, NCV_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.0025),
    )

    range_button = model.part("range_button")
    range_button.visual(
        Box((0.014, 0.003, 0.007)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=charcoal_mat,
        name="cap",
    )
    model.articulation(
        "range_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=range_button,
        origin=Origin(xyz=(RANGE_X, BODY_FRONT_Y, RANGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.0025),
    )

    upper_jaw = model.part("upper_jaw")
    upper_jaw.visual(
        mesh_from_cadquery(make_upper_jaw_shape(), "upper_jaw"),
        material=charcoal_mat,
        name="jaw",
    )
    upper_jaw.visual(
        Box((0.004, 0.008, 0.004)),
        origin=Origin(
            xyz=(
                UPPER_TIP_X - HINGE_X,
                0.0,
                UPPER_TIP_Z - HINGE_Z,
            )
        ),
        material=charcoal_mat,
        name="tip",
    )
    model.articulation(
        "jaw_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_jaw,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    upper_jaw = object_model.get_part("upper_jaw")
    selector = object_model.get_part("selector")
    trigger = object_model.get_part("trigger")
    ncv_button = object_model.get_part("ncv_button")
    range_button = object_model.get_part("range_button")

    jaw_hinge = object_model.get_articulation("jaw_hinge")
    selector_spin = object_model.get_articulation("selector_spin")
    trigger_slide = object_model.get_articulation("trigger_slide")
    ncv_press = object_model.get_articulation("ncv_press")
    range_press = object_model.get_articulation("range_press")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        upper_jaw,
        body,
        axis="z",
        positive_elem="tip",
        negative_elem="jaw_tip",
        max_gap=0.006,
        max_penetration=0.0,
        name="jaw closes to a narrow split",
    )

    jaw_upper = jaw_hinge.motion_limits.upper if jaw_hinge.motion_limits is not None else None
    closed_tip = aabb_center(ctx.part_element_world_aabb(upper_jaw, elem="tip"))
    with ctx.pose({jaw_hinge: jaw_upper}):
        open_tip = aabb_center(ctx.part_element_world_aabb(upper_jaw, elem="tip"))
    ctx.check(
        "jaw opens upward",
        closed_tip is not None
        and open_tip is not None
        and jaw_upper is not None
        and open_tip[2] > closed_tip[2] + 0.015,
        details=f"closed_tip={closed_tip}, open_tip={open_tip}, upper={jaw_upper}",
    )

    pointer_rest = aabb_center(ctx.part_element_world_aabb(selector, elem="pointer"))
    with ctx.pose({selector_spin: pi * 0.5}):
        pointer_turn = aabb_center(ctx.part_element_world_aabb(selector, elem="pointer"))
    ctx.check(
        "selector pointer sweeps around the dial",
        pointer_rest is not None
        and pointer_turn is not None
        and abs(pointer_turn[0] - pointer_rest[0]) > 0.008
        and abs(pointer_turn[2] - pointer_rest[2]) > 0.004,
        details=f"rest={pointer_rest}, turned={pointer_turn}",
    )

    trigger_rest = ctx.part_world_position(trigger)
    trigger_upper = trigger_slide.motion_limits.upper if trigger_slide.motion_limits is not None else None
    with ctx.pose({trigger_slide: trigger_upper}):
        trigger_pressed = ctx.part_world_position(trigger)
    ctx.check(
        "trigger slides into the handle",
        trigger_rest is not None
        and trigger_pressed is not None
        and trigger_upper is not None
        and trigger_pressed[1] < trigger_rest[1] - 0.003,
        details=f"rest={trigger_rest}, pressed={trigger_pressed}, upper={trigger_upper}",
    )

    ncv_rest = ctx.part_world_position(ncv_button)
    range_rest = ctx.part_world_position(range_button)
    ncv_upper = ncv_press.motion_limits.upper if ncv_press.motion_limits is not None else None
    range_upper = range_press.motion_limits.upper if range_press.motion_limits is not None else None

    with ctx.pose({ncv_press: ncv_upper}):
        ncv_pressed = ctx.part_world_position(ncv_button)
        range_during_ncv = ctx.part_world_position(range_button)
    ctx.check(
        "ncv button presses independently",
        ncv_rest is not None
        and ncv_pressed is not None
        and range_rest is not None
        and range_during_ncv is not None
        and ncv_upper is not None
        and ncv_pressed[1] < ncv_rest[1] - 0.0015
        and abs(range_during_ncv[1] - range_rest[1]) < 1e-6,
        details=(
            f"ncv_rest={ncv_rest}, ncv_pressed={ncv_pressed}, "
            f"range_rest={range_rest}, range_during_ncv={range_during_ncv}"
        ),
    )

    with ctx.pose({range_press: range_upper}):
        range_pressed = ctx.part_world_position(range_button)
        ncv_during_range = ctx.part_world_position(ncv_button)
    ctx.check(
        "range button presses independently",
        range_rest is not None
        and range_pressed is not None
        and ncv_rest is not None
        and ncv_during_range is not None
        and range_upper is not None
        and range_pressed[1] < range_rest[1] - 0.0015
        and abs(ncv_during_range[1] - ncv_rest[1]) < 1e-6,
        details=(
            f"range_rest={range_rest}, range_pressed={range_pressed}, "
            f"ncv_rest={ncv_rest}, ncv_during_range={ncv_during_range}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

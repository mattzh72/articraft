from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.034
BODY_FRONT_Y = BODY_DEPTH * 0.5
DISPLAY_RECESS_DEPTH = 0.0016
BUTTON_RECESS_DEPTH = 0.0012
TRIGGER_SLOT_DEPTH = 0.018

JAW_CENTER_X = -0.004
JAW_CENTER_Z = 0.181
JAW_OUTER_RADIUS = 0.031
JAW_INNER_RADIUS = 0.019
JAW_DEPTH = 0.036

HINGE_ANGLE_DEG = 148.0
HINGE_X = JAW_CENTER_X + JAW_OUTER_RADIUS * math.cos(math.radians(HINGE_ANGLE_DEG))
HINGE_Z = JAW_CENTER_Z + JAW_OUTER_RADIUS * math.sin(math.radians(HINGE_ANGLE_DEG))


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    if end_deg < start_deg:
        end_deg += 360.0
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append(
            (
                center_x + radius * math.cos(angle),
                center_z + radius * math.sin(angle),
            )
        )
    return points


def _annular_sector(
    center_x: float,
    center_z: float,
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    *,
    depth: float,
    segments: int = 28,
):
    outer = _arc_points(center_x, center_z, outer_radius, start_deg, end_deg, segments=segments)
    inner = list(
        reversed(
            _arc_points(center_x, center_z, inner_radius, start_deg, end_deg, segments=segments)
        )
    )
    return cq.Workplane("XZ").polyline(outer + inner).close().extrude(depth, both=True)


def _front_box_cut(width: float, depth: float, height: float, *, center_x: float, center_z: float):
    return cq.Workplane("XY").box(width, depth, height).translate(
        (center_x, BODY_FRONT_Y - (depth * 0.5), center_z)
    )


def _body_shape():
    housing = (
        cq.Workplane("XY")
        .rect(0.044, BODY_DEPTH)
        .workplane(offset=0.090)
        .rect(0.050, BODY_DEPTH)
        .workplane(offset=0.040)
        .rect(0.058, BODY_DEPTH)
        .workplane(offset=0.011)
        .rect(0.060, BODY_DEPTH)
        .loft()
    )

    head_bridge = cq.Workplane("XY").box(0.028, 0.036, 0.010).translate((-0.014, 0.0, 0.145))
    body = housing.union(head_bridge)

    body = body.cut(_front_box_cut(0.046, DISPLAY_RECESS_DEPTH, 0.032, center_x=0.0, center_z=0.121))
    body = body.cut(_front_box_cut(0.052, BUTTON_RECESS_DEPTH, 0.020, center_x=0.0, center_z=0.046))
    body = body.cut(_front_box_cut(0.018, TRIGGER_SLOT_DEPTH, 0.026, center_x=0.0, center_z=0.150))
    return body


def _jaw_frame_shape():
    jaw_neck = cq.Workplane("XY").box(0.022, 0.036, 0.026).translate((-0.020, 0.0, 0.181))
    fixed_jaw = _annular_sector(
        JAW_CENTER_X,
        JAW_CENTER_Z,
        JAW_OUTER_RADIUS,
        JAW_INNER_RADIUS,
        205.0,
        350.0,
        depth=JAW_DEPTH,
    )
    return jaw_neck.union(fixed_jaw)


def _jaw_arc_shape():
    jaw_arc = _annular_sector(
        JAW_CENTER_X,
        JAW_CENTER_Z,
        JAW_OUTER_RADIUS,
        JAW_INNER_RADIUS,
        10.0,
        132.0,
        depth=JAW_DEPTH * 0.88,
    )
    return jaw_arc.translate((-HINGE_X, 0.0, -HINGE_Z))
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_meter")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    face_dark = model.material("face_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.22, 0.42, 0.34, 0.55))
    button_gray = model.material("button_gray", rgba=(0.28, 0.30, 0.31, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "clamp_meter_body"), material=body_dark, name="shell")
    body.inertial = Inertial.from_geometry(
        Box((0.072, 0.038, 0.212)),
        mass=0.33,
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
    )

    jaw_frame = model.part("jaw_frame")
    jaw_frame.visual(
        mesh_from_cadquery(_jaw_frame_shape(), "clamp_meter_jaw_frame"),
        material=face_dark,
        name="frame",
    )
    jaw_frame.inertial = Inertial.from_geometry(
        Box((0.050, JAW_DEPTH, 0.060)),
        mass=0.05,
        origin=Origin(xyz=(-0.010, 0.0, 0.182)),
    )
    model.articulation(
        "body_to_jaw_frame",
        ArticulationType.FIXED,
        parent=body,
        child=jaw_frame,
        origin=Origin(),
    )

    display = model.part("display")
    display.visual(
        Box((0.040, 0.0014, 0.027)),
        origin=Origin(xyz=(0.0, 0.0007, 0.0)),
        material=glass,
        name="window",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - DISPLAY_RECESS_DEPTH, 0.121)),
    )

    jaw = model.part("jaw")
    jaw.visual(mesh_from_cadquery(_jaw_arc_shape(), "clamp_meter_jaw_arc"), material=face_dark, name="jaw_arc")
    jaw.visual(
        Cylinder(radius=0.0046, length=JAW_DEPTH * 0.94),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=face_dark,
        name="hinge_barrel",
    )
    jaw.visual(
        Box((0.014, JAW_DEPTH * 0.72, 0.010)),
        origin=Origin(xyz=(0.007, 0.0, -0.002)),
        material=face_dark,
        name="hinge_bridge",
    )
    jaw.visual(
        Box((0.005, JAW_DEPTH * 0.58, 0.005)),
        origin=Origin(
            xyz=(
                (JAW_CENTER_X + 0.024) - HINGE_X,
                0.0,
                (JAW_CENTER_Z + 0.006) - HINGE_Z,
            )
        ),
        material=face_dark,
        name="tip_pad",
    )
    jaw.inertial = Inertial.from_geometry(
        Box((0.065, JAW_DEPTH, 0.060)),
        mass=0.05,
        origin=Origin(xyz=(0.005, 0.0, -0.010)),
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=0.78),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.012, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.004, -0.007)),
        material=face_dark,
        name="trigger_stem",
    )
    trigger.visual(
        Box((0.018, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.006, -0.018)),
        material=face_dark,
        name="trigger_pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.018, 0.012, 0.022)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.006, -0.011)),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - TRIGGER_SLOT_DEPTH, 0.163)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.012),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.0155, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="dial_base",
    )
    selector.visual(
        Cylinder(radius=0.0125, length=0.004),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="dial_cap",
    )
    selector.visual(
        Box((0.0035, 0.0012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0102, 0.005)),
        material=button_gray,
        name="dial_pointer",
    )
    selector.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0155, length=0.010),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, 0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=8.0),
    )

    button_specs = (-0.016, 0.0, 0.016)
    for index, center_x in enumerate(button_specs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.011, 0.0034, 0.010)),
            origin=Origin(xyz=(0.0, 0.0017, 0.0)),
            material=button_gray,
            name="cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.011, 0.0034, 0.010)),
            mass=0.004,
            origin=Origin(xyz=(0.0, 0.0017, 0.0)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(center_x, BODY_FRONT_Y - BUTTON_RECESS_DEPTH, 0.046)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.0012),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    display = object_model.get_part("display")
    jaw = object_model.get_part("jaw")
    trigger = object_model.get_part("trigger")
    selector = object_model.get_part("selector")
    jaw_joint = object_model.get_articulation("body_to_jaw")
    trigger_joint = object_model.get_articulation("body_to_trigger")
    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    ctx.allow_overlap(
        body,
        trigger,
        elem_a="shell",
        elem_b="trigger_pad",
        reason="The short trigger retracts into a simplified upper-handle slot proxy.",
    )
    ctx.allow_overlap(
        "jaw_frame",
        trigger,
        elem_a="frame",
        elem_b="trigger_stem",
        reason="The trigger stem passes into the simplified head housing while sliding open the jaw.",
    )

    ctx.expect_gap(display, selector, axis="z", min_gap=0.009, name="display stays above selector")
    ctx.expect_gap(selector, button_parts[1], axis="z", min_gap=0.012, name="selector stays above button bank")
    ctx.expect_overlap(button_parts[0], button_parts[1], axes="z", min_overlap=0.008, name="button caps share a common bank height")

    closed_jaw_aabb = ctx.part_world_aabb(jaw)
    jaw_upper = jaw_joint.motion_limits.upper if jaw_joint.motion_limits is not None else None
    if jaw_upper is not None:
        with ctx.pose({jaw_joint: jaw_upper}):
            open_jaw_aabb = ctx.part_world_aabb(jaw)
        ctx.check(
            "jaw swings clear when opened",
            closed_jaw_aabb is not None
            and open_jaw_aabb is not None
            and open_jaw_aabb[0][2] < closed_jaw_aabb[0][2] - 0.020,
            details=f"closed={closed_jaw_aabb}, open={open_jaw_aabb}",
        )

    trigger_rest = ctx.part_world_position(trigger)
    trigger_upper = trigger_joint.motion_limits.upper if trigger_joint.motion_limits is not None else None
    if trigger_upper is not None:
        with ctx.pose({trigger_joint: trigger_upper}):
            trigger_pulled = ctx.part_world_position(trigger)
        ctx.check(
            "trigger slides downward",
            trigger_rest is not None
            and trigger_pulled is not None
            and trigger_pulled[2] < trigger_rest[2] - 0.008,
            details=f"rest={trigger_rest}, pulled={trigger_pulled}",
        )

    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0008,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.expect_overlap(selector, body, axes="x", min_overlap=0.020, name="selector stays centered on the control face")

    return ctx.report()


object_model = build_object_model()

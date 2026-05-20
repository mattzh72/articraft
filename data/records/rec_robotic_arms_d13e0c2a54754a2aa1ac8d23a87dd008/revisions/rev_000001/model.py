from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size, center, fillet: float = 0.0):
    shape = cq.Workplane("XY").box(*size).translate(center)
    if fillet > 0:
        shape = _safe_fillet(shape, radius=fillet)
    return shape


def _safe_fillet(shape, radius: float, selector: str | None = None):
    try:
        if selector is None:
            return shape.edges().fillet(radius)
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _cylinder(radius: float, length: float, center, axis: str = "z"):
    shape = cq.Workplane("XY").cylinder(length, radius)
    if axis == "x":
        shape = shape.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        shape = shape.rotate((0, 0, 0), (1, 0, 0), -90)
    return shape.translate(center)


def _union_all(shapes):
    body = shapes[0]
    for shape in shapes[1:]:
        body = body.union(shape)
    return body


def _base_body():
    shapes = [
        _cylinder(0.29, 0.075, (0.0, 0.0, 0.0375)),
        _cylinder(0.145, 0.185, (0.0, 0.0, 0.165)),
        _cylinder(0.185, 0.045, (0.0, 0.0, 0.2775)),
    ]
    for i in range(8):
        angle = i * math.tau / 8.0
        x = 0.225 * math.cos(angle)
        y = 0.225 * math.sin(angle)
        shapes.append(_cylinder(0.020, 0.014, (x, y, 0.082)))
    body = _union_all(shapes)
    return _safe_fillet(body, radius=0.008, selector="|Z")


def _shoulder_body():
    shapes = [
        _cylinder(0.178, 0.078, (0.0, 0.0, 0.039)),
        _cylinder(0.110, 0.180, (0.0, 0.0, 0.168)),
        _box((0.155, 0.170, 0.050), (0.0, 0.0, 0.185), fillet=0.014),
        _box((0.165, 0.044, 0.230), (0.0, 0.078, 0.280), fillet=0.012),
        _box((0.165, 0.044, 0.230), (0.0, -0.078, 0.280), fillet=0.012),
    ]
    body = _union_all(shapes)
    bore = _cylinder(0.047, 0.260, (0.0, 0.0, 0.320), axis="y")
    swing = _cylinder(0.067, 0.128, (0.0, 0.0, 0.320), axis="y")
    body = body.cut(bore).cut(swing)
    return _safe_fillet(body, radius=0.006, selector="|Z")


def _upper_arm_body():
    shapes = [
        _cylinder(0.080, 0.112, (0.0, 0.0, 0.0), axis="y"),
        _cylinder(0.034, 0.232, (0.0, 0.0, 0.0), axis="y"),
        _box((0.130, 0.086, 0.086), (0.126, 0.0, 0.0), fillet=0.018),
        _box((0.405, 0.135, 0.100), (0.352, 0.0, 0.0), fillet=0.022),
        _box((0.148, 0.035, 0.178), (0.618, 0.0715, 0.0), fillet=0.014),
        _box((0.148, 0.035, 0.178), (0.618, -0.0715, 0.0), fillet=0.014),
    ]
    body = _union_all(shapes)
    bore = _cylinder(0.040, 0.240, (0.622, 0.0, 0.0), axis="y")
    body = body.cut(bore)
    return _safe_fillet(body, radius=0.004, selector="|X")


def _forearm_body():
    shapes = [
        _cylinder(0.070, 0.108, (0.0, 0.0, 0.0), axis="y"),
        _cylinder(0.030, 0.216, (0.0, 0.0, 0.0), axis="y"),
        _box((0.115, 0.082, 0.082), (0.105, 0.0, 0.0), fillet=0.016),
        _box((0.325, 0.104, 0.086), (0.300, 0.0, 0.0), fillet=0.018),
        _box((0.132, 0.030, 0.145), (0.516, 0.060, 0.0), fillet=0.012),
        _box((0.132, 0.030, 0.145), (0.516, -0.060, 0.0), fillet=0.012),
    ]
    body = _union_all(shapes)
    bore = _cylinder(0.032, 0.205, (0.520, 0.0, 0.0), axis="y")
    body = body.cut(bore)
    return _safe_fillet(body, radius=0.004, selector="|X")


def _wrist_pitch_body():
    shapes = [
        _cylinder(0.052, 0.090, (0.0, 0.0, 0.0), axis="y"),
        _cylinder(0.024, 0.172, (0.0, 0.0, 0.0), axis="y"),
        _box((0.118, 0.066, 0.070), (0.078, 0.0, 0.0), fillet=0.012),
        _cylinder(0.047, 0.080, (0.115, 0.0, 0.0), axis="x"),
    ]
    return _safe_fillet(_union_all(shapes), radius=0.003)


def _wrist_roll_body():
    shapes = [
        _cylinder(0.044, 0.170, (0.085, 0.0, 0.0), axis="x"),
        _cylinder(0.070, 0.026, (0.180, 0.0, 0.0), axis="x"),
        _box((0.115, 0.165, 0.075), (0.2175, 0.0, 0.0), fillet=0.010),
        _box((0.050, 0.025, 0.095), (0.250, 0.075, 0.0), fillet=0.006),
        _box((0.050, 0.025, 0.095), (0.250, -0.075, 0.0), fillet=0.006),
    ]
    return _safe_fillet(_union_all(shapes), radius=0.003)


def _finger_body():
    shapes = [
        _box((0.050, 0.026, 0.062), (0.000, 0.0, 0.0), fillet=0.006),
        _box((0.135, 0.022, 0.036), (0.075, 0.0, 0.000), fillet=0.006),
        _box((0.034, 0.030, 0.072), (0.150, 0.0, -0.018), fillet=0.006),
    ]
    return _safe_fillet(_union_all(shapes), radius=0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cad_robotic_arm")

    dark = model.material("anodized_dark", rgba=(0.06, 0.065, 0.07, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    alloy = model.material("brushed_alloy", rgba=(0.72, 0.74, 0.72, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.06, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_casting", tolerance=0.0015),
        material=dark,
        name="base_casting",
    )

    shoulder = model.part("shoulder")
    shoulder.visual(
        mesh_from_cadquery(_shoulder_body(), "shoulder_yoke", tolerance=0.0015),
        material=graphite,
        name="shoulder_yoke",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_body(), "upper_arm_link", tolerance=0.0015),
        material=orange,
        name="upper_arm_link",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_body(), "forearm_link", tolerance=0.0015),
        material=alloy,
        name="forearm_link",
    )

    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(
        mesh_from_cadquery(_wrist_pitch_body(), "wrist_pitch_housing", tolerance=0.0015),
        material=graphite,
        name="wrist_pitch_housing",
    )

    wrist_roll = model.part("wrist_roll")
    wrist_roll.visual(
        mesh_from_cadquery(_wrist_roll_body(), "wrist_roll_tool", tolerance=0.0015),
        material=dark,
        name="wrist_roll_tool",
    )

    finger_0 = model.part("finger_0")
    finger_0.visual(
        mesh_from_cadquery(_finger_body(), "finger_0_jaw", tolerance=0.0015),
        material=graphite,
        name="finger_0_jaw",
    )
    finger_0.visual(
        Box((0.060, 0.006, 0.048)),
        origin=Origin(xyz=(0.122, -0.014, -0.008)),
        material=rubber,
        name="finger_0_pad",
    )

    finger_1 = model.part("finger_1")
    finger_1.visual(
        mesh_from_cadquery(_finger_body(), "finger_1_jaw", tolerance=0.0015),
        material=graphite,
        name="finger_1_jaw",
    )
    finger_1.visual(
        Box((0.060, 0.006, 0.048)),
        origin=Origin(xyz=(0.122, 0.014, -0.008)),
        material=rubber,
        name="finger_1_pad",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.6),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.3, lower=-1.35, upper=1.15),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.622, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=190.0, velocity=1.6, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "wrist_pitch_axis",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch,
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.2, lower=-1.85, upper=1.85),
    )
    model.articulation(
        "wrist_roll_axis",
        ArticulationType.CONTINUOUS,
        parent=wrist_pitch,
        child=wrist_roll,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=3.0),
    )
    model.articulation(
        "finger_0_slide",
        ArticulationType.PRISMATIC,
        parent=wrist_roll,
        child=finger_0,
        origin=Origin(xyz=(0.300, 0.033, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.10, lower=0.0, upper=0.036),
    )
    model.articulation(
        "finger_1_slide",
        ArticulationType.PRISMATIC,
        parent=wrist_roll,
        child=finger_1,
        origin=Origin(xyz=(0.300, -0.033, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.10, lower=0.0, upper=0.036),
        mimic=Mimic(joint="finger_0_slide", multiplier=1.0, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_pitch = object_model.get_part("wrist_pitch")
    wrist_roll = object_model.get_part("wrist_roll")
    finger_0 = object_model.get_part("finger_0")
    finger_1 = object_model.get_part("finger_1")

    ctx.allow_overlap(
        shoulder,
        upper_arm,
        elem_a="shoulder_yoke",
        elem_b="upper_arm_link",
        reason="The shoulder trunnion and bearing hub are intentionally modeled as a captured seated pivot inside the simplified yoke bore.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="upper_arm_link",
        elem_b="forearm_link",
        reason="The elbow axle and bearing hub are intentionally represented as a captured pin through the forked elbow housing.",
    )
    ctx.allow_overlap(
        forearm,
        wrist_pitch,
        elem_a="forearm_link",
        elem_b="wrist_pitch_housing",
        reason="The wrist pitch trunnion is intentionally represented as a seated captured pivot in the forearm fork.",
    )

    ctx.expect_contact(
        shoulder,
        base,
        elem_a="shoulder_yoke",
        elem_b="base_casting",
        contact_tol=0.002,
        name="turntable sits on pedestal",
    )
    ctx.expect_within(
        upper_arm,
        shoulder,
        axes="y",
        inner_elem="upper_arm_link",
        outer_elem="shoulder_yoke",
        margin=0.080,
        name="upper arm captured by shoulder yoke span",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="y",
        inner_elem="forearm_link",
        outer_elem="upper_arm_link",
        margin=0.075,
        name="forearm captured by elbow fork span",
    )
    ctx.expect_within(
        wrist_pitch,
        forearm,
        axes="y",
        inner_elem="wrist_pitch_housing",
        outer_elem="forearm_link",
        margin=0.055,
        name="wrist pitch hub captured by forearm fork span",
    )
    ctx.expect_overlap(
        wrist_pitch,
        forearm,
        axes="x",
        elem_a="wrist_pitch_housing",
        elem_b="forearm_link",
        min_overlap=0.04,
        name="wrist pitch pivot remains inserted",
    )
    ctx.expect_contact(
        wrist_roll,
        wrist_pitch,
        elem_a="wrist_roll_tool",
        elem_b="wrist_pitch_housing",
        contact_tol=0.002,
        name="roll bearing seats against wrist housing",
    )
    ctx.expect_gap(
        finger_0,
        finger_1,
        axis="y",
        positive_elem="finger_0_pad",
        negative_elem="finger_1_pad",
        min_gap=0.010,
        name="gripper pads have a real clearance",
    )

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    finger_slide = object_model.get_articulation("finger_0_slide")

    rest_forearm = ctx.part_world_position(forearm)
    with ctx.pose({shoulder_pitch: 0.60, elbow_pitch: -0.80}):
        posed_forearm = ctx.part_world_position(forearm)
    ctx.check(
        "arm pitch joints move the serial chain",
        rest_forearm is not None
        and posed_forearm is not None
        and abs(posed_forearm[2] - rest_forearm[2]) > 0.15,
        details=f"rest={rest_forearm}, posed={posed_forearm}",
    )

    closed_gap_ok = ctx.expect_gap(
        finger_0,
        finger_1,
        axis="y",
        positive_elem="finger_0_pad",
        negative_elem="finger_1_pad",
        min_gap=0.010,
        name="closed gripper retains pad clearance",
    )
    closed_aabb = ctx.part_element_world_aabb(finger_0, elem="finger_0_pad")
    with ctx.pose({finger_slide: 0.030}):
        opened_aabb = ctx.part_element_world_aabb(finger_0, elem="finger_0_pad")
        ctx.expect_gap(
            finger_0,
            finger_1,
            axis="y",
            positive_elem="finger_0_pad",
            negative_elem="finger_1_pad",
            min_gap=0.060,
            name="opposing fingers slide open together",
        )
    ctx.check(
        "finger slide travels outward",
        closed_gap_ok
        and closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] > closed_aabb[0][1] + 0.025,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

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


BODY_WIDTH = 0.120
BODY_DEPTH = 0.052
HEAD_DEPTH = 0.068
BODY_HEIGHT = 0.228
JAW_CENTER_Z = 0.252
JAW_OUTER_RADIUS = 0.046
JAW_INNER_RADIUS = 0.028
JAW_PIVOT_X = -0.048
JAW_PIVOT_Z = 0.25134


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate(
        (center[0], center[1] - (length * 0.5), center[2])
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate(
        (center[0] - (length * 0.5), center[1], center[2])
    )


def _largest_solid(workplane: cq.Workplane) -> cq.Workplane:
    solids = workplane.val().Solids()
    largest = max(solids, key=lambda solid: solid.Volume())
    return cq.Workplane("XY").newObject([largest])


def _make_body_shell() -> cq.Workplane:
    body_outline = [
        (-0.041, 0.000),
        (0.041, 0.000),
        (0.050, 0.052),
        (0.058, 0.148),
        (0.058, 0.205),
        (0.050, BODY_HEIGHT),
        (-0.050, BODY_HEIGHT),
        (-0.058, 0.205),
        (-0.058, 0.148),
        (-0.050, 0.052),
    ]
    shell = cq.Workplane("XZ").polyline(body_outline).close().extrude(BODY_DEPTH / 2.0, both=True)

    head_block = _box((BODY_WIDTH, HEAD_DEPTH, 0.090), (0.0, 0.0, 0.185))
    neck_block = _box((0.086, HEAD_DEPTH, 0.030), (0.0, 0.0, 0.220))
    jaw_ring = (
        cq.Workplane("XZ")
        .center(0.0, JAW_CENTER_Z)
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(HEAD_DEPTH / 2.0, both=True)
    )
    fixed_jaw = jaw_ring.cut(_box((0.160, 0.100, 0.086), (0.0, 0.0, 0.272))).cut(
        _box((0.070, 0.100, 0.120), (-0.066, 0.0, 0.252))
    )

    shell = shell.union(head_block).union(neck_block).union(fixed_jaw)

    display_cut = _box((0.064, 0.012, 0.044), (0.0, 0.028, 0.150))
    selector_cut = _cyl_y(0.034, 0.010, (0.0, 0.021, 0.105))
    front_button_0_cut = _cyl_y(0.011, 0.010, (-0.020, 0.021, 0.057))
    front_button_1_cut = _cyl_y(0.011, 0.010, (0.020, 0.021, 0.057))
    side_button_cut = _box((0.010, 0.028, 0.018), (0.057, 0.000, 0.128))

    shell = (
        shell.cut(display_cut)
        .cut(selector_cut)
        .cut(front_button_0_cut)
        .cut(front_button_1_cut)
        .cut(side_button_cut)
    )

    shell = shell.union(_cyl_y(0.008, 0.012, (JAW_PIVOT_X, -0.017, JAW_PIVOT_Z)))
    shell = shell.union(_cyl_y(0.008, 0.012, (JAW_PIVOT_X, 0.017, JAW_PIVOT_Z)))
    shell = shell.union(_box((0.060, 0.018, 0.014), (0.0, -0.029, 0.028)))

    return _largest_solid(shell)


def _make_jaw() -> cq.Workplane:
    jaw_ring = (
        cq.Workplane("XZ")
        .center(0.048, 0.0)
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(0.010, both=True)
    )
    free_side_trim = _box((0.120, 0.080, 0.120), (0.094, 0.0, -0.066)).rotate(
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        24.0,
    )
    lower_trim = _box((0.094, 0.080, 0.066), (0.052, 0.0, -0.026))
    jaw_arc = jaw_ring.cut(free_side_trim).cut(lower_trim)

    hinge_barrel = _cyl_y(0.007, 0.020, (0.0, 0.0, 0.0))
    hinge_cheek = _box((0.018, 0.020, 0.022), (0.010, 0.0, 0.030))
    bridge = _box((0.028, 0.020, 0.016), (0.018, 0.0, 0.018))
    tip = _box((0.016, 0.020, 0.018), (0.082, 0.0, 0.002))

    return _largest_solid(jaw_arc.union(hinge_barrel).union(hinge_cheek).union(bridge).union(tip))


def _make_stand() -> cq.Workplane:
    leg = _box((0.056, 0.008, 0.128), (0.0, -0.004, 0.064))
    upper_pad = _box((0.068, 0.014, 0.012), (0.0, -0.008, 0.123))
    hinge_barrel = _cyl_x(0.0055, 0.046, (0.0, -0.004, 0.0))
    root_web = _box((0.018, 0.008, 0.018), (0.0, -0.004, 0.014))
    return _largest_solid(leg.union(upper_pad).union(hinge_barrel).union(root_web))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_clamp_meter")

    shell_orange = model.material("shell_orange", rgba=(0.922, 0.467, 0.102, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.132, 0.136, 0.145, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.200, 0.205, 0.215, 1.0))
    button_gray = model.material("button_gray", rgba=(0.350, 0.360, 0.375, 1.0))
    display_glass = model.material("display_glass", rgba=(0.160, 0.260, 0.300, 0.45))
    label_gray = model.material("label_gray", rgba=(0.700, 0.705, 0.695, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "clamp_meter_body_shell"),
        material=shell_orange,
        name="body_shell",
    )
    body.visual(
        Box((0.086, 0.012, 0.116)),
        origin=Origin(xyz=(0.0, 0.010, 0.086)),
        material=rubber_black,
        name="front_overmold",
    )
    body.visual(
        Box((0.038, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.030, 0.150)),
        material=display_glass,
        name="display_glass",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.031, 0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="selector_boss",
    )
    body.visual(
        Box((0.046, 0.0015, 0.014)),
        origin=Origin(xyz=(0.0, 0.0265, 0.072)),
        material=label_gray,
        name="button_legend",
    )
    body.visual(
        Box((0.012, 0.042, 0.120)),
        origin=Origin(xyz=(-0.047, 0.0, 0.094)),
        material=rubber_black,
        name="left_bumper",
    )
    body.visual(
        Box((0.012, 0.042, 0.120)),
        origin=Origin(xyz=(0.047, 0.0, 0.094)),
        material=rubber_black,
        name="right_bumper",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_make_jaw(), "clamp_meter_jaw"),
        material=dark_plastic,
        name="jaw_shell",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.062,
                0.022,
                body_style="skirted",
                top_diameter=0.050,
                skirt=KnobSkirt(0.072, 0.005, flare=0.04),
                grip=KnobGrip(style="fluted", count=18, depth=0.0013),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "clamp_meter_selector_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="selector_knob",
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand(), "clamp_meter_stand"),
        material=dark_plastic,
        name="stand_leg",
    )

    side_button = model.part("side_button")
    side_button.visual(
        Box((0.002, 0.012, 0.008)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=button_gray,
        name="side_button_guide",
    )
    side_button.visual(
        Box((0.005, 0.018, 0.012)),
        origin=Origin(xyz=(0.0055, 0.0, 0.0)),
        material=button_gray,
        name="side_button_cap",
    )

    for index, x_pos in enumerate((-0.020, 0.020)):
        button = model.part(f"front_button_{index}")
        button.visual(
            Cylinder(radius=0.0065, length=0.002),
            origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_gray,
            name="button_guide",
        )
        button.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_gray,
            name="button_cap",
        )
        model.articulation(
            f"body_to_front_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.024, 0.057)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.002,
            ),
        )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(JAW_PIVOT_X, 0.0, JAW_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=0.0,
            upper=1.70,
        ),
    )

    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.036, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.038, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(0.050, 0.0, 0.128)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.05,
            lower=0.0,
            upper=0.002,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    knob = object_model.get_part("knob")
    stand = object_model.get_part("stand")
    side_button = object_model.get_part("side_button")
    front_button_0 = object_model.get_part("front_button_0")
    front_button_1 = object_model.get_part("front_button_1")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    knob_joint = object_model.get_articulation("body_to_knob")
    stand_joint = object_model.get_articulation("body_to_stand")
    side_button_joint = object_model.get_articulation("body_to_side_button")
    front_button_joint_0 = object_model.get_articulation("body_to_front_button_0")
    front_button_joint_1 = object_model.get_articulation("body_to_front_button_1")

    ctx.expect_overlap(
        knob,
        body,
        axes="xz",
        min_overlap=0.045,
        name="selector knob sits centered on the meter face",
    )
    ctx.expect_overlap(
        front_button_0,
        body,
        axes="xz",
        min_overlap=0.012,
        name="upper front button footprint stays on the front shell",
    )
    ctx.expect_overlap(
        front_button_1,
        body,
        axes="xz",
        min_overlap=0.012,
        name="lower front button footprint stays on the front shell",
    )
    ctx.expect_overlap(
        side_button,
        body,
        axes="yz",
        min_overlap=0.010,
        name="side flashlight button remains supported by the side wall",
    )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        details=f"joint_type={knob_joint.articulation_type}, limits={knob_limits}",
    )

    jaw_limits = jaw_joint.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        jaw_rest = ctx.part_element_world_aabb(jaw, elem="jaw_shell")
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            jaw_open = ctx.part_element_world_aabb(jaw, elem="jaw_shell")
        ctx.check(
            "jaw swings upward from the head",
            jaw_rest is not None
            and jaw_open is not None
            and jaw_open[1][2] > jaw_rest[1][2] + 0.020
            and jaw_open[0][0] < jaw_rest[0][0] - 0.010,
            details=f"rest={jaw_rest}, open={jaw_open}",
        )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        stand_rest = ctx.part_element_world_aabb(stand, elem="stand_leg")
        with ctx.pose({stand_joint: stand_limits.upper}):
            stand_open = ctx.part_element_world_aabb(stand, elem="stand_leg")
        ctx.check(
            "rear stand swings behind the body",
            stand_rest is not None
            and stand_open is not None
            and stand_open[0][1] < stand_rest[0][1] - 0.035,
            details=f"rest={stand_rest}, open={stand_open}",
        )

    front_button_0_rest = ctx.part_world_position(front_button_0)
    front_button_1_rest = ctx.part_world_position(front_button_1)
    with ctx.pose({front_button_joint_0: 0.002, front_button_joint_1: 0.0}):
        front_button_0_pressed = ctx.part_world_position(front_button_0)
        front_button_1_static = ctx.part_world_position(front_button_1)
    ctx.check(
        "front button 0 depresses independently",
        front_button_0_rest is not None
        and front_button_0_pressed is not None
        and front_button_1_rest is not None
        and front_button_1_static is not None
        and front_button_0_pressed[1] < front_button_0_rest[1] - 0.0015
        and abs(front_button_1_static[1] - front_button_1_rest[1]) < 1e-6,
        details=(
            f"rest={front_button_0_rest}, pressed={front_button_0_pressed}, "
            f"other_button={front_button_1_static}, other_rest={front_button_1_rest}"
        ),
    )

    with ctx.pose({front_button_joint_0: 0.0, front_button_joint_1: 0.002}):
        front_button_1_pressed = ctx.part_world_position(front_button_1)
        front_button_0_static = ctx.part_world_position(front_button_0)
    ctx.check(
        "front button 1 depresses independently",
        front_button_1_rest is not None
        and front_button_1_pressed is not None
        and front_button_0_rest is not None
        and front_button_0_static is not None
        and front_button_1_pressed[1] < front_button_1_rest[1] - 0.0015
        and abs(front_button_0_static[1] - front_button_0_rest[1]) < 1e-6,
        details=(
            f"rest={front_button_1_rest}, pressed={front_button_1_pressed}, "
            f"other_button={front_button_0_static}, other_rest={front_button_0_rest}"
        ),
    )

    side_button_rest = ctx.part_world_position(side_button)
    with ctx.pose({side_button_joint: 0.002}):
        side_button_pressed = ctx.part_world_position(side_button)
    ctx.check(
        "side flashlight button depresses into the side wall",
        side_button_rest is not None
        and side_button_pressed is not None
        and side_button_pressed[0] < side_button_rest[0] - 0.0015,
        details=f"rest={side_button_rest}, pressed={side_button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.110
BODY_DEPTH = 0.048
BODY_HEIGHT = 0.132
HEAD_WIDTH = 0.094
HEAD_DEPTH = 0.060
HEAD_HEIGHT = 0.060
JAW_CENTER_Z = 0.184
JAW_OUTER_RADIUS = 0.043
JAW_INNER_RADIUS = 0.027
HINGE_X = 0.041


def _body_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.010)
    )
    head = (
        cq.Workplane("XY")
        .box(HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT)
        .translate((0.0, 0.0, 0.145))
        .edges("|Z")
        .fillet(0.008)
    )
    ring = (
        cq.Workplane("XZ")
        .circle(JAW_OUTER_RADIUS)
        .circle(JAW_INNER_RADIUS)
        .extrude(0.017, both=True)
        .translate((0.0, 0.0, JAW_CENTER_Z))
    )
    fixed_cut = (
        cq.Workplane("XY")
        .box(0.140, 0.120, 0.070)
        .translate((0.0, 0.0, JAW_CENTER_Z + 0.027))
    )
    fixed_jaw = ring.cut(fixed_cut)
    return lower.union(head).union(fixed_jaw)


def _jaw_shape() -> cq.Workplane:
    upper_arc = (
        cq.Workplane("XZ")
        .circle(JAW_OUTER_RADIUS - 0.0010)
        .circle(JAW_INNER_RADIUS + 0.0010)
        .extrude(0.014, both=True)
        .translate((0.0, 0.0, JAW_CENTER_Z))
    )
    lower_cut = (
        cq.Workplane("XY")
        .box(0.140, 0.120, 0.070)
        .translate((0.0, 0.0, JAW_CENTER_Z - 0.036))
    )
    hinge_cap = (
        cq.Workplane("XZ")
        .circle(0.008)
        .extrude(0.010)
        .translate((HINGE_X, 0.014, JAW_CENTER_Z))
    )
    jaw = upper_arc.cut(lower_cut).union(hinge_cap)
    return jaw.translate((-HINGE_X, 0.0, -JAW_CENTER_Z))


def _selector_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.018,
            body_style="skirted",
            top_diameter=0.040,
            base_diameter=0.046,
            center=False,
        ),
        "selector_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_clamp_meter")

    shell = model.material("shell", rgba=(0.18, 0.19, 0.20, 1.0))
    panel = model.material("panel", rgba=(0.08, 0.09, 0.10, 1.0))
    jaw_mat = model.material("jaw", rgba=(0.13, 0.14, 0.15, 1.0))
    accent = model.material("accent", rgba=(0.94, 0.58, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.22, 0.43, 0.47, 0.45))
    button_mat = model.material("button", rgba=(0.30, 0.31, 0.33, 1.0))
    selector_mat = model.material("selector", rgba=(0.16, 0.17, 0.18, 1.0))
    stand_mat = model.material("stand", rgba=(0.15, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "clamp_meter_body"),
        material=shell,
        name="body_shell",
    )
    body.visual(
        Box((0.082, 0.003, 0.118)),
        origin=Origin(xyz=(0.0, 0.0265, 0.094)),
        material=panel,
        name="front_panel",
    )
    body.visual(
        Box((0.072, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.0280, 0.133)),
        material=panel,
        name="display_bezel",
    )
    body.visual(
        Box((0.058, 0.0015, 0.038)),
        origin=Origin(xyz=(0.0, 0.03075, 0.133)),
        material=glass,
        name="display",
    )
    body.visual(
        Box((0.080, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, 0.0275, 0.094)),
        material=panel,
        name="button_bank",
    )
    body.visual(
        Box((0.010, 0.020, 0.070)),
        origin=Origin(xyz=(-0.052, 0.010, 0.060)),
        material=accent,
        name="grip_0",
    )
    body.visual(
        Box((0.010, 0.020, 0.070)),
        origin=Origin(xyz=(0.052, 0.010, 0.060)),
        material=accent,
        name="grip_1",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.0, 0.029, 0.081), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="selector_mark",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "clamp_meter_jaw"),
        material=jaw_mat,
        name="jaw_arc",
    )

    selector = model.part("selector")
    selector.visual(
        _selector_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=selector_mat,
        name="selector",
    )
    selector.visual(
        Box((0.006, 0.002, 0.015)),
        origin=Origin(xyz=(0.0, 0.019, 0.015)),
        material=accent,
        name="pointer",
    )

    button_centers = (-0.024, 0.0, 0.024)
    for index, x_pos in enumerate(button_centers):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.009, 0.010)),
            origin=Origin(xyz=(0.0, 0.0045, 0.0)),
            material=button_mat,
            name="cap",
        )
        button.visual(
            Box((0.010, 0.003, 0.006)),
            origin=Origin(xyz=(0.0, 0.0015, 0.0)),
            material=button_mat,
            name="stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.029, 0.094)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        Box((0.012, 0.004, 0.090)),
        origin=Origin(xyz=(-0.018, -0.002, 0.045)),
        material=stand_mat,
        name="rail_0",
    )
    stand.visual(
        Box((0.012, 0.004, 0.090)),
        origin=Origin(xyz=(0.018, -0.002, 0.045)),
        material=stand_mat,
        name="rail_1",
    )
    stand.visual(
        Box((0.050, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, 0.088)),
        material=stand_mat,
        name="bridge",
    )
    stand.visual(
        Box((0.040, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.004)),
        material=stand_mat,
        name="foot"),

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(HINGE_X, 0.0, JAW_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, 0.0265, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.024, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]

    jaw_joint = object_model.get_articulation("body_to_jaw")
    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    ctx.check(
        "selector rotates continuously",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.motion_limits is not None
        and selector_joint.motion_limits.lower is None
        and selector_joint.motion_limits.upper is None,
        details=str(selector_joint.motion_limits),
    )

    ctx.expect_gap(
        jaw,
        body,
        axis="z",
        positive_elem="jaw_arc",
        negative_elem="display",
        min_gap=0.020,
        name="jaw sits above the display",
    )

    for index, button in enumerate(buttons):
        ctx.expect_gap(
            body,
            button,
            axis="z",
            positive_elem="display",
            negative_elem="cap",
            min_gap=0.010,
            name=f"display stays above button_{index}",
        )
        ctx.expect_gap(
            button,
            selector,
            axis="z",
            positive_elem="cap",
            negative_elem="selector",
            min_gap=0.008,
            name=f"button_{index} stays above the selector",
        )

    jaw_rest_aabb = ctx.part_world_aabb(jaw)
    jaw_upper = jaw_joint.motion_limits.upper if jaw_joint.motion_limits is not None else None
    if jaw_upper is not None:
        with ctx.pose({jaw_joint: jaw_upper}):
            jaw_open_aabb = ctx.part_world_aabb(jaw)
        ctx.check(
            "jaw opens away from the body",
            jaw_rest_aabb is not None
            and jaw_open_aabb is not None
            and jaw_open_aabb[1][0] > jaw_rest_aabb[1][0] + 0.010,
            details=f"rest={jaw_rest_aabb}, open={jaw_open_aabb}",
        )

    stand_rest_aabb = ctx.part_world_aabb(stand)
    stand_upper = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else None
    if stand_upper is not None:
        with ctx.pose({stand_joint: stand_upper}):
            stand_open_aabb = ctx.part_world_aabb(stand)
        ctx.check(
            "stand swings rearward",
            stand_rest_aabb is not None
            and stand_open_aabb is not None
            and stand_open_aabb[0][1] < stand_rest_aabb[0][1] - 0.018,
            details=f"rest={stand_rest_aabb}, open={stand_open_aabb}",
        )

    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()

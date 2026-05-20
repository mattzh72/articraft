from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.295
BODY_WIDTH = 0.345
BODY_HEIGHT = 0.155
BODY_FLOOR = 0.022
BODY_FRONT_X = BODY_DEPTH * 0.5
BODY_BACK_X = -BODY_DEPTH * 0.5

LID_DEPTH = 0.270
LID_WIDTH = 0.320
LID_HEIGHT = 0.036


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.040)
        .faces(">Z")
        .edges()
        .fillet(0.018)
    )

    cavity = (
        cq.Workplane("XY")
        .box(0.225, 0.265, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_FLOOR + BODY_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.026)
    )

    return outer.cut(cavity)


def _lid_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, LID_HEIGHT)
        .translate((LID_DEPTH * 0.5, 0.0, LID_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.026)
        .faces(">Z")
        .edges()
        .fillet(0.012)
    )

    inner = (
        cq.Workplane("XY")
        .box(LID_DEPTH - 0.028, LID_WIDTH - 0.036, LID_HEIGHT - 0.006)
        .translate((LID_DEPTH * 0.5, 0.0, (LID_HEIGHT - 0.006) * 0.5))
        .edges("|Z")
        .fillet(0.020)
    )

    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.77, 0.79, 0.81, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    soft_black = model.material("soft_black", rgba=(0.11, 0.11, 0.12, 1.0))
    lens_smoke = model.material("lens_smoke", rgba=(0.24, 0.28, 0.30, 0.72))
    amber = model.material("amber", rgba=(0.83, 0.58, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "rice_cooker_body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((0.012, 0.250, 0.020)),
        origin=Origin(xyz=(BODY_BACK_X + 0.004, 0.0, BODY_HEIGHT - 0.014)),
        material=trim_gray,
        name="hinge_band",
    )
    body.visual(
        Box((0.010, 0.094, 0.006)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.005, 0.0, 0.122)),
        material=trim_gray,
        name="latch_frame_top",
    )
    body.visual(
        Box((0.010, 0.094, 0.006)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.005, 0.0, 0.098)),
        material=trim_gray,
        name="latch_frame_bottom",
    )
    body.visual(
        Box((0.010, 0.008, 0.030)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.005, -0.043, 0.110)),
        material=trim_gray,
        name="latch_frame_left",
    )
    body.visual(
        Box((0.010, 0.008, 0.030)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.005, 0.043, 0.110)),
        material=trim_gray,
        name="latch_frame_right",
    )

    foot_positions = (
        (-0.090, -0.110),
        (-0.090, 0.110),
        (0.090, -0.110),
        (0.090, 0.110),
    )
    for index, (x_pos, y_pos) in enumerate(foot_positions):
        body.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.006)),
            material=rubber,
            name=f"foot_{index}",
        )

    control_cluster = model.part("control_cluster")
    control_cluster.visual(
        Box((0.014, 0.120, 0.056)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=charcoal,
        name="cluster_fascia",
    )
    control_cluster.visual(
        Box((0.004, 0.066, 0.012)),
        origin=Origin(xyz=(0.011, 0.0, 0.017)),
        material=lens_smoke,
        name="status_window",
    )
    control_cluster.visual(
        Cylinder(radius=0.038, length=0.005),
        origin=Origin(
            xyz=(0.010, 0.0, -0.013),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_gray,
        name="dial_seat",
    )
    control_cluster.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(
            xyz=(0.012, -0.022, 0.017),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=amber,
        name="cook_lamp",
    )
    control_cluster.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(
            xyz=(0.012, 0.022, 0.017),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=amber,
        name="warm_lamp",
    )
    model.articulation(
        "body_to_control_cluster",
        ArticulationType.FIXED,
        parent=body,
        child=control_cluster,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, 0.048)),
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.012, 0.078, 0.022)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=soft_black,
        name="latch_button",
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch,
        origin=Origin(xyz=(BODY_FRONT_X + 0.010, 0.0, 0.110)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.10,
            lower=0.0,
            upper=0.004,
        ),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "rice_cooker_lid_shell"),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.060, 0.094, 0.018)),
        origin=Origin(xyz=(0.208, 0.0, 0.034)),
        material=charcoal,
        name="front_handle",
    )
    lid.visual(
        Box((0.028, 0.100, 0.010)),
        origin=Origin(xyz=(0.244, 0.0, 0.010)),
        material=trim_gray,
        name="front_lip",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(BODY_BACK_X + 0.012, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=0.0,
            upper=1.30,
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.054,
                0.020,
                body_style="skirted",
                top_diameter=0.044,
                skirt=KnobSkirt(0.070, 0.006, flare=0.06),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "rice_cooker_selector_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="dial_knob",
    )
    selector_dial.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(
            xyz=(0.003, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_gray,
        name="dial_hub",
    )
    model.articulation(
        "control_cluster_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=control_cluster,
        child=selector_dial,
        origin=Origin(xyz=(0.012, 0.0, -0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_cluster = object_model.get_part("control_cluster")
    latch = object_model.get_part("latch")
    lid = object_model.get_part("lid")
    selector_dial = object_model.get_part("selector_dial")

    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("body_to_latch")
    dial_joint = object_model.get_articulation("control_cluster_to_selector_dial")

    ctx.expect_overlap(
        control_cluster,
        body,
        axes="y",
        min_overlap=0.10,
        name="control cluster stays centered on the front shell",
    )
    ctx.expect_origin_gap(
        latch,
        selector_dial,
        axis="z",
        min_gap=0.055,
        name="selector dial sits below the latch",
    )
    ctx.expect_origin_distance(
        latch,
        selector_dial,
        axes="y",
        max_dist=0.010,
        name="latch and selector dial share the front centerline",
    )

    ctx.check(
        "selector dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=(
            f"type={dial_joint.articulation_type!r}, "
            f"limits={dial_joint.motion_limits!r}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.220,
            name="closed lid broadly covers the cooker body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.001,
            name="closed lid seats close to the shell opening",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
    with ctx.pose({lid_hinge: 1.20}):
        open_handle_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")

    handle_lifts = (
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][2] > closed_handle_aabb[0][2] + 0.10
    )
    ctx.check(
        "lid handle rises in the open pose",
        handle_lifts,
        details=f"closed={closed_handle_aabb!r}, open={open_handle_aabb!r}",
    )

    latch_rest = ctx.part_world_position(latch)
    with ctx.pose({latch_slide: 0.004}):
        latch_pressed = ctx.part_world_position(latch)
    ctx.check(
        "latch button presses inward",
        latch_rest is not None
        and latch_pressed is not None
        and latch_pressed[0] < latch_rest[0] - 0.003,
        details=f"rest={latch_rest!r}, pressed={latch_pressed!r}",
    )

    return ctx.report()


object_model = build_object_model()

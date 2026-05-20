from __future__ import annotations

import math

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

BODY_HEIGHT = 0.315
UPPER_HEIGHT = 0.245
TOTAL_HEIGHT = BODY_HEIGHT + UPPER_HEIGHT - 0.005
BODY_RADIUS = 0.108
UPPER_RADIUS = 0.114
WALL_THICKNESS = 0.004
FILTER_TRAVEL = 0.165
TWIST_TRAVEL = math.radians(38.0)
BUTTON_TRAVEL = 0.0035


def _lower_body_shape():
    shell = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT)
    shell = shell.faces(">Z").shell(-WALL_THICKNESS)

    foot_ring = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS + 0.006)
        .circle(BODY_RADIUS - 0.014)
        .extrude(0.018)
    )
    guide_collar = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS - WALL_THICKNESS)
        .circle(BODY_RADIUS - 0.014)
        .extrude(0.024)
        .translate((0.0, 0.0, BODY_HEIGHT - 0.024))
    )
    return shell.union(foot_ring).union(guide_collar)


def _upper_shell_shape():
    shell = cq.Workplane("XY").circle(UPPER_RADIUS).extrude(UPPER_HEIGHT)
    shell = shell.faces("<Z").shell(-WALL_THICKNESS)
    shell = shell.faces(">Z").workplane().circle(0.046).cutBlind(-0.010)

    seat_ring = (
        cq.Workplane("XY")
        .circle(UPPER_RADIUS - WALL_THICKNESS)
        .circle(BODY_RADIUS - WALL_THICKNESS)
        .extrude(0.008)
    )
    bar_a = cq.Workplane("XY").box(0.098, 0.008, 0.003).translate(
        (0.0, 0.0, UPPER_HEIGHT - 0.0015)
    )
    bar_b = cq.Workplane("XY").box(0.008, 0.098, 0.003).translate(
        (0.0, 0.0, UPPER_HEIGHT - 0.0015)
    )
    return shell.union(seat_ring).union(bar_a).union(bar_b)


def _filter_shape():
    media_outer_radius = 0.083
    media_inner_radius = 0.031
    media_height = 0.220
    top_cap_height = 0.010
    bottom_cap_height = 0.015
    cap_radius = 0.086

    top_cap = (
        cq.Workplane("XY")
        .circle(cap_radius)
        .circle(media_inner_radius)
        .extrude(top_cap_height)
        .translate((0.0, 0.0, -top_cap_height))
    )
    media = (
        cq.Workplane("XY")
        .circle(media_outer_radius)
        .circle(media_inner_radius)
        .extrude(media_height)
        .translate((0.0, 0.0, -(top_cap_height + media_height)))
    )
    bottom_cap = (
        cq.Workplane("XY")
        .circle(cap_radius)
        .circle(media_inner_radius)
        .extrude(bottom_cap_height)
        .translate((0.0, 0.0, -(top_cap_height + media_height + bottom_cap_height)))
    )
    guide_ring = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS - 0.014)
        .circle(cap_radius - 0.002)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.013))
    )

    handle_spine = cq.Workplane("XY").box(0.074, 0.010, 0.008)
    handle_bridge = cq.Workplane("XY").box(0.060, 0.014, 0.008).translate((0.0, 0.0, 0.022))
    handle_post_a = cq.Workplane("XY").box(0.010, 0.010, 0.024).translate((0.020, 0.0, 0.010))
    handle_post_b = cq.Workplane("XY").box(0.010, 0.010, 0.024).translate((-0.020, 0.0, 0.010))

    filter_shape = (
        top_cap.union(media)
        .union(bottom_cap)
        .union(guide_ring)
        .union(handle_spine)
        .union(handle_bridge)
        .union(handle_post_a)
        .union(handle_post_b)
    )

    rib_z = -(top_cap_height + media_height * 0.5)
    for rib_index in range(18):
        angle_deg = 360.0 * rib_index / 18.0
        rib = (
            cq.Workplane("XY")
            .box(0.006, 0.014, media_height * 0.94)
            .translate((media_outer_radius, 0.0, rib_z))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        filter_shape = filter_shape.union(rib)

    return filter_shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_purifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    shell_grey = model.material("shell_grey", rgba=(0.83, 0.84, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    filter_cream = model.material("filter_cream", rgba=(0.95, 0.94, 0.88, 1.0))
    button_black = model.material("button_black", rgba=(0.09, 0.09, 0.10, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.35, 0.57, 0.72, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(_lower_body_shape(), "purifier_lower_body"),
        material=shell_grey,
        name="body_shell",
    )
    lower_body.visual(
        Box((0.020, 0.054, 0.074)),
        origin=Origin(xyz=(BODY_RADIUS + 0.004, 0.0, 0.155)),
        material=shell_white,
        name="button_pad",
    )

    upper_shell = model.part("upper_shell")
    upper_shell.visual(
        mesh_from_cadquery(_upper_shell_shape(), "purifier_upper_shell"),
        material=shell_white,
        name="shell_body",
    )
    upper_shell.visual(
        Box((0.006, 0.028, 0.050)),
        origin=Origin(xyz=(UPPER_RADIUS + 0.0015, 0.0, 0.145)),
        material=accent_blue,
        name="shell_badge",
    )

    filter_cylinder = model.part("filter_cylinder")
    filter_cylinder.visual(
        mesh_from_cadquery(_filter_shape(), "purifier_filter_cylinder"),
        material=filter_cream,
        name="filter_body",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button_black,
        name="button_cap",
    )

    model.articulation(
        "lower_body_to_upper_shell",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_shell,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=TWIST_TRAVEL,
        ),
    )

    model.articulation(
        "lower_body_to_filter_cylinder",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=filter_cylinder,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.017)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.12,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    model.articulation(
        "lower_body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=power_button,
        origin=Origin(
            xyz=(BODY_RADIUS + 0.014, 0.0, 0.155),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.04,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_body = object_model.get_part("lower_body")
    upper_shell = object_model.get_part("upper_shell")
    filter_cylinder = object_model.get_part("filter_cylinder")
    power_button = object_model.get_part("power_button")

    twist_joint = object_model.get_articulation("lower_body_to_upper_shell")
    filter_joint = object_model.get_articulation("lower_body_to_filter_cylinder")
    button_joint = object_model.get_articulation("lower_body_to_power_button")

    ctx.allow_overlap(
        lower_body,
        upper_shell,
        elem_a="body_shell",
        elem_b="shell_body",
        reason="The twist-lock upper shell is intentionally modeled as a nested skirt over the lower housing shell.",
    )
    ctx.allow_overlap(
        lower_body,
        filter_cylinder,
        elem_a="body_shell",
        elem_b="filter_body",
        reason="The removable filter cartridge is intentionally represented as nested inside the cylindrical housing shell.",
    )

    ctx.expect_within(
        filter_cylinder,
        lower_body,
        axes="xy",
        margin=0.0,
        name="filter stays centered in the purifier body",
    )
    ctx.expect_within(
        lower_body,
        upper_shell,
        axes="xy",
        margin=0.010,
        name="upper shell sits concentrically over the lower body",
    )

    filter_rest = ctx.part_world_position(filter_cylinder)
    with ctx.pose({filter_joint: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_cylinder,
            lower_body,
            axes="xy",
            margin=0.0,
            name="extended filter stays aligned with the lower body",
        )
        ctx.expect_overlap(
            filter_cylinder,
            lower_body,
            axes="z",
            min_overlap=0.095,
            name="extended filter retains insertion in the purifier body",
        )
        filter_extended = ctx.part_world_position(filter_cylinder)

    ctx.check(
        "filter cylinder pulls upward",
        filter_rest is not None
        and filter_extended is not None
        and filter_extended[2] > filter_rest[2] + 0.12,
        details=f"rest={filter_rest}, extended={filter_extended}",
    )

    button_rest = ctx.part_world_position(power_button)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        button_pressed = ctx.part_world_position(power_button)

    ctx.check(
        "power button depresses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] < button_rest[0] - 0.002,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    badge_rest = _aabb_center(ctx.part_element_world_aabb(upper_shell, elem="shell_badge"))
    with ctx.pose({twist_joint: TWIST_TRAVEL}):
        badge_twisted = _aabb_center(ctx.part_element_world_aabb(upper_shell, elem="shell_badge"))

    ctx.check(
        "upper shell visibly twists around the vertical lock",
        badge_rest is not None
        and badge_twisted is not None
        and badge_twisted[1] > badge_rest[1] + 0.05,
        details=f"rest={badge_rest}, twisted={badge_twisted}",
    )

    return ctx.report()


object_model = build_object_model()

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

LEG_COUNT = 3
LEG_SPLAY = math.radians(24.0)
LEG_TOP_RADIUS = 0.102
LEG_TOP_Z = 1.05
OUTER_LEG_LENGTH = 0.76
INNER_LEG_LENGTH = 0.58
INNER_LEG_INSERTION = 0.18
LEG_TRAVEL = 0.14
CENTER_SLEEVE_HEIGHT = 0.30
CENTER_SLEEVE_TOP = 1.34
CENTER_COLUMN_LENGTH = 0.56
CENTER_COLUMN_INSERTION = 0.19
CENTER_COLUMN_TRAVEL = 0.18


def _hollow_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(length)
        .cut(
            cq.Workplane("XY")
            .circle(inner_radius)
            .extrude(length + 0.002)
            .translate((0.0, 0.0, -0.001))
        )
    )


def _outer_leg_shape() -> cq.Workplane:
    sleeve = _hollow_tube(0.021, 0.017, OUTER_LEG_LENGTH).translate((0.0, 0.0, -OUTER_LEG_LENGTH))
    lower_collar = (
        _hollow_tube(0.024, 0.0155, 0.12)
        .translate((0.0, 0.0, -OUTER_LEG_LENGTH))
    )
    hinge_block = (
        cq.Workplane("XY")
        .box(0.042, 0.028, 0.050, centered=(True, True, False))
        .translate((0.0, 0.0, -0.050))
    )
    return hinge_block.union(sleeve).union(lower_collar)


def _center_sleeve_shape() -> cq.Workplane:
    sleeve = _hollow_tube(0.030, 0.023, CENTER_SLEEVE_HEIGHT)
    knuckle = (
        cq.Workplane("XY")
        .circle(0.038)
        .extrude(0.040)
        .translate((0.0, 0.0, -0.040))
    )
    return knuckle.union(sleeve)


def _inner_leg_body_shape() -> cq.Workplane:
    stage = cq.Workplane("XY").circle(0.014).extrude(INNER_LEG_LENGTH).translate((0.0, 0.0, -0.400))
    foot = cq.Workplane("XY").circle(0.022).extrude(0.060).translate((0.0, 0.0, -0.412))
    return stage.union(foot)


def _leg_mount(angle: float) -> Origin:
    return Origin(
        xyz=(
            LEG_TOP_RADIUS * math.cos(angle),
            LEG_TOP_RADIUS * math.sin(angle),
            LEG_TOP_Z,
        ),
        rpy=(0.0, -LEG_SPLAY, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spotting_scope_tripod")

    anodized_black = model.material("anodized_black", rgba=(0.13, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.25, 0.27, 0.29, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.68, 0.70, 0.72, 1.0))
    olive = model.material("olive", rgba=(0.29, 0.34, 0.22, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.24, 0.28, 0.65))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.080, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.090)),
        material=anodized_black,
        name="hub",
    )
    base.visual(
        mesh_from_cadquery(_center_sleeve_shape(), "center_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, CENTER_SLEEVE_TOP - CENTER_SLEEVE_HEIGHT)),
        material=graphite,
        name="column_sleeve",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        base.visual(
            Box((0.072, 0.038, 0.060)),
            origin=Origin(
                xyz=(
                    0.082 * math.cos(angle),
                    0.082 * math.sin(angle),
                    1.080,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=anodized_black,
            name=f"leg_socket_{index}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(
                xyz=(
                    0.050 * math.cos(angle),
                    0.050 * math.sin(angle),
                    1.020,
                ),
                rpy=(math.pi / 2.0, 0.0, angle),
            ),
            material=graphite,
            name=f"brace_{index}",
        )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        outer_leg = model.part(f"outer_leg_{index}")
        outer_leg.visual(
            mesh_from_cadquery(_outer_leg_shape(), f"outer_leg_{index}"),
            material=graphite,
            name="sleeve",
        )

        inner_leg = model.part(f"inner_leg_{index}")
        inner_leg.visual(
            mesh_from_cadquery(_inner_leg_body_shape(), f"inner_leg_body_{index}"),
            material=graphite,
            name="stage_body",
        )
        inner_leg.visual(
            Cylinder(radius=0.0170, length=0.180),
            origin=Origin(xyz=(0.0, 0.0, 0.230)),
            material=satin_silver,
            name="guide_section",
        )

        model.articulation(
            f"leg_mount_{index}",
            ArticulationType.FIXED,
            parent=base,
            child=outer_leg,
            origin=_leg_mount(angle),
        )
        model.articulation(
            f"leg_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=outer_leg,
            child=inner_leg,
            origin=Origin(xyz=(0.0, 0.0, -OUTER_LEG_LENGTH)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.12,
                lower=0.0,
                upper=LEG_TRAVEL,
            ),
        )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=0.020, length=CENTER_COLUMN_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, (CENTER_COLUMN_LENGTH / 2.0) - CENTER_COLUMN_INSERTION),
        ),
        material=satin_silver,
        name="column_tube",
    )
    center_column.visual(
        Cylinder(radius=0.023, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=satin_silver,
        name="guide_band",
    )
    center_column.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=anodized_black,
        name="top_cap",
    )
    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, CENTER_SLEEVE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.10,
            lower=0.0,
            upper=CENTER_COLUMN_TRAVEL,
        ),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.042, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=anodized_black,
        name="pan_drum",
    )
    pan_head.visual(
        Box((0.095, 0.090, 0.040)),
        origin=Origin(xyz=(0.010, 0.0, 0.080)),
        material=graphite,
        name="head_block",
    )
    pan_head.visual(
        Box((0.030, 0.012, 0.090)),
        origin=Origin(xyz=(0.000, 0.043, 0.125)),
        material=graphite,
        name="yoke_arm_0",
    )
    pan_head.visual(
        Box((0.030, 0.012, 0.090)),
        origin=Origin(xyz=(0.000, -0.043, 0.125)),
        material=graphite,
        name="yoke_arm_1",
    )
    model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=center_column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )

    tilt_plate = model.part("tilt_plate")
    tilt_plate.visual(
        Cylinder(radius=0.017, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="trunnion",
    )
    tilt_plate.visual(
        Box((0.100, 0.060, 0.030)),
        origin=Origin(xyz=(0.050, 0.0, 0.015)),
        material=graphite,
        name="receiver",
    )
    tilt_plate.visual(
        Box((0.168, 0.062, 0.008)),
        origin=Origin(xyz=(0.086, 0.0, 0.034)),
        material=satin_silver,
        name="plate_deck",
    )
    tilt_plate.visual(
        Box((0.130, 0.010, 0.014)),
        origin=Origin(xyz=(0.086, 0.024, 0.021)),
        material=satin_silver,
        name="rail_0",
    )
    tilt_plate.visual(
        Box((0.130, 0.010, 0.014)),
        origin=Origin(xyz=(0.086, -0.024, 0.021)),
        material=satin_silver,
        name="rail_1",
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.55,
            upper=1.15,
        ),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="pivot",
    )
    latch.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.006)),
        material=anodized_black,
        name="grip",
    )
    latch.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.041, 0.0, 0.006)),
        material=rubber,
        name="tip",
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=tilt_plate,
        child=latch,
        origin=Origin(xyz=(0.020, -0.036, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=0.95,
        ),
    )

    scope = model.part("scope")
    scope.visual(
        Box((0.090, 0.032, 0.010)),
        origin=Origin(xyz=(0.040, 0.0, 0.005)),
        material=anodized_black,
        name="mount_foot",
    )
    scope.visual(
        Box((0.030, 0.024, 0.036)),
        origin=Origin(xyz=(0.040, 0.0, 0.028)),
        material=anodized_black,
        name="mount_post",
    )
    scope.visual(
        Cylinder(radius=0.045, length=0.330),
        origin=Origin(xyz=(0.185, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="body",
    )
    scope.visual(
        Cylinder(radius=0.049, length=0.036),
        origin=Origin(xyz=(0.125, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="focus_ring",
    )
    scope.visual(
        Cylinder(radius=0.056, length=0.110),
        origin=Origin(xyz=(0.375, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="objective_hood",
    )
    scope.visual(
        Cylinder(radius=0.020, length=0.070),
        origin=Origin(xyz=(0.000, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="eyepiece",
    )
    scope.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(-0.050, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyecup",
    )
    scope.visual(
        Cylinder(radius=0.048, length=0.006),
        origin=Origin(xyz=(0.430, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    model.articulation(
        "plate_to_scope",
        ArticulationType.FIXED,
        parent=tilt_plate,
        child=scope,
        origin=Origin(xyz=(0.050, 0.0, 0.038)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    leg_outer = object_model.get_part("outer_leg_0")
    leg_inner = object_model.get_part("inner_leg_0")
    leg_slide = object_model.get_articulation("leg_slide_0")
    base = object_model.get_part("base")
    center_column = object_model.get_part("center_column")
    pan_head = object_model.get_part("pan_head")
    scope = object_model.get_part("scope")
    column_slide = object_model.get_articulation("column_slide")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.allow_overlap(
        "base",
        "center_column",
        elem_a="column_sleeve",
        elem_b="guide_band",
        reason="The center-column guide band is intentionally represented as sliding inside the sleeve proxy.",
    )
    for index in range(LEG_COUNT):
        ctx.allow_overlap(
            f"outer_leg_{index}",
            f"inner_leg_{index}",
            elem_a="sleeve",
            elem_b="guide_section",
            reason="The leg guide section is intentionally represented as sliding inside the outer sleeve proxy.",
        )
        ctx.allow_overlap(
            "base",
            f"outer_leg_{index}",
            elem_a=f"leg_socket_{index}",
            elem_b="sleeve",
            reason="The cast tripod crown intentionally nests around the upper leg shoulder at the socket mount.",
        )

    ctx.expect_within(
        leg_inner,
        leg_outer,
        axes="xy",
        inner_elem="guide_section",
        outer_elem="sleeve",
        margin=0.010,
        name="representative leg guide stays centered in the sleeve",
    )
    ctx.expect_overlap(
        leg_inner,
        leg_outer,
        axes="z",
        elem_a="guide_section",
        elem_b="sleeve",
        min_overlap=0.16,
        name="representative leg keeps hidden guide engagement at rest",
    )

    rest_leg_pos = ctx.part_world_position(leg_inner)
    with ctx.pose({leg_slide: LEG_TRAVEL}):
        ctx.expect_within(
            leg_inner,
            leg_outer,
            axes="xy",
            inner_elem="guide_section",
            outer_elem="sleeve",
            margin=0.010,
            name="extended representative leg guide stays centered in the sleeve",
        )
        ctx.expect_overlap(
            leg_inner,
            leg_outer,
            axes="z",
            elem_a="guide_section",
            elem_b="sleeve",
            min_overlap=0.08,
            name="extended representative leg keeps retained insertion",
        )
        extended_leg_pos = ctx.part_world_position(leg_inner)

    rest_leg_radius = None
    extended_leg_radius = None
    if rest_leg_pos is not None:
        rest_leg_radius = math.hypot(rest_leg_pos[0], rest_leg_pos[1])
    if extended_leg_pos is not None:
        extended_leg_radius = math.hypot(extended_leg_pos[0], extended_leg_pos[1])
    ctx.check(
        "representative leg extends downward and outward",
        rest_leg_pos is not None
        and extended_leg_pos is not None
        and extended_leg_radius is not None
        and rest_leg_radius is not None
        and extended_leg_pos[2] < rest_leg_pos[2] - 0.05
        and extended_leg_radius > rest_leg_radius + 0.04,
        details=f"rest={rest_leg_pos}, extended={extended_leg_pos}",
    )

    ctx.expect_within(
        center_column,
        base,
        axes="xy",
        inner_elem="guide_band",
        outer_elem="column_sleeve",
        margin=0.004,
        name="center column guide band stays inside the sleeve",
    )
    ctx.expect_overlap(
        center_column,
        base,
        axes="z",
        elem_a="guide_band",
        elem_b="column_sleeve",
        min_overlap=0.18,
        name="center column retains insertion at rest",
    )

    rest_head_pos = ctx.part_world_position(pan_head)
    with ctx.pose({column_slide: CENTER_COLUMN_TRAVEL}):
        ctx.expect_within(
            center_column,
            base,
            axes="xy",
            inner_elem="guide_band",
            outer_elem="column_sleeve",
            margin=0.004,
            name="extended center column guide stays inside the sleeve",
        )
        ctx.expect_overlap(
            center_column,
            base,
            axes="z",
            elem_a="guide_band",
            elem_b="column_sleeve",
            min_overlap=0.02,
            name="extended center column still retains insertion",
        )
        extended_head_pos = ctx.part_world_position(pan_head)

    ctx.check(
        "center column raises the head",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] > rest_head_pos[2] + 0.12,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    scope_rest = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_hood"))
    with ctx.pose({pan_axis: math.pi / 2.0}):
        scope_pan = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_hood"))
    ctx.check(
        "pan head swings the scope around the vertical axis",
        scope_rest is not None
        and scope_pan is not None
        and abs(scope_pan[1]) > abs(scope_rest[1]) + 0.18
        and abs(scope_pan[0]) < abs(scope_rest[0]) - 0.12,
        details=f"rest={scope_rest}, panned={scope_pan}",
    )

    with ctx.pose({tilt_axis: 0.0}):
        scope_level = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_hood"))
    with ctx.pose({tilt_axis: 0.95}):
        scope_tilted = aabb_center(ctx.part_element_world_aabb(scope, elem="objective_hood"))
    ctx.check(
        "tilt plate raises the scope nose",
        scope_level is not None
        and scope_tilted is not None
        and scope_tilted[2] > scope_level[2] + 0.10,
        details=f"level={scope_level}, tilted={scope_tilted}",
    )

    latch_closed = aabb_center(ctx.part_element_world_aabb("latch", elem="grip"))
    with ctx.pose({latch_pivot: 0.75}):
        latch_open = aabb_center(ctx.part_element_world_aabb("latch", elem="grip"))
    ctx.check(
        "locking latch rotates upward on its pivot",
        latch_closed is not None
        and latch_open is not None
        and latch_open[2] > latch_closed[2] + 0.01
        and latch_open[0] < latch_closed[0] - 0.004,
        details=f"closed={latch_closed}, open={latch_open}",
    )

    return ctx.report()


object_model = build_object_model()

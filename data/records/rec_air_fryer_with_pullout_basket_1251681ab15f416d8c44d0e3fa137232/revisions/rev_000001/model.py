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

BODY_DEPTH = 0.36
BODY_WIDTH = 0.33
BODY_HEIGHT = 0.35

DRAWER_TRAVEL = 0.18
DRAWER_ORIGIN_X = 0.182
DRAWER_ORIGIN_Z = 0.053


def _build_body_shell() -> cq.Workplane:
    outer_profile = [
        (-0.180, 0.000),
        (0.162, 0.000),
        (0.176, 0.040),
        (0.180, 0.150),
        (0.176, 0.245),
        (0.162, 0.294),
        (0.110, 0.337),
        (0.000, 0.350),
        (-0.110, 0.337),
        (-0.162, 0.294),
        (-0.176, 0.245),
        (-0.180, 0.150),
        (-0.180, 0.000),
    ]
    inner_profile = [
        (-0.160, 0.024),
        (0.152, 0.024),
        (0.156, 0.132),
        (0.153, 0.220),
        (0.142, 0.266),
        (0.096, 0.311),
        (0.000, 0.322),
        (-0.096, 0.311),
        (-0.142, 0.266),
        (-0.153, 0.220),
        (-0.156, 0.132),
        (-0.160, 0.024),
    ]

    outer = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(BODY_WIDTH)
        .translate((0.0, BODY_WIDTH / 2.0, 0.0))
    )
    inner = (
        cq.Workplane("XZ")
        .polyline(inner_profile)
        .close()
        .extrude(0.292)
        .translate((0.0, 0.146, 0.0))
    )

    drawer_aperture = (
        cq.Workplane("XY")
        .box(0.092, 0.282, 0.170, centered=(False, True, False))
        .translate((0.146, 0.0, 0.046))
    )

    return outer.cut(inner).cut(drawer_aperture)


def _build_drawer_shell() -> cq.Workplane:
    bucket = (
        cq.Workplane("XY")
        .box(0.235, 0.272, 0.145, centered=(False, True, False))
        .translate((-0.235, 0.0, 0.0))
    )
    fascia = cq.Workplane("XY").box(0.014, 0.286, 0.156, centered=(False, True, False))
    post_a = (
        cq.Workplane("XY")
        .box(0.026, 0.030, 0.064, centered=(False, True, False))
        .translate((0.014, -0.047, 0.044))
    )
    post_b = (
        cq.Workplane("XY")
        .box(0.026, 0.030, 0.064, centered=(False, True, False))
        .translate((0.014, 0.047, 0.044))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.056, 0.116, 0.024, centered=(False, True, False))
        .translate((0.040, 0.0, 0.066))
    )
    rail_a = (
        cq.Workplane("XY")
        .box(0.028, 0.006, 0.040, centered=(False, True, False))
        .translate((0.028, -0.016, 0.084))
    )
    rail_b = (
        cq.Workplane("XY")
        .box(0.028, 0.006, 0.040, centered=(False, True, False))
        .translate((0.028, 0.016, 0.084))
    )

    drawer = bucket.union(fascia).union(post_a).union(post_b).union(grip).union(rail_a).union(rail_b)

    cavity = (
        cq.Workplane("XY")
        .box(0.206, 0.248, 0.160, centered=(False, True, False))
        .translate((-0.218, 0.0, 0.010))
    )
    drawer = drawer.cut(cavity)

    latch_pocket = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.034, centered=(False, True, False))
        .translate((0.026, 0.0, 0.090))
    )
    drawer = drawer.cut(latch_pocket)

    for x_pos in (-0.184, -0.144, -0.104, -0.064):
        for y_pos in (-0.076, -0.038, 0.000, 0.038, 0.076):
            slot = (
                cq.Workplane("XY")
                .box(0.028, 0.006, 0.014, centered=(True, True, False))
                .translate((x_pos, y_pos, 0.002))
            )
            drawer = drawer.cut(slot)

    return drawer


def _timer_knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.026,
            body_style="skirted",
            top_diameter=0.037,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "timer_dial",
    )


def _temperature_knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.028,
            body_style="skirted",
            top_diameter=0.043,
            skirt=KnobSkirt(0.060, 0.006, flare=0.07),
            grip=KnobGrip(style="fluted", count=18, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "temperature_dial",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_stainless_air_fryer")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    panel_black = model.material("panel_black", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.15, 0.15, 0.16, 1.0))
    basket_dark = model.material("basket_dark", rgba=(0.17, 0.17, 0.18, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    latch_red = model.material("latch_red", rgba=(0.82, 0.14, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "air_fryer_body_shell"),
        material=stainless,
        name="housing_shell",
    )
    body.visual(
        Box((0.320, 0.310, 0.024)),
        origin=Origin(xyz=(-0.002, 0.0, 0.012)),
        material=trim_black,
        name="base_band",
    )
    body.visual(
        Box((0.010, 0.232, 0.124)),
        origin=Origin(xyz=(0.181, 0.0, 0.272)),
        material=panel_black,
        name="control_panel",
    )
    body.visual(
        Box((0.010, 0.034, 0.052)),
        origin=Origin(xyz=(0.181, 0.090, 0.258)),
        material=panel_black,
        name="switch_bezel",
    )
    body.visual(
        Box((0.200, 0.018, 0.030)),
        origin=Origin(xyz=(0.024, -0.086, 0.038)),
        material=trim_black,
        name="drawer_guide_0",
    )
    body.visual(
        Box((0.200, 0.018, 0.030)),
        origin=Origin(xyz=(0.024, 0.086, 0.038)),
        material=trim_black,
        name="drawer_guide_1",
    )
    for x_pos in (-0.115, 0.115):
        for y_pos in (-0.105, 0.105):
            body.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(x_pos, y_pos, 0.004)),
                material=trim_black,
            )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_shell(), "air_fryer_drawer_shell"),
        material=stainless,
        name="drawer_shell",
    )
    drawer.visual(
        Box((0.032, 0.108, 0.022)),
        origin=Origin(xyz=(0.066, 0.0, 0.077)),
        material=trim_black,
        name="handle_cover",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        _timer_knob_mesh(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="timer_knob",
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        _temperature_knob_mesh(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="temperature_knob",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.006, 0.018, 0.040)),
        origin=Origin(xyz=(0.003, -0.009, 0.020)),
        material=trim_black,
        name="rocker_paddle",
    )
    power_rocker.visual(
        Cylinder(radius=0.0025, length=0.040),
        origin=Origin(xyz=(0.0025, 0.0, 0.020)),
        material=trim_black,
        name="rocker_hinge",
    )

    release_latch = model.part("release_latch")
    release_latch.visual(
        Box((0.020, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=latch_red,
        name="latch_cap",
    )
    release_latch.visual(
        Box((0.010, 0.020, 0.006)),
        origin=Origin(xyz=(0.003, 0.0, 0.019)),
        material=latch_red,
        name="latch_thumb",
    )
    release_latch.visual(
        Box((0.012, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=latch_red,
        name="latch_tongue",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_ORIGIN_X, 0.0, DRAWER_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.22,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(0.186, 0.0, 0.314)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=temperature_dial,
        origin=Origin(xyz=(0.186, 0.0, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(0.186, 0.099, 0.238)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.18,
            upper=0.32,
        ),
    )
    model.articulation(
        "drawer_to_release_latch",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=release_latch,
        origin=Origin(xyz=(0.041, 0.0, 0.093)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.12,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    release_latch = object_model.get_part("release_latch")
    power_rocker = object_model.get_part("power_rocker")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    timer_joint = object_model.get_articulation("body_to_timer_dial")
    temperature_joint = object_model.get_articulation("body_to_temperature_dial")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    latch_joint = object_model.get_articulation("drawer_to_release_latch")

    ctx.check(
        "timer dial is continuous",
        timer_joint.articulation_type == ArticulationType.CONTINUOUS
        and timer_joint.motion_limits is not None
        and timer_joint.motion_limits.lower is None
        and timer_joint.motion_limits.upper is None,
        details=f"type={timer_joint.articulation_type!r}, limits={timer_joint.motion_limits!r}",
    )
    ctx.check(
        "temperature dial is continuous",
        temperature_joint.articulation_type == ArticulationType.CONTINUOUS
        and temperature_joint.motion_limits is not None
        and temperature_joint.motion_limits.lower is None
        and temperature_joint.motion_limits.upper is None,
        details=f"type={temperature_joint.articulation_type!r}, limits={temperature_joint.motion_limits!r}",
    )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="yz",
            min_overlap=0.12,
            name="drawer stays aligned with the body opening when closed",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    upper_drawer = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if upper_drawer is not None:
        with ctx.pose({drawer_slide: upper_drawer}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.045,
                name="drawer remains retained in the body at full extension",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.12,
                name="drawer remains centered while extended",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends outward from the front",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.15,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    lower_rocker = rocker_joint.motion_limits.lower if rocker_joint.motion_limits is not None else None
    upper_rocker = rocker_joint.motion_limits.upper if rocker_joint.motion_limits is not None else None
    if lower_rocker is not None and upper_rocker is not None:
        with ctx.pose({rocker_joint: lower_rocker}):
            rocker_lower_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_paddle")
        with ctx.pose({rocker_joint: upper_rocker}):
            rocker_upper_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_paddle")
        rocker_lower_max_x = rocker_lower_aabb[1][0] if rocker_lower_aabb is not None else None
        rocker_upper_max_x = rocker_upper_aabb[1][0] if rocker_upper_aabb is not None else None
        ctx.check(
            "power rocker swings outward at its free edge",
            rocker_lower_max_x is not None
            and rocker_upper_max_x is not None
            and rocker_upper_max_x > rocker_lower_max_x + 0.004,
            details=f"lower_max_x={rocker_lower_max_x}, upper_max_x={rocker_upper_max_x}",
        )

    rest_latch_pos = ctx.part_world_position(release_latch)
    upper_latch = latch_joint.motion_limits.upper if latch_joint.motion_limits is not None else None
    if upper_latch is not None:
        with ctx.pose({latch_joint: upper_latch}):
            ctx.expect_overlap(
                release_latch,
                drawer,
                axes="xy",
                min_overlap=0.015,
                name="release latch stays guided by the handle rails",
            )
            extended_latch_pos = ctx.part_world_position(release_latch)
        ctx.check(
            "release latch slides upward on the handle",
            rest_latch_pos is not None
            and extended_latch_pos is not None
            and extended_latch_pos[2] > rest_latch_pos[2] + 0.015,
            details=f"rest={rest_latch_pos}, extended={extended_latch_pos}",
        )

    return ctx.report()


object_model = build_object_model()

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


def _elliptical_tube(
    outer_rx: float,
    outer_ry: float,
    inner_rx: float,
    inner_ry: float,
    height: float,
    *,
    center_y: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .center(0.0, center_y)
        .ellipse(outer_rx, outer_ry)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )
    cutter = (
        cq.Workplane("XY")
        .center(0.0, center_y)
        .ellipse(inner_rx, inner_ry)
        .extrude(height + 0.006)
        .translate((0.0, 0.0, z0 - 0.003))
    )
    return outer.cut(cutter)


def _bowl_shell() -> cq.Workplane:
    """Transparent oval work bowl: a tapered, thin-walled open shell."""
    height = 0.305
    outer = (
        cq.Workplane("XY")
        .ellipse(0.124, 0.104)
        .workplane(offset=height)
        .ellipse(0.166, 0.136)
        .loft(combine=True)
    )
    # Start the cutter above the bottom to leave a molded floor, and run it
    # above the rim so the top remains visibly open.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .ellipse(0.111, 0.091)
        .workplane(offset=height + 0.030)
        .ellipse(0.156, 0.126)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _lid_with_feed_tube() -> cq.Workplane:
    """Rear-hinged clear lid, modeled in the lid frame with the hinge on X."""
    lid_center_y = -0.145
    lid = (
        cq.Workplane("XY")
        .center(0.0, lid_center_y)
        .ellipse(0.176, 0.147)
        .extrude(0.052)
        .translate((0.0, 0.0, -0.016))
    )
    underside_cavity = (
        cq.Workplane("XY")
        .center(0.0, lid_center_y)
        .ellipse(0.158, 0.129)
        .extrude(0.048)
        .translate((0.0, 0.0, -0.022))
    )
    lid = lid.cut(underside_cavity)

    tube_y = -0.170
    tube = _elliptical_tube(0.053, 0.042, 0.039, 0.029, 0.225, center_y=tube_y, z0=0.010)
    top_bead = _elliptical_tube(0.058, 0.047, 0.039, 0.029, 0.015, center_y=tube_y, z0=0.220)
    lid = lid.union(tube).union(top_bead)

    feed_opening = (
        cq.Workplane("XY")
        .center(0.0, tube_y)
        .ellipse(0.039, 0.029)
        .extrude(0.285)
        .translate((0.0, 0.0, -0.030))
    )
    return lid.cut(feed_opening)


def _pusher_geometry() -> cq.Workplane:
    shoulder = cq.Workplane("XY").ellipse(0.060, 0.046).extrude(0.028)
    tall_grip = cq.Workplane("XY").ellipse(0.040, 0.030).extrude(0.180).translate((0.0, 0.0, 0.028))
    top_cap = cq.Workplane("XY").ellipse(0.052, 0.039).extrude(0.032).translate((0.0, 0.0, 0.208))
    return shoulder.union(tall_grip).union(top_cap)


def _cutting_disc_geometry() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(0.105).extrude(0.006).translate((0.0, 0.0, -0.003))
    center_hole = cq.Workplane("XY").circle(0.019).extrude(0.050).translate((0.0, 0.0, -0.025))
    hub = cq.Workplane("XY").circle(0.030).extrude(0.020).translate((0.0, 0.0, -0.010)).cut(center_hole)
    disc = disc.cut(center_hole).union(hub)

    blade_a = (
        cq.Workplane("XY")
        .box(0.120, 0.014, 0.008)
        .translate((0.030, 0.000, 0.005))
        .rotate((0, 0, 0), (0, 0, 1), 16)
    )
    blade_b = (
        cq.Workplane("XY")
        .box(0.090, 0.012, 0.006)
        .translate((-0.035, 0.000, 0.004))
        .rotate((0, 0, 0), (0, 0, 1), 196)
    )
    return disc.union(blade_a).union(blade_b)


def _rounded_base() -> cq.Workplane:
    return cq.Workplane("XY").box(0.440, 0.360, 0.105).edges("|Z").fillet(0.040)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_food_processor")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.78, 1.0))
    dark = model.material("smoked_control_glass", rgba=(0.035, 0.040, 0.045, 1.0))
    translucent = model.material("clear_smoke_polycarbonate", rgba=(0.62, 0.82, 0.94, 0.34))
    metal = model.material("brushed_stainless_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.010, 0.010, 0.010, 1.0))
    button_blue = model.material("blue_preset_buttons", rgba=(0.12, 0.30, 0.55, 1.0))
    soft_gray = model.material("soft_gray_plastic", rgba=(0.50, 0.51, 0.50, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base(), "rounded_motor_base", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=warm_white,
        name="rounded_motor_base",
    )
    base.visual(
        Box((0.360, 0.085, 0.165)),
        origin=Origin(xyz=(0.0, -0.197, 0.110)),
        material=warm_white,
        name="control_pod_body",
    )
    base.visual(
        Box((0.315, 0.010, 0.125)),
        origin=Origin(xyz=(0.0, -0.244, 0.126)),
        material=dark,
        name="control_face",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=soft_gray,
        name="bowl_seat_collar",
    )
    base.visual(
        mesh_from_cadquery(_bowl_shell(), "oval_work_bowl", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=translucent,
        name="oval_work_bowl",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.188), rpy=(0.0, 0.0, 0.0)),
        material=soft_gray,
        name="drive_shaft",
    )
    base.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=soft_gray,
        name="disc_support_flange",
    )
    base.visual(
        Box((0.380, 0.038, 0.305)),
        origin=Origin(xyz=(0.0, 0.166, 0.2575)),
        material=warm_white,
        name="rear_hinge_tower",
    )
    base.visual(
        Box((0.076, 0.038, 0.030)),
        origin=Origin(xyz=(-0.154, 0.166, 0.425)),
        material=warm_white,
        name="hinge_support_0",
    )
    base.visual(
        Box((0.076, 0.038, 0.030)),
        origin=Origin(xyz=(0.154, 0.166, 0.425)),
        material=warm_white,
        name="hinge_support_1",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(-0.154, 0.151, 0.435), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="hinge_barrel_0",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.154, 0.151, 0.435), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="hinge_barrel_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_with_feed_tube(), "rear_hinged_lid", tolerance=0.0012),
        material=translucent,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.125, 0.032, 0.007)),
        origin=Origin(xyz=(0.0, -0.016, 0.003)),
        material=soft_gray,
        name="hinge_leaf",
    )

    pusher = model.part("feed_pusher")
    pusher.visual(
        mesh_from_cadquery(_pusher_geometry(), "feed_pusher", tolerance=0.0012),
        material=black,
        name="pusher_plunger",
    )

    disc = model.part("cutting_disc")
    disc.visual(
        mesh_from_cadquery(_cutting_disc_geometry(), "cutting_disc", tolerance=0.001),
        material=metal,
        name="slicing_disc",
    )

    selector = model.part("selector_dial")
    selector_knob = KnobGeometry(
        0.086,
        0.032,
        body_style="skirted",
        top_diameter=0.070,
        skirt=KnobSkirt(0.100, 0.006, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=22, depth=0.0015),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    selector.visual(
        mesh_from_geometry(selector_knob, "selector_dial"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="dial_cap",
    )

    button_positions = {
        "preset_button_0": (0.0, -0.246, 0.198),
        "preset_button_1": (0.0, -0.246, 0.054),
        "preset_button_2": (-0.126, -0.246, 0.126),
        "preset_button_3": (0.126, -0.246, 0.126),
    }
    for name, (x, y, z) in button_positions.items():
        button = model.part(name)
        # The button body is a proud rectangular push cap; its part frame is on
        # the control face, and positive prismatic travel pushes it inward.
        button.visual(
            Box((0.052, 0.017, 0.028)),
            origin=Origin(xyz=(0.0, -0.0085, 0.0)),
            material=button_blue,
            name="button_cap",
        )
        button.visual(
            Box((0.030, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.0178, 0.0)),
            material=translucent,
            name="button_label",
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.009),
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.151, 0.435)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.170, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=-0.120, upper=0.0),
    )
    model.articulation(
        "base_to_cutting_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, 0.268)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=40.0),
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector,
        origin=Origin(xyz=(0.0, -0.249, 0.126)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    disc = object_model.get_part("cutting_disc")
    selector = object_model.get_part("selector_dial")

    lid_joint = object_model.get_articulation("base_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_feed_pusher")
    disc_joint = object_model.get_articulation("base_to_cutting_disc")
    selector_joint = object_model.get_articulation("base_to_selector_dial")

    ctx.check(
        "all primary user mechanisms are articulated",
        len(object_model.articulations) == 8,
        details=f"articulation_count={len(object_model.articulations)}",
    )

    with ctx.pose({lid_joint: 0.0}):
        closed_lid_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(
            lid,
            "base",
            axis="z",
            positive_elem="lid_shell",
            negative_elem="oval_work_bowl",
            max_penetration=0.004,
            name="closed lid seats on bowl rim",
        )

    with ctx.pose({lid_joint: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: -0.120}):
        lowered_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "feed pusher slides downward into feed tube",
        rest_pusher is not None and lowered_pusher is not None and lowered_pusher[2] < rest_pusher[2] - 0.10,
        details=f"rest={rest_pusher}, lowered={lowered_pusher}",
    )

    ctx.check(
        "cutting disc is continuous about vertical shaft",
        disc_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(disc_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={disc_joint.articulation_type}, axis={disc_joint.axis}",
    )
    ctx.check(
        "selector dial is continuous about control face normal",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(selector_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={selector_joint.articulation_type}, axis={selector_joint.axis}",
    )

    for i in range(4):
        button = object_model.get_part(f"preset_button_{i}")
        joint = object_model.get_articulation(f"base_to_preset_button_{i}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.009}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"preset button {i} moves inward independently",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.006,
            details=f"rest={rest}, pressed={pressed}",
        )

    ctx.expect_within(
        disc,
        "base",
        axes="xy",
        inner_elem="slicing_disc",
        outer_elem="oval_work_bowl",
        margin=0.020,
        name="cutting disc fits within oval bowl footprint",
    )
    ctx.expect_within(
        selector,
        "base",
        axes="xz",
        inner_elem="dial_cap",
        outer_elem="control_face",
        margin=0.015,
        name="selector dial is centered on control pod face",
    )

    return ctx.report()


object_model = build_object_model()

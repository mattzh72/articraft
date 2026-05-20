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

BASE_TOP_Z = 0.100
BOWL_OFFSET_Y = 0.010
BOWL_HINGE_Y = 0.082
BOWL_HINGE_Z = 0.191
FEED_TUBE_X = 0.028
FEED_TUBE_Y = -0.060
FEED_TUBE_TOP_Z = 0.122


def _base_body_shape() -> object:
    plinth = cq.Workplane("XY").box(0.220, 0.268, 0.012).translate((0.0, 0.0, 0.006))
    body = (
        cq.Workplane("XY")
        .rect(0.236, 0.282)
        .workplane(offset=0.086)
        .center(0.0, 0.010)
        .rect(0.204, 0.242)
        .loft(combine=True)
    )
    return body.union(plinth)


def _control_cluster_shape() -> object:
    return cq.Workplane("XY").box(0.118, 0.022, 0.072).translate((0.0, -0.145, 0.048))


def _bowl_shell_shape() -> object:
    outer = (
        cq.Workplane("XY")
        .circle(0.053)
        .workplane(offset=0.050)
        .circle(0.068)
        .workplane(offset=0.070)
        .circle(0.075)
        .workplane(offset=0.060)
        .circle(0.080)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .circle(0.045)
        .workplane(offset=0.050)
        .circle(0.062)
        .workplane(offset=0.070)
        .circle(0.069)
        .workplane(offset=0.072)
        .circle(0.074)
        .loft(combine=True)
    )
    shell = outer.cut(inner.translate((0.0, 0.0, 0.006)))
    drive_opening = cq.Workplane("XY").circle(0.018).extrude(0.020)
    return shell.cut(drive_opening)


def _lid_shell_shape() -> object:
    shell = cq.Workplane("XY").circle(0.084).extrude(0.016)
    shell = shell.cut(cq.Workplane("XY").circle(0.079).extrude(0.014))
    feed_hole = cq.Workplane("XY").circle(0.0315).extrude(0.022).translate((FEED_TUBE_X, FEED_TUBE_Y, -0.002))
    return shell.cut(feed_hole).translate((0.0, -0.090, -0.002))


def _feed_tube_shape() -> object:
    outer = cq.Workplane("XY").circle(0.035).extrude(0.112)
    inner = cq.Workplane("XY").circle(0.031).extrude(0.116).translate((0.0, 0.0, -0.002))
    return outer.cut(inner).translate((FEED_TUBE_X, FEED_TUBE_Y, 0.010))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_food_processor")

    base_white = model.material("base_white", rgba=(0.90, 0.89, 0.85, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.73, 0.75, 0.78, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.79, 0.84, 0.90, 0.35))
    pusher_white = model.material("pusher_white", rgba=(0.94, 0.94, 0.95, 0.92))
    control_black = model.material("control_black", rgba=(0.15, 0.15, 0.16, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "food_processor_body"),
        material=base_white,
        name="body_shell",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.014),
        origin=Origin(xyz=(0.0, BOWL_OFFSET_Y, 0.093)),
        material=base_white,
        name="deck_ring",
    )
    base.visual(
        mesh_from_cadquery(_control_cluster_shape(), "food_processor_control_cluster"),
        material=panel_grey,
        name="control_cluster",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell_shape(), "food_processor_bowl"),
        material=clear_smoke,
        name="bowl_shell",
    )
    bowl.visual(
        Box((0.122, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.078, 0.183)),
        material=clear_smoke,
        name="rear_bracket",
    )
    bowl.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.042, 0.086, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_smoke,
        name="hinge_barrel_0",
    )
    bowl.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.042, 0.086, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_smoke,
        name="hinge_barrel_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "food_processor_lid"),
        material=clear_smoke,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_feed_tube_shape(), "food_processor_feed_tube"),
        material=clear_smoke,
        name="feed_tube",
    )
    lid.visual(
        Box((0.050, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.001, 0.010)),
        material=clear_smoke,
        name="rear_bridge",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(xyz=(0.0, 0.006, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_smoke,
        name="hinge_barrel",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=pusher_white,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=pusher_white,
        name="pusher_handle",
    )
    pusher.visual(
        Cylinder(radius=0.0265, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=pusher_white,
        name="pusher_body",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=blade_metal,
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=blade_metal,
        name="drive_collar",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blade_metal,
        name="shaft",
    )
    spindle.visual(
        Box((0.060, 0.014, 0.002)),
        origin=Origin(xyz=(0.024, 0.0, 0.033), rpy=(0.0, 0.18, 0.05)),
        material=blade_metal,
        name="blade_0",
    )
    spindle.visual(
        Box((0.060, 0.014, 0.002)),
        origin=Origin(xyz=(-0.024, 0.0, 0.039), rpy=(0.0, -0.18, math.pi + 0.05)),
        material=blade_metal,
        name="blade_1",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.024,
                body_style="skirted",
                top_diameter=0.033,
                skirt=KnobSkirt(0.050, 0.006, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "food_processor_speed_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="knob_shell",
    )

    for index, x in enumerate((-0.026, 0.026)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, -0.005, 0.009)),
            material=control_black,
            name="button_cap",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.156, 0.032)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.0035),
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, BOWL_OFFSET_Y, BASE_TOP_Z)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, BOWL_HINGE_Y, BOWL_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(FEED_TUBE_X, FEED_TUBE_Y, FEED_TUBE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.10, lower=0.0, upper=0.055),
    )
    model.articulation(
        "base_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, BOWL_OFFSET_Y, BASE_TOP_Z + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -0.156, 0.067)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    spindle = object_model.get_part("spindle")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_hinge = object_model.get_articulation("bowl_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    button_joint_0 = object_model.get_articulation("base_to_button_0")

    ctx.expect_origin_distance(
        spindle,
        bowl,
        axes="xy",
        max_dist=0.002,
        name="spindle stays centered in the bowl",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            bowl,
            axes="xy",
            elem_a="lid_shell",
            elem_b="bowl_shell",
            min_overlap=0.120,
            name="closed lid covers the bowl opening",
        )
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="bowl_shell",
            max_gap=0.015,
            max_penetration=0.0,
            name="closed lid sits just above the bowl rim",
        )
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="feed_tube",
            margin=0.001,
            name="pusher body aligns inside the feed tube at rest",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_body",
            elem_b="feed_tube",
            min_overlap=0.090,
            name="resting pusher remains deeply retained in the feed tube",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="feed_tube",
            margin=0.001,
            name="pusher body stays aligned when the lid is raised",
        )

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.055}):
        pusher_pressed = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="feed_tube",
            margin=0.001,
            name="pressed pusher stays centered in the tube",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_body",
            elem_b="feed_tube",
            min_overlap=0.035,
            name="pressed pusher still retains insertion in the tube",
        )

    ctx.check(
        "pusher travels downward into the feed tube",
        pusher_rest is not None and pusher_pressed is not None and pusher_pressed[2] < pusher_rest[2] - 0.040,
        details=f"rest={pusher_rest}, pressed={pusher_pressed}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.0035}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_static = ctx.part_world_position(button_1)

    ctx.check(
        "front buttons are independent push controls",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_static is not None
        and button_0_pressed[1] < button_0_rest[1] - 0.002
        and abs(button_1_static[1] - button_1_rest[1]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_static={button_1_static}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

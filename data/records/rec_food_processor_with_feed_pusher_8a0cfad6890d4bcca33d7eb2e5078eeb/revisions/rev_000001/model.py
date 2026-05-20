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


BASE_W = 0.300
BASE_D = 0.220
BASE_H = 0.064
BASE_COLLAR_H = 0.008

BOWL_R = 0.108
BOWL_H = 0.072
BOWL_WALL = 0.004
BOWL_FLOOR = 0.004
BOWL_ORIGIN = (0.0, 0.010, BASE_H + BASE_COLLAR_H)

HINGE_Y = 0.118
HINGE_Z = 0.080

LID_R = 0.112
LID_CENTER_Y = -0.123
FEED_X = 0.038
FEED_Y = -0.072


def _build_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
    base = base.edges("|Z").fillet(0.012)
    base = base.edges(">Z").fillet(0.008)

    collar = (
        cq.Workplane("XY")
        .workplane(offset=BASE_H - 0.001)
        .circle(0.090)
        .extrude(BASE_COLLAR_H + 0.001)
    )

    control_cavity = (
        cq.Workplane("XY")
        .box(0.205, 0.048, 0.058, centered=(True, True, False))
        .translate((0.005, -0.086, 0.004))
    )

    return base.union(collar).cut(control_cavity)


def _build_bowl_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(BOWL_R).extrude(BOWL_H)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_FLOOR)
        .circle(BOWL_R - BOWL_WALL - 0.002)
        .extrude(BOWL_H - BOWL_FLOOR + 0.004)
    )
    shell = outer.cut(inner)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_H - 0.004)
        .circle(BOWL_R + 0.005)
        .circle(BOWL_R - BOWL_WALL - 0.001)
        .extrude(0.004)
    )
    return shell.union(rim)


def _build_hinge_mount_shape() -> cq.Workplane:
    bridge = (
        cq.Workplane("XY")
        .box(0.086, 0.010, 0.006, centered=(True, True, False))
        .translate((0.0, BOWL_R - 0.001, BOWL_H - 0.012))
    )
    ear_base = cq.Workplane("XY").box(0.020, 0.008, 0.028, centered=(True, True, False))
    ear_left = ear_base.translate((-0.032, HINGE_Y + 0.004, BOWL_H - 0.006))
    ear_right = ear_base.translate((0.032, HINGE_Y + 0.004, BOWL_H - 0.006))
    web_base = cq.Workplane("XY").box(0.012, 0.016, 0.010, centered=(True, True, False))
    web_left = web_base.translate((-0.032, HINGE_Y - 0.004, BOWL_H - 0.012))
    web_right = web_base.translate((0.032, HINGE_Y - 0.004, BOWL_H - 0.012))
    return bridge.union(web_left).union(web_right).union(ear_left).union(ear_right)


def _build_drive_post_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").workplane(offset=0.003).circle(0.018).extrude(0.008)
    post = cq.Workplane("XY").workplane(offset=0.003).circle(0.0065).extrude(0.025)
    support_ring = (
        cq.Workplane("XY")
        .workplane(offset=0.022)
        .circle(0.014)
        .circle(0.006)
        .extrude(0.002)
    )
    return collar.union(post).union(support_ring)


def _build_lid_panel_shape() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .center(0.0, LID_CENTER_Y)
        .circle(LID_R)
        .extrude(0.004)
    )
    rim = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(0.0, LID_CENTER_Y)
        .circle(LID_R + 0.002)
        .circle(LID_R - 0.008)
        .extrude(0.012)
    )
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(0.006)
        .extrude(0.044)
        .translate((-0.022, 0.0, 0.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.052, 0.010, 0.010, centered=(True, True, False))
        .translate((0.0, -0.005, -0.002))
    )
    front_tab = (
        cq.Workplane("XY")
        .box(0.030, 0.014, 0.010, centered=(True, True, False))
        .translate((0.0, LID_CENTER_Y - LID_R + 0.007, -0.006))
    )
    feed_opening = (
        cq.Workplane("XY")
        .workplane(offset=-0.012)
        .center(FEED_X, FEED_Y)
        .ellipse(0.031, 0.018)
        .extrude(0.028)
    )
    return top_plate.union(rim).union(hinge_barrel).union(rear_bridge).union(front_tab).cut(feed_opening)


def _build_feed_tube_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.002)
        .center(FEED_X, FEED_Y)
        .ellipse(0.038, 0.024)
        .extrude(0.088)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(FEED_X, FEED_Y)
        .ellipse(0.031, 0.018)
        .extrude(0.100)
    )
    lip = (
        cq.Workplane("XY")
        .workplane(offset=0.086)
        .center(FEED_X, FEED_Y)
        .ellipse(0.040, 0.026)
        .ellipse(0.032, 0.019)
        .extrude(0.004)
    )
    return outer.cut(inner).union(lip)


def _build_pusher_shape() -> cq.Workplane:
    body = cq.Workplane("XY").workplane(offset=-0.066).ellipse(0.0285, 0.0160).extrude(0.070)
    cap = cq.Workplane("XY").workplane(offset=0.004).ellipse(0.034, 0.020).extrude(0.010)
    handle = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .box(0.056, 0.016, 0.010, centered=(True, True, False))
    )
    return body.union(cap).union(handle)


def _build_cutter_disc_shape() -> cq.Workplane:
    hole = cq.Workplane("XY").circle(0.0075).extrude(0.014)
    disc = cq.Workplane("XY").circle(0.080).circle(0.0075).extrude(0.002)
    hub = cq.Workplane("XY").circle(0.018).circle(0.0075).extrude(0.012)

    blade = cq.Workplane("XY").box(0.072, 0.014, 0.003)
    blade_a = blade.translate((0.020, 0.0, 0.0025)).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 18.0)
    blade_b = blade.translate((-0.020, 0.0, 0.0025)).rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -18.0)

    return disc.union(hub).union(blade_a).union(blade_b).cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_food_processor")

    body_plastic = model.material("body_plastic", rgba=(0.15, 0.16, 0.17, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.84, 0.88, 0.38))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.82, 1.0))
    button_grey = model.material("button_grey", rgba=(0.83, 0.85, 0.87, 1.0))
    pusher_grey = model.material("pusher_grey", rgba=(0.88, 0.89, 0.90, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "processor_base"),
        material=body_plastic,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(-0.040, -0.108, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="dial_bezel",
    )
    base.visual(
        Box((0.205, 0.004, 0.058)),
        origin=Origin(xyz=(0.005, -0.108, 0.033)),
        material=metal,
        name="control_panel",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_build_bowl_shell_shape(), "processor_bowl"),
        material=clear_smoke,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(_build_hinge_mount_shape(), "processor_hinge_mount"),
        material=body_plastic,
        name="hinge_mount",
    )
    bowl.visual(
        mesh_from_cadquery(_build_drive_post_shape(), "processor_drive_post"),
        material=dark_trim,
        name="drive_post",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=BOWL_ORIGIN),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_panel_shape(), "processor_lid_panel"),
        material=clear_smoke,
        name="lid_panel",
    )
    lid.visual(
        mesh_from_cadquery(_build_feed_tube_shape(), "processor_feed_tube"),
        material=clear_smoke,
        name="feed_tube",
    )

    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_build_pusher_shape(), "processor_pusher"),
        material=pusher_grey,
        name="pusher_shell",
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(FEED_X, FEED_Y, 0.086)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.055,
        ),
    )

    cutter_disc = model.part("cutter_disc")
    cutter_disc.visual(
        mesh_from_cadquery(_build_cutter_disc_shape(), "processor_cutter_disc"),
        material=metal,
        name="disc",
    )
    model.articulation(
        "bowl_to_cutter_disc",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=cutter_disc,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    control_dial = model.part("control_dial")
    control_dial.visual(
        Cylinder(radius=0.025, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="dial_knob",
    )
    control_dial.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="dial_cap",
    )
    model.articulation(
        "base_to_control_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=control_dial,
        origin=Origin(xyz=(-0.040, -0.110, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    for index, x_pos in enumerate((0.020, 0.047, 0.074)):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.019, 0.010, 0.013)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        model.articulation(
            f"base_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, -0.110, 0.031)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    cutter_disc = object_model.get_part("cutter_disc")

    lid_hinge = object_model.get_articulation("bowl_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")

    ctx.allow_overlap(
        bowl,
        lid,
        elem_a="hinge_mount",
        elem_b="lid_panel",
        reason="The simplified rear hinge ears intentionally interleave the lid skirt around the hinge axis.",
    )
    ctx.allow_overlap(
        lid,
        pusher,
        elem_a="feed_tube",
        elem_b="pusher_shell",
        reason="The pusher is intentionally represented as a close nested fit inside the simplified feed-tube sleeve.",
    )

    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="bowl_shell",
        max_gap=0.012,
        max_penetration=0.0,
        name="lid sits just above the bowl rim",
    )
    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        elem_a="lid_panel",
        elem_b="bowl_shell",
        min_overlap=0.18,
        name="lid covers the bowl opening",
    )
    ctx.expect_overlap(
        cutter_disc,
        bowl,
        axes="xy",
        elem_a="disc",
        elem_b="bowl_shell",
        min_overlap=0.15,
        name="cutter disc stays centered over the bowl floor",
    )
    ctx.expect_gap(
        lid,
        cutter_disc,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="disc",
        min_gap=0.030,
        name="lid leaves real cavity above the cutter disc",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shell",
        outer_elem="feed_tube",
        margin=0.006,
        name="pusher stays centered in the feed tube at rest",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shell",
        elem_b="feed_tube",
        min_overlap=0.050,
        name="retracted pusher remains inserted in the feed tube",
    )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward on the rear hinge",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.060,
        details=f"closed={lid_rest_aabb}, open={lid_open_aabb}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shell",
            outer_elem="feed_tube",
            margin=0.006,
            name="pressed pusher stays aligned in the feed tube",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shell",
            elem_b="feed_tube",
            min_overlap=0.020,
            name="pressed pusher still retains guide engagement",
        )
        pusher_pressed = ctx.part_world_position(pusher)
    ctx.check(
        "pusher travels downward into the feed tube",
        pusher_rest is not None
        and pusher_pressed is not None
        and pusher_pressed[2] < pusher_rest[2] - 0.040,
        details=f"rest={pusher_rest}, pressed={pusher_pressed}",
    )

    for index in range(3):
        button = object_model.get_part(f"preset_button_{index}")
        joint = object_model.get_articulation(f"base_to_preset_button_{index}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"preset button {index} presses inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.003,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()

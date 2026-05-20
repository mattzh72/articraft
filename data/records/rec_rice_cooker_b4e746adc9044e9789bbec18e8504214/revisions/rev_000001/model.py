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


BODY_RADIUS = 0.118
BODY_HEIGHT = 0.158
BODY_WALL = 0.0025
HINGE_X = -0.103
HINGE_Z = 0.162

LID_RADIUS = 0.110
LID_HEIGHT = 0.026
LID_WALL = 0.0022
LID_CENTER_X = 0.103


def _make_body_shell() -> object:
    return cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT).faces(">Z").shell(-BODY_WALL)


def _make_ring(outer_radius: float, inner_radius: float, height: float) -> object:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _make_lid_cap() -> object:
    return (
        cq.Workplane("XY")
        .circle(LID_RADIUS)
        .extrude(LID_HEIGHT)
        .edges(">Z")
        .fillet(0.006)
        .faces("<Z")
        .shell(-LID_WALL)
        .translate((LID_CENTER_X, 0.0, 0.002))
    )


def _make_lid_rim() -> object:
    return _make_ring(0.106, 0.097, 0.010).translate((LID_CENTER_X, 0.0, -0.006))


def _make_vent_housing() -> object:
    vent = (
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.010)
        .edges("|Z")
        .fillet(0.0025)
    )
    for slot_y in (-0.005, 0.0, 0.005):
        vent = vent.cut(
            cq.Workplane("XY").box(0.018, 0.0022, 0.008).translate((0.001, slot_y, 0.002))
        )
    return vent


def _make_panel_pod(depth: float, width: float, height: float, fillet: float) -> object:
    radius = min(fillet, depth * 0.24, width * 0.08, height * 0.24)
    return cq.Workplane("XY").box(depth, width, height).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    warm_white = model.material("warm_white", rgba=(0.93, 0.93, 0.90, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    button_white = model.material("button_white", rgba=(0.96, 0.96, 0.94, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.15, 0.19, 0.22, 0.85))
    warm_orange = model.material("warm_orange", rgba=(0.93, 0.58, 0.18, 1.0))
    cook_red = model.material("cook_red", rgba=(0.79, 0.16, 0.12, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_make_body_shell(), "body_shell"), material=stainless, name="body_shell")
    body.visual(
        Cylinder(radius=0.1205, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=stainless,
        name="body_belly",
    )
    body.visual(
        Cylinder(radius=0.121, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=charcoal,
        name="base_ring",
    )
    body.visual(
        mesh_from_cadquery(_make_ring(0.118, 0.109, 0.006), "body_top_trim"),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=warm_white,
        name="top_trim",
    )
    body.visual(
        mesh_from_cadquery(_make_panel_pod(0.014, 0.082, 0.058, 0.007), "control_panel"),
        origin=Origin(xyz=(0.122, 0.0, 0.060)),
        material=warm_white,
        name="control_panel",
    )
    body.visual(
        Box((0.003, 0.034, 0.011)),
        origin=Origin(xyz=(0.129, 0.0, 0.084)),
        material=dark_glass,
        name="panel_window",
    )
    body.visual(
        mesh_from_cadquery(_make_panel_pod(0.016, 0.060, 0.024, 0.006), "latch_housing"),
        origin=Origin(xyz=(0.121, 0.0, 0.104)),
        material=warm_white,
        name="latch_housing",
    )
    for suffix, y_pos in (("0", -0.050), ("1", 0.050)):
        body.visual(
            Box((0.018, 0.020, 0.018)),
            origin=Origin(xyz=(-0.108, y_pos, 0.153)),
            material=charcoal,
            name=f"hinge_bridge_{suffix}",
        )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_make_lid_cap(), "lid_cap"), material=warm_white, name="lid_cap")
    lid.visual(mesh_from_cadquery(_make_lid_rim(), "lid_rim"), material=stainless, name="lid_rim")
    lid.visual(
        Box((0.022, 0.030, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.014)),
        material=charcoal,
        name="hinge_mount",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.0048, length=0.028),
        origin=Origin(xyz=(0.100, -0.038, 0.032)),
        material=charcoal,
        name="handle_post_0",
    )
    lid.visual(
        Cylinder(radius=0.0048, length=0.028),
        origin=Origin(xyz=(0.100, 0.038, 0.032)),
        material=charcoal,
        name="handle_post_1",
    )
    lid.visual(
        Cylinder(radius=0.0055, length=0.086),
        origin=Origin(xyz=(0.100, 0.0, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="handle_grip",
    )
    lid.visual(
        mesh_from_cadquery(_make_vent_housing(), "steam_vent_housing"),
        origin=Origin(xyz=(0.050, 0.0, 0.030)),
        material=warm_white,
        name="steam_vent",
    )
    lid.visual(
        Box((0.0025, 0.020, 0.003)),
        origin=Origin(xyz=(0.064, 0.0, 0.033)),
        material=dark_glass,
        name="vent_insert",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.010, 0.046, 0.016)),
        origin=Origin(xyz=(0.005, 0.0, 0.008)),
        material=button_white,
        name="button_cap",
    )

    cook_button = model.part("cook_button")
    cook_button.visual(
        Box((0.008, 0.028, 0.013)),
        origin=Origin(xyz=(0.004, 0.0, 0.0065)),
        material=cook_red,
        name="button_cap",
    )

    warm_button = model.part("warm_button")
    warm_button.visual(
        Box((0.008, 0.028, 0.013)),
        origin=Origin(xyz=(0.004, 0.0, 0.0065)),
        material=warm_orange,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.129, 0.0, 0.103)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.04, lower=0.0, upper=0.0045),
    )
    model.articulation(
        "body_to_cook_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cook_button,
        origin=Origin(xyz=(0.126, -0.019, 0.056)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.03, lower=0.0, upper=0.0025),
    )
    model.articulation(
        "body_to_warm_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=warm_button,
        origin=Origin(xyz=(0.126, 0.019, 0.056)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.03, lower=0.0, upper=0.0025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    cook_button = object_model.get_part("cook_button")
    warm_button = object_model.get_part("warm_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_joint = object_model.get_articulation("body_to_latch_button")
    cook_joint = object_model.get_articulation("body_to_cook_button")
    warm_joint = object_model.get_articulation("body_to_warm_button")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_cap",
            negative_elem="body_shell",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid cap sits just above body shell",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_cap",
            elem_b="body_shell",
            min_overlap=0.205,
            name="closed lid covers the cooker body",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_cap")
    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_cap")
        ctx.check(
            "lid opens upward",
            closed_lid_aabb is not None
            and opened_lid_aabb is not None
            and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.070,
            details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
        )

    def _check_button_travel(part, joint, min_delta: float, name: str) -> None:
        rest_pos = ctx.part_world_position(part)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        pushed_pos = None
        if upper is not None:
            with ctx.pose({joint: upper}):
                pushed_pos = ctx.part_world_position(part)
        ctx.check(
            name,
            rest_pos is not None and pushed_pos is not None and pushed_pos[0] < rest_pos[0] - min_delta,
            details=f"rest={rest_pos}, pushed={pushed_pos}",
        )

    _check_button_travel(latch_button, latch_joint, 0.0035, "front latch button depresses into the body")
    _check_button_travel(cook_button, cook_joint, 0.0018, "cook button depresses independently")
    _check_button_travel(warm_button, warm_joint, 0.0018, "warm button depresses independently")
    ctx.expect_origin_distance(
        cook_button,
        warm_button,
        axes="y",
        min_dist=0.030,
        max_dist=0.050,
        name="cook and warm buttons sit as a compact two-button stack",
    )

    return ctx.report()


object_model = build_object_model()

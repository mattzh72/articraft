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


BODY_RADIUS = 0.132
BODY_BASE_Z = 0.022
BODY_HEIGHT = 0.188
BODY_TOP_Z = BODY_BASE_Z + BODY_HEIGHT
BODY_WALL = 0.003

BASE_RING_OUTER = 0.108
BASE_RING_INNER = 0.080
BASE_RING_Z = 0.012
BASE_RING_HEIGHT = 0.010

CLUSTER_WIDTH = 0.126
CLUSTER_DEPTH = 0.058
CLUSTER_HEIGHT = 0.048
CLUSTER_CENTER_Y = BODY_RADIUS + 0.024
CLUSTER_CENTER_Z = 0.092

RELEASE_WIDTH = 0.036
RELEASE_DEPTH = 0.014
RELEASE_HEIGHT = 0.010
RELEASE_CENTER = (0.0, CLUSTER_CENTER_Y + CLUSTER_DEPTH / 2.0 - 0.006, CLUSTER_CENTER_Z + 0.010)
RELEASE_TRAVEL = 0.006

MODE_BUTTON_WIDTH = 0.028
MODE_BUTTON_DEPTH = 0.020
MODE_BUTTON_HEIGHT = 0.008
MODE_BUTTON_Y = CLUSTER_CENTER_Y + 0.004
MODE_BUTTON_Z = CLUSTER_CENTER_Z + CLUSTER_HEIGHT / 2.0 + MODE_BUTTON_HEIGHT / 2.0
MODE_BUTTON_TRAVEL = 0.003

LID_RADIUS = 0.140
LID_HEIGHT = 0.036
LID_WALL = 0.003
LID_CENTER_Y = 0.135
LID_BOTTOM_Z = -0.006
HINGE_ORIGIN = (0.0, -BODY_RADIUS + 0.004, BODY_TOP_Z + 0.006)


def _build_body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS)
        .extrude(BODY_HEIGHT)
        .edges(">Z")
        .fillet(0.009)
        .faces(">Z")
        .shell(-BODY_WALL)
        .translate((0.0, 0.0, BODY_BASE_Z))
    )

    base_ring = (
        cq.Workplane("XY")
        .circle(BASE_RING_OUTER)
        .circle(BASE_RING_INNER)
        .extrude(BASE_RING_HEIGHT)
        .translate((0.0, 0.0, BASE_RING_Z))
    )

    cluster = (
        cq.Workplane("XY")
        .box(CLUSTER_WIDTH, CLUSTER_DEPTH, CLUSTER_HEIGHT)
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.004)
        .translate((0.0, CLUSTER_CENTER_Y, CLUSTER_CENTER_Z))
    )

    release_pocket = cq.Workplane("XY").box(0.044, 0.020, 0.012).translate(RELEASE_CENTER)
    left_well = cq.Workplane("XY").box(0.034, 0.024, 0.006).translate(
        (-0.026, MODE_BUTTON_Y, CLUSTER_CENTER_Z + CLUSTER_HEIGHT / 2.0 - 0.002)
    )
    right_well = cq.Workplane("XY").box(0.034, 0.024, 0.006).translate(
        (0.026, MODE_BUTTON_Y, CLUSTER_CENTER_Z + CLUSTER_HEIGHT / 2.0 - 0.002)
    )

    cluster = cluster.cut(release_pocket).cut(left_well).cut(right_well)

    return shell.union(base_ring).union(cluster)


def _build_body_collar_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.129)
        .circle(0.110)
        .extrude(0.006)
        .translate((0.0, 0.0, BODY_TOP_Z - 0.006))
    )


def _build_body_rim_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.120)
        .circle(0.110)
        .extrude(0.005)
        .translate((0.0, 0.0, BODY_TOP_Z - 0.005))
    )


def _build_lid_shell_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(LID_RADIUS)
        .extrude(LID_HEIGHT)
        .edges(">Z")
        .fillet(0.014)
        .faces("<Z")
        .shell(-LID_WALL)
        .translate((0.0, LID_CENTER_Y, LID_BOTTOM_Z))
    )


def _build_inner_frame_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.141).circle(0.101).extrude(0.0024)
    strap_x = cq.Workplane("XY").box(0.206, 0.016, 0.0024)
    strap_y = cq.Workplane("XY").box(0.016, 0.206, 0.0024)
    center_plate = cq.Workplane("XY").circle(0.033).extrude(0.0024)
    return ring.union(strap_x).union(strap_y).union(center_plate).translate((0.0, LID_CENTER_Y, -0.0028))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    body_white = model.material("body_white", rgba=(0.930, 0.930, 0.920, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.660, 0.680, 0.700, 1.0))
    steel = model.material("steel", rgba=(0.760, 0.780, 0.800, 1.0))
    charcoal = model.material("charcoal", rgba=(0.180, 0.190, 0.200, 1.0))
    rubber = model.material("rubber", rgba=(0.120, 0.120, 0.120, 1.0))
    soft_white = model.material("soft_white", rgba=(0.960, 0.960, 0.955, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "rice_cooker_body"),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_body_rim_shape(), "rice_cooker_body_rim"),
        material=steel,
        name="body_rim",
    )
    body.visual(
        mesh_from_cadquery(_build_body_collar_shape(), "rice_cooker_body_collar"),
        material=body_white,
        name="body_collar",
    )
    body.visual(
        Box((0.094, 0.036, 0.0015)),
        origin=Origin(xyz=(0.0, MODE_BUTTON_Y, CLUSTER_CENTER_Z + CLUSTER_HEIGHT / 2.0 + 0.00075)),
        material=panel_grey,
        name="control_panel",
    )
    body.visual(
        Box((0.080, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, HINGE_ORIGIN[1] - 0.003, HINGE_ORIGIN[2] - 0.004)),
        material=charcoal,
        name="body_hinge_leaf",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell_shape(), "rice_cooker_lid"),
        material=soft_white,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_build_inner_frame_shape(), "rice_cooker_inner_frame"),
        material=steel,
        name="inner_frame",
    )
    lid.visual(
        Box((0.062, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.003, -0.004)),
        material=charcoal,
        name="lid_hinge_leaf",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((RELEASE_WIDTH, RELEASE_DEPTH, RELEASE_HEIGHT)),
        material=panel_grey,
        name="release_cap",
    )
    release_button.visual(
        Box((0.018, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=panel_grey,
        name="release_stem",
    )
    model.articulation(
        "release_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(RELEASE_CENTER[0], RELEASE_CENTER[1] - 0.0015, RELEASE_CENTER[2])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=RELEASE_TRAVEL,
        ),
    )

    left_mode_button = model.part("left_mode_button")
    left_mode_button.visual(
        Box((MODE_BUTTON_WIDTH, MODE_BUTTON_DEPTH, MODE_BUTTON_HEIGHT)),
        material=panel_grey,
        name="left_mode_cap",
    )
    model.articulation(
        "left_mode_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_mode_button,
        origin=Origin(xyz=(-0.026, MODE_BUTTON_Y, MODE_BUTTON_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=MODE_BUTTON_TRAVEL,
        ),
    )

    right_mode_button = model.part("right_mode_button")
    right_mode_button.visual(
        Box((MODE_BUTTON_WIDTH, MODE_BUTTON_DEPTH, MODE_BUTTON_HEIGHT)),
        material=panel_grey,
        name="right_mode_cap",
    )
    model.articulation(
        "right_mode_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_mode_button,
        origin=Origin(xyz=(0.026, MODE_BUTTON_Y, MODE_BUTTON_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=MODE_BUTTON_TRAVEL,
        ),
    )

    foot_positions = {
        "front_left_foot": (-0.072, 0.068, 0.006),
        "front_right_foot": (0.072, 0.068, 0.006),
        "rear_left_foot": (-0.072, -0.054, 0.006),
        "rear_right_foot": (0.072, -0.054, 0.006),
    }
    for foot_name, foot_xyz in foot_positions.items():
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.014, length=0.012),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"{foot_name}_mount",
            ArticulationType.FIXED,
            parent=body,
            child=foot,
            origin=Origin(xyz=foot_xyz),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    release_button = object_model.get_part("release_button")
    left_mode_button = object_model.get_part("left_mode_button")
    right_mode_button = object_model.get_part("right_mode_button")

    lid_hinge = object_model.get_articulation("lid_hinge")
    release_slide = object_model.get_articulation("release_slide")
    left_mode_slide = object_model.get_articulation("left_mode_slide")
    right_mode_slide = object_model.get_articulation("right_mode_slide")

    lid_limits = lid_hinge.motion_limits
    release_limits = release_slide.motion_limits
    left_limits = left_mode_slide.motion_limits
    right_limits = right_mode_slide.motion_limits

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_rim",
            min_overlap=0.220,
            name="closed lid covers body opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="inner_frame",
            negative_elem="body_rim",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed lid frame sits just above the cooker rim",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            opened_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.090,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    if release_limits is not None and release_limits.upper is not None:
        release_rest = ctx.part_world_position(release_button)
        with ctx.pose({release_slide: release_limits.upper}):
            release_pressed = ctx.part_world_position(release_button)
        ctx.check(
            "release button slides inward",
            release_rest is not None
            and release_pressed is not None
            and release_pressed[1] < release_rest[1] - 0.004,
            details=f"rest={release_rest}, pressed={release_pressed}",
        )

    if left_limits is not None and left_limits.upper is not None:
        left_rest = ctx.part_world_position(left_mode_button)
        with ctx.pose({left_mode_slide: left_limits.upper}):
            left_pressed = ctx.part_world_position(left_mode_button)
        ctx.check(
            "left mode button depresses downward",
            left_rest is not None
            and left_pressed is not None
            and left_pressed[2] < left_rest[2] - 0.002,
            details=f"rest={left_rest}, pressed={left_pressed}",
        )

    if right_limits is not None and right_limits.upper is not None:
        right_rest = ctx.part_world_position(right_mode_button)
        with ctx.pose({right_mode_slide: right_limits.upper}):
            right_pressed = ctx.part_world_position(right_mode_button)
        ctx.check(
            "right mode button depresses downward",
            right_rest is not None
            and right_pressed is not None
            and right_pressed[2] < right_rest[2] - 0.002,
            details=f"rest={right_rest}, pressed={right_pressed}",
        )

    return ctx.report()


object_model = build_object_model()

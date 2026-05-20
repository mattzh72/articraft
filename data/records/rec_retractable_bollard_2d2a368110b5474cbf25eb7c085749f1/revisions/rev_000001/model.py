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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POST_RADIUS = 0.135
BORE_RADIUS = 0.143
POST_LENGTH = 1.31
POST_RETAINED_AT_REST = 0.48
POST_TRAVEL = 0.68
POST_TOP_Z = POST_LENGTH - POST_RETAINED_AT_REST


def _build_housing_shell() -> cq.Workplane:
    sleeve_outer_radius = 0.176
    collar_outer_radius = 0.206
    flange_outer_radius = 0.226
    sleeve_bottom_z = -1.18
    neck_bottom_z = -0.09
    neck_top_z = 0.065
    flange_bottom_z = 0.049

    sleeve = (
        cq.Workplane("XY")
        .circle(sleeve_outer_radius)
        .circle(BORE_RADIUS)
        .extrude(neck_bottom_z - sleeve_bottom_z)
        .translate((0.0, 0.0, sleeve_bottom_z))
    )
    neck = (
        cq.Workplane("XY")
        .circle(collar_outer_radius)
        .circle(BORE_RADIUS)
        .extrude(neck_top_z - neck_bottom_z)
        .translate((0.0, 0.0, neck_bottom_z))
    )
    flange = (
        cq.Workplane("XY")
        .circle(flange_outer_radius)
        .circle(BORE_RADIUS)
        .extrude(neck_top_z - flange_bottom_z)
        .translate((0.0, 0.0, flange_bottom_z))
    )

    housing = sleeve.union(neck).union(flange)

    service_cut = (
        cq.Workplane("XY")
        .box(0.108, 0.12, 0.124)
        .translate((0.004, 0.18, -0.006))
    )
    return housing.cut(service_cut)


def _build_post_shell() -> cq.Workplane:
    return cq.Workplane("XY").circle(POST_RADIUS).extrude(POST_LENGTH)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traffic_control_bollard")

    housing_paint = model.material("housing_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    post_finish = model.material("post_finish", rgba=(0.33, 0.36, 0.39, 1.0))
    reflector = model.material("reflector", rgba=(0.90, 0.92, 0.94, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.72, 0.74, 0.76, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell"),
        material=housing_paint,
        name="housing_shell",
    )
    housing.visual(
        Box((0.010, 0.018, 0.115)),
        origin=Origin(xyz=(0.060, 0.197, -0.006)),
        material=hinge_steel,
        name="hinge_leaf",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.054, 0.211, 0.036)),
        material=hinge_steel,
        name="hinge_upper",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.054, 0.211, -0.048)),
        material=hinge_steel,
        name="hinge_lower",
    )
    for name, xyz, size in (
        ("guide_x_pos", (0.141, 0.0, 0.000), (0.012, 0.055, 0.140)),
        ("guide_x_neg", (-0.141, 0.0, 0.000), (0.012, 0.055, 0.140)),
        ("guide_y_pos", (0.0, 0.141, 0.000), (0.055, 0.012, 0.140)),
        ("guide_y_neg", (0.0, -0.141, 0.000), (0.055, 0.012, 0.140)),
    ):
        housing.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=hinge_steel,
            name=name,
        )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_build_post_shell(), "post_shell"),
        origin=Origin(xyz=(0.0, 0.0, -POST_RETAINED_AT_REST)),
        material=post_finish,
        name="body_shell",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS + 0.0015, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=reflector,
        name="band_lower",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS + 0.0015, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=reflector,
        name="band_upper",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=hinge_steel,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=knob_finish,
        name="hub",
    )
    knob.visual(
        Box((0.110, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=knob_finish,
        name="wing",
    )
    knob.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.042, 0.0, 0.044)),
        material=latch_finish,
        name="grip_nub",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.100, 0.010, 0.110)),
        origin=Origin(xyz=(-0.050, -0.013, 0.0)),
        material=housing_paint,
        name="panel_skin",
    )
    service_door.visual(
        Cylinder(radius=0.008, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel",
    )
    service_door.visual(
        Box((0.016, 0.010, 0.030)),
        origin=Origin(xyz=(-0.072, -0.004, 0.0)),
        material=latch_finish,
        name="latch",
    )

    model.articulation(
        "housing_to_post",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.30,
            lower=-POST_TRAVEL,
            upper=0.0,
        ),
    )
    model.articulation(
        "post_to_knob",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "housing_to_service_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=service_door,
        origin=Origin(xyz=(0.054, 0.211, -0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    post = object_model.get_part("post")
    knob = object_model.get_part("knob")
    service_door = object_model.get_part("service_door")

    post_slide = object_model.get_articulation("housing_to_post")
    knob_spin = object_model.get_articulation("post_to_knob")
    door_hinge = object_model.get_articulation("housing_to_service_door")

    ctx.expect_origin_distance(
        post,
        housing,
        axes="xy",
        max_dist=0.001,
        name="post stays concentric with the ground housing",
    )
    ctx.expect_overlap(
        post,
        housing,
        axes="z",
        elem_a="body_shell",
        elem_b="housing_shell",
        min_overlap=0.47,
        name="raised post remains deeply inserted in the housing sleeve",
    )
    ctx.expect_gap(
        post,
        housing,
        axis="z",
        positive_elem="band_upper",
        negative_elem="housing_shell",
        min_gap=0.55,
        name="upper reflective band sits well above the collar at rest",
    )

    raised_post_pos = ctx.part_world_position(post)
    with ctx.pose({post_slide: POST_TRAVEL * -1.0}):
        lowered_post_pos = ctx.part_world_position(post)
        ctx.expect_overlap(
            post,
            housing,
            axes="z",
            elem_a="body_shell",
            elem_b="housing_shell",
            min_overlap=1.00,
            name="lowered post still remains captured by the housing sleeve",
        )

    ctx.check(
        "post retracts downward into the housing",
        raised_post_pos is not None
        and lowered_post_pos is not None
        and lowered_post_pos[2] < raised_post_pos[2] - 0.60,
        details=f"raised={raised_post_pos}, lowered={lowered_post_pos}",
    )

    rest_nub_center = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_nub"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        turned_nub_center = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_nub"))

    ctx.check(
        "locking knob turns about its short shaft",
        rest_nub_center is not None
        and turned_nub_center is not None
        and abs(turned_nub_center[0] - rest_nub_center[0]) > 0.03
        and abs(turned_nub_center[1] - rest_nub_center[1]) > 0.03,
        details=f"rest={rest_nub_center}, turned={turned_nub_center}",
    )

    closed_panel_center = _aabb_center(
        ctx.part_element_world_aabb(service_door, elem="panel_skin")
    )
    with ctx.pose({door_hinge: 1.10}):
        open_panel_center = _aabb_center(
            ctx.part_element_world_aabb(service_door, elem="panel_skin")
        )

    ctx.check(
        "service door swings clear of the collar",
        closed_panel_center is not None
        and open_panel_center is not None
        and open_panel_center[0] > closed_panel_center[0] + 0.03
        and open_panel_center[1] < closed_panel_center[1] - 0.02,
        details=f"closed={closed_panel_center}, open={open_panel_center}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_LENGTH = 0.180
BASE_WIDTH = 0.110
BASE_HEIGHT = 0.020
PEDESTAL_LENGTH = 0.068
PEDESTAL_WIDTH = 0.060
PEDESTAL_HEIGHT = 0.022
BASE_PIVOT_X = -0.020
BASE_PIVOT_Z = 0.046

LOWER_LINK_LENGTH = 0.120
UPPER_LINK_LENGTH = 0.108


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
    )
    pedestal = (
        cq.Workplane("XY")
        .box(
            PEDESTAL_LENGTH,
            PEDESTAL_WIDTH,
            PEDESTAL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-0.034, 0.0, BASE_HEIGHT))
        .edges("|Z")
        .fillet(0.006)
    )
    cheek_size = (0.020, 0.010, 0.026)
    cheek_z = BASE_HEIGHT + PEDESTAL_HEIGHT - 0.002
    cheek_x = BASE_PIVOT_X - 0.002
    cheek_0 = (
        cq.Workplane("XY")
        .box(*cheek_size, centered=(True, True, False))
        .translate((cheek_x, -0.015, cheek_z))
        .edges("|Z")
        .fillet(0.002)
    )
    cheek_1 = (
        cq.Workplane("XY")
        .box(*cheek_size, centered=(True, True, False))
        .translate((cheek_x, 0.015, cheek_z))
        .edges("|Z")
        .fillet(0.002)
    )
    switch_bezel = (
        cq.Workplane("XY")
        .box(0.036, 0.003, 0.020, centered=(True, True, False))
        .translate((0.034, BASE_WIDTH / 2.0 + 0.0015, 0.002))
        .edges("|X")
        .fillet(0.0012)
    )
    switch_recess = (
        cq.Workplane("XY")
        .box(0.028, 0.010, 0.018, centered=(True, True, False))
        .translate((0.034, BASE_WIDTH / 2.0 - 0.004, 0.004))
    )
    return foot.union(pedestal).union(cheek_0).union(cheek_1).union(switch_bezel).cut(switch_recess)


def _lower_link_shape() -> cq.Workplane:
    pivot = (
        cq.Workplane("XY")
        .cylinder(0.014, 0.007, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.020, 0.020, 0.010, centered=(False, True, True))
        .translate((0.006, 0.0, 0.0))
        .edges("|X")
        .fillet(0.002)
    )
    rail_length = LOWER_LINK_LENGTH - 0.034
    rail_x = 0.018
    rail_0 = (
        cq.Workplane("XY")
        .box(rail_length, 0.008, 0.013, centered=(False, True, True))
        .translate((rail_x, -0.014, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    rail_1 = (
        cq.Workplane("XY")
        .box(rail_length, 0.008, 0.013, centered=(False, True, True))
        .translate((rail_x, 0.014, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    tip_bridge = (
        cq.Workplane("XY")
        .box(0.020, 0.028, 0.012, centered=(True, True, True))
        .translate((LOWER_LINK_LENGTH - 0.020, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.002)
    )
    lug_x = LOWER_LINK_LENGTH - 0.010
    lug_0 = (
        cq.Workplane("XY")
        .box(0.020, 0.006, 0.026, centered=(True, True, True))
        .translate((lug_x, -0.013, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    lug_1 = (
        cq.Workplane("XY")
        .box(0.020, 0.006, 0.026, centered=(True, True, True))
        .translate((lug_x, 0.013, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    return pivot.union(shoulder).union(rail_0).union(rail_1).union(tip_bridge).union(lug_0).union(lug_1)


def _upper_link_shape() -> cq.Workplane:
    pivot = (
        cq.Workplane("XY")
        .cylinder(0.022, 0.007, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    spine = (
        cq.Workplane("XY")
        .box(0.086, 0.016, 0.012, centered=(False, True, True))
        .translate((0.006, 0.0, 0.0))
        .edges("|X")
        .fillet(0.002)
    )
    yoke_stem = (
        cq.Workplane("XY")
        .box(0.022, 0.066, 0.014, centered=(True, True, True))
        .translate((UPPER_LINK_LENGTH - 0.026, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.002)
    )
    cheek_x = UPPER_LINK_LENGTH - 0.008
    cheek_0 = (
        cq.Workplane("XY")
        .box(0.016, 0.006, 0.036, centered=(True, True, True))
        .translate((cheek_x, -0.035, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    cheek_1 = (
        cq.Workplane("XY")
        .box(0.016, 0.006, 0.036, centered=(True, True, True))
        .translate((cheek_x, 0.035, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    return pivot.union(spine).union(yoke_stem).union(cheek_0).union(cheek_1)


def _head_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("YZ")
        .circle(0.027)
        .extrude(0.078)
        .translate((0.010, 0.0, 0.0))
    )
    front_ring = (
        cq.Workplane("YZ")
        .circle(0.030)
        .circle(0.021)
        .extrude(0.006)
        .translate((0.081, 0.0, 0.0))
    )
    trunnion = (
        cq.Workplane("XY")
        .cylinder(0.068, 0.0055, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    neck = (
        cq.Workplane("XY")
        .box(0.012, 0.018, 0.012, centered=(False, True, True))
        .translate((-0.001, 0.0, 0.0))
        .edges("|X")
        .fillet(0.0015)
    )
    return shell.union(front_ring).union(trunnion).union(neck)


def _switch_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.028, 0.010, 0.016, centered=(True, True, True))
        .translate((0.0, 0.004, 0.0))
        .edges("|X")
        .fillet(0.0025)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_lamp")

    model.material("base_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("arm_satin", rgba=(0.63, 0.65, 0.68, 1.0))
    model.material("head_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("switch_black", rgba=(0.09, 0.09, 0.10, 1.0))
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_shell"),
        material="base_gray",
        name="base_shell",
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        mesh_from_cadquery(_lower_link_shape(), "lower_link"),
        material="arm_satin",
        name="lower_frame",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link"),
        material="arm_satin",
        name="upper_frame",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "head_shell"),
        material="head_black",
        name="head_shell",
    )

    switch = model.part("switch")
    switch.visual(
        mesh_from_cadquery(_switch_shape(), "rocker_switch"),
        material="switch_black",
        name="rocker_body",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_link,
        origin=Origin(xyz=(BASE_PIVOT_X, 0.0, 0.052), rpy=(0.0, -0.85, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.55,
            upper=0.65,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=upper_link,
        origin=Origin(xyz=(LOWER_LINK_LENGTH, 0.0, 0.0), rpy=(0.0, 0.45, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.90,
        ),
    )
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=head,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0), rpy=(0.0, 0.95, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-0.55,
            upper=0.80,
        ),
    )
    model.articulation(
        "switch_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=switch,
        origin=Origin(xyz=(0.034, 0.059, 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    switch = object_model.get_part("switch")
    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_hinge = object_model.get_articulation("head_hinge")
    switch_hinge = object_model.get_articulation("switch_hinge")

    ctx.expect_origin_distance(
        switch,
        base,
        axes="y",
        min_dist=0.050,
        max_dist=0.060,
        name="switch sits on the base side",
    )
    ctx.allow_overlap(
        base,
        object_model.get_part("lower_link"),
        elem_a="base_shell",
        elem_b="lower_frame",
        reason="The base hinge is represented with a compact clevis and a solid lower-link pivot block that intentionally nest at the shoulder joint.",
    )
    ctx.allow_overlap(
        object_model.get_part("lower_link"),
        object_model.get_part("upper_link"),
        elem_a="lower_frame",
        elem_b="upper_frame",
        reason="The elbow uses simplified solid fork and barrel geometry, so the joint bodies intentionally occupy the same local hinge envelope.",
    )
    ctx.allow_overlap(
        object_model.get_part("upper_link"),
        head,
        elem_a="upper_frame",
        elem_b="head_shell",
        reason="The head trunnion and end bracket are simplified as solid hinge proxies that intentionally embed within the same tilt-joint region.",
    )
    ctx.expect_overlap(
        switch,
        base,
        axes="xz",
        min_overlap=0.016,
        name="switch aligns with the side recess footprint",
    )

    rest_head = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    ctx.check(
        "rest head stays above the base",
        rest_head is not None and rest_head[2] > 0.135,
        details=f"rest_head={rest_head}",
    )

    with ctx.pose({base_hinge: 0.35, elbow_hinge: 0.45}):
        raised_head = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    ctx.check(
        "arm chain raises the spot head",
        rest_head is not None
        and raised_head is not None
        and raised_head[2] > rest_head[2] + 0.045,
        details=f"rest_head={rest_head}, raised_head={raised_head}",
    )

    with ctx.pose({head_hinge: -0.35}):
        lowered_shell = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    with ctx.pose({head_hinge: 0.55}):
        lifted_shell = _aabb_center(ctx.part_element_world_aabb(head, elem="head_shell"))
    ctx.check(
        "head hinge tilts the lamp head",
        lowered_shell is not None
        and lifted_shell is not None
        and lifted_shell[2] > lowered_shell[2] + 0.015,
        details=f"lowered_shell={lowered_shell}, lifted_shell={lifted_shell}",
    )

    with ctx.pose({switch_hinge: -0.25}):
        switch_low = _aabb_center(ctx.part_element_world_aabb(switch, elem="rocker_body"))
    with ctx.pose({switch_hinge: 0.25}):
        switch_high = _aabb_center(ctx.part_element_world_aabb(switch, elem="rocker_body"))
    ctx.check(
        "rocker switch pivots about its local hinge",
        switch_low is not None
        and switch_high is not None
        and abs(switch_high[2] - switch_low[2]) > 0.0015,
        details=f"switch_low={switch_low}, switch_high={switch_high}",
    )

    return ctx.report()


object_model = build_object_model()

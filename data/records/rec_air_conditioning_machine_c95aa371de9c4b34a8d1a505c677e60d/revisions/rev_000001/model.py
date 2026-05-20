from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.82
BODY_DEPTH = 0.22
BODY_HEIGHT = 0.60
SHELL_WALL = 0.004

PLINTH_WIDTH = 0.76
PLINTH_DEPTH = 0.17
PLINTH_HEIGHT = 0.025

INTAKE_OPENING_WIDTH = 0.72
INTAKE_OPENING_HEIGHT = 0.29
INTAKE_OPENING_DEPTH = 0.08
INTAKE_OPENING_CENTER_Z = 0.225

FILTER_COVER_WIDTH = 0.76
FILTER_COVER_HEIGHT = 0.34
FILTER_COVER_THICKNESS = 0.018
FILTER_HINGE_Z = 0.41

OUTLET_OPENING_WIDTH = 0.70
OUTLET_OPENING_DEPTH = 0.074
OUTLET_OPENING_CENTER_Y = -0.011
OUTLET_FLAP_DEPTH = 0.068
OUTLET_FLAP_THICKNESS = 0.012
OUTLET_HINGE_Y = OUTLET_OPENING_CENTER_Y + OUTLET_OPENING_DEPTH / 2.0
OUTLET_HINGE_Z = BODY_HEIGHT


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )

    upper_front_slope = (
        cq.Workplane("XY")
        .box(BODY_WIDTH + 0.08, BODY_DEPTH * 1.25, 0.18)
        .translate((0.0, -BODY_DEPTH / 2.0 + 0.03, BODY_HEIGHT - 0.01))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -31.0)
    )
    outer = outer.cut(upper_front_slope)

    inner = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 2.0 * SHELL_WALL, BODY_DEPTH - 2.0 * SHELL_WALL, BODY_HEIGHT - 2.0 * SHELL_WALL)
        .translate((0.0, 0.0, SHELL_WALL + (BODY_HEIGHT - 2.0 * SHELL_WALL) / 2.0))
    )
    outer = outer.cut(inner)

    intake_opening = (
        cq.Workplane("XY")
        .box(INTAKE_OPENING_WIDTH, INTAKE_OPENING_DEPTH, INTAKE_OPENING_HEIGHT)
        .translate(
            (
                0.0,
                -BODY_DEPTH / 2.0 + INTAKE_OPENING_DEPTH / 2.0 - 0.002,
                INTAKE_OPENING_CENTER_Z,
            )
        )
    )
    outer = outer.cut(intake_opening)

    outlet_opening = (
        cq.Workplane("XY")
        .box(OUTLET_OPENING_WIDTH, OUTLET_OPENING_DEPTH, 0.08)
        .translate((0.0, OUTLET_OPENING_CENTER_Y, BODY_HEIGHT - 0.01))
    )
    outer = outer.cut(outlet_opening)

    return outer


def _filter_cover_shape() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .box(FILTER_COVER_WIDTH, FILTER_COVER_THICKNESS, FILTER_COVER_HEIGHT)
        .translate((0.0, -FILTER_COVER_THICKNESS / 2.0, -FILTER_COVER_HEIGHT / 2.0))
    )
    slot_cutter = None
    slot_z = -0.075
    while slot_z >= -FILTER_COVER_HEIGHT + 0.075:
        cutter = (
            cq.Workplane("XY")
            .box(FILTER_COVER_WIDTH - 0.14, FILTER_COVER_THICKNESS + 0.012, 0.009)
            .translate((0.0, -FILTER_COVER_THICKNESS / 2.0, slot_z))
        )
        slot_cutter = cutter if slot_cutter is None else slot_cutter.union(cutter)
        slot_z -= 0.026
    if slot_cutter is not None:
        cover = cover.cut(slot_cutter)
    pull_lip = (
        cq.Workplane("XY")
        .box(0.26, 0.010, 0.018)
        .translate((0.0, -0.020, -FILTER_COVER_HEIGHT + 0.022))
    )
    return cover.union(pull_lip)


def _outlet_flap_shape() -> cq.Workplane:
    flap = (
        cq.Workplane("XY")
        .box(OUTLET_OPENING_WIDTH - 0.01, OUTLET_FLAP_DEPTH, OUTLET_FLAP_THICKNESS)
        .translate((0.0, -OUTLET_FLAP_DEPTH / 2.0, OUTLET_FLAP_THICKNESS / 2.0 + 0.001))
    )
    hinge_barrel = (
        cq.Workplane("XY")
        .cylinder(OUTLET_OPENING_WIDTH - 0.05, 0.0035)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, -0.003, 0.0035))
    )
    nose = (
        cq.Workplane("XY")
        .cylinder(OUTLET_OPENING_WIDTH - 0.01, 0.007)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, -OUTLET_FLAP_DEPTH + 0.005, OUTLET_FLAP_THICKNESS / 2.0 + 0.001))
    )
    support_ears = None
    for x_pos in (-0.36, 0.36):
        ear = (
            cq.Workplane("XY")
            .box(0.03, 0.012, 0.010)
            .translate((x_pos, 0.006, 0.005))
        )
        support_ears = ear if support_ears is None else support_ears.union(ear)
    return hinge_barrel.union(flap).union(nose).union(support_ears)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_console_air_conditioner")

    model.material("shell_white", color=(0.92, 0.93, 0.91))
    model.material("trim_gray", color=(0.32, 0.34, 0.36))
    model.material("display_black", color=(0.10, 0.12, 0.14))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material="shell_white",
        name="shell",
    )
    body.visual(
        Box((PLINTH_WIDTH, PLINTH_DEPTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material="trim_gray",
        name="plinth",
    )
    body.visual(
        Box((0.18, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + 0.004, 0.505)),
        material="display_black",
        name="display",
    )

    filter_cover = model.part("filter_cover")
    filter_cover.visual(
        mesh_from_cadquery(_filter_cover_shape(), "filter_cover_body"),
        material="shell_white",
        name="cover_body",
    )

    outlet_flap = model.part("outlet_flap")
    outlet_flap.visual(
        mesh_from_cadquery(_outlet_flap_shape(), "outlet_flap"),
        material="shell_white",
        name="flap",
    )

    model.articulation(
        "body_to_filter_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_cover,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0, FILTER_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=10.0, velocity=1.4),
    )
    model.articulation(
        "body_to_outlet_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=outlet_flap,
        origin=Origin(xyz=(0.0, OUTLET_HINGE_Y, OUTLET_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=5.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    filter_cover = object_model.get_part("filter_cover")
    outlet_flap = object_model.get_part("outlet_flap")
    filter_hinge = object_model.get_articulation("body_to_filter_cover")
    outlet_hinge = object_model.get_articulation("body_to_outlet_flap")

    ctx.expect_gap(
        body,
        filter_cover,
        axis="y",
        positive_elem="shell",
        negative_elem="cover_body",
        max_gap=0.006,
        max_penetration=0.0,
        name="filter cover sits just proud of the intake frame",
    )
    ctx.expect_overlap(
        filter_cover,
        body,
        axes="xz",
        elem_a="cover_body",
        elem_b="shell",
        min_overlap=0.28,
        name="filter cover spans the front intake section",
    )
    ctx.expect_gap(
        outlet_flap,
        body,
        axis="z",
        positive_elem="flap",
        negative_elem="shell",
        max_gap=0.008,
        max_penetration=0.0,
        name="outlet flap rests close to the top outlet deck",
    )
    ctx.expect_overlap(
        outlet_flap,
        body,
        axes="x",
        elem_a="flap",
        elem_b="shell",
        min_overlap=0.65,
        name="outlet flap spans the discharge width",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(filter_cover, elem="cover_body")
    closed_flap_aabb = ctx.part_element_world_aabb(outlet_flap, elem="flap")

    cover_upper = filter_hinge.motion_limits.upper if filter_hinge.motion_limits is not None else 1.0
    flap_upper = outlet_hinge.motion_limits.upper if outlet_hinge.motion_limits is not None else 1.0

    with ctx.pose({filter_hinge: cover_upper, outlet_hinge: flap_upper}):
        open_cover_aabb = ctx.part_element_world_aabb(filter_cover, elem="cover_body")
        open_flap_aabb = ctx.part_element_world_aabb(outlet_flap, elem="flap")

    ctx.check(
        "filter cover swings upward and outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][2] > closed_cover_aabb[0][2] + 0.18
        and open_cover_aabb[0][1] < closed_cover_aabb[0][1] - 0.06,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )
    ctx.check(
        "outlet flap lifts above the top opening",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.03
        and open_flap_aabb[0][1] > closed_flap_aabb[0][1] + 0.02,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

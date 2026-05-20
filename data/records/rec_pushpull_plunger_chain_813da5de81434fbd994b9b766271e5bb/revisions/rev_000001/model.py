from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GUIDE_LENGTH = 0.160
GUIDE_WIDTH = 0.090
GUIDE_FOOT_HEIGHT = 0.014
GUIDE_BODY_LENGTH = 0.148
GUIDE_BODY_WIDTH = 0.062
GUIDE_BODY_HEIGHT = 0.046
GUIDE_TOTAL_HEIGHT = GUIDE_FOOT_HEIGHT + GUIDE_BODY_HEIGHT

ROD_CENTER_Z = 0.034
GUIDE_SLOT_WIDTH = 0.024
GUIDE_SLOT_HEIGHT = 0.016

SLIDE_ORIGIN_X = -0.055
SLIDE_TRAVEL = 0.050

ROD_SHANK_LENGTH = 0.130
ROD_SHANK_WIDTH = 0.018
ROD_SHANK_HEIGHT = 0.012
ROD_HEAD_LENGTH = 0.030
ROD_HEAD_WIDTH = 0.032
ROD_HEAD_HEIGHT = 0.024
ROD_HEAD_Z = 0.003

FLAP_WIDTH = 0.020
FLAP_HEIGHT = 0.028
FLAP_THICKNESS = 0.004
FLAP_HINGE_RADIUS = 0.003
FLAP_UPPER = 1.05


def _guide_block_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        GUIDE_FOOT_HEIGHT,
        centered=(True, True, False),
    )
    bed = (
        cq.Workplane("XY")
        .box(0.140, 0.034, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, GUIDE_FOOT_HEIGHT))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(0.148, 0.013, 0.034, centered=(True, True, False))
        .translate((0.0, 0.0245, GUIDE_FOOT_HEIGHT))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(0.148, 0.013, 0.034, centered=(True, True, False))
        .translate((0.0, -0.0245, GUIDE_FOOT_HEIGHT))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, GUIDE_BODY_WIDTH, 0.018, centered=(True, True, False))
        .translate((-0.065, 0.0, GUIDE_FOOT_HEIGHT + 0.016))
    )
    front_mouth = (
        cq.Workplane("XY")
        .box(0.012, GUIDE_BODY_WIDTH, 0.014, centered=(True, True, False))
        .translate((0.069, 0.0, GUIDE_FOOT_HEIGHT))
    )
    return foot.union(bed).union(left_rail).union(right_rail).union(rear_bridge).union(front_mouth)


def _plunger_rod_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").box(
        ROD_SHANK_LENGTH,
        ROD_SHANK_WIDTH,
        ROD_SHANK_HEIGHT,
        centered=(False, True, True),
    )

    carrier_block = (
        cq.Workplane("XY")
        .box(
            0.018,
            0.024,
            0.018,
            centered=(False, True, False),
        )
        .translate((ROD_SHANK_LENGTH - 0.004, 0.0, 0.0))
    )
    hinge_pedestal = (
        cq.Workplane("XY")
        .box(
            0.014,
            0.012,
            0.012,
            centered=(False, True, False),
        )
        .translate((ROD_SHANK_LENGTH + 0.014, 0.0, 0.006))
    )

    return rod.union(carrier_block).union(hinge_pedestal)


def _output_flap_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(FLAP_THICKNESS, FLAP_WIDTH, FLAP_HEIGHT)
        .translate((FLAP_THICKNESS / 2.0, 0.0, -FLAP_HEIGHT / 2.0))
    )
    hinge_barrel = cq.Workplane("XZ").circle(FLAP_HINGE_RADIUS).extrude(0.005, both=True)
    lower_lip = (
        cq.Workplane("XY")
        .box(0.012, FLAP_WIDTH * 0.55, 0.004)
        .translate((0.007, 0.0, -FLAP_HEIGHT + 0.004))
    )
    return panel.union(hinge_barrel).union(lower_lip)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guide_plunger_with_output_flap")

    model.material("guide_cast", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("rod_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("flap_orange", rgba=(0.90, 0.53, 0.16, 1.0))

    guide_block = model.part("guide_block")
    guide_block.visual(
        mesh_from_cadquery(_guide_block_shape(), "guide_block"),
        material="guide_cast",
        name="guide_body",
    )
    guide_block.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_TOTAL_HEIGHT)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOTAL_HEIGHT / 2.0)),
    )

    plunger_rod = model.part("plunger_rod")
    plunger_rod.visual(
        mesh_from_cadquery(_plunger_rod_shape(), "plunger_rod"),
        material="rod_steel",
        name="rod_body",
    )
    plunger_rod.inertial = Inertial.from_geometry(
        Box((ROD_SHANK_LENGTH + ROD_HEAD_LENGTH, ROD_HEAD_WIDTH, ROD_HEAD_HEIGHT)),
        mass=0.45,
        origin=Origin(xyz=(0.074, 0.0, ROD_HEAD_Z)),
    )

    output_flap = model.part("output_flap")
    output_flap.visual(
        mesh_from_cadquery(_output_flap_shape(), "output_flap"),
        material="flap_orange",
        name="flap_panel",
    )
    output_flap.inertial = Inertial.from_geometry(
        Box((0.012, FLAP_WIDTH, FLAP_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(0.006, 0.0, -FLAP_HEIGHT / 2.0)),
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=guide_block,
        child=plunger_rod,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, ROD_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=90.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "rod_to_flap",
        ArticulationType.REVOLUTE,
        parent=plunger_rod,
        child=output_flap,
        origin=Origin(xyz=(ROD_SHANK_LENGTH + 0.031, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FLAP_UPPER,
            effort=8.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    guide_block = object_model.get_part("guide_block")
    plunger_rod = object_model.get_part("plunger_rod")
    output_flap = object_model.get_part("output_flap")
    guide_slide = object_model.get_articulation("guide_slide")
    rod_to_flap = object_model.get_articulation("rod_to_flap")

    ctx.check(
        "prismatic slide uses forward x axis",
        tuple(guide_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={guide_slide.axis}",
    )
    ctx.check(
        "flap hinge pitches about carried y axis",
        tuple(rod_to_flap.axis) == (0.0, -1.0, 0.0),
        details=f"axis={rod_to_flap.axis}",
    )

    ctx.expect_overlap(
        plunger_rod,
        guide_block,
        axes="x",
        elem_a="rod_body",
        elem_b="guide_body",
        min_overlap=0.128,
        name="rod remains deeply inserted when retracted",
    )
    ctx.expect_gap(
        output_flap,
        guide_block,
        axis="x",
        positive_elem="flap_panel",
        negative_elem="guide_body",
        min_gap=0.006,
        name="closed flap sits ahead of guide front",
    )

    rod_rest = ctx.part_world_position(plunger_rod)
    flap_rest = ctx.part_world_position(output_flap)
    with ctx.pose({guide_slide: SLIDE_TRAVEL, rod_to_flap: 0.0}):
        rod_extended = ctx.part_world_position(plunger_rod)
        flap_carried = ctx.part_world_position(output_flap)
        ctx.expect_overlap(
            plunger_rod,
            guide_block,
            axes="x",
            elem_a="rod_body",
            elem_b="guide_body",
            min_overlap=0.078,
            name="rod keeps retained insertion at full extension",
        )

    slide_delta = None
    carried_delta = None
    if rod_rest is not None and rod_extended is not None:
        slide_delta = rod_extended[0] - rod_rest[0]
    if flap_rest is not None and flap_carried is not None:
        carried_delta = flap_carried[0] - flap_rest[0]

    ctx.check(
        "positive slide extends rod forward",
        slide_delta is not None and slide_delta > 0.049,
        details=f"delta_x={slide_delta}",
    )
    ctx.check(
        "flap hinge is carried by the rod",
        slide_delta is not None
        and carried_delta is not None
        and isclose(carried_delta, slide_delta, abs_tol=1e-4),
        details=f"rod_delta={slide_delta}, flap_delta={carried_delta}",
    )

    with ctx.pose({guide_slide: 0.030, rod_to_flap: 0.0}):
        flap_closed_aabb = ctx.part_element_world_aabb(output_flap, elem="flap_panel")
    with ctx.pose({guide_slide: 0.030, rod_to_flap: FLAP_UPPER}):
        flap_open_aabb = ctx.part_element_world_aabb(output_flap, elem="flap_panel")

    flap_closed_center = (
        _aabb_center(flap_closed_aabb) if flap_closed_aabb is not None else None
    )
    flap_open_center = (
        _aabb_center(flap_open_aabb) if flap_open_aabb is not None else None
    )
    ctx.check(
        "positive hinge swings flap outward and upward",
        flap_closed_center is not None
        and flap_open_center is not None
        and flap_open_center[0] > flap_closed_center[0] + 0.008
        and flap_open_center[2] > flap_closed_center[2] + 0.004,
        details=f"closed_center={flap_closed_center}, open_center={flap_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

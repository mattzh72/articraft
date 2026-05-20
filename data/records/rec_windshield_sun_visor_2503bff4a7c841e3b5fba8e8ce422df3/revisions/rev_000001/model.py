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


VISOR_LENGTH = 0.340
VISOR_DEPTH = 0.148
VISOR_THICKNESS = 0.028
VISOR_WALL = 0.005
VISOR_CORNER = 0.018
VISOR_Y_OFFSET = 0.010
VISOR_TOP_GAP = 0.006

EXTENDER_LENGTH = 0.220
EXTENDER_DEPTH = 0.108
EXTENDER_THICKNESS = 0.007
EXTENDER_START_X = 0.055
EXTENDER_TRAVEL = 0.085

ROD_RADIUS = 0.0035
ROD_LENGTH = 0.355
ROD_COLLAR_RADIUS = 0.006
ROD_COLLAR_LENGTH = 0.014
SWIVEL_PIN_RADIUS = 0.005
SWIVEL_PIN_LENGTH = 0.012

CLIP_X = 0.356


def _visor_shell_shape() -> object:
    outer = (
        cq.Workplane("XY")
        .box(VISOR_LENGTH, VISOR_DEPTH, VISOR_THICKNESS)
        .edges("|Z")
        .fillet(VISOR_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            VISOR_LENGTH - (2.0 * VISOR_WALL),
            VISOR_DEPTH - (2.0 * VISOR_WALL),
            VISOR_THICKNESS - (2.0 * VISOR_WALL),
        )
        .edges("|Z")
        .fillet(VISOR_CORNER - VISOR_WALL)
    )
    outboard_opening = cq.Workplane("XY").box(
        0.024,
        VISOR_DEPTH - (2.0 * VISOR_WALL),
        VISOR_THICKNESS - (2.0 * VISOR_WALL) - 0.004,
    ).translate((VISOR_LENGTH * 0.5 - 0.012, 0.0, 0.0))
    return outer.cut(inner).cut(outboard_opening)


def _extender_shape() -> object:
    return (
        cq.Workplane("XY")
        .box(EXTENDER_LENGTH, EXTENDER_DEPTH, EXTENDER_THICKNESS)
        .edges("|Z")
        .fillet(0.006)
    )


def _pivot_bracket_cover_shape() -> object:
    pad = (
        cq.Workplane("XY")
        .box(0.030, 0.026, 0.010)
        .translate((0.010, 0.000, 0.000))
    )
    boss = cq.Workplane("XY").circle(0.0135).extrude(0.010).translate((-0.004, 0.000, -0.005))
    return pad.union(boss)


def _retaining_clip_cover_shape() -> object:
    spine = (
        cq.Workplane("XY")
        .box(0.012, 0.014, 0.018)
        .translate((0.006, 0.000, 0.000))
    )
    top_arm = cq.Workplane("XY").box(0.020, 0.012, 0.004).translate((-0.004, 0.000, 0.007))
    bottom_arm = cq.Workplane("XY").box(0.020, 0.012, 0.004).translate((-0.004, 0.000, -0.007))
    lead_in = cq.Workplane("YZ").circle(0.0035).extrude(0.006).translate((-0.014, 0.000, 0.000))
    return spine.union(top_arm).union(bottom_arm).union(lead_in)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_sun_visor")

    headliner = model.material("headliner", rgba=(0.78, 0.77, 0.72, 1.0))
    bracket_plastic = model.material("bracket_plastic", rgba=(0.28, 0.29, 0.30, 1.0))
    rod_metal = model.material("rod_metal", rgba=(0.58, 0.59, 0.61, 1.0))
    visor_vinyl = model.material("visor_vinyl", rgba=(0.72, 0.69, 0.61, 1.0))
    extender_vinyl = model.material("extender_vinyl", rgba=(0.64, 0.61, 0.55, 1.0))

    mount = model.part("mount")
    mount.visual(
        Box((0.430, 0.038, 0.006)),
        origin=Origin(xyz=(0.185, 0.000, 0.014)),
        material=headliner,
        name="roof_strip",
    )
    mount.visual(
        Box((0.040, 0.028, 0.008)),
        origin=Origin(xyz=(0.012, 0.000, 0.010)),
        material=bracket_plastic,
        name="pivot_bracket",
    )
    mount.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=bracket_plastic,
        name="pivot_socket",
    )
    mount.visual(
        mesh_from_cadquery(_pivot_bracket_cover_shape(), "pivot_bracket_cover"),
        origin=Origin(xyz=(0.010, 0.000, 0.010)),
        material=bracket_plastic,
        name="pivot_bracket_cover",
    )
    mount.visual(
        Box((0.006, 0.014, 0.018)),
        origin=Origin(xyz=(CLIP_X + 0.006, 0.000, 0.000)),
        material=bracket_plastic,
        name="clip_spine",
    )
    mount.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(CLIP_X + 0.003, 0.000, 0.007)),
        material=bracket_plastic,
        name="clip_post",
    )
    mount.visual(
        Box((0.014, 0.012, 0.004)),
        origin=Origin(xyz=(CLIP_X - 0.003, 0.000, 0.007)),
        material=bracket_plastic,
        name="clip_top_lip",
    )
    mount.visual(
        Box((0.014, 0.012, 0.004)),
        origin=Origin(xyz=(CLIP_X - 0.003, 0.000, -0.007)),
        material=bracket_plastic,
        name="clip_bottom_lip",
    )
    mount.visual(
        mesh_from_cadquery(_retaining_clip_cover_shape(), "retaining_clip_cover"),
        origin=Origin(xyz=(CLIP_X + 0.014, 0.000, 0.004)),
        material=bracket_plastic,
        name="clip_cover",
    )

    rod = model.part("rod")
    rod.visual(
        Cylinder(radius=SWIVEL_PIN_RADIUS, length=SWIVEL_PIN_LENGTH),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=rod_metal,
        name="swivel_pin",
    )
    rod.visual(
        Cylinder(radius=ROD_COLLAR_RADIUS, length=0.016),
        origin=Origin(xyz=(0.026, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rod_metal,
        name="pivot_collar",
    )
    rod.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        origin=Origin(xyz=(ROD_LENGTH * 0.5, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rod_metal,
        name="hinge_rod",
    )

    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_visor_shell_shape(), "visor_shell"),
        origin=Origin(
            xyz=(
                VISOR_LENGTH * 0.5,
                VISOR_Y_OFFSET + (VISOR_DEPTH * 0.5),
                -VISOR_TOP_GAP - (VISOR_THICKNESS * 0.5),
            )
        ),
        material=visor_vinyl,
        name="visor_shell",
    )
    visor.visual(
        Box((0.032, VISOR_Y_OFFSET, VISOR_TOP_GAP - ROD_RADIUS)),
        origin=Origin(xyz=(0.062, VISOR_Y_OFFSET * 0.5, (-VISOR_TOP_GAP - ROD_RADIUS) * 0.5)),
        material=visor_vinyl,
        name="hinge_saddle_0",
    )
    visor.visual(
        Box((0.032, VISOR_Y_OFFSET, VISOR_TOP_GAP - ROD_RADIUS)),
        origin=Origin(xyz=(0.152, VISOR_Y_OFFSET * 0.5, (-VISOR_TOP_GAP - ROD_RADIUS) * 0.5)),
        material=visor_vinyl,
        name="hinge_saddle_1",
    )
    visor.visual(
        Box((0.028, VISOR_Y_OFFSET, VISOR_TOP_GAP - ROD_RADIUS)),
        origin=Origin(xyz=(0.244, VISOR_Y_OFFSET * 0.5, (-VISOR_TOP_GAP - ROD_RADIUS) * 0.5)),
        material=visor_vinyl,
        name="hinge_saddle_2",
    )

    extender = model.part("extender")
    extender.visual(
        mesh_from_cadquery(_extender_shape(), "visor_extender"),
        origin=Origin(xyz=(EXTENDER_LENGTH * 0.5, 0.000, 0.000)),
        material=extender_vinyl,
        name="extender_panel",
    )
    extender.visual(
        Box((0.170, 0.015, 0.003)),
        origin=Origin(xyz=(0.085, 0.0615, 0.000)),
        material=extender_vinyl,
        name="guide_rail_0",
    )
    extender.visual(
        Box((0.170, 0.015, 0.003)),
        origin=Origin(xyz=(0.085, -0.0615, 0.000)),
        material=extender_vinyl,
        name="guide_rail_1",
    )

    swing_joint = model.articulation(
        "mount_to_rod",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=rod,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    tilt_joint = model.articulation(
        "rod_to_visor",
        ArticulationType.REVOLUTE,
        parent=rod,
        child=visor,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    slide_joint = model.articulation(
        "visor_to_extender",
        ArticulationType.PRISMATIC,
        parent=visor,
        child=extender,
        origin=Origin(
            xyz=(
                EXTENDER_START_X,
                VISOR_Y_OFFSET + (VISOR_DEPTH * 0.5),
                -VISOR_TOP_GAP - (VISOR_THICKNESS * 0.5),
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=EXTENDER_TRAVEL,
        ),
    )

    swing_joint.meta["qc_samples"] = [0.0, math.radians(45.0), math.radians(90.0)]
    tilt_joint.meta["qc_samples"] = [0.0, math.radians(45.0), math.radians(95.0)]
    slide_joint.meta["qc_samples"] = [0.0, EXTENDER_TRAVEL * 0.5, EXTENDER_TRAVEL]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount = object_model.get_part("mount")
    rod = object_model.get_part("rod")
    visor = object_model.get_part("visor")
    extender = object_model.get_part("extender")

    swing = object_model.get_articulation("mount_to_rod")
    tilt = object_model.get_articulation("rod_to_visor")
    slide = object_model.get_articulation("visor_to_extender")

    ctx.expect_gap(
        visor,
        rod,
        axis="y",
        positive_elem="visor_shell",
        negative_elem="hinge_rod",
        min_gap=0.005,
        max_gap=0.012,
        name="visor panel sits clearly aft of the hinge rod",
    )
    ctx.expect_gap(
        visor,
        mount,
        axis="y",
        positive_elem="visor_shell",
        negative_elem="clip_spine",
        min_gap=0.002,
        name="visor panel stays separate from the retaining clip hardware",
    )
    ctx.expect_within(
        extender,
        visor,
        axes="yz",
        inner_elem="extender_panel",
        outer_elem="visor_shell",
        margin=0.0,
        name="collapsed extender stays nested between visor skins",
    )
    ctx.expect_overlap(
        extender,
        visor,
        axes="x",
        elem_a="extender_panel",
        elem_b="visor_shell",
        min_overlap=0.210,
        name="collapsed extender remains deeply inserted in the visor sleeve",
    )

    rest_visor_aabb = ctx.part_element_world_aabb(visor, elem="visor_shell")
    rest_extender_aabb = ctx.part_element_world_aabb(extender, elem="extender_panel")
    rest_center = _aabb_center(rest_visor_aabb)

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            lowered_aabb = ctx.part_element_world_aabb(visor, elem="visor_shell")
        ctx.check(
            "visor rotates downward from the headliner",
            rest_visor_aabb is not None
            and lowered_aabb is not None
            and lowered_aabb[0][2] < rest_visor_aabb[0][2] - 0.090,
            details=f"rest={rest_visor_aabb}, lowered={lowered_aabb}",
        )

    swing_limits = swing.motion_limits
    if swing_limits is not None and swing_limits.upper is not None:
        with ctx.pose({swing: swing_limits.upper}):
            swung_center = _aabb_center(ctx.part_element_world_aabb(visor, elem="visor_shell"))
        ctx.check(
            "visor swivels sideways at the bracket",
            rest_center is not None
            and swung_center is not None
            and swung_center[1] > rest_center[1] + 0.070,
            details=f"rest_center={rest_center}, swung_center={swung_center}",
        )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_within(
                extender,
                visor,
                axes="yz",
                inner_elem="extender_panel",
                outer_elem="visor_shell",
                margin=0.0,
                name="extended extender stays centered inside the visor sleeve",
            )
            ctx.expect_overlap(
                extender,
                visor,
                axes="x",
                elem_a="extender_panel",
                elem_b="visor_shell",
                min_overlap=0.190,
                name="extended extender keeps retained insertion in the visor sleeve",
            )
            extended_extender_aabb = ctx.part_element_world_aabb(extender, elem="extender_panel")
        ctx.check(
            "extender slides outward from the visor body",
            rest_extender_aabb is not None
            and extended_extender_aabb is not None
            and extended_extender_aabb[1][0] > rest_extender_aabb[1][0] + 0.060,
            details=f"rest={rest_extender_aabb}, extended={extended_extender_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

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


LOWER_TUBE_BOTTOM = 0.085
LOWER_TUBE_LENGTH = 0.670
LOWER_TUBE_TOP = LOWER_TUBE_BOTTOM + LOWER_TUBE_LENGTH
MAST_TRAVEL = 0.320


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    black_metal = model.material("black_powder_coated_metal", rgba=(0.02, 0.022, 0.024, 1.0))
    graphite_metal = model.material("graphite_sheet_metal", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_slot = model.material("shadowed_perforations", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_drawer = model.material("satin_black_drawer", rgba=(0.035, 0.036, 0.038, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    polished = model.material("brushed_inner_mast", rgba=(0.45, 0.46, 0.44, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.205, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=black_metal,
        name="weighted_disk",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=black_metal,
        name="raised_hub",
    )
    lower_tube_shape = (
        cq.Workplane("XY")
        .circle(0.026)
        .circle(0.0185)
        .extrude(LOWER_TUBE_LENGTH)
    )
    base.visual(
        mesh_from_cadquery(lower_tube_shape, "lower_tube", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, LOWER_TUBE_BOTTOM)),
        material=black_metal,
        name="lower_tube",
    )
    clamp_collar_shape = (
        cq.Workplane("XY")
        .circle(0.039)
        .circle(0.013)
        .extrude(0.025)
    )
    base.visual(
        mesh_from_cadquery(clamp_collar_shape, "clamp_collar", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, LOWER_TUBE_TOP - 0.0105)),
        material=black_metal,
        name="clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.063, 0.0, LOWER_TUBE_TOP + 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="clamp_knob",
    )
    for index, (x, y) in enumerate(((0.145, 0.105), (-0.145, 0.105), (0.145, -0.105), (-0.145, -0.105))):
        base.visual(
            Cylinder(radius=0.027, length=0.008),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0135, length=1.020),
        # The moving tube extends well below its joint frame so it remains
        # retained inside the lower sleeve at full height.
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=polished,
        name="inner_tube",
    )
    mast.visual(
        Box((0.034, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        material=black_metal,
        name="head_stem",
    )
    mast.visual(
        Box((0.046, 0.160, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=black_metal,
        name="yoke_bridge",
    )
    for name, y in (("yoke_ear_0", -0.073), ("yoke_ear_1", 0.073)):
        mast.visual(
            Box((0.044, 0.014, 0.090)),
            origin=Origin(xyz=(0.0, y, 0.600)),
            material=black_metal,
            name=name,
        )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.015, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="hinge_barrel",
    )
    desk.visual(
        Box((0.070, 0.110, 0.030)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=black_metal,
        name="hinge_plate",
    )
    desk.visual(
        Box((0.014, 0.620, 0.340)),
        origin=Origin(xyz=(-0.075, 0.0, 0.100)),
        material=graphite_metal,
        name="panel_sheet",
    )
    desk.visual(
        Box((0.030, 0.014, 0.340)),
        origin=Origin(xyz=(-0.075, -0.313, 0.100)),
        material=black_metal,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.030, 0.014, 0.340)),
        origin=Origin(xyz=(-0.075, 0.313, 0.100)),
        material=black_metal,
        name="side_flange_1",
    )
    desk.visual(
        Cylinder(radius=0.010, length=0.620),
        origin=Origin(xyz=(-0.075, 0.0, 0.275), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="rolled_top_lip",
    )
    desk.visual(
        Box((0.160, 0.620, 0.024)),
        origin=Origin(xyz=(-0.145, 0.0, -0.082)),
        material=black_metal,
        name="deep_bottom_shelf",
    )
    desk.visual(
        Box((0.018, 0.620, 0.070)),
        origin=Origin(xyz=(-0.225, 0.0, -0.046)),
        material=black_metal,
        name="front_retaining_lip",
    )
    for name, y in (("shelf_cheek_0", -0.310), ("shelf_cheek_1", 0.310)):
        desk.visual(
            Box((0.160, 0.018, 0.070)),
            origin=Origin(xyz=(-0.145, y, -0.046)),
            material=black_metal,
            name=name,
        )
    for row, z in enumerate((0.065, 0.145, 0.215)):
        for column, y in enumerate((-0.200, -0.075, 0.075, 0.200)):
            desk.visual(
                Box((0.004, 0.072, 0.014)),
                origin=Origin(xyz=(-0.084, y, z)),
                material=dark_slot,
                name=f"slot_{row}_{column}",
            )
    desk.visual(
        Box((0.250, 0.018, 0.018)),
        origin=Origin(xyz=(-0.125, -0.197, -0.103)),
        material=black_metal,
        name="drawer_runner_0",
    )
    desk.visual(
        Box((0.250, 0.018, 0.018)),
        origin=Origin(xyz=(-0.125, 0.197, -0.103)),
        material=black_metal,
        name="drawer_runner_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.200, 0.360, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=satin_drawer,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.200, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, -0.180, 0.001)),
        material=satin_drawer,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.200, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.180, 0.001)),
        material=satin_drawer,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.014, 0.360, 0.038)),
        origin=Origin(xyz=(-0.100, 0.0, 0.003)),
        material=satin_drawer,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.360, 0.028)),
        origin=Origin(xyz=(0.100, 0.0, -0.002)),
        material=satin_drawer,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.220, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.188, 0.023)),
        material=black_metal,
        name="side_runner_0",
    )
    drawer.visual(
        Box((0.220, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.188, 0.023)),
        material=black_metal,
        name="side_runner_1",
    )
    drawer.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(-0.114, 0.0, 0.004), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="finger_pull",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TUBE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.20, lower=0.0, upper=MAST_TRAVEL),
    )
    model.articulation(
        "desk_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.25, upper=0.60),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(-0.145, 0.0, -0.137)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    drawer = object_model.get_part("drawer")
    mast_slide = object_model.get_articulation("mast_slide")
    desk_tilt = object_model.get_articulation("desk_tilt")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.allow_overlap(
        base,
        mast,
        elem_a="clamp_collar",
        elem_b="inner_tube",
        reason="The clamp collar is modeled as lightly gripping the telescoping mast tube.",
    )
    ctx.allow_overlap(
        desk,
        drawer,
        elem_a="drawer_runner_0",
        elem_b="side_runner_0",
        reason="The short under-desk runner is modeled as a captured sliding rail with slight local overlap.",
    )
    ctx.allow_overlap(
        desk,
        drawer,
        elem_a="drawer_runner_1",
        elem_b="side_runner_1",
        reason="The short under-desk runner is modeled as a captured sliding rail with slight local overlap.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_tube",
        margin=0.002,
        name="mast stays centered in lower tube",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_tube",
        min_overlap=0.200,
        name="collapsed mast retains long insertion",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="clamp_collar",
        min_overlap=0.020,
        name="mast passes through clamping collar",
    )

    mast_rest = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: MAST_TRAVEL}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_tube",
            min_overlap=0.140,
            name="extended mast remains captured in sleeve",
        )
        mast_extended = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward",
        mast_rest is not None and mast_extended is not None and mast_extended[2] > mast_rest[2] + 0.25,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    panel_rest = ctx.part_element_world_aabb(desk, elem="panel_sheet")
    with ctx.pose({desk_tilt: 0.50}):
        panel_tilted = ctx.part_element_world_aabb(desk, elem="panel_sheet")
    ctx.check(
        "desk tilts on horizontal axis",
        panel_rest is not None
        and panel_tilted is not None
        and (panel_tilted[1][0] - panel_rest[1][0]) > 0.035,
        details=f"rest_panel_aabb={panel_rest}, tilted_panel_aabb={panel_tilted}",
    )

    ctx.expect_gap(
        desk,
        drawer,
        axis="z",
        positive_elem="drawer_runner_0",
        negative_elem="side_runner_0",
        max_gap=0.002,
        max_penetration=0.004,
        name="drawer runner is captured in fixed runner",
    )
    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.120}):
        ctx.expect_overlap(
            drawer,
            desk,
            axes="x",
            elem_a="side_runner_0",
            elem_b="drawer_runner_0",
            min_overlap=0.045,
            name="extended drawer remains on runner",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides out toward player",
        drawer_rest is not None and drawer_extended is not None and drawer_extended[0] < drawer_rest[0] - 0.09,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    return ctx.report()


object_model = build_object_model()

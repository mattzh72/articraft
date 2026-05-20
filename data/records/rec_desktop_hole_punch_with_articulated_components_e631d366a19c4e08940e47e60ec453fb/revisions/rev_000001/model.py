from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_LENGTH = 0.34
BASE_WIDTH = 0.078
BASE_THICKNESS = 0.008
HINGE_X = -0.145
HINGE_Z = 0.042
HANDLE_LENGTH = 0.258
HANDLE_OPEN = math.radians(62.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_hole_punch")

    body_finish = model.material("body_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.58, 0.61, 0.65, 1.0))
    guide_finish = model.material("guide_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.58, 0.16, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=body_finish,
        name="bottom_plate",
    )
    base.visual(
        Box((0.090, BASE_WIDTH, 0.030)),
        origin=Origin(xyz=(-0.125, 0.0, 0.023)),
        material=body_finish,
        name="rear_housing",
    )
    base.visual(
        Box((0.220, 0.050, 0.006)),
        origin=Origin(xyz=(0.030, 0.0, 0.015)),
        material=body_finish,
        name="throat_table",
    )
    base.visual(
        Box((0.190, 0.006, 0.010)),
        origin=Origin(xyz=(0.040, 0.022, 0.011)),
        material=body_finish,
        name="side_web_0",
    )
    base.visual(
        Box((0.190, 0.006, 0.010)),
        origin=Origin(xyz=(0.040, -0.022, 0.011)),
        material=body_finish,
        name="side_web_1",
    )
    base.visual(
        Box((0.035, 0.014, 0.036)),
        origin=Origin(xyz=(HINGE_X, 0.031, 0.026)),
        material=body_finish,
        name="hinge_cheek_0",
    )
    base.visual(
        Box((0.035, 0.014, 0.036)),
        origin=Origin(xyz=(HINGE_X, -0.031, 0.026)),
        material=body_finish,
        name="hinge_cheek_1",
    )
    base.visual(
        Box((0.050, 0.034, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, 0.021)),
        material=body_finish,
        name="die_block",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.062, 0.0, 0.030)),
        material=trim_finish,
        name="die_insert",
    )
    base.visual(
        Box((0.230, 0.010, 0.014)),
        origin=Origin(xyz=(0.040, -0.026, 0.015)),
        material=trim_finish,
        name="front_rail",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.056, 0.046, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, 0.012)),
        material=handle_finish,
        name="rear_riser",
    )
    handle.visual(
        Box((0.184, 0.040, 0.012)),
        origin=Origin(xyz=(0.138, 0.0, 0.026)),
        material=handle_finish,
        name="grip_spine",
    )
    handle.visual(
        Box((0.150, 0.024, 0.004)),
        origin=Origin(xyz=(0.148, 0.0, 0.034)),
        material=trim_finish,
        name="grip_pad",
    )
    handle.visual(
        Box((0.050, 0.028, 0.020)),
        origin=Origin(xyz=(0.198, 0.0, 0.011)),
        material=handle_finish,
        name="nose_bridge",
    )
    handle.visual(
        Box((0.060, 0.034, 0.020)),
        origin=Origin(xyz=(0.224, 0.0, -0.002)),
        material=handle_finish,
        name="front_nose",
    )
    handle.visual(
        Box((0.022, 0.014, 0.004)),
        origin=Origin(xyz=(0.214, 0.0, -0.006)),
        material=trim_finish,
        name="punch_shoe",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.4,
            lower=0.0,
            upper=HANDLE_OPEN,
        ),
    )

    paper_guide = model.part("paper_guide")
    paper_guide.visual(
        Box((0.042, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=guide_finish,
        name="guide_saddle",
    )
    paper_guide.visual(
        Box((0.042, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.007, 0.000)),
        material=guide_finish,
        name="rear_jaw",
    )
    paper_guide.visual(
        Box((0.042, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.007, 0.000)),
        material=guide_finish,
        name="front_jaw",
    )
    paper_guide.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(0.025, -0.012, 0.013)),
        material=guide_finish,
        name="stop_plate",
    )
    paper_guide.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.028, -0.010, 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="knob_boss",
    )

    model.articulation(
        "base_to_paper_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=paper_guide,
        origin=Origin(xyz=(0.040, -0.026, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.15,
            lower=-0.085,
            upper=0.085,
        ),
    )

    stop_knob = model.part("stop_knob")
    stop_knob.visual(
        Cylinder(radius=0.0025, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_finish,
        name="shaft",
    )
    stop_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.012,
                0.006,
                body_style="mushroom",
                top_diameter=0.009,
                edge_radius=0.0008,
                center=False,
            ),
            "paper_guide_stop_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=knob_finish,
        name="knob_shell",
    )
    stop_knob.visual(
        Box((0.004, 0.006, 0.003)),
        origin=Origin(xyz=(0.0045, 0.0, 0.010)),
        material=knob_finish,
        name="knob_tab",
    )

    model.articulation(
        "paper_guide_to_stop_knob",
        ArticulationType.CONTINUOUS,
        parent=paper_guide,
        child=stop_knob,
        origin=Origin(xyz=(0.031, -0.013, 0.011), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    paper_guide = object_model.get_part("paper_guide")
    stop_knob = object_model.get_part("stop_knob")
    handle_hinge = object_model.get_articulation("base_to_handle")
    guide_slide = object_model.get_articulation("base_to_paper_guide")
    knob_spin = object_model.get_articulation("paper_guide_to_stop_knob")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="punch_shoe",
        negative_elem="die_insert",
        min_gap=0.0005,
        max_gap=0.010,
        name="punch shoe hovers just above die at rest",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        elem_a="punch_shoe",
        elem_b="die_insert",
        min_overlap=0.010,
        name="punch shoe stays centered over die",
    )

    rest_aabb = ctx.part_element_world_aabb(handle, elem="front_nose")
    with ctx.pose({handle_hinge: HANDLE_OPEN}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="punch_shoe",
            negative_elem="die_insert",
            min_gap=0.060,
            name="opened handle lifts punch shoe well clear of die",
        )
        open_aabb = ctx.part_element_world_aabb(handle, elem="front_nose")

    rest_top = rest_aabb[1][2] if rest_aabb is not None else None
    open_top = open_aabb[1][2] if open_aabb is not None else None
    ctx.check(
        "front of handle rises on opening",
        rest_top is not None and open_top is not None and open_top > rest_top + 0.070,
        details=f"rest_top={rest_top}, open_top={open_top}",
    )

    ctx.expect_gap(
        paper_guide,
        base,
        axis="y",
        positive_elem="rear_jaw",
        negative_elem="front_rail",
        min_gap=0.0,
        max_gap=0.0005,
        name="guide rear jaw tracks the back of the fence rail",
    )
    ctx.expect_gap(
        base,
        paper_guide,
        axis="y",
        positive_elem="front_rail",
        negative_elem="front_jaw",
        min_gap=0.0,
        max_gap=0.0005,
        name="guide front jaw tracks the front of the fence rail",
    )
    ctx.expect_overlap(
        paper_guide,
        base,
        axes="x",
        elem_a="guide_saddle",
        elem_b="front_rail",
        min_overlap=0.040,
        name="guide saddle remains engaged on the rail at rest",
    )

    guide_rest = ctx.part_world_position(paper_guide)
    with ctx.pose({guide_slide: 0.085}):
        ctx.expect_overlap(
            paper_guide,
            base,
            axes="x",
            elem_a="guide_saddle",
            elem_b="front_rail",
            min_overlap=0.040,
            name="guide saddle remains engaged on the rail at full travel",
        )
        guide_extended = ctx.part_world_position(paper_guide)

    ctx.check(
        "paper guide slides forward along the reach",
        guide_rest is not None
        and guide_extended is not None
        and guide_extended[0] > guide_rest[0] + 0.070,
        details=f"rest={guide_rest}, extended={guide_extended}",
    )

    tab_rest = ctx.part_element_world_aabb(stop_knob, elem="knob_tab")
    with ctx.pose({knob_spin: 1.3}):
        tab_rotated = ctx.part_element_world_aabb(stop_knob, elem="knob_tab")

    rest_center = (
        None
        if tab_rest is None
        else tuple((tab_rest[0][i] + tab_rest[1][i]) / 2.0 for i in range(3))
    )
    rotated_center = (
        None
        if tab_rotated is None
        else tuple((tab_rotated[0][i] + tab_rotated[1][i]) / 2.0 for i in range(3))
    )
    ctx.check(
        "stop knob visibly rotates about its side shaft",
        rest_center is not None
        and rotated_center is not None
        and abs(rotated_center[2] - rest_center[2]) > 0.004,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()

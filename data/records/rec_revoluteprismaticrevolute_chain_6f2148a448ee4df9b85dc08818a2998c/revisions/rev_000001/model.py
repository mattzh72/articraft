from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_hinge_slider_wrist")

    dark_steel = Material("dark parkerized steel", color=(0.10, 0.11, 0.12, 1.0))
    blue_anodized = Material("blue anodized aluminum", color=(0.07, 0.19, 0.36, 1.0))
    brushed_steel = Material("brushed slider steel", color=(0.68, 0.70, 0.68, 1.0))
    black_rubber = Material("black rubber grip", color=(0.015, 0.015, 0.014, 1.0))
    brass_bushing = Material("warm brass bushings", color=(0.74, 0.56, 0.25, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.30, 0.17, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, 0.080)),
        material=dark_steel,
        name="top_plate",
    )
    top_support.visual(
        Box((0.072, 0.022, 0.110)),
        origin=Origin(xyz=(0.0, 0.063, 0.018)),
        material=dark_steel,
        name="clevis_lug_0",
    )
    top_support.visual(
        Box((0.072, 0.022, 0.110)),
        origin=Origin(xyz=(0.0, -0.063, 0.018)),
        material=dark_steel,
        name="clevis_lug_1",
    )
    top_support.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, 0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass_bushing,
        name="hinge_washer_0",
    )
    top_support.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, -0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass_bushing,
        name="hinge_washer_1",
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        Cylinder(radius=0.022, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_anodized,
        name="hinge_barrel",
    )
    pivot_frame.visual(
        Box((0.038, 0.060, 0.225)),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=blue_anodized,
        name="swing_spine",
    )
    pivot_frame.visual(
        Box((0.070, 0.018, 0.044)),
        origin=Origin(xyz=(0.035, 0.035, -0.238)),
        material=blue_anodized,
        name="guide_root_0",
    )
    pivot_frame.visual(
        Box((0.070, 0.018, 0.044)),
        origin=Origin(xyz=(0.035, -0.035, -0.238)),
        material=blue_anodized,
        name="guide_root_1",
    )
    pivot_frame.visual(
        Box((0.160, 0.012, 0.036)),
        origin=Origin(xyz=(0.105, 0.038, -0.250)),
        material=blue_anodized,
        name="guide_rail_0",
    )
    pivot_frame.visual(
        Box((0.160, 0.012, 0.036)),
        origin=Origin(xyz=(0.105, -0.038, -0.250)),
        material=blue_anodized,
        name="guide_rail_1",
    )
    pivot_frame.visual(
        Box((0.160, 0.084, 0.012)),
        origin=Origin(xyz=(0.105, 0.0, -0.232)),
        material=blue_anodized,
        name="guide_top_cap",
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.175, 0.040, 0.024)),
        origin=Origin(xyz=(0.0475, 0.0, 0.0)),
        material=brushed_steel,
        name="slide_tongue",
    )
    slider.visual(
        Box((0.046, 0.016, 0.034)),
        origin=Origin(xyz=(0.155, 0.026, 0.0)),
        material=brushed_steel,
        name="wrist_fork_0",
    )
    slider.visual(
        Box((0.046, 0.016, 0.034)),
        origin=Origin(xyz=(0.155, -0.026, 0.0)),
        material=brushed_steel,
        name="wrist_fork_1",
    )

    wrist_tab = model.part("wrist_tab")
    wrist_tab.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass_bushing,
        name="wrist_knuckle",
    )
    wrist_tab.visual(
        Box((0.094, 0.030, 0.010)),
        origin=Origin(xyz=(0.053, 0.0, -0.012)),
        material=black_rubber,
        name="tab_plate",
    )
    wrist_tab.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.100, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rounded_tip",
    )

    model.articulation(
        "top_to_frame",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=pivot_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=slider,
        origin=Origin(xyz=(0.055, 0.0, -0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.070),
    )
    model.articulation(
        "slider_to_wrist",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=wrist_tab,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    pivot_frame = object_model.get_part("pivot_frame")
    slider = object_model.get_part("slider")
    wrist_tab = object_model.get_part("wrist_tab")
    frame_joint = object_model.get_articulation("top_to_frame")
    slide_joint = object_model.get_articulation("frame_to_slider")
    wrist_joint = object_model.get_articulation("slider_to_wrist")

    ctx.expect_overlap(
        pivot_frame,
        top_support,
        axes="xz",
        elem_a="hinge_barrel",
        elem_b="clevis_lug_0",
        min_overlap=0.035,
        name="frame hinge barrel shares the clevis bore projection",
    )
    ctx.expect_within(
        slider,
        pivot_frame,
        axes="y",
        inner_elem="slide_tongue",
        outer_elem="guide_top_cap",
        margin=0.0,
        name="slider tongue is laterally captured by the guide",
    )
    ctx.expect_overlap(
        slider,
        pivot_frame,
        axes="x",
        elem_a="slide_tongue",
        elem_b="guide_rail_0",
        min_overlap=0.14,
        name="retracted slider remains engaged in the short guide",
    )
    ctx.expect_overlap(
        wrist_tab,
        slider,
        axes="xz",
        elem_a="wrist_knuckle",
        elem_b="wrist_fork_0",
        min_overlap=0.020,
        name="wrist knuckle is held between the fork ears",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slide_joint: 0.070}):
        extended_slider_pos = ctx.part_world_position(slider)
        ctx.expect_overlap(
            slider,
            pivot_frame,
            axes="x",
            elem_a="slide_tongue",
            elem_b="guide_rail_0",
            min_overlap=0.095,
            name="extended slider still has retained insertion",
        )
    ctx.check(
        "slider extends along the frame rail",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.060,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    with ctx.pose({frame_joint: 0.55}):
        swung_slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "pivot frame swings under the top support",
        rest_slider_pos is not None
        and swung_slider_pos is not None
        and swung_slider_pos[0] < rest_slider_pos[0] - 0.10,
        details=f"rest={rest_slider_pos}, swung={swung_slider_pos}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(wrist_tab, elem="tab_plate")
    with ctx.pose({wrist_joint: 0.70}):
        dipped_tab_aabb = ctx.part_element_world_aabb(wrist_tab, elem="tab_plate")
    ctx.check(
        "wrist tab pivots about its knuckle",
        rest_tab_aabb is not None
        and dipped_tab_aabb is not None
        and dipped_tab_aabb[0][2] < rest_tab_aabb[0][2] - 0.035,
        details=f"rest={rest_tab_aabb}, dipped={dipped_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

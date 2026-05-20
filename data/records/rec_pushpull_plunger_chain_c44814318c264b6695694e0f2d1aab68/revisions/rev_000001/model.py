from __future__ import annotations

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

import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_plunger_lever")

    cast_iron = Material("satin_blackened_casting", color=(0.07, 0.08, 0.08, 1.0))
    dark_metal = Material("dark_blued_steel", color=(0.16, 0.17, 0.18, 1.0))
    polished = Material("polished_steel", color=(0.68, 0.70, 0.70, 1.0))
    rubber = Material("matte_red_rubber", color=(0.55, 0.04, 0.03, 1.0))
    brass = Material("brushed_brass_tab", color=(0.78, 0.58, 0.26, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        Box((0.240, 0.100, 0.015)),
        origin=Origin(xyz=(0.000, 0.000, 0.0075)),
        material=cast_iron,
        name="base_plate",
    )
    guide.visual(
        Box((0.180, 0.060, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
        material=cast_iron,
        name="mounting_plinth",
    )
    guide.visual(
        Box((0.180, 0.080, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.052)),
        material=cast_iron,
        name="bottom_wall",
    )
    guide.visual(
        Box((0.180, 0.080, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.118)),
        material=cast_iron,
        name="top_wall",
    )
    guide.visual(
        Box((0.180, 0.014, 0.080)),
        origin=Origin(xyz=(0.000, 0.033, 0.085)),
        material=cast_iron,
        name="side_wall_0",
    )
    guide.visual(
        Box((0.180, 0.014, 0.080)),
        origin=Origin(xyz=(0.000, -0.033, 0.085)),
        material=cast_iron,
        name="side_wall_1",
    )
    guide.visual(
        Box((0.039, 0.012, 0.026)),
        origin=Origin(xyz=(0.1085, 0.033, 0.052)),
        material=cast_iron,
        name="hinge_ear_0",
    )
    guide.visual(
        Box((0.039, 0.012, 0.026)),
        origin=Origin(xyz=(0.1085, -0.033, 0.052)),
        material=cast_iron,
        name="hinge_ear_1",
    )
    guide.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.126, 0.031, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="ear_barrel_0",
    )
    guide.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.126, -0.031, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="ear_barrel_1",
    )
    guide.visual(
        Box((0.060, 0.018, 0.017)),
        origin=Origin(xyz=(-0.030, 0.000, 0.0675)),
        material=dark_metal,
        name="lower_bearing_pad",
    )

    rod = model.part("plunger_rod")
    rod.visual(
        Cylinder(radius=0.009, length=0.236),
        origin=Origin(xyz=(-0.005, 0.000, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="shaft",
    )
    rod.visual(
        Box((0.026, 0.026, 0.026)),
        origin=Origin(xyz=(0.110, 0.000, 0.085)),
        material=dark_metal,
        name="square_tip",
    )
    rod.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-0.130, 0.000, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="push_button",
    )

    tab = model.part("hinged_tab")
    tab.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tab_barrel",
    )
    tab.visual(
        Box((0.008, 0.044, 0.070)),
        origin=Origin(xyz=(0.004, 0.000, 0.035)),
        material=brass,
        name="tab_plate",
    )
    tab.visual(
        Box((0.003, 0.026, 0.020)),
        origin=Origin(xyz=(-0.0015, 0.000, 0.034)),
        material=dark_metal,
        name="wear_pad",
    )

    model.articulation(
        "guide_to_rod",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=rod,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.035),
    )
    model.articulation(
        "guide_to_tab",
        ArticulationType.REVOLUTE,
        parent=guide,
        child=tab,
        origin=Origin(xyz=(0.126, 0.000, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.85),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    guide = object_model.get_part("guide_body")
    rod = object_model.get_part("plunger_rod")
    tab = object_model.get_part("hinged_tab")
    rod_slide = object_model.get_articulation("guide_to_rod")
    tab_hinge = object_model.get_articulation("guide_to_tab")

    ctx.expect_contact(
        rod,
        tab,
        elem_a="square_tip",
        elem_b="wear_pad",
        contact_tol=0.001,
        name="squared plunger tip bears on the tab wear pad",
    )
    ctx.expect_gap(
        guide,
        rod,
        axis="z",
        positive_elem="top_wall",
        negative_elem="shaft",
        min_gap=0.010,
        max_gap=0.030,
        name="shaft clears the top of the rectangular guide bore",
    )
    ctx.expect_gap(
        rod,
        guide,
        axis="z",
        positive_elem="shaft",
        negative_elem="bottom_wall",
        min_gap=0.010,
        max_gap=0.030,
        name="shaft clears the bottom of the rectangular guide bore",
    )
    ctx.expect_overlap(
        rod,
        guide,
        axes="x",
        elem_a="shaft",
        elem_b="top_wall",
        min_overlap=0.150,
        name="shaft remains well captured through the guide length",
    )

    rest_rod_position = ctx.part_world_position(rod)
    with ctx.pose({rod_slide: 0.035}):
        extended_rod_position = ctx.part_world_position(rod)
        ctx.expect_overlap(
            rod,
            guide,
            axes="x",
            elem_a="shaft",
            elem_b="top_wall",
            min_overlap=0.130,
            name="extended shaft is still retained in the guide",
        )

    ctx.check(
        "prismatic joint drives the plunger forward",
        rest_rod_position is not None
        and extended_rod_position is not None
        and extended_rod_position[0] > rest_rod_position[0] + 0.030,
        details=f"rest={rest_rod_position}, extended={extended_rod_position}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
    with ctx.pose({tab_hinge: 0.70}):
        swung_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")

    ctx.check(
        "revolute joint swings the tab outward from the front face",
        rest_tab_aabb is not None
        and swung_tab_aabb is not None
        and swung_tab_aabb[1][0] > rest_tab_aabb[1][0] + 0.020,
        details=f"rest={rest_tab_aabb}, swung={swung_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

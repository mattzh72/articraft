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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _housing_shell() -> cq.Workplane:
    """Low, wide open-top plastic tub with a front filter slot."""
    width = 0.82
    depth = 0.23
    height = 0.155
    wall = 0.018
    bottom = 0.024

    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    inner_cavity = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height, centered=(True, True, False))
        .translate((0.0, 0.0, bottom))
    )
    filter_cut = (
        cq.Workplane("XY")
        .box(0.690, 0.070, 0.078, centered=(True, True, True))
        .translate((0.0, -depth / 2.0, 0.096))
    )
    return outer.cut(inner_cavity).cut(filter_cut)


def _lid_shell() -> cq.Workplane:
    """Thin rear-hinged service lid; local origin is on the rear hinge line."""
    width = 0.790
    depth = 0.235
    thickness = 0.018
    lid = (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, False))
        .translate((0.0, -depth / 2.0, 0.0))
        .edges("|Z")
        .fillet(0.010)
    )
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_sill_air_purifier")

    warm_plastic = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.055, 1.0))
    filter_media = model.material("dark_filter_media", rgba=(0.015, 0.018, 0.017, 1.0))
    brushed_metal = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell", tolerance=0.0015),
        material=warm_plastic,
        name="housing_shell",
    )

    # Interior front guide rails for the removable pre-filter cartridge.
    housing.visual(
        Box((0.780, 0.170, 0.006)),
        origin=Origin(xyz=(0.0, -0.035, 0.0555)),
        material=dark_plastic,
        name="lower_guide_rail",
    )
    housing.visual(
        Box((0.780, 0.170, 0.006)),
        origin=Origin(xyz=(0.0, -0.035, 0.138)),
        material=dark_plastic,
        name="upper_guide_rail",
    )

    # Rubber feet that keep the shallow purifier stable on a window sill.
    for index, x in enumerate((-0.330, 0.330)):
        for y in (-0.070, 0.070):
            housing.visual(
                Box((0.080, 0.035, 0.008)),
                origin=Origin(xyz=(x, y, -0.004)),
                material=rubber,
                name=f"rubber_foot_{index}_{0 if y < 0 else 1}",
            )

    # Two fixed hinge leaves bolted to the rear wall; the rotating barrels live on the lid.
    for index, x in enumerate((-0.245, 0.245)):
        housing.visual(
            Box((0.130, 0.004, 0.030)),
            origin=Origin(xyz=(x, 0.117, 0.130)),
            material=brushed_metal,
            name=f"fixed_hinge_leaf_{index}",
        )

    # Subtle rear window gasket/seal where the unit meets the window frame.
    housing.visual(
        Box((0.760, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.120, 0.055)),
        material=rubber,
        name="rear_window_gasket",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.0012),
        material=warm_plastic,
        name="lid_shell",
    )
    lid_grille = SlotPatternPanelGeometry(
        (0.580, 0.120),
        0.003,
        slot_size=(0.060, 0.006),
        pitch=(0.080, 0.018),
        frame=0.012,
        corner_radius=0.006,
        stagger=True,
    )
    lid.visual(
        mesh_from_geometry(lid_grille, "top_service_grille"),
        origin=Origin(xyz=(0.0, -0.118, 0.019)),
        material=dark_plastic,
        name="top_service_grille",
    )
    for index, x in enumerate((-0.245, 0.245)):
        lid.visual(
            Box((0.130, 0.060, 0.006)),
            origin=Origin(xyz=(x, -0.030, 0.004)),
            material=brushed_metal,
            name=f"moving_hinge_leaf_{index}",
        )
        lid.visual(
            Cylinder(radius=0.007, length=0.120),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"hinge_barrel_{index}",
        )

    pre_filter = model.part("pre_filter")
    filter_panel = SlotPatternPanelGeometry(
        (0.660, 0.066),
        0.006,
        slot_size=(0.045, 0.0045),
        pitch=(0.062, 0.012),
        frame=0.010,
        corner_radius=0.004,
        stagger=True,
    )
    pre_filter.visual(
        mesh_from_geometry(filter_panel, "pre_filter_face"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=filter_media,
        name="filter_face",
    )
    pre_filter.visual(
        Box((0.700, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, 0.038)),
        material=dark_plastic,
        name="top_filter_frame",
    )
    pre_filter.visual(
        Box((0.700, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, -0.038)),
        material=dark_plastic,
        name="bottom_filter_frame",
    )
    pre_filter.visual(
        Box((0.010, 0.010, 0.084)),
        origin=Origin(xyz=(-0.350, -0.006, 0.0)),
        material=dark_plastic,
        name="filter_frame_0",
    )
    pre_filter.visual(
        Box((0.010, 0.010, 0.084)),
        origin=Origin(xyz=(0.350, -0.006, 0.0)),
        material=dark_plastic,
        name="filter_frame_1",
    )
    pre_filter.visual(
        Box((0.660, 0.215, 0.005)),
        origin=Origin(xyz=(0.0, 0.1035, -0.034)),
        material=dark_plastic,
        name="lower_runner",
    )
    pre_filter.visual(
        Box((0.660, 0.215, 0.005)),
        origin=Origin(xyz=(0.0, 0.1035, 0.034)),
        material=dark_plastic,
        name="upper_runner",
    )
    pre_filter.visual(
        Box((0.160, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=dark_plastic,
        name="pull_handle",
    )

    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, 0.124, 0.155)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "housing_to_pre_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=pre_filter,
        origin=Origin(xyz=(0.0, -0.120, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.22, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    pre_filter = object_model.get_part("pre_filter")
    lid_hinge = object_model.get_articulation("housing_to_lid")
    filter_slide = object_model.get_articulation("housing_to_pre_filter")

    ctx.check(
        "top lid uses a rear revolute hinge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper >= 1.0,
        details=f"type={lid_hinge.articulation_type}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "two visible barrel hinges are present",
        len([v for v in lid.visuals if v.name and v.name.startswith("hinge_barrel_")]) == 2,
        details=tuple(v.name for v in lid.visuals),
    )
    ctx.expect_gap(
        lid,
        housing,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="housing_shell",
        min_gap=0.0,
        max_gap=0.002,
        name="closed lid sits just above the housing rim",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "positive lid motion opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    ctx.check(
        "front pre-filter uses an outward prismatic slide",
        filter_slide.articulation_type == ArticulationType.PRISMATIC
        and filter_slide.motion_limits is not None
        and filter_slide.motion_limits.upper is not None
        and filter_slide.motion_limits.upper >= 0.11,
        details=f"type={filter_slide.articulation_type}, limits={filter_slide.motion_limits}",
    )
    ctx.expect_gap(
        pre_filter,
        housing,
        axis="z",
        positive_elem="lower_runner",
        negative_elem="lower_guide_rail",
        max_gap=0.001,
        max_penetration=0.00001,
        name="lower runner rides above lower guide rail",
    )
    ctx.expect_gap(
        housing,
        pre_filter,
        axis="z",
        positive_elem="upper_guide_rail",
        negative_elem="upper_runner",
        min_gap=0.001,
        max_gap=0.008,
        name="upper guide rail caps the filter runner",
    )
    ctx.expect_overlap(
        pre_filter,
        housing,
        axes="y",
        elem_a="lower_runner",
        elem_b="lower_guide_rail",
        min_overlap=0.12,
        name="closed filter runner is engaged in the rails",
    )
    rest_filter_pos = ctx.part_world_position(pre_filter)
    with ctx.pose({filter_slide: 0.120}):
        ctx.expect_overlap(
            pre_filter,
            housing,
            axes="y",
            elem_a="lower_runner",
            elem_b="lower_guide_rail",
            min_overlap=0.07,
            name="extended filter still has retained rail insertion",
        )
        extended_filter_pos = ctx.part_world_position(pre_filter)
    ctx.check(
        "positive filter motion slides out the front",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[1] < rest_filter_pos[1] - 0.105,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    return ctx.report()


object_model = build_object_model()

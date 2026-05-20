from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_rail_module")

    cast_iron = Material("painted_cast_iron", color=(0.18, 0.22, 0.24, 1.0))
    rail_steel = Material("ground_rail_steel", color=(0.62, 0.64, 0.62, 1.0))
    carriage_paint = Material("carriage_blue", color=(0.08, 0.19, 0.36, 1.0))
    bearing_black = Material("black_bearing_housing", color=(0.04, 0.045, 0.05, 1.0))
    spindle_steel = Material("brushed_spindle_steel", color=(0.72, 0.72, 0.68, 1.0))
    brass = Material("brass_index_tab", color=(0.86, 0.58, 0.20, 1.0))

    base = model.part("base_guide")
    base.visual(
        Box((0.86, 0.22, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.78, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.055, 0.0375)),
        material=rail_steel,
        name="guide_0",
    )
    base.visual(
        Box((0.78, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.055, 0.0375)),
        material=rail_steel,
        name="guide_1",
    )
    base.visual(
        Box((0.80, 0.066, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=Material("oiled_center_channel", color=(0.02, 0.025, 0.026, 1.0)),
        name="center_channel",
    )
    for index, x in enumerate((-0.405, 0.405)):
        base.visual(
            Box((0.035, 0.19, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.050)),
            material=cast_iron,
            name=f"end_stop_{index}",
        )

    carriage = model.part("carriage")
    for index, y in enumerate((-0.055, 0.055)):
        carriage.visual(
            Box((0.180, 0.035, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0275)),
            material=carriage_paint,
            name=f"side_way_{index}",
        )
        carriage.visual(
            Box((0.160, 0.026, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.005)),
            material=rail_steel,
            name=f"slide_shoe_{index}",
        )

    for index, x in enumerate((-0.0725, 0.0725)):
        carriage.visual(
            Box((0.035, 0.140, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0345)),
            material=carriage_paint,
            name=f"cross_bridge_{index}",
        )

    bearing_flange = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.060, -0.005), (0.060, 0.005)],
        inner_profile=[(0.031, -0.005), (0.031, 0.005)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    carriage.visual(
        mesh_from_geometry(bearing_flange, "bearing_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=bearing_black,
        name="bearing_flange",
    )

    bearing_collar = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.047, -0.025), (0.047, 0.025)],
        inner_profile=[(0.030, -0.025), (0.030, 0.025)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    carriage.visual(
        mesh_from_geometry(bearing_collar, "bearing_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=bearing_black,
        name="bearing_collar",
    )
    for index, y in enumerate((-0.043, 0.043)):
        carriage.visual(
            Box((0.105, 0.016, 0.033)),
            origin=Origin(xyz=(0.0, y, 0.0735)),
            material=carriage_paint,
            name=f"collar_web_{index}",
        )

    spindle = model.part("spindle")
    spindle_body = LatheGeometry(
        [
            (0.000, 0.025),
            (0.038, 0.025),
            (0.038, 0.038),
            (0.030, 0.046),
            (0.030, 0.104),
            (0.020, 0.118),
            (0.000, 0.118),
        ],
        segments=72,
        closed=True,
    )
    spindle.visual(
        mesh_from_geometry(spindle_body, "spindle_body"),
        origin=Origin(),
        material=spindle_steel,
        name="spindle_body",
    )
    spindle.visual(
        Box((0.020, 0.010, 0.018)),
        origin=Origin(xyz=(0.038, 0.0, 0.082)),
        material=brass,
        name="drive_tab",
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.220, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.440),
    )
    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    rail_slide = object_model.get_articulation("rail_slide")
    spindle_spin = object_model.get_articulation("spindle_spin")

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="slide_shoe_0",
        negative_elem="guide_0",
        max_gap=0.0005,
        max_penetration=0.0,
        name="carriage shoe sits on rail guide",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="slide_shoe_1",
        elem_b="guide_1",
        min_overlap=0.020,
        name="second shoe remains registered on guide",
    )
    ctx.expect_within(
        spindle,
        carriage,
        axes="xy",
        inner_elem="spindle_body",
        outer_elem="bearing_collar",
        margin=0.012,
        name="spindle is centered inside bearing collar footprint",
    )
    ctx.expect_gap(
        spindle,
        carriage,
        axis="z",
        positive_elem="spindle_body",
        negative_elem="bearing_collar",
        max_gap=0.0005,
        max_penetration=0.0,
        name="spindle shoulder seats on bearing collar",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({rail_slide: 0.440}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="slide_shoe_0",
            elem_b="guide_0",
            min_overlap=0.120,
            name="extended carriage stays engaged on rail",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "rail slide moves carriage along x",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.40
        and abs(extended_pos[1] - rest_pos[1]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(spindle, elem="drive_tab")
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        turned_tab_aabb = ctx.part_element_world_aabb(spindle, elem="drive_tab")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    rest_tab = _aabb_center(rest_tab_aabb)
    turned_tab = _aabb_center(turned_tab_aabb)
    ctx.check(
        "spindle revolute joint turns the drive tab about vertical axis",
        rest_tab is not None
        and turned_tab is not None
        and turned_tab[1] > rest_tab[1] + 0.025
        and turned_tab[0] < rest_tab[0] - 0.025,
        details=f"rest_tab={rest_tab}, turned_tab={turned_tab}",
    )

    return ctx.report()


object_model = build_object_model()

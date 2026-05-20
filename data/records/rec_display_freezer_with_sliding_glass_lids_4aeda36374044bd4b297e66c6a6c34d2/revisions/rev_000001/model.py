from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_display_freezer")

    enamel = model.material("white_powder_coated_enamel", rgba=(0.92, 0.94, 0.93, 1.0))
    dark = model.material("dark_recessed_liner", rgba=(0.05, 0.06, 0.065, 1.0))
    rubber = model.material("black_rubber_gasket", rgba=(0.01, 0.012, 0.014, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.55, 0.82, 0.95, 0.38))
    safety_glass = model.material("slightly_darker_hatch_glass", rgba=(0.42, 0.76, 0.92, 0.48))
    label = model.material("small_blue_energy_label", rgba=(0.05, 0.18, 0.72, 1.0))

    cabinet = model.part("cabinet")

    # A low, broad insulated chest with an open recessed well rather than a solid
    # block.  The glass lids ride just above the aluminum top frame.
    cabinet.visual(
        Box((2.26, 1.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="toe_plinth",
    )
    cabinet.visual(
        Box((2.20, 0.085, 0.66)),
        origin=Origin(xyz=(0.0, -0.5075, 0.41)),
        material=enamel,
        name="front_wall",
    )
    cabinet.visual(
        Box((2.20, 0.085, 0.66)),
        origin=Origin(xyz=(0.0, 0.5075, 0.41)),
        material=enamel,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.085, 0.94, 0.66)),
        origin=Origin(xyz=(-1.0575, 0.0, 0.41)),
        material=enamel,
        name="end_wall_0",
    )
    cabinet.visual(
        Box((0.085, 0.94, 0.66)),
        origin=Origin(xyz=(1.0575, 0.0, 0.41)),
        material=enamel,
        name="end_wall_1",
    )
    cabinet.visual(
        Box((2.03, 0.77, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=dark,
        name="freezer_floor",
    )
    cabinet.visual(
        Box((2.00, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, -0.385, 0.185)),
        material=dark,
        name="front_liner",
    )
    cabinet.visual(
        Box((2.00, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, 0.385, 0.185)),
        material=dark,
        name="rear_liner",
    )
    cabinet.visual(
        Box((0.018, 0.74, 0.18)),
        origin=Origin(xyz=(-1.0, 0.0, 0.185)),
        material=dark,
        name="end_liner_0",
    )
    cabinet.visual(
        Box((0.018, 0.74, 0.18)),
        origin=Origin(xyz=(1.0, 0.0, 0.185)),
        material=dark,
        name="end_liner_1",
    )
    cabinet.visual(
        Box((2.24, 0.070, 0.055)),
        origin=Origin(xyz=(0.0, -0.505, 0.7675)),
        material=aluminum,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((2.24, 0.070, 0.055)),
        origin=Origin(xyz=(0.0, 0.505, 0.7675)),
        material=aluminum,
        name="rear_top_rail",
    )
    cabinet.visual(
        Box((0.070, 0.94, 0.055)),
        origin=Origin(xyz=(-1.085, 0.0, 0.7675)),
        material=aluminum,
        name="end_top_rail_0",
    )
    cabinet.visual(
        Box((0.070, 0.94, 0.055)),
        origin=Origin(xyz=(1.085, 0.0, 0.7675)),
        material=aluminum,
        name="end_top_rail_1",
    )
    cabinet.visual(
        Box((0.026, 0.99, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=aluminum,
        name="center_track_divider",
    )
    cabinet.visual(
        Box((2.06, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, -0.440, 0.801)),
        material=rubber,
        name="front_slide_gasket",
    )
    cabinet.visual(
        Box((2.06, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.440, 0.801)),
        material=rubber,
        name="rear_slide_gasket",
    )
    cabinet.visual(
        Box((1.04, 0.028, 0.046)),
        origin=Origin(xyz=(-0.52, -0.422, 0.818)),
        material=aluminum,
        name="upper_front_track",
    )
    cabinet.visual(
        Box((1.04, 0.028, 0.046)),
        origin=Origin(xyz=(-0.52, 0.422, 0.818)),
        material=aluminum,
        name="upper_rear_track",
    )
    cabinet.visual(
        Box((0.18, 0.005, 0.10)),
        origin=Origin(xyz=(-0.82, -0.552, 0.45)),
        material=label,
        name="energy_label",
    )

    lid_0 = model.part("lid_0")
    # Upper sliding glass panel with a rectangular quick-access opening.
    lid_0.visual(
        Box((1.04, 0.045, 0.034)),
        origin=Origin(xyz=(0.0, -0.3875, 0.0)),
        material=aluminum,
        name="front_frame",
    )
    lid_0.visual(
        Box((1.04, 0.045, 0.034)),
        origin=Origin(xyz=(0.0, 0.3875, 0.0)),
        material=aluminum,
        name="rear_frame",
    )
    lid_0.visual(
        Box((0.045, 0.82, 0.034)),
        origin=Origin(xyz=(-0.4975, 0.0, 0.0)),
        material=aluminum,
        name="side_frame_0",
    )
    lid_0.visual(
        Box((0.045, 0.82, 0.034)),
        origin=Origin(xyz=(0.4975, 0.0, 0.0)),
        material=aluminum,
        name="side_frame_1",
    )
    lid_0.visual(
        Box((0.18, 0.72, 0.018)),
        origin=Origin(xyz=(-0.385, 0.0, 0.0)),
        material=glass,
        name="glass_side_0",
    )
    lid_0.visual(
        Box((0.48, 0.72, 0.018)),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material=glass,
        name="glass_side_1",
    )
    lid_0.visual(
        Box((0.32, 0.120, 0.018)),
        origin=Origin(xyz=(-0.15, -0.310, 0.0)),
        material=glass,
        name="glass_front_strip",
    )
    lid_0.visual(
        Box((0.32, 0.160, 0.018)),
        origin=Origin(xyz=(-0.15, 0.290, 0.0)),
        material=glass,
        name="glass_rear_strip",
    )
    lid_0.visual(
        Box((0.022, 0.505, 0.026)),
        origin=Origin(xyz=(-0.321, -0.02, 0.004)),
        material=aluminum,
        name="hatch_side_trim_0",
    )
    lid_0.visual(
        Box((0.022, 0.505, 0.026)),
        origin=Origin(xyz=(0.021, -0.02, 0.004)),
        material=aluminum,
        name="hatch_side_trim_1",
    )
    lid_0.visual(
        Box((0.364, 0.022, 0.026)),
        origin=Origin(xyz=(-0.15, -0.281, 0.004)),
        material=aluminum,
        name="hatch_front_trim",
    )
    lid_0.visual(
        Box((0.364, 0.022, 0.026)),
        origin=Origin(xyz=(-0.15, 0.241, 0.004)),
        material=aluminum,
        name="hatch_rear_trim",
    )
    lid_0.visual(
        Box((0.34, 0.052, 0.012)),
        origin=Origin(xyz=(-0.15, 0.255, 0.021)),
        material=aluminum,
        name="fixed_hinge_leaf",
    )
    lid_0.visual(
        Box((1.02, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, -0.422, 0.0)),
        material=aluminum,
        name="front_track_shoe",
    )
    lid_0.visual(
        Box((1.02, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, 0.422, 0.0)),
        material=aluminum,
        name="rear_track_shoe",
    )

    lid_1 = model.part("lid_1")
    # Lower sliding panel in the staggered track, so either main lid can slide
    # over the other like a real supermarket chest freezer.
    lid_1.visual(
        Box((1.04, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, -0.3875, 0.0)),
        material=aluminum,
        name="front_frame",
    )
    lid_1.visual(
        Box((1.04, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, 0.3875, 0.0)),
        material=aluminum,
        name="rear_frame",
    )
    lid_1.visual(
        Box((0.045, 0.82, 0.030)),
        origin=Origin(xyz=(-0.4975, 0.0, 0.0)),
        material=aluminum,
        name="side_frame_0",
    )
    lid_1.visual(
        Box((0.045, 0.82, 0.030)),
        origin=Origin(xyz=(0.4975, 0.0, 0.0)),
        material=aluminum,
        name="side_frame_1",
    )
    lid_1.visual(
        Box((0.96, 0.72, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="glass_panel",
    )
    lid_1.visual(
        Box((0.018, 0.72, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=aluminum,
        name="center_grip_bar",
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.27, 0.430, 0.018)),
        origin=Origin(xyz=(0.0, -0.215, 0.0)),
        material=safety_glass,
        name="hatch_glass",
    )
    service_hatch.visual(
        Box((0.270, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.010, 0.006)),
        material=aluminum,
        name="hatch_hinge_edge",
    )
    service_hatch.visual(
        Box((0.270, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.430, 0.002)),
        material=aluminum,
        name="hatch_front_edge",
    )
    service_hatch.visual(
        Box((0.018, 0.430, 0.020)),
        origin=Origin(xyz=(-0.135, -0.215, 0.002)),
        material=aluminum,
        name="hatch_side_edge_0",
    )
    service_hatch.visual(
        Box((0.018, 0.430, 0.020)),
        origin=Origin(xyz=(0.135, -0.215, 0.002)),
        material=aluminum,
        name="hatch_side_edge_1",
    )
    service_hatch.visual(
        Cylinder(radius=0.012, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.0295), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="short_hinge_barrel",
    )
    service_hatch.visual(
        Box((0.014, 0.014, 0.040)),
        origin=Origin(xyz=(-0.085, -0.345, 0.029)),
        material=aluminum,
        name="handle_post_0",
    )
    service_hatch.visual(
        Box((0.014, 0.014, 0.040)),
        origin=Origin(xyz=(0.085, -0.345, 0.029)),
        material=aluminum,
        name="handle_post_1",
    )
    service_hatch.visual(
        Box((0.220, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.345, 0.055)),
        material=aluminum,
        name="low_pull_handle",
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-0.52, 0.0, 0.858)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.48),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(0.52, 0.0, 0.822)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.48),
    )
    model.articulation(
        "lid_0_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=lid_0,
        child=service_hatch,
        origin=Origin(xyz=(-0.15, 0.210, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    service_hatch = object_model.get_part("service_hatch")
    lid_0_slide = object_model.get_articulation("cabinet_to_lid_0")
    lid_1_slide = object_model.get_articulation("cabinet_to_lid_1")
    hatch_hinge = object_model.get_articulation("lid_0_to_service_hatch")

    ctx.check(
        "main lids use prismatic slides",
        lid_0_slide.articulation_type == ArticulationType.PRISMATIC
        and lid_1_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "service hatch uses short revolute hinge",
        hatch_hinge.articulation_type == ArticulationType.REVOLUTE
        and hatch_hinge.motion_limits is not None
        and hatch_hinge.motion_limits.upper is not None
        and 0.9 <= hatch_hinge.motion_limits.upper <= 1.3,
    )

    ctx.expect_overlap(lid_0, cabinet, axes="xy", min_overlap=0.72, name="lid_0 covers top well")
    ctx.expect_overlap(lid_1, cabinet, axes="xy", min_overlap=0.72, name="lid_1 covers top well")
    ctx.expect_gap(
        lid_1,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem="front_frame",
        negative_elem="front_slide_gasket",
        name="lower lid sits just above slide gasket",
    )
    ctx.expect_gap(
        lid_0,
        service_hatch,
        axis="y",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="fixed_hinge_leaf",
        negative_elem="short_hinge_barrel",
        name="hatch hinge barrel has small fixed-leaf clearance",
    )
    ctx.expect_within(
        service_hatch,
        lid_0,
        axes="xy",
        inner_elem="hatch_glass",
        margin=0.0,
        name="service hatch is set into lid footprint",
    )

    lid_0_rest = ctx.part_world_position(lid_0)
    with ctx.pose({lid_0_slide: 0.48}):
        lid_0_extended = ctx.part_world_position(lid_0)
        ctx.expect_overlap(lid_0, cabinet, axes="y", min_overlap=0.72, name="lid_0 remains on tracks")
    ctx.check(
        "lid_0 slides along cabinet length",
        lid_0_rest is not None
        and lid_0_extended is not None
        and lid_0_extended[0] > lid_0_rest[0] + 0.40,
    )

    lid_1_rest = ctx.part_world_position(lid_1)
    with ctx.pose({lid_1_slide: 0.48}):
        lid_1_extended = ctx.part_world_position(lid_1)
        ctx.expect_overlap(lid_1, cabinet, axes="y", min_overlap=0.72, name="lid_1 remains on tracks")
    ctx.check(
        "lid_1 slides opposite along cabinet length",
        lid_1_rest is not None
        and lid_1_extended is not None
        and lid_1_extended[0] < lid_1_rest[0] - 0.40,
    )

    hatch_closed = ctx.part_world_aabb(service_hatch)
    with ctx.pose({hatch_hinge: 1.0}):
        hatch_open = ctx.part_world_aabb(service_hatch)
    ctx.check(
        "service hatch swings upward",
        hatch_closed is not None
        and hatch_open is not None
        and hatch_open[1][2] > hatch_closed[1][2] + 0.14,
    )

    return ctx.report()


object_model = build_object_model()

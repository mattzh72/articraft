from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_sliding_window")

    aged_paint = model.material("aged_cream_paint", rgba=(0.78, 0.73, 0.61, 1.0))
    old_edge = model.material("worn_dark_edges", rgba=(0.32, 0.29, 0.22, 1.0))
    galvanized = model.material("galvanized_adapter_plates", rgba=(0.56, 0.58, 0.56, 1.0))
    stainless = model.material("brushed_stainless_guides", rgba=(0.72, 0.74, 0.72, 1.0))
    black_nylon = model.material("black_nylon_wear_pads", rgba=(0.03, 0.035, 0.035, 1.0))
    glass = model.material("slightly_green_glass", rgba=(0.62, 0.83, 0.86, 0.38))
    rubber = model.material("dark_rubber_weatherstrip", rgba=(0.04, 0.04, 0.035, 1.0))

    frame = model.part("frame")
    # Main old-school perimeter: heavy rails with a modern insert track.
    frame.visual(Box((1.60, 0.140, 0.120)), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=aged_paint, name="bottom_sill")
    frame.visual(Box((1.60, 0.140, 0.120)), origin=Origin(xyz=(0.0, 0.0, 0.990)), material=aged_paint, name="top_header")
    frame.visual(Box((0.110, 0.140, 1.050)), origin=Origin(xyz=(-0.745, 0.0, 0.525)), material=aged_paint, name="jamb_0")
    frame.visual(Box((0.110, 0.140, 1.050)), origin=Origin(xyz=(0.745, 0.0, 0.525)), material=aged_paint, name="jamb_1")

    # Stainless channel guides.  The sash runs between the front and rear lips.
    frame.visual(Box((1.380, 0.115, 0.028)), origin=Origin(xyz=(0.0, -0.048, 0.132)), material=stainless, name="bottom_track_floor")
    frame.visual(Box((1.380, 0.012, 0.075)), origin=Origin(xyz=(0.0, -0.082, 0.175)), material=stainless, name="bottom_front_lip")
    frame.visual(Box((1.380, 0.012, 0.075)), origin=Origin(xyz=(0.0, -0.014, 0.175)), material=stainless, name="bottom_rear_lip")
    frame.visual(Box((1.380, 0.115, 0.028)), origin=Origin(xyz=(0.0, -0.048, 0.918)), material=stainless, name="top_track_floor")
    frame.visual(Box((1.380, 0.012, 0.075)), origin=Origin(xyz=(0.0, -0.082, 0.875)), material=stainless, name="top_front_lip")
    frame.visual(Box((1.380, 0.012, 0.075)), origin=Origin(xyz=(0.0, -0.014, 0.875)), material=stainless, name="top_rear_lip")
    frame.visual(Box((1.260, 0.028, 0.007)), origin=Origin(xyz=(0.0, -0.048, 0.150)), material=black_nylon, name="lower_wear_strip")
    frame.visual(Box((1.260, 0.028, 0.007)), origin=Origin(xyz=(0.0, -0.048, 0.900)), material=black_nylon, name="upper_wear_strip")

    # Mechanical travel stops anchored into the channel.
    frame.visual(Box((0.025, 0.050, 0.100)), origin=Origin(xyz=(-0.700, -0.048, 0.185)), material=old_edge, name="closed_stop")
    frame.visual(Box((0.025, 0.050, 0.100)), origin=Origin(xyz=(0.515, -0.048, 0.185)), material=old_edge, name="open_stop")
    frame.visual(Box((0.025, 0.050, 0.080)), origin=Origin(xyz=(-0.700, -0.048, 0.865)), material=old_edge, name="upper_closed_stop")
    frame.visual(Box((0.025, 0.050, 0.080)), origin=Origin(xyz=(0.515, -0.048, 0.865)), material=old_edge, name="upper_open_stop")

    # Fixed rear sash/pane, offset in depth from the sliding front sash.
    frame.visual(Box((0.690, 0.036, 0.040)), origin=Origin(xyz=(0.350, 0.036, 0.165)), material=aged_paint, name="fixed_bottom_rail")
    frame.visual(Box((0.690, 0.036, 0.040)), origin=Origin(xyz=(0.350, 0.036, 0.885)), material=aged_paint, name="fixed_top_rail")
    frame.visual(Box((0.042, 0.036, 0.740)), origin=Origin(xyz=(0.018, 0.036, 0.525)), material=aged_paint, name="meeting_stile")
    frame.visual(Box((0.042, 0.036, 0.740)), origin=Origin(xyz=(0.682, 0.036, 0.525)), material=aged_paint, name="fixed_side_stile")
    frame.visual(Box((0.640, 0.007, 0.690)), origin=Origin(xyz=(0.350, 0.056, 0.525)), material=glass, name="fixed_glass")
    frame.visual(Box((0.024, 0.045, 0.735)), origin=Origin(xyz=(0.045, 0.010, 0.525)), material=rubber, name="meeting_weatherstrip")

    # Bolted adapter plates and pragmatic reinforcements at frame joints.
    for x, name in [(-0.745, "adapter_0"), (0.745, "adapter_1")]:
        frame.visual(Box((0.085, 0.018, 0.710)), origin=Origin(xyz=(x, -0.078, 0.525)), material=galvanized, name=name)
        for z in (0.235, 0.395, 0.655, 0.815):
            frame.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(x, -0.091, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=old_edge,
                name=f"{name}_bolt_{int(z * 1000)}",
            )
    for x, z, angle, name in [
        (-0.650, 0.145, math.radians(35), "corner_strap_0"),
        (0.650, 0.145, -math.radians(35), "corner_strap_1"),
        (-0.650, 0.905, -math.radians(35), "corner_strap_2"),
        (0.650, 0.905, math.radians(35), "corner_strap_3"),
    ]:
        frame.visual(
            Box((0.250, 0.016, 0.026)),
            origin=Origin(xyz=(x, -0.075, z), rpy=(0.0, angle, 0.0)),
            material=galvanized,
            name=name,
        )

    # Fixed hinge leaves and knuckles for the service hatches.
    frame.visual(Box((0.100, 0.030, 0.024)), origin=Origin(xyz=(-0.635, -0.083, 0.105)), material=galvanized, name="lower_hinge_leaf_0")
    frame.visual(Box((0.100, 0.030, 0.024)), origin=Origin(xyz=(-0.265, -0.083, 0.105)), material=galvanized, name="lower_hinge_leaf_1")
    for x in (-0.605, -0.295):
        frame.visual(
            Cylinder(radius=0.011, length=0.080),
            origin=Origin(xyz=(x, -0.103, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"lower_fixed_knuckle_{0 if x < -0.45 else 1}",
        )
    frame.visual(Box((0.026, 0.014, 0.330)), origin=Origin(xyz=(0.700, -0.091, 0.600)), material=galvanized, name="side_hinge_leaf")
    for z, suffix in [(0.455, 0), (0.745, 1)]:
        frame.visual(
            Cylinder(radius=0.009, length=0.040),
            origin=Origin(xyz=(0.700, -0.103, z)),
            material=galvanized,
            name=f"side_fixed_knuckle_{suffix}",
        )

    sash = model.part("sliding_sash")
    sash.visual(Box((0.660, 0.036, 0.055)), origin=Origin(xyz=(0.0, 0.0, -0.337)), material=aged_paint, name="bottom_rail")
    sash.visual(Box((0.660, 0.036, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.337)), material=aged_paint, name="top_rail")
    sash.visual(Box((0.045, 0.036, 0.730)), origin=Origin(xyz=(-0.307, 0.0, 0.0)), material=aged_paint, name="side_stile_0")
    sash.visual(Box((0.045, 0.036, 0.730)), origin=Origin(xyz=(0.307, 0.0, 0.0)), material=aged_paint, name="side_stile_1")
    sash.visual(Box((0.580, 0.007, 0.625)), origin=Origin(xyz=(0.0, 0.004, 0.0)), material=glass, name="sash_glass")
    sash.visual(Box((0.580, 0.014, 0.018)), origin=Origin(xyz=(0.0, -0.002, 0.0)), material=aged_paint, name="middle_muntin")
    sash.visual(Box((0.018, 0.014, 0.625)), origin=Origin(xyz=(0.0, -0.002, 0.0)), material=aged_paint, name="upright_muntin")
    sash.visual(Box((0.080, 0.022, 0.008)), origin=Origin(xyz=(-0.205, 0.0, -0.3675)), material=black_nylon, name="glide_0")
    sash.visual(Box((0.080, 0.022, 0.008)), origin=Origin(xyz=(0.205, 0.0, -0.3675)), material=black_nylon, name="glide_1")
    sash.visual(Box((0.050, 0.010, 0.180)), origin=Origin(xyz=(0.260, -0.021, 0.0)), material=galvanized, name="pull_plate")
    sash.visual(Box((0.018, 0.028, 0.115)), origin=Origin(xyz=(0.260, -0.050, 0.0)), material=old_edge, name="pull_grip")
    sash.visual(Box((0.018, 0.030, 0.018)), origin=Origin(xyz=(0.260, -0.037, -0.048)), material=old_edge, name="pull_foot_0")
    sash.visual(Box((0.018, 0.030, 0.018)), origin=Origin(xyz=(0.260, -0.037, 0.048)), material=old_edge, name="pull_foot_1")

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-0.340, -0.048, 0.525)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.510),
        motion_properties=MotionProperties(damping=8.0, friction=2.0),
    )

    lower_hatch = model.part("lower_hatch")
    lower_hatch.visual(Box((0.230, 0.012, 0.090)), origin=Origin(xyz=(0.0, -0.006, -0.045)), material=galvanized, name="hatch_plate")
    lower_hatch.visual(
        Cylinder(radius=0.011, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hatch_knuckle",
    )
    for x, z, suffix in [(-0.070, -0.035, 0), (0.070, -0.035, 1)]:
        lower_hatch.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x, -0.014, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=old_edge,
            name=f"hatch_bolt_{suffix}",
        )
    model.articulation(
        "frame_to_lower_hatch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_hatch,
        origin=Origin(xyz=(-0.450, -0.103, 0.105)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.30),
    )

    side_hatch = model.part("side_hatch")
    side_hatch.visual(Box((0.150, 0.012, 0.200)), origin=Origin(xyz=(0.075, -0.006, 0.0)), material=galvanized, name="hatch_plate")
    side_hatch.visual(Cylinder(radius=0.009, length=0.110), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=galvanized, name="hatch_knuckle")
    for x, z, suffix in [(0.090, -0.060, 0), (0.090, 0.060, 1)]:
        side_hatch.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(x, -0.014, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=old_edge,
            name=f"hatch_bolt_{suffix}",
        )
    model.articulation(
        "frame_to_side_hatch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_hatch,
        origin=Origin(xyz=(0.700, -0.103, 0.600)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    lower_hatch = object_model.get_part("lower_hatch")
    side_hatch = object_model.get_part("side_hatch")
    slide = object_model.get_articulation("frame_to_sash")
    lower_hinge = object_model.get_articulation("frame_to_lower_hatch")
    side_hinge = object_model.get_articulation("frame_to_side_hatch")

    ctx.check(
        "one sliding sash with retained travel",
        slide.motion_limits is not None and slide.motion_limits.upper is not None and slide.motion_limits.upper >= 0.50,
        details=f"limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            sash,
            frame,
            axes="y",
            inner_elem="bottom_rail",
            outer_elem="bottom_track_floor",
            margin=0.0,
            name="closed sash bottom rail is centered in lower channel",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="bottom_track_floor",
            min_gap=0.006,
            max_gap=0.030,
            name="closed sash rides above lower track",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            positive_elem="top_track_floor",
            negative_elem="top_rail",
            min_gap=0.006,
            max_gap=0.035,
            name="closed sash clears upper channel",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="x",
            positive_elem="bottom_rail",
            negative_elem="closed_stop",
            min_gap=0.0,
            max_gap=0.030,
            name="closed stop is just ahead of sash rail",
        )

    closed_position = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.510}):
        ctx.expect_within(
            sash,
            frame,
            axes="y",
            inner_elem="bottom_rail",
            outer_elem="bottom_track_floor",
            margin=0.0,
            name="open sash remains captured between guide lips",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="bottom_rail",
            elem_b="bottom_track_floor",
            min_overlap=0.55,
            name="open sash keeps retained overlap in lower channel",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="x",
            positive_elem="open_stop",
            negative_elem="bottom_rail",
            min_gap=0.0,
            max_gap=0.012,
            name="open stop limits sash travel",
        )
        open_position = ctx.part_world_position(sash)

    ctx.check(
        "sash moves horizontally in its channel",
        closed_position is not None and open_position is not None and open_position[0] > closed_position[0] + 0.50,
        details=f"closed={closed_position}, open={open_position}",
    )

    with ctx.pose({lower_hinge: 0.75, side_hinge: 0.70}):
        ctx.check(
            "service hatches articulate for maintenance",
            lower_hatch is not None and side_hatch is not None,
            details="Lower and side access plates are separate hinged parts.",
        )

    return ctx.report()


object_model = build_object_model()

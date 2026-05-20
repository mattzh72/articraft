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
    model = ArticulatedObject(
        name="cost_optimized_sliding_security_gate",
        meta={
            "design_notes": (
                "Low part-count sliding security gate: one welded/static track frame, "
                "one tube-and-picket gate leaf with captured rollers, and one stamped latch handle."
            )
        },
    )

    galvanized = Material("galvanized_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    powder = Material("black_powder_coat", rgba=(0.02, 0.023, 0.022, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    zinc = Material("zinc_plated_hardware", rgba=(0.86, 0.78, 0.50, 1.0))

    # Root: one weldment/bolt-up frame made from simple extrusions and stampings.
    track_frame = model.part("track_frame")
    track_frame.visual(
        Box((3.55, 0.30, 0.040)),
        origin=Origin(xyz=(0.50, 0.0, 0.020)),
        material=galvanized,
        name="floor_sill",
    )
    track_frame.visual(
        Box((3.35, 0.055, 0.045)),
        origin=Origin(xyz=(0.50, 0.0, 0.0625)),
        material=galvanized,
        name="bottom_rail",
    )
    for y, name in [(-0.078, "front_track_lip"), (0.078, "rear_track_lip")]:
        track_frame.visual(
            Box((3.35, 0.026, 0.060)),
            origin=Origin(xyz=(0.50, y, 0.090)),
            material=galvanized,
            name=name,
        )

    for x, name in [(-1.28, "latch_post"), (2.25, "end_post")]:
        track_frame.visual(
            Box((0.120, 0.160, 1.75)),
            origin=Origin(xyz=(x, 0.0, 0.875)),
            material=powder,
            name=name,
        )

    track_frame.visual(
        Box((3.55, 0.200, 0.035)),
        origin=Origin(xyz=(0.50, 0.0, 1.755)),
        material=galvanized,
        name="top_channel_web",
    )
    for y, name in [(-0.0825, "front_top_lip"), (0.0825, "rear_top_lip")]:
        track_frame.visual(
            Box((3.55, 0.035, 0.166)),
            origin=Origin(xyz=(0.50, y, 1.656)),
            material=galvanized,
            name=name,
        )

    # Bolt-on clamp plates show a simple assembly sequence: set rail, clamp to posts, tighten.
    for x, name in [(-1.28, "top_clamp_0"), (2.25, "top_clamp_1")]:
        track_frame.visual(
            Box((0.165, 0.018, 0.210)),
            origin=Origin(xyz=(x, -0.106, 1.665)),
            material=galvanized,
            name=name,
        )
        for dz, bolt_name in [(-0.060, f"{name}_bolt_0"), (0.060, f"{name}_bolt_1")]:
            track_frame.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(
                    xyz=(x, -0.118, 1.665 + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=zinc,
                name=bolt_name,
            )

    for x, name in [(-1.075, "closed_stop"), (2.160, "open_stop")]:
        track_frame.visual(
            Box((0.070, 0.180, 0.190)),
            origin=Origin(xyz=(x, 0.0, 0.180)),
            material=powder,
            name=name,
        )

    # Stamped latch receiver: two guarded jaws welded/bolted to the latch post.
    for z, name in [(1.125, "receiver_upper_jaw"), (0.775, "receiver_lower_jaw")]:
        track_frame.visual(
            Box((0.130, 0.080, 0.060)),
            origin=Origin(xyz=(-1.1575, -0.065, z)),
            material=powder,
            name=name,
        )
    for z, name in [(1.125, "receiver_upper_bolt"), (0.775, "receiver_lower_bolt")]:
        track_frame.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(
                xyz=(-1.185, -0.111, z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=name,
        )

    # Moving leaf: one manufactured subassembly with tubular perimeter, pickets,
    # integral guide fin, and captured roller saddles.
    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((2.000, 0.080, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=powder,
        name="bottom_tube",
    )
    gate_leaf.visual(
        Box((2.000, 0.080, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 1.485)),
        material=powder,
        name="top_tube",
    )
    for x, name in [(-0.960, "leading_upright"), (0.960, "trailing_upright")]:
        gate_leaf.visual(
            Box((0.080, 0.080, 1.240)),
            origin=Origin(xyz=(x, 0.0, 0.885)),
            material=powder,
            name=name,
        )

    for i, x in enumerate([-0.640, -0.320, 0.0, 0.320, 0.640]):
        gate_leaf.visual(
            Box((0.035, 0.045, 1.200)),
            origin=Origin(xyz=(x, 0.0, 0.895)),
            material=powder,
            name=f"picket_{i}",
        )

    gate_leaf.visual(
        Box((1.890, 0.035, 0.035)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.900),
            rpy=(0.0, -math.atan2(1.05, 1.75), 0.0),
        ),
        material=powder,
        name="diagonal_brace",
    )
    gate_leaf.visual(
        Box((1.850, 0.036, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 1.595)),
        material=galvanized,
        name="top_guide_fin",
    )

    for i, x in enumerate([-0.680, 0.680]):
        gate_leaf.visual(
            Cylinder(radius=0.080, length=0.050),
            origin=Origin(
                xyz=(x, 0.0, 0.165),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rubber,
            name=f"roller_{i}",
        )
        gate_leaf.visual(
            Box((0.115, 0.130, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.243)),
            material=galvanized,
            name=f"roller_saddle_{i}",
        )
        gate_leaf.visual(
            Cylinder(radius=0.022, length=0.150),
            origin=Origin(
                xyz=(x, 0.0, 0.165),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=f"roller_axle_{i}",
        )

    gate_leaf.visual(
        Box((0.260, 0.014, 0.200)),
        origin=Origin(xyz=(-0.840, -0.045, 0.950)),
        material=galvanized,
        name="latch_mount",
    )
    for dz, name in [(-0.055, "latch_mount_bolt_0"), (0.055, "latch_mount_bolt_1")]:
        gate_leaf.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(
                xyz=(-0.765, -0.057, 0.950 + dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=name,
        )

    slide = model.articulation(
        "track_to_leaf",
        ArticulationType.PRISMATIC,
        parent=track_frame,
        child=gate_leaf,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.55, lower=0.0, upper=1.05),
    )
    slide.meta["purpose"] = "Gate leaf slides along the full rail/channel travel with retained roller support."

    # Single stamped latch handle/pawl, pinned to the leaf.  It is a separate
    # moving part because the handle is a visible user control.
    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(
            xyz=(0.0, -0.009, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=zinc,
        name="pivot_disk",
    )
    latch_handle.visual(
        Box((0.240, 0.022, 0.032)),
        origin=Origin(xyz=(-0.1325, -0.020, 0.0)),
        material=galvanized,
        name="latch_tongue",
    )
    latch_handle.visual(
        Box((0.050, 0.026, 0.260)),
        origin=Origin(xyz=(0.055, -0.022, -0.110)),
        material=galvanized,
        name="pull_tab",
    )
    model.articulation(
        "leaf_to_latch",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=latch_handle,
        origin=Origin(xyz=(-0.840, -0.052, 0.950)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("track_frame")
    leaf = object_model.get_part("gate_leaf")
    latch = object_model.get_part("latch_handle")
    slide = object_model.get_articulation("track_to_leaf")
    latch_joint = object_model.get_articulation("leaf_to_latch")

    limits = slide.motion_limits
    ctx.check(
        "leaf travel is bounded by the manufactured rail length",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.0 <= limits.upper <= 1.10,
        details=f"limits={limits}",
    )

    # The rollers sit on the bottom rail, and the guide fin stays captured in
    # the upper channel envelope.  These are the constraints that make the gate
    # travel feel like a real guided sliding leaf instead of a floating panel.
    for roller_name in ("roller_0", "roller_1"):
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem=roller_name,
            negative_elem="bottom_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{roller_name} rests on bottom rail",
        )
        ctx.expect_within(
            leaf,
            frame,
            axes="x",
            inner_elem=roller_name,
            outer_elem="bottom_rail",
            margin=0.0,
            name=f"{roller_name} remains inside rail length",
        )

    ctx.expect_within(
        leaf,
        frame,
        axes="xy",
        inner_elem="top_guide_fin",
        outer_elem="top_channel_web",
        margin=0.0,
        name="top guide fin is captured by top channel width",
    )
    ctx.expect_gap(
        frame,
        leaf,
        axis="z",
        positive_elem="top_channel_web",
        negative_elem="top_guide_fin",
        min_gap=0.040,
        max_gap=0.060,
        name="top guide fin clears upper channel web",
    )

    ctx.expect_gap(
        latch,
        frame,
        axis="x",
        positive_elem="latch_tongue",
        negative_elem="receiver_upper_jaw",
        max_gap=0.006,
        max_penetration=0.0,
        name="latch tongue reaches receiver without crossing it",
    )
    ctx.expect_overlap(
        latch,
        frame,
        axes="y",
        elem_a="latch_tongue",
        elem_b="receiver_upper_jaw",
        min_overlap=0.015,
        name="latch tongue aligns with receiver jaws in depth",
    )
    ctx.expect_gap(
        leaf,
        latch,
        axis="y",
        positive_elem="latch_mount",
        negative_elem="pivot_disk",
        max_gap=0.001,
        max_penetration=0.0,
        name="latch pivot disk seats on leaf mount",
    )

    rest_pos = ctx.part_world_position(leaf)
    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({slide: limits.upper}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="roller_1",
            negative_elem="bottom_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="open pose roller still bears on rail",
        )
        ctx.expect_within(
            leaf,
            frame,
            axes="x",
            inner_elem="roller_1",
            outer_elem="bottom_rail",
            margin=0.0,
            name="open pose roller is retained on rail",
        )
        open_pos = ctx.part_world_position(leaf)

    ctx.check(
        "slide joint moves leaf along positive track direction",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    with ctx.pose({latch_joint: 0.30}):
        raised_latch_aabb = ctx.part_world_aabb(latch)

    ctx.check(
        "latch handle lifts the pawl upward when operated",
        rest_latch_aabb is not None
        and raised_latch_aabb is not None
        and raised_latch_aabb[1][2] > rest_latch_aabb[1][2] + 0.025,
        details=f"rest={rest_latch_aabb}, raised={raised_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

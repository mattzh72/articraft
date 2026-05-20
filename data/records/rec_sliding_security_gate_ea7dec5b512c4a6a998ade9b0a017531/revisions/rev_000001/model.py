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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = Material("galvanized_dark", rgba=(0.12, 0.14, 0.15, 1.0))
    gate_paint = Material("black_powder_coat", rgba=(0.02, 0.025, 0.025, 1.0))
    concrete = Material("cast_concrete", rgba=(0.45, 0.43, 0.39, 1.0))
    guide_wear = Material("worn_steel_track", rgba=(0.24, 0.25, 0.25, 1.0))

    fixed_frame = model.part("fixed_frame")

    # A continuous curb keeps the fixed frame and guide hardware physically tied
    # together while reading as a realistic concrete base for a security gate.
    fixed_frame.visual(
        Box((5.40, 0.55, 0.08)),
        origin=Origin(xyz=(0.85, 0.0, 0.04)),
        material=concrete,
        name="concrete_curb",
    )

    for x, name in [(-1.75, "latch_post"), (1.75, "receiver_post"), (3.45, "tail_post")]:
        fixed_frame.visual(
            Box((0.12, 0.12, 2.10)),
            origin=Origin(xyz=(x, -0.30, 1.05)),
            material=galvanized,
            name=name,
        )
        fixed_frame.visual(
            Box((0.16, 0.13, 0.08)),
            origin=Origin(xyz=(x, -0.245, 2.04)),
            material=galvanized,
            name=f"{name}_bracket",
        )

    # Fixed, open-bottom top track.  The broad back plate bolts to the posts;
    # the downward cheeks form a visible slot around the moving carrier.
    fixed_frame.visual(
        Box((5.40, 0.46, 0.12)),
        origin=Origin(xyz=(0.85, 0.0, 2.06)),
        material=galvanized,
        name="top_track_roof",
    )
    fixed_frame.visual(
        Box((5.40, 0.05, 0.28)),
        origin=Origin(xyz=(0.85, -0.13, 1.88)),
        material=galvanized,
        name="rear_track_cheek",
    )
    fixed_frame.visual(
        Box((5.40, 0.05, 0.28)),
        origin=Origin(xyz=(0.85, 0.13, 1.88)),
        material=galvanized,
        name="front_track_cheek",
    )

    # The bottom guide is an open channel that laterally captures the lower
    # rail without occupying the moving leaf's plane.
    fixed_frame.visual(
        Box((5.40, 0.28, 0.05)),
        origin=Origin(xyz=(0.85, 0.0, 0.105)),
        material=guide_wear,
        name="bottom_guide_base",
    )
    fixed_frame.visual(
        Box((5.40, 0.035, 0.15)),
        origin=Origin(xyz=(0.85, -0.09, 0.155)),
        material=guide_wear,
        name="rear_guide_lip",
    )
    fixed_frame.visual(
        Box((5.40, 0.035, 0.15)),
        origin=Origin(xyz=(0.85, 0.09, 0.155)),
        material=guide_wear,
        name="front_guide_lip",
    )

    gate_leaf = model.part("gate_leaf")

    # Rectangular moving leaf frame.
    gate_leaf.visual(
        Box((3.04, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.53)),
        material=gate_paint,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((3.04, 0.145, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=gate_paint,
        name="bottom_rail",
    )
    for x, name in [(-1.50, "end_stile"), (1.50, "nose_stile")]:
        gate_leaf.visual(
            Box((0.08, 0.08, 1.43)),
            origin=Origin(xyz=(x, 0.0, 0.865)),
            material=gate_paint,
            name=name,
        )

    # Closely spaced vertical pickets make the leaf read as a security gate and
    # overlap the rails slightly so each bar is welded into the leaf assembly.
    for i, x in enumerate([-1.20, -0.90, -0.60, -0.30, 0.0, 0.30, 0.60, 0.90, 1.20]):
        gate_leaf.visual(
            Cylinder(radius=0.018, length=1.32),
            origin=Origin(xyz=(x, 0.0, 0.86)),
            material=gate_paint,
            name=f"picket_{i}",
        )

    # Three welded hanger straps connect the leaf frame to the captured sliding
    # carrier that runs inside the top track.
    for i, x in enumerate([-1.05, 0.0, 1.05]):
        gate_leaf.visual(
            Box((0.07, 0.045, 0.34)),
            origin=Origin(xyz=(x, 0.0, 1.68)),
            material=gate_paint,
            name=f"hanger_{i}",
        )
        gate_leaf.visual(
            Cylinder(radius=0.045, length=0.055),
            origin=Origin(xyz=(x, 0.0, 1.88), rpy=(1.57079632679, 0.0, 0.0)),
            material=guide_wear,
            name=f"roller_{i}",
        )

    gate_leaf.visual(
        Box((3.10, 0.21, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=guide_wear,
        name="top_carrier",
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=1.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate")

    ctx.check(
        "gate leaf uses a prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "long slide travel is retained by track length",
        slide.motion_limits.upper is not None and slide.motion_limits.upper >= 1.75,
        details=f"limits={slide.motion_limits}",
    )

    ctx.expect_within(
        gate_leaf,
        fixed_frame,
        axes="xy",
        inner_elem="top_carrier",
        outer_elem="top_track_roof",
        margin=0.002,
        name="closed carrier is inside top track footprint",
    )
    ctx.expect_overlap(
        gate_leaf,
        fixed_frame,
        axes="x",
        elem_a="top_carrier",
        elem_b="top_track_roof",
        min_overlap=2.9,
        name="closed carrier has generous top-track engagement",
    )
    ctx.expect_within(
        gate_leaf,
        fixed_frame,
        axes="xy",
        inner_elem="bottom_rail",
        outer_elem="bottom_guide_base",
        margin=0.002,
        name="closed lower rail is guided in the bottom channel footprint",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            gate_leaf,
            fixed_frame,
            axes="xy",
            inner_elem="top_carrier",
            outer_elem="top_track_roof",
            margin=0.002,
            name="extended carrier remains inside top track footprint",
        )
        ctx.expect_overlap(
            gate_leaf,
            fixed_frame,
            axes="x",
            elem_a="top_carrier",
            elem_b="top_track_roof",
            min_overlap=2.9,
            name="extended carrier keeps generous retained insertion",
        )
        ctx.expect_within(
            gate_leaf,
            fixed_frame,
            axes="xy",
            inner_elem="bottom_rail",
            outer_elem="bottom_guide_base",
            margin=0.002,
            name="extended lower rail remains guided by bottom channel",
        )
        extended_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate translates along the track axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 1.75
        and abs(extended_pos[1] - rest_pos[1]) < 0.001
        and abs(extended_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()

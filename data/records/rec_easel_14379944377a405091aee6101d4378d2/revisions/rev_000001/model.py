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


WOOD = Material("oiled_beech", rgba=(0.72, 0.48, 0.25, 1.0))
DARK_METAL = Material("blackened_steel", rgba=(0.04, 0.04, 0.04, 1.0))
BRASS = Material("aged_brass", rgba=(0.86, 0.63, 0.25, 1.0))
RUBBER = Material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="h_frame_studio_easel")
    model.material("oiled_beech", rgba=WOOD.rgba)
    model.material("blackened_steel", rgba=DARK_METAL.rgba)
    model.material("aged_brass", rgba=BRASS.rgba)
    model.material("dark_rubber", rgba=RUBBER.rgba)

    frame = model.part("frame")

    # Fixed H-frame: two straight front uprights tied together by cross-members.
    post_x = (-0.33, 0.33)
    for x, leg_name, foot_name in (
        (-0.33, "front_leg_0", "front_foot_0"),
        (0.33, "front_leg_1", "front_foot_1"),
    ):
        frame.visual(
            Box((0.055, 0.055, 1.76)),
            origin=Origin(xyz=(x, 0.0, 0.88)),
            material=WOOD,
            name=leg_name,
        )
        frame.visual(
            Box((0.18, 0.16, 0.045)),
            origin=Origin(xyz=(x, -0.025, 0.022)),
            material=WOOD,
            name=foot_name,
        )

    # Cross-bars sit mostly behind the slide path but lap slightly into the
    # front posts, so the fixed frame is one connected wooden assembly.
    for name, z, sx, sy, sz in (
        ("lower_rail", 0.20, 0.76, 0.060, 0.060),
        ("middle_rail", 0.95, 0.74, 0.058, 0.055),
        ("top_rail", 1.64, 0.78, 0.060, 0.060),
    ):
        frame.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(0.0, 0.052, z)),
            material=WOOD,
            name=name,
        )

    # Rear support legs are leaned back like a studio easel's stance.
    rear_bottom_y = 0.68
    rear_top_y = 0.045
    rear_bottom_z = 0.035
    rear_top_z = 1.46
    rear_len = math.sqrt((rear_top_y - rear_bottom_y) ** 2 + (rear_top_z - rear_bottom_z) ** 2)
    rear_roll = math.atan2(rear_bottom_y - rear_top_y, rear_top_z - rear_bottom_z)
    for i, x in enumerate(post_x):
        frame.visual(
            Box((0.050, 0.050, rear_len)),
            origin=Origin(
                xyz=(x, (rear_bottom_y + rear_top_y) / 2.0, (rear_bottom_z + rear_top_z) / 2.0),
                rpy=(rear_roll, 0.0, 0.0),
            ),
            material=WOOD,
            name=f"rear_leg_{i}",
        )
        frame.visual(
            Box((0.09, 0.085, 0.085)),
            origin=Origin(xyz=(x, 0.028, 1.46)),
            material=WOOD,
            name=f"rear_hinge_block_{i}",
        )
        frame.visual(
            Box((0.18, 0.16, 0.045)),
            origin=Origin(xyz=(x, rear_bottom_y, 0.022)),
            material=WOOD,
            name=f"rear_foot_{i}",
        )

    frame.visual(
        Box((0.78, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, 0.605, 0.20)),
        material=WOOD,
        name="rear_cross_rail",
    )
    frame.visual(
        Box((0.70, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.437, 0.58), rpy=(rear_roll, 0.0, 0.0)),
        material=WOOD,
        name="rear_spreader",
    )

    # Sliding lower canvas support with two U-shaped locking slide brackets.
    canvas_bar = model.part("canvas_bar")
    canvas_bar.visual(
        Box((0.88, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.087, 0.0)),
        material=WOOD,
        name="support_shelf",
    )
    canvas_bar.visual(
        Box((0.88, 0.026, 0.115)),
        origin=Origin(xyz=(0.0, -0.122, 0.032)),
        material=WOOD,
        name="raised_lip",
    )
    for i, x in enumerate(post_x):
        canvas_bar.visual(
            Box((0.086, 0.014, 0.140)),
            origin=Origin(xyz=(x, -0.062, 0.0)),
            material=DARK_METAL,
            name=f"lock_plate_{i}",
        )
        canvas_bar.visual(
            Box((0.015, 0.075, 0.140)),
            origin=Origin(xyz=(x - 0.035, -0.020, 0.0)),
            material=DARK_METAL,
            name=f"slide_cheek_{i}_0",
        )
        canvas_bar.visual(
            Box((0.015, 0.075, 0.140)),
            origin=Origin(xyz=(x + 0.035, -0.020, 0.0)),
            material=DARK_METAL,
            name=f"slide_cheek_{i}_1",
        )

    canvas_slide = model.articulation(
        "frame_to_canvas_bar",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=canvas_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.42),
    )

    # Top clamp bar, also carried by slide collars on the same front legs.
    top_clamp = model.part("top_clamp")
    top_clamp.visual(
        Box((0.82, 0.052, 0.060)),
        origin=Origin(xyz=(0.0, -0.088, 0.0)),
        material=WOOD,
        name="clamp_bar",
    )
    top_clamp.visual(
        Box((0.30, 0.028, 0.110)),
        origin=Origin(xyz=(0.0, -0.125, -0.045)),
        material=WOOD,
        name="clamp_jaw",
    )
    top_clamp.visual(
        Box((0.28, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, -0.143, -0.047)),
        material=RUBBER,
        name="cork_pad",
    )
    for i, x in enumerate(post_x):
        top_clamp.visual(
            Box((0.082, 0.014, 0.120)),
            origin=Origin(xyz=(x, -0.062, 0.0)),
            material=DARK_METAL,
            name=f"clamp_lock_plate_{i}",
        )
        top_clamp.visual(
            Box((0.014, 0.073, 0.120)),
            origin=Origin(xyz=(x - 0.0345, -0.020, 0.0)),
            material=DARK_METAL,
            name=f"clamp_cheek_{i}_0",
        )
        top_clamp.visual(
            Box((0.014, 0.073, 0.120)),
            origin=Origin(xyz=(x + 0.0345, -0.020, 0.0)),
            material=DARK_METAL,
            name=f"clamp_cheek_{i}_1",
        )

    top_slide = model.articulation(
        "frame_to_top_clamp",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=top_clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.16, lower=0.0, upper=0.40),
    )

    # User-facing lock knobs rotate on screw axes normal to each slide plate.
    # They are separate articulated controls, while the screw tips just meet
    # the plates so the knobs are visibly mounted without broad penetration.
    def add_lock_knob(parent_part, parent_slide_name: str, x: float, z: float, plate_name: str):
        knob = model.part(parent_slide_name)
        knob.visual(
            Cylinder(radius=0.034, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BRASS,
            name="knob_disk",
        )
        knob.visual(
            Cylinder(radius=0.006, length=0.086),
            origin=Origin(xyz=(0.0, 0.048, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=DARK_METAL,
            name="screw",
        )
        model.articulation(
            f"{parent_part.name}_to_{parent_slide_name}",
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=knob,
            origin=Origin(xyz=(x, -0.160, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )
        return knob

    for i, x in enumerate(post_x):
        add_lock_knob(canvas_bar, f"canvas_lock_{i}", x, -0.05, f"lock_plate_{i}")
        add_lock_knob(top_clamp, f"clamp_lock_{i}", x, 0.05, f"clamp_lock_plate_{i}")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    canvas_bar = object_model.get_part("canvas_bar")
    top_clamp = object_model.get_part("top_clamp")
    canvas_slide = object_model.get_articulation("frame_to_canvas_bar")
    top_slide = object_model.get_articulation("frame_to_top_clamp")

    ctx.expect_overlap(
        canvas_bar,
        frame,
        axes="x",
        elem_a="support_shelf",
        elem_b="front_leg_0",
        min_overlap=0.04,
        name="lower support spans a front leg",
    )
    ctx.expect_overlap(
        top_clamp,
        frame,
        axes="x",
        elem_a="clamp_bar",
        elem_b="front_leg_1",
        min_overlap=0.04,
        name="top clamp spans a front leg",
    )

    rest_canvas_pos = ctx.part_world_position(canvas_bar)
    with ctx.pose({canvas_slide: 0.42}):
        high_canvas_pos = ctx.part_world_position(canvas_bar)
        ctx.expect_gap(
            top_clamp,
            canvas_bar,
            axis="z",
            min_gap=0.04,
            name="raised canvas bar remains below top clamp",
        )
    ctx.check(
        "canvas bar slides upward on the front legs",
        rest_canvas_pos is not None
        and high_canvas_pos is not None
        and high_canvas_pos[2] > rest_canvas_pos[2] + 0.38,
        details=f"rest={rest_canvas_pos}, high={high_canvas_pos}",
    )

    rest_clamp_pos = ctx.part_world_position(top_clamp)
    with ctx.pose({top_slide: 0.40}):
        high_clamp_pos = ctx.part_world_position(top_clamp)
    ctx.check(
        "top clamp slides upward prismatically",
        rest_clamp_pos is not None
        and high_clamp_pos is not None
        and high_clamp_pos[2] > rest_clamp_pos[2] + 0.35,
        details=f"rest={rest_clamp_pos}, high={high_clamp_pos}",
    )

    return ctx.report()


object_model = build_object_model()

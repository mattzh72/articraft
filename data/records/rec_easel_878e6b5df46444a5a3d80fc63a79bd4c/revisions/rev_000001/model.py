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
    model = ArticulatedObject(name="heavy_duty_h_frame_easel")

    beech = model.material("sealed_beech_wood", color=(0.72, 0.48, 0.25, 1.0))
    endgrain = model.material("darker_endgrain", color=(0.48, 0.30, 0.14, 1.0))
    black_steel = model.material("black_powder_coated_steel", color=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", color=(0.01, 0.01, 0.009, 1.0))

    frame = model.part("front_frame")

    # Two tall uprights of the H-frame.
    for x, name in ((-0.45, "upright_0"), (0.45, "upright_1")):
        frame.visual(
            Box((0.080, 0.080, 1.860)),
            origin=Origin(xyz=(x, 0.0, 0.960)),
            material=beech,
            name=name,
        )
        frame.visual(
            Box((0.130, 0.520, 0.060)),
            origin=Origin(xyz=(x, -0.090, 0.030)),
            material=endgrain,
            name=f"front_foot_{0 if x < 0 else 1}",
        )

    # The two fixed horizontal crossbars that tie the uprights into an H.
    for z, name in ((0.360, "lower_crossbar"), (1.340, "upper_crossbar")):
        frame.visual(
            Box((0.960, 0.070, 0.085)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=beech,
            name=name,
        )

    # Dark steel vertical guide channels mounted proud on the front faces.
    frame.visual(
        Box((0.012, 0.018, 1.430)),
        origin=Origin(xyz=(-0.473, -0.047, 0.980)),
        material=black_steel,
        name="channel_0_inner",
    )
    frame.visual(
        Box((0.012, 0.018, 1.430)),
        origin=Origin(xyz=(-0.427, -0.047, 0.980)),
        material=black_steel,
        name="channel_0_outer",
    )
    frame.visual(
        Box((0.012, 0.018, 1.430)),
        origin=Origin(xyz=(0.427, -0.047, 0.980)),
        material=black_steel,
        name="channel_1_inner",
    )
    frame.visual(
        Box((0.012, 0.018, 1.430)),
        origin=Origin(xyz=(0.473, -0.047, 0.980)),
        material=black_steel,
        name="channel_1_outer",
    )

    # Rear pivot block and a U-shaped fixed sleeve for the telescoping brace.
    pivot = (0.0, 0.100, 1.340)
    brace_angle = -2.447  # local +Z points rearward and downward.
    brace_sin = math.sin(brace_angle)
    brace_cos = math.cos(brace_angle)

    def along_brace(local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
        return (
            pivot[0] + local_x,
            pivot[1] + local_y * brace_cos - local_z * brace_sin,
            pivot[2] + local_y * brace_sin + local_z * brace_cos,
        )

    frame.visual(
        Box((0.180, 0.110, 0.120)),
        origin=Origin(xyz=(0.0, 0.065, 1.340)),
        material=black_steel,
        name="rear_pivot_block",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.210),
        origin=Origin(xyz=(0.0, 0.052, 1.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="pivot_pin",
    )

    # U-channel sleeve: two cheeks plus a rear web, leaving clearance for the
    # rectangular sliding strut.
    frame.visual(
        Box((0.022, 0.056, 0.640)),
        origin=Origin(xyz=along_brace(-0.047, 0.0, 0.320), rpy=(brace_angle, 0.0, 0.0)),
        material=black_steel,
        name="sleeve_cheek_0",
    )
    frame.visual(
        Box((0.022, 0.056, 0.640)),
        origin=Origin(xyz=along_brace(0.047, 0.0, 0.320), rpy=(brace_angle, 0.0, 0.0)),
        material=black_steel,
        name="sleeve_cheek_1",
    )
    frame.visual(
        Box((0.116, 0.014, 0.640)),
        origin=Origin(xyz=along_brace(0.0, 0.035, 0.320), rpy=(brace_angle, 0.0, 0.0)),
        material=black_steel,
        name="sleeve_web",
    )

    # Visible bolt heads where crossbars and channel rails are fixed.
    for i, (x, z) in enumerate(
        [
            (-0.45, 0.360),
            (0.45, 0.360),
            (-0.45, 1.340),
            (0.45, 1.340),
            (-0.45, 0.730),
            (0.45, 0.730),
            (-0.45, 1.670),
            (0.45, 1.670),
        ]
    ):
        frame.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, -0.046, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_steel,
            name=f"bolt_{i}",
        )

    shelf = model.part("shelf")
    shelf.visual(
        Box((1.120, 0.140, 0.060)),
        origin=Origin(xyz=(0.0, -0.145, 0.000)),
        material=beech,
        name="shelf_board",
    )
    shelf.visual(
        Box((1.160, 0.036, 0.090)),
        origin=Origin(xyz=(0.0, -0.215, 0.045)),
        material=endgrain,
        name="canvas_lip",
    )
    shelf.visual(
        Box((0.034, 0.016, 0.240)),
        origin=Origin(xyz=(-0.45, -0.064, 0.000)),
        material=black_steel,
        name="slider_tongue_0",
    )
    shelf.visual(
        Box((0.090, 0.028, 0.170)),
        origin=Origin(xyz=(-0.45, -0.083, 0.000)),
        material=black_steel,
        name="shelf_bracket_0",
    )
    shelf.visual(
        Box((0.034, 0.016, 0.240)),
        origin=Origin(xyz=(0.45, -0.064, 0.000)),
        material=black_steel,
        name="slider_tongue_1",
    )
    shelf.visual(
        Box((0.090, 0.028, 0.170)),
        origin=Origin(xyz=(0.45, -0.083, 0.000)),
        material=black_steel,
        name="shelf_bracket_1",
    )

    model.articulation(
        "frame_to_shelf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=-0.250, upper=0.850),
    )

    rear_strut = model.part("rear_strut")
    rear_strut.visual(
        Box((0.072, 0.028, 1.500)),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=black_steel,
        name="inner_strut",
    )
    rear_strut.visual(
        Box((0.230, 0.065, 0.060)),
        origin=Origin(xyz=(0.0, -0.004, 1.615)),
        material=rubber,
        name="rear_pad",
    )

    model.articulation(
        "frame_to_rear_strut",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rear_strut,
        origin=Origin(xyz=pivot, rpy=(brace_angle, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.15, lower=-0.200, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("front_frame")
    shelf = object_model.get_part("shelf")
    rear_strut = object_model.get_part("rear_strut")
    shelf_slide = object_model.get_articulation("frame_to_shelf")
    strut_slide = object_model.get_articulation("frame_to_rear_strut")

    ctx.expect_contact(
        shelf,
        frame,
        elem_a="slider_tongue_0",
        elem_b="channel_0_outer",
        contact_tol=0.001,
        name="first shelf tongue bears on guide rail",
    )
    ctx.expect_contact(
        shelf,
        frame,
        elem_a="slider_tongue_1",
        elem_b="channel_1_inner",
        contact_tol=0.001,
        name="second shelf tongue bears on guide rail",
    )
    ctx.expect_overlap(
        shelf,
        frame,
        axes="z",
        elem_a="slider_tongue_0",
        elem_b="channel_0_inner",
        min_overlap=0.150,
        name="shelf carriage engages vertical guide",
    )

    base_shelf_pos = ctx.part_world_position(shelf)
    with ctx.pose({shelf_slide: 0.600}):
        raised_shelf_pos = ctx.part_world_position(shelf)
        ctx.expect_overlap(
            shelf,
            frame,
            axes="z",
            elem_a="slider_tongue_0",
            elem_b="channel_0_inner",
            min_overlap=0.150,
            name="raised shelf remains captured in guide",
        )

    ctx.check(
        "shelf moves upward on prismatic channels",
        base_shelf_pos is not None
        and raised_shelf_pos is not None
        and raised_shelf_pos[2] > base_shelf_pos[2] + 0.500,
        details=f"base={base_shelf_pos}, raised={raised_shelf_pos}",
    )

    ctx.expect_within(
        rear_strut,
        frame,
        axes="x",
        inner_elem="inner_strut",
        outer_elem="sleeve_web",
        margin=0.004,
        name="rear strut is centered in telescoping sleeve",
    )
    ctx.expect_contact(
        rear_strut,
        frame,
        elem_a="inner_strut",
        elem_b="sleeve_cheek_0",
        contact_tol=0.001,
        name="rear strut bears on sleeve cheek",
    )
    ctx.expect_overlap(
        rear_strut,
        frame,
        axes="z",
        elem_a="inner_strut",
        elem_b="sleeve_web",
        min_overlap=0.120,
        name="rear strut retains insertion in sleeve",
    )

    base_strut_pos = ctx.part_world_position(rear_strut)
    with ctx.pose({strut_slide: 0.120}):
        extended_strut_pos = ctx.part_world_position(rear_strut)
        ctx.expect_overlap(
            rear_strut,
            frame,
            axes="z",
            elem_a="inner_strut",
            elem_b="sleeve_web",
            min_overlap=0.060,
            name="extended rear strut remains telescoped",
        )

    ctx.check(
        "rear brace extends rearward on prismatic joint",
        base_strut_pos is not None
        and extended_strut_pos is not None
        and extended_strut_pos[1] > base_strut_pos[1] + 0.060,
        details=f"base={base_strut_pos}, extended={extended_strut_pos}",
    )

    return ctx.report()


object_model = build_object_model()

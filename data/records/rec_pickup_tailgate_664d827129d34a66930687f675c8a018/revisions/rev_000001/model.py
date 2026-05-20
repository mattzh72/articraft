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
    model = ArticulatedObject(name="multi_function_tailgate")

    body_paint = model.material("deep_blue_paint", rgba=(0.05, 0.13, 0.25, 1.0))
    dark_paint = model.material("black_plastic", rgba=(0.01, 0.012, 0.014, 1.0))
    steel = model.material("brushed_steel", rgba=(0.56, 0.57, 0.55, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    red_lens = model.material("red_lens", rgba=(0.75, 0.03, 0.02, 1.0))
    amber_lens = model.material("amber_lens", rgba=(1.0, 0.42, 0.03, 1.0))

    bed = model.part("bed")
    bed.visual(
        Box((1.78, 1.25, 0.08)),
        origin=Origin(xyz=(0.0, 0.60, 0.51)),
        material=steel,
        name="bed_floor",
    )
    bed.visual(
        Box((0.11, 1.35, 0.56)),
        origin=Origin(xyz=(-0.89, 0.62, 0.81)),
        material=body_paint,
        name="side_wall_0",
    )
    bed.visual(
        Box((0.11, 1.35, 0.56)),
        origin=Origin(xyz=(0.89, 0.62, 0.81)),
        material=body_paint,
        name="side_wall_1",
    )
    bed.visual(
        Box((1.78, 0.09, 0.56)),
        origin=Origin(xyz=(0.0, 1.28, 0.81)),
        material=body_paint,
        name="front_wall",
    )
    bed.visual(
        Box((1.76, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.00, 0.52)),
        material=body_paint,
        name="rear_sill",
    )
    bed.visual(
        Box((0.15, 0.16, 0.66)),
        origin=Origin(xyz=(-0.91, 0.00, 0.83)),
        material=body_paint,
        name="rear_post_0",
    )
    bed.visual(
        Box((0.15, 0.16, 0.66)),
        origin=Origin(xyz=(0.91, 0.00, 0.83)),
        material=body_paint,
        name="rear_post_1",
    )
    bed.visual(
        Box((0.055, 0.018, 0.34)),
        origin=Origin(xyz=(-0.99, -0.078, 0.89)),
        material=red_lens,
        name="tail_lamp_0",
    )
    bed.visual(
        Box((0.055, 0.018, 0.34)),
        origin=Origin(xyz=(0.99, -0.078, 0.89)),
        material=red_lens,
        name="tail_lamp_1",
    )
    bed.visual(
        Box((0.05, 0.020, 0.055)),
        origin=Origin(xyz=(-0.99, -0.090, 1.05)),
        material=amber_lens,
        name="turn_lamp_0",
    )
    bed.visual(
        Box((0.05, 0.020, 0.055)),
        origin=Origin(xyz=(0.99, -0.090, 1.05)),
        material=amber_lens,
        name="turn_lamp_1",
    )
    bed.visual(
        Box((1.88, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, -0.23, 0.36)),
        material=dark_paint,
        name="step_bumper",
    )
    bed.visual(
        Box((0.16, 0.24, 0.11)),
        origin=Origin(xyz=(-0.45, -0.11, 0.43)),
        material=dark_paint,
        name="bumper_bracket_0",
    )
    bed.visual(
        Box((0.16, 0.24, 0.11)),
        origin=Origin(xyz=(0.45, -0.11, 0.43)),
        material=dark_paint,
        name="bumper_bracket_1",
    )
    bed.visual(
        Box((0.20, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, -0.32, 0.33)),
        material=steel,
        name="receiver_hitch",
    )
    for i, x in enumerate((-0.66, -0.22, 0.22, 0.66)):
        bed.visual(
            Box((0.09, 0.052, 0.10)),
            origin=Origin(xyz=(x, -0.028, 0.55)),
            material=steel,
            name=f"fixed_hinge_lug_{i}",
        )

    main_gate = model.part("main_gate")
    main_gate.visual(
        Box((1.62, 0.060, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=body_paint,
        name="bottom_rail",
    )
    main_gate.visual(
        Box((1.62, 0.060, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=body_paint,
        name="top_rail",
    )
    main_gate.visual(
        Box((0.18, 0.060, 0.58)),
        origin=Origin(xyz=(-0.72, 0.0, 0.29)),
        material=body_paint,
        name="side_stile_0",
    )
    main_gate.visual(
        Box((0.18, 0.060, 0.58)),
        origin=Origin(xyz=(0.72, 0.0, 0.29)),
        material=body_paint,
        name="side_stile_1",
    )
    main_gate.visual(
        Box((0.055, 0.060, 0.410)),
        origin=Origin(xyz=(-0.315, 0.0, 0.300)),
        material=body_paint,
        name="center_jamb_0",
    )
    main_gate.visual(
        Box((0.055, 0.060, 0.410)),
        origin=Origin(xyz=(0.315, 0.0, 0.300)),
        material=body_paint,
        name="center_jamb_1",
    )
    main_gate.visual(
        Box((0.43, 0.012, 0.060)),
        origin=Origin(xyz=(-0.525, -0.036, 0.315)),
        material=dark_paint,
        name="left_recess",
    )
    main_gate.visual(
        Box((0.43, 0.012, 0.060)),
        origin=Origin(xyz=(0.525, -0.036, 0.315)),
        material=dark_paint,
        name="right_recess",
    )
    main_gate.visual(
        Box((0.43, 0.014, 0.024)),
        origin=Origin(xyz=(-0.525, -0.035, 0.132)),
        material=dark_paint,
        name="lower_character_line_0",
    )
    main_gate.visual(
        Box((0.43, 0.014, 0.024)),
        origin=Origin(xyz=(0.525, -0.035, 0.132)),
        material=dark_paint,
        name="lower_character_line_1",
    )
    main_gate.visual(
        Box((0.43, 0.014, 0.024)),
        origin=Origin(xyz=(-0.525, -0.035, 0.468)),
        material=dark_paint,
        name="upper_character_line_0",
    )
    main_gate.visual(
        Box((0.43, 0.014, 0.024)),
        origin=Origin(xyz=(0.525, -0.035, 0.468)),
        material=dark_paint,
        name="upper_character_line_1",
    )
    main_gate.visual(
        Box((0.016, 0.014, 0.340)),
        origin=Origin(xyz=(-0.281, -0.037, 0.305)),
        material=steel,
        name="center_hinge_leaf",
    )
    main_gate.visual(
        Box((0.11, 0.016, 0.050)),
        origin=Origin(xyz=(0.00, -0.034, 0.505)),
        material=dark_paint,
        name="main_latch_handle",
    )
    for i, x in enumerate((-0.53, 0.0, 0.53)):
        main_gate.visual(
            Cylinder(radius=0.024, length=0.30),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"lower_hinge_knuckle_{i}",
        )

    center_door = model.part("center_door")
    center_door.visual(
        Box((0.505, 0.046, 0.340)),
        origin=Origin(xyz=(0.2525, 0.036, 0.170)),
        material=body_paint,
        name="door_skin",
    )
    center_door.visual(
        Box((0.505, 0.010, 0.028)),
        origin=Origin(xyz=(0.2525, 0.006, 0.325)),
        material=rubber,
        name="top_seal",
    )
    center_door.visual(
        Box((0.505, 0.010, 0.028)),
        origin=Origin(xyz=(0.2525, 0.006, 0.015)),
        material=rubber,
        name="bottom_seal",
    )
    center_door.visual(
        Box((0.028, 0.010, 0.340)),
        origin=Origin(xyz=(0.492, 0.006, 0.170)),
        material=rubber,
        name="free_edge_seal",
    )
    center_door.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=steel,
        name="vertical_hinge_barrel",
    )
    center_door.visual(
        Box((0.15, 0.018, 0.052)),
        origin=Origin(xyz=(0.252, 0.005, 0.255)),
        material=dark_paint,
        name="center_latch_handle",
    )
    center_door.visual(
        Box((0.070, 0.012, 0.030)),
        origin=Origin(xyz=(0.472, -0.001, 0.255)),
        material=steel,
        name="striker_plate",
    )

    model.articulation(
        "bed_to_main_gate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=main_gate,
        origin=Origin(xyz=(0.0, -0.082, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "main_gate_to_center_door",
        ArticulationType.REVOLUTE,
        parent=main_gate,
        child=center_door,
        origin=Origin(xyz=(-0.255, -0.036, 0.135)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.65),
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

    bed = object_model.get_part("bed")
    main_gate = object_model.get_part("main_gate")
    center_door = object_model.get_part("center_door")
    main_hinge = object_model.get_articulation("bed_to_main_gate")
    center_hinge = object_model.get_articulation("main_gate_to_center_door")

    ctx.expect_within(
        center_door,
        main_gate,
        axes="xz",
        margin=0.010,
        inner_elem="door_skin",
        name="center door stays within gate width",
    )
    ctx.expect_overlap(
        center_door,
        main_gate,
        axes="y",
        min_overlap=0.020,
        elem_a="door_skin",
        elem_b="center_jamb_1",
        name="center door sits in the tailgate plane",
    )
    ctx.expect_overlap(
        main_gate,
        bed,
        axes="x",
        min_overlap=1.20,
        elem_a="bottom_rail",
        elem_b="rear_sill",
        name="main gate spans the bed opening",
    )

    closed_top = ctx.part_element_world_aabb(main_gate, elem="top_rail")
    with ctx.pose({main_hinge: 1.35}):
        dropped_top = ctx.part_element_world_aabb(main_gate, elem="top_rail")
    ctx.check(
        "main gate drops downward and rearward",
        closed_top is not None
        and dropped_top is not None
        and dropped_top[1][2] < closed_top[1][2] - 0.35
        and dropped_top[0][1] < closed_top[0][1] - 0.35,
        details=f"closed_top={closed_top}, dropped_top={dropped_top}",
    )

    closed_door = ctx.part_element_world_aabb(center_door, elem="door_skin")
    with ctx.pose({center_hinge: 1.25}):
        swung_door = ctx.part_element_world_aabb(center_door, elem="door_skin")
    ctx.check(
        "center door swings out on vertical hinge",
        closed_door is not None
        and swung_door is not None
        and swung_door[0][1] < closed_door[0][1] - 0.25
        and (swung_door[1][0] - swung_door[0][0]) < (closed_door[1][0] - closed_door[0][0]) - 0.15,
        details=f"closed_door={closed_door}, swung_door={swung_door}",
    )

    return ctx.report()


object_model = build_object_model()

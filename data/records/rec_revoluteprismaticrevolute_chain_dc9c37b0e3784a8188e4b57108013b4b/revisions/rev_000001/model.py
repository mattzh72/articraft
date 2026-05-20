from __future__ import annotations

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
    model = ArticulatedObject(name="low_profile_rotary_slide_rotary")

    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark = model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    blue = model.material("blue_anodized_carriage", rgba=(0.10, 0.28, 0.62, 1.0))
    black = model.material("black_fasteners", rgba=(0.015, 0.015, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.50, 0.26, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark,
        name="ground_plate",
    )
    base.visual(
        Cylinder(radius=0.074, length=0.025),
        origin=Origin(xyz=(-0.18, 0.0, 0.0375)),
        material=steel,
        name="fixed_bearing",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.215, -0.095), (-0.215, 0.095), (0.215, -0.095), (0.215, 0.095))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, 0.027)),
            material=black,
            name=f"screw_head_{index}",
        )

    hinged_body = model.part("hinged_body")
    hinged_body.visual(
        Cylinder(radius=0.068, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=steel,
        name="rotary_disk",
    )
    hinged_body.visual(
        Box((0.34, 0.10, 0.024)),
        origin=Origin(xyz=(0.17, 0.0, 0.022)),
        material=dark,
        name="main_arm",
    )
    hinged_body.visual(
        Box((0.250, 0.018, 0.022)),
        origin=Origin(xyz=(0.205, 0.054, 0.045)),
        material=steel,
        name="guide_rail_0",
    )
    hinged_body.visual(
        Box((0.250, 0.018, 0.022)),
        origin=Origin(xyz=(0.205, -0.054, 0.045)),
        material=steel,
        name="guide_rail_1",
    )
    hinged_body.visual(
        Box((0.028, 0.126, 0.024)),
        origin=Origin(xyz=(0.070, 0.0, 0.046)),
        material=steel,
        name="rear_slide_stop",
    )

    linear_stage = model.part("linear_stage")
    linear_stage.visual(
        Box((0.180, 0.065, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=blue,
        name="slider_plate",
    )
    linear_stage.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.105, 0.0, 0.019)),
        material=steel,
        name="wrist_pad",
    )
    linear_stage.visual(
        Box((0.090, 0.018, 0.004)),
        origin=Origin(xyz=(-0.025, 0.0, 0.018)),
        material=steel,
        name="center_wear_strip",
    )

    end_fork = model.part("end_fork")
    end_fork.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="wrist_boss",
    )
    end_fork.visual(
        Box((0.070, 0.060, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, 0.006)),
        material=steel,
        name="fork_bridge",
    )
    end_fork.visual(
        Box((0.105, 0.016, 0.012)),
        origin=Origin(xyz=(0.115, 0.026, 0.006)),
        material=steel,
        name="fork_tine_0",
    )
    end_fork.visual(
        Box((0.105, 0.016, 0.012)),
        origin=Origin(xyz=(0.115, -0.026, 0.006)),
        material=steel,
        name="fork_tine_1",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hinged_body,
        origin=Origin(xyz=(-0.18, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "body_to_stage",
        ArticulationType.PRISMATIC,
        parent=hinged_body,
        child=linear_stage,
        origin=Origin(xyz=(0.190, 0.0, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.090),
    )
    model.articulation(
        "stage_to_fork",
        ArticulationType.REVOLUTE,
        parent=linear_stage,
        child=end_fork,
        origin=Origin(xyz=(0.105, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    hinged_body = object_model.get_part("hinged_body")
    linear_stage = object_model.get_part("linear_stage")
    end_fork = object_model.get_part("end_fork")
    base_to_body = object_model.get_articulation("base_to_body")
    body_to_stage = object_model.get_articulation("body_to_stage")
    stage_to_fork = object_model.get_articulation("stage_to_fork")

    ctx.check(
        "motion order is revolute prismatic revolute",
        base_to_body.articulation_type == ArticulationType.REVOLUTE
        and body_to_stage.articulation_type == ArticulationType.PRISMATIC
        and stage_to_fork.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"types={base_to_body.articulation_type}, "
            f"{body_to_stage.articulation_type}, {stage_to_fork.articulation_type}"
        ),
    )
    ctx.check(
        "single serial chain from base to fork",
        base_to_body.parent == base.name
        and base_to_body.child == hinged_body.name
        and body_to_stage.parent == hinged_body.name
        and body_to_stage.child == linear_stage.name
        and stage_to_fork.parent == linear_stage.name
        and stage_to_fork.child == end_fork.name,
        details="Expected base -> hinged_body -> linear_stage -> end_fork.",
    )

    ctx.expect_contact(
        base,
        hinged_body,
        elem_a="fixed_bearing",
        elem_b="rotary_disk",
        name="base bearing supports hinged body",
    )
    ctx.expect_gap(
        linear_stage,
        hinged_body,
        axis="z",
        positive_elem="slider_plate",
        negative_elem="main_arm",
        max_gap=0.001,
        max_penetration=0.00001,
        name="linear stage rides on the arm bed",
    )
    ctx.expect_within(
        linear_stage,
        hinged_body,
        axes="y",
        inner_elem="slider_plate",
        outer_elem="main_arm",
        margin=0.0,
        name="slider fits between guide rails",
    )
    ctx.expect_overlap(
        linear_stage,
        hinged_body,
        axes="x",
        elem_a="slider_plate",
        elem_b="main_arm",
        min_overlap=0.12,
        name="slider remains captured at rest",
    )
    ctx.expect_contact(
        linear_stage,
        end_fork,
        elem_a="wrist_pad",
        elem_b="wrist_boss",
        name="end fork sits on the wrist pad",
    )

    rest_stage_pos = ctx.part_world_position(linear_stage)
    with ctx.pose({body_to_stage: 0.090}):
        extended_stage_pos = ctx.part_world_position(linear_stage)
        ctx.expect_overlap(
            linear_stage,
            hinged_body,
            axes="x",
            elem_a="slider_plate",
            elem_b="main_arm",
            min_overlap=0.10,
            name="extended slider keeps retained insertion",
        )
        ctx.expect_within(
            linear_stage,
            hinged_body,
            axes="y",
            inner_elem="slider_plate",
            outer_elem="main_arm",
            margin=0.0,
            name="extended slider remains laterally guided",
        )
    ctx.check(
        "prismatic stage extends along the body",
        rest_stage_pos is not None
        and extended_stage_pos is not None
        and extended_stage_pos[0] > rest_stage_pos[0] + 0.08,
        details=f"rest={rest_stage_pos}, extended={extended_stage_pos}",
    )

    with ctx.pose({base_to_body: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(linear_stage, elem="slider_plate")
    with ctx.pose({base_to_body: 0.65}):
        swept_aabb = ctx.part_element_world_aabb(linear_stage, elem="slider_plate")
    if rest_aabb is not None and swept_aabb is not None:
        rest_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
        swept_y = 0.5 * (swept_aabb[0][1] + swept_aabb[1][1])
    else:
        rest_y = swept_y = None
    ctx.check(
        "base rotary joint sweeps the slide body",
        rest_y is not None and swept_y is not None and swept_y > rest_y + 0.06,
        details=f"rest_y={rest_y}, swept_y={swept_y}",
    )

    with ctx.pose({stage_to_fork: 0.0}):
        fork_rest_aabb = ctx.part_element_world_aabb(end_fork, elem="fork_tine_0")
    with ctx.pose({stage_to_fork: 0.80}):
        fork_swept_aabb = ctx.part_element_world_aabb(end_fork, elem="fork_tine_0")
    if fork_rest_aabb is not None and fork_swept_aabb is not None:
        fork_rest_y = 0.5 * (fork_rest_aabb[0][1] + fork_rest_aabb[1][1])
        fork_swept_y = 0.5 * (fork_swept_aabb[0][1] + fork_swept_aabb[1][1])
    else:
        fork_rest_y = fork_swept_y = None
    ctx.check(
        "end fork rotary joint yaws the tines",
        fork_rest_y is not None
        and fork_swept_y is not None
        and fork_swept_y > fork_rest_y + 0.06,
        details=f"rest_y={fork_rest_y}, swept_y={fork_swept_y}",
    )

    return ctx.report()


object_model = build_object_model()

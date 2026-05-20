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
    model = ArticulatedObject(name="two_link_elbow_arm")

    base_mat = model.material("satin_grey", rgba=(0.45, 0.47, 0.48, 1.0))
    link_mat = model.material("blue_painted_aluminum", rgba=(0.05, 0.22, 0.58, 1.0))
    joint_mat = model.material("brushed_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    pad_mat = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    shoulder_height = 0.270
    upper_length = 0.420
    forearm_length = 0.355

    base = model.part("base")
    base.visual(
        Box((0.320, 0.240, 0.035)),
        origin=Origin(xyz=(0.000, 0.000, 0.0175)),
        material=base_mat,
        name="base_plate",
    )
    base.visual(
        Box((0.110, 0.120, 0.190)),
        origin=Origin(xyz=(0.000, 0.000, 0.130)),
        material=base_mat,
        name="pedestal",
    )
    for side, y in (("cheek_0", -0.055), ("cheek_1", 0.055)):
        base.visual(
            Box((0.080, 0.026, 0.130)),
            origin=Origin(xyz=(0.000, y, shoulder_height)),
            material=base_mat,
            name=f"shoulder_{side}",
        )
    for side, y in (("cap_0", -0.077), ("cap_1", 0.077)):
        base.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(0.000, y, shoulder_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=joint_mat,
            name=f"shoulder_{side}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.028, length=0.084),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Box((0.350, 0.050, 0.045)),
        origin=Origin(xyz=(0.180, 0.000, 0.000)),
        material=link_mat,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.052, 0.100, 0.046)),
        origin=Origin(xyz=(upper_length - 0.055, 0.000, 0.000)),
        material=link_mat,
        name="elbow_bridge",
    )
    for side, y in (("cheek_0", -0.050), ("cheek_1", 0.050)):
        upper_arm.visual(
            Box((0.075, 0.024, 0.075)),
            origin=Origin(xyz=(upper_length, y, 0.000)),
            material=link_mat,
            name=f"elbow_{side}",
        )
    for side, y in (("cap_0", -0.071), ("cap_1", 0.071)):
        upper_arm.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=Origin(xyz=(upper_length, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=joint_mat,
            name=f"elbow_{side}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.024, length=0.076),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.320, 0.042, 0.038)),
        origin=Origin(xyz=(0.175, 0.000, 0.000)),
        material=link_mat,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.032, 0.076, 0.060)),
        origin=Origin(xyz=(forearm_length - 0.030, 0.000, 0.000)),
        material=joint_mat,
        name="tool_flange",
    )
    forearm.visual(
        Box((0.026, 0.105, 0.085)),
        origin=Origin(xyz=(forearm_length - 0.004, 0.000, 0.000)),
        material=pad_mat,
        name="tool_pad",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.000, 0.000, shoulder_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.0, upper=1.45),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(upper_length, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.5, lower=0.0, upper=2.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check(
        "shoulder and elbow are parallel horizontal revolutes",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tuple(shoulder.axis) == (0.0, -1.0, 0.0)
        and tuple(elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder.articulation_type}/{shoulder.axis}, elbow={elbow.articulation_type}/{elbow.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_within(
            upper_arm,
            base,
            axes="y",
            inner_elem="shoulder_barrel",
            outer_elem="base_plate",
            margin=0.090,
            name="shoulder barrel sits within base width",
        )
        ctx.expect_gap(
            upper_arm,
            base,
            axis="z",
            positive_elem="shoulder_barrel",
            negative_elem="pedestal",
            min_gap=0.010,
            max_gap=0.030,
            name="shoulder barrel clears pedestal top",
        )
        ctx.expect_within(
            forearm,
            upper_arm,
            axes="y",
            inner_elem="elbow_barrel",
            outer_elem="elbow_bridge",
            margin=0.010,
            name="elbow barrel is captured between fork cheeks",
        )
        ctx.expect_overlap(
            forearm,
            upper_arm,
            axes="x",
            elem_a="elbow_barrel",
            elem_b="elbow_cheek_0",
            min_overlap=0.020,
            name="elbow barrel aligns with fork pivot",
        )

    rest_tip = ctx.part_element_world_aabb(forearm, elem="tool_pad")
    with ctx.pose({shoulder: 0.45, elbow: 0.90}):
        raised_tip = ctx.part_element_world_aabb(forearm, elem="tool_pad")
    ctx.check(
        "tool pad rises when both joints flex upward",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[0][2] + 0.10,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()

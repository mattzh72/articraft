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
    model = ArticulatedObject(name="two_link_hinged_arm")

    painted_steel = Material("painted_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    link_blue = Material("anodized_blue", rgba=(0.05, 0.26, 0.62, 1.0))
    pin_metal = Material("brushed_pin", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.22, 0.16, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=painted_steel,
        name="base_plate",
    )
    for y, name in ((0.059, "cheek_0"), (-0.059, "cheek_1")):
        base.visual(
            Box((0.110, 0.018, 0.161)),
            origin=Origin(xyz=(0.0, y, 0.1045)),
            material=painted_steel,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.011, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="base_pin",
    )
    for y, name in ((0.080, "pin_head_0"), (-0.080, "pin_head_1")):
        base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_metal,
            name=name,
        )
    for x in (-0.073, 0.073):
        for y in (-0.047, 0.047):
            base.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, y, 0.025)),
                material=pin_metal,
                name=f"mount_screw_{x:+.3f}_{y:+.3f}",
            )

    primary = model.part("primary_link")
    primary.visual(
        Cylinder(radius=0.043, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="shoulder_boss",
    )
    primary.visual(
        Box((0.370, 0.034, 0.032)),
        origin=Origin(xyz=(0.220, 0.018, 0.0)),
        material=link_blue,
        name="primary_web",
    )
    primary.visual(
        Box((0.285, 0.040, 0.008)),
        origin=Origin(xyz=(0.225, 0.018, 0.019)),
        material=link_blue,
        name="raised_spine",
    )
    primary.visual(
        Cylinder(radius=0.036, length=0.045),
        origin=Origin(xyz=(0.420, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="elbow_lug",
    )
    primary.visual(
        Cylinder(radius=0.0105, length=0.120),
        origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="elbow_pin",
    )

    secondary = model.part("secondary_link")
    secondary.visual(
        Cylinder(radius=0.034, length=0.045),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="elbow_bearing",
    )
    secondary.visual(
        Box((0.300, 0.030, 0.028)),
        origin=Origin(xyz=(0.165, -0.030, 0.0)),
        material=link_blue,
        name="secondary_web",
    )
    secondary.visual(
        Box((0.050, 0.065, 0.055)),
        origin=Origin(xyz=(0.335, -0.030, 0.0)),
        material=rubber,
        name="end_pad",
    )

    model.articulation(
        "base_to_primary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.0, upper=1.25),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-1.45, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    shoulder = object_model.get_articulation("base_to_primary")
    elbow = object_model.get_articulation("primary_to_secondary")

    ctx.allow_overlap(
        base,
        primary,
        elem_a="base_pin",
        elem_b="shoulder_boss",
        reason="The base pin is intentionally captured through the primary link shoulder bearing.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="elbow_bearing",
        reason="The elbow pin is intentionally captured through the secondary link bearing.",
    )

    ctx.expect_within(
        base,
        primary,
        axes="xz",
        inner_elem="base_pin",
        outer_elem="shoulder_boss",
        margin=0.001,
        name="base pin is centered in shoulder bearing",
    )
    ctx.expect_overlap(
        base,
        primary,
        axes="y",
        elem_a="base_pin",
        elem_b="shoulder_boss",
        min_overlap=0.060,
        name="shoulder bearing is retained on base pin",
    )
    ctx.expect_within(
        primary,
        secondary,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_bearing",
        margin=0.001,
        name="elbow pin is centered in secondary bearing",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_bearing",
        min_overlap=0.040,
        name="secondary bearing is retained on elbow pin",
    )

    ctx.check(
        "two parallel revolute pin axes",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tuple(shoulder.axis) == tuple(elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )

    rest_elbow_pos = ctx.part_world_position(secondary)
    with ctx.pose({shoulder: 0.65}):
        raised_elbow_pos = ctx.part_world_position(secondary)
    ctx.check(
        "primary joint raises the elbow",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.08,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_pad = ctx.part_element_world_aabb(secondary, elem="end_pad")
    with ctx.pose({elbow: 0.75}):
        raised_pad = ctx.part_element_world_aabb(secondary, elem="end_pad")
    if rest_pad is not None and raised_pad is not None:
        rest_pad_z = (rest_pad[0][2] + rest_pad[1][2]) * 0.5
        raised_pad_z = (raised_pad[0][2] + raised_pad[1][2]) * 0.5
    else:
        rest_pad_z = raised_pad_z = None
    ctx.check(
        "secondary joint raises the end pad",
        rest_pad_z is not None and raised_pad_z is not None and raised_pad_z > rest_pad_z + 0.08,
        details=f"rest_z={rest_pad_z}, raised_z={raised_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()

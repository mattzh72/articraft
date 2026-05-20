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


PIVOT_Z = 0.125


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_pitch_cradle")

    graphite = model.material("graphite_powdercoat", rgba=(0.10, 0.11, 0.12, 1.0))
    gunmetal = model.material("dark_gunmetal", rgba=(0.18, 0.19, 0.20, 1.0))
    face_black = model.material("plain_black_faceplate", rgba=(0.015, 0.016, 0.017, 1.0))
    pivot_steel = model.material("brushed_pivot_steel", rgba=(0.55, 0.56, 0.54, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.420, 0.300, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Box((0.260, 0.180, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=graphite,
        name="raised_pad",
    )
    for y, name in ((0.098, "yoke_cheek_0"), (-0.098, "yoke_cheek_1")):
        base.visual(
            Box((0.105, 0.028, 0.120)),
            origin=Origin(xyz=(0.0, y, 0.118)),
            material=graphite,
            name=name,
        )
    for y, name in ((0.082, "bearing_pad_0"), (-0.082, "bearing_pad_1")):
        base.visual(
            Cylinder(radius=0.036, length=0.008),
            origin=Origin(xyz=(0.0, y, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pivot_steel,
            name=name,
        )

    head = model.part("head")
    head.visual(
        Box((0.130, 0.118, 0.090)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=gunmetal,
        name="head_shell",
    )
    head.visual(
        Box((0.010, 0.094, 0.064)),
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
        material=face_black,
        name="faceplate",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.156),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="pitch_pin",
    )
    for y, name in ((0.068, "pivot_hub_0"), (-0.068, "pivot_hub_1")):
        head.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pivot_steel,
            name=name,
        )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        # Positive pitch lifts the plain faceplate; negative pitch noses it down.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.25, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("base_to_head")

    ctx.check(
        "single pitch revolute joint",
        len(object_model.articulations) == 1,
        details=f"articulations={len(object_model.articulations)}",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="pitch_pin",
        elem_b="bearing_pad_0",
        contact_tol=0.001,
        name="pin seats in first yoke bearing",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="pitch_pin",
        elem_b="bearing_pad_1",
        contact_tol=0.001,
        name="pin seats in second yoke bearing",
    )

    rest_face = ctx.part_element_world_aabb(head, elem="faceplate")
    with ctx.pose({pitch: 0.55}):
        raised_face = ctx.part_element_world_aabb(head, elem="faceplate")
    with ctx.pose({pitch: -0.25}):
        lowered_face = ctx.part_element_world_aabb(head, elem="faceplate")

    def _center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_z = _center_z(rest_face)
    raised_z = _center_z(raised_face)
    lowered_z = _center_z(lowered_face)
    ctx.check(
        "positive pitch raises faceplate",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.025,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )
    ctx.check(
        "negative pitch lowers faceplate",
        rest_z is not None and lowered_z is not None and lowered_z < rest_z - 0.010,
        details=f"rest_z={rest_z}, lowered_z={lowered_z}",
    )

    return ctx.report()


object_model = build_object_model()

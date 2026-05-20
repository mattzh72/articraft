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


def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp")

    steel = model.material("steel", rgba=(0.23, 0.24, 0.26, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    diffuser = model.material("diffuser", rgba=(0.93, 0.94, 0.90, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.30, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="base_plate",
    )
    stand.visual(
        Box((0.06, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=steel,
        name="post_socket",
    )
    stand.visual(
        Cylinder(radius=0.0135, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.588)),
        material=aluminum,
        name="post_shaft",
    )
    stand.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(-0.015, 0.0, 1.123)),
        material=steel,
        name="top_yoke",
    )

    neck = model.part("neck")
    neck.visual(
        Box((0.025, 0.05, 0.05)),
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
        material=steel,
        name="neck_hub",
    )
    neck.visual(
        Box((0.085, 0.032, 0.028)),
        origin=Origin(xyz=(0.0425, 0.0, 0.0)),
        material=aluminum,
        name="neck_arm",
    )
    neck.visual(
        Box((0.010, 0.05, 0.040)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=steel,
        name="lamp_mount",
    )

    lamp_bar = model.part("lamp_bar")
    lamp_bar.visual(
        Box((0.014, 0.050, 0.040)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=steel,
        name="hinge_block",
    )
    lamp_bar.visual(
        Box((0.44, 0.055, 0.028)),
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        material=aluminum,
        name="bar_shell",
    )
    lamp_bar.visual(
        Box((0.050, 0.060, 0.034)),
        origin=Origin(xyz=(0.415, 0.0, 0.0)),
        material=aluminum,
        name="lamp_head",
    )
    lamp_bar.visual(
        Box((0.38, 0.043, 0.004)),
        origin=Origin(xyz=(0.230, 0.0, -0.016)),
        material=diffuser,
        name="diffuser",
    )

    model.articulation(
        "stand_to_neck",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 1.123)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.85,
            upper=0.95,
        ),
    )
    model.articulation(
        "neck_to_lamp_bar",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=lamp_bar,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.80,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    neck = object_model.get_part("neck")
    lamp_bar = object_model.get_part("lamp_bar")
    stand_to_neck = object_model.get_articulation("stand_to_neck")
    neck_to_lamp_bar = object_model.get_articulation("neck_to_lamp_bar")

    ctx.expect_contact(
        neck,
        stand,
        elem_a="neck_hub",
        elem_b="top_yoke",
        name="neck mounts directly to the post head",
    )
    ctx.expect_contact(
        lamp_bar,
        neck,
        elem_a="hinge_block",
        elem_b="lamp_mount",
        name="lamp bar mounts directly to the neck",
    )
    ctx.expect_gap(
        lamp_bar,
        stand,
        axis="z",
        positive_elem="bar_shell",
        negative_elem="base_plate",
        min_gap=1.08,
        name="lamp bar sits well above the flat floor base",
    )
    ctx.expect_origin_gap(
        lamp_bar,
        stand,
        axis="x",
        min_gap=0.07,
        name="lamp assembly projects forward from the post",
    )

    rest_bar = ctx.part_element_world_aabb(lamp_bar, elem="bar_shell")
    rest_center_z = _center_z(rest_bar)

    with ctx.pose({stand_to_neck: 0.55}):
        raised_bar = ctx.part_element_world_aabb(lamp_bar, elem="bar_shell")
        raised_center_z = _center_z(raised_bar)
    ctx.check(
        "neck hinge lifts the lamp bar",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.035,
        details=f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}",
    )

    with ctx.pose({neck_to_lamp_bar: -0.55}):
        tilted_bar = ctx.part_element_world_aabb(lamp_bar, elem="bar_shell")
        tilted_center_z = _center_z(tilted_bar)
    ctx.check(
        "lamp bar tilts downward from the neck hinge",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z < rest_center_z - 0.030,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )

    with ctx.pose({stand_to_neck: -0.45, neck_to_lamp_bar: -0.55}):
        ctx.expect_gap(
            lamp_bar,
            stand,
            axis="z",
            positive_elem="diffuser",
            negative_elem="base_plate",
            min_gap=0.68,
            name="reading pose keeps the lamp head above the base",
        )

    return ctx.report()


object_model = build_object_model()

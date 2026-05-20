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
    model = ArticulatedObject(name="pedestrian_barrier_gate")

    safety_orange = model.material("safety_orange", rgba=(1.0, 0.45, 0.05, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    white_paint = model.material("white_paint", rgba=(0.96, 0.96, 0.90, 1.0))
    reflector_red = model.material("reflector_red", rgba=(0.90, 0.02, 0.02, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.16, 0.17, 0.18, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.42, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=black_rubber,
        name="plinth",
    )
    cabinet.visual(
        Box((0.36, 0.28, 0.828)),
        origin=Origin(xyz=(0.0, 0.0, 0.492)),
        material=safety_orange,
        name="body_shell",
    )
    cabinet.visual(
        Box((0.39, 0.31, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.943)),
        material=dark_metal,
        name="top_cap",
    )
    cabinet.visual(
        Box((0.25, 0.014, 0.46)),
        origin=Origin(xyz=(-0.02, -0.145, 0.48)),
        material=panel_gray,
        name="front_access_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.075, -0.157, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="round_lock",
    )
    cabinet.visual(
        Box((0.18, 0.030, 0.18)),
        origin=Origin(xyz=(0.18, -0.151, 0.78)),
        material=dark_metal,
        name="hinge_backplate",
    )
    cabinet.visual(
        Cylinder(radius=0.035, length=0.062),
        origin=Origin(xyz=(0.18, -0.194, 0.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="fixed_shaft",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.075, length=0.045),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="moving_hub",
    )
    arm.visual(
        Box((1.45, 0.070, 0.055)),
        origin=Origin(xyz=(0.790, 0.0, 0.0)),
        material=white_paint,
        name="bar",
    )
    for index, x in enumerate((0.34, 0.70, 1.06, 1.42)):
        arm.visual(
            Box((0.16, 0.074, 0.059)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=reflector_red,
            name=f"red_band_{index}",
        )
    arm.visual(
        Box((0.055, 0.078, 0.063)),
        origin=Origin(xyz=(1.540, 0.0, 0.0)),
        material=reflector_red,
        name="end_cap",
    )

    model.articulation(
        "cabinet_to_arm",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=arm,
        origin=Origin(xyz=(0.18, -0.2475, 0.78)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    arm = object_model.get_part("arm")
    hinge = object_model.get_articulation("cabinet_to_arm")

    ctx.check(
        "arm hinge has 90 degree travel",
        hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-6
        and abs(hinge.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={hinge.motion_limits}",
    )

    ctx.expect_contact(
        cabinet,
        arm,
        elem_a="fixed_shaft",
        elem_b="moving_hub",
        contact_tol=0.001,
        name="hinge hub is seated on fixed shaft",
    )
    ctx.expect_gap(
        arm,
        cabinet,
        axis="x",
        min_gap=0.035,
        positive_elem="bar",
        negative_elem="body_shell",
        name="closed bar clears cabinet side",
    )

    closed_bar_aabb = ctx.part_element_world_aabb(arm, elem="bar")
    with ctx.pose({hinge: math.pi / 2.0}):
        raised_bar_aabb = ctx.part_element_world_aabb(arm, elem="bar")

    ctx.check(
        "bar swings from horizontal to vertical",
        closed_bar_aabb is not None
        and raised_bar_aabb is not None
        and (closed_bar_aabb[1][0] - closed_bar_aabb[0][0]) > 1.35
        and (raised_bar_aabb[1][2] - raised_bar_aabb[0][2]) > 1.35
        and raised_bar_aabb[1][2] > 2.20,
        details=f"closed={closed_bar_aabb}, raised={raised_bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

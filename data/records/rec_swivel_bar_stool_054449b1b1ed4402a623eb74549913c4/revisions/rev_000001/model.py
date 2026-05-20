from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _seat_outline(front_x: float, rear_x: float, half_width: float) -> list[tuple[float, float]]:
    return [
        (front_x, 0.0),
        (front_x * 0.82, half_width * 0.82),
        (front_x * 0.30, half_width),
        (-rear_x * 0.38, half_width * 0.94),
        (-rear_x, 0.0),
        (-rear_x * 0.38, -half_width * 0.94),
        (front_x * 0.30, -half_width),
        (front_x * 0.82, -half_width * 0.82),
    ]


def _seat_shape():
    lower = _seat_outline(0.168, 0.150, 0.170)
    mid = _seat_outline(0.182, 0.162, 0.186)
    top = _seat_outline(0.194, 0.176, 0.205)

    seat = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .spline(lower)
        .close()
        .workplane(offset=0.034)
        .spline(mid)
        .close()
        .workplane(offset=0.076)
        .spline(top)
        .close()
        .loft(combine=True)
    )

    primary_saddle = cq.Workplane("XY").sphere(0.24).translate((0.0, 0.0, 0.22))
    rear_pocket = cq.Workplane("XY").sphere(0.17).translate((-0.040, 0.0, 0.168))
    return seat.cut(primary_saddle).cut(rear_pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.215, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.207, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="floor_pad",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0515)),
        material=brushed_steel,
        name="lower_shroud",
    )
    base.visual(
        Cylinder(radius=0.043, length=0.521),
        origin=Origin(xyz=(0.0, 0.0, 0.3395)),
        material=brushed_steel,
        name="pedestal_post",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=dark_steel,
        name="step_collar",
    )
    base.visual(
        Box((0.014, 0.060, 0.018)),
        origin=Origin(xyz=(0.053, 0.0, 0.273)),
        material=dark_steel,
        name="step_mount",
    )
    base.visual(
        Box((0.018, 0.020, 0.032)),
        origin=Origin(xyz=(0.068, 0.028, 0.285)),
        material=dark_steel,
        name="hinge_ear_0",
    )
    base.visual(
        Box((0.018, 0.020, 0.032)),
        origin=Origin(xyz=(0.068, -0.028, 0.285)),
        material=dark_steel,
        name="hinge_ear_1",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=dark_steel,
        name="swivel_housing",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_seat_shape(), "seat_shell"),
        material=black_vinyl,
        name="seat_shell",
    )
    seat.visual(
        Box((0.180, 0.240, 0.032)),
        origin=Origin(xyz=(0.006, 0.0, 0.016)),
        material=black_vinyl,
        name="seat_core",
    )
    seat.visual(
        Cylinder(radius=0.090, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="seat_pan",
    )
    seat.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=brushed_steel,
        name="seat_hub",
    )

    front_step = model.part("front_step")
    front_step.visual(
        Cylinder(radius=0.0075, length=0.036),
        origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
        material=brushed_steel,
        name="step_barrel",
    )
    front_step.visual(
        Box((0.060, 0.024, 0.030)),
        origin=Origin(xyz=(0.032, 0.0, -0.010)),
        material=dark_steel,
        name="step_arm",
    )
    front_step.visual(
        Box((0.120, 0.090, 0.014)),
        origin=Origin(xyz=(0.072, 0.0, -0.016)),
        material=dark_steel,
        name="step_tread",
    )
    front_step.visual(
        Box((0.070, 0.010, 0.004)),
        origin=Origin(xyz=(0.082, 0.020, -0.007)),
        material=rubber,
        name="grip_strip_0",
    )
    front_step.visual(
        Box((0.070, 0.010, 0.004)),
        origin=Origin(xyz=(0.082, -0.020, -0.007)),
        material=rubber,
        name="grip_strip_1",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )
    model.articulation(
        "step_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=front_step,
        origin=Origin(xyz=(0.070, 0.0, 0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    front_step = object_model.get_part("front_step")
    seat_swivel = object_model.get_articulation("seat_swivel")
    step_hinge = object_model.get_articulation("step_hinge")

    ctx.expect_origin_gap(
        seat,
        base,
        axis="z",
        min_gap=0.61,
        max_gap=0.64,
        name="counter height seat elevation",
    )

    with ctx.pose({seat_swivel: 0.0}):
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="seat_hub",
            negative_elem="swivel_housing",
            max_gap=0.002,
            max_penetration=0.0,
            name="seat hub rests on swivel housing",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            elem_a="seat_hub",
            elem_b="swivel_housing",
            min_overlap=0.050,
            name="seat hub stays centered on swivel housing",
        )

    with ctx.pose({seat_swivel: 1.8}):
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="seat_hub",
            negative_elem="swivel_housing",
            max_gap=0.002,
            max_penetration=0.0,
            name="swiveled seat keeps vertical support",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            elem_a="seat_hub",
            elem_b="swivel_housing",
            min_overlap=0.050,
            name="swiveled seat stays centered on axis",
        )

    deployed_tread_aabb = None
    with ctx.pose({step_hinge: 0.0}):
        ctx.expect_gap(
            front_step,
            base,
            axis="x",
            positive_elem="step_tread",
            negative_elem="pedestal_post",
            min_gap=0.035,
            name="deployed step projects ahead of pedestal",
        )
        ctx.expect_gap(
            front_step,
            base,
            axis="z",
            positive_elem="step_tread",
            negative_elem="base_plate",
            min_gap=0.22,
            name="deployed step clears the circular base",
        )
        deployed_tread_aabb = ctx.part_element_world_aabb(front_step, elem="step_tread")

    folded_tread_aabb = None
    with ctx.pose({step_hinge: -1.15}):
        ctx.expect_gap(
            front_step,
            base,
            axis="x",
            positive_elem="step_tread",
            negative_elem="pedestal_post",
            min_gap=0.002,
            name="folded step still clears the pedestal post",
        )
        folded_tread_aabb = ctx.part_element_world_aabb(front_step, elem="step_tread")

    ctx.check(
        "step folds downward toward the post",
        deployed_tread_aabb is not None
        and folded_tread_aabb is not None
        and folded_tread_aabb[0][2] < deployed_tread_aabb[0][2] - 0.09
        and folded_tread_aabb[1][0] < deployed_tread_aabb[1][0] - 0.07,
        details=f"deployed={deployed_tread_aabb}, folded={folded_tread_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

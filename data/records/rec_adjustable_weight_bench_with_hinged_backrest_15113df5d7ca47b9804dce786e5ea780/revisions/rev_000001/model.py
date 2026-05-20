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


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_incline_utility_bench")

    frame = model.material("frame", rgba=(0.13, 0.14, 0.15, 1.0))
    accent = model.material("accent", rgba=(0.28, 0.29, 0.31, 1.0))
    upholstery = model.material("upholstery", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.64, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    knob_red = model.material("knob_red", rgba=(0.67, 0.14, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.09, 0.58, 0.05)),
        origin=Origin(xyz=(-0.24, 0.0, 0.025)),
        material=frame,
        name="front_foot",
    )
    base.visual(
        Box((0.10, 0.64, 0.05)),
        origin=Origin(xyz=(0.53, 0.0, 0.025)),
        material=frame,
        name="rear_foot",
    )
    for x, y, sx, sy in (
        (-0.24, -0.25, 0.05, 0.06),
        (-0.24, 0.25, 0.05, 0.06),
        (0.53, -0.27, 0.06, 0.07),
        (0.53, 0.27, 0.06, 0.07),
    ):
        base.visual(
            Box((sx, sy, 0.008)),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=None,
        )

    _add_tube(
        base,
        (-0.24, -0.11, 0.068),
        (0.53, -0.11, 0.068),
        radius=0.018,
        material=frame,
        name="lower_rail_0",
    )
    _add_tube(
        base,
        (-0.24, 0.11, 0.068),
        (0.53, 0.11, 0.068),
        radius=0.018,
        material=frame,
        name="lower_rail_1",
    )
    _add_tube(
        base,
        (-0.19, -0.11, 0.05),
        (-0.19, -0.11, 0.39),
        radius=0.018,
        material=frame,
        name="front_post_0",
    )
    _add_tube(
        base,
        (-0.19, 0.11, 0.05),
        (-0.19, 0.11, 0.39),
        radius=0.018,
        material=frame,
        name="front_post_1",
    )
    _add_tube(
        base,
        (0.05, -0.11, 0.068),
        (0.05, -0.11, 0.39),
        radius=0.018,
        material=frame,
        name="junction_post_0",
    )
    _add_tube(
        base,
        (0.05, 0.11, 0.068),
        (0.05, 0.11, 0.39),
        radius=0.018,
        material=frame,
        name="junction_post_1",
    )
    _add_tube(
        base,
        (-0.19, -0.11, 0.39),
        (0.05, -0.11, 0.39),
        radius=0.017,
        material=frame,
        name="seat_rail_0",
    )
    _add_tube(
        base,
        (-0.19, 0.11, 0.39),
        (0.05, 0.11, 0.39),
        radius=0.017,
        material=frame,
        name="seat_rail_1",
    )
    _add_tube(
        base,
        (-0.19, -0.11, 0.39),
        (-0.19, 0.11, 0.39),
        radius=0.016,
        material=frame,
        name="front_hinge_crossbar",
    )
    _add_tube(
        base,
        (0.05, -0.11, 0.39),
        (0.05, 0.11, 0.39),
        radius=0.016,
        material=frame,
        name="backrest_hinge_crossbar",
    )
    _add_tube(
        base,
        (-0.19, -0.11, 0.068),
        (-0.19, 0.11, 0.068),
        radius=0.014,
        material=frame,
        name="front_lower_crossbar",
    )
    _add_tube(
        base,
        (0.05, -0.11, 0.068),
        (0.05, 0.11, 0.068),
        radius=0.014,
        material=frame,
        name="center_lower_crossbar",
    )
    _add_tube(
        base,
        (0.05, -0.11, 0.39),
        (0.45, -0.11, 0.15),
        radius=0.017,
        material=frame,
        name="rear_brace_0",
    )
    _add_tube(
        base,
        (0.05, 0.11, 0.39),
        (0.45, 0.11, 0.15),
        radius=0.017,
        material=frame,
        name="rear_brace_1",
    )
    _add_tube(
        base,
        (0.45, -0.11, 0.15),
        (0.53, -0.11, 0.068),
        radius=0.016,
        material=frame,
        name="rear_drop_0",
    )
    _add_tube(
        base,
        (0.45, 0.11, 0.15),
        (0.53, 0.11, 0.068),
        radius=0.016,
        material=frame,
        name="rear_drop_1",
    )
    _add_tube(
        base,
        (0.18, 0.11, 0.068),
        (0.18, 0.20, 0.068),
        radius=0.016,
        material=frame,
        name="ladder_mount_outtrigger",
    )
    _add_tube(
        base,
        (0.53, -0.11, 0.068),
        (0.53, 0.11, 0.068),
        radius=0.014,
        material=frame,
        name="rear_lower_crossbar",
    )
    base.visual(
        Box((0.22, 0.22, 0.022)),
        origin=Origin(xyz=(-0.07, 0.0, 0.349)),
        material=accent,
        name="seat_platform",
    )
    base.visual(
        Box((0.085, 0.100, 0.048)),
        origin=Origin(xyz=(0.18, 0.20, 0.076)),
        material=accent,
        name="ladder_pivot_mount",
    )
    base.visual(
        Box((0.090, 0.012, 0.072)),
        origin=Origin(xyz=(0.18, 0.162, 0.126)),
        material=accent,
        name="ladder_clevis_inner",
    )
    base.visual(
        Box((0.090, 0.012, 0.072)),
        origin=Origin(xyz=(0.18, 0.238, 0.126)),
        material=accent,
        name="ladder_clevis_outer",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    seat.visual(
        Box((0.030, 0.020, 0.028)),
        origin=Origin(xyz=(0.015, -0.060, 0.014)),
        material=steel,
        name="hinge_lug_0",
    )
    seat.visual(
        Box((0.030, 0.020, 0.028)),
        origin=Origin(xyz=(0.015, 0.060, 0.014)),
        material=steel,
        name="hinge_lug_1",
    )
    seat.visual(
        Box((0.205, 0.285, 0.012)),
        origin=Origin(xyz=(0.1175, 0.0, 0.006)),
        material=accent,
        name="backer",
    )
    seat.visual(
        Box((0.215, 0.295, 0.055)),
        origin=Origin(xyz=(0.1225, 0.0, 0.0395)),
        material=upholstery,
        name="upholstery",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(0.024, 0.0, 0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    backrest.visual(
        Box((0.040, 0.020, 0.028)),
        origin=Origin(xyz=(0.024, -0.060, 0.014)),
        material=steel,
        name="hinge_lug_0",
    )
    backrest.visual(
        Box((0.040, 0.020, 0.028)),
        origin=Origin(xyz=(0.024, 0.060, 0.014)),
        material=steel,
        name="hinge_lug_1",
    )
    backrest.visual(
        Box((0.77, 0.29, 0.012)),
        origin=Origin(xyz=(0.405, 0.0, 0.006)),
        material=accent,
        name="backer",
    )
    backrest.visual(
        Box((0.80, 0.30, 0.055)),
        origin=Origin(xyz=(0.420, 0.0, 0.0395)),
        material=upholstery,
        name="upholstery",
    )
    backrest.visual(
        Box((0.46, 0.014, 0.040)),
        origin=Origin(xyz=(0.25, 0.145, -0.020)),
        material=frame,
        name="selector_plate_upper",
    )
    backrest.visual(
        Box((0.40, 0.014, 0.034)),
        origin=Origin(xyz=(0.31, 0.145, -0.110)),
        material=frame,
        name="selector_plate_lower",
    )
    backrest.visual(
        Box((0.10, 0.014, 0.076)),
        origin=Origin(xyz=(0.07, 0.145, -0.058)),
        material=frame,
        name="selector_plate_front_bridge",
    )
    backrest.visual(
        Box((0.06, 0.014, 0.060)),
        origin=Origin(xyz=(0.49, 0.145, -0.066)),
        material=frame,
        name="selector_plate_rear_bridge",
    )
    backrest.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.29, 0.145, -0.068), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="selector_bushing",
    )
    backrest.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.29, 0.144, -0.068)),
        material=steel,
        name="selector_boss",
    )
    backrest.visual(
        Box((0.030, 0.018, 0.028)),
        origin=Origin(xyz=(0.29, 0.144, -0.046)),
        material=steel,
        name="selector_stem_block",
    )

    ladder = model.part("ladder")
    ladder.visual(
        Cylinder(radius=0.013, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    _add_tube(
        ladder,
        (0.0, -0.022, 0.0),
        (0.22, -0.022, 0.19),
        radius=0.009,
        material=steel,
        name="ladder_rail_inner",
    )
    _add_tube(
        ladder,
        (0.0, 0.022, 0.0),
        (0.22, 0.022, 0.19),
        radius=0.009,
        material=steel,
        name="ladder_rail_outer",
    )
    for index, t in enumerate((0.22, 0.45, 0.68, 0.88)):
        x = 0.22 * t
        z = 0.19 * t
        _add_tube(
            ladder,
            (x, -0.022, z),
            (x, 0.022, z),
            radius=0.007,
            material=steel,
            name=f"rung_{index}",
        )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="collar",
    )
    selector_knob.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_red,
        name="pull_knob",
    )

    model.articulation(
        "base_to_backrest",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(0.05, 0.0, 0.405)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=1.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "base_to_seat",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(-0.18, 0.0, 0.405)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=0.0,
            upper=0.32,
        ),
    )
    model.articulation(
        "base_to_ladder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=ladder,
        origin=Origin(xyz=(0.20, 0.20, 0.14)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=-0.35,
            upper=0.45,
        ),
    )
    model.articulation(
        "backrest_to_selector_knob",
        ArticulationType.PRISMATIC,
        parent=backrest,
        child=selector_knob,
        origin=Origin(xyz=(0.29, 0.155, -0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.10,
            lower=0.0,
            upper=0.028,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    ladder = object_model.get_part("ladder")
    selector_knob = object_model.get_part("selector_knob")

    backrest_hinge = object_model.get_articulation("base_to_backrest")
    seat_hinge = object_model.get_articulation("base_to_seat")
    ladder_hinge = object_model.get_articulation("base_to_ladder")
    knob_slide = object_model.get_articulation("backrest_to_selector_knob")

    backrest_upper = backrest_hinge.motion_limits.upper if backrest_hinge.motion_limits else None
    seat_upper = seat_hinge.motion_limits.upper if seat_hinge.motion_limits else None
    ladder_upper = ladder_hinge.motion_limits.upper if ladder_hinge.motion_limits else None
    knob_upper = knob_slide.motion_limits.upper if knob_slide.motion_limits else None

    with ctx.pose(
        {
            backrest_hinge: 0.0,
            seat_hinge: 0.0,
            ladder_hinge: 0.0,
            knob_slide: 0.0,
        }
    ):
        ctx.expect_gap(
            backrest,
            seat,
            axis="x",
            min_gap=0.006,
            max_gap=0.020,
            positive_elem="upholstery",
            negative_elem="upholstery",
            name="flat pads keep a narrow hinge gap",
        )
        ctx.expect_overlap(
            backrest,
            seat,
            axes="y",
            min_overlap=0.27,
            elem_a="upholstery",
            elem_b="upholstery",
            name="seat and backrest stay aligned across the bench width",
        )
        ctx.expect_gap(
            ladder,
            backrest,
            axis="y",
            min_gap=0.015,
            positive_elem="ladder_rail_inner",
            negative_elem="selector_plate_upper",
            name="support ladder sits behind the selector plate",
        )

        knob_rest = ctx.part_world_position(selector_knob)
        backrest_rest_aabb = ctx.part_world_aabb(backrest)
        seat_rest_aabb = ctx.part_world_aabb(seat)
        ladder_rest_aabb = ctx.part_world_aabb(ladder)

    if backrest_upper is not None:
        with ctx.pose({backrest_hinge: backrest_upper}):
            backrest_raised_aabb = ctx.part_world_aabb(backrest)
        ctx.check(
            "backrest rotates upward from the seat junction",
            backrest_rest_aabb is not None
            and backrest_raised_aabb is not None
            and backrest_raised_aabb[1][2] > backrest_rest_aabb[1][2] + 0.25,
            details=f"rest={backrest_rest_aabb}, raised={backrest_raised_aabb}",
        )

    if seat_upper is not None:
        with ctx.pose({seat_hinge: seat_upper}):
            seat_raised_aabb = ctx.part_world_aabb(seat)
        ctx.check(
            "seat pivots upward on its separate front hinge",
            seat_rest_aabb is not None
            and seat_raised_aabb is not None
            and seat_raised_aabb[1][2] > seat_rest_aabb[1][2] + 0.03,
            details=f"rest={seat_rest_aabb}, raised={seat_raised_aabb}",
        )

    if ladder_upper is not None:
        with ctx.pose({ladder_hinge: ladder_upper}):
            ladder_raised_aabb = ctx.part_world_aabb(ladder)
        ctx.check(
            "rear ladder support rotates upward from the lower frame pivot",
            ladder_rest_aabb is not None
            and ladder_raised_aabb is not None
            and ladder_raised_aabb[1][2] > ladder_rest_aabb[1][2] + 0.06,
            details=f"rest={ladder_rest_aabb}, raised={ladder_raised_aabb}",
        )

    if knob_upper is not None:
        with ctx.pose({knob_slide: knob_upper}):
            knob_extended = ctx.part_world_position(selector_knob)
        ctx.check(
            "selector knob pulls outward on a short prismatic stroke",
            knob_rest is not None
            and knob_extended is not None
            and knob_extended[1] > knob_rest[1] + 0.020,
            details=f"rest={knob_rest}, extended={knob_extended}",
        )

    return ctx.report()


object_model = build_object_model()

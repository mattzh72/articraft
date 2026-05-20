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
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _cylinder_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_cylinder_rpy(start, end)),
        material=material,
        name=name,
    )


def _circle_points(radius: float, z: float, count: int) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / count),
            radius * math.sin((2.0 * math.pi * i) / count),
            z,
        )
        for i in range(count)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_stool_with_hook")

    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    walnut = model.material("walnut", rgba=(0.60, 0.39, 0.21, 1.0))
    black_powder = model.material("black_powder", rgba=(0.10, 0.10, 0.11, 1.0))

    seat_shell = mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.190, 0.030, 0.061, 0.300),
                (-0.105, 0.023, 0.056, 0.388),
                (0.000, 0.018, 0.048, 0.430),
                (0.105, 0.017, 0.043, 0.398),
                (0.190, 0.020, 0.039, 0.324),
            ],
            exponents=2.2,
            segments=72,
        ),
        "seat_shell",
    )
    footrest_ring = mesh_from_geometry(
        tube_from_spline_points(
            _circle_points(0.165, 0.305, 24),
            radius=0.012,
            closed_spline=True,
            samples_per_segment=10,
            radial_segments=22,
            cap_ends=False,
            up_hint=(0.0, 0.0, 1.0),
        ),
        "footrest_ring",
    )
    hook_loop = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, -0.012, -0.008),
                (0.000, -0.045, -0.012),
                (0.000, -0.108, -0.022),
                (0.000, -0.128, -0.052),
                (0.000, -0.102, -0.078),
                (0.000, -0.055, -0.070),
            ],
            radius=0.0055,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "hook_loop",
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.220, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_steel,
        name="floor_base",
    )
    pedestal_base.visual(
        Cylinder(radius=0.168, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=brushed_steel,
        name="lower_shroud",
    )
    pedestal_base.visual(
        Cylinder(radius=0.044, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=brushed_steel,
        name="column_collar",
    )
    pedestal_base.visual(
        Cylinder(radius=0.031, length=0.642),
        origin=Origin(xyz=(0.0, 0.0, 0.389)),
        material=brushed_steel,
        name="pedestal_tube",
    )
    pedestal_base.visual(
        Cylinder(radius=0.054, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=brushed_steel,
        name="footrest_collar",
    )
    pedestal_base.visual(
        footrest_ring,
        material=brushed_steel,
        name="footrest_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        start = (0.052 * math.cos(angle), 0.052 * math.sin(angle), 0.316)
        end = (0.165 * math.cos(angle), 0.165 * math.sin(angle), 0.305)
        _add_member(
            pedestal_base,
            start,
            end,
            radius=0.008,
            material=brushed_steel,
            name=f"footrest_strut_{index}",
        )
    pedestal_base.visual(
        Cylinder(radius=0.058, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.7225)),
        material=dark_steel,
        name="bearing_housing",
    )

    seat = model.part("seat")
    seat.visual(
        seat_shell,
        material=walnut,
        name="seat_shell",
    )
    seat.visual(
        Box((0.172, 0.172, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black_powder,
        name="swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_steel,
        name="swivel_hub",
    )
    for index, (start, end) in enumerate(
        (
            ((0.038, 0.038, 0.008), (0.080, 0.095, 0.021)),
            ((-0.038, 0.038, 0.008), (-0.080, 0.095, 0.021)),
            ((0.038, -0.038, 0.008), (0.070, -0.090, 0.026)),
            ((-0.038, -0.038, 0.008), (-0.070, -0.090, 0.026)),
        )
    ):
        _add_member(
            seat,
            start,
            end,
            radius=0.006,
            material=black_powder,
            name=f"seat_brace_{index}",
        )
    for name, x_center in (("hinge_bracket_0", -0.026), ("hinge_bracket_1", 0.026)):
        seat.visual(
            Box((0.022, 0.026, 0.024)),
            origin=Origin(xyz=(x_center, -0.158, 0.018)),
            material=dark_steel,
            name=name,
        )
    for name, x_center in (("hinge_barrel_0", -0.016), ("hinge_barrel_1", 0.016)):
        seat.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(x_center, -0.169, 0.010),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=name,
        )
    seat.visual(
        Cylinder(radius=0.003, length=0.050),
        origin=Origin(
            xyz=(0.0, -0.169, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="hinge_pin",
    )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hook_barrel",
    )
    hook.visual(
        Box((0.016, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, -0.006)),
        material=dark_steel,
        name="hook_tab",
    )
    hook.visual(
        hook_loop,
        material=dark_steel,
        name="hook_loop",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.8),
    )
    model.articulation(
        "hook_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=hook,
        origin=Origin(xyz=(0.0, -0.169, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal_base = object_model.get_part("pedestal_base")
    seat = object_model.get_part("seat")
    hook = object_model.get_part("hook")
    seat_swivel = object_model.get_articulation("seat_swivel")
    hook_hinge = object_model.get_articulation("hook_hinge")

    ctx.allow_overlap(
        seat,
        hook,
        elem_a="hinge_pin",
        elem_b="hook_barrel",
        reason="The luggage hook rotates on a steel hinge pin captured through the hook barrel.",
    )

    ctx.expect_gap(
        seat,
        pedestal_base,
        axis="z",
        positive_elem="swivel_plate",
        negative_elem="bearing_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat swivel plate seats on the bearing housing",
    )
    ctx.expect_overlap(
        seat,
        pedestal_base,
        axes="xy",
        elem_a="swivel_plate",
        elem_b="bearing_housing",
        min_overlap=0.100,
        name="seat swivel stays centered over the pedestal axis",
    )
    ctx.expect_gap(
        seat,
        hook,
        axis="z",
        positive_elem="seat_shell",
        negative_elem="hook_loop",
        min_gap=0.008,
        name="stowed hook sits below the wood seat shell",
    )

    with ctx.pose({seat_swivel: 1.30}):
        ctx.expect_gap(
            seat,
            pedestal_base,
            axis="z",
            positive_elem="swivel_plate",
            negative_elem="bearing_housing",
            max_gap=0.001,
            max_penetration=0.0,
            name="seat stays seated on the bearing while rotated",
        )
        ctx.expect_overlap(
            seat,
            pedestal_base,
            axes="xy",
            elem_a="swivel_plate",
            elem_b="bearing_housing",
            min_overlap=0.100,
            name="seat remains centered while swiveled",
        )

    rest_aabb = ctx.part_element_world_aabb(hook, elem="hook_loop")
    with ctx.pose({hook_hinge: 1.20}):
        deployed_aabb = ctx.part_element_world_aabb(hook, elem="hook_loop")

    rest_low = None if rest_aabb is None else rest_aabb[0][2]
    deployed_low = None if deployed_aabb is None else deployed_aabb[0][2]
    ctx.check(
        "hook deploys downward",
        rest_low is not None
        and deployed_low is not None
        and deployed_low < rest_low - 0.045,
        details=f"rest_low={rest_low}, deployed_low={deployed_low}",
    )

    return ctx.report()


object_model = build_object_model()

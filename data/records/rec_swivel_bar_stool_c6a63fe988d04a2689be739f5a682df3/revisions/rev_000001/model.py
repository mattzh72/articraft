from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_swivel_stool")

    black_metal = Material("satin_black_metal", rgba=(0.02, 0.022, 0.024, 1.0))
    chrome = Material("brushed_chrome", rgba=(0.74, 0.76, 0.76, 1.0))
    dark_vinyl = Material("oxblood_vinyl", rgba=(0.36, 0.055, 0.045, 1.0))
    seam = Material("dark_seam", rgba=(0.08, 0.025, 0.020, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.225, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_metal,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.435),
        origin=Origin(xyz=(0.0, 0.0, 0.2575)),
        material=chrome,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.100, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.4875)),
        material=black_metal,
        name="top_bearing",
    )

    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.170, tube=0.010, radial_segments=20, tubular_segments=80),
        "foot_ring",
    )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=chrome,
        name="foot_ring",
    )
    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal.visual(
            Box((0.150, 0.014, 0.014)),
            origin=Origin(xyz=(0.103, 0.0, 0.205), rpy=(0.0, 0.0, yaw)),
            material=chrome,
            name=f"foot_spoke_{index}",
        )

    seat = model.part("seat")
    seat_shell = LatheGeometry(
        [
            (0.000, 0.020),
            (0.155, 0.020),
            (0.190, 0.028),
            (0.205, 0.048),
            (0.196, 0.073),
            (0.155, 0.086),
            (0.000, 0.086),
        ],
        segments=96,
    )
    seat.visual(
        mesh_from_geometry(seat_shell, "rounded_seat_shell"),
        material=dark_vinyl,
        name="seat_shell",
    )
    seat.visual(
        Cylinder(radius=0.175, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=black_metal,
        name="underside_pan",
    )
    seat.visual(
        Cylinder(radius=0.095, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="swivel_plate",
    )
    seat.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.145, tube=0.0025, radial_segments=10, tubular_segments=72),
            "seat_top_seam",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=seam,
        name="top_seam",
    )

    ring_points = []
    for i in range(18):
        theta = 2.0 * math.pi * i / 18.0
        ring_points.append((0.178 * math.cos(theta), 0.210, 0.250 + 0.178 * math.sin(theta)))
    seat.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                ring_points,
                radius=0.013,
                samples_per_segment=8,
                closed_spline=True,
                radial_segments=18,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "backrest_ring_tube",
        ),
        material=black_metal,
        name="backrest_ring",
    )
    for side, x in (("0", -0.125), ("1", 0.125)):
        support = tube_from_spline_points(
            [
                (x, 0.118, 0.050),
                (x * 1.06, 0.155, 0.090),
                (x * 1.12, 0.188, 0.142),
                (x * 1.18, 0.205, 0.160),
            ],
            radius=0.011,
            samples_per_segment=8,
            radial_segments=16,
            up_hint=(0.0, 1.0, 0.0),
        )
        seat.visual(
            mesh_from_geometry(support, f"back_support_{side}"),
            material=black_metal,
            name=f"back_support_{side}",
        )

    seat.visual(
        Box((0.058, 0.080, 0.010)),
        origin=Origin(xyz=(0.175, 0.0, 0.012)),
        material=black_metal,
        name="hinge_plate",
    )
    seat.visual(
        Box((0.034, 0.006, 0.034)),
        origin=Origin(xyz=(0.185, -0.033, 0.000)),
        material=black_metal,
        name="hinge_cheek_0",
    )
    seat.visual(
        Box((0.034, 0.006, 0.034)),
        origin=Origin(xyz=(0.185, 0.033, 0.000)),
        material=black_metal,
        name="hinge_cheek_1",
    )

    hook = model.part("side_hook")
    hook.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    hook.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.003, -0.010, -0.002),
                    (0.042, -0.020, -0.011),
                    (0.085, -0.036, -0.014),
                    (0.126, -0.010, -0.014),
                    (0.118, 0.046, -0.014),
                    (0.075, 0.062, -0.014),
                ],
                radius=0.006,
                samples_per_segment=10,
                radial_segments=14,
                up_hint=(0.0, 0.0, 1.0),
            ),
            "folding_hook_rod",
        ),
        material=chrome,
        name="hook_rod",
    )

    model.articulation(
        "pedestal_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    model.articulation(
        "seat_to_side_hook",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=hook,
        origin=Origin(xyz=(0.185, 0.0, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.45),
        motion_properties=MotionProperties(damping=0.03, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    hook = object_model.get_part("side_hook")
    swivel = object_model.get_articulation("pedestal_to_seat")
    hook_hinge = object_model.get_articulation("seat_to_side_hook")

    ctx.check(
        "seat swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {swivel.articulation_type}",
    )
    ctx.check(
        "hook has downward hinge travel",
        hook_hinge.motion_limits is not None
        and hook_hinge.motion_limits.lower == 0.0
        and hook_hinge.motion_limits.upper is not None
        and 1.2 <= hook_hinge.motion_limits.upper <= 1.7,
        details=f"limits={hook_hinge.motion_limits}",
    )

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_plate",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel bearing plates meet vertically",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="swivel_plate",
        elem_b="top_bearing",
        min_overlap=0.08,
        name="swivel plates are concentric",
    )
    ctx.expect_contact(
        hook,
        seat,
        elem_a="hinge_barrel",
        elem_b="hinge_cheek_1",
        contact_tol=0.002,
        name="hook barrel is captured by hinge cheek",
    )

    rest_hook_aabb = ctx.part_world_aabb(hook)
    with ctx.pose({hook_hinge: hook_hinge.motion_limits.upper}):
        down_hook_aabb = ctx.part_world_aabb(hook)
    ctx.check(
        "hook swings downward",
        rest_hook_aabb is not None
        and down_hook_aabb is not None
        and down_hook_aabb[0][2] < rest_hook_aabb[0][2] - 0.050,
        details=f"rest={rest_hook_aabb}, down={down_hook_aabb}",
    )

    rest_ring = ctx.part_element_world_aabb(seat, elem="backrest_ring")
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_ring = ctx.part_element_world_aabb(seat, elem="backrest_ring")
    if rest_ring is not None and turned_ring is not None:
        rest_center = (
            (rest_ring[0][0] + rest_ring[1][0]) * 0.5,
            (rest_ring[0][1] + rest_ring[1][1]) * 0.5,
        )
        turned_center = (
            (turned_ring[0][0] + turned_ring[1][0]) * 0.5,
            (turned_ring[0][1] + turned_ring[1][1]) * 0.5,
        )
        moved = abs(turned_center[0] - rest_center[0]) > 0.12 and abs(turned_center[1] - rest_center[1]) > 0.12
    else:
        moved = False
    ctx.check(
        "backrest ring follows seat swivel",
        moved,
        details=f"rest_ring={rest_ring}, turned_ring={turned_ring}",
    )

    return ctx.report()


object_model = build_object_model()

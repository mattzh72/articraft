from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe_mesh(name: str, profile: list[tuple[float, float]], *, segments: int = 72):
    return mesh_from_geometry(LatheGeometry(profile, segments=segments), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    chrome = model.material("chrome", rgba=(0.76, 0.78, 0.80, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.17, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    trumpet_mesh = _lathe_mesh(
        "trumpet_base",
        [
            (0.0, 0.0),
            (0.090, 0.0),
            (0.155, 0.002),
            (0.196, 0.007),
            (0.214, 0.014),
            (0.210, 0.020),
            (0.148, 0.032),
            (0.066, 0.043),
            (0.040, 0.051),
            (0.0, 0.051),
        ],
        segments=84,
    )
    seat_shell_mesh = _lathe_mesh(
        "seat_shell",
        [
            (0.0, 0.004),
            (0.085, 0.004),
            (0.150, 0.006),
            (0.176, 0.012),
            (0.188, 0.022),
            (0.190, 0.038),
            (0.184, 0.053),
            (0.166, 0.061),
            (0.0, 0.061),
        ],
        segments=80,
    )

    base = model.part("base")
    base.visual(trumpet_mesh, material=chrome, name="trumpet_shell")
    base.visual(
        Cylinder(radius=0.165, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="floor_ring",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.625),
        origin=Origin(xyz=(0.0, 0.0, 0.3575)),
        material=satin_steel,
        name="post",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=chrome,
        name="bearing_collar",
    )

    seat = model.part("seat")
    seat.visual(seat_shell_mesh, material=black_vinyl, name="seat_shell")
    seat.visual(
        Cylinder(radius=0.044, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_frame,
        name="hub_sleeve",
    )
    seat.visual(
        Box((0.070, 0.074, 0.032)),
        origin=Origin(xyz=(-0.176, 0.0, 0.021)),
        material=dark_frame,
        name="rear_mount",
    )
    seat.visual(
        Box((0.020, 0.034, 0.014)),
        origin=Origin(xyz=(-0.219, 0.0, 0.043)),
        material=dark_frame,
        name="hinge_web",
    )
    seat.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(-0.228, 0.0, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_lug",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="barrel_0",
    )
    backrest.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="barrel_1",
    )
    backrest.visual(
        Box((0.018, 0.018, 0.180)),
        origin=Origin(xyz=(-0.010, -0.044, 0.090)),
        material=dark_frame,
        name="arm_0",
    )
    backrest.visual(
        Box((0.018, 0.018, 0.180)),
        origin=Origin(xyz=(-0.010, 0.044, 0.090)),
        material=dark_frame,
        name="arm_1",
    )
    backrest.visual(
        Box((0.020, 0.118, 0.026)),
        origin=Origin(xyz=(-0.018, 0.0, 0.178)),
        material=dark_frame,
        name="crossmember",
    )
    backrest.visual(
        Box((0.048, 0.260, 0.110)),
        origin=Origin(xyz=(-0.052, 0.0, 0.220)),
        material=black_vinyl,
        name="back_pad",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=6.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.228, 0.0, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    swivel = object_model.get_articulation("base_to_seat")
    backrest_hinge = object_model.get_articulation("seat_to_backrest")

    ctx.expect_origin_distance(
        seat,
        base,
        axes="xy",
        max_dist=0.001,
        name="seat axis stays centered on the pedestal",
    )
    ctx.expect_origin_gap(
        seat,
        base,
        axis="z",
        min_gap=0.72,
        max_gap=0.76,
        name="seat height matches indoor bar stool scale",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="hub_sleeve",
        negative_elem="bearing_collar",
        min_gap=0.0,
        max_gap=0.001,
        name="seat hub seats directly on the bearing collar",
    )
    ctx.expect_origin_gap(
        seat,
        backrest,
        axis="x",
        min_gap=0.21,
        max_gap=0.25,
        name="backrest hinge sits behind the seat axis",
    )

    rest_backrest_pos = ctx.part_world_position(backrest)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_backrest_pos = ctx.part_world_position(backrest)
    ctx.check(
        "seat swivel carries the rear bracket around the pedestal",
        rest_backrest_pos is not None
        and turned_backrest_pos is not None
        and abs(turned_backrest_pos[0]) < 0.03
        and abs(turned_backrest_pos[1]) > 0.18
        and abs(
            math.hypot(rest_backrest_pos[0], rest_backrest_pos[1])
            - math.hypot(turned_backrest_pos[0], turned_backrest_pos[1])
        )
        < 0.01,
        details=f"rest={rest_backrest_pos}, turned={turned_backrest_pos}",
    )

    rest_backrest_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({backrest_hinge: 1.05}):
        folded_backrest_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest folds rearward from its upright position",
        rest_backrest_aabb is not None
        and folded_backrest_aabb is not None
        and folded_backrest_aabb[0][0] < rest_backrest_aabb[0][0] - 0.04
        and folded_backrest_aabb[1][2] < rest_backrest_aabb[1][2] - 0.10,
        details=f"rest={rest_backrest_aabb}, folded={folded_backrest_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

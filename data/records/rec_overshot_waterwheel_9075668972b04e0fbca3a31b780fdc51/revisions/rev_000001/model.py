from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="demo_overshot_mill")

    frame_wood = model.material("frame_wood", rgba=(0.36, 0.24, 0.15, 1.0))
    wheel_wood = model.material("wheel_wood", rgba=(0.48, 0.34, 0.21, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.26, 0.28, 0.30, 1.0))
    steel_light = model.material("steel_light", rgba=(0.66, 0.68, 0.70, 1.0))
    chute_metal = model.material("chute_metal", rgba=(0.62, 0.66, 0.68, 1.0))
    guard_frame = model.material("guard_frame", rgba=(0.48, 0.53, 0.57, 1.0))
    guard_glass = model.material("guard_glass", rgba=(0.76, 0.84, 0.88, 0.36))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((0.28, 0.52, 0.52)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.02, 0.26)),
    )

    for side in (-1.0, 1.0):
        x_runner = side * 0.10
        x_cheek = side * 0.084
        x_post = side * 0.095
        support_frame.visual(
            Box((0.06, 0.50, 0.05)),
            origin=Origin(xyz=(x_runner, 0.02, 0.025)),
            material=frame_wood,
            name=f"runner_{'left' if side < 0 else 'right'}",
        )
        support_frame.visual(
            Box((0.018, 0.16, 0.34)),
            origin=Origin(xyz=(x_cheek, 0.02, 0.195)),
            material=frame_wood,
            name=f"cheek_{'left' if side < 0 else 'right'}",
        )
        support_frame.visual(
            Box((0.038, 0.05, 0.045)),
            origin=Origin(xyz=(x_cheek, 0.02, 0.3875)),
            material=frame_wood,
            name=f"bearing_cap_{'left' if side < 0 else 'right'}",
        )
        support_frame.visual(
            Box((0.028, 0.05, 0.06)),
            origin=Origin(xyz=(side * 0.075, 0.02, 0.28)),
            material=steel_dark,
            name=f"bearing_pad_{'left' if side < 0 else 'right'}",
        )
        support_frame.visual(
            Box((0.03, 0.035, 0.38)),
            origin=Origin(xyz=(x_post, 0.242, 0.19)),
            material=frame_wood,
            name=f"front_post_{'left' if side < 0 else 'right'}",
        )

    support_frame.visual(
        Box((0.22, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.16, 0.045)),
        material=frame_wood,
        name="rear_sill",
    )
    support_frame.visual(
        Box((0.22, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.12, 0.045)),
        material=frame_wood,
        name="front_sill",
    )
    support_frame.visual(
        Box((0.21, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.242, 0.39)),
        material=frame_wood,
        name="front_top_beam",
    )

    support_frame.visual(
        Box((0.13, 0.18, 0.012)),
        origin=Origin(xyz=(0.0, 0.145, 0.500)),
        material=chute_metal,
        name="feed_floor",
    )
    support_frame.visual(
        Box((0.010, 0.16, 0.05)),
        origin=Origin(xyz=(-0.058, 0.145, 0.519)),
        material=chute_metal,
        name="feed_wall_left",
    )
    support_frame.visual(
        Box((0.010, 0.16, 0.05)),
        origin=Origin(xyz=(0.058, 0.145, 0.519)),
        material=chute_metal,
        name="feed_wall_right",
    )
    support_frame.visual(
        Box((0.13, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.065, 0.509)),
        material=chute_metal,
        name="feed_back_lip",
    )
    support_frame.visual(
        Box((0.05, 0.052, 0.10)),
        origin=Origin(xyz=(0.0, 0.219, 0.447)),
        material=chute_metal,
        name="feed_spout",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.10),
        mass=2.4,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.185, tube=0.012, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0),
        "overshot_wheel_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=steel_dark,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=steel_dark,
        name="right_rim",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="axle_core",
    )
    wheel.visual(
        Cylinder(radius=0.028, length=0.09),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub_flange_left",
    )
    wheel.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub_flange_right",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="axle_collar_left",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="axle_collar_right",
    )

    for side in (-1.0, 1.0):
        x_spoke = side * 0.034
        for index in range(6):
            angle = (2.0 * pi * index) / 6.0 + (pi / 6.0 if side > 0 else 0.0)
            inner = (x_spoke, sin(angle) * 0.026, cos(angle) * 0.026)
            outer = (x_spoke, sin(angle) * 0.185, cos(angle) * 0.185)
            _add_member(
                wheel,
                inner,
                outer,
                radius=0.006,
                material=wheel_wood,
                name=f"{'right' if side > 0 else 'left'}_spoke_{index}",
            )

    for index in range(12):
        angle = (2.0 * pi * index) / 12.0
        wheel.visual(
            Box((0.094, 0.056, 0.014)),
            origin=Origin(
                xyz=(0.0, sin(angle) * 0.167, cos(angle) * 0.167),
                rpy=(angle - 0.18, 0.0, 0.0),
            ),
            material=wheel_wood,
            name=f"paddle_{index:02d}",
        )

    guard = model.part("guard_panel")
    guard.inertial = Inertial.from_geometry(
        Box((0.18, 0.012, 0.16)),
        mass=0.6,
        origin=Origin(xyz=(-0.09, 0.0, 0.08)),
    )
    guard.visual(
        Box((0.018, 0.010, 0.15)),
        origin=Origin(xyz=(-0.009, 0.0, 0.075)),
        material=guard_frame,
        name="guard_hinge_stile",
    )
    guard.visual(
        Box((0.018, 0.010, 0.15)),
        origin=Origin(xyz=(-0.169, 0.0, 0.075)),
        material=guard_frame,
        name="guard_latch_stile",
    )
    guard.visual(
        Box((0.16, 0.010, 0.016)),
        origin=Origin(xyz=(-0.089, 0.0, 0.142)),
        material=guard_frame,
        name="guard_top_rail",
    )
    guard.visual(
        Box((0.16, 0.010, 0.016)),
        origin=Origin(xyz=(-0.089, 0.0, 0.008)),
        material=guard_frame,
        name="guard_bottom_rail",
    )
    guard.visual(
        Box((0.144, 0.004, 0.120)),
        origin=Origin(xyz=(-0.089, 0.0, 0.075)),
        material=guard_glass,
        name="guard_window",
    )
    guard.visual(
        Cylinder(radius=0.005, length=0.145),
        origin=Origin(xyz=(-0.002, 0.0, 0.075)),
        material=guard_frame,
        name="guard_hinge_barrel",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=14.0),
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=guard,
        origin=Origin(xyz=(0.132, 0.267, 0.285)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    guard = object_model.get_part("guard_panel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    guard_hinge = object_model.get_articulation("guard_hinge")

    ctx.check(
        "wheel spin articulation is continuous on the horizontal axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_spin.axis) == (-1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "guard hinge rotates about the frame-side vertical hinge line",
        guard_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in guard_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"type={guard_hinge.articulation_type}, axis={guard_hinge.axis}",
    )

    with ctx.pose({guard_hinge: 0.0}):
        ctx.expect_gap(
            support_frame,
            wheel,
            axis="z",
            positive_elem="feed_floor",
            min_gap=0.01,
            max_gap=0.08,
            name="feed chute sits just above the wheel crown",
        )
        ctx.expect_gap(
            guard,
            wheel,
            axis="y",
            positive_elem="guard_window",
            min_gap=0.025,
            max_gap=0.09,
            name="closed guard stays in front of the wheel",
        )

        closed_aabb = ctx.part_element_world_aabb(guard, elem="guard_window")

    with ctx.pose({guard_hinge: 1.1}):
        open_aabb = ctx.part_element_world_aabb(guard, elem="guard_window")

    ctx.check(
        "guard swings outward at the feed side",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

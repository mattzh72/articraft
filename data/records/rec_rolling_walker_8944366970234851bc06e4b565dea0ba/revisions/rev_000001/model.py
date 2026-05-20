from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _build_caster_fork(part, *, side_sign: float, fork_material, axle_material) -> None:
    wheel_offset = 0.045 * side_sign
    part.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=axle_material,
        name="bearing_cap",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=fork_material,
        name="stem",
    )
    part.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(wheel_offset * 0.32, 0.0, -0.030)),
        material=fork_material,
        name="offset_post",
    )
    part.visual(
        Box((0.044, 0.018, 0.010)),
        origin=Origin(xyz=(wheel_offset, 0.0, -0.049)),
        material=fork_material,
        name="yoke",
    )
    part.visual(
        Box((0.008, 0.018, 0.056)),
        origin=Origin(xyz=(wheel_offset - 0.018, 0.0, -0.082)),
        material=fork_material,
        name="inner_fork_leg",
    )
    part.visual(
        Box((0.008, 0.018, 0.056)),
        origin=Origin(xyz=(wheel_offset + 0.018, 0.0, -0.082)),
        material=fork_material,
        name="outer_fork_leg",
    )


def _build_caster_wheel(part, *, tire_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="axle_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    frame_silver = model.material("frame_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tip_gray = model.material("tip_gray", rgba=(0.25, 0.26, 0.28, 1.0))
    caster_gray = model.material("caster_gray", rgba=(0.43, 0.45, 0.48, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.70, 0.72, 0.74, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.56, 0.96)),
        mass=7.2,
        origin=Origin(xyz=(0.0, -0.01, 0.48)),
    )

    left_side_points = [
        (0.255, 0.215, 0.169),
        (0.255, 0.200, 0.440),
        (0.252, 0.135, 0.720),
        (0.248, 0.020, 0.885),
        (0.246, -0.080, 0.895),
        (0.240, -0.165, 0.850),
        (0.224, -0.225, 0.560),
        (0.208, -0.245, 0.240),
        (0.205, -0.245, 0.030),
    ]
    side_radius = 0.014
    frame.visual(
        _mesh(
            "left_side_frame",
            tube_from_spline_points(
                left_side_points,
                radius=side_radius,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_silver,
        name="left_side_frame",
    )
    frame.visual(
        _mesh(
            "right_side_frame",
            tube_from_spline_points(
                _mirror_x(left_side_points),
                radius=side_radius,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_silver,
        name="right_side_frame",
    )

    left_support_points = [
        (0.224, -0.225, 0.560),
        (0.232, -0.205, 0.690),
        (0.240, -0.165, 0.850),
    ]
    frame.visual(
        _mesh(
            "left_rear_support",
            tube_from_spline_points(
                left_support_points,
                radius=0.011,
                samples_per_segment=8,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_silver,
        name="left_rear_support",
    )
    frame.visual(
        _mesh(
            "right_rear_support",
            tube_from_spline_points(
                _mirror_x(left_support_points),
                radius=0.011,
                samples_per_segment=8,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_silver,
        name="right_rear_support",
    )

    frame.visual(
        Cylinder(radius=0.011, length=0.510),
        origin=Origin(xyz=(0.0, 0.200, 0.440), rpy=(0.0, 1.57079632679, 0.0)),
        material=frame_silver,
        name="front_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.448),
        origin=Origin(xyz=(0.0, -0.225, 0.560), rpy=(0.0, 1.57079632679, 0.0)),
        material=frame_silver,
        name="rear_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.135),
        origin=Origin(xyz=(0.246, -0.085, 0.900), rpy=(1.57079632679, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.135),
        origin=Origin(xyz=(-0.246, -0.085, 0.900), rpy=(1.57079632679, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    frame.visual(
        Box((0.036, 0.024, 0.016)),
        origin=Origin(xyz=(0.255, 0.215, 0.163)),
        material=frame_silver,
        name="left_caster_mount",
    )
    frame.visual(
        Box((0.036, 0.024, 0.016)),
        origin=Origin(xyz=(-0.255, 0.215, 0.163)),
        material=frame_silver,
        name="right_caster_mount",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.045),
        origin=Origin(xyz=(0.205, -0.245, 0.0225)),
        material=tip_gray,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.045),
        origin=Origin(xyz=(-0.205, -0.245, 0.0225)),
        material=tip_gray,
        name="right_rear_tip",
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.090, 0.030, 0.130)),
        mass=0.28,
        origin=Origin(xyz=(0.022, 0.0, -0.068)),
    )
    _build_caster_fork(
        left_caster_fork,
        side_sign=1.0,
        fork_material=caster_gray,
        axle_material=hub_gray,
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.090, 0.030, 0.130)),
        mass=0.28,
        origin=Origin(xyz=(-0.022, 0.0, -0.068)),
    )
    _build_caster_fork(
        right_caster_fork,
        side_sign=-1.0,
        fork_material=caster_gray,
        axle_material=hub_gray,
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.028),
        mass=0.34,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(left_caster_wheel, tire_material=wheel_black, hub_material=hub_gray)

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.028),
        mass=0.34,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(right_caster_wheel, tire_material=wheel_black, hub_material=hub_gray)

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.255, 0.215, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.255, 0.215, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.045, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=30.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.045, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=30.0),
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

    frame = object_model.get_part("frame")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")

    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="left_grip",
        negative_elem="left_rear_tip",
        min_gap=0.82,
        name="handle height is walker-like",
    )
    ctx.expect_overlap(
        frame,
        frame,
        axes="x",
        elem_a="front_crossbar",
        elem_b="rear_crossbar",
        min_overlap=0.40,
        name="crossbars span between side frames",
    )
    ctx.expect_origin_gap(
        left_caster_wheel,
        left_caster_fork,
        axis="x",
        min_gap=0.035,
        max_gap=0.055,
        name="left front wheel sits outboard of swivel stem",
    )
    ctx.expect_origin_gap(
        right_caster_fork,
        right_caster_wheel,
        axis="x",
        min_gap=0.035,
        max_gap=0.055,
        name="right front wheel sits outboard of swivel stem",
    )
    ctx.expect_origin_gap(
        left_caster_fork,
        left_caster_wheel,
        axis="z",
        min_gap=0.095,
        max_gap=0.115,
        name="left caster wheel hangs below the fork stem",
    )

    left_rest = ctx.part_world_position(left_caster_wheel)
    with ctx.pose({left_swivel: pi / 2.0, right_swivel: pi / 2.0}):
        left_turned = ctx.part_world_position(left_caster_wheel)
        right_turned = ctx.part_world_position(right_caster_wheel)

    ctx.check(
        "left caster swivel swings wheel fore-aft around the stem",
        left_rest is not None
        and left_turned is not None
        and abs(left_turned[0] - left_rest[0]) > 0.035
        and abs(left_turned[1] - left_rest[1]) > 0.035,
        details=f"rest={left_rest}, turned={left_turned}",
    )
    ctx.check(
        "paired casters can splay in opposite directions",
        left_turned is not None
        and right_turned is not None
        and left_turned[1] > 0.24
        and right_turned[1] < 0.19,
        details=f"left_turned={left_turned}, right_turned={right_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

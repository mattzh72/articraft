from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 12,
    radial_segments: int = 18,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _wheel_tire_mesh(name: str, *, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.48, -half_width * 0.98),
        (radius * 0.72, -half_width),
        (radius * 0.90, -half_width * 0.84),
        (radius * 0.98, -half_width * 0.54),
        (radius, -half_width * 0.18),
        (radius, half_width * 0.18),
        (radius * 0.98, half_width * 0.54),
        (radius * 0.90, half_width * 0.84),
        (radius * 0.72, half_width),
        (radius * 0.48, half_width * 0.98),
        (radius * 0.41, half_width * 0.36),
        (radius * 0.38, 0.0),
        (radius * 0.41, -half_width * 0.36),
        (radius * 0.48, -half_width * 0.98),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64).rotate_x(-math.pi / 2.0), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_paint = model.material("tray_paint", rgba=(0.76, 0.14, 0.10, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip = model.material("grip", rgba=(0.24, 0.17, 0.10, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((1.28, 0.64, 0.62)),
        mass=18.0,
        origin=Origin(xyz=(-0.12, 0.0, 0.33)),
    )

    tray_pitch = -0.16
    body.visual(
        Box((0.58, 0.46, 0.014)),
        origin=Origin(xyz=(0.00, 0.0, 0.42), rpy=(0.0, tray_pitch, 0.0)),
        material=tray_paint,
        name="tray_bottom",
    )
    body.visual(
        Box((0.54, 0.018, 0.20)),
        origin=Origin(xyz=(0.00, 0.236, 0.49), rpy=(-0.15, tray_pitch, 0.0)),
        material=tray_paint,
        name="tray_left_wall",
    )
    body.visual(
        Box((0.54, 0.018, 0.20)),
        origin=Origin(xyz=(0.00, -0.236, 0.49), rpy=(0.15, tray_pitch, 0.0)),
        material=tray_paint,
        name="tray_right_wall",
    )
    body.visual(
        Box((0.022, 0.42, 0.18)),
        origin=Origin(xyz=(0.29, 0.0, 0.47), rpy=(0.0, -0.10, 0.0)),
        material=tray_paint,
        name="tray_front_wall",
    )
    body.visual(
        Box((0.032, 0.34, 0.112)),
        origin=Origin(xyz=(-0.240, 0.0, 0.434), rpy=(0.0, tray_pitch, 0.0)),
        material=tray_paint,
        name="tray_rear_wall",
    )

    body.visual(
        Box((0.62, 0.03, 0.04)),
        origin=Origin(xyz=(-0.01, 0.225, 0.41), rpy=(0.0, tray_pitch, 0.0)),
        material=frame_steel,
        name="left_side_rail_support",
    )
    body.visual(
        Box((0.62, 0.03, 0.04)),
        origin=Origin(xyz=(-0.01, -0.225, 0.41), rpy=(0.0, tray_pitch, 0.0)),
        material=frame_steel,
        name="right_side_rail_support",
    )
    body.visual(
        Box((0.06, 0.48, 0.03)),
        origin=Origin(xyz=(-0.35, 0.0, 0.34), rpy=(0.0, 0.06, 0.0)),
        material=frame_steel,
        name="rear_leg_cross_brace",
    )
    body.visual(
        Box((0.12, 0.56, 0.05)),
        origin=Origin(xyz=(-0.60, 0.0, 0.515), rpy=(0.0, 0.08, 0.0)),
        material=frame_steel,
        name="rear_handle_cross_brace",
    )

    left_handle_points = [
        (-0.90, 0.24, 0.60),
        (-0.68, 0.24, 0.55),
        (-0.44, 0.245, 0.48),
        (-0.16, 0.252, 0.43),
        (0.06, 0.240, 0.40),
        (0.18, 0.10, 0.37),
    ]
    right_handle_points = _mirror_y(left_handle_points)
    body.visual(
        _tube_mesh("left_handle", left_handle_points, radius=0.016, samples_per_segment=16),
        material=frame_steel,
        name="left_handle",
    )
    body.visual(
        _tube_mesh("right_handle", right_handle_points, radius=0.016, samples_per_segment=16),
        material=frame_steel,
        name="right_handle",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.16),
        origin=Origin(xyz=(-0.88, 0.24, 0.60), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip,
        name="left_grip",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.16),
        origin=Origin(xyz=(-0.88, -0.24, 0.60), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip,
        name="right_grip",
    )

    fork_left_points = [(0.18, 0.10, 0.37), (0.34, 0.075, 0.17)]
    fork_right_points = _mirror_y(fork_left_points)
    body.visual(
        _tube_mesh("fork_left", fork_left_points, radius=0.013, samples_per_segment=2, radial_segments=16),
        material=frame_steel,
        name="fork_left",
    )
    body.visual(
        _tube_mesh("fork_right", fork_right_points, radius=0.013, samples_per_segment=2, radial_segments=16),
        material=frame_steel,
        name="fork_right",
    )
    left_leg_points = [(-0.31, 0.22, 0.345), (-0.48, 0.22, 0.014)]
    right_leg_points = _mirror_y(left_leg_points)
    body.visual(
        _tube_mesh("left_leg", left_leg_points, radius=0.015, samples_per_segment=2, radial_segments=16),
        material=frame_steel,
        name="left_leg",
    )
    body.visual(
        _tube_mesh("right_leg", right_leg_points, radius=0.015, samples_per_segment=2, radial_segments=16),
        material=frame_steel,
        name="right_leg",
    )
    body.visual(
        Box((0.07, 0.034, 0.012)),
        origin=Origin(xyz=(-0.49, 0.22, 0.006)),
        material=frame_steel,
        name="left_foot",
    )
    body.visual(
        Box((0.07, 0.034, 0.012)),
        origin=Origin(xyz=(-0.49, -0.22, 0.006)),
        material=frame_steel,
        name="right_foot",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(xyz=(0.34, 0.064, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_steel,
        name="axle_stub_left",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(xyz=(0.34, -0.064, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_steel,
        name="axle_stub_right",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.085),
        mass=2.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    wheel.visual(
        _wheel_tire_mesh("wheel_tire", radius=0.17, width=0.085),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.108, length=0.018),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="rim_left",
    )
    wheel.visual(
        Cylinder(radius=0.108, length=0.018),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="rim_right",
    )
    wheel.visual(
        Cylinder(radius=0.056, length=0.054),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_steel,
        name="axle_sleeve",
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.34, 0.0, 0.17)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("body_to_wheel")

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="axle_stub_left",
        elem_b="axle_sleeve",
        reason="The wheel rotates on the left axle stub, which is intentionally modeled as inserted into the sleeve bearing.",
    )
    ctx.allow_overlap(
        body,
        wheel,
        elem_a="axle_stub_right",
        elem_b="axle_sleeve",
        reason="The wheel rotates on the right axle stub, which is intentionally modeled as inserted into the sleeve bearing.",
    )

    ctx.expect_gap(
        body,
        wheel,
        axis="z",
        positive_elem="tray_bottom",
        negative_elem="tire",
        min_gap=0.012,
        max_gap=0.050,
        name="wheel stays tucked just below the tray",
    )
    ctx.expect_gap(
        body,
        wheel,
        axis="y",
        positive_elem="fork_left",
        negative_elem="tire",
        min_gap=0.008,
        max_gap=0.035,
        name="left fork clears the tire",
    )
    ctx.expect_gap(
        wheel,
        body,
        axis="y",
        positive_elem="tire",
        negative_elem="fork_right",
        min_gap=0.008,
        max_gap=0.035,
        name="right fork clears the tire",
    )

    wheel_tire_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    left_foot_aabb = ctx.part_element_world_aabb(body, elem="left_foot")
    right_foot_aabb = ctx.part_element_world_aabb(body, elem="right_foot")
    if wheel_tire_aabb and left_foot_aabb and right_foot_aabb:
        ground_points = [
            wheel_tire_aabb[0][2],
            left_foot_aabb[0][2],
            right_foot_aabb[0][2],
        ]
        same_plane = max(ground_points) - min(ground_points) <= 0.015 and max(abs(v) for v in ground_points) <= 0.015
    else:
        same_plane = False
        ground_points = None
    ctx.check(
        "wheel and legs share a realistic resting plane",
        same_plane,
        details=f"ground_points={ground_points}",
    )

    with ctx.pose({wheel_joint: math.pi / 2.0}):
        ctx.expect_gap(
            body,
            wheel,
            axis="z",
            positive_elem="tray_bottom",
            negative_elem="tire",
            min_gap=0.012,
            max_gap=0.050,
            name="wheel spin pose still clears the tray",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

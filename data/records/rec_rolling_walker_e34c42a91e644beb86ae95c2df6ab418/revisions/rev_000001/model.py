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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _add_caster_wheel(part, *, mesh_name: str, tire_material, hub_material) -> None:
    wheel_radius = 0.060
    wheel_width = 0.028
    half_width = wheel_width * 0.5
    tire_profile = [
        (wheel_radius * 0.50, -half_width * 0.95),
        (wheel_radius * 0.78, -half_width),
        (wheel_radius * 0.92, -half_width * 0.82),
        (wheel_radius, -half_width * 0.35),
        (wheel_radius, half_width * 0.35),
        (wheel_radius * 0.92, half_width * 0.82),
        (wheel_radius * 0.78, half_width),
        (wheel_radius * 0.50, half_width * 0.95),
        (wheel_radius * 0.42, half_width * 0.28),
        (wheel_radius * 0.39, 0.0),
        (wheel_radius * 0.42, -half_width * 0.28),
        (wheel_radius * 0.50, -half_width * 0.95),
    ]
    tire_mesh = _save_mesh(mesh_name, LatheGeometry(tire_profile, segments=48).rotate_y(pi / 2.0))
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(
        Cylinder(radius=0.026, length=wheel_width * 0.72),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="outer_hub_cap",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="inner_hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.81, 0.83, 0.85, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tire_gray = model.material("tire_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.48, 0.92)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    side_loop_points = [
        (0.275, 0.230, 0.217),
        (0.275, 0.230, 0.480),
        (0.275, 0.230, 0.820),
        (0.275, 0.075, 0.885),
        (0.275, -0.105, 0.885),
        (0.275, -0.165, 0.805),
        (0.275, -0.165, 0.520),
        (0.275, -0.185, 0.050),
    ]
    tube_radius = 0.011
    frame.visual(
        _save_mesh(
            "walker_left_side_loop",
            tube_from_spline_points(
                side_loop_points,
                radius=tube_radius,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=aluminum,
        name="left_side_loop",
    )
    frame.visual(
        _save_mesh(
            "walker_right_side_loop",
            tube_from_spline_points(
                _mirror_x(side_loop_points),
                radius=tube_radius,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=aluminum,
        name="right_side_loop",
    )

    frame.visual(
        Cylinder(radius=0.010, length=0.55),
        origin=Origin(xyz=(0.0, 0.230, 0.480), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="front_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.55),
        origin=Origin(xyz=(0.0, -0.165, 0.520), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="rear_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.0125, length=0.050),
        origin=Origin(xyz=(0.275, 0.230, 0.217)),
        material=dark_gray,
        name="left_front_socket",
    )
    frame.visual(
        Cylinder(radius=0.0125, length=0.050),
        origin=Origin(xyz=(-0.275, 0.230, 0.217)),
        material=dark_gray,
        name="right_front_socket",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.275, -0.185, 0.025)),
        material=foot_rubber,
        name="left_rear_foot",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(-0.275, -0.185, 0.025)),
        material=foot_rubber,
        name="right_rear_foot",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.125),
        origin=Origin(xyz=(0.275, -0.045, 0.885), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.125),
        origin=Origin(xyz=(-0.275, -0.045, 0.885), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.065, 0.060, 0.135)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.020, -0.067)),
    )
    left_caster_fork.visual(
        Cylinder(radius=0.007, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=dark_gray,
        name="stem",
    )
    left_caster_fork.visual(
        Box((0.045, 0.042, 0.014)),
        origin=Origin(xyz=(0.0, -0.018, -0.062)),
        material=dark_gray,
        name="crown",
    )
    for x_sign, arm_name in ((1.0, "outer_arm"), (-1.0, "inner_arm")):
        left_caster_fork.visual(
            Cylinder(radius=0.005, length=0.058),
            origin=Origin(xyz=(0.020 * x_sign, -0.030, -0.091)),
            material=dark_gray,
            name=arm_name,
        )
    left_caster_fork.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.020, -0.030, -0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="outer_boss",
    )
    left_caster_fork.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(-0.020, -0.030, -0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="inner_boss",
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.065, 0.060, 0.135)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.020, -0.067)),
    )
    right_caster_fork.visual(
        Cylinder(radius=0.007, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=dark_gray,
        name="stem",
    )
    right_caster_fork.visual(
        Box((0.045, 0.042, 0.014)),
        origin=Origin(xyz=(0.0, -0.018, -0.062)),
        material=dark_gray,
        name="crown",
    )
    for x_sign, arm_name in ((1.0, "outer_arm"), (-1.0, "inner_arm")):
        right_caster_fork.visual(
            Cylinder(radius=0.005, length=0.058),
            origin=Origin(xyz=(0.020 * x_sign, -0.030, -0.091)),
            material=dark_gray,
            name=arm_name,
        )
    right_caster_fork.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.020, -0.030, -0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="outer_boss",
    )
    right_caster_fork.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(-0.020, -0.030, -0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="inner_boss",
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.028),
        mass=0.25,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(
        left_caster_wheel,
        mesh_name="left_caster_wheel_tire",
        tire_material=tire_gray,
        hub_material=aluminum,
    )
    left_caster_wheel.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="axle",
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.028),
        mass=0.25,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(
        right_caster_wheel,
        mesh_name="right_caster_wheel_tire",
        tire_material=tire_gray,
        hub_material=aluminum,
    )
    right_caster_wheel.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="axle",
    )

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.275, 0.230, 0.192)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.275, 0.230, 0.192)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, -0.030, -0.132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, -0.030, -0.132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
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
    left_swivel = object_model.get_articulation("left_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_spin")
    left_wheel = object_model.get_part("left_caster_wheel")

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "walker frame uses realistic adult proportions",
            0.52 <= frame_width <= 0.66 and 0.34 <= frame_depth <= 0.48 and 0.86 <= frame_height <= 0.92,
            details=f"width={frame_width:.3f}, depth={frame_depth:.3f}, height={frame_height:.3f}",
        )

    left_grip_aabb = ctx.part_element_world_aabb(frame, elem="left_grip")
    left_foot_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_foot")
    if left_grip_aabb is not None and left_foot_aabb is not None:
        grip_clearance = left_grip_aabb[0][2] - left_foot_aabb[1][2]
        ctx.check(
            "hand grips sit well above rear support feet",
            grip_clearance >= 0.78,
            details=f"vertical_clearance={grip_clearance:.3f}",
        )

    left_wheel_aabb = ctx.part_element_world_aabb(left_wheel, elem="tire")
    if left_wheel_aabb is not None and left_foot_aabb is not None:
        wheel_center_lead = (left_wheel_aabb[0][1] + left_wheel_aabb[1][1]) * 0.5 - (
            (left_foot_aabb[0][1] + left_foot_aabb[1][1]) * 0.5
        )
        wheel_ground = left_wheel_aabb[0][2]
        foot_ground = left_foot_aabb[0][2]
        ctx.check(
            "front caster leads the rear foot and both reach the floor plane",
            wheel_center_lead >= 0.36 and abs(wheel_ground) <= 0.002 and abs(foot_ground) <= 0.002,
            details=f"center_lead={wheel_center_lead:.3f}, wheel_ground={wheel_ground:.4f}, foot_ground={foot_ground:.4f}",
        )

    rest_center = ctx.part_world_position(left_wheel)
    with ctx.pose({left_swivel: 1.1}):
        turned_center = ctx.part_world_position(left_wheel)
    ctx.check(
        "left caster swivel changes wheel heading position",
        rest_center is not None
        and turned_center is not None
        and (
            abs(turned_center[0] - rest_center[0]) > 0.020
            or abs(turned_center[1] - rest_center[1]) > 0.020
        ),
        details=f"rest={rest_center}, turned={turned_center}",
    )

    spin_rest = ctx.part_world_position(left_wheel)
    with ctx.pose({left_spin: 2.4}):
        spin_center = ctx.part_world_position(left_wheel)
    ctx.check(
        "wheel spin keeps caster wheel centered on its axle",
        spin_rest is not None
        and spin_center is not None
        and max(abs(a - b) for a, b in zip(spin_rest, spin_center)) <= 1e-6,
        details=f"rest={spin_rest}, spun={spin_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

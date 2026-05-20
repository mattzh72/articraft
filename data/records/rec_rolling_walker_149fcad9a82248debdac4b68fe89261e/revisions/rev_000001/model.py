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


def _mirror_points_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _add_caster_swivel(part, *, material, accent_material) -> None:
    part.visual(
        Cylinder(radius=0.0085, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=accent_material,
        name="stem",
    )
    part.visual(
        Box((0.040, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, -0.056)),
        material=material,
        name="yoke_block",
    )
    for x_sign, blade_name in ((1.0, "outer_fork_leg"), (-1.0, "inner_fork_leg")):
        part.visual(
            Box((0.006, 0.022, 0.074)),
            origin=Origin(xyz=(0.019 * x_sign, -0.020, -0.087)),
            material=material,
            name=blade_name,
        )
        part.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(
                xyz=(0.021 * x_sign, -0.020, -0.122),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=accent_material,
            name=f"{blade_name}_axle_stub",
        )


def _add_caster_wheel(part, *, tire_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    frame_satin = model.material("frame_satin", rgba=(0.80, 0.81, 0.83, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.36, 0.38, 1.0))
    grip_black = model.material("grip_black", rgba=(0.16, 0.16, 0.16, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.28, 0.29, 0.30, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.68, 0.69, 0.71, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.58, 0.93)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    half_width = 0.27
    tube_radius = 0.013
    rear_foot_y = -0.18
    front_mount_y = 0.18
    handle_z = 0.89
    caster_mount_z = 0.182
    frame_front_leg_end_z = 0.238

    left_side_points = [
        (half_width, rear_foot_y, 0.036),
        (half_width, rear_foot_y, 0.42),
        (half_width, -0.17, 0.80),
        (half_width, -0.05, handle_z),
        (half_width, 0.11, handle_z),
        (half_width, front_mount_y, 0.74),
        (half_width, front_mount_y, frame_front_leg_end_z),
    ]
    right_side_points = _mirror_points_x(left_side_points)

    left_loop = tube_from_spline_points(
        left_side_points,
        radius=tube_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    right_loop = tube_from_spline_points(
        right_side_points,
        radius=tube_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(_mesh("left_side_loop", left_loop), material=frame_satin, name="left_side_loop")
    frame.visual(_mesh("right_side_loop", right_loop), material=frame_satin, name="right_side_loop")

    frame.visual(
        Cylinder(radius=0.010, length=half_width * 2.0),
        origin=Origin(xyz=(0.0, front_mount_y, 0.49), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_satin,
        name="front_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=half_width * 2.0),
        origin=Origin(xyz=(0.0, rear_foot_y, 0.43), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_satin,
        name="rear_crossbar",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(half_width, rear_foot_y, 0.018)),
        material=rubber_gray,
        name="rear_left_foot",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(-half_width, rear_foot_y, 0.018)),
        material=rubber_gray,
        name="rear_right_foot",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(half_width, 0.02, handle_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(-half_width, 0.02, handle_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    frame.visual(
        Cylinder(radius=0.017, length=0.056),
        origin=Origin(xyz=(half_width, front_mount_y, 0.210)),
        material=dark_metal,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.056),
        origin=Origin(xyz=(-half_width, front_mount_y, 0.210)),
        material=dark_metal,
        name="right_caster_socket",
    )

    left_caster = model.part("left_caster")
    left_caster.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.14)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.01, -0.07)),
    )
    _add_caster_swivel(left_caster, material=dark_metal, accent_material=wheel_gray)

    right_caster = model.part("right_caster")
    right_caster.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.14)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.01, -0.07)),
    )
    _add_caster_swivel(right_caster, material=dark_metal, accent_material=wheel_gray)

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.024),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(left_front_wheel, tire_material=rubber_gray, hub_material=wheel_gray)

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.024),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(right_front_wheel, tire_material=rubber_gray, hub_material=wheel_gray)

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_caster,
        origin=Origin(xyz=(half_width, front_mount_y, caster_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_caster,
        origin=Origin(xyz=(-half_width, front_mount_y, caster_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "left_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, -0.022, -0.122)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "right_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, -0.022, -0.122)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_caster = object_model.get_part("left_caster")
    right_caster = object_model.get_part("right_caster")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")

    ctx.expect_origin_gap(
        left_caster,
        right_caster,
        axis="x",
        min_gap=0.52,
        max_gap=0.56,
        name="caster stems span the walker width",
    )
    ctx.expect_origin_gap(
        left_front_wheel,
        right_front_wheel,
        axis="x",
        min_gap=0.52,
        max_gap=0.56,
        name="front wheels stay laterally symmetric",
    )
    ctx.expect_origin_gap(
        left_caster,
        left_front_wheel,
        axis="y",
        min_gap=0.018,
        max_gap=0.026,
        name="left caster wheel trails behind its swivel stem",
    )
    ctx.expect_origin_gap(
        right_caster,
        right_front_wheel,
        axis="y",
        min_gap=0.018,
        max_gap=0.026,
        name="right caster wheel trails behind its swivel stem",
    )
    ctx.expect_gap(
        frame,
        left_front_wheel,
        axis="z",
        positive_elem="left_caster_socket",
        negative_elem="tire",
        min_gap=0.055,
        max_gap=0.070,
        name="left wheel sits below the frame socket",
    )
    ctx.expect_gap(
        frame,
        right_front_wheel,
        axis="z",
        positive_elem="right_caster_socket",
        negative_elem="tire",
        min_gap=0.055,
        max_gap=0.070,
        name="right wheel sits below the frame socket",
    )

    def elem_center(elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(frame, elem=elem_name)
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    left_grip_center = elem_center("left_grip")
    right_grip_center = elem_center("right_grip")
    left_foot_center = elem_center("rear_left_foot")
    right_foot_center = elem_center("rear_right_foot")

    def mirrored_pair_ok(a, b, *, x_tol: float, yz_tol: float) -> bool:
        return (
            a is not None
            and b is not None
            and abs(a[0] + b[0]) <= x_tol
            and abs(a[1] - b[1]) <= yz_tol
            and abs(a[2] - b[2]) <= yz_tol
        )

    ctx.check(
        "hand grips are nearly mirrored about the centerline",
        mirrored_pair_ok(left_grip_center, right_grip_center, x_tol=0.008, yz_tol=0.004),
        details=f"left={left_grip_center}, right={right_grip_center}",
    )
    ctx.check(
        "rear support feet are nearly mirrored about the centerline",
        mirrored_pair_ok(left_foot_center, right_foot_center, x_tol=0.008, yz_tol=0.004),
        details=f"left={left_foot_center}, right={right_foot_center}",
    )

    rest_pos = ctx.part_world_position(left_front_wheel)
    with ctx.pose({left_swivel: 1.2, right_swivel: -1.2}):
        turned_left = ctx.part_world_position(left_front_wheel)
        turned_right = ctx.part_world_position(right_front_wheel)

    ctx.check(
        "caster swivels move the wheels sideways when turned",
        rest_pos is not None
        and turned_left is not None
        and turned_right is not None
        and turned_left[0] > rest_pos[0] + 0.015
        and turned_right[0] < -rest_pos[0] - 0.015,
        details=f"rest={rest_pos}, left_turned={turned_left}, right_turned={turned_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

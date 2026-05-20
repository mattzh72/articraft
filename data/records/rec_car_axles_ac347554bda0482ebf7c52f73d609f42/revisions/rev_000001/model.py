from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _merge_geometries(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _cylinder_along_y(radius: float, length: float, *, radial_segments: int = 28):
    return CylinderGeometry(radius, length, radial_segments=radial_segments).rotate_x(
        math.pi / 2.0
    )


def _build_cross_member_mesh():
    beam = BoxGeometry((0.16, 1.04, 0.06))
    left_mount_cap = BoxGeometry((0.12, 0.10, 0.04)).translate(0.0, 0.57, -0.02)
    right_mount_cap = BoxGeometry((0.12, 0.10, 0.04)).translate(0.0, -0.57, -0.02)
    left_plate_inboard = BoxGeometry((0.10, 0.018, 0.16)).translate(0.0, 0.530, -0.08)
    left_plate_outboard = BoxGeometry((0.10, 0.018, 0.16)).translate(0.0, 0.610, -0.08)
    right_plate_inboard = BoxGeometry((0.10, 0.018, 0.16)).translate(0.0, -0.530, -0.08)
    right_plate_outboard = BoxGeometry((0.10, 0.018, 0.16)).translate(0.0, -0.610, -0.08)
    return _merge_geometries(
        beam,
        left_mount_cap,
        right_mount_cap,
        left_plate_inboard,
        left_plate_outboard,
        right_plate_inboard,
        right_plate_outboard,
    )


def _build_trailing_arm_mesh(side_sign: float):
    pivot_sleeve = _cylinder_along_y(0.04, 0.062)
    front_web = BoxGeometry((0.12, 0.05, 0.08)).translate(0.06, 0.0, 0.0)
    main_beam = BoxGeometry((0.40, 0.08, 0.10)).translate(0.26, side_sign * 0.035, 0.0)
    lower_reinforcement = BoxGeometry((0.20, 0.10, 0.05)).translate(
        0.34,
        side_sign * 0.020,
        -0.045,
    )
    diagonal_brace = (
        BoxGeometry((0.28, 0.05, 0.05))
        .rotate_z(side_sign * 0.16)
        .translate(0.30, side_sign * 0.005, 0.035)
    )
    hub_carrier = BoxGeometry((0.18, 0.12, 0.18)).translate(0.53, side_sign * 0.060, 0.0)
    return _merge_geometries(
        pivot_sleeve,
        front_web,
        main_beam,
        lower_reinforcement,
        diagonal_brace,
        hub_carrier,
    )


def _span(aabb, axis_index: int) -> float:
    return float(aabb[1][axis_index] - aabb[0][axis_index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_rear_trailing_arm_suspension")

    cross_member_paint = model.material(
        "cross_member_paint",
        rgba=(0.24, 0.25, 0.27, 1.0),
    )
    arm_paint = model.material("arm_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.77, 0.79, 0.82, 1.0))
    brake_steel = model.material("brake_steel", rgba=(0.56, 0.57, 0.59, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    cross_member = model.part("cross_member")
    cross_member.visual(
        mesh_from_geometry(_build_cross_member_mesh(), "rear_suspension_cross_member"),
        material=cross_member_paint,
        name="cross_member_shell",
    )
    cross_member.inertial = Inertial.from_geometry(
        Box((0.16, 1.24, 0.20)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    tire_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.22,
            tube=0.08,
            radial_segments=20,
            tubular_segments=48,
        ).rotate_x(math.pi / 2.0),
        "rear_suspension_tire",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        trailing_arm = model.part(f"{side_name}_trailing_arm")
        trailing_arm.visual(
            mesh_from_geometry(
                _build_trailing_arm_mesh(side_sign),
                f"{side_name}_trailing_arm_shell",
            ),
            material=arm_paint,
            name="arm_shell",
        )
        trailing_arm.inertial = Inertial.from_geometry(
            Box((0.62, 0.16, 0.18)),
            mass=18.0,
            origin=Origin(xyz=(0.31, side_sign * 0.04, 0.0)),
        )

        model.articulation(
            f"cross_member_to_{side_name}_trailing_arm",
            ArticulationType.REVOLUTE,
            parent=cross_member,
            child=trailing_arm,
            origin=Origin(xyz=(0.0, side_sign * 0.57, -0.08)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1200.0,
                velocity=2.0,
                lower=-0.30,
                upper=0.55,
            ),
        )

        hub = model.part(f"{side_name}_hub")
        hub.visual(
            Cylinder(radius=0.09, length=0.03),
            origin=Origin(
                xyz=(0.0, side_sign * 0.015, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=machined_steel,
            name="hub_flange",
        )
        hub.visual(
            Cylinder(radius=0.15, length=0.012),
            origin=Origin(
                xyz=(0.0, side_sign * 0.036, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brake_steel,
            name="brake_disc",
        )
        hub.visual(
            Cylinder(radius=0.22, length=0.16),
            origin=Origin(
                xyz=(0.0, side_sign * 0.11, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=wheel_silver,
            name="road_wheel",
        )
        hub.visual(
            Cylinder(radius=0.06, length=0.18),
            origin=Origin(
                xyz=(0.0, side_sign * 0.11, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=machined_steel,
            name="hub_barrel",
        )
        hub.visual(
            tire_mesh,
            origin=Origin(xyz=(0.0, side_sign * 0.11, 0.0)),
            material=tire_rubber,
            name="tire",
        )
        hub.inertial = Inertial.from_geometry(
            Box((0.60, 0.22, 0.60)),
            mass=27.0,
            origin=Origin(xyz=(0.0, side_sign * 0.11, 0.0)),
        )

        model.articulation(
            f"{side_name}_trailing_arm_to_{side_name}_hub",
            ArticulationType.CONTINUOUS,
            parent=trailing_arm,
            child=hub,
            origin=Origin(xyz=(0.58, side_sign * 0.12, 0.0)),
            axis=(0.0, side_sign, 0.0),
            motion_limits=MotionLimits(effort=200.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cross_member = object_model.get_part("cross_member")
    left_arm = object_model.get_part("left_trailing_arm")
    right_arm = object_model.get_part("right_trailing_arm")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_pivot = object_model.get_articulation("cross_member_to_left_trailing_arm")
    right_pivot = object_model.get_articulation("cross_member_to_right_trailing_arm")
    left_axle = object_model.get_articulation("left_trailing_arm_to_left_hub")
    right_axle = object_model.get_articulation("right_trailing_arm_to_right_hub")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_arm,
        cross_member,
        name="left trailing arm sleeve seats in the cross-member hanger",
    )
    ctx.expect_contact(
        right_arm,
        cross_member,
        name="right trailing arm sleeve seats in the cross-member hanger",
    )
    ctx.expect_contact(
        left_hub,
        left_arm,
        elem_a="hub_flange",
        name="left hub flange mounts against the arm tip",
    )
    ctx.expect_contact(
        right_hub,
        right_arm,
        elem_a="hub_flange",
        name="right hub flange mounts against the arm tip",
    )

    cross_member_aabb = ctx.part_world_aabb(cross_member)
    cross_member_is_transverse = (
        cross_member_aabb is not None
        and _span(cross_member_aabb, 1) > 6.0 * _span(cross_member_aabb, 0)
        and _span(cross_member_aabb, 1) > 6.0 * _span(cross_member_aabb, 2)
    )
    ctx.check(
        "cross-member reads as a flat transverse beam",
        cross_member_is_transverse,
        details=f"aabb={cross_member_aabb}",
    )

    pivot_axes_ok = all(
        abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 1e-9 and abs(joint.axis[2]) < 1e-9
        for joint in (left_pivot, right_pivot)
    )
    ctx.check(
        "trailing-arm pivots use horizontal lateral axes",
        pivot_axes_ok,
        details=f"left_axis={left_pivot.axis}, right_axis={right_pivot.axis}",
    )

    axle_axes_ok = all(
        abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 1e-9 and abs(joint.axis[2]) < 1e-9
        for joint in (left_axle, right_axle)
    )
    ctx.check(
        "wheel hubs spin on transverse axle axes",
        axle_axes_ok,
        details=f"left_axis={left_axle.axis}, right_axis={right_axle.axis}",
    )

    left_rest = ctx.part_world_position(left_hub)
    right_rest = ctx.part_world_position(right_hub)
    left_upper = left_pivot.motion_limits.upper if left_pivot.motion_limits is not None else None
    right_upper = right_pivot.motion_limits.upper if right_pivot.motion_limits is not None else None

    left_bumped = None
    if left_upper is not None:
        with ctx.pose({left_pivot: left_upper}):
            left_bumped = ctx.part_world_position(left_hub)
    right_bumped = None
    if right_upper is not None:
        with ctx.pose({right_pivot: right_upper}):
            right_bumped = ctx.part_world_position(right_hub)

    ctx.check(
        "left trailing arm positive rotation lifts the hub",
        left_rest is not None
        and left_bumped is not None
        and left_bumped[2] > left_rest[2] + 0.10,
        details=f"rest={left_rest}, bumped={left_bumped}",
    )
    ctx.check(
        "right trailing arm positive rotation lifts the hub",
        right_rest is not None
        and right_bumped is not None
        and right_bumped[2] > right_rest[2] + 0.10,
        details=f"rest={right_rest}, bumped={right_bumped}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

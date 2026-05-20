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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    stone = model.material("stone", rgba=(0.60, 0.58, 0.54, 1.0))
    timber = model.material("timber", rgba=(0.46, 0.31, 0.17, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.33, 0.24, 0.14, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.20, 0.21, 0.22, 1.0))

    axle_height = 1.75
    wheel_radius = 1.31
    wheel_width = 0.56
    rim_tube = 0.035
    support_x = 0.46
    rim_offset = wheel_width * 0.5

    rim_mesh = _save_mesh(
        "waterwheel_rim_ring",
        TorusGeometry(
            radius=wheel_radius,
            tube=rim_tube,
            radial_segments=18,
            tubular_segments=96,
        ).rotate_y(math.pi / 2.0),
    )

    frame = model.part("frame")
    frame.visual(
        Box((1.65, 0.90, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=stone,
        name="base_beam",
    )
    frame.visual(
        Box((0.34, 0.78, 0.16)),
        origin=Origin(xyz=(-support_x, 0.0, 0.08)),
        material=stone,
        name="left_footing",
    )
    frame.visual(
        Box((0.34, 0.78, 0.16)),
        origin=Origin(xyz=(support_x, 0.0, 0.08)),
        material=stone,
        name="right_footing",
    )
    frame.visual(
        Box((0.22, 0.55, 1.20)),
        origin=Origin(xyz=(-support_x, 0.0, 0.88)),
        material=stone,
        name="left_pedestal",
    )
    frame.visual(
        Box((0.22, 0.55, 1.20)),
        origin=Origin(xyz=(support_x, 0.0, 0.88)),
        material=stone,
        name="right_pedestal",
    )
    frame.visual(
        Box((0.24, 0.32, 0.21)),
        origin=Origin(xyz=(-support_x, 0.0, 1.585)),
        material=dark_iron,
        name="left_bearing",
    )
    frame.visual(
        Box((0.24, 0.32, 0.21)),
        origin=Origin(xyz=(support_x, 0.0, 1.585)),
        material=dark_iron,
        name="right_bearing",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.65, 0.90, 1.70)),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.06, length=1.70),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="main_axle",
    )
    rotor.visual(
        Cylinder(radius=0.14, length=0.62),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub",
    )
    rotor.visual(
        rim_mesh,
        origin=Origin(xyz=(-rim_offset, 0.0, 0.0)),
        material=wet_timber,
        name="rim_left",
    )
    rotor.visual(
        rim_mesh,
        origin=Origin(xyz=(rim_offset, 0.0, 0.0)),
        material=wet_timber,
        name="rim_right",
    )

    spoke_count = 8
    for index in range(spoke_count):
        angle = index * math.tau / spoke_count
        rotor.visual(
            Box((0.52, 0.06, 1.26)),
            origin=Origin(
                xyz=(0.0, 0.69 * math.sin(angle), 0.69 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=timber,
            name=f"spoke_{index:02d}",
        )

    bucket_count = 12
    for index in range(bucket_count):
        angle = index * math.tau / bucket_count + (math.pi / bucket_count)
        rotor.visual(
            Box((0.54, 0.18, 0.06)),
            origin=Origin(
                xyz=(0.0, 1.25 * math.sin(angle), 1.25 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=wet_timber,
            name=f"bucket_floor_{index:02d}",
        )
        rotor.visual(
            Box((0.54, 0.035, 0.22)),
            origin=Origin(
                xyz=(0.0, 1.16 * math.sin(angle), 1.16 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=timber,
            name=f"bucket_divider_{index:02d}",
        )

    rotor.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.66, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="pinion_hub",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.76, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="pinion_body",
    )
    for index in range(16):
        angle = index * math.tau / 16.0
        rotor.visual(
            Box((0.08, 0.05, 0.05)),
            origin=Origin(
                xyz=(0.76, 0.175 * math.sin(angle), 0.175 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_iron,
            name=f"pinion_tooth_{index:02d}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.35, length=1.70),
        mass=850.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "axle_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    axle_rotation = object_model.get_articulation("axle_rotation")

    frame.get_visual("left_bearing")
    frame.get_visual("right_bearing")
    rotor.get_visual("main_axle")
    rotor.get_visual("rim_left")
    rotor.get_visual("rim_right")
    rotor.get_visual("pinion_body")

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

    limits = axle_rotation.motion_limits
    ctx.check(
        "axle_joint_is_continuous",
        axle_rotation.articulation_type == ArticulationType.CONTINUOUS,
        f"expected continuous axle rotation, got {axle_rotation.articulation_type}",
    )
    ctx.check(
        "axle_joint_rotates_about_horizontal_x",
        tuple(round(component, 6) for component in axle_rotation.axis) == (1.0, 0.0, 0.0),
        f"expected axle axis (1, 0, 0), got {axle_rotation.axis}",
    )
    ctx.check(
        "continuous_joint_has_no_angle_stops",
        limits is not None
        and limits.lower is None
        and limits.upper is None
        and limits.velocity > 0.0,
        "continuous axle should have velocity limit but no lower/upper angle limits",
    )

    with ctx.pose({axle_rotation: 0.0}):
        ctx.expect_contact(
            rotor,
            frame,
            contact_tol=0.002,
            name="rotor_is_supported_on_bearings_at_rest",
        )

        left_bearing_aabb = ctx.part_element_world_aabb(frame, elem="left_bearing")
        right_bearing_aabb = ctx.part_element_world_aabb(frame, elem="right_bearing")
        left_rim_aabb = ctx.part_element_world_aabb(rotor, elem="rim_left")
        right_rim_aabb = ctx.part_element_world_aabb(rotor, elem="rim_right")
        pinion_aabb = ctx.part_element_world_aabb(rotor, elem="pinion_body")
        rotor_aabb = ctx.part_world_aabb(rotor)

        wheel_between_supports = (
            left_rim_aabb[0][0] > left_bearing_aabb[1][0] + 0.01
            and right_rim_aabb[1][0] < right_bearing_aabb[0][0] - 0.01
        )
        ctx.check(
            "main_wheel_sits_between_support_blocks",
            wheel_between_supports,
            (
                f"left rim x-range={left_rim_aabb[0][0]:.3f}..{left_rim_aabb[1][0]:.3f}, "
                f"right rim x-range={right_rim_aabb[0][0]:.3f}..{right_rim_aabb[1][0]:.3f}, "
                f"bearing spans={left_bearing_aabb[0][0]:.3f}..{left_bearing_aabb[1][0]:.3f} and "
                f"{right_bearing_aabb[0][0]:.3f}..{right_bearing_aabb[1][0]:.3f}"
            ),
        )
        ctx.check(
            "pinion_is_outboard_on_axle_end",
            pinion_aabb[0][0] > right_bearing_aabb[1][0] + 0.05,
            (
                f"pinion x-range={pinion_aabb[0][0]:.3f}..{pinion_aabb[1][0]:.3f} should sit "
                f"outboard of right bearing ending at x={right_bearing_aabb[1][0]:.3f}"
            ),
        )
        ctx.check(
            "waterwheel_has_large_realistic_diameter",
            (rotor_aabb[1][2] - rotor_aabb[0][2]) >= 2.5,
            f"rotor z-span is {rotor_aabb[1][2] - rotor_aabb[0][2]:.3f} m; expected a large waterwheel",
        )

    with ctx.pose({axle_rotation: math.pi / 3.0}):
        ctx.expect_contact(
            rotor,
            frame,
            contact_tol=0.002,
            name="rotor_remains_supported_after_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

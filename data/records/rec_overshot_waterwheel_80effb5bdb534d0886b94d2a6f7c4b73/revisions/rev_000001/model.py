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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


WHEEL_CENTER_Z = 1.40
WHEEL_RADIUS = 0.74
WHEEL_WIDTH = 0.34
BUCKET_OUTER_RADIUS = 0.83


def _box_along_yz(length: float, thickness: float, width_x: float, roll: float) -> tuple[Box, Origin]:
    """A box whose long local Y axis is rolled in the local YZ plane."""
    return Box((width_x, length, thickness)), Origin(rpy=(roll, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.46, 0.30, 0.16, 1.0))
    dark_wet_wood = model.material("dark_wet_wood", rgba=(0.28, 0.17, 0.08, 1.0))
    end_grain = model.material("end_grain", rgba=(0.56, 0.38, 0.20, 1.0))
    iron = model.material("blackened_iron", rgba=(0.06, 0.06, 0.055, 1.0))
    water = model.material("flume_water", rgba=(0.25, 0.55, 0.85, 0.55))

    frame = model.part("frame")

    # Continuous timber skids tie the entire static structure together so the
    # elevated flume, brake post, and wheel stands read as one supported mill.
    for x in (-0.58, 0.58):
        frame.visual(
            Box((0.12, 2.95, 0.08)),
            origin=Origin(xyz=(x, -0.70, 0.04)),
            material=weathered_wood,
            name=f"ground_skid_{x:+.0f}".replace("+", "p").replace("-", "m"),
        )
    for y, name in [
        (-1.78, "flume_ground_tie"),
        (-1.22, "brake_ground_tie"),
        (-0.55, "flume_low_ground_tie"),
        (0.0, "wheel_ground_tie"),
    ]:
        frame.visual(
            Box((1.36, 0.13, 0.09)),
            origin=Origin(xyz=(0.0, y, 0.075)),
            material=weathered_wood,
            name=name,
        )

    # Paired side supports hold the horizontal shaft between them.
    for x, side_name, bearing_ring_name in [
        (-0.58, "support_0", "support_0_bearing_ring"),
        (0.58, "support_1", "support_1_bearing_ring"),
    ]:
        for y in (-0.34, 0.34):
            frame.visual(
                Box((0.12, 0.12, 1.38)),
                origin=Origin(xyz=(x, y, 0.76)),
                material=weathered_wood,
                name=f"{side_name}_post_{'front' if y < 0 else 'rear'}",
            )
        frame.visual(
            Box((0.14, 0.82, 0.10)),
            origin=Origin(xyz=(x, 0.0, 1.02), rpy=(0.0, 0.0, 0.0)),
            material=weathered_wood,
            name=f"{side_name}_cross_tie",
        )
        for y, roll, label in [(-0.18, -0.48, "brace_front"), (0.18, 0.48, "brace_rear")]:
            frame.visual(
                Box((0.10, 0.58, 0.08)),
                origin=Origin(xyz=(x, y, 0.91), rpy=(roll, 0.0, 0.0)),
                material=weathered_wood,
                name=f"{side_name}_{label}",
            )
        # U-shaped bearing blocks encircle, but do not intersect, the moving shaft.
        frame.visual(
            Box((0.14, 0.26, 0.22)),
            origin=Origin(xyz=(x, -0.205, WHEEL_CENTER_Z)),
            material=weathered_wood,
            name=f"{side_name}_bearing_cheek_0",
        )
        frame.visual(
            Box((0.14, 0.26, 0.22)),
            origin=Origin(xyz=(x, 0.205, WHEEL_CENTER_Z)),
            material=weathered_wood,
            name=f"{side_name}_bearing_cheek_1",
        )
        frame.visual(
            Box((0.14, 0.50, 0.07)),
            origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z + 0.122)),
            material=weathered_wood,
            name=f"{side_name}_bearing_cap",
        )
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.074, 0.012, radial_segments=20, tubular_segments=32), bearing_ring_name),
            origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=bearing_ring_name,
        )

    # A sloped open timber flume feeds the top of the overshot wheel.
    flume_roll = math.atan2(-0.24, 1.55)
    frame.visual(
        Box((0.48, 1.66, 0.055)),
        origin=Origin(xyz=(0.0, -1.18, 2.39), rpy=(flume_roll, 0.0, 0.0)),
        material=weathered_wood,
        name="flume_bottom",
    )
    for x, label in [(-0.265, "flume_side_0"), (0.265, "flume_side_1")]:
        frame.visual(
            Box((0.055, 1.66, 0.24)),
            origin=Origin(xyz=(x, -1.18, 2.49), rpy=(flume_roll, 0.0, 0.0)),
            material=weathered_wood,
            name=label,
        )
    frame.visual(
        Box((0.32, 1.46, 0.025)),
        origin=Origin(xyz=(0.0, -1.24, 2.424), rpy=(flume_roll, 0.0, 0.0)),
        material=water,
        name="water_in_flume",
    )
    for y, z, label, xs in [
        (-1.78, 1.27, "flume_post_high", (-0.24, 0.24)),
        (-0.55, 1.17, "flume_post_low", (-0.42, 0.42)),
    ]:
        for x in xs:
            frame.visual(
                Box((0.10, 0.10, 2.45)),
                origin=Origin(xyz=(x, y, z)),
                material=weathered_wood,
                name=f"{label}_{'a' if x < 0 else 'b'}",
            )
        frame.visual(
            Box((0.68, 0.11, 0.10)),
            origin=Origin(xyz=(0.0, y, z + 1.13)),
            material=weathered_wood,
            name=f"{label}_saddle",
        )

    # Brake support with a clevis-like clip: the moving arm's hinge sleeve lives
    # between these fixed cheeks and remains captured through its motion.
    frame.visual(
        Box((0.11, 0.11, 1.90)),
        origin=Origin(xyz=(0.33, -1.24, 1.005)),
        material=weathered_wood,
        name="brake_post",
    )
    frame.visual(
        Box((0.32, 0.035, 0.24)),
        origin=Origin(xyz=(0.33, -1.235, 1.90)),
        material=iron,
        name="brake_backplate",
    )
    for x, label in [(0.22, "brake_cheek_0"), (0.44, "brake_cheek_1")]:
        frame.visual(
            Box((0.028, 0.26, 0.20)),
            origin=Origin(xyz=(x, -1.105, 1.90)),
            material=iron,
            name=label,
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                rim=WheelRim(inner_radius=0.60, flange_height=0.035, flange_thickness=0.018),
                hub=WheelHub(radius=0.12, width=0.42, cap_style="flat"),
                face=WheelFace(dish_depth=0.010, front_inset=0.010, rear_inset=0.010),
                spokes=WheelSpokes(style="straight", count=12, thickness=0.030, window_radius=0.030),
                bore=WheelBore(style="round", diameter=0.075),
            ),
            "waterwheel_spoked_rim",
        ),
        material=weathered_wood,
        name="spoked_rim",
    )
    wheel.visual(
        Cylinder(radius=0.063, length=1.36),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="shaft",
    )
    wheel.visual(
        Cylinder(radius=0.130, length=0.44),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_wood,
        name="hub_sleeve",
    )
    wheel.visual(
        mesh_from_geometry(TorusGeometry(0.770, 0.055, radial_segments=20, tubular_segments=64), "bucket_support_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_wood,
        name="bucket_support_ring",
    )
    for x, label in [(-0.19, "iron_tire_0"), (0.19, "iron_tire_1")]:
        wheel.visual(
            mesh_from_geometry(TorusGeometry(WHEEL_RADIUS + 0.012, 0.018, radial_segments=20, tubular_segments=48), label),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=label,
        )
    for i in range(16):
        angle = 2.0 * math.pi * i / 16.0
        radial_y = math.sin(angle)
        radial_z = math.cos(angle)
        wheel.visual(
            Box((0.41, 0.18, 0.060)),
            origin=Origin(
                xyz=(0.0, radial_y * (BUCKET_OUTER_RADIUS - 0.020), radial_z * (BUCKET_OUTER_RADIUS - 0.020)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=dark_wet_wood if i % 2 else end_grain,
            name=f"bucket_{i}",
        )
        wheel.visual(
            Box((0.045, 0.14, 0.12)),
            origin=Origin(
                xyz=(-0.225, radial_y * (BUCKET_OUTER_RADIUS - 0.035), radial_z * (BUCKET_OUTER_RADIUS - 0.035)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=dark_wet_wood,
            name=f"bucket_side_0_{i}",
        )
        wheel.visual(
            Box((0.045, 0.14, 0.12)),
            origin=Origin(
                xyz=(0.225, radial_y * (BUCKET_OUTER_RADIUS - 0.035), radial_z * (BUCKET_OUTER_RADIUS - 0.035)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=dark_wet_wood,
            name=f"bucket_side_1_{i}",
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8),
    )

    brake_arm = model.part("brake_arm")
    brake_roll = math.atan2(-0.25, 0.42)
    brake_arm.visual(
        Cylinder(radius=0.042, length=0.20),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="pivot_sleeve",
    )
    brake_arm.visual(
        Box((0.11, 0.19, 0.07)),
        origin=Origin(xyz=(0.0, 0.07, -0.045), rpy=(brake_roll, 0.0, 0.0)),
        material=iron,
        name="lever_socket",
    )
    brake_arm.visual(
        Box((0.075, 1.05, 0.075)),
        origin=Origin(xyz=(0.0, 0.45 * math.cos(brake_roll), 0.45 * math.sin(brake_roll)), rpy=(brake_roll, 0.0, 0.0)),
        material=weathered_wood,
        name="lever",
    )
    brake_arm.visual(
        Cylinder(radius=0.030, length=0.24),
        origin=Origin(xyz=(0.0, 0.025, 0.11)),
        material=iron,
        name="hand_grip",
    )
    brake_arm.visual(
        Box((0.10, 0.18, 0.09)),
        origin=Origin(
            xyz=(0.035, 0.28 * math.cos(brake_roll), 0.28 * math.sin(brake_roll)),
            rpy=(brake_roll, 0.0, 0.0),
        ),
        material=dark_wet_wood,
        name="brake_shoe",
    )
    model.articulation(
        "frame_to_brake_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brake_arm,
        origin=Origin(xyz=(0.33, -1.105, 1.90)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.7, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    brake_arm = object_model.get_part("brake_arm")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    brake_joint = object_model.get_articulation("frame_to_brake_arm")

    ctx.check(
        "wheel_joint_is_continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={wheel_joint.articulation_type!r}",
    )
    ctx.check(
        "brake_joint_is_limited_revolute",
        brake_joint.articulation_type == ArticulationType.REVOLUTE
        and brake_joint.motion_limits is not None
        and brake_joint.motion_limits.lower == 0.0
        and brake_joint.motion_limits.upper > 0.0,
        details=f"joint_type={brake_joint.articulation_type!r}, limits={brake_joint.motion_limits!r}",
    )

    ctx.expect_within(
        brake_arm,
        frame,
        axes="xz",
        margin=0.012,
        inner_elem="pivot_sleeve",
        outer_elem="brake_backplate",
        name="brake sleeve stays captured by clevis span",
    )
    ctx.expect_gap(
        brake_arm,
        frame,
        axis="y",
        min_gap=0.0,
        max_gap=0.080,
        positive_elem="pivot_sleeve",
        negative_elem="brake_backplate",
        name="brake sleeve is clipped close to support",
    )
    for bearing_name in ("support_0_bearing_ring", "support_1_bearing_ring"):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=bearing_name,
            elem_b="shaft",
            reason="The iron shaft is intentionally seated through the bearing ring as a rotating journal.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="yz",
            margin=0.003,
            inner_elem="shaft",
            outer_elem=bearing_name,
            name=f"shaft centered in {bearing_name}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            min_overlap=0.020,
            elem_a="shaft",
            elem_b=bearing_name,
            name=f"shaft retained by {bearing_name}",
        )
    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        margin=0.030,
        inner_elem="shaft",
        outer_elem="support_1_bearing_ring",
        name="shaft passes through side bearing opening",
    )

    rest_bucket = ctx.part_element_world_aabb(wheel, elem="bucket_0")
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        spun_bucket = ctx.part_element_world_aabb(wheel, elem="bucket_0")
    if rest_bucket is not None and spun_bucket is not None:
        rest_center_y = (rest_bucket[0][1] + rest_bucket[1][1]) * 0.5
        spun_center_y = (spun_bucket[0][1] + spun_bucket[1][1]) * 0.5
        rest_center_z = (rest_bucket[0][2] + rest_bucket[1][2]) * 0.5
        spun_center_z = (spun_bucket[0][2] + spun_bucket[1][2]) * 0.5
        ctx.check(
            "wheel bucket moves around axle",
            abs(spun_center_y - rest_center_y) > 0.35 and abs(spun_center_z - rest_center_z) > 0.35,
            details=f"rest_yz={(rest_center_y, rest_center_z)}, spun_yz={(spun_center_y, spun_center_z)}",
        )
    else:
        ctx.fail("wheel bucket aabb available", "Expected bucket_0 AABBs in rest and spun poses.")

    shoe_near = ctx.part_element_world_aabb(brake_arm, elem="brake_shoe")
    hinge_near = ctx.part_world_position(brake_arm)
    with ctx.pose({brake_joint: 0.45}):
        shoe_released = ctx.part_element_world_aabb(brake_arm, elem="brake_shoe")
        hinge_released = ctx.part_world_position(brake_arm)
    if shoe_near is not None and shoe_released is not None:
        def radial_center(aabb):
            cy = (aabb[0][1] + aabb[1][1]) * 0.5
            cz = (aabb[0][2] + aabb[1][2]) * 0.5
            return math.hypot(cy, cz - WHEEL_CENTER_Z)

        ctx.check(
            "brake shoe moves toward rim",
            radial_center(shoe_near) < radial_center(shoe_released) - 0.010,
            details=f"near_r={radial_center(shoe_near):.3f}, released_r={radial_center(shoe_released):.3f}",
        )
    else:
        ctx.fail("brake shoe aabb available", "Expected brake_shoe AABBs in near and released poses.")
    ctx.check(
        "brake hinge pivot remains attached",
        hinge_near is not None
        and hinge_released is not None
        and math.dist(tuple(hinge_near), tuple(hinge_released)) < 1.0e-6,
        details=f"near={hinge_near}, released={hinge_released}",
    )

    return ctx.report()


object_model = build_object_model()

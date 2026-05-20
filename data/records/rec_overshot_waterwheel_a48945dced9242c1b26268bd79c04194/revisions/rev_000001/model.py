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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, thickness: float):
    annulus = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(annulus, name)


def _hollow_sleeve_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    half_length: float,
):
    outer_profile = [
        (outer_radius, -half_length),
        (outer_radius, half_length),
    ]
    inner_profile = [
        (inner_radius, -half_length),
        (inner_radius, half_length),
    ]
    sleeve = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(sleeve, name)


def _polar_yz(radius: float, angle: float) -> tuple[float, float]:
    return radius * math.sin(angle), radius * math.cos(angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    frame_timber = model.material("frame_timber", rgba=(0.45, 0.31, 0.19, 1.0))
    wheel_timber = model.material("wheel_timber", rgba=(0.52, 0.37, 0.22, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.34, 0.25, 0.17, 1.0))
    iron = model.material("iron", rgba=(0.33, 0.34, 0.36, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    brake_block = model.material("brake_block", rgba=(0.20, 0.16, 0.12, 1.0))

    wheel_radius = 1.55
    wheel_width = 0.84
    axle_z = 2.05
    shaft_radius = 0.065
    support_x = 0.72
    post_y = 0.44
    brake_pivot_z = 4.12

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.10, 3.60, 4.20)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
    )

    for side in (-1.0, 1.0):
        x = side * 0.78
        frame.visual(
            Box((0.22, 3.30, 0.16)),
            origin=Origin(xyz=(x, 0.0, 0.08)),
            material=frame_timber,
        )

    for y in (-1.25, 0.0, 1.55):
        frame.visual(
            Box((1.78, 0.22, 0.16)),
            origin=Origin(xyz=(0.0, y, 0.08)),
            material=frame_timber,
        )

    for side in (-1.0, 1.0):
        x = side * support_x
        for y in (-post_y, post_y):
            frame.visual(
                Box((0.18, 0.20, 3.68)),
                origin=Origin(xyz=(x, y, 2.00)),
                material=frame_timber,
            )
        frame.visual(
            Box((0.24, 0.98, 0.18)),
            origin=Origin(xyz=(x, 0.0, 3.84)),
            material=frame_timber,
        )
        frame.visual(
            Box((0.24, 0.88, 0.24)),
            origin=Origin(xyz=(x, 0.0, axle_z)),
            material=frame_timber,
        )
        for brace_y, brace_roll in ((-0.20, 0.36), (0.20, -0.36)):
            frame.visual(
                Box((0.12, 0.14, 1.95)),
                origin=Origin(xyz=(x, brace_y, 1.22), rpy=(brace_roll, 0.0, 0.0)),
                material=frame_timber,
            )
    for side in (-1.0, 1.0):
        frame.visual(
            Box((0.24, 0.06, 0.06)),
            origin=Origin(xyz=(side * 0.60, 0.54, 3.84)),
            material=frame_timber,
            name="flume_pad_left" if side < 0.0 else "flume_pad_right",
        )

    frame.visual(
        Box((0.08, 0.20, 0.20)),
        origin=Origin(xyz=(-0.56, 0.0, axle_z)),
        material=iron,
        name="left_bearing_cap",
    )
    frame.visual(
        Box((0.08, 0.20, 0.20)),
        origin=Origin(xyz=(0.56, 0.0, axle_z)),
        material=iron,
        name="right_bearing_cap",
    )

    for y in (-0.075, 0.075):
        frame.visual(
            Box((0.18, 0.04, 0.38)),
            origin=Origin(xyz=(-support_x, y, brake_pivot_z)),
            material=dark_iron,
        )

    waterwheel = model.part("waterwheel")
    waterwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=420.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rim_mesh = _annulus_mesh(
        "waterwheel_rim_ring",
        outer_radius=wheel_radius,
        inner_radius=1.41,
        thickness=0.06,
    )
    hub_mesh = _annulus_mesh(
        "waterwheel_hub_ring",
        outer_radius=0.19,
        inner_radius=shaft_radius,
        thickness=0.05,
    )
    waterwheel.visual(rim_mesh, origin=Origin(xyz=(-0.39, 0.0, 0.0)), material=wheel_timber, name="rim_left")
    waterwheel.visual(rim_mesh, origin=Origin(xyz=(0.39, 0.0, 0.0)), material=wheel_timber, name="rim_right")
    waterwheel.visual(hub_mesh, origin=Origin(xyz=(-0.25, 0.0, 0.0)), material=wheel_timber, name="hub_left")
    waterwheel.visual(hub_mesh, origin=Origin(xyz=(0.25, 0.0, 0.0)), material=wheel_timber, name="hub_right")
    waterwheel.visual(
        Cylinder(radius=shaft_radius, length=1.04),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="axle",
    )

    for spoke_index in range(8):
        angle = 2.0 * math.pi * spoke_index / 8.0
        y, z = _polar_yz(0.81, angle)
        waterwheel.visual(
            Box((0.82, 0.08, 1.24)),
            origin=Origin(xyz=(0.0, y, z), rpy=(-angle, 0.0, 0.0)),
            material=wheel_timber,
            name="spoke_0" if spoke_index == 0 else None,
        )

    bucket_count = 12
    bucket_step = 2.0 * math.pi / bucket_count
    for bucket_index in range(bucket_count):
        angle = bucket_index * bucket_step
        divider_y, divider_z = _polar_yz(1.17, angle)
        waterwheel.visual(
            Box((0.82, 0.05, 0.58)),
            origin=Origin(xyz=(0.0, divider_y, divider_z), rpy=(-angle, 0.0, 0.0)),
            material=wheel_timber,
            name="bucket_0" if bucket_index == 0 else None,
        )

        back_angle = angle + bucket_step * 0.34
        back_y, back_z = _polar_yz(1.34, angle + bucket_step * 0.38)
        waterwheel.visual(
            Box((0.82, 0.06, 0.34)),
            origin=Origin(xyz=(0.0, back_y, back_z), rpy=(-back_angle, 0.0, 0.0)),
            material=wheel_timber,
        )

    flume = model.part("flume")
    flume.inertial = Inertial.from_geometry(
        Box((0.95, 1.80, 4.10)),
        mass=150.0,
        origin=Origin(xyz=(0.0, 0.90, 2.05)),
    )

    flume_roll = 0.25
    flume.visual(
        Box((0.84, 1.60, 0.10)),
        origin=Origin(xyz=(0.0, 0.85, 3.90), rpy=(flume_roll, 0.0, 0.0)),
        material=wet_timber,
        name="trough_floor",
    )
    for x in (-0.38, 0.38):
        flume.visual(
            Box((0.08, 1.60, 0.28)),
            origin=Origin(xyz=(x, 0.85, 4.06), rpy=(flume_roll, 0.0, 0.0)),
            material=wet_timber,
        )
    flume.visual(
        Box((0.84, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.22, 3.82), rpy=(flume_roll, 0.0, 0.0)),
        material=wet_timber,
    )
    flume.visual(
        Box((0.84, 0.10, 0.22)),
        origin=Origin(xyz=(0.0, 1.59, 4.10), rpy=(flume_roll, 0.0, 0.0)),
        material=wet_timber,
    )
    flume.visual(
        Box((1.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.76, 3.96), rpy=(flume_roll, 0.0, 0.0)),
        material=frame_timber,
    )
    flume.visual(
        Box((1.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 1.02, 4.04), rpy=(flume_roll, 0.0, 0.0)),
        material=frame_timber,
    )
    for x in (-0.60, 0.60):
        flume.visual(
            Box((0.12, 0.50, 0.12)),
            origin=Origin(xyz=(x, 0.82, 3.93)),
            material=frame_timber,
        )

    brake_arm = model.part("brake_arm")
    brake_arm.inertial = Inertial.from_geometry(
        Box((0.86, 0.18, 0.82)),
        mass=55.0,
        origin=Origin(xyz=(0.08, 0.0, -0.18)),
    )
    brake_arm.visual(
        Box((0.32, 0.11, 0.08)),
        material=dark_iron,
        name="hinge_tongue",
    )
    brake_arm.visual(
        Box((0.04, 0.09, 0.35)),
        origin=Origin(xyz=(0.16, 0.0, -0.215)),
        material=dark_iron,
    )
    brake_arm.visual(
        Box((0.44, 0.08, 0.10)),
        origin=Origin(xyz=(0.22, 0.0, -0.44)),
        material=frame_timber,
    )
    brake_arm.visual(
        Box((0.24, 0.08, 0.10)),
        origin=Origin(xyz=(-0.12, 0.0, 0.08)),
        material=frame_timber,
    )
    brake_arm.visual(
        Box((0.12, 0.08, 0.08)),
        origin=Origin(xyz=(0.42, 0.0, -0.44)),
        material=brake_block,
        name="brake_shoe",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=waterwheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0),
    )
    model.articulation(
        "frame_to_flume",
        ArticulationType.FIXED,
        parent=frame,
        child=flume,
        origin=Origin(),
    )
    model.articulation(
        "brake_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brake_arm,
        origin=Origin(xyz=(-support_x, 0.0, brake_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=-0.30, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    waterwheel = object_model.get_part("waterwheel")
    flume = object_model.get_part("flume")
    brake_arm = object_model.get_part("brake_arm")
    wheel_spin = object_model.get_articulation("wheel_spin")
    brake_pivot = object_model.get_articulation("brake_pivot")

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

    assert wheel_spin.axis == (1.0, 0.0, 0.0)
    assert brake_pivot.axis == (0.0, 1.0, 0.0)

    ctx.expect_contact(waterwheel, frame, elem_a="axle", elem_b="left_bearing_cap", name="left_hub_bears_on_shaft")
    ctx.expect_contact(waterwheel, frame, elem_a="axle", elem_b="right_bearing_cap", name="right_hub_bears_on_shaft")
    ctx.expect_contact(flume, frame, name="flume_supported_by_frame")
    ctx.expect_contact(brake_arm, frame, name="brake_arm_clipped_to_support")
    ctx.expect_overlap(flume, waterwheel, axes="x", min_overlap=0.70, name="flume_centered_over_wheel_width")
    ctx.expect_gap(
        flume,
        waterwheel,
        axis="z",
        positive_elem="trough_floor",
        min_gap=0.04,
        max_gap=0.30,
        name="flume_floor_clears_wheel_top",
    )

    bucket_rest = ctx.part_element_world_aabb(waterwheel, elem="bucket_0")
    assert bucket_rest is not None
    with ctx.pose({wheel_spin: 1.10}):
        bucket_spun = ctx.part_element_world_aabb(waterwheel, elem="bucket_0")
        assert bucket_spun is not None
        assert bucket_spun[1][2] < bucket_rest[1][2] - 0.55

    shoe_rest = ctx.part_element_world_aabb(brake_arm, elem="brake_shoe")
    assert shoe_rest is not None
    with ctx.pose({brake_pivot: -0.20}):
        ctx.expect_contact(brake_arm, frame, name="brake_stays_hinged_when_lifted")
    with ctx.pose({brake_pivot: 0.18}):
        shoe_engaged = ctx.part_element_world_aabb(brake_arm, elem="brake_shoe")
        assert shoe_engaged is not None
        assert shoe_engaged[0][2] < shoe_rest[0][2] - 0.04
        ctx.expect_contact(brake_arm, frame, name="brake_stays_hinged_when_engaged")
        ctx.expect_contact(
            brake_arm,
            waterwheel,
            elem_a="brake_shoe",
            elem_b="rim_left",
            name="brake_shoe_reaches_rim",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

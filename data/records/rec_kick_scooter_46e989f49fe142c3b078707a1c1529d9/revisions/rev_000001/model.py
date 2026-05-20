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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stability_kick_scooter")

    aluminum = model.material("aluminum", rgba=(0.74, 0.77, 0.80, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.28, 0.30, 0.33, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    accent = model.material("accent_blue", rgba=(0.17, 0.37, 0.62, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def xz_section(
        width: float,
        height: float,
        *,
        y: float,
        z_center: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)]

    def wheel_profile(
        *,
        radius: float,
        width: float,
        hub_radius: float,
        hub_width: float,
    ) -> list[tuple[float, float]]:
        half_width = width * 0.5
        hub_half = hub_width * 0.5
        bore_radius = max(hub_radius * 0.42, 0.005)
        inner_rim = radius * 0.68
        shoulder_r = radius * 0.90
        return [
            (bore_radius, -hub_half * 0.62),
            (hub_radius, -hub_half),
            (inner_rim, -half_width * 0.42),
            (shoulder_r, -half_width * 0.26),
            (radius, -half_width * 0.08),
            (radius, half_width * 0.08),
            (shoulder_r, half_width * 0.26),
            (inner_rim, half_width * 0.42),
            (hub_radius, hub_half),
            (bore_radius, hub_half * 0.62),
            (bore_radius, -hub_half * 0.62),
        ]

    def add_wheel(
        part,
        *,
        mesh_name: str,
        radius: float,
        width: float,
        hub_radius: float,
        hub_width: float,
        cap_length: float | None = None,
    ) -> None:
        wheel_mesh = save_mesh(
            mesh_name,
            LatheGeometry(
                wheel_profile(radius=radius, width=width, hub_radius=hub_radius, hub_width=hub_width),
                segments=64,
            ).rotate_y(pi / 2.0),
        )
        part.visual(wheel_mesh, material=rubber, name="wheel_shell")
        part.visual(
            Cylinder(radius=radius * 0.42, length=width * 0.72),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_alloy,
            name="hub_barrel",
        )
        part.visual(
            Cylinder(radius=hub_radius * 0.88, length=cap_length if cap_length is not None else width * 0.94),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=accent,
            name="hub_cap",
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.84, 0.68, 1.00)),
        mass=8.6,
        origin=Origin(xyz=(0.0, -0.01, 0.26)),
    )

    deck_shell = section_loft(
        [
            xz_section(0.255, 0.020, y=-0.335, z_center=0.108, radius=0.009),
            xz_section(0.245, 0.022, y=-0.255, z_center=0.107, radius=0.010),
            xz_section(0.188, 0.028, y=-0.135, z_center=0.101, radius=0.013),
            xz_section(0.145, 0.024, y=0.025, z_center=0.100, radius=0.011),
            xz_section(0.118, 0.026, y=0.165, z_center=0.102, radius=0.012),
            xz_section(0.072, 0.050, y=0.252, z_center=0.118, radius=0.014),
        ]
    )
    frame.visual(save_mesh("scooter_deck_shell", deck_shell), material=aluminum, name="deck_shell")
    frame.visual(
        Box((0.110, 0.385, 0.003)),
        origin=Origin(xyz=(0.0, -0.025, 0.1155)),
        material=grip_black,
        name="grip_pad",
    )
    frame.visual(
        Box((0.060, 0.430, 0.022)),
        origin=Origin(xyz=(0.0, -0.030, 0.089)),
        material=dark_alloy,
        name="underside_spine",
    )
    frame.visual(
        Box((0.084, 0.095, 0.042)),
        origin=Origin(xyz=(0.0, 0.196, 0.136)),
        material=dark_alloy,
        name="front_gusset",
    )
    frame.visual(
        Cylinder(radius=0.033, length=0.085),
        origin=Origin(xyz=(0.0, 0.230, 0.1875)),
        material=dark_alloy,
        name="head_tube",
    )
    frame.visual(
        Box((0.164, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, -0.272, 0.103)),
        material=dark_alloy,
        name="rear_axle_beam",
    )

    for side_name, x_sign in (("left", 1.0), ("right", -1.0)):
        x_center = x_sign * 0.105
        inboard_x = x_sign * 0.084
        outboard_x = x_sign * 0.126
        strut_points = [
            (x_sign * 0.086, -0.172, 0.098),
            (x_sign * 0.093, -0.220, 0.106),
            (x_sign * 0.098, -0.272, 0.109),
        ]
        frame.visual(
            save_mesh(f"{side_name}_rear_strut", tube_from_spline_points(strut_points, radius=0.011, samples_per_segment=10)),
            material=dark_alloy,
            name=f"{side_name}_rear_strut",
        )
        frame.visual(
            Box((0.015, 0.020, 0.014)),
            origin=Origin(xyz=(x_sign * 0.083, -0.272, 0.102)),
            material=dark_alloy,
            name=f"{side_name}_rear_inner_arm",
        )
        frame.visual(
            Box((0.006, 0.022, 0.088)),
            origin=Origin(xyz=(inboard_x, -0.272, 0.070)),
            material=dark_alloy,
            name=f"{side_name}_rear_inboard_plate",
        )
        frame.visual(
            Box((0.006, 0.022, 0.088)),
            origin=Origin(xyz=(outboard_x, -0.272, 0.070)),
            material=dark_alloy,
            name=f"{side_name}_rear_outboard_plate",
        )
        frame.visual(
            Box((0.048, 0.024, 0.014)),
            origin=Origin(xyz=(x_center, -0.272, 0.118)),
            material=dark_alloy,
            name=f"{side_name}_rear_bridge",
        )

    front_assembly = model.part("front_assembly")
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.60, 0.30, 0.86)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.015, 0.40)),
    )
    front_assembly.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.095)),
        material=dark_alloy,
        name="steering_collar",
    )
    front_assembly.visual(
        save_mesh(
            "steering_column",
            tube_from_spline_points(
                [
                    (0.0, 0.012, 0.095),
                    (0.0, 0.010, 0.308),
                    (0.0, -0.006, 0.572),
                    (0.0, -0.018, 0.818),
                ],
                radius=0.016,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=aluminum,
        name="steering_column",
    )
    front_assembly.visual(
        save_mesh(
            "fork_spine",
            tube_from_spline_points(
                [
                    (0.0, 0.024, 0.090),
                    (0.0, 0.070, 0.104),
                    (0.0, 0.124, 0.112),
                    (0.0, 0.168, 0.108),
                ],
                radius=0.014,
                samples_per_segment=10,
                radial_segments=18,
            ),
        ),
        material=dark_alloy,
        name="fork_spine",
    )
    front_assembly.visual(
        save_mesh(
            "handlebar_tube",
            tube_from_spline_points(
                [
                    (-0.285, -0.022, 0.794),
                    (-0.175, -0.020, 0.824),
                    (-0.070, -0.018, 0.838),
                    (0.070, -0.018, 0.838),
                    (0.175, -0.020, 0.824),
                    (0.285, -0.022, 0.794),
                ],
                radius=0.012,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=aluminum,
        name="handlebar_tube",
    )
    front_assembly.visual(
        Box((0.062, 0.026, 0.052)),
        origin=Origin(xyz=(0.0, -0.020, 0.818)),
        material=dark_alloy,
        name="handlebar_clamp",
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.294, -0.022, 0.794), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.294, -0.022, 0.794), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    front_assembly.visual(
        Box((0.090, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.170, 0.100)),
        material=dark_alloy,
        name="fork_crown",
    )
    front_assembly.visual(
        Box((0.012, 0.024, 0.150)),
        origin=Origin(xyz=(0.030, 0.190, 0.030)),
        material=dark_alloy,
        name="left_fork_leg",
    )
    front_assembly.visual(
        Box((0.012, 0.024, 0.150)),
        origin=Origin(xyz=(-0.030, 0.190, 0.030)),
        material=dark_alloy,
        name="right_fork_leg",
    )
    front_assembly.visual(
        Box((0.016, 0.024, 0.028)),
        origin=Origin(xyz=(0.030, 0.190, -0.045)),
        material=dark_alloy,
        name="left_dropout",
    )
    front_assembly.visual(
        Box((0.016, 0.024, 0.028)),
        origin=Origin(xyz=(-0.030, 0.190, -0.045)),
        material=dark_alloy,
        name="right_dropout",
    )
    front_assembly.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.0, 0.190, -0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="front_axle",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.045),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel(
        front_wheel,
        mesh_name="front_wheel_mesh",
        radius=0.100,
        width=0.045,
        hub_radius=0.026,
        hub_width=0.025,
        cap_length=0.044,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.028),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel(
        rear_left_wheel,
        mesh_name="rear_left_wheel_mesh",
        radius=0.045,
        width=0.028,
        hub_radius=0.017,
        hub_width=0.018,
        cap_length=0.036,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.028),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel(
        rear_right_wheel,
        mesh_name="rear_right_wheel_mesh",
        radius=0.045,
        width=0.028,
        hub_radius=0.017,
        hub_width=0.018,
        cap_length=0.036,
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_assembly,
        origin=Origin(xyz=(0.0, 0.230, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.6, lower=-0.62, upper=0.62),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.190, -0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.105, -0.272, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=28.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.105, -0.272, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_assembly = object_model.get_part("front_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")

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
    ctx.allow_overlap(
        frame,
        front_assembly,
        elem_a="head_tube",
        elem_b="fork_spine",
        reason="The steering stem passes through the head tube as the front assembly yaws.",
    )
    ctx.allow_overlap(
        front_assembly,
        front_wheel,
        elem_a="front_axle",
        elem_b="hub_cap",
        reason="Front wheel hub rotates around the fixed fork axle.",
    )
    ctx.allow_overlap(
        front_assembly,
        front_wheel,
        elem_a="front_axle",
        elem_b="hub_barrel",
        reason="The front axle runs through the wheel hub barrel for continuous wheel spin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("steering axis is vertical yaw", steering_yaw.axis == (0.0, 0.0, 1.0), details=str(steering_yaw.axis))
    ctx.check(
        "all wheel spin axes follow axle direction",
        front_wheel_spin.axis == (1.0, 0.0, 0.0)
        and rear_left_wheel_spin.axis == (1.0, 0.0, 0.0)
        and rear_right_wheel_spin.axis == (1.0, 0.0, 0.0),
        details=f"{front_wheel_spin.axis}, {rear_left_wheel_spin.axis}, {rear_right_wheel_spin.axis}",
    )

    ctx.expect_contact(frame, front_assembly, elem_a="head_tube", elem_b="steering_collar")

    ctx.expect_contact(front_wheel, front_assembly, elem_b="front_axle", name="front wheel is supported by fork axle")
    ctx.expect_gap(
        front_assembly,
        front_wheel,
        axis="x",
        positive_elem="left_fork_leg",
        min_gap=0.001,
        max_gap=0.008,
        name="front wheel clears left fork leg",
    )
    ctx.expect_gap(
        front_wheel,
        front_assembly,
        axis="x",
        negative_elem="right_fork_leg",
        min_gap=0.001,
        max_gap=0.008,
        name="front wheel clears right fork leg",
    )
    ctx.expect_contact(rear_left_wheel, frame, elem_b="left_rear_inboard_plate", name="left rear wheel is supported inboard")
    ctx.expect_contact(rear_left_wheel, frame, elem_b="left_rear_outboard_plate", name="left rear wheel is supported outboard")
    ctx.expect_contact(rear_right_wheel, frame, elem_b="right_rear_inboard_plate", name="right rear wheel is supported inboard")
    ctx.expect_contact(rear_right_wheel, frame, elem_b="right_rear_outboard_plate", name="right rear wheel is supported outboard")

    front_rest = ctx.part_world_position(front_wheel)
    assert front_rest is not None
    with ctx.pose({steering_yaw: 0.45}):
        front_turned = ctx.part_world_position(front_wheel)
        assert front_turned is not None
        ctx.check(
            "steering swings front wheel laterally",
            front_turned[0] < front_rest[0] - 0.04,
            details=f"rest={front_rest}, turned={front_turned}",
        )
        ctx.expect_contact(front_wheel, front_assembly, elem_b="front_axle", name="front wheel stays on axle in steer pose")
        ctx.expect_overlap(
            front_wheel,
            front_assembly,
            axes="yz",
            elem_b="left_fork_leg",
            min_overlap=0.02,
            name="front wheel stays captured at left fork in steer pose",
        )
        ctx.expect_overlap(
            front_wheel,
            front_assembly,
            axes="yz",
            elem_b="right_fork_leg",
            min_overlap=0.02,
            name="front wheel stays captured at right fork in steer pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 14,
    radial_segments: int = 18,
):
    return _save_mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
    )


def _wheel_tire_mesh(name: str, *, radius: float, width: float):
    half_w = width * 0.5
    profile = [
        (radius * 0.56, -half_w * 0.96),
        (radius * 0.78, -half_w),
        (radius * 0.93, -half_w * 0.78),
        (radius, -half_w * 0.30),
        (radius, half_w * 0.30),
        (radius * 0.93, half_w * 0.78),
        (radius * 0.78, half_w),
        (radius * 0.56, half_w * 0.96),
        (radius * 0.46, half_w * 0.34),
        (radius * 0.42, 0.0),
        (radius * 0.46, -half_w * 0.34),
        (radius * 0.56, -half_w * 0.96),
    ]
    return _save_mesh(name, LatheGeometry(profile, segments=64).rotate_x(math.pi / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wheelbarrow")

    tray_green = model.material("tray_green", rgba=(0.29, 0.41, 0.25, 1.0))
    frame_black = model.material("frame_black", rgba=(0.16, 0.16, 0.17, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.75, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.55, 0.64, 0.62)),
        mass=16.0,
        origin=Origin(xyz=(0.16, 0.0, 0.31)),
    )

    left_handle_points = [
        (-0.56, 0.26, 0.56),
        (-0.30, 0.22, 0.45),
        (0.08, 0.18, 0.25),
        (0.34, 0.15, 0.24),
        (0.54, 0.11, 0.22),
        (0.62, 0.10, 0.21),
    ]
    right_handle_points = _mirror_y(left_handle_points)
    left_fork_points = [
        (0.62, 0.10, 0.21),
        (0.75, 0.10, 0.24),
        (0.86, 0.078, 0.20),
    ]
    right_fork_points = _mirror_y(left_fork_points)
    left_leg_points = [
        (0.08, 0.18, 0.25),
        (-0.02, 0.18, 0.16),
        (-0.12, 0.18, 0.02),
    ]
    right_leg_points = _mirror_y(left_leg_points)
    left_riser_points = [
        (0.08, 0.18, 0.25),
        (-0.08, 0.19, 0.29),
        (-0.30, 0.22, 0.45),
    ]
    right_riser_points = _mirror_y(left_riser_points)

    chassis.visual(
        _tube_mesh("wheelbarrow_left_handle", left_handle_points, radius=0.018),
        material=frame_black,
        name="left_handle_rail",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_right_handle", right_handle_points, radius=0.018),
        material=frame_black,
        name="right_handle_rail",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_left_fork", left_fork_points, radius=0.017),
        material=frame_black,
        name="left_fork_leg",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_right_fork", right_fork_points, radius=0.017),
        material=frame_black,
        name="right_fork_leg",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_left_leg", left_leg_points, radius=0.018),
        material=frame_black,
        name="left_rear_leg_tube",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_right_leg", right_leg_points, radius=0.018),
        material=frame_black,
        name="right_rear_leg_tube",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_left_riser", left_riser_points, radius=0.014),
        material=frame_black,
        name="left_riser_brace",
    )
    chassis.visual(
        _tube_mesh("wheelbarrow_right_riser", right_riser_points, radius=0.014),
        material=frame_black,
        name="right_riser_brace",
    )
    chassis.visual(
        _tube_mesh(
            "wheelbarrow_mid_crossmember",
            [(0.34, -0.15, 0.24), (0.34, 0.15, 0.24)],
            radius=0.015,
            samples_per_segment=2,
        ),
        material=frame_black,
        name="mid_crossmember",
    )
    chassis.visual(
        _tube_mesh(
            "wheelbarrow_front_crossmember",
            [(0.54, -0.11, 0.22), (0.54, 0.11, 0.22)],
            radius=0.014,
            samples_per_segment=2,
        ),
        material=frame_black,
        name="front_crossmember",
    )
    chassis.visual(
        _tube_mesh(
            "wheelbarrow_rear_leg_spreader",
            [(-0.02, -0.18, 0.16), (-0.02, 0.18, 0.16)],
            radius=0.012,
            samples_per_segment=2,
        ),
        material=frame_black,
        name="rear_leg_spreader",
    )
    chassis.visual(
        Box((0.08, 0.17, 0.020)),
        origin=Origin(xyz=(0.58, 0.0, 0.228)),
        material=frame_black,
        name="fork_crown",
    )
    chassis.visual(
        Box((0.14, 0.08, 0.020)),
        origin=Origin(xyz=(0.34, 0.15, 0.255)),
        material=steel,
        name="left_mount_pad",
    )
    chassis.visual(
        Box((0.14, 0.08, 0.020)),
        origin=Origin(xyz=(0.34, -0.15, 0.255)),
        material=steel,
        name="right_mount_pad",
    )
    chassis.visual(
        Box((0.09, 0.05, 0.020)),
        origin=Origin(xyz=(-0.12, 0.18, 0.010)),
        material=steel,
        name="left_rear_foot",
    )
    chassis.visual(
        Box((0.09, 0.05, 0.020)),
        origin=Origin(xyz=(-0.12, -0.18, 0.010)),
        material=steel,
        name="right_rear_foot",
    )
    chassis.visual(
        Box((0.11, 0.028, 0.028)),
        origin=Origin(xyz=(0.84, 0.067, 0.20)),
        material=steel,
        name="left_dropout",
    )
    chassis.visual(
        Box((0.11, 0.028, 0.028)),
        origin=Origin(xyz=(0.84, -0.067, 0.20)),
        material=steel,
        name="right_dropout",
    )
    chassis.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.87, 0.086, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_axle_cap",
    )
    chassis.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.87, -0.086, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_axle_cap",
    )
    chassis.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(-0.54, 0.26, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    chassis.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(-0.54, -0.26, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.84, 0.64, 0.25)),
        mass=9.0,
        origin=Origin(xyz=(0.38, 0.0, 0.11)),
    )
    tray.visual(
        Box((0.56, 0.42, 0.014)),
        origin=Origin(xyz=(0.28, 0.0, 0.007)),
        material=tray_green,
        name="floor",
    )
    tray.visual(
        Box((0.012, 0.56, 0.18)),
        origin=Origin(xyz=(-0.006, 0.0, 0.095)),
        material=tray_green,
        name="rear_wall",
    )
    tray.visual(
        Box((0.48, 0.012, 0.20)),
        origin=Origin(xyz=(0.23, 0.227, 0.108), rpy=(0.18, 0.0, 0.0)),
        material=tray_green,
        name="left_side_main",
    )
    tray.visual(
        Box((0.48, 0.012, 0.20)),
        origin=Origin(xyz=(0.23, -0.227, 0.108), rpy=(-0.18, 0.0, 0.0)),
        material=tray_green,
        name="right_side_main",
    )
    tray.visual(
        Box((0.24, 0.012, 0.18)),
        origin=Origin(xyz=(0.59, 0.154, 0.097), rpy=(0.20, 0.0, -0.30)),
        material=tray_green,
        name="left_nose_cheek",
    )
    tray.visual(
        Box((0.24, 0.012, 0.18)),
        origin=Origin(xyz=(0.59, -0.154, 0.097), rpy=(-0.20, 0.0, 0.30)),
        material=tray_green,
        name="right_nose_cheek",
    )
    tray.visual(
        Box((0.012, 0.31, 0.12)),
        origin=Origin(xyz=(0.73, 0.0, 0.185), rpy=(0.0, -0.55, 0.0)),
        material=tray_green,
        name="nose_panel",
    )
    tray.visual(
        _tube_mesh(
            "wheelbarrow_left_rim",
            [(0.00, 0.285, 0.18), (0.22, 0.275, 0.22), (0.54, 0.205, 0.23), (0.72, 0.145, 0.18)],
            radius=0.010,
        ),
        material=tray_green,
        name="left_rim",
    )
    tray.visual(
        _tube_mesh(
            "wheelbarrow_right_rim",
            _mirror_y([(0.00, 0.285, 0.18), (0.22, 0.275, 0.22), (0.54, 0.205, 0.23), (0.72, 0.145, 0.18)]),
            radius=0.010,
        ),
        material=tray_green,
        name="right_rim",
    )
    tray.visual(
        Box((0.020, 0.58, 0.018)),
        origin=Origin(xyz=(0.002, 0.0, 0.182)),
        material=tray_green,
        name="rear_rim",
    )
    tray.visual(
        Box((0.30, 0.05, 0.016)),
        origin=Origin(xyz=(0.23, 0.105, -0.008)),
        material=steel,
        name="left_floor_runner",
    )
    tray.visual(
        Box((0.30, 0.05, 0.016)),
        origin=Origin(xyz=(0.23, -0.105, -0.008)),
        material=steel,
        name="right_floor_runner",
    )
    tray.visual(
        Box((0.18, 0.045, 0.014)),
        origin=Origin(xyz=(0.49, 0.090, -0.007)),
        material=steel,
        name="left_nose_doubler",
    )
    tray.visual(
        Box((0.18, 0.045, 0.014)),
        origin=Origin(xyz=(0.49, -0.090, -0.007)),
        material=steel,
        name="right_nose_doubler",
    )

    left_adapter = model.part("left_adapter")
    left_adapter.inertial = Inertial.from_geometry(
        Box((0.18, 0.07, 0.03)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
    )
    left_adapter.visual(
        Box((0.18, 0.07, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=zinc,
        name="left_adapter_top_plate",
    )
    left_adapter.visual(
        Box((0.070, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, -0.016)),
        material=steel,
        name="left_adapter_web",
    )
    left_adapter.visual(
        Box((0.13, 0.044, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=steel,
        name="left_adapter_clamp",
    )
    left_adapter.visual(
        Box((0.12, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.030, -0.016)),
        material=zinc,
        name="left_adapter_side_ear",
    )
    for bolt_x in (-0.040, 0.040):
        left_adapter.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(xyz=(bolt_x, 0.031, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"left_side_bolt_{'p' if bolt_x > 0 else 'm'}",
        )

    right_adapter = model.part("right_adapter")
    right_adapter.inertial = Inertial.from_geometry(
        Box((0.18, 0.07, 0.03)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
    )
    right_adapter.visual(
        Box((0.18, 0.07, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=zinc,
        name="right_adapter_top_plate",
    )
    right_adapter.visual(
        Box((0.070, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.004, -0.016)),
        material=steel,
        name="right_adapter_web",
    )
    right_adapter.visual(
        Box((0.13, 0.044, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=steel,
        name="right_adapter_clamp",
    )
    right_adapter.visual(
        Box((0.12, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.030, -0.016)),
        material=zinc,
        name="right_adapter_side_ear",
    )
    for bolt_x in (-0.040, 0.040):
        right_adapter.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(xyz=(bolt_x, -0.031, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"right_side_bolt_{'p' if bolt_x > 0 else 'm'}",
        )

    rear_hatch_left = model.part("rear_hatch_left")
    rear_hatch_left.inertial = Inertial.from_geometry(
        Box((0.022, 0.18, 0.12)),
        mass=0.25,
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
    )
    rear_hatch_left.visual(
        Box((0.006, 0.18, 0.11)),
        material=zinc,
        name="hatch_plate",
    )
    rear_hatch_left.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(-0.002, -0.074, 0.032)),
        material=steel,
        name="upper_hinge_barrel",
    )
    rear_hatch_left.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(-0.002, -0.074, -0.032)),
        material=steel,
        name="lower_hinge_barrel",
    )
    rear_hatch_left.visual(
        Box((0.012, 0.020, 0.020)),
        origin=Origin(xyz=(-0.005, 0.073, 0.0)),
        material=steel,
        name="latch_block",
    )
    for bolt_y, bolt_z in ((-0.050, -0.032), (-0.050, 0.032), (0.050, -0.032), (0.050, 0.032)):
        rear_hatch_left.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(-0.005, bolt_y, bolt_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
        )

    rear_hatch_right = model.part("rear_hatch_right")
    rear_hatch_right.inertial = Inertial.from_geometry(
        Box((0.022, 0.18, 0.12)),
        mass=0.25,
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
    )
    rear_hatch_right.visual(
        Box((0.006, 0.18, 0.11)),
        material=zinc,
        name="hatch_plate",
    )
    rear_hatch_right.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(-0.002, 0.074, 0.032)),
        material=steel,
        name="upper_hinge_barrel",
    )
    rear_hatch_right.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(-0.002, 0.074, -0.032)),
        material=steel,
        name="lower_hinge_barrel",
    )
    rear_hatch_right.visual(
        Box((0.012, 0.020, 0.020)),
        origin=Origin(xyz=(-0.005, -0.073, 0.0)),
        material=steel,
        name="latch_block",
    )
    for bolt_y, bolt_z in ((-0.050, -0.032), (-0.050, 0.032), (0.050, -0.032), (0.050, 0.032)):
        rear_hatch_right.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(-0.005, bolt_y, bolt_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
        )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.09),
        mass=4.2,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    front_wheel.visual(
        _wheel_tire_mesh("wheelbarrow_front_tire", radius=0.20, width=0.09),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.145, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="rim_band",
    )
    front_wheel.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="left_rim_disc",
    )
    front_wheel.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="right_rim_disc",
    )
    front_wheel.visual(
        Cylinder(radius=0.056, length=0.112),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_barrel",
    )
    front_wheel.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_bearing_cap",
    )
    front_wheel.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_bearing_cap",
    )
    for angle_index in range(5):
        angle = angle_index * (2.0 * math.pi / 5.0)
        ring_x = math.cos(angle) * 0.034
        ring_z = math.sin(angle) * 0.034
        front_wheel.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=(ring_x, 0.031, ring_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"left_lug_{angle_index}",
        )
        front_wheel.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=(ring_x, -0.031, ring_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"right_lug_{angle_index}",
        )

    model.articulation(
        "chassis_to_tray",
        ArticulationType.FIXED,
        parent=chassis,
        child=tray,
        origin=Origin(xyz=(0.02, 0.0, 0.302)),
    )
    model.articulation(
        "chassis_to_left_adapter",
        ArticulationType.FIXED,
        parent=chassis,
        child=left_adapter,
        origin=Origin(xyz=(0.34, 0.15, 0.290)),
    )
    model.articulation(
        "chassis_to_right_adapter",
        ArticulationType.FIXED,
        parent=chassis,
        child=right_adapter,
        origin=Origin(xyz=(0.34, -0.15, 0.290)),
    )
    model.articulation(
        "tray_to_rear_hatch_left",
        ArticulationType.FIXED,
        parent=tray,
        child=rear_hatch_left,
        origin=Origin(xyz=(-0.015, 0.155, 0.095)),
    )
    model.articulation(
        "tray_to_rear_hatch_right",
        ArticulationType.FIXED,
        parent=tray,
        child=rear_hatch_right,
        origin=Origin(xyz=(-0.015, -0.155, 0.095)),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_wheel,
        origin=Origin(xyz=(0.84, 0.0, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    tray = object_model.get_part("tray")
    left_adapter = object_model.get_part("left_adapter")
    right_adapter = object_model.get_part("right_adapter")
    rear_hatch_left = object_model.get_part("rear_hatch_left")
    rear_hatch_right = object_model.get_part("rear_hatch_right")
    front_wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("front_wheel_spin")

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
        tray,
        left_adapter,
        elem_a="left_floor_runner",
        elem_b="left_adapter_top_plate",
        name="left adapter supports tray runner",
    )
    ctx.expect_contact(
        tray,
        right_adapter,
        elem_a="right_floor_runner",
        elem_b="right_adapter_top_plate",
        name="right adapter supports tray runner",
    )
    ctx.expect_contact(
        chassis,
        left_adapter,
        elem_a="left_mount_pad",
        elem_b="left_adapter_clamp",
        name="left adapter clamps to chassis pad",
    )
    ctx.expect_contact(
        chassis,
        right_adapter,
        elem_a="right_mount_pad",
        elem_b="right_adapter_clamp",
        name="right adapter clamps to chassis pad",
    )
    ctx.expect_contact(rear_hatch_left, tray, elem_b="rear_wall", name="left service hatch mounted to rear wall")
    ctx.expect_contact(rear_hatch_right, tray, elem_b="rear_wall", name="right service hatch mounted to rear wall")
    ctx.expect_contact(front_wheel, chassis, name="front wheel supported at fork and axle")

    ctx.check(
        "front wheel uses continuous spin about axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (0.0, 1.0, 0.0),
        f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    wheel_aabb = ctx.part_world_aabb(front_wheel)
    left_foot_aabb = ctx.part_element_world_aabb(chassis, elem="left_rear_foot")
    right_foot_aabb = ctx.part_element_world_aabb(chassis, elem="right_rear_foot")
    if wheel_aabb and left_foot_aabb and right_foot_aabb:
        support_levels = [wheel_aabb[0][2], left_foot_aabb[0][2], right_foot_aabb[0][2]]
        wheel_center_x = 0.5 * (wheel_aabb[0][0] + wheel_aabb[1][0])
        rear_support_x = 0.5 * (
            (0.5 * (left_foot_aabb[0][0] + left_foot_aabb[1][0]))
            + (0.5 * (right_foot_aabb[0][0] + right_foot_aabb[1][0]))
        )
        ctx.check(
            "wheelbarrow rests on wheel and rear legs at near-common ground level",
            max(support_levels) - min(support_levels) <= 0.012,
            f"support z levels={support_levels}",
        )
        ctx.check(
            "front wheel sits well ahead of rear legs",
            wheel_center_x - rear_support_x >= 0.90,
            f"wheel_center_x={wheel_center_x:.3f}, rear_support_x={rear_support_x:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_rolling_walker")

    frame_paint = model.material("frame_paint", rgba=(0.36, 0.39, 0.42, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    grip_elastomer = model.material("grip_elastomer", rgba=(0.08, 0.09, 0.10, 1.0))
    lever_metal = model.material("lever_metal", rgba=(0.78, 0.79, 0.80, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

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

    def _add_wheel_visuals(
        part,
        *,
        tire_radius: float,
        tire_width: float,
        hub_radius: float,
        hub_width: float,
        spoke_cap_radius: float,
        tire_name: str,
    ) -> None:
        spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
        part.visual(
            Cylinder(radius=tire_radius, length=tire_width),
            origin=spin_origin,
            material=wheel_rubber,
            name=tire_name,
        )
        part.visual(
            Cylinder(radius=hub_radius, length=hub_width),
            origin=spin_origin,
            material=hub_metal,
            name="hub_shell",
        )
        part.visual(
            Cylinder(radius=spoke_cap_radius, length=0.004),
            origin=Origin(
                xyz=(hub_width * 0.5 - 0.002, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hub_metal,
            name="outer_cap",
        )
        part.visual(
            Cylinder(radius=spoke_cap_radius, length=0.004),
            origin=Origin(
                xyz=(-(hub_width * 0.5) + 0.002, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hub_metal,
            name="inner_cap",
        )

    lower_frame = model.part("lower_frame")
    lower_frame.inertial = Inertial.from_geometry(
        Box((0.66, 0.72, 0.58)),
        mass=7.8,
        origin=Origin(xyz=(0.0, -0.01, 0.29)),
    )

    lower_frame.visual(
        _tube_mesh(
            "left_lower_side_rail",
            [
                (0.24, 0.24, 0.21),
                (0.25, 0.15, 0.29),
                (0.26, 0.03, 0.40),
                (0.24, -0.04, 0.47),
            ],
            radius=0.016,
        ),
        material=frame_paint,
        name="left_lower_side_rail",
    )
    lower_frame.visual(
        _tube_mesh(
            "right_lower_side_rail",
            _mirror_x(
                [
                    (0.24, 0.24, 0.21),
                    (0.25, 0.15, 0.29),
                    (0.26, 0.03, 0.40),
                    (0.24, -0.04, 0.47),
                ]
            ),
            radius=0.016,
        ),
        material=frame_paint,
        name="right_lower_side_rail",
    )
    lower_frame.visual(
        _tube_mesh(
            "left_rear_leg",
            [
                (0.24, -0.04, 0.47),
                (0.27, -0.14, 0.32),
                (0.265, -0.25, 0.15),
            ],
            radius=0.016,
            samples_per_segment=12,
        ),
        material=frame_paint,
        name="left_rear_leg",
    )
    lower_frame.visual(
        _tube_mesh(
            "right_rear_leg",
            _mirror_x(
                [
                    (0.24, -0.04, 0.47),
                    (0.27, -0.14, 0.32),
                    (0.265, -0.25, 0.15),
                ]
            ),
            radius=0.016,
            samples_per_segment=12,
        ),
        material=frame_paint,
        name="right_rear_leg",
    )
    lower_frame.visual(
        Cylinder(radius=0.016, length=0.48),
        origin=Origin(xyz=(0.0, 0.24, 0.21), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="front_cross_tube",
    )
    lower_frame.visual(
        Cylinder(radius=0.014, length=0.54),
        origin=Origin(xyz=(0.0, -0.18, 0.258), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="rear_cross_tube",
    )
    lower_frame.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(0.24, 0.24, 0.195)),
        material=frame_paint,
        name="left_caster_sleeve",
    )
    lower_frame.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(-0.24, 0.24, 0.195)),
        material=frame_paint,
        name="right_caster_sleeve",
    )
    lower_frame.visual(
        Box((0.12, 0.05, 0.044)),
        origin=Origin(xyz=(0.18, -0.03, 0.446)),
        material=polymer_dark,
        name="left_hinge_cradle",
    )
    lower_frame.visual(
        Box((0.12, 0.05, 0.044)),
        origin=Origin(xyz=(-0.18, -0.03, 0.446)),
        material=polymer_dark,
        name="right_hinge_cradle",
    )
    lower_frame.visual(
        Box((0.055, 0.055, 0.05)),
        origin=Origin(xyz=(0.265, -0.25, 0.12)),
        material=frame_paint,
        name="left_rear_axle_boss",
    )
    lower_frame.visual(
        Box((0.055, 0.055, 0.05)),
        origin=Origin(xyz=(-0.265, -0.25, 0.12)),
        material=frame_paint,
        name="right_rear_axle_boss",
    )
    lower_frame.visual(
        Box((0.026, 0.032, 0.052)),
        origin=Origin(xyz=(0.225, -0.225, 0.165)),
        material=polymer_dark,
        name="left_brake_housing",
    )
    lower_frame.visual(
        Box((0.026, 0.032, 0.052)),
        origin=Origin(xyz=(-0.225, -0.225, 0.165)),
        material=polymer_dark,
        name="right_brake_housing",
    )
    upper_frame = model.part("upper_frame")
    upper_frame.inertial = Inertial.from_geometry(
        Box((0.50, 0.34, 0.50)),
        mass=3.3,
        origin=Origin(xyz=(0.0, -0.13, 0.25)),
    )
    upper_frame.visual(
        Cylinder(radius=0.022, length=0.40),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="hinge_barrel",
    )
    upper_frame.visual(
        _tube_mesh(
            "left_upper_upright",
            [
                (0.19, 0.0, 0.0),
                (0.20, -0.05, 0.18),
                (0.20, -0.10, 0.36),
                (0.20, -0.12, 0.43),
            ],
            radius=0.016,
        ),
        material=frame_paint,
        name="left_upright",
    )
    upper_frame.visual(
        _tube_mesh(
            "right_upper_upright",
            _mirror_x(
                [
                    (0.19, 0.0, 0.0),
                    (0.20, -0.05, 0.18),
                    (0.20, -0.10, 0.36),
                    (0.20, -0.12, 0.43),
                ]
            ),
            radius=0.016,
        ),
        material=frame_paint,
        name="right_upright",
    )
    upper_frame.visual(
        Cylinder(radius=0.016, length=0.34),
        origin=Origin(xyz=(0.0, -0.12, 0.43), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="top_bridge",
    )
    upper_frame.visual(
        Box((0.06, 0.05, 0.05)),
        origin=Origin(xyz=(0.20, -0.145, 0.44)),
        material=polymer_dark,
        name="left_handle_housing",
    )
    upper_frame.visual(
        Box((0.06, 0.05, 0.05)),
        origin=Origin(xyz=(-0.20, -0.145, 0.44)),
        material=polymer_dark,
        name="right_handle_housing",
    )
    upper_frame.visual(
        Cylinder(radius=0.017, length=0.11),
        origin=Origin(xyz=(0.20, -0.19, 0.44), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_elastomer,
        name="left_grip",
    )
    upper_frame.visual(
        Cylinder(radius=0.017, length=0.11),
        origin=Origin(xyz=(-0.20, -0.19, 0.44), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_elastomer,
        name="right_grip",
    )
    upper_frame.visual(
        Cylinder(radius=0.008, length=0.03),
        origin=Origin(xyz=(0.20, -0.125, 0.458), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer_dark,
        name="left_lever_pivot",
    )
    upper_frame.visual(
        Cylinder(radius=0.008, length=0.03),
        origin=Origin(xyz=(-0.20, -0.125, 0.458), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer_dark,
        name="right_lever_pivot",
    )
    upper_frame.visual(
        _tube_mesh(
            "left_brake_lever",
            [
                (0.20, -0.126, 0.458),
                (0.208, -0.094, 0.448),
                (0.214, -0.060, 0.431),
            ],
            radius=0.005,
            samples_per_segment=8,
            radial_segments=14,
        ),
        material=lever_metal,
        name="left_brake_lever",
    )
    upper_frame.visual(
        _tube_mesh(
            "right_brake_lever",
            _mirror_x(
                [
                    (0.20, -0.126, 0.458),
                    (0.208, -0.094, 0.448),
                    (0.214, -0.060, 0.431),
                ]
            ),
            radius=0.005,
            samples_per_segment=8,
            radial_segments=14,
        ),
        material=lever_metal,
        name="right_brake_lever",
    )

    accessory_tray = model.part("accessory_tray")
    accessory_tray.inertial = Inertial.from_geometry(
        Box((0.26, 0.18, 0.04)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )
    accessory_tray.visual(
        Box((0.24, 0.16, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=polymer_dark,
        name="tray_floor",
    )
    accessory_tray.visual(
        Box((0.24, 0.005, 0.028)),
        origin=Origin(xyz=(0.0, 0.0775, 0.014)),
        material=polymer_dark,
        name="tray_front_wall",
    )
    accessory_tray.visual(
        Box((0.24, 0.005, 0.028)),
        origin=Origin(xyz=(0.0, -0.0775, 0.014)),
        material=polymer_dark,
        name="tray_rear_wall",
    )
    accessory_tray.visual(
        Box((0.005, 0.15, 0.028)),
        origin=Origin(xyz=(0.1175, 0.0, 0.014)),
        material=polymer_dark,
        name="tray_left_wall",
    )
    accessory_tray.visual(
        Box((0.005, 0.15, 0.028)),
        origin=Origin(xyz=(-0.1175, 0.0, 0.014)),
        material=polymer_dark,
        name="tray_right_wall",
    )
    accessory_tray.visual(
        Box((0.03, 0.22, 0.014)),
        origin=Origin(xyz=(0.065, -0.09, -0.007)),
        material=polymer_dark,
        name="left_undertray_rail",
    )
    accessory_tray.visual(
        Box((0.03, 0.22, 0.014)),
        origin=Origin(xyz=(-0.065, -0.09, -0.007)),
        material=polymer_dark,
        name="right_undertray_rail",
    )
    accessory_tray.visual(
        Box((0.03, 0.03, 0.041)),
        origin=Origin(xyz=(0.065, -0.20, -0.0275)),
        material=polymer_dark,
        name="left_mount_post",
    )
    accessory_tray.visual(
        Box((0.03, 0.03, 0.041)),
        origin=Origin(xyz=(-0.065, -0.20, -0.0275)),
        material=polymer_dark,
        name="right_mount_post",
    )

    front_left_fork = model.part("front_left_fork")
    front_left_fork.inertial = Inertial.from_geometry(
        Box((0.055, 0.03, 0.14)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
    )
    front_left_fork.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=frame_paint,
        name="stem",
    )
    front_left_fork.visual(
        Box((0.036, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=frame_paint,
        name="fork_crown",
    )
    front_left_fork.visual(
        Box((0.008, 0.016, 0.078)),
        origin=Origin(xyz=(0.016, 0.0, -0.078)),
        material=frame_paint,
        name="outer_arm",
    )
    front_left_fork.visual(
        Box((0.008, 0.016, 0.078)),
        origin=Origin(xyz=(-0.016, 0.0, -0.078)),
        material=frame_paint,
        name="inner_arm",
    )

    front_right_fork = model.part("front_right_fork")
    front_right_fork.inertial = Inertial.from_geometry(
        Box((0.055, 0.03, 0.14)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
    )
    front_right_fork.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=frame_paint,
        name="stem",
    )
    front_right_fork.visual(
        Box((0.036, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=frame_paint,
        name="fork_crown",
    )
    front_right_fork.visual(
        Box((0.008, 0.016, 0.078)),
        origin=Origin(xyz=(0.016, 0.0, -0.078)),
        material=frame_paint,
        name="outer_arm",
    )
    front_right_fork.visual(
        Box((0.008, 0.016, 0.078)),
        origin=Origin(xyz=(-0.016, 0.0, -0.078)),
        material=frame_paint,
        name="inner_arm",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.03),
        mass=0.45,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        tire_radius=0.07,
        tire_width=0.024,
        hub_radius=0.03,
        hub_width=0.032,
        spoke_cap_radius=0.02,
        tire_name="tire",
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.03),
        mass=0.45,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        tire_radius=0.07,
        tire_width=0.024,
        hub_radius=0.03,
        hub_width=0.032,
        spoke_cap_radius=0.02,
        tire_name="tire",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.045),
        mass=0.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_left_wheel,
        tire_radius=0.10,
        tire_width=0.035,
        hub_radius=0.05,
        hub_width=0.045,
        spoke_cap_radius=0.032,
        tire_name="tire",
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.045),
        mass=0.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_right_wheel,
        tire_radius=0.10,
        tire_width=0.035,
        hub_radius=0.05,
        hub_width=0.045,
        spoke_cap_radius=0.032,
        tire_name="tire",
    )

    frame_fold = model.articulation(
        "frame_fold",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=upper_frame,
        origin=Origin(xyz=(0.0, -0.02, 0.49)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "tray_mount",
        ArticulationType.FIXED,
        parent=lower_frame,
        child=accessory_tray,
        origin=Origin(xyz=(0.0, 0.02, 0.32)),
    )
    front_left_caster_swivel = model.articulation(
        "front_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=front_left_fork,
        origin=Origin(xyz=(0.24, 0.24, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    front_right_caster_swivel = model.articulation(
        "front_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=front_right_fork,
        origin=Origin(xyz=(-0.24, 0.24, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_left_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_right_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.315, -0.25, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=22.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.315, -0.25, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=22.0),
    )

    # Silence unused locals from articulation creation while keeping readable setup.
    _ = (frame_fold, front_left_caster_swivel, front_right_caster_swivel)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    upper_frame = object_model.get_part("upper_frame")
    accessory_tray = object_model.get_part("accessory_tray")
    front_left_fork = object_model.get_part("front_left_fork")
    front_right_fork = object_model.get_part("front_right_fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    frame_fold = object_model.get_articulation("frame_fold")
    front_left_caster_swivel = object_model.get_articulation("front_left_caster_swivel")
    front_right_caster_swivel = object_model.get_articulation("front_right_caster_swivel")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(upper_frame, lower_frame, name="folded_handle_frame_is_supported")
    ctx.expect_contact(accessory_tray, lower_frame, name="tray_is_supported_by_frame")
    ctx.expect_contact(front_left_fork, lower_frame, name="left_caster_mount_is_supported")
    ctx.expect_contact(front_right_fork, lower_frame, name="right_caster_mount_is_supported")
    ctx.expect_contact(front_left_wheel, front_left_fork, name="left_front_wheel_supported")
    ctx.expect_contact(front_right_wheel, front_right_fork, name="right_front_wheel_supported")
    ctx.expect_contact(rear_left_wheel, lower_frame, name="left_rear_wheel_supported")
    ctx.expect_contact(rear_right_wheel, lower_frame, name="right_rear_wheel_supported")

    ctx.check(
        "fold_axis_points_across_walker",
        tuple(frame_fold.axis) == (-1.0, 0.0, 0.0),
        f"expected fold axis (-1, 0, 0), got {frame_fold.axis}",
    )
    ctx.check(
        "caster_swivels_are_vertical",
        tuple(front_left_caster_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(front_right_caster_swivel.axis) == (0.0, 0.0, 1.0),
        (
            "expected both front caster swivel axes to be vertical, got "
            f"{front_left_caster_swivel.axis} and {front_right_caster_swivel.axis}"
        ),
    )
    ctx.check(
        "all_wheel_spins_are_lateral",
        all(
            tuple(joint.axis) == (1.0, 0.0, 0.0)
            for joint in (
                front_left_wheel_spin,
                front_right_wheel_spin,
                rear_left_wheel_spin,
                rear_right_wheel_spin,
            )
        ),
        "every wheel spin joint should use the lateral x-axis",
    )

    def _elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    open_bridge_center = _elem_center(upper_frame, "top_bridge")
    with ctx.pose({frame_fold: 1.10}):
        folded_bridge_center = _elem_center(upper_frame, "top_bridge")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")

    frame_motion_ok = (
        open_bridge_center is not None
        and folded_bridge_center is not None
        and folded_bridge_center[1] > open_bridge_center[1] + 0.25
        and folded_bridge_center[2] < open_bridge_center[2] - 0.12
    )
    ctx.check(
        "frame_folds_forward_and_down",
        frame_motion_ok,
        (
            "top bridge should move forward and downward in the folded pose; "
            f"open={open_bridge_center}, folded={folded_bridge_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

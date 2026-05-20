from __future__ import annotations

from math import pi

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_steel = model.material("tray_steel", rgba=(0.74, 0.10, 0.08, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    rubber_grip = model.material("rubber_grip", rgba=(0.08, 0.08, 0.08, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.73, 0.76, 0.80, 1.0))
    zinc_plated = model.material("zinc_plated", rgba=(0.64, 0.67, 0.70, 1.0))

    upper_body = model.part("upper_body_module")
    upper_body.inertial = Inertial.from_geometry(
        Box((0.78, 1.60, 0.70)),
        mass=26.0,
        origin=Origin(xyz=(0.0, -0.10, 0.38)),
    )

    upper_body.visual(
        Box((0.58, 0.72, 0.018)),
        origin=Origin(xyz=(0.0, -0.03, 0.41), rpy=(-0.11, 0.0, 0.0)),
        material=tray_steel,
        name="tray_floor",
    )
    upper_body.visual(
        Box((0.020, 0.72, 0.22)),
        origin=Origin(xyz=(0.29, -0.01, 0.52), rpy=(-0.11, 0.21, 0.0)),
        material=tray_steel,
        name="tray_left_wall",
    )
    upper_body.visual(
        Box((0.020, 0.72, 0.22)),
        origin=Origin(xyz=(-0.29, -0.01, 0.52), rpy=(-0.11, -0.21, 0.0)),
        material=tray_steel,
        name="tray_right_wall",
    )
    upper_body.visual(
        Box((0.54, 0.020, 0.24)),
        origin=Origin(xyz=(0.0, 0.34, 0.50), rpy=(-0.52, 0.0, 0.0)),
        material=tray_steel,
        name="tray_front_wall",
    )
    upper_body.visual(
        Box((0.48, 0.020, 0.14)),
        origin=Origin(xyz=(0.0, -0.37, 0.50), rpy=(0.08, 0.0, 0.0)),
        material=tray_steel,
        name="tray_back_wall",
    )

    left_handle = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.26, -0.81, 0.63),
                (0.27, -0.60, 0.60),
                (0.29, -0.36, 0.55),
                (0.30, -0.05, 0.49),
                (0.26, 0.22, 0.42),
                (0.14, 0.43, 0.32),
            ],
            radius=0.021,
            samples_per_segment=14,
            radial_segments=18,
        ),
        "upper_body_left_handle",
    )
    right_handle = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.26, -0.81, 0.63),
                (-0.27, -0.60, 0.60),
                (-0.29, -0.36, 0.55),
                (-0.30, -0.05, 0.49),
                (-0.26, 0.22, 0.42),
                (-0.14, 0.43, 0.32),
            ],
            radius=0.021,
            samples_per_segment=14,
            radial_segments=18,
        ),
        "upper_body_right_handle",
    )
    upper_body.visual(left_handle, material=frame_steel, name="left_handle")
    upper_body.visual(right_handle, material=frame_steel, name="right_handle")

    upper_body.visual(
        Box((0.60, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.46, 0.57)),
        material=frame_steel,
        name="rear_cross_brace",
    )
    upper_body.visual(
        Box((0.36, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.18, 0.45), rpy=(-0.10, 0.0, 0.0)),
        material=frame_steel,
        name="undertray_cross_brace",
    )
    upper_body.visual(
        Box((0.30, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.43, 0.32)),
        material=frame_steel,
        name="front_mount_block",
    )

    left_leg = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.21, -0.42, 0.54),
                (0.20, -0.44, 0.36),
                (0.18, -0.47, 0.14),
                (0.16, -0.49, 0.02),
            ],
            radius=0.018,
            samples_per_segment=12,
            radial_segments=16,
        ),
        "upper_body_left_leg",
    )
    right_leg = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.21, -0.42, 0.54),
                (-0.20, -0.44, 0.36),
                (-0.18, -0.47, 0.14),
                (-0.16, -0.49, 0.02),
            ],
            radius=0.018,
            samples_per_segment=12,
            radial_segments=16,
        ),
        "upper_body_right_leg",
    )
    upper_body.visual(left_leg, material=frame_steel, name="left_leg")
    upper_body.visual(right_leg, material=frame_steel, name="right_leg")
    upper_body.visual(
        Box((0.05, 0.07, 0.06)),
        origin=Origin(xyz=(0.23, -0.43, 0.54)),
        material=frame_steel,
        name="left_leg_clamp",
    )
    upper_body.visual(
        Box((0.05, 0.07, 0.06)),
        origin=Origin(xyz=(-0.23, -0.43, 0.54)),
        material=frame_steel,
        name="right_leg_clamp",
    )
    upper_body.visual(
        Box((0.08, 0.028, 0.018)),
        origin=Origin(xyz=(0.16, -0.49, 0.009)),
        material=frame_steel,
        name="left_foot",
    )
    upper_body.visual(
        Box((0.08, 0.028, 0.018)),
        origin=Origin(xyz=(-0.16, -0.49, 0.009)),
        material=frame_steel,
        name="right_foot",
    )
    upper_body.visual(
        Box((0.42, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, -0.47, 0.12)),
        material=frame_steel,
        name="leg_spreader",
    )

    upper_body.visual(
        Box((0.12, 0.040, 0.030)),
        origin=Origin(xyz=(0.26, -0.83, 0.63)),
        material=rubber_grip,
        name="left_grip",
    )
    upper_body.visual(
        Box((0.12, 0.040, 0.030)),
        origin=Origin(xyz=(-0.26, -0.83, 0.63)),
        material=rubber_grip,
        name="right_grip",
    )

    fork_module = model.part("fork_module")
    fork_module.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.24)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.12, -0.06)),
    )
    fork_module.visual(
        Box((0.22, 0.05, 0.09)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=frame_steel,
        name="fork_mount_plate",
    )
    fork_module.visual(
        Box((0.12, 0.09, 0.07)),
        origin=Origin(xyz=(0.0, 0.085, -0.035)),
        material=frame_steel,
        name="fork_neck_block",
    )
    fork_module.visual(
        Box((0.24, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.12, -0.02)),
        material=frame_steel,
        name="fork_crown",
    )

    left_fork_arm = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.10, 0.09, -0.02),
                (0.106, 0.16, -0.05),
                (0.109, 0.24, -0.09),
                (0.110, 0.34, -0.12),
            ],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        ),
        "fork_left_arm",
    )
    right_fork_arm = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.10, 0.09, -0.02),
                (-0.106, 0.16, -0.05),
                (-0.109, 0.24, -0.09),
                (-0.110, 0.34, -0.12),
            ],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        ),
        "fork_right_arm",
    )
    fork_module.visual(left_fork_arm, material=frame_steel, name="left_fork_arm")
    fork_module.visual(right_fork_arm, material=frame_steel, name="right_fork_arm")
    fork_module.visual(
        Box((0.04, 0.03, 0.05)),
        origin=Origin(xyz=(0.11, 0.34, -0.12)),
        material=frame_steel,
        name="left_dropout",
    )
    fork_module.visual(
        Box((0.04, 0.03, 0.05)),
        origin=Origin(xyz=(-0.11, 0.34, -0.12)),
        material=frame_steel,
        name="right_dropout",
    )

    axle_module = model.part("axle_module")
    axle_module.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.18),
        mass=0.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    axle_module.visual(
        Cylinder(radius=0.0075, length=0.18),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc_plated,
        name="axle_shaft",
    )
    axle_module.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc_plated,
        name="left_spacer",
    )
    axle_module.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc_plated,
        name="right_spacer",
    )
    axle_module.visual(
        Box((0.016, 0.030, 0.030)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=zinc_plated,
        name="left_nut",
    )
    axle_module.visual(
        Box((0.016, 0.030, 0.030)),
        origin=Origin(xyz=(-0.085, 0.0, 0.0)),
        material=zinc_plated,
        name="right_nut",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.09),
        mass=4.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    tire_profile = [
        (0.096, -0.040),
        (0.152, -0.044),
        (0.182, -0.032),
        (0.190, -0.010),
        (0.190, 0.010),
        (0.182, 0.032),
        (0.152, 0.044),
        (0.096, 0.040),
        (0.090, 0.016),
        (0.090, -0.016),
        (0.096, -0.040),
    ]
    rim_profile = [
        (0.018, -0.028),
        (0.060, -0.028),
        (0.094, -0.016),
        (0.098, -0.004),
        (0.098, 0.004),
        (0.094, 0.016),
        (0.060, 0.028),
        (0.018, 0.028),
        (0.018, 0.014),
        (0.050, 0.014),
        (0.074, 0.0),
        (0.050, -0.014),
        (0.018, -0.014),
        (0.018, -0.028),
    ]
    hub_outer_profile = [
        (0.018, -0.060),
        (0.026, -0.060),
        (0.034, -0.048),
        (0.040, -0.024),
        (0.040, 0.024),
        (0.034, 0.048),
        (0.026, 0.060),
        (0.018, 0.060),
    ]
    hub_inner_profile = [
        (0.014, -0.060),
        (0.014, 0.060),
    ]
    front_wheel.visual(
        mesh_from_geometry(LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0), "wheelbarrow_tire"),
        material=wheel_rubber,
        name="tire",
    )
    front_wheel.visual(
        mesh_from_geometry(LatheGeometry(rim_profile, segments=56).rotate_y(pi / 2.0), "wheelbarrow_rim"),
        material=wheel_metal,
        name="rim",
    )
    front_wheel.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                hub_outer_profile,
                hub_inner_profile,
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ).rotate_y(pi / 2.0),
            "wheelbarrow_hub",
        ),
        material=zinc_plated,
        name="hub_shell",
    )

    model.articulation(
        "upper_body_to_fork",
        ArticulationType.FIXED,
        parent=upper_body,
        child=fork_module,
        origin=Origin(xyz=(0.0, 0.46, 0.32)),
    )
    model.articulation(
        "fork_to_axle",
        ArticulationType.FIXED,
        parent=fork_module,
        child=axle_module,
        origin=Origin(xyz=(0.0, 0.34, -0.12)),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_module,
        child=front_wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
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

    upper_body = object_model.get_part("upper_body_module")
    fork_module = object_model.get_part("fork_module")
    axle_module = object_model.get_part("axle_module")
    front_wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("front_wheel_spin")

    ctx.check(
        "front wheel uses a continuous spin joint",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_spin.articulation_type}",
    )
    ctx.check(
        "front wheel spins about the axle axis",
        tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )
    ctx.expect_gap(
        fork_module,
        upper_body,
        axis="y",
        positive_elem="fork_mount_plate",
        negative_elem="front_mount_block",
        max_gap=0.001,
        max_penetration=0.0,
        name="fork module seats against the front mount block",
    )
    ctx.expect_contact(
        front_wheel,
        axle_module,
        elem_a="hub_shell",
        elem_b="left_spacer",
        name="left spacer seats against the wheel hub",
    )
    ctx.expect_contact(
        front_wheel,
        axle_module,
        elem_a="hub_shell",
        elem_b="right_spacer",
        name="right spacer seats against the wheel hub",
    )
    ctx.expect_gap(
        front_wheel,
        upper_body,
        axis="y",
        min_gap=0.14,
        name="front wheel stays forward of the tray body",
    )

    rest_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({wheel_spin: pi / 2.0}):
        spun_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "spinning the wheel keeps its axle location fixed",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

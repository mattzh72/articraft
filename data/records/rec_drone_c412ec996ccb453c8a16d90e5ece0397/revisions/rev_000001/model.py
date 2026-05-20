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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _regular_polygon_profile(radius: float, sides: int, *, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + (2.0 * math.pi * i) / sides),
            radius * math.sin(phase + (2.0 * math.pi * i) / sides),
        )
        for i in range(sides)
    ]


def _xy_from_local(angle: float, local_x: float, local_y: float = 0.0) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * local_x - s * local_y, s * local_x + c * local_y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_lift_octorotor")

    frame_gray = model.material("frame_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    carbon_black = model.material("carbon_black", rgba=(0.10, 0.11, 0.12, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    landing_gray = model.material("landing_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    prop_black = model.material("prop_black", rgba=(0.07, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.58, 0.08, 0.08, 1.0))

    outer_octagon = _regular_polygon_profile(0.285, 8, phase=math.pi / 8.0)
    core_octagon = _regular_polygon_profile(0.195, 8, phase=math.pi / 8.0)
    plate_mesh = _save_mesh("center_plate", ExtrudeGeometry(outer_octagon, 0.008))
    core_mesh = _save_mesh("center_core", ExtrudeGeometry(core_octagon, 0.046))
    arm_mesh = _save_mesh(
        "arm_tube",
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (0.40, 0.0, 0.0)],
            radius=0.022,
            samples_per_segment=4,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    blade_mesh = _save_mesh(
        "prop_blade",
        ExtrudeGeometry(
            [
                (0.0, -0.018),
                (0.028, -0.028),
                (0.090, -0.031),
                (0.155, -0.026),
                (0.202, -0.015),
                (0.202, 0.015),
                (0.155, 0.026),
                (0.090, 0.031),
                (0.028, 0.028),
                (0.0, 0.018),
            ],
            0.004,
        ),
    )
    leg_strut_mesh = _save_mesh(
        "landing_leg_strut",
        tube_from_spline_points(
            [
                (0.030, 0.0, -0.036),
                (0.085, 0.0, -0.155),
                (0.165, 0.0, -0.320),
                (0.250, 0.0, -0.468),
            ],
            radius=0.016,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    airframe = model.part("airframe")
    airframe.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=frame_gray,
        name="lower_plate",
    )
    airframe.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=frame_gray,
        name="upper_plate",
    )
    airframe.visual(
        core_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=frame_gray,
        name="core_housing",
    )
    airframe.visual(
        Box((0.24, 0.16, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=carbon_black,
        name="battery_deck",
    )
    airframe.visual(
        Box((0.28, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=accent_red,
        name="retention_rail_x",
    )
    airframe.visual(
        Box((0.070, 0.28, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=accent_red,
        name="retention_rail_y",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((1.30, 1.30, 0.18)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    arm_angles = [i * (math.pi / 4.0) for i in range(8)]
    arm_root_radius = 0.175
    motor_radius = arm_root_radius + 0.40

    for index, angle in enumerate(arm_angles):
        arm_root_x, arm_root_y = _xy_from_local(angle, arm_root_radius)
        root_clamp_x, root_clamp_y = _xy_from_local(angle, 0.205)
        tip_clamp_x, tip_clamp_y = _xy_from_local(angle, 0.540)
        motor_x, motor_y = _xy_from_local(angle, motor_radius)

        airframe.visual(
            arm_mesh,
            origin=Origin(xyz=(arm_root_x, arm_root_y, 0.054), rpy=(0.0, 0.0, angle)),
            material=carbon_black,
            name=f"arm_{index:02d}",
        )
        airframe.visual(
            Box((0.090, 0.070, 0.028)),
            origin=Origin(xyz=(root_clamp_x, root_clamp_y, 0.053), rpy=(0.0, 0.0, angle)),
            material=frame_gray,
            name=f"arm_root_clamp_{index:02d}",
        )
        airframe.visual(
            Box((0.085, 0.054, 0.026)),
            origin=Origin(xyz=(tip_clamp_x, tip_clamp_y, 0.058), rpy=(0.0, 0.0, angle)),
            material=frame_gray,
            name=f"arm_tip_clamp_{index:02d}",
        )
        airframe.visual(
            Cylinder(radius=0.043, length=0.070),
            origin=Origin(xyz=(motor_x, motor_y, 0.089)),
            material=motor_gray,
            name=f"motor_pod_{index:02d}",
        )
        airframe.visual(
            Cylinder(radius=0.054, length=0.010),
            origin=Origin(xyz=(motor_x, motor_y, 0.127)),
            material=motor_gray,
            name=f"motor_cap_{index:02d}",
        )

    leg_angles = [math.pi / 8.0 + i * (math.pi / 2.0) for i in range(4)]
    hinge_radius = 0.225
    for index, angle in enumerate(leg_angles):
        mount_x, mount_y = _xy_from_local(angle, hinge_radius)
        brace_x, brace_y = _xy_from_local(angle, hinge_radius - 0.040)

        airframe.visual(
            Box((0.095, 0.055, 0.030)),
            origin=Origin(xyz=(mount_x, mount_y, 0.005), rpy=(0.0, 0.0, angle)),
            material=landing_gray,
            name=f"gear_mount_{index:02d}",
        )
        airframe.visual(
            Box((0.055, 0.038, 0.024)),
            origin=Origin(xyz=(brace_x, brace_y, 0.010), rpy=(0.0, 0.0, angle)),
            material=landing_gray,
            name=f"gear_brace_{index:02d}",
        )

    for index, angle in enumerate(arm_angles):
        motor_x, motor_y = _xy_from_local(angle, motor_radius)

        propeller = model.part(f"propeller_{index:02d}")
        propeller.visual(
            Cylinder(radius=0.028, length=0.016),
            material=motor_gray,
            name="hub",
        )
        propeller.visual(
            blade_mesh,
            material=prop_black,
            name="blade_a",
        )
        propeller.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi)),
            material=prop_black,
            name="blade_b",
        )
        propeller.inertial = Inertial.from_geometry(Cylinder(radius=0.205, length=0.016), mass=0.12)

        model.articulation(
            f"motor_spin_{index:02d}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=propeller,
            origin=Origin(xyz=(motor_x, motor_y, 0.140)),
            axis=(0.0, 0.0, 1.0 if index % 2 == 0 else -1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=120.0),
        )

    for index, angle in enumerate(leg_angles):
        hinge_x, hinge_y = _xy_from_local(angle, hinge_radius)

        leg = model.part(f"landing_leg_{index:02d}")
        leg.visual(
            Cylinder(radius=0.010, length=0.056),
            origin=Origin(xyz=(0.0, 0.0, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=landing_gray,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.050, 0.036, 0.020)),
            origin=Origin(xyz=(0.022, 0.0, -0.020)),
            material=landing_gray,
            name="hinge_knuckle",
        )
        leg.visual(
            leg_strut_mesh,
            material=landing_gray,
            name="main_strut",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.180),
            origin=Origin(xyz=(0.270, 0.0, -0.485), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=landing_gray,
            name="foot",
        )
        leg.visual(
            Box((0.042, 0.042, 0.034)),
            origin=Origin(xyz=(0.252, 0.0, -0.476)),
            material=landing_gray,
            name="foot_mount",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.30, 0.16, 0.46)),
            mass=0.45,
            origin=Origin(xyz=(0.14, 0.0, -0.22)),
        )

        model.articulation(
            f"gear_fold_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=leg,
            origin=Origin(xyz=(hinge_x, hinge_y, 0.0), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=1.2,
                lower=0.0,
                upper=1.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propellers = [object_model.get_part(f"propeller_{index:02d}") for index in range(8)]
    legs = [object_model.get_part(f"landing_leg_{index:02d}") for index in range(4)]
    prop_joints = [object_model.get_articulation(f"motor_spin_{index:02d}") for index in range(8)]
    leg_joints = [object_model.get_articulation(f"gear_fold_{index:02d}") for index in range(4)]

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

    for index, propeller in enumerate(propellers):
        joint = prop_joints[index]
        ctx.check(
            f"propeller_{index:02d} uses a continuous vertical axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and abs(joint.axis[0]) < 1e-9
            and abs(joint.axis[1]) < 1e-9
            and abs(abs(joint.axis[2]) - 1.0) < 1e-9,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_contact(
            propeller,
            airframe,
            elem_a="hub",
            elem_b=f"motor_cap_{index:02d}",
            name=f"propeller_{index:02d} hub seats on motor cap",
        )
        ctx.expect_overlap(
            propeller,
            airframe,
            axes="xy",
            elem_a="hub",
            elem_b=f"motor_cap_{index:02d}",
            min_overlap=0.040,
            name=f"propeller_{index:02d} hub stays centered over motor pod",
        )

    for index, leg in enumerate(legs):
        joint = leg_joints[index]
        ctx.check(
            f"landing_leg_{index:02d} folds on a radial hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.8
            and joint.axis == (0.0, -1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )
        ctx.expect_contact(
            leg,
            airframe,
            elem_a="hinge_knuckle",
            elem_b=f"gear_mount_{index:02d}",
            name=f"landing_leg_{index:02d} hinge cheek seats on center mount",
        )
        ctx.expect_contact(
            leg,
            airframe,
            elem_a="hinge_barrel",
            elem_b=f"gear_mount_{index:02d}",
            name=f"landing_leg_{index:02d} hinge barrel aligns under the mount block",
        )
        ctx.expect_gap(
            airframe,
            leg,
            axis="z",
            positive_elem="lower_plate",
            negative_elem="foot",
            min_gap=0.32,
            name=f"landing_leg_{index:02d} foot hangs well below the center plate",
        )

        deployed_aabb = ctx.part_element_world_aabb(leg, elem="foot")
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper}):
            folded_aabb = ctx.part_element_world_aabb(leg, elem="foot")
        ctx.check(
            f"landing_leg_{index:02d} folds upward for transport",
            deployed_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][2] > deployed_aabb[1][2] + 0.25,
            details=f"deployed={deployed_aabb}, folded={folded_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _regular_polygon_profile(
    sides: int,
    *,
    radius: float,
    rotation: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(rotation + (2.0 * math.pi * index) / sides),
            radius * math.sin(rotation + (2.0 * math.pi * index) / sides),
        )
        for index in range(sides)
    ]


def _build_propeller_mesh(name: str):
    blade_profile = rounded_rect_profile(
        width=0.030,
        height=0.0026,
        radius=0.0007,
        corner_segments=4,
    )
    blade_path = [
        (0.014, 0.0, 0.0000),
        (0.060, 0.010, 0.0018),
        (0.118, 0.015, 0.0036),
        (0.168, 0.004, 0.0020),
    ]

    blade_a = sweep_profile_along_spline(
        blade_path,
        profile=blade_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    blade_b = sweep_profile_along_spline(
        blade_path,
        profile=blade_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    ).rotate_z(math.pi)

    hub = CylinderGeometry(radius=0.017, height=0.004)
    hub.merge(blade_a)
    hub.merge(blade_b)
    return mesh_from_geometry(hub, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="octocopter")

    carbon = model.material("carbon", rgba=(0.11, 0.12, 0.13, 1.0))
    prop_black = model.material("prop_black", rgba=(0.15, 0.15, 0.16, 1.0))
    motor_metal = model.material("motor_metal", rgba=(0.72, 0.73, 0.75, 1.0))
    battery_gray = model.material("battery_gray", rgba=(0.34, 0.35, 0.38, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.92, 0.40, 0.08, 1.0))
    electronics_gray = model.material("electronics_gray", rgba=(0.28, 0.29, 0.31, 1.0))

    plate_radius = 0.141
    plate_thickness = 0.008
    arm_length = 0.400
    arm_width = 0.032
    arm_height = 0.018
    arm_center_z = 0.006
    motor_radius_from_center = 0.530
    mount_center_z = 0.014
    mount_thickness = 0.006

    octagon_profile = _regular_polygon_profile(
        8,
        radius=plate_radius,
        rotation=math.pi / 8.0,
    )
    center_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(octagon_profile, plate_thickness),
        "center_plate",
    )
    propeller_mesh = _build_propeller_mesh("octocopter_propeller")

    airframe = model.part("airframe")
    airframe.visual(
        center_plate_mesh,
        material=carbon,
        name="center_plate",
    )
    airframe.visual(
        Box((0.148, 0.074, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=battery_gray,
        name="battery_pack",
    )
    airframe.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=electronics_gray,
        name="electronics_stack",
    )
    airframe.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=electronics_gray,
        name="gps_mast",
    )
    airframe.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=accent_orange,
        name="gps_puck",
    )

    arm_start = plate_radius * math.cos(math.pi / 8.0)
    arm_center_radius = arm_start + arm_length * 0.5

    for index in range(8):
        angle = index * (math.pi / 4.0)
        airframe.visual(
            Box((arm_length, arm_width, arm_height)),
            origin=Origin(
                xyz=(arm_center_radius * math.cos(angle), arm_center_radius * math.sin(angle), arm_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=carbon,
            name=f"arm_{index}",
        )
        airframe.visual(
            Cylinder(radius=0.026, length=mount_thickness),
            origin=Origin(
                xyz=(
                    motor_radius_from_center * math.cos(angle),
                    motor_radius_from_center * math.sin(angle),
                    mount_center_z,
                ),
            ),
            material=motor_metal,
            name=f"motor_mount_{index}",
        )

    for index, angle in enumerate((0.0, math.pi / 4.0)):
        stripe_radius = arm_start + 0.72 * arm_length
        airframe.visual(
            Box((0.062, arm_width + 0.001, 0.0012)),
            origin=Origin(
                xyz=(stripe_radius * math.cos(angle), stripe_radius * math.sin(angle), arm_center_z + arm_height * 0.5 + 0.0006),
                rpy=(0.0, 0.0, angle),
            ),
            material=accent_orange,
            name=f"orientation_stripe_{index}",
        )

    airframe.inertial = Inertial.from_geometry(
        Box((1.18, 1.18, 0.10)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    for index in range(8):
        rotor = model.part(f"rotor_{index}")
        rotor.visual(
            Cylinder(radius=0.021, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=motor_metal,
            name="motor_bell",
        )
        rotor.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.031)),
            material=motor_metal,
            name="motor_cap",
        )
        rotor.visual(
            Cylinder(radius=0.0045, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0355)),
            material=motor_metal,
            name="prop_shaft",
        )
        rotor.visual(
            propeller_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.036)),
            material=prop_black,
            name="propeller",
        )
        rotor.inertial = Inertial.from_geometry(
            Cylinder(radius=0.022, length=0.042),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
        )

        angle = index * (math.pi / 4.0)
        model.articulation(
            f"airframe_to_rotor_{index}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=rotor,
            origin=Origin(
                xyz=(
                    motor_radius_from_center * math.cos(angle),
                    motor_radius_from_center * math.sin(angle),
                    mount_center_z + mount_thickness * 0.5,
                )
            ),
            axis=(0.0, 0.0, 1.0 if index % 2 == 0 else -1.0),
            motion_limits=MotionLimits(effort=1.4, velocity=220.0),
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

    airframe = object_model.get_part("airframe")

    rotor_positions: list[tuple[float, float, float]] = []
    rotor_angles: list[float] = []
    rotor_radii: list[float] = []

    for index in range(8):
        rotor = object_model.get_part(f"rotor_{index}")
        joint = object_model.get_articulation(f"airframe_to_rotor_{index}")

        axis = joint.axis
        joint_limits = joint.motion_limits
        axis_is_vertical = (
            axis is not None
            and len(axis) == 3
            and math.isclose(axis[0], 0.0, abs_tol=1e-9)
            and math.isclose(axis[1], 0.0, abs_tol=1e-9)
            and math.isclose(abs(axis[2]), 1.0, abs_tol=1e-9)
        )
        ctx.check(
            f"rotor_{index} uses a vertical continuous spin axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and axis_is_vertical
            and joint_limits is not None
            and joint_limits.lower is None
            and joint_limits.upper is None,
            details=f"type={joint.articulation_type}, axis={axis}, limits={joint_limits}",
        )
        ctx.expect_contact(
            rotor,
            airframe,
            elem_a="motor_bell",
            elem_b=f"motor_mount_{index}",
            name=f"rotor_{index} motor bell seats on its arm mount",
        )

        pos = ctx.part_world_position(rotor)
        if pos is not None:
            rotor_positions.append(pos)
            rotor_radii.append(math.hypot(pos[0], pos[1]))
            rotor_angles.append((math.atan2(pos[1], pos[0]) + 2.0 * math.pi) % (2.0 * math.pi))

    ctx.check(
        "all eight rotor origins resolve",
        len(rotor_positions) == 8,
        details=f"resolved_positions={rotor_positions}",
    )

    if len(rotor_positions) == 8:
        mean_radius = sum(rotor_radii) / len(rotor_radii)
        max_radius_error = max(abs(radius - mean_radius) for radius in rotor_radii)
        max_height_error = max(abs(pos[2] - rotor_positions[0][2]) for pos in rotor_positions)

        rotor_angles.sort()
        angle_steps = [
            rotor_angles[(index + 1) % 8] - rotor_angles[index]
            if index < 7
            else rotor_angles[0] + 2.0 * math.pi - rotor_angles[index]
            for index in range(8)
        ]
        target_step = math.pi / 4.0
        max_step_error = max(abs(step - target_step) for step in angle_steps)

        ctx.check(
            "rotors are equally spaced on a common radius",
            max_radius_error <= 1e-6 and max_height_error <= 1e-6 and max_step_error <= 1e-6,
            details=(
                f"radii={rotor_radii}, mean_radius={mean_radius}, "
                f"max_radius_error={max_radius_error}, heights={rotor_positions}, "
                f"max_height_error={max_height_error}, angle_steps={angle_steps}, "
                f"max_step_error={max_step_error}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

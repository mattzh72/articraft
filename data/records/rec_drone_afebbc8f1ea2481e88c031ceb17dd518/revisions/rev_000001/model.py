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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


ARM_THICKNESS = 0.006
ARM_TOP_Z = ARM_THICKNESS
MOTOR_MOUNT_Z = ARM_TOP_Z
MOTOR_HEIGHT = 0.022
PROP_RADIUS = 0.062


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _add_arm(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    width: float,
    thickness: float,
    overlap: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy) + overlap
    yaw = math.atan2(dy, dx)
    center = _midpoint(start, end)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(xyz=center, rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _blade_section(
    x: float,
    *,
    chord: float,
    thickness: float,
    pitch_deg: float,
    z_offset: float,
) -> list[tuple[float, float, float]]:
    pitch = math.radians(pitch_deg)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    profile = rounded_rect_profile(
        chord,
        thickness,
        radius=min(thickness * 0.45, chord * 0.16),
        corner_segments=4,
    )
    return [
        (
            x,
            y * cos_pitch - z * sin_pitch,
            y * sin_pitch + z * cos_pitch + z_offset,
        )
        for y, z in profile
    ]


def _propeller_mesh(name: str):
    hub = CylinderGeometry(radius=0.0065, height=0.003).translate(0.0, 0.0, 0.0015)
    hub.merge(CylinderGeometry(radius=0.0035, height=0.004).translate(0.0, 0.0, 0.004))

    blade = section_loft(
        [
            _blade_section(0.0045, chord=0.016, thickness=0.0025, pitch_deg=17.0, z_offset=0.0021),
            _blade_section(0.020, chord=0.015, thickness=0.0022, pitch_deg=14.0, z_offset=0.0022),
            _blade_section(0.040, chord=0.012, thickness=0.0017, pitch_deg=10.0, z_offset=0.0020),
            _blade_section(0.058, chord=0.008, thickness=0.0012, pitch_deg=6.0, z_offset=0.0018),
            _blade_section(PROP_RADIUS, chord=0.0038, thickness=0.0008, pitch_deg=4.0, z_offset=0.0016),
        ]
    )
    hub.merge(blade)
    hub.merge(blade.copy().rotate_z((2.0 * math.pi) / 3.0))
    hub.merge(blade.copy().rotate_z((4.0 * math.pi) / 3.0))
    return mesh_from_geometry(hub, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_h_frame_quadcopter")

    carbon = model.material("carbon", rgba=(0.08, 0.08, 0.09, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.34, 0.35, 0.37, 1.0))
    prop_black = model.material("prop_black", rgba=(0.05, 0.05, 0.06, 1.0))
    nylon = model.material("nylon", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.120, 0.075, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=carbon,
        name="lower_plate",
    )
    frame.visual(
        Box((0.084, 0.050, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=carbon,
        name="upper_plate",
    )
    frame.visual(
        Box((0.036, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=nylon,
        name="electronics_stack",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            frame.visual(
                Cylinder(radius=0.003, length=0.020),
                origin=Origin(xyz=(0.026 * x_sign, 0.017 * y_sign, 0.012)),
                material=dark_aluminum,
                name=f"standoff_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )

    motor_mounts = {
        "front_left": (0.116, 0.090, MOTOR_MOUNT_Z),
        "front_right": (0.116, -0.090, MOTOR_MOUNT_Z),
        "rear_left": (-0.116, 0.090, MOTOR_MOUNT_Z),
        "rear_right": (-0.116, -0.090, MOTOR_MOUNT_Z),
    }
    arm_roots = {
        "front_left": (0.038, 0.027, ARM_THICKNESS * 0.5),
        "front_right": (0.038, -0.027, ARM_THICKNESS * 0.5),
        "rear_left": (-0.038, 0.027, ARM_THICKNESS * 0.5),
        "rear_right": (-0.038, -0.027, ARM_THICKNESS * 0.5),
    }

    for name, mount_xyz in motor_mounts.items():
        root_xyz = arm_roots[name]
        _add_arm(
            frame,
            start=root_xyz,
            end=(mount_xyz[0], mount_xyz[1], ARM_THICKNESS * 0.5),
            width=0.016,
            thickness=ARM_THICKNESS,
            overlap=0.024,
            material=carbon,
            name=f"{name}_arm",
        )
        frame.visual(
            Box((0.032, 0.032, ARM_THICKNESS)),
            origin=Origin(xyz=(mount_xyz[0], mount_xyz[1], ARM_THICKNESS * 0.5)),
            material=carbon,
            name=f"{name}_motor_pad",
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.260, 0.210, 0.030)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    prop_mesh = _propeller_mesh("tri_blade_propeller")

    rotor_directions = {
        "front_left": 1.0,
        "front_right": -1.0,
        "rear_left": -1.0,
        "rear_right": 1.0,
    }

    for corner, mount_xyz in motor_mounts.items():
        motor = model.part(f"{corner}_motor")
        motor.visual(
            Cylinder(radius=0.014, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=dark_aluminum,
            name="mount_base",
        )
        motor.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=anodized_black,
            name="motor_can",
        )
        motor.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0195)),
            material=dark_aluminum,
            name="motor_cap",
        )
        motor.inertial = Inertial.from_geometry(
            Cylinder(radius=0.014, length=MOTOR_HEIGHT),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, MOTOR_HEIGHT * 0.5)),
        )

        model.articulation(
            f"frame_to_{corner}_motor",
            ArticulationType.FIXED,
            parent=frame,
            child=motor,
            origin=Origin(xyz=mount_xyz),
        )

        propeller = model.part(f"{corner}_propeller")
        propeller.visual(
            prop_mesh,
            material=prop_black,
            name="propeller_assembly",
        )
        propeller.inertial = Inertial.from_geometry(
            Cylinder(radius=PROP_RADIUS, length=0.008),
            mass=0.006,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
        )

        model.articulation(
            f"{corner}_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=motor,
            child=propeller,
            origin=Origin(xyz=(0.0, 0.0, MOTOR_HEIGHT)),
            axis=(0.0, 0.0, rotor_directions[corner]),
            motion_limits=MotionLimits(effort=0.3, velocity=1400.0),
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

    frame = object_model.get_part("frame")
    front_left_motor = object_model.get_part("front_left_motor")
    front_right_motor = object_model.get_part("front_right_motor")
    rear_left_motor = object_model.get_part("rear_left_motor")
    rear_right_motor = object_model.get_part("rear_right_motor")
    front_left_propeller = object_model.get_part("front_left_propeller")
    front_right_propeller = object_model.get_part("front_right_propeller")
    rear_left_propeller = object_model.get_part("rear_left_propeller")
    rear_right_propeller = object_model.get_part("rear_right_propeller")

    motors = {
        "front_left": front_left_motor,
        "front_right": front_right_motor,
        "rear_left": rear_left_motor,
        "rear_right": rear_right_motor,
    }
    propellers = {
        "front_left": front_left_propeller,
        "front_right": front_right_propeller,
        "rear_left": rear_left_propeller,
        "rear_right": rear_right_propeller,
    }

    for corner in motors:
        motor = motors[corner]
        propeller = propellers[corner]
        spin_joint = object_model.get_articulation(f"{corner}_prop_spin")

        ctx.expect_contact(
            motor,
            frame,
            contact_tol=0.0008,
            name=f"{corner} motor is mounted to the carbon frame",
        )
        ctx.expect_contact(
            propeller,
            motor,
            contact_tol=0.0008,
            name=f"{corner} propeller seats on the motor hub",
        )
        ctx.expect_gap(
            propeller,
            frame,
            axis="z",
            min_gap=0.0015,
            name=f"{corner} propeller clears the frame deck",
        )
        limits = spin_joint.motion_limits
        axis = spin_joint.axis
        ctx.check(
            f"{corner} propeller uses a continuous vertical spin axis",
            spin_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and abs(axis[0]) < 1e-9
            and abs(axis[1]) < 1e-9
            and abs(abs(axis[2]) - 1.0) < 1e-9,
            details=f"type={spin_joint.articulation_type}, axis={axis}, limits={limits}",
        )

    ctx.expect_origin_distance(
        front_left_motor,
        rear_right_motor,
        axes="xy",
        min_dist=0.275,
        max_dist=0.295,
        name="diagonal motor spacing stays in 5-inch racing quad scale",
    )
    ctx.expect_origin_distance(
        front_left_motor,
        front_right_motor,
        axes="y",
        min_dist=0.175,
        max_dist=0.185,
        name="front motor track width is compact and symmetric",
    )
    ctx.expect_origin_distance(
        front_left_motor,
        rear_left_motor,
        axes="x",
        min_dist=0.225,
        max_dist=0.235,
        name="front to rear motor spacing matches an H-frame wheelbase",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    rounded_rect_profile,
)


ARM_CONFIGS = (
    ("front_left", 1.0, 1.0, 1.0),
    ("front_right", 1.0, -1.0, -1.0),
    ("rear_left", -1.0, 1.0, -1.0),
    ("rear_right", -1.0, -1.0, 1.0),
)


def _build_propeller_mesh():
    prop_profile = [
        (-0.158, 0.0),
        (-0.147, 0.010),
        (-0.120, 0.018),
        (-0.070, 0.020),
        (-0.020, 0.008),
        (0.020, 0.008),
        (0.070, 0.020),
        (0.120, 0.018),
        (0.147, 0.010),
        (0.158, 0.0),
        (0.147, -0.010),
        (0.120, -0.018),
        (0.070, -0.020),
        (0.020, -0.008),
        (-0.020, -0.008),
        (-0.070, -0.020),
        (-0.120, -0.018),
        (-0.147, -0.010),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(prop_profile, 0.004, center=True),
        "x8_propeller_blade",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_x8_drone")

    plate_carbon = model.material("plate_carbon", rgba=(0.14, 0.15, 0.16, 1.0))
    frame_carbon = model.material("frame_carbon", rgba=(0.11, 0.12, 0.13, 1.0))
    motor_black = model.material("motor_black", rgba=(0.18, 0.18, 0.19, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))

    center_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.22, 0.22, 0.016), 0.008, center=True),
        "x8_center_plate",
    )
    propeller_mesh = _build_propeller_mesh()

    center_frame = model.part("center_frame")
    center_frame.visual(
        center_plate_mesh,
        material=plate_carbon,
        name="center_plate",
    )
    center_frame.visual(
        Box((0.090, 0.090, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=plate_carbon,
        name="lower_power_block",
    )
    center_frame.visual(
        Box((0.125, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=machined_aluminum,
        name="battery_strap_fore_aft",
    )
    center_frame.visual(
        Box((0.020, 0.125, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=machined_aluminum,
        name="battery_strap_left_right",
    )

    arm_start = 0.090
    arm_end = 0.280
    beam_length = math.sqrt(2.0) * (arm_end - arm_start)

    for arm_name, sign_x, sign_y, _spin_sign in ARM_CONFIGS:
        yaw = math.atan2(sign_y, sign_x)
        beam_center = (
            sign_x * 0.5 * (arm_start + arm_end),
            sign_y * 0.5 * (arm_start + arm_end),
            0.0,
        )
        root_center = (sign_x * 0.118, sign_y * 0.118, 0.0)
        corner_center = (sign_x * 0.102, sign_y * 0.102, 0.0)
        motor_center = (sign_x * arm_end, sign_y * arm_end, 0.0)

        center_frame.visual(
            Box((beam_length, 0.038, 0.018)),
            origin=Origin(xyz=beam_center, rpy=(0.0, 0.0, yaw)),
            material=frame_carbon,
            name=f"arm_beam_{arm_name}",
        )
        center_frame.visual(
            Box((0.074, 0.056, 0.016)),
            origin=Origin(xyz=root_center, rpy=(0.0, 0.0, yaw)),
            material=plate_carbon,
            name=f"arm_root_{arm_name}",
        )
        center_frame.visual(
            Box((0.042, 0.042, 0.020)),
            origin=Origin(xyz=corner_center),
            material=plate_carbon,
            name=f"corner_clamp_{arm_name}",
        )
        center_frame.visual(
            Cylinder(radius=0.026, length=0.022),
            origin=Origin(xyz=motor_center),
            material=frame_carbon,
            name=f"motor_pod_{arm_name}",
        )
        center_frame.visual(
            Cylinder(radius=0.0085, length=0.074),
            origin=Origin(xyz=motor_center),
            material=machined_aluminum,
            name=f"motor_spine_{arm_name}",
        )
        center_frame.visual(
            Cylinder(radius=0.031, length=0.006),
            origin=Origin(xyz=(motor_center[0], motor_center[1], 0.011)),
            material=machined_aluminum,
            name=f"upper_mount_{arm_name}",
        )
        center_frame.visual(
            Cylinder(radius=0.031, length=0.006),
            origin=Origin(xyz=(motor_center[0], motor_center[1], -0.011)),
            material=machined_aluminum,
            name=f"lower_mount_{arm_name}",
        )
        center_frame.visual(
            Cylinder(radius=0.019, length=0.022),
            origin=Origin(xyz=(motor_center[0], motor_center[1], 0.031)),
            material=motor_black,
            name=f"upper_motor_{arm_name}",
        )
        center_frame.visual(
            Cylinder(radius=0.019, length=0.022),
            origin=Origin(xyz=(motor_center[0], motor_center[1], -0.031)),
            material=motor_black,
            name=f"lower_motor_{arm_name}",
        )

    center_frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.12)),
        mass=4.5,
        origin=Origin(),
    )

    for arm_name, sign_x, sign_y, spin_sign in ARM_CONFIGS:
        motor_x = sign_x * arm_end
        motor_y = sign_y * arm_end
        for level_name, z_pos, axis_sign in (
            ("upper", 0.046, spin_sign),
            ("lower", -0.046, -spin_sign),
        ):
            prop = model.part(f"{arm_name}_{level_name}_prop")
            prop.visual(
                propeller_mesh,
                material=prop_black,
                name="blade_planform",
            )
            prop.visual(
                Cylinder(radius=0.022, length=0.008),
                material=motor_black,
                name="hub_shell",
            )
            prop.inertial = Inertial.from_geometry(
                Cylinder(radius=0.160, length=0.008),
                mass=0.045,
                origin=Origin(),
            )
            model.articulation(
                f"{arm_name}_{level_name}_spin",
                ArticulationType.CONTINUOUS,
                parent=center_frame,
                child=prop,
                origin=Origin(xyz=(motor_x, motor_y, z_pos)),
                axis=(0.0, 0.0, axis_sign),
                motion_limits=MotionLimits(effort=0.8, velocity=120.0),
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

    center_frame = object_model.get_part("center_frame")

    ctx.check(
        "center frame has one rigid backbone part",
        center_frame.name == "center_frame",
        details=f"got {center_frame.name}",
    )

    for arm_name, _sign_x, _sign_y, spin_sign in ARM_CONFIGS:
        upper_prop = object_model.get_part(f"{arm_name}_upper_prop")
        lower_prop = object_model.get_part(f"{arm_name}_lower_prop")
        upper_joint = object_model.get_articulation(f"{arm_name}_upper_spin")
        lower_joint = object_model.get_articulation(f"{arm_name}_lower_spin")

        ctx.check(
            f"{arm_name} upper rotor uses a vertical continuous joint",
            upper_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(upper_joint.axis) == (0.0, 0.0, spin_sign),
            details=f"type={upper_joint.articulation_type}, axis={upper_joint.axis}",
        )
        ctx.check(
            f"{arm_name} lower rotor counter-rotates on a vertical continuous joint",
            lower_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(lower_joint.axis) == (0.0, 0.0, -spin_sign),
            details=f"type={lower_joint.articulation_type}, axis={lower_joint.axis}",
        )
        ctx.expect_origin_distance(
            upper_prop,
            lower_prop,
            axes="xy",
            max_dist=1e-6,
            name=f"{arm_name} coaxial prop pair shares one motor axis",
        )
        ctx.expect_origin_gap(
            upper_prop,
            lower_prop,
            axis="z",
            min_gap=0.091,
            max_gap=0.093,
            name=f"{arm_name} coaxial prop pair is vertically stacked",
        )
        ctx.expect_overlap(
            upper_prop,
            center_frame,
            axes="xy",
            min_overlap=0.038,
            elem_b=f"upper_motor_{arm_name}",
            name=f"{arm_name} upper prop sits over the upper motor",
        )
        ctx.expect_overlap(
            lower_prop,
            center_frame,
            axes="xy",
            min_overlap=0.038,
            elem_b=f"lower_motor_{arm_name}",
            name=f"{arm_name} lower prop sits under the lower motor",
        )
        ctx.expect_contact(
            upper_prop,
            center_frame,
            elem_a="hub_shell",
            elem_b=f"upper_motor_{arm_name}",
            contact_tol=5e-4,
            name=f"{arm_name} upper prop hub seats on the upper motor",
        )
        ctx.expect_contact(
            center_frame,
            lower_prop,
            elem_a=f"lower_motor_{arm_name}",
            elem_b="hub_shell",
            contact_tol=5e-4,
            name=f"{arm_name} lower prop hub seats on the lower motor",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_RADIUS = 0.205
BODY_APOTHEM = BODY_RADIUS * math.cos(math.pi / 6.0)
BODY_HEIGHT = 0.048
TOP_CAP_RADIUS = 0.132
TOP_CAP_HEIGHT = 0.010
BATTERY_TRAY_SIZE = (0.108, 0.072, 0.012)
BATTERY_TRAY_Z = -0.004
CENTER_SPINE_SIZE = (0.032, 0.032, 0.022)
PAYLOAD_SIZE = (0.180, 0.120, 0.085)
PAYLOAD_MOUNT_SIZE = (0.120, 0.080, 0.008)
PAYLOAD_STRUT_RADIUS = 0.007
PAYLOAD_STRUT_HEIGHT = 0.060

ARM_ROOT_LENGTH = 0.052
ARM_ROOT_WIDTH = 0.050
ARM_ROOT_HEIGHT = 0.026
ARM_BEAM_START = 0.030
ARM_BEAM_LENGTH = 0.182
ARM_BEAM_RADIUS = 0.012
ARM_LENGTH = 0.238

MOTOR_MOUNT_LENGTH = 0.050
MOTOR_MOUNT_WIDTH = 0.046
MOTOR_MOUNT_HEIGHT = 0.008
MOTOR_RADIUS = 0.027
MOTOR_HEIGHT = 0.028
ROTOR_JOINT_Z = MOTOR_HEIGHT

PROP_SHAFT_RADIUS = 0.006
PROP_SHAFT_HEIGHT = 0.010
PROP_HUB_RADIUS = 0.026
PROP_HUB_HEIGHT = 0.008
PROP_BLADE_THICKNESS = 0.004


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _regular_polygon_profile(radius: float, sides: int, *, start_angle: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(start_angle + (2.0 * math.pi * index / sides)),
            radius * math.sin(start_angle + (2.0 * math.pi * index / sides)),
        )
        for index in range(sides)
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_propeller_blade_mesh():
    blade_profile = [
        (0.012, -0.016),
        (0.040, -0.014),
        (0.090, -0.010),
        (0.130, -0.005),
        (0.145, -0.0025),
        (0.145, 0.0025),
        (0.130, 0.005),
        (0.090, 0.010),
        (0.040, 0.014),
        (0.012, 0.016),
    ]
    return _save_mesh(
        "hex_drone_prop_blade.obj",
        ExtrudeGeometry.from_z0(blade_profile, PROP_BLADE_THICKNESS, cap=True, closed=True),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexarotor_delivery_drone", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.90, 0.92, 0.95, 1.0))
    shell_gray = model.material("shell_gray", rgba=(0.70, 0.73, 0.78, 1.0))
    carbon = model.material("carbon", rgba=(0.16, 0.17, 0.19, 1.0))
    motor_black = model.material("motor_black", rgba=(0.10, 0.10, 0.11, 1.0))
    prop_gray = model.material("prop_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body_shell_mesh = _save_mesh(
        "hex_drone_body_shell.obj",
        ExtrudeGeometry(
            _regular_polygon_profile(BODY_RADIUS, 6, start_angle=math.pi / 6.0),
            BODY_HEIGHT,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    top_cap_mesh = _save_mesh(
        "hex_drone_top_cap.obj",
        ExtrudeGeometry(
            _regular_polygon_profile(TOP_CAP_RADIUS, 6, start_angle=math.pi / 6.0),
            TOP_CAP_HEIGHT,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    prop_blade_mesh = _build_propeller_blade_mesh()

    body = model.part("body")
    body.visual(body_shell_mesh, material=shell_white, name="body_shell")
    body.visual(
        top_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BODY_HEIGHT + 0.5 * TOP_CAP_HEIGHT)),
        material=shell_gray,
        name="top_cap",
    )
    body.visual(
        Box(BATTERY_TRAY_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BATTERY_TRAY_Z)),
        material=shell_gray,
        name="battery_tray",
    )
    body.visual(
        Box(CENTER_SPINE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=shell_gray,
        name="center_spine",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BODY_HEIGHT + TOP_CAP_HEIGHT + 0.004)),
        material=sensor_black,
        name="gps_puck",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.10)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    payload = model.part("payload_pod")
    payload.visual(
        Box(PAYLOAD_MOUNT_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * PAYLOAD_MOUNT_SIZE[2])),
        material=carbon,
        name="mount_plate",
    )
    for x_pos in (-0.058, 0.058):
        for y_pos in (-0.042, 0.042):
            payload.visual(
                Cylinder(radius=PAYLOAD_STRUT_RADIUS, length=PAYLOAD_STRUT_HEIGHT),
                origin=Origin(xyz=(x_pos, y_pos, -0.5 * PAYLOAD_STRUT_HEIGHT)),
                material=carbon,
                name=f"strut_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )
    payload.visual(
        Box(PAYLOAD_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -PAYLOAD_STRUT_HEIGHT - 0.5 * PAYLOAD_SIZE[2])),
        material=shell_gray,
        name="payload_case",
    )
    payload.visual(
        Box((PAYLOAD_SIZE[0] * 0.94, PAYLOAD_SIZE[1] * 0.86, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -PAYLOAD_STRUT_HEIGHT - 0.010)),
        material=shell_white,
        name="payload_lid",
    )
    payload.visual(
        Box((0.020, PAYLOAD_SIZE[1] + 0.004, 0.010)),
        origin=Origin(xyz=(-0.040, 0.0, -PAYLOAD_STRUT_HEIGHT - 0.5 * PAYLOAD_SIZE[2])),
        material=carbon,
        name="retention_band_left",
    )
    payload.visual(
        Box((0.020, PAYLOAD_SIZE[1] + 0.004, 0.010)),
        origin=Origin(xyz=(0.040, 0.0, -PAYLOAD_STRUT_HEIGHT - 0.5 * PAYLOAD_SIZE[2])),
        material=carbon,
        name="retention_band_right",
    )
    payload.inertial = Inertial.from_geometry(
        Box((0.22, 0.15, 0.16)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )

    model.articulation(
        "body_to_payload_pod",
        ArticulationType.FIXED,
        parent=body,
        child=payload,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * BODY_HEIGHT)),
    )

    for index in range(6):
        angle = 2.0 * math.pi * index / 6.0

        arm = model.part(f"arm_{index}")
        arm.visual(
            Box((ARM_ROOT_LENGTH, ARM_ROOT_WIDTH, ARM_ROOT_HEIGHT)),
            origin=Origin(xyz=(0.5 * ARM_ROOT_LENGTH, 0.0, 0.0)),
            material=carbon,
            name="root_fairing",
        )
        arm.visual(
            Cylinder(radius=ARM_BEAM_RADIUS, length=ARM_BEAM_LENGTH),
            origin=Origin(
                xyz=(ARM_BEAM_START + 0.5 * ARM_BEAM_LENGTH, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=carbon,
            name="beam",
        )
        arm.visual(
            Box((MOTOR_MOUNT_LENGTH, MOTOR_MOUNT_WIDTH, MOTOR_MOUNT_HEIGHT)),
            origin=Origin(xyz=(ARM_LENGTH - 0.5 * MOTOR_MOUNT_LENGTH, 0.0, 0.004)),
            material=carbon,
            name="motor_mount",
        )
        arm.visual(
            Cylinder(radius=MOTOR_RADIUS, length=MOTOR_HEIGHT),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.5 * MOTOR_HEIGHT)),
            material=motor_black,
            name="motor_can",
        )
        arm.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, MOTOR_HEIGHT + 0.002)),
            material=shell_gray,
            name="motor_cap",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.30, 0.06, 0.06)),
            mass=0.34,
            origin=Origin(xyz=(0.13, 0.0, 0.015)),
        )

        model.articulation(
            f"body_to_arm_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=arm,
            origin=Origin(
                xyz=(*_polar_xy(BODY_APOTHEM, angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
        )

        prop = model.part(f"propeller_{index}")
        prop.visual(
            Cylinder(radius=PROP_SHAFT_RADIUS, length=PROP_SHAFT_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, 0.5 * PROP_SHAFT_HEIGHT)),
            material=motor_black,
            name="shaft",
        )
        prop.visual(
            Cylinder(radius=PROP_HUB_RADIUS, length=PROP_HUB_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, PROP_SHAFT_HEIGHT + 0.5 * PROP_HUB_HEIGHT)),
            material=motor_black,
            name="hub",
        )
        prop.visual(
            prop_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, PROP_SHAFT_HEIGHT + PROP_HUB_HEIGHT)),
            material=prop_gray,
            name="blade_a",
        )
        prop.visual(
            prop_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, PROP_SHAFT_HEIGHT + PROP_HUB_HEIGHT), rpy=(0.0, 0.0, math.pi)),
            material=prop_gray,
            name="blade_b",
        )
        prop.inertial = Inertial.from_geometry(
            Cylinder(radius=0.15, length=0.024),
            mass=0.07,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
        )

        model.articulation(
            f"arm_to_propeller_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=prop,
            origin=Origin(xyz=(ARM_LENGTH, 0.0, ROTOR_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=80.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    body_shell = body.get_visual("body_shell")
    payload = object_model.get_part("payload_pod")
    payload_mount = payload.get_visual("mount_plate")
    payload_case = payload.get_visual("payload_case")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    body_shell_aabb = ctx.part_element_world_aabb(body, elem=body_shell)
    if body_shell_aabb is None:
        ctx.fail("body_shell_aabb_exists", "Body shell AABB could not be resolved.")
    else:
        body_dx = body_shell_aabb[1][0] - body_shell_aabb[0][0]
        body_dy = body_shell_aabb[1][1] - body_shell_aabb[0][1]
        body_dz = body_shell_aabb[1][2] - body_shell_aabb[0][2]
        ctx.check(
            "body_hex_planform_scale",
            0.34 <= min(body_dx, body_dy) <= 0.38 and 0.39 <= max(body_dx, body_dy) <= 0.43,
            details=f"body planform was {(body_dx, body_dy)}",
        )
        ctx.check(
            "body_reads_flat",
            body_dz <= 0.055,
            details=f"body height was {body_dz}",
        )

    propeller_0 = object_model.get_part("propeller_0")
    propeller_3 = object_model.get_part("propeller_3")
    ctx.expect_origin_distance(propeller_0, propeller_3, axes="x", min_dist=0.76, max_dist=0.88)
    ctx.expect_contact(payload, body, elem_a=payload_mount, elem_b=body_shell, name="payload_pod_attached")
    ctx.expect_gap(
        body,
        payload,
        axis="z",
        min_gap=0.055,
        positive_elem=body_shell,
        negative_elem=payload_case,
        name="payload_case_hangs_below_body",
    )
    ctx.expect_within(payload, body, axes="xy", margin=0.02, name="payload_centered_under_body")

    def _aabb_center(aabb) -> tuple[float, float, float]:
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    for index in range(6):
        arm = object_model.get_part(f"arm_{index}")
        prop = object_model.get_part(f"propeller_{index}")
        root_fairing = arm.get_visual("root_fairing")
        motor_can = arm.get_visual("motor_can")
        shaft = prop.get_visual("shaft")
        blade_a = prop.get_visual("blade_a")
        rotor_joint = object_model.get_articulation(f"arm_to_propeller_{index}")
        limits = rotor_joint.motion_limits

        ctx.expect_contact(
            arm,
            body,
            elem_a=root_fairing,
            elem_b=body_shell,
            name=f"arm_{index}_mounted_to_body",
        )
        ctx.expect_overlap(prop, arm, axes="xy", min_overlap=0.010, name=f"propeller_{index}_over_motor")
        ctx.expect_gap(
            prop,
            arm,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=shaft,
            negative_elem=motor_can,
            name=f"propeller_{index}_shaft_seated_on_motor",
        )

        blade_rest_aabb = ctx.part_element_world_aabb(prop, elem=blade_a)
        if blade_rest_aabb is None:
            ctx.fail(f"propeller_{index}_blade_rest_aabb", "Blade AABB could not be resolved in rest pose.")
        else:
            blade_rest_center = _aabb_center(blade_rest_aabb)
            with ctx.pose({rotor_joint: math.pi / 2.0}):
                ctx.expect_gap(
                    prop,
                    arm,
                    axis="z",
                    max_gap=0.0005,
                    max_penetration=0.0,
                    positive_elem=shaft,
                    negative_elem=motor_can,
                    name=f"propeller_{index}_shaft_seated_in_spin_pose",
                )
                blade_spin_aabb = ctx.part_element_world_aabb(prop, elem=blade_a)
                if blade_spin_aabb is None:
                    ctx.fail(
                        f"propeller_{index}_blade_spin_aabb",
                        "Blade AABB could not be resolved in spin pose.",
                    )
                else:
                    blade_spin_center = _aabb_center(blade_spin_aabb)
                    center_shift = math.dist(blade_rest_center, blade_spin_center)
                    ctx.check(
                        f"propeller_{index}_articulates",
                        center_shift >= 0.090,
                        details=f"blade center shift was {center_shift}",
                    )

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({rotor_joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"arm_to_propeller_{index}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"arm_to_propeller_{index}_lower_no_floating")
            with ctx.pose({rotor_joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"arm_to_propeller_{index}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"arm_to_propeller_{index}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

ROTOR_LAYOUT = {
    "front_left": (-0.165, 0.140, 0.020),
    "front_right": (0.165, 0.140, 0.020),
    "rear_left": (-0.155, -0.140, 0.020),
    "rear_right": (0.155, -0.140, 0.020),
}


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in (
        {"name": name, "color": rgba},
        {"name": name, "rgba": rgba},
        {"name": name},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            continue
    try:
        return Material(name, rgba)
    except TypeError:
        return Material(name)


def _register_materials(model: ArticulatedObject, *materials: Material) -> None:
    if not hasattr(model, "materials"):
        return
    for material in materials:
        model.materials.append(material)


def _vector_rpy(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    horiz = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(horiz, dz)
    return (0.0, pitch, yaw)


def _midpoint(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_midpoint(start, end), rpy=_vector_rpy(start, end)),
        material=material,
    )


def _build_airframe_shell():
    shell = superellipse_side_loft(
        [
            (-0.082, -0.012, 0.018, 0.052),
            (-0.056, -0.016, 0.025, 0.090),
            (-0.010, -0.022, 0.034, 0.122),
            (0.036, -0.021, 0.036, 0.112),
            (0.072, -0.016, 0.030, 0.084),
            (0.096, -0.010, 0.016, 0.044),
        ],
        exponents=2.7,
        segments=64,
    )
    shell.merge(BoxGeometry((0.072, 0.058, 0.016)).translate(0.0, -0.014, 0.036))
    shell.merge(BoxGeometry((0.046, 0.034, 0.008)).translate(0.0, -0.052, 0.029))
    shell.merge(SphereGeometry(0.024).scale(1.35, 0.95, 0.55).translate(0.0, 0.061, -0.007))
    shell.merge(BoxGeometry((0.040, 0.030, 0.010)).translate(0.0, 0.060, -0.017))
    return mesh_from_geometry(shell, ASSETS.mesh_path("airframe_shell.obj"))


SHELL_WHITE = _make_material("shell_white", (0.93, 0.94, 0.96, 1.0))
CARBON_GRAY = _make_material("carbon_gray", (0.22, 0.24, 0.27, 1.0))
SATIN_BLACK = _make_material("satin_black", (0.08, 0.09, 0.10, 1.0))
BRUSHED_METAL = _make_material("brushed_metal", (0.69, 0.71, 0.74, 1.0))
RUBBER_BLACK = _make_material("rubber_black", (0.05, 0.05, 0.06, 1.0))
GLASS_SMOKE = _make_material("glass_smoke", (0.18, 0.22, 0.27, 0.40))
STATUS_RED = _make_material("status_red", (0.73, 0.18, 0.16, 1.0))
STATUS_GREEN = _make_material("status_green", (0.24, 0.60, 0.31, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quadcopter_drone", assets=ASSETS)
    _register_materials(
        model,
        SHELL_WHITE,
        CARBON_GRAY,
        SATIN_BLACK,
        BRUSHED_METAL,
        RUBBER_BLACK,
        GLASS_SMOKE,
        STATUS_RED,
        STATUS_GREEN,
    )

    body = model.part("body")
    body.visual(_build_airframe_shell(), material=SHELL_WHITE)
    body.visual(
        Box((0.100, 0.118, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, -0.0165)),
        material=CARBON_GRAY,
    )
    body.visual(
        Box((0.044, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.048, -0.0175)),
        material=CARBON_GRAY,
    )
    body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.018, 0.045)),
        material=BRUSHED_METAL,
    )
    body.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, -0.018, 0.049)),
        material=SHELL_WHITE,
    )
    for x_sign in (-1.0, 1.0):
        sensor_x = x_sign * 0.019
        body.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(sensor_x, 0.094, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=SATIN_BLACK,
        )
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(sensor_x, 0.100, 0.003)),
            material=GLASS_SMOKE,
        )
    for tip_x in (-0.040, 0.040):
        body.visual(
            Sphere(radius=0.003),
            origin=Origin(xyz=(tip_x, -0.100, 0.072)),
            material=RUBBER_BLACK,
        )
    _add_cylinder_between(
        body, (-0.024, -0.070, 0.024), (-0.040, -0.100, 0.072), 0.0025, SATIN_BLACK
    )
    _add_cylinder_between(body, (0.024, -0.070, 0.024), (0.040, -0.100, 0.072), 0.0025, SATIN_BLACK)

    for rotor_name, motor_pos in ROTOR_LAYOUT.items():
        arm_start = (motor_pos[0] * 0.34, motor_pos[1] * 0.34, 0.014)
        _add_cylinder_between(body, arm_start, motor_pos, 0.010, CARBON_GRAY)

    body.inertial = Inertial.from_geometry(
        Box((0.400, 0.320, 0.090)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    landing_gear = model.part("landing_gear")
    for x_sign in (-1.0, 1.0):
        mount_x = x_sign * 0.052
        rail_x = x_sign * 0.108
        landing_gear.visual(
            Box((0.024, 0.052, 0.008)),
            origin=Origin(xyz=(mount_x, 0.0, -0.010)),
            material=CARBON_GRAY,
        )
        landing_gear.visual(
            Cylinder(radius=0.006, length=0.188),
            origin=Origin(xyz=(rail_x, 0.0, -0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=RUBBER_BLACK,
        )
        for y_pos in (-0.072, 0.072):
            landing_gear.visual(
                Cylinder(radius=0.008, length=0.024),
                origin=Origin(xyz=(rail_x, y_pos, -0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=SATIN_BLACK,
            )
        _add_cylinder_between(
            landing_gear,
            (mount_x, -0.026, -0.010),
            (rail_x, -0.060, -0.104),
            0.0048,
            BRUSHED_METAL,
        )
        _add_cylinder_between(
            landing_gear,
            (mount_x, 0.026, -0.010),
            (rail_x, 0.060, -0.104),
            0.0048,
            BRUSHED_METAL,
        )
    landing_gear.visual(
        Box((0.086, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=CARBON_GRAY,
    )
    landing_gear.visual(
        Cylinder(radius=0.005, length=0.216),
        origin=Origin(xyz=(0.0, -0.060, -0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SATIN_BLACK,
    )
    landing_gear.visual(
        Cylinder(radius=0.005, length=0.216),
        origin=Origin(xyz=(0.0, 0.060, -0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SATIN_BLACK,
    )
    landing_gear.inertial = Inertial.from_geometry(
        Box((0.250, 0.190, 0.130)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )
    model.articulation(
        "body_to_landing_gear",
        ArticulationType.FIXED,
        parent="body",
        child="landing_gear",
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
    )

    gimbal_yaw = model.part("gimbal_yaw")
    gimbal_yaw.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=BRUSHED_METAL,
    )
    gimbal_yaw.visual(
        Box((0.050, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=SATIN_BLACK,
    )
    gimbal_yaw.visual(
        Box((0.054, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=SATIN_BLACK,
    )
    for x_sign in (-1.0, 1.0):
        gimbal_yaw.visual(
            Box((0.004, 0.008, 0.026)),
            origin=Origin(xyz=(x_sign * 0.0225, 0.0, -0.024)),
            material=SATIN_BLACK,
        )
        gimbal_yaw.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(x_sign * 0.0225, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=BRUSHED_METAL,
        )
    gimbal_yaw.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.040)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )
    model.articulation(
        "gimbal_yaw",
        ArticulationType.REVOLUTE,
        parent="body",
        child="gimbal_yaw",
        origin=Origin(xyz=(0.0, 0.048, -0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.5, lower=-0.9, upper=0.9),
    )

    camera_pitch = model.part("camera_pitch")
    camera_pitch.visual(
        Cylinder(radius=0.0035, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRUSHED_METAL,
    )
    camera_pitch.visual(
        Box((0.016, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.008, -0.005)),
        material=SATIN_BLACK,
    )
    camera_pitch.visual(
        Box((0.010, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.014, -0.017)),
        material=SATIN_BLACK,
    )
    camera_pitch.visual(
        Box((0.030, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.028, -0.038)),
        material=CARBON_GRAY,
    )
    camera_pitch.visual(
        Box((0.022, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.017, -0.025)),
        material=SATIN_BLACK,
    )
    camera_pitch.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.0, 0.042, -0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=SATIN_BLACK,
    )
    camera_pitch.visual(
        Sphere(radius=0.0075),
        origin=Origin(xyz=(0.0, 0.048, -0.038)),
        material=GLASS_SMOKE,
    )
    camera_pitch.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.050)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.024, -0.032)),
    )
    model.articulation(
        "gimbal_pitch",
        ArticulationType.REVOLUTE,
        parent="gimbal_yaw",
        child="camera_pitch",
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.5, lower=-0.35, upper=0.65),
    )

    for rotor_name, motor_pos in ROTOR_LAYOUT.items():
        motor_name = f"motor_{rotor_name}"
        prop_name = f"prop_{rotor_name}"
        spin_joint = f"spin_{rotor_name}"

        motor = model.part(motor_name)
        accent_material = (
            STATUS_RED
            if rotor_name.endswith("left") and rotor_name.startswith("front")
            else STATUS_GREEN
            if rotor_name.endswith("right") and rotor_name.startswith("front")
            else BRUSHED_METAL
        )
        motor.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=BRUSHED_METAL,
        )
        motor.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=SATIN_BLACK,
        )
        motor.visual(
            Cylinder(radius=0.015, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=accent_material,
        )
        motor.inertial = Inertial.from_geometry(
            Cylinder(radius=0.016, length=0.024),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
        )
        model.articulation(
            f"body_to_{motor_name}",
            ArticulationType.FIXED,
            parent="body",
            child=motor_name,
            origin=Origin(xyz=motor_pos),
        )

        prop = model.part(prop_name)
        prop.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=SATIN_BLACK,
        )
        prop.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=BRUSHED_METAL,
        )
        prop.visual(
            Box((0.168, 0.016, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=SATIN_BLACK,
        )
        prop.visual(
            Box((0.050, 0.024, 0.0035)),
            origin=Origin(xyz=(0.0, 0.0, 0.0042)),
            material=SATIN_BLACK,
        )
        prop.visual(
            Box((0.046, 0.010, 0.0026)),
            origin=Origin(xyz=(-0.060, 0.005, 0.0045), rpy=(0.0, 0.0, 0.16)),
            material=SATIN_BLACK,
        )
        prop.visual(
            Box((0.046, 0.010, 0.0026)),
            origin=Origin(xyz=(0.060, -0.005, 0.0045), rpy=(0.0, 0.0, -0.16)),
            material=SATIN_BLACK,
        )
        prop.inertial = Inertial.from_geometry(
            Cylinder(radius=0.086, length=0.010),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
        )
        model.articulation(
            spin_joint,
            ArticulationType.CONTINUOUS,
            parent=motor_name,
            child=prop_name,
            origin=Origin(xyz=(0.0, 0.0, 0.026)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=120.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "camera_pitch",
        "gimbal_yaw",
        reason="the pitch axle rides inside the yaw cradle and generated collision hulls are conservative around the nested gimbal frame",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("landing_gear", "body", axes="xy", max_dist=0.015)
    ctx.expect_aabb_overlap("landing_gear", "body", axes="xy", min_overlap=0.060)
    ctx.expect_aabb_gap("body", "landing_gear", axis="z", max_gap=0.010, max_penetration=0.0)

    ctx.expect_origin_distance("gimbal_yaw", "body", axes="xy", max_dist=0.080)
    ctx.expect_aabb_overlap("gimbal_yaw", "body", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("body", "gimbal_yaw", axis="z", max_gap=0.012, max_penetration=0.0)
    ctx.expect_aabb_overlap("camera_pitch", "body", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("body", "camera_pitch", axis="z", max_gap=0.080, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "gimbal_pitch", "camera_pitch", world_axis="z", direction="positive", min_delta=0.01
    )

    rotor_pairs = [
        ("motor_front_left", "prop_front_left"),
        ("motor_front_right", "prop_front_right"),
        ("motor_rear_left", "prop_rear_left"),
        ("motor_rear_right", "prop_rear_right"),
    ]
    for motor_name, prop_name in rotor_pairs:
        ctx.expect_origin_distance(prop_name, motor_name, axes="xy", max_dist=0.003)
        ctx.expect_aabb_overlap(prop_name, motor_name, axes="xy", min_overlap=0.024)
        ctx.expect_aabb_gap(prop_name, motor_name, axis="z", max_gap=0.010, max_penetration=0.0)

    with ctx.pose(gimbal_pitch=-0.35):
        ctx.expect_aabb_overlap("camera_pitch", "body", axes="xy", min_overlap=0.016)
        ctx.expect_aabb_gap("body", "camera_pitch", axis="z", max_gap=0.075, max_penetration=0.0)

    with ctx.pose(gimbal_yaw=0.9, gimbal_pitch=0.65):
        ctx.expect_aabb_overlap("camera_pitch", "body", axes="xy", min_overlap=0.012)
        ctx.expect_aabb_gap("body", "camera_pitch", axis="z", max_gap=0.100, max_penetration=0.0)
        ctx.expect_aabb_gap("body", "gimbal_yaw", axis="z", max_gap=0.012, max_penetration=0.0)

    with ctx.pose(gimbal_yaw=-0.9, gimbal_pitch=-0.2):
        ctx.expect_aabb_overlap("camera_pitch", "body", axes="xy", min_overlap=0.012)
        ctx.expect_aabb_gap("body", "camera_pitch", axis="z", max_gap=0.080, max_penetration=0.0)

    with ctx.pose(
        spin_front_left=math.pi / 2.0,
        spin_front_right=math.pi / 2.0,
        spin_rear_left=math.pi / 2.0,
        spin_rear_right=math.pi / 2.0,
    ):
        for motor_name, prop_name in rotor_pairs:
            ctx.expect_origin_distance(prop_name, motor_name, axes="xy", max_dist=0.003)
            ctx.expect_aabb_overlap(prop_name, motor_name, axes="xy", min_overlap=0.020)
            ctx.expect_aabb_gap(prop_name, motor_name, axis="z", max_gap=0.010, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

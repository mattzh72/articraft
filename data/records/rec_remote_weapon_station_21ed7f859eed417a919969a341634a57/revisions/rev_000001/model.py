from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilized_weapon_station")

    olive = model.material("olive", rgba=(0.34, 0.38, 0.29, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.26, 0.29, 0.22, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.22, 0.24, 0.27, 1.0))
    graphite = model.material("graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    sensor_gray = model.material("sensor_gray", rgba=(0.48, 0.50, 0.52, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.08, 0.12, 0.16, 0.95))

    rear_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.24, 0.40, 0.03),
            0.18,
            cap=True,
            center=True,
        ),
        "turret_rear_shell",
    )
    roof_blister_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.18, 0.22, 0.025),
            0.06,
            cap=True,
            center=True,
        ),
        "turret_roof_blister",
    )
    sensor_capsule_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.055, length=0.10),
        "sensor_pod_shell",
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.21, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=gunmetal,
        name="pedestal_drum",
    )
    yaw_base.visual(
        Cylinder(radius=0.12, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=olive,
        name="pedestal_neck",
    )
    yaw_base.visual(
        Cylinder(radius=0.24, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=gunmetal,
        name="yaw_ring",
    )

    turret_body = model.part("turret_body")
    turret_body.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_olive,
        name="yaw_skirt",
    )
    turret_body.visual(
        rear_shell_mesh,
        origin=Origin(xyz=(-0.13, 0.0, 0.15)),
        material=olive,
        name="rear_shell",
    )
    turret_body.visual(
        Box((0.18, 0.08, 0.18)),
        origin=Origin(xyz=(0.05, 0.15, 0.13)),
        material=olive,
        name="left_cheek",
    )
    turret_body.visual(
        Box((0.18, 0.08, 0.18)),
        origin=Origin(xyz=(0.05, -0.15, 0.13)),
        material=olive,
        name="right_cheek",
    )
    turret_body.visual(
        roof_blister_mesh,
        origin=Origin(xyz=(-0.07, 0.0, 0.26)),
        material=dark_olive,
        name="roof_blister",
    )
    turret_body.visual(
        Box((0.10, 0.16, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.08)),
        material=dark_olive,
        name="center_spine",
    )

    sensor_pod = model.part("sensor_pod")
    sensor_pod.visual(
        Box((0.05, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.02, 0.05)),
        material=gunmetal,
        name="mount_bracket",
    )
    sensor_pod.visual(
        sensor_capsule_mesh,
        origin=Origin(xyz=(0.02, 0.095, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=sensor_gray,
        name="pod_shell",
    )
    sensor_pod.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(0.125, 0.095, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="lens_hood",
    )
    sensor_pod.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.145, 0.095, 0.055)),
        material=lens_glass,
        name="lens_globe",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.03, length=0.22),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    cradle.visual(
        Box((0.08, 0.12, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=dark_olive,
        name="breech_block",
    )
    cradle.visual(
        Box((0.24, 0.12, 0.09)),
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        material=olive,
        name="receiver_body",
    )
    cradle.visual(
        Box((0.05, 0.028, 0.055)),
        origin=Origin(xyz=(0.20, -0.112, -0.005)),
        material=dark_olive,
        name="guard_hinge_block",
    )
    cradle.visual(
        Box((0.08, 0.05, 0.025)),
        origin=Origin(xyz=(0.16, -0.083, -0.0375)),
        material=dark_olive,
        name="guard_hinge_brace",
    )
    cradle.visual(
        Cylinder(radius=0.032, length=0.46),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_jacket",
    )
    cradle.visual(
        Cylinder(radius=0.021, length=0.10),
        origin=Origin(xyz=(0.61, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="muzzle_device",
    )

    guard_arm = model.part("guard_arm")
    guard_arm.visual(
        Box((0.024, 0.028, 0.048)),
        origin=Origin(xyz=(0.012, 0.014, 0.0)),
        material=gunmetal,
        name="hinge_lug",
    )
    guard_arm.visual(
        Box((0.30, 0.016, 0.042)),
        origin=Origin(xyz=(0.162, 0.014, 0.0)),
        material=graphite,
        name="arm_beam",
    )
    guard_arm.visual(
        Cylinder(radius=0.017, length=0.045),
        origin=Origin(xyz=(0.315, 0.014, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="guard_nose",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.CONTINUOUS,
        parent=yaw_base,
        child=turret_body,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )
    model.articulation(
        "sensor_mount",
        ArticulationType.FIXED,
        parent=turret_body,
        child=sensor_pod,
        origin=Origin(xyz=(0.02, 0.19, 0.09)),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=turret_body,
        child=cradle,
        origin=Origin(xyz=(0.03, 0.0, 0.13)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.0,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=guard_arm,
        origin=Origin(xyz=(0.205, -0.098, -0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
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

    yaw_base = object_model.get_part("yaw_base")
    turret_body = object_model.get_part("turret_body")
    sensor_pod = object_model.get_part("sensor_pod")
    cradle = object_model.get_part("cradle")
    guard_arm = object_model.get_part("guard_arm")

    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    guard_hinge = object_model.get_articulation("guard_hinge")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (yaw_base, turret_body, sensor_pod, cradle, guard_arm)),
        details="Expected yaw base, turret body, sensor pod, cradle, and guard arm.",
    )

    ctx.expect_contact(
        turret_body,
        yaw_base,
        elem_a="yaw_skirt",
        elem_b="yaw_ring",
        contact_tol=1e-4,
        name="turret body sits on the yaw ring",
    )
    ctx.expect_contact(
        cradle,
        turret_body,
        elem_a="trunnion_shaft",
        elem_b="left_cheek",
        contact_tol=1e-4,
        name="cradle trunnion seats in the left cheek",
    )
    ctx.expect_contact(
        sensor_pod,
        turret_body,
        elem_a="mount_bracket",
        elem_b="left_cheek",
        contact_tol=1e-4,
        name="sensor pod bracket mounts to the turret shoulder",
    )
    ctx.expect_contact(
        guard_arm,
        cradle,
        elem_a="hinge_lug",
        elem_b="guard_hinge_block",
        contact_tol=1e-4,
        name="guard arm hinges from the cradle bracket",
    )

    rest_sensor = ctx.part_world_position(sensor_pod)
    with ctx.pose({yaw_joint: 1.0}):
        yawed_sensor = ctx.part_world_position(sensor_pod)
    ctx.check(
        "yaw joint swings the turret around the vertical axis",
        rest_sensor is not None
        and yawed_sensor is not None
        and abs(yawed_sensor[0] - rest_sensor[0]) > 0.10
        and abs(yawed_sensor[1] - rest_sensor[1]) > 0.05
        and abs(yawed_sensor[2] - rest_sensor[2]) < 0.01,
        details=f"rest={rest_sensor}, yawed={yawed_sensor}",
    )

    rest_barrel = ctx.part_element_world_aabb(cradle, elem="barrel_jacket")
    pitch_up = pitch_joint.motion_limits.upper if pitch_joint.motion_limits is not None else 0.9
    with ctx.pose({pitch_joint: pitch_up}):
        raised_barrel = ctx.part_element_world_aabb(cradle, elem="barrel_jacket")
    ctx.check(
        "pitch joint raises the gun cradle",
        rest_barrel is not None
        and raised_barrel is not None
        and raised_barrel[1][2] > rest_barrel[1][2] + 0.18,
        details=f"rest={rest_barrel}, raised={raised_barrel}",
    )

    rest_guard = ctx.part_element_world_aabb(guard_arm, elem="arm_beam")
    guard_open = guard_hinge.motion_limits.upper if guard_hinge.motion_limits is not None else 1.0
    with ctx.pose({guard_hinge: guard_open}):
        deployed_guard = ctx.part_element_world_aabb(guard_arm, elem="arm_beam")
        ctx.expect_gap(
            guard_arm,
            turret_body,
            axis="x",
            min_gap=0.04,
            name="deployed guard clears the turret housing forward",
        )
    ctx.check(
        "guard arm folds away from its stowed position",
        rest_guard is not None
        and deployed_guard is not None
        and deployed_guard[0][2] < rest_guard[0][2] - 0.10,
        details=f"rest={rest_guard}, deployed={deployed_guard}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

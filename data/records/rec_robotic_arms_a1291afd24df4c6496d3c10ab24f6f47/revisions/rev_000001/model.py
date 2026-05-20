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
    rounded_rect_profile,
    section_loft,
)


def _xy_section(length: float, width: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(length, width, radius)]


def _yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robotic_arm")

    base_paint = model.material("base_paint", rgba=(0.24, 0.25, 0.27, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.90, 0.91, 0.92, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    joint_steel = model.material("joint_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.10, 0.34, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.19, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_paint,
        name="pedestal_foot",
    )
    base.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=base_paint,
        name="pedestal_riser",
    )
    shoulder_housing = section_loft(
        [
            _xy_section(0.24, 0.22, 0.040, 0.09),
            _xy_section(0.21, 0.20, 0.040, 0.18),
            _xy_section(0.18, 0.18, 0.035, 0.30),
            _xy_section(0.16, 0.16, 0.030, 0.42),
        ]
    )
    base.visual(
        mesh_from_geometry(shoulder_housing, "shoulder_housing"),
        material=base_paint,
        name="shoulder_housing",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=joint_steel,
        name="shoulder_bearing",
    )
    base.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(-0.095, 0.0, 0.24)),
        material=trim_dark,
        name="service_panel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 0.46)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.082, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=joint_steel,
        name="shoulder_turntable",
    )
    upper_arm.visual(
        Box((0.14, 0.16, 0.12)),
        origin=Origin(xyz=(0.055, 0.0, 0.085)),
        material=trim_dark,
        name="shoulder_motor_block",
    )
    upper_arm.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.12, 0.15, 0.028, 0.03),
                    _yz_section(0.10, 0.13, 0.024, 0.22),
                    _yz_section(0.08, 0.11, 0.022, 0.42),
                    _yz_section(0.10, 0.11, 0.020, 0.52),
                ]
            ),
            "upper_arm_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=arm_paint,
        name="upper_link_shell",
    )
    upper_arm.visual(
        Box((0.050, 0.026, 0.14)),
        origin=Origin(xyz=(0.535, 0.057, 0.09)),
        material=trim_dark,
        name="elbow_clevis_left",
    )
    upper_arm.visual(
        Box((0.050, 0.026, 0.14)),
        origin=Origin(xyz=(0.535, -0.057, 0.09)),
        material=trim_dark,
        name="elbow_clevis_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.64, 0.18, 0.18)),
        mass=24.0,
        origin=Origin(xyz=(0.28, 0.0, 0.09)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.038, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_steel,
        name="elbow_trunnion",
    )
    forearm.visual(
        Box((0.085, 0.070, 0.095)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=trim_dark,
        name="elbow_motor_housing",
    )
    forearm.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.070, 0.095, 0.020, 0.04),
                    _yz_section(0.064, 0.086, 0.018, 0.20),
                    _yz_section(0.058, 0.078, 0.016, 0.36),
                    _yz_section(0.054, 0.070, 0.014, 0.46),
                ]
            ),
            "forearm_shell",
        ),
        material=arm_paint,
        name="forearm_shell",
    )
    forearm.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(xyz=(0.465, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="wrist_mount",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.54, 0.10, 0.10)),
        mass=15.0,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_steel,
        name="wrist_collar",
    )
    wrist_head.visual(
        Box((0.105, 0.080, 0.095)),
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        material=arm_paint,
        name="wrist_body",
    )
    wrist_head.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.169, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.040, 0.032, 0.022)),
        origin=Origin(xyz=(0.118, 0.0, 0.044)),
        material=accent_blue,
        name="wrist_sensor",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.12)),
        mass=4.5,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.4,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.58, 0.0, 0.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=1.6,
            lower=-1.2,
            upper=1.9,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=3.0,
            lower=-3.0,
            upper=3.0,
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

    base = object_model.get_part("base")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.expect_origin_gap(
        wrist_head,
        base,
        axis="x",
        min_gap=0.85,
        name="rest pose reaches well forward of the pedestal",
    )
    ctx.expect_origin_gap(
        wrist_head,
        base,
        axis="z",
        min_gap=0.50,
        name="rest pose keeps the wrist above the pedestal",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    rest_sensor_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="wrist_sensor"))

    with ctx.pose({shoulder_yaw: 1.0}):
        yawed_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings the arm laterally",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and abs(yawed_wrist_pos[1]) > 0.75
        and abs(yawed_wrist_pos[2] - rest_wrist_pos[2]) < 0.02,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({elbow_pitch: 1.0}):
        elbowed_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch lifts the forearm and wrist",
        rest_wrist_pos is not None
        and elbowed_wrist_pos is not None
        and elbowed_wrist_pos[2] > rest_wrist_pos[2] + 0.30,
        details=f"rest={rest_wrist_pos}, elbowed={elbowed_wrist_pos}",
    )

    with ctx.pose({wrist_roll: 1.0}):
        rolled_wrist_pos = ctx.part_world_position(wrist_head)
        rolled_sensor_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="wrist_sensor"))
    ctx.check(
        "wrist roll spins the asymmetric head around the forearm axis",
        rest_wrist_pos is not None
        and rolled_wrist_pos is not None
        and rest_sensor_center is not None
        and rolled_sensor_center is not None
        and abs(rolled_wrist_pos[0] - rest_wrist_pos[0]) < 1e-6
        and abs(rolled_wrist_pos[1] - rest_wrist_pos[1]) < 1e-6
        and abs(rolled_wrist_pos[2] - rest_wrist_pos[2]) < 1e-6
        and (
            abs(rolled_sensor_center[1] - rest_sensor_center[1]) > 0.02
            or abs(rolled_sensor_center[2] - rest_sensor_center[2]) > 0.02
        ),
        details=(
            f"rest_origin={rest_wrist_pos}, rolled_origin={rolled_wrist_pos}, "
            f"rest_sensor={rest_sensor_center}, rolled_sensor={rolled_sensor_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

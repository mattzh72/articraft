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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(x_pos: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y_pos, z_pos) for y_pos, z_pos in rounded_rect_profile(width_y, height_z, radius)]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robot_arm")

    pedestal_gray = model.material("pedestal_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    robot_white = model.material("robot_white", rgba=(0.89, 0.90, 0.92, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.47, 0.14, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))

    upper_arm_mesh = _save_mesh(
        "upper_arm_shell",
        section_loft(
            [
                _yz_section(0.04, 0.14, 0.11, 0.026),
                _yz_section(0.18, 0.15, 0.12, 0.028),
                _yz_section(0.40, 0.13, 0.105, 0.024),
                _yz_section(0.52, 0.115, 0.095, 0.022),
            ]
        ),
    )
    forearm_mesh = _save_mesh(
        "forearm_shell",
        section_loft(
            [
                _yz_section(0.03, 0.105, 0.090, 0.020),
                _yz_section(0.14, 0.096, 0.084, 0.019),
                _yz_section(0.28, 0.086, 0.074, 0.017),
                _yz_section(0.39, 0.080, 0.068, 0.016),
            ]
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.23, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=pedestal_gray,
        name="foot_plate",
    )
    pedestal_base.visual(
        Cylinder(radius=0.17, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=pedestal_gray,
        name="base_skirt",
    )
    pedestal_base.visual(
        Cylinder(radius=0.12, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=robot_white,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=pedestal_gray,
        name="shoulder_bearing_pedestal",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.52),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
    )

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Cylinder(radius=0.145, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=pedestal_gray,
        name="turntable_drum",
    )
    shoulder_housing.visual(
        Box((0.18, 0.22, 0.18)),
        origin=Origin(xyz=(0.09, 0.0, 0.15)),
        material=robot_white,
        name="gearbox_body",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(xyz=(0.10, 0.0, 0.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_orange,
        name="motor_cap",
    )
    shoulder_housing.visual(
        Box((0.03, 0.16, 0.12)),
        origin=Origin(xyz=(0.195, 0.0, 0.14)),
        material=dark_trim,
        name="arm_mount_saddle",
    )
    shoulder_housing.inertial = Inertial.from_geometry(
        Box((0.24, 0.26, 0.24)),
        mass=18.0,
        origin=Origin(xyz=(0.10, 0.0, 0.12)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.05, 0.14, 0.12)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=dark_trim,
        name="root_block",
    )
    upper_arm.visual(
        upper_arm_mesh,
        material=safety_orange,
        name="beam_shell",
    )
    upper_arm.visual(
        Box((0.24, 0.09, 0.04)),
        origin=Origin(xyz=(0.24, 0.0, 0.07)),
        material=robot_white,
        name="service_cover",
    )
    upper_arm.visual(
        Box((0.09, 0.025, 0.12)),
        origin=Origin(xyz=(0.555, 0.060, 0.0)),
        material=robot_white,
        name="elbow_yoke_left",
    )
    upper_arm.visual(
        Box((0.09, 0.025, 0.12)),
        origin=Origin(xyz=(0.555, -0.060, 0.0)),
        material=robot_white,
        name="elbow_yoke_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.60, 0.16, 0.15)),
        mass=14.0,
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.042, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_collar",
    )
    forearm.visual(
        forearm_mesh,
        material=robot_white,
        name="beam_shell",
    )
    forearm.visual(
        Box((0.21, 0.07, 0.03)),
        origin=Origin(xyz=(0.19, 0.0, -0.042)),
        material=dark_trim,
        name="underslung_cover",
    )
    forearm.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(0.41, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_orange,
        name="wrist_roll_housing",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.44, 0.11, 0.11)),
        mass=9.0,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.050, length=0.06),
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_barrel",
    )
    wrist_head.visual(
        Box((0.10, 0.11, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=robot_white,
        name="head_body",
    )
    wrist_head.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=Origin(xyz=(0.136, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.03, 0.02, 0.025)),
        origin=Origin(xyz=(0.090, 0.0, 0.0625)),
        material=safety_orange,
        name="roll_indicator",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.13)),
        mass=3.5,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=shoulder_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=1.8,
            lower=-3.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "shoulder_to_upper_arm",
        ArticulationType.FIXED,
        parent=shoulder_housing,
        child=upper_arm,
        origin=Origin(xyz=(0.21, 0.0, 0.14)),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.56, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.2,
            lower=-0.25,
            upper=1.85,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=4.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal_base = object_model.get_part("pedestal_base")
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.expect_gap(
        shoulder_housing,
        pedestal_base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder housing seats on pedestal",
    )
    ctx.expect_origin_gap(
        upper_arm,
        shoulder_housing,
        axis="x",
        min_gap=0.18,
        max_gap=0.24,
        name="upper arm projects forward from shoulder housing",
    )

    upper_beam_aabb = ctx.part_element_world_aabb(upper_arm, elem="beam_shell")
    forearm_beam_aabb = ctx.part_element_world_aabb(forearm, elem="beam_shell")
    if upper_beam_aabb is not None and forearm_beam_aabb is not None:
        upper_beam_length = upper_beam_aabb[1][0] - upper_beam_aabb[0][0]
        forearm_beam_length = forearm_beam_aabb[1][0] - forearm_beam_aabb[0][0]
        ctx.check(
            "primary link is longer than secondary link",
            upper_beam_length > forearm_beam_length + 0.10,
            details=f"upper={upper_beam_length:.3f}, forearm={forearm_beam_length:.3f}",
        )
    else:
        ctx.fail("primary link is longer than secondary link", "beam visual AABBs were unavailable")

    rest_wrist_position = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 1.10}):
        raised_wrist_position = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow positive motion lifts the wrist",
        rest_wrist_position is not None
        and raised_wrist_position is not None
        and raised_wrist_position[2] > rest_wrist_position[2] + 0.20,
        details=f"rest={rest_wrist_position}, raised={raised_wrist_position}",
    )

    with ctx.pose({shoulder_yaw: 0.90}):
        yawed_wrist_position = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder positive motion swings arm toward positive y",
        rest_wrist_position is not None
        and yawed_wrist_position is not None
        and yawed_wrist_position[1] > rest_wrist_position[1] + 0.30,
        details=f"rest={rest_wrist_position}, yawed={yawed_wrist_position}",
    )

    indicator_rest_aabb = ctx.part_element_world_aabb(wrist_head, elem="roll_indicator")
    with ctx.pose({wrist_roll: math.pi / 2.0}):
        indicator_rolled_aabb = ctx.part_element_world_aabb(wrist_head, elem="roll_indicator")
    indicator_rest_center = _aabb_center(indicator_rest_aabb)
    indicator_rolled_center = _aabb_center(indicator_rolled_aabb)
    ctx.check(
        "wrist roll moves the asymmetric indicator around the forearm axis",
        indicator_rest_center is not None
        and indicator_rolled_center is not None
        and indicator_rolled_center[1] < indicator_rest_center[1] - 0.035
        and abs(indicator_rolled_center[2] - indicator_rest_center[2]) > 0.035,
        details=f"rest={indicator_rest_center}, rolled={indicator_rolled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

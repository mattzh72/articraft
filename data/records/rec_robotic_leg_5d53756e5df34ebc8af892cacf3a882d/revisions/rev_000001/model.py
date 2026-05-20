from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _xy_section(
    width_x: float,
    width_y: float,
    *,
    z: float,
    radius: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (center_x + x, center_y + y, z)
        for x, y in rounded_rect_profile(width_x, width_y, radius)
    )


def _loft_mesh(name: str, sections: list[tuple[tuple[float, float, float], ...]]):
    return mesh_from_geometry(section_loft(sections), name)


def _yz_section(
    width_y: float,
    height_z: float,
    *,
    x: float,
    radius: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (x, center_y + y, center_z + z)
        for y, z in rounded_rect_profile(width_y, height_z, radius)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    housing_gray = model.material("housing_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    link_gray = model.material("link_gray", rgba=(0.32, 0.35, 0.39, 1.0))
    joint_dark = model.material("joint_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    upper_housing = model.part("upper_housing")
    upper_housing.visual(
        _loft_mesh(
            "upper_housing_shell",
            [
                _xy_section(0.112, 0.090, z=0.130, radius=0.018),
                _xy_section(0.172, 0.128, z=0.052, radius=0.022),
                _xy_section(0.160, 0.118, z=-0.006, radius=0.020),
                _xy_section(0.118, 0.096, z=-0.036, radius=0.016),
            ],
        ),
        material=housing_gray,
        name="housing_shell",
    )
    upper_housing.visual(
        Box((0.122, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=joint_dark,
        name="mount_plate",
    )
    upper_housing.visual(
        Box((0.058, 0.042, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=joint_dark,
        name="mount_neck",
    )
    upper_housing.visual(
        Box((0.042, 0.014, 0.112)),
        origin=Origin(xyz=(0.0, 0.036, -0.074)),
        material=joint_dark,
        name="hip_left_tower",
    )
    upper_housing.visual(
        Box((0.042, 0.014, 0.112)),
        origin=Origin(xyz=(0.0, -0.036, -0.074)),
        material=joint_dark,
        name="hip_right_tower",
    )
    upper_housing.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.045, -0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_dark,
        name="hip_left_pod",
    )
    upper_housing.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, -0.045, -0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_dark,
        name="hip_right_pod",
    )
    upper_housing.inertial = Inertial.from_geometry(
        Box((0.180, 0.130, 0.270)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        _loft_mesh(
            "thigh_link_shell",
            [
                _xy_section(0.052, 0.028, z=-0.010, radius=0.009, center_x=0.004),
                _xy_section(0.068, 0.036, z=-0.090, radius=0.011, center_x=0.008),
                _xy_section(0.078, 0.046, z=-0.188, radius=0.013, center_x=0.014),
                _xy_section(0.046, 0.034, z=-0.258, radius=0.009, center_x=0.018),
            ],
        ),
        material=link_gray,
        name="thigh_shell",
    )
    thigh_link.visual(
        Cylinder(radius=0.024, length=0.058),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_dark,
        name="hip_collar",
    )
    thigh_link.visual(
        Box((0.052, 0.028, 0.044)),
        origin=Origin(xyz=(0.006, 0.0, -0.022)),
        material=joint_dark,
        name="hip_hub",
    )
    thigh_link.visual(
        Box((0.032, 0.050, 0.032)),
        origin=Origin(xyz=(0.036, 0.0, -0.272)),
        material=joint_dark,
        name="knee_bridge",
    )
    thigh_link.visual(
        Box((0.036, 0.010, 0.044)),
        origin=Origin(xyz=(0.018, 0.024, -0.306)),
        material=joint_dark,
        name="knee_left_cheek",
    )
    thigh_link.visual(
        Box((0.036, 0.010, 0.044)),
        origin=Origin(xyz=(0.018, -0.024, -0.306)),
        material=joint_dark,
        name="knee_right_cheek",
    )
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.090, 0.070, 0.330)),
        mass=5.0,
        origin=Origin(xyz=(0.012, 0.0, -0.156)),
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        _loft_mesh(
            "shank_link_shell",
            [
                _xy_section(0.050, 0.026, z=-0.010, radius=0.008, center_x=0.004),
                _xy_section(0.058, 0.030, z=-0.085, radius=0.010, center_x=0.007),
                _xy_section(0.064, 0.040, z=-0.176, radius=0.011, center_x=0.010),
                _xy_section(0.040, 0.032, z=-0.236, radius=0.008, center_x=0.010),
            ],
        ),
        material=link_gray,
        name="shank_shell",
    )
    shank_link.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_dark,
        name="knee_collar",
    )
    shank_link.visual(
        Box((0.046, 0.026, 0.040)),
        origin=Origin(xyz=(0.005, 0.0, -0.020)),
        material=joint_dark,
        name="knee_hub",
    )
    shank_link.visual(
        Box((0.026, 0.046, 0.020)),
        origin=Origin(xyz=(0.032, 0.0, -0.244)),
        material=joint_dark,
        name="ankle_bridge",
    )
    shank_link.visual(
        Box((0.032, 0.010, 0.052)),
        origin=Origin(xyz=(0.010, 0.024, -0.280)),
        material=joint_dark,
        name="ankle_left_cheek",
    )
    shank_link.visual(
        Box((0.032, 0.010, 0.052)),
        origin=Origin(xyz=(0.010, -0.024, -0.280)),
        material=joint_dark,
        name="ankle_right_cheek",
    )
    shank_link.inertial = Inertial.from_geometry(
        Box((0.075, 0.060, 0.300)),
        mass=3.2,
        origin=Origin(xyz=(0.010, 0.0, -0.145)),
    )

    foot_section = model.part("foot_section")
    foot_section.visual(
        _loft_mesh(
            "foot_shell",
            [
                _yz_section(0.022, 0.032, x=0.004, radius=0.006, center_z=-0.014),
                _yz_section(0.032, 0.046, x=0.030, radius=0.008, center_z=-0.042),
                _yz_section(0.056, 0.046, x=0.082, radius=0.010, center_z=-0.080),
                _yz_section(0.068, 0.022, x=0.132, radius=0.006, center_z=-0.104),
            ],
        ),
        material=link_gray,
        name="foot_shell",
    )
    foot_section.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_dark,
        name="ankle_collar",
    )
    foot_section.visual(
        Box((0.040, 0.026, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=joint_dark,
        name="ankle_hub",
    )
    foot_section.visual(
        Box((0.150, 0.070, 0.010)),
        origin=Origin(xyz=(0.058, 0.0, -0.116)),
        material=rubber_black,
        name="sole_pad",
    )
    foot_section.inertial = Inertial.from_geometry(
        Box((0.170, 0.075, 0.120)),
        mass=1.8,
        origin=Origin(xyz=(0.060, 0.0, -0.070)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.0,
            lower=-0.95,
            upper=0.90,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.020, 0.0, -0.306)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=2.4,
            lower=0.0,
            upper=2.30,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=foot_section,
        origin=Origin(xyz=(0.012, 0.0, -0.280)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=2.8,
            lower=-0.65,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper_housing = object_model.get_part("upper_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    foot_section = object_model.get_part("foot_section")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.expect_origin_distance(
        thigh_link,
        upper_housing,
        axes="y",
        max_dist=1e-6,
        name="thigh chain stays centered on the housing midline",
    )
    ctx.expect_origin_distance(
        shank_link,
        upper_housing,
        axes="y",
        max_dist=1e-6,
        name="shank stays centered on the module midline",
    )
    ctx.expect_origin_distance(
        foot_section,
        upper_housing,
        axes="y",
        max_dist=1e-6,
        name="foot stays centered on the module midline",
    )

    thigh_rest = ctx.part_element_world_aabb(thigh_link, elem="thigh_shell")
    with ctx.pose({hip_pitch: 0.75}):
        thigh_flexed = ctx.part_element_world_aabb(thigh_link, elem="thigh_shell")
    ctx.check(
        "hip positive rotation swings the thigh forward",
        thigh_rest is not None
        and thigh_flexed is not None
        and thigh_flexed[1][0] > thigh_rest[1][0] + 0.05,
        details=f"rest={thigh_rest}, flexed={thigh_flexed}",
    )

    shank_rest = ctx.part_element_world_aabb(shank_link, elem="shank_shell")
    with ctx.pose({knee_pitch: 1.40}):
        shank_flexed = ctx.part_element_world_aabb(shank_link, elem="shank_shell")
    ctx.check(
        "knee positive rotation folds the shank forward",
        shank_rest is not None
        and shank_flexed is not None
        and shank_flexed[1][0] > shank_rest[1][0] + 0.08,
        details=f"rest={shank_rest}, flexed={shank_flexed}",
    )

    foot_rest = ctx.part_element_world_aabb(foot_section, elem="sole_pad")
    with ctx.pose({ankle_pitch: 0.45}):
        foot_pitched = ctx.part_element_world_aabb(foot_section, elem="sole_pad")
    ctx.check(
        "ankle positive rotation lifts the forefoot",
        foot_rest is not None
        and foot_pitched is not None
        and foot_pitched[1][2] > foot_rest[1][2] + 0.02,
        details=f"rest={foot_rest}, pitched={foot_pitched}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

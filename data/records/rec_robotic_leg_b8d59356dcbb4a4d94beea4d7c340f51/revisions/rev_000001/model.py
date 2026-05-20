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
    z: float,
    width: float,
    depth: float,
    radius: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_center + x, y_center + y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _yz_section(
    x: float,
    depth: float,
    height: float,
    radius: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z_center + z)
        for y, z in rounded_rect_profile(depth, height, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    carbon = model.material("carbon", rgba=(0.10, 0.11, 0.12, 1.0))
    alloy = model.material("alloy", rgba=(0.67, 0.70, 0.74, 1.0))
    accent = model.material("accent", rgba=(0.84, 0.48, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    upper_leg_housing = model.part("upper_leg_housing")
    upper_leg_housing.inertial = Inertial.from_geometry(
        Box((0.32, 0.22, 0.84)),
        mass=28.0,
        origin=Origin(xyz=(0.01, 0.0, 0.42)),
    )
    housing_shell = section_loft(
        [
            _xy_section(0.00, 0.22, 0.18, 0.028),
            _xy_section(0.26, 0.18, 0.16, 0.025),
            _xy_section(0.58, 0.18, 0.15, 0.024, x_center=0.01),
            _xy_section(0.76, 0.25, 0.17, 0.028, x_center=0.02),
        ]
    )
    upper_leg_housing.visual(
        mesh_from_geometry(housing_shell, "upper_leg_housing_shell"),
        material=graphite,
        name="housing_shell",
    )
    upper_leg_housing.visual(
        Box((0.24, 0.18, 0.06)),
        origin=Origin(xyz=(0.00, 0.0, 0.03)),
        material=carbon,
        name="base_pedestal",
    )
    upper_leg_housing.visual(
        Box((0.08, 0.06, 0.10)),
        origin=Origin(xyz=(0.11, 0.0, 0.74)),
        material=carbon,
        name="hip_spine",
    )
    upper_leg_housing.visual(
        Box((0.06, 0.03, 0.056)),
        origin=Origin(xyz=(0.162, 0.065, 0.76)),
        material=carbon,
        name="hip_yoke_left",
    )
    upper_leg_housing.visual(
        Box((0.06, 0.03, 0.056)),
        origin=Origin(xyz=(0.162, -0.065, 0.76)),
        material=carbon,
        name="hip_yoke_right",
    )
    upper_leg_housing.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(xyz=(0.095, 0.074, 0.76), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hip_motor_left",
    )
    upper_leg_housing.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(xyz=(0.095, -0.074, 0.76), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hip_motor_right",
    )
    upper_leg_housing.visual(
        Box((0.16, 0.14, 0.18)),
        origin=Origin(xyz=(-0.02, 0.0, 0.22)),
        material=accent,
        name="power_core",
    )

    thigh_link = model.part("thigh_link")
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.38)),
        mass=11.0,
        origin=Origin(xyz=(0.03, 0.0, -0.19)),
    )
    thigh_shell = section_loft(
        [
            _xy_section(-0.03, 0.14, 0.11, 0.018, x_center=0.04),
            _xy_section(-0.12, 0.15, 0.12, 0.020, x_center=0.05),
            _xy_section(-0.22, 0.12, 0.10, 0.017, x_center=0.08),
            _xy_section(-0.30, 0.13, 0.09, 0.015, x_center=0.09),
        ]
    )
    thigh_link.visual(
        mesh_from_geometry(thigh_shell, "thigh_link_shell"),
        material=carbon,
        name="thigh_shell",
    )
    thigh_link.visual(
        Cylinder(radius=0.028, length=0.10),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hip_axle_housing",
    )
    thigh_link.visual(
        Box((0.05, 0.08, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, -0.03)),
        material=graphite,
        name="hip_mount_block",
    )
    thigh_link.visual(
        Cylinder(radius=0.040, length=0.03),
        origin=Origin(xyz=(0.09, 0.055, -0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="knee_actuator_left",
    )
    thigh_link.visual(
        Cylinder(radius=0.040, length=0.03),
        origin=Origin(xyz=(0.09, -0.055, -0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="knee_actuator_right",
    )
    thigh_link.visual(
        Box((0.07, 0.08, 0.18)),
        origin=Origin(xyz=(0.10, 0.0, -0.19)),
        material=accent,
        name="thigh_service_cover",
    )

    shank_link = model.part("shank_link")
    shank_link.inertial = Inertial.from_geometry(
        Box((0.20, 0.14, 0.33)),
        mass=8.0,
        origin=Origin(xyz=(0.04, 0.0, -0.16)),
    )
    shank_shell = section_loft(
        [
            _xy_section(-0.05, 0.11, 0.08, 0.014, x_center=0.04),
            _xy_section(-0.14, 0.11, 0.08, 0.014, x_center=0.05),
            _xy_section(-0.22, 0.10, 0.08, 0.013, x_center=0.07),
            _xy_section(-0.26, 0.11, 0.09, 0.014, x_center=0.09),
        ]
    )
    shank_link.visual(
        mesh_from_geometry(shank_shell, "shank_link_shell"),
        material=graphite,
        name="shank_shell",
    )
    shank_link.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="knee_axle_housing",
    )
    shank_link.visual(
        Box((0.05, 0.07, 0.05)),
        origin=Origin(xyz=(0.04, 0.0, -0.03)),
        material=graphite,
        name="knee_mount_block",
    )
    shank_link.visual(
        Cylinder(radius=0.032, length=0.025),
        origin=Origin(xyz=(0.10, 0.045, -0.30), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="ankle_actuator_left",
    )
    shank_link.visual(
        Cylinder(radius=0.032, length=0.025),
        origin=Origin(xyz=(0.10, -0.045, -0.30), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="ankle_actuator_right",
    )
    shank_link.visual(
        Box((0.04, 0.02, 0.09)),
        origin=Origin(xyz=(0.085, 0.045, -0.255)),
        material=graphite,
        name="ankle_cheek_left",
    )
    shank_link.visual(
        Box((0.04, 0.02, 0.09)),
        origin=Origin(xyz=(0.085, -0.045, -0.255)),
        material=graphite,
        name="ankle_cheek_right",
    )
    shank_link.visual(
        Box((0.06, 0.07, 0.14)),
        origin=Origin(xyz=(0.10, 0.0, -0.15)),
        material=accent,
        name="shank_access_panel",
    )

    ankle_foot = model.part("ankle_foot")
    ankle_foot.inertial = Inertial.from_geometry(
        Box((0.30, 0.14, 0.16)),
        mass=4.5,
        origin=Origin(xyz=(0.09, 0.0, -0.06)),
    )
    foot_shell = section_loft(
        [
            _yz_section(0.03, 0.09, 0.07, 0.012, z_center=-0.03),
            _yz_section(0.12, 0.11, 0.07, 0.012, z_center=-0.05),
            _yz_section(0.20, 0.11, 0.05, 0.010, z_center=-0.06),
            _yz_section(0.27, 0.08, 0.03, 0.008, z_center=-0.06),
        ]
    )
    ankle_foot.visual(
        mesh_from_geometry(foot_shell, "ankle_foot_shell"),
        material=carbon,
        name="foot_shell",
    )
    ankle_foot.visual(
        Cylinder(radius=0.022, length=0.056),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="ankle_axle_housing",
    )
    ankle_foot.visual(
        Box((0.05, 0.07, 0.04)),
        origin=Origin(xyz=(0.03, 0.0, -0.02)),
        material=graphite,
        name="ankle_mount_block",
    )
    ankle_foot.visual(
        Box((0.22, 0.10, 0.022)),
        origin=Origin(xyz=(0.15, 0.0, -0.095)),
        material=rubber,
        name="sole_pad",
    )
    ankle_foot.visual(
        Box((0.06, 0.06, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, -0.03)),
        material=accent,
        name="heel_block",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg_housing,
        child=thigh_link,
        origin=Origin(xyz=(0.20, 0.0, 0.76)),
        # The thigh shell hangs mostly along local -Z, so -Y makes positive
        # motion swing the leg forward toward +X.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=2.6,
            lower=-0.75,
            upper=0.95,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.09, 0.0, -0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=3.0,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=ankle_foot,
        origin=Origin(xyz=(0.10, 0.0, -0.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=3.0,
            lower=-0.65,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("upper_leg_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.check(
        "hip joint sits well above ground line",
        hip.origin.xyz[2] >= 0.70,
        details=f"hip_z={hip.origin.xyz[2]:.3f}",
    )
    ctx.expect_origin_gap(
        thigh,
        shank,
        axis="z",
        min_gap=0.22,
        name="knee origin is well below the hip origin",
    )
    ctx.expect_origin_gap(
        shank,
        foot,
        axis="z",
        min_gap=0.12,
        name="ankle origin is below the knee origin",
    )
    foot_aabb = ctx.part_world_aabb(foot)
    ctx.check(
        "foot keeps visible ground clearance",
        foot_aabb is not None and foot_aabb[0][2] >= 0.01,
        details=f"foot_aabb={foot_aabb}",
    )

    rest_foot_pos = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.10, ankle: -0.30}):
        flexed_foot_pos = ctx.part_world_position(foot)
        ctx.check(
            "knee flexion tucks the foot rearward and upward",
            rest_foot_pos is not None
            and flexed_foot_pos is not None
            and flexed_foot_pos[0] > rest_foot_pos[0] + 0.10
            and flexed_foot_pos[2] > rest_foot_pos[2] + 0.05,
            details=f"rest={rest_foot_pos}, flexed={flexed_foot_pos}",
        )

    with ctx.pose({hip: 0.55}):
        pitched_foot_pos = ctx.part_world_position(foot)
        ctx.check(
            "hip pitch swings the leg forward",
            rest_foot_pos is not None
            and pitched_foot_pos is not None
            and pitched_foot_pos[0] > rest_foot_pos[0] + 0.12,
            details=f"rest={rest_foot_pos}, pitched={pitched_foot_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

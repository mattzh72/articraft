from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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


THIGH_LENGTH = 0.36
SHANK_LENGTH = 0.34

HIP_GAP = 0.074
HIP_PLATE_THICK = 0.012
HIP_PLATE_Y = HIP_GAP * 0.5 + HIP_PLATE_THICK * 0.5

KNEE_GAP = 0.060
KNEE_PLATE_THICK = 0.010
KNEE_PLATE_Y = KNEE_GAP * 0.5 + KNEE_PLATE_THICK * 0.5

ANKLE_GAP = 0.052
ANKLE_PLATE_THICK = 0.009
ANKLE_PLATE_Y = ANKLE_GAP * 0.5 + ANKLE_PLATE_THICK * 0.5


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_rr_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_rr_section(
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_robotic_leg")

    shell_gray = model.material("shell_gray", rgba=(0.76, 0.79, 0.82, 1.0))
    cover_dark = model.material("cover_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.34, 0.36, 0.40, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))

    hip_module = model.part("hip_module")
    hip_module.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.22)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    hip_housing = section_loft(
        [
            _xy_rr_section(0.14, 0.10, 0.018, 0.04),
            _xy_rr_section(0.17, 0.12, 0.020, 0.10),
            _xy_rr_section(0.12, 0.09, 0.016, 0.15),
        ]
    )
    hip_module.visual(_mesh("hip_housing", hip_housing), material=shell_gray, name="hip_housing")
    hip_module.visual(
        Box((0.10, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=shell_gray,
        name="torso_mount_flange",
    )
    hip_module.visual(
        Box((0.055, HIP_PLATE_THICK, 0.08)),
        origin=Origin(xyz=(0.0, HIP_PLATE_Y, 0.0)),
        material=steel_dark,
        name="left_hip_plate",
    )
    hip_module.visual(
        Box((0.055, HIP_PLATE_THICK, 0.08)),
        origin=Origin(xyz=(0.0, -HIP_PLATE_Y, 0.0)),
        material=steel_dark,
        name="right_hip_plate",
    )
    hip_module.visual(
        Box((0.014, HIP_GAP + 2.0 * HIP_PLATE_THICK, 0.05)),
        origin=Origin(xyz=(0.062, 0.0, 0.015)),
        material=steel_dark,
        name="front_gusset",
    )
    hip_module.visual(
        Box((0.014, HIP_GAP + 2.0 * HIP_PLATE_THICK, 0.05)),
        origin=Origin(xyz=(-0.062, 0.0, 0.015)),
        material=steel_dark,
        name="rear_gusset",
    )
    for side, y in (("left", HIP_PLATE_Y + 0.010), ("right", -(HIP_PLATE_Y + 0.010))):
        hip_module.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name=f"{side}_hip_clamp_bolt",
        )
    for ix, x in enumerate((-0.030, 0.030), start=1):
        for iy, y in enumerate((-0.050, 0.050), start=1):
            hip_module.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.162)),
                material=steel_dark,
                name=f"mount_bolt_{ix}_{iy}",
            )

    thigh_module = model.part("thigh_module")
    thigh_module.inertial = Inertial.from_geometry(
        Box((0.16, 0.10, THIGH_LENGTH)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -THIGH_LENGTH * 0.5)),
    )
    thigh_shell = section_loft(
        [
            _xy_rr_section(0.13, 0.080, 0.016, -0.05),
            _xy_rr_section(0.12, 0.076, 0.014, -0.16),
            _xy_rr_section(0.10, 0.070, 0.012, -0.30),
            _xy_rr_section(0.092, 0.068, 0.011, -0.334),
        ]
    )
    thigh_module.visual(_mesh("thigh_shell", thigh_shell), material=shell_gray, name="thigh_shell")
    thigh_module.visual(
        Cylinder(radius=0.025, length=HIP_GAP),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="hip_trunnion",
    )
    thigh_module.visual(
        Box((0.110, 0.050, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=steel_dark,
        name="hip_drive_block",
    )
    for side, x in (("front", 0.060), ("rear", -0.060)):
        thigh_module.visual(
            Box((0.008, 0.050, 0.180)),
            origin=Origin(xyz=(x, 0.0, -0.180)),
            material=cover_dark,
            name=f"{side}_thigh_service_cover",
        )
        bolt_x = x + (0.004 if x > 0.0 else -0.004)
        for z in (-0.230, -0.130):
            for y in (-0.018, 0.018):
                thigh_module.visual(
                    Cylinder(radius=0.004, length=0.004),
                    origin=Origin(
                        xyz=(bolt_x, y, z),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material=steel_dark,
                    name=f"{side}_thigh_bolt_{int((z + 0.30) * 1000)}_{int((y + 0.03) * 1000)}",
                )
    thigh_module.visual(
        Box((0.086, KNEE_GAP + 2.0 * KNEE_PLATE_THICK, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.317)),
        material=steel_dark,
        name="knee_bridge",
    )
    thigh_module.visual(
        Box((0.035, KNEE_PLATE_THICK, 0.062)),
        origin=Origin(xyz=(0.0, KNEE_PLATE_Y, -THIGH_LENGTH + 0.001)),
        material=steel_dark,
        name="left_knee_plate",
    )
    thigh_module.visual(
        Box((0.035, KNEE_PLATE_THICK, 0.062)),
        origin=Origin(xyz=(0.0, -KNEE_PLATE_Y, -THIGH_LENGTH + 0.001)),
        material=steel_dark,
        name="right_knee_plate",
    )
    for side, y in (("left", KNEE_PLATE_Y + 0.007), ("right", -(KNEE_PLATE_Y + 0.007))):
        thigh_module.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.0, y, -THIGH_LENGTH + 0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name=f"{side}_knee_clamp_bolt",
        )

    shank_module = model.part("shank_module")
    shank_module.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, SHANK_LENGTH)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, -SHANK_LENGTH * 0.5)),
    )
    shank_shell = section_loft(
        [
            _xy_rr_section(0.105, 0.070, 0.014, -0.05),
            _xy_rr_section(0.092, 0.066, 0.013, -0.16),
            _xy_rr_section(0.082, 0.060, 0.011, -0.28),
            _xy_rr_section(0.076, 0.058, 0.010, -0.304),
        ]
    )
    shank_module.visual(_mesh("shank_shell", shank_shell), material=shell_gray, name="shank_shell")
    shank_module.visual(
        Cylinder(radius=0.022, length=KNEE_GAP),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="knee_trunnion",
    )
    shank_module.visual(
        Box((0.095, 0.048, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=steel_dark,
        name="knee_drive_block",
    )
    for side, x in (("front", 0.048), ("rear", -0.048)):
        shank_module.visual(
            Box((0.008, 0.042, 0.160)),
            origin=Origin(xyz=(x, 0.0, -0.170)),
            material=cover_dark,
            name=f"{side}_shank_service_cover",
        )
        bolt_x = x + (0.004 if x > 0.0 else -0.004)
        for z in (-0.220, -0.120):
            for y in (-0.016, 0.016):
                shank_module.visual(
                    Cylinder(radius=0.0035, length=0.004),
                    origin=Origin(
                        xyz=(bolt_x, y, z),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material=steel_dark,
                    name=f"{side}_shank_bolt_{int((z + 0.30) * 1000)}_{int((y + 0.03) * 1000)}",
                )
    shank_module.visual(
        Box((0.072, ANKLE_GAP + 2.0 * ANKLE_PLATE_THICK, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.292)),
        material=steel_dark,
        name="ankle_bridge",
    )
    shank_module.visual(
        Box((0.030, ANKLE_PLATE_THICK, 0.052)),
        origin=Origin(xyz=(0.0, ANKLE_PLATE_Y, -SHANK_LENGTH + 0.012)),
        material=steel_dark,
        name="left_ankle_plate",
    )
    shank_module.visual(
        Box((0.030, ANKLE_PLATE_THICK, 0.052)),
        origin=Origin(xyz=(0.0, -ANKLE_PLATE_Y, -SHANK_LENGTH + 0.012)),
        material=steel_dark,
        name="right_ankle_plate",
    )
    for side, y in (("left", ANKLE_PLATE_Y + 0.005), ("right", -(ANKLE_PLATE_Y + 0.005))):
        shank_module.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(
                xyz=(0.0, y, -SHANK_LENGTH + 0.012),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel_dark,
            name=f"{side}_ankle_clamp_bolt",
        )

    foot_module = model.part("foot_module")
    foot_module.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.08)),
        mass=1.8,
        origin=Origin(xyz=(0.11, 0.0, -0.040)),
    )
    foot_shell = section_loft(
        [
            _yz_rr_section(0.040, 0.028, 0.008, -0.025, z_center=-0.036),
            _yz_rr_section(0.050, 0.042, 0.010, 0.030, z_center=-0.042),
            _yz_rr_section(0.074, 0.050, 0.012, 0.120, z_center=-0.046),
            _yz_rr_section(0.052, 0.030, 0.009, 0.220, z_center=-0.040),
        ]
    )
    foot_module.visual(_mesh("foot_shell", foot_shell), material=shell_gray, name="foot_shell")
    foot_module.visual(
        Cylinder(radius=0.018, length=ANKLE_GAP),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="ankle_barrel",
    )
    foot_module.visual(
        Box((0.040, 0.034, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=steel_dark,
        name="ankle_core",
    )
    foot_module.visual(
        Box((0.070, 0.064, 0.014)),
        origin=Origin(xyz=(0.190, 0.0, -0.060)),
        material=rubber_black,
        name="toe_pad",
    )
    foot_module.visual(
        Box((0.050, 0.058, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, -0.057)),
        material=rubber_black,
        name="heel_pad",
    )
    foot_module.visual(
        Box((0.028, 0.060, 0.016)),
        origin=Origin(xyz=(0.226, 0.0, -0.044)),
        material=cover_dark,
        name="toe_bumper",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_module,
        child=thigh_module,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=4.0,
            lower=math.radians(-45.0),
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_module,
        child=shank_module,
        origin=Origin(xyz=(0.0, 0.0, -THIGH_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_module,
        child=foot_module,
        origin=Origin(xyz=(0.0, 0.0, -SHANK_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=5.0,
            lower=math.radians(-25.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_module = object_model.get_part("hip_module")
    thigh_module = object_model.get_part("thigh_module")
    shank_module = object_model.get_part("shank_module")
    foot_module = object_model.get_part("foot_module")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(hip_module, thigh_module, name="hip joint carries thigh through clamped trunnion faces")
    ctx.expect_contact(thigh_module, shank_module, name="knee joint carries shank through clevis faces")
    ctx.expect_contact(shank_module, foot_module, name="ankle joint carries foot through clevis faces")
    ctx.expect_origin_gap(
        hip_module,
        foot_module,
        axis="z",
        min_gap=0.66,
        max_gap=0.74,
        name="standing leg has realistic hip to ankle drop",
    )
    ctx.expect_origin_gap(
        thigh_module,
        shank_module,
        axis="z",
        min_gap=0.34,
        max_gap=0.38,
        name="knee is positioned below hip by upper leg length",
    )

    hip_limits = hip_pitch.motion_limits
    knee_limits = knee_pitch.motion_limits
    ankle_limits = ankle_pitch.motion_limits
    ctx.check(
        "serial joint limits are plausible",
        hip_limits is not None
        and knee_limits is not None
        and ankle_limits is not None
        and hip_limits.lower is not None
        and hip_limits.upper is not None
        and knee_limits.lower == 0.0
        and knee_limits.upper is not None
        and ankle_limits.lower is not None
        and ankle_limits.upper is not None
        and hip_limits.lower < 0.0 < hip_limits.upper
        and math.radians(120.0) <= knee_limits.upper <= math.radians(145.0)
        and math.radians(-30.0) <= ankle_limits.lower <= math.radians(-20.0)
        and math.radians(35.0) <= ankle_limits.upper <= math.radians(45.0),
        "Expected hip, knee, and ankle limits to match a practical robotic leg pitch chain.",
    )

    with ctx.pose({hip_pitch: hip_limits.upper}):
        hip_pos = ctx.part_world_position(hip_module)
        foot_pos = ctx.part_world_position(foot_module)
        ctx.check(
            "positive hip pitch swings the leg forward",
            hip_pos is not None and foot_pos is not None and foot_pos[0] > hip_pos[0] + 0.25,
            f"Hip open pose should move the foot forward; hip={hip_pos}, foot={foot_pos}.",
        )

    with ctx.pose({knee_pitch: knee_limits.upper * 0.8}):
        thigh_pos = ctx.part_world_position(thigh_module)
        foot_pos = ctx.part_world_position(foot_module)
        ctx.check(
            "positive knee pitch folds the shank rearward",
            thigh_pos is not None
            and foot_pos is not None
            and foot_pos[0] < thigh_pos[0] - 0.10
            and foot_pos[2] > -0.58,
            f"Knee flex pose should pull the ankle back and upward; thigh={thigh_pos}, foot={foot_pos}.",
        )

    with ctx.pose({ankle_pitch: ankle_limits.lower}):
        dorsiflex_toe_aabb = ctx.part_element_world_aabb(foot_module, elem="toe_pad")
    with ctx.pose({ankle_pitch: ankle_limits.upper}):
        plantar_toe_aabb = ctx.part_element_world_aabb(foot_module, elem="toe_pad")
    ctx.check(
        "ankle pitch changes toe height in the correct direction",
        dorsiflex_toe_aabb is not None
        and plantar_toe_aabb is not None
        and plantar_toe_aabb[0][2] < dorsiflex_toe_aabb[0][2] - 0.05,
        f"Expected plantarflexed toe to sit lower than dorsiflexed toe; dorsiflex={dorsiflex_toe_aabb}, plantar={plantar_toe_aabb}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

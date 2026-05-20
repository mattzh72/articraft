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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    support_gray = model.material("support_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.46, 0.49, 0.53, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    def _xy_section(width: float, depth: float, radius: float, z: float):
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]

    thigh_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.075, 0.060, 0.012, -0.020),
                _xy_section(0.110, 0.080, 0.016, -0.130),
                _xy_section(0.095, 0.072, 0.014, -0.255),
            ]
        ),
        "thigh_shell",
    )
    shank_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.062, 0.052, 0.010, -0.015),
                _xy_section(0.084, 0.070, 0.014, -0.125),
                _xy_section(0.066, 0.056, 0.011, -0.230),
            ]
        ),
        "shank_shell",
    )

    support = model.part("support_housing")
    support.visual(
        Box((0.180, 0.120, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=panel_gray,
        name="mount_plate",
    )
    support.visual(
        Box((0.120, 0.100, 0.220)),
        origin=Origin(xyz=(0.000, 0.000, 0.132)),
        material=support_gray,
        name="main_column",
    )
    support.visual(
        Box((0.070, 0.108, 0.128)),
        origin=Origin(xyz=(-0.060, 0.000, 0.120)),
        material=panel_gray,
        name="rear_drive_pack",
    )
    support.visual(
        Box((0.100, 0.045, 0.090)),
        origin=Origin(xyz=(0.015, 0.0225, 0.225)),
        material=support_gray,
        name="hip_carrier",
    )
    support.visual(
        Cylinder(radius=0.028, length=0.042),
        origin=Origin(xyz=(0.020, 0.021, 0.225), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hip_output_cartridge",
    )
    support.visual(
        Box((0.060, 0.040, 0.055)),
        origin=Origin(xyz=(0.045, 0.028, 0.182)),
        material=dark_metal,
        name="hip_motor_pod",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.200, 0.120, 0.270)),
        mass=18.0,
        origin=Origin(xyz=(0.000, 0.000, 0.135)),
    )

    thigh = model.part("thigh_link")
    thigh.visual(
        Cylinder(radius=0.033, length=0.028),
        origin=Origin(xyz=(0.000, 0.014, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hip_hub",
    )
    thigh.visual(
        Box((0.060, 0.050, 0.070)),
        origin=Origin(xyz=(0.020, 0.042, -0.018)),
        material=dark_metal,
        name="hip_knuckle",
    )
    thigh.visual(
        thigh_mesh,
        origin=Origin(xyz=(0.030, 0.070, 0.000)),
        material=panel_gray,
        name="thigh_shell",
    )
    thigh.visual(
        Box((0.040, 0.020, 0.080)),
        origin=Origin(xyz=(0.030, 0.010, -0.280)),
        material=dark_metal,
        name="knee_mount_block",
    )
    thigh.visual(
        Box((0.046, 0.040, 0.030)),
        origin=Origin(xyz=(0.030, 0.048, -0.245)),
        material=dark_metal,
        name="knee_bridge_web",
    )
    thigh.visual(
        Box((0.040, 0.014, 0.014)),
        origin=Origin(xyz=(0.030, 0.024, -0.247)),
        material=dark_metal,
        name="knee_outer_link",
    )
    thigh.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.030, 0.011, -0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="knee_trunnion",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.130, 0.110, 0.340)),
        mass=7.5,
        origin=Origin(xyz=(0.030, 0.060, -0.160)),
    )

    shank = model.part("shank_link")
    shank.visual(
        Cylinder(radius=0.029, length=0.022),
        origin=Origin(xyz=(0.000, 0.011, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="knee_hub",
    )
    shank.visual(
        Box((0.050, 0.042, 0.060)),
        origin=Origin(xyz=(0.010, 0.048, -0.034)),
        material=dark_metal,
        name="upper_shank_knuckle",
    )
    shank.visual(
        Box((0.028, 0.022, 0.038)),
        origin=Origin(xyz=(0.008, 0.021, -0.020)),
        material=dark_metal,
        name="knee_bridge",
    )
    shank.visual(
        shank_mesh,
        origin=Origin(xyz=(0.018, 0.062, 0.000)),
        material=panel_gray,
        name="shank_shell",
    )
    shank.visual(
        Box((0.038, 0.018, 0.068)),
        origin=Origin(xyz=(0.010, 0.009, -0.244)),
        material=dark_metal,
        name="ankle_mount_block",
    )
    shank.visual(
        Box((0.042, 0.034, 0.026)),
        origin=Origin(xyz=(0.010, 0.040, -0.220)),
        material=dark_metal,
        name="ankle_bridge_web",
    )
    shank.visual(
        Box((0.036, 0.010, 0.018)),
        origin=Origin(xyz=(0.010, 0.021, -0.236)),
        material=dark_metal,
        name="ankle_outer_link",
    )
    shank.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.010, 0.010, -0.270), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="ankle_trunnion",
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.110, 0.100, 0.300)),
        mass=5.2,
        origin=Origin(xyz=(0.016, 0.055, -0.145)),
    )

    foot = model.part("ankle_foot")
    foot.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.000, 0.010, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.055, 0.045, 0.060)),
        origin=Origin(xyz=(0.015, 0.040, -0.030)),
        material=dark_metal,
        name="ankle_block",
    )
    foot.visual(
        Box((0.040, 0.042, 0.020)),
        origin=Origin(xyz=(-0.012, 0.040, -0.052)),
        material=panel_gray,
        name="heel_pad",
    )
    foot.visual(
        Box((0.112, 0.070, 0.030)),
        origin=Origin(xyz=(0.060, 0.055, -0.052)),
        material=panel_gray,
        name="midfoot_shell",
    )
    foot.visual(
        Box((0.088, 0.060, 0.022)),
        origin=Origin(xyz=(0.144, 0.055, -0.046)),
        material=panel_gray,
        name="toe_shell",
    )
    foot.visual(
        Cylinder(radius=0.011, length=0.060),
        origin=Origin(xyz=(0.188, 0.055, -0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="toe_tip",
    )
    foot.visual(
        Box((0.185, 0.074, 0.008)),
        origin=Origin(xyz=(0.092, 0.055, -0.067)),
        material=dark_metal,
        name="sole_pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.235, 0.090, 0.085)),
        mass=2.6,
        origin=Origin(xyz=(0.085, 0.050, -0.040)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=support,
        child=thigh,
        origin=Origin(xyz=(0.020, 0.050, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.5,
            lower=math.radians(-55.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.030, 0.022, -0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.010, 0.020, -0.270)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=3.5,
            lower=math.radians(-40.0),
            upper=math.radians(30.0),
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

    support = object_model.get_part("support_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_gap(
        thigh,
        support,
        axis="y",
        positive_elem="hip_hub",
        negative_elem="hip_carrier",
        min_gap=0.004,
        max_gap=0.008,
        name="thigh chain sits outboard of the fixed support",
    )
    ctx.expect_gap(
        shank,
        support,
        axis="y",
        positive_elem="shank_shell",
        negative_elem="main_column",
        min_gap=0.020,
        name="lower chain stays offset to one side of the support",
    )
    ctx.expect_origin_gap(
        thigh,
        support,
        axis="z",
        min_gap=0.18,
        name="hip axis is elevated on the support housing",
    )
    ctx.expect_origin_gap(
        thigh,
        shank,
        axis="z",
        min_gap=0.24,
        name="knee is below the hip axis",
    )
    ctx.expect_origin_gap(
        shank,
        foot,
        axis="z",
        min_gap=0.22,
        name="ankle is below the knee axis",
    )

    rest_aabb = ctx.part_element_world_aabb(thigh, elem="knee_trunnion")
    with ctx.pose({hip: math.radians(30.0)}):
        flexed_aabb = ctx.part_element_world_aabb(thigh, elem="knee_trunnion")
    ctx.check(
        "positive hip motion swings the thigh forward",
        rest_aabb is not None
        and flexed_aabb is not None
        and ((flexed_aabb[0][0] + flexed_aabb[1][0]) * 0.5) > ((rest_aabb[0][0] + rest_aabb[1][0]) * 0.5) + 0.02,
        details=f"rest={rest_aabb}, flexed={flexed_aabb}",
    )
    rest_ankle = ctx.part_element_world_aabb(shank, elem="ankle_trunnion")
    with ctx.pose({knee: math.radians(80.0)}):
        flexed_ankle = ctx.part_element_world_aabb(shank, elem="ankle_trunnion")
    ctx.check(
        "positive knee motion draws the ankle forward",
        rest_ankle is not None
        and flexed_ankle is not None
        and ((flexed_ankle[0][0] + flexed_ankle[1][0]) * 0.5) > ((rest_ankle[0][0] + rest_ankle[1][0]) * 0.5) + 0.10,
        details=f"rest={rest_ankle}, flexed={flexed_ankle}",
    )
    rest_toe = ctx.part_element_world_aabb(foot, elem="toe_tip")
    with ctx.pose({ankle: math.radians(20.0)}):
        dorsiflexed_toe = ctx.part_element_world_aabb(foot, elem="toe_tip")
    ctx.check(
        "positive ankle motion lifts the toe",
        rest_toe is not None
        and dorsiflexed_toe is not None
        and ((dorsiflexed_toe[0][2] + dorsiflexed_toe[1][2]) * 0.5) > ((rest_toe[0][2] + rest_toe[1][2]) * 0.5) + 0.04,
        details=f"rest={rest_toe}, dorsiflexed={dorsiflexed_toe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

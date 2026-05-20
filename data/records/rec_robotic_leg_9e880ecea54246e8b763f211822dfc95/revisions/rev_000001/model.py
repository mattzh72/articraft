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
)


def _lateral_cylinder(radius: float, length: float):
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    housing_dark = model.material("housing_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    link_gray = model.material("link_gray", rgba=(0.46, 0.49, 0.53, 1.0))
    machined = model.material("machined", rgba=(0.73, 0.76, 0.80, 1.0))
    sole_dark = model.material("sole_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    sensor_blue = model.material("sensor_blue", rgba=(0.22, 0.40, 0.50, 1.0))

    housing = model.part("upper_leg_housing")
    housing.visual(
        Box((0.160, 0.100, 0.110)),
        origin=Origin(xyz=(0.000, 0.000, 0.075)),
        material=housing_dark,
        name="main_core",
    )
    housing.visual(
        Box((0.188, 0.132, 0.032)),
        origin=Origin(xyz=(0.000, 0.000, 0.133)),
        material=panel_dark,
        name="top_cover",
    )
    housing.visual(
        Box((0.096, 0.108, 0.020)),
        origin=Origin(xyz=(-0.018, 0.000, 0.038)),
        material=housing_dark,
        name="hip_saddle",
    )
    housing.visual(
        Box((0.040, 0.028, 0.084)),
        origin=Origin(xyz=(0.006, 0.056, 0.004)),
        material=panel_dark,
        name="left_hip_cheek",
    )
    housing.visual(
        Box((0.040, 0.028, 0.084)),
        origin=Origin(xyz=(0.006, -0.056, 0.004)),
        material=panel_dark,
        name="right_hip_cheek",
    )
    hip_pod_geom, hip_pod_origin = _lateral_cylinder(0.040, 0.044)
    housing.visual(
        hip_pod_geom,
        origin=Origin(xyz=(0.012, 0.066, 0.038), rpy=hip_pod_origin.rpy),
        material=housing_dark,
        name="left_motor_pod",
    )
    housing.visual(
        hip_pod_geom,
        origin=Origin(xyz=(0.012, -0.066, 0.038), rpy=hip_pod_origin.rpy),
        material=housing_dark,
        name="right_motor_pod",
    )
    housing.visual(
        Box((0.082, 0.080, 0.030)),
        origin=Origin(xyz=(-0.048, 0.000, 0.050)),
        material=panel_dark,
        name="rear_pack",
    )
    housing.visual(
        Box((0.058, 0.078, 0.014)),
        origin=Origin(xyz=(0.030, 0.000, 0.090)),
        material=sensor_blue,
        name="front_service_panel",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.188, 0.148, 0.150)),
        mass=8.5,
        origin=Origin(xyz=(0.000, 0.000, 0.070)),
    )

    thigh = model.part("thigh_link")
    hip_spool_geom, hip_spool_origin = _lateral_cylinder(0.018, 0.084)
    thigh.visual(
        hip_spool_geom,
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=hip_spool_origin.rpy),
        material=machined,
        name="hip_spool",
    )
    thigh.visual(
        Box((0.094, 0.066, 0.072)),
        origin=Origin(xyz=(0.000, 0.000, -0.043)),
        material=link_gray,
        name="hip_shoulder",
    )
    thigh.visual(
        Box((0.076, 0.058, 0.228)),
        origin=Origin(xyz=(0.000, 0.000, -0.178)),
        material=link_gray,
        name="thigh_beam",
    )
    thigh.visual(
        Box((0.028, 0.056, 0.170)),
        origin=Origin(xyz=(0.026, 0.000, -0.168)),
        material=panel_dark,
        name="front_rib",
    )
    thigh.visual(
        Box((0.086, 0.036, 0.048)),
        origin=Origin(xyz=(0.000, 0.000, -0.255)),
        material=panel_dark,
        name="knee_bridge",
    )
    thigh.visual(
        Box((0.028, 0.018, 0.068)),
        origin=Origin(xyz=(0.000, 0.031, -0.270)),
        material=link_gray,
        name="left_knee_ear",
    )
    thigh.visual(
        Box((0.028, 0.018, 0.068)),
        origin=Origin(xyz=(0.000, -0.031, -0.270)),
        material=link_gray,
        name="right_knee_ear",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.110, 0.072, 0.320)),
        mass=4.2,
        origin=Origin(xyz=(0.000, 0.000, -0.160)),
    )

    shank = model.part("shank_link")
    knee_spool_geom, knee_spool_origin = _lateral_cylinder(0.017, 0.044)
    shank.visual(
        knee_spool_geom,
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=knee_spool_origin.rpy),
        material=machined,
        name="knee_spool",
    )
    shank.visual(
        Box((0.088, 0.056, 0.054)),
        origin=Origin(xyz=(0.000, 0.000, -0.036)),
        material=link_gray,
        name="knee_shoulder",
    )
    shank.visual(
        Box((0.070, 0.050, 0.220)),
        origin=Origin(xyz=(0.000, 0.000, -0.172)),
        material=link_gray,
        name="shank_beam",
    )
    shank.visual(
        Box((0.022, 0.048, 0.150)),
        origin=Origin(xyz=(-0.024, 0.000, -0.162)),
        material=panel_dark,
        name="rear_spine",
    )
    shank.visual(
        Box((0.078, 0.034, 0.042)),
        origin=Origin(xyz=(0.000, 0.000, -0.252)),
        material=panel_dark,
        name="ankle_bridge",
    )
    shank.visual(
        Box((0.026, 0.016, 0.060)),
        origin=Origin(xyz=(0.000, 0.029, -0.264)),
        material=link_gray,
        name="left_ankle_ear",
    )
    shank.visual(
        Box((0.026, 0.016, 0.060)),
        origin=Origin(xyz=(0.000, -0.029, -0.264)),
        material=link_gray,
        name="right_ankle_ear",
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.095, 0.062, 0.305)),
        mass=3.2,
        origin=Origin(xyz=(0.000, 0.000, -0.155)),
    )

    foot = model.part("ankle_foot")
    ankle_spool_geom, ankle_spool_origin = _lateral_cylinder(0.014, 0.042)
    foot.visual(
        ankle_spool_geom,
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=ankle_spool_origin.rpy),
        material=machined,
        name="ankle_spool",
    )
    foot.visual(
        Box((0.082, 0.066, 0.048)),
        origin=Origin(xyz=(-0.004, 0.000, -0.030)),
        material=link_gray,
        name="ankle_case",
    )
    foot.visual(
        Box((0.164, 0.060, 0.040)),
        origin=Origin(xyz=(0.046, 0.000, -0.070)),
        material=panel_dark,
        name="foot_body",
    )
    foot.visual(
        Box((0.062, 0.056, 0.024)),
        origin=Origin(xyz=(0.108, 0.000, -0.060)),
        material=link_gray,
        name="toe_block",
    )
    foot.visual(
        Box((0.052, 0.052, 0.018)),
        origin=Origin(xyz=(-0.040, 0.000, -0.082)),
        material=link_gray,
        name="heel_block",
    )
    foot.visual(
        Box((0.188, 0.066, 0.010)),
        origin=Origin(xyz=(0.038, 0.000, -0.092)),
        material=sole_dark,
        name="sole_plate",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.188, 0.070, 0.104)),
        mass=1.6,
        origin=Origin(xyz=(0.040, 0.000, -0.055)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=2.3,
            lower=-0.70,
            upper=1.10,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.000, 0.000, -0.310)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.6,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.000, 0.000, -0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=3.0,
            lower=-0.55,
            upper=0.65,
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

    ctx.expect_overlap(
        housing,
        thigh,
        axes="xy",
        min_overlap=0.055,
        name="hip link stays nested in the housing footprint",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="z",
        positive_elem="knee_bridge",
        negative_elem="knee_spool",
        min_gap=0.012,
        max_gap=0.016,
        name="knee bridge sits just above the knee spool",
    )
    ctx.expect_contact(
        shank,
        thigh,
        elem_a="knee_spool",
        elem_b="left_knee_ear",
        name="left knee ear captures the knee spool",
    )
    ctx.expect_contact(
        shank,
        thigh,
        elem_a="knee_spool",
        elem_b="right_knee_ear",
        name="right knee ear captures the knee spool",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="z",
        positive_elem="ankle_bridge",
        negative_elem="ankle_spool",
        min_gap=0.011,
        max_gap=0.016,
        name="ankle bridge sits just above the ankle spool",
    )
    ctx.expect_contact(
        foot,
        shank,
        elem_a="ankle_spool",
        elem_b="left_ankle_ear",
        name="left ankle ear captures the ankle spool",
    )
    ctx.expect_contact(
        foot,
        shank,
        elem_a="ankle_spool",
        elem_b="right_ankle_ear",
        name="right ankle ear captures the ankle spool",
    )
    ctx.expect_overlap(
        thigh,
        shank,
        axes="xy",
        min_overlap=0.040,
        name="thigh and shank stay laterally aligned through the knee",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return (
            (mn[0] + mx[0]) * 0.5,
            (mn[1] + mx[1]) * 0.5,
            (mn[2] + mx[2]) * 0.5,
        )

    rest_ankle_origin = ctx.part_world_position(foot)
    rest_toe_center = _aabb_center(ctx.part_element_world_aabb(foot, elem="toe_block"))

    with ctx.pose({hip: 0.75}):
        hip_swung_ankle_origin = ctx.part_world_position(foot)
    ctx.check(
        "hip swings the distal leg forward",
        rest_ankle_origin is not None
        and hip_swung_ankle_origin is not None
        and hip_swung_ankle_origin[0] > rest_ankle_origin[0] + 0.16
        and hip_swung_ankle_origin[2] > rest_ankle_origin[2] + 0.06,
        details=f"rest={rest_ankle_origin}, swung={hip_swung_ankle_origin}",
    )

    with ctx.pose({knee: 1.25}):
        knee_folded_ankle_origin = ctx.part_world_position(foot)
    ctx.check(
        "knee folds the ankle upward into a compact posture",
        rest_ankle_origin is not None
        and knee_folded_ankle_origin is not None
        and knee_folded_ankle_origin[2] > rest_ankle_origin[2] + 0.12
        and knee_folded_ankle_origin[0] < rest_ankle_origin[0] - 0.12,
        details=f"rest={rest_ankle_origin}, folded={knee_folded_ankle_origin}",
    )

    with ctx.pose({ankle: 0.45}):
        raised_toe_center = _aabb_center(ctx.part_element_world_aabb(foot, elem="toe_block"))
    ctx.check(
        "ankle pitches the toe upward",
        rest_toe_center is not None
        and raised_toe_center is not None
        and raised_toe_center[2] > rest_toe_center[2] + 0.025,
        details=f"rest_toe={rest_toe_center}, raised_toe={raised_toe_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

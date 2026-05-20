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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_link_mesh(name: str, size: tuple[float, float, float], radius: float):
    sx, sy, sz = size
    profile = rounded_rect_profile(sx, sy, radius=radius, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, sz, cap=True, center=True, closed=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.31, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.66, 0.70, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.09, 0.10, 0.11, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.20, 0.38, 0.48, 1.0))

    upper_housing_mesh = _rounded_link_mesh(
        "upper_leg_housing_shell",
        (0.24, 0.17, 0.14),
        radius=0.030,
    )
    thigh_mesh = _rounded_link_mesh(
        "thigh_fairing",
        (0.11, 0.11, 0.30),
        radius=0.020,
    )
    shank_mesh = _rounded_link_mesh(
        "shank_fairing",
        (0.09, 0.10, 0.21),
        radius=0.018,
    )
    ankle_body_mesh = _rounded_link_mesh(
        "ankle_body",
        (0.09, 0.10, 0.10),
        radius=0.016,
    )

    upper_leg_housing = model.part("upper_leg_housing")
    upper_leg_housing.visual(
        upper_housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=graphite,
        name="main_housing",
    )
    upper_leg_housing.visual(
        Box((0.16, 0.13, 0.05)),
        origin=Origin(xyz=(0.005, 0.0, 0.145)),
        material=gunmetal,
        name="service_cover",
    )
    upper_leg_housing.visual(
        Box((0.070, 0.022, 0.110)),
        origin=Origin(xyz=(0.005, 0.045, -0.035)),
        material=steel,
        name="hip_yoke_left",
    )
    upper_leg_housing.visual(
        Box((0.070, 0.022, 0.110)),
        origin=Origin(xyz=(0.005, -0.045, -0.035)),
        material=steel,
        name="hip_yoke_right",
    )
    upper_leg_housing.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.005, 0.060, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hip_axle_cap_left",
    )
    upper_leg_housing.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.005, -0.060, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hip_axle_cap_right",
    )
    upper_leg_housing.visual(
        Box((0.07, 0.18, 0.06)),
        origin=Origin(xyz=(0.065, 0.0, 0.03)),
        material=gunmetal,
        name="drive_pack",
    )
    upper_leg_housing.visual(
        Box((0.07, 0.006, 0.04)),
        origin=Origin(xyz=(0.092, 0.0, 0.105)),
        material=sensor_glass,
        name="status_window",
    )
    upper_leg_housing.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.18)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        thigh_mesh,
        origin=Origin(xyz=(0.03, 0.0, -0.225)),
        material=gunmetal,
        name="thigh_beam",
    )
    thigh_link.visual(
        Box((0.085, 0.068, 0.090)),
        origin=Origin(xyz=(0.005, 0.0, -0.045)),
        material=steel,
        name="hip_knuckle",
    )
    thigh_link.visual(
        Box((0.090, 0.018, 0.10)),
        origin=Origin(xyz=(0.04, 0.029, -0.420)),
        material=graphite,
        name="knee_clevis_left",
    )
    thigh_link.visual(
        Box((0.090, 0.018, 0.10)),
        origin=Origin(xyz=(0.04, -0.029, -0.420)),
        material=graphite,
        name="knee_clevis_right",
    )
    thigh_link.visual(
        Box((0.05, 0.09, 0.11)),
        origin=Origin(xyz=(0.075, 0.0, -0.26)),
        material=black_polymer,
        name="thigh_actuator_pod",
    )
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.14, 0.16, 0.46)),
        mass=6.0,
        origin=Origin(xyz=(0.035, 0.0, -0.20)),
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        shank_mesh,
        origin=Origin(xyz=(0.015, 0.0, -0.195)),
        material=gunmetal,
        name="shank_beam",
    )
    shank_link.visual(
        Box((0.072, 0.040, 0.090)),
        origin=Origin(xyz=(0.010, 0.0, -0.045)),
        material=steel,
        name="knee_knuckle",
    )
    shank_link.visual(
        Box((0.085, 0.022, 0.07)),
        origin=Origin(xyz=(0.03, 0.040, -0.335)),
        material=graphite,
        name="ankle_clevis_left",
    )
    shank_link.visual(
        Box((0.085, 0.022, 0.07)),
        origin=Origin(xyz=(0.03, -0.040, -0.335)),
        material=graphite,
        name="ankle_clevis_right",
    )
    shank_link.visual(
        Box((0.045, 0.08, 0.10)),
        origin=Origin(xyz=(0.055, 0.0, -0.20)),
        material=black_polymer,
        name="shank_actuator_pod",
    )
    shank_link.inertial = Inertial.from_geometry(
        Box((0.12, 0.14, 0.40)),
        mass=4.2,
        origin=Origin(xyz=(0.02, 0.0, -0.19)),
    )

    ankle_foot = model.part("ankle_foot")
    ankle_foot.visual(
        ankle_body_mesh,
        origin=Origin(xyz=(0.01, 0.0, -0.09)),
        material=graphite,
        name="ankle_housing",
    )
    ankle_foot.visual(
        Box((0.065, 0.058, 0.070)),
        origin=Origin(xyz=(0.01, 0.0, -0.015)),
        material=steel,
        name="ankle_knuckle",
    )
    ankle_foot.visual(
        Box((0.16, 0.11, 0.04)),
        origin=Origin(xyz=(0.045, 0.0, -0.09)),
        material=black_polymer,
        name="foot_pad",
    )
    ankle_foot.visual(
        Box((0.05, 0.08, 0.026)),
        origin=Origin(xyz=(0.105, 0.0, -0.068)),
        material=black_polymer,
        name="toe_cap",
    )
    ankle_foot.visual(
        Box((0.04, 0.07, 0.024)),
        origin=Origin(xyz=(-0.02, 0.0, -0.07)),
        material=black_polymer,
        name="heel_bumper",
    )
    ankle_foot.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.12)),
        mass=1.7,
        origin=Origin(xyz=(0.03, 0.0, -0.06)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg_housing,
        child=thigh_link,
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.5,
            lower=-0.9,
            upper=0.8,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.04, 0.0, -0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=3.2,
            lower=0.0,
            upper=2.1,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=ankle_foot,
        origin=Origin(xyz=(0.03, 0.0, -0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=3.8,
            lower=-0.7,
            upper=0.7,
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
    upper_leg_housing = object_model.get_part("upper_leg_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    ankle_foot = object_model.get_part("ankle_foot")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    for joint_name in ("hip_pitch", "knee_pitch", "ankle_pitch"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} uses a lateral pitch axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_origin_gap(
        upper_leg_housing,
        thigh_link,
        axis="z",
        min_gap=0.02,
        name="upper housing sits above the thigh root",
    )
    ctx.expect_origin_gap(
        thigh_link,
        shank_link,
        axis="z",
        min_gap=0.25,
        name="thigh root sits above the shank root",
    )
    ctx.expect_origin_gap(
        shank_link,
        ankle_foot,
        axis="z",
        min_gap=0.20,
        name="shank root sits above the ankle-foot root",
    )

    foot_aabb = ctx.part_world_aabb(ankle_foot)
    thigh_aabb = ctx.part_world_aabb(thigh_link)
    shank_aabb = ctx.part_world_aabb(shank_link)
    if foot_aabb and thigh_aabb and shank_aabb:
        foot_dims = [foot_aabb[1][i] - foot_aabb[0][i] for i in range(3)]
        thigh_dims = [thigh_aabb[1][i] - thigh_aabb[0][i] for i in range(3)]
        shank_dims = [shank_aabb[1][i] - shank_aabb[0][i] for i in range(3)]
        ctx.check(
            "ankle-foot stays compact relative to upstream links",
            foot_dims[2] < shank_dims[2] * 0.45 and foot_dims[0] < thigh_dims[2] * 0.55,
            details=f"foot_dims={foot_dims}, thigh_dims={thigh_dims}, shank_dims={shank_dims}",
        )
    else:
        ctx.fail("ankle-foot size measurement available", details="Missing world AABB for one or more parts")

    rest_foot_pos = ctx.part_world_position(ankle_foot)
    with ctx.pose({hip_pitch: 0.45, knee_pitch: 1.10, ankle_pitch: -0.25}):
        flexed_foot_pos = ctx.part_world_position(ankle_foot)
        ctx.check(
            "leg folds upward in a flexed pose",
            rest_foot_pos is not None
            and flexed_foot_pos is not None
            and flexed_foot_pos[2] > rest_foot_pos[2] + 0.12
            and abs(flexed_foot_pos[0] - rest_foot_pos[0]) > 0.10,
            details=f"rest={rest_foot_pos}, flexed={flexed_foot_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

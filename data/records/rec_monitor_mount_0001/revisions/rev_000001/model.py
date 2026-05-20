from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


# >>> USER_CODE_START
def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_mount", assets=ASSETS)

    model.material("powder_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("anodized_aluminum", rgba=(0.67, 0.69, 0.73, 1.0))
    model.material("satin_steel", rgba=(0.49, 0.50, 0.53, 1.0))
    model.material("polymer_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("accent_red", rgba=(0.72, 0.18, 0.15, 1.0))

    desk_clamp = model.part("desk_clamp")
    _add_box(desk_clamp, (0.10, 0.058, 0.012), (0.015, 0.0, 0.006), "powder_black")
    _add_box(desk_clamp, (0.030, 0.060, 0.030), (-0.028, 0.0, 0.015), "powder_black")
    _add_box(desk_clamp, (0.055, 0.058, 0.080), (-0.005, 0.0, -0.040), "powder_black")
    _add_box(desk_clamp, (0.020, 0.058, 0.170), (-0.028, 0.0, -0.055), "powder_black")
    _add_box(desk_clamp, (0.012, 0.058, 0.060), (0.041, 0.0, -0.024), "powder_black")
    _add_cylinder(
        desk_clamp,
        radius=0.008,
        length=0.075,
        xyz=(0.008, 0.0, -0.055),
        material="satin_steel",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _add_cylinder(
        desk_clamp,
        radius=0.022,
        length=0.012,
        xyz=(-0.036, 0.0, -0.055),
        material="polymer_dark",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _add_cylinder(
        desk_clamp,
        radius=0.015,
        length=0.008,
        xyz=(0.047, 0.0, -0.055),
        material="polymer_dark",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _add_cylinder(
        desk_clamp, radius=0.025, length=0.020, xyz=(-0.028, 0.0, 0.040), material="powder_black"
    )
    _add_cylinder(
        desk_clamp,
        radius=0.016,
        length=0.275,
        xyz=(-0.028, 0.0, 0.1775),
        material="anodized_aluminum",
    )
    _add_cylinder(
        desk_clamp, radius=0.019, length=0.030, xyz=(-0.028, 0.0, 0.055), material="satin_steel"
    )
    _add_cylinder(
        desk_clamp, radius=0.006, length=0.008, xyz=(0.024, 0.018, 0.012), material="satin_steel"
    )
    _add_cylinder(
        desk_clamp, radius=0.006, length=0.008, xyz=(0.024, -0.018, 0.012), material="satin_steel"
    )
    desk_clamp.inertial = Inertial.from_geometry(
        Box((0.110, 0.060, 0.455)),
        mass=3.8,
        origin=Origin(xyz=(0.010, 0.0, 0.088)),
    )

    lower_arm = model.part("lower_arm")
    _add_cylinder(
        lower_arm, radius=0.028, length=0.024, xyz=(0.0, 0.0, 0.012), material="anodized_aluminum"
    )
    _add_cylinder(
        lower_arm, radius=0.030, length=0.004, xyz=(0.0, 0.0, 0.026), material="accent_red"
    )
    _add_box(lower_arm, (0.190, 0.040, 0.018), (0.115, 0.0, 0.012), "anodized_aluminum")
    _add_box(lower_arm, (0.155, 0.034, 0.007), (0.115, 0.0, 0.0245), "polymer_dark")
    _add_box(lower_arm, (0.120, 0.022, 0.006), (0.115, 0.0, 0.0045), "satin_steel")
    _add_box(lower_arm, (0.038, 0.008, 0.042), (0.226, 0.017, 0.012), "anodized_aluminum")
    _add_box(lower_arm, (0.038, 0.008, 0.042), (0.226, -0.017, 0.012), "anodized_aluminum")
    _add_cylinder(
        lower_arm,
        radius=0.011,
        length=0.034,
        xyz=(0.226, 0.0, 0.012),
        material="satin_steel",
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_box(lower_arm, (0.032, 0.010, 0.010), (0.070, 0.0, 0.028), "polymer_dark")
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.245, 0.045, 0.034)),
        mass=1.2,
        origin=Origin(xyz=(0.122, 0.0, 0.016)),
    )

    upper_arm = model.part("upper_arm")
    _add_cylinder(
        upper_arm,
        radius=0.0105,
        length=0.026,
        xyz=(0.0, 0.0, 0.0),
        material="satin_steel",
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_box(upper_arm, (0.185, 0.038, 0.017), (0.0975, 0.0, 0.0), "anodized_aluminum")
    _add_box(upper_arm, (0.155, 0.032, 0.007), (0.0975, 0.0, 0.0115), "polymer_dark")
    _add_box(upper_arm, (0.100, 0.020, 0.005), (0.095, 0.0, -0.008), "satin_steel")
    _add_box(upper_arm, (0.030, 0.024, 0.018), (0.188, 0.0, 0.0), "anodized_aluminum")
    _add_cylinder(
        upper_arm, radius=0.022, length=0.022, xyz=(0.205, 0.0, 0.0), material="anodized_aluminum"
    )
    _add_box(upper_arm, (0.028, 0.010, 0.010), (0.066, 0.0, 0.021), "polymer_dark")
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.228, 0.042, 0.030)),
        mass=1.0,
        origin=Origin(xyz=(0.108, 0.0, 0.002)),
    )

    head = model.part("head")
    _add_cylinder(head, radius=0.020, length=0.022, xyz=(0.0, 0.0, 0.0), material="polymer_dark")
    _add_box(head, (0.042, 0.024, 0.018), (0.026, 0.0, 0.0), "polymer_dark")
    _add_box(head, (0.012, 0.042, 0.018), (0.046, 0.0, 0.0), "satin_steel")
    _add_box(head, (0.018, 0.010, 0.072), (0.055, 0.016, 0.0), "satin_steel")
    _add_box(head, (0.018, 0.010, 0.072), (0.055, -0.016, 0.0), "satin_steel")
    _add_box(head, (0.020, 0.030, 0.010), (0.018, 0.0, 0.014), "polymer_dark")
    head.inertial = Inertial.from_geometry(
        Box((0.084, 0.045, 0.075)),
        mass=0.45,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )

    vesa_plate = model.part("vesa_plate")
    _add_box(vesa_plate, (0.005, 0.125, 0.125), (0.0, 0.0, 0.0), "powder_black")
    _add_cylinder(
        vesa_plate,
        radius=0.010,
        length=0.034,
        xyz=(-0.004, 0.0, 0.0),
        material="satin_steel",
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_box(vesa_plate, (0.012, 0.070, 0.012), (0.006, 0.0, 0.0), "satin_steel")
    _add_box(vesa_plate, (0.012, 0.012, 0.088), (0.006, 0.024, 0.0), "satin_steel")
    _add_box(vesa_plate, (0.012, 0.012, 0.088), (0.006, -0.024, 0.0), "satin_steel")
    for y in (-0.0375, 0.0375):
        for z in (-0.0375, 0.0375):
            _add_cylinder(
                vesa_plate,
                radius=0.006,
                length=0.010,
                xyz=(0.006, y, z),
                material="satin_steel",
                rpy=(0.0, math.pi / 2.0, 0.0),
            )
    vesa_plate.inertial = Inertial.from_geometry(
        Box((0.022, 0.125, 0.125)),
        mass=0.55,
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_swivel",
        ArticulationType.REVOLUTE,
        parent="desk_clamp",
        child="lower_arm",
        origin=Origin(xyz=(-0.028, 0.0, 0.318)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.4, upper=2.4, effort=60.0, velocity=2.0),
    )
    model.articulation(
        "lower_to_upper_elbow",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_arm",
        origin=Origin(xyz=(0.226, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=45.0, velocity=2.0),
    )
    model.articulation(
        "upper_to_head_yaw",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="head",
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.6, upper=1.6, effort=18.0, velocity=2.5),
    )
    model.articulation(
        "head_to_vesa_tilt",
        ArticulationType.REVOLUTE,
        parent="head",
        child="vesa_plate",
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.9, effort=18.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "lower_arm", "upper_arm", reason="elbow clevis captures the upper arm pivot sleeve"
    )
    ctx.allow_overlap(
        "upper_arm", "head", reason="yaw bearing collar nests inside the front turret"
    )
    ctx.allow_overlap(
        "head", "vesa_plate", reason="screen tilt trunnion sits between the head yoke ears"
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_gap("lower_arm", "desk_clamp", axis="z", max_gap=0.01, max_penetration=0.008)
    ctx.expect_aabb_overlap("lower_arm", "desk_clamp", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("upper_arm", "lower_arm", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("head", "upper_arm", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("head", "vesa_plate", axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance("head", "vesa_plate", axes="xy", max_dist=0.09)
    ctx.expect_origin_gap("vesa_plate", "desk_clamp", axis="z", min_gap=0.02)

    ctx.expect_joint_motion_axis(
        "base_to_lower_swivel",
        "lower_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "lower_to_upper_elbow",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(lower_to_upper_elbow=1.2):
        ctx.expect_origin_gap("upper_arm", "lower_arm", axis="z", min_gap=0.01)
        ctx.expect_origin_gap("head", "lower_arm", axis="z", min_gap=0.05)
        ctx.expect_origin_gap("vesa_plate", "desk_clamp", axis="z", min_gap=0.10)

    with ctx.pose(base_to_lower_swivel=1.1):
        ctx.expect_aabb_gap("lower_arm", "desk_clamp", axis="z", max_gap=0.02, max_penetration=0.008)
        ctx.expect_origin_gap("vesa_plate", "desk_clamp", axis="z", min_gap=0.02)
        ctx.expect_origin_distance("head", "vesa_plate", axes="xy", max_dist=0.09)

    with ctx.pose(upper_to_head_yaw=1.2):
        ctx.expect_aabb_overlap("head", "upper_arm", axes="xy", min_overlap=0.01)
        ctx.expect_origin_distance("head", "vesa_plate", axes="xy", max_dist=0.09)

    with ctx.pose(head_to_vesa_tilt=0.8):
        ctx.expect_aabb_overlap("head", "vesa_plate", axes="xy", min_overlap=0.02)
        ctx.expect_origin_distance("head", "vesa_plate", axes="xy", max_dist=0.09)

    with ctx.pose(head_to_vesa_tilt=-0.5):
        ctx.expect_aabb_overlap("head", "vesa_plate", axes="xy", min_overlap=0.02)
        ctx.expect_origin_distance("head", "vesa_plate", axes="xy", max_dist=0.09)

    with ctx.pose({"base_to_lower_swivel": -1.0, "lower_to_upper_elbow": 1.0}):
        ctx.expect_origin_gap("head", "desk_clamp", axis="z", min_gap=0.06)
        ctx.expect_origin_gap("vesa_plate", "desk_clamp", axis="z", min_gap=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

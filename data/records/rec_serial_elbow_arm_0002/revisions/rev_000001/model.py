from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_SIZE = (0.090, 0.080, 0.045)
SHOULDER_AXIS_Z = 0.066
UPPER_ARM_LENGTH = 0.180
FOREARM_LENGTH = 0.145
LINK_WALL = 0.004
UPPER_ARM_SECTION = (0.038, 0.044)
FOREARM_SECTION = (0.034, 0.040)
SHOULDER_HUB_RADIUS = 0.022
SHOULDER_HUB_THICKNESS = 0.022
ELBOW_HUB_RADIUS = 0.019
ELBOW_HUB_THICKNESS = 0.020


def _box_section(
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .rect(outer_width, outer_height)
        .rect(outer_width - 2.0 * wall, outer_height - 2.0 * wall)
        .extrude(length)
    )


def _make_base_shape() -> cq.Workplane:
    pedestal = cq.Workplane("XY").box(*BASE_SIZE).translate((0.0, 0.0, BASE_SIZE[2] / 2.0))

    ear_height = 0.052
    ear_size = (0.024, 0.010, ear_height)
    ear_y = 0.019
    ear_z = BASE_SIZE[2] + ear_height / 2.0
    left_ear = cq.Workplane("XY").box(*ear_size).translate((0.0, ear_y, ear_z))
    right_ear = cq.Workplane("XY").box(*ear_size).translate((0.0, -ear_y, ear_z))

    shoulder_backbone = (
        cq.Workplane("XY").box(0.028, 0.050, 0.018).translate((-0.008, 0.0, BASE_SIZE[2] + 0.009))
    )

    shoulder_bore = (
        cq.Workplane("XZ").center(0.0, SHOULDER_AXIS_Z).circle(0.012).extrude(0.100, both=True)
    )

    return pedestal.union(left_ear).union(right_ear).union(shoulder_backbone).cut(shoulder_bore)


def _make_upper_arm_shape() -> cq.Workplane:
    root_blend_len = 0.024
    clevis_len = 0.024
    clevis_ear_thickness = 0.007
    clevis_gap = 0.026

    tube = _box_section(
        UPPER_ARM_LENGTH - root_blend_len - clevis_len,
        UPPER_ARM_SECTION[0],
        UPPER_ARM_SECTION[1],
        LINK_WALL,
    ).translate((root_blend_len, 0.0, 0.0))

    root_block = (
        cq.Workplane("XY")
        .box(root_blend_len, 0.030, 0.046)
        .translate((root_blend_len / 2.0, 0.0, 0.0))
    )
    root_hub = (
        cq.Workplane("XZ").circle(SHOULDER_HUB_RADIUS).extrude(SHOULDER_HUB_THICKNESS, both=True)
    )

    ear_y = clevis_gap / 2.0 + clevis_ear_thickness / 2.0
    left_clevis_ear = (
        cq.Workplane("XY")
        .box(clevis_len, clevis_ear_thickness, 0.048)
        .translate((UPPER_ARM_LENGTH - clevis_len / 2.0, ear_y, 0.0))
    )
    right_clevis_ear = (
        cq.Workplane("XY")
        .box(clevis_len, clevis_ear_thickness, 0.048)
        .translate((UPPER_ARM_LENGTH - clevis_len / 2.0, -ear_y, 0.0))
    )
    clevis_web = (
        cq.Workplane("XY")
        .box(0.014, 0.026, 0.030)
        .translate((UPPER_ARM_LENGTH - clevis_len - 0.007, 0.0, 0.0))
    )

    shoulder_bore = cq.Workplane("XZ").circle(0.010).extrude(0.060, both=True)
    elbow_bore = (
        cq.Workplane("XZ").center(UPPER_ARM_LENGTH, 0.0).circle(0.010).extrude(0.070, both=True)
    )

    return (
        root_hub.union(root_block)
        .union(tube)
        .union(clevis_web)
        .union(left_clevis_ear)
        .union(right_clevis_ear)
        .cut(shoulder_bore)
        .cut(elbow_bore)
    )


def _make_forearm_shape() -> cq.Workplane:
    root_blend_len = 0.020
    tip_block_len = 0.022

    tube = _box_section(
        FOREARM_LENGTH - root_blend_len - tip_block_len,
        FOREARM_SECTION[0],
        FOREARM_SECTION[1],
        LINK_WALL,
    ).translate((root_blend_len, 0.0, 0.0))

    root_block = (
        cq.Workplane("XY")
        .box(root_blend_len, 0.026, 0.042)
        .translate((root_blend_len / 2.0, 0.0, 0.0))
    )
    root_hub = cq.Workplane("XZ").circle(ELBOW_HUB_RADIUS).extrude(ELBOW_HUB_THICKNESS, both=True)

    tip_block = (
        cq.Workplane("XY")
        .box(tip_block_len, 0.030, 0.044)
        .translate((FOREARM_LENGTH - tip_block_len / 2.0, 0.0, 0.0))
    )
    pilot_face = (
        cq.Workplane("YZ")
        .rect(0.026, 0.032)
        .extrude(0.008)
        .translate((FOREARM_LENGTH - 0.008, 0.0, 0.0))
    )

    elbow_bore = cq.Workplane("XZ").circle(0.010).extrude(0.060, both=True)

    return root_hub.union(root_block).union(tube).union(tip_block).union(pilot_face).cut(elbow_bore)


def _make_tool_plate_shape() -> cq.Workplane:
    adapter = cq.Workplane("YZ").rect(0.028, 0.036).extrude(0.015)
    plate = cq.Workplane("YZ").rect(0.058, 0.074).extrude(0.006).translate((0.015, 0.0, 0.0))
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.036, 0.052, forConstruction=True)
        .vertices()
        .hole(0.006)
    )
    return adapter.union(plate)


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material_name: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_serial_elbow_arm", assets=ASSETS)

    model.material("base_gray", rgba=(0.36, 0.39, 0.43, 1.0))
    model.material("arm_blue", rgba=(0.18, 0.34, 0.72, 1.0))
    model.material("dark_metal", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("tool_silver", rgba=(0.72, 0.74, 0.78, 1.0))

    base = model.part("base")
    _add_visual_mesh(base, _make_base_shape(), "compact_elbow_arm_base.obj", "base_gray")


    base.inertial = Inertial.from_geometry(
        Box(BASE_SIZE),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    _add_visual_mesh(upper_arm, _make_upper_arm_shape(), "compact_elbow_arm_upper.obj", "arm_blue")



    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.038, 0.044)),
        mass=0.68,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    _add_visual_mesh(forearm, _make_forearm_shape(), "compact_elbow_arm_forearm.obj", "dark_metal")


    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, 0.034, 0.040)),
        mass=0.44,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    tool_plate = model.part("tool_plate")
    _add_visual_mesh(
        tool_plate, _make_tool_plate_shape(), "compact_elbow_arm_tool_plate.obj", "tool_silver"
    )


    tool_plate.inertial = Inertial.from_geometry(
        Box((0.021, 0.058, 0.074)),
        mass=0.12,
        origin=Origin(xyz=(0.0135, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.05, upper=1.25, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.10, effort=12.0, velocity=2.5),
    )
    model.articulation(
        "tool_plate_mount",
        ArticulationType.FIXED,
        parent=forearm,
        child=tool_plate,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("upper_arm", "base", axes="xy", min_overlap=0.010)
    ctx.expect_origin_distance("upper_arm", "base", axes="xy", max_dist=0.14)
    ctx.expect_origin_distance("forearm", "upper_arm", axes="xy", max_dist=0.20)
    ctx.expect_origin_distance("tool_plate", "forearm", axes="xy", max_dist=0.15)
    ctx.expect_joint_motion_axis(
        "shoulder_joint",
        "tool_plate",
        world_axis="z",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "elbow_joint",
        "tool_plate",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )

    with ctx.pose(shoulder_joint=0.95):
        ctx.expect_aabb_gap("forearm", "base", axis="z", max_gap=0.30, max_penetration=0.0)
        ctx.expect_aabb_gap("tool_plate", "base", axis="z", max_gap=0.40, max_penetration=0.0)

    with ctx.pose(elbow_joint=1.40):
        ctx.expect_aabb_gap("tool_plate", "upper_arm", axis="z", max_gap=0.14, max_penetration=0.0)

    with ctx.pose(shoulder_joint=0.75, elbow_joint=1.30):
        ctx.expect_origin_gap("tool_plate", "base", axis="z", min_gap=0.12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
HALF_PI = math.pi * 0.5

JOINT_BARREL_RADIUS = 0.0055
PIN_HOLE_RADIUS = 0.0022
BARREL_LENGTH = 0.0095
CLEVIS_EAR_THICKNESS = 0.0032
CLEVIS_GAP = 0.0106
EAR_OFFSET = CLEVIS_GAP * 0.5 + CLEVIS_EAR_THICKNESS * 0.5


def _add_visual_mesh(part, shape, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _joint_cylinder_collision(part, radius: float, length: float, x: float = 0.0) -> None:
    pass


def _make_base_shape() -> cq.Workplane:
    mount_block = cq.Workplane("XY").box(0.030, 0.022, 0.018).translate((-0.019, 0.0, 0.0))
    neck = cq.Workplane("XY").box(0.010, 0.014, 0.012).translate((-0.0045, 0.0, 0.0))
    ear_1 = (
        cq.Workplane("XZ")
        .circle(JOINT_BARREL_RADIUS * 1.05)
        .extrude(CLEVIS_EAR_THICKNESS * 0.5, both=True)
        .translate((0.0, EAR_OFFSET, 0.0))
    )
    ear_2 = (
        cq.Workplane("XZ")
        .circle(JOINT_BARREL_RADIUS * 1.05)
        .extrude(CLEVIS_EAR_THICKNESS * 0.5, both=True)
        .translate((0.0, -EAR_OFFSET, 0.0))
    )
    gusset_1 = (
        cq.Workplane("XY")
        .box(0.010, CLEVIS_EAR_THICKNESS, 0.013)
        .translate((-0.005, EAR_OFFSET, 0.0))
    )
    gusset_2 = (
        cq.Workplane("XY")
        .box(0.010, CLEVIS_EAR_THICKNESS, 0.013)
        .translate((-0.005, -EAR_OFFSET, 0.0))
    )
    joint_bore = cq.Workplane("XZ").circle(PIN_HOLE_RADIUS).extrude(0.030, both=True)
    mount_hole_1 = (
        cq.Workplane("XY").circle(0.0024).extrude(0.012, both=True).translate((-0.024, 0.0065, 0.0))
    )
    mount_hole_2 = (
        cq.Workplane("XY")
        .circle(0.0024)
        .extrude(0.012, both=True)
        .translate((-0.024, -0.0065, 0.0))
    )
    return (
        mount_block.union(neck)
        .union(ear_1)
        .union(ear_2)
        .union(gusset_1)
        .union(gusset_2)
        .cut(joint_bore)
        .cut(mount_hole_1)
        .cut(mount_hole_2)
    )


def _make_clevis_phalanx_shape(
    *,
    length: float,
    body_width: float,
    root_height: float,
    tip_height: float,
) -> cq.Workplane:
    body_start = JOINT_BARREL_RADIUS * 0.55
    mid_x = length * 0.62
    end_x = length - JOINT_BARREL_RADIUS * 1.15
    body = (
        cq.Workplane("YZ", origin=(body_start, 0.0, 0.0))
        .rect(body_width, root_height)
        .workplane(offset=mid_x - body_start)
        .rect(body_width * 0.90, root_height * 0.82)
        .workplane(offset=end_x - mid_x)
        .rect(body_width * 0.76, tip_height)
        .loft(combine=True)
    )
    spine = (
        cq.Workplane("XY")
        .box(length * 0.34, body_width * 0.34, root_height * 0.24)
        .translate((length * 0.28, 0.0, root_height * 0.17))
    )
    proximal_barrel = (
        cq.Workplane("XZ").circle(JOINT_BARREL_RADIUS).extrude(BARREL_LENGTH * 0.5, both=True)
    )
    bridge_len = max(length * 0.18, 0.009)
    lug_1 = (
        cq.Workplane("XY")
        .box(bridge_len, CLEVIS_EAR_THICKNESS, tip_height * 0.92)
        .translate((length - bridge_len * 0.5, EAR_OFFSET, 0.0))
    )
    lug_2 = (
        cq.Workplane("XY")
        .box(bridge_len, CLEVIS_EAR_THICKNESS, tip_height * 0.92)
        .translate((length - bridge_len * 0.5, -EAR_OFFSET, 0.0))
    )
    ear_1 = (
        cq.Workplane("XZ")
        .circle(JOINT_BARREL_RADIUS * 0.97)
        .extrude(CLEVIS_EAR_THICKNESS * 0.5, both=True)
        .translate((length, EAR_OFFSET, 0.0))
    )
    ear_2 = (
        cq.Workplane("XZ")
        .circle(JOINT_BARREL_RADIUS * 0.97)
        .extrude(CLEVIS_EAR_THICKNESS * 0.5, both=True)
        .translate((length, -EAR_OFFSET, 0.0))
    )
    proximal_bore = cq.Workplane("XZ").circle(PIN_HOLE_RADIUS).extrude(0.030, both=True)
    distal_bore = (
        cq.Workplane("XZ")
        .circle(PIN_HOLE_RADIUS)
        .extrude(0.030, both=True)
        .translate((length, 0.0, 0.0))
    )
    return (
        body.union(spine)
        .union(proximal_barrel)
        .union(lug_1)
        .union(lug_2)
        .union(ear_1)
        .union(ear_2)
        .cut(proximal_bore)
        .cut(distal_bore)
    )


def _make_terminal_phalanx_shape(
    *,
    length: float,
    body_width: float,
    root_height: float,
    tip_height: float,
) -> cq.Workplane:
    body_start = JOINT_BARREL_RADIUS * 0.55
    mid_x = length * 0.58
    nose_x = length * 0.88
    body = (
        cq.Workplane("YZ", origin=(body_start, 0.0, 0.0))
        .rect(body_width, root_height)
        .workplane(offset=mid_x - body_start)
        .rect(body_width * 0.86, root_height * 0.74)
        .workplane(offset=nose_x - mid_x)
        .rect(body_width * 0.58, tip_height)
        .workplane(offset=length - nose_x)
        .rect(body_width * 0.28, tip_height * 0.50)
        .loft(combine=True)
    )
    spine = (
        cq.Workplane("XY")
        .box(length * 0.28, body_width * 0.30, root_height * 0.22)
        .translate((length * 0.24, 0.0, root_height * 0.16))
    )
    tip_cap = (
        cq.Workplane("XY")
        .box(length * 0.11, body_width * 0.36, tip_height * 0.42)
        .translate((length * 0.95, 0.0, 0.0))
    )
    proximal_barrel = (
        cq.Workplane("XZ").circle(JOINT_BARREL_RADIUS).extrude(BARREL_LENGTH * 0.5, both=True)
    )
    proximal_bore = cq.Workplane("XZ").circle(PIN_HOLE_RADIUS).extrude(0.030, both=True)
    return body.union(spine).union(tip_cap).union(proximal_barrel).cut(proximal_bore)


def _build_base_part(model: ArticulatedObject):
    base = model.part("base_mount")
    _add_visual_mesh(base, _make_base_shape(), "base_mount.obj", "mount_grey")



    base.inertial = Inertial.from_geometry(
        Box((0.030, 0.022, 0.018)),
        mass=0.32,
        origin=Origin(xyz=(-0.019, 0.0, 0.0)),
    )
    return base


def _build_clevis_segment(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    body_width: float,
    root_height: float,
    tip_height: float,
    mass: float,
    material: str,
):
    part = model.part(name)
    _add_visual_mesh(
        part,
        _make_clevis_phalanx_shape(
            length=length,
            body_width=body_width,
            root_height=root_height,
            tip_height=tip_height,
        ),
        f"{name}.obj",
        material,
    )
    body_box_length = max(length * 0.66, 0.022)
    body_box_center = body_box_length * 0.5 + 0.008
    _joint_cylinder_collision(part, radius=JOINT_BARREL_RADIUS * 0.88, length=BARREL_LENGTH * 0.88)



    part.inertial = Inertial.from_geometry(
        Box((body_box_length, body_width * 0.82, root_height * 0.72)),
        mass=mass,
        origin=Origin(xyz=(body_box_center, 0.0, 0.0)),
    )
    return part


def _build_terminal_segment(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    body_width: float,
    root_height: float,
    tip_height: float,
    mass: float,
    material: str,
):
    part = model.part(name)
    _add_visual_mesh(
        part,
        _make_terminal_phalanx_shape(
            length=length,
            body_width=body_width,
            root_height=root_height,
            tip_height=tip_height,
        ),
        f"{name}.obj",
        material,
    )
    body_box_length = max(length * 0.70, 0.020)
    body_box_center = body_box_length * 0.5 + 0.007
    _joint_cylinder_collision(part, radius=JOINT_BARREL_RADIUS * 0.88, length=BARREL_LENGTH * 0.88)


    part.inertial = Inertial.from_geometry(
        Box((body_box_length, body_width * 0.80, root_height * 0.70)),
        mass=mass,
        origin=Origin(xyz=(body_box_center, 0.0, 0.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_finger_chain", assets=ASSETS)
    model.material("mount_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("proximal_blue", rgba=(0.34, 0.42, 0.56, 1.0))
    model.material("middle_steel", rgba=(0.58, 0.60, 0.65, 1.0))
    model.material("distal_brass", rgba=(0.72, 0.58, 0.30, 1.0))

    base = _build_base_part(model)
    proximal = _build_clevis_segment(
        model,
        name="proximal",
        length=0.058,
        body_width=0.010,
        root_height=0.015,
        tip_height=0.012,
        mass=0.14,
        material="proximal_blue",
    )
    middle = _build_clevis_segment(
        model,
        name="middle",
        length=0.043,
        body_width=0.009,
        root_height=0.013,
        tip_height=0.010,
        mass=0.10,
        material="middle_steel",
    )
    distal = _build_terminal_segment(
        model,
        name="distal",
        length=0.032,
        body_width=0.008,
        root_height=0.011,
        tip_height=0.0075,
        mass=0.07,
        material="distal_brass",
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=4.0, velocity=2.0),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=3.2, velocity=2.5),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=2.3, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.0015, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("base_mount", "proximal", axes="xy", min_overlap=0.004)
    ctx.expect_aabb_overlap("proximal", "middle", axes="xy", min_overlap=0.004)
    ctx.expect_aabb_overlap("middle", "distal", axes="xy", min_overlap=0.004)

    ctx.expect_aabb_gap("proximal", "base_mount", axis="z", max_gap=0.003, max_penetration=0.020)
    ctx.expect_aabb_gap("middle", "proximal", axis="z", max_gap=0.003, max_penetration=0.020)
    ctx.expect_aabb_gap("distal", "middle", axis="z", max_gap=0.003, max_penetration=0.020)

    ctx.expect_joint_motion_axis(
        "base_to_proximal",
        "proximal",
        world_axis="z",
        direction="negative",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "proximal_to_middle",
        "middle",
        world_axis="z",
        direction="negative",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_distal",
        "distal",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

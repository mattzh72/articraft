from __future__ import annotations

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
def _hollow_box_section(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, width, height)
    inner = cq.Workplane("XY").box(length + 0.002, width - 2.0 * wall, height - 2.0 * wall)
    return outer.cut(inner)


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.30, 0.22, 0.04).translate((0.0, 0.0, 0.02))
    column = cq.Workplane("XY").box(0.12, 0.14, 0.28).translate((0.0, 0.0, 0.18))
    rear_rib = cq.Workplane("XY").box(0.07, 0.08, 0.16).translate((-0.035, 0.0, 0.16))
    shoulder_plate_left = cq.Workplane("XY").box(0.08, 0.014, 0.12).translate((0.0, 0.053, 0.36))
    shoulder_plate_right = cq.Workplane("XY").box(0.08, 0.014, 0.12).translate((0.0, -0.053, 0.36))
    shoulder_pin = cq.Workplane("XZ").cylinder(0.092, 0.013).translate((0.0, 0.0, 0.36))
    service_box = cq.Workplane("XY").box(0.09, 0.09, 0.05).translate((0.02, 0.0, 0.08))
    return (
        foot.union(column)
        .union(rear_rib)
        .union(shoulder_plate_left)
        .union(shoulder_plate_right)
        .union(shoulder_pin)
        .union(service_box)
    )


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_housing = cq.Workplane("XY").box(0.06, 0.082, 0.10).translate((0.012, 0.0, 0.0))
    beam = _hollow_box_section(0.28, 0.08, 0.10, 0.011).translate((0.18, 0.0, 0.0))
    elbow_housing = cq.Workplane("XY").box(0.08, 0.12, 0.12).translate((0.34, 0.0, 0.0))
    elbow_pin = cq.Workplane("XZ").cylinder(0.10, 0.011).translate((0.36, 0.0, 0.0))
    cable_tray = cq.Workplane("XY").box(0.12, 0.045, 0.014).translate((0.19, 0.0, 0.057))
    return shoulder_housing.union(beam).union(elbow_housing).union(elbow_pin).union(cable_tray)


def _make_forearm_shape() -> cq.Workplane:
    elbow_housing = cq.Workplane("XY").box(0.06, 0.10, 0.10).translate((0.02, 0.0, 0.0))
    beam = _hollow_box_section(0.30, 0.07, 0.09, 0.010).translate((0.21, 0.0, 0.0))
    guide_deck = cq.Workplane("XY").box(0.26, 0.08, 0.014).translate((0.25, 0.0, 0.052))
    rail_left = cq.Workplane("XY").box(0.24, 0.012, 0.010).translate((0.27, 0.022, 0.064))
    rail_right = cq.Workplane("XY").box(0.24, 0.012, 0.010).translate((0.27, -0.022, 0.064))
    carriage_saddle = cq.Workplane("XY").box(0.08, 0.09, 0.03).translate((0.18, 0.0, 0.078))
    nose_housing = cq.Workplane("XY").box(0.04, 0.09, 0.09).translate((0.39, 0.0, 0.018))
    probe_manifold = cq.Workplane("XY").box(0.06, 0.05, 0.05).translate((0.40, 0.0, -0.022))
    return (
        elbow_housing.union(beam)
        .union(guide_deck)
        .union(rail_left)
        .union(rail_right)
        .union(carriage_saddle)
        .union(nose_housing)
        .union(probe_manifold)
    )


def _make_tool_slide_shape() -> cq.Workplane:
    rear_cap = cq.Workplane("XY").box(0.012, 0.08, 0.05).translate((0.0, 0.0, 0.0))
    carriage = cq.Workplane("XY").box(0.08, 0.09, 0.05).translate((0.04, 0.0, 0.0))
    guide_block_left = cq.Workplane("XY").box(0.038, 0.018, 0.018).translate((0.03, 0.022, -0.028))
    guide_block_right = (
        cq.Workplane("XY").box(0.038, 0.018, 0.018).translate((0.03, -0.022, -0.028))
    )
    extension_beam = _hollow_box_section(0.18, 0.06, 0.06, 0.008).translate((0.17, 0.0, 0.0))
    sensor_block = cq.Workplane("XY").box(0.05, 0.055, 0.055).translate((0.285, 0.0, 0.0))
    lens = cq.Workplane("YZ").cylinder(0.02, 0.012).translate((0.32, 0.0, 0.0))
    return (
        rear_cap.union(carriage)
        .union(guide_block_left)
        .union(guide_block_right)
        .union(extension_beam)
        .union(sensor_block)
        .union(lens)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_inspection_arm", assets=ASSETS)

    model.material("machine_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("safety_orange", rgba=(0.90, 0.48, 0.12, 1.0))
    model.material("slide_metal", rgba=(0.76, 0.79, 0.82, 1.0))
    model.material("optic_blue", rgba=(0.17, 0.26, 0.46, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "base.obj", "machine_gray")



    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.18, 0.34)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    upper_arm = model.part("upper_arm")
    _add_mesh_visual(upper_arm, _make_upper_arm_shape(), "upper_arm.obj", "safety_orange")



    upper_arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.08, 0.10)),
        mass=7.5,
        origin=Origin(xyz=(0.18, 0.0, 0.01)),
    )

    forearm = model.part("forearm")
    _add_mesh_visual(forearm, _make_forearm_shape(), "forearm.obj", "safety_orange")




    forearm.inertial = Inertial.from_geometry(
        Box((0.28, 0.07, 0.09)),
        mass=5.0,
        origin=Origin(xyz=(0.21, 0.0, 0.01)),
    )

    tool_slide = model.part("tool_slide")
    _add_mesh_visual(tool_slide, _make_tool_slide_shape(), "tool_slide.obj", "slide_metal")
    tool_slide.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.322, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material="optic_blue",
    )



    tool_slide.inertial = Inertial.from_geometry(
        Box((0.22, 0.06, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.05, effort=60.0, velocity=1.5),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.50, effort=45.0, velocity=1.8),
    )
    model.articulation(
        "extension",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=tool_slide,
        origin=Origin(xyz=(0.18, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.16, effort=20.0, velocity=0.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "shoulder", "upper_arm", world_axis="z", direction="positive", min_delta=0.04
    )
    ctx.expect_joint_motion_axis(
        "elbow", "forearm", world_axis="z", direction="positive", min_delta=0.03
    )
    ctx.expect_joint_motion_axis(
        "extension", "tool_slide", world_axis="x", direction="positive", min_delta=0.05
    )
    ctx.expect_aabb_overlap("upper_arm", "base", axes="xy", min_overlap=0.004)

    with ctx.pose(extension=0.0):
        ctx.expect_origin_gap("tool_slide", "forearm", axis="z", min_gap=0.02)
        ctx.expect_aabb_gap("tool_slide", "forearm", axis="z", max_gap=0.05, max_penetration=0.05)

    with ctx.pose(extension=0.16):
        ctx.expect_origin_gap("tool_slide", "forearm", axis="z", min_gap=0.02)
        ctx.expect_aabb_gap("tool_slide", "forearm", axis="z", max_gap=0.05, max_penetration=0.05)

    with ctx.pose(shoulder=1.0):
        ctx.expect_aabb_gap("upper_arm", "base", axis="z", max_gap=0.20, max_penetration=0.12)
        ctx.expect_origin_gap("upper_arm", "base", axis="z", min_gap=0.03)

    with ctx.pose(shoulder=0.85, elbow=1.20):
        ctx.expect_origin_gap("forearm", "base", axis="z", min_gap=0.08)
        ctx.expect_origin_gap("tool_slide", "base", axis="z", min_gap=0.12)

    with ctx.pose(shoulder=0.45, elbow=0.80, extension=0.16):
        ctx.expect_origin_gap("tool_slide", "forearm", axis="z", min_gap=0.02)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

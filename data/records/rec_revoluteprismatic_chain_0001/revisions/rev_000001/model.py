from __future__ import annotations

from math import pi

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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


def _base_mount_shape() -> cq.Workplane:
    hole_positions = [
        (-0.055, -0.032),
        (-0.055, 0.032),
        (0.055, -0.032),
        (0.055, 0.032),
    ]

    plate = cq.Workplane("XY").box(
        0.18,
        0.11,
        0.012,
        centered=(True, True, False),
    )
    column = (
        cq.Workplane("XY")
        .box(0.05, 0.08, 0.08, centered=(True, True, False))
        .translate((-0.03, 0.0, 0.012))
    )
    ear_left = (
        cq.Workplane("XY")
        .box(0.032, 0.01, 0.05, centered=(True, True, False))
        .translate((-0.004, 0.023, 0.07))
    )
    ear_right = (
        cq.Workplane("XY")
        .box(0.032, 0.01, 0.05, centered=(True, True, False))
        .translate((-0.004, -0.023, 0.07))
    )
    relief = cq.Workplane("XZ").rect(0.018, 0.03).extrude(0.026).translate((-0.005, -0.013, 0.038))
    holes = cq.Workplane("XY").pushPoints(hole_positions).circle(0.0055).extrude(0.012)

    return plate.union(column).union(ear_left).union(ear_right).cut(relief).cut(holes)


def _pivot_arm_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(0.0125).extrude(0.036).translate((0.0, -0.018, 0.0))
    neck = cq.Workplane("YZ").rect(0.032, 0.04).extrude(0.035)
    beam = cq.Workplane("YZ").rect(0.056, 0.048).extrude(0.24).translate((0.035, 0.0, 0.0))
    cavity = cq.Workplane("YZ").rect(0.044, 0.034).extrude(0.205).translate((0.07, 0.0, 0.0))

    return barrel.union(neck).union(beam).cut(cavity)


def _extension_rail_shape() -> cq.Workplane:
    boot = cq.Workplane("YZ").rect(0.046, 0.034).extrude(0.024).translate((-0.012, 0.0, 0.0))
    rail = cq.Workplane("YZ").rect(0.044, 0.032).extrude(0.19).translate((-0.008, 0.0, 0.0))
    return boot.union(rail)


def _inspection_head_shape() -> cq.Workplane:
    head_body = cq.Workplane("YZ").rect(0.05, 0.034).extrude(0.055).translate((0.178, 0.0, 0.0))
    hood = cq.Workplane("YZ").rect(0.046, 0.008).extrude(0.028).translate((0.19, 0.0, 0.02))
    bezel = cq.Workplane("YZ").circle(0.016).extrude(0.004).translate((0.229, 0.0, 0.0))
    return head_body.union(hood).union(bezel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_arm", assets=ASSETS)

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("zinc", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("aluminum", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("anodized_dark", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("glass", rgba=(0.52, 0.72, 0.84, 0.55))
    model.material("amber", rgba=(0.93, 0.64, 0.17, 0.95))

    base_mount = model.part("base_mount")
    base_mount.visual(
        mesh_from_cadquery(_base_mount_shape(), "base_mount.obj", assets=ASSETS),
        material="powder_steel",
    )
    for x_pos in (-0.055, 0.055):
        for y_pos in (-0.032, 0.032):
            base_mount.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x_pos, y_pos, 0.014)),
                material="zinc",
            )
    base_mount.inertial = Inertial.from_geometry(
        Box((0.18, 0.11, 0.12)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        mesh_from_cadquery(_pivot_arm_shape(), "pivot_arm.obj", assets=ASSETS),
        material="aluminum",
    )
    pivot_arm.inertial = Inertial.from_geometry(
        Box((0.275, 0.056, 0.048)),
        mass=1.3,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
    )

    extension_stage = model.part("extension_stage")
    extension_stage.visual(
        mesh_from_cadquery(_extension_rail_shape(), "extension_rail.obj", assets=ASSETS),
        material="anodized_dark",
    )
    extension_stage.visual(
        mesh_from_cadquery(_inspection_head_shape(), "inspection_head.obj", assets=ASSETS),
        material="matte_black",
    )
    extension_stage.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.242, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="glass",
    )
    extension_stage.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.229, 0.0, -0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material="amber",
    )
    extension_stage.inertial = Inertial.from_geometry(
        Box((0.26, 0.05, 0.04)),
        mass=0.95,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.1,
            effort=18.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "stage_extension",
        ArticulationType.PRISMATIC,
        parent=pivot_arm,
        child=extension_stage,
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.14,
            effort=12.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("base_mount", "pivot_arm")
    ctx.expect_aabb_overlap("base_mount", "pivot_arm", axes="yz", min_overlap=0.03)
    ctx.expect_aabb_contact("pivot_arm", "extension_stage")
    ctx.expect_aabb_overlap("pivot_arm", "extension_stage", axes="yz", min_overlap=0.025)
    ctx.expect_joint_motion_axis(
        "base_hinge",
        "pivot_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "stage_extension",
        "extension_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.06,
    )

    with ctx.pose(base_hinge=1.0):
        ctx.expect_aabb_contact("base_mount", "pivot_arm")
        ctx.expect_aabb_overlap("pivot_arm", "extension_stage", axes="yz", min_overlap=0.02)

    with ctx.pose(stage_extension=0.14):
        ctx.expect_aabb_contact("pivot_arm", "extension_stage")
        ctx.expect_aabb_overlap("pivot_arm", "extension_stage", axes="yz", min_overlap=0.018)

    with ctx.pose(base_hinge=1.0, stage_extension=0.14):
        ctx.expect_aabb_contact("base_mount", "pivot_arm")
        ctx.expect_aabb_overlap("pivot_arm", "extension_stage", axes="yz", min_overlap=0.018)
        ctx.expect_aabb_gap(
            "extension_stage",
            "base_mount",
            axis="x",
            max_gap=0.20,
            max_penetration=0.0,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

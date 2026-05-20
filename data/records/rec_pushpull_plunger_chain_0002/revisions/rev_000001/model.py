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
OUTER_SLEEVE_OUTER_R = 0.03
OUTER_SLEEVE_INNER_R = 0.024
OUTER_SLEEVE_LENGTH = 0.13

CARRIER_SHUTTLE_R = 0.020
CARRIER_SHUTTLE_LENGTH = 0.075
CARRIER_GUIDE_Z = 0.085
CARRIER_SLEEVE_OUTER_R = 0.017
CARRIER_SLEEVE_INNER_R = 0.0115
CARRIER_SLEEVE_LENGTH = 0.065

PLUNGER_ROD_R = 0.0085
PLUNGER_ROD_LENGTH = 0.145

CARRIER_TRAVEL = 0.06
PLUNGER_TRAVEL = 0.055


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(OUTER_SLEEVE_OUTER_R)
        .circle(OUTER_SLEEVE_INNER_R)
        .extrude(OUTER_SLEEVE_LENGTH)
    )
    base_plate = cq.Workplane("XY").box(0.11, 0.09, 0.018).translate((0.0, 0.0, -0.009))
    left_upright = cq.Workplane("XY").box(0.016, 0.05, 0.055).translate((-0.035, 0.0, 0.0175))
    right_upright = cq.Workplane("XY").box(0.016, 0.05, 0.055).translate((0.035, 0.0, 0.0175))
    bridge = cq.Workplane("XY").box(0.08, 0.03, 0.018).translate((0.0, 0.0, 0.009))
    clamp_ring = (
        cq.Workplane("XY")
        .circle(0.034)
        .circle(OUTER_SLEEVE_OUTER_R)
        .extrude(0.014)
        .translate((0.0, 0.0, 0.018))
    )
    return (
        sleeve.union(base_plate)
        .union(left_upright)
        .union(right_upright)
        .union(bridge)
        .union(clamp_ring)
    )


def _build_carrier_shape() -> cq.Workplane:
    shuttle = cq.Workplane("XY").circle(CARRIER_SHUTTLE_R).extrude(CARRIER_SHUTTLE_LENGTH)
    upper_sleeve = (
        cq.Workplane("XY")
        .circle(CARRIER_SLEEVE_OUTER_R)
        .circle(CARRIER_SLEEVE_INNER_R)
        .extrude(CARRIER_SLEEVE_LENGTH)
        .translate((0.0, 0.0, CARRIER_GUIDE_Z))
    )
    saddle = cq.Workplane("XY").box(0.032, 0.022, 0.014).translate((0.0, 0.0, 0.078))
    left_web = cq.Workplane("XY").box(0.009, 0.026, 0.034).translate((-0.014, 0.0, 0.091))
    right_web = cq.Workplane("XY").box(0.009, 0.026, 0.034).translate((0.014, 0.0, 0.091))
    collar = (
        cq.Workplane("XY")
        .circle(0.022)
        .circle(CARRIER_SHUTTLE_R)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.073))
    )
    return shuttle.union(upper_sleeve).union(saddle).union(left_web).union(right_web).union(collar)


def _build_plunger_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").circle(PLUNGER_ROD_R).extrude(PLUNGER_ROD_LENGTH)
    rear_collar = cq.Workplane("XY").circle(0.0105).extrude(0.012)
    nose_boss = cq.Workplane("XY").circle(0.012).extrude(0.014).translate((0.0, 0.0, 0.145))
    striker_head = cq.Workplane("XY").box(0.026, 0.026, 0.02).translate((0.0, 0.0, 0.155))
    return rod.union(rear_collar).union(nose_boss).union(striker_head)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chained_plunger_mechanism", assets=ASSETS)

    model.material("frame_steel", rgba=(0.63, 0.66, 0.72, 1.0))
    model.material("dark_anodized", rgba=(0.2, 0.22, 0.26, 1.0))
    model.material("plunger_finish", rgba=(0.86, 0.88, 0.9, 1.0))

    base_frame = model.part("base_frame")
    _add_visual_mesh(base_frame, _build_base_shape(), "base_frame.obj", "frame_steel")







    base_frame.inertial = Inertial.from_geometry(
        Box((0.11, 0.09, 0.09)), mass=1.6, origin=Origin(xyz=(0.0, 0.0, 0.03))
    )

    carrier_slide = model.part("carrier_slide")
    _add_visual_mesh(carrier_slide, _build_carrier_shape(), "carrier_slide.obj", "dark_anodized")






    carrier_slide.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.15)), mass=0.42, origin=Origin(xyz=(0.0, 0.0, 0.085))
    )

    plunger_rod = model.part("plunger_rod")
    _add_visual_mesh(plunger_rod, _build_plunger_shape(), "plunger_rod.obj", "plunger_finish")


    plunger_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.17), mass=0.19, origin=Origin(xyz=(0.0, 0.0, 0.085))
    )

    model.articulation(
        "base_to_carrier",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=carrier_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CARRIER_TRAVEL, effort=120.0, velocity=0.25),
    )
    model.articulation(
        "carrier_to_plunger",
        ArticulationType.PRISMATIC,
        parent=carrier_slide,
        child=plunger_rod,
        origin=Origin(xyz=(0.0, 0.0, CARRIER_GUIDE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PLUNGER_TRAVEL, effort=80.0, velocity=0.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("carrier_slide", "base_frame", axes="xy", max_dist=0.01)
    ctx.expect_origin_distance("plunger_rod", "carrier_slide", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("carrier_slide", "base_frame", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("plunger_rod", "carrier_slide", axes="xy", min_overlap=0.014)
    ctx.expect_origin_gap("plunger_rod", "carrier_slide", axis="z", min_gap=0.02)
    ctx.expect_joint_motion_axis(
        "base_to_carrier",
        "carrier_slide",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "carrier_to_plunger",
        "plunger_rod",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

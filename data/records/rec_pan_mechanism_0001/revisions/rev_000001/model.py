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
def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.110).extrude(0.006)
    deck = cq.Workplane("XY").circle(0.095).extrude(0.016).translate((0.0, 0.0, 0.006))
    boss = cq.Workplane("XY").circle(0.033).extrude(0.008).translate((0.0, 0.0, 0.022))
    return flange.union(deck).union(boss)


def _build_turntable_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(0.090).extrude(0.008)
    mounting_pad = cq.Workplane("XY").circle(0.040).extrude(0.003).translate((0.0, 0.0, 0.008))
    spindle = cq.Workplane("XY").circle(0.024).extrude(0.006).translate((0.0, 0.0, -0.006))
    return plate.union(mounting_pad).union(spindle)


def _build_pedestal_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(0.060, 0.050, 0.006).translate((0.0, 0.0, 0.003))
    column = cq.Workplane("XY").box(0.038, 0.030, 0.074).translate((0.0, 0.0, 0.043))
    left_cheek = cq.Workplane("XY").box(0.008, 0.050, 0.050).translate((-0.019, 0.0, 0.031))
    right_cheek = cq.Workplane("XY").box(0.008, 0.050, 0.050).translate((0.019, 0.0, 0.031))
    top_pad = cq.Workplane("XY").box(0.075, 0.055, 0.006).translate((0.0, 0.0, 0.083))
    return base_plate.union(column).union(left_cheek).union(right_cheek).union(top_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_turntable_sensor_mount", assets=ASSETS)
    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("pedestal_gray", rgba=(0.58, 0.60, 0.64, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _build_base_shape(), "turntable_base.obj", "powder_black")



    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.030),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    turntable = model.part("turntable_plate")
    _add_mesh_visual(
        turntable,
        _build_turntable_shape(),
        "turntable_plate.obj",
        "machined_aluminum",
    )

    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.011),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
    )

    pedestal = model.part("payload_pedestal")
    _add_mesh_visual(pedestal, _build_pedestal_shape(), "payload_pedestal.obj", "pedestal_gray")



    pedestal.inertial = Inertial.from_geometry(
        Box((0.075, 0.055, 0.089)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.0445)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.967,
            upper=2.967,
            effort=12.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "turntable_to_pedestal",
        ArticulationType.FIXED,
        parent=turntable,
        child=pedestal,
        origin=Origin(xyz=(0.035, 0.0, 0.008)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("turntable_plate", "base", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("payload_pedestal", "turntable_plate", axes="xy", max_dist=0.04)
    ctx.expect_origin_distance("payload_pedestal", "base", axes="xy", max_dist=0.04)

    ctx.expect_origin_gap("turntable_plate", "base", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("payload_pedestal", "turntable_plate", axis="z", min_gap=0.0)

    ctx.expect_aabb_overlap("turntable_plate", "base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_overlap("payload_pedestal", "turntable_plate", axes="xy", min_overlap=0.05)

    ctx.expect_aabb_gap("turntable_plate", "base", axis="z", max_gap=0.002, max_penetration=0.0)
    ctx.expect_aabb_gap("payload_pedestal", "turntable_plate", axis="z", max_gap=0.002, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_turntable",
        "payload_pedestal",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

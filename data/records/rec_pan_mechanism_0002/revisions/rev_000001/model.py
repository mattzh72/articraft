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
def _frame_body_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(0.32, 0.22, 0.014)
        .translate((0.0, 0.0, 0.007))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.12, -0.075),
                (-0.12, 0.075),
                (0.12, -0.075),
                (0.12, 0.075),
            ]
        )
        .slot2D(0.040, 0.013, 90)
        .cutThruAll()
    )

    left_skid = cq.Workplane("XY").box(0.26, 0.028, 0.018).translate((0.0, 0.074, 0.009))
    right_skid = cq.Workplane("XY").box(0.26, 0.028, 0.018).translate((0.0, -0.074, 0.009))

    left_upright = cq.Workplane("XY").box(0.014, 0.18, 0.078).translate((0.103, 0.0, 0.053))
    left_upright = left_upright.cut(
        cq.Workplane("XY").box(0.020, 0.105, 0.042).translate((0.103, 0.0, 0.053))
    )

    right_upright = cq.Workplane("XY").box(0.014, 0.18, 0.078).translate((-0.103, 0.0, 0.053))
    right_upright = right_upright.cut(
        cq.Workplane("XY").box(0.020, 0.105, 0.042).translate((-0.103, 0.0, 0.053))
    )

    front_tie = cq.Workplane("XY").box(0.196, 0.012, 0.050).translate((0.0, 0.084, 0.057))
    rear_tie = cq.Workplane("XY").box(0.196, 0.012, 0.050).translate((0.0, -0.084, 0.057))

    bridge = (
        cq.Workplane("XY")
        .box(0.24, 0.18, 0.012)
        .translate((0.0, 0.0, 0.094))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.058)
        .cutThruAll()
    )

    return (
        base_plate.union(left_skid)
        .union(right_skid)
        .union(left_upright)
        .union(right_upright)
        .union(front_tie)
        .union(rear_tie)
        .union(bridge)
    )


def _frame_race_shape() -> cq.Workplane:
    pedestal = cq.Workplane("XY").circle(0.056).extrude(0.010).translate((0.0, 0.0, 0.106))
    outer_race = (
        cq.Workplane("XY").circle(0.075).circle(0.058).extrude(0.007).translate((0.0, 0.0, 0.116))
    )
    inner_race = (
        cq.Workplane("XY").circle(0.052).circle(0.043).extrude(0.004).translate((0.0, 0.0, 0.119))
    )
    return pedestal.union(outer_race).union(inner_race)


def _platform_hub_shape() -> cq.Workplane:
    lower_ring = (
        cq.Workplane("XY").circle(0.068).circle(0.030).extrude(0.007).translate((0.0, 0.0, 0.001))
    )
    spindle = cq.Workplane("XY").circle(0.034).extrude(0.012).translate((0.0, 0.0, 0.008))
    cable_bore = cq.Workplane("XY").circle(0.016).extrude(0.026)
    return lower_ring.union(spindle).cut(cable_bore)


def _platform_deck_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(0.24, 0.17, 0.012)
        .translate((0.035, 0.0, 0.022))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .slot2D(0.095, 0.026, 0)
        .cutBlind(-0.012)
    )

    left_rib = cq.Workplane("XY").box(0.13, 0.014, 0.020).translate((0.045, 0.055, 0.032))
    right_rib = cq.Workplane("XY").box(0.13, 0.014, 0.020).translate((0.045, -0.055, 0.032))
    front_plinth = cq.Workplane("XY").box(0.082, 0.110, 0.020).translate((0.110, 0.0, 0.036))
    rear_counter = cq.Workplane("XY").box(0.042, 0.080, 0.024).translate((-0.028, 0.0, 0.042))
    service_box = cq.Workplane("XY").box(0.038, 0.058, 0.018).translate((0.008, -0.050, 0.040))

    return (
        deck.union(left_rib)
        .union(right_rib)
        .union(front_plinth)
        .union(rear_counter)
        .union(service_box)
    )


def _sensor_pod_shape() -> cq.Workplane:
    sensor_body = cq.Workplane("XY").box(0.090, 0.090, 0.042).translate((0.128, 0.0, 0.058))
    sensor_roof = cq.Workplane("XY").box(0.074, 0.074, 0.014).translate((0.128, 0.0, 0.086))
    lens_barrel = cq.Workplane("YZ").circle(0.021).extrude(0.018).translate((0.168, 0.0, 0.060))
    return sensor_body.union(sensor_roof).union(lens_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_scanning_platform", assets=ASSETS)

    model.material("frame_paint", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("machined_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("safety_orange", rgba=(0.93, 0.46, 0.15, 1.0))
    model.material("sensor_black", rgba=(0.10, 0.11, 0.12, 1.0))

    mount_frame = model.part("mount_frame")
    mount_frame.visual(
        mesh_from_cadquery(_frame_body_shape(), "mount_frame_body.obj", assets=ASSETS),
        material="frame_paint",
    )
    mount_frame.visual(
        mesh_from_cadquery(_frame_race_shape(), "mount_frame_race.obj", assets=ASSETS),
        material="machined_steel",
    )










    mount_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.22, 0.123)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0615)),
    )

    scanner_platform = model.part("scanner_platform")
    scanner_platform.visual(
        mesh_from_cadquery(_platform_hub_shape(), "scanner_platform_hub.obj", assets=ASSETS),
        material="machined_steel",
    )
    scanner_platform.visual(
        mesh_from_cadquery(_platform_deck_shape(), "scanner_platform_deck.obj", assets=ASSETS),
        material="safety_orange",
    )
    scanner_platform.visual(
        mesh_from_cadquery(_sensor_pod_shape(), "scanner_platform_sensor.obj", assets=ASSETS),
        material="sensor_black",
    )






    scanner_platform.inertial = Inertial.from_geometry(
        Box((0.30, 0.17, 0.10)),
        mass=8.5,
        origin=Origin(xyz=(0.060, 0.0, 0.050)),
    )

    model.articulation(
        "frame_to_platform_pan",
        ArticulationType.REVOLUTE,
        parent=mount_frame,
        child=scanner_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=40.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.001, overlap_volume_tol=0.0)
    ctx.expect_origin_gap("scanner_platform", "mount_frame", axis="z", min_gap=0.0)
    ctx.expect_origin_distance("scanner_platform", "mount_frame", axes="xy", max_dist=0.08)
    ctx.expect_aabb_overlap("scanner_platform", "mount_frame", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_gap("scanner_platform", "mount_frame", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "frame_to_platform_pan",
        "scanner_platform",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

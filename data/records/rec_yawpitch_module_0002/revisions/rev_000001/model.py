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
# Prefer `model = ArticulatedObject(..., assets=ASSETS)`.
# Use `add_cadquery_visual(...)` for mesh + conservative proxy in one step.
# Use `mesh_from_cadquery(..., "part.obj", assets=ASSETS)` for manual control.
# `TestContext(object_model)` will infer asset_root from model.assets.
def _bearing_ring(outer_radius: float, inner_radius: float, height: float):
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _add_mesh_visual(part, shape, filename: str, material: str):
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_shape():
    plate = cq.Workplane("XY").box(0.18, 0.18, 0.016).translate((0.0, 0.0, 0.008))
    pedestal = cq.Workplane("XY").circle(0.052).extrude(0.032).translate((0.0, 0.0, 0.016))
    lower_ring = _bearing_ring(0.066, 0.043, 0.008).translate((0.0, 0.0, 0.040))
    return plate.union(pedestal).union(lower_ring)


def _build_yaw_stage_shape():
    ring = _bearing_ring(0.062, 0.042, 0.010)
    hub = cq.Workplane("XY").circle(0.030).extrude(0.014)
    deck = cq.Workplane("XY").box(0.14, 0.14, 0.012).translate((0.0, 0.0, 0.016))

    cheek_profile = [
        (-0.025, 0.0),
        (-0.025, 0.082),
        (-0.010, 0.082),
        (0.022, 0.048),
        (0.022, 0.0),
    ]
    left_cheek = (
        cq.Workplane("XZ")
        .polyline(cheek_profile)
        .close()
        .extrude(0.010)
        .translate((0.0, -0.060, 0.0))
    )
    right_cheek = (
        cq.Workplane("XZ")
        .polyline(cheek_profile)
        .close()
        .extrude(0.010)
        .translate((0.0, 0.050, 0.0))
    )

    rear_bridge = cq.Workplane("XY").box(0.014, 0.100, 0.052).translate((-0.030, 0.0, 0.052))
    top_tie = cq.Workplane("XY").box(0.012, 0.082, 0.012).translate((-0.028, 0.0, 0.094))
    left_boss = cq.Workplane("XZ").circle(0.014).extrude(0.008).translate((0.0, -0.060, 0.088))
    right_boss = cq.Workplane("XZ").circle(0.014).extrude(0.008).translate((0.0, 0.052, 0.088))

    return (
        ring.union(hub)
        .union(deck)
        .union(left_cheek)
        .union(right_cheek)
        .union(rear_bridge)
        .union(top_tie)
        .union(left_boss)
        .union(right_boss)
    )


def _build_pitch_yoke_shape():
    shaft = cq.Workplane("XZ").circle(0.008).extrude(0.104)
    left_hub = cq.Workplane("XZ").circle(0.012).extrude(0.006).translate((0.0, -0.006, 0.0))
    right_hub = cq.Workplane("XZ").circle(0.012).extrude(0.006).translate((0.0, 0.104, 0.0))
    back_plate = cq.Workplane("XY").box(0.012, 0.078, 0.048).translate((-0.008, 0.052, 0.0))
    bottom_mount = cq.Workplane("XY").box(0.052, 0.078, 0.008).translate((0.010, 0.052, -0.028))
    top_cap = cq.Workplane("XY").box(0.056, 0.074, 0.010).translate((0.020, 0.052, 0.030))

    body = cq.Workplane("XY").box(0.080, 0.070, 0.050).translate((0.038, 0.052, 0.0))
    body = (
        body.faces(">X").workplane(centerOption="CenterOfMass").rect(0.030, 0.020).cutBlind(-0.010)
    )

    return (
        shaft.union(left_hub)
        .union(right_hub)
        .union(back_plate)
        .union(bottom_mount)
        .union(top_cap)
        .union(body)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_turret_module", assets=ASSETS)

    model.material("base_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("structure", rgba=(0.33, 0.35, 0.39, 1.0))
    model.material("payload", rgba=(0.46, 0.49, 0.53, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _build_base_shape(), "turret_base.obj", "base_paint")


    base.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.060)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    yaw_stage = model.part("yaw_stage")
    _add_mesh_visual(yaw_stage, _build_yaw_stage_shape(), "turret_yaw_stage.obj", "structure")






    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, 0.110)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.008, length=0.104),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material="structure",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material="structure",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.104, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material="structure",
    )
    pitch_yoke.visual(
        Box((0.012, 0.078, 0.048)), origin=Origin(xyz=(-0.008, 0.052, 0.0)), material="structure"
    )
    pitch_yoke.visual(
        Box((0.052, 0.078, 0.008)), origin=Origin(xyz=(0.010, 0.052, -0.028)), material="payload"
    )
    pitch_yoke.visual(
        Box((0.056, 0.074, 0.010)), origin=Origin(xyz=(0.020, 0.052, 0.030)), material="payload"
    )
    pitch_yoke.visual(
        Box((0.080, 0.070, 0.050)), origin=Origin(xyz=(0.038, 0.052, 0.0)), material="payload"
    )
    pitch_yoke.visual(
        Box((0.012, 0.034, 0.018)), origin=Origin(xyz=(0.078, 0.052, 0.0)), material="structure"
    )




    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.095, 0.110, 0.060)),
        mass=0.95,
        origin=Origin(xyz=(0.028, 0.052, 0.0)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, -0.052, 0.088)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.05, effort=12.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("yaw_stage", "base", axes="xy", min_overlap=0.09)
    ctx.expect_aabb_gap("yaw_stage", "base", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "base_to_yaw", "pitch_yoke", world_axis="y", direction="positive", min_delta=0.02
    )
    ctx.expect_joint_motion_axis(
        "yaw_to_pitch", "pitch_yoke", world_axis="z", direction="positive", min_delta=0.03
    )
    ctx.expect_origin_distance("pitch_yoke", "yaw_stage", axes="xy", max_dist=0.055)
    ctx.expect_origin_gap("pitch_yoke", "base", axis="z", min_gap=0.03)
    ctx.expect_aabb_overlap("pitch_yoke", "yaw_stage", axes="xy", min_overlap=0.04)

    def posed(**angles):
        try:
            return ctx.pose(**angles)
        except TypeError:
            return ctx.pose(angles)

    with posed(base_to_yaw=-2.4):
        ctx.expect_aabb_overlap("yaw_stage", "base", axes="xy", min_overlap=0.08)
        ctx.expect_origin_gap("pitch_yoke", "base", axis="z", min_gap=0.03)

    with posed(base_to_yaw=2.4):
        ctx.expect_aabb_overlap("yaw_stage", "base", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("pitch_yoke", "yaw_stage", axes="xy", max_dist=0.055)

    with posed(yaw_to_pitch=-0.75):
        ctx.expect_origin_gap("pitch_yoke", "base", axis="z", min_gap=0.015)
        ctx.expect_aabb_overlap("pitch_yoke", "yaw_stage", axes="xy", min_overlap=0.03)

    with posed(yaw_to_pitch=1.05):
        ctx.expect_origin_gap("pitch_yoke", "base", axis="z", min_gap=0.05)
        ctx.expect_origin_distance("pitch_yoke", "yaw_stage", axes="xy", max_dist=0.06)

    with posed(base_to_yaw=1.2, yaw_to_pitch=1.05):
        ctx.expect_origin_gap("pitch_yoke", "base", axis="z", min_gap=0.05)
        ctx.expect_aabb_overlap("yaw_stage", "base", axes="xy", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

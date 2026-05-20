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


def _filleted_box(length: float, width: float, height: float, fillet: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(length, width, height)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid


def _machined_link_bar(length: float, width: float, height: float, fillet: float) -> cq.Workplane:
    body = _filleted_box(length, width, height, fillet)
    pocket = (
        cq.Workplane("XY")
        .box(length * 0.72, width * 0.58, height * 0.32)
        .translate((0.0, 0.0, height * 0.16))
    )
    return body.cut(pocket)


def _pivot_hub(
    length: float,
    width: float,
    height: float,
    pin_radius: float,
    fillet: float,
) -> cq.Workplane:
    hub = _filleted_box(length, width, height, fillet)
    bore = cq.Workplane("XZ").circle(pin_radius).extrude(width * 1.4, both=True)
    side_relief = (
        cq.Workplane("XY")
        .box(length * 0.52, width * 1.2, height * 0.26)
        .translate((0.0, 0.0, height * 0.15))
    )
    return hub.cut(bore).cut(side_relief)


def _clevis_block(
    length: float,
    outer_width: float,
    height: float,
    slot_width: float,
    slot_height: float,
    pin_radius: float,
    fillet: float,
) -> cq.Workplane:
    body = _filleted_box(length, outer_width, height, fillet)
    slot = cq.Workplane("XY").box(length * 0.82, slot_width, slot_height)
    bore = cq.Workplane("XZ").circle(pin_radius).extrude(outer_width * 1.4, both=True)
    return body.cut(slot).cut(bore)


def _make_base_shape() -> cq.Workplane:
    pedestal = _filleted_box(0.14, 0.12, 0.018, 0.004).translate((0.0, 0.0, -0.111))
    column = _filleted_box(0.06, 0.06, 0.12, 0.004).translate((0.0, 0.0, -0.051))
    shoulder_cartridge = _filleted_box(0.07, 0.08, 0.036, 0.003).translate((0.0, 0.0, -0.010))
    shoulder_clevis = _clevis_block(
        length=0.05,
        outer_width=0.072,
        height=0.060,
        slot_width=0.044,
        slot_height=0.042,
        pin_radius=0.010,
        fillet=0.003,
    ).translate((0.0, 0.0, 0.006))
    return pedestal.union(column).union(shoulder_cartridge).union(shoulder_clevis)


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _pivot_hub(0.050, 0.028, 0.048, pin_radius=0.010, fillet=0.003)
    shoulder_motor = _filleted_box(0.052, 0.050, 0.040, 0.003).translate((0.020, 0.0, -0.010))
    arm_bar = _machined_link_bar(0.18, 0.036, 0.028, 0.003).translate((0.104, 0.0, 0.0))
    elbow_mount = _filleted_box(0.050, 0.040, 0.038, 0.003).translate((0.182, 0.0, 0.0))
    elbow_clevis = _clevis_block(
        length=0.042,
        outer_width=0.056,
        height=0.052,
        slot_width=0.030,
        slot_height=0.040,
        pin_radius=0.0085,
        fillet=0.0025,
    ).translate((0.210, 0.0, 0.0))
    return shoulder_hub.union(shoulder_motor).union(arm_bar).union(elbow_mount).union(elbow_clevis)


def _make_forearm_shape() -> cq.Workplane:
    elbow_hub = _pivot_hub(0.040, 0.026, 0.044, pin_radius=0.0085, fillet=0.0025)
    forearm_bar = _machined_link_bar(0.15, 0.032, 0.026, 0.0025).translate((0.082, 0.0, 0.0))
    wrist_mount = _filleted_box(0.040, 0.032, 0.032, 0.0025).translate((0.146, 0.0, 0.0))
    wrist_clevis = _clevis_block(
        length=0.032,
        outer_width=0.046,
        height=0.042,
        slot_width=0.024,
        slot_height=0.031,
        pin_radius=0.0065,
        fillet=0.002,
    ).translate((0.165, 0.0, 0.0))
    return elbow_hub.union(forearm_bar).union(wrist_mount).union(wrist_clevis)


def _make_tool_shape() -> cq.Workplane:
    wrist_hub = _pivot_hub(0.030, 0.022, 0.034, pin_radius=0.0065, fillet=0.002)
    tool_body = _filleted_box(0.036, 0.034, 0.034, 0.0025).translate((0.020, 0.0, 0.0))
    flange = cq.Workplane("YZ").circle(0.022).extrude(0.008).translate((0.040, 0.0, 0.0))
    pilot = cq.Workplane("YZ").circle(0.008).extrude(0.028).translate((0.048, 0.0, 0.0))
    return wrist_hub.union(tool_body).union(flange).union(pilot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_robotic_arm", assets=ASSETS)

    model.material("painted_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.84, 1.0))
    model.material("tool_black", rgba=(0.12, 0.12, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base.obj", assets=ASSETS),
        material="painted_steel",
    )




    base.inertial = Inertial.from_geometry(
        Box((0.14, 0.12, 0.16)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm_shape(), "upper_arm.obj", assets=ASSETS),
        material="machined_aluminum",
    )






    upper_arm.inertial = Inertial.from_geometry(
        Box((0.24, 0.06, 0.06)),
        mass=1.2,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_make_forearm_shape(), "forearm.obj", assets=ASSETS),
        material="machined_aluminum",
    )





    forearm.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.05)),
        mass=0.8,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        mesh_from_cadquery(_make_tool_shape(), "tool_flange.obj", assets=ASSETS),
        material="tool_black",
    )




    tool_flange.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.05)),
        mass=0.25,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.20, effort=40.0, velocity=1.5),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=28.0, velocity=1.8),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=tool_flange,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=12.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "base",
        "upper_arm",
        reason="shoulder clevis envelopes the arm-side hub around the hinge pin",
    )
    ctx.allow_overlap(
        "upper_arm", "forearm", reason="elbow clevis nests the forearm hub at the hinge"
    )
    ctx.allow_overlap(
        "forearm", "tool_flange", reason="wrist yoke wraps the wrist carrier around the hinge axis"
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.002, overlap_volume_tol=0.0)
    ctx.expect_aabb_overlap("upper_arm", "base", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("tool_flange", "forearm", axes="xy", min_overlap=0.008)
    ctx.expect_origin_distance("upper_arm", "base", axes="xy", max_dist=0.14)
    ctx.expect_origin_distance("forearm", "upper_arm", axes="xy", max_dist=0.22)
    ctx.expect_origin_distance("tool_flange", "forearm", axes="xy", max_dist=0.18)
    ctx.expect_joint_motion_axis(
        "shoulder_hinge",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "elbow_hinge",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.025,
    )
    ctx.expect_joint_motion_axis(
        "wrist_hinge",
        "tool_flange",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )
    return ctx.report()


object_model = build_object_model()

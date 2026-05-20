from __future__ import annotations

from contextlib import nullcontext

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
def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _pose(ctx: TestContext, **joint_values):
    pose_fn = getattr(ctx, "pose", None)
    if pose_fn is None:
        return nullcontext()
    try:
        return pose_fn(joint_values)
    except TypeError:
        return pose_fn(**joint_values)


def _make_base_shape() -> cq.Workplane:
    bed = cq.Workplane("XY").box(0.34, 0.12, 0.012).translate((0.0, 0.0, 0.006))
    bed = (
        bed.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.125, -0.04),
                (-0.125, 0.04),
                (0.125, -0.04),
                (0.125, 0.04),
            ]
        )
        .hole(0.009)
    )

    rail_core = _box((0.29, 0.024, 0.018), (0.0, 0.0, 0.021))
    rail_way_pos = _box((0.29, 0.016, 0.014), (0.0, 0.021, 0.019))
    rail_way_neg = _box((0.29, 0.016, 0.014), (0.0, -0.021, 0.019))
    stop_left = _box((0.018, 0.06, 0.028), (-0.146, 0.0, 0.026))
    stop_right = _box((0.018, 0.06, 0.028), (0.146, 0.0, 0.026))

    return _fuse(bed, rail_core, rail_way_pos, rail_way_neg, stop_left, stop_right)


def _make_carriage_shape() -> cq.Workplane:
    lower = _box((0.095, 0.084, 0.024), (0.0, 0.0, 0.012))
    top = cq.Workplane("XY").box(0.108, 0.09, 0.012).translate((0.002, 0.0, 0.03))
    top = (
        top.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.028, -0.028),
                (-0.028, 0.028),
                (0.018, -0.028),
                (0.018, 0.028),
            ]
        )
        .hole(0.006)
    )
    pedestal = _box((0.044, 0.058, 0.028), (0.036, 0.0, 0.05))
    nose = _box((0.018, 0.05, 0.018), (0.062, 0.0, 0.037))
    disc = cq.Workplane("XY").circle(0.026).extrude(0.008).translate((0.052, 0.0, 0.06))
    disc_bolts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.014, -0.014),
                (-0.014, 0.014),
                (0.014, -0.014),
                (0.014, 0.014),
            ]
        )
        .circle(0.003)
        .extrude(0.002)
        .translate((0.052, 0.0, 0.068))
    )

    return _fuse(lower, top, pedestal, nose, disc, disc_bolts)


def _make_shoulder_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.022).extrude(0.014)
    flange = cq.Workplane("XY").rect(0.048, 0.048).extrude(0.004).translate((0.0, 0.0, 0.014))
    flange_bolts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.015, -0.015),
                (-0.015, 0.015),
                (0.015, -0.015),
                (0.015, 0.015),
            ]
        )
        .circle(0.003)
        .extrude(0.002)
        .translate((0.0, 0.0, 0.018))
    )
    web = _box((0.032, 0.05, 0.024), (0.02, 0.0, 0.02))
    beam = _box((0.108, 0.03, 0.024), (0.055, 0.0, 0.026))
    top_rib = _box((0.07, 0.05, 0.008), (0.055, 0.0, 0.042))
    clevis_left = _box((0.03, 0.008, 0.042), (0.112, 0.018, 0.028))
    clevis_right = _box((0.03, 0.008, 0.042), (0.112, -0.018, 0.028))

    return _fuse(hub, flange, flange_bolts, web, beam, top_rib, clevis_left, clevis_right)


def _make_forearm_shape() -> cq.Workplane:
    root_lug = cq.Workplane("XZ").circle(0.013).extrude(0.022).translate((0.0, -0.011, 0.0))
    beam = _box((0.098, 0.022, 0.02), (0.06, 0.0, 0.0))
    upper_rib = _box((0.078, 0.042, 0.006), (0.06, 0.0, 0.013))
    end_pad = _box((0.026, 0.045, 0.028), (0.113, 0.0, 0.0))
    tool_face = _box((0.008, 0.032, 0.04), (0.128, 0.0, 0.0))
    tip_bolts = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.01, -0.01),
                (-0.01, 0.01),
                (0.01, -0.01),
                (0.01, 0.01),
            ]
        )
        .circle(0.0025)
        .extrude(0.002)
        .translate((0.132, 0.0, 0.0))
    )

    return _fuse(root_lug, beam, upper_rib, end_pad, tool_face, tip_bolts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_dual_rotary_module", assets=ASSETS)

    model.material("base_steel", rgba=(0.53, 0.56, 0.60, 1.0))
    model.material("carriage_dark", rgba=(0.20, 0.22, 0.26, 1.0))
    model.material("joint_blue", rgba=(0.24, 0.39, 0.65, 1.0))
    model.material("arm_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "module_base.obj", "base_steel")




    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.12, 0.04)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    carriage = model.part("carriage")
    _add_mesh_visual(carriage, _make_carriage_shape(), "module_carriage.obj", "carriage_dark")



    carriage.inertial = Inertial.from_geometry(
        Box((0.11, 0.09, 0.07)),
        mass=1.7,
        origin=Origin(xyz=(0.01, 0.0, 0.035)),
    )

    shoulder = model.part("shoulder_link")
    _add_mesh_visual(shoulder, _make_shoulder_shape(), "module_shoulder.obj", "joint_blue")




    shoulder.inertial = Inertial.from_geometry(
        Box((0.125, 0.05, 0.05)),
        mass=1.15,
        origin=Origin(xyz=(0.06, 0.0, 0.025)),
    )

    forearm = model.part("forearm_link")
    _add_mesh_visual(forearm, _make_forearm_shape(), "module_forearm.obj", "arm_silver")



    forearm.inertial = Inertial.from_geometry(
        Box((0.135, 0.045, 0.04)),
        mass=0.85,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.08, 0.0, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.14, effort=250.0, velocity=0.25),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.052, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=40.0, velocity=1.2),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.112, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.1, effort=25.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "carriage_to_shoulder",
        "shoulder_link",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "shoulder_to_forearm",
        "forearm_link",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )

    ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap("carriage", "base", axis="z", max_gap=0.01, max_penetration=0.01)
    ctx.expect_aabb_overlap("shoulder_link", "carriage", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_gap("shoulder_link", "carriage", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_origin_distance("shoulder_link", "carriage", axes="xy", max_dist=0.12)
    ctx.expect_origin_distance("forearm_link", "shoulder_link", axes="xy", max_dist=0.14)
    ctx.expect_origin_gap("shoulder_link", "base", axis="z", min_gap=0.02)
    ctx.expect_origin_gap("forearm_link", "carriage", axis="z", min_gap=0.01)

    with _pose(ctx, base_to_carriage=0.14):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_gap("carriage", "base", axis="z", max_gap=0.01, max_penetration=0.01)

    with _pose(ctx, carriage_to_shoulder=0.9):
        ctx.expect_origin_distance("shoulder_link", "carriage", axes="xy", max_dist=0.12)
        ctx.expect_origin_gap("shoulder_link", "base", axis="z", min_gap=0.02)

    with _pose(ctx, shoulder_to_forearm=-0.45):
        ctx.expect_origin_gap("forearm_link", "base", axis="z", min_gap=0.0)
        ctx.expect_origin_distance("forearm_link", "shoulder_link", axes="xy", max_dist=0.16)

    with _pose(ctx, shoulder_to_forearm=1.0):
        ctx.expect_origin_gap("forearm_link", "carriage", axis="z", min_gap=0.04)
        ctx.expect_origin_distance("forearm_link", "shoulder_link", axes="xy", max_dist=0.16)

    with _pose(ctx, base_to_carriage=0.14, carriage_to_shoulder=0.9, shoulder_to_forearm=1.0):
        ctx.expect_origin_gap("forearm_link", "base", axis="z", min_gap=0.04)
        ctx.expect_origin_distance("forearm_link", "carriage", axes="xy", max_dist=0.24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

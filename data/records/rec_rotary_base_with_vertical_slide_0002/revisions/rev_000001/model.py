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
BASE_SIZE = (0.30, 0.24, 0.05)
ROTARY_JOINT_Z = 0.034
LIFT_JOINT_ORIGIN = (0.076, 0.0, 0.145)
LIFT_TRAVEL = 0.18


def _rounded_box(
    size: tuple[float, float, float],
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    radius: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(center)


def _cylinder(
    radius: float,
    height: float,
    *,
    bottom_z: float = 0.0,
    center_xy: tuple[float, float] = (0.0, 0.0),
) -> cq.Workplane:
    x, y = center_xy
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, bottom_z))
    )


def _export_visual(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape.val(), filename, assets=ASSETS)


def _make_base_shape() -> cq.Workplane:
    body = _rounded_box(BASE_SIZE, radius=0.010)
    top_plinth = _rounded_box((0.18, 0.15, 0.014), center=(0.006, 0.0, 0.018), radius=0.004)
    rear_drive_cover = _rounded_box(
        (0.095, 0.170, 0.028),
        center=(-0.045, 0.0, 0.001),
        radius=0.006,
    )
    bearing_seat = _cylinder(0.055, 0.008, bottom_z=0.025)
    return body.union(top_plinth).union(rear_drive_cover).union(bearing_seat)


def _make_rotary_head_shape() -> cq.Workplane:
    turntable = _cylinder(0.090, 0.018, bottom_z=0.0)
    hub = _cylinder(0.048, 0.012, bottom_z=0.0)
    column = _rounded_box((0.082, 0.072, 0.300), center=(0.030, 0.0, 0.168), radius=0.005)
    top_cap = _rounded_box((0.110, 0.084, 0.018), center=(0.030, 0.0, 0.327), radius=0.004)
    guide_rail = _rounded_box((0.012, 0.034, 0.240), center=(0.069, 0.0, 0.170), radius=0.001)
    side_motor = _rounded_box((0.050, 0.092, 0.046), center=(-0.030, 0.0, 0.038), radius=0.004)
    return turntable.union(hub).union(column).union(top_cap).union(guide_rail).union(side_motor)


def _make_carriage_shape() -> cq.Workplane:
    guide_shoe = _rounded_box((0.018, 0.048, 0.070), center=(0.009, 0.0, 0.0), radius=0.001)
    carriage_body = _rounded_box((0.058, 0.094, 0.080), center=(0.039, 0.0, 0.0), radius=0.004)
    top_plate = _rounded_box((0.092, 0.072, 0.014), center=(0.046, 0.0, 0.047), radius=0.002)
    front_face = _rounded_box((0.010, 0.060, 0.060), center=(0.078, 0.0, 0.0), radius=0.001)
    tool_block = _rounded_box((0.040, 0.040, 0.028), center=(0.064, 0.0, 0.053), radius=0.002)
    return guide_shoe.union(carriage_body).union(top_plate).union(front_face).union(tool_block)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_lift_module", assets=ASSETS)

    model.material("base_paint", rgba=(0.26, 0.29, 0.31, 1.0))
    model.material("machined_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("safety_orange", rgba=(0.92, 0.46, 0.12, 1.0))

    base = model.part("base_frame")
    base.visual(_export_visual(_make_base_shape(), "base_frame.obj"), material="base_paint")


    base.inertial = Inertial.from_geometry(Box((0.30, 0.24, 0.07)), mass=12.0)

    rotary = model.part("rotary_head")
    rotary.visual(
        _export_visual(_make_rotary_head_shape(), "rotary_head.obj"), material="machined_steel"
    )



    rotary.inertial = Inertial.from_geometry(
        Box((0.20, 0.18, 0.34)),
        mass=6.5,
        origin=Origin(xyz=(0.025, 0.0, 0.170)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        _export_visual(_make_carriage_shape(), "carriage.obj"), material="safety_orange"
    )



    carriage.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.10)),
        mass=2.2,
        origin=Origin(xyz=(0.045, 0.0, 0.010)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.5, upper=2.5, effort=25.0, velocity=1.5),
    )
    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=rotary,
        child=carriage,
        origin=Origin(xyz=LIFT_JOINT_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=120.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("rotary_head", "base_frame", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_gap("rotary_head", "base_frame", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_origin_gap("carriage", "base_frame", axis="z", min_gap=0.08)
    ctx.expect_aabb_overlap("carriage", "rotary_head", axes="xy", min_overlap=0.01)
    ctx.expect_origin_distance("carriage", "base_frame", axes="xy", max_dist=0.13)
    ctx.expect_joint_motion_axis(
        "base_yaw", "carriage", world_axis="y", direction="positive", min_delta=0.01
    )
    ctx.expect_joint_motion_axis(
        "column_lift", "carriage", world_axis="z", direction="positive", min_delta=0.02
    )

    with ctx.pose(column_lift=LIFT_TRAVEL):
        ctx.expect_origin_gap("carriage", "base_frame", axis="z", min_gap=0.20)
        ctx.expect_aabb_overlap("carriage", "rotary_head", axes="xy", min_overlap=0.01)
        ctx.expect_origin_distance("carriage", "base_frame", axes="xy", max_dist=0.13)

    with ctx.pose(base_yaw=1.2):
        ctx.expect_aabb_overlap("rotary_head", "base_frame", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("rotary_head", "base_frame", axes="xy", max_dist=0.03)
        ctx.expect_origin_distance("carriage", "base_frame", axes="xy", max_dist=0.13)

    with ctx.pose(base_yaw=1.2, column_lift=LIFT_TRAVEL):
        ctx.expect_origin_gap("carriage", "base_frame", axis="z", min_gap=0.20)
        ctx.expect_origin_distance("carriage", "base_frame", axes="xy", max_dist=0.13)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

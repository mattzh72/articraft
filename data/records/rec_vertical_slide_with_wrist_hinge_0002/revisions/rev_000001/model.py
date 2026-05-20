from __future__ import annotations

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


# >>> USER_CODE_START
import cadquery as cq


def _add_visual_mesh(part, shape, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_column_shape():
    base_plate = (
        cq.Workplane("XY")
        .box(0.22, 0.14, 0.04)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, 0.02))
    )
    back_plate = (
        cq.Workplane("XY")
        .box(0.16, 0.014, 0.44)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, -0.020, 0.26))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.18, 0.05, 0.025)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -0.005, 0.4925))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.40)
        .edges("|Z")
        .fillet(0.0015)
        .translate((-0.055, 0.0, 0.26))
    )
    rail_right = rail_left.translate((0.11, 0.0, 0.0))
    lower_stiffener = (
        cq.Workplane("XY")
        .box(0.13, 0.024, 0.05)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, -0.006, 0.085))
    )
    upper_stiffener = lower_stiffener.translate((0.0, 0.0, 0.28))
    return (
        base_plate.union(back_plate)
        .union(top_cap)
        .union(rail_left)
        .union(rail_right)
        .union(lower_stiffener)
        .union(upper_stiffener)
    )


def _make_bearing_block():
    body = cq.Workplane("XY").box(0.040, 0.032, 0.058).edges("|Z").fillet(0.003)
    rail_relief = cq.Workplane("XY").box(0.022, 0.018, 0.048).translate((0.0, -0.004, 0.0))
    top_cap = cq.Workplane("XY").box(0.028, 0.008, 0.010).translate((0.0, 0.014, 0.016))
    return body.cut(rail_relief).union(top_cap)


def _make_carriage_shape():
    block_left = _make_bearing_block().translate((-0.055, 0.006, 0.0))
    block_right = _make_bearing_block().translate((0.055, 0.006, 0.0))
    bridge = (
        cq.Workplane("XY")
        .box(0.15, 0.024, 0.086)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.018, 0.0))
    )
    wrist_mount = (
        cq.Workplane("XY")
        .box(0.072, 0.028, 0.072)
        .edges("|X")
        .fillet(0.004)
        .translate((0.0, 0.046, 0.0))
    )
    rotary_boss = (
        cq.Workplane("YZ").circle(0.024).extrude(0.016, both=True).translate((0.0, 0.060, 0.0))
    )
    return block_left.union(block_right).union(bridge).union(wrist_mount).union(rotary_boss)


def _make_wrist_shape():
    mount_pad = (
        cq.Workplane("XY")
        .box(0.070, 0.014, 0.060)
        .edges("|X")
        .fillet(0.003)
        .translate((0.0, 0.007, 0.0))
    )
    housing = (
        cq.Workplane("YZ").circle(0.030).extrude(0.050, both=True).translate((0.0, 0.033, 0.0))
    )
    arm = (
        cq.Workplane("XY")
        .box(0.050, 0.080, 0.018)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.074, 0.0))
    )
    tool_flange = (
        cq.Workplane("XZ").circle(0.026).extrude(0.006, both=True).translate((0.0, 0.115, 0.0))
    )
    return mount_pad.union(housing).union(arm).union(tool_flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="z_axis_wrist_module", assets=ASSETS)

    model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("ground_rail", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("machine_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("safety_orange", rgba=(0.88, 0.50, 0.12, 1.0))

    column = model.part("column")
    carriage = model.part("carriage")
    wrist = model.part("wrist")

    _add_visual_mesh(column, _make_column_shape(), "column.obj", "machine_gray")
    _add_visual_mesh(carriage, _make_carriage_shape(), "carriage.obj", "painted_steel")
    _add_visual_mesh(wrist, _make_wrist_shape(), "wrist.obj", "safety_orange")




    column.inertial = Inertial.from_geometry(
        Box((0.22, 0.14, 0.50)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -0.006, 0.25)),
    )




    carriage.inertial = Inertial.from_geometry(
        Box((0.15, 0.060, 0.090)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
    )





    wrist.inertial = Inertial.from_geometry(
        Box((0.10, 0.121, 0.08)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0605, 0.0)),
    )

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.004, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.22,
            effort=900.0,
            velocity=0.35,
        ),
    )

    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.85,
            upper=0.90,
            effort=45.0,
            velocity=2.5,
        ),
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
        "z_slide",
        "carriage",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "wrist",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(z_slide=0.0, wrist_pitch=0.0):
        ctx.expect_origin_distance("carriage", "column", axes="xy", max_dist=0.02)
        ctx.expect_aabb_overlap("carriage", "column", axes="xy", min_overlap=0.05)
        ctx.expect_aabb_overlap("wrist", "carriage", axes="xy", min_overlap=0.02)

    with ctx.pose(z_slide=0.22, wrist_pitch=0.0):
        ctx.expect_origin_distance("carriage", "column", axes="xy", max_dist=0.02)
        ctx.expect_aabb_overlap("carriage", "column", axes="xy", min_overlap=0.05)

    with ctx.pose(z_slide=0.18, wrist_pitch=0.75):
        ctx.expect_aabb_overlap("wrist", "carriage", axes="xy", min_overlap=0.02)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

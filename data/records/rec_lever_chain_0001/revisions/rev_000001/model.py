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
BAR_THICKNESS = 0.004
PIVOT_RADIUS = 0.012
PIVOT_HOLE_RADIUS = 0.0032
BASE_THICKNESS = 0.012
FIRST_JOINT_Z = 0.018
JOINT_RISE = 0.012
PIVOT_CLEARANCE = 0.0005

INPUT_LENGTH = 0.110
RELAY_LENGTH = 0.095
FOLLOWER_LENGTH = 0.082
OUTPUT_LENGTH = 0.068


def _lever_shape(
    *,
    length: float,
    width: float,
    pad_radius: float,
    hole_radius: float,
    thickness: float,
    standoff_height: float,
    tip_radius: float | None = None,
) -> cq.Workplane:
    tip_radius = pad_radius if tip_radius is None else tip_radius
    shape = (
        cq.Workplane("XY")
        .circle(pad_radius)
        .extrude(thickness)
        .union(
            cq.Workplane("XY", origin=(length / 2.0, 0.0, 0.0)).box(
                length,
                width,
                thickness,
                centered=(True, True, False),
            )
        )
        .union(cq.Workplane("XY", origin=(length, 0.0, 0.0)).circle(tip_radius).extrude(thickness))
    )

    if standoff_height > thickness:
        shape = shape.union(
            cq.Workplane("XY", origin=(length, 0.0, thickness))
            .circle(pad_radius * 0.64)
            .extrude(standoff_height - thickness)
        )

    slot_length = length - 2.8 * pad_radius
    if slot_length > 0.014:
        shape = shape.cut(
            cq.Workplane("XY", origin=(length / 2.0, 0.0, -0.001))
            .slot2D(slot_length, width * 0.40, 0.0)
            .extrude(thickness + 0.002)
        )

    hole_depth = max(thickness, standoff_height) + 0.002
    shape = shape.cut(
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .pushPoints([(0.0, 0.0), (length, 0.0)])
        .circle(hole_radius)
        .extrude(hole_depth)
    )
    return shape


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY", origin=(0.030, 0.0, 0.0))
        .box(0.160, 0.085, BASE_THICKNESS, centered=(True, True, False))
        .cut(
            cq.Workplane("XY", origin=(0.070, 0.025, -0.001))
            .slot2D(0.022, 0.006, 0.0)
            .extrude(BASE_THICKNESS + 0.002)
        )
        .cut(
            cq.Workplane("XY", origin=(0.070, -0.025, -0.001))
            .slot2D(0.022, 0.006, 0.0)
            .extrude(BASE_THICKNESS + 0.002)
        )
    )

    pedestal_height = FIRST_JOINT_Z - BASE_THICKNESS - PIVOT_CLEARANCE
    pedestal = (
        cq.Workplane("XY", origin=(0.0, 0.0, BASE_THICKNESS))
        .circle(PIVOT_RADIUS * 1.10)
        .extrude(pedestal_height)
    )

    return foot.union(pedestal).cut(
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(PIVOT_HOLE_RADIUS)
        .extrude(FIRST_JOINT_Z + 0.002)
    )


def _add_base_part(model: ArticulatedObject):
    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base.obj", assets=ASSETS), material="graphite")

    pedestal_height = FIRST_JOINT_Z - BASE_THICKNESS - PIVOT_CLEARANCE

    base.inertial = Inertial.from_geometry(
        Box((0.160, 0.085, FIRST_JOINT_Z)),
        mass=1.5,
        origin=Origin(xyz=(0.030, 0.0, FIRST_JOINT_Z / 2.0)),
    )
    return base


def _add_lever_part(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    pad_radius: float,
    tip_radius: float,
    hole_radius: float,
    standoff_height: float,
    mass: float,
    material: str,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(
            _lever_shape(
                length=length,
                width=width,
                pad_radius=pad_radius,
                hole_radius=hole_radius,
                thickness=BAR_THICKNESS,
                standoff_height=standoff_height,
                tip_radius=tip_radius,
            ),
            f"{name}.obj",
            assets=ASSETS,
        ),
        material=material,
    )

    strip_length = max(length - 2.1 * pad_radius, 0.012)


    outboard_height = max(BAR_THICKNESS, standoff_height)


    overall_x = length + pad_radius + tip_radius
    center_x = (length + tip_radius - pad_radius) / 2.0
    overall_z = max(BAR_THICKNESS, standoff_height)
    part.inertial = Inertial.from_geometry(
        Box((overall_x, max(width, 2.0 * pad_radius), overall_z)),
        mass=mass,
        origin=Origin(xyz=(center_x, 0.0, overall_z / 2.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_link_lever_mechanism", assets=ASSETS)

    model.material("graphite", rgba=(0.22, 0.24, 0.28, 1.0))
    model.material("blue_lever", rgba=(0.22, 0.46, 0.76, 1.0))
    model.material("amber_lever", rgba=(0.85, 0.55, 0.16, 1.0))
    model.material("green_lever", rgba=(0.33, 0.61, 0.33, 1.0))
    model.material("silver_lever", rgba=(0.72, 0.74, 0.78, 1.0))

    base = _add_base_part(model)
    input_lever = _add_lever_part(
        model,
        name="input_lever",
        length=INPUT_LENGTH,
        width=0.020,
        pad_radius=0.013,
        tip_radius=0.012,
        hole_radius=PIVOT_HOLE_RADIUS,
        standoff_height=JOINT_RISE - PIVOT_CLEARANCE,
        mass=0.24,
        material="blue_lever",
    )
    relay_arm = _add_lever_part(
        model,
        name="relay_arm",
        length=RELAY_LENGTH,
        width=0.018,
        pad_radius=0.012,
        tip_radius=0.0115,
        hole_radius=PIVOT_HOLE_RADIUS,
        standoff_height=JOINT_RISE - PIVOT_CLEARANCE,
        mass=0.19,
        material="amber_lever",
    )
    follower_arm = _add_lever_part(
        model,
        name="follower_arm",
        length=FOLLOWER_LENGTH,
        width=0.017,
        pad_radius=0.0115,
        tip_radius=0.011,
        hole_radius=PIVOT_HOLE_RADIUS,
        standoff_height=JOINT_RISE - PIVOT_CLEARANCE,
        mass=0.16,
        material="green_lever",
    )
    output_lever = _add_lever_part(
        model,
        name="output_lever",
        length=OUTPUT_LENGTH,
        width=0.016,
        pad_radius=0.011,
        tip_radius=0.010,
        hole_radius=PIVOT_HOLE_RADIUS,
        standoff_height=BAR_THICKNESS,
        mass=0.12,
        material="silver_lever",
    )

    model.articulation(
        "base_to_input",
        ArticulationType.REVOLUTE,
        parent=base,
        child=input_lever,
        origin=Origin(xyz=(0.0, 0.0, FIRST_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=8.0, velocity=2.5),
    )
    model.articulation(
        "input_to_relay",
        ArticulationType.REVOLUTE,
        parent=input_lever,
        child=relay_arm,
        origin=Origin(xyz=(INPUT_LENGTH, 0.0, JOINT_RISE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=6.0, velocity=2.8),
    )
    model.articulation(
        "relay_to_follower",
        ArticulationType.REVOLUTE,
        parent=relay_arm,
        child=follower_arm,
        origin=Origin(xyz=(RELAY_LENGTH, 0.0, JOINT_RISE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=5.0, velocity=3.0),
    )
    model.articulation(
        "follower_to_output",
        ArticulationType.REVOLUTE,
        parent=follower_arm,
        child=output_lever,
        origin=Origin(xyz=(FOLLOWER_LENGTH, 0.0, JOINT_RISE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.90, effort=4.0, velocity=3.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "base_to_input",
        "input_lever",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "input_to_relay",
        "relay_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "relay_to_follower",
        "follower_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "follower_to_output",
        "output_lever",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )

    ctx.expect_aabb_overlap("input_lever", "base", axes="xy", min_overlap=0.016)
    ctx.expect_aabb_gap("input_lever", "base", axis="z", max_gap=0.0015, max_penetration=0.0)
    ctx.expect_aabb_overlap("relay_arm", "input_lever", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_gap("relay_arm", "input_lever", axis="z", max_gap=0.0015, max_penetration=0.0)
    ctx.expect_aabb_overlap("follower_arm", "relay_arm", axes="xy", min_overlap=0.011)
    ctx.expect_aabb_gap("follower_arm", "relay_arm", axis="z", max_gap=0.0015, max_penetration=0.0)
    ctx.expect_aabb_overlap("output_lever", "follower_arm", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_gap("output_lever", "follower_arm", axis="z", max_gap=0.0015, max_penetration=0.0)
    ctx.expect_origin_gap("output_lever", "base", axis="z", min_gap=0.02)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

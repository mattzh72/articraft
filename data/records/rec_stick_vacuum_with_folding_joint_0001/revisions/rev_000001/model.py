from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, MESH_DIR / name)


def _build_body_shell_mesh():
    profile = [
        (0.015, -0.065),
        (0.027, -0.055),
        (0.035, -0.018),
        (0.039, 0.048),
        (0.036, 0.106),
        (0.024, 0.125),
    ]
    return _mesh(
        "vacuum_body_shell.obj", LatheGeometry(profile, segments=52).translate(0.0, 0.0, 0.01)
    )


def _build_handle_mesh():
    return _mesh(
        "vacuum_handle.obj",
        tube_from_spline_points(
            [
                (0.0, -0.028, -0.022),
                (0.0, -0.058, 0.032),
                (0.0, -0.074, 0.118),
                (0.0, -0.050, 0.188),
                (0.0, -0.017, 0.160),
                (0.0, 0.004, 0.094),
            ],
            radius=0.014,
            samples_per_segment=20,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def _build_head_shell_mesh():
    shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.272, 0.102, 0.026, corner_segments=12),
        0.018,
        cap=True,
        closed=True,
    ).translate(0.0, 0.012, -0.040)
    crown = BoxGeometry((0.150, 0.050, 0.010)).translate(0.0, 0.010, -0.026)
    front_lip = BoxGeometry((0.228, 0.012, 0.007)).translate(0.0, 0.054, -0.034)
    rear_ridge = BoxGeometry((0.180, 0.016, 0.009)).translate(0.0, -0.037, -0.033)
    shell.merge(crown).merge(front_lip).merge(rear_ridge)
    return _mesh("vacuum_floor_head_shell.obj", shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stick_vacuum", assets=ASSETS)

    satin_graphite = Material(name="satin_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    brushed_aluminum = Material(name="brushed_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    soft_black = Material(name="soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    smoke_clear = Material(name="smoke_clear", rgba=(0.72, 0.80, 0.88, 0.34))
    bronze_accent = Material(name="bronze_accent", rgba=(0.72, 0.42, 0.16, 1.0))
    model.materials.extend(
        [satin_graphite, brushed_aluminum, soft_black, smoke_clear, bronze_accent]
    )

    body = model.part("body")
    body.visual(_build_body_shell_mesh(), material=satin_graphite)
    body.visual(
        Box((0.048, 0.034, 0.140)),
        origin=Origin(xyz=(0.0, -0.024, 0.000)),
        material=satin_graphite,
    )
    body.visual(
        Cylinder(radius=0.011, length=0.128),
        origin=Origin(xyz=(0.0, -0.074, 0.108), rpy=(0.72, 0.0, 0.0)),
        material=soft_black,
    )
    body.visual(
        Cylinder(radius=0.010, length=0.088),
        origin=Origin(xyz=(0.0, -0.027, 0.118), rpy=(0.10, 0.0, 0.0)),
        material=soft_black,
    )
    body.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, -0.046, 0.176), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
    )
    body.visual(
        Box((0.020, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, -0.026, 0.090)),
        material=soft_black,
    )
    body.visual(
        Cylinder(radius=0.031, length=0.160),
        origin=Origin(xyz=(0.0, 0.010, -0.130)),
        material=smoke_clear,
    )
    body.visual(
        Cylinder(radius=0.036, length=0.032),
        origin=Origin(xyz=(0.0, 0.000, -0.065)),
        material=bronze_accent,
    )
    body.visual(
        Cylinder(radius=0.019, length=0.086),
        origin=Origin(xyz=(0.0, 0.000, -0.246)),
        material=brushed_aluminum,
    )
    body.visual(
        Box((0.022, 0.046, 0.008)),
        origin=Origin(xyz=(0.0, 0.000, 0.137)),
        material=soft_black,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.120, 0.150, 0.420)),
        mass=2.35,
        origin=Origin(xyz=(0.0, -0.010, -0.020)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.016, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.010, -0.016)),
        material=soft_black,
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.016, 0.010, 0.030)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.010, -0.016)),
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(Box((0.012, 0.022, 0.006)), material=bronze_accent)
    mode_slider.inertial = Inertial.from_geometry(
        Box((0.012, 0.022, 0.006)),
        mass=0.02,
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.0165, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=brushed_aluminum,
    )
    wand.visual(
        Cylinder(radius=0.0190, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=brushed_aluminum,
    )
    wand.visual(
        Cylinder(radius=0.0185, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.545)),
        material=soft_black,
    )
    wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.565),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, -0.2825)),
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=soft_black,
    )
    swivel.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=soft_black,
    )
    swivel.visual(
        Box((0.086, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=soft_black,
    )
    swivel.visual(
        Cylinder(radius=0.009, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.076), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
    )
    swivel.visual(
        Box((0.012, 0.018, 0.052)),
        origin=Origin(xyz=(-0.036, 0.0, -0.069)),
        material=soft_black,
    )
    swivel.visual(
        Box((0.012, 0.018, 0.052)),
        origin=Origin(xyz=(0.036, 0.0, -0.069)),
        material=soft_black,
    )
    swivel.inertial = Inertial.from_geometry(
        Box((0.090, 0.030, 0.100)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(_build_head_shell_mesh(), material=satin_graphite)
    floor_head.visual(
        Box((0.060, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=soft_black,
    )
    floor_head.visual(
        Cylinder(radius=0.0065, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
    )
    floor_head.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(-0.024, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
    )
    floor_head.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.024, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
    )
    floor_head.visual(
        Box((0.240, 0.068, 0.006)),
        origin=Origin(xyz=(0.0, 0.016, -0.040)),
        material=soft_black,
    )
    floor_head.visual(
        Box((0.168, 0.012, 0.007)),
        origin=Origin(xyz=(0.0, 0.050, -0.032)),
        material=brushed_aluminum,
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.280, 0.110, 0.050)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.012, -0.028)),
    )

    model.articulation(
        "trigger_pull",
        ArticulationType.REVOLUTE,
        parent="body",
        child="trigger",
        origin=Origin(xyz=(0.0, -0.043, 0.105)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "power_slider",
        ArticulationType.PRISMATIC,
        parent="body",
        child="mode_slider",
        origin=Origin(xyz=(0.0, -0.010, 0.143)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.15,
            lower=0.0,
            upper=0.014,
        ),
    )
    model.articulation(
        "body_to_wand",
        ArticulationType.FIXED,
        parent="body",
        child="wand",
        origin=Origin(xyz=(0.0, 0.0, -0.289)),
    )
    model.articulation(
        "wand_swivel",
        ArticulationType.REVOLUTE,
        parent="wand",
        child="swivel",
        origin=Origin(xyz=(0.0, 0.0, -0.555)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.5,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent="swivel",
        child="floor_head",
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=2.5,
            lower=0.0,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "body",
        "trigger",
        reason="Trigger nests inside the handle guard and conservative generated collisions read the close packaging as overlap.",
    )
    ctx.allow_overlap(
        "body",
        "mode_slider",
        reason="The slider rides in a recessed control track with slight intentional inset into the motor housing.",
    )
    ctx.allow_overlap(
        "floor_head",
        "swivel",
        reason="The folding neck uses an enclosed socket around the swivel yoke, so convex collision hulls overlap at the hinge.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    body_pos = ctx.part_world_position("body")
    slider_rest = ctx.part_world_position("mode_slider")
    wand_pos = ctx.part_world_position("wand")
    swivel_pos = ctx.part_world_position("swivel")
    head_pos = ctx.part_world_position("floor_head")
    if not (body_pos[2] > wand_pos[2] > swivel_pos[2] > head_pos[2]):
        raise AssertionError("Vacuum assembly should step downward from body to wand to head.")
    if not slider_rest[2] > body_pos[2] + 0.12:
        raise AssertionError("Mode slider should sit high on the motor body spine.")
    if not abs(slider_rest[0] - body_pos[0]) < 0.01:
        raise AssertionError("Mode slider should remain centered on the vacuum body.")

    ctx.expect_origin_distance("body", "wand", axes="xy", max_dist=0.03)
    ctx.expect_aabb_gap("body", "wand", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_gap("wand", "swivel", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_overlap("floor_head", "wand", axes="xy", min_overlap=0.012)
    ctx.expect_origin_distance("floor_head", "body", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("trigger", "body", axes="xy", max_dist=0.08)

    ctx.expect_joint_motion_axis(
        "trigger_pull",
        "trigger",
        world_axis="y",
        direction="negative",
        min_delta=0.004,
    )
    ctx.expect_joint_motion_axis(
        "power_slider",
        "mode_slider",
        world_axis="y",
        direction="positive",
        min_delta=0.006,
    )
    ctx.expect_joint_motion_axis(
        "wand_swivel",
        "floor_head",
        world_axis="x",
        direction="negative",
        min_delta=0.006,
    )
    ctx.expect_joint_motion_axis(
        "head_pitch",
        "floor_head",
        world_axis="y",
        direction="positive",
        min_delta=0.010,
    )

    with ctx.pose(trigger_pull=0.42):
        ctx.expect_origin_distance("trigger", "body", axes="xy", max_dist=0.08)

    with ctx.pose(power_slider=0.014):
        ctx.expect_origin_distance("mode_slider", "body", axes="xy", max_dist=0.04)
        slider_high = ctx.part_world_position("mode_slider")
        if not slider_high[1] > slider_rest[1] + 0.010:
            raise AssertionError("Mode slider should travel forward along its track.")
        if not abs(slider_high[2] - slider_rest[2]) < 0.002:
            raise AssertionError("Mode slider should stay level while sliding.")

    with ctx.pose(head_pitch=0.75):
        ctx.expect_aabb_overlap("floor_head", "wand", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("floor_head", "body", axes="xy", max_dist=0.07)

    with ctx.pose(wand_swivel=0.75):
        ctx.expect_aabb_overlap("floor_head", "wand", axes="xy", min_overlap=0.010)
        ctx.expect_origin_distance("floor_head", "body", axes="xy", max_dist=0.06)

    with ctx.pose(wand_swivel=-0.75):
        ctx.expect_aabb_overlap("floor_head", "wand", axes="xy", min_overlap=0.010)
        ctx.expect_origin_distance("floor_head", "body", axes="xy", max_dist=0.06)

    with ctx.pose(wand_swivel=0.55, head_pitch=0.55):
        ctx.expect_aabb_overlap("floor_head", "wand", axes="xy", min_overlap=0.006)
        ctx.expect_origin_distance("floor_head", "body", axes="xy", max_dist=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

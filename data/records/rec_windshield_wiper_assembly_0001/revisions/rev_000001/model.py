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
    CylinderGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(ASSETS.asset_root)
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _build_base_mesh():
    base = BoxGeometry((0.052, 0.032, 0.008)).translate(0.0, 0.0, 0.004)
    pedestal = BoxGeometry((0.041, 0.026, 0.010)).translate(-0.001, 0.0, 0.013)
    cap = BoxGeometry((0.026, 0.020, 0.007)).translate(0.008, 0.0, 0.019)
    rear_rib = BoxGeometry((0.018, 0.016, 0.008)).translate(-0.014, 0.0, 0.010)
    shoulder_right = BoxGeometry((0.014, 0.006, 0.010)).translate(0.004, 0.010, 0.013)
    shoulder_left = BoxGeometry((0.014, 0.006, 0.010)).translate(0.004, -0.010, 0.013)
    nose = BoxGeometry((0.016, 0.016, 0.005)).rotate_y(-0.18).translate(0.016, 0.0, 0.021)
    base.merge(pedestal).merge(cap).merge(rear_rib)
    base.merge(shoulder_right).merge(shoulder_left).merge(nose)
    return base


def _build_arm_mesh():
    arm = BoxGeometry((0.090, 0.020, 0.0054)).translate(0.058, 0.0005, 0.0014)
    mid = BoxGeometry((0.112, 0.015, 0.0045)).translate(0.160, 0.0022, 0.0018)
    tip = BoxGeometry((0.080, 0.010, 0.0034)).translate(0.262, 0.0040, 0.0007)
    hub = CylinderGeometry(radius=0.014, height=0.005, radial_segments=28).translate(0.0, 0.0, 0.0)
    knuckle = BoxGeometry((0.022, 0.022, 0.007)).translate(0.018, 0.0, 0.0005)
    adapter = BoxGeometry((0.032, 0.018, 0.006)).translate(0.300, 0.0045, 0.0000)
    bridge_plate = BoxGeometry((0.026, 0.013, 0.0022)).translate(0.108, 0.0, 0.0125)
    spring_path = [
        (0.022, 0.0058, 0.0012),
        (0.090, 0.0068, 0.0138),
        (0.170, 0.0060, 0.0090),
        (0.228, 0.0054, 0.0050),
    ]
    arm.merge(mid).merge(tip).merge(hub).merge(knuckle).merge(adapter).merge(bridge_plate)
    arm.merge(
        tube_from_spline_points(
            spring_path,
            radius=0.00135,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        )
    )
    arm.merge(
        tube_from_spline_points(
            _mirror_y(spring_path),
            radius=0.00135,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        )
    )
    return arm


def _build_blade_frame_mesh():
    frame = BoxGeometry((0.010, 0.308, 0.004)).translate(0.0, 0.0, -0.0040)
    saddle = BoxGeometry((0.026, 0.030, 0.006)).translate(0.0, 0.0, -0.0010)
    spoiler = BoxGeometry((0.012, 0.252, 0.0035)).rotate_y(-0.32).translate(0.0048, 0.0, 0.0002)
    inner_right = BoxGeometry((0.010, 0.104, 0.0028)).rotate_x(-0.12).translate(0.0, 0.056, 0.0018)
    inner_left = BoxGeometry((0.010, 0.104, 0.0028)).rotate_x(0.12).translate(0.0, -0.056, 0.0018)
    outer_right = BoxGeometry((0.008, 0.082, 0.0026)).rotate_x(-0.18).translate(0.0, 0.135, -0.0008)
    outer_left = BoxGeometry((0.008, 0.082, 0.0026)).rotate_x(0.18).translate(0.0, -0.135, -0.0008)
    claw_inner_right = BoxGeometry((0.006, 0.016, 0.010)).translate(0.0, 0.094, -0.0045)
    claw_inner_left = BoxGeometry((0.006, 0.016, 0.010)).translate(0.0, -0.094, -0.0045)
    claw_outer_right = BoxGeometry((0.006, 0.016, 0.010)).translate(0.0, 0.152, -0.0048)
    claw_outer_left = BoxGeometry((0.006, 0.016, 0.010)).translate(0.0, -0.152, -0.0048)
    end_cap_right = BoxGeometry((0.010, 0.012, 0.010)).translate(0.0, 0.167, -0.0048)
    end_cap_left = BoxGeometry((0.010, 0.012, 0.010)).translate(0.0, -0.167, -0.0048)
    frame.merge(saddle).merge(spoiler)
    frame.merge(inner_right).merge(inner_left).merge(outer_right).merge(outer_left)
    frame.merge(claw_inner_right).merge(claw_inner_left).merge(claw_outer_right).merge(
        claw_outer_left
    )
    frame.merge(end_cap_right).merge(end_cap_left)
    return frame


def build_object_model() -> ArticulatedObject:
    painted_black = _make_material("painted_black", (0.12, 0.12, 0.13, 1.0))
    textured_black = _make_material("textured_black", (0.08, 0.08, 0.09, 1.0))
    rubber_black = _make_material("rubber_black", (0.03, 0.03, 0.03, 1.0))
    steel = _make_material("brushed_steel", (0.57, 0.59, 0.62, 1.0))

    model = ArticulatedObject(name="windshield_wiper_assembly", assets=ASSETS)
    model.materials.extend([painted_black, textured_black, rubber_black, steel])

    base_mesh = mesh_from_geometry(_build_base_mesh(), MESH_DIR / "wiper_base_mount.obj")
    arm_mesh = mesh_from_geometry(_build_arm_mesh(), MESH_DIR / "wiper_arm.obj")
    blade_frame_mesh = mesh_from_geometry(
        _build_blade_frame_mesh(), MESH_DIR / "wiper_blade_frame.obj"
    )

    base_mount = model.part("base_mount")
    base_mount.visual(base_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=textured_black)
    base_mount.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
    )
    base_mount.visual(
        Cylinder(radius=0.015, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=steel,
    )
    base_mount.inertial = Inertial.from_geometry(
        Box((0.052, 0.032, 0.024)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    wiper_arm = model.part("wiper_arm")
    wiper_arm.visual(arm_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=painted_black)
    wiper_arm.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=steel,
    )
    wiper_arm.inertial = Inertial.from_geometry(
        Box((0.320, 0.026, 0.020)),
        mass=0.20,
        origin=Origin(xyz=(0.160, 0.002, 0.004)),
    )

    blade_support = model.part("blade_support")
    blade_support.visual(
        blade_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_black,
    )
    blade_support.visual(
        Box((0.0045, 0.340, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, -0.0100)),
        material=rubber_black,
    )
    blade_support.visual(
        Cylinder(radius=0.003, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    blade_support.inertial = Inertial.from_geometry(
        Box((0.020, 0.340, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    model.articulation(
        "wiper_sweep",
        ArticulationType.REVOLUTE,
        parent="base_mount",
        child="wiper_arm",
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.4,
            lower=-0.55,
            upper=1.25,
        ),
    )
    model.articulation(
        "blade_pitch",
        ArticulationType.REVOLUTE,
        parent="wiper_arm",
        child="blade_support",
        origin=Origin(xyz=(0.300, 0.0045, 0.0000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=-0.30,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("wiper_arm", "base_mount", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_overlap("blade_support", "wiper_arm", axes="xy", min_overlap=0.006)
    ctx.expect_joint_motion_axis(
        "wiper_sweep",
        "wiper_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "wiper_sweep",
        "blade_support",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )

    with ctx.pose(wiper_sweep=-0.45):
        ctx.expect_aabb_overlap("wiper_arm", "base_mount", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_overlap("blade_support", "wiper_arm", axes="xy", min_overlap=0.006)

    with ctx.pose(wiper_sweep=1.15):
        ctx.expect_aabb_overlap("wiper_arm", "base_mount", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_overlap("blade_support", "wiper_arm", axes="xy", min_overlap=0.006)

    with ctx.pose(blade_pitch=-0.26):
        ctx.expect_aabb_overlap("blade_support", "wiper_arm", axes="xy", min_overlap=0.006)

    with ctx.pose(blade_pitch=0.26):
        ctx.expect_aabb_overlap("blade_support", "wiper_arm", axes="xy", min_overlap=0.006)

    with ctx.pose({"wiper_sweep": 1.05, "blade_pitch": 0.20}):
        ctx.expect_aabb_overlap("blade_support", "wiper_arm", axes="xy", min_overlap=0.006)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

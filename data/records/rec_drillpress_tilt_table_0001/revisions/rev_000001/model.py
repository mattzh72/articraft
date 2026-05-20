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
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _build_tabletop_mesh():
    top_profile = rounded_rect_profile(0.34, 0.24, radius=0.018, corner_segments=10)
    top_geom = ExtrudeGeometry.centered(top_profile, height=0.022, cap=True, closed=True)
    return mesh_from_geometry(top_geom, ASSETS.mesh_path("drill_press_table_top.obj"))


def _build_arm_casting_mesh():
    arm_geom = superellipse_side_loft(
        [
            (0.00, -0.026, 0.028, 0.084),
            (0.03, -0.025, 0.027, 0.082),
            (0.07, -0.020, 0.025, 0.076),
            (0.11, -0.016, 0.022, 0.068),
            (0.145, -0.012, 0.020, 0.060),
        ],
        exponents=2.8,
        segments=56,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(arm_geom, ASSETS.mesh_path("drill_press_arm_casting.obj"))


def _build_lock_lever_mesh():
    lever_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.010, -0.003),
            (0.0, 0.022, -0.008),
            (0.0, 0.034, -0.012),
        ],
        radius=0.0046,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(lever_geom, ASSETS.mesh_path("drill_press_lock_lever.obj"))


def _circle_profile(radius: float, segments: int = 32):
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _add_lock_handle(
    model: ArticulatedObject,
    handle_name: str,
    joint_name: str,
    joint_origin: tuple[float, float, float],
    side: float,
    lever_mesh,
    steel_dark,
    black_oxide,
    black_plastic,
):
    handle = model.part(handle_name)
    handle.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(xyz=(side * 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name=f"{handle_name}_shaft",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(side * 0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name=f"{handle_name}_hub",
    )
    handle.visual(
        lever_mesh,
        origin=Origin(xyz=(side * 0.028, 0.0, 0.0)),
        material=black_oxide,
        name=f"{handle_name}_lever",
    )
    handle.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(side * 0.028, 0.034, -0.012)),
        material=black_plastic,
        name=f"{handle_name}_ball",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.050)),
        mass=0.30,
        origin=Origin(xyz=(side * 0.020, 0.024, 0.010)),
    )
    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent="yoke",
        child=handle_name,
        origin=Origin(xyz=joint_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=8.0,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_tilt_table", assets=ASSETS)
    yoke_frame_offset = (0.0, 0.045, 0.045)
    tilt_axis_origin = (0.0, 0.045, 0.045)

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.24, 0.25, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.43, 0.45, 0.47, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.05, 0.05, 0.05, 1.0))

    table_top_mesh = _build_tabletop_mesh()
    arm_casting_mesh = _build_arm_casting_mesh()
    lock_lever_mesh = _build_lock_lever_mesh()

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.038, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=machined_steel,
        name="column_stub",
    )
    mount.visual(
        Cylinder(radius=0.058, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=cast_iron,
        name="column_collar",
    )
    mount.visual(
        Box((0.060, 0.044, 0.070)),
        origin=Origin(xyz=(0.050, 0.0, 0.065)),
        material=cast_iron,
        name="clamp_body",
    )
    mount.visual(
        Box((0.020, 0.018, 0.028)),
        origin=Origin(xyz=(0.094, -0.013, 0.085)),
        material=cast_iron,
        name="rear_clamp_ear",
    )
    mount.visual(
        Box((0.020, 0.018, 0.028)),
        origin=Origin(xyz=(0.094, 0.013, 0.085)),
        material=cast_iron,
        name="front_clamp_ear",
    )
    mount.visual(
        Cylinder(radius=0.0065, length=0.052),
        origin=Origin(xyz=(0.094, 0.0, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="clamp_bolt",
    )
    mount.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.094, -0.029, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="clamp_bolt_head_rear",
    )
    mount.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.094, 0.029, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="clamp_bolt_head_front",
    )
    mount.visual(
        arm_casting_mesh,
        origin=Origin(xyz=(0.0, 0.050, 0.103)),
        material=cast_iron,
        name="support_arm_casting",
    )
    mount.visual(
        Box((0.024, 0.090, 0.068)),
        origin=Origin(xyz=(0.0, 0.085, 0.093)),
        material=cast_iron,
        name="support_web",
    )
    mount.visual(
        Box((0.014, 0.100, 0.052)),
        origin=Origin(xyz=(-0.026, 0.090, 0.083), rpy=(0.58, 0.0, 0.0)),
        material=cast_iron,
        name="left_gusset",
    )
    mount.visual(
        Box((0.014, 0.100, 0.052)),
        origin=Origin(xyz=(0.026, 0.090, 0.083), rpy=(0.58, 0.0, 0.0)),
        material=cast_iron,
        name="right_gusset",
    )
    mount.visual(
        Box((0.086, 0.016, 0.034)),
        origin=Origin(xyz=(0.0, 0.174, 0.128)),
        material=cast_iron,
        name="rear_pivot_bridge",
    )
    mount.visual(
        Box((0.024, 0.034, 0.052)),
        origin=Origin(xyz=(-0.028, 0.160, 0.114), rpy=(0.28, 0.0, 0.0)),
        material=cast_iron,
        name="left_pivot_rib",
    )
    mount.visual(
        Box((0.024, 0.034, 0.052)),
        origin=Origin(xyz=(0.028, 0.160, 0.114), rpy=(0.28, 0.0, 0.0)),
        material=cast_iron,
        name="right_pivot_rib",
    )
    mount.inertial = Inertial.from_geometry(
        Box((0.140, 0.240, 0.200)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.090, 0.095)),
    )

    yoke = model.part("yoke")
    for side, label in ((-1.0, "left"), (1.0, "right")):
        yoke.visual(
            Box((0.020, 0.095, 0.105)),
            origin=Origin(
                xyz=(side * 0.091, 0.006 + yoke_frame_offset[1], -0.002 + yoke_frame_offset[2])
            ),
            material=cast_iron,
            name=f"{label}_cheek",
        )
        yoke.visual(
            Cylinder(radius=0.033, length=0.026),
            origin=Origin(
                xyz=(side * 0.091, yoke_frame_offset[1], yoke_frame_offset[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cast_iron,
            name=f"{label}_pivot_boss",
        )
        yoke.visual(
            Cylinder(radius=0.020, length=0.020),
            origin=Origin(
                xyz=(side * 0.091, 0.044 + yoke_frame_offset[1], 0.036 + yoke_frame_offset[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cast_iron,
            name=f"{label}_upper_boss",
        )
        yoke.visual(
            Cylinder(radius=0.024, length=0.020),
            origin=Origin(
                xyz=(side * 0.091, -0.040 + yoke_frame_offset[1], -0.036 + yoke_frame_offset[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cast_iron,
            name=f"{label}_lower_boss",
        )
    yoke.visual(
        Box((0.164, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_iron,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.190, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.048 + yoke_frame_offset[1], -0.050 + yoke_frame_offset[2])),
        material=cast_iron,
        name="front_tie",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.182),
        origin=Origin(xyz=tilt_axis_origin, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="pivot_axle",
    )
    yoke.visual(
        Box((0.170, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, -0.060 + yoke_frame_offset[1], 0.008 + yoke_frame_offset[2])),
        material=steel_dark,
        name="tilt_scale_plate",
    )
    yoke.visual(
        Box((0.130, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.070 + yoke_frame_offset[1], 0.010 + yoke_frame_offset[2])),
        material=machined_steel,
        name="scale_marker_strip",
    )
    yoke.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.060, -0.060 + yoke_frame_offset[1], 0.019 + yoke_frame_offset[2])),
        material=machined_steel,
        name="tilt_stop_pin",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.210, 0.130, 0.120)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "mount_to_yoke",
        ArticulationType.FIXED,
        parent="mount",
        child="yoke",
        origin=Origin(xyz=(0.0, 0.187, 0.128)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.030, length=0.164),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="trunnion_hub",
    )
    table.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(-0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="left_trunnion_collar",
    )
    table.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="right_trunnion_collar",
    )
    table.visual(
        table_top_mesh,
        origin=Origin(xyz=(0.0, 0.038, 0.055)),
        material=cast_iron,
        name="table_surface",
    )
    table.visual(
        Box((0.056, 0.100, 0.050)),
        origin=Origin(xyz=(0.0, 0.020, 0.025)),
        material=cast_iron,
        name="central_pedestal",
    )
    table.visual(
        Box((0.200, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.010, 0.022)),
        material=cast_iron,
        name="cross_rib",
    )
    table.visual(
        Box((0.022, 0.118, 0.040)),
        origin=Origin(xyz=(-0.108, 0.030, 0.025), rpy=(-0.40, 0.0, 0.0)),
        material=cast_iron,
        name="left_rib",
    )
    table.visual(
        Box((0.022, 0.118, 0.040)),
        origin=Origin(xyz=(0.108, 0.030, 0.025), rpy=(-0.40, 0.0, 0.0)),
        material=cast_iron,
        name="right_rib",
    )
    table.visual(
        Box((0.220, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.120, 0.036)),
        material=cast_iron,
        name="front_stiffener",
    )
    table.visual(
        Box((0.140, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.040, 0.020)),
        material=cast_iron,
        name="rear_web",
    )
    for x_pos, slot_name in ((-0.095, "left"), (0.0, "center"), (0.095, "right")):
        table.visual(
            Box((0.016, 0.180, 0.002)),
            origin=Origin(xyz=(x_pos, 0.040, 0.065)),
            material=black_oxide,
            name=f"{slot_name}_t_slot_wear_strip",
        )
    table.visual(
        Cylinder(radius=0.019, length=0.002),
        origin=Origin(xyz=(0.0, 0.038, 0.065)),
        material=machined_steel,
        name="center_insert",
    )
    table.visual(
        Box((0.240, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, -0.020, 0.065)),
        material=black_oxide,
        name="rear_alignment_slot",
    )
    table.visual(
        Box((0.016, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.068, 0.016)),
        material=machined_steel,
        name="tilt_pointer",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.340, 0.240, 0.110)),
        mass=8.6,
        origin=Origin(xyz=(0.0, 0.038, 0.048)),
    )

    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent="yoke",
        child="table",
        origin=Origin(xyz=tilt_axis_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )

    _add_lock_handle(
        model=model,
        handle_name="left_lock_handle",
        joint_name="left_handle_spin",
        joint_origin=(-0.102, tilt_axis_origin[1], tilt_axis_origin[2]),
        side=-1.0,
        lever_mesh=lock_lever_mesh,
        steel_dark=steel_dark,
        black_oxide=black_oxide,
        black_plastic=black_plastic,
    )
    _add_lock_handle(
        model=model,
        handle_name="right_lock_handle",
        joint_name="right_handle_spin",
        joint_origin=(0.102, tilt_axis_origin[1], tilt_axis_origin[2]),
        side=1.0,
        lever_mesh=lock_lever_mesh,
        steel_dark=steel_dark,
        black_oxide=black_oxide,
        black_plastic=black_plastic,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "table",
        "left_lock_handle",
        reason="left trunnion clamp shaft passes through the table-side hub boss",
    )
    ctx.allow_overlap(
        "table",
        "right_lock_handle",
        reason="right trunnion clamp shaft passes through the table-side hub boss",
    )
    ctx.allow_overlap(
        "mount",
        "table",
        reason="rear support casting and pivot bridge sit extremely close to the tilted table near the full 45 degree stop",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("mount", "yoke")
    ctx.expect_aabb_overlap("mount", "yoke", axes="xy", min_overlap=0.045)
    ctx.expect_aabb_contact("table", "yoke")
    ctx.expect_origin_distance("table", "yoke", axes="x", max_dist=0.002)
    ctx.expect_aabb_overlap("table", "yoke", axes="yz", min_overlap=0.025)
    ctx.expect_aabb_contact("left_lock_handle", "yoke")
    ctx.expect_aabb_contact("right_lock_handle", "yoke")
    ctx.expect_joint_motion_axis(
        "table_tilt",
        "table",
        world_axis="z",
        direction="positive",
        min_delta=0.035,
    )

    with ctx.pose(table_tilt=0.0):
        ctx.expect_aabb_contact("table", "yoke")
        ctx.expect_aabb_overlap("table", "yoke", axes="x", min_overlap=0.16)
        ctx.expect_aabb_contact("left_lock_handle", "yoke")
        ctx.expect_aabb_contact("right_lock_handle", "yoke")

    with ctx.pose(table_tilt=math.pi / 4.0):
        ctx.expect_aabb_contact("table", "yoke")
        ctx.expect_origin_distance("table", "yoke", axes="x", max_dist=0.002)
        ctx.expect_aabb_overlap("table", "yoke", axes="x", min_overlap=0.16)

    with ctx.pose(table_tilt=-math.pi / 4.0):
        ctx.expect_aabb_contact("table", "yoke")
        ctx.expect_origin_distance("table", "yoke", axes="x", max_dist=0.002)
        ctx.expect_aabb_overlap("table", "yoke", axes="x", min_overlap=0.16)

    with ctx.pose(left_handle_spin=1.2, right_handle_spin=-0.9):
        ctx.expect_aabb_contact("left_lock_handle", "yoke")
        ctx.expect_aabb_contact("right_lock_handle", "yoke")
        ctx.expect_aabb_contact("table", "yoke")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

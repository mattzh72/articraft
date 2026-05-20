from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow", assets=ASSETS)

    tray_green = model.material("tray_green", rgba=(0.22, 0.34, 0.20, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def mirror_points_x(
        points: list[tuple[float, float, float]],
    ) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def save_mesh(geometry, name: str):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    frame = model.part("frame")
    upper_left_points = [
        (-0.30, 0.90, 0.43),
        (-0.27, 0.72, 0.41),
        (-0.25, 0.52, 0.36),
        (-0.245, 0.34, 0.31),
        (-0.22, 0.16, 0.25),
    ]
    fork_left_points = [
        (-0.22, 0.16, 0.25),
        (-0.17, 0.11, 0.18),
        (-0.12, 0.07, 0.11),
        (-0.085, 0.04, 0.07),
        (-0.06, 0.02, 0.04),
    ]
    leg_left_points = [
        (-0.25, 0.52, 0.36),
        (-0.23, 0.58, 0.20),
        (-0.20, 0.64, 0.02),
        (-0.16, 0.70, -0.18),
    ]
    frame.visual(
        save_mesh(
            tube_from_spline_points(
                upper_left_points,
                radius=0.014,
                samples_per_segment=20,
                radial_segments=20,
            ),
            "frame_upper_left.obj",
        ),
        material=frame_gray,
    )
    frame.visual(
        save_mesh(
            tube_from_spline_points(
                mirror_points_x(upper_left_points),
                radius=0.014,
                samples_per_segment=20,
                radial_segments=20,
            ),
            "frame_upper_right.obj",
        ),
        material=frame_gray,
    )
    frame.visual(
        save_mesh(
            tube_from_spline_points(
                fork_left_points,
                radius=0.014,
                samples_per_segment=18,
                radial_segments=20,
            ),
            "frame_fork_left.obj",
        ),
        material=frame_gray,
    )
    frame.visual(
        save_mesh(
            tube_from_spline_points(
                mirror_points_x(fork_left_points),
                radius=0.014,
                samples_per_segment=18,
                radial_segments=20,
            ),
            "frame_fork_right.obj",
        ),
        material=frame_gray,
    )
    frame.visual(
        save_mesh(
            tube_from_spline_points(
                leg_left_points,
                radius=0.013,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "frame_leg_left.obj",
        ),
        material=frame_gray,
    )
    frame.visual(
        save_mesh(
            tube_from_spline_points(
                mirror_points_x(leg_left_points),
                radius=0.013,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "frame_leg_right.obj",
        ),
        material=frame_gray,
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.42),
        origin=Origin(xyz=(0.0, 0.16, 0.25), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.13),
        origin=Origin(xyz=(0.0, 0.02, 0.04), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.0, 0.09, 0.13), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.49),
        origin=Origin(xyz=(0.0, 0.34, 0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.54),
        origin=Origin(xyz=(0.0, 0.72, 0.41), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.32),
        origin=Origin(xyz=(0.0, 0.70, -0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.14),
        origin=Origin(xyz=(-0.30, 0.90, 0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.14),
        origin=Origin(xyz=(0.30, 0.90, 0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 1.12, 0.64)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.43, 0.14)),
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.016, length=0.42),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    tray.visual(
        Box((0.18, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.05, 0.03)),
        material=zinc_steel,
    )
    tray.visual(
        Box((0.44, 0.72, 0.020)),
        origin=Origin(xyz=(0.0, 0.36, 0.075), rpy=(0.10, 0.0, 0.0)),
        material=tray_green,
    )
    tray.visual(
        Box((0.018, 0.74, 0.26)),
        origin=Origin(xyz=(-0.225, 0.37, 0.18)),
        material=tray_green,
    )
    tray.visual(
        Box((0.018, 0.74, 0.26)),
        origin=Origin(xyz=(0.225, 0.37, 0.18)),
        material=tray_green,
    )
    tray.visual(
        Box((0.44, 0.020, 0.26)),
        origin=Origin(xyz=(0.0, 0.03, 0.18)),
        material=tray_green,
    )
    tray.visual(
        Box((0.56, 0.020, 0.19)),
        origin=Origin(xyz=(0.0, 0.71, 0.16)),
        material=tray_green,
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.74),
        origin=Origin(xyz=(-0.226, 0.37, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.74),
        origin=Origin(xyz=(0.226, 0.37, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.46),
        origin=Origin(xyz=(0.0, 0.03, 0.31), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.58),
        origin=Origin(xyz=(0.0, 0.71, 0.28), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    tray.visual(
        Box((0.040, 0.56, 0.050)),
        origin=Origin(xyz=(-0.12, 0.34, 0.045), rpy=(0.10, 0.0, 0.0)),
        material=zinc_steel,
    )
    tray.visual(
        Box((0.040, 0.56, 0.050)),
        origin=Origin(xyz=(0.12, 0.34, 0.045), rpy=(0.10, 0.0, 0.0)),
        material=zinc_steel,
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.60, 0.78, 0.34)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.36, 0.16)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        save_mesh(
            TorusGeometry(radius=0.155, tube=0.035, radial_segments=20, tubular_segments=42),
            "wheel_tire.obj",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
    )
    wheel.visual(
        Cylinder(radius=0.128, length=0.045),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    wheel.visual(
        Cylinder(radius=0.140, length=0.010),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    wheel.visual(
        Cylinder(radius=0.140, length=0.010),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    wheel.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
    )
    for spoke_index in range(8):
        angle = spoke_index * (math.pi / 4.0)
        spoke_radius = 0.082
        wheel.visual(
            Cylinder(radius=0.0055, length=0.11),
            origin=Origin(
                xyz=(0.0, spoke_radius * math.sin(angle), spoke_radius * math.cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=zinc_steel,
        )
    wheel.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(-0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.065),
        mass=2.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="wheel",
        origin=Origin(xyz=(0.0, 0.02, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=20.0,
        ),
    )
    model.articulation(
        "tray_dump",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="tray",
        origin=Origin(xyz=(0.0, 0.16, 0.25)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=0.0,
            upper=1.05,
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
        max_pose_samples=144,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("wheel", "frame")
    ctx.expect_aabb_overlap("wheel", "frame", axes="xz", min_overlap=0.06)
    ctx.expect_aabb_contact("tray", "frame")
    ctx.expect_aabb_overlap("tray", "frame", axes="x", min_overlap=0.35)
    ctx.expect_aabb_gap("tray", "wheel", axis="z", max_gap=0.08, max_penetration=0.0)
    ctx.expect_aabb_overlap("tray", "wheel", axes="xy", min_overlap=0.06)
    ctx.expect_joint_motion_axis(
        "tray_dump", "tray", world_axis="z", direction="positive", min_delta=0.15
    )
    ctx.expect_joint_motion_axis(
        "tray_dump", "tray", world_axis="y", direction="negative", min_delta=0.10
    )

    with ctx.pose(wheel_spin=math.pi / 2.0):
        ctx.expect_aabb_contact("wheel", "frame")
        ctx.expect_aabb_gap("tray", "wheel", axis="z", max_gap=0.08, max_penetration=0.0)

    with ctx.pose(tray_dump=1.0):
        ctx.expect_aabb_contact("tray", "frame")
        ctx.expect_aabb_overlap("tray", "frame", axes="x", min_overlap=0.35)
        ctx.expect_aabb_gap("tray", "wheel", axis="z", max_gap=0.08, max_penetration=0.003)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

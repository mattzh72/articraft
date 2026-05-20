from __future__ import annotations

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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_rolling_walker", assets=ASSETS)

    powder_coat = model.material("powder_coat", rgba=(0.31, 0.35, 0.30, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    zinc = model.material("zinc", rgba=(0.70, 0.72, 0.74, 1.0))
    molded_gray = model.material("molded_gray", rgba=(0.38, 0.40, 0.43, 1.0))

    frame_half_width = 0.245
    front_y = 0.225
    rear_y = -0.225
    top_z = 0.88
    caster_mount_z = 0.165
    tube_radius = 0.014

    def mesh_file(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_dir / name)

    def mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    frame = model.part("main_frame")

    left_side_loop_pts = [
        (-frame_half_width, rear_y, 0.035),
        (-frame_half_width, -0.215, 0.205),
        (-frame_half_width, -0.195, 0.46),
        (-frame_half_width, -0.145, 0.76),
        (-frame_half_width, -0.03, top_z),
        (-frame_half_width, 0.10, top_z),
        (-0.243, 0.185, 0.80),
        (-0.236, front_y, 0.50),
        (-0.235, front_y, 0.215),
    ]
    right_side_loop_pts = mirror_x(left_side_loop_pts)

    frame.visual(
        mesh_file(
            "walker_left_side_loop.obj",
            tube_from_spline_points(
                left_side_loop_pts,
                radius=tube_radius,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=powder_coat,
        name="left_side_loop",
    )
    frame.visual(
        mesh_file(
            "walker_right_side_loop.obj",
            tube_from_spline_points(
                right_side_loop_pts,
                radius=tube_radius,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=powder_coat,
        name="right_side_loop",
    )

    frame.visual(
        mesh_file(
            "walker_left_lower_rail.obj",
            wire_from_points(
                [(-0.235, -0.215, 0.205), (-0.235, front_y, 0.215)],
                radius=tube_radius * 0.92,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            ),
        ),
        material=powder_coat,
        name="left_lower_rail",
    )
    frame.visual(
        mesh_file(
            "walker_right_lower_rail.obj",
            wire_from_points(
                [(0.235, -0.215, 0.205), (0.235, front_y, 0.215)],
                radius=tube_radius * 0.92,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            ),
        ),
        material=powder_coat,
        name="right_lower_rail",
    )

    frame.visual(
        mesh_file(
            "walker_left_diagonal_brace.obj",
            wire_from_points(
                [(-0.235, 0.01, 0.212), (-0.239, 0.155, 0.585)],
                radius=0.0105,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            ),
        ),
        material=powder_coat,
        name="left_diagonal_brace",
    )
    frame.visual(
        mesh_file(
            "walker_right_diagonal_brace.obj",
            wire_from_points(
                [(0.235, 0.01, 0.212), (0.239, 0.155, 0.585)],
                radius=0.0105,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            ),
        ),
        material=powder_coat,
        name="right_diagonal_brace",
    )

    frame.visual(
        Cylinder(radius=0.012, length=0.47),
        origin=Origin(xyz=(0.0, 0.165, 0.795), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="front_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.45),
        origin=Origin(xyz=(0.0, -0.05, 0.865), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="rear_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.49),
        origin=Origin(xyz=(0.0, 0.02, 0.212), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="utility_lower_crossbar",
    )

    for side, sx in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Cylinder(radius=0.022, length=0.17),
            origin=Origin(xyz=(sx * frame_half_width, 0.03, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"{side}_grip",
        )
        for grip_end_y in (-0.055, 0.115):
            frame.visual(
                Sphere(radius=0.021),
                origin=Origin(xyz=(sx * frame_half_width, grip_end_y, top_z)),
                material=dark_trim,
                name=f"{side}_grip_end_{'rear' if grip_end_y < 0.0 else 'front'}",
            )

        frame.visual(
            Box((0.032, 0.020, 0.032)),
            origin=Origin(xyz=(sx * frame_half_width, 0.038, 0.855)),
            material=powder_coat,
            name=f"{side}_brake_bracket",
        )
        frame.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(
                xyz=(sx * 0.265, 0.038, 0.855),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"{side}_brake_bolt",
        )

        frame.visual(
            Box((0.052, 0.034, 0.05)),
            origin=Origin(xyz=(sx * 0.235, front_y, 0.19)),
            material=powder_coat,
            name=f"{side}_socket_plate",
        )
        frame.visual(
            Box((0.012, 0.056, 0.09)),
            origin=Origin(xyz=(sx * 0.221, front_y, 0.248)),
            material=powder_coat,
            name=f"{side}_socket_gusset",
        )
        frame.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(
                xyz=(sx * 0.265, front_y, 0.19),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"{side}_caster_bolt",
        )

        frame.visual(
            Cylinder(radius=0.018, length=0.036),
            origin=Origin(xyz=(sx * frame_half_width, rear_y, 0.018)),
            material=rubber,
            name=f"{side}_rear_tip",
        )

    left_brake = model.part("left_brake_lever")
    left_brake.visual(
        mesh_file(
            "walker_left_brake_lever.obj",
            tube_from_spline_points(
                [
                    (0.0, -0.002, -0.003),
                    (0.0, 0.018, -0.016),
                    (0.0, 0.036, -0.038),
                    (0.0, 0.026, -0.076),
                ],
                radius=0.0065,
                samples_per_segment=16,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=dark_trim,
        name="left_lever_body",
    )
    left_brake.visual(
        Cylinder(radius=0.009, length=0.04),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="left_pivot_barrel",
    )
    left_brake.visual(
        Box((0.020, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, 0.024, -0.070)),
        material=dark_trim,
        name="left_lever_pad",
    )

    right_brake = model.part("right_brake_lever")
    right_brake.visual(
        mesh_file(
            "walker_right_brake_lever.obj",
            tube_from_spline_points(
                [
                    (0.0, -0.002, -0.003),
                    (0.0, 0.018, -0.016),
                    (0.0, 0.036, -0.038),
                    (0.0, 0.026, -0.076),
                ],
                radius=0.0065,
                samples_per_segment=16,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=dark_trim,
        name="right_lever_body",
    )
    right_brake.visual(
        Cylinder(radius=0.009, length=0.04),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="right_pivot_barrel",
    )
    right_brake.visual(
        Box((0.020, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, 0.024, -0.070)),
        material=dark_trim,
        name="right_lever_pad",
    )

    for side, sx in (("left", -1.0), ("right", 1.0)):
        caster = model.part(f"{side}_caster")
        caster.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=molded_gray,
            name=f"{side}_swivel_cap",
        )
        caster.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=zinc,
            name=f"{side}_kingpin",
        )
        caster.visual(
            Box((0.046, 0.024, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.043)),
            material=molded_gray,
            name=f"{side}_yoke_bridge",
        )
        caster.visual(
            Box((0.010, 0.018, 0.086)),
            origin=Origin(xyz=(-0.019, 0.0, -0.086)),
            material=molded_gray,
            name=f"{side}_fork_inboard",
        )
        caster.visual(
            Box((0.010, 0.018, 0.086)),
            origin=Origin(xyz=(0.019, 0.0, -0.086)),
            material=molded_gray,
            name=f"{side}_fork_outboard",
        )
        caster.visual(
            Box((0.010, 0.018, 0.020)),
            origin=Origin(xyz=(-0.019, 0.0, -0.115)),
            material=molded_gray,
            name=f"{side}_axle_block_inboard",
        )
        caster.visual(
            Box((0.010, 0.018, 0.020)),
            origin=Origin(xyz=(0.019, 0.0, -0.115)),
            material=molded_gray,
            name=f"{side}_axle_block_outboard",
        )

        wheel = model.part(f"{side}_wheel")
        wheel.visual(
            Cylinder(radius=0.05, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"{side}_tire",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=molded_gray,
            name=f"{side}_hub",
        )
        wheel.visual(
            Cylinder(radius=0.005, length=0.046),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"{side}_axle_spindle",
        )

        model.articulation(
            f"{side}_caster_yaw",
            ArticulationType.CONTINUOUS,
            parent="main_frame",
            child=f"{side}_caster",
            origin=Origin(xyz=(sx * 0.235, front_y, caster_mount_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=14.0, velocity=6.0),
        )
        model.articulation(
            f"{side}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=f"{side}_caster",
            child=f"{side}_wheel",
            origin=Origin(xyz=(0.0, 0.0, -0.115)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=9.0, velocity=18.0),
        )

    model.articulation(
        "left_brake_hinge",
        ArticulationType.REVOLUTE,
        parent="main_frame",
        child="left_brake_lever",
        origin=Origin(xyz=(-frame_half_width, 0.040, 0.865)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=0.0, upper=0.60),
    )
    model.articulation(
        "right_brake_hinge",
        ArticulationType.REVOLUTE,
        parent="main_frame",
        child="right_brake_lever",
        origin=Origin(xyz=(frame_half_width, 0.040, 0.865)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=0.0, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap("main_frame", "left_brake_lever", reason="lever pivot barrel seats inside the welded brake bracket")
    ctx.allow_overlap("main_frame", "right_brake_lever", reason="lever pivot barrel seats inside the welded brake bracket")
    ctx.allow_overlap("main_frame", "left_caster", reason="caster thrust cap seats tightly against the socket plate")
    ctx.allow_overlap("main_frame", "right_caster", reason="caster thrust cap seats tightly against the socket plate")
    ctx.allow_overlap("left_caster", "left_wheel", reason="service axle spindle passes through the wheel hub bushings")
    ctx.allow_overlap("right_caster", "right_wheel", reason="service axle spindle passes through the wheel hub bushings")

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_gap(
        "main_frame",
        "left_caster",
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem="left_socket_plate",
        negative_elem="left_swivel_cap",
    )
    ctx.expect_aabb_gap(
        "main_frame",
        "right_caster",
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem="right_socket_plate",
        negative_elem="right_swivel_cap",
    )
    ctx.expect_aabb_overlap("left_wheel", "left_caster", axes="yz", min_overlap=0.018)
    ctx.expect_aabb_overlap("right_wheel", "right_caster", axes="yz", min_overlap=0.018)
    ctx.expect_aabb_gap(
        "left_caster",
        "left_wheel",
        axis="z",
        min_gap=0.006,
        max_gap=0.030,
        positive_elem="left_yoke_bridge",
        negative_elem="left_tire",
    )
    ctx.expect_aabb_gap(
        "right_caster",
        "right_wheel",
        axis="z",
        min_gap=0.006,
        max_gap=0.030,
        positive_elem="right_yoke_bridge",
        negative_elem="right_tire",
    )
    ctx.expect_aabb_gap(
        "main_frame",
        "left_brake_lever",
        axis="z",
        min_gap=0.005,
        max_gap=0.055,
        positive_elem="left_grip",
        negative_elem="left_lever_pad",
    )
    ctx.expect_aabb_gap(
        "main_frame",
        "right_brake_lever",
        axis="z",
        min_gap=0.005,
        max_gap=0.055,
        positive_elem="right_grip",
        negative_elem="right_lever_pad",
    )
    ctx.expect_joint_motion_axis("left_brake_hinge", "left_brake_lever", world_axis="z", direction="positive", min_delta=0.01)
    ctx.expect_joint_motion_axis("right_brake_hinge", "right_brake_lever", world_axis="z", direction="positive", min_delta=0.01)

    with ctx.pose(left_caster_yaw=1.15, right_caster_yaw=-1.05):
        ctx.expect_aabb_gap(
            "main_frame",
            "left_caster",
            axis="z",
            max_gap=0.001,
            max_penetration=0.002,
            positive_elem="left_socket_plate",
            negative_elem="left_swivel_cap",
        )
        ctx.expect_aabb_gap(
            "main_frame",
            "right_caster",
            axis="z",
            max_gap=0.001,
            max_penetration=0.002,
            positive_elem="right_socket_plate",
            negative_elem="right_swivel_cap",
        )

    with ctx.pose(left_brake_hinge=0.55, right_brake_hinge=0.55):
        ctx.expect_aabb_gap(
            "main_frame",
            "left_brake_lever",
            axis="z",
            min_gap=0.0,
            max_gap=0.040,
            positive_elem="left_grip",
            negative_elem="left_lever_pad",
        )
        ctx.expect_aabb_gap(
            "main_frame",
            "right_brake_lever",
            axis="z",
            min_gap=0.0,
            max_gap=0.040,
            positive_elem="right_grip",
            negative_elem="right_lever_pad",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

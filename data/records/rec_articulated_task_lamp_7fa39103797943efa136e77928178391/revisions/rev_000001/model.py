from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_shade_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.023, 0.014),
            (0.046, 0.085),
            (0.073, 0.152),
            (0.090, 0.184),
        ],
        [
            (0.010, 0.010),
            (0.014, 0.022),
            (0.038, 0.088),
            (0.065, 0.155),
            (0.082, 0.180),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return shell.rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_desk_lamp")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.19, 0.21, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.62, 0.64, 0.66, 1.0))

    clamp_frame = model.part("clamp_frame")
    clamp_frame.visual(
        Box((0.036, 0.060, 0.320)),
        origin=Origin(xyz=(0.000, 0.000, 0.160)),
        material=painted_steel,
        name="spine",
    )
    clamp_frame.visual(
        Box((0.090, 0.060, 0.024)),
        origin=Origin(xyz=(0.045, 0.000, 0.286)),
        material=painted_steel,
        name="upper_jaw",
    )
    clamp_frame.visual(
        Box((0.078, 0.060, 0.028)),
        origin=Origin(xyz=(0.039, 0.000, 0.070)),
        material=painted_steel,
        name="lower_jaw",
    )
    clamp_frame.visual(
        Cylinder(radius=0.025, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.322)),
        material=warm_grey,
        name="head_bearing",
    )
    clamp_frame.visual(
        Box((0.026, 0.064, 0.032)),
        origin=Origin(xyz=(-0.005, 0.000, 0.304)),
        material=painted_steel,
        name="head_brace",
    )
    clamp_frame.inertial = Inertial.from_geometry(
        Box((0.110, 0.060, 0.350)),
        mass=1.2,
        origin=Origin(xyz=(0.030, 0.000, 0.175)),
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.007, length=0.210),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=brushed_aluminum,
        name="threaded_rod",
    )
    clamp_screw.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.094)),
        material=brushed_aluminum,
        name="swivel_washer",
    )
    clamp_screw.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.118)),
        material=satin_black,
        name="pressure_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.004, length=0.080),
        origin=Origin(xyz=(0.000, 0.000, -0.074), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=brushed_aluminum,
        name="handle_bar",
    )
    clamp_screw.inertial = Inertial.from_geometry(
        Box((0.080, 0.030, 0.220)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=warm_grey,
        name="yaw_hub",
    )
    boom.visual(
        Box((0.050, 0.035, 0.050)),
        origin=Origin(xyz=(0.025, 0.000, 0.026)),
        material=painted_steel,
        name="base_block",
    )
    boom.visual(
        _save_mesh(
            "boom_upper_tube",
            tube_from_spline_points(
                [
                    (0.020, 0.000, 0.040),
                    (0.220, 0.000, 0.040),
                    (0.410, 0.000, 0.034),
                ],
                radius=0.007,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=painted_steel,
        name="upper_tube",
    )
    boom.visual(
        _save_mesh(
            "boom_lower_tube",
            tube_from_spline_points(
                [
                    (0.020, 0.000, 0.016),
                    (0.220, 0.000, 0.016),
                    (0.410, 0.000, 0.012),
                ],
                radius=0.007,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=painted_steel,
        name="lower_tube",
    )
    boom.visual(
        Box((0.018, 0.020, 0.030)),
        origin=Origin(xyz=(0.225, 0.000, 0.026)),
        material=painted_steel,
        name="mid_brace",
    )
    boom.visual(
        Box((0.016, 0.040, 0.028)),
        origin=Origin(xyz=(0.407, 0.000, 0.022)),
        material=painted_steel,
        name="fork_root_block",
    )
    boom.visual(
        Box((0.014, 0.008, 0.040)),
        origin=Origin(xyz=(0.421, 0.024, 0.022)),
        material=painted_steel,
        name="fork_cheek_left",
    )
    boom.visual(
        Box((0.014, 0.008, 0.040)),
        origin=Origin(xyz=(0.421, -0.024, 0.022)),
        material=painted_steel,
        name="fork_cheek_right",
    )
    boom.inertial = Inertial.from_geometry(
        Box((0.470, 0.070, 0.060)),
        mass=0.62,
        origin=Origin(xyz=(0.235, 0.000, 0.026)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=warm_grey,
        name="tilt_axle",
    )
    shade.visual(
        Box((0.020, 0.012, 0.012)),
        origin=Origin(xyz=(0.010, 0.000, 0.000)),
        material=warm_grey,
        name="shade_bracket",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(
            xyz=(0.032, 0.000, -0.008),
            rpy=(0.000, math.pi / 2.0 - 0.55, 0.000),
        ),
        material=painted_steel,
        name="rear_collar",
    )
    shade.visual(
        _save_mesh("conical_shade_shell", _build_shade_shell_mesh()),
        origin=Origin(xyz=(0.046, 0.000, -0.020), rpy=(0.000, -0.55, 0.000)),
        material=painted_steel,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.210, 0.190, 0.140)),
        mass=0.44,
        origin=Origin(xyz=(0.090, 0.000, -0.040)),
    )

    model.articulation(
        "frame_to_screw",
        ArticulationType.PRISMATIC,
        parent=clamp_frame,
        child=clamp_screw,
        origin=Origin(xyz=(0.046, 0.000, 0.070)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.050,
            lower=0.000,
            upper=0.055,
        ),
    )
    model.articulation(
        "head_to_boom",
        ArticulationType.REVOLUTE,
        parent=clamp_frame,
        child=boom,
        origin=Origin(xyz=(0.000, 0.000, 0.342)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.500,
            lower=-1.900,
            upper=1.900,
        ),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(0.432, 0.000, 0.022)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.800,
            lower=-0.850,
            upper=0.650,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp_frame = object_model.get_part("clamp_frame")
    clamp_screw = object_model.get_part("clamp_screw")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")

    screw_slide = object_model.get_articulation("frame_to_screw")
    boom_swing = object_model.get_articulation("head_to_boom")
    shade_tilt = object_model.get_articulation("boom_to_shade")

    upper_jaw = clamp_frame.get_visual("upper_jaw")
    head_bearing = clamp_frame.get_visual("head_bearing")
    pressure_pad = clamp_screw.get_visual("pressure_pad")
    yaw_hub = boom.get_visual("yaw_hub")
    fork_cheek_left = boom.get_visual("fork_cheek_left")
    tilt_axle = shade.get_visual("tilt_axle")
    shade_shell = shade.get_visual("shade_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        clamp_frame,
        clamp_screw,
        reason="The threaded clamp screw intentionally passes through the lower jaw's tapped hole.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        boom,
        clamp_frame,
        elem_a=yaw_hub,
        elem_b=head_bearing,
        contact_tol=0.0005,
        name="boom_hub_seats_on_head_bearing",
    )
    ctx.expect_contact(
        shade,
        boom,
        elem_a=tilt_axle,
        elem_b=fork_cheek_left,
        contact_tol=0.0005,
        name="shade_axle_bears_on_fork",
    )
    ctx.expect_gap(
        clamp_frame,
        clamp_screw,
        axis="z",
        positive_elem=upper_jaw,
        negative_elem=pressure_pad,
        min_gap=0.070,
        max_gap=0.100,
        name="rest_clamp_opening",
    )

    screw_rest = ctx.part_world_position(clamp_screw)
    screw_limits = screw_slide.motion_limits
    if screw_rest is not None and screw_limits is not None and screw_limits.upper is not None:
        with ctx.pose({screw_slide: screw_limits.upper}):
            screw_tight = ctx.part_world_position(clamp_screw)
            if screw_tight is not None:
                ctx.check(
                    "screw_moves_upward_when_tightened",
                    screw_tight[2] > screw_rest[2] + 0.040,
                    details=f"rest_z={screw_rest[2]:.4f}, tightened_z={screw_tight[2]:.4f}",
                )
            ctx.expect_gap(
                clamp_frame,
                clamp_screw,
                axis="z",
                positive_elem=upper_jaw,
                negative_elem=pressure_pad,
                min_gap=0.015,
                max_gap=0.040,
                name="tightened_clamp_nears_desk_thickness",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="screw_upper_no_unexpected_overlap")
            ctx.fail_if_isolated_parts(name="screw_upper_no_floating")

    boom_limits = boom_swing.motion_limits
    if boom_limits is not None and boom_limits.lower is not None and boom_limits.upper is not None:
        with ctx.pose({boom_swing: boom_limits.lower}):
            shade_lower_pos = ctx.part_world_position(shade)
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_lower_no_floating")
        with ctx.pose({boom_swing: boom_limits.upper}):
            shade_upper_pos = ctx.part_world_position(shade)
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_upper_no_floating")
        if shade_lower_pos is not None and shade_upper_pos is not None:
            ctx.check(
                "boom_sweeps_across_work_area",
                shade_lower_pos[1] < -0.250 and shade_upper_pos[1] > 0.250,
                details=(
                    f"lower_shade_y={shade_lower_pos[1]:.4f}, "
                    f"upper_shade_y={shade_upper_pos[1]:.4f}"
                ),
            )

    tilt_limits = shade_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({shade_tilt: tilt_limits.lower}):
            lower_aabb = ctx.part_element_world_aabb(shade, elem=shade_shell)
            ctx.expect_contact(
                shade,
                boom,
                elem_a=tilt_axle,
                elem_b=fork_cheek_left,
                contact_tol=0.0005,
                name="shade_lower_pose_stays_mounted",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_lower_no_floating")
        with ctx.pose({shade_tilt: tilt_limits.upper}):
            upper_aabb = ctx.part_element_world_aabb(shade, elem=shade_shell)
            ctx.expect_contact(
                shade,
                boom,
                elem_a=tilt_axle,
                elem_b=fork_cheek_left,
                contact_tol=0.0005,
                name="shade_upper_pose_stays_mounted",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_upper_no_floating")
        if lower_aabb is not None and upper_aabb is not None:
            ctx.check(
                "shade_tilt_changes_beam_height",
                abs(lower_aabb[0][2] - upper_aabb[0][2]) > 0.050,
                details=(
                    f"lower_min_z={lower_aabb[0][2]:.4f}, "
                    f"upper_min_z={upper_aabb[0][2]:.4f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

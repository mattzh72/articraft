from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HERE = Path(__file__).parent


def _x_axis_cylinder_origin(
    center: tuple[float, float, float],
) -> Origin:
    return Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0))


def _y_axis_cylinder_origin(
    center: tuple[float, float, float],
) -> Origin:
    return Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woodworking_vise")

    cast_iron = model.material("cast_iron", rgba=(0.25, 0.26, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.74, 1.0))
    bench_wood = model.material("bench_wood", rgba=(0.56, 0.37, 0.18, 1.0))
    jaw_wood = model.material("jaw_wood", rgba=(0.72, 0.56, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.40, 0.43, 1.0))

    bench = model.part("bench")
    bench.visual(
        Box((0.55, 0.55, 0.08)),
        origin=Origin(xyz=(-0.275, 0.0, 0.96)),
        material=bench_wood,
        name="bench_edge",
    )
    bench.inertial = Inertial.from_geometry(
        Box((0.55, 0.55, 0.08)),
        mass=28.0,
        origin=Origin(xyz=(-0.275, 0.0, 0.96)),
    )

    fixed_jaw = model.part("fixed_jaw")
    fixed_jaw.visual(
        Box((0.018, 0.28, 0.14)),
        origin=Origin(xyz=(0.009, 0.0, 0.070)),
        material=cast_iron,
        name="mount_plate",
    )
    fixed_jaw.visual(
        Box((0.028, 0.22, 0.016)),
        origin=Origin(xyz=(0.014, 0.0, 0.126)),
        material=cast_iron,
        name="top_flange",
    )
    fixed_jaw.visual(
        Box((0.018, 0.050, 0.050)),
        origin=Origin(xyz=(0.012, 0.0, 0.070)),
        material=cast_iron,
        name="screw_nut_housing",
    )
    fixed_jaw.visual(
        Box((0.018, 0.034, 0.040)),
        origin=Origin(xyz=(0.012, -0.055, 0.025)),
        material=cast_iron,
        name="left_rod_support",
    )
    fixed_jaw.visual(
        Box((0.018, 0.034, 0.040)),
        origin=Origin(xyz=(0.012, 0.055, 0.025)),
        material=cast_iron,
        name="right_rod_support",
    )
    fixed_jaw.visual(
        Box((0.010, 0.210, 0.100)),
        origin=Origin(xyz=(0.023, 0.0, 0.086)),
        material=jaw_wood,
        name="fixed_liner",
    )
    fixed_jaw.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=_x_axis_cylinder_origin((0.023, -0.120, 0.070)),
        material=dark_steel,
        name="left_bolt_head",
    )
    fixed_jaw.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=_x_axis_cylinder_origin((0.023, 0.120, 0.070)),
        material=dark_steel,
        name="right_bolt_head",
    )
    fixed_jaw.visual(
        Cylinder(radius=0.009, length=0.350),
        origin=_x_axis_cylinder_origin((0.196, -0.055, 0.025)),
        material=steel,
        name="left_guide_rod",
    )
    fixed_jaw.visual(
        Cylinder(radius=0.009, length=0.350),
        origin=_x_axis_cylinder_origin((0.196, 0.055, 0.025)),
        material=steel,
        name="right_guide_rod",
    )
    fixed_jaw.inertial = Inertial.from_geometry(
        Box((0.36, 0.30, 0.15)),
        mass=10.5,
        origin=Origin(xyz=(0.18, 0.0, 0.075)),
    )

    sliding_jaw = model.part("sliding_jaw")
    sliding_jaw.visual(
        Box((0.010, 0.210, 0.100)),
        origin=Origin(xyz=(0.005, 0.0, 0.086)),
        material=jaw_wood,
        name="moving_liner",
    )
    sliding_jaw.visual(
        Box((0.024, 0.210, 0.020)),
        origin=Origin(xyz=(0.022, 0.0, 0.126)),
        material=cast_iron,
        name="jaw_face_casting",
    )
    sliding_jaw.visual(
        Box((0.024, 0.210, 0.022)),
        origin=Origin(xyz=(0.022, 0.0, 0.047)),
        material=cast_iron,
        name="lower_carriage",
    )
    sliding_jaw.visual(
        Box((0.072, 0.120, 0.020)),
        origin=Origin(xyz=(0.070, 0.0, 0.116)),
        material=cast_iron,
        name="upper_bridge",
    )
    sliding_jaw.visual(
        Box((0.024, 0.056, 0.048)),
        origin=Origin(xyz=(0.128, 0.0, 0.070)),
        material=cast_iron,
        name="handle_boss",
    )
    sliding_jaw.visual(
        Box((0.024, 0.040, 0.112)),
        origin=Origin(xyz=(0.022, -0.085, 0.080)),
        material=cast_iron,
        name="left_face_cheek",
    )
    sliding_jaw.visual(
        Box((0.024, 0.040, 0.112)),
        origin=Origin(xyz=(0.022, 0.085, 0.080)),
        material=cast_iron,
        name="right_face_cheek",
    )
    sliding_jaw.visual(
        Box((0.024, 0.050, 0.050)),
        origin=Origin(xyz=(0.022, 0.0, 0.070)),
        material=cast_iron,
        name="center_boss",
    )
    sliding_jaw.visual(
        Box((0.082, 0.020, 0.024)),
        origin=Origin(xyz=(0.075, -0.080, 0.041)),
        material=cast_iron,
        name="left_side_block",
    )
    sliding_jaw.visual(
        Box((0.082, 0.020, 0.024)),
        origin=Origin(xyz=(0.075, 0.080, 0.041)),
        material=cast_iron,
        name="right_side_block",
    )
    sliding_jaw.visual(
        Box((0.082, 0.040, 0.040)),
        origin=Origin(xyz=(0.075, 0.0, 0.070)),
        material=cast_iron,
        name="center_spine",
    )
    sliding_jaw.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=_x_axis_cylinder_origin((0.099, -0.055, 0.025)),
        material=steel,
        name="left_sleeve",
    )
    sliding_jaw.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=_x_axis_cylinder_origin((0.099, 0.055, 0.025)),
        material=steel,
        name="right_sleeve",
    )
    sliding_jaw.inertial = Inertial.from_geometry(
        Box((0.17, 0.24, 0.13)),
        mass=6.8,
        origin=Origin(xyz=(0.085, 0.0, 0.065)),
    )

    handle_assembly = model.part("handle_assembly")
    handle_assembly.visual(
        Cylinder(radius=0.012, length=0.076),
        origin=_x_axis_cylinder_origin((0.038, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    handle_assembly.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=_x_axis_cylinder_origin((0.080, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    handle_assembly.visual(
        Cylinder(radius=0.007, length=0.160),
        origin=_y_axis_cylinder_origin((0.080, 0.0, 0.0)),
        material=steel,
        name="tommy_bar",
    )
    handle_assembly.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=_y_axis_cylinder_origin((0.080, -0.094, 0.0)),
        material=jaw_wood,
        name="left_grip",
    )
    handle_assembly.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=_y_axis_cylinder_origin((0.080, 0.094, 0.0)),
        material=jaw_wood,
        name="right_grip",
    )
    handle_assembly.inertial = Inertial.from_geometry(
        Box((0.11, 0.22, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    model.articulation(
        "bench_to_fixed_jaw",
        ArticulationType.FIXED,
        parent=bench,
        child=fixed_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.786)),
    )
    model.articulation(
        "fixed_to_sliding_jaw",
        ArticulationType.PRISMATIC,
        parent=fixed_jaw,
        child=sliding_jaw,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.10, lower=0.0, upper=0.20),
    )
    model.articulation(
        "sliding_jaw_to_handle",
        ArticulationType.CONTINUOUS,
        parent=sliding_jaw,
        child=handle_assembly,
        origin=Origin(xyz=(0.116, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    bench = object_model.get_part("bench")
    fixed_jaw = object_model.get_part("fixed_jaw")
    sliding_jaw = object_model.get_part("sliding_jaw")
    handle_assembly = object_model.get_part("handle_assembly")
    slide = object_model.get_articulation("fixed_to_sliding_jaw")
    handle_spin = object_model.get_articulation("sliding_jaw_to_handle")

    bench_edge = bench.get_visual("bench_edge")
    mount_plate = fixed_jaw.get_visual("mount_plate")
    top_flange = fixed_jaw.get_visual("top_flange")
    fixed_liner = fixed_jaw.get_visual("fixed_liner")
    left_rod = fixed_jaw.get_visual("left_guide_rod")
    right_rod = fixed_jaw.get_visual("right_guide_rod")

    moving_liner = sliding_jaw.get_visual("moving_liner")
    left_sleeve = sliding_jaw.get_visual("left_sleeve")
    right_sleeve = sliding_jaw.get_visual("right_sleeve")
    handle_boss = sliding_jaw.get_visual("handle_boss")
    spindle = handle_assembly.get_visual("spindle")
    tommy_bar = handle_assembly.get_visual("tommy_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        fixed_jaw,
        sliding_jaw,
        elem_a=left_rod,
        elem_b=left_sleeve,
        reason="the left guide rod occupies the unmodeled bore inside the moving jaw sleeve",
    )
    ctx.allow_overlap(
        fixed_jaw,
        sliding_jaw,
        elem_a=right_rod,
        elem_b=right_sleeve,
        reason="the right guide rod occupies the unmodeled bore inside the moving jaw sleeve",
    )
    ctx.allow_overlap(
        handle_assembly,
        sliding_jaw,
        elem_a=spindle,
        elem_b=handle_boss,
        reason="the spindle passes through an unmodeled bore in the cast front boss",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.check(
        "sliding_jaw_uses_prismatic_joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        "the moving jaw should slide on a prismatic joint",
    )
    ctx.check(
        "sliding_jaw_travels_along_x",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected slide axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "handle_rotates_continuously",
        handle_spin.articulation_type == ArticulationType.CONTINUOUS,
        "the operating handle should rotate continuously about the screw axis",
    )
    ctx.check(
        "handle_rotates_about_screw_axis",
        tuple(handle_spin.axis) == (1.0, 0.0, 0.0),
        f"expected handle axis (1, 0, 0), got {handle_spin.axis}",
    )

    ctx.expect_gap(
        fixed_jaw,
        bench,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=mount_plate,
        negative_elem=bench_edge,
        name="fixed_jaw_mounts_flush_to_bench_edge",
    )
    ctx.expect_gap(
        bench,
        fixed_jaw,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=bench_edge,
        negative_elem=top_flange,
        name="top_flange_seats_under_bench",
    )
    ctx.expect_overlap(
        sliding_jaw,
        fixed_jaw,
        axes="yz",
        min_overlap=0.095,
        elem_a=moving_liner,
        elem_b=fixed_liner,
        name="jaw_faces_align_across_height_and_width",
    )
    ctx.expect_gap(
        sliding_jaw,
        fixed_jaw,
        axis="x",
        min_gap=0.003,
        max_gap=0.005,
        positive_elem=moving_liner,
        negative_elem=fixed_liner,
        name="closed_pose_keeps_a_small_visible_opening",
    )
    ctx.expect_overlap(
        sliding_jaw,
        fixed_jaw,
        axes="yz",
        min_overlap=0.018,
        elem_a=left_sleeve,
        elem_b=left_rod,
        name="left_guide_sleeve_stays_centered_on_left_rod",
    )
    ctx.expect_overlap(
        sliding_jaw,
        fixed_jaw,
        axes="yz",
        min_overlap=0.018,
        elem_a=right_sleeve,
        elem_b=right_rod,
        name="right_guide_sleeve_stays_centered_on_right_rod",
    )
    ctx.expect_overlap(
        handle_assembly,
        sliding_jaw,
        axes="yz",
        min_overlap=0.024,
        elem_a=spindle,
        elem_b=handle_boss,
        name="handle_spindle_seats_in_the_front_boss",
    )

    with ctx.pose({slide: 0.0, handle_spin: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="vise_closed_pose_has_no_unallowed_overlaps")
        ctx.fail_if_isolated_parts(name="vise_closed_pose_has_no_floating_parts")
        ctx.expect_overlap(
            handle_assembly,
            sliding_jaw,
            axes="yz",
            min_overlap=0.024,
            elem_a=spindle,
            elem_b=handle_boss,
            name="handle_spindle_stays_centered_in_boss_when_resting",
        )

    with ctx.pose({slide: 0.0, handle_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            handle_assembly,
            sliding_jaw,
            axes="yz",
            min_overlap=0.024,
            elem_a=spindle,
            elem_b=handle_boss,
            name="handle_remains_mounted_when_rotated",
        )

    upper = slide.motion_limits.upper
    if upper is not None:
        with ctx.pose({slide: upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="vise_open_pose_has_no_unallowed_overlaps")
            ctx.fail_if_isolated_parts(name="vise_open_pose_has_no_floating_parts")
            ctx.expect_gap(
                sliding_jaw,
                fixed_jaw,
                axis="x",
                min_gap=0.203,
                max_gap=0.205,
                positive_elem=moving_liner,
                negative_elem=fixed_liner,
                name="open_pose_creates_a_wide_clamping_gap",
            )
            ctx.expect_overlap(
                sliding_jaw,
                fixed_jaw,
                axes="yz",
                min_overlap=0.018,
                elem_a=left_sleeve,
                elem_b=left_rod,
                name="left_sleeve_stays_engaged_when_open",
            )
            ctx.expect_overlap(
                sliding_jaw,
                fixed_jaw,
                axes="yz",
                min_overlap=0.018,
                elem_a=right_sleeve,
                elem_b=right_rod,
                name="right_sleeve_stays_engaged_when_open",
            )

        with ctx.pose({slide: upper, handle_spin: math.pi / 2.0}):
            ctx.expect_overlap(
                handle_assembly,
                sliding_jaw,
                axes="yz",
                min_overlap=0.024,
                elem_a=spindle,
                elem_b=handle_boss,
                name="handle_stays_mounted_with_jaw_open_and_handle_turned",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

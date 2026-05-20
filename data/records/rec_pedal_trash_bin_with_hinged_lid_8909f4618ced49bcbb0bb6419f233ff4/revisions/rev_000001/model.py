from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_trash_bin")

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    lid_metal = model.material("lid_metal", rgba=(0.73, 0.74, 0.76, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.14, 0.14, 0.14, 1.0))
    bucket_gray = model.material("bucket_gray", rgba=(0.79, 0.81, 0.84, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.22, 0.22, 0.24, 1.0))

    outer_w = 0.290
    outer_d = 0.350
    outer_h = 0.470
    shell_t = 0.006
    bottom_t = 0.008

    lid_axis_y = -outer_d / 2.0 + 0.010
    lid_axis_z = outer_h - 0.004

    body = model.part("outer_shell")
    wall_h = outer_h
    front_back_w = outer_w - 2.0 * shell_t + 0.001
    body.visual(
        Box((outer_w - 0.010, outer_d - 0.010, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=stainless,
        name="shell_bottom",
    )
    body.visual(
        Box((front_back_w, shell_t, wall_h)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 - shell_t / 2.0, wall_h / 2.0)),
        material=stainless,
        name="shell_front_wall",
    )
    body.visual(
        Box((front_back_w, shell_t, wall_h)),
        origin=Origin(xyz=(0.0, -outer_d / 2.0 + shell_t / 2.0, wall_h / 2.0)),
        material=stainless,
        name="shell_rear_wall",
    )
    body.visual(
        Box((shell_t, outer_d, wall_h)),
        origin=Origin(xyz=(outer_w / 2.0 - shell_t / 2.0, 0.0, wall_h / 2.0)),
        material=stainless,
        name="shell_right_wall",
    )
    body.visual(
        Box((shell_t, outer_d, wall_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + shell_t / 2.0, 0.0, wall_h / 2.0)),
        material=stainless,
        name="shell_left_wall",
    )
    pedal_bracket_y = outer_d / 2.0 + 0.010
    for x_pos, name in ((-0.105, "pedal_bracket_left"), (0.105, "pedal_bracket_right")):
        body.visual(
            Box((0.020, 0.024, 0.022)),
            origin=Origin(xyz=(x_pos, pedal_bracket_y, 0.058)),
            material=stainless,
            name=name,
        )

    body.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(-0.092, lid_axis_y, lid_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_knuckle_left",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.092, lid_axis_y, lid_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_knuckle_right",
    )

    lid = model.part("lid")
    lid_panel_depth = 0.355
    lid_panel_start = 0.014
    lid_panel_z = 0.017
    lid.visual(
        Box((0.296, lid_panel_depth, 0.014)),
        origin=Origin(xyz=(0.0, lid_panel_start + lid_panel_depth / 2.0, lid_panel_z)),
        material=lid_metal,
        name="lid_panel",
    )
    lid.visual(
        Box((0.118, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.009, 0.006)),
        material=lid_metal,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lid_metal,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.260, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, lid_panel_start + lid_panel_depth - 0.007, 0.008)),
        material=lid_metal,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.008, lid_panel_depth - 0.030, 0.026)),
        origin=Origin(xyz=(0.149, lid_panel_start + (lid_panel_depth - 0.030) / 2.0 + 0.015, 0.006)),
        material=lid_metal,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((0.008, lid_panel_depth - 0.030, 0.026)),
        origin=Origin(xyz=(-0.149, lid_panel_start + (lid_panel_depth - 0.030) / 2.0 + 0.015, 0.006)),
        material=lid_metal,
        name="lid_left_skirt",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.007, length=0.190),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=pedal_black,
        name="pedal_pivot_rod",
    )
    pedal.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(-0.060, 0.018, -0.010)),
        material=pedal_black,
        name="pedal_arm_left",
    )
    pedal.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(0.060, 0.018, -0.010)),
        material=pedal_black,
        name="pedal_arm_right",
    )
    pedal.visual(
        Box((0.150, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, 0.045, -0.022)),
        material=pedal_black,
        name="pedal_bridge",
    )
    pedal.visual(
        Box((0.182, 0.046, 0.008)),
        origin=Origin(xyz=(0.0, 0.079, -0.034), rpy=(-0.18, 0.0, 0.0)),
        material=pedal_black,
        name="pedal_tread",
    )

    bucket = model.part("inner_bucket")
    bucket_w = 0.252
    bucket_d = 0.312
    bucket_h = 0.388
    bucket_t = 0.0045
    bucket_lip_t = 0.012
    bucket.visual(
        Box((bucket_w, bucket_d, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bucket_gray,
        name="bucket_bottom",
    )
    bucket.visual(
        Box((bucket_w - 2.0 * bucket_t + 0.001, bucket_t, bucket_h)),
        origin=Origin(xyz=(0.0, bucket_d / 2.0 - bucket_t / 2.0, bucket_h / 2.0)),
        material=bucket_gray,
        name="bucket_front_wall",
    )
    bucket.visual(
        Box((bucket_w - 2.0 * bucket_t + 0.001, bucket_t, bucket_h)),
        origin=Origin(xyz=(0.0, -bucket_d / 2.0 + bucket_t / 2.0, bucket_h / 2.0)),
        material=bucket_gray,
        name="bucket_rear_wall",
    )
    bucket.visual(
        Box((bucket_t, bucket_d, bucket_h)),
        origin=Origin(xyz=(bucket_w / 2.0 - bucket_t / 2.0, 0.0, bucket_h / 2.0)),
        material=bucket_gray,
        name="bucket_right_wall",
    )
    bucket.visual(
        Box((bucket_t, bucket_d, bucket_h)),
        origin=Origin(xyz=(-bucket_w / 2.0 + bucket_t / 2.0, 0.0, bucket_h / 2.0)),
        material=bucket_gray,
        name="bucket_left_wall",
    )
    bucket.visual(
        Box((bucket_w + 0.010, bucket_lip_t, 0.010)),
        origin=Origin(xyz=(0.0, bucket_d / 2.0 - 0.001, bucket_h - 0.004)),
        material=bucket_gray,
        name="bucket_front_rim",
    )
    bucket.visual(
        Box((bucket_w + 0.010, bucket_lip_t, 0.010)),
        origin=Origin(xyz=(0.0, -bucket_d / 2.0 + 0.001, bucket_h - 0.004)),
        material=bucket_gray,
        name="bucket_rear_rim",
    )
    bucket.visual(
        Box((bucket_lip_t, bucket_d - 0.020, 0.010)),
        origin=Origin(xyz=(bucket_w / 2.0 - 0.001, 0.0, bucket_h - 0.004)),
        material=bucket_gray,
        name="bucket_right_rim",
    )
    bucket.visual(
        Box((bucket_lip_t, bucket_d - 0.020, 0.010)),
        origin=Origin(xyz=(-bucket_w / 2.0 + 0.001, 0.0, bucket_h - 0.004)),
        material=bucket_gray,
        name="bucket_left_rim",
    )
    bucket.visual(
        Box((0.014, 0.016, 0.020)),
        origin=Origin(xyz=(-0.118, 0.0, 0.353)),
        material=bucket_gray,
        name="bucket_handle_boss_left",
    )
    bucket.visual(
        Box((0.014, 0.016, 0.020)),
        origin=Origin(xyz=(0.118, 0.0, 0.353)),
        material=bucket_gray,
        name="bucket_handle_boss_right",
    )

    handle_path = [
        (-0.112, 0.000, 0.000),
        (-0.104, -0.018, 0.020),
        (-0.062, -0.050, 0.053),
        (0.000, -0.064, 0.072),
        (0.062, -0.050, 0.053),
        (0.104, -0.018, 0.020),
        (0.112, 0.000, 0.000),
    ]
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            handle_path,
            radius=0.0032,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "inner_bucket_bail_handle",
    )
    bucket_handle = model.part("inner_bucket_handle")
    bucket_handle.visual(
        handle_mesh,
        material=handle_dark,
        name="bucket_handle_wire",
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_axis_y, lid_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "shell_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, outer_d / 2.0 + 0.018, 0.058)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=0.55),
    )
    model.articulation(
        "shell_to_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, bottom_t)),
    )
    model.articulation(
        "bucket_to_handle",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=bucket_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.353)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("outer_shell")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    bucket = object_model.get_part("inner_bucket")
    bucket_handle = object_model.get_part("inner_bucket_handle")

    lid_joint = object_model.get_articulation("shell_to_lid")
    pedal_joint = object_model.get_articulation("shell_to_pedal")
    handle_joint = object_model.get_articulation("bucket_to_handle")

    ctx.expect_overlap(
        lid,
        bucket,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.24,
        name="closed lid covers the bucket opening footprint",
    )
    ctx.expect_within(
        bucket,
        body,
        axes="xy",
        margin=0.0,
        name="inner bucket stays nested inside the outer shell footprint",
    )
    ctx.expect_within(
        bucket_handle,
        bucket,
        axes="xy",
        inner_elem="bucket_handle_wire",
        margin=0.020,
        name="bucket handle rests inside the bucket footprint",
    )

    closed_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
    rest_pedal = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
    rest_handle = ctx.part_element_world_aabb(bucket_handle, elem="bucket_handle_wire")

    with ctx.pose({lid_joint: 1.05}):
        opened_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({pedal_joint: 0.45}):
        pressed_pedal = ctx.part_element_world_aabb(pedal, elem="pedal_tread")

    with ctx.pose({handle_joint: 1.00}):
        raised_handle = ctx.part_element_world_aabb(bucket_handle, elem="bucket_handle_wire")

    ctx.check(
        "lid rotates upward from the rear hinge",
        closed_lid is not None
        and opened_lid is not None
        and opened_lid[1][2] > closed_lid[1][2] + 0.200
        and opened_lid[1][1] < closed_lid[1][1] - 0.150,
        details=f"closed={closed_lid}, opened={opened_lid}",
    )
    ctx.check(
        "pedal toe drops when pressed",
        rest_pedal is not None
        and pressed_pedal is not None
        and pressed_pedal[0][2] < rest_pedal[0][2] - 0.015,
        details=f"rest={rest_pedal}, pressed={pressed_pedal}",
    )
    ctx.check(
        "inner bucket handle lifts from the rim pivots",
        rest_handle is not None
        and raised_handle is not None
        and raised_handle[1][2] > rest_handle[1][2] + 0.015
        and raised_handle[1][1] > rest_handle[1][1] + 0.040,
        details=f"rest={rest_handle}, raised={raised_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

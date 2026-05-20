from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_step_trash_bin")

    dark_plastic = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    shell_plastic = model.material("warm_gray_shell", rgba=(0.62, 0.64, 0.62, 1.0))
    inner_plastic = model.material("pale_inner_bucket", rgba=(0.86, 0.87, 0.83, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.48, 0.50, 0.50, 1.0))
    pedal_metal = model.material("satin_pedal", rgba=(0.72, 0.72, 0.68, 1.0))

    body_w = 0.34
    body_d = 0.26
    body_h = 0.45
    wall = 0.018

    shell = model.part("outer_shell")
    shell.visual(
        Box((body_w, body_d, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_plastic,
        name="base_plinth",
    )
    shell.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2 + wall / 2, 0.0, body_h / 2)),
        material=shell_plastic,
        name="side_wall_0",
    )
    shell.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2 - wall / 2, 0.0, body_h / 2)),
        material=shell_plastic,
        name="side_wall_1",
    )
    shell.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2 + wall / 2, body_h / 2)),
        material=shell_plastic,
        name="front_wall",
    )
    shell.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2 - wall / 2, body_h / 2)),
        material=shell_plastic,
        name="rear_wall",
    )
    # A dark lower recess behind the foot pedal, connected to the front wall.
    shell.visual(
        Box((0.245, 0.008, 0.090)),
        origin=Origin(xyz=(0.0, -body_d / 2 - 0.004, 0.075)),
        material=dark_plastic,
        name="pedal_recess",
    )
    # Thin top rim strips leave the top genuinely open while showing wall thickness.
    shell.visual(
        Box((body_w, wall * 0.90, 0.014)),
        origin=Origin(xyz=(0.0, -body_d / 2 + wall / 2, body_h - 0.007)),
        material=dark_plastic,
        name="top_rim_front",
    )
    shell.visual(
        Box((body_w, wall * 0.90, 0.014)),
        origin=Origin(xyz=(0.0, body_d / 2 - wall / 2, body_h - 0.007)),
        material=dark_plastic,
        name="top_rim_rear",
    )
    shell.visual(
        Box((wall * 0.90, body_d, 0.014)),
        origin=Origin(xyz=(-body_w / 2 + wall / 2, 0.0, body_h - 0.007)),
        material=dark_plastic,
        name="top_rim_0",
    )
    shell.visual(
        Box((wall * 0.90, body_d, 0.014)),
        origin=Origin(xyz=(body_w / 2 - wall / 2, 0.0, body_h - 0.007)),
        material=dark_plastic,
        name="top_rim_1",
    )

    hinge_y = body_d / 2 + 0.020
    hinge_z = body_h + 0.013
    shell.visual(
        Box((0.140, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, body_d / 2 + 0.024, body_h + 0.009)),
        material=hinge_metal,
        name="rear_hinge_leaf",
    )
    shell.visual(
        Box((0.140, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, body_d / 2 + 0.013, body_h - 0.010)),
        material=hinge_metal,
        name="rear_hinge_foot",
    )
    shell.visual(
        Cylinder(radius=0.008, length=0.120),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2, 0.0)),
        material=hinge_metal,
        name="rear_hinge_knuckle",
    )

    pivot_y = -body_d / 2 - 0.012
    pivot_z = 0.080
    for i, x in enumerate((-0.133, 0.133)):
        shell.visual(
            Box((0.034, 0.024, 0.062)),
            origin=Origin(xyz=(x, -body_d / 2 - 0.010, pivot_z)),
            material=dark_plastic,
            name=f"pedal_pivot_ear_{i}",
        )

    bucket = model.part("inner_bucket")
    bucket_w = 0.285
    bucket_d = 0.205
    bucket_bottom = 0.040
    bucket_wall_h = 0.350
    bucket_top = bucket_bottom + bucket_wall_h
    bucket_wall = 0.010
    bucket.visual(
        Box((bucket_w, bucket_d, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, bucket_bottom + 0.009)),
        material=inner_plastic,
        name="bucket_floor",
    )
    bucket.visual(
        Box((bucket_wall, bucket_d, bucket_wall_h)),
        origin=Origin(xyz=(-bucket_w / 2 + bucket_wall / 2, 0.0, bucket_bottom + bucket_wall_h / 2)),
        material=inner_plastic,
        name="bucket_side_0",
    )
    bucket.visual(
        Box((bucket_wall, bucket_d, bucket_wall_h)),
        origin=Origin(xyz=(bucket_w / 2 - bucket_wall / 2, 0.0, bucket_bottom + bucket_wall_h / 2)),
        material=inner_plastic,
        name="bucket_side_1",
    )
    bucket.visual(
        Box((bucket_w, bucket_wall, bucket_wall_h)),
        origin=Origin(xyz=(0.0, -bucket_d / 2 + bucket_wall / 2, bucket_bottom + bucket_wall_h / 2)),
        material=inner_plastic,
        name="bucket_front",
    )
    bucket.visual(
        Box((bucket_w, bucket_wall, bucket_wall_h)),
        origin=Origin(xyz=(0.0, bucket_d / 2 - bucket_wall / 2, bucket_bottom + bucket_wall_h / 2)),
        material=inner_plastic,
        name="bucket_rear",
    )
    bucket.visual(
        Box((bucket_w + 0.018, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -bucket_d / 2 + 0.006, bucket_top + 0.007)),
        material=inner_plastic,
        name="bucket_rim_front",
    )
    bucket.visual(
        Box((bucket_w + 0.018, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, bucket_d / 2 - 0.006, bucket_top + 0.007)),
        material=inner_plastic,
        name="bucket_rim_rear",
    )
    bucket.visual(
        Box((0.012, bucket_d, 0.014)),
        origin=Origin(xyz=(-bucket_w / 2 - 0.006, 0.0, bucket_top + 0.007)),
        material=inner_plastic,
        name="bucket_rim_0",
    )
    bucket.visual(
        Box((0.012, bucket_d, 0.014)),
        origin=Origin(xyz=(bucket_w / 2 + 0.006, 0.0, bucket_top + 0.007)),
        material=inner_plastic,
        name="bucket_rim_1",
    )
    handle_pivot_z = bucket_top + 0.018
    bucket.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(
            xyz=(-(bucket_w / 2 - bucket_wall / 2), 0.0, handle_pivot_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=hinge_metal,
        name="side_pivot_0",
    )
    bucket.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(
            xyz=(bucket_w / 2 - bucket_wall / 2, 0.0, handle_pivot_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=hinge_metal,
        name="side_pivot_1",
    )

    lid = model.part("lid")
    lid_w = 0.365
    lid_d = 0.290
    lid_t = 0.026
    lid_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(lid_w, lid_d, 0.034, corner_segments=10),
            lid_t,
            cap=True,
            center=True,
        ),
        "rounded_lid_panel",
    )
    lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.0, -lid_d / 2 - 0.010, 0.0)),
        material=dark_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.210, 0.115, 0.006)),
        origin=Origin(xyz=(0.0, -lid_d / 2 - 0.010, lid_t / 2 + 0.003)),
        material=shell_plastic,
        name="raised_lid_pad",
    )
    for i, x in enumerate((-0.125, 0.125)):
        lid.visual(
            Cylinder(radius=0.008, length=0.076),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_knuckle_{i}",
        )
        lid.visual(
            Box((0.040, 0.018, 0.008)),
            origin=Origin(xyz=(x, -0.010, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_strap_{i}",
        )

    pedal = model.part("front_pedal")
    pedal_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.225, 0.082, 0.017, corner_segments=8),
            0.018,
            cap=True,
            center=True,
        ),
        "rounded_step_pedal",
    )
    pedal.visual(
        Cylinder(radius=0.007, length=0.225),
        origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
        material=hinge_metal,
        name="pedal_axle",
    )
    pedal.visual(
        pedal_mesh,
        origin=Origin(xyz=(0.0, -0.075, -0.026)),
        material=pedal_metal,
        name="pedal_pad",
    )
    pedal.visual(
        Box((0.050, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, -0.040, -0.012)),
        material=hinge_metal,
        name="pedal_arm",
    )

    bucket_handle = model.part("bucket_handle")
    bail_half_w = bucket_w / 2 - bucket_wall / 2
    bail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-bail_half_w, 0.0, 0.0),
                (-0.105, -0.030, -0.004),
                (0.0, -0.078, -0.006),
                (0.105, -0.030, -0.004),
                (bail_half_w, 0.0, 0.0),
            ],
            radius=0.0035,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
        "folding_bucket_bail",
    )
    bucket_handle.visual(
        bail_mesh,
        material=hinge_metal,
        name="bail_wire",
    )

    model.articulation(
        "shell_to_bucket",
        ArticulationType.FIXED,
        parent=shell,
        child=bucket,
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, body_h + lid_t / 2)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "bucket_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=bucket_handle,
        origin=Origin(xyz=(0.0, 0.0, handle_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("outer_shell")
    bucket = object_model.get_part("inner_bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("front_pedal")
    handle = object_model.get_part("bucket_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_pivot = object_model.get_articulation("pedal_pivot")
    handle_pivot = object_model.get_articulation("bucket_handle_pivot")

    ctx.allow_overlap(
        bucket,
        handle,
        elem_a="side_pivot_0",
        elem_b="bail_wire",
        reason="The bail wire end is intentionally captured inside the left side pivot boss.",
    )
    ctx.allow_overlap(
        bucket,
        handle,
        elem_a="side_pivot_1",
        elem_b="bail_wire",
        reason="The bail wire end is intentionally captured inside the right side pivot boss.",
    )
    for i in (0, 1):
        ctx.expect_overlap(
            bucket,
            handle,
            axes="xz",
            elem_a=f"side_pivot_{i}",
            elem_b="bail_wire",
            min_overlap=0.003,
            name=f"bail wire captured in side pivot {i}",
        )

    ctx.expect_within(
        bucket,
        shell,
        axes="xy",
        margin=0.002,
        name="inner bucket sits inside the rectangular outer shell footprint",
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_rim_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid rests on the front top rim",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_panel",
        elem_b="top_rim_front",
        min_overlap=0.010,
        name="lid covers the body opening at rest",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid hinge lifts the front edge upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_pad")
    with ctx.pose({pedal_pivot: 0.40}):
        depressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_pad")
    ctx.check(
        "front pedal rotates downward when depressed",
        rest_pedal_aabb is not None
        and depressed_pedal_aabb is not None
        and depressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.015,
        details=f"rest={rest_pedal_aabb}, depressed={depressed_pedal_aabb}",
    )

    folded_handle_aabb = ctx.part_element_world_aabb(handle, elem="bail_wire")
    with ctx.pose({handle_pivot: 1.20}):
        raised_handle_aabb = ctx.part_element_world_aabb(handle, elem="bail_wire")
    ctx.check(
        "inner bucket bail handle rotates upward from the rim",
        folded_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > folded_handle_aabb[1][2] + 0.045,
        details=f"folded={folded_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

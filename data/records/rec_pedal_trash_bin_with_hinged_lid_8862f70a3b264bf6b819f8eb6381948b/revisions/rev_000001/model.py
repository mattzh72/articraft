from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _open_rounded_box(width: float, depth: float, height: float, wall: float, bottom: float, radius: float):
    """CadQuery open-topped rectangular shell, in meters, with its base at z=0."""
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - bottom + 0.02, centered=(True, True, False))
        .translate((0.0, 0.0, bottom))
    )
    return outer.cut(inner)


def _rounded_slab(width: float, depth: float, thickness: float, radius: float, y_center: float = 0.0):
    slab = (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, True))
        .edges("|Z")
        .fillet(radius)
    )
    return slab.translate((0.0, y_center, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_kitchen_pedal_bin")

    brushed_steel = model.material("soft_brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_seam = model.material("dark_shadow_seam", rgba=(0.035, 0.038, 0.04, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    bucket_plastic = model.material("warm_gray_plastic", rgba=(0.62, 0.64, 0.62, 1.0))
    handle_plastic = model.material("pale_bucket_handle", rgba=(0.78, 0.80, 0.78, 1.0))

    shell_w = 0.36
    shell_d = 0.28
    shell_h = 0.46
    wall = 0.014
    bottom = 0.024
    lid_t = 0.024
    hinge_y = shell_d / 2.0 + 0.020
    hinge_z = shell_h + 0.003 + lid_t / 2.0

    shell = model.part("outer_shell")
    shell.visual(
        Box((shell_w, shell_d, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=brushed_steel,
        name="bottom_pan",
    )
    shell.visual(
        Box((shell_w, wall, shell_h)),
        origin=Origin(xyz=(0.0, -shell_d / 2.0 + wall / 2.0, shell_h / 2.0)),
        material=brushed_steel,
        name="front_wall",
    )
    shell.visual(
        Box((shell_w, wall, shell_h)),
        origin=Origin(xyz=(0.0, shell_d / 2.0 - wall / 2.0, shell_h / 2.0)),
        material=brushed_steel,
        name="rear_wall",
    )
    shell.visual(
        Box((wall, shell_d, shell_h)),
        origin=Origin(xyz=(-shell_w / 2.0 + wall / 2.0, 0.0, shell_h / 2.0)),
        material=brushed_steel,
        name="side_wall_0",
    )
    shell.visual(
        Box((wall, shell_d, shell_h)),
        origin=Origin(xyz=(shell_w / 2.0 - wall / 2.0, 0.0, shell_h / 2.0)),
        material=brushed_steel,
        name="side_wall_1",
    )

    top_rim = (
        cq.Workplane("XY")
        .box(shell_w + 0.006, shell_d + 0.006, 0.006, centered=(True, True, True))
        .cut(cq.Workplane("XY").box(shell_w - 2.0 * wall + 0.020, shell_d - 2.0 * wall + 0.020, 0.010, centered=(True, True, True)))
        .translate((0.0, 0.0, shell_h - 0.001))
    )
    shell.visual(mesh_from_cadquery(top_rim, "dark_top_seam"), material=dark_seam, name="top_seam")

    # Rear hinge knuckles and lower-front pedal brackets are fixed to the outer shell.
    for x in (-0.122, 0.122):
        shell.visual(
            Cylinder(radius=0.009, length=0.075),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_seam,
            name=f"rear_hinge_knuckle_{0 if x < 0 else 1}",
        )
        shell.visual(
            Box((0.080, 0.018, 0.028)),
            origin=Origin(xyz=(x, shell_d / 2.0 + 0.011, shell_h + 0.004)),
            material=brushed_steel,
            name=f"rear_hinge_mount_{0 if x < 0 else 1}",
        )
    shell.visual(
        Cylinder(radius=0.0035, length=0.305),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_seam,
        name="rear_hinge_pin",
    )

    for x in (-0.118, 0.118):
        shell.visual(
            Box((0.026, 0.020, 0.056)),
            origin=Origin(xyz=(x, -shell_d / 2.0 - 0.010, 0.058)),
            material=brushed_steel,
            name=f"pedal_pivot_cheek_{0 if x < 0 else 1}",
        )
    for x in (-0.125, 0.125):
        shell.visual(
            Box((0.055, 0.045, 0.010)),
            origin=Origin(xyz=(x, -0.080, 0.005)),
            material=black_plastic,
            name=f"rubber_foot_{0 if x < 0 else 1}",
        )

    bucket = model.part("inner_bucket")
    bucket_w = 0.318
    bucket_d = 0.236
    bucket_h = 0.420
    bucket_bottom_z = bottom
    bucket_wall = 0.007
    bucket_floor = 0.009
    bucket.visual(
        Box((bucket_w, bucket_d, bucket_floor)),
        origin=Origin(xyz=(0.0, 0.0, bucket_bottom_z + bucket_floor / 2.0)),
        material=bucket_plastic,
        name="bucket_floor",
    )
    bucket.visual(
        Box((bucket_w, bucket_wall, bucket_h)),
        origin=Origin(xyz=(0.0, -bucket_d / 2.0 + bucket_wall / 2.0, bucket_bottom_z + bucket_h / 2.0)),
        material=bucket_plastic,
        name="bucket_front_wall",
    )
    bucket.visual(
        Box((bucket_w, bucket_wall, bucket_h)),
        origin=Origin(xyz=(0.0, bucket_d / 2.0 - bucket_wall / 2.0, bucket_bottom_z + bucket_h / 2.0)),
        material=bucket_plastic,
        name="bucket_rear_wall",
    )
    bucket.visual(
        Box((bucket_wall, bucket_d, bucket_h)),
        origin=Origin(xyz=(-bucket_w / 2.0 + bucket_wall / 2.0, 0.0, bucket_bottom_z + bucket_h / 2.0)),
        material=bucket_plastic,
        name="bucket_side_wall_0",
    )
    bucket.visual(
        Box((bucket_wall, bucket_d, bucket_h)),
        origin=Origin(xyz=(bucket_w / 2.0 - bucket_wall / 2.0, 0.0, bucket_bottom_z + bucket_h / 2.0)),
        material=bucket_plastic,
        name="bucket_side_wall_1",
    )
    bucket.visual(
        Box((bucket_w + 0.020, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -bucket_d / 2.0 - 0.002, bucket_bottom_z + bucket_h - 0.018)),
        material=bucket_plastic,
        name="front_bucket_lip",
    )
    handle_axis_z = bucket_bottom_z + bucket_h - 0.045
    handle_axis_y = -0.010
    for x, suffix in [(-bucket_w / 2.0 + 0.010, "0"), (bucket_w / 2.0 - 0.010, "1")]:
        bucket.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=(x, handle_axis_y, handle_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bucket_plastic,
            name=f"handle_side_pivot_{suffix}",
        )

    lid = model.part("lid")
    lid_w = shell_w + 0.030
    lid_d = shell_d + 0.030
    # The rear edge sits just in front of the hinge barrels, leaving a clean shadow seam.
    lid.visual(
        mesh_from_cadquery(_rounded_slab(lid_w, lid_d, lid_t, 0.014, y_center=-(lid_d / 2.0 + 0.020)), "flat_lid_panel"),
        material=brushed_steel,
        name="flat_lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_seam,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.135, 0.025, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, -0.006)),
        material=dark_seam,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((lid_w - 0.035, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -(lid_d + 0.014), -0.010)),
        material=dark_seam,
        name="front_lid_shadow",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Box((0.220, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.012, 0.044), rpy=(0.10, 0.0, 0.0)),
        material=brushed_steel,
        name="front_pedal_plate",
    )
    pedal.visual(
        Box((0.188, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -0.023, 0.045), rpy=(0.10, 0.0, 0.0)),
        material=black_plastic,
        name="rubber_pedal_pad",
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_seam,
        name="pedal_pivot_sleeve",
    )

    handle = model.part("bucket_handle")
    handle_wire = tube_from_spline_points(
        [
            (-bucket_w / 2.0 + 0.013, 0.0, 0.0),
            (-bucket_w / 2.0 + 0.035, -0.020, 0.035),
            (0.0, -0.036, 0.058),
            (bucket_w / 2.0 - 0.035, -0.020, 0.035),
            (bucket_w / 2.0 - 0.013, 0.0, 0.0),
        ],
        radius=0.003,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(mesh_from_geometry(handle_wire, "bucket_handle_wire"), material=handle_plastic, name="handle_wire")

    model.articulation(
        "shell_to_bucket",
        ArticulationType.FIXED,
        parent=shell,
        child=bucket,
        origin=Origin(),
    )
    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "front_pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(0.0, -shell_d / 2.0 - 0.024, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=3.0, lower=0.0, upper=0.38),
    )
    model.articulation(
        "bucket_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=handle,
        origin=Origin(xyz=(0.0, handle_axis_y, handle_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.15, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("outer_shell")
    bucket = object_model.get_part("inner_bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    handle = object_model.get_part("bucket_handle")
    lid_hinge = object_model.get_articulation("rear_lid_hinge")
    pedal_pivot = object_model.get_articulation("front_pedal_pivot")
    handle_pivot = object_model.get_articulation("bucket_handle_pivot")

    ctx.allow_overlap(
        lid,
        shell,
        elem_a="lid_hinge_knuckle",
        elem_b="rear_hinge_pin",
        reason="The fixed hinge pin is intentionally captured inside the rotating lid hinge knuckle.",
    )
    ctx.expect_within(
        shell,
        lid,
        axes="yz",
        inner_elem="rear_hinge_pin",
        outer_elem="lid_hinge_knuckle",
        margin=0.001,
        name="hinge pin is centered inside lid knuckle",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="x",
        elem_a="lid_hinge_knuckle",
        elem_b="rear_hinge_pin",
        min_overlap=0.110,
        name="lid knuckle captures hinge pin length",
    )

    for pivot in ("handle_side_pivot_0", "handle_side_pivot_1"):
        ctx.allow_overlap(
            handle,
            bucket,
            elem_a="handle_wire",
            elem_b=pivot,
            reason="The inner-bucket handle wire ends are intentionally captured in the molded side pivot bosses.",
        )
        ctx.expect_overlap(
            handle,
            bucket,
            axes="xyz",
            elem_a="handle_wire",
            elem_b=pivot,
            min_overlap=0.005,
            name=f"{pivot} captures handle wire end",
        )

    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="flat_lid_panel",
        negative_elem="top_seam",
        min_gap=0.0,
        max_gap=0.006,
        name="flat lid keeps a narrow top seam",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="flat_lid_panel",
        elem_b="top_seam",
        min_overlap=0.240,
        name="lid covers rectangular top opening",
    )
    ctx.expect_within(
        bucket,
        shell,
        axes="xy",
        margin=0.004,
        name="inner bucket nests inside outer shell footprint",
    )
    ctx.expect_gap(
        lid,
        handle,
        axis="z",
        positive_elem="flat_lid_panel",
        negative_elem="handle_wire",
        min_gap=0.001,
        max_gap=0.020,
        name="bucket handle remains visible below lid line",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_lid_shadow")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_lid_shadow")
    ctx.check(
        "rear hinge opens the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="front_pedal_plate")
    with ctx.pose({pedal_pivot: 0.34}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="front_pedal_plate")
    ctx.check(
        "front pedal plate swings outward on lower pivot",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][1] < rest_pedal_aabb[0][1] - 0.015,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_wire")
    with ctx.pose({handle_pivot: 0.55}):
        lifted_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_wire")
    ctx.check(
        "inner bucket handle rotates on side pivots",
        rest_handle_aabb is not None
        and lifted_handle_aabb is not None
        and lifted_handle_aabb[0][1] < rest_handle_aabb[0][1] - 0.015,
        details=f"rest={rest_handle_aabb}, lifted={lifted_handle_aabb}",
    )
    return ctx.report()


object_model = build_object_model()

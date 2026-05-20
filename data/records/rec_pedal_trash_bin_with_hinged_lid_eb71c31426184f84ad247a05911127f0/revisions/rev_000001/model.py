from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0007, angular_tolerance=0.08)


def _ring(outer_radius: float, inner_radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(height)
        .cut(cq.Workplane("XY").circle(inner_radius).extrude(height))
        .translate((0.0, 0.0, z0))
    )


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center_xyz
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y - length * 0.5, z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_bin")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.27, 0.28, 0.30, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.12, 0.12, 0.13, 1.0))
    bucket_gray = model.material("bucket_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.50, 0.53, 0.56, 1.0))

    body_radius = 0.135
    body_height = 0.430
    body_wall = 0.0025
    bucket_radius = 0.121
    bucket_height = 0.390
    bucket_wall = 0.0022
    hinge_axis_x = -(body_radius + 0.013)
    hinge_axis_z = body_height + 0.012
    pedal_axis_x = body_radius + 0.015
    pedal_axis_z = 0.064

    body = model.part("body")

    body_shell = (
        cq.Workplane("XY")
        .circle(body_radius)
        .extrude(body_height)
        .cut(
            cq.Workplane("XY")
            .circle(body_radius - body_wall)
            .extrude(body_height - 0.004)
            .translate((0.0, 0.0, 0.004))
        )
        .union(_ring(body_radius + 0.004, body_radius - 0.001, 0.012, body_height - 0.012))
        .union(_ring(body_radius + 0.006, body_radius - 0.010, 0.014, 0.000))
        .union(_ring(bucket_radius + 0.004, bucket_radius - 0.030, 0.010, 0.002))
    )

    hinge_band = cq.Workplane("XY").box(0.024, 0.084, 0.012).translate((hinge_axis_x + 0.008, 0.0, 0.414))
    hinge_band = hinge_band.union(cq.Workplane("XY").box(0.016, 0.016, 0.038).translate((hinge_axis_x + 0.006, -0.044, 0.425)))
    hinge_band = hinge_band.union(cq.Workplane("XY").box(0.016, 0.016, 0.038).translate((hinge_axis_x + 0.006, 0.044, 0.425)))
    hinge_band = hinge_band.union(_y_cylinder(0.0065, 0.030, (hinge_axis_x, -0.044, hinge_axis_z)))
    hinge_band = hinge_band.union(_y_cylinder(0.0065, 0.030, (hinge_axis_x, 0.044, hinge_axis_z)))

    pedal_mounts = cq.Workplane("XY").box(0.014, 0.070, 0.016).translate((body_radius + 0.001, 0.0, pedal_axis_z - 0.022))
    for y in (-0.058, 0.058):
        lug = cq.Workplane("XY").box(0.018, 0.012, 0.038).translate((body_radius + 0.006, y, pedal_axis_z))
        gusset = cq.Workplane("XY").box(0.020, 0.010, 0.022).translate((body_radius + 0.002, y, pedal_axis_z - 0.020))
        pedal_mounts = pedal_mounts.union(lug).union(gusset)

    body_shell = body_shell.union(hinge_band).union(pedal_mounts).combine()
    body.visual(_mesh(body_shell, "step_bin_shell"), material=stainless, name="shell")

    bucket = model.part("bucket")
    handle_axis_x = bucket_radius - 0.011
    handle_axis_z = bucket_height - 0.042

    bucket_shell = (
        cq.Workplane("XY")
        .circle(bucket_radius)
        .extrude(bucket_height)
        .cut(
            cq.Workplane("XY")
            .circle(bucket_radius - bucket_wall)
            .extrude(bucket_height - 0.003)
            .translate((0.0, 0.0, 0.003))
        )
        .union(_ring(bucket_radius + 0.003, bucket_radius - 0.002, 0.008, bucket_height - 0.008))
    )

    handle_mounts = cq.Workplane("XY").box(0.004, 0.034, 0.018).translate((bucket_radius - 0.001, 0.0, handle_axis_z))
    handle_mounts = handle_mounts.union(
        cq.Workplane("XY").box(0.010, 0.010, 0.020).translate((bucket_radius - 0.003, -0.021, handle_axis_z))
    )
    handle_mounts = handle_mounts.union(
        cq.Workplane("XY").box(0.010, 0.010, 0.020).translate((bucket_radius - 0.003, 0.021, handle_axis_z))
    )

    bucket.visual(_mesh(bucket_shell, "step_bin_bucket"), material=bucket_gray, name="bucket")
    bucket.visual(_mesh(handle_mounts, "step_bin_handle_mounts"), material=bucket_gray, name="handle_mounts")

    lid = model.part("lid")
    lid_radius = 0.139
    lid_center_x = -hinge_axis_x

    outer_cap = (
        cq.Workplane("XY")
        .sphere(0.200)
        .translate((lid_center_x, 0.0, -0.145))
        .intersect(cq.Workplane("XY").circle(lid_radius).extrude(0.052).translate((lid_center_x, 0.0, 0.004)))
    )
    outer_skirt = (
        cq.Workplane("XY")
        .circle(lid_radius)
        .extrude(0.022)
        .cut(cq.Workplane("XY").circle(lid_radius - 0.010).extrude(0.022))
        .translate((lid_center_x, 0.0, -0.012))
    )
    inner_cap = (
        cq.Workplane("XY")
        .sphere(0.193)
        .translate((lid_center_x, 0.0, -0.139))
        .intersect(
            cq.Workplane("XY")
            .circle(lid_radius - 0.010)
            .extrude(0.044)
            .translate((lid_center_x, 0.0, 0.008))
        )
    )
    inner_skirt = (
        cq.Workplane("XY")
        .circle(lid_radius - 0.008)
        .extrude(0.026)
        .translate((lid_center_x, 0.0, -0.012))
    )
    lid_cover = outer_cap.union(outer_skirt).cut(inner_cap.union(inner_skirt))

    lid_hinge_leaf = cq.Workplane("XY").box(0.024, 0.040, 0.008).translate((0.010, 0.0, 0.004))
    lid_hinge_leaf = lid_hinge_leaf.union(_y_cylinder(0.0056, 0.046, (0.0, 0.0, 0.0)))

    lid_cover = lid_cover.union(lid_hinge_leaf).combine()
    lid.visual(_mesh(lid_cover, "step_bin_lid_cover"), material=stainless, name="cover")

    pedal = model.part("pedal")
    pedal_bar = _y_cylinder(0.0045, 0.082, (0.0, 0.0, 0.0))
    pedal_bar = pedal_bar.union(cq.Workplane("XY").box(0.016, 0.082, 0.016).translate((0.012, 0.0, -0.008)))
    pedal_bar = pedal_bar.union(cq.Workplane("XY").box(0.048, 0.082, 0.010).translate((0.036, 0.0, -0.011)))
    pedal_bar = pedal_bar.union(cq.Workplane("XY").box(0.014, 0.082, 0.018).translate((0.061, 0.0, -0.014)))
    pedal.visual(_mesh(pedal_bar, "step_bin_pedal_bar"), material=pedal_black, name="bar")

    handle = model.part("handle")
    handle_grip = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.004)
        .lineTo(-0.008, 0.004)
        .threePointArc((-0.028, -0.006), (-0.030, -0.030))
        .lineTo(-0.030, -0.046)
        .threePointArc((-0.016, -0.060), (0.0, -0.052))
        .lineTo(0.0, -0.038)
        .threePointArc((-0.012, -0.028), (-0.012, -0.016))
        .lineTo(-0.004, -0.006)
        .close()
        .extrude(0.042)
        .translate((0.0, -0.021, 0.0))
        .combine()
    )
    handle.visual(_mesh(handle_grip, "step_bin_handle_grip"), material=handle_gray, name="grip")

    model.articulation(
        "bucket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(pedal_axis_x, 0.0, pedal_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=0.0, upper=0.55),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=handle,
        origin=Origin(xyz=(handle_axis_x, 0.0, handle_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    handle = object_model.get_part("handle")

    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_pivot = object_model.get_articulation("pedal_pivot")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.allow_overlap(
        body,
        bucket,
        elem_a="shell",
        elem_b="bucket",
        reason="The thin open-top outer shell and the removable inner bucket are modeled as independent cavity meshes, so the nested bucket fit is represented by mesh shells that the overlap QC treats conservatively.",
    )
    ctx.allow_overlap(
        bucket,
        handle,
        elem_a="bucket",
        elem_b="grip",
        reason="The small flip handle sits inside the open bucket cavity and its pivot stubs nest against the bucket wall, which the shell-mesh overlap QC treats as a solid overlap.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="shell",
        elem_b="cover",
        reason="The rear hinge band and lid sleeve are represented as interleaved mesh-backed hinge geometry inside the shell and cover visuals, so the hinge-line nest keeps a small intentional overlap.",
    )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="cover",
        elem_b="shell",
        min_overlap=0.22,
        name="lid covers the cylindrical body opening",
    )
    ctx.expect_gap(
        lid,
        handle,
        axis="z",
        positive_elem="cover",
        negative_elem="grip",
        min_gap=0.020,
        max_gap=0.120,
        name="inner bucket handle remains visible below the lid line",
    )
    shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
    bucket_aabb = ctx.part_element_world_aabb(bucket, elem="bucket")
    lid_cover_aabb = ctx.part_element_world_aabb(lid, elem="cover")
    pedal_aabb = ctx.part_element_world_aabb(pedal, elem="bar")

    ctx.check(
        "inner bucket rim stays below the outer body rim",
        bucket_aabb is not None
        and lid_cover_aabb is not None
        and bucket_aabb[1][2] > 0.395
        and bucket_aabb[1][2] < 0.420
        and lid_cover_aabb[0][2] > bucket_aabb[1][2] + 0.015,
        details=f"bucket={bucket_aabb}, lid={lid_cover_aabb}",
    )
    ctx.check(
        "lid sits close to the body rim",
        lid_cover_aabb is not None
        and lid_cover_aabb[0][2] >= 0.429
        and lid_cover_aabb[0][2] <= 0.444,
        details=f"lid={lid_cover_aabb}",
    )
    ctx.check(
        "pedal bar sits in front of the shell",
        pedal_aabb is not None
        and pedal_aabb[0][0] > 0.139
        and pedal_aabb[0][0] < 0.220,
        details=f"pedal={pedal_aabb}",
    )

    closed_lid_aabb = lid_cover_aabb
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="cover")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="bar")
    with ctx.pose({pedal_pivot: pedal_pivot.motion_limits.upper}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="bar")
    ctx.check(
        "pedal rotates downward when pressed",
        closed_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.010,
        details=f"closed={closed_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="grip")
    with ctx.pose({handle_pivot: handle_pivot.motion_limits.upper}):
        raised_handle_aabb = ctx.part_element_world_aabb(handle, elem="grip")
    ctx.check(
        "inner bucket handle lifts on its side pivots",
        closed_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.015,
        details=f"closed={closed_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

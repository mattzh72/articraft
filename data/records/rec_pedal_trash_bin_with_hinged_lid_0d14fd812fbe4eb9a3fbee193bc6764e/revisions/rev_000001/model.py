from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _ring(radius: float, inner_radius: float, height: float, z_center: float = 0.0):
    outer = cq.Workplane("XY").cylinder(height, radius)
    inner = cq.Workplane("XY").cylinder(height * 1.4, inner_radius)
    return outer.cut(inner).translate((0.0, 0.0, z_center))


def _hollow_body():
    height = 0.620
    outer_radius = 0.180
    wall = 0.018
    bottom_thickness = 0.035

    outer = cq.Workplane("XY").cylinder(height, outer_radius).translate((0.0, 0.0, height / 2.0))
    inner = (
        cq.Workplane("XY")
        .cylinder(height + 0.080, outer_radius - wall)
        .translate((0.0, 0.0, bottom_thickness + (height + 0.080) / 2.0))
    )
    shell = outer.cut(inner)
    try:
        return shell.edges("|Z").fillet(0.006)
    except Exception:
        return shell


def _domed_lid():
    dome = (
        cq.Workplane("XY")
        .circle(0.186)
        .workplane(offset=0.018)
        .circle(0.182)
        .workplane(offset=0.022)
        .circle(0.158)
        .workplane(offset=0.022)
        .circle(0.110)
        .workplane(offset=0.014)
        .circle(0.035)
        .loft(combine=True)
        .translate((0.190, 0.0, 0.000))
    )
    return dome


def _rounded_box(size, fillet):
    block = cq.Workplane("XY").box(*size)
    try:
        return block.edges("|Z").fillet(fillet)
    except Exception:
        return block


def _pivot_lug():
    block = cq.Workplane("XY").box(0.052, 0.036, 0.052)
    bore = cq.Workplane("XZ").cylinder(0.070, 0.015)
    return block.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_close_step_bin")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    lid_gray = model.material("satin_warm_gray", rgba=(0.62, 0.61, 0.56, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.034, 0.032, 1.0))
    brushed_metal = model.material("brushed_stainless", rgba=(0.70, 0.70, 0.66, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.08, 0.08, 0.075, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_hollow_body(), "body_shell", tolerance=0.0012, angular_tolerance=0.08),
        material=warm_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_ring(0.192, 0.156, 0.038, 0.624), "top_rim", tolerance=0.001),
        material=warm_white,
        name="top_rim",
    )
    body.visual(
        mesh_from_cadquery(_ring(0.192, 0.132, 0.040, 0.022), "base_ring", tolerance=0.001),
        material=warm_white,
        name="base_ring",
    )
    body.visual(
        Box((0.045, 0.245, 0.034)),
        origin=Origin(xyz=(-0.174, 0.0, 0.624)),
        material=warm_white,
        name="rear_hinge_band",
    )
    for idx, y in enumerate((-0.115, 0.115)):
        body.visual(
            Cylinder(radius=0.012, length=0.055),
            origin=Origin(xyz=(-0.190, y, 0.660), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"main_hinge_knuckle_{idx}",
        )
        body.visual(
            Box((0.034, 0.038, 0.020)),
            origin=Origin(xyz=(-0.205, y, 0.646)),
            material=warm_white,
            name=f"main_hinge_boss_{idx}",
        )

    body.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(xyz=(-0.200, 0.120, 0.565)),
        material=shadow,
        name="soft_close_damper",
    )
    body.visual(
        Box((0.018, 0.034, 0.070)),
        origin=Origin(xyz=(-0.188, 0.120, 0.590)),
        material=warm_white,
        name="damper_bracket",
    )

    body.visual(
        Cylinder(radius=0.0065, length=0.035),
        origin=Origin(xyz=(-0.190, -0.084, 0.552), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="service_hinge_knuckle_0",
    )
    body.visual(
        Box((0.033, 0.026, 0.020)),
        origin=Origin(xyz=(-0.177, -0.084, 0.542)),
        material=warm_white,
        name="service_hinge_boss_0",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.035),
        origin=Origin(xyz=(-0.190, 0.084, 0.552), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="service_hinge_knuckle_1",
    )
    body.visual(
        Box((0.033, 0.026, 0.020)),
        origin=Origin(xyz=(-0.177, 0.084, 0.542)),
        material=warm_white,
        name="service_hinge_boss_1",
    )

    body.visual(
        Box((0.050, 0.036, 0.026)),
        origin=Origin(xyz=(0.160, -0.112, 0.090)),
        material=warm_white,
        name="pedal_lug_bridge_0",
    )
    body.visual(
        mesh_from_cadquery(_pivot_lug(), "pedal_lug_0"),
        origin=Origin(xyz=(0.192, -0.112, 0.090)),
        material=warm_white,
        name="pedal_lug_0",
    )
    body.visual(
        mesh_from_cadquery(_ring(0.017, 0.0115, 0.038), "pedal_pivot_bushing_0"),
        origin=Origin(xyz=(0.206, -0.112, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="pedal_pivot_bushing_0",
    )
    body.visual(
        Box((0.050, 0.036, 0.026)),
        origin=Origin(xyz=(0.160, 0.112, 0.090)),
        material=warm_white,
        name="pedal_lug_bridge_1",
    )
    body.visual(
        mesh_from_cadquery(_pivot_lug(), "pedal_lug_1"),
        origin=Origin(xyz=(0.192, 0.112, 0.090)),
        material=warm_white,
        name="pedal_lug_1",
    )
    body.visual(
        mesh_from_cadquery(_ring(0.017, 0.0115, 0.038), "pedal_pivot_bushing_1"),
        origin=Origin(xyz=(0.206, 0.112, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="pedal_pivot_bushing_1",
    )

    main_lid = model.part("main_lid")
    main_lid.visual(
        mesh_from_cadquery(_domed_lid(), "dome_shell", tolerance=0.0012, angular_tolerance=0.07),
        material=lid_gray,
        name="dome_shell",
    )
    main_lid.visual(
        mesh_from_cadquery(_ring(0.174, 0.151, 0.012, -0.011).translate((0.190, 0.0, 0.0)), "gasket_ring"),
        material=dark_rubber,
        name="gasket_ring",
    )
    main_lid.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="hinge_knuckle",
    )
    for idx, y in enumerate((-0.042, 0.042)):
        main_lid.visual(
            Box((0.100, 0.019, 0.014)),
            origin=Origin(xyz=(0.052, y, -0.002)),
            material=lid_gray,
            name=f"hinge_strap_{idx}",
        )
    main_lid.visual(
        mesh_from_cadquery(_rounded_box((0.054, 0.130, 0.016), 0.006).translate((0.358, 0.0, 0.014)), "front_lip"),
        material=dark_rubber,
        name="front_lip",
    )

    service_cover = model.part("service_cover")
    service_cover.visual(
        mesh_from_cadquery(
            _rounded_box((0.014, 0.155, 0.115), 0.005).translate((-0.011, 0.0, -0.060)),
            "service_cover_panel",
        ),
        material=lid_gray,
        name="cover_panel",
    )
    service_cover.visual(
        Cylinder(radius=0.006, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="hinge_knuckle",
    )
    for idx, y in enumerate((-0.030, 0.030)):
        service_cover.visual(
            Box((0.012, 0.014, 0.052)),
            origin=Origin(xyz=(-0.006, y, -0.027)),
            material=lid_gray,
            name=f"cover_strap_{idx}",
        )
    service_cover.visual(
        Box((0.010, 0.085, 0.012)),
        origin=Origin(xyz=(-0.022, 0.0, -0.112)),
        material=dark_rubber,
        name="thumb_tab",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.010, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="pivot_tube",
    )
    for idx, y in enumerate((-0.055, 0.055)):
        pedal.visual(
            Box((0.104, 0.016, 0.014)),
            origin=Origin(xyz=(0.055, y, -0.016)),
            material=brushed_metal,
            name=f"pedal_arm_{idx}",
        )
    pedal.visual(
        mesh_from_cadquery(_rounded_box((0.124, 0.172, 0.025), 0.010).translate((0.103, 0.0, -0.036)), "foot_pad"),
        material=brushed_metal,
        name="foot_pad",
    )
    for idx, x in enumerate((0.066, 0.103, 0.140)):
        pedal.visual(
            Box((0.010, 0.145, 0.004)),
            origin=Origin(xyz=(x, 0.0, -0.025)),
            material=dark_rubber,
            name=f"tread_{idx}",
        )

    main_hinge = model.articulation(
        "body_to_main_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=main_lid,
        origin=Origin(xyz=(-0.190, 0.0, 0.660)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.18),
        motion_properties=MotionProperties(damping=0.45, friction=0.04),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.206, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=0.36),
        motion_properties=MotionProperties(damping=0.12, friction=0.03),
    )
    model.articulation(
        "body_to_service_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_cover,
        origin=Origin(xyz=(-0.190, 0.0, 0.552)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.3, lower=0.0, upper=1.05),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    main_lid = object_model.get_part("main_lid")
    pedal = object_model.get_part("pedal")
    service_cover = object_model.get_part("service_cover")
    main_hinge = object_model.get_articulation("body_to_main_lid")
    pedal_pivot = object_model.get_articulation("body_to_pedal")
    service_hinge = object_model.get_articulation("body_to_service_cover")

    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_lug_0",
        elem_b="pivot_tube",
        reason="The pedal pivot tube is intentionally captured through the molded front bearing lug proxy.",
    )
    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_lug_1",
        elem_b="pivot_tube",
        reason="The pedal pivot tube is intentionally captured through the molded front bearing lug proxy.",
    )
    ctx.expect_within(
        pedal,
        body,
        axes="xz",
        inner_elem="pivot_tube",
        outer_elem="pedal_lug_0",
        margin=0.0,
        name="pivot tube passes through bearing lug 0",
    )
    ctx.expect_overlap(
        body,
        pedal,
        axes="y",
        elem_a="pedal_lug_0",
        elem_b="pivot_tube",
        min_overlap=0.030,
        name="pivot tube remains engaged in lug 0",
    )
    ctx.expect_within(
        pedal,
        body,
        axes="xz",
        inner_elem="pivot_tube",
        outer_elem="pedal_lug_1",
        margin=0.0,
        name="pivot tube passes through bearing lug 1",
    )
    ctx.expect_overlap(
        body,
        pedal,
        axes="y",
        elem_a="pedal_lug_1",
        elem_b="pivot_tube",
        min_overlap=0.030,
        name="pivot tube remains engaged in lug 1",
    )

    ctx.expect_gap(
        main_lid,
        body,
        axis="z",
        positive_elem="gasket_ring",
        negative_elem="top_rim",
        min_gap=-0.001,
        max_gap=0.003,
        name="lid gasket sits just above rim",
    )
    ctx.expect_overlap(
        main_lid,
        body,
        axes="xy",
        elem_a="dome_shell",
        elem_b="top_rim",
        min_overlap=0.30,
        name="domed lid covers cylindrical rim",
    )
    ctx.expect_overlap(
        service_cover,
        body,
        axes="y",
        elem_a="cover_panel",
        elem_b="service_hinge_knuckle_0",
        min_overlap=0.010,
        name="rear service cover shares hinge span",
    )

    rest_lip = ctx.part_element_world_aabb(main_lid, elem="front_lip")
    with ctx.pose({main_hinge: 1.05}):
        open_lip = ctx.part_element_world_aabb(main_lid, elem="front_lip")
    ctx.check(
        "main lid opens upward from rear hinge",
        rest_lip is not None
        and open_lip is not None
        and open_lip[0][2] > rest_lip[0][2] + 0.16,
        details=f"rest={rest_lip}, open={open_lip}",
    )

    rest_pad = ctx.part_element_world_aabb(pedal, elem="foot_pad")
    with ctx.pose({pedal_pivot: 0.30}):
        pressed_pad = ctx.part_element_world_aabb(pedal, elem="foot_pad")
    ctx.check(
        "pedal front pad depresses downward",
        rest_pad is not None
        and pressed_pad is not None
        and pressed_pad[0][2] < rest_pad[0][2] - 0.020,
        details=f"rest={rest_pad}, pressed={pressed_pad}",
    )

    rest_cover = ctx.part_element_world_aabb(service_cover, elem="cover_panel")
    with ctx.pose({service_hinge: 0.75}):
        open_cover = ctx.part_element_world_aabb(service_cover, elem="cover_panel")
    ctx.check(
        "service cover opens outward at rear",
        rest_cover is not None
        and open_cover is not None
        and open_cover[0][0] < rest_cover[0][0] - 0.030,
        details=f"rest={rest_cover}, open={open_cover}",
    )

    return ctx.report()


object_model = build_object_model()

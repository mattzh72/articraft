from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_router")

    body_mat = model.material("slightly_textured_black_plastic", rgba=(0.018, 0.020, 0.023, 1.0))
    satin_mat = model.material("satin_dark_top_panel", rgba=(0.040, 0.044, 0.050, 1.0))
    rubber_mat = model.material("matte_rubber_black", rgba=(0.004, 0.004, 0.005, 1.0))
    smoky_mat = model.material("smoky_translucent_cover", rgba=(0.020, 0.030, 0.045, 0.62))
    window_mat = model.material("gloss_black_light_window", rgba=(0.002, 0.003, 0.006, 1.0))
    green_mat = model.material("green_status_led", rgba=(0.08, 0.95, 0.20, 1.0))
    blue_mat = model.material("blue_status_led", rgba=(0.08, 0.32, 1.0, 1.0))
    amber_mat = model.material("amber_status_led", rgba=(1.0, 0.58, 0.05, 1.0))
    metal_mat = model.material("muted_port_metal", rgba=(0.55, 0.58, 0.56, 1.0))

    width = 0.280
    depth = 0.180
    height = 0.034

    housing_shape = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(0.014)
        .translate((0.0, 0.0, height / 2.0))
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(housing_shape, "rounded_router_housing", tolerance=0.0008),
        material=body_mat,
        name="body_shell",
    )
    housing.visual(
        Box((0.225, 0.122, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, height + 0.0008)),
        material=satin_mat,
        name="top_inset_panel",
    )
    for idx, x in enumerate((-0.072, -0.048, -0.024, 0.024, 0.048, 0.072)):
        housing.visual(
            Box((0.006, 0.074, 0.0012)),
            origin=Origin(xyz=(x, 0.014, height + 0.0020)),
            material=rubber_mat,
            name=f"top_vent_{idx}",
        )

    # The light lenses live behind the articulated smoky front cover.
    housing.visual(
        Box((0.158, 0.0030, 0.012)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.0010, 0.020)),
        material=window_mat,
        name="light_window",
    )
    for idx, (x, mat) in enumerate(
        ((-0.054, green_mat), (-0.018, blue_mat), (0.018, green_mat), (0.054, amber_mat))
    ):
        housing.visual(
            Box((0.011, 0.0012, 0.0045)),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.0029, 0.020)),
            material=mat,
            name=f"indicator_{idx}",
        )

    # Rear service detail helps the flat black object read as a network router.
    housing.visual(
        Box((0.116, 0.0030, 0.014)),
        origin=Origin(xyz=(-0.014, depth / 2.0 + 0.0010, 0.018)),
        material=window_mat,
        name="rear_port_recess",
    )
    for idx, x in enumerate((-0.048, -0.020, 0.008, 0.036)):
        housing.visual(
            Box((0.018, 0.0012, 0.009)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.0028, 0.018)),
            material=metal_mat,
            name=f"ethernet_port_{idx}",
        )

    pod_size = (0.018, 0.036, 0.022)
    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        for row_name, y in (("front", -0.058), ("rear", 0.058)):
            housing.visual(
                Box(pod_size),
                origin=Origin(xyz=(sx * (width / 2.0 + pod_size[0] / 2.0), y, 0.020)),
                material=body_mat,
                name=f"{row_name}_{side_name}_pod",
            )

    # Small fixed ears on the top-front edge bracket the status-cover hinge pin.
    for idx, x in enumerate((-0.098, 0.098)):
        housing.visual(
            Box((0.012, 0.006, 0.010)),
            origin=Origin(xyz=(x, -depth / 2.0 - 0.0008, 0.028)),
            material=body_mat,
            name=f"cover_hinge_ear_{idx}",
        )

    paddle_shape = (
        cq.Workplane("YZ")
        .center(0.0, 0.0805)
        .rect(0.026, 0.105)
        .extrude(0.006, both=True)
        .edges("|X")
        .fillet(0.006)
    )

    antenna_specs = (
        ("front_left_antenna", -1.0, -0.058, (0.0, -1.0, 0.0)),
        ("rear_left_antenna", -1.0, 0.058, (0.0, -1.0, 0.0)),
        ("front_right_antenna", 1.0, -0.058, (0.0, 1.0, 0.0)),
        ("rear_right_antenna", 1.0, 0.058, (0.0, 1.0, 0.0)),
    )
    for part_name, sx, y, axis in antenna_specs:
        antenna = model.part(part_name)
        antenna.visual(
            Cylinder(radius=0.004, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_mat,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0028, length=0.031),
            origin=Origin(xyz=(0.0, 0.0, 0.0155)),
            material=rubber_mat,
            name="root_stem",
        )
        antenna.visual(
            mesh_from_cadquery(paddle_shape, f"{part_name}_paddle", tolerance=0.0007),
            material=rubber_mat,
            name="paddle",
        )
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(sx * (width / 2.0 + pod_size[0]), y, 0.028)),
            axis=axis,
            motion_limits=MotionLimits(effort=1.2, velocity=2.8, lower=0.0, upper=1.25),
        )

    cover = model.part("status_cover")
    cover.visual(
        Box((0.172, 0.0030, 0.018)),
        origin=Origin(xyz=(0.0, -0.0025, -0.009)),
        material=smoky_mat,
        name="cover_panel",
    )
    cover.visual(
        Cylinder(radius=0.0032, length=0.184),
        origin=Origin(xyz=(0.0, -0.0010, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoky_mat,
        name="hinge_barrel",
    )
    model.articulation(
        "housing_to_status_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.0060, 0.029)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cover = object_model.get_part("status_cover")
    cover_joint = object_model.get_articulation("housing_to_status_cover")

    antenna_names = (
        "front_left_antenna",
        "rear_left_antenna",
        "front_right_antenna",
        "rear_right_antenna",
    )
    ctx.check(
        "four antenna root hinges",
        all(object_model.get_articulation(f"housing_to_{name}") is not None for name in antenna_names),
        details="Each side-mounted paddle antenna should have its own revolute root hinge.",
    )

    ctx.expect_overlap(
        cover,
        housing,
        axes="xz",
        elem_a="cover_panel",
        elem_b="light_window",
        min_overlap=0.010,
        name="status cover spans the indicator window",
    )
    ctx.expect_gap(
        housing,
        cover,
        axis="y",
        positive_elem="light_window",
        negative_elem="cover_panel",
        min_gap=0.002,
        max_gap=0.006,
        name="cover sits just in front of the light lenses",
    )
    for ear_name in ("cover_hinge_ear_0", "cover_hinge_ear_1"):
        ctx.expect_contact(
            cover,
            housing,
            elem_a="hinge_barrel",
            elem_b=ear_name,
            contact_tol=0.0002,
            name=f"status hinge barrel reaches {ear_name}",
        )

    for name in antenna_names:
        pod_name = name.replace("antenna", "pod")
        antenna = object_model.get_part(name)
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=pod_name,
            contact_tol=0.0005,
            name=f"{name} barrel is seated in its side pod",
        )

    cover_rest = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({cover_joint: 1.0}):
        cover_open = ctx.part_element_world_aabb(cover, elem="cover_panel")
    ctx.check(
        "status cover opens outward from the front edge",
        cover_rest is not None
        and cover_open is not None
        and cover_open[0][1] < cover_rest[0][1] - 0.010,
        details=f"closed_aabb={cover_rest}, open_aabb={cover_open}",
    )

    for name in antenna_names:
        antenna = object_model.get_part(name)
        joint = object_model.get_articulation(f"housing_to_{name}")
        rest_aabb = ctx.part_element_world_aabb(antenna, elem="paddle")
        with ctx.pose({joint: 1.25}):
            tilted_aabb = ctx.part_element_world_aabb(antenna, elem="paddle")
        if "right" in name:
            ok = rest_aabb is not None and tilted_aabb is not None and tilted_aabb[1][0] > rest_aabb[1][0] + 0.060
        else:
            ok = rest_aabb is not None and tilted_aabb is not None and tilted_aabb[0][0] < rest_aabb[0][0] - 0.060
        ctx.check(
            f"{name} rotates outward from its side",
            ok,
            details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

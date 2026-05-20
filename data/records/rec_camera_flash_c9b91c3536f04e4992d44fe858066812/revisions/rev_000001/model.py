from __future__ import annotations

from math import pi

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


def _rounded_box(size: tuple[float, float, float], fillet: float = 0.003):
    """Small molded-plastic enclosure shape with softly radiused vertical edges."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(fillet)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_shoe_camera_flash")

    matte_black = model.material("matte_black", rgba=(0.025, 0.026, 0.028, 1.0))
    soft_black = model.material("soft_black", rgba=(0.060, 0.062, 0.066, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.14, 0.15, 0.16, 1.0))
    shoe_metal = model.material("shoe_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    diffuser = model.material("milky_diffuser", rgba=(0.92, 0.94, 0.90, 0.68))
    red_window = model.material("red_sensor_window", rgba=(0.55, 0.05, 0.04, 0.78))
    label_grey = model.material("printed_label_grey", rgba=(0.70, 0.71, 0.68, 1.0))

    # The root frame is centered on the bottom of the hot-shoe foot.
    body = model.part("body")
    body.visual(
        Box((0.030, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=shoe_metal,
        name="mounting_foot",
    )
    for index, y in enumerate((-0.016, 0.016)):
        body.visual(
            Box((0.034, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.006)),
            material=shoe_metal,
            name=f"shoe_rail_{index}",
        )
    body.visual(
        Box((0.044, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=soft_black,
        name="base_plate",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box((0.038, 0.060, 0.105), 0.004), "flash_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0665)),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        Box((0.003, 0.020, 0.014)),
        origin=Origin(xyz=(0.020, -0.014, 0.046)),
        material=red_window,
        name="focus_sensor",
    )
    body.visual(
        Box((0.003, 0.022, 0.010)),
        origin=Origin(xyz=(0.020, 0.014, 0.074)),
        material=dark_grey,
        name="status_display",
    )
    body.visual(
        Box((0.002, 0.033, 0.004)),
        origin=Origin(xyz=(0.0198, 0.0, 0.099)),
        material=label_grey,
        name="brand_label",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=dark_grey,
        name="swivel_socket",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.1265)),
        material=shoe_metal,
        name="swivel_index_ring",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_grey,
        name="swivel_disk",
    )
    neck.visual(
        Box((0.018, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=soft_black,
        name="neck_post",
    )
    neck.visual(
        Box((0.020, 0.088, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=soft_black,
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.044, 0.044)):
        neck.visual(
            Box((0.018, 0.006, 0.034)),
            origin=Origin(xyz=(0.0, y, 0.039)),
            material=soft_black,
            name=f"yoke_cheek_{index}",
        )
        neck.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.039), rpy=(pi / 2.0, 0.0, 0.0)),
            material=shoe_metal,
            name=f"hinge_bushing_{index}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shoe_metal,
        name="tilt_barrel",
    )
    head.visual(
        mesh_from_cadquery(_rounded_box((0.072, 0.078, 0.045), 0.004), "flash_head_shell"),
        origin=Origin(xyz=(0.037, 0.0, 0.016)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.066, 0.030)),
        origin=Origin(xyz=(0.075, 0.0, 0.017)),
        material=diffuser,
        name="diffuser",
    )
    head.visual(
        Box((0.003, 0.052, 0.018)),
        origin=Origin(xyz=(0.0775, 0.0, 0.017)),
        material=label_grey,
        name="reflector_panel",
    )
    head.visual(
        Box((0.010, 0.070, 0.004)),
        origin=Origin(xyz=(0.038, 0.0, 0.0402)),
        material=soft_black,
        name="bounce_card_slot",
    )

    model.articulation(
        "neck_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=-1.5708, upper=1.5708),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.30, velocity=1.8, lower=-0.35, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    neck_swivel = object_model.get_articulation("neck_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="swivel_disk",
        negative_elem="swivel_index_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="neck sits on body swivel bearing",
    )
    ctx.expect_within(
        head,
        neck,
        axes="y",
        inner_elem="tilt_barrel",
        outer_elem="yoke_bridge",
        margin=0.001,
        name="head tilt barrel is captured between yoke cheeks",
    )

    rest_diffuser_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
    with ctx.pose({neck_swivel: 0.65}):
        swivel_diffuser_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
    with ctx.pose({head_tilt: 0.80}):
        tilted_diffuser_aabb = ctx.part_element_world_aabb(head, elem="diffuser")

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "vertical swivel turns flash head sideways",
        rest_diffuser_aabb is not None
        and swivel_diffuser_aabb is not None
        and abs(_center_y(swivel_diffuser_aabb) - _center_y(rest_diffuser_aabb)) > 0.030,
        details=f"rest={rest_diffuser_aabb}, swivel={swivel_diffuser_aabb}",
    )
    ctx.check(
        "positive tilt raises front diffuser",
        rest_diffuser_aabb is not None
        and tilted_diffuser_aabb is not None
        and _center_z(tilted_diffuser_aabb) > _center_z(rest_diffuser_aabb) + 0.025,
        details=f"rest={rest_diffuser_aabb}, tilted={tilted_diffuser_aabb}",
    )

    total_aabb = ctx.part_world_aabb(head)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "small camera accessory scale",
        total_aabb is not None
        and body_aabb is not None
        and body_aabb[1][2] < 0.14
        and total_aabb[1][2] < 0.23,
        details=f"body={body_aabb}, head={total_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

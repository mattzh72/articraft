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
)


OPTICAL_AXIS = (1.0, 0.0, 0.0)
CYLINDER_TO_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_x(x: float) -> Origin:
    return Origin(xyz=(x, 0.0, 0.0), rpy=CYLINDER_TO_X.rpy)


def _annular_tube_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A centered hollow tube with its axis on local +X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _add_x_cylinder(part, radius: float, length: float, x: float, name: str, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin_x(x),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens")

    off_white = Material("warm_off_white", rgba=(0.88, 0.86, 0.78, 1.0))
    satin_black = Material("satin_black", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = Material("ribbed_black_rubber", rgba=(0.025, 0.026, 0.024, 1.0))
    dark_gray = Material("dark_gray", rgba=(0.12, 0.12, 0.115, 1.0))
    metal = Material("brushed_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    glass = Material("coated_blue_glass", rgba=(0.08, 0.20, 0.32, 0.72))
    white_mark = Material("engraved_white", rgba=(0.95, 0.94, 0.88, 1.0))

    body = model.part("lens_body")
    # Fixed optical tube, rear mount, retained shoulders, and the flared front objective.
    _add_x_cylinder(body, 0.040, 0.045, -0.235, "rear_mount", satin_black)
    _add_x_cylinder(body, 0.052, 0.012, -0.209, "mount_flange", metal)
    _add_x_cylinder(body, 0.052, 0.083, -0.162, "rear_barrel", off_white)
    _add_x_cylinder(body, 0.047, 0.128, -0.061, "zoom_neck", dark_gray)
    _add_x_cylinder(body, 0.056, 0.006, -0.1155, "zoom_rear_lip", off_white)
    _add_x_cylinder(body, 0.056, 0.006, 0.0005, "zoom_front_lip", off_white)
    _add_x_cylinder(body, 0.053, 0.066, 0.033, "collar_neck", off_white)
    _add_x_cylinder(body, 0.058, 0.004, 0.007, "collar_rear_lip", off_white)
    _add_x_cylinder(body, 0.058, 0.004, 0.059, "collar_front_lip", off_white)
    _add_x_cylinder(body, 0.060, 0.112, 0.121, "front_barrel", off_white)
    body.visual(
        mesh_from_cadquery(_annular_tube_x(0.067, 0.058, 0.050), "front_step"),
        origin=Origin(xyz=(0.199, 0.0, 0.0)),
        material=off_white,
        name="front_step",
    )
    body.visual(
        mesh_from_cadquery(_annular_tube_x(0.078, 0.064, 0.085), "hood_shell"),
        origin=Origin(xyz=(0.261, 0.0, 0.0)),
        material=off_white,
        name="hood_shell",
    )
    body.visual(
        mesh_from_cadquery(_annular_tube_x(0.064, 0.058, 0.006), "retaining_ring"),
        origin=Origin(xyz=(0.224, 0.0, 0.0)),
        material=satin_black,
        name="retaining_ring",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.004),
        origin=_origin_x(0.220),
        material=glass,
        name="front_glass",
    )
    body.visual(
        Box((0.052, 0.028, 0.004)),
        origin=Origin(xyz=(-0.158, 0.0, 0.054)),
        material=satin_black,
        name="focus_window",
    )

    zoom = model.part("zoom_barrel")
    zoom.visual(
        mesh_from_cadquery(_annular_tube_x(0.061, 0.050, 0.110), "zoom_ring"),
        material=rubber,
        name="zoom_ring",
    )
    rib_count = 20
    for idx in range(rib_count):
        theta = 2.0 * math.pi * idx / rib_count
        radius = 0.061
        zoom.visual(
            Box((0.094, 0.0045, 0.006)),
            origin=Origin(
                xyz=(0.0, -radius * math.sin(theta), radius * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=rubber,
            name=f"grip_rib_{idx:02d}",
        )
    zoom.visual(
        Box((0.030, 0.012, 0.006)),
        origin=Origin(xyz=(0.032, 0.0, 0.061)),
        material=white_mark,
        name="zoom_mark",
    )

    collar = model.part("tripod_collar")
    collar.visual(
        mesh_from_cadquery(_annular_tube_x(0.070, 0.057, 0.048), "collar_ring"),
        material=off_white,
        name="collar_ring",
    )
    collar.visual(
        Box((0.034, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.073, 0.0)),
        material=off_white,
        name="clamp_boss",
    )
    collar.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(xyz=(0.0, 0.091, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="lock_knob",
    )
    collar.visual(
        Box((0.046, 0.036, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=off_white,
        name="foot_stem",
    )
    collar.visual(
        Box((0.110, 0.065, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.128)),
        material=off_white,
        name="tripod_foot",
    )
    for idx, y in enumerate((-0.027, 0.027)):
        collar.visual(
            Box((0.096, 0.009, 0.014)),
            origin=Origin(xyz=(0.0, y, -0.144)),
            material=metal,
            name=f"dovetail_rail_{idx}",
        )

    model.articulation(
        "body_to_zoom_barrel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=zoom,
        origin=Origin(xyz=(-0.0575, 0.0, 0.0)),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_tripod_collar",
        ArticulationType.REVOLUTE,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lens_body")
    zoom = object_model.get_part("zoom_barrel")
    collar = object_model.get_part("tripod_collar")
    zoom_joint = object_model.get_articulation("body_to_zoom_barrel")
    collar_joint = object_model.get_articulation("body_to_tripod_collar")

    ctx.check(
        "zoom barrel rotates about the optical axis",
        tuple(round(v, 6) for v in zoom_joint.axis) == OPTICAL_AXIS,
        details=f"axis={zoom_joint.axis}",
    )
    ctx.check(
        "tripod collar rotates about the optical axis",
        tuple(round(v, 6) for v in collar_joint.axis) == OPTICAL_AXIS,
        details=f"axis={collar_joint.axis}",
    )
    ctx.expect_overlap(
        zoom,
        body,
        axes="x",
        elem_a="zoom_ring",
        elem_b="zoom_neck",
        min_overlap=0.090,
        name="zoom ring is retained over the fixed neck",
    )
    ctx.expect_overlap(
        collar,
        body,
        axes="x",
        elem_a="collar_ring",
        elem_b="collar_neck",
        min_overlap=0.040,
        name="tripod collar wraps the fixed collar neck",
    )
    ctx.expect_within(
        body,
        zoom,
        axes="yz",
        inner_elem="zoom_neck",
        outer_elem="zoom_ring",
        margin=0.004,
        name="fixed zoom neck sits inside the zoom barrel",
    )
    ctx.expect_within(
        body,
        collar,
        axes="yz",
        inner_elem="collar_neck",
        outer_elem="collar_ring",
        margin=0.004,
        name="fixed collar neck sits inside the rotating collar",
    )

    def _center_from_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    rest_foot = _center_from_aabb(ctx.part_element_world_aabb(collar, elem="tripod_foot"))
    with ctx.pose({collar_joint: math.pi / 2.0}):
        portrait_foot = _center_from_aabb(ctx.part_element_world_aabb(collar, elem="tripod_foot"))
    ctx.check(
        "collar carries the tripod foot into portrait orientation",
        rest_foot[2] < -0.11 and portrait_foot[1] > 0.10 and abs(portrait_foot[2]) < 0.03,
        details=f"rest={rest_foot}, portrait={portrait_foot}",
    )

    rest_mark = _center_from_aabb(ctx.part_element_world_aabb(zoom, elem="zoom_mark"))
    with ctx.pose({zoom_joint: 0.8}):
        turned_mark = _center_from_aabb(ctx.part_element_world_aabb(zoom, elem="zoom_mark"))
    ctx.check(
        "zoom rotation carries the focal length mark around the barrel",
        turned_mark[1] < rest_mark[1] - 0.035 and turned_mark[2] < rest_mark[2] - 0.015,
        details=f"rest={rest_mark}, turned={turned_mark}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

LENS_LENGTH = 0.286

IRIS_CENTER_Z = 0.040
ZOOM_CENTER_Z = 0.106
FOCUS_CENTER_Z = 0.1835

IRIS_SUPPORT_RADIUS = 0.041
ZOOM_SUPPORT_RADIUS = 0.046
FOCUS_SUPPORT_RADIUS = 0.049


def build_front_barrel_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").workplane(offset=0.226).circle(0.071).extrude(0.060)
    front_cell = cq.Workplane("XY").workplane(offset=0.226).circle(0.052).extrude(0.022)
    front_opening = cq.Workplane("XY").workplane(offset=0.248).circle(0.0555).extrude(0.038)
    return outer.cut(front_cell).cut(front_opening)


def build_geared_ring(
    *,
    inner_radius: float,
    width: float,
    wall: float,
    tooth_height: float,
    tooth_count: int,
    tooth_fill: float = 0.58,
) -> cq.Workplane:
    base_outer_radius = inner_radius + wall
    bore = cq.Workplane("XY").circle(inner_radius).extrude((width + 0.004) / 2.0, both=True)
    ring = cq.Workplane("XY").circle(base_outer_radius).extrude(width / 2.0, both=True).cut(bore)

    tooth_band_width = width * 0.82
    tooth_pitch = 2.0 * math.pi * (base_outer_radius + tooth_height * 0.5) / tooth_count
    tooth_width = tooth_pitch * tooth_fill
    tooth = (
        cq.Workplane("XY")
        .rect(tooth_height, tooth_width)
        .extrude(tooth_band_width / 2.0, both=True)
        .translate((base_outer_radius + tooth_height / 2.0, 0.0, 0.0))
    )
    for idx in range(tooth_count):
        ring = ring.union(tooth.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 360.0 * idx / tooth_count))

    return ring.cut(bore)


def add_rotating_ring(
    model: ArticulatedObject,
    *,
    name: str,
    band_name: str,
    mesh_name: str,
    center_z: float,
    inner_radius: float,
    width: float,
    wall: float,
    tooth_height: float,
    tooth_count: int,
    material: str,
) -> None:
    ring = model.part(name)
    ring.visual(
        mesh_from_cadquery(
            build_geared_ring(
                inner_radius=inner_radius,
                width=width,
                wall=wall,
                tooth_height=tooth_height,
                tooth_count=tooth_count,
            ),
            mesh_name,
        ),
        material=material,
        name=band_name,
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent="body",
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_zoom_lens")

    body_black = model.material("body_black", rgba=(0.08, 0.08, 0.09, 1.0))
    ring_black = model.material("ring_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.039, length=0.030),
        material=body_black,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        name="rear_body",
    )
    body.visual(
        Cylinder(radius=IRIS_SUPPORT_RADIUS, length=0.026),
        material=body_black,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        name="iris_support",
    )
    body.visual(
        Cylinder(radius=ZOOM_SUPPORT_RADIUS, length=0.080),
        material=body_black,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        name="zoom_body",
    )
    body.visual(
        Cylinder(radius=FOCUS_SUPPORT_RADIUS, length=0.083),
        material=body_black,
        origin=Origin(xyz=(0.0, 0.0, 0.1715)),
        name="focus_body",
    )
    body.visual(
        Cylinder(radius=0.054, length=0.017),
        material=body_black,
        origin=Origin(xyz=(0.0, 0.0, 0.2195)),
        name="front_collar",
    )
    body.visual(
        mesh_from_cadquery(build_front_barrel_shell(), "front_barrel_shell"),
        material=body_black,
        name="front_barrel",
    )

    add_rotating_ring(
        model,
        name="iris_ring",
        band_name="iris_ring_band",
        mesh_name="iris_ring",
        center_z=IRIS_CENTER_Z,
        inner_radius=IRIS_SUPPORT_RADIUS,
        width=0.020,
        wall=0.0032,
        tooth_height=0.0028,
        tooth_count=52,
        material=ring_black,
    )
    add_rotating_ring(
        model,
        name="zoom_ring",
        band_name="zoom_ring_band",
        mesh_name="zoom_ring",
        center_z=ZOOM_CENTER_Z,
        inner_radius=ZOOM_SUPPORT_RADIUS,
        width=0.044,
        wall=0.0036,
        tooth_height=0.0031,
        tooth_count=60,
        material=ring_black,
    )
    add_rotating_ring(
        model,
        name="focus_ring",
        band_name="focus_ring_band",
        mesh_name="focus_ring",
        center_z=FOCUS_CENTER_Z,
        inner_radius=FOCUS_SUPPORT_RADIUS,
        width=0.050,
        wall=0.0038,
        tooth_height=0.0035,
        tooth_count=68,
        material=ring_black,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    zoom_ring = object_model.get_part("zoom_ring")
    iris_ring = object_model.get_part("iris_ring")

    focus_joint = object_model.get_articulation("body_to_focus_ring")
    zoom_joint = object_model.get_articulation("body_to_zoom_ring")
    iris_joint = object_model.get_articulation("body_to_iris_ring")

    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="focus_body",
        elem_b="focus_ring_band",
        reason="The focus control is represented as a rotating sleeve wrapped around the focus support band.",
    )
    ctx.allow_overlap(
        body,
        zoom_ring,
        elem_a="zoom_body",
        elem_b="zoom_ring_band",
        reason="The zoom control is represented as a rotating sleeve wrapped around the zoom support band.",
    )
    ctx.allow_overlap(
        body,
        iris_ring,
        elem_a="iris_support",
        elem_b="iris_ring_band",
        reason="The iris control is represented as a rotating sleeve wrapped around the rear iris support band.",
    )

    for joint in (focus_joint, zoom_joint, iris_joint):
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name} spins about the lens axis",
            tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has unbounded rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )

    ctx.expect_overlap(
        focus_ring,
        body,
        axes="xy",
        min_overlap=0.10,
        name="focus ring stays coaxial with the barrel",
    )
    ctx.expect_overlap(
        zoom_ring,
        body,
        axes="xy",
        min_overlap=0.095,
        name="zoom ring stays coaxial with the barrel",
    )
    ctx.expect_overlap(
        iris_ring,
        body,
        axes="xy",
        min_overlap=0.085,
        name="iris ring stays coaxial with the rear barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="z",
        min_overlap=0.045,
        name="focus ring overlaps its support band",
    )
    ctx.expect_overlap(
        zoom_ring,
        body,
        axes="z",
        min_overlap=0.040,
        name="zoom ring overlaps its support band",
    )
    ctx.expect_overlap(
        iris_ring,
        body,
        axes="z",
        min_overlap=0.018,
        name="iris ring overlaps its support band",
    )
    ctx.expect_origin_gap(
        focus_ring,
        zoom_ring,
        axis="z",
        min_gap=0.06,
        name="focus ring sits ahead of zoom ring",
    )
    ctx.expect_origin_gap(
        zoom_ring,
        iris_ring,
        axis="z",
        min_gap=0.05,
        name="zoom ring sits ahead of iris ring",
    )

    with ctx.pose(
        {
            focus_joint: 0.9,
            zoom_joint: -1.1,
            iris_joint: 1.4,
        }
    ):
        ctx.expect_overlap(
            focus_ring,
            body,
            axes="xy",
            min_overlap=0.10,
            name="focus ring remains coaxial while rotated",
        )
        ctx.expect_overlap(
            zoom_ring,
            body,
            axes="xy",
            min_overlap=0.095,
            name="zoom ring remains coaxial while rotated",
        )
        ctx.expect_overlap(
            iris_ring,
            body,
            axes="xy",
            min_overlap=0.085,
            name="iris ring remains coaxial while rotated",
        )

    return ctx.report()


object_model = build_object_model()

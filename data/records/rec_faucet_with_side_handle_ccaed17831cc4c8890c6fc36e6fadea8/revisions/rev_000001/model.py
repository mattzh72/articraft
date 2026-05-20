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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, *, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rod_guide_mesh(outer_radius: float, inner_radius: float, height: float):
    return mesh_from_cadquery(
        cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height),
        "rod_guide_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_bathroom_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.71, 0.73, 0.75, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=chrome,
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=chrome,
        name="lower_body",
    )
    body.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=chrome,
        name="body_core",
    )
    body.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.010, 0.0, 0.079)),
        material=chrome,
        name="upper_body",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.025, 0.0, 0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="spout_neck",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.023, 0.0, 0.092),
                    (0.046, 0.0, 0.114),
                    (0.088, 0.0, 0.120),
                    (0.126, 0.0, 0.105),
                ],
                radius=0.0105,
                samples_per_segment=22,
                radial_segments=20,
                cap_ends=True,
            ),
            "faucet_spout",
        ),
        material=chrome,
        name="spout",
    )
    body.visual(
        Cylinder(radius=0.0135, length=0.020),
        origin=Origin(xyz=(0.136, 0.0, 0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="nozzle_shell",
    )
    body.visual(
        Cylinder(radius=0.0090, length=0.006),
        origin=Origin(xyz=(0.144, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_insert,
        name="aerator",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.005, 0.028, 0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_socket",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.024),
        origin=Origin(xyz=(-0.006, 0.023, 0.093), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="guide_bridge",
    )
    body.visual(
        _rod_guide_mesh(0.0065, 0.0038, 0.026),
        origin=Origin(xyz=(-0.014, 0.034, 0.088)),
        material=chrome,
        name="rod_guide",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0085, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_barrel",
    )
    handle.visual(
        Box((0.010, 0.042, 0.008)),
        origin=Origin(xyz=(-0.001, 0.030, 0.009)),
        material=satin_metal,
        name="lever_arm",
    )
    handle.visual(
        Box((0.012, 0.020, 0.006)),
        origin=Origin(xyz=(-0.001, 0.047, 0.016), rpy=(0.22, 0.0, 0.0)),
        material=satin_metal,
        name="grip_pad",
    )

    pull_rod = model.part("pull_rod")
    pull_rod.visual(
        Cylinder(radius=0.0030, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=satin_metal,
        name="rod_shaft",
    )
    pull_rod.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=satin_metal,
        name="rod_knob",
    )
    pull_rod.visual(
        Sphere(radius=0.0070),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=satin_metal,
        name="rod_cap",
    )

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.005, 0.036, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.20,
            upper=0.90,
        ),
    )
    model.articulation(
        "rod_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull_rod,
        origin=Origin(xyz=(-0.014, 0.034, 0.101)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.028,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    pull_rod = object_model.get_part("pull_rod")
    handle_pivot = object_model.get_articulation("handle_pivot")
    rod_slide = object_model.get_articulation("rod_slide")

    ctx.expect_contact(
        handle,
        body,
        elem_a="pivot_barrel",
        elem_b="handle_socket",
        name="handle pivots from the side socket",
    )

    ctx.expect_within(
        pull_rod,
        body,
        axes="xy",
        inner_elem="rod_shaft",
        outer_elem="rod_guide",
        margin=0.0,
        name="pull rod stays centered in the rear guide at rest",
    )
    ctx.expect_overlap(
        pull_rod,
        body,
        axes="z",
        elem_a="rod_shaft",
        elem_b="rod_guide",
        min_overlap=0.020,
        name="pull rod remains inserted through the rear guide at rest",
    )

    aerator_box = ctx.part_element_world_aabb(body, elem="aerator")
    ctx.check(
        "spout projects forward above the deck",
        aerator_box is not None and aerator_box[1][0] > 0.145 and aerator_box[0][2] > 0.093,
        details=f"aerator_box={aerator_box}",
    )

    grip_rest = ctx.part_element_world_aabb(handle, elem="grip_pad")
    rod_rest = ctx.part_world_position(pull_rod)
    handle_upper = handle_pivot.motion_limits.upper if handle_pivot.motion_limits is not None else None
    rod_upper = rod_slide.motion_limits.upper if rod_slide.motion_limits is not None else None

    with ctx.pose({handle_pivot: handle_upper, rod_slide: rod_upper}):
        grip_raised = ctx.part_element_world_aabb(handle, elem="grip_pad")
        rod_raised = ctx.part_world_position(pull_rod)

        ctx.expect_within(
            pull_rod,
            body,
            axes="xy",
            inner_elem="rod_shaft",
            outer_elem="rod_guide",
            margin=0.0,
            name="pull rod stays centered in the rear guide when lifted",
        )
        ctx.expect_overlap(
            pull_rod,
            body,
            axes="z",
            elem_a="rod_shaft",
            elem_b="rod_guide",
            min_overlap=0.020,
            name="pull rod remains retained in the rear guide when lifted",
        )
        ctx.check(
            "side lever lifts upward",
            grip_rest is not None
            and grip_raised is not None
            and grip_raised[1][2] > grip_rest[1][2] + 0.018,
            details=f"rest={grip_rest}, raised={grip_raised}",
        )
        ctx.check(
            "drain pull rod slides upward",
            rod_rest is not None
            and rod_raised is not None
            and rod_raised[2] > rod_rest[2] + 0.020,
            details=f"rest={rod_rest}, raised={rod_raised}",
        )

    return ctx.report()


object_model = build_object_model()

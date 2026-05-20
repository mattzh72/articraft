from __future__ import annotations

import math

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
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_spigot_hose_bib")

    brass = model.material("warm_brass", rgba=(0.78, 0.55, 0.25, 1.0))
    dark_brass = model.material("dark_threaded_brass", rgba=(0.50, 0.35, 0.15, 1.0))
    red = model.material("red_painted_handle", rgba=(0.70, 0.05, 0.03, 1.0))
    galvanized = model.material("galvanized_wall_bracket", rgba=(0.46, 0.48, 0.46, 1.0))
    screw_metal = model.material("dark_screw_heads", rgba=(0.10, 0.10, 0.10, 1.0))

    body = model.part("spigot_body")

    # A flat wall escutcheon/bracket that the hose bib threads through.
    body.visual(
        Box((0.030, 0.220, 0.180)),
        origin=Origin(xyz=(0.000, 0.000, 0.090)),
        material=galvanized,
        name="wall_bracket",
    )
    for y in (-0.080, 0.080):
        for z in (0.035, 0.145):
            body.visual(
                Cylinder(radius=0.010, length=0.008),
                origin=Origin(xyz=(0.018, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_metal,
                name=f"screw_head_{'p' if y > 0 else 'n'}_{'u' if z > 0.09 else 'l'}",
            )

    # Rectangular valve body protruding horizontally from the bracket.
    body.visual(
        Box((0.205, 0.074, 0.070)),
        origin=Origin(xyz=(0.118, 0.000, 0.100)),
        material=brass,
        name="rectangular_body",
    )

    # Rear threaded inlet pipe through the bracket.
    body.visual(
        Cylinder(radius=0.028, length=0.145),
        origin=Origin(xyz=(-0.045, 0.000, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="inlet_pipe",
    )
    for i, x in enumerate((-0.105, -0.090, -0.075, -0.060, -0.045, -0.030)):
        body.visual(
            Cylinder(radius=0.031, length=0.006),
            origin=Origin(xyz=(x, 0.000, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"inlet_thread_{i}",
        )

    # Front packing gland and collar on the horizontal valve spindle axis.
    body.visual(
        Cylinder(radius=0.029, length=0.040),
        origin=Origin(xyz=(0.240, 0.000, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="packing_gland",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.222, 0.000, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="gland_hex_proxy",
    )

    # Downward spout and fixed hose-thread outlet.
    body.visual(
        Cylinder(radius=0.023, length=0.095),
        origin=Origin(xyz=(0.158, 0.000, 0.020)),
        material=brass,
        name="down_spout",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.066),
        origin=Origin(xyz=(0.158, 0.000, -0.060)),
        material=dark_brass,
        name="hose_connector_core",
    )
    hose_hex = cq.Workplane("XY").polygon(6, 0.076).extrude(0.018)
    body.visual(
        mesh_from_cadquery(hose_hex, "hose_connector_hex", tolerance=0.0008),
        origin=Origin(xyz=(0.158, 0.000, -0.034)),
        material=brass,
        name="hose_connector_hex",
    )
    for i, z in enumerate((-0.087, -0.075, -0.063, -0.051)):
        body.visual(
            Cylinder(radius=0.032, length=0.004),
            origin=Origin(xyz=(0.158, 0.000, z)),
            material=brass,
            name=f"hose_thread_{i}",
        )

    handle = model.part("cross_handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.038),
        origin=Origin(xyz=(0.019, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="spindle",
    )
    handle.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.048, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(0.060, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="wide_grip",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.120),
        origin=Origin(xyz=(0.060, 0.000, 0.000)),
        material=red,
        name="short_grip",
    )
    for y in (-0.080, 0.080):
        handle.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.060, y, 0.000)),
            material=red,
            name=f"wide_grip_end_{'p' if y > 0 else 'n'}",
        )
    for z in (-0.060, 0.060):
        handle.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(0.060, 0.000, z)),
            material=red,
            name=f"short_grip_end_{'u' if z > 0 else 'd'}",
        )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.260, 0.000, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("spigot_body")
    handle = object_model.get_part("cross_handle")
    joint = object_model.get_articulation("body_to_handle")

    ctx.check(
        "cross handle uses a horizontal spindle revolute",
        joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(joint.axis) == (1.0, 0.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower <= -math.pi
        and joint.motion_limits.upper >= math.pi,
        details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="spindle",
        elem_b="packing_gland",
        contact_tol=0.001,
        name="rotating spindle seats against packing gland",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="yz",
        elem_a="spindle",
        elem_b="packing_gland",
        min_overlap=0.020,
        name="spindle is centered on valve-body bore",
    )

    rest_end = ctx.part_element_world_aabb(handle, elem="wide_grip_end_p")
    with ctx.pose({joint: math.pi / 2.0}):
        turned_end = ctx.part_element_world_aabb(handle, elem="wide_grip_end_p")
    if rest_end is not None and turned_end is not None:
        rest_center_y = (rest_end[0][1] + rest_end[1][1]) / 2.0
        rest_center_z = (rest_end[0][2] + rest_end[1][2]) / 2.0
        turned_center_y = (turned_end[0][1] + turned_end[1][1]) / 2.0
        turned_center_z = (turned_end[0][2] + turned_end[1][2]) / 2.0
        ctx.check(
            "handle end rotates in the yz grip plane",
            rest_center_y > 0.070
            and abs(rest_center_z - 0.100) < 0.010
            and abs(turned_center_y) < 0.010
            and turned_center_z > 0.170,
            details=f"rest_yz={(rest_center_y, rest_center_z)}, turned_yz={(turned_center_y, turned_center_z)}",
        )
    else:
        ctx.fail("handle end rotates in the yz grip plane", "could not measure handle-end AABBs")

    return ctx.report()


object_model = build_object_model()

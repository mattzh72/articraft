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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _output_faceplate() -> cq.Workplane:
    face = cq.Workplane("XY").circle(0.095).extrude(0.035)
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        x = 0.065 * math.cos(angle)
        y = 0.065 * math.sin(angle)
        cutter = cq.Workplane("XY").center(x, y).circle(0.008).extrude(0.055).translate((0.0, 0.0, -0.010))
        face = face.cut(cutter)
    return face


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_yaw_module")

    dark_cast = Material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    blued_steel = Material("blued_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    machined = Material("machined_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber_black = Material("black_recess", rgba=(0.015, 0.015, 0.012, 1.0))
    red_mark = Material("red_index_mark", rgba=(0.9, 0.08, 0.035, 1.0))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.28, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_cast,
        name="heavy_foot",
    )
    support.visual(
        Cylinder(radius=0.065, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=dark_cast,
        name="pedestal",
    )
    support.visual(
        mesh_from_cadquery(_annular_cylinder(0.150, 0.058, 0.140), "bearing_housing"),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=blued_steel,
        name="bearing_housing",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = 0.20 * math.cos(angle)
        y = 0.20 * math.sin(angle)
        support.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(x, y, 0.072)),
            material=rubber_black,
            name=f"anchor_recess_{i}",
        )
    for i, angle in enumerate((0.0, math.pi / 2.0)):
        support.visual(
            Box((0.210, 0.020, 0.105)),
            origin=Origin(xyz=(0.0, 0.0, 0.388), rpy=(0.0, 0.0, angle)),
            material=dark_cast,
            name=f"tower_web_{i}",
        )

    output = model.part("output_face")
    output.visual(
        Cylinder(radius=0.045, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.0425)),
        material=machined,
        name="pilot_shaft",
    )
    output.visual(
        mesh_from_cadquery(_output_faceplate(), "output_faceplate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined,
        name="faceplate",
    )
    output.visual(
        Cylinder(radius=0.045, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=machined,
        name="center_boss",
    )
    output.visual(
        Box((0.066, 0.012, 0.004)),
        origin=Origin(xyz=(0.033, 0.0, 0.059)),
        material=red_mark,
        name="index_mark",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=output,
        origin=Origin(xyz=(0.0, 0.0, 0.530)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    support = object_model.get_part("support")
    output = object_model.get_part("output_face")
    yaw = object_model.get_articulation("yaw_axis")

    ctx.check(
        "single yaw joint",
        len(object_model.articulations) == 1 and yaw.articulation_type == ArticulationType.REVOLUTE,
        details="The module should have exactly one revolute yaw articulation.",
    )
    ctx.check(
        "vertical support axis",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"yaw axis={yaw.axis}",
    )
    ctx.expect_gap(
        output,
        support,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="faceplate",
        negative_elem="bearing_housing",
        name="output face is seated on fixed bearing housing",
    )
    ctx.expect_contact(
        output,
        support,
        elem_a="faceplate",
        elem_b="bearing_housing",
        contact_tol=0.001,
        name="thrust face contacts the bearing housing",
    )

    support_aabb = ctx.part_element_world_aabb(support, elem="bearing_housing")
    output_aabb = ctx.part_element_world_aabb(output, elem="faceplate")
    if support_aabb is not None and output_aabb is not None:
        (smin, smax), (omin, omax) = support_aabb, output_aabb
        support_width = max(smax[0] - smin[0], smax[1] - smin[1])
        output_width = max(omax[0] - omin[0], omax[1] - omin[1])
        ctx.check(
            "moving member smaller than support package",
            output_width < support_width * 0.72,
            details=f"output_width={output_width:.3f}, support_width={support_width:.3f}",
        )
    else:
        ctx.fail("moving member smaller than support package", "Could not measure bearing housing and output faceplate.")

    mark_rest = ctx.part_element_world_aabb(output, elem="index_mark")
    with ctx.pose({yaw: math.pi / 2.0}):
        mark_turned = ctx.part_element_world_aabb(output, elem="index_mark")
        ctx.expect_gap(
            output,
            support,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="faceplate",
            negative_elem="bearing_housing",
            name="turned output face remains seated on housing",
        )
    if mark_rest is not None and mark_turned is not None:
        (rmin, rmax), (tmin, tmax) = mark_rest, mark_turned
        rest_center = ((rmin[0] + rmax[0]) * 0.5, (rmin[1] + rmax[1]) * 0.5)
        turned_center = ((tmin[0] + tmax[0]) * 0.5, (tmin[1] + tmax[1]) * 0.5)
        ctx.check(
            "index mark yaws with output face",
            turned_center[1] > rest_center[1] + 0.025 and abs(turned_center[0]) < 0.015,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )
    else:
        ctx.fail("index mark yaws with output face", "Could not measure the moving index mark.")

    return ctx.report()


object_model = build_object_model()

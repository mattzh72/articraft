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


PLATE_T = 0.018
PIN_R = 0.008
HOLE_R = 0.016
WASHER_R = 0.028


def _circle_solid(x: float, y: float, radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def _hole_cutter(x: float, y: float, radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(thickness * 6.0)
        .translate((0.0, 0.0, -thickness * 3.0))
    )


def _straight_plate(
    p0: tuple[float, float],
    p1: tuple[float, float],
    *,
    width: float,
    thickness: float,
    holes: tuple[tuple[float, float], ...] = (),
) -> cq.Workplane:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))

    body = (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .translate((length / 2.0, 0.0, 0.0))
    )
    body = body.union(_circle_solid(0.0, 0.0, width / 2.0, thickness))
    body = body.union(_circle_solid(length, 0.0, width / 2.0, thickness))
    body = body.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
    body = body.translate((p0[0], p0[1], 0.0))

    for hx, hy in holes:
        body = body.cut(_hole_cutter(hx, hy, HOLE_R, thickness))
    return body


def _curved_input_plate() -> cq.Workplane:
    """A thickened Bezier centerline with round pivot pads and through holes."""

    p0 = (0.0, 0.0)
    pc = (0.13, 0.19)
    p1 = (0.33, 0.085)
    width = 0.060
    half = width / 2.0
    samples = 24

    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    for i in range(samples + 1):
        t = i / samples
        x = (1.0 - t) ** 2 * p0[0] + 2.0 * (1.0 - t) * t * pc[0] + t**2 * p1[0]
        y = (1.0 - t) ** 2 * p0[1] + 2.0 * (1.0 - t) * t * pc[1] + t**2 * p1[1]
        tx = 2.0 * (1.0 - t) * (pc[0] - p0[0]) + 2.0 * t * (p1[0] - pc[0])
        ty = 2.0 * (1.0 - t) * (pc[1] - p0[1]) + 2.0 * t * (p1[1] - pc[1])
        mag = math.hypot(tx, ty)
        nx, ny = -ty / mag, tx / mag
        left.append((x + nx * half, y + ny * half))
        right.append((x - nx * half, y - ny * half))

    body = (
        cq.Workplane("XY")
        .polyline(left + list(reversed(right)))
        .close()
        .extrude(PLATE_T)
        .translate((0.0, 0.0, -PLATE_T / 2.0))
    )
    body = body.union(_circle_solid(*p0, width / 2.0, PLATE_T))
    body = body.union(_circle_solid(*p1, width / 2.0, PLATE_T))
    body = body.cut(_hole_cutter(*p0, HOLE_R, PLATE_T))
    body = body.cut(_hole_cutter(*p1, HOLE_R, PLATE_T))
    return body


def _output_plate() -> cq.Workplane:
    pivot = (0.0, 0.0)
    end = (0.42, 0.045)
    dx, dy = end[0] - pivot[0], end[1] - pivot[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length

    bar = _straight_plate(pivot, end, width=0.050, thickness=PLATE_T)
    tab_start = (end[0] - ux * 0.020, end[1] - uy * 0.020)
    tab_end = (end[0] + ux * 0.105, end[1] + uy * 0.105)
    tab_hole = (end[0] + ux * 0.060, end[1] + uy * 0.060)
    tab = _straight_plate(tab_start, tab_end, width=0.095, thickness=PLATE_T)
    body = bar.union(tab)
    body = body.cut(_hole_cutter(*pivot, HOLE_R, PLATE_T))
    body = body.cut(_hole_cutter(*tab_hole, 0.012, PLATE_T))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_lever_linkage")

    base_mat = model.material("painted_cast_base", rgba=(0.12, 0.13, 0.14, 1.0))
    input_mat = model.material("blue_steel_plate", rgba=(0.05, 0.24, 0.70, 1.0))
    coupler_mat = model.material("brushed_coupler_plate", rgba=(0.72, 0.68, 0.55, 1.0))
    output_mat = model.material("red_oxide_plate", rgba=(0.65, 0.12, 0.08, 1.0))
    pin_mat = model.material("dark_pin_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    bolt_mat = model.material("bright_worn_edges", rgba=(0.80, 0.78, 0.70, 1.0))

    base_lug = model.part("base_lug")
    base_lug.visual(
        Box((0.12, 0.12, 0.082)),
        origin=Origin(xyz=(-0.125, 0.0, 0.0)),
        material=base_mat,
        name="anchor_block",
    )
    cheek = _straight_plate((-0.135, 0.0), (0.0, 0.0), width=0.074, thickness=0.014)
    base_lug.visual(
        mesh_from_cadquery(cheek, "lower_base_cheek"),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=base_mat,
        name="lower_cheek",
    )
    base_lug.visual(
        mesh_from_cadquery(cheek, "upper_base_cheek"),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=base_mat,
        name="upper_cheek",
    )
    base_lug.visual(
        Cylinder(radius=PIN_R, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_mat,
        name="base_pin_shaft",
    )
    base_lug.visual(
        Cylinder(radius=WASHER_R, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=pin_mat,
        name="base_pin_head_0",
    )
    base_lug.visual(
        Cylinder(radius=WASHER_R, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=pin_mat,
        name="base_pin_head_1",
    )
    base_lug.visual(
        Cylinder(radius=WASHER_R, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=pin_mat,
        name="base_thrust_washer_0",
    )
    base_lug.visual(
        Cylinder(radius=WASHER_R, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=pin_mat,
        name="base_thrust_washer_1",
    )
    for i, by in enumerate((-0.038, 0.038)):
        base_lug.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(-0.155, by, 0.044)),
            material=bolt_mat,
            name=f"mount_bolt_{i}",
        )

    input_lever = model.part("input_lever")
    input_lever.visual(
        mesh_from_cadquery(_curved_input_plate(), "curved_input_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=input_mat,
        name="input_plate",
    )
    # The outboard pin is carried by the input lever and reaches up to the raised coupler.
    for zc, lname in ((0.013, "input_lower_washer"), (-0.013, "input_lower_head")):
        input_lever.visual(
            Cylinder(radius=WASHER_R, length=0.008),
            origin=Origin(xyz=(0.33, 0.085, zc)),
            material=pin_mat,
            name=lname,
        )
    input_lever.visual(
        Cylinder(radius=PIN_R, length=0.070),
        origin=Origin(xyz=(0.33, 0.085, 0.015)),
        material=pin_mat,
        name="input_pin_shaft",
    )
    input_lever.visual(
        Cylinder(radius=WASHER_R * 0.95, length=0.006),
        origin=Origin(xyz=(0.33, 0.085, 0.020)),
        material=pin_mat,
        name="input_thrust_washer",
    )
    input_lever.visual(
        Cylinder(radius=WASHER_R * 0.92, length=0.006),
        origin=Origin(xyz=(0.33, 0.085, 0.047)),
        material=pin_mat,
        name="input_pin_head",
    )

    coupler_lever = model.part("coupler_lever")
    coupler_end = (0.160, -0.075)
    coupler_lever.visual(
        mesh_from_cadquery(
            _straight_plate((0.0, 0.0), coupler_end, width=0.052, thickness=PLATE_T, holes=((0.0, 0.0), coupler_end)),
            "short_coupler_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=coupler_mat,
        name="coupler_plate",
    )
    coupler_lever.visual(
        Cylinder(radius=PIN_R, length=0.102),
        origin=Origin(xyz=(coupler_end[0], coupler_end[1], 0.0)),
        material=pin_mat,
        name="coupler_pin_shaft",
    )
    for zc, thick, lname in (
        (0.045, 0.008, "coupler_top_head"),
        (0.020, 0.008, "coupler_upper_washer"),
        (-0.020, 0.006, "coupler_thrust_washer"),
        (-0.047, 0.007, "coupler_lower_head"),
    ):
        coupler_lever.visual(
            Cylinder(radius=WASHER_R, length=thick),
            origin=Origin(xyz=(coupler_end[0], coupler_end[1], zc)),
            material=pin_mat,
            name=lname,
        )

    output_lever = model.part("output_lever")
    output_lever.visual(
        mesh_from_cadquery(_output_plate(), "long_output_plate"),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=output_mat,
        name="output_plate",
    )
    output_lever.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.482, 0.052, -0.020)),
        material=bolt_mat,
        name="tab_bushing",
    )

    model.articulation(
        "base_to_input",
        ArticulationType.REVOLUTE,
        parent=base_lug,
        child=input_lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=math.radians(-45), upper=math.radians(85)),
    )
    model.articulation(
        "input_to_coupler",
        ArticulationType.REVOLUTE,
        parent=input_lever,
        child=coupler_lever,
        origin=Origin(xyz=(0.33, 0.085, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.8, lower=math.radians(-80), upper=math.radians(80)),
    )
    model.articulation(
        "coupler_to_output",
        ArticulationType.REVOLUTE,
        parent=coupler_lever,
        child=output_lever,
        origin=Origin(xyz=(coupler_end[0], coupler_end[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.4, lower=math.radians(-95), upper=math.radians(95)),
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

    joints = (
        object_model.get_articulation("base_to_input"),
        object_model.get_articulation("input_to_coupler"),
        object_model.get_articulation("coupler_to_output"),
    )
    ctx.check(
        "three revolute linkage joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=str([j.articulation_type for j in joints]),
    )
    ctx.check(
        "joint axes are parallel to linkage normal",
        all(tuple(round(v, 6) for v in (j.axis or ())) == (0.0, 0.0, 1.0) for j in joints),
        details=str([j.axis for j in joints]),
    )
    ctx.expect_gap(
        "coupler_lever",
        "input_lever",
        axis="z",
        min_gap=0.004,
        max_gap=0.030,
        positive_elem="coupler_plate",
        negative_elem="input_plate",
        name="coupler is raised off the input plate",
    )
    ctx.expect_gap(
        "coupler_lever",
        "output_lever",
        axis="z",
        min_gap=0.040,
        max_gap=0.070,
        positive_elem="coupler_plate",
        negative_elem="output_plate",
        name="output plate is visibly offset below the coupler",
    )
    with ctx.pose({"base_to_input": math.radians(35), "input_to_coupler": math.radians(-28), "coupler_to_output": math.radians(42)}):
        ctx.expect_gap(
            "coupler_lever",
            "input_lever",
            axis="z",
            min_gap=0.004,
            positive_elem="coupler_plate",
            negative_elem="input_plate",
            name="offset stack remains clear in a working pose",
        )
        ctx.expect_overlap(
            "output_lever",
            "coupler_lever",
            axes="xy",
            min_overlap=0.010,
            elem_a="output_plate",
            elem_b="coupler_plate",
            name="output pivot plate remains under the coupler pivot",
        )

    return ctx.report()


object_model = build_object_model()

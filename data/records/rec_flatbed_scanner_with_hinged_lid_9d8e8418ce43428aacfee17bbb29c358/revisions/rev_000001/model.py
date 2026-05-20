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


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float], *, fillet: float = 0.0):
    shape = cq.Workplane("XY").box(*size).translate(center)
    if fillet > 0.0:
        shape = shape.edges("|Z").fillet(fillet)
    return shape


def _scanner_body_shape():
    body = _cq_box((0.72, 0.46, 0.160), (0.0, 0.0, 0.080), fillet=0.024)

    # Raised, continuous frame around the glass platen.
    for center, size in (
        ((0.0, -0.172, 0.166), (0.620, 0.026, 0.014)),
        ((0.0, 0.172, 0.166), (0.620, 0.026, 0.014)),
        ((-0.310, 0.0, 0.166), (0.026, 0.370, 0.014)),
        ((0.310, 0.0, 0.166), (0.026, 0.370, 0.014)),
    ):
        body = body.union(_cq_box(size, center))

    # Rear hinge pedestals are part of the scanner body casting.
    for x in (-0.230, 0.230):
        body = body.union(_cq_box((0.165, 0.046, 0.066), (x, 0.268, 0.193)))

    # A shallow front recess for the sliding calibration reference.
    body = body.union(_cq_box((0.660, 0.074, 0.006), (0.0, -0.178, 0.165)))
    # Rear bridge ties the exposed hinge pedestals back into the body shell.
    body = body.union(_cq_box((0.650, 0.058, 0.028), (0.0, 0.248, 0.170)))
    return body


def _lid_cap_shape():
    lid = _cq_box((0.760, 0.430, 0.040), (0.0, -0.235, 0.000), fillet=0.026)
    # Raised perimeter ribs make the wide lid read as a molded scanner cover.
    for center, size in (
        ((0.0, -0.435, 0.024), (0.670, 0.018, 0.010)),
        ((-0.355, -0.235, 0.024), (0.018, 0.355, 0.010)),
        ((0.355, -0.235, 0.024), (0.018, 0.355, 0.010)),
    ):
        lid = lid.union(_cq_box(size, center, fillet=0.003))
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="object_scanning_platform")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.055, 0.060, 0.066, 1.0))
    soft_black = model.material("soft_black", rgba=(0.002, 0.002, 0.003, 1.0))
    glass_blue = model.material("smoked_glass", rgba=(0.08, 0.20, 0.26, 0.58))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.66, 0.68, 1.0))
    calibration_white = model.material("calibration_white", rgba=(0.94, 0.94, 0.88, 1.0))
    calibration_black = model.material("calibration_black", rgba=(0.0, 0.0, 0.0, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.40, 0.41, 0.39, 1.0))

    body = model.part("scanner_body")
    body.visual(
        mesh_from_cadquery(_scanner_body_shape(), "scanner_body_shell", tolerance=0.001),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        Box((0.540, 0.280, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.163)),
        material=glass_blue,
        name="glass_platen",
    )
    body.visual(
        Box((0.640, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, -0.178, 0.170)),
        material=dark_graphite,
        name="calibration_track",
    )
    # Interleaved fixed hinge knuckles at the rear edge.
    for x, visual_name in ((-0.230, "rear_hinge_knuckle_0"), (0.230, "rear_hinge_knuckle_1")):
        body.visual(
            Cylinder(radius=0.018, length=0.140),
            origin=Origin(xyz=(x, 0.255, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_aluminum,
            name=visual_name,
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cap_shape(), "scanner_lid_cap", tolerance=0.001),
        material=dark_graphite,
        name="lid_cap",
    )
    lid.visual(
        Box((0.560, 0.270, 0.009)),
        origin=Origin(xyz=(0.0, -0.245, -0.0235)),
        material=rubber_gray,
        name="foam_pad",
    )
    lid.visual(
        Box((0.660, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.024, -0.010)),
        material=brushed_aluminum,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="lid_hinge_knuckle",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.130, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=soft_black,
        name="turntable_base",
    )
    turntable.visual(
        Cylinder(radius=0.112, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=brushed_aluminum,
        name="turntable_platter",
    )
    turntable.visual(
        Cylinder(radius=0.028, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=dark_graphite,
        name="centering_pad",
    )

    calibration_bar = model.part("calibration_bar")
    calibration_bar.visual(
        Box((0.080, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=soft_black,
        name="bar_carriage",
    )
    calibration_bar.visual(
        Box((0.170, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=calibration_white,
        name="reference_bar",
    )
    for i, x in enumerate((-0.060, -0.020, 0.020, 0.060)):
        calibration_bar.visual(
            Box((0.017, 0.028, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.0185)),
            material=calibration_black,
            name=f"fiducial_{i}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.255, 0.220)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.25),
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "bar_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=calibration_bar,
        origin=Origin(xyz=(-0.260, -0.178, 0.174)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=0.0, upper=0.520),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    lid = object_model.get_part("lid")
    turntable = object_model.get_part("turntable")
    calibration_bar = object_model.get_part("calibration_bar")
    lid_hinge = object_model.get_articulation("lid_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")
    bar_slide = object_model.get_articulation("bar_slide")

    ctx.check(
        "turntable is on a continuous revolute joint",
        turntable_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {turntable_spin.articulation_type}",
    )
    ctx.check(
        "calibration bar slides along the long x edge",
        bar_slide.articulation_type == ArticulationType.PRISMATIC and tuple(bar_slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={bar_slide.articulation_type}, axis={bar_slide.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, bar_slide: 0.0, turntable_spin: 0.0}):
        ctx.expect_gap(
            turntable,
            body,
            axis="z",
            positive_elem="turntable_base",
            negative_elem="glass_platen",
            max_gap=0.0015,
            max_penetration=0.0,
            name="turntable sits on the platen center",
        )
        ctx.expect_origin_distance(
            turntable,
            body,
            axes="xy",
            max_dist=0.001,
            name="turntable joint origin is at platen center",
        )
        ctx.expect_gap(
            calibration_bar,
            body,
            axis="z",
            positive_elem="bar_carriage",
            negative_elem="calibration_track",
            max_gap=0.0015,
            max_penetration=0.0002,
            name="calibration carriage rides on its edge track",
        )
        ctx.expect_within(
            calibration_bar,
            body,
            axes="xy",
            inner_elem="bar_carriage",
            outer_elem="calibration_track",
            margin=0.004,
            name="calibration carriage starts inside the long-edge track",
        )
        ctx.expect_gap(
            lid,
            turntable,
            axis="z",
            positive_elem="foam_pad",
            negative_elem="turntable_platter",
            min_gap=0.003,
            name="closed lid clears the object turntable",
        )
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.30, name="wide lid covers the scanner body")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_bar_pos = ctx.part_world_position(calibration_bar)
    with ctx.pose({bar_slide: 0.520}):
        extended_bar_pos = ctx.part_world_position(calibration_bar)
        ctx.expect_within(
            calibration_bar,
            body,
            axes="xy",
            inner_elem="bar_carriage",
            outer_elem="calibration_track",
            margin=0.004,
            name="calibration carriage remains captured at full travel",
        )
    ctx.check(
        "calibration reference translates along the platen long edge",
        rest_bar_pos is not None and extended_bar_pos is not None and extended_bar_pos[0] > rest_bar_pos[0] + 0.45,
        details=f"rest={rest_bar_pos}, extended={extended_bar_pos}",
    )

    with ctx.pose({turntable_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            turntable,
            body,
            axes="xy",
            max_dist=0.001,
            name="turntable stays centered while spinning",
        )

    return ctx.report()


object_model = build_object_model()

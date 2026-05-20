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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small CadQuery helper for cast-looking housings."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _pan_saddle_shape() -> cq.Workplane:
    """One-piece rounded saddle on top of the yaw turntable."""
    upright = _rounded_box((0.18, 0.09, 0.17), 0.014).translate((0.0, 0.0, 0.185))
    top_boss = _rounded_box((0.27, 0.11, 0.07), 0.012).translate((0.0, 0.0, 0.255))
    web = _rounded_box((0.13, 0.07, 0.13), 0.010).translate((0.0, 0.0, 0.115))
    return upright.union(top_boss).union(web)


def _tilting_fork_shape() -> cq.Workplane:
    """Connected upper fork with open front clearance and flat faces for brackets."""
    side_plate = _rounded_box((0.030, 0.160, 0.300), 0.007)
    fork = side_plate.translate((0.175, 0.0, 0.120)).union(
        side_plate.translate((-0.175, 0.0, 0.120))
    )
    rear_bridge = _rounded_box((0.390, 0.030, 0.260), 0.006).translate(
        (0.0, -0.074, 0.125)
    )
    top_cross = _rounded_box((0.390, 0.160, 0.030), 0.006).translate(
        (0.0, 0.0, 0.262)
    )
    lower_lip = _rounded_box((0.390, 0.026, 0.050), 0.005).translate(
        (0.0, 0.074, 0.010)
    )
    bore_cutter = (
        cq.Workplane("XY")
        .cylinder(0.500, 0.049)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    return fork.union(rear_bridge).union(top_cross).union(lower_lip).cut(bore_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_tilt_surveillance_head")

    cast_dark = model.material("dark_cast_aluminum", color=(0.16, 0.17, 0.18, 1.0))
    cast_mid = model.material("graphite_cast_housing", color=(0.27, 0.29, 0.30, 1.0))
    bracket_mat = model.material("flat_black_bracket", color=(0.03, 0.035, 0.04, 1.0))
    bolt_mat = model.material("black_fasteners", color=(0.005, 0.005, 0.006, 1.0))
    seam_mat = model.material("rubber_bearing_seam", color=(0.01, 0.012, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.205, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_dark,
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=cast_dark,
        name="fixed_pedestal",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=cast_mid,
        name="bearing_ring",
    )
    for idx, (x, y) in enumerate(((0.155, 0.155), (-0.155, 0.155), (-0.155, -0.155), (0.155, -0.155))):
        base.visual(
            Cylinder(radius=0.038, length=0.030),
            origin=Origin(xyz=(x, y, 0.015)),
            material=cast_dark,
            name=f"mounting_ear_{idx}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.032)),
            material=bolt_mat,
            name=f"anchor_bolt_{idx}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.136, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=cast_mid,
        name="rotating_plate",
    )
    turntable.visual(
        Cylinder(radius=0.108, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=seam_mat,
        name="pan_seam",
    )
    turntable.visual(
        Cylinder(radius=0.092, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=cast_mid,
        name="rotary_neck",
    )
    turntable.visual(
        mesh_from_cadquery(_pan_saddle_shape(), "pan_saddle_housing", tolerance=0.001),
        material=cast_mid,
        name="saddle_housing",
    )
    turntable.visual(
        Cylinder(radius=0.038, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.270), rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_dark,
        name="tilt_bearing_barrel",
    )
    turntable.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(-0.195, 0.0, 0.270), rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_dark,
        name="tilt_washer_0",
    )
    turntable.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.195, 0.0, 0.270), rpy=(0.0, pi / 2.0, 0.0)),
        material=cast_dark,
        name="tilt_washer_1",
    )
    turntable.visual(
        Box((0.185, 0.012, 0.052)),
        origin=Origin(xyz=(0.0, 0.046, 0.158)),
        material=bracket_mat,
        name="front_cable_clamp",
    )

    fork = model.part("fork")
    fork.visual(
        mesh_from_cadquery(_tilting_fork_shape(), "tilting_upper_fork", tolerance=0.001),
        material=cast_mid,
        name="fork_cast",
    )
    fork.visual(
        Box((0.285, 0.014, 0.185)),
        origin=Origin(xyz=(0.0, 0.086, 0.165)),
        material=bracket_mat,
        name="equipment_plate",
    )
    fork.visual(
        Box((0.250, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.105, 0.258)),
        material=bracket_mat,
        name="top_equipment_rail",
    )
    for idx, x in enumerate((-0.100, 0.100)):
        fork.visual(
            Box((0.050, 0.030, 0.006)),
            origin=Origin(xyz=(x, 0.116, 0.262)),
            material=bolt_mat,
            name=f"rail_slot_{idx}",
        )
    for idx, (x, z) in enumerate(((-0.092, 0.112), (0.092, 0.112), (-0.092, 0.218), (0.092, 0.218))):
        fork.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(x, 0.096, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_mat,
            name=f"face_screw_{idx}",
        )
    model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.9, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    fork = object_model.get_part("fork")
    pan = object_model.get_articulation("pan_axis")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem="rotating_plate",
        negative_elem="bearing_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotary plate seats on fixed bearing ring",
    )
    ctx.expect_overlap(
        fork,
        turntable,
        axes="xyz",
        elem_a="fork_cast",
        elem_b="tilt_bearing_barrel",
        min_overlap=0.030,
        name="fork bore surrounds the tilt bearing barrel",
    )
    ctx.allow_overlap(
        fork,
        turntable,
        elem_a="fork_cast",
        elem_b="tilt_washer_0",
        reason="The thin thrust washer is intentionally seated slightly into the fork cheek to show a captured tilt bearing.",
    )
    ctx.allow_overlap(
        fork,
        turntable,
        elem_a="fork_cast",
        elem_b="tilt_washer_1",
        reason="The thin thrust washer is intentionally seated slightly into the fork cheek to show a captured tilt bearing.",
    )
    ctx.expect_gap(
        fork,
        turntable,
        axis="x",
        positive_elem="fork_cast",
        negative_elem="tilt_washer_0",
        max_penetration=0.004,
        name="negative thrust washer only lightly seats into fork",
    )
    ctx.expect_gap(
        turntable,
        fork,
        axis="x",
        positive_elem="tilt_washer_1",
        negative_elem="fork_cast",
        max_penetration=0.004,
        name="positive thrust washer only lightly seats into fork",
    )

    def aabb_center_z(aabb):
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def aabb_center_xy(aabb):
        return (0.5 * (aabb[0][0] + aabb[1][0]), 0.5 * (aabb[0][1] + aabb[1][1]))

    rest_plate = ctx.part_element_world_aabb(fork, elem="equipment_plate")
    with ctx.pose({tilt: 0.55}):
        tilted_plate = ctx.part_element_world_aabb(fork, elem="equipment_plate")
    ctx.check(
        "positive tilt raises the equipment bracket",
        rest_plate is not None
        and tilted_plate is not None
        and aabb_center_z(tilted_plate) > aabb_center_z(rest_plate) + 0.020,
        details=f"rest={rest_plate}, tilted={tilted_plate}",
    )

    with ctx.pose({pan: pi / 2.0}):
        panned_plate = ctx.part_element_world_aabb(fork, elem="equipment_plate")
    rest_xy = aabb_center_xy(rest_plate) if rest_plate is not None else None
    panned_xy = aabb_center_xy(panned_plate) if panned_plate is not None else None
    ctx.check(
        "pan axis rotates the upper head around the base",
        rest_xy is not None
        and panned_xy is not None
        and abs(panned_xy[0] - rest_xy[0]) > 0.055,
        details=f"rest_xy={rest_xy}, panned_xy={panned_xy}",
    )

    return ctx.report()


object_model = build_object_model()

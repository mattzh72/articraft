from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.24
BASE_THICKNESS = 0.018
RING_INNER_RADIUS = 0.095
RING_OUTER_RADIUS = 0.145
RING_HEIGHT = 0.055
RING_TOP_Z = BASE_THICKNESS + RING_HEIGHT
TRAY_GAP = 0.0
TRAY_RADIUS = 0.325
TRAY_FLOOR_THICKNESS = 0.014
TRAY_RIM_HEIGHT = 0.054
TRAY_RIM_WIDTH = 0.026


def annular_cylinder(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_start: float,
) -> cq.Workplane:
    """A hollow cylindrical ring with its bottom at z_start."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_start))
    )


def make_tray_shape() -> cq.Workplane:
    """One connected shallow plastic turntable tray, authored in the shelf frame."""
    bearing_pad = annular_cylinder(
        outer_radius=RING_OUTER_RADIUS,
        inner_radius=RING_INNER_RADIUS,
        height=0.008,
        z_start=TRAY_GAP,
    )
    floor = (
        cq.Workplane("XY")
        .circle(TRAY_RADIUS - TRAY_RIM_WIDTH)
        .extrude(TRAY_FLOOR_THICKNESS)
        .translate((0.0, 0.0, TRAY_GAP + 0.006))
    )
    outer_rim = annular_cylinder(
        outer_radius=TRAY_RADIUS,
        inner_radius=TRAY_RADIUS - TRAY_RIM_WIDTH,
        height=TRAY_RIM_HEIGHT,
        z_start=TRAY_GAP + 0.006,
    )
    small_inner_bead = annular_cylinder(
        outer_radius=0.073,
        inner_radius=0.048,
        height=0.010,
        z_start=TRAY_GAP + TRAY_FLOOR_THICKNESS + 0.005,
    )
    return bearing_pad.union(floor).union(outer_rim).union(small_inner_bead)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_pantry_lazy_susan")

    matte_white = model.material("matte_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    warm_gray = model.material("warm_gray_base", rgba=(0.58, 0.56, 0.52, 1.0))
    dark_bearing = model.material("dark_bearing_ring", rgba=(0.08, 0.08, 0.075, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.72, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=warm_gray,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(
            annular_cylinder(
                outer_radius=RING_OUTER_RADIUS,
                inner_radius=RING_INNER_RADIUS,
                height=RING_HEIGHT + 0.001,
                z_start=BASE_THICKNESS - 0.001,
            ),
            "support_ring",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        material=dark_bearing,
        name="support_ring",
    )
    base.visual(
        Cylinder(radius=0.034, length=RING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + RING_HEIGHT / 2.0)),
        material=brushed_metal,
        name="center_boss",
    )

    shelf = model.part("shelf")
    shelf.visual(
        mesh_from_cadquery(
            make_tray_shape(),
            "rotating_tray",
            tolerance=0.0009,
            angular_tolerance=0.08,
        ),
        material=matte_white,
        name="tray",
    )

    model.articulation(
        "base_to_shelf",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.0, RING_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shelf = object_model.get_part("shelf")
    spin = object_model.get_articulation("base_to_shelf")

    ctx.check(
        "shelf uses continuous center rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        shelf,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="tray",
        negative_elem="support_ring",
        name="tray sits on support ring race",
    )
    ctx.expect_overlap(
        shelf,
        base,
        axes="xy",
        min_overlap=0.12,
        elem_a="tray",
        elem_b="support_ring",
        name="tray centered over support ring",
    )
    ctx.expect_overlap(
        shelf,
        base,
        axes="xy",
        min_overlap=0.20,
        elem_a="tray",
        elem_b="base_plate",
        name="broad tray sits over stable base plate",
    )

    rest_position = ctx.part_world_position(shelf)
    with ctx.pose({spin: math.pi / 2.0}):
        rotated_position = ctx.part_world_position(shelf)
        ctx.expect_gap(
            shelf,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="tray",
            negative_elem="support_ring",
            name="rotated tray remains seated on support ring",
        )
    ctx.check(
        "rotation stays on the central vertical axis",
        rest_position is not None
        and rotated_position is not None
        and abs(rest_position[0] - rotated_position[0]) < 1e-6
        and abs(rest_position[1] - rotated_position[1]) < 1e-6,
        details=f"rest={rest_position}, rotated={rotated_position}",
    )

    return ctx.report()


object_model = build_object_model()

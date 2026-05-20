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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _tray_shell():
    deck = cq.Workplane("XY").circle(0.320).extrude(0.026)
    raised_lip = _annular_cylinder(0.335, 0.295, 0.020).translate((0.0, 0.0, 0.026))
    return deck.union(raised_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_tabletop_lazy_susan")

    bamboo = model.material("warm_bamboo", rgba=(0.72, 0.49, 0.27, 1.0))
    dark_wood = model.material("dark_endgrain", rgba=(0.38, 0.21, 0.10, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    brass = model.material("small_brass_inlay", rgba=(0.88, 0.66, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.245, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_wood,
        name="base_disc",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.110, 0.064, 0.009), "lower_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=steel,
        name="lower_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=steel,
        name="bearing_spindle",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.235, 0.182, 0.003), "rubber_foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=rubber,
        name="rubber_foot_ring",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shell(), "tray_shell", tolerance=0.0008, angular_tolerance=0.05),
        material=bamboo,
        name="tray_shell",
    )
    tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.112, 0.064, 0.009), "upper_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="upper_bearing_race",
    )
    tray.visual(
        Cylinder(radius=0.050, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, -0.0065)),
        material=steel,
        name="upper_hub",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.0015),
        origin=Origin(xyz=(0.180, 0.0, 0.02625)),
        material=brass,
        name="brass_marker",
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    turntable = object_model.get_articulation("base_to_tray")

    ctx.check(
        "tray rotates on a continuous vertical axis",
        turntable.articulation_type == ArticulationType.CONTINUOUS and tuple(turntable.axis) == (0.0, 0.0, 1.0),
        details=f"type={turntable.articulation_type}, axis={turntable.axis}",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_shell",
        negative_elem="base_disc",
        min_gap=0.025,
        max_gap=0.045,
        name="visible shadow gap between tray and base discs",
    )
    ctx.expect_within(
        base,
        tray,
        axes="xy",
        inner_elem="base_disc",
        outer_elem="tray_shell",
        margin=0.0,
        name="smaller base sits within the round tray footprint",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="upper_bearing_race",
        negative_elem="bearing_spindle",
        min_gap=0.003,
        max_gap=0.006,
        name="bearing races clear without intersecting",
    )
    ctx.expect_contact(
        tray,
        base,
        elem_a="upper_hub",
        elem_b="bearing_spindle",
        contact_tol=0.0005,
        name="central hub rests on the bearing spindle",
    )

    rest_marker_aabb = ctx.part_element_world_aabb(tray, elem="brass_marker")
    with ctx.pose({turntable: math.pi / 2.0}):
        rotated_marker_aabb = ctx.part_element_world_aabb(tray, elem="brass_marker")

    def _center_x_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_marker_xy = _center_x_y(rest_marker_aabb)
    rotated_marker_xy = _center_x_y(rotated_marker_aabb)
    ctx.check(
        "off-center tray marker follows the rotating platform",
        rest_marker_xy is not None
        and rotated_marker_xy is not None
        and rest_marker_xy[0] > 0.15
        and abs(rest_marker_xy[1]) < 0.02
        and abs(rotated_marker_xy[0]) < 0.02
        and rotated_marker_xy[1] > 0.15,
        details=f"rest={rest_marker_xy}, rotated={rotated_marker_xy}",
    )

    return ctx.report()


object_model = build_object_model()

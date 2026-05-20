from __future__ import annotations

import math

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
import cadquery as cq


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A vertical ring with a real central opening, authored in meters."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_tabletop_lazy_susan")

    warm_wood = Material("warm_oiled_wood", rgba=(0.72, 0.44, 0.20, 1.0))
    darker_wood = Material("darker_end_grain", rgba=(0.42, 0.23, 0.10, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_shadow = Material("dark_shadow_gap", rgba=(0.025, 0.022, 0.018, 1.0))
    brass = Material("brass_inlay", rgba=(0.95, 0.70, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=warm_wood,
        name="base_body",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0355)),
        material=brushed_steel,
        name="lower_race",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=brushed_steel,
        name="center_pin",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.04575)),
        material=dark_shadow,
        name="bearing_seam",
    )

    upper_tray = model.part("upper_tray")
    # Child part frame is the spin axis through the bearing stack.  Local Z=0
    # is deliberately below the wooden tray body so the empty side gap reads.
    upper_tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.238, 0.205, 0.009), "edge_skirt"),
        material=darker_wood,
        name="edge_skirt",
    )
    upper_tray.visual(
        Cylinder(radius=0.240, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=warm_wood,
        name="tray_plate",
    )
    upper_tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.252, 0.222, 0.026), "raised_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=warm_wood,
        name="raised_rim",
    )
    upper_tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.058, 0.022, 0.014), "upper_race"),
        origin=Origin(xyz=(0.0, 0.0, -0.0065)),
        material=brushed_steel,
        name="upper_race",
    )
    upper_tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.045, 0.022, 0.006), "upper_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=brushed_steel,
        name="upper_flange",
    )
    upper_tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.182, 0.176, 0.0012), "outer_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=darker_wood,
        name="outer_groove",
    )
    upper_tray.visual(
        mesh_from_cadquery(_annular_cylinder(0.112, 0.106, 0.0012), "inner_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=darker_wood,
        name="inner_groove",
    )
    upper_tray.visual(
        Cylinder(radius=0.014, length=0.002),
        origin=Origin(xyz=(0.145, 0.0, 0.028)),
        material=brass,
        name="orientation_dot",
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=upper_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_tray = object_model.get_part("upper_tray")
    spin = object_model.get_articulation("base_to_tray")

    ctx.expect_gap(
        upper_tray,
        base,
        axis="z",
        positive_elem="edge_skirt",
        negative_elem="base_body",
        min_gap=0.020,
        max_gap=0.035,
        name="visible shadow gap between discs",
    )
    ctx.expect_within(
        base,
        upper_tray,
        axes="xy",
        inner_elem="base_body",
        outer_elem="tray_plate",
        margin=0.0,
        name="base is smaller than tray footprint",
    )
    ctx.expect_overlap(
        upper_tray,
        base,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.050,
        name="bearing races share central axis",
    )
    ctx.expect_contact(
        upper_tray,
        base,
        elem_a="upper_race",
        elem_b="bearing_seam",
        contact_tol=0.0002,
        name="bearing stack provides central support",
    )

    rest_aabb = ctx.part_element_world_aabb(upper_tray, elem="orientation_dot")
    with ctx.pose({spin: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(upper_tray, elem="orientation_dot")

    if rest_aabb is not None and rotated_aabb is not None:
        rest_center = (
            (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0,
            (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0,
        )
        rotated_center = (
            (rotated_aabb[0][0] + rotated_aabb[1][0]) / 2.0,
            (rotated_aabb[0][1] + rotated_aabb[1][1]) / 2.0,
        )
    else:
        rest_center = rotated_center = None

    ctx.check(
        "upper tray rotates continuously about center",
        rest_center is not None
        and rotated_center is not None
        and rest_center[0] > 0.13
        and abs(rest_center[1]) < 0.02
        and rotated_center[1] > 0.13
        and abs(rotated_center[0]) < 0.02,
        details=f"rest={rest_center}, rotated={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()

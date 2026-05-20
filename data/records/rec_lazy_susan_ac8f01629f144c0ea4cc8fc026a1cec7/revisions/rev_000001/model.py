from __future__ import annotations

import cadquery as cq
import math

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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    """CadQuery ring centered on the local origin, aligned with +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet_lazy_susan")

    cabinet_mat = model.material("painted_cabinet", rgba=(0.78, 0.66, 0.50, 1.0))
    shelf_mat = model.material("maple_shelves", rgba=(0.86, 0.66, 0.38, 1.0))
    edge_mat = model.material("sealed_edge", rgba=(0.56, 0.34, 0.16, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))

    bearing_ring = mesh_from_cadquery(
        _annular_cylinder(0.080, 0.036, 0.035),
        "bearing_ring",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    shelf_lip = mesh_from_cadquery(
        _annular_cylinder(0.365, 0.335, 0.045),
        "raised_shelf_lip",
        tolerance=0.001,
        angular_tolerance=0.08,
    )

    base = model.part("base")
    base.visual(
        Box((1.00, 1.00, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cabinet_mat,
        name="base_plate",
    )
    base.visual(
        Box((1.00, 0.035, 0.84)),
        origin=Origin(xyz=(0.0, -0.500, 0.440)),
        material=cabinet_mat,
        name="rear_wall",
    )
    base.visual(
        Box((0.035, 1.00, 0.84)),
        origin=Origin(xyz=(-0.500, 0.0, 0.440)),
        material=cabinet_mat,
        name="side_wall",
    )
    base.visual(
        Box((1.00, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, -0.500, 0.860)),
        material=cabinet_mat,
        name="rear_top_rail",
    )
    base.visual(
        Box((0.055, 1.00, 0.040)),
        origin=Origin(xyz=(-0.500, 0.0, 0.860)),
        material=cabinet_mat,
        name="side_top_rail",
    )
    base.visual(
        Box((0.50, 0.040, 0.035)),
        origin=Origin(xyz=(-0.250, 0.0, 0.8575)),
        material=cabinet_mat,
        name="top_crossarm_x",
    )
    base.visual(
        Box((0.040, 0.50, 0.035)),
        origin=Origin(xyz=(0.0, -0.250, 0.8575)),
        material=cabinet_mat,
        name="top_crossarm_y",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.8575)),
        material=metal_mat,
        name="top_retainer",
    )
    base.visual(
        bearing_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=metal_mat,
        name="lower_bearing",
    )
    base.visual(
        bearing_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.8275)),
        material=metal_mat,
        name="upper_bearing",
    )

    shelf_assembly = model.part("shelf_assembly")
    shelf_assembly.visual(
        Cylinder(radius=0.025, length=0.800),
        origin=Origin(xyz=(0.0, 0.0, 0.445)),
        material=metal_mat,
        name="center_shaft",
    )
    shelf_assembly.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=metal_mat,
        name="lower_pivot",
    )
    shelf_assembly.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=metal_mat,
        name="top_pivot",
    )

    shelf_assembly.visual(
        Cylinder(radius=0.360, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=shelf_mat,
        name="lower_shelf",
    )
    shelf_assembly.visual(
        Box((0.035, 0.180, 0.060)),
        origin=Origin(xyz=(0.345, 0.0, 0.325)),
        material=edge_mat,
        name="lower_front_stop",
    )
    shelf_assembly.visual(
        Cylinder(radius=0.360, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=shelf_mat,
        name="upper_shelf",
    )
    shelf_assembly.visual(
        Box((0.035, 0.180, 0.060)),
        origin=Origin(xyz=(0.345, 0.0, 0.660)),
        material=edge_mat,
        name="upper_front_stop",
    )

    for z, suffix in ((0.275, "lower"), (0.610, "upper")):
        shelf_assembly.visual(
            shelf_lip,
            origin=Origin(xyz=(0.0, 0.0, z + 0.040)),
            material=edge_mat,
            name=f"{suffix}_rim",
        )
        shelf_assembly.visual(
            Cylinder(radius=0.075, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=metal_mat,
            name=f"{suffix}_hub",
        )
        shelf_assembly.visual(
            Box((0.330, 0.026, 0.020)),
            origin=Origin(xyz=(0.165, 0.0, z - 0.030)),
            material=metal_mat,
            name=f"{suffix}_brace_x",
        )
        shelf_assembly.visual(
            Box((0.330, 0.026, 0.020)),
            origin=Origin(xyz=(-0.165, 0.0, z - 0.030)),
            material=metal_mat,
            name=f"{suffix}_brace_negx",
        )
        shelf_assembly.visual(
            Box((0.330, 0.026, 0.020)),
            origin=Origin(xyz=(0.0, 0.165, z - 0.030), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=metal_mat,
            name=f"{suffix}_brace_y",
        )
        shelf_assembly.visual(
            Box((0.330, 0.026, 0.020)),
            origin=Origin(xyz=(0.0, -0.165, z - 0.030), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=metal_mat,
            name=f"{suffix}_brace_negy",
        )

    model.articulation(
        "base_to_shelves",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shelf_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shelves = object_model.get_part("shelf_assembly")
    joint = object_model.get_articulation("base_to_shelves")

    ctx.expect_contact(
        shelves,
        base,
        elem_a="lower_pivot",
        elem_b="base_plate",
        contact_tol=0.0005,
        name="lower pivot bears on base plate",
    )
    ctx.expect_contact(
        shelves,
        base,
        elem_a="top_pivot",
        elem_b="top_retainer",
        contact_tol=0.0005,
        name="top pivot reaches top retainer",
    )
    ctx.expect_within(
        shelves,
        base,
        axes="xy",
        inner_elem="lower_shelf",
        outer_elem="base_plate",
        margin=0.0,
        name="lower round shelf fits within cabinet footprint",
    )
    ctx.expect_within(
        shelves,
        base,
        axes="xy",
        inner_elem="upper_shelf",
        outer_elem="base_plate",
        margin=0.0,
        name="upper round shelf fits within cabinet footprint",
    )

    rest_guard = ctx.part_element_world_aabb(shelves, elem="lower_front_stop")
    with ctx.pose({joint: math.pi / 2.0}):
        rotated_guard = ctx.part_element_world_aabb(shelves, elem="lower_front_stop")
        ctx.expect_gap(
            shelves,
            base,
            axis="x",
            min_gap=0.02,
            positive_elem="lower_shelf",
            negative_elem="side_wall",
            name="rotated shelf clears side wall",
        )
        ctx.expect_gap(
            shelves,
            base,
            axis="y",
            min_gap=0.02,
            positive_elem="lower_shelf",
            negative_elem="rear_wall",
            name="rotated shelf clears rear wall",
        )
    if rest_guard is not None and rotated_guard is not None:
        rest_center = (
            (rest_guard[0][0] + rest_guard[1][0]) / 2.0,
            (rest_guard[0][1] + rest_guard[1][1]) / 2.0,
        )
        rotated_center = (
            (rotated_guard[0][0] + rotated_guard[1][0]) / 2.0,
            (rotated_guard[0][1] + rotated_guard[1][1]) / 2.0,
        )
    else:
        rest_center = None
        rotated_center = None
    ctx.check(
        "shelf assembly rotates about vertical shaft",
        rest_center is not None
        and rotated_center is not None
        and rest_center[0] > 0.30
        and abs(rest_center[1]) < 0.10
        and abs(rotated_center[0]) < 0.10
        and rotated_center[1] > 0.30,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()

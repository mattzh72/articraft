from __future__ import annotations

from math import pi

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


def _branch_body(name: str, *, length: float, side: float) -> object:
    """One connected outrigger branch: hollow pivot hub plus a radial arm."""
    hub_outer = 0.047
    hub_inner = 0.021
    thickness = 0.045
    arm_width = 0.038
    arm_thickness = 0.026
    overlap = 0.008

    hub = (
        cq.Workplane("XY")
        .circle(hub_outer)
        .circle(hub_inner)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    arm = cq.Workplane("XY").box(length, arm_width, arm_thickness).translate(
        (side * (hub_outer + length / 2.0 - overlap), 0.0, 0.0)
    )
    rounded_tip = (
        cq.Workplane("XY")
        .circle(arm_width * 0.58)
        .extrude(arm_thickness)
        .translate((side * (hub_outer + length - overlap), 0.0, -arm_thickness / 2.0))
    )
    top_rib = cq.Workplane("XY").box(length * 0.62, arm_width * 0.28, 0.010).translate(
        (side * (hub_outer + length * 0.45), 0.0, arm_thickness / 2.0 + 0.005)
    )
    return hub.union(arm).union(rounded_tip).union(top_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_outrigger_rotary_head")

    cast_iron = Material("dark_cast_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    collar_paint = Material("charcoal_collars", rgba=(0.13, 0.14, 0.15, 1.0))
    bolt_metal = Material("blackened_bolts", rgba=(0.02, 0.02, 0.018, 1.0))
    long_paint = Material("safety_yellow_branch", rgba=(0.95, 0.68, 0.08, 1.0))
    short_paint = Material("blue_branch", rgba=(0.08, 0.24, 0.74, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.18, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="heavy_foot",
    )
    stand.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=cast_iron,
        name="foot_boss",
    )
    stand.visual(
        Cylinder(radius=0.025, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=brushed_steel,
        name="short_mast",
    )

    # Lower collar block and bearing support for the longer branch.
    lower_pivot = (0.115, 0.0, 0.350)
    upper_pivot = (-0.105, 0.0, 0.500)
    branch_h = 0.045
    washer_h = 0.012
    bearing_gap = 0.0

    def add_collar(level: str, pivot: tuple[float, float, float], connector_x: float) -> None:
        px, py, pz = pivot
        washer_top = pz - branch_h / 2.0 - bearing_gap
        washer_center_z = washer_top - washer_h / 2.0
        connector_top = washer_center_z - washer_h / 2.0
        connector_center_z = connector_top - 0.020

        stand.visual(
            Box((0.086, 0.076, 0.068)),
            origin=Origin(xyz=(0.0, 0.0, connector_center_z)),
            material=collar_paint,
            name=f"{level}_collar_block",
        )
        stand.visual(
            Box((abs(px) + 0.060, 0.054, 0.040)),
            origin=Origin(xyz=(connector_x, py, connector_center_z)),
            material=collar_paint,
            name=f"{level}_support_lug",
        )
        if level == "lower":
            stand.visual(
                Cylinder(radius=0.052, length=washer_h),
                origin=Origin(xyz=(px, py, washer_center_z)),
                material=brushed_steel,
                name="lower_washer",
            )
        else:
            stand.visual(
                Cylinder(radius=0.052, length=washer_h),
                origin=Origin(xyz=(px, py, washer_center_z)),
                material=brushed_steel,
                name="upper_washer",
            )
        stand.visual(
            Cylinder(radius=0.014, length=branch_h + washer_h + 0.030),
            origin=Origin(xyz=(px, py, pz + 0.002)),
            material=brushed_steel,
            name=f"{level}_pivot_pin",
        )
        for y in (-0.043, 0.043):
            stand.visual(
                Cylinder(radius=0.009, length=0.010),
                origin=Origin(
                    xyz=(0.0, y, connector_center_z + 0.012),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=bolt_metal,
                name=f"{level}_clamp_bolt_{'front' if y < 0 else 'rear'}",
            )

    add_collar("lower", lower_pivot, connector_x=0.050)
    add_collar("upper", upper_pivot, connector_x=-0.045)

    long_branch = model.part("long_branch")
    long_branch.visual(
        mesh_from_cadquery(_branch_body("long_branch_body", length=0.410, side=1.0), "long_branch_body"),
        material=long_paint,
        name="long_branch_body",
    )

    short_branch = model.part("short_branch")
    short_branch.visual(
        mesh_from_cadquery(
            _branch_body("short_branch_body", length=0.250, side=-1.0),
            "short_branch_body",
        ),
        material=short_paint,
        name="short_branch_body",
    )

    model.articulation(
        "lower_branch_axis",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=long_branch,
        origin=Origin(xyz=lower_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "upper_branch_axis",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=short_branch,
        origin=Origin(xyz=upper_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-1.45, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    long_branch = object_model.get_part("long_branch")
    short_branch = object_model.get_part("short_branch")
    lower_axis = object_model.get_articulation("lower_branch_axis")
    upper_axis = object_model.get_articulation("upper_branch_axis")

    ctx.check(
        "two uncoupled revolute branch axes",
        len(object_model.articulations) == 2
        and lower_axis.articulation_type == ArticulationType.REVOLUTE
        and upper_axis.articulation_type == ArticulationType.REVOLUTE
        and lower_axis.mimic is None
        and upper_axis.mimic is None,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "branch axes are separate vertical supports",
        tuple(lower_axis.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_axis.axis) == (0.0, 0.0, 1.0)
        and abs(lower_axis.origin.xyz[0] - upper_axis.origin.xyz[0]) > 0.15
        and abs(lower_axis.origin.xyz[2] - upper_axis.origin.xyz[2]) > 0.10,
        details=f"lower={lower_axis.origin}, upper={upper_axis.origin}",
    )

    ctx.expect_contact(
        long_branch,
        stand,
        elem_a="long_branch_body",
        elem_b="lower_washer",
        name="long branch hub is seated on lower collar washer",
    )
    ctx.expect_overlap(
        long_branch,
        stand,
        axes="xy",
        elem_a="long_branch_body",
        elem_b="lower_washer",
        min_overlap=0.050,
        name="long branch hub footprint covers lower washer",
    )
    ctx.expect_contact(
        short_branch,
        stand,
        elem_a="short_branch_body",
        elem_b="upper_washer",
        name="short branch hub is seated on upper collar washer",
    )
    ctx.expect_overlap(
        short_branch,
        stand,
        axes="xy",
        elem_a="short_branch_body",
        elem_b="upper_washer",
        min_overlap=0.050,
        name="short branch hub footprint covers upper washer",
    )

    def aabb_center_y(part) -> float | None:
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        return (bounds[0][1] + bounds[1][1]) / 2.0

    def aabb_dx(part) -> float | None:
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        return bounds[1][0] - bounds[0][0]

    long_dx = aabb_dx(long_branch)
    short_dx = aabb_dx(short_branch)
    ctx.check(
        "one branch is visibly longer than the other",
        long_dx is not None and short_dx is not None and long_dx > short_dx + 0.12,
        details=f"long_dx={long_dx}, short_dx={short_dx}",
    )

    rest_long_y = aabb_center_y(long_branch)
    rest_short_y = aabb_center_y(short_branch)
    with ctx.pose({lower_axis: 0.85}):
        moved_long_y = aabb_center_y(long_branch)
        held_short_y = aabb_center_y(short_branch)
    ctx.check(
        "lower branch rotates without driving upper branch",
        rest_long_y is not None
        and moved_long_y is not None
        and rest_short_y is not None
        and held_short_y is not None
        and moved_long_y > rest_long_y + 0.12
        and abs(held_short_y - rest_short_y) < 0.002,
        details=(
            f"rest_long_y={rest_long_y}, moved_long_y={moved_long_y}, "
            f"rest_short_y={rest_short_y}, held_short_y={held_short_y}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

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


def _radial_xy(radius: float, yaw: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(yaw), radius * math.sin(yaw), z)


def _arm_backbone(
    *,
    arm_length: float,
    hub_outer: float,
    hub_inner: float,
    hub_height: float,
    bar_width: float,
    bar_height: float,
) -> cq.Workplane:
    """A hollow pivot sleeve welded to a straight tooling arm."""

    ring = (
        cq.Workplane("XY")
        .circle(hub_outer)
        .circle(hub_inner)
        .extrude(hub_height)
        .translate((0.0, 0.0, -hub_height / 2.0))
    )
    bar_start = hub_outer - 0.004
    bar = (
        cq.Workplane("XY")
        .box(arm_length, bar_width, bar_height)
        .translate((bar_start + arm_length / 2.0, 0.0, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.040, bar_width * 1.45, bar_height * 1.35)
        .translate((hub_outer + 0.012, 0.0, 0.0))
    )
    return ring.union(bar).union(neck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_tooling_tree")

    cast_iron = Material("cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    column_metal = Material("brushed_column", rgba=(0.46, 0.47, 0.46, 1.0))
    clamp_paint = Material("clamp_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    pin_steel = Material("bearing_pin_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    arm_paint = Material("tool_arm_blue", rgba=(0.05, 0.18, 0.36, 1.0))
    plate_yellow = Material("faceplate_yellow", rgba=(0.92, 0.65, 0.12, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.015, 0.015, 0.012, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.260, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_iron,
        name="foot_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=cast_iron,
        name="base_boss",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.850),
        origin=Origin(xyz=(0.0, 0.0, 0.495)),
        material=column_metal,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.047, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.933)),
        material=column_metal,
        name="top_cap",
    )

    block_heights = (0.300, 0.550, 0.795)
    block_yaws = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    pivot_radius = 0.185
    lug_outer_radius = 0.1525

    for idx, (height, yaw) in enumerate(zip(block_heights, block_yaws)):
        pedestal.visual(
            Box((0.125, 0.105, 0.070)),
            origin=Origin(xyz=(0.0, 0.0, height), rpy=(0.0, 0.0, yaw)),
            material=clamp_paint,
            name=f"clamp_{idx}",
        )
        pedestal.visual(
            Box((0.095, 0.080, 0.070)),
            origin=Origin(xyz=_radial_xy(0.105, yaw, height), rpy=(0.0, 0.0, yaw)),
            material=clamp_paint,
            name=f"lug_{idx}",
        )
        for plate_name, z_offset in (("lower_fork", -0.034), ("upper_fork", 0.034)):
            pedestal.visual(
                Box((0.075, 0.075, 0.010)),
                origin=Origin(
                    xyz=_radial_xy(0.185, yaw, height + z_offset),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=clamp_paint,
                name=f"{plate_name}_{idx}",
            )
        pedestal.visual(
            Cylinder(radius=0.010, length=0.092),
            origin=Origin(xyz=_radial_xy(pivot_radius, yaw, height)),
            material=pin_steel,
            name=f"pin_{idx}",
        )
        for side in (-1.0, 1.0):
            local_r = 0.008
            tangential = 0.046 * side
            x = 0.100 * math.cos(yaw) - tangential * math.sin(yaw)
            y = 0.100 * math.sin(yaw) + tangential * math.cos(yaw)
            pedestal.visual(
                Cylinder(radius=local_r, length=0.012),
                origin=Origin(
                    xyz=(x, y, height + 0.018),
                    rpy=(math.pi / 2.0, 0.0, yaw),
                ),
                material=black_oxide,
                name=f"clamp_screw_{idx}_{int(side > 0)}",
            )

    arm_length = 0.300
    hub_outer = 0.024
    hub_inner = 0.013
    # The sleeve touches the fixed fork plates at its top and bottom faces, so
    # the rotary arm reads as physically carried by the clamp block rather than
    # floating around a clearance-only bearing.
    hub_height = 0.058
    bar_width = 0.028
    bar_height = 0.024
    face_thickness = 0.025
    face_center_x = hub_outer - 0.004 + arm_length + face_thickness / 2.0 - 0.002

    for idx, (height, yaw) in enumerate(zip(block_heights, block_yaws)):
        arm = model.part(f"arm_{idx}")
        arm.visual(
            mesh_from_cadquery(
                _arm_backbone(
                    arm_length=arm_length,
                    hub_outer=hub_outer,
                    hub_inner=hub_inner,
                    hub_height=hub_height,
                    bar_width=bar_width,
                    bar_height=bar_height,
                ),
                f"arm_backbone_{idx}",
                tolerance=0.0006,
                angular_tolerance=0.08,
            ),
            material=arm_paint,
            name=f"backbone_{idx}",
        )
        arm.visual(
            Box((face_thickness, 0.105, 0.075)),
            origin=Origin(xyz=(face_center_x, 0.0, 0.0)),
            material=plate_yellow,
            name=f"faceplate_{idx}",
        )
        for bolt_idx, (y, z) in enumerate(
            ((-0.031, -0.023), (-0.031, 0.023), (0.031, -0.023), (0.031, 0.023))
        ):
            arm.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(
                    xyz=(face_center_x + face_thickness / 2.0 + 0.002, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_oxide,
                name=f"face_bolt_{idx}_{bolt_idx}",
            )
        model.articulation(
            f"branch_{idx}_pivot",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=arm,
            origin=Origin(xyz=_radial_xy(pivot_radius, yaw, height), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-1.15, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    arms = [object_model.get_part(f"arm_{idx}") for idx in range(3)]
    pivots = [object_model.get_articulation(f"branch_{idx}_pivot") for idx in range(3)]

    ctx.check(
        "three independent revolute branch joints",
        all(p.joint_type == ArticulationType.REVOLUTE and p.mimic is None for p in pivots),
        details=[(p.name, p.joint_type, p.mimic) for p in pivots],
    )

    for idx, (arm, pivot) in enumerate(zip(arms, pivots)):
        limits = pivot.motion_limits
        ctx.check(
            f"branch {idx} has realistic swing stops",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and -1.3 < limits.lower < -0.8
            and 0.8 < limits.upper < 1.3,
            details=f"limits={limits}",
        )
        ctx.expect_within(
            pedestal,
            arm,
            axes="xy",
            inner_elem=f"pin_{idx}",
            outer_elem=f"backbone_{idx}",
            margin=0.002,
            name=f"branch {idx} bearing pin is captured by hub",
        )
        ctx.expect_overlap(
            arm,
            pedestal,
            axes="z",
            elem_a=f"backbone_{idx}",
            elem_b=f"pin_{idx}",
            min_overlap=0.035,
            name=f"branch {idx} hub overlaps pin height",
        )

    rest_aabb_0 = ctx.part_element_world_aabb(arms[0], elem="faceplate_0")
    rest_aabb_1 = ctx.part_element_world_aabb(arms[1], elem="faceplate_1")
    with ctx.pose({pivots[0]: 0.75}):
        moved_aabb_0 = ctx.part_element_world_aabb(arms[0], elem="faceplate_0")
        still_aabb_1 = ctx.part_element_world_aabb(arms[1], elem="faceplate_1")

    def _center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    ctx.check(
        "branch 0 pivots without dragging branch 1",
        rest_aabb_0 is not None
        and moved_aabb_0 is not None
        and rest_aabb_1 is not None
        and still_aabb_1 is not None
        and _center_y(moved_aabb_0) is not None
        and _center_y(rest_aabb_0) is not None
        and _center_y(moved_aabb_0) > _center_y(rest_aabb_0) + 0.15
        and abs(_center_y(still_aabb_1) - _center_y(rest_aabb_1)) < 1e-6,
        details=f"rest0={rest_aabb_0}, moved0={moved_aabb_0}, rest1={rest_aabb_1}, still1={still_aabb_1}",
    )

    return ctx.report()


object_model = build_object_model()

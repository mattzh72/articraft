from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HOUSING_D = 0.170
HOUSING_W = 0.460
HOUSING_H = 0.480
SHELL_WALL = 0.024
OPENING_W = HOUSING_W - 2.0 * SHELL_WALL
OPENING_H = HOUSING_H - 2.0 * SHELL_WALL
OUTER_CORNER = 0.028
INNER_CORNER = 0.018

PAN_X = 0.100
ROTOR_X = 0.10592
FRONT_GUARD_X = 0.156
REAR_GUARD_X = 0.016
CONTROL_X = 0.132
CONTROL_Z = -0.178
CONTROL_BOSS_LEN = 0.012
CONTROL_JOINT_Y = HOUSING_W / 2.0


def _ring_on_x_plane(outer_radius: float, inner_radius: float, thickness: float, center_x: float) -> cq.Workplane:
    ring = cq.Workplane("YZ").circle(outer_radius).circle(inner_radius).extrude(thickness)
    return ring.translate((center_x - thickness / 2.0, 0.0, 0.0))


def _disk_on_x_plane(radius: float, thickness: float, center_x: float) -> cq.Workplane:
    disk = cq.Workplane("YZ").circle(radius).extrude(thickness)
    return disk.translate((center_x - thickness / 2.0, 0.0, 0.0))


def _rounded_ring(
    depth: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    outer_radius: float,
    inner_radius: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(depth, outer_y, outer_z).edges("|X").fillet(outer_radius)
    inner = cq.Workplane("XY").box(depth + 0.004, inner_y, inner_z).edges("|X").fillet(inner_radius)
    return outer.cut(inner)


def _build_bracket_shape() -> cq.Workplane:
    wall_plate = cq.Workplane("XY").box(0.010, 0.150, 0.280).edges("|X").fillet(0.012).translate((0.005, 0.0, 0.0))
    arm_block = cq.Workplane("XY").box(0.078, 0.056, 0.084).edges("|X").fillet(0.010).translate((0.049, 0.0, 0.0))
    upper_rib = (
        cq.Workplane("XY")
        .box(0.060, 0.050, 0.018)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 34.0)
        .translate((0.037, 0.0, 0.060))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.060, 0.050, 0.018)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -34.0)
        .translate((0.037, 0.0, -0.060))
    )
    pan_head = _disk_on_x_plane(radius=0.031, thickness=0.014, center_x=PAN_X - 0.007)
    return wall_plate.union(arm_block).union(upper_rib).union(lower_rib).union(pan_head)


def _build_housing_body_shape() -> cq.Workplane:
    shell_band = _rounded_ring(
        HOUSING_D,
        HOUSING_W,
        HOUSING_H,
        OPENING_W,
        OPENING_H,
        OUTER_CORNER,
        INNER_CORNER,
    ).translate((HOUSING_D / 2.0, 0.0, 0.0))

    front_bezel = _rounded_ring(
        0.010,
        HOUSING_W,
        HOUSING_H,
        HOUSING_W - 0.050,
        HOUSING_H - 0.050,
        OUTER_CORNER,
        0.020,
    ).translate((HOUSING_D - 0.005, 0.0, 0.0))

    rear_bezel = _rounded_ring(
        0.008,
        HOUSING_W,
        HOUSING_H,
        HOUSING_W - 0.068,
        HOUSING_H - 0.068,
        OUTER_CORNER,
        0.016,
    ).translate((0.004, 0.0, 0.0))

    rear_guard = _ring_on_x_plane(0.190, 0.184, 0.004, REAR_GUARD_X)
    rear_guard = rear_guard.union(_ring_on_x_plane(0.108, 0.104, 0.004, REAR_GUARD_X))
    rear_guard = rear_guard.union(cq.Workplane("XY").box(0.004, OPENING_W + 0.004, 0.006).translate((REAR_GUARD_X, 0.0, 0.0)))
    rear_guard = rear_guard.union(cq.Workplane("XY").box(0.004, 0.006, OPENING_H + 0.004).translate((REAR_GUARD_X, 0.0, 0.0)))
    rear_guard = rear_guard.union(
        cq.Workplane("XY")
        .box(0.004, 0.290, 0.005)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 45.0)
        .translate((REAR_GUARD_X, 0.0, 0.0))
    )
    rear_guard = rear_guard.union(
        cq.Workplane("XY")
        .box(0.004, 0.290, 0.005)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -45.0)
        .translate((REAR_GUARD_X, 0.0, 0.0))
    )

    rear_can = cq.Workplane("YZ").circle(0.050).extrude(0.052).translate((0.022, 0.0, 0.0))
    front_bell = cq.Workplane("YZ").circle(0.037).extrude(0.016).translate((0.074, 0.0, 0.0))
    axle_sleeve = cq.Workplane("YZ").circle(0.007).extrude(0.008).translate((0.090, 0.0, 0.0))
    support_y = cq.Workplane("XY").box(0.010, OPENING_W + 0.010, 0.010).translate((0.054, 0.0, 0.0))
    support_z = cq.Workplane("XY").box(0.010, 0.010, OPENING_H + 0.010).translate((0.054, 0.0, 0.0))
    support_d1 = (
        cq.Workplane("XY")
        .box(0.008, 0.320, 0.008)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 38.0)
        .translate((0.048, 0.0, 0.0))
    )
    support_d2 = (
        cq.Workplane("XY")
        .box(0.008, 0.320, 0.008)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -38.0)
        .translate((0.048, 0.0, 0.0))
    )
    rear_pad = _disk_on_x_plane(0.036, 0.010, 0.005)
    rear_neck = cq.Workplane("YZ").circle(0.022).extrude(0.012).translate((0.010, 0.0, 0.0))
    control_boss = cq.Workplane("XZ").circle(0.024).extrude(CONTROL_BOSS_LEN).translate((CONTROL_X, HOUSING_W / 2.0, CONTROL_Z))

    return (
        shell_band.union(front_bezel)
        .union(rear_bezel)
        .union(rear_guard)
        .union(rear_can)
        .union(front_bell)
        .union(axle_sleeve)
        .union(support_y)
        .union(support_z)
        .union(support_d1)
        .union(support_d2)
        .union(rear_pad)
        .union(rear_neck)
        .union(control_boss)
    )


def _build_front_guard_shape() -> cq.Workplane:
    front_guard = _ring_on_x_plane(0.194, 0.188, 0.004, FRONT_GUARD_X)
    front_guard = front_guard.union(_ring_on_x_plane(0.146, 0.142, 0.004, FRONT_GUARD_X))
    front_guard = front_guard.union(_ring_on_x_plane(0.098, 0.094, 0.004, FRONT_GUARD_X))
    front_guard = front_guard.union(_ring_on_x_plane(0.042, 0.030, 0.006, FRONT_GUARD_X))
    front_guard = front_guard.union(cq.Workplane("XY").box(0.004, OPENING_W + 0.010, 0.006).translate((FRONT_GUARD_X, 0.0, 0.0)))
    front_guard = front_guard.union(cq.Workplane("XY").box(0.004, 0.006, OPENING_H + 0.010).translate((FRONT_GUARD_X, 0.0, 0.0)))
    for angle in (30.0, -30.0, 60.0, -60.0):
        front_guard = front_guard.union(
            cq.Workplane("XY")
            .box(0.004, 0.308, 0.005)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
            .translate((FRONT_GUARD_X, 0.0, 0.0))
        )
    return front_guard


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_bracket_box_fan")

    bracket_finish = model.material("bracket_finish", rgba=(0.28, 0.30, 0.32, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.86, 0.87, 0.83, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.20, 0.22, 0.24, 0.92))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        mesh_from_cadquery(_build_bracket_shape(), "bracket"),
        material=bracket_finish,
        name="bracket_shell",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_body_shape(), "housing_body"),
        material=housing_finish,
        name="housing_body",
    )
    housing.visual(
        mesh_from_cadquery(_build_front_guard_shape(), "front_guard"),
        material=housing_finish,
        name="front_guard",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.155,
                0.041,
                5,
                thickness=0.024,
                blade_pitch_deg=30.0,
                blade_sweep_deg=22.0,
                blade=FanRotorBlade(shape="broad", camber=0.16, tip_pitch_deg=16.0),
                hub=FanRotorHub(style="spinner", bore_diameter=0.010),
            ),
            "blade",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_finish,
        name="blade",
    )
    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_shaft",
    )
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.016,
                body_style="tapered",
                top_diameter=0.030,
                base_diameter=0.036,
                edge_radius=0.0015,
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_cap",
    )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=housing,
        origin=Origin(xyz=(PAN_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(ROTOR_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=35.0),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector_knob,
        origin=Origin(xyz=(CONTROL_X, CONTROL_JOINT_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    selector_knob = object_model.get_part("selector_knob")

    pan_joint = object_model.get_articulation("bracket_to_housing")
    blade_joint = object_model.get_articulation("housing_to_blade")
    knob_joint = object_model.get_articulation("housing_to_selector_knob")

    ctx.allow_overlap(
        housing,
        blade,
        elem_a="housing_body",
        elem_b="blade",
        reason="The rotating hub is intentionally simplified as nesting over the motor nose instead of modeling a separate hollow bearing fit.",
    )

    ctx.expect_origin_gap(
        blade,
        housing,
        axis="x",
        min_gap=0.10,
        max_gap=0.13,
        name="blade axle sits forward inside housing",
    )
    ctx.expect_origin_distance(
        blade,
        housing,
        axes="yz",
        max_dist=0.001,
        name="blade stays centered in housing",
    )
    ctx.expect_gap(
        housing,
        blade,
        axis="x",
        positive_elem="front_guard",
        negative_elem="blade",
        min_gap=0.020,
        max_gap=0.040,
        name="blade clears front guard",
    )
    ctx.expect_origin_gap(
        selector_knob,
        housing,
        axis="y",
        min_gap=0.22,
        name="selector knob sits on housing side",
    )
    ctx.expect_origin_gap(
        housing,
        selector_knob,
        axis="z",
        min_gap=0.14,
        max_gap=0.22,
        name="selector knob sits beneath grille",
    )

    ctx.check(
        "pan joint is revolute",
        pan_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pan_joint.articulation_type}",
    )
    ctx.check(
        "blade joint is continuous",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type}",
    )
    ctx.check(
        "selector knob joint is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) / 2.0 for i in range(3))

    rest_center = aabb_center(ctx.part_world_aabb(housing))
    turned_center = None
    upper = pan_joint.motion_limits.upper if pan_joint.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({pan_joint: upper}):
            turned_center = aabb_center(ctx.part_world_aabb(housing))
    ctx.check(
        "pan swings housing sideways",
        rest_center is not None and turned_center is not None and turned_center[1] > rest_center[1] + 0.05,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()

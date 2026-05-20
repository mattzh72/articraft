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


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center_xyz
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y + length / 2.0, z))
    )


def _make_bayonet_collar() -> cq.Workplane:
    ring = _ring(0.048, 0.0395, 0.022)
    shoulder = cq.Workplane("XY", origin=(0.0, 0.0, 0.003)).circle(0.055).circle(0.0405).extrude(0.006)
    return ring.union(shoulder)


def _make_cup_shell() -> cq.Workplane:
    shell_outer = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.020))
        .circle(0.043)
        .workplane(offset=0.090)
        .circle(0.047)
        .loft(combine=True)
    )
    shell_inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.024))
        .circle(0.0375)
        .workplane(offset=0.086)
        .circle(0.041)
        .loft(combine=True)
    )
    cup = shell_outer.cut(shell_inner)

    lug_radius = 0.0375
    for angle_deg in (0.0, 120.0, 240.0):
        angle_rad = math.radians(angle_deg)
        lug = (
            cq.Workplane("XY")
            .box(0.014, 0.008, 0.004)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            .translate((lug_radius * math.cos(angle_rad), lug_radius * math.sin(angle_rad), 0.020))
        )
        cup = cup.union(lug)

    support_z = 0.113
    for y_center in (-0.026, 0.026):
        arm = cq.Workplane("XY").box(0.012, 0.014, 0.010).translate((-0.050, y_center, support_z))
        cup = cup.union(arm)

    cup = cup.union(_y_cylinder(0.0045, 0.016, (-0.050, -0.026, 0.117)))
    cup = cup.union(_y_cylinder(0.0045, 0.016, (-0.050, 0.026, 0.117)))
    return cup


def _make_cup_rim() -> cq.Workplane:
    return _ring(0.050, 0.041, 0.002)


def _make_lid() -> cq.Workplane:
    lid_panel = (
        cq.Workplane("XY")
        .circle(0.047)
        .extrude(0.006)
        .translate((0.050, 0.0, -0.005))
    )
    front_tab = cq.Workplane("XY").box(0.014, 0.020, 0.006).translate((0.094, 0.0, -0.001))
    hinge_barrel = _y_cylinder(0.0045, 0.030, (0.000, 0.000, 0.000))
    return lid_panel.union(front_tab).union(hinge_barrel)


def _make_blade() -> cq.Workplane:
    hub = cq.Workplane("XY", origin=(0.0, 0.0, -0.004)).circle(0.006).extrude(0.010)
    shaft = cq.Workplane("XY", origin=(0.0, 0.0, -0.010)).circle(0.0025).extrude(0.006)

    lower_blade = (
        cq.Workplane("XY")
        .box(0.056, 0.010, 0.0014)
        .translate((0.0, 0.0, 0.001))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 14.0)
    )
    upper_blade = (
        cq.Workplane("XY")
        .box(0.010, 0.046, 0.0014)
        .translate((0.0, 0.0, 0.004))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -16.0)
    )
    return hub.union(shaft).union(lower_blade).union(upper_blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_homogenizer_blender")

    model.material("housing_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("housing_trim", rgba=(0.34, 0.36, 0.40, 1.0))
    model.material("metal", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("cup_clear", rgba=(0.76, 0.87, 0.96, 0.42))
    model.material("lid_clear", rgba=(0.84, 0.90, 0.95, 0.48))
    model.material("seal_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("blade_metal", rgba=(0.79, 0.81, 0.84, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.205, 0.155, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="housing_trim",
        name="base_plinth",
    )
    housing.visual(
        Box((0.188, 0.142, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="housing_dark",
        name="motor_housing",
    )
    housing.visual(
        Box((0.082, 0.018, 0.024)),
        origin=Origin(xyz=(0.034, -0.050, 0.102)),
        material="housing_trim",
        name="support_0",
    )
    housing.visual(
        Box((0.082, 0.018, 0.024)),
        origin=Origin(xyz=(0.034, 0.050, 0.102)),
        material="housing_trim",
        name="support_1",
    )
    housing.visual(
        Box((0.052, 0.092, 0.020)),
        origin=Origin(xyz=(-0.024, 0.0, 0.104)),
        material="housing_trim",
        name="support_bridge",
    )
    housing.visual(
        mesh_from_cadquery(_make_bayonet_collar(), "bayonet_collar"),
        origin=Origin(xyz=(0.048, 0.0, 0.096)),
        material="metal",
        name="bayonet_collar",
    )

    cup = model.part("cup")
    cup.visual(
        mesh_from_cadquery(_make_cup_shell(), "cup_shell"),
        material="cup_clear",
        name="cup_shell",
    )
    cup.visual(
        mesh_from_cadquery(_make_cup_rim(), "cup_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material="seal_dark",
        name="cup_rim",
    )
    cup.visual(
        Cylinder(radius=0.037, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="seal_dark",
        name="cup_skirt",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid(), "lid_shell"),
        material="lid_clear",
        name="lid_shell",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_make_blade(), "blade_rotor"),
        material="blade_metal",
        name="blade_rotor",
    )

    model.articulation(
        "housing_to_cup",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cup,
        origin=Origin(xyz=(0.048, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.radians(25.0),
            upper=math.radians(25.0),
            effort=2.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "cup_to_lid",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=lid,
        origin=Origin(xyz=(-0.050, 0.0, 0.117)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(112.0),
            effort=1.5,
            velocity=2.0,
        ),
    )
    model.articulation(
        "cup_to_blade",
        ArticulationType.CONTINUOUS,
        parent=cup,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cup = object_model.get_part("cup")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")

    cup_twist = object_model.get_articulation("housing_to_cup")
    lid_hinge = object_model.get_articulation("cup_to_lid")
    blade_spin = object_model.get_articulation("cup_to_blade")

    ctx.expect_within(
        cup,
        housing,
        axes="xy",
        inner_elem="cup_skirt",
        outer_elem="bayonet_collar",
        margin=0.003,
        name="cup skirt stays centered in the bayonet collar",
    )
    ctx.expect_overlap(
        cup,
        housing,
        axes="z",
        elem_a="cup_skirt",
        elem_b="bayonet_collar",
        min_overlap=0.018,
        name="cup skirt remains inserted in the bayonet collar",
    )
    ctx.expect_gap(
        lid,
        cup,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="cup_rim",
        min_gap=0.0,
        max_gap=0.004,
        name="closed lid seats just above the cup rim",
    )
    ctx.expect_overlap(
        lid,
        cup,
        axes="xy",
        elem_a="lid_shell",
        elem_b="cup_rim",
        min_overlap=0.075,
        name="closed lid covers the chamber opening",
    )
    ctx.expect_within(
        blade,
        cup,
        axes="xy",
        inner_elem="blade_rotor",
        outer_elem="cup_shell",
        margin=0.010,
        name="blade stays within the chamber wall footprint",
    )
    ctx.expect_gap(
        lid,
        blade,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="blade_rotor",
        min_gap=0.065,
        name="blade stays well below the closed lid",
    )

    with ctx.pose({cup_twist: math.radians(20.0)}):
        ctx.expect_within(
            cup,
            housing,
            axes="xy",
            inner_elem="cup_skirt",
            outer_elem="bayonet_collar",
            margin=0.003,
            name="twisted cup stays centered in the collar",
        )
        ctx.expect_overlap(
            cup,
            housing,
            axes="z",
            elem_a="cup_skirt",
            elem_b="bayonet_collar",
            min_overlap=0.018,
            name="twisted cup remains retained in the collar",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: math.radians(100.0)}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid panel swings upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and ((open_lid_aabb[0][2] + open_lid_aabb[1][2]) / 2.0)
        > ((closed_lid_aabb[0][2] + closed_lid_aabb[1][2]) / 2.0) + 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({blade_spin: math.pi / 3.0}):
        ctx.expect_within(
            blade,
            cup,
            axes="xy",
            inner_elem="blade_rotor",
            outer_elem="cup_shell",
            margin=0.010,
            name="spun blade remains within the chamber wall footprint",
        )

    return ctx.report()


object_model = build_object_model()

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


MATTE_BLACK = Material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
DARK_HARDWARE = Material("black_hardware", rgba=(0.025, 0.025, 0.022, 1.0))
RUBBER = Material("rubber", rgba=(0.0, 0.0, 0.0, 1.0))
WARM_GLASS = Material("warm_lens_glass", rgba=(1.0, 0.72, 0.32, 0.45))
BRUSHED_EDGE = Material("worn_black_edge", rgba=(0.12, 0.115, 0.105, 1.0))


def _annular_cylinder_z(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A centered tube whose axis is local Z."""

    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _spotlight_can_shell() -> cq.Workplane:
    """Hollow PAR-style can, front retaining rim, and closed rear cap."""

    body_shell = (
        cq.Workplane("YZ")
        .circle(0.160)
        .circle(0.134)
        .extrude(0.580)
        .translate((-0.290, 0.0, 0.0))
    )
    front_rim = (
        cq.Workplane("YZ")
        .circle(0.180)
        .circle(0.124)
        .extrude(0.060)
        .translate((0.275, 0.0, 0.0))
    )
    rear_cap = (
        cq.Workplane("YZ")
        .circle(0.150)
        .extrude(0.035)
        .translate((-0.305, 0.0, 0.0))
    )
    return body_shell.union(front_rim).union(rear_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_spotlight")

    for material in (MATTE_BLACK, DARK_HARDWARE, RUBBER, WARM_GLASS, BRUSHED_EDGE):
        model.material(material.name, rgba=material.rgba)

    base = model.part("base")
    base.visual(
        Box((0.560, 0.400, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="matte_black",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material="worn_black_edge",
        name="turntable_disk",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder_z(0.055, 0.034, 0.052), "base_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material="black_hardware",
        name="base_collar",
    )
    for i, (x, y) in enumerate(
        ((-0.210, -0.145), (-0.210, 0.145), (0.210, -0.145), (0.210, 0.145))
    ):
        base.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material="rubber",
            name=f"foot_{i}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.028, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material="black_hardware",
        name="pan_stem",
    )
    yoke.visual(
        Box((0.250, 0.520, 0.045)),
        origin=Origin(xyz=(-0.035, 0.0, 0.105)),
        material="matte_black",
        name="lower_bridge",
    )
    for i, y in enumerate((-0.238, 0.238)):
        yoke.visual(
            Box((0.070, 0.034, 0.320)),
            origin=Origin(xyz=(0.000, y, 0.250)),
            material="matte_black",
            name=f"arm_{i}",
        )
        yoke.visual(
            Cylinder(radius=0.058, length=0.034),
            origin=Origin(xyz=(0.000, y, 0.320), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="black_hardware",
            name=f"bearing_{i}",
        )
    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_spotlight_can_shell(), "can_shell"),
        material="matte_black",
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.128, length=0.012),
        origin=Origin(xyz=(0.333, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="warm_lens_glass",
        name="front_lens",
    )
    can.visual(
        Cylinder(radius=0.034, length=0.442),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="black_hardware",
        name="trunnion_axle",
    )
    for i, y in enumerate((-0.169, 0.169)):
        can.visual(
            Cylinder(radius=0.064, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="worn_black_edge",
            name=f"side_boss_{i}",
        )
    for i, z in enumerate((-0.055, 0.0, 0.055)):
        can.visual(
            Box((0.006, 0.090, 0.010)),
            origin=Origin(xyz=(-0.306, 0.0, z)),
            material="black_hardware",
            name=f"rear_vent_{i}",
        )
    for i, y in enumerate((-0.053, 0.053)):
        can.visual(
            Cylinder(radius=0.008, length=0.030),
            origin=Origin(xyz=(-0.235, y, 0.154), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="black_hardware",
            name=f"hinge_knuckle_{i}",
        )

    service_flap = model.part("service_flap")
    service_flap.visual(
        Cylinder(radius=0.008, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="black_hardware",
        name="flap_barrel",
    )
    service_flap.visual(
        Box((0.036, 0.058, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        material="black_hardware",
        name="flap_hinge_leaf",
    )
    service_flap.visual(
        Box((0.145, 0.112, 0.006)),
        origin=Origin(xyz=(0.078, 0.0, 0.011)),
        material="matte_black",
        name="flap_panel",
    )
    service_flap.visual(
        Box((0.018, 0.060, 0.012)),
        origin=Origin(xyz=(0.152, 0.0, 0.018)),
        material="worn_black_edge",
        name="flap_latch_lip",
    )
    for i, y in enumerate((-0.058, 0.058)):
        service_flap.visual(
            Box((0.125, 0.006, 0.008)),
            origin=Origin(xyz=(0.080, y, 0.017)),
            material="black_hardware",
            name=f"flap_side_rib_{i}",
        )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.2, lower=-0.90, upper=1.05),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=service_flap,
        origin=Origin(xyz=(-0.235, 0.0, 0.154)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    service_flap = object_model.get_part("service_flap")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_hinge = object_model.get_articulation("tilt_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")

    ctx.allow_overlap(
        can,
        service_flap,
        elem_a="can_shell",
        elem_b="flap_barrel",
        reason="The short service-flap hinge barrel is intentionally seated through the rear shell hinge line.",
    )
    ctx.expect_within(
        yoke,
        base,
        axes="xy",
        inner_elem="pan_stem",
        outer_elem="base_collar",
        margin=0.004,
        name="pan stem is centered inside the base collar",
    )
    ctx.expect_overlap(
        yoke,
        base,
        axes="z",
        elem_a="pan_stem",
        elem_b="base_collar",
        min_overlap=0.040,
        name="pan stem remains inserted in the collar",
    )
    ctx.expect_contact(
        can,
        yoke,
        elem_a="trunnion_axle",
        elem_b="bearing_1",
        contact_tol=0.0015,
        name="can trunnion reaches the yoke bearing",
    )
    ctx.expect_contact(
        service_flap,
        can,
        elem_a="flap_barrel",
        elem_b="can_shell",
        contact_tol=0.004,
        name="service flap hinge sits on the rear shell",
    )

    front_closed = ctx.part_element_world_aabb(can, elem="front_lens")
    with ctx.pose({tilt_hinge: 0.75}):
        front_raised = ctx.part_element_world_aabb(can, elem="front_lens")
    ctx.check(
        "positive tilt raises the beam end",
        front_closed is not None
        and front_raised is not None
        and ((front_raised[0][2] + front_raised[1][2]) * 0.5)
        > ((front_closed[0][2] + front_closed[1][2]) * 0.5) + 0.12,
        details=f"closed={front_closed}, raised={front_raised}",
    )

    flap_closed = ctx.part_element_world_aabb(service_flap, elem="flap_latch_lip")
    with ctx.pose({flap_hinge: 1.00}):
        flap_open = ctx.part_element_world_aabb(service_flap, elem="flap_latch_lip")
    ctx.check(
        "service flap opens away from the rear shell",
        flap_closed is not None
        and flap_open is not None
        and ((flap_open[0][2] + flap_open[1][2]) * 0.5)
        > ((flap_closed[0][2] + flap_closed[1][2]) * 0.5) + 0.06,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    yoke_closed = ctx.part_world_aabb(yoke)
    with ctx.pose({pan_joint: 0.70}):
        yoke_panned = ctx.part_world_aabb(yoke)
    ctx.check(
        "vertical pan joint changes yoke heading",
        yoke_closed is not None
        and yoke_panned is not None
        and abs((yoke_panned[1][0] - yoke_panned[0][0]) - (yoke_closed[1][0] - yoke_closed[0][0])) > 0.02,
        details=f"closed={yoke_closed}, panned={yoke_panned}",
    )

    return ctx.report()


object_model = build_object_model()

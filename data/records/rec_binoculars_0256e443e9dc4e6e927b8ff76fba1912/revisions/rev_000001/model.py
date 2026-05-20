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


BARREL_Y = 0.055
BARREL_RADIUS = 0.038
BARREL_LENGTH = 0.235


def _eyeshield_mask_shape() -> cq.Workplane:
    """Rubber face shield with two eye openings, authored in the hood frame.

    Local frame convention: the hinge line is the local Y axis at (x=0, z=0).
    The deployed hood hangs downward and slightly rearward along negative X.
    """

    width = 0.158
    height = 0.112
    thickness = 0.016
    eye_spacing = 0.108
    eye_radius = 0.024

    return (
        cq.Workplane("YZ")
        .center(0.0, -height / 2.0)
        .rect(width, height)
        .pushPoints([(-eye_spacing / 2.0, -0.010), (eye_spacing / 2.0, -0.010)])
        .circle(eye_radius)
        .extrude(-thickness)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_vision_binoculars")

    model.material("matte_black_polymer", rgba=(0.025, 0.028, 0.026, 1.0))
    model.material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    model.material("dark_graphite", rgba=(0.10, 0.12, 0.11, 1.0))
    model.material("coated_glass", rgba=(0.12, 0.48, 0.38, 0.72))
    model.material("ir_glass", rgba=(0.32, 0.05, 0.04, 0.82))
    model.material("screw_metal", rgba=(0.42, 0.44, 0.42, 1.0))

    bridge = model.part("bridge")

    # Two long night-vision optical tubes, permanently molded to the central bridge.
    for index, y_pos in enumerate((-BARREL_Y, BARREL_Y)):
        bridge.visual(
            Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="matte_black_polymer",
            name=("objective_barrel_0", "objective_barrel_1")[index],
        )
        bridge.visual(
            Cylinder(radius=0.050, length=0.062),
            origin=Origin(xyz=(0.087, y_pos, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="dark_graphite",
            name=("front_objective_collar_0", "front_objective_collar_1")[index],
        )
        bridge.visual(
            Cylinder(radius=0.037, length=0.006),
            origin=Origin(xyz=(0.121, y_pos, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="coated_glass",
            name=("green_objective_lens_0", "green_objective_lens_1")[index],
        )
        bridge.visual(
            Cylinder(radius=0.030, length=0.038),
            origin=Origin(xyz=(-0.117, y_pos, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="rubber_black",
            name=("rear_eyepiece_0", "rear_eyepiece_1")[index],
        )
        bridge.visual(
            Cylinder(radius=0.021, length=0.004),
            origin=Origin(xyz=(-0.137, y_pos, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="coated_glass",
            name=("eyepiece_lens_0", "eyepiece_lens_1")[index],
        )

    # Central molded bridge and raised brow physically tie the two barrels together.
    bridge.visual(
        Box((0.178, 0.090, 0.054)),
        origin=Origin(xyz=(-0.006, 0.0, 0.003)),
        material="matte_black_polymer",
        name="central_bridge_block",
    )
    bridge.visual(
        Box((0.155, 0.042, 0.082)),
        origin=Origin(xyz=(0.008, 0.0, 0.020)),
        material="dark_graphite",
        name="raised_electronics_hump",
    )
    bridge.visual(
        Box((0.042, 0.168, 0.030)),
        origin=Origin(xyz=(-0.122, 0.0, 0.049)),
        material="rubber_black",
        name="rear_hinge_brow",
    )
    bridge.visual(
        Box((0.040, 0.166, 0.026)),
        origin=Origin(xyz=(0.116, 0.0, 0.044)),
        material="matte_black_polymer",
        name="front_yoke_crossbar",
    )

    # Fixed IR illuminator mounted above the bridge at the front.
    bridge.visual(
        Box((0.058, 0.040, 0.044)),
        origin=Origin(xyz=(0.085, 0.0, 0.065)),
        material="dark_graphite",
        name="ir_mount_pedestal",
    )
    bridge.visual(
        Cylinder(radius=0.022, length=0.045),
        origin=Origin(xyz=(0.113, 0.0, 0.088), rpy=(0.0, pi / 2.0, 0.0)),
        material="matte_black_polymer",
        name="ir_illuminator_body",
    )
    bridge.visual(
        Cylinder(radius=0.016, length=0.005),
        origin=Origin(xyz=(0.138, 0.0, 0.088), rpy=(0.0, pi / 2.0, 0.0)),
        material="ir_glass",
        name="ir_illuminator_lens",
    )

    # Parent-side hinge knuckles for the folding eyeshield.
    for y_pos in (-0.092, 0.092):
        bridge.visual(
            Box((0.020, 0.020, 0.026)),
            origin=Origin(xyz=(-0.139, y_pos, 0.064)),
            material="rubber_black",
            name=f"eyeshield_hinge_lug_{'lower' if y_pos < 0.0 else 'upper'}",
        )
        bridge.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(-0.145, y_pos, 0.071), rpy=(pi / 2.0, 0.0, 0.0)),
            material="screw_metal",
            name=f"eyeshield_hinge_pin_end_{'lower' if y_pos < 0.0 else 'upper'}",
        )

    # Parent-side hinge knuckles for the small IR lens cap.
    for y_pos in (-0.021, 0.021):
        bridge.visual(
            Box((0.012, 0.010, 0.018)),
            origin=Origin(xyz=(0.141, y_pos, 0.111)),
            material="matte_black_polymer",
            name=f"ir_cap_hinge_lug_{'lower' if y_pos < 0.0 else 'upper'}",
        )

    eyeshield = model.part("eyeshield_hood")
    eyeshield.visual(
        mesh_from_cadquery(_eyeshield_mask_shape(), "eyeshield_hood"),
        material="rubber_black",
        name="rubber_eye_mask",
    )
    eyeshield.visual(
        Cylinder(radius=0.005, length=0.164),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="eyeshield_hinge_barrel",
    )
    for y_pos in (-0.084, 0.084):
        eyeshield.visual(
            Box((0.031, 0.012, 0.070)),
            origin=Origin(xyz=(-0.021, y_pos, -0.066)),
            material="rubber_black",
            name=f"side_eye_cup_{'lower' if y_pos < 0.0 else 'upper'}",
        )

    ir_cap = model.part("ir_cap")
    ir_cap.visual(
        Cylinder(radius=0.022, length=0.007),
        origin=Origin(xyz=(0.004, 0.0, -0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="cap_disk",
    )
    ir_cap.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="cap_hinge_barrel",
    )
    ir_cap.visual(
        Box((0.010, 0.014, 0.026)),
        origin=Origin(xyz=(0.003, 0.0, -0.012)),
        material="rubber_black",
        name="cap_flexible_strap",
    )

    model.articulation(
        "eyeshield_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=eyeshield,
        origin=Origin(xyz=(-0.145, 0.0, 0.071)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.05, effort=1.8, velocity=2.0),
    )
    model.articulation(
        "ir_cap_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=ir_cap,
        origin=Origin(xyz=(0.145, 0.0, 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.70, effort=0.45, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    eyeshield = object_model.get_part("eyeshield_hood")
    ir_cap = object_model.get_part("ir_cap")
    eyeshield_hinge = object_model.get_articulation("eyeshield_hinge")
    ir_cap_hinge = object_model.get_articulation("ir_cap_hinge")

    ctx.expect_overlap(
        bridge,
        bridge,
        elem_a="objective_barrel_0",
        elem_b="front_objective_collar_0",
        axes="x",
        min_overlap=0.020,
        name="front collar is fixed around first objective barrel",
    )
    ctx.expect_overlap(
        bridge,
        bridge,
        elem_a="objective_barrel_1",
        elem_b="front_objective_collar_1",
        axes="x",
        min_overlap=0.020,
        name="front collar is fixed around second objective barrel",
    )
    ctx.expect_gap(
        bridge,
        eyeshield,
        axis="x",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem="eyepiece_lens_0",
        negative_elem="rubber_eye_mask",
        name="folding hood sits just behind eyepieces when deployed",
    )
    ctx.expect_gap(
        ir_cap,
        bridge,
        axis="x",
        min_gap=0.003,
        max_gap=0.018,
        positive_elem="cap_disk",
        negative_elem="ir_illuminator_lens",
        name="ir cap is directly in front of illuminator lens when closed",
    )

    hood_rest = ctx.part_element_world_aabb(eyeshield, elem="rubber_eye_mask")
    cap_rest = ctx.part_element_world_aabb(ir_cap, elem="cap_disk")
    with ctx.pose({eyeshield_hinge: 2.0, ir_cap_hinge: 1.70}):
        hood_open = ctx.part_element_world_aabb(eyeshield, elem="rubber_eye_mask")
        cap_open = ctx.part_element_world_aabb(ir_cap, elem="cap_disk")
        ctx.expect_gap(
            eyeshield,
            bridge,
            axis="z",
            min_gap=0.006,
            positive_elem="rubber_eye_mask",
            negative_elem="rear_hinge_brow",
            name="opened eyeshield flips above rear brow",
        )
        ctx.expect_gap(
            ir_cap,
            bridge,
            axis="z",
            min_gap=0.004,
            positive_elem="cap_disk",
            negative_elem="ir_illuminator_lens",
            name="opened ir cap clears illuminator lens upward",
        )

    ctx.check(
        "hinged parts lift when opened",
        hood_rest is not None
        and hood_open is not None
        and cap_rest is not None
        and cap_open is not None
        and hood_open[1][2] > hood_rest[1][2] + 0.050
        and cap_open[0][2] > cap_rest[0][2] + 0.035,
        details=f"hood_rest={hood_rest}, hood_open={hood_open}, cap_rest={cap_rest}, cap_open={cap_open}",
    )

    return ctx.report()


object_model = build_object_model()

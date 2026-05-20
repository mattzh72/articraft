from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_DEPTH = 0.272
BASE_WIDTH = 0.312
BASE_HEIGHT = 0.070

LID_DEPTH = 0.262
LID_WIDTH = 0.304

HINGE_X = -0.116
HINGE_Z = 0.064

STRAP_HINGE_X = 0.145
STRAP_HINGE_Z = 0.058
STRAP_CLOSED_ANGLE = math.radians(22.0)


def _base_geometry() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )

    top_pocket = (
        cq.Workplane("XY")
        .box(0.228, 0.270, 0.020, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_HEIGHT - 0.020))
    )

    front_pedestal = (
        cq.Workplane("XY")
        .box(0.030, 0.040, 0.0155, centered=(True, True, False))
        .translate((BASE_DEPTH / 2.0 + 0.005, 0.0, 0.039))
    )

    ear_offset_y = 0.018
    ears = (
        cq.Workplane("XY")
        .box(0.011, 0.008, 0.015, centered=(True, True, False))
        .translate((BASE_DEPTH / 2.0 - 0.002, ear_offset_y, 0.051))
        .union(
            cq.Workplane("XY")
            .box(0.011, 0.008, 0.015, centered=(True, True, False))
            .translate((BASE_DEPTH / 2.0 - 0.002, -ear_offset_y, 0.051))
        )
    )

    rear_shoulder_y = BASE_WIDTH / 2.0 - 0.034
    rear_shoulders = (
        cq.Workplane("XY")
        .box(0.030, 0.030, 0.012, centered=(True, True, False))
        .translate((-BASE_DEPTH / 2.0 + 0.022, rear_shoulder_y, BASE_HEIGHT - 0.012))
        .union(
            cq.Workplane("XY")
            .box(0.030, 0.030, 0.012, centered=(True, True, False))
            .translate((-BASE_DEPTH / 2.0 + 0.022, -rear_shoulder_y, BASE_HEIGHT - 0.012))
        )
    )

    return shell.cut(top_pocket).union(front_pedestal).union(ears).union(rear_shoulders)


def _lid_geometry() -> cq.Workplane:
    rear_overhang = 0.006
    shell_x = LID_DEPTH / 2.0 - rear_overhang

    lower_shell = (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, 0.038, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((shell_x, 0.0, 0.006))
    )

    crown = (
        cq.Workplane("XY")
        .box(0.214, 0.250, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .translate((shell_x + 0.006, 0.0, 0.028))
    )

    front_nose = (
        cq.Workplane("XY")
        .box(0.040, 0.120, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((LID_DEPTH - rear_overhang - 0.028, 0.0, 0.011))
    )

    inner_cavity = (
        cq.Workplane("XY")
        .box(0.226, 0.268, 0.017, centered=(True, True, False))
        .translate((shell_x + 0.004, 0.0, 0.006))
    )

    return lower_shell.union(crown).union(front_nose).cut(inner_cavity)


def _strap_geometry() -> cq.Workplane:
    barrel_radius = 0.0035
    barrel_length = 0.026

    barrel = (
        cq.Workplane("XZ")
        .circle(barrel_radius)
        .extrude(barrel_length)
        .translate((0.0, -barrel_length / 2.0, 0.0))
    )

    strap = (
        cq.Workplane("XY")
        .box(0.0032, 0.034, 0.060, centered=(True, True, False))
        .translate((0.0, 0.0, 0.001))
    )

    hook = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.007, centered=(True, True, False))
        .translate((-0.0045, 0.0, 0.056))
    )

    side_rib_y = 0.013
    side_ribs = (
        cq.Workplane("XY")
        .box(0.0022, 0.004, 0.038, centered=(True, True, False))
        .translate((0.0, side_rib_y, 0.010))
        .union(
            cq.Workplane("XY")
            .box(0.0022, 0.004, 0.038, centered=(True, True, False))
            .translate((0.0, -side_rib_y, 0.010))
        )
    )

    return (
        barrel.union(strap).union(hook).union(side_ribs).rotate(
            (0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            math.degrees(STRAP_CLOSED_ANGLE),
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sandwich_press")

    cast_black = model.material("cast_black", rgba=(0.16, 0.16, 0.17, 1.0))
    lid_black = model.material("lid_black", rgba=(0.12, 0.12, 0.13, 1.0))
    strap_steel = model.material("strap_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dial_black = model.material("dial_black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_geometry(), "press_base"),
        material=cast_black,
        name="base_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_geometry(), "press_lid"),
        material=lid_black,
        name="lid_shell",
    )

    latch_strap = model.part("latch_strap")
    latch_strap.visual(
        mesh_from_cadquery(_strap_geometry(), "latch_strap"),
        material=strap_steel,
        name="strap_body",
    )

    dial = model.part("browning_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.020,
                body_style="skirted",
                top_diameter=0.025,
                base_diameter=0.032,
                crown_radius=0.002,
                edge_radius=0.0012,
                center=False,
            ),
            "browning_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_body",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    model.articulation(
        "base_to_latch_strap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch_strap,
        origin=Origin(xyz=(STRAP_HINGE_X, 0.0, STRAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    model.articulation(
        "base_to_browning_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.028, -BASE_WIDTH / 2.0, 0.036)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch_strap = object_model.get_part("latch_strap")
    dial = object_model.get_part("browning_dial")

    lid_hinge = object_model.get_articulation("base_to_lid")
    lid_limits = lid_hinge.motion_limits
    strap_joint = object_model.get_articulation("base_to_latch_strap")
    strap_limits = strap_joint.motion_limits

    ctx.allow_overlap(
        base,
        latch_strap,
        elem_a="base_shell",
        elem_b="strap_body",
        reason="The latch strap barrel is intentionally simplified as a captured hinge nested into the front lip support.",
    )

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="base_shell",
        min_gap=0.0,
        max_gap=0.004,
        name="lid casting sits above the base seam",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="base_shell",
        min_overlap=0.220,
        name="lid covers the base footprint",
    )
    ctx.expect_gap(
        base,
        dial,
        axis="y",
        positive_elem="base_shell",
        negative_elem="dial_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial stays separate from the right side wall",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward on the rear hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.090,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    closed_strap_aabb = ctx.part_element_world_aabb(latch_strap, elem="strap_body")
    if strap_limits is not None and strap_limits.upper is not None:
        with ctx.pose({strap_joint: strap_limits.upper}):
            open_strap_aabb = ctx.part_element_world_aabb(latch_strap, elem="strap_body")
            ctx.expect_gap(
                latch_strap,
                lid,
                axis="x",
                positive_elem="strap_body",
                negative_elem="lid_shell",
                min_gap=0.001,
                name="released latch strap clears the lid front",
            )
        ctx.check(
            "latch strap swings forward from the base lip",
            closed_strap_aabb is not None
            and open_strap_aabb is not None
            and open_strap_aabb[1][0] > closed_strap_aabb[1][0] + 0.020
            and open_strap_aabb[0][2] < closed_strap_aabb[0][2] - 0.010,
            details=f"closed={closed_strap_aabb}, open={open_strap_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.42
DEPTH = 0.45
BASE_HEIGHT = 0.11
HINGE_Y = -0.185
HINGE_Z = 0.142
LOWER_PLATE_SIZE = (0.34, 0.31, 0.022)
TOP_PLATE_SIZE = (0.335, 0.295, 0.018)


def cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def cq_cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def build_base_body() -> cq.Workplane:
    body = cq_box((WIDTH, DEPTH, BASE_HEIGHT), (0.0, 0.0, BASE_HEIGHT / 2.0))

    top_recess = cq_box((0.372, 0.342, 0.016), (0.0, 0.015, BASE_HEIGHT - 0.008))
    knob_bore = cq_box((0.05, 0.018, 0.018), (-WIDTH / 2.0 + 0.025, 0.11, 0.074))
    rocker_opening = cq_box((0.07, 0.04, 0.06), (WIDTH / 2.0 - 0.015, 0.09, 0.06))

    return body.cut(top_recess).cut(knob_bore).cut(rocker_opening)


def build_top_shell() -> cq.Workplane:
    shell = cq_box((0.38, 0.29, 0.056), (0.0, 0.19, 0.028))
    cheek_left = cq_box((0.04, 0.09, 0.036), (-0.16, 0.05, 0.022))
    cheek_right = cq_box((0.04, 0.09, 0.036), (0.16, 0.05, 0.022))
    frame_left = cq_box((0.02, 0.24, 0.018), (-0.17, 0.17, -0.004))
    frame_right = cq_box((0.02, 0.24, 0.018), (0.17, 0.17, -0.004))
    frame_front = cq_box((0.34, 0.02, 0.018), (0.0, 0.287, -0.004))
    frame_rear = cq_box((0.34, 0.02, 0.018), (0.0, 0.055, -0.004))
    front_post_left = cq_box((0.022, 0.02, 0.03), (-0.11, 0.333, 0.01))
    front_post_right = cq_box((0.022, 0.02, 0.03), (0.11, 0.333, 0.01))
    return (
        shell.union(cheek_left)
        .union(cheek_right)
        .union(frame_left)
        .union(frame_right)
        .union(frame_front)
        .union(frame_rear)
        .union(front_post_left)
        .union(front_post_right)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_contact_grill")

    stainless = model.material("stainless", rgba=(0.72, 0.73, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.26, 0.27, 0.29, 1.0))
    plate_iron = model.material("plate_iron", rgba=(0.12, 0.12, 0.12, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))
    power_red = model.material("power_red", rgba=(0.72, 0.14, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_body(), "grill_base_body"),
        material=stainless,
        name="body_shell",
    )
    base.visual(
        Box(LOWER_PLATE_SIZE),
        origin=Origin(xyz=(0.0, 0.015, 0.097)),
        material=plate_iron,
        name="lower_plate",
    )
    base.visual(
        Box((0.36, 0.018, 0.03)),
        origin=Origin(xyz=(0.0, -0.214, 0.123)),
        material=dark_metal,
        name="rear_support",
    )
    base.visual(
        Box((0.02, 0.028, 0.022)),
        origin=Origin(xyz=(-0.08, -0.199, 0.132)),
        material=dark_metal,
        name="hinge_post_0",
    )
    base.visual(
        Box((0.02, 0.028, 0.022)),
        origin=Origin(xyz=(0.08, -0.199, 0.132)),
        material=dark_metal,
        name="hinge_post_1",
    )
    base.visual(
        Box((0.012, 0.006, 0.05)),
        origin=Origin(xyz=(0.196, 0.111, 0.06)),
        material=dark_metal,
        name="switch_rail_front",
    )
    base.visual(
        Box((0.012, 0.006, 0.05)),
        origin=Origin(xyz=(0.196, 0.069, 0.06)),
        material=dark_metal,
        name="switch_rail_rear",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(
            xyz=(0.0, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="hinge_barrel",
    )

    platen = model.part("platen")
    platen.visual(
        mesh_from_cadquery(build_top_shell(), "grill_top_shell"),
        material=stainless,
        name="shell",
    )
    platen.visual(
        Box(TOP_PLATE_SIZE),
        origin=Origin(xyz=(0.0, 0.155, -0.022)),
        material=plate_iron,
        name="top_plate",
    )
    platen.visual(
        Cylinder(radius=0.014, length=0.11),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_knuckle_0",
    )
    platen.visual(
        Cylinder(radius=0.014, length=0.11),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_knuckle_1",
    )
    platen.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.0, 0.345, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_black,
        name="handle_bar",
    )

    model.articulation(
        "base_to_platen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platen,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.042, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "browning_knob",
        ),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=control_black,
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shaft",
    )
    knob.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(-0.018, 0.0, 0.013)),
        material=control_black,
        name="pointer_fin",
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(-WIDTH / 2.0, 0.11, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    rocker = model.part("rocker")
    rocker.visual(
        Box((0.014, 0.036, 0.046)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=control_black,
        name="rocker_body",
    )
    rocker.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    rocker.visual(
        Box((0.003, 0.018, 0.012)),
        origin=Origin(xyz=(0.0095, 0.0, 0.012)),
        material=power_red,
        name="rocker_lens",
    )
    model.articulation(
        "base_to_rocker",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(WIDTH / 2.0 - 0.008, 0.09, 0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.32, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    knob = object_model.get_part("knob")
    platen = object_model.get_part("platen")
    rocker = object_model.get_part("rocker")
    platen_hinge = object_model.get_articulation("base_to_platen")
    knob_joint = object_model.get_articulation("base_to_knob")
    rocker_joint = object_model.get_articulation("base_to_rocker")
    limits = platen_hinge.motion_limits

    with ctx.pose({platen_hinge: 0.0}):
        ctx.expect_gap(
            platen,
            base,
            axis="z",
            positive_elem="top_plate",
            negative_elem="lower_plate",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed platen rests just above lower plate",
        )
        ctx.expect_overlap(
            platen,
            base,
            axes="xy",
            elem_a="top_plate",
            elem_b="lower_plate",
            min_overlap=0.25,
            name="closed platen covers the cooking area",
        )
        ctx.expect_gap(
            base,
            knob,
            axis="x",
            positive_elem="body_shell",
            negative_elem="knob_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="browning knob mounts flush to the left wall",
        )

    knob_pointer_rest = ctx.part_element_world_aabb(knob, elem="pointer_fin")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_pointer_turned = ctx.part_element_world_aabb(knob, elem="pointer_fin")
        ctx.check(
            "browning knob pointer rotates around its shaft",
            knob_pointer_rest is not None
            and knob_pointer_turned is not None
            and abs(((knob_pointer_turned[0][1] + knob_pointer_turned[1][1]) / 2.0) - ((knob_pointer_rest[0][1] + knob_pointer_rest[1][1]) / 2.0)) > 0.008,
            details=f"rest={knob_pointer_rest}, turned={knob_pointer_turned}",
        )

    if limits is not None and limits.upper is not None:
        closed_handle = ctx.part_element_world_aabb(platen, elem="handle_bar")
        with ctx.pose({platen_hinge: limits.upper}):
            open_handle = ctx.part_element_world_aabb(platen, elem="handle_bar")
            ctx.check(
                "handle rises when platen opens",
                closed_handle is not None
                and open_handle is not None
                and ((open_handle[0][2] + open_handle[1][2]) / 2.0) > ((closed_handle[0][2] + closed_handle[1][2]) / 2.0) + 0.16,
                details=f"closed_handle={closed_handle}, open_handle={open_handle}",
            )

    rocker_lens_rest = ctx.part_element_world_aabb(rocker, elem="rocker_lens")
    with ctx.pose({rocker_joint: rocker_joint.motion_limits.upper}):
        rocker_lens_on = ctx.part_element_world_aabb(rocker, elem="rocker_lens")
        ctx.expect_contact(
            rocker,
            base,
            elem_a="rocker_body",
            elem_b="switch_rail_front",
            contact_tol=0.001,
            name="power rocker stays captured by the side opening",
        )
    with ctx.pose({rocker_joint: rocker_joint.motion_limits.lower}):
        rocker_lens_off = ctx.part_element_world_aabb(rocker, elem="rocker_lens")
    ctx.check(
        "power rocker tips on its local pivot",
        rocker_lens_rest is not None
        and rocker_lens_on is not None
        and rocker_lens_off is not None
        and abs(((rocker_lens_on[0][0] + rocker_lens_on[1][0]) / 2.0) - ((rocker_lens_off[0][0] + rocker_lens_off[1][0]) / 2.0)) > 0.006,
        details=f"rest={rocker_lens_rest}, on={rocker_lens_on}, off={rocker_lens_off}",
    )

    return ctx.report()


object_model = build_object_model()

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


def build_handwheel_shape() -> cq.Workplane:
    outer_radius = 0.14
    rim_inner_radius = 0.112
    wheel_width = 0.022
    hub_radius = 0.038
    axle_radius = 0.012
    axle_length = 0.18
    spoke_width = 0.018
    spoke_length = rim_inner_radius - hub_radius + 0.012
    spoke_center_z = (hub_radius + rim_inner_radius) * 0.5

    wheel = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(rim_inner_radius)
        .extrude(wheel_width / 2.0, both=True)
        .union(
            cq.Workplane("YZ")
            .circle(hub_radius)
            .extrude(axle_length / 2.0, both=True)
        )
    )

    for angle_deg in (0.0, 45.0, 90.0, 135.0):
        spoke = (
            cq.Workplane("YZ")
            .center(0.0, spoke_center_z)
            .rect(spoke_width, spoke_length)
            .extrude(wheel_width / 2.0, both=True)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        )
        wheel = wheel.union(spoke)

    return wheel


def aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_sluice_gate")

    frame_paint = model.material("frame_paint", rgba=(0.33, 0.40, 0.43, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.22, 0.42, 0.34, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.27, 0.29, 0.31, 1.0))
    wheel_paint = model.material("wheel_paint", rgba=(0.69, 0.10, 0.08, 1.0))
    lock_paint = model.material("lock_paint", rgba=(0.82, 0.66, 0.17, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.60, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=frame_paint,
        name="sill",
    )
    frame.visual(
        Box((0.11, 0.16, 0.66)),
        origin=Origin(xyz=(-0.245, 0.0, 0.45)),
        material=frame_paint,
        name="guide_0",
    )
    frame.visual(
        Box((0.11, 0.16, 0.66)),
        origin=Origin(xyz=(0.245, 0.0, 0.45)),
        material=frame_paint,
        name="guide_1",
    )
    frame.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(-0.21, 0.0, 0.73)),
        material=frame_paint,
        name="header_0",
    )
    frame.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(0.21, 0.0, 0.73)),
        material=frame_paint,
        name="header_1",
    )
    frame.visual(
        Box((0.18, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, -0.07, 0.73)),
        material=frame_paint,
        name="actuator_beam",
    )
    frame.visual(
        Box((0.09, 0.10, 0.32)),
        origin=Origin(xyz=(-0.18, 0.0, 0.94)),
        material=frame_paint,
        name="yoke_post_0",
    )
    frame.visual(
        Box((0.09, 0.10, 0.32)),
        origin=Origin(xyz=(0.18, 0.0, 0.94)),
        material=frame_paint,
        name="yoke_post_1",
    )
    frame.visual(
        Box((0.18, 0.06, 0.32)),
        origin=Origin(xyz=(0.0, -0.07, 0.94)),
        material=frame_paint,
        name="actuator_post",
    )
    frame.visual(
        Box((0.16, 0.10, 0.08)),
        origin=Origin(xyz=(-0.16, 0.0, 1.14)),
        material=frame_paint,
        name="yoke_cap_0",
    )
    frame.visual(
        Box((0.16, 0.10, 0.08)),
        origin=Origin(xyz=(0.16, 0.0, 1.14)),
        material=frame_paint,
        name="yoke_cap_1",
    )
    frame.visual(
        Box((0.18, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, -0.08, 0.88)),
        material=steel_dark,
        name="gearbox",
    )
    frame.visual(
        Box((0.07, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, -0.06, 1.04)),
        material=steel_dark,
        name="stem_guide",
    )
    frame.visual(
        Box((0.08, 0.14, 0.14)),
        origin=Origin(xyz=(-0.13, 0.11, 1.02)),
        material=frame_paint,
        name="wheel_support_0",
    )
    frame.visual(
        Box((0.08, 0.14, 0.14)),
        origin=Origin(xyz=(0.13, 0.11, 1.02)),
        material=frame_paint,
        name="wheel_support_1",
    )
    frame.visual(
        Box((0.012, 0.035, 0.04)),
        origin=Origin(xyz=(0.176, 0.165, 1.08)),
        material=steel_dark,
        name="tab_bracket_0",
    )
    frame.visual(
        Box((0.012, 0.035, 0.04)),
        origin=Origin(xyz=(0.204, 0.165, 1.08)),
        material=steel_dark,
        name="tab_bracket_1",
    )
    frame.visual(
        Box((0.028, 0.050, 0.020)),
        origin=Origin(xyz=(0.184, 0.145, 1.05)),
        material=steel_dark,
        name="tab_mount",
    )

    gate_panel = model.part("gate_panel")
    gate_panel.visual(
        Box((0.376, 0.045, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=gate_paint,
        name="closure_plate",
    )
    gate_panel.visual(
        Box((0.22, 0.055, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=gate_paint,
        name="head_block",
    )
    gate_panel.visual(
        Box((0.040, 0.012, 0.52)),
        origin=Origin(xyz=(-0.115, 0.028, 0.26)),
        material=steel_dark,
        name="stiffener_0",
    )
    gate_panel.visual(
        Box((0.040, 0.012, 0.52)),
        origin=Origin(xyz=(0.115, 0.028, 0.26)),
        material=steel_dark,
        name="stiffener_1",
    )
    gate_panel.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=steel_dark,
        name="stem",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(build_handwheel_shape(), "handwheel_body"),
        material=wheel_paint,
        name="wheel_body",
    )
    spinner_angle = math.radians(55.0)
    spinner_radius = 0.122
    handwheel.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(
            xyz=(
                0.0,
                spinner_radius * math.cos(spinner_angle),
                spinner_radius * math.sin(spinner_angle),
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_dark,
        name="spinner",
    )

    lock_tab = model.part("lock_tab")
    lock_tab.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="pivot_barrel",
    )
    lock_tab.visual(
        Box((0.012, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, 0.015, -0.040)),
        material=lock_paint,
        name="tab_arm",
    )
    lock_tab.visual(
        Box((0.020, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.026, -0.078)),
        material=lock_paint,
        name="pawl",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.26,
            effort=900.0,
            velocity=0.15,
        ),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.18, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=6.0,
        ),
    )
    model.articulation(
        "frame_to_lock_tab",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lock_tab,
        origin=Origin(xyz=(0.190, 0.165, 1.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.10,
            effort=8.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_panel = object_model.get_part("gate_panel")
    handwheel = object_model.get_part("handwheel")
    lock_tab = object_model.get_part("lock_tab")

    panel_joint = object_model.get_articulation("frame_to_panel")
    wheel_joint = object_model.get_articulation("frame_to_handwheel")
    tab_joint = object_model.get_articulation("frame_to_lock_tab")

    ctx.expect_gap(
        gate_panel,
        frame,
        axis="z",
        positive_elem="closure_plate",
        negative_elem="sill",
        max_gap=0.001,
        max_penetration=0.0,
        name="panel seats on sill in the closed pose",
    )
    ctx.expect_gap(
        gate_panel,
        frame,
        axis="x",
        positive_elem="closure_plate",
        negative_elem="guide_0",
        min_gap=0.001,
        max_gap=0.005,
        name="panel keeps a narrow clearance from guide_0",
    )
    ctx.expect_gap(
        frame,
        gate_panel,
        axis="x",
        positive_elem="guide_1",
        negative_elem="closure_plate",
        min_gap=0.001,
        max_gap=0.005,
        name="panel keeps a narrow clearance from guide_1",
    )
    ctx.expect_overlap(
        gate_panel,
        frame,
        axes="z",
        elem_a="closure_plate",
        elem_b="guide_0",
        min_overlap=0.54,
        name="closed panel overlaps the guide height",
    )
    ctx.expect_contact(
        handwheel,
        frame,
        elem_a="wheel_body",
        elem_b="wheel_support_0",
        contact_tol=0.002,
        name="handwheel axle bears against the yoke support",
    )
    ctx.expect_contact(
        lock_tab,
        frame,
        elem_a="pivot_barrel",
        elem_b="tab_bracket_0",
        contact_tol=1e-6,
        name="lock tab is mounted on its bracket",
    )

    rest_panel_pos = ctx.part_world_position(gate_panel)
    with ctx.pose({panel_joint: 0.26}):
        extended_panel_pos = ctx.part_world_position(gate_panel)
        ctx.expect_gap(
            gate_panel,
            frame,
            axis="x",
            positive_elem="closure_plate",
            negative_elem="guide_0",
            min_gap=0.001,
            max_gap=0.005,
            name="panel stays centered in guide_0 at full lift",
        )
        ctx.expect_gap(
            frame,
            gate_panel,
            axis="x",
            positive_elem="guide_1",
            negative_elem="closure_plate",
            min_gap=0.001,
            max_gap=0.005,
            name="panel stays centered in guide_1 at full lift",
        )
        ctx.expect_overlap(
            gate_panel,
            frame,
            axes="z",
            elem_a="closure_plate",
            elem_b="guide_0",
            min_overlap=0.38,
            name="raised panel remains retained by the guide",
        )
    ctx.check(
        "panel lifts upward",
        rest_panel_pos is not None
        and extended_panel_pos is not None
        and extended_panel_pos[2] > rest_panel_pos[2] + 0.20,
        details=f"rest={rest_panel_pos}, extended={extended_panel_pos}",
    )

    spinner_rest = aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner"))
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        spinner_rotated = aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner"))
    ctx.check(
        "handwheel rotates the spinner around the shaft",
        spinner_rest is not None
        and spinner_rotated is not None
        and abs(spinner_rotated[1] - spinner_rest[1]) > 0.05
        and math.dist(spinner_rest, spinner_rotated) > 0.15,
        details=f"rest={spinner_rest}, rotated={spinner_rotated}",
    )

    pawl_rest = aabb_center(ctx.part_element_world_aabb(lock_tab, elem="pawl"))
    with ctx.pose({tab_joint: 0.95}):
        pawl_released = aabb_center(ctx.part_element_world_aabb(lock_tab, elem="pawl"))
    ctx.check(
        "lock tab swings away from the wheel face",
        pawl_rest is not None
        and pawl_released is not None
        and pawl_released[1] > pawl_rest[1] + 0.03
        and pawl_released[2] > pawl_rest[2] + 0.02,
        details=f"rest={pawl_rest}, released={pawl_released}",
    )

    return ctx.report()


object_model = build_object_model()

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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


GLASS_THICKNESS = 0.0035
JUG_OUTER_RADIUS = 0.068
JUG_MAX_RADIUS = 0.080
JUG_OPENING_RADIUS = 0.063
JUG_GLASS_HEIGHT = 0.188
BASE_HEIGHT = 0.026


def _jug_glass_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(JUG_OUTER_RADIUS - 0.001)
        .workplane(offset=0.018)
        .circle(JUG_OUTER_RADIUS + 0.002)
        .workplane(offset=0.072)
        .circle(JUG_MAX_RADIUS)
        .workplane(offset=0.080)
        .circle(0.074)
        .workplane(offset=0.018)
        .circle(0.069)
        .loft(combine=True)
    )
    return outer.faces(">Z").shell(-GLASS_THICKNESS)


def _jug_top_trim_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.075).circle(JUG_OPENING_RADIUS).extrude(0.012)
    rear_hinge_bridge = (
        cq.Workplane("XY")
        .center(-0.089, 0.0)
        .box(0.034, 0.030, 0.012, centered=(True, True, False))
    )
    front_spout_nose = (
        cq.Workplane("XY")
        .center(0.075, 0.0)
        .box(0.016, 0.034, 0.010, centered=(True, True, False))
    )
    return ring.union(rear_hinge_bridge).union(front_spout_nose).translate((0.0, 0.0, 0.180))


def _jug_bottom_collar_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.074).circle(0.056).extrude(0.020)
    rear_pedestal = (
        cq.Workplane("XY")
        .center(-0.089, 0.0)
        .box(0.032, 0.034, 0.060, centered=(True, True, False))
    )
    return ring.union(rear_pedestal)


def _lid_shape() -> cq.Workplane:
    lid_disk = (
        cq.Workplane("XY")
        .center(0.063, 0.0)
        .circle(0.066)
        .extrude(0.003)
    )
    lid_dome = (
        cq.Workplane("XY")
        .workplane(offset=0.003)
        .center(0.063, 0.0)
        .circle(0.066)
        .workplane(offset=0.010)
        .center(0.063, 0.0)
        .circle(0.048)
        .loft(combine=True)
    )
    hinge_barrel = (
        cq.Workplane("YZ")
        .center(0.007, 0.0)
        .circle(0.006)
        .extrude(0.020, both=True)
    )
    front_lip = (
        cq.Workplane("XY")
        .center(0.124, 0.0)
        .box(0.008, 0.024, 0.010, centered=(True, True, False))
    )
    return lid_disk.union(lid_dome).union(hinge_barrel).union(front_lip)


def _handle_arch() -> object:
    return tube_from_spline_points(
        [
            (-0.119, 0.0, 0.044),
            (-0.155, 0.0, 0.092),
            (-0.157, 0.0, 0.170),
            (-0.129, 0.0, 0.219),
        ],
        radius=0.012,
        samples_per_segment=20,
        radial_segments=22,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_electric_kettle")

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.12, 0.13, 1.0))
    handle_black = model.material("handle_black", rgba=(0.11, 0.11, 0.12, 1.0))
    control_black = model.material("control_black", rgba=(0.15, 0.15, 0.16, 1.0))
    control_grey = model.material("control_grey", rgba=(0.28, 0.28, 0.30, 1.0))
    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.77, 0.90, 0.96, 0.28))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.112, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_black,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.081, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=trim_black,
        name="support_ring",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.100, 0.0, 0.026)),
        material=trim_black,
        name="selector_boss",
    )

    jug = model.part("jug")
    jug.visual(
        mesh_from_cadquery(_jug_glass_shape(), "kettle_glass_jug"),
        material=smoked_glass,
        name="glass_wall",
    )
    jug.visual(
        mesh_from_cadquery(_jug_top_trim_shape(), "kettle_top_trim"),
        material=trim_black,
        name="top_trim",
    )
    jug.visual(
        mesh_from_cadquery(_jug_bottom_collar_shape(), "kettle_bottom_collar"),
        material=trim_black,
        name="seat_ring",
    )
    jug.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, GLASS_THICKNESS + 0.002)),
        material=stainless,
        name="heater_plate",
    )
    model.articulation(
        "base_to_jug",
        ArticulationType.PRISMATIC,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.20,
            lower=0.0,
            upper=0.125,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(_handle_arch(), "kettle_handle_arch"),
        material=handle_black,
        name="grip_loop",
    )
    handle.visual(
        Box((0.024, 0.034, 0.024)),
        origin=Origin(xyz=(-0.117, 0.0, 0.047)),
        material=handle_black,
        name="lower_mount",
    )
    handle.visual(
        Box((0.016, 0.018, 0.014)),
        origin=Origin(xyz=(-0.114, 0.0, 0.213)),
        material=handle_black,
        name="upper_bridge",
    )
    handle.visual(
        Box((0.030, 0.006, 0.018)),
        origin=Origin(xyz=(-0.114, 0.011, 0.226)),
        material=handle_black,
        name="button_rail_0",
    )
    handle.visual(
        Box((0.030, 0.006, 0.018)),
        origin=Origin(xyz=(-0.114, -0.011, 0.226)),
        material=handle_black,
        name="button_rail_1",
    )
    handle.visual(
        Box((0.008, 0.026, 0.018)),
        origin=Origin(xyz=(-0.126, 0.0, 0.226)),
        material=handle_black,
        name="button_bridge",
    )
    handle.visual(
        Box((0.028, 0.006, 0.020)),
        origin=Origin(xyz=(-0.143, 0.017, 0.062)),
        material=handle_black,
        name="switch_cheek_0",
    )
    handle.visual(
        Box((0.028, 0.006, 0.020)),
        origin=Origin(xyz=(-0.143, 0.039, 0.062)),
        material=handle_black,
        name="switch_cheek_1",
    )
    handle.visual(
        Box((0.010, 0.028, 0.008)),
        origin=Origin(xyz=(-0.152, 0.028, 0.074)),
        material=handle_black,
        name="switch_bridge",
    )
    model.articulation(
        "jug_to_handle",
        ArticulationType.FIXED,
        parent=jug,
        child=handle,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "kettle_lid"),
        material=trim_black,
        name="lid_shell",
    )
    model.articulation(
        "jug_to_lid",
        ArticulationType.REVOLUTE,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(-JUG_OPENING_RADIUS, 0.0, 0.199)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    button = model.part("button")
    button.visual(
        Box((0.022, 0.014, 0.006)),
        material=control_grey,
        name="button_cap",
    )
    model.articulation(
        "handle_to_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=button,
        origin=Origin(xyz=(-0.114, 0.0, 0.232)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="lever_pivot",
    )
    lever.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, -0.006)),
        material=control_black,
        name="lever_paddle",
    )
    lever.visual(
        Box((0.008, 0.012, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.010)),
        material=control_black,
        name="lever_tip",
    )
    model.articulation(
        "handle_to_lever",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=lever,
        origin=Origin(xyz=(-0.140, 0.028, 0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=0.55,
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.012,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=10, depth=0.0006),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005, angle_deg=0.0),
                center=False,
            ),
            "kettle_selector_knob",
        ),
        material=control_black,
        name="selector_knob_shell",
    )
    model.articulation(
        "base_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_knob,
        origin=Origin(xyz=(0.100, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jug = object_model.get_part("jug")
    lid = object_model.get_part("lid")
    button = object_model.get_part("button")
    lever = object_model.get_part("lever")
    selector_knob = object_model.get_part("selector_knob")
    jug_lift = object_model.get_articulation("base_to_jug")
    lid_hinge = object_model.get_articulation("jug_to_lid")
    button_slide = object_model.get_articulation("handle_to_button")
    lever_pivot = object_model.get_articulation("handle_to_lever")
    selector_joint = object_model.get_articulation("base_to_selector_knob")

    ctx.expect_gap(
        jug,
        base,
        axis="z",
        positive_elem="seat_ring",
        negative_elem="support_ring",
        max_gap=0.0025,
        max_penetration=0.0,
        name="jug rests on the heating base ring",
    )
    ctx.expect_overlap(
        jug,
        base,
        axes="xy",
        elem_a="seat_ring",
        elem_b="support_ring",
        min_overlap=0.120,
        name="jug footprint stays centered over the base ring",
    )
    ctx.expect_gap(
        lid,
        jug,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="top_trim",
        max_gap=0.006,
        max_penetration=0.0,
        name="lid sits just above the top trim when closed",
    )
    ctx.expect_overlap(
        lid,
        jug,
        axes="xy",
        elem_a="lid_shell",
        elem_b="top_trim",
        min_overlap=0.120,
        name="closed lid covers the jug opening",
    )
    ctx.expect_gap(
        selector_knob,
        base,
        axis="z",
        positive_elem="selector_knob_shell",
        negative_elem="selector_boss",
        max_gap=0.0025,
        max_penetration=0.0,
        name="selector knob sits on the selector boss",
    )
    ctx.expect_overlap(
        selector_knob,
        base,
        axes="xy",
        elem_a="selector_knob_shell",
        elem_b="selector_boss",
        min_overlap=0.020,
        name="selector knob stays centered on the base boss",
    )

    jug_rest = ctx.part_world_position(jug)
    with ctx.pose({jug_lift: 0.120}):
        jug_lifted = ctx.part_world_position(jug)
        ctx.expect_gap(
            jug,
            base,
            axis="z",
            positive_elem="seat_ring",
            negative_elem="support_ring",
            min_gap=0.100,
            name="jug lifts clearly away from the base",
        )
    ctx.check(
        "jug lift moves upward",
        jug_rest is not None
        and jug_lifted is not None
        and jug_lifted[2] > jug_rest[2] + 0.10,
        details=f"rest={jug_rest}, lifted={jug_lifted}",
    )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.10}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and ((lid_open_aabb[0][2] + lid_open_aabb[1][2]) * 0.5)
        > ((lid_rest_aabb[0][2] + lid_rest_aabb[1][2]) * 0.5) + 0.03,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.004}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "button depresses downward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_tip")
    with ctx.pose({lever_pivot: 0.50}):
        lever_on_aabb = ctx.part_element_world_aabb(lever, elem="lever_tip")
    ctx.check(
        "lever swings downward",
        lever_rest_aabb is not None
        and lever_on_aabb is not None
        and ((lever_on_aabb[0][2] + lever_on_aabb[1][2]) * 0.5)
        < ((lever_rest_aabb[0][2] + lever_rest_aabb[1][2]) * 0.5) - 0.006,
        details=f"rest={lever_rest_aabb}, on={lever_on_aabb}",
    )

    selector_limits = selector_joint.motion_limits
    ctx.check(
        "selector knob uses continuous rotation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_limits is not None
        and selector_limits.lower is None
        and selector_limits.upper is None,
        details=(
            f"type={selector_joint.articulation_type}, "
            f"limits={None if selector_limits is None else (selector_limits.lower, selector_limits.upper)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()

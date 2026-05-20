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


def _x_axis_tube(outer_radius: float, inner_radius: float, length: float):
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length + 0.004).translate((-0.002, 0.0, 0.0))
    return outer.cut(inner).translate((-length * 0.5, 0.0, 0.0))


def _x_axis_rect_tube(
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    length: float,
):
    outer = cq.Workplane("YZ").rect(outer_y, outer_z).extrude(length)
    inner = cq.Workplane("YZ").rect(inner_y, inner_z).extrude(length + 0.004).translate((-0.002, 0.0, 0.0))
    return outer.cut(inner).translate((-length * 0.5, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_air_pump")

    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    molded_black = model.material("molded_black", rgba=(0.14, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(0.260, 0.078, 0.011)
            .translate((0.0, 0.0, -0.0195))
            .union(cq.Workplane("XY").box(0.260, 0.078, 0.011).translate((0.0, 0.0, 0.0195)))
            .union(cq.Workplane("XY").box(0.260, 0.019, 0.028).translate((0.0, -0.0295, 0.0)))
            .union(cq.Workplane("XY").box(0.260, 0.019, 0.028).translate((0.0, 0.0295, 0.0))),
            "pump_guide_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=frame_steel,
        name="guide_shell",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=frame_steel,
        name="barrel_base",
    )
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(0.047).circle(0.035).extrude(0.560),
            "pump_barrel_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=satin_steel,
        name="barrel_shell",
    )
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(0.056).circle(0.0125).extrude(0.050),
            "pump_top_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=frame_steel,
        name="top_collar",
    )
    body.visual(
        Box((0.120, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.031, 0.054)),
        material=grip_rubber,
        name="pedal_tread",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=satin_steel,
        name="piston_rod",
    )
    handle.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=frame_steel,
        name="handle_stop",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_steel,
        name="top_bar",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(-0.115, 0.0, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="top_pad_0",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.115, 0.0, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="top_pad_1",
    )

    stabilizer = model.part("stabilizer")
    stabilizer.visual(
        Cylinder(radius=0.011, length=0.500),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="slide_bar",
    )
    stabilizer.visual(
        Box((0.100, 0.040, 0.028)),
        origin=Origin(),
        material=frame_steel,
        name="slider_block",
    )
    stabilizer.visual(
        Box((0.020, 0.040, 0.034)),
        origin=Origin(xyz=(-0.205, 0.0, -0.004)),
        material=frame_steel,
        name="web_0",
    )
    stabilizer.visual(
        Box((0.020, 0.040, 0.034)),
        origin=Origin(xyz=(0.205, 0.0, -0.004)),
        material=frame_steel,
        name="web_1",
    )
    stabilizer.visual(
        Box((0.050, 0.090, 0.024)),
        origin=Origin(xyz=(-0.240, 0.0, -0.017)),
        material=grip_rubber,
        name="foot_0",
    )
    stabilizer.visual(
        Box((0.050, 0.090, 0.024)),
        origin=Origin(xyz=(0.240, 0.0, -0.017)),
        material=grip_rubber,
        name="foot_1",
    )

    grip = model.part("grip")
    grip.visual(
        mesh_from_cadquery(
            _x_axis_tube(outer_radius=0.028, inner_radius=0.0100, length=0.036)
            .translate((-0.030, 0.0, 0.0))
            .union(_x_axis_tube(outer_radius=0.028, inner_radius=0.0100, length=0.036).translate((0.030, 0.0, 0.0))),
            "pump_grip_sleeve",
        ),
        material=molded_black,
        name="sleeve",
    )
    grip.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(0.016, 0.040, 0.050)
            .translate((-0.022, 0.013, -0.043))
            .union(cq.Workplane("XY").box(0.016, 0.040, 0.050).translate((0.022, 0.013, -0.043))),
            "pump_grip_bridge",
        ),
        material=molded_black,
        name="bridge",
    )
    grip.visual(
        Cylinder(radius=0.013, length=0.110),
        origin=Origin(xyz=(0.0, 0.028, -0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="loop_bar",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.60,
            lower=0.0,
            upper=0.200,
        ),
    )
    model.articulation(
        "body_to_stabilizer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stabilizer,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=-0.080,
            upper=0.080,
        ),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    stabilizer = object_model.get_part("stabilizer")
    grip = object_model.get_part("grip")

    handle_slide = object_model.get_articulation("body_to_handle")
    stabilizer_slide = object_model.get_articulation("body_to_stabilizer")
    grip_spin = object_model.get_articulation("handle_to_grip")

    handle_limits = handle_slide.motion_limits
    stabilizer_limits = stabilizer_slide.motion_limits

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.allow_overlap(
        body,
        stabilizer,
        elem_a="guide_shell",
        elem_b="slider_block",
        reason="The lower stabilizer uses a retained sliding carriage nested inside the guide shell.",
    )
    ctx.allow_overlap(
        grip,
        handle,
        elem_a="sleeve",
        elem_b="top_bar",
        reason="The folding grip is intentionally modeled as rotating sleeves wrapped around the T-handle crossbar.",
    )

    ctx.expect_contact(
        handle,
        body,
        elem_a="handle_stop",
        elem_b="top_collar",
        name="handle rests on the top collar when collapsed",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        name="piston rod stays centered in the barrel",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.18,
        name="collapsed piston rod remains deeply inserted",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_slide: handle_limits.upper}):
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="barrel_shell",
                name="extended piston rod stays centered in the barrel",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a="piston_rod",
                elem_b="barrel_shell",
                min_overlap=0.14,
                name="extended piston rod still retains insertion",
            )
            extended_handle_pos = ctx.part_world_position(handle)
        ctx.check(
            "handle extends upward along the cylinder axis",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > rest_handle_pos[2] + 0.15,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    ctx.expect_within(
        stabilizer,
        body,
        axes="yz",
        inner_elem="slide_bar",
        outer_elem="guide_shell",
        margin=0.005,
        name="stabilizer bar stays centered in the lower guide",
    )
    ctx.expect_overlap(
        stabilizer,
        body,
        axes="x",
        elem_a="slider_block",
        elem_b="guide_shell",
        min_overlap=0.10,
        name="collapsed stabilizer carriage remains captured in the guide",
    )

    left_pos = None
    right_pos = None
    if stabilizer_limits is not None and stabilizer_limits.lower is not None and stabilizer_limits.upper is not None:
        with ctx.pose({stabilizer_slide: stabilizer_limits.lower}):
            ctx.expect_within(
                stabilizer,
                body,
                axes="yz",
                inner_elem="slide_bar",
                outer_elem="guide_shell",
                margin=0.005,
                name="stabilizer stays centered at the left travel stop",
            )
            ctx.expect_overlap(
                stabilizer,
                body,
                axes="x",
                elem_a="slider_block",
                elem_b="guide_shell",
                min_overlap=0.02,
                name="left-travel stabilizer carriage stays retained",
            )
            left_pos = ctx.part_world_position(stabilizer)
        with ctx.pose({stabilizer_slide: stabilizer_limits.upper}):
            ctx.expect_within(
                stabilizer,
                body,
                axes="yz",
                inner_elem="slide_bar",
                outer_elem="guide_shell",
                margin=0.005,
                name="stabilizer stays centered at the right travel stop",
            )
            ctx.expect_overlap(
                stabilizer,
                body,
                axes="x",
                elem_a="slider_block",
                elem_b="guide_shell",
                min_overlap=0.02,
                name="right-travel stabilizer carriage stays retained",
            )
            right_pos = ctx.part_world_position(stabilizer)
        ctx.check(
            "stabilizer slides laterally across the base",
            left_pos is not None and right_pos is not None and right_pos[0] > left_pos[0] + 0.14,
            details=f"left={left_pos}, right={right_pos}",
        )

    ctx.expect_overlap(
        grip,
        handle,
        axes="x",
        elem_a="sleeve",
        elem_b="top_bar",
        min_overlap=0.03,
        name="grip sleeves remain wrapped around the top bar",
    )

    top_bar_center = aabb_center(ctx.part_element_world_aabb(handle, elem="top_bar"))
    grip_origin = ctx.part_world_position(grip)
    ctx.check(
        "grip axis stays centered on the T-handle crossbar",
        top_bar_center is not None
        and grip_origin is not None
        and abs(grip_origin[0] - top_bar_center[0]) <= 0.001
        and abs(grip_origin[1] - top_bar_center[1]) <= 0.001
        and abs(grip_origin[2] - top_bar_center[2]) <= 0.001,
        details=f"grip_origin={grip_origin}, top_bar_center={top_bar_center}",
    )

    loop_rest = aabb_center(ctx.part_element_world_aabb(grip, elem="loop_bar"))
    with ctx.pose({grip_spin: math.pi / 2.0}):
        loop_folded = aabb_center(ctx.part_element_world_aabb(grip, elem="loop_bar"))
    ctx.check(
        "folding grip rotates around the crossbar axis",
        loop_rest is not None
        and loop_folded is not None
        and abs(loop_folded[1] - loop_rest[1]) > 0.035
        and abs(loop_folded[2] - loop_rest[2]) > 0.08,
        details=f"rest={loop_rest}, folded={loop_folded}",
    )

    return ctx.report()


object_model = build_object_model()

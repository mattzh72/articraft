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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_stand_mixer")

    porcelain = model.material("warm_porcelain", rgba=(0.92, 0.89, 0.83, 1.0))
    shadow = model.material("soft_shadow", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    steel = model.material("brushed_steel", rgba=(0.73, 0.74, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.33, 0.34, 1.0))
    satin = model.material("satin_chrome", rgba=(0.82, 0.82, 0.80, 1.0))

    base_shell = (
        cq.Workplane("XY")
        .box(0.64, 0.38, 0.075)
        .edges("|Z")
        .fillet(0.055)
        .edges(">Z")
        .fillet(0.018)
    )
    column_shell = (
        cq.Workplane("XY")
        .box(0.18, 0.24, 0.422)
        .edges("|Z")
        .fillet(0.050)
        .edges(">Z")
        .fillet(0.030)
    )
    head_shell = (
        cq.Workplane("XY")
        .box(0.405, 0.255, 0.165)
        .edges("|Z")
        .fillet(0.080)
        .edges(">Z")
        .fillet(0.050)
        .edges("<Z")
        .fillet(0.035)
    )
    bowl_shell = (
        cq.Workplane("XY")
        .circle(0.158)
        .circle(0.136)
        .extrude(0.205)
        .union(cq.Workplane("XY").circle(0.158).extrude(0.026))
        .edges(">Z")
        .fillet(0.006)
    )
    carriage_ring = (
        cq.Workplane("XY")
        .circle(0.176)
        .circle(0.138)
        .extrude(0.026)
    )
    bowl_lip = (
        cq.Workplane("XY")
        .circle(0.166)
        .circle(0.137)
        .extrude(0.014)
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_shell, "base_shell", tolerance=0.001),
        origin=Origin(xyz=(0.08, 0.0, 0.038)),
        material=porcelain,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(column_shell, "rear_column", tolerance=0.001),
        origin=Origin(xyz=(-0.205, 0.0, 0.271)),
        material=porcelain,
        name="rear_column",
    )
    base.visual(
        Cylinder(radius=0.043, length=0.062),
        origin=Origin(xyz=(-0.150, 0.094, 0.525), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel_0",
    )
    base.visual(
        Cylinder(radius=0.043, length=0.062),
        origin=Origin(xyz=(-0.150, -0.094, 0.525), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel_1",
    )
    base.visual(
        Box((0.024, 0.022, 0.245)),
        origin=Origin(xyz=(-0.106, 0.075, 0.280)),
        material=dark_steel,
        name="bowl_slide_rail_0",
    )
    base.visual(
        Box((0.024, 0.022, 0.245)),
        origin=Origin(xyz=(-0.106, -0.075, 0.280)),
        material=dark_steel,
        name="bowl_slide_rail_1",
    )
    base.visual(
        Box((0.076, 0.006, 0.036)),
        origin=Origin(xyz=(-0.150, 0.123, 0.215)),
        material=shadow,
        name="lock_slot",
    )
    base.visual(
        Box((0.050, 0.006, 0.050)),
        origin=Origin(xyz=(-0.180, -0.123, 0.220)),
        material=shadow,
        name="selector_recess",
    )
    for foot_index, x in enumerate((-0.155, 0.315)):
        for y_index, y in enumerate((-0.130, 0.130)):
            base.visual(
                Cylinder(radius=0.032, length=0.014),
                origin=Origin(xyz=(x, y, 0.007)),
                material=rubber,
                name=f"foot_{foot_index}_{y_index}",
            )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.040, 0.128, 0.165)),
        origin=Origin(xyz=(0.000, 0.0, 0.085)),
        material=dark_steel,
        name="slide_plate",
    )
    bowl_carriage.visual(
        Box((0.285, 0.072, 0.036)),
        origin=Origin(xyz=(0.142, 0.0, 0.035)),
        material=dark_steel,
        name="carriage_arm",
    )
    bowl_carriage.visual(
        mesh_from_cadquery(carriage_ring, "carriage_ring", tolerance=0.001),
        origin=Origin(xyz=(0.260, 0.0, 0.045)),
        material=dark_steel,
        name="support_ring",
    )
    bowl_carriage.visual(
        mesh_from_cadquery(bowl_shell, "mixing_bowl", tolerance=0.001),
        origin=Origin(xyz=(0.260, 0.0, 0.055)),
        material=steel,
        name="bowl_shell",
    )
    bowl_carriage.visual(
        mesh_from_cadquery(bowl_lip, "bowl_lip", tolerance=0.001),
        origin=Origin(xyz=(0.260, 0.0, 0.258)),
        material=satin,
        name="bowl_lip",
    )
    bowl_carriage.visual(
        Box((0.070, 0.018, 0.030)),
        origin=Origin(xyz=(0.105, 0.085, 0.077)),
        material=dark_steel,
        name="side_clip_0",
    )
    bowl_carriage.visual(
        Box((0.070, 0.018, 0.030)),
        origin=Origin(xyz=(0.105, -0.085, 0.077)),
        material=dark_steel,
        name="side_clip_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(head_shell, "oval_head", tolerance=0.001),
        origin=Origin(xyz=(0.290, 0.0, 0.055)),
        material=porcelain,
        name="head_shell",
    )
    head.visual(
        Box((0.135, 0.090, 0.085)),
        origin=Origin(xyz=(0.060, 0.0, 0.030)),
        material=porcelain,
        name="hinge_neck",
    )
    head.visual(
        Cylinder(radius=0.041, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    head.visual(
        Cylinder(radius=0.056, length=0.026),
        origin=Origin(xyz=(0.320, 0.0, -0.036)),
        material=satin,
        name="tool_hub",
    )
    head.visual(
        Box((0.120, 0.010, 0.028)),
        origin=Origin(xyz=(0.125, -0.124, 0.030)),
        material=shadow,
        name="speed_ticks",
    )

    hook_path = [
        (0.000, 0.000, -0.055),
        (0.000, 0.000, -0.105),
        (0.040, 0.000, -0.132),
        (0.064, 0.000, -0.164),
        (0.033, 0.000, -0.176),
        (-0.029, 0.000, -0.166),
        (-0.052, 0.000, -0.126),
    ]
    hook_mesh = mesh_from_geometry(
        tube_from_spline_points(
            hook_path,
            radius=0.0125,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "dough_hook",
    )

    tool = model.part("tool")
    tool.visual(
        Cylinder(radius=0.024, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=satin,
        name="tool_shaft",
    )
    tool.visual(
        hook_mesh,
        material=satin,
        name="dough_hook",
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="selector_knob",
    )
    speed_selector.visual(
        Box((0.010, 0.010, 0.046)),
        origin=Origin(xyz=(0.018, -0.022, 0.012)),
        material=satin,
        name="selector_pointer",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.052, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=dark_steel,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.022, 0.006, 0.018)),
        origin=Origin(xyz=(0.030, 0.022, 0.0)),
        material=satin,
        name="lock_tab",
    )

    model.articulation(
        "bowl_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(-0.090, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.08, lower=0.0, upper=0.035),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.150, 0.0, 0.525)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.7, lower=0.0, upper=0.85),
    )
    model.articulation(
        "tool_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool,
        origin=Origin(xyz=(0.320, 0.0, -0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )
    model.articulation(
        "speed_select",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(-0.180, -0.126, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "head_lock_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.150, 0.126, 0.215)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=0.0, upper=0.026),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    tool = object_model.get_part("tool")
    selector = object_model.get_part("speed_selector")
    lock = object_model.get_part("head_lock")

    bowl_lift = object_model.get_articulation("bowl_lift")
    head_tilt = object_model.get_articulation("head_tilt")
    tool_spin = object_model.get_articulation("tool_spin")
    speed_select = object_model.get_articulation("speed_select")
    lock_slide = object_model.get_articulation("head_lock_slide")

    expected_tree = {
        "bowl_lift": (ArticulationType.PRISMATIC, "base", "bowl_carriage"),
        "head_tilt": (ArticulationType.REVOLUTE, "base", "head"),
        "tool_spin": (ArticulationType.CONTINUOUS, "head", "tool"),
        "speed_select": (ArticulationType.REVOLUTE, "base", "speed_selector"),
        "head_lock_slide": (ArticulationType.PRISMATIC, "base", "head_lock"),
    }
    for joint_name, (joint_type, parent_name, child_name) in expected_tree.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} matches stand mixer tree",
            joint.articulation_type == joint_type
            and joint.parent == parent_name
            and joint.child == child_name,
            details=(
                f"type={joint.articulation_type}, parent={joint.parent}, "
                f"child={joint.child}"
            ),
        )

    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.035,
        name="closed head clears raised bowl rim at rest",
    )
    ctx.expect_within(
        tool,
        bowl,
        axes="xy",
        inner_elem="dough_hook",
        outer_elem="bowl_shell",
        margin=0.010,
        name="dough hook stays centered inside the cylindrical bowl",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="z",
        elem_a="slide_plate",
        elem_b="bowl_slide_rail_0",
        min_overlap=0.120,
        name="bowl carriage remains visibly captured by the base slide",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    with ctx.pose({bowl_lift: bowl_lift.motion_limits.upper}):
        raised_bowl_pos = ctx.part_world_position(bowl)
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.000,
            name="bowl lift upper stop still fits below head",
        )
    ctx.check(
        "bowl lift raises the carriage",
        rest_bowl_pos is not None
        and raised_bowl_pos is not None
        and raised_bowl_pos[2] > rest_bowl_pos[2] + 0.030,
        details=f"rest={rest_bowl_pos}, raised={raised_bowl_pos}",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        tilted_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head tilt raises the oval head",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.075,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    rest_tool_aabb = ctx.part_world_aabb(tool)
    with ctx.pose({tool_spin: math.pi / 2.0}):
        spun_tool_aabb = ctx.part_world_aabb(tool)
    ctx.check(
        "continuous tool spin sweeps the dough hook",
        rest_tool_aabb is not None
        and spun_tool_aabb is not None
        and (spun_tool_aabb[1][1] - spun_tool_aabb[0][1])
        > (rest_tool_aabb[1][1] - rest_tool_aabb[0][1]) + 0.050,
        details=f"rest={rest_tool_aabb}, spun={spun_tool_aabb}",
    )

    rest_selector_aabb = ctx.part_world_aabb(selector)
    with ctx.pose({speed_select: speed_select.motion_limits.upper}):
        turned_selector_aabb = ctx.part_world_aabb(selector)
    ctx.check(
        "speed selector rotates on the base",
        rest_selector_aabb is not None
        and turned_selector_aabb is not None
        and abs(turned_selector_aabb[1][2] - rest_selector_aabb[1][2]) > 0.004,
        details=f"rest={rest_selector_aabb}, turned={turned_selector_aabb}",
    )

    rest_lock_pos = ctx.part_world_position(lock)
    with ctx.pose({lock_slide: lock_slide.motion_limits.upper}):
        shifted_lock_pos = ctx.part_world_position(lock)
    ctx.check(
        "head lock slides prismatically along the base",
        rest_lock_pos is not None
        and shifted_lock_pos is not None
        and shifted_lock_pos[0] > rest_lock_pos[0] + 0.020,
        details=f"rest={rest_lock_pos}, shifted={shifted_lock_pos}",
    )

    return ctx.report()


object_model = build_object_model()

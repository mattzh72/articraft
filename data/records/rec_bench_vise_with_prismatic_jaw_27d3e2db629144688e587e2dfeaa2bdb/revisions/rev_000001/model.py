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


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _rear_body_shape():
    body = cq.Workplane("XY").box(0.118, 0.160, 0.180).translate((-0.060, 0.0, 0.0))
    back_plate = cq.Workplane("XY").box(0.028, 0.142, 0.168).translate((-0.118, 0.0, -0.004))
    mounting_flange = cq.Workplane("XY").box(0.108, 0.130, 0.028).translate((-0.074, 0.0, -0.098))
    screw_box = cq.Workplane("YZ").workplane(offset=-0.040).circle(0.050).extrude(0.035, both=True)
    upper_boss = cq.Workplane("YZ").workplane(offset=-0.034).center(0.0, 0.050).circle(0.026).extrude(0.028, both=True)
    lower_boss = cq.Workplane("YZ").workplane(offset=-0.034).center(0.0, -0.050).circle(0.026).extrude(0.028, both=True)
    return body.union(back_plate).union(mounting_flange).union(screw_box).union(upper_boss).union(lower_boss)


def _front_carriage_shape():
    carriage = cq.Workplane("XY").box(0.126, 0.160, 0.188).translate((0.091, 0.0, 0.0))
    screw_hub = cq.Workplane("YZ").workplane(offset=0.154).circle(0.042).extrude(0.018, both=True)
    lever_boss = cq.Workplane("XZ").workplane(offset=0.091).center(0.118, 0.040).circle(0.015).extrude(0.012, both=True)
    shape = carriage.union(screw_hub).union(lever_boss)

    for bore_radius, bore_z in ((0.0130, 0.050), (0.0130, -0.050), (0.0190, 0.0)):
        cutter = (
            cq.Workplane("YZ")
            .workplane(offset=0.091)
            .center(0.0, bore_z)
            .circle(bore_radius)
            .extrude(0.230, both=True)
        )
        shape = shape.cut(cutter)

    return shape


def _handle_collar_shape():
    return (
        cq.Workplane("YZ")
        .workplane(offset=0.015)
        .circle(0.020)
        .circle(0.016)
        .extrude(0.015, both=True)
    )


def _front_chop_shape():
    chop = (
        cq.Workplane("YZ")
        .workplane(offset=0.019)
        .rect(0.360, 0.160)
        .extrude(0.019, both=True)
    )
    for hole_radius, hole_z in ((0.018, 0.050), (0.018, -0.050), (0.032, 0.0)):
        cutter = (
            cq.Workplane("YZ")
            .workplane(offset=0.019)
            .center(0.0, hole_z)
            .circle(hole_radius)
            .extrude(0.030, both=True)
        )
        chop = chop.cut(cutter)
    return chop


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woodworking_quick_release_vise")

    cast_iron = model.material("cast_iron", rgba=(0.25, 0.28, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.65, 0.67, 0.70, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.17, 0.18, 0.19, 1.0))
    oak = model.material("oak", rgba=(0.73, 0.58, 0.39, 1.0))
    walnut = model.material("walnut", rgba=(0.35, 0.23, 0.14, 1.0))

    rear_jaw = model.part("rear_jaw")
    rear_jaw.visual(
        mesh_from_cadquery(_rear_body_shape(), "rear_body"),
        material=cast_iron,
        name="rear_body",
    )
    rear_jaw.visual(
        Box((0.045, 0.360, 0.160)),
        origin=Origin(xyz=(-0.0225, 0.0, 0.0)),
        material=oak,
        name="rear_chop",
    )
    rear_jaw.visual(
        Cylinder(radius=0.012, length=0.440),
        origin=_x_cylinder_origin(0.215, 0.0, 0.050),
        material=steel,
        name="guide_rod_upper",
    )
    rear_jaw.visual(
        Cylinder(radius=0.012, length=0.440),
        origin=_x_cylinder_origin(0.215, 0.0, -0.050),
        material=steel,
        name="guide_rod_lower",
    )
    rear_jaw.visual(
        Cylinder(radius=0.014, length=0.175),
        origin=_x_cylinder_origin(0.0825, 0.0, 0.0),
        material=blackened_steel,
        name="lead_screw",
    )
    rear_jaw.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=_x_cylinder_origin(-0.018, 0.0, 0.0),
        material=steel,
        name="thrust_collar",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        mesh_from_cadquery(_front_carriage_shape(), "guide_carriage"),
        material=cast_iron,
        name="guide_carriage",
    )
    front_jaw.visual(
        mesh_from_cadquery(_front_chop_shape(), "front_chop"),
        material=oak,
        name="front_chop",
    )
    front_jaw.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=_y_cylinder_origin(0.118, 0.093, 0.040),
        material=steel,
        name="lever_mount",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.320),
        origin=_y_cylinder_origin(0.012, 0.0, 0.0),
        material=steel,
        name="handle_bar",
    )
    handle.visual(
        mesh_from_cadquery(_handle_collar_shape(), "handle_collar"),
        material=steel,
        name="handle_collar",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=_y_cylinder_origin(0.012, 0.130, 0.0),
        material=walnut,
        name="grip_0",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=_y_cylinder_origin(0.012, -0.130, 0.0),
        material=walnut,
        name="grip_1",
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=_y_cylinder_origin(0.0, 0.009, 0.0),
        material=steel,
        name="lever_hub",
    )
    release_lever.visual(
        Box((0.072, 0.014, 0.018)),
        origin=Origin(xyz=(0.036, 0.009, -0.012)),
        material=blackened_steel,
        name="lever_arm",
    )
    release_lever.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=_y_cylinder_origin(0.072, 0.009, -0.012),
        material=walnut,
        name="lever_tip",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=rear_jaw,
        child=front_jaw,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.16, lower=0.0, upper=0.260),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=handle,
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=front_jaw,
        child=release_lever,
        origin=Origin(xyz=(0.118, 0.106, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-0.60, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_jaw = object_model.get_part("rear_jaw")
    front_jaw = object_model.get_part("front_jaw")
    handle = object_model.get_part("handle")
    release_lever = object_model.get_part("release_lever")
    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")
    lever_pivot = object_model.get_articulation("lever_pivot")

    jaw_travel = 0.260
    if jaw_slide.motion_limits is not None and jaw_slide.motion_limits.upper is not None:
        jaw_travel = jaw_slide.motion_limits.upper

    ctx.expect_gap(
        front_jaw,
        rear_jaw,
        axis="x",
        positive_elem="front_chop",
        negative_elem="rear_chop",
        min_gap=0.0,
        max_gap=0.003,
        name="closed jaws nearly meet",
    )
    ctx.expect_origin_distance(
        front_jaw,
        rear_jaw,
        axes="yz",
        max_dist=0.001,
        name="front jaw stays centered on the rear jaw",
    )
    ctx.expect_overlap(
        front_jaw,
        rear_jaw,
        axes="x",
        elem_a="guide_carriage",
        elem_b="guide_rod_upper",
        min_overlap=0.120,
        name="guide carriage captures the upper guide at rest",
    )

    rest_front_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({jaw_slide: jaw_travel}):
        ctx.expect_origin_gap(
            front_jaw,
            rear_jaw,
            axis="x",
            min_gap=jaw_travel - 0.002,
            max_gap=jaw_travel + 0.002,
            name="front jaw opens by the intended travel",
        )
        ctx.expect_overlap(
            front_jaw,
            rear_jaw,
            axes="x",
            elem_a="guide_carriage",
            elem_b="guide_rod_upper",
            min_overlap=0.090,
            name="guide carriage stays retained on the upper guide when opened",
        )
        ctx.expect_overlap(
            front_jaw,
            rear_jaw,
            axes="yz",
            elem_a="guide_carriage",
            elem_b="guide_rod_upper",
            min_overlap=0.020,
            name="guide carriage stays aligned around the upper guide when opened",
        )
        extended_front_pos = ctx.part_world_position(front_jaw)

    ctx.check(
        "front jaw translates forward on the screw axis",
        rest_front_pos is not None
        and extended_front_pos is not None
        and extended_front_pos[0] > rest_front_pos[0] + 0.200,
        details=f"rest={rest_front_pos}, extended={extended_front_pos}",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_spin: math.pi / 2.0}):
        quarter_turn_handle_aabb = ctx.part_world_aabb(handle)

    rest_handle_y_span = None
    quarter_turn_handle_z_span = None
    if rest_handle_aabb is not None:
        rest_handle_y_span = rest_handle_aabb[1][1] - rest_handle_aabb[0][1]
    if quarter_turn_handle_aabb is not None:
        quarter_turn_handle_z_span = quarter_turn_handle_aabb[1][2] - quarter_turn_handle_aabb[0][2]

    ctx.check(
        "handle rotates around the lead-screw axis",
        rest_handle_y_span is not None
        and quarter_turn_handle_z_span is not None
        and rest_handle_y_span > 0.280
        and quarter_turn_handle_z_span > 0.280,
        details=(
            f"rest_y_span={rest_handle_y_span}, "
            f"quarter_turn_z_span={quarter_turn_handle_z_span}"
        ),
    )

    rest_tip_center = _aabb_center(ctx.part_element_world_aabb(release_lever, elem="lever_tip"))
    with ctx.pose({lever_pivot: 0.60}):
        lifted_tip_center = _aabb_center(ctx.part_element_world_aabb(release_lever, elem="lever_tip"))

    ctx.check(
        "quick-release lever swings upward beside the handle",
        rest_tip_center is not None
        and lifted_tip_center is not None
        and lifted_tip_center[2] > rest_tip_center[2] + 0.020,
        details=f"rest_tip={rest_tip_center}, lifted_tip={lifted_tip_center}",
    )

    return ctx.report()


object_model = build_object_model()

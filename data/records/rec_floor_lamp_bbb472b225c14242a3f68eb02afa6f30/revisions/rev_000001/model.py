from __future__ import annotations

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


BASE_RADIUS = 0.17
BASE_THICKNESS = 0.03
SLEEVE_OUTER_RADIUS = 0.019
SLEEVE_INNER_RADIUS = 0.0155
SLEEVE_LENGTH = 0.72
SLIDE_TRAVEL = 0.22
MAST_RADIUS = 0.0135
MAST_LENGTH = 0.88
MAST_CENTER_Z = 0.14
HINGE_Z = 0.621
HINGE_X = 0.032


def _tube_shape(*, outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _head_hardware_shape() -> cq.Workplane:
    trunnion_radius = 0.0065
    trunnion_length = 0.015
    arm_radius = 0.008
    arm_length = 0.17
    shade_center_x = arm_length
    trunnion = cq.Workplane("XZ").circle(trunnion_radius).extrude(trunnion_length / 2.0, both=True)
    arm = cq.Workplane("YZ").circle(arm_radius).extrude(arm_length)
    socket = (
        cq.Workplane("XY")
        .center(shade_center_x, 0.0)
        .circle(0.018)
        .extrude(0.03, both=True)
        .translate((0.0, 0.0, 0.02))
    )
    return trunnion.union(arm).union(socket)


def _head_shade_shape() -> cq.Workplane:
    arm_length = 0.17
    shade_center_x = arm_length
    shade_center_z = -0.03
    shade_height = 0.16
    shade_outer_radius = 0.11
    shade_inner_radius = 0.107
    top_ring_z = shade_center_z + (shade_height / 2.0) - 0.004
    bottom_ring_z = shade_center_z - (shade_height / 2.0) + 0.004

    shade_shell = (
        cq.Workplane("XY")
        .center(shade_center_x, 0.0)
        .circle(shade_outer_radius)
        .circle(shade_inner_radius)
        .extrude(shade_height / 2.0, both=True)
        .translate((0.0, 0.0, shade_center_z))
    )
    top_ring = (
        cq.Workplane("XY")
        .center(shade_center_x, 0.0)
        .circle(shade_outer_radius)
        .circle(0.098)
        .extrude(0.004, both=True)
        .translate((0.0, 0.0, top_ring_z))
    )
    bottom_ring = (
        cq.Workplane("XY")
        .center(shade_center_x, 0.0)
        .circle(shade_outer_radius)
        .circle(0.103)
        .extrude(0.004, both=True)
        .translate((0.0, 0.0, bottom_ring_z))
    )
    shade_mount = (
        cq.Workplane("XY")
        .center(shade_center_x, 0.0)
        .circle(0.032)
        .circle(0.018)
        .extrude(0.004, both=True)
        .translate((0.0, 0.0, top_ring_z))
    )

    shade = shade_shell.union(top_ring).union(bottom_ring).union(shade_mount)

    spoke_local = cq.Workplane("XY").box(0.086, 0.004, 0.004).translate((0.056, 0.0, 0.0))
    for angle_deg in (0.0, 120.0, 240.0):
        shade = shade.union(
            spoke_local.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg).translate(
                (shade_center_x, 0.0, top_ring_z)
            )
        )

    return shade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_lamp")

    model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("brushed_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("warm_linen", rgba=(0.93, 0.90, 0.82, 0.96))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="powder_black",
        name="base_disk",
    )
    stand.visual(
        mesh_from_cadquery(
            _tube_shape(
                outer_radius=SLEEVE_OUTER_RADIUS,
                inner_radius=SLEEVE_INNER_RADIUS,
                length=SLEEVE_LENGTH,
            ),
            "outer_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
        material="brushed_steel",
        name="outer_sleeve",
    )
    stand.visual(
        mesh_from_cadquery(
            _tube_shape(
                outer_radius=0.033,
                inner_radius=SLEEVE_INNER_RADIUS,
                length=0.055,
            ),
            "lower_socket",
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
        material="powder_black",
        name="lower_socket",
    )
    stand.visual(
        mesh_from_cadquery(
            _tube_shape(
                outer_radius=0.026,
                inner_radius=SLEEVE_INNER_RADIUS,
                length=0.065,
            ),
            "top_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + SLEEVE_LENGTH - 0.040)),
        material="powder_black",
        name="top_collar",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=MAST_RADIUS, length=MAST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, MAST_CENTER_Z)),
        material="brushed_steel",
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.024, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="powder_black",
        name="lift_collar",
    )
    mast.visual(
        Cylinder(radius=0.019, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material="powder_black",
        name="head_socket",
    )
    mast.visual(
        Box((HINGE_X, 0.016, 0.014)),
        origin=Origin(xyz=(HINGE_X / 2.0, 0.0, 0.602)),
        material="powder_black",
        name="hinge_neck",
    )
    for index, y_center in enumerate((-0.0095, 0.0095)):
        mast.visual(
            Box((0.014, 0.004, 0.026)),
            origin=Origin(xyz=(HINGE_X, y_center, HINGE_Z + 0.001)),
            material="powder_black",
            name=f"hinge_ear_{index}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_hardware_shape(), "head_hardware"),
        material="powder_black",
        name="head_hardware",
    )
    head.visual(
        mesh_from_cadquery(_head_shade_shape(), "shade"),
        material="warm_linen",
        name="shade",
    )

    model.articulation(
        "stand_to_mast",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + SLEEVE_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-0.75,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    mast_slide = object_model.get_articulation("stand_to_mast")
    head_tilt = object_model.get_articulation("mast_to_head")

    ctx.expect_within(
        mast,
        stand,
        axes="xy",
        inner_elem="mast_tube",
        outer_elem="outer_sleeve",
        margin=0.0015,
        name="mast stays centered inside the sleeve",
    )
    ctx.expect_overlap(
        mast,
        stand,
        axes="z",
        elem_a="mast_tube",
        elem_b="outer_sleeve",
        min_overlap=0.28,
        name="collapsed mast remains inserted in the sleeve",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    slide_upper = mast_slide.motion_limits.upper if mast_slide.motion_limits is not None else 0.0
    with ctx.pose({mast_slide: slide_upper}):
        ctx.expect_within(
            mast,
            stand,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="outer_sleeve",
            margin=0.0015,
            name="extended mast stays centered inside the sleeve",
        )
        ctx.expect_overlap(
            mast,
            stand,
            axes="z",
            elem_a="mast_tube",
            elem_b="outer_sleeve",
            min_overlap=0.075,
            name="extended mast still retains insertion in the sleeve",
        )
        extended_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.18,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_head_aabb = ctx.part_element_world_aabb(head, elem="shade")
    tilt_lower = head_tilt.motion_limits.lower if head_tilt.motion_limits is not None else 0.0
    tilt_upper = head_tilt.motion_limits.upper if head_tilt.motion_limits is not None else 0.0
    with ctx.pose({head_tilt: tilt_upper}):
        raised_head_aabb = ctx.part_element_world_aabb(head, elem="shade")
    with ctx.pose({head_tilt: tilt_lower}):
        lowered_head_aabb = ctx.part_element_world_aabb(head, elem="shade")

    ctx.check(
        "head tilts upward at positive rotation",
        rest_head_aabb is not None
        and raised_head_aabb is not None
        and raised_head_aabb[1][2] > rest_head_aabb[1][2] + 0.03,
        details=f"rest={rest_head_aabb}, raised={raised_head_aabb}",
    )
    ctx.check(
        "head tilts downward at negative rotation",
        rest_head_aabb is not None
        and lowered_head_aabb is not None
        and lowered_head_aabb[0][2] < rest_head_aabb[0][2] - 0.03,
        details=f"rest={rest_head_aabb}, lowered={lowered_head_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

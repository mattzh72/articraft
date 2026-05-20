from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belgian_waffle_maker")

    body_black = model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.56, 0.56, 0.58, 1.0))
    knob_black = model.material("knob_black", rgba=(0.14, 0.14, 0.14, 1.0))
    knob_ring = model.material("knob_ring", rgba=(0.74, 0.74, 0.76, 1.0))
    indicator_gray = model.material("indicator_gray", rgba=(0.84, 0.84, 0.86, 1.0))

    base = model.part("base")

    base_profile = [
        (0.0, 0.0),
        (0.112, 0.0),
        (0.132, 0.004),
        (0.142, 0.018),
        (0.142, 0.036),
        (0.138, 0.043),
        (0.132, 0.047),
        (0.0, 0.047),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=72), "base_body_shell"),
        material=body_black,
        name="body_shell",
    )
    base.visual(
        Cylinder(radius=0.136, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=plate_gray,
        name="lower_plate",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.148, 0.0, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_ring,
        name="knob_socket",
    )

    rear_bracket = (
        cq.Workplane("XY")
        .workplane(offset=0.020)
        .center(-0.152, 0.0)
        .rect(0.030, 0.128)
        .extrude(0.066)
    )
    rear_slot = (
        cq.Workplane("XY")
        .workplane(offset=0.050)
        .center(-0.152, 0.0)
        .rect(0.034, 0.094)
        .extrude(0.036)
    )
    rear_bracket = rear_bracket.cut(rear_slot)
    rear_bracket = rear_bracket.edges("|Z").fillet(0.004)
    rear_bracket = rear_bracket.edges(">Z").fillet(0.003)
    base.visual(
        mesh_from_cadquery(rear_bracket, "rear_bracket"),
        material=body_black,
        name="rear_bracket",
    )

    lid = model.part("lid")

    lid_outer_profile = [
        (0.0, 0.040),
        (0.050, 0.039),
        (0.095, 0.032),
        (0.128, 0.014),
        (0.140, -0.006),
        (0.140, -0.020),
    ]
    lid_inner_profile = [
        (0.0, 0.035),
        (0.045, 0.034),
        (0.090, 0.028),
        (0.124, 0.013),
        (0.134, -0.004),
        (0.134, -0.020),
    ]
    lid.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                lid_outer_profile,
                lid_inner_profile,
                segments=72,
            ),
            "lid_top_shell",
        ),
        origin=Origin(xyz=(0.142, 0.0, -0.002)),
        material=body_black,
        name="top_shell",
    )
    lid.visual(
        Cylinder(radius=0.136, length=0.010),
        origin=Origin(xyz=(0.142, 0.0, -0.0145)),
        material=plate_gray,
        name="upper_plate",
    )
    lid.visual(
        Box((0.044, 0.086, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, -0.004)),
        material=body_black,
        name="hinge_spine",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_ring,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.040, 0.094, 0.014)),
        origin=Origin(xyz=(0.272, 0.0, -0.007)),
        material=body_black,
        name="front_handle",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_ring,
        name="knob_cap",
    )
    knob.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(0.023, 0.0, 0.015)),
        material=indicator_gray,
        name="indicator",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.135, 0.0, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.155, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("knob")
    lid_hinge = object_model.get_articulation("base_to_lid")
    knob_joint = object_model.get_articulation("base_to_knob")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            max_gap=0.003,
            max_penetration=0.0,
            name="upper and lower cooking plates close with a thin gap",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="top_shell",
            elem_b="body_shell",
            min_overlap=0.24,
            name="closed lid covers the circular base housing",
        )
        ctx.expect_contact(
            knob,
            base,
            elem_a="knob_body",
            elem_b="knob_socket",
            name="thermostat knob seats against the front socket",
        )

    lid_limits = lid_hinge.motion_limits
    closed_handle_center = None
    open_handle_center = None

    with ctx.pose({lid_hinge: 0.0}):
        closed_handle_center = _aabb_center(
            ctx.part_element_world_aabb(lid, elem="front_handle")
        )

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="front_handle",
                negative_elem="lower_plate",
                min_gap=0.08,
                name="opened lid front lifts well above the lower plate",
            )
            ctx.expect_contact(
                knob,
                base,
                elem_a="knob_body",
                elem_b="knob_socket",
                name="thermostat knob remains seated while rotated",
            )
            open_handle_center = _aabb_center(
                ctx.part_element_world_aabb(lid, elem="front_handle")
            )

    ctx.check(
        "front handle arcs upward when the lid opens",
        closed_handle_center is not None
        and open_handle_center is not None
        and open_handle_center[2] > closed_handle_center[2] + 0.10
        and open_handle_center[0] < closed_handle_center[0] - 0.04,
        details=f"closed={closed_handle_center}, open={open_handle_center}",
    )

    with ctx.pose({knob_joint: math.pi / 2.0}):
        ctx.expect_contact(
            knob,
            base,
            elem_a="knob_body",
            elem_b="knob_socket",
            name="continuous knob stays mounted at a turned pose",
        )

    return ctx.report()


object_model = build_object_model()

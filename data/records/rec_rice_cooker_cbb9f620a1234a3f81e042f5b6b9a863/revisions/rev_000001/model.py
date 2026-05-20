from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rice_cooker")

    base = model.part("base")
    base_shape = (
        cq.Workplane("XY")
        .ellipse(0.15, 0.125)
        .extrude(0.18)
        .edges("<Z").fillet(0.02)
        .faces(">Z").shell(-0.005)
    )
    base.visual(
        mesh_from_cadquery(base_shape, "base_shell"),
        name="base_visual",
    )

    lid = model.part("lid")
    lid_shape = (
        cq.Workplane("XY")
        .ellipse(0.15, 0.125)
        .extrude(0.05)
        .edges(">Z").fillet(0.02)
        .faces("<Z").shell(-0.005)
        .faces(">Z").workplane().center(0, -0.08).hole(0.03)
    )
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid_shell"),
        origin=Origin(xyz=(0.0, 0.125, 0.0)),
        name="lid_visual",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, -0.125, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.5),
    )

    vent_cap = model.part("vent_cap")
    vent_cap_shape = (
        cq.Workplane("XY")
        .circle(0.02)
        .extrude(0.005)
    )
    vent_cap.visual(
        mesh_from_cadquery(vent_cap_shape, "vent_cap_shell"),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        name="vent_cap_visual",
    )

    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, 0.030, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.5),
    )

    front_button = model.part("front_button")
    button_shape = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.01)
        .edges(">Z").fillet(0.002)
    )
    front_button.visual(
        mesh_from_cadquery(button_shape, "button_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-1.5708, 0.0, 0.0)),
        name="button_visual",
    )

    model.articulation(
        "base_to_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=front_button,
        origin=Origin(xyz=(0.0, 0.120, 0.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=0.005),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    front_button = object_model.get_part("front_button")

    lid_joint = object_model.get_articulation("base_to_lid")
    vent_joint = object_model.get_articulation("lid_to_vent_cap")
    button_joint = object_model.get_articulation("base_to_button")

    # The button intersects the base, allow it
    ctx.allow_overlap(base, front_button, reason="Button is embedded in the base housing.")
    
    # At rest, lid sits on base
    ctx.expect_gap(lid, base, axis="z", max_gap=0.001, max_penetration=0.0, name="lid sits on base")
    
    # At rest, vent cap sits on lid
    ctx.expect_gap(vent_cap, lid, axis="z", max_gap=0.001, max_penetration=0.0, name="vent cap sits on lid")

    # Test opened lid
    rest_lid_pos = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.5}):
        open_lid_pos = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            rest_lid_pos is not None and open_lid_pos is not None and open_lid_pos[1][2] > rest_lid_pos[1][2] + 0.1,
            details=f"rest={rest_lid_pos}, open={open_lid_pos}",
        )

    # Test opened vent cap
    rest_vent_pos = ctx.part_world_aabb(vent_cap)
    with ctx.pose({vent_joint: 1.5}):
        open_vent_pos = ctx.part_world_aabb(vent_cap)
        ctx.check(
            "vent cap opens upward",
            rest_vent_pos is not None and open_vent_pos is not None and open_vent_pos[1][2] > rest_vent_pos[1][2] + 0.01,
            details=f"rest={rest_vent_pos}, open={open_vent_pos}",
        )

    # Test button press
    rest_button_pos = ctx.part_world_aabb(front_button)
    with ctx.pose({button_joint: 0.005}):
        pressed_button_pos = ctx.part_world_aabb(front_button)
        ctx.check(
            "button moves inward",
            rest_button_pos is not None and pressed_button_pos is not None and pressed_button_pos[0][1] < rest_button_pos[0][1] - 0.002,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
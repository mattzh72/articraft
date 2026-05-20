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
)


STAINLESS = Material("brushed_stainless", rgba=(0.64, 0.66, 0.65, 1.0))
DARK = Material("black_control_glass", rgba=(0.015, 0.016, 0.017, 1.0))
BUTTON = Material("satin_black_buttons", rgba=(0.025, 0.026, 0.028, 1.0))
FILTER = Material("bright_baffle_steel", rgba=(0.78, 0.79, 0.76, 1.0))


def _canopy_shell() -> cq.Workplane:
    """Hollow, deep commercial hood shell extruded across its width."""

    width = 1.24
    side_wall = 0.035
    outer_profile = [
        (-0.42, 0.00),
        (0.43, 0.00),
        (0.43, 0.18),
        (0.25, 0.54),
        (-0.42, 0.54),
    ]
    inner_profile = [
        (-0.35, -0.035),
        (0.32, -0.035),
        (0.32, 0.12),
        (0.17, 0.49),
        (-0.35, 0.49),
    ]

    outer = (
        cq.Workplane("YZ")
        .polyline(outer_profile)
        .close()
        .extrude(width)
        .translate((-width / 2.0, 0.0, 0.0))
    )
    cut_width = width - 2.0 * side_wall
    inner = (
        cq.Workplane("YZ")
        .polyline(inner_profile)
        .close()
        .extrude(cut_width)
        .translate((-cut_width / 2.0, 0.0, 0.0))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_canopy_range_hood")

    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_cadquery(_canopy_shell(), "canopy_shell", tolerance=0.002),
        material=STAINLESS,
        name="canopy_shell",
    )
    canopy.visual(
        Box((1.18, 0.050, 0.115)),
        origin=Origin(xyz=(0.0, 0.445, 0.100)),
        material=STAINLESS,
        name="front_rail",
    )
    canopy.visual(
        Box((0.760, 0.006, 0.064)),
        origin=Origin(xyz=(0.0, 0.473, 0.105)),
        material=DARK,
        name="control_insert",
    )
    canopy.visual(
        Box((1.10, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.295, 0.070)),
        material=STAINLESS,
        name="rear_hinge_ledge",
    )
    canopy.visual(
        Box((1.10, 0.065, 0.020)),
        origin=Origin(xyz=(0.0, -0.326, 0.065)),
        material=STAINLESS,
        name="hinge_backer",
    )
    canopy.visual(
        Box((0.026, 0.720, 0.024)),
        origin=Origin(xyz=(0.0, 0.040, 0.026)),
        material=STAINLESS,
        name="center_spine",
    )

    button_xs = (-0.30, -0.15, 0.0, 0.15, 0.30)
    for idx, x_pos in enumerate(button_xs):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.078, 0.018, 0.034)),
            origin=Origin(xyz=(0.0, 0.009, 0.0)),
            material=BUTTON,
            name="cap",
        )
        model.articulation(
            f"button_slide_{idx}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(x_pos, 0.476, 0.105)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.012),
        )

    for idx, x_center in enumerate((-0.285, 0.285)):
        filt = model.part(f"filter_{idx}")
        filt.visual(
            Cylinder(radius=0.010, length=0.500),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=FILTER,
            name="hinge_barrel",
        )
        filt.visual(
            Box((0.520, 0.030, 0.018)),
            origin=Origin(xyz=(0.0, 0.024, -0.002)),
            material=FILTER,
            name="rear_rail",
        )
        filt.visual(
            Box((0.520, 0.036, 0.020)),
            origin=Origin(xyz=(0.0, 0.532, -0.002)),
            material=FILTER,
            name="front_lip",
        )
        filt.visual(
            Box((0.026, 0.520, 0.018)),
            origin=Origin(xyz=(-0.247, 0.278, -0.004)),
            material=FILTER,
            name="side_rail_0",
        )
        filt.visual(
            Box((0.026, 0.520, 0.018)),
            origin=Origin(xyz=(0.247, 0.278, -0.004)),
            material=FILTER,
            name="side_rail_1",
        )
        for rib_idx, rib_x in enumerate((-0.165, -0.055, 0.055, 0.165)):
            filt.visual(
                Box((0.045, 0.502, 0.020)),
                origin=Origin(xyz=(rib_x, 0.278, -0.006)),
                material=FILTER,
                name=f"baffle_{rib_idx}",
            )
        filt.visual(
            Box((0.450, 0.470, 0.004)),
            origin=Origin(xyz=(0.0, 0.285, -0.017)),
            material=DARK,
            name="shadow_gap",
        )

        model.articulation(
            f"filter_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=filt,
            origin=Origin(xyz=(x_center, -0.270, 0.070)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")

    button_parts = [object_model.get_part(f"button_{i}") for i in range(5)]
    button_joints = [object_model.get_articulation(f"button_slide_{i}") for i in range(5)]
    filter_parts = [object_model.get_part(f"filter_{i}") for i in range(2)]
    filter_joints = [object_model.get_articulation(f"filter_hinge_{i}") for i in range(2)]

    ctx.check(
        "five independent button sliders",
        len(button_parts) == 5
        and len(button_joints) == 5
        and all(j.motion_limits is not None and j.motion_limits.upper == 0.012 for j in button_joints),
        details="Expected five separate 12 mm prismatic button controls.",
    )
    ctx.check(
        "two hinged baffle filters",
        len(filter_parts) == 2 and len(filter_joints) == 2,
        details="Expected exactly two rear-hinged filter panels.",
    )

    for i, button in enumerate(button_parts):
        ctx.expect_gap(
            button,
            canopy,
            axis="y",
            min_gap=0.0,
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="cap",
            negative_elem="control_insert",
            name=f"button_{i} rests proud on rail",
        )

    rest_positions = [ctx.part_world_position(button) for button in button_parts]
    for i, joint in enumerate(button_joints):
        with ctx.pose({joint: 0.012}):
            pressed_positions = [ctx.part_world_position(button) for button in button_parts]
        moved = (
            rest_positions[i] is not None
            and pressed_positions[i] is not None
            and pressed_positions[i][1] < rest_positions[i][1] - 0.010
        )
        others_stationary = all(
            j == i
            or (
                rest_positions[j] is not None
                and pressed_positions[j] is not None
                and abs(pressed_positions[j][1] - rest_positions[j][1]) < 1e-6
            )
            for j in range(5)
        )
        ctx.check(
            f"button_{i} depresses independently",
            moved and others_stationary,
            details=f"rest={rest_positions}, pressed={pressed_positions}",
        )

    for i, (filt, joint) in enumerate(zip(filter_parts, filter_joints)):
        closed_aabb = ctx.part_element_world_aabb(filt, elem="front_lip")
        with ctx.pose({joint: 1.05}):
            open_aabb = ctx.part_element_world_aabb(filt, elem="front_lip")
        closed_z = closed_aabb[0][2] if closed_aabb is not None else None
        open_z = open_aabb[0][2] if open_aabb is not None else None
        ctx.check(
            f"filter_{i} rotates downward",
            closed_z is not None and open_z is not None and open_z < closed_z - 0.35,
            details=f"closed_front_lip={closed_aabb}, open_front_lip={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

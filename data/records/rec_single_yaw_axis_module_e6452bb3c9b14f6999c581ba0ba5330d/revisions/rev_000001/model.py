from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_pan_stage")

    painted_steel = Material("painted_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    blackened = Material("blackened_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    tool_blue = Material("anodized_tool_plate", rgba=(0.05, 0.16, 0.32, 1.0))
    orange = Material("orange_index_mark", rgba=(1.0, 0.45, 0.08, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.72, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=painted_steel,
        name="top_beam",
    )
    support.visual(
        Box((0.24, 0.12, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.6325)),
        material=painted_steel,
        name="underside_boss",
    )
    for x, name in ((-0.095, "hanger_0"), (0.095, "hanger_1")):
        support.visual(
            Box((0.045, 0.038, 0.31)),
            origin=Origin(xyz=(x, 0.0, 0.475)),
            material=painted_steel,
            name=name,
        )
    for x, name in ((-0.095, "bearing_side_0"), (0.095, "bearing_side_1")):
        support.visual(
            Box((0.045, 0.15, 0.085)),
            origin=Origin(xyz=(x, 0.0, 0.31)),
            material=dark_steel,
            name=name,
        )
    for y, name in ((-0.090, "bearing_rail_0"), (0.090, "bearing_rail_1")):
        support.visual(
            Box((0.235, 0.035, 0.085)),
            origin=Origin(xyz=(0.0, y, 0.31)),
            material=dark_steel,
            name=name,
        )
    for x, name in ((-0.25, "bolt_0"), (0.25, "bolt_1")):
        support.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.745)),
            material=blackened,
            name=name,
        )

    rotary_head = model.part("rotary_head")
    rotary_head.visual(
        Cylinder(radius=0.032, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blackened,
        name="spindle",
    )
    rotary_head.visual(
        Cylinder(radius=0.075, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=dark_steel,
        name="rotary_hub",
    )
    rotary_head.visual(
        Cylinder(radius=0.052, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.1375)),
        material=dark_steel,
        name="plate_boss",
    )

    slot = rounded_rect_profile(0.125, 0.026, 0.013, corner_segments=8)
    rotary_head.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.26, 0.18, 0.022, corner_segments=10),
                [
                    _translated_profile(slot[::-1], 0.0, -0.045),
                    _translated_profile(slot[::-1], 0.0, 0.045),
                ],
                0.025,
                center=True,
            ),
            "tooling_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=tool_blue,
        name="tooling_plate",
    )
    rotary_head.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(0.105, 0.0, -0.159)),
        material=orange,
        name="index_tab",
    )
    for x, y, name in (
        (-0.095, -0.065, "plate_screw_0"),
        (0.095, -0.065, "plate_screw_1"),
        (-0.095, 0.065, "plate_screw_2"),
        (0.095, 0.065, "plate_screw_3"),
    ):
        rotary_head.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(x, y, -0.1615)),
            material=blackened,
            name=name,
        )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=rotary_head,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    rotary_head = object_model.get_part("rotary_head")
    pan_axis = object_model.get_articulation("pan_axis")

    ctx.check(
        "single vertical revolute pan joint",
        len(object_model.articulations) == 1
        and pan_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(pan_axis.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        rotary_head,
        support,
        axes="z",
        min_overlap=0.06,
        elem_a="spindle",
        elem_b="bearing_side_0",
        name="spindle passes through fixed bearing height",
    )
    ctx.expect_gap(
        support,
        rotary_head,
        axis="x",
        min_gap=0.035,
        positive_elem="bearing_side_1",
        negative_elem="spindle",
        name="spindle clears the positive bearing cheek",
    )
    ctx.expect_gap(
        rotary_head,
        support,
        axis="x",
        min_gap=0.035,
        positive_elem="spindle",
        negative_elem="bearing_side_0",
        name="spindle clears the negative bearing cheek",
    )
    ctx.expect_gap(
        support,
        rotary_head,
        axis="z",
        min_gap=0.45,
        positive_elem="top_beam",
        negative_elem="tooling_plate",
        name="fixed top beam sits above the moving tooling plate",
    )

    rest_aabb = ctx.part_element_world_aabb(rotary_head, elem="index_tab")
    rest_center = (
        (
            (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5,
            (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5,
        )
        if rest_aabb is not None
        else None
    )
    with ctx.pose({pan_axis: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotary_head, elem="index_tab")
        turned_center = (
            (
                (turned_aabb[0][0] + turned_aabb[1][0]) * 0.5,
                (turned_aabb[0][1] + turned_aabb[1][1]) * 0.5,
            )
            if turned_aabb is not None
            else None
        )
    ctx.check(
        "tooling plate rotates about the hanging vertical axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.09
        and abs(rest_center[1]) < 0.02
        and turned_center[1] > 0.09
        and abs(turned_center[0]) < 0.03,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()

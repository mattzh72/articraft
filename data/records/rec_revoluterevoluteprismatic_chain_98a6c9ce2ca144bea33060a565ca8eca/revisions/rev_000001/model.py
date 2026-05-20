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
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 32,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    if clockwise:
        angles = [2.0 * math.pi * i / segments for i in range(segments, 0, -1)]
    else:
        angles = [2.0 * math.pi * i / segments for i in range(segments)]
    return [(cx + radius * math.cos(a), cy + radius * math.sin(a)) for a in angles]


def _stadium_profile(length: float, width: float, *, segments: int = 24) -> list[tuple[float, float]]:
    """Capsule outline with end-hole centers at x=0 and x=length."""

    radius = width * 0.5
    right = [
        (
            length + radius * math.cos(-math.pi / 2.0 + math.pi * i / segments),
            radius * math.sin(-math.pi / 2.0 + math.pi * i / segments),
        )
        for i in range(segments + 1)
    ]
    left = [
        (
            radius * math.cos(math.pi / 2.0 + math.pi * i / segments),
            radius * math.sin(math.pi / 2.0 + math.pi * i / segments),
        )
        for i in range(segments + 1)
    ]
    return [(0.0, -radius), (length, -radius), *right, (0.0, radius), *left]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_elbow_arm")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    blue_anodized = Material("blue_anodized_aluminum", rgba=(0.10, 0.22, 0.42, 1.0))
    black_steel = Material("blackened_steel", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = Material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))

    model.materials.extend([dark_anodized, blue_anodized, black_steel, satin_steel, rubber])

    link_width = 0.105
    link_thickness = 0.018
    inner_length = 0.43
    outer_length = 0.34
    pivot_hole_radius = 0.022

    inner_plate = ExtrudeWithHolesGeometry(
        _stadium_profile(inner_length, link_width),
        [
            _circle_profile(0.0, 0.0, pivot_hole_radius, clockwise=True),
            _circle_profile(inner_length, 0.0, pivot_hole_radius, clockwise=True),
        ],
        link_thickness,
    )
    outer_plate = ExtrudeWithHolesGeometry(
        _stadium_profile(outer_length, link_width),
        [_circle_profile(0.0, 0.0, pivot_hole_radius, clockwise=True)],
        link_thickness,
    )

    base = model.part("base")
    base.visual(
        Box((0.46, 0.28, 0.030)),
        origin=Origin(xyz=(0.09, 0.0, 0.015)),
        material=dark_anodized,
        name="floor_plate",
    )
    base.visual(
        Box((0.34, 0.17, 0.016)),
        origin=Origin(xyz=(0.02, 0.0, 0.037)),
        material=dark_anodized,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=satin_steel,
        name="slew_bearing",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=black_steel,
        name="base_pivot_pin",
    )
    for x, y, idx in (
        (-0.105, -0.095, 0),
        (-0.105, 0.095, 1),
        (0.275, -0.095, 2),
        (0.275, 0.095, 3),
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )

    inner_link = model.part("inner_link")
    inner_link.visual(
        mesh_from_geometry(inner_plate, "inner_link_plate"),
        origin=Origin(),
        material=blue_anodized,
        name="inner_plate",
    )
    inner_link.visual(
        mesh_from_geometry(TorusGeometry(radius=0.026, tube=0.004), "base_thrust_washer"),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=satin_steel,
        name="base_thrust_washer",
    )
    inner_link.visual(
        Cylinder(radius=0.014, length=0.094),
        origin=Origin(xyz=(inner_length, 0.0, 0.047)),
        material=black_steel,
        name="elbow_pivot_pin",
    )
    inner_link.visual(
        mesh_from_geometry(TorusGeometry(radius=0.025, tube=0.004), "elbow_lower_washer"),
        origin=Origin(xyz=(inner_length, 0.0, 0.013)),
        material=satin_steel,
        name="elbow_lower_washer",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_geometry(outer_plate, "outer_link_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=blue_anodized,
        name="outer_plate",
    )
    outer_link.visual(
        mesh_from_geometry(TorusGeometry(radius=0.025, tube=0.004), "elbow_upper_washer"),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=satin_steel,
        name="elbow_upper_washer",
    )
    guide_center_x = outer_length + 0.002
    outer_link.visual(
        Box((0.190, 0.012, 0.038)),
        origin=Origin(xyz=(guide_center_x, 0.038, 0.054)),
        material=dark_anodized,
        name="guide_side_pos",
    )
    outer_link.visual(
        Box((0.190, 0.012, 0.038)),
        origin=Origin(xyz=(guide_center_x, -0.038, 0.054)),
        material=dark_anodized,
        name="guide_side_neg",
    )
    outer_link.visual(
        Box((0.130, 0.085, 0.010)),
        origin=Origin(xyz=(outer_length - 0.012, 0.0, 0.077)),
        material=dark_anodized,
        name="guide_top_keeper",
    )
    outer_link.visual(
        Box((0.046, 0.074, 0.022)),
        origin=Origin(xyz=(outer_length - 0.080, 0.0, 0.028)),
        material=dark_anodized,
        name="guide_mount_block",
    )

    tool_head = model.part("tool_head")
    tool_head.visual(
        Box((0.170, 0.042, 0.024)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=black_steel,
        name="ram",
    )
    tool_head.visual(
        Box((0.150, 0.012, 0.022)),
        origin=Origin(xyz=(-0.035, 0.026, 0.0)),
        material=satin_steel,
        name="rail_glide_pos",
    )
    tool_head.visual(
        Box((0.150, 0.012, 0.022)),
        origin=Origin(xyz=(-0.035, -0.026, 0.0)),
        material=satin_steel,
        name="rail_glide_neg",
    )
    tool_head.visual(
        Box((0.044, 0.056, 0.034)),
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
        material=satin_steel,
        name="nose_block",
    )
    tool_head.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="tool_face",
    )

    base_to_inner = model.articulation(
        "base_to_inner",
        ArticulationType.REVOLUTE,
        parent=base,
        child=inner_link,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-1.75, upper=1.75),
    )
    inner_to_outer = model.articulation(
        "inner_to_outer",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=outer_link,
        origin=Origin(xyz=(inner_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    outer_to_tool = model.articulation(
        "outer_to_tool",
        ArticulationType.PRISMATIC,
        parent=outer_link,
        child=tool_head,
        origin=Origin(xyz=(outer_length + 0.052, 0.0, 0.053)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    model.meta["primary_chain"] = (base_to_inner.name, inner_to_outer.name, outer_to_tool.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    tool_head = object_model.get_part("tool_head")
    base_to_inner = object_model.get_articulation("base_to_inner")
    inner_to_outer = object_model.get_articulation("inner_to_outer")
    outer_to_tool = object_model.get_articulation("outer_to_tool")

    ctx.allow_overlap(
        base,
        inner_link,
        elem_a="base_pivot_pin",
        elem_b="inner_plate",
        reason="The base pivot pin is intentionally captured through the first link's bored pivot eye.",
    )
    ctx.allow_overlap(
        inner_link,
        outer_link,
        elem_a="elbow_pivot_pin",
        elem_b="outer_plate",
        reason="The elbow pivot pin is intentionally captured through the outer link's bored pivot eye.",
    )

    ctx.check(
        "two revolutes then one prismatic",
        (
            base_to_inner.articulation_type == ArticulationType.REVOLUTE
            and inner_to_outer.articulation_type == ArticulationType.REVOLUTE
            and outer_to_tool.articulation_type == ArticulationType.PRISMATIC
        ),
        details=(
            f"types: {base_to_inner.articulation_type}, "
            f"{inner_to_outer.articulation_type}, {outer_to_tool.articulation_type}"
        ),
    )
    ctx.check(
        "serial elbow chain",
        (
            base_to_inner.parent == "base"
            and base_to_inner.child == "inner_link"
            and inner_to_outer.parent == "inner_link"
            and inner_to_outer.child == "outer_link"
            and outer_to_tool.parent == "outer_link"
            and outer_to_tool.child == "tool_head"
        ),
        details=(
            f"{base_to_inner.parent}->{base_to_inner.child}, "
            f"{inner_to_outer.parent}->{inner_to_outer.child}, "
            f"{outer_to_tool.parent}->{outer_to_tool.child}"
        ),
    )
    ctx.expect_overlap(
        inner_link,
        base,
        axes="xy",
        elem_a="inner_plate",
        elem_b="slew_bearing",
        min_overlap=0.035,
        name="inner link is supported over base bearing",
    )
    ctx.expect_overlap(
        base,
        inner_link,
        axes="z",
        elem_a="base_pivot_pin",
        elem_b="inner_plate",
        min_overlap=0.010,
        name="base pivot pin passes through inner pivot eye",
    )
    ctx.expect_gap(
        inner_link,
        base,
        axis="z",
        positive_elem="inner_plate",
        negative_elem="slew_bearing",
        min_gap=0.003,
        max_gap=0.020,
        name="inner link rides just above base bearing",
    )
    ctx.expect_overlap(
        outer_link,
        inner_link,
        axes="xy",
        elem_a="outer_plate",
        elem_b="elbow_pivot_pin",
        min_overlap=0.025,
        name="outer link is centered on elbow pivot",
    )
    ctx.expect_overlap(
        inner_link,
        outer_link,
        axes="z",
        elem_a="elbow_pivot_pin",
        elem_b="outer_plate",
        min_overlap=0.010,
        name="elbow pin passes through outer pivot eye",
    )
    ctx.expect_within(
        tool_head,
        outer_link,
        axes="yz",
        inner_elem="ram",
        margin=0.001,
        name="ram cross-section stays inside guide envelope",
    )
    ctx.expect_overlap(
        tool_head,
        outer_link,
        axes="x",
        elem_a="ram",
        elem_b="guide_side_pos",
        min_overlap=0.090,
        name="collapsed ram remains deeply inserted in guide",
    )
    ctx.expect_gap(
        outer_link,
        tool_head,
        axis="z",
        positive_elem="guide_top_keeper",
        negative_elem="ram",
        min_gap=0.004,
        max_gap=0.012,
        name="top keeper clears ram",
    )
    ctx.expect_gap(
        outer_link,
        tool_head,
        axis="y",
        positive_elem="guide_side_pos",
        negative_elem="ram",
        min_gap=0.006,
        max_gap=0.018,
        name="positive guide side clears ram",
    )
    ctx.expect_gap(
        tool_head,
        outer_link,
        axis="y",
        positive_elem="ram",
        negative_elem="guide_side_neg",
        min_gap=0.006,
        max_gap=0.018,
        name="negative guide side clears ram",
    )

    rest_tool_pos = ctx.part_world_position(tool_head)
    with ctx.pose({outer_to_tool: 0.120}):
        extended_tool_pos = ctx.part_world_position(tool_head)
        ctx.expect_overlap(
            tool_head,
            outer_link,
            axes="x",
            elem_a="ram",
            elem_b="guide_side_pos",
            min_overlap=0.043,
            name="extended ram retains insertion in guide",
        )
        ctx.expect_within(
            tool_head,
            outer_link,
            axes="yz",
            inner_elem="ram",
            margin=0.001,
            name="extended ram stays in guide envelope",
        )
    ctx.check(
        "prismatic tool head extends outward",
        (
            rest_tool_pos is not None
            and extended_tool_pos is not None
            and extended_tool_pos[0] > rest_tool_pos[0] + 0.10
        ),
        details=f"rest={rest_tool_pos}, extended={extended_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()

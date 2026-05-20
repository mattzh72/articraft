from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_rect_tube(
    part,
    *,
    prefix: str,
    x_min: float,
    x_max: float,
    width: float,
    height: float,
    wall: float,
    material: str,
) -> None:
    """Build an open-ended rectangular box-section tube along local +X."""
    length = x_max - x_min
    cx = 0.5 * (x_min + x_max)

    # Top and bottom flanges span the full width; side webs fill the gap between
    # them so the four primitives read as one continuous hollow extrusion.
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(cx, 0.0, 0.5 * height - 0.5 * wall)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(cx, 0.0, -0.5 * height + 0.5 * wall)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    part.visual(
        Box((length, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(cx, 0.5 * width - 0.5 * wall, 0.0)),
        material=material,
        name=f"{prefix}_side_wall_0",
    )
    part.visual(
        Box((length, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(cx, -0.5 * width + 0.5 * wall, 0.0)),
        material=material,
        name=f"{prefix}_side_wall_1",
    )


def _add_guide_pads(
    part,
    *,
    prefix: str,
    x_center: float,
    length: float,
    cavity_width: float,
    cavity_height: float,
    pad_thickness: float,
    side_pad_height: float,
    top_pad_width: float,
    material: str,
) -> None:
    """Add four low-friction bearing pads just inside a box-section mouth."""
    side_y = 0.5 * cavity_width - 0.5 * pad_thickness
    top_z = 0.5 * cavity_height - 0.5 * pad_thickness

    part.visual(
        Box((length, pad_thickness, side_pad_height)),
        origin=Origin(xyz=(x_center, side_y, 0.0)),
        material=material,
        name=f"{prefix}_side_pad_0",
    )
    part.visual(
        Box((length, pad_thickness, side_pad_height)),
        origin=Origin(xyz=(x_center, -side_y, 0.0)),
        material=material,
        name=f"{prefix}_side_pad_1",
    )
    part.visual(
        Box((length, top_pad_width, pad_thickness)),
        origin=Origin(xyz=(x_center, 0.0, top_z)),
        material=material,
        name=f"{prefix}_top_pad",
    )
    part.visual(
        Box((length, top_pad_width, pad_thickness)),
        origin=Origin(xyz=(x_center, 0.0, -top_z)),
        material=material,
        name=f"{prefix}_bottom_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_boom")

    model.material("outer_anodized_aluminum", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("middle_brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("inner_dark_aluminum", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("guide_pad_polymer", rgba=(0.92, 0.86, 0.68, 1.0))
    model.material("black_hardware", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("tool_head_blue", rgba=(0.10, 0.22, 0.48, 1.0))
    model.material("tool_steel", rgba=(0.62, 0.64, 0.66, 1.0))

    outer_stage = model.part("outer_stage")
    _add_rect_tube(
        outer_stage,
        prefix="outer",
        x_min=0.0,
        x_max=1.15,
        width=0.180,
        height=0.220,
        wall=0.018,
        material="outer_anodized_aluminum",
    )
    _add_guide_pads(
        outer_stage,
        prefix="outer_front",
        x_center=1.075,
        length=0.150,
        cavity_width=0.144,
        cavity_height=0.184,
        pad_thickness=0.009,
        side_pad_height=0.060,
        top_pad_width=0.070,
        material="guide_pad_polymer",
    )
    _add_guide_pads(
        outer_stage,
        prefix="outer_rear",
        x_center=0.105,
        length=0.130,
        cavity_width=0.144,
        cavity_height=0.184,
        pad_thickness=0.009,
        side_pad_height=0.052,
        top_pad_width=0.062,
        material="guide_pad_polymer",
    )
    # Compact rear mounting saddle, integrated with the outer tube.
    outer_stage.visual(
        Box((0.380, 0.250, 0.026)),
        origin=Origin(xyz=(0.190, 0.0, -0.123)),
        material="black_hardware",
        name="mount_base_plate",
    )
    outer_stage.visual(
        Box((0.150, 0.030, 0.165)),
        origin=Origin(xyz=(0.135, 0.105, -0.040)),
        material="black_hardware",
        name="mount_lug_0",
    )
    outer_stage.visual(
        Box((0.150, 0.030, 0.165)),
        origin=Origin(xyz=(0.135, -0.105, -0.040)),
        material="black_hardware",
        name="mount_lug_1",
    )
    outer_stage.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.135, 0.122, -0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="outer_anodized_aluminum",
        name="pivot_boss_0",
    )
    outer_stage.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.135, -0.122, -0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="outer_anodized_aluminum",
        name="pivot_boss_1",
    )

    middle_stage = model.part("middle_stage")
    _add_rect_tube(
        middle_stage,
        prefix="middle",
        x_min=-0.950,
        x_max=0.180,
        width=0.126,
        height=0.166,
        wall=0.014,
        material="middle_brushed_aluminum",
    )
    _add_guide_pads(
        middle_stage,
        prefix="middle_front",
        x_center=0.110,
        length=0.120,
        cavity_width=0.098,
        cavity_height=0.138,
        pad_thickness=0.007,
        side_pad_height=0.044,
        top_pad_width=0.052,
        material="guide_pad_polymer",
    )
    middle_stage.visual(
        Box((0.030, 0.126, 0.166)),
        origin=Origin(xyz=(-0.950, 0.0, 0.0)),
        material="black_hardware",
        name="rear_stop_band",
    )
    _add_rect_tube(
        middle_stage,
        prefix="front_band",
        x_min=0.165,
        x_max=0.195,
        width=0.126,
        height=0.166,
        wall=0.014,
        material="black_hardware",
    )

    inner_stage = model.part("inner_stage")
    _add_rect_tube(
        inner_stage,
        prefix="inner",
        x_min=-0.750,
        x_max=0.130,
        width=0.084,
        height=0.116,
        wall=0.010,
        material="inner_dark_aluminum",
    )
    inner_stage.visual(
        Box((0.026, 0.084, 0.116)),
        origin=Origin(xyz=(-0.750, 0.0, 0.0)),
        material="black_hardware",
        name="inner_rear_stop",
    )
    inner_stage.visual(
        Box((0.026, 0.084, 0.116)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material="black_hardware",
        name="inner_front_cap",
    )

    tool_head = model.part("tool_head")
    tool_head.visual(
        Box((0.080, 0.130, 0.132)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material="tool_head_blue",
        name="mount_block",
    )
    tool_head.visual(
        Box((0.110, 0.030, 0.150)),
        origin=Origin(xyz=(0.115, 0.060, 0.0)),
        material="tool_head_blue",
        name="fork_cheek_0",
    )
    tool_head.visual(
        Box((0.110, 0.030, 0.150)),
        origin=Origin(xyz=(0.115, -0.060, 0.0)),
        material="tool_head_blue",
        name="fork_cheek_1",
    )
    tool_head.visual(
        Cylinder(radius=0.032, length=0.160),
        origin=Origin(xyz=(0.165, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tool_steel",
        name="tool_cross_pin",
    )
    tool_head.visual(
        Box((0.050, 0.102, 0.018)),
        origin=Origin(xyz=(0.170, 0.0, -0.084)),
        material="tool_steel",
        name="flat_tool_tip",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(xyz=(1.150, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.620),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.35, lower=0.0, upper=0.520),
    )
    model.articulation(
        "inner_to_tool",
        ArticulationType.FIXED,
        parent=inner_stage,
        child=tool_head,
        origin=Origin(xyz=(0.143, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    tool = object_model.get_part("tool_head")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.004,
        name="middle stage is concentrically nested in outer section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.004,
        name="inner stage is concentrically nested in middle section",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.300,
        name="middle stage retains insertion when compact",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.250,
        name="inner stage retains insertion when compact",
    )
    ctx.expect_contact(
        tool,
        inner,
        elem_a="mount_block",
        elem_b="inner_front_cap",
        contact_tol=0.001,
        name="tool head mounts directly to inner stage nose",
    )

    collapsed_tool_pos = ctx.part_world_position(tool)
    with ctx.pose({outer_slide: 0.620, inner_slide: 0.520}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.004,
            name="middle stage stays centered at full extension",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.004,
            name="inner stage stays centered at full extension",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.250,
            name="middle stage remains captured at full extension",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.180,
            name="inner stage remains captured at full extension",
        )
        extended_tool_pos = ctx.part_world_position(tool)

    ctx.check(
        "tool head extends outward along the boom axis",
        collapsed_tool_pos is not None
        and extended_tool_pos is not None
        and extended_tool_pos[0] > collapsed_tool_pos[0] + 1.10,
        details=f"collapsed={collapsed_tool_pos}, extended={extended_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()

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


LINK_LENGTH = 0.24
INNER_DEPTH = 0.096
OUTER_DEPTH = 0.066
INNER_WIDTH = 0.100
OUTER_WIDTH = 0.078
PIVOT_Z = 0.105


def _add_y_cylinder(part, *, name, x, y, z, radius, length, material):
    """Add a cylinder whose axis runs along the model's Y direction."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _knuckle_segments(pattern: str, width: float) -> list[tuple[float, float]]:
    if pattern == "none":
        return []
    if pattern == "center":
        return [(0.0, 0.038)]
    if pattern == "outer":
        seg_len = 0.024
        center = width / 2.0 - seg_len / 2.0
        return [(-center, seg_len), (center, seg_len)]
    raise ValueError(f"unknown knuckle pattern {pattern!r}")


def _add_ladder_link(
    part,
    *,
    length: float,
    depth: float,
    width: float,
    start_pattern: str,
    end_pattern: str,
    frame_material,
    pin_material,
    end_pad: bool = False,
) -> None:
    rail_h = 0.018
    cross_x = 0.028
    cross_z = depth + rail_h
    rail_len = length - 0.070

    part.visual(
        Box((rail_len, width, rail_h)),
        origin=Origin(xyz=(length / 2.0, 0.0, depth / 2.0)),
        material=frame_material,
        name="upper_rail",
    )
    part.visual(
        Box((rail_len, width, rail_h)),
        origin=Origin(xyz=(length / 2.0, 0.0, -depth / 2.0)),
        material=frame_material,
        name="lower_rail",
    )
    part.visual(
        Box((cross_x, width, cross_z)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=frame_material,
        name="start_crosshead",
    )
    part.visual(
        Box((cross_x, width, cross_z)),
        origin=Origin(xyz=(length - 0.035, 0.0, 0.0)),
        material=frame_material,
        name="end_crosshead",
    )

    start_segments = _knuckle_segments(start_pattern, width)
    end_segments = _knuckle_segments(end_pattern, width)

    for idx, (y, seg_len) in enumerate(start_segments):
        suffix = "" if len(start_segments) == 1 else f"_{idx}"
        _add_y_cylinder(
            part,
            name=f"start_barrel{suffix}",
            x=0.0,
            y=y,
            z=0.0,
            radius=0.015,
            length=seg_len,
            material=pin_material,
        )
        part.visual(
            Box((0.026, seg_len, 0.030)),
            origin=Origin(xyz=(0.020, y, 0.0)),
            material=frame_material,
            name=f"start_web{suffix}",
        )

    if end_segments:
        _add_y_cylinder(
            part,
            name="end_pin",
            x=length,
            y=0.0,
            z=0.0,
            radius=0.005,
            length=width + 0.024,
            material=pin_material,
        )

    for idx, (y, seg_len) in enumerate(end_segments):
        suffix = "" if len(end_segments) == 1 else f"_{idx}"
        _add_y_cylinder(
            part,
            name=f"end_barrel{suffix}",
            x=length,
            y=y,
            z=0.0,
            radius=0.015,
            length=seg_len,
            material=pin_material,
        )
        part.visual(
            Box((0.026, seg_len, 0.030)),
            origin=Origin(xyz=(length - 0.020, y, 0.0)),
            material=frame_material,
            name=f"end_web{suffix}",
        )

    if end_pad:
        part.visual(
            Box((0.026, width, depth + 0.026)),
            origin=Origin(xyz=(length - 0.013, 0.0, 0.0)),
            material=frame_material,
            name="tip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_foldout_arm")

    base_mat = model.material("blackened_base", rgba=(0.055, 0.060, 0.065, 1.0))
    inner_mat = model.material("deep_blue_powdercoat", rgba=(0.05, 0.16, 0.34, 1.0))
    outer_mat = model.material("blue_gray_powdercoat", rgba=(0.30, 0.38, 0.44, 1.0))
    platform_mat = model.material("brushed_platform", rgba=(0.62, 0.64, 0.61, 1.0))
    pin_mat = model.material("dark_pin_steel", rgba=(0.015, 0.015, 0.017, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((0.250, 0.180, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, 0.009)),
        material=base_mat,
        name="ground_plate",
    )
    base.visual(
        Box((0.055, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, -0.070, 0.060)),
        material=base_mat,
        name="yoke_cheek_0",
    )
    base.visual(
        Box((0.055, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, 0.070, 0.060)),
        material=base_mat,
        name="yoke_cheek_1",
    )
    _add_y_cylinder(
        base,
        name="base_barrel_0",
        x=0.0,
        y=-0.070,
        z=PIVOT_Z,
        radius=0.016,
        length=0.024,
        material=pin_mat,
    )
    _add_y_cylinder(
        base,
        name="base_barrel_1",
        x=0.0,
        y=0.070,
        z=PIVOT_Z,
        radius=0.016,
        length=0.024,
        material=pin_mat,
    )
    _add_y_cylinder(
        base,
        name="base_pin",
        x=0.0,
        y=0.0,
        z=PIVOT_Z,
        radius=0.005,
        length=0.160,
        material=pin_mat,
    )

    inner_link_0 = model.part("inner_link_0")
    _add_ladder_link(
        inner_link_0,
        length=LINK_LENGTH,
        depth=INNER_DEPTH,
        width=INNER_WIDTH,
        start_pattern="center",
        end_pattern="outer",
        frame_material=inner_mat,
        pin_material=pin_mat,
    )

    inner_link_1 = model.part("inner_link_1")
    _add_ladder_link(
        inner_link_1,
        length=LINK_LENGTH,
        depth=INNER_DEPTH,
        width=INNER_WIDTH,
        start_pattern="center",
        end_pattern="outer",
        frame_material=inner_mat,
        pin_material=pin_mat,
    )

    outer_link_0 = model.part("outer_link_0")
    _add_ladder_link(
        outer_link_0,
        length=LINK_LENGTH,
        depth=OUTER_DEPTH,
        width=OUTER_WIDTH,
        start_pattern="center",
        end_pattern="outer",
        frame_material=outer_mat,
        pin_material=pin_mat,
    )

    outer_link_1 = model.part("outer_link_1")
    _add_ladder_link(
        outer_link_1,
        length=LINK_LENGTH,
        depth=OUTER_DEPTH,
        width=OUTER_WIDTH,
        start_pattern="center",
        end_pattern="none",
        frame_material=outer_mat,
        pin_material=pin_mat,
        end_pad=True,
    )

    platform = model.part("platform_bracket")
    platform.visual(
        Box((0.030, OUTER_WIDTH, 0.074)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=platform_mat,
        name="mount_block",
    )
    platform.visual(
        Box((0.115, 0.090, 0.014)),
        origin=Origin(xyz=(0.0875, 0.0, 0.044)),
        material=platform_mat,
        name="tool_plate",
    )
    platform.visual(
        Box((0.030, 0.014, 0.065)),
        origin=Origin(xyz=(0.073, -0.045, 0.018)),
        material=platform_mat,
        name="side_flange_0",
    )
    platform.visual(
        Box((0.030, 0.014, 0.065)),
        origin=Origin(xyz=(0.073, 0.045, 0.018)),
        material=platform_mat,
        name="side_flange_1",
    )
    _add_y_cylinder(
        platform,
        name="platform_boss",
        x=0.018,
        y=0.0,
        z=0.0,
        radius=0.016,
        length=0.060,
        material=pin_mat,
    )

    joint_limits = MotionLimits(effort=20.0, velocity=2.0, lower=-1.35, upper=1.35)
    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=inner_link_0,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "elbow_pivot_0",
        ArticulationType.REVOLUTE,
        parent=inner_link_0,
        child=inner_link_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "elbow_pivot_1",
        ArticulationType.REVOLUTE,
        parent=inner_link_1,
        child=outer_link_0,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "wrist_pivot",
        ArticulationType.REVOLUTE,
        parent=outer_link_0,
        child=outer_link_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "platform_mount",
        ArticulationType.FIXED,
        parent=outer_link_1,
        child=platform,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    revolutes = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "four serial revolute pivots",
        len(revolutes) == 4,
        details=f"found {[joint.name for joint in revolutes]}",
    )
    ctx.check(
        "pivot axes are parallel in one plane",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0) for joint in revolutes),
        details=f"axes={[joint.axis for joint in revolutes]}",
    )

    inner_link = object_model.get_part("inner_link_0")
    outer_link = object_model.get_part("outer_link_0")
    inner_depth = inner_link.get_visual("start_crosshead").geometry.size[2]
    outer_depth = outer_link.get_visual("start_crosshead").geometry.size[2]
    ctx.check(
        "inner boxed links are deeper",
        inner_depth > outer_depth + 0.020,
        details=f"inner_depth={inner_depth}, outer_depth={outer_depth}",
    )

    last_link = object_model.get_part("outer_link_1")
    platform = object_model.get_part("platform_bracket")
    ctx.expect_contact(
        platform,
        last_link,
        elem_a="mount_block",
        elem_b="tip_pad",
        contact_tol=0.001,
        name="platform bracket is fixed to the last link",
    )

    hinge_interfaces = [
        ("base_plate", "inner_link_0", "base_pin", "start_barrel"),
        ("inner_link_0", "inner_link_1", "end_pin", "start_barrel"),
        ("inner_link_1", "outer_link_0", "end_pin", "start_barrel"),
        ("outer_link_0", "outer_link_1", "end_pin", "start_barrel"),
    ]
    for parent_name, child_name, pin_name, barrel_name in hinge_interfaces:
        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a=pin_name,
            elem_b=barrel_name,
            reason="The dark hinge pin is intentionally shown captured through the boxed link's bore.",
        )
        ctx.expect_overlap(
            parent_name,
            child_name,
            axes="y",
            elem_a=pin_name,
            elem_b=barrel_name,
            min_overlap=0.030,
            name=f"{parent_name} pin captures {child_name} barrel",
        )

    wrist = object_model.get_articulation("wrist_pivot")
    rest_pos = ctx.part_world_position(platform)
    with ctx.pose({wrist: 0.75}):
        folded_pos = ctx.part_world_position(platform)
    ctx.check(
        "distal platform follows the revolute chain",
        rest_pos is not None
        and folded_pos is not None
        and folded_pos[2] < rest_pos[2] - 0.040,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()

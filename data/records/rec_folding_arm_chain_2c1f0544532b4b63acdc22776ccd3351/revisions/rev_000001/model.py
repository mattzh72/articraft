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


LINK_SECTION_Y = 0.024
LINK_SECTION_Z = 0.024
BOSS_RADIUS = 0.018
CHEEK_Y = 0.023
CHEEK_THICKNESS = 0.010
CHEEK_X = 0.075
CHEEK_Z = 0.060
BRIDGE_Y = 0.0382


def _add_distal_fork(part, length: float, material) -> None:
    """Compact two-cheek fork at the distal revolute axis of a slim link."""
    part.visual(
        Box((0.055, BRIDGE_Y, 0.030)),
        origin=Origin(xyz=(length - 0.0475, 0.0, 0.0)),
        material=material,
        name="distal_bridge",
    )
    part.visual(
        Box((CHEEK_X, CHEEK_THICKNESS, CHEEK_Z)),
        origin=Origin(xyz=(length, CHEEK_Y, 0.0)),
        material=material,
        name="distal_cheek_pos",
    )
    part.visual(
        Box((CHEEK_X, CHEEK_THICKNESS, CHEEK_Z)),
        origin=Origin(xyz=(length, -CHEEK_Y, 0.0)),
        material=material,
        name="distal_cheek_neg",
    )


def _add_arm_link(part, length: float, material, boss_material, *, terminal: bool = False, pad_material=None) -> None:
    """Simple rigid service-arm link with a proximal bearing boss."""
    bar_start = 0.012
    bar_end = length - (0.038 if terminal else 0.050)
    bar_length = bar_end - bar_start

    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=0.044),
        origin=Origin(),
        material=boss_material,
        name="prox_boss",
    )
    part.visual(
        Box((bar_length, LINK_SECTION_Y, LINK_SECTION_Z)),
        origin=Origin(xyz=(bar_start + bar_length / 2.0, 0.0, 0.0)),
        material=material,
        name="link_bar",
    )

    if terminal:
        part.visual(
            Box((0.078, 0.064, 0.018)),
            origin=Origin(xyz=(length + 0.021, 0.0, -0.003)),
            material=pad_material,
            name="end_pad",
        )
        part.visual(
            Box((0.038, 0.032, 0.022)),
            origin=Origin(xyz=(length - 0.021, 0.0, 0.0)),
            material=material,
            name="pad_neck",
        )
    else:
        _add_distal_fork(part, length, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_service_folding_arm_chain")

    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.095, 0.105, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.060, 0.055, 1.000)),
        origin=Origin(xyz=(-0.080, 0.0, -0.500)),
        material=dark_anodized,
        name="upright_spine",
    )
    spine.visual(
        Box((0.180, 0.110, 0.036)),
        origin=Origin(xyz=(-0.080, 0.0, -1.016)),
        material=dark_anodized,
        name="ground_foot",
    )
    spine.visual(
        Box((0.055, BRIDGE_Y, 0.036)),
        origin=Origin(xyz=(-0.0475, 0.0, 0.0)),
        material=dark_anodized,
        name="base_bridge",
    )
    spine.visual(
        Box((CHEEK_X, CHEEK_THICKNESS, CHEEK_Z)),
        origin=Origin(xyz=(0.0, CHEEK_Y, 0.0)),
        material=dark_anodized,
        name="base_cheek_pos",
    )
    spine.visual(
        Box((CHEEK_X, CHEEK_THICKNESS, CHEEK_Z)),
        origin=Origin(xyz=(0.0, -CHEEK_Y, 0.0)),
        material=dark_anodized,
        name="base_cheek_neg",
    )
    link_0 = model.part("link_0")
    link_1 = model.part("link_1")
    link_2 = model.part("link_2")

    length_0 = 0.380
    length_1 = 0.320
    length_2 = 0.260

    _add_arm_link(link_0, length_0, brushed_steel, hinge_black)
    _add_arm_link(link_1, length_1, brushed_steel, hinge_black)
    _add_arm_link(link_2, length_2, brushed_steel, hinge_black, terminal=True, pad_material=rubber)

    revolute_limits = MotionLimits(effort=18.0, velocity=2.0, lower=-1.35, upper=1.35)
    model.articulation(
        "spine_to_link_0",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(length_0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(length_1, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2, lower=-1.65, upper=1.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("spine_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
    ]
    ctx.check(
        "three serial revolute joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
    )
    ctx.check(
        "joint axes are parallel vertical",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
    )

    spine = object_model.get_part("spine")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")

    ctx.expect_gap(
        spine,
        link_0,
        axis="y",
        min_gap=0.0,
        max_gap=0.0008,
        positive_elem="base_cheek_pos",
        negative_elem="prox_boss",
        name="base positive cheek clears proximal boss",
    )
    ctx.expect_gap(
        link_0,
        spine,
        axis="y",
        min_gap=0.0,
        max_gap=0.0008,
        positive_elem="prox_boss",
        negative_elem="base_cheek_neg",
        name="base negative cheek clears proximal boss",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="y",
        min_gap=0.0,
        max_gap=0.0008,
        positive_elem="distal_cheek_pos",
        negative_elem="prox_boss",
        name="middle positive cheek clears second boss",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        min_gap=0.0,
        max_gap=0.0008,
        positive_elem="prox_boss",
        negative_elem="distal_cheek_neg",
        name="middle negative cheek clears second boss",
    )
    ctx.expect_gap(
        link_1,
        link_2,
        axis="y",
        min_gap=0.0,
        max_gap=0.0008,
        positive_elem="distal_cheek_pos",
        negative_elem="prox_boss",
        name="terminal positive cheek clears final boss",
    )
    ctx.expect_gap(
        link_2,
        link_1,
        axis="y",
        min_gap=0.0,
        max_gap=0.0008,
        positive_elem="prox_boss",
        negative_elem="distal_cheek_neg",
        name="terminal negative cheek clears final boss",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_pad = _aabb_center(ctx.part_element_world_aabb(link_2, elem="end_pad"))
    with ctx.pose({joints[2]: 0.9}):
        folded_pad = _aabb_center(ctx.part_element_world_aabb(link_2, elem="end_pad"))
    ctx.check(
        "final hinge swings the end pad",
        rest_pad is not None
        and folded_pad is not None
        and math.hypot(folded_pad[0] - rest_pad[0], folded_pad[1] - rest_pad[1]) > 0.08,
        details=f"rest={rest_pad}, folded={folded_pad}",
    )

    return ctx.report()


object_model = build_object_model()

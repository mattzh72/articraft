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


LINK_LENGTH = 0.160
FOLD_ANGLE = math.radians(5.0)
FOLD_TURN = math.radians(152.0)
OPEN_ROOT = math.radians(5.0)
OPEN_TURN = math.radians(5.0)


def _pin_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Cylinder local +Z rotated onto the hinge-pin +Y axis."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_root_bracket(model: ArticulatedObject):
    root = model.part("root_bracket")
    root.visual(
        Box((0.120, 0.120, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, -0.060)),
        material="dark_anodized",
        name="mount_foot",
    )
    root.visual(
        Box((0.018, 0.120, 0.110)),
        origin=Origin(xyz=(-0.055, 0.0, -0.010)),
        material="dark_anodized",
        name="back_plate",
    )
    for side, y in (("pos", 0.038), ("neg", -0.038)):
        root.visual(
            Box((0.090, 0.012, 0.080)),
            origin=Origin(xyz=(-0.005, y, 0.0)),
            material="dark_anodized",
            name=f"clevis_cheek_{side}",
        )
        root.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=_pin_origin(0.0, 0.047 if y > 0.0 else -0.047, 0.0),
            material="brushed_steel",
            name=f"root_pin_cap_{side}",
        )
    root.visual(
        Box((0.065, 0.088, 0.012)),
        origin=Origin(xyz=(-0.006, 0.0, -0.043)),
        material="dark_anodized",
        name="clevis_floor",
    )
    root.visual(
        Box((0.016, 0.075, 0.010)),
        origin=Origin(xyz=(0.038, 0.0, -0.027)),
        material="stop_black",
        name="root_stop_bar",
    )
    return root


def _add_link(model: ArticulatedObject, index: int, *, terminal: bool = False):
    link = model.part(f"link_{index}")
    plate_len = LINK_LENGTH - (0.005 if terminal else 0.028)
    plate_y = 0.026
    for side, y in (("pos", plate_y), ("neg", -plate_y)):
        link.visual(
            Box((plate_len, 0.012, 0.012)),
            origin=Origin(xyz=(plate_len / 2.0, y, 0.0)),
            material="dark_anodized",
            name=f"side_plate_{side}",
        )
        link.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=_pin_origin(0.0, y, 0.0),
            material="dark_anodized",
            name=f"prox_boss_{side}",
        )
        link.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=_pin_origin(0.0, 0.034 if y > 0.0 else -0.034, 0.0),
            material="brushed_steel",
            name=f"prox_pin_cap_{side}",
        )
    if terminal:
        for side, y in (("pos", 0.015), ("neg", -0.015)):
            link.visual(
                Box((0.058, 0.008, 0.010)),
                origin=Origin(xyz=(LINK_LENGTH + 0.026, y, 0.0)),
                material="dark_anodized",
                name=f"fork_tine_{side}",
            )
            link.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=_pin_origin(LINK_LENGTH + 0.052, y, 0.0),
                material="brushed_steel",
                name=f"fork_boss_{side}",
            )
        link.visual(
            Box((0.020, 0.044, 0.010)),
            origin=Origin(xyz=(LINK_LENGTH - 0.004, 0.0, 0.0)),
            material="dark_anodized",
            name="fork_bridge",
        )
        link.visual(
            Box((0.010, 0.026, 0.020)),
            origin=Origin(xyz=(LINK_LENGTH + 0.058, 0.0, 0.0)),
            material="stop_black",
            name="fork_tip_stop",
        )
    else:
        link.visual(
            Box((0.052, 0.040, 0.034)),
            origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
            material="dark_anodized",
            name="hinge_block",
        )
        link.visual(
            Cylinder(radius=0.018, length=0.040),
            origin=_pin_origin(LINK_LENGTH, 0.0, 0.0),
            material="brushed_steel",
            name="block_pin_boss",
        )
        for side, y in (("pos", 0.015), ("neg", -0.015)):
            link.visual(
                Box((0.025, 0.022, 0.008)),
                origin=Origin(xyz=(LINK_LENGTH - 0.0375, 0.021 if y > 0.0 else -0.021, -0.004)),
                material="dark_anodized",
                name=f"block_side_web_{side}",
            )
        link.visual(
            Box((0.030, 0.030, 0.006)),
            origin=Origin(xyz=(LINK_LENGTH - 0.030, 0.0, 0.020)),
            material="stop_black",
            name="fold_stop_tooth",
        )
        link.visual(
            Box((0.030, 0.030, 0.006)),
            origin=Origin(xyz=(LINK_LENGTH + 0.020, 0.0, -0.020)),
            material="stop_black",
            name="open_stop_tooth",
        )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_accordion_arm")
    model.material("dark_anodized", rgba=(0.06, 0.065, 0.070, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    model.material("stop_black", rgba=(0.015, 0.014, 0.012, 1.0))

    root = _add_root_bracket(model)
    links = [_add_link(model, i, terminal=(i == 4)) for i in range(5)]

    model.articulation(
        "root_to_link_0",
        ArticulationType.REVOLUTE,
        parent=root,
        child=links[0],
        origin=Origin(rpy=(0.0, -FOLD_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=OPEN_ROOT),
    )

    fixed_turns = (-FOLD_TURN, FOLD_TURN, -FOLD_TURN, FOLD_TURN)
    open_pitch_turn = -OPEN_TURN
    for i, fixed_pitch in enumerate(fixed_turns):
        open_q = open_pitch_turn - fixed_pitch
        lower = min(0.0, open_q)
        upper = max(0.0, open_q)
        model.articulation(
            f"link_{i}_to_link_{i + 1}",
            ArticulationType.REVOLUTE,
            parent=links[i],
            child=links[i + 1],
            origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0), rpy=(0.0, fixed_pitch, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=lower, upper=upper),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    links = [object_model.get_part(f"link_{i}") for i in range(5)]
    root = object_model.get_part("root_bracket")
    joints = [
        object_model.get_articulation("root_to_link_0"),
        *[object_model.get_articulation(f"link_{i}_to_link_{i + 1}") for i in range(4)],
    ]

    ctx.check(
        "five rigid links and one rooted bracket",
        len(object_model.parts) == 6 and len(object_model.root_parts()) == 1,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "all hinge axes share the same in-plane family",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )
    ctx.expect_overlap(
        links[0],
        root,
        axes="xz",
        elem_a="prox_boss_pos",
        elem_b="clevis_cheek_pos",
        min_overlap=0.018,
        name="root clevis surrounds first pivot boss",
    )
    ctx.expect_gap(
        root,
        links[0],
        axis="y",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="clevis_cheek_pos",
        negative_elem="prox_boss_pos",
        name="root clevis cheek bears on first pivot boss",
    )
    for i in range(4):
        parent = links[i]
        child = links[i + 1]
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            max_gap=0.001,
            max_penetration=0.000001,
            positive_elem="prox_boss_pos",
            negative_elem="hinge_block",
            name=f"folded hinge {i} has supported side clearance",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="xz",
            elem_a="prox_boss_pos",
            elem_b="hinge_block",
            min_overlap=0.018,
            name=f"folded hinge {i} keeps pin bosses coaxial",
        )

    folded_fork_aabb = ctx.part_world_aabb(links[-1])
    folded_pos = ctx.part_world_position(links[-1])
    open_pose = {
        joints[0]: OPEN_ROOT,
        joints[1]: (-OPEN_TURN) - (-FOLD_TURN),
        joints[2]: (-OPEN_TURN) - FOLD_TURN,
        joints[3]: (-OPEN_TURN) - (-FOLD_TURN),
        joints[4]: (-OPEN_TURN) - FOLD_TURN,
    }
    with ctx.pose(open_pose):
        open_pos = ctx.part_world_position(links[-1])
        open_fork_aabb = ctx.part_world_aabb(links[-1])
        ctx.check(
            "opened chain stretches forward into a shallow arc",
            folded_pos is not None
            and open_pos is not None
            and open_pos[0] > folded_pos[0] + 0.45
            and open_pos[2] < folded_pos[2],
            details=f"folded={folded_pos}, opened={open_pos}",
        )
        ctx.check(
            "fork tab reaches beyond the compact folded package",
            folded_fork_aabb is not None
            and open_fork_aabb is not None
            and open_fork_aabb[1][0] > folded_fork_aabb[1][0] + 0.45,
            details=f"folded_aabb={folded_fork_aabb}, opened_aabb={open_fork_aabb}",
        )
        for i in range(4):
            ctx.expect_gap(
                links[i + 1],
                links[i],
                axis="y",
                max_gap=0.001,
                max_penetration=0.000001,
                positive_elem="prox_boss_pos",
                negative_elem="hinge_block",
                name=f"opened hinge {i} keeps supported side clearance",
            )
            ctx.expect_overlap(
                links[i + 1],
                links[i],
                axes="xz",
                elem_a="prox_boss_pos",
                elem_b="hinge_block",
                min_overlap=0.018,
                name=f"opened hinge {i} remains supported on its pivot",
            )

    return ctx.report()


object_model = build_object_model()

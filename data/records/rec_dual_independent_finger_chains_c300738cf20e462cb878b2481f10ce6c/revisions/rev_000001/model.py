from __future__ import annotations

import math

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
)


HINGE_RPY = (math.pi / 2.0, 0.0, 0.0)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _hinge_barrel(part, *, material: Material, name: str) -> None:
    part.visual(
        # Long enough to read as a captured hinge pin through the clevis ears.
        Cylinder(radius=0.014, length=0.068),
        origin=Origin(rpy=HINGE_RPY),
        material=material,
        name=name,
    )


def _forked_phalanx(part, *, length: float, body_material: Material, joint_material: Material) -> None:
    """A single link with a proximal barrel and a distal fork for the next joint."""
    _hinge_barrel(part, material=joint_material, name="proximal_barrel")

    # Narrow central spine fits between the previous fork ears during bending.
    spine_start = 0.010
    spine_end = length - 0.025
    part.visual(
        Box((spine_end - spine_start, 0.028, 0.018)),
        origin=Origin(xyz=((spine_start + spine_end) / 2.0, 0.0, 0.0)),
        material=body_material,
        name="link_spine",
    )

    # Full-width bridge sits just behind the next barrel, tying both fork ears
    # to the spine without occupying the moving child barrel volume.
    part.visual(
        Box((0.012, 0.058, 0.020)),
        origin=Origin(xyz=(length - 0.023, 0.0, 0.0)),
        material=body_material,
        name="yoke_bridge",
    )
    for side, y in (("upper", 0.024), ("lower", -0.024)):
        part.visual(
            Box((0.030, 0.010, 0.036)),
            origin=Origin(xyz=(length - 0.009, y, 0.0)),
            material=body_material,
            name=f"{side}_fork_ear",
        )


def _tipped_phalanx(
    part,
    *,
    length: float,
    body_material: Material,
    joint_material: Material,
    pad_material: Material,
) -> None:
    """Final link with a blunt rubber pad fixed to its end."""
    _hinge_barrel(part, material=joint_material, name="proximal_barrel")

    spine_start = 0.010
    spine_end = length - 0.020
    part.visual(
        Box((spine_end - spine_start, 0.028, 0.018)),
        origin=Origin(xyz=((spine_start + spine_end) / 2.0, 0.0, 0.0)),
        material=body_material,
        name="link_spine",
    )
    part.visual(
        Box((0.040, 0.044, 0.028)),
        origin=Origin(xyz=(length - 0.004, 0.0, 0.0)),
        material=pad_material,
        name="tip_pad_block",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.044),
        origin=Origin(xyz=(length + 0.016, 0.0, 0.0), rpy=HINGE_RPY),
        material=pad_material,
        name="rounded_tip_face",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_palm_dual_finger")

    aluminum = _mat("satin_aluminum", (0.62, 0.66, 0.68, 1.0))
    dark_steel = _mat("dark_pivot_steel", (0.08, 0.09, 0.10, 1.0))
    rubber = _mat("matte_black_rubber", (0.01, 0.012, 0.012, 1.0))
    seam = _mat("black_split_groove", (0.0, 0.0, 0.0, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.180, 0.190, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=aluminum,
        name="palm_block",
    )
    # Visible central split/groove: one block still carries both independent
    # finger roots, but the dark relief line makes the split-palm construction
    # legible from above and from the front.
    palm.visual(
        Box((0.162, 0.010, 0.004)),
        origin=Origin(xyz=(-0.004, 0.0, 0.067)),
        material=seam,
        name="top_split_groove",
    )
    palm.visual(
        Box((0.006, 0.012, 0.050)),
        origin=Origin(xyz=(0.092, 0.0, 0.041)),
        material=seam,
        name="front_split_groove",
    )

    root_x = 0.110
    root_z = 0.045
    root_ys = {"left": 0.050, "right": -0.050}
    for side, root_y in root_ys.items():
        for cue, y in (("outer", root_y + 0.027), ("inner", root_y - 0.027)):
            palm.visual(
                Box((0.042, 0.010, 0.045)),
                origin=Origin(xyz=(root_x - 0.014, y, root_z)),
                material=aluminum,
                name=f"{side}_{cue}_root_lug",
            )

    lengths = {"proximal": 0.108, "middle": 0.086, "distal": 0.070}
    joints = []
    for side, root_y in root_ys.items():
        proximal = model.part(f"{side}_proximal")
        middle = model.part(f"{side}_middle")
        distal = model.part(f"{side}_distal")

        _forked_phalanx(proximal, length=lengths["proximal"], body_material=aluminum, joint_material=dark_steel)
        _forked_phalanx(middle, length=lengths["middle"], body_material=aluminum, joint_material=dark_steel)
        _tipped_phalanx(
            distal,
            length=lengths["distal"],
            body_material=aluminum,
            joint_material=dark_steel,
            pad_material=rubber,
        )

        limits = MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.15)
        joints.append(
            model.articulation(
                f"{side}_root_pivot",
                ArticulationType.REVOLUTE,
                parent=palm,
                child=proximal,
                origin=Origin(xyz=(root_x, root_y, root_z)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=limits,
            )
        )
        joints.append(
            model.articulation(
                f"{side}_knuckle_pivot",
                ArticulationType.REVOLUTE,
                parent=proximal,
                child=middle,
                origin=Origin(xyz=(lengths["proximal"], 0.0, 0.0)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=limits,
            )
        )
        joints.append(
            model.articulation(
                f"{side}_tip_pivot",
                ArticulationType.REVOLUTE,
                parent=middle,
                child=distal,
                origin=Origin(xyz=(lengths["middle"], 0.0, 0.0)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=limits,
            )
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    right_proximal = object_model.get_part("right_proximal")
    left_distal = object_model.get_part("left_distal")
    right_distal = object_model.get_part("right_distal")

    for side in ("left", "right"):
        root_part = object_model.get_part(f"{side}_proximal")
        for cue in ("inner", "outer"):
            ctx.allow_overlap(
                root_part,
                palm,
                elem_a="proximal_barrel",
                elem_b=f"{side}_{cue}_root_lug",
                reason="Captured root pivot barrel intentionally passes through the palm clevis lug.",
            )
            ctx.expect_overlap(
                root_part,
                palm,
                axes="xy",
                elem_a="proximal_barrel",
                elem_b=f"{side}_{cue}_root_lug",
                min_overlap=0.004,
                name=f"{side} root barrel captured by {cue} lug",
            )

        for parent_name, child_name in (
            (f"{side}_proximal", f"{side}_middle"),
            (f"{side}_middle", f"{side}_distal"),
        ):
            parent = object_model.get_part(parent_name)
            child = object_model.get_part(child_name)
            for ear in ("upper_fork_ear", "lower_fork_ear"):
                ctx.allow_overlap(
                    parent,
                    child,
                    elem_a=ear,
                    elem_b="proximal_barrel",
                    reason="Captured phalanx hinge barrel intentionally passes through the fork ear.",
                )
                ctx.expect_overlap(
                    parent,
                    child,
                    axes="xy",
                    elem_a=ear,
                    elem_b="proximal_barrel",
                    min_overlap=0.004,
                    name=f"{parent_name} captures {child_name} {ear}",
                )

    revolute_joints = [
        j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two independent three-joint fingers",
        len(revolute_joints) == 6 and all(j.mimic is None for j in revolute_joints),
        details=f"revolute={len(revolute_joints)}, mimics={[j.name for j in revolute_joints if j.mimic is not None]}",
    )
    ctx.expect_origin_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.095,
        name="separate root pivots on split palm",
    )
    ctx.expect_gap(
        left_proximal,
        palm,
        axis="x",
        min_gap=0.002,
        max_gap=0.030,
        positive_elem="proximal_barrel",
        negative_elem="palm_block",
        name="left root barrel is just ahead of palm face",
    )
    ctx.expect_gap(
        right_proximal,
        palm,
        axis="x",
        min_gap=0.002,
        max_gap=0.030,
        positive_elem="proximal_barrel",
        negative_elem="palm_block",
        name="right root barrel is just ahead of palm face",
    )

    left_tip = left_distal.get_visual("tip_pad_block")
    right_tip = right_distal.get_visual("tip_pad_block")
    ctx.check("left distal has blunt tip pad", left_tip is not None)
    ctx.check("right distal has blunt tip pad", right_tip is not None)

    left_root = object_model.get_articulation("left_root_pivot")
    left_knuckle = object_model.get_articulation("left_knuckle_pivot")
    left_tip_joint = object_model.get_articulation("left_tip_pivot")
    right_root = object_model.get_articulation("right_root_pivot")
    right_knuckle = object_model.get_articulation("right_knuckle_pivot")
    right_tip_joint = object_model.get_articulation("right_tip_pivot")

    rest_left = ctx.part_world_position(left_distal)
    rest_right = ctx.part_world_position(right_distal)
    with ctx.pose({left_root: 0.70, left_knuckle: 0.55, left_tip_joint: 0.45}):
        curled_left = ctx.part_world_position(left_distal)
        still_right = ctx.part_world_position(right_distal)
    ctx.check(
        "left chain curls without driving right chain",
        rest_left is not None
        and curled_left is not None
        and rest_right is not None
        and still_right is not None
        and curled_left[2] > rest_left[2] + 0.045
        and max(abs(still_right[i] - rest_right[i]) for i in range(3)) < 1e-6,
        details=f"rest_left={rest_left}, curled_left={curled_left}, rest_right={rest_right}, still_right={still_right}",
    )

    rest_right_again = ctx.part_world_position(right_distal)
    with ctx.pose({right_root: 0.65, right_knuckle: 0.50, right_tip_joint: 0.40}):
        curled_right = ctx.part_world_position(right_distal)
    ctx.check(
        "right chain curls independently",
        rest_right_again is not None
        and curled_right is not None
        and curled_right[2] > rest_right_again[2] + 0.040,
        details=f"rest_right={rest_right_again}, curled_right={curled_right}",
    )

    return ctx.report()


object_model = build_object_model()

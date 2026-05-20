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
import cadquery as cq


HINGE_Z = 0.085
LANE_Y = 0.045
LINK_WIDTH = 0.024
LINK_THICKNESS = 0.026
HUB_RADIUS = 0.015
CHEEK_GAP = 0.034
HUB_WIDTH = CHEEK_GAP
CHEEK_THICKNESS = 0.008
CHEEK_X = 0.058
CHEEK_Z = 0.050
PIN_RADIUS = 0.006


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_pin_caps(part, x: float, y_center: float, z: float, material, prefix: str = "") -> None:
    """Small visible screw/pin heads on the outside of a cheek pair."""

    side_offset = CHEEK_GAP / 2.0 + CHEEK_THICKNESS + 0.001
    for sign, suffix in ((-1.0, "neg"), (1.0, "pos")):
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=0.004),
            origin=_y_cylinder_origin(x, y_center + sign * side_offset, z),
            material=material,
            name=f"{prefix}pin_cap_{suffix}",
        )


def _add_base_clevis(palm, lane_y: float, lane_name: str, cheek_mat, pin_mat) -> None:
    """Add the grounded palm-side cheek pair for one base knuckle."""

    cheek_center_offset = CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0
    base_cheek_names = {
        "left": ("left_base_cheek_inner", "left_base_cheek_outer"),
        "right": ("right_base_cheek_inner", "right_base_cheek_outer"),
    }[lane_name]
    for sign, visual_name in (
        (-1.0, base_cheek_names[0]),
        (1.0, base_cheek_names[1]),
    ):
        palm.visual(
            Box((CHEEK_X, CHEEK_THICKNESS, CHEEK_Z)),
            origin=Origin(
                xyz=(
                    0.0,
                    lane_y + sign * cheek_center_offset,
                    HINGE_Z,
                )
            ),
            material=cheek_mat,
            name=visual_name,
        )
    _add_pin_caps(palm, 0.0, lane_y, HINGE_Z, pin_mat, prefix=f"{lane_name}_base_")


def _add_link_visuals(
    part,
    *,
    length: float,
    has_distal_clevis: bool,
    link_mat,
    cheek_mat,
    pin_mat,
) -> None:
    """Build one narrow phalange-like link in a frame at its proximal hinge."""

    bar_start = 0.012
    bar_end = length - (0.050 if has_distal_clevis else 0.010)
    bar_length = bar_end - bar_start

    part.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_WIDTH),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=pin_mat,
        name="proximal_hub",
    )
    part.visual(
        Box((bar_length, LINK_WIDTH, LINK_THICKNESS)),
        origin=Origin(xyz=((bar_start + bar_end) / 2.0, 0.0, 0.0)),
        material=link_mat,
        name="link_web",
    )

    if has_distal_clevis:
        # A yoke bridge behind the hinge connects both cheek plates to the
        # slender web while leaving an open central gap for the next link hub.
        bridge_center = length - 0.042
        part.visual(
            Box((0.026, CHEEK_GAP + 2.0 * CHEEK_THICKNESS, LINK_THICKNESS)),
            origin=Origin(xyz=(bridge_center, 0.0, 0.0)),
            material=link_mat,
            name="cheek_bridge",
        )

        cheek_center_offset = CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0
        for sign, visual_name in (
            (-1.0, "distal_cheek_near"),
            (1.0, "distal_cheek_far"),
        ):
            part.visual(
                Box((CHEEK_X, CHEEK_THICKNESS, CHEEK_Z)),
                origin=Origin(
                    xyz=(
                        length,
                        sign * cheek_center_offset,
                        0.0,
                    )
                ),
                material=cheek_mat,
                name=visual_name,
            )
        _add_pin_caps(part, length, 0.0, 0.0, pin_mat)
    else:
        part.visual(
            Cylinder(radius=0.013, length=LINK_WIDTH),
            origin=_y_cylinder_origin(length, 0.0, 0.0),
            material=link_mat,
            name="rounded_tip",
        )
        part.visual(
            Box((0.024, LINK_WIDTH, 0.012)),
            origin=Origin(xyz=(length - 0.012, 0.0, -0.011)),
            material=link_mat,
            name="tip_pad",
        )


def _finger_chain(
    model: ArticulatedObject,
    palm,
    *,
    side: str,
    lane_y: float,
    lengths: tuple[float, float, float],
    link_mat,
    cheek_mat,
    pin_mat,
) -> None:
    proximal = model.part(f"{side}_proximal")
    middle = model.part(f"{side}_middle")
    distal = model.part(f"{side}_distal")

    _add_link_visuals(
        proximal,
        length=lengths[0],
        has_distal_clevis=True,
        link_mat=link_mat,
        cheek_mat=cheek_mat,
        pin_mat=pin_mat,
    )
    _add_link_visuals(
        middle,
        length=lengths[1],
        has_distal_clevis=True,
        link_mat=link_mat,
        cheek_mat=cheek_mat,
        pin_mat=pin_mat,
    )
    _add_link_visuals(
        distal,
        length=lengths[2],
        has_distal_clevis=False,
        link_mat=link_mat,
        cheek_mat=cheek_mat,
        pin_mat=pin_mat,
    )

    flex_limits = MotionLimits(effort=8.0, velocity=3.0, lower=-0.25, upper=1.35)
    curl_limits = MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.55)

    model.articulation(
        f"palm_to_{side}_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=proximal,
        origin=Origin(xyz=(0.0, lane_y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=flex_limits,
    )
    model.articulation(
        f"{side}_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=curl_limits,
    )
    model.articulation(
        f"{side}_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=curl_limits,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_palm_twin_fingers")

    palm_mat = model.material("matte_black_palm", rgba=(0.035, 0.040, 0.045, 1.0))
    bridge_mat = model.material("dark_anodized_bridge", rgba=(0.08, 0.09, 0.10, 1.0))
    link_mat = model.material("brushed_aluminum_links", rgba=(0.70, 0.74, 0.76, 1.0))
    cheek_mat = model.material("blue_knuckle_cheeks", rgba=(0.10, 0.23, 0.42, 1.0))
    pin_mat = model.material("steel_pins", rgba=(0.46, 0.47, 0.48, 1.0))

    palm = model.part("palm_bridge")
    palm.visual(
        Box((0.130, 0.175, 0.030)),
        origin=Origin(xyz=(-0.045, 0.0, 0.015)),
        material=palm_mat,
        name="ground_plate",
    )
    palm.visual(
        Box((0.060, 0.165, 0.034)),
        origin=Origin(xyz=(-0.030, 0.0, 0.047)),
        material=bridge_mat,
        name="raised_bridge",
    )
    palm.visual(
        Box((0.024, 0.145, 0.016)),
        origin=Origin(xyz=(-0.007, 0.0, 0.060)),
        material=bridge_mat,
        name="front_lip",
    )

    _add_base_clevis(palm, LANE_Y, "left", cheek_mat, pin_mat)
    _add_base_clevis(palm, -LANE_Y, "right", cheek_mat, pin_mat)

    _finger_chain(
        model,
        palm,
        side="left",
        lane_y=LANE_Y,
        lengths=(0.116, 0.096, 0.084),
        link_mat=link_mat,
        cheek_mat=cheek_mat,
        pin_mat=pin_mat,
    )
    _finger_chain(
        model,
        palm,
        side="right",
        lane_y=-LANE_Y,
        lengths=(0.110, 0.092, 0.080),
        link_mat=link_mat,
        cheek_mat=cheek_mat,
        pin_mat=pin_mat,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm_bridge")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_distal = object_model.get_part("right_distal")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two independent three-joint chains",
        len(object_model.articulations) == 6
        and len(revolute_joints) == 6
        and all(joint.mimic is None for joint in object_model.articulations),
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )

    for side in ("left", "right"):
        for joint_name in (
            f"palm_to_{side}_proximal",
            f"{side}_proximal_to_middle",
            f"{side}_middle_to_distal",
        ):
            joint = object_model.get_articulation(joint_name)
            limits = joint.motion_limits
            ctx.check(
                f"{joint_name} has flexion travel",
                limits is not None
                and limits.lower is not None
                and limits.upper is not None
                and limits.upper - limits.lower > 1.2,
                details=f"limits={limits}",
            )

    ctx.expect_origin_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.085,
        max_gap=0.095,
        name="finger lanes are separated across the bridge",
    )
    ctx.expect_origin_distance(
        left_proximal,
        left_middle,
        axes="x",
        min_dist=0.114,
        max_dist=0.118,
        name="left proximal link is the longest segment",
    )
    ctx.expect_origin_distance(
        left_middle,
        left_distal,
        axes="x",
        min_dist=0.094,
        max_dist=0.098,
        name="left distal joint is set after a shorter middle link",
    )

    ctx.expect_contact(
        left_proximal,
        palm,
        elem_a="proximal_hub",
        elem_b="left_base_cheek_inner",
        name="base knuckle hub is captured by palm cheek",
    )
    ctx.expect_contact(
        left_middle,
        left_proximal,
        elem_a="proximal_hub",
        elem_b="distal_cheek_near",
        name="middle hub is captured by proximal cheek",
    )

    left_tip_rest = ctx.part_world_aabb(left_distal)
    right_tip_rest = ctx.part_world_position(right_distal)
    with ctx.pose(
        {
            "palm_to_left_proximal": 0.55,
            "left_proximal_to_middle": 0.45,
            "left_middle_to_distal": 0.35,
        }
    ):
        left_tip_flexed = ctx.part_world_aabb(left_distal)
        right_tip_after_left_pose = ctx.part_world_position(right_distal)

    ctx.check(
        "positive left flexion curls the fingertip upward",
        left_tip_rest is not None
        and left_tip_flexed is not None
        and left_tip_flexed[1][2] > left_tip_rest[1][2] + 0.055,
        details=f"rest={left_tip_rest}, flexed={left_tip_flexed}",
    )
    ctx.check(
        "right finger is unaffected by left finger pose",
        right_tip_rest is not None
        and right_tip_after_left_pose is not None
        and max(
            abs(a - b)
            for a, b in zip(right_tip_rest, right_tip_after_left_pose)
        )
        < 1e-6,
        details=f"rest={right_tip_rest}, after={right_tip_after_left_pose}",
    )

    return ctx.report()


object_model = build_object_model()

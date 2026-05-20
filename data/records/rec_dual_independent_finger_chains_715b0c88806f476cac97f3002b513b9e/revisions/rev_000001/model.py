from __future__ import annotations

from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _clevis_link_profile(length: float, root_width: float, tip_width: float) -> list[tuple[float, float]]:
    """Plan-view outline for one tapered flat link with a distal fork.

    The link frame is at its proximal pin.  The distal end is a U-shaped
    clevis: side ears occupy the outside of the link and a center slot accepts
    the next link's narrow tongue without sharing geometry.
    """

    tongue_half = 0.0060
    gap_half = 0.0068
    root_half = root_width * 0.5
    tip_half = tip_width * 0.5
    back = 0.018
    tongue_front = 0.018
    body_start = 0.026
    fork_start = length - 0.028
    slot_back = length - 0.019
    front = length + 0.018

    return [
        (-back, tongue_half),
        (tongue_front, tongue_half),
        (body_start, root_half),
        (fork_start, tip_half),
        (front, tip_half),
        (front, gap_half),
        (slot_back, gap_half),
        (slot_back, -gap_half),
        (front, -gap_half),
        (front, -tip_half),
        (fork_start, -tip_half),
        (body_start, -root_half),
        (tongue_front, -tongue_half),
        (-back, -tongue_half),
    ]


def _tip_link_profile(length: float, root_width: float, tip_width: float) -> list[tuple[float, float]]:
    """Plan-view outline for the terminal tapered link that carries a pad."""

    tongue_half = 0.0060
    root_half = root_width * 0.5
    tip_half = tip_width * 0.5
    back = 0.018
    tongue_front = 0.018
    body_start = 0.026
    return [
        (-back, tongue_half),
        (tongue_front, tongue_half),
        (body_start, root_half),
        (length, tip_half),
        (length + 0.006, tip_half * 0.80),
        (length + 0.006, -tip_half * 0.80),
        (length, -tip_half),
        (body_start, -root_half),
        (tongue_front, -tongue_half),
        (-back, -tongue_half),
    ]


def _extruded_link_mesh(profile: list[tuple[float, float]], thickness: float, name: str):
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_gripper_hand_study")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.60, 0.62, 0.60, 1.0))
    anodized = model.material("blue_anodized_links", rgba=(0.18, 0.31, 0.55, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.145, 0.145, 0.035)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0175)),
        material=dark_steel,
        name="rear_plate",
    )
    palm.visual(
        Box((0.050, 0.118, 0.019)),
        origin=Origin(xyz=(0.005, 0.0, 0.0445)),
        material=dark_steel,
        name="front_tower_boss",
    )

    root_x = 0.014
    finger_z = 0.061
    tower_gap_half = 0.0076
    cheek_width = 0.0070
    cheek_offset = tower_gap_half + cheek_width * 0.5
    finger_centers = (-0.023, 0.023)

    for finger_i, y_center in enumerate(finger_centers):
        for cheek_i, sign in enumerate((-1.0, 1.0)):
            palm.visual(
                Box((0.046, cheek_width, 0.016)),
                origin=Origin(
                    xyz=(root_x - 0.004, y_center + sign * cheek_offset, finger_z)
                ),
                material=dark_steel,
                name=f"tower_{finger_i}_cheek_{cheek_i}",
            )
            palm.visual(
                Box((0.018, cheek_width, 0.029)),
                origin=Origin(
                    xyz=(root_x - 0.020, y_center + sign * cheek_offset, 0.050)
                ),
                material=dark_steel,
                name=f"tower_{finger_i}_web_{cheek_i}",
            )

    for screw_i, (x, y) in enumerate(
        ((-0.102, -0.055), (-0.102, 0.055), (-0.028, -0.055), (-0.028, 0.055))
    ):
        palm.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, y, 0.0365)),
            material=brushed_steel,
            name=f"mount_screw_{screw_i}",
        )

    thickness = 0.012
    lengths = (0.075, 0.065, 0.055)
    widths = ((0.025, 0.021), (0.021, 0.018), (0.018, 0.014))
    limits = MotionLimits(effort=8.0, velocity=3.5, lower=-0.95, upper=0.95)

    for finger_i, y_center in enumerate(finger_centers):
        root = model.part(f"finger_{finger_i}_root")
        middle = model.part(f"finger_{finger_i}_middle")
        tip = model.part(f"finger_{finger_i}_tip")

        chain = (root, middle, tip)
        for link_i, link in enumerate(chain):
            is_tip = link_i == 2
            profile = (
                _tip_link_profile(lengths[link_i], *widths[link_i])
                if is_tip
                else _clevis_link_profile(lengths[link_i], *widths[link_i])
            )
            link.visual(
                _extruded_link_mesh(profile, thickness, f"finger_{finger_i}_link_{link_i}"),
                origin=Origin(),
                material=anodized,
                name="tapered_link",
            )
            link.visual(
                Cylinder(radius=0.00785, length=0.014),
                origin=Origin(xyz=(0.0, 0.0, 0.001)),
                material=brushed_steel,
                name="pin_post",
            )
            link.visual(
                Cylinder(radius=0.0056, length=0.003),
                origin=Origin(xyz=(0.0, 0.0, 0.009)),
                material=brushed_steel,
                name="pin_head",
            )

        tip.visual(
            Box((0.027, 0.025, 0.018)),
            origin=Origin(xyz=(lengths[2] + 0.016, 0.0, 0.0)),
            material=rubber,
            name="fingertip_pad",
        )

        model.articulation(
            f"finger_{finger_i}_root_joint",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=root,
            origin=Origin(xyz=(root_x, y_center, finger_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=limits,
        )
        model.articulation(
            f"finger_{finger_i}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=root,
            child=middle,
            origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=limits,
        )
        model.articulation(
            f"finger_{finger_i}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=tip,
            origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for finger_i in (0, 1):
        palm = object_model.get_part("palm")
        root = object_model.get_part(f"finger_{finger_i}_root")
        middle = object_model.get_part(f"finger_{finger_i}_middle")
        tip = object_model.get_part(f"finger_{finger_i}_tip")

        for cheek_i, cheek_name in enumerate(
            (f"tower_{finger_i}_cheek_0", f"tower_{finger_i}_cheek_1")
        ):
            ctx.allow_overlap(
                palm,
                root,
                elem_a=cheek_name,
                elem_b="pin_post",
                reason=(
                    "The root pin is intentionally captured by the simplified "
                    "unbored palm tower cheek."
                ),
            )
            if cheek_i == 0:
                ctx.expect_gap(
                    root,
                    palm,
                    axis="y",
                    positive_elem="pin_post",
                    negative_elem=cheek_name,
                    max_penetration=0.001,
                    max_gap=0.001,
                    name=f"finger {finger_i} root pin seats against lower tower cheek",
                )
            else:
                ctx.expect_gap(
                    palm,
                    root,
                    axis="y",
                    positive_elem=cheek_name,
                    negative_elem="pin_post",
                    max_penetration=0.001,
                    max_gap=0.001,
                    name=f"finger {finger_i} root pin seats against upper tower cheek",
                )

        ctx.allow_overlap(
            root,
            middle,
            elem_a="tapered_link",
            elem_b="pin_post",
            reason=(
                "The middle link's visible pin is intentionally captured in the "
                "simplified unbored distal clevis of the root link."
            ),
        )
        ctx.expect_overlap(
            middle,
            root,
            axes="xyz",
            elem_a="pin_post",
            elem_b="tapered_link",
            min_overlap=0.006,
            name=f"finger {finger_i} middle pin is retained in root clevis",
        )

        ctx.allow_overlap(
            middle,
            tip,
            elem_a="tapered_link",
            elem_b="pin_post",
            reason=(
                "The tip link's visible pin is intentionally captured in the "
                "simplified unbored distal clevis of the middle link."
            ),
        )
        ctx.expect_overlap(
            tip,
            middle,
            axes="xyz",
            elem_a="pin_post",
            elem_b="tapered_link",
            min_overlap=0.006,
            name=f"finger {finger_i} tip pin is retained in middle clevis",
        )

        for joint_name in (
            f"finger_{finger_i}_root_joint",
            f"finger_{finger_i}_middle_joint",
            f"finger_{finger_i}_tip_joint",
        ):
            joint = object_model.get_articulation(joint_name)
            ctx.check(
                f"{joint_name} is an independent revolute joint",
                joint.articulation_type == ArticulationType.REVOLUTE and joint.mimic is None,
                details=f"type={joint.articulation_type!r}, mimic={joint.mimic!r}",
            )

        ctx.check(
            f"finger {finger_i} has a rigid fingertip pad",
            tip.get_visual("fingertip_pad") is not None,
            details="terminal link should carry a named rubber pad visual",
        )

    lower_root = object_model.get_part("finger_0_root")
    upper_root = object_model.get_part("finger_1_root")
    ctx.expect_gap(
        upper_root,
        lower_root,
        axis="y",
        min_gap=0.006,
        max_gap=0.030,
        name="parallel finger chains sit close without sharing links",
    )

    finger_0_tip = object_model.get_part("finger_0_tip")
    finger_1_tip = object_model.get_part("finger_1_tip")
    rest_0 = ctx.part_world_position(finger_0_tip)
    rest_1 = ctx.part_world_position(finger_1_tip)
    with ctx.pose(
        {
            "finger_0_root_joint": -0.35,
            "finger_0_middle_joint": -0.25,
            "finger_0_tip_joint": -0.15,
        }
    ):
        moved_0 = ctx.part_world_position(finger_0_tip)
        still_1 = ctx.part_world_position(finger_1_tip)

    ctx.check(
        "finger 0 articulates independently",
        rest_0 is not None
        and moved_0 is not None
        and rest_1 is not None
        and still_1 is not None
        and moved_0[1] < rest_0[1] - 0.025
        and all(isclose(still_1[i], rest_1[i], abs_tol=1e-6) for i in range(3)),
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, still_1={still_1}",
    )

    return ctx.report()


object_model = build_object_model()

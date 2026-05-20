from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_palm_dual_finger_module")

    anodized_blue = model.material("anodized_blue", rgba=(0.08, 0.24, 0.54, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.035, 1.0))
    fastener = model.material("black_fastener", rgba=(0.01, 0.01, 0.012, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.24, 0.46, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=anodized_blue,
        name="base_block",
    )
    palm.visual(
        Box((0.22, 0.42, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=brushed_steel,
        name="top_cover",
    )
    palm.visual(
        Box((0.030, 0.46, 0.012)),
        origin=Origin(xyz=(-0.105, 0.0, 0.012)),
        material=dark_steel,
        name="rear_ground_foot",
    )
    palm.visual(
        Box((0.030, 0.46, 0.012)),
        origin=Origin(xyz=(0.105, 0.0, 0.012)),
        material=dark_steel,
        name="front_ground_foot",
    )

    finger_y_positions = (-0.085, 0.085)
    base_joint_x = 0.150
    base_joint_z = 0.120
    barrel_radius = 0.021
    barrel_width = 0.052
    clevis_gap = 0.0
    cheek_thickness = 0.014
    cheek_offset = barrel_width * 0.5 + clevis_gap + cheek_thickness * 0.5

    for finger_index, y_pos in enumerate(finger_y_positions):
        palm.visual(
            Box((0.092, 0.096, 0.016)),
            origin=Origin(xyz=(0.120, y_pos, 0.078)),
            material=anodized_blue,
            name=f"finger_{finger_index}_mount_floor",
        )
        for side_sign in (-1.0, 1.0):
            palm.visual(
                Box((0.070, cheek_thickness, 0.056)),
                origin=Origin(
                    xyz=(
                        0.145,
                        y_pos + side_sign * cheek_offset,
                        0.107,
                    )
                ),
                material=brushed_steel,
                name=f"finger_{finger_index}_clevis_{int(side_sign > 0)}",
            )
        palm.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.070, y_pos - 0.032, 0.084)),
            material=fastener,
            name=f"finger_{finger_index}_screw_0",
        )
        palm.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.070, y_pos + 0.032, 0.084)),
            material=fastener,
            name=f"finger_{finger_index}_screw_1",
        )

    palm.inertial = Inertial.from_geometry(
        Box((0.26, 0.48, 0.12)),
        mass=3.5,
        origin=Origin(xyz=(0.02, 0.0, 0.060)),
    )

    def add_finger_link(
        part_name: str,
        *,
        length: float,
        is_tip: bool = False,
    ):
        link = model.part(part_name)
        body_length = length - 2.0 * barrel_radius + 0.001
        body_min_x = barrel_radius - 0.001
        body_center_x = body_min_x + body_length * 0.5
        link.visual(
            Cylinder(radius=barrel_radius, length=barrel_width),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="hinge_barrel",
        )
        link.visual(
            Box((body_length, 0.040, 0.028)),
            origin=Origin(xyz=(body_center_x, 0.0, 0.0)),
            material=dark_steel,
            name="link_body",
        )
        if is_tip:
            link.visual(
                Box((0.034, 0.046, 0.034)),
                origin=Origin(xyz=(length - 0.015, 0.0, 0.0)),
                material=black_rubber,
                name="rubber_tip",
            )
        else:
            link.visual(
                Box((0.014, 0.050, 0.034)),
                origin=Origin(xyz=(length - barrel_radius - 0.007, 0.0, 0.0)),
                material=brushed_steel,
                name="end_yoke_face",
            )
        link.inertial = Inertial.from_geometry(
            Box((length, 0.055, 0.055)),
            mass=0.18 if is_tip else 0.25,
            origin=Origin(xyz=(length * 0.5, 0.0, 0.0)),
        )
        return link

    finger_lengths = {
        0: (0.155, 0.125, 0.095),
        1: (0.170, 0.115, 0.105),
    }

    fingers = {}
    for finger_index, lengths in finger_lengths.items():
        proximal = add_finger_link(
            f"finger_{finger_index}_proximal",
            length=lengths[0],
        )
        middle = add_finger_link(
            f"finger_{finger_index}_middle",
            length=lengths[1],
        )
        tip = add_finger_link(
            f"finger_{finger_index}_tip",
            length=lengths[2],
            is_tip=True,
        )
        fingers[finger_index] = (proximal, middle, tip)

        model.articulation(
            f"finger_{finger_index}_knuckle",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(base_joint_x, finger_y_positions[finger_index], base_joint_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.15),
        )
        model.articulation(
            f"finger_{finger_index}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=7.0, velocity=3.0, lower=0.0, upper=1.25),
        )
        model.articulation(
            f"finger_{finger_index}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=tip,
            origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    finger_joint_names = [
        "finger_0_knuckle",
        "finger_0_middle_joint",
        "finger_0_tip_joint",
        "finger_1_knuckle",
        "finger_1_middle_joint",
        "finger_1_tip_joint",
    ]
    finger_joints = [object_model.get_articulation(name) for name in finger_joint_names]
    ctx.check(
        "two fingers each have three revolute joints",
        len(finger_joints) == 6
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in finger_joints),
    )

    # The two fingers intentionally have slightly different segment proportions,
    # avoiding a mirrored/toy-like duplicate pair while preserving a matched module.
    f0_proximal = object_model.get_articulation("finger_0_middle_joint").origin.xyz[0]
    f1_proximal = object_model.get_articulation("finger_1_middle_joint").origin.xyz[0]
    f0_middle = object_model.get_articulation("finger_0_tip_joint").origin.xyz[0]
    f1_middle = object_model.get_articulation("finger_1_tip_joint").origin.xyz[0]
    ctx.check(
        "finger link lengths are intentionally unequal",
        abs(f0_proximal - f1_proximal) > 0.010 and abs(f0_middle - f1_middle) > 0.005,
        details=(
            f"finger_0=({f0_proximal:.3f}, {f0_middle:.3f}), "
            f"finger_1=({f1_proximal:.3f}, {f1_middle:.3f})"
        ),
    )

    palm = object_model.get_part("palm")
    for finger_index in (0, 1):
        proximal = object_model.get_part(f"finger_{finger_index}_proximal")
        middle = object_model.get_part(f"finger_{finger_index}_middle")
        tip = object_model.get_part(f"finger_{finger_index}_tip")
        ctx.expect_contact(
            proximal,
            palm,
            contact_tol=0.00005,
            name=f"finger {finger_index} knuckle is carried by the palm clevis",
        )
        ctx.expect_contact(
            middle,
            proximal,
            contact_tol=0.00005,
            name=f"finger {finger_index} middle hinge remains mechanically seated",
        )
        ctx.expect_contact(
            tip,
            middle,
            contact_tol=0.00005,
            name=f"finger {finger_index} tip hinge remains mechanically seated",
        )

    finger_0_tip = object_model.get_part("finger_0_tip")
    rest_tip_aabb = ctx.part_world_aabb(finger_0_tip)
    with ctx.pose(
        {
            "finger_0_knuckle": 0.75,
            "finger_0_middle_joint": 0.55,
            "finger_0_tip_joint": 0.35,
        }
    ):
        curled_tip_aabb = ctx.part_world_aabb(finger_0_tip)
    ctx.check(
        "positive joint travel curls a finger away from the palm face",
        rest_tip_aabb is not None
        and curled_tip_aabb is not None
        and curled_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.050,
        details=f"rest_tip_aabb={rest_tip_aabb}, curled_tip_aabb={curled_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

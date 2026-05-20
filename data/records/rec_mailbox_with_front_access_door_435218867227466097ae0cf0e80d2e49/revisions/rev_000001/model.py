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


BODY_W = 0.62
BODY_D = 0.48
BODY_H = 0.78
BODY_Z0 = 0.72
FRONT_Y = -BODY_D / 2.0

DOOR_W = 0.50
DOOR_H = 0.46
DOOR_T = 0.024
DOOR_BOTTOM_Z = BODY_Z0 + 0.07
DOOR_HINGE_Y = FRONT_Y - 0.042

FLAP_W = 0.46
FLAP_D = 0.28
FLAP_T = 0.018
FLAP_HINGE_Y = 0.10
FLAP_HINGE_Z = BODY_Z0 + BODY_H + 0.065


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secure_parcel_mailbox")

    model.material("powder_coated_body", rgba=(0.15, 0.19, 0.22, 1.0))
    model.material("darker_seams", rgba=(0.035, 0.04, 0.045, 1.0))
    model.material("door_blue_grey", rgba=(0.20, 0.28, 0.34, 1.0))
    model.material("slot_flap_finish", rgba=(0.18, 0.24, 0.29, 1.0))
    model.material("brushed_metal", rgba=(0.68, 0.70, 0.68, 1.0))
    model.material("rubber_black", rgba=(0.01, 0.012, 0.014, 1.0))
    model.material("warning_yellow", rgba=(0.93, 0.72, 0.16, 1.0))

    housing = model.part("housing")

    # Ground-mounted plinth and post.
    housing.visual(
        Box((0.46, 0.36, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="powder_coated_body",
        name="plinth",
    )
    housing.visual(
        Box((0.16, 0.14, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material="powder_coated_body",
        name="post",
    )
    housing.visual(
        Box((0.48, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 - 0.028)),
        material="powder_coated_body",
        name="support_plate",
    )

    # Boxy parcel housing; built as connected metal panels with an open front bay.
    wall_t = 0.035
    housing.visual(
        Box((BODY_W, wall_t, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - wall_t / 2.0, BODY_Z0 + BODY_H / 2.0)),
        material="powder_coated_body",
        name="back_wall",
    )
    housing.visual(
        Box((wall_t, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + wall_t / 2.0, 0.0, BODY_Z0 + BODY_H / 2.0)),
        material="powder_coated_body",
        name="side_wall_0",
    )
    housing.visual(
        Box((wall_t, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - wall_t / 2.0, 0.0, BODY_Z0 + BODY_H / 2.0)),
        material="powder_coated_body",
        name="side_wall_1",
    )
    housing.visual(
        Box((BODY_W, BODY_D, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + 0.0225)),
        material="powder_coated_body",
        name="floor_pan",
    )
    housing.visual(
        Box((BODY_W, BODY_D, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + BODY_H - 0.0225)),
        material="powder_coated_body",
        name="top_deck",
    )

    # Front frame around the retrieval opening.
    frame_y = FRONT_Y - 0.015
    frame_t = 0.030
    side_stile_w = (BODY_W - DOOR_W) / 2.0
    door_frame_h = DOOR_H + 0.10
    door_frame_z = DOOR_BOTTOM_Z + door_frame_h / 2.0 - 0.025
    housing.visual(
        Box((side_stile_w, frame_t, door_frame_h)),
        origin=Origin(xyz=(-(DOOR_W / 2.0 + side_stile_w / 2.0), frame_y, door_frame_z)),
        material="powder_coated_body",
        name="door_stile_0",
    )
    housing.visual(
        Box((side_stile_w, frame_t, door_frame_h)),
        origin=Origin(xyz=((DOOR_W / 2.0 + side_stile_w / 2.0), frame_y, door_frame_z)),
        material="powder_coated_body",
        name="door_stile_1",
    )
    housing.visual(
        Box((BODY_W, frame_t, 0.055)),
        origin=Origin(xyz=(0.0, frame_y, DOOR_BOTTOM_Z - 0.030)),
        material="powder_coated_body",
        name="front_threshold",
    )
    housing.visual(
        Box((BODY_W, frame_t, 0.060)),
        origin=Origin(xyz=(0.0, frame_y, DOOR_BOTTOM_Z + DOOR_H + 0.035)),
        material="powder_coated_body",
        name="front_header",
    )
    housing.visual(
        Box((BODY_W, frame_t, 0.155)),
        origin=Origin(xyz=(0.0, frame_y, BODY_Z0 + BODY_H - 0.125)),
        material="powder_coated_body",
        name="upper_front_panel",
    )

    # A shallow overhanging top hood, black mail throat, and visible rain lips.
    housing.visual(
        Box((BODY_W + 0.075, BODY_D + 0.075, 0.055)),
        origin=Origin(xyz=(0.0, -0.015, BODY_Z0 + BODY_H + 0.0275)),
        material="powder_coated_body",
        name="top_hood",
    )
    housing.visual(
        Box((BODY_W + 0.09, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.055, BODY_Z0 + BODY_H + 0.015)),
        material="powder_coated_body",
        name="front_rain_lip",
    )
    housing.visual(
        Box((0.045, BODY_D + 0.095, 0.055)),
        origin=Origin(xyz=(-BODY_W / 2.0 - 0.025, -0.010, BODY_Z0 + BODY_H + 0.025)),
        material="powder_coated_body",
        name="hood_cheek_0",
    )
    housing.visual(
        Box((0.045, BODY_D + 0.095, 0.055)),
        origin=Origin(xyz=(BODY_W / 2.0 + 0.025, -0.010, BODY_Z0 + BODY_H + 0.025)),
        material="powder_coated_body",
        name="hood_cheek_1",
    )
    housing.visual(
        Box((0.42, 0.075, 0.006)),
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y - 0.115, BODY_Z0 + BODY_H + 0.058)),
        material="rubber_black",
        name="mail_slot_recess",
    )
    housing.visual(
        Box((0.50, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y + 0.008, BODY_Z0 + BODY_H + 0.058)),
        material="brushed_metal",
        name="slot_hinge_strip",
    )

    # Visual parcel/mail labeling and security bolts.
    housing.visual(
        Box((0.26, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.032, BODY_Z0 + BODY_H - 0.16)),
        material="warning_yellow",
        name="mail_label",
    )
    for x in (-0.23, 0.23):
        housing.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, FRONT_Y - 0.032, BODY_Z0 + BODY_H - 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_metal",
            name=f"front_bolt_{0 if x < 0 else 1}",
        )

    front_door = model.part("front_door")
    front_door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_H / 2.0)),
        material="door_blue_grey",
        name="door_panel",
    )
    # Raised perimeter embossing on the door skin.
    front_door.visual(
        Box((DOOR_W, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.017, DOOR_H - 0.025)),
        material="darker_seams",
        name="door_top_rail",
    )
    front_door.visual(
        Box((DOOR_W, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.017, 0.035)),
        material="darker_seams",
        name="door_bottom_rail",
    )
    front_door.visual(
        Box((0.030, 0.010, DOOR_H - 0.070)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.035, -0.017, DOOR_H / 2.0)),
        material="darker_seams",
        name="door_side_rail_0",
    )
    front_door.visual(
        Box((0.030, 0.010, DOOR_H - 0.070)),
        origin=Origin(xyz=(DOOR_W / 2.0 - 0.035, -0.017, DOOR_H / 2.0)),
        material="darker_seams",
        name="door_side_rail_1",
    )
    front_door.visual(
        Cylinder(radius=0.018, length=DOOR_W + 0.035),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_metal",
        name="lower_hinge_barrel",
    )
    front_door.visual(
        Box((0.24, 0.025, 0.035)),
        origin=Origin(xyz=(0.0, -0.056, DOOR_H * 0.61)),
        material="brushed_metal",
        name="pull_handle",
    )
    for x in (-0.09, 0.09):
        front_door.visual(
            Box((0.035, 0.048, 0.035)),
            origin=Origin(xyz=(x, -0.032, DOOR_H * 0.61)),
            material="brushed_metal",
            name=f"handle_post_{0 if x < 0 else 1}",
        )
    front_door.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.0, -0.017, DOOR_H * 0.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="lock_cylinder",
    )
    front_door.visual(
        Box((0.008, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, -0.024, DOOR_H * 0.78)),
        material="rubber_black",
        name="key_slot",
    )

    slot_flap = model.part("slot_flap")
    slot_flap.visual(
        Box((FLAP_W, FLAP_D, FLAP_T)),
        origin=Origin(xyz=(0.0, -FLAP_D / 2.0, FLAP_T / 2.0)),
        material="slot_flap_finish",
        name="flap_panel",
    )
    slot_flap.visual(
        Cylinder(radius=0.014, length=FLAP_W + 0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_metal",
        name="rear_hinge_barrel",
    )
    slot_flap.visual(
        Box((FLAP_W - 0.060, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, -FLAP_D + 0.025, FLAP_T + 0.013)),
        material="brushed_metal",
        name="front_pull_lip",
    )
    slot_flap.visual(
        Box((0.30, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -FLAP_D / 2.0, FLAP_T + 0.009)),
        material="darker_seams",
        name="flap_center_rib",
    )

    model.articulation(
        "housing_to_front_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_door,
        origin=Origin(xyz=(0.0, DOOR_HINGE_Y, DOOR_BOTTOM_Z)),
        # The closed door panel rises along local +Z; +X rotates it down and forward.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "housing_to_slot_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=slot_flap,
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        # The closed flap extends toward the front along local -Y; -X lifts it upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    front_door = object_model.get_part("front_door")
    slot_flap = object_model.get_part("slot_flap")
    front_hinge = object_model.get_articulation("housing_to_front_door")
    slot_hinge = object_model.get_articulation("housing_to_slot_flap")

    ctx.allow_overlap(
        housing,
        slot_flap,
        elem_a="slot_hinge_strip",
        elem_b="rear_hinge_barrel",
        reason="The visible top flap hinge barrel is intentionally captured in the fixed hinge strip.",
    )

    with ctx.pose({front_hinge: 0.0, slot_hinge: 0.0}):
        ctx.expect_overlap(
            front_door,
            housing,
            axes="xz",
            min_overlap=0.40,
            elem_a="door_panel",
            name="closed retrieval door covers lower front opening",
        )
        ctx.expect_gap(
            housing,
            front_door,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0001,
            positive_elem="front_threshold",
            negative_elem="door_panel",
            name="retrieval door sits just proud of front frame",
        )
        ctx.expect_overlap(
            slot_flap,
            housing,
            axes="xy",
            min_overlap=0.060,
            elem_a="flap_panel",
            elem_b="mail_slot_recess",
            name="closed top flap covers mail slot throat",
        )
        ctx.expect_gap(
            slot_flap,
            housing,
            axis="z",
            max_penetration=0.012,
            positive_elem="rear_hinge_barrel",
            negative_elem="slot_hinge_strip",
            name="slot hinge barrel is retained in fixed strip",
        )
        ctx.expect_gap(
            slot_flap,
            housing,
            axis="z",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="flap_panel",
            negative_elem="mail_slot_recess",
            name="slot flap rests above recessed slot",
        )

        closed_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
        closed_flap_aabb = ctx.part_element_world_aabb(slot_flap, elem="flap_panel")

    with ctx.pose({front_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    ctx.check(
        "front door opens down and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.20
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.15,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({slot_hinge: 1.05}):
        open_flap_aabb = ctx.part_element_world_aabb(slot_flap, elem="flap_panel")
    ctx.check(
        "mail slot flap lifts about rear edge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.12
        and open_flap_aabb[1][1] <= closed_flap_aabb[1][1] + 0.03,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

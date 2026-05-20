from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_W = 0.50
BODY_H = 0.33
BODY_D = 0.25
WALL = 0.025

FLANGE_W = 0.62
FLANGE_H = 0.43
FLANGE_IN_W = 0.46
FLANGE_IN_H = 0.30
FLANGE_T = 0.028

DOOR_W = 0.39
DOOR_H = 0.265
DOOR_T = 0.035
DOOR_PREOPEN = 0.62
DOOR_HINGE_Y = (FLANGE_T / 2.0) + 0.010 + (DOOR_T / 2.0)

DIAL_DIAMETER = 0.105
DIAL_HEIGHT = 0.023
DIAL_Y = (DOOR_T / 2.0) + 0.004 + (DIAL_HEIGHT / 2.0)

HANDLE_HUB_RADIUS = 0.025
HANDLE_DEPTH = 0.026
HANDLE_Y = (DOOR_T / 2.0) + 0.004 + (HANDLE_DEPTH / 2.0)
HANDLE_Z = -0.082

FLAP_W = 0.31
FLAP_H = 0.075
FLAP_T = 0.007
FLAP_Y = -0.080
FLAP_HINGE_Z = 0.118


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_wall_safe")

    model.material("powder_coated_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("dark_interior_steel", rgba=(0.07, 0.075, 0.08, 1.0))
    model.material("door_steel", rgba=(0.24, 0.25, 0.26, 1.0))
    model.material("recess_shadow", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("dial_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("dial_mark", rgba=(0.86, 0.86, 0.80, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.61, 0.56, 1.0))
    model.material("flap_blue_gray", rgba=(0.34, 0.40, 0.46, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D + WALL / 2.0, 0.0)),
        material="dark_interior_steel",
        name="back_wall",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + WALL / 2.0, -BODY_D / 2.0, 0.0)),
        material="dark_interior_steel",
        name="side_wall_0",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL / 2.0, -BODY_D / 2.0, 0.0)),
        material="dark_interior_steel",
        name="side_wall_1",
    )
    body.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, BODY_H / 2.0 - WALL / 2.0)),
        material="dark_interior_steel",
        name="top_wall",
    )
    body.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, -BODY_H / 2.0 + WALL / 2.0)),
        material="dark_interior_steel",
        name="bottom_wall",
    )

    flange_side_w = (FLANGE_W - FLANGE_IN_W) / 2.0
    flange_rail_h = (FLANGE_H - FLANGE_IN_H) / 2.0
    body.visual(
        Box((flange_side_w, FLANGE_T, FLANGE_H)),
        origin=Origin(xyz=(-(FLANGE_IN_W / 2.0 + flange_side_w / 2.0), 0.0, 0.0)),
        material="powder_coated_steel",
        name="flange_side_0",
    )
    body.visual(
        Box((flange_side_w, FLANGE_T, FLANGE_H)),
        origin=Origin(xyz=((FLANGE_IN_W / 2.0 + flange_side_w / 2.0), 0.0, 0.0)),
        material="powder_coated_steel",
        name="flange_side_1",
    )
    body.visual(
        Box((FLANGE_W, FLANGE_T, flange_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_IN_H / 2.0 + flange_rail_h / 2.0)),
        material="powder_coated_steel",
        name="flange_top",
    )
    body.visual(
        Box((FLANGE_W, FLANGE_T, flange_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, -(FLANGE_IN_H / 2.0 + flange_rail_h / 2.0))),
        material="powder_coated_steel",
        name="flange_bottom",
    )
    body.visual(
        Box((0.030, 0.032, DOOR_H * 0.94)),
        origin=Origin(xyz=(DOOR_W / 2.0 + 0.026, 0.030, 0.0)),
        material="brushed_metal",
        name="hinge_mount",
    )
    body.visual(
        Box((0.014, 0.020, 0.038)),
        origin=Origin(xyz=(-0.160, FLAP_Y, FLAP_HINGE_Z + 0.012)),
        material="brushed_metal",
        name="flap_bracket_0",
    )
    body.visual(
        Box((0.014, 0.020, 0.038)),
        origin=Origin(xyz=(0.160, FLAP_Y, FLAP_HINGE_Z + 0.012)),
        material="brushed_metal",
        name="flap_bracket_1",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_W / 2.0, 0.0, 0.0)),
        material="door_steel",
        name="door_slab",
    )
    door.visual(
        Box((DOOR_W - 0.060, 0.003, 0.012)),
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_T / 2.0 + 0.0008, 0.106)),
        material="recess_shadow",
        name="recess_top",
    )
    door.visual(
        Box((DOOR_W - 0.060, 0.003, 0.012)),
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_T / 2.0 + 0.0008, -0.119)),
        material="recess_shadow",
        name="recess_bottom",
    )
    door.visual(
        Box((0.012, 0.003, DOOR_H - 0.070)),
        origin=Origin(xyz=(-(DOOR_W - 0.060) + 0.006, DOOR_T / 2.0 + 0.0008, -0.006)),
        material="recess_shadow",
        name="recess_side_0",
    )
    door.visual(
        Box((0.012, 0.003, DOOR_H - 0.070)),
        origin=Origin(xyz=(-0.006, DOOR_T / 2.0 + 0.0008, -0.006)),
        material="recess_shadow",
        name="recess_side_1",
    )
    door.visual(
        Cylinder(0.013, DOOR_H * 0.94),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_metal",
        name="hinge_barrel",
    )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            DIAL_DIAMETER,
            DIAL_HEIGHT,
            body_style="faceted",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=48, depth=0.0011, width=0.0016),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "combination_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dial_black",
        name="dial_cap",
    )
    dial.visual(
        Cylinder(0.018, 0.006),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="center_cap",
    )
    dial.visual(
        Cylinder(0.010, 0.040),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="dial_spindle",
    )
    handle = model.part("handle")
    handle.visual(
        Cylinder(HANDLE_HUB_RADIUS, HANDLE_DEPTH),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="front_hub",
    )
    handle.visual(
        Cylinder(0.012, 0.040),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="handle_spindle",
    )
    for idx, angle in enumerate((0.0, math.pi, -math.pi / 2.0)):
        radius = 0.032
        handle.visual(
            Box((0.052, 0.016, 0.014)),
            origin=Origin(
                xyz=(radius * math.cos(angle), 0.009, radius * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material="brushed_metal",
            name=f"spoke_{idx}",
        )

    document_flap = model.part("document_flap")
    document_flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, 0.0, -FLAP_H / 2.0)),
        material="flap_blue_gray",
        name="flap_panel",
    )
    document_flap.visual(
        Cylinder(0.006, FLAP_W),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_metal",
        name="top_sleeve",
    )
    document_flap.visual(
        Box((0.105, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.006, -FLAP_H + 0.006)),
        material="brushed_metal",
        name="finger_lip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(DOOR_W / 2.0, DOOR_HINGE_Y, 0.0),
            rpy=(0.0, 0.0, -DOOR_PREOPEN),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-DOOR_PREOPEN, upper=1.20, effort=18.0, velocity=1.2),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-DOOR_W / 2.0, DIAL_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(-DOOR_W / 2.0, HANDLE_Y, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi / 2.0, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "body_to_document_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=document_flap,
        origin=Origin(xyz=(0.0, FLAP_Y, FLAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=2.5, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    document_flap = object_model.get_part("document_flap")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_handle")
    flap_hinge = object_model.get_articulation("body_to_document_flap")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_mount",
        elem_b="hinge_barrel",
        reason="The door hinge barrel is intentionally captured in the fixed hinge mount at the flange.",
    )
    ctx.allow_overlap(
        body,
        document_flap,
        elem_a="flap_bracket_0",
        elem_b="top_sleeve",
        reason="The flap sleeve is intentionally captured by the top hinge bracket.",
    )
    ctx.allow_overlap(
        body,
        document_flap,
        elem_a="flap_bracket_1",
        elem_b="top_sleeve",
        reason="The flap sleeve is intentionally captured by the top hinge bracket.",
    )
    ctx.allow_overlap(
        dial,
        door,
        elem_a="dial_spindle",
        elem_b="door_slab",
        reason="The dial spindle intentionally passes into the safe door so the rotating dial is mechanically retained.",
    )
    ctx.allow_overlap(
        handle,
        door,
        elem_a="handle_spindle",
        elem_b="door_slab",
        reason="The handle spindle intentionally passes into the safe door so the rotating handle is mechanically retained.",
    )

    ctx.check(
        "primary mechanisms are articulated",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and handle_joint.articulation_type == ArticulationType.REVOLUTE
        and flap_hinge.articulation_type == ArticulationType.REVOLUTE,
    )

    with ctx.pose({door_hinge: -DOOR_PREOPEN}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.004,
            max_gap=0.020,
            positive_elem="door_slab",
            negative_elem="flange_top",
            name="closed door sits just proud of the front flange",
        )
        ctx.expect_within(
            door,
            body,
            axes="xz",
            margin=0.010,
            inner_elem="door_slab",
            name="closed door remains compact within the broad flange width",
        )
        ctx.expect_gap(
            dial,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.005,
            positive_elem="dial_cap",
            negative_elem="door_slab",
            name="combination dial is mounted to the door face",
        )
        ctx.expect_overlap(
            dial,
            door,
            axes="y",
            min_overlap=0.018,
            elem_a="dial_spindle",
            elem_b="door_slab",
            name="dial spindle is retained through the door slab",
        )
        ctx.expect_gap(
            handle,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.005,
            positive_elem="front_hub",
            negative_elem="door_slab",
            name="spoke handle hub is mounted to the door face",
        )
        ctx.expect_overlap(
            handle,
            door,
            axes="y",
            min_overlap=0.018,
            elem_a="handle_spindle",
            elem_b="door_slab",
            name="handle spindle is retained through the door slab",
        )

    closed_aabb = None
    open_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: -DOOR_PREOPEN}):
        closed_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "right hinge swings door outward to reveal the opening",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.06,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        min_overlap=0.20,
        elem_a="hinge_barrel",
        elem_b="hinge_mount",
        name="door barrel stays retained in the fixed hinge mount",
    )

    flap_pos = ctx.part_world_position(document_flap)
    ctx.check(
        "document flap is a separate high panel inside the cavity",
        flap_pos is not None and flap_pos[1] < -0.03 and flap_pos[2] > 0.09,
        details=f"flap_origin={flap_pos}",
    )
    ctx.expect_overlap(
        document_flap,
        body,
        axes="x",
        min_overlap=0.001,
        elem_a="top_sleeve",
        elem_b="flap_bracket_0",
        name="flap sleeve is retained by one hinge bracket",
    )
    ctx.expect_overlap(
        document_flap,
        body,
        axes="x",
        min_overlap=0.001,
        elem_a="top_sleeve",
        elem_b="flap_bracket_1",
        name="flap sleeve is retained by the other hinge bracket",
    )

    with ctx.pose({flap_hinge: 1.0}):
        lifted_pos = ctx.part_world_aabb(document_flap)
    rest_pos = ctx.part_world_aabb(document_flap)
    ctx.check(
        "document flap rotates on a top horizontal hinge",
        lifted_pos is not None
        and rest_pos is not None
        and lifted_pos[1][1] > rest_pos[1][1] + 0.02,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    return ctx.report()


object_model = build_object_model()

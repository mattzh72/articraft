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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_DEPTH = 0.45
BODY_WIDTH = 0.58
BODY_HEIGHT = 0.36

WALL_T = 0.018
TOP_T = 0.016
BOTTOM_T = 0.020
BACK_T = 0.018
FRONT_T = 0.016

DOOR_WIDTH = 0.42
DOOR_HEIGHT = 0.27
DOOR_T = 0.022
DOOR_CENTER_Y = -0.050
DOOR_HINGE_Z = -0.142

CONTROL_W = 0.120
CONTROL_CENTER_Y = 0.220
KNOB_RADIUS = 0.026
KNOB_ZS = (0.085, -0.005)

VENT_W = 0.280
VENT_L = 0.090
VENT_T = 0.004
VENT_FRAME_H = 0.012
VENT_REAR_X = (-BODY_DEPTH / 2.0) + 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_oven")

    model.material("oven_shell", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("oven_trim", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("door_frame", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("glass", rgba=(0.22, 0.30, 0.34, 0.42))
    model.material("control_knob", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("indicator", rgba=(0.85, 0.86, 0.88, 1.0))
    model.material("vent_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("foot", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")

    # Main shell.
    body.visual(
        Box((BODY_DEPTH, WALL_T, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -(BODY_WIDTH - WALL_T) / 2.0, 0.0)),
        material="oven_shell",
        name="side_wall_0",
    )
    body.visual(
        Box((BODY_DEPTH, WALL_T, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, (BODY_WIDTH - WALL_T) / 2.0, 0.0)),
        material="oven_shell",
        name="side_wall_1",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT - TOP_T) / 2.0)),
        material="oven_shell",
        name="top_shell",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, -(BODY_HEIGHT - BOTTOM_T) / 2.0)),
        material="oven_shell",
        name="bottom_shell",
    )
    body.visual(
        Box((BACK_T, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_DEPTH - BACK_T) / 2.0, 0.0, 0.0)),
        material="oven_shell",
        name="rear_shell",
    )

    # Front trim and right-side control column.
    body.visual(
        Box((FRONT_T, 0.030, 0.308)),
        origin=Origin(
            xyz=((BODY_DEPTH - FRONT_T) / 2.0, -0.275, -0.004),
        ),
        material="oven_trim",
        name="door_bezel_side",
    )
    body.visual(
        Box((FRONT_T, 0.452, 0.040)),
        origin=Origin(
            xyz=((BODY_DEPTH - FRONT_T) / 2.0, -0.055, 0.126),
        ),
        material="oven_trim",
        name="door_bezel_top",
    )
    body.visual(
        Box((FRONT_T, 0.452, 0.034)),
        origin=Origin(
            xyz=((BODY_DEPTH - FRONT_T) / 2.0, -0.055, -0.163),
        ),
        material="oven_trim",
        name="door_bezel_bottom",
    )
    body.visual(
        Box((FRONT_T, CONTROL_W, 0.318)),
        origin=Origin(
            xyz=((BODY_DEPTH - FRONT_T) / 2.0, CONTROL_CENTER_Y, -0.003),
        ),
        material="oven_trim",
        name="control_panel",
    )
    body.visual(
        Box((0.004, CONTROL_W - 0.028, 0.250)),
        origin=Origin(
            xyz=((BODY_DEPTH / 2.0) - 0.014, CONTROL_CENTER_Y, -0.010),
        ),
        material="vent_dark",
        name="control_inset",
    )

    # Counter feet.
    foot_size = (0.044, 0.034, 0.014)
    for idx, (foot_x, foot_y) in enumerate(
        (
            (0.150, -0.210),
            (0.150, 0.210),
            (-0.150, -0.210),
            (-0.150, 0.210),
        )
    ):
        body.visual(
            Box(foot_size),
            origin=Origin(
                xyz=(foot_x, foot_y, -(BODY_HEIGHT / 2.0) - (foot_size[2] / 2.0)),
            ),
            material="foot",
            name=f"foot_{idx}",
        )

    # Top rear steam vent surround.
    vent_side_size = (VENT_L, 0.020, VENT_FRAME_H)
    for idx, vent_y in enumerate((-0.130, 0.130)):
        body.visual(
            Box(vent_side_size),
            origin=Origin(
                xyz=(
                    VENT_REAR_X + (VENT_L / 2.0),
                    vent_y,
                    (BODY_HEIGHT / 2.0) + (VENT_FRAME_H / 2.0),
                ),
            ),
            material="oven_trim",
            name=f"vent_cheek_{idx}",
        )
    body.visual(
        Box((0.020, VENT_W, VENT_FRAME_H)),
        origin=Origin(
            xyz=(
                VENT_REAR_X,
                0.0,
                (BODY_HEIGHT / 2.0) + (VENT_FRAME_H / 2.0),
            ),
        ),
        material="oven_trim",
        name="vent_rear_bridge",
    )
    body.visual(
        Box((0.068, VENT_W - 0.034, 0.003)),
        origin=Origin(
            xyz=(
                VENT_REAR_X + 0.045,
                0.0,
                (BODY_HEIGHT / 2.0) + 0.0015,
            ),
        ),
        material="vent_dark",
        name="vent_slot",
    )

    door = model.part("door")
    stile_w = 0.050
    top_rail_h = 0.048
    bottom_rail_h = 0.060

    for idx, side_y in enumerate(
        (
            -(DOOR_WIDTH / 2.0) + (stile_w / 2.0),
            (DOOR_WIDTH / 2.0) - (stile_w / 2.0),
        )
    ):
        door.visual(
            Box((DOOR_T, stile_w, DOOR_HEIGHT)),
            origin=Origin(xyz=(DOOR_T / 2.0, side_y, DOOR_HEIGHT / 2.0)),
            material="door_frame",
            name=f"door_stile_{idx}",
        )
    door.visual(
        Box((DOOR_T, DOOR_WIDTH, top_rail_h)),
        origin=Origin(xyz=(DOOR_T / 2.0, 0.0, DOOR_HEIGHT - (top_rail_h / 2.0))),
        material="door_frame",
        name="door_rail_top",
    )
    door.visual(
        Box((DOOR_T, DOOR_WIDTH, bottom_rail_h)),
        origin=Origin(xyz=(DOOR_T / 2.0, 0.0, bottom_rail_h / 2.0)),
        material="door_frame",
        name="door_rail_bottom",
    )
    door.visual(
        Box((0.004, DOOR_WIDTH - 0.068, DOOR_HEIGHT - 0.116)),
        origin=Origin(
            xyz=(0.010, 0.0, 0.058 + ((DOOR_HEIGHT - 0.116) / 2.0)),
        ),
        material="glass",
        name="door_glass",
    )

    handle_z = DOOR_HEIGHT - 0.046
    handle_post_x = DOOR_T + 0.014
    for idx, post_y in enumerate((-0.110, 0.110)):
        door.visual(
            Box((0.028, 0.018, 0.020)),
            origin=Origin(xyz=(handle_post_x, post_y, handle_z)),
            material="oven_trim",
            name=f"door_handle_post_{idx}",
        )
    door.visual(
        Cylinder(radius=0.008, length=0.288),
        origin=Origin(
            xyz=(DOOR_T + 0.030, 0.0, handle_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="oven_trim",
        name="door_handle_bar",
    )

    knob_geom = mesh_from_geometry(
        KnobGeometry(
            diameter=0.044,
            height=0.026,
            body_style="skirted",
            top_diameter=0.035,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "oven_knob",
    )

    knob_parts = []
    for idx, knob_z in enumerate(KNOB_ZS):
        knob = model.part(f"knob_{idx}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material="oven_trim",
            name="shaft",
        )
        knob.visual(
            knob_geom,
            origin=Origin(),
            material="control_knob",
            name="cap",
        )
        knob_parts.append((knob, knob_z))

    vent_flap = model.part("vent_flap")
    vent_flap.visual(
        Box((VENT_L, VENT_W, VENT_T)),
        origin=Origin(xyz=(VENT_L / 2.0, 0.0, VENT_T / 2.0)),
        material="oven_trim",
        name="flap_panel",
    )
    vent_flap.visual(
        Cylinder(radius=0.004, length=VENT_W - 0.016),
        origin=Origin(
            xyz=(0.006, 0.0, 0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="oven_trim",
        name="flap_hinge_barrel",
    )

    door_joint = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(BODY_DEPTH / 2.0, DOOR_CENTER_Y, DOOR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.48,
            effort=24.0,
            velocity=1.5,
        ),
    )

    for knob_idx, (knob, knob_z) in enumerate(knob_parts):
        model.articulation(
            f"knob_{knob_idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(
                xyz=(BODY_DEPTH / 2.0, CONTROL_CENTER_Y, knob_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        )

    model.articulation(
        "vent_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=vent_flap,
        origin=Origin(
            xyz=(
                VENT_REAR_X,
                0.0,
                (BODY_HEIGHT / 2.0) + VENT_FRAME_H,
            ),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.10,
            effort=4.0,
            velocity=1.8,
        ),
    )

    # Keep the joint object alive for clarity when reading the authored backbone.
    assert door_joint is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    vent_flap = object_model.get_part("vent_flap")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    door_hinge = object_model.get_articulation("door_hinge")
    vent_hinge = object_model.get_articulation("vent_hinge")
    knob_0_spin = object_model.get_articulation("knob_0_spin")
    knob_1_spin = object_model.get_articulation("knob_1_spin")

    ctx.expect_gap(
        door,
        body,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        name="door sits flush with the oven front",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.20,
        name="door covers the front opening footprint",
    )
    ctx.expect_gap(
        vent_flap,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="vent flap rests on the rear vent surround",
    )

    knob_0_pos = ctx.part_world_position(knob_0)
    knob_1_pos = ctx.part_world_position(knob_1)
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "control knobs stay on the front control column",
        knob_0_pos is not None
        and knob_1_pos is not None
        and body_pos is not None
        and knob_0_pos[0] > body_pos[0] + 0.21
        and knob_1_pos[0] > body_pos[0] + 0.21
        and abs(knob_0_pos[1] - knob_1_pos[1]) < 0.001
        and knob_0_pos[2] > knob_1_pos[2] + 0.07,
        details=f"knob_0={knob_0_pos}, knob_1={knob_1_pos}, body={body_pos}",
    )

    ctx.check(
        "knob joints are continuous rotary controls",
        knob_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"knob_0={knob_0_spin.articulation_type}, knob_1={knob_1_spin.articulation_type}",
    )
    ctx.check(
        "door and vent have realistic hinge ranges",
        door_hinge.motion_limits is not None
        and vent_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and vent_hinge.motion_limits.upper is not None
        and 1.30 <= door_hinge.motion_limits.upper <= 1.60
        and 0.80 <= vent_hinge.motion_limits.upper <= 1.20,
        details=(
            f"door_limits={door_hinge.motion_limits}, "
            f"vent_limits={vent_hinge.motion_limits}"
        ),
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.40}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.12
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_vent_aabb = ctx.part_world_aabb(vent_flap)
    with ctx.pose({vent_hinge: 0.95}):
        open_vent_aabb = ctx.part_world_aabb(vent_flap)
    ctx.check(
        "vent flap lifts above the rear deck",
        closed_vent_aabb is not None
        and open_vent_aabb is not None
        and open_vent_aabb[1][2] > closed_vent_aabb[1][2] + 0.05
        and open_vent_aabb[0][0] < closed_vent_aabb[0][0] + 0.01,
        details=f"closed={closed_vent_aabb}, open={open_vent_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
